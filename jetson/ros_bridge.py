"""ROS 桥（Jetson → 实车控制）。

设计目标：
- 提供一个对算法"干净"的接口：publish_cmd_vel(steer_deg, speed_mps)
- 内部实现可插拔：
  1. 真实 ROS 节点（用 rospy / rclpy），发布到 /cmd_vel 或自定义话题
  2. 模拟后端（直接 print 或写文件），用于无 ROS 环境调试

为什么这么做：
- 现在还没有实车控制代码，先把"算法 → 控制指令"的接口定好
- 后面接入 ROS 时只需要替换 backend，不动算法主循环
- 配置 ros_bridge.backend: "ros" | "mock" 切换
"""
from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from typing import Optional, Protocol

log = logging.getLogger(__name__)


@dataclass
class CmdVel:
    """通用速度指令。"""
    linear_mps: float       # 前进速度 m/s（>0 前进，<0 后退）
    angular_radps: float    # 角速度 rad/s（>0 左转，<0 右转，符号约定以 ROS REP-103 为准）
    steer_deg: float = 0.0  # 原始转向角（度）保留，方便 ROS 之外的自驾仪用
    speed: float = 0.0      # 原始速度（无量纲）保留
    ts: float = 0.0         # 发出时刻（monotonic time）

    def is_zero(self) -> bool:
        return abs(self.linear_mps) < 1e-3 and abs(self.angular_radps) < 1e-3


class _Backend(Protocol):
    def send(self, cmd: CmdVel) -> None: ...
    def stop(self) -> None: ...
    def close(self) -> None: ...


class MockBackend:
    """无 ROS 时的占位实现：把每条指令打到日志。"""

    def __init__(self, wheelbase_m: float = 0.30) -> None:
        self.wheelbase_m = float(wheelbase_m)
        self._last: Optional[CmdVel] = None

    def send(self, cmd: CmdVel) -> None:
        self._last = cmd
        log.info(
            "[MOCK ROS]  v=%.3f m/s  w=%.3f rad/s  steer=%+.2f°  speed=%.3f",
            cmd.linear_mps, cmd.angular_radps, cmd.steer_deg, cmd.speed,
        )

    def stop(self) -> None:
        log.info("[MOCK ROS]  STOP（发零速度）")
        self.send(CmdVel(linear_mps=0.0, angular_radps=0.0, ts=time.monotonic()))

    def close(self) -> None:
        self.stop()


class RosBackend:
    """真实 ROS 节点后端（依赖 rospy，运行时才 import）。

    接入时把 publish_to_ros.py / 自驾仪 launch 文件准备好，把
    ``ros_bridge = RosBridge(backend="ros")`` 即可，调用接口不变。

    注：本文件不强制 import rospy，没装 ROS 时不报错。
    """

    def __init__(self, cmd_vel_topic: str = "/cmd_vel", wheelbase_m: float = 0.30) -> None:
        self.topic = cmd_vel_topic
        self.wheelbase_m = float(wheelbase_m)
        self._pub = None
        self._twist_msg = None
        self._rospy = None
        try:
            import rospy  # type: ignore
            from geometry_msgs.msg import Twist  # type: ignore
        except Exception as e:  # noqa: BLE001
            log.error("无法 import rospy / geometry_msgs：%s", e)
            raise
        self._rospy = rospy
        self._twist_msg = Twist
        try:
            self._pub = rospy.Publisher(self.topic, Twist, queue_size=1)
        except Exception as e:  # noqa: BLE001
            log.error("创建 Publisher 失败 (topic=%s)：%s", self.topic, e)
            raise
        log.info("ROS Publisher 已就绪: %s", self.topic)

    def send(self, cmd: CmdVel) -> None:
        if self._pub is None or self._rospy is None:
            return
        msg = self._twist_msg()
        msg.linear.x = float(cmd.linear_mps)
        msg.angular.z = float(cmd.angular_radps)
        self._pub.publish(msg)

    def stop(self) -> None:
        if self._pub is None:
            return
        msg = self._twist_msg()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self._pub.publish(msg)
        log.info("[ROS] STOP")

    def close(self) -> None:
        self.stop()


class RosBridge:
    """算法 → 实车的统一入口。

    用法::

        bridge = RosBridge(backend="mock", wheelbase_m=0.30)
        bridge.publish_cmd_vel(steer_deg=10.0, speed=0.5)  # 来自 controller.decide
        bridge.stop()
    """

    def __init__(self, backend: str = "mock", wheelbase_m: float = 0.30) -> None:
        self.backend_name = backend
        self.wheelbase_m = float(wheelbase_m)
        self._backend: _Backend
        if backend == "mock":
            self._backend = MockBackend(wheelbase_m=wheelbase_m)
        elif backend in ("ros", "rospy", "ros2"):
            self._backend = RosBackend(wheelbase_m=wheelbase_m)
        else:
            raise ValueError(f"未知 backend: {backend!r}（支持 'mock' / 'ros'）")
        self._last: Optional[CmdVel] = None

    def publish_cmd_vel(self, steer_deg: float, speed: float) -> CmdVel:
        """根据 controller.decide 的 (steer_deg, speed) 算出 ROS 用的 (v, w)。

        - speed 是无量纲（0~1）→ 乘 max_speed_mps 转 m/s
        - steer_deg → 角速度 w = v * tan(steer) / wheelbase（阿克曼近似）
        - 转向角限幅到 ±max_steer_deg（如果上层没限）
        """
        max_speed_mps = 0.5  # 默认上限；可由 cfg 覆盖
        v = float(speed) * max_speed_mps
        steer_rad = math.radians(steer_deg)
        # 防止 tan(±π/2) 爆炸
        steer_rad = max(min(steer_rad, math.radians(60.0)), math.radians(-60.0))
        w = v * math.tan(steer_rad) / self.wheelbase_m
        cmd = CmdVel(linear_mps=v, angular_radps=w,
                     steer_deg=steer_deg, speed=speed, ts=time.monotonic())
        self._backend.send(cmd)
        self._last = cmd
        return cmd

    def stop(self) -> None:
        self._backend.stop()
        self._last = CmdVel(linear_mps=0.0, angular_radps=0.0, ts=time.monotonic())

    def close(self) -> None:
        self._backend.close()
