# -*- coding: utf-8 -*-
"""ROS 桥（Jetson → 实车控制）。

设计目标：
- 算法只调用 ``publish_cmd_vel(steer_deg, speed)`` —— 不需要懂 ROS
- backend 可插拔：mock / 真 ROS (rospy)
- 真 ROS 模式额外订阅 /xcar/sonar[1-4] 与 /xcar/sensors 做：
  * 紧急停车：超声 < sonar_stop_m 时强制 zero
  * 低电量告警：bat < bat_warn (0.1V) 时打 log

适配车型：
- zonesion xcar（ros_pkgs/leading_line/scripts/xcar/xcar_ros.py）：3-DOF 全向底盘，
  /cmd_vel 是 geometry_msgs/Twist，把 steer → angular.z、speed → linear.x
  lateral (linear.y) 默认 0（"在 4WD 上开阿克曼车"风格）。
- 也兼容传统阿克曼底盘（只填 linear.x + angular.z）
"""
from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional, Protocol

log = logging.getLogger(__name__)


@dataclass
class CmdVel:
    """通用速度指令。"""
    linear_mps: float = 0.0         # 前进速度 m/s（>0 前进，<0 后退）
    angular_radps: float = 0.0      # 角速度 rad/s（左正右负，ROS REP-103）
    linear_y_mps: float = 0.0       # 横向速度（4WD 全向平台用；阿克曼恒为 0）
    steer_deg: float = 0.0          # 原始转向角（度），便于自驾仪/调试
    speed: float = 0.0              # 原始速度（0~1），保留
    ts: float = 0.0                 # 发出时刻（monotonic time）

    def is_zero(self) -> bool:
        return (
            abs(self.linear_mps) < 1e-3
            and abs(self.angular_radps) < 1e-3
            and abs(self.linear_y_mps) < 1e-3
        )


class _Backend(Protocol):
    def send(self, cmd: CmdVel) -> None: ...
    def stop(self) -> None: ...
    def close(self) -> None: ...


# ---------------------------------------------------------------------------
# Mock：把每条指令打到日志，无 ROS 时调试用
# ---------------------------------------------------------------------------

class MockBackend:
    """无 ROS 时的占位实现：把每条指令打到日志。"""

    def __init__(self, wheelbase_m: float = 0.30) -> None:
        self.wheelbase_m = float(wheelbase_m)
        self._last: Optional[CmdVel] = None
        self._sonar_stop_m: float = -1.0   # <0 表示未启用
        self._sonar_cm: List[float] = [999.0, 999.0, 999.0, 999.0]

    def send(self, cmd: CmdVel) -> None:
        self._last = cmd
        log.info(
            "[MOCK ROS]  vx=%.3f  vy=%.3f  w=%.3f  steer=%+.2f°  speed=%.3f",
            cmd.linear_mps, cmd.linear_y_mps, cmd.angular_radps,
            cmd.steer_deg, cmd.speed,
        )

    def stop(self) -> None:
        log.info("[MOCK ROS]  STOP（发零速度）")
        self.send(CmdVel(linear_mps=0.0, angular_radps=0.0, ts=time.monotonic()))

    def close(self) -> None:
        self.stop()


# ---------------------------------------------------------------------------
# ROS 后端：真发 /cmd_vel；可选订 /xcar/sonar[1-4] 做紧急停车
# ---------------------------------------------------------------------------

class RosBackend:
    """真 ROS 节点后端（依赖 rospy）。

    行为：
    1. Publisher: /cmd_vel (geometry_msgs/Twist)
    2. 可选 Subscriber: /xcar/sonar1..4 (sensor_msgs/Range) —— 任一距离 < sonar_stop_m 则强制 zero
    3. 可选 Subscriber: /xcar/sensors (Int32MultiArray) —— 打 bat/temp/... 状态

    启动顺序无关：sonar 订阅失败只是禁用安全停，不影响控制。
    """

    def __init__(
        self,
        cmd_vel_topic: str = "/cmd_vel",
        wheelbase_m: float = 0.30,
        sonar_topics: Optional[List[str]] = None,
        sonar_stop_m: float = 0.30,
        sensors_topic: str = "/xcar/sensors",
        bat_warn: int = 70,        # 0.1V 单位；7.0V 告警
        bat_critical: int = 65,    # 6.5V 自动停车
    ) -> None:
        self.topic = cmd_vel_topic
        self.wheelbase_m = float(wheelbase_m)
        self.sonar_topics = sonar_topics or [
            "/xcar/sonar1", "/xcar/sonar2", "/xcar/sonar3", "/xcar/sonar4",
        ]
        self.sonar_stop_m = float(sonar_stop_m)
        self.sensors_topic = sensors_topic
        self.bat_warn = int(bat_warn)
        self.bat_critical = int(bat_critical)

        self._lock = threading.Lock()
        self._sonar_m: List[float] = [999.0] * len(self.sonar_topics)
        self._sonar_ts: List[float] = [0.0] * len(self.sonar_topics)
        self._bat: int = 0
        self._temp: int = 0
        self._estop_active: bool = False
        self._low_bat: bool = False

        try:
            import rospy  # type: ignore
            from geometry_msgs.msg import Twist  # type: ignore
            from sensor_msgs.msg import Range  # type: ignore
            from std_msgs.msg import Int32MultiArray  # type: ignore
        except Exception as e:  # noqa: BLE001
            log.error("无法 import rospy / geometry_msgs / sensor_msgs / std_msgs：%s", e)
            raise
        self._rospy = rospy
        self._Twist = Twist
        self._Range = Range
        self._Int32MultiArray = Int32MultiArray

        # 在外部已有 ROS 节点上下文中初始化（如果没有 init_node，则跳过）
        if not rospy.get_name() == "/unnamed":
            self._ros_ctx_ok = True
        else:
            self._ros_ctx_ok = False
            log.warning("rospy.init_node() 还没调用；建议在主入口先 init 再 RosBackend()")

        try:
            self._pub = rospy.Publisher(self.topic, Twist, queue_size=1)
        except Exception as e:  # noqa: BLE001
            log.error("创建 Publisher 失败 (topic=%s)：%s", self.topic, e)
            raise
        log.info("ROS Publisher 已就绪: %s", self.topic)

        # sonar 订阅
        for i, t in enumerate(self.sonar_topics):
            try:
                rospy.Subscriber(t, self._Range, self._make_sonar_cb(i), queue_size=1)
            except Exception as e:  # noqa: BLE001
                log.warning("sonar 订阅失败 %s: %s（紧急停车禁用）", t, e)
                self.sonar_topics[i] = None  # 标记失效

        # sensors 订阅（Int32MultiArray：[bat, temp, humi, _, pressure, light, tvoc, smoke, dist1..4]）
        try:
            rospy.Subscriber(self.sensors_topic, self._Int32MultiArray, self._sensors_cb, queue_size=1)
        except Exception as e:  # noqa: BLE001
            log.warning("sensors 订阅失败 %s: %s（状态监控禁用）", self.sensors_topic, e)

    # ----- 订阅回调 -----

    def _make_sonar_cb(self, idx: int):
        def _cb(msg):
            r = float(getattr(msg, "range", 999.0))
            with self._lock:
                self._sonar_m[idx] = r
                self._sonar_ts[idx] = time.monotonic()
        return _cb

    def _sensors_cb(self, msg):
        data = list(getattr(msg, "data", []) or [])
        if len(data) < 8:
            return
        with self._lock:
            self._bat = int(data[0])
            self._temp = int(data[1])
        # 状态日志（不每帧打）
        if self._bat and self._bat <= self.bat_critical:
            if not self._low_bat:
                log.error("电量过低 (%.1fV) —— 强制停车", self._bat / 10.0)
                self._low_bat = True
        elif self._bat and self._bat <= self.bat_warn:
            if not self._low_bat:
                log.warning("电量偏低 (%.1fV)", self._bat / 10.0)

    # ----- 控制接口 -----

    def _check_estop(self) -> bool:
        """任一 sonar 距离 < sonar_stop_m 就强制停车。"""
        if not any(self.sonar_topics):
            return False
        now = time.monotonic()
        with self._lock:
            min_dist = min(
                (d for i, d in enumerate(self._sonar_m) if self.sonar_topics[i] and (now - self._sonar_ts[i] < 1.0)),
                default=999.0,
            )
        if min_dist < self.sonar_stop_m:
            if not self._estop_active:
                log.warning("紧急停车：最近障碍 %.2f m < %.2f m", min_dist, self.sonar_stop_m)
                self._estop_active = True
            return True
        self._estop_active = False
        return False

    def _check_battery(self) -> bool:
        if self._bat and self._bat <= self.bat_critical:
            return True
        return False

    def send(self, cmd: CmdVel) -> None:
        if self._pub is None:
            return
        # 安全闸门
        if self._check_estop() or self._check_battery():
            self.send_zero()
            return
        msg = self._Twist()
        msg.linear.x = float(cmd.linear_mps)
        msg.linear.y = float(cmd.linear_y_mps)
        msg.angular.z = float(cmd.angular_radps)
        try:
            self._pub.publish(msg)
        except Exception as e:  # noqa: BLE001
            log.debug("publish 失败: %s", e)

    def send_zero(self) -> None:
        msg = self._Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        try:
            self._pub.publish(msg)
        except Exception:  # noqa: BLE001
            pass

    def stop(self) -> None:
        self.send_zero()
        log.info("[ROS] STOP")

    def close(self) -> None:
        self.send_zero()

    # 便于上层读取传感器
    def snapshot(self) -> dict:
        with self._lock:
            return {
                "sonar_m": list(self._sonar_m),
                "bat_0_1v": self._bat,
                "temp_0_1c": self._temp,
                "estop_active": self._estop_active,
                "low_bat": self._low_bat,
            }


# ---------------------------------------------------------------------------
# 统一入口
# ---------------------------------------------------------------------------

class RosBridge:
    """算法 → 实车的统一入口。

    用法::

        bridge = RosBridge(backend="mock", wheelbase_m=0.30)
        bridge.publish_cmd_vel(steer_deg=10.0, speed=0.5)
        bridge.stop()
    """

    # 默认最大线速度（m/s）；可由调用方在 publish_cmd_vel 时覆盖
    DEFAULT_MAX_SPEED_MPS = 0.5

    def __init__(
        self,
        backend: str = "mock",
        wheelbase_m: float = 0.30,
        max_speed_mps: float = DEFAULT_MAX_SPEED_MPS,
        sonar_stop_m: float = 0.30,
        bat_warn: int = 70,
        bat_critical: int = 65,
        cmd_vel_topic: str = "/cmd_vel",
    ) -> None:
        self.backend_name = backend
        self.wheelbase_m = float(wheelbase_m)
        self.max_speed_mps = float(max_speed_mps)
        self.sonar_stop_m = float(sonar_stop_m)
        self.bat_warn = int(bat_warn)
        self.bat_critical = int(bat_critical)
        self.cmd_vel_topic = cmd_vel_topic
        self._last: Optional[CmdVel] = None
        if backend == "mock":
            self._backend: _Backend = MockBackend(wheelbase_m=wheelbase_m)
        elif backend in ("ros", "rospy", "ros2"):
            self._backend = RosBackend(
                cmd_vel_topic=cmd_vel_topic,
                wheelbase_m=wheelbase_m,
                sonar_stop_m=sonar_stop_m,
                bat_warn=bat_warn,
                bat_critical=bat_critical,
            )
        else:
            raise ValueError(f"未知 backend: {backend!r}（支持 'mock' / 'ros'）")

    def publish_cmd_vel(
        self,
        steer_deg: float,
        speed: float,
        lateral: float = 0.0,
    ) -> CmdVel:
        """根据 controller.decide 的 (steer_deg, speed) 算出 ROS 用的 (vx, vy, w)。

        - speed 是无量纲（0~1）→ 乘 max_speed_mps 转 m/s
        - steer_deg → 角速度 w = v · tan(steer) / wheelbase（阿克曼近似）
        - lateral：4WD 全向平台才用；阿克曼平台忽略（默认 0）
        """
        v = float(speed) * self.max_speed_mps
        steer_rad = max(min(math.radians(steer_deg), math.radians(60.0)), math.radians(-60.0))
        w = v * math.tan(steer_rad) / self.wheelbase_m
        vy = float(lateral) * self.max_speed_mps
        cmd = CmdVel(
            linear_mps=v, angular_radps=w, linear_y_mps=vy,
            steer_deg=steer_deg, speed=speed, ts=time.monotonic(),
        )
        self._backend.send(cmd)
        self._last = cmd
        return cmd

    def stop(self) -> None:
        self._backend.stop()
        self._last = CmdVel(linear_mps=0.0, angular_radps=0.0, ts=time.monotonic())

    def close(self) -> None:
        self._backend.close()

    @property
    def backend(self) -> _Backend:
        return self._backend

    def snapshot(self) -> dict:
        if hasattr(self._backend, "snapshot"):
            return self._backend.snapshot()  # type: ignore[attr-defined]
        return {}