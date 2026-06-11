#!/usr/bin/env python3
"""leading_line ROS 节点（ROS 1 Noetic，阿克曼底盘）。

订阅摄像头话题 → 跑算法（路径规划 + 箭头 + QR 识别）→ 算 (steer, speed)
→ 转 (linear.x, angular.z) → 发布到 /cmd_vel。

依赖：
  - jetson.algo.color_segmenter / path_planner / controller / visualizer
  - jetson.overrides.FrameOverrides
  - protocol.mode_resolver.select_mode

参数（见 config/params.yaml）:
  ~config_path     算法 YAML 路径
  ~image_topic     输入摄像头话题（默认 /usb_cam/image_raw）
  ~cmd_vel_topic   输出话题（默认 /cmd_vel）
  ~wheelbase_m     阿克曼轴距（米）
  ~max_speed       限速
  ~min_speed       最低速度（低于不发）
  ~publish_hz      /cmd_vel 上限频率
  ~default_mode    启动模式（blue_path / green_path / test）
  ~image_timeout_s 多久没新图像就自动停车

安全：
  - RUNNING 中才发 /cmd_vel；STOP/IDLE 时发 0
  - 摄像头断流超过 image_timeout_s 自动发 0
  - 收到 SIGINT/SIGTERM 时最后发一次 0
"""
from __future__ import annotations

# 直跑兼容：把项目根加进 sys.path，让 from jetson.* import 能解析
import sys
from pathlib import Path
_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import math
import signal
import threading
import time
from typing import Optional

import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from jetson.algo import color_segmenter, controller, path_planner
from jetson.overrides import FrameOverrides
from protocol import select_mode


class LeadingLineNode:
    """ROS 节点：图像 → 算法 → /cmd_vel。"""

    def __init__(self) -> None:
        rospy.init_node("leading_line", log_level=rospy.INFO)
        self.log = rospy.get_logger().getChild("leading_line")

        # ---- 读参数 ----
        cfg_path = rospy.get_param("~config_path", "jetson/config.yaml")
        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.wheelbase = float(rospy.get_param("~wheelbase_m", 0.30))
        self.max_speed = float(rospy.get_param("~max_speed", 0.30))
        self.min_speed = float(rospy.get_param("~min_speed", 0.05))
        self.publish_hz = float(rospy.get_param("~publish_hz", 30.0))
        default_mode = rospy.get_param("~default_mode", "blue_path")
        self.image_timeout_s = float(rospy.get_param("~image_timeout_s", 1.0))

        # ---- 加载算法配置 ----
        cfg_path_abs = str(Path(cfg_path).expanduser().resolve())
        if not Path(cfg_path_abs).exists():
            self.log.error("找不到 config: %s", cfg_path_abs)
            raise SystemExit(1)
        with open(cfg_path_abs, "r", encoding="utf-8") as fp:
            self._raw_cfg = yaml.safe_load(fp)
        self.cfg, self.mode_meta = select_mode(self._raw_cfg, default_mode)
        self.log.info("模式: requested=%s actual=%s label=%s",
                      self.mode_meta.get("requested"),
                      self.mode_meta.get("name"),
                      self.mode_meta.get("label"))

        # ---- 状态 ----
        self._bridge = CvBridge()
        self._frame_lock = threading.Lock()
        self._last_frame: Optional[np.ndarray] = None
        self._last_frame_t: float = 0.0
        self._state: str = "IDLE"      # IDLE / RUNNING / STOPPED
        self._mode: str = self.mode_meta.get("name") or default_mode
        self._smoother = path_planner.PathSmoother(
            alpha=float(self.cfg.get("temporal", {}).get("alpha", 0.4))
        )
        self._overrides = FrameOverrides(self.cfg)
        self._overrides.set_state_change_cb(self._on_qr_state_change)

        # ---- ROS 通信 ----
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.image_sub = rospy.Subscriber(
            self.image_topic, Image, self._on_image, queue_size=1
        )
        rospy.on_shutdown(self._on_shutdown)

        # /cmd_vel 节流定时器
        period = 1.0 / max(self.publish_hz, 1.0)
        self._timer = rospy.Timer(rospy.Duration(period), self._tick)

        # 启动期默认 IDLE；不跑算法
        self.log.info("就绪：image_topic=%s  cmd_vel=%s  wheelbase=%.2fm",
                      self.image_topic, self.cmd_vel_topic, self.wheelbase)
        # 立刻给底盘一个 0 速度（避免 turn_on_wheeltec_robot 1s 超时停车产生的跳动）
        self._publish_twist(0.0, 0.0)

    # ---------------- ROS 回调 ----------------

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:  # noqa: BLE001
            self.log.warn("cv_bridge 失败: %s", e)
            return
        with self._frame_lock:
            self._last_frame = frame
            self._last_frame_t = time.monotonic()

    def _on_qr_state_change(self, old, new) -> None:
        old_s, new_s = str(old), str(new)
        self.log.info("QR 状态机: %s → %s", old_s, new_s)

    def _on_shutdown(self) -> None:
        self.log.info("节点关闭，发最后一次 0 速度")
        self._publish_twist(0.0, 0.0)

    # ---------------- 主循环 ----------------

    def _tick(self, _evt) -> None:
        """每秒最多 publish_hz 次；没新图像就发 0。"""
        with self._frame_lock:
            frame = self._last_frame
            t = self._last_frame_t

        # 摄像头断流超时
        if frame is None or (time.monotonic() - t) > self.image_timeout_s:
            self._publish_twist(0.0, 0.0)
            return

        # IDLE / STOPPED 时不发控制量
        if self._state != "RUNNING":
            self._publish_twist(0.0, 0.0)
            return

        steer, speed = self._process_one_frame(frame)
        v, w = self._steer_to_cmd_vel(steer, speed)
        self._publish_twist(v, w)

    # ---------------- 算法 ----------------

    def _process_one_frame(self, frame: np.ndarray):
        """path → override；返回最终 (steer_deg, speed)。"""
        cfg = self.cfg
        # 1) 颜色分割
        road_mask, floor_mask = color_segmenter.make_masks(
            frame,
            cfg["colors"]["road"]["hsv_lower"],
            cfg["colors"]["road"]["hsv_upper"],
            cfg["colors"]["floor"]["hsv_lower"],
            cfg["colors"]["floor"]["hsv_upper"],
        )
        road_mask = color_segmenter.clean_mask(
            road_mask,
            int(cfg["morphology"]["kernel_size"]),
            int(cfg["morphology"]["opening_iter"]),
            int(cfg["morphology"]["closing_iter"]),
        )
        min_road_px = int(cfg.get("filter", {}).get("min_road_area_px", 0))
        road_mask = color_segmenter.keep_largest_component(road_mask, min_area=min_road_px)

        # 2) 路径规划
        edges = path_planner.plan(road_mask, cfg)
        steer, speed, _lookahead = controller.decide(
            edges["center"], frame.shape[1], cfg["controller"]
        )

        # 3) 时域平滑
        path = edges["center"]
        if cfg.get("temporal", {}).get("reset_on_no_road", True) and (
            path is None or (path.size and np.all(np.isnan(path[:, 0])))
        ):
            self._smoother.reset()
        path = self._smoother.update(path)

        # 4) override 层（箭头 + QR）
        ov = self._overrides.tick(
            frame=frame,
            path_steer=steer,
            path_speed=speed,
            pipeline_running=(self._state == "RUNNING"),
        )
        return ov.steer_deg, ov.speed

    # ---------------- 控制量转换 ----------------

    def _steer_to_cmd_vel(self, steer_deg: float, speed: float) -> tuple[float, float]:
        """(steer_deg, speed) → (linear.x, angular.z) for Ackermann."""
        v = float(np.clip(speed, 0.0, self.max_speed))
        if abs(v) < self.min_speed:
            v = 0.0
        steer_rad = math.radians(steer_deg)
        # 阿克曼：w = v · tan(steer) / L
        w = v * math.tan(steer_rad) / max(self.wheelbase, 1e-3)
        return v, w

    def _publish_twist(self, v: float, w: float) -> None:
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        try:
            self.cmd_pub.publish(msg)
        except Exception as e:  # noqa: BLE001
            self.log.warn("publish 失败: %s", e)

    # ---------------- 外部命令（service / topic 也可，这里用 service） ----------------

    def set_state(self, state: str) -> None:
        assert state in ("IDLE", "RUNNING", "STOPPED")
        if state == self._state:
            return
        self.log.info("状态切换: %s → %s", self._state, state)
        if state in ("IDLE", "STOPPED"):
            self._overrides.on_stop()
            self._publish_twist(0.0, 0.0)
        elif state == "RUNNING":
            self._overrides.on_start()
        self._state = state

    def set_mode(self, mode_name: str) -> None:
        """运行时切模式：从 raw_cfg 重新 select_mode，更新 colors/visualization。"""
        self.cfg, meta = select_mode(self._raw_cfg, mode_name)
        self._mode = meta.get("name") or mode_name
        # override 层也要用新 cfg 重新初始化
        self._overrides = FrameOverrides(self.cfg)
        self._overrides.set_state_change_cb(self._on_qr_state_change)
        self.log.info("模式切换: %s (label=%s)", self._mode, meta.get("label"))


def main() -> int:
    node = None
    try:
        node = LeadingLineNode()
    except SystemExit as e:
        return int(e.code) if isinstance(e.code, int) else 1
    except Exception as e:  # noqa: BLE001
        rospy.logfatal("节点启动失败: %s", e)
        return 1

    # 注册一个 SetBool 服务让外部（PC / 测试脚本）能切 RUNNING / STOPPED
    from std_srvs.srv import SetBool, SetBoolResponse
    from std_srvs.srv import SetBoolRequest

    def _on_start_stop(req: SetBoolRequest):
        node.set_state("RUNNING" if req.data else "STOPPED")
        return SetBoolResponse(success=True, message=node._state)

    _ = rospy.Service("~start_stop", SetBool, _on_start_stop)

    # 模式切换服务：req.data 是模式名（blue_path / green_path / test）
    def _on_set_mode(req: SetBoolRequest):
        node.set_mode(req.data)
        return SetBoolResponse(success=True, message=node._mode)

    _ = rospy.Service("~set_mode", SetBool, _on_set_mode)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
