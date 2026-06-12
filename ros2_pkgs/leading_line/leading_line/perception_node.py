# -*- coding: utf-8 -*-
"""perception_node.py — ROS2 节点：/image → algo → /ackermann_cmd。"""
from __future__ import annotations

import logging
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import SetBool

from leading_line.ackermann_bridge import make_rclpy_bridge
from leading_line.perception_pipeline import PerceptionPipeline
from protocol import select_mode

log = logging.getLogger(__name__)


class PerceptionNode(Node):
    """ROS2 节点：图像 → 算法 → ackermann_msgs。"""

    def __init__(self) -> None:
        super().__init__("leading_line")

        # ---- 读参数 ----
        self.declare_parameter("config_path", "config.yaml")
        self.declare_parameter("image_topic", "/usb_cam/image_raw")
        self.declare_parameter("ackermann_topic", "/ackermann_cmd")
        self.declare_parameter("wheelbase_m", 0.30)
        self.declare_parameter("max_steer_deg", 30.0)
        self.declare_parameter("max_speed_mps", 0.30)
        self.declare_parameter("min_speed_mps", 0.05)
        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("default_mode", "blue_path")
        self.declare_parameter("image_timeout_s", 1.0)
        self.declare_parameter("publish_debug_image", False)
        # 急停 / 低电量
        self.declare_parameter("sonar_stop_m", 0.30)
        self.declare_parameter("sonar_topics", [
            "/xcar/sonar1", "/xcar/sonar2", "/xcar/sonar3", "/xcar/sonar4",
        ])
        self.declare_parameter("bat_warn_0p1V", 70)
        self.declare_parameter("bat_critical_0p1V", 65)

        p = self.get_parameters_by_prefix("")
        cfg_path = str(p["config_path"].value)
        self._image_topic = str(p["image_topic"].value)
        self._ackermann_topic = str(p["ackermann_topic"].value)
        self._max_steer_deg = float(p["max_steer_deg"].value)
        self._max_speed_mps = float(p["max_speed_mps"].value)
        self._min_speed_mps = float(p["min_speed_mps"].value)
        self._publish_hz = float(p["publish_hz"].value)
        self._default_mode = str(p["default_mode"].value)
        self._image_timeout_s = float(p["image_timeout_s"].value)
        self._publish_debug_image = bool(p["publish_debug_image"].value)
        self._sonar_stop_m = float(p["sonar_stop_m"].value)
        self._sonar_topics = list(p["sonar_topics"].value)
        self._bat_warn = int(p["bat_warn_0p1V"].value)
        self._bat_critical = int(p["bat_critical_0p1V"].value)

        # ---- 加载算法配置 ----
        cfg_abs = str(Path(cfg_path).expanduser().resolve())
        if not Path(cfg_abs).exists():
            self.get_logger().error(f"找不到 config: {cfg_abs}")
            raise SystemExit(1)
        with open(cfg_abs, "r", encoding="utf-8") as fp:
            self._raw_cfg = yaml.safe_load(fp)
        self._eff_cfg, self._mode_meta = select_mode(self._raw_cfg, self._default_mode)
        self.get_logger().info(
            f"模式: requested={self._mode_meta.get('requested')} "
            f"actual={self._mode_meta.get('name')} label={self._mode_meta.get('label')}"
        )

        # ---- ackermann bridge ----
        self._bridge = make_rclpy_bridge(
            self,
            max_speed_mps=self._max_speed_mps,
            max_steer_deg=self._max_steer_deg,
            topic=self._ackermann_topic,
        )

        # ---- 算法 pipeline ----
        self._pipeline = PerceptionPipeline(self._eff_cfg)

        # ---- 状态 ----
        self._cv = CvBridge()
        self._frame_lock = threading.Lock()
        self._last_frame: Optional[np.ndarray] = None
        self._last_frame_t: float = 0.0
        self._state: str = "IDLE"  # IDLE / RUNNING / STOPPED
        # 急停
        self._sonar_lock = threading.Lock()
        self._sonar_m: dict = {t: 999.0 for t in self._sonar_topics}
        self._sonar_ts: dict = {t: 0.0 for t in self._sonar_topics}
        self._bat_0p1V: int = 0
        self._estop_active: bool = False
        self._low_bat: bool = False

        # ---- ROS 通信 ----
        self._image_sub = self.create_subscription(
            Image, self._image_topic, self._on_image, 1
        )
        self._ackermann_pub = self.create_publisher(
            AckermannDriveStamped, self._ackermann_topic, 10
        )
        # debug 标注图（PC 监控推流用）
        if self._publish_debug_image:
            self._debug_pub = self.create_publisher(
                Image, "/debug/annotated_image", 5
            )
        else:
            self._debug_pub = None
        # 服务
        self._start_stop_srv = self.create_service(
            SetBool, "~/start_stop", self._on_start_stop
        )
        self._set_mode_srv = self.create_service(
            SetBool, "~/set_mode", self._on_set_mode
        )
        # 急停订阅
        for t in self._sonar_topics:
            self.create_subscription(Range, t, self._make_sonar_cb(t), 1)
        # 电池状态
        self.create_subscription(Int32MultiArray, "/xcar/sensors", self._on_sensors, 1)

        # tick timer
        period = 1.0 / max(self._publish_hz, 1.0)
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"就绪: image={self._image_topic} cmd={self._ackermann_topic} "
            f"max_steer={self._max_steer_deg}deg max_speed={self._max_speed_mps}m/s "
            f"sonar_stop={self._sonar_stop_m}m"
        )
        # 启动时发 0
        self._bridge.stop()

    # ---- 急停 / 电池 ----
    def _make_sonar_cb(self, topic: str):
        def _cb(msg):
            r = float(getattr(msg, "range", 999.0))
            with self._sonar_lock:
                self._sonar_m[topic] = r
                self._sonar_ts[topic] = time.monotonic()
        return _cb

    def _on_sensors(self, msg):
        data = list(getattr(msg, "data", []) or [])
        if len(data) < 1:
            return
        self._bat_0p1V = int(data[0])
        if self._bat_0p1V and self._bat_0p1V <= self._bat_critical:
            if not self._low_bat:
                self.get_logger().error(f"电量过低 ({self._bat_0p1V / 10.0:.1f}V) 强制停车")
                self._low_bat = True
        elif self._bat_0p1V and self._bat_0p1V <= self._bat_warn:
            self.get_logger().warning(f"电量偏低 ({self._bat_0p1V / 10.0:.1f}V)")

    def _check_estop(self) -> bool:
        if self._sonar_stop_m < 0:
            return False
        now = time.monotonic()
        with self._sonar_lock:
            fresh = [
                self._sonar_m[t] for t in self._sonar_topics
                if (now - self._sonar_ts[t]) < 1.0
            ]
        if not fresh:
            return False
        if min(fresh) < self._sonar_stop_m:
            if not self._estop_active:
                self.get_logger().warning(
                    f"紧急停车：最近障碍 {min(fresh):.2f}m < {self._sonar_stop_m:.2f}m"
                )
                self._estop_active = True
            return True
        self._estop_active = False
        return False

    def _check_battery(self) -> bool:
        return self._bat_0p1V > 0 and self._bat_0p1V <= self._bat_critical

    # ---- 图像回调 ----
    def _on_image(self, msg: Image) -> None:
        try:
            frame = self._cv.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"cv_bridge 失败: {e}")
            return
        with self._frame_lock:
            self._last_frame = frame
            self._last_frame_t = time.monotonic()

    # ---- 服务 ----
    def _on_start_stop(self, req, resp):
        new_state = "RUNNING" if req.data else "STOPPED"
        self._set_state(new_state)
        resp.success = True
        resp.message = self._state
        return resp

    def _on_set_mode(self, req, resp):
        mode_name = str(req.data) if isinstance(req.data, str) else ""
        # SetBool.data 在 std_srvs 是 bool；这里改用 data 字段当 mode name 需改 srv 类型。
        # 兼容：req.data 强制当 str 处理（很多 ROS1 SetBool 实现就这么干）
        self._set_mode(mode_name)
        resp.success = True
        resp.message = self._mode_meta.get("name", mode_name)
        return resp

    def _set_state(self, new_state: str) -> None:
        if new_state == self._state:
            return
        self.get_logger().info(f"状态切换: {self._state} → {new_state}")
        if new_state in ("IDLE", "STOPPED"):
            self._pipeline._overrides.on_stop()
            self._bridge.stop()
        elif new_state == "RUNNING":
            self._pipeline._overrides.on_start()
        self._state = new_state

    def _set_mode(self, mode_name: str) -> None:
        eff, meta = select_mode(self._raw_cfg, mode_name)
        self._eff_cfg = eff
        self._mode_meta = meta
        self._pipeline.set_mode_cfg(eff)
        self.get_logger().info(f"模式切换: {meta.get('name')} (label={meta.get('label')})")

    # ---- 主循环 ----
    def _tick(self) -> None:
        # 急停 / 电池
        if self._check_estop() or self._check_battery():
            self._bridge.stop()
            return
        # 图像 timeout
        with self._frame_lock:
            frame = self._last_frame
            t = self._last_frame_t
        if frame is None or (time.monotonic() - t) > self._image_timeout_s:
            self._bridge.stop()
            return
        if self._state != "RUNNING":
            self._bridge.stop()
            return

        # 处理
        try:
            result = self._pipeline.process_frame(frame, pipeline_running=True)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"算法处理异常: {e}")
            self._bridge.stop()
            return

        steer = float(result["steer_deg"])
        speed = float(result["speed"])
        # 速度 clip
        speed = float(np.clip(speed, 0.0, 1.0))
        if speed < self._min_speed_mps / max(self._max_speed_mps, 1e-3):
            speed = 0.0
        self._bridge.publish(steer, speed)

        # 调试图（可选）
        if self._debug_pub is not None:
            try:
                vis = result.get("edges", {}).get("center", None)
                # 简化：在原帧上画路径 + HUD
                dbg_img = frame.copy()
                if vis is not None and vis.size:
                    pts = vis.astype(int)
                    for i in range(len(pts) - 1):
                        cv2 = __import__("cv2")
                        cv2.line(dbg_img, tuple(pts[i]), tuple(pts[i + 1]), (0, 255, 0), 2)
                cv2 = __import__("cv2")
                cv2.putText(
                    dbg_img, f"steer={steer:+.1f} spd={speed:.2f} src={result['source']}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA,
                )
                self._debug_pub.publish(self._cv.cv2_to_imgmsg(dbg_img, "bgr8"))
            except Exception:  # noqa: BLE001
                pass


def main(args=None) -> int:
    rclpy.init(args=args)
    node = PerceptionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._bridge.stop()  # noqa: SLF001
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
