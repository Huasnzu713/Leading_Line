#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations

# 直跑兼容：把项目根加进 sys.path，让 from vehicle.* import 能解析
import sys
from pathlib import Path
_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import json
import threading
import time
from typing import List, Optional

import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from vehicle.algo import color_segmenter, controller, path_planner
from vehicle.overrides import FrameOverrides
from vehicle.ros_bridge import RosBridge
from protocol import select_mode


class LeadingLineNode:
    """ROS 节点：图像 → 算法 → RosBridge → /cmd_vel。"""

    def __init__(self) -> None:
        rospy.init_node("leading_line", log_level=rospy.INFO)
        self.log = rospy.get_logger().get_child("leading_line")

        # ---- 读参数 ----
        cfg_path = rospy.get_param("~config_path", "config.yaml")
        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.wheelbase = float(rospy.get_param("~wheelbase_m", 0.30))
        self.max_speed = float(rospy.get_param("~max_speed", 0.30))
        self.min_speed = float(rospy.get_param("~min_speed", 0.05))
        self.publish_hz = float(rospy.get_param("~publish_hz", 30.0))
        default_mode = rospy.get_param("~default_mode", "blue_path")
        self.image_timeout_s = float(rospy.get_param("~image_timeout_s", 1.0))
        sonar_stop_m = float(rospy.get_param("~sonar_stop_m", 0.30))
        sonar_topics_raw = rospy.get_param(
            "~sonar_topics",
            '["/xcar/sonar1","/xcar/sonar2","/xcar/sonar3","/xcar/sonar4"]',
        )
        if isinstance(sonar_topics_raw, str):
            try:
                self.sonar_topics: List[str] = json.loads(sonar_topics_raw)
            except Exception:  # noqa: BLE001
                self.sonar_topics = [t.strip() for t in sonar_topics_raw.split(",") if t.strip()]
        else:
            self.sonar_topics = list(sonar_topics_raw)

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

        # ---- RosBridge（运动学换算 + /cmd_vel Publisher + sonar/bat 安全闸）----
        self.ros_bridge = RosBridge(
            backend="ros",
            wheelbase_m=self.wheelbase,
            max_speed_mps=self.max_speed,
            sonar_stop_m=sonar_stop_m,
            cmd_vel_topic=self.cmd_vel_topic,
        )
        # 重新挂上我们这节点的 sonar 话题
        self.ros_bridge.backend.sonar_topics = self.sonar_topics  # type: ignore[attr-defined]
        # 如果是 RosBackend，订阅之（RosBridge 已尝试订阅过，这里补订一遍最新列表）
        if hasattr(self.ros_bridge.backend, "_rospy") and self.ros_bridge.backend._rospy is not None:  # type: ignore[attr-defined]
            from sensor_msgs.msg import Range  # type: ignore
            for i, t in enumerate(self.sonar_topics):
                try:
                    self.ros_bridge.backend._rospy.Subscriber(  # type: ignore[attr-defined]
                        t, Range, self.ros_bridge.backend._make_sonar_cb(i), queue_size=1  # type: ignore[attr-defined]
                    )
                except Exception as e:  # noqa: BLE001
                    self.log.warn("sonar 订阅失败 %s: %s", t, e)

        # ---- ROS 通信（图像订阅 + timer）----
        self.image_sub = rospy.Subscriber(
            self.image_topic, Image, self._on_image, queue_size=1
        )
        rospy.on_shutdown(self._on_shutdown)

        # /cmd_vel 节流定时器
        period = 1.0 / max(self.publish_hz, 1.0)
        self._timer = rospy.Timer(rospy.Duration(period), self._tick)

        self.log.info("就绪：image_topic=%s  cmd_vel=%s  wheelbase=%.2fm  sonar_stop=%.2fm",
                      self.image_topic, self.cmd_vel_topic, self.wheelbase, sonar_stop_m)
        # 启动期先发一次 0 速度（避免 xcar 1s 没 /cmd_vel 自动停的"跳动"）
        self.ros_bridge.stop()

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
        try:
            self.ros_bridge.stop()
        except Exception:  # noqa: BLE001
            pass

    # ---------------- 主循环 ----------------

    def _tick(self, _evt) -> None:
        """每秒最多 publish_hz 次。"""
        with self._frame_lock:
            frame = self._last_frame
            t = self._last_frame_t

        # 摄像头断流超时
        if frame is None or (time.monotonic() - t) > self.image_timeout_s:
            self.ros_bridge.stop()
            return

        # IDLE / STOPPED 时不发控制量
        if self._state != "RUNNING":
            self.ros_bridge.stop()
            return

        steer, speed = self._process_one_frame(frame)
        # 速度裁剪
        speed = float(np.clip(speed, 0.0, 1.0))
        if speed < self.min_speed / max(self.max_speed, 1e-3):
            speed = 0.0
        # RosBridge 负责：(steer, speed) → (vx, vy, w) + sonar/bat 安全 + /cmd_vel
        self.ros_bridge.publish_cmd_vel(steer_deg=steer, speed=speed, lateral=0.0)

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

    # ---------------- 外部命令 ----------------

    def set_state(self, state: str) -> None:
        assert state in ("IDLE", "RUNNING", "STOPPED")
        if state == self._state:
            return
        self.log.info("状态切换: %s → %s", self._state, state)
        if state in ("IDLE", "STOPPED"):
            self._overrides.on_stop()
            self.ros_bridge.stop()
        elif state == "RUNNING":
            self._overrides.on_start()
        self._state = state

    def set_mode(self, mode_name: str) -> None:
        """运行时切模式：从 raw_cfg 重新 select_mode，更新 colors/visualization。"""
        self.cfg, meta = select_mode(self._raw_cfg, mode_name)
        self._mode = meta.get("name") or mode_name
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

    from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest

    def _on_start_stop(req: SetBoolRequest):
        node.set_state("RUNNING" if req.data else "STOPPED")
        return SetBoolResponse(success=True, message=node._state)

    _ = rospy.Service("~start_stop", SetBool, _on_start_stop)

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