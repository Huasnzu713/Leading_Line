"""Jetson 主流水线：摄像头 → 算法 → override → 渲染 → UDP 发图，同时响应 TCP 命令。

关键设计：
- 单线程：摄像头 read 是阻塞的，JPEG 编码/UDP 发包都是 ms 级，没必要多线程
- TCP 命令是非阻塞轮询：get(timeout=0) 立即拿，没命令就继续做算法
- 模式切换是"读 cfg["modes"][name]"的快路径：换模式不重启摄像头
- ROS 接口是 publish_cmd_vel(steer, speed)；未收到 START 时不发任何指令
- override 层：箭头 + QR 在路径算法之上叠加；QR POLICY_ACTIVE 时直接接管
- 状态回推：每秒一次 STATUS 文本回包；状态切换 / QR 状态变化时立即推
- 退出由 signal 或 TCP 收到 QUIT 触发；finally 里关摄像头/UDP/TCP/ROS
"""
from __future__ import annotations

import logging
import signal
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

from jetson.algo import color_segmenter, controller, path_planner, visualizer
from jetson.recognition.arrow.arrow_detector import annotate as arrow_annotate
from protocol import (
    STATE_ERROR,
    STATE_IDLE,
    STATE_RUNNING,
    STATE_STOPPED,
    select_mode,
)
from jetson.recognition.qr.qr_decoder import draw_qr_overlay

from .overrides import (
    FrameOverrides,
    SRC_ARROW,
    SRC_QR,
    SRC_PATH,
)
from .ros_bridge import RosBridge

log = logging.getLogger(__name__)


# 主动状态回推的最小间隔（秒）；状态切换/QK 变化时仍会立即推
_STATUS_PUSH_PERIOD_S = 1.0
# 状态回推里 fps/mode 等的格式
_STATUS_FMT = "state={state} mode={mode} fps={fps:.1f} ros={ros} clients={clients}"


@dataclass
class _Runtime:
    mode: str = "blue_path"
    state: str = STATE_IDLE
    last_cmd_kind: str = ""
    frames: int = 0
    last_fps_t: float = 0.0
    fps: float = 0.0
    last_status_t: float = 0.0
    last_source: str = SRC_PATH
    last_qr_state: str = "IDLE"


class Pipeline:
    """在 Jetson 上跑：采集 + 算法 + override + UDP 推流 + 响应命令。"""

    def __init__(
        self,
        cfg: dict,
        video_sender,
        cmd_receiver,
        ros_bridge: RosBridge,
        jpeg_quality: int = 70,
        fps_cap: float = 30.0,
    ) -> None:
        self.cfg = cfg
        self.video_sender = video_sender
        self.cmd_receiver = cmd_receiver
        self.ros = ros_bridge
        self.jpeg_quality = int(jpeg_quality)
        self.fps_cap = float(fps_cap)
        self.rt = _Runtime()
        self._stop = False
        self._smoother = None
        self._cap: Optional[cv2.VideoCapture] = None
        self._effective_cfg: dict = {}
        self._mode_meta: dict = {}
        # override 层（箭头 + QR）；配置里 overrides.* 关闭时各模块静默 no-op
        self._overrides = FrameOverrides(cfg)
        self._overrides.set_state_change_cb(self._on_qr_state_change)
        # 默认载入 blue_path
        self._apply_mode(self.rt.mode)

    # ----- 模式 / 状态切换 -----

    def _apply_mode(self, mode_name: str) -> None:
        eff, meta = select_mode(self.cfg, mode_name)
        self._effective_cfg = eff
        self._mode_meta = meta
        self.rt.mode = meta.get("name") or mode_name
        log.info(
            "切换模式: requested=%s actual=%s label=%s fallback=%s",
            meta.get("requested"), meta.get("name"), meta.get("label"), meta.get("fallback"),
        )

    def _set_state(self, new_state: str) -> None:
        if new_state == self.rt.state:
            return
        log.info("状态切换: %s → %s", self.rt.state, new_state)
        if new_state in (STATE_IDLE, STATE_STOPPED):
            # 安全：停车
            self.ros.stop()
            self._overrides.on_stop()
        self.rt.state = new_state
        # 立即推一条 STATUS
        self._push_status_now(f"state_change:{new_state}")

    def _on_qr_state_change(self, old, new) -> None:
        old_s, new_s = str(old), str(new)
        log.info("QR 状态机: %s → %s", old_s, new_s)
        self.rt.last_qr_state = new_s
        self.cmd_receiver.push_status("INFO", f"qr_state:{old_s}->{new_s}")
        if new_s == "REPORTING":
            # 状态机报告：让 PC 知道需要 ACK
            self.cmd_receiver.push_status("STATUS", "REPORTING")

    # ----- 状态回推 -----

    def _push_status_now(self, reason: str = "") -> None:
        n = self.cmd_receiver.push_status(
            "STATUS",
            _STATUS_FMT.format(
                state=self.rt.state,
                mode=self.rt.mode,
                fps=self.rt.fps,
                ros=self.ros.backend_name,
                clients=self.cmd_receiver.client_count(),
            ),
        )
        if n:
            log.debug("STATUS 推给 %d 个客户端 (%s)", n, reason)

    def _maybe_periodic_status(self) -> None:
        now = time.monotonic()
        if now - self.rt.last_status_t < _STATUS_PUSH_PERIOD_S:
            return
        if not self.cmd_receiver.has_clients():
            return
        self._push_status_now("periodic")
        self.rt.last_status_t = now

    # ----- 命令处理 -----

    def _handle_cmd(self, cmd) -> None:
        kind = cmd.kind.upper()
        payload = cmd.payload.strip()
        self.rt.last_cmd_kind = kind
        if kind == "MODE":
            self._apply_mode(payload)
        elif kind == "START":
            self._set_state(STATE_RUNNING)
            self._overrides.on_start()
        elif kind == "STOP":
            self._set_state(STATE_STOPPED)
        elif kind == "PING":
            # CommandReceiver 内部已经回 PONG；这里不做事
            pass
        elif kind == "ACK":
            # PC 回 ack 了 REPORTING
            self._overrides.on_ack()
        elif kind == "INFO":
            # 调试用：PC 主动发个 INFO 文本到 Jetson
            log.info("PC INFO: %s", payload)
        elif kind == "QUIT":
            log.info("收到 QUIT，准备退出")
            self._stop = True
            self._set_state(STATE_IDLE)
        else:
            log.warning("未知命令: %s %r", kind, payload)

    # ----- 算法处理一帧 -----

    def _process_frame(self, frame):
        cfg = self._effective_cfg
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
        edges = path_planner.plan(road_mask, cfg)
        steer, speed, lookahead = controller.decide(
            edges["center"], frame.shape[1], cfg["controller"]
        )
        # 时域平滑
        if self._smoother is None:
            from jetson.algo.path_planner import PathSmoother
            self._smoother = PathSmoother(alpha=float(cfg.get("temporal", {}).get("alpha", 0.4)))
        path = edges["center"]
        if cfg.get("temporal", {}).get("reset_on_no_road", True) and (
            path is None or (path.size and np.all(np.isnan(path[:, 0])))
        ):
            self._smoother.reset()
        path = self._smoother.update(path)
        edges = {**edges, "center": path}

        # override 层（箭头 + QR）—— 不改 path 输出，只决定最终下发的 (steer, speed)
        ov = self._overrides.tick(
            frame=frame,
            path_steer=steer,
            path_speed=speed,
            pipeline_running=(self.rt.state == STATE_RUNNING),
        )
        final_steer, final_speed = ov.steer_deg, ov.speed
        self.rt.last_source = ov.source
        if ov.qr_state is not None:
            self.rt.last_qr_state = str(ov.qr_state)

        # 只在 RUNNING 时把控制量推到 ROS
        if self.rt.state == STATE_RUNNING:
            self.ros.publish_cmd_vel(final_steer, final_speed)

        return road_mask, floor_mask, edges, final_steer, final_speed, lookahead, ov

    # ----- 主循环 -----

    def _open_camera(self) -> cv2.VideoCapture:
        cam_cfg = self.cfg["camera"]
        cap = cv2.VideoCapture(int(cam_cfg["index"]))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cam_cfg["width"]))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cam_cfg["height"]))
        cap.set(cv2.CAP_PROP_FRAME_FPS, int(cam_cfg.get("fps", 30)))
        if not cap.isOpened():
            raise RuntimeError(f"无法打开摄像头 index={cam_cfg['index']}")
        return cap

    def run(self) -> None:
        signal.signal(signal.SIGINT, self._on_signal)
        signal.signal(signal.SIGTERM, self._on_signal)

        self.cmd_receiver.start()
        # 启动时推一条 STATUS（让 PC 一连上就知道 Jetson 真实状态）
        self._push_status_now("startup")
        self._cap = self._open_camera()
        log.info("摄像头已打开，开始主循环")
        self.rt.last_fps_t = time.monotonic()
        self.rt.last_status_t = time.monotonic()
        min_dt = 1.0 / max(self.fps_cap, 1.0)

        try:
            while not self._stop:
                loop_t0 = time.monotonic()

                # 1) 拉命令（非阻塞）
                cmd = self.cmd_receiver.get(timeout=0.0)
                if cmd is not None:
                    self._handle_cmd(cmd)

                # 2) 读一帧
                ok, frame = self._cap.read()
                if not ok or frame is None:
                    log.warning("摄像头掉线，尝试重连...")
                    self._set_state(STATE_ERROR)
                    time.sleep(0.5)
                    self._cap.release()
                    self._cap = self._open_camera()
                    continue

                # 3) 算法处理 + override
                try:
                    road_mask, floor_mask, edges, steer, speed, lookahead, ov = (
                        self._process_frame(frame)
                    )
                except Exception as e:  # noqa: BLE001
                    log.exception("算法处理异常: %s", e)
                    self._set_state(STATE_ERROR)
                    continue

                # 4) 渲染（用当前模式的可视化配色；source 标到 HUD 方便调试）
                vis = visualizer.draw(
                    frame, road_mask, edges, steer, speed, lookahead,
                    self._effective_cfg["visualization"], floor_mask=floor_mask,
                )
                vis = self._draw_override_hud(vis, ov)
                # 5) 编码 + UDP 发图
                ok, buf = cv2.imencode(
                    ".jpg", vis,
                    [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
                )
                if ok:
                    self.video_sender.send(buf.tobytes(), ts_ms=int(time.time() * 1000))

                # 6) FPS 统计
                self.rt.frames += 1
                now = time.monotonic()
                if now - self.rt.last_fps_t >= 1.0:
                    self.rt.fps = self.rt.frames / (now - self.rt.last_fps_t)
                    self.rt.frames = 0
                    self.rt.last_fps_t = now
                    log.debug(
                        "FPS=%.1f mode=%s state=%s ros=%s src=%s qr=%s",
                        self.rt.fps, self.rt.mode, self.rt.state,
                        self.ros.backend_name, self.rt.last_source, self.rt.last_qr_state,
                    )

                # 7) 主动状态回推（每秒一次）
                self._maybe_periodic_status()

                # 8) 节流到目标帧率
                dt = time.monotonic() - loop_t0
                if dt < min_dt:
                    time.sleep(min_dt - dt)
        finally:
            log.info("Pipeline 退出，关闭资源")
            self._push_status_now("shutdown")
            self.ros.stop()
            if self._cap is not None:
                self._cap.release()
            self.cmd_receiver.close()
            self.video_sender.close()
            self.ros.close()

    # ----- HUD -----

    def _draw_override_hud(self, vis: np.ndarray, ov) -> np.ndarray:
        """画 3 个东西：QR 外框、箭头标注、左下 source HUD 文字。返回新图。"""
        try:
            # 1) QR 外框
            qr_obj = ov.debug.get("last_qr_obj")
            if qr_obj is not None:
                vis = draw_qr_overlay(vis, qr_obj, color_bgr=(0, 255, 255), thickness=2)
            # 2) 箭头标注（直接覆盖 vis）
            if ov.arrow is not None:
                vis = arrow_annotate(vis, ov.arrow)
            # 3) HUD
            lines = [
                f"src={ov.source}",
                f"qr={self.rt.last_qr_state}",
            ]
            if ov.arrow is not None:
                lines.append(
                    f"arr={ov.arrow.direction}/{ov.arrow.confidence:.2f}"
                )
            text = " | ".join(lines)
            cv2.putText(
                vis, text, (10, vis.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA,
            )
        except Exception as e:  # noqa: BLE001
            log.debug("HUD 绘制异常: %s", e)
        return vis

    def _on_signal(self, signum, _frame) -> None:
        log.info("收到信号 %s，退出中", signum)
        self._stop = True
