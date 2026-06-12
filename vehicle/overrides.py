# -*- coding: utf-8 -*-
"""路径识别之外的"覆盖层"。

把箭头方向识别和二维码状态机叠在路径算法之上：

```
   frame
     │
     ▼
 path → (steer_path, speed_path)        ← color/planner/controller
     │
     ▼
 FrameOverrides.tick(frame, dt)         ← 本模块
     │
     ├── state_machine  → (steer_qr,  speed_qr)  优先级最高
     ├── arrow_detector    → (steer_arr, —)         中优先级
     │
     ▼
 final (steer, speed, source, debug)
```

优先级（高 → 低）：
1. **QR 策略**：QR 状态机 `POLICY_ACTIVE` 时直接接管 (steer, speed)
   - `CRUISE`（NaN, NaN）特殊：不接管，回到 path 输出
2. **箭头方向**：高置信度时把 `steer` 替换成预定义角度（一帧立刻生效）
3. **路径算法**：(steer_path, speed_path) 是兜底

每个 override 都可以独立打开/关掉（cfg.overrides.arrow.enabled / cfg.overrides.qr.enabled）。
"""
from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

from vehicle.recognition.arrow_detector import ArrowResult, detect_arrow
from vehicle.recognition.qr.decoder import DecodedQR, decode_qr_codes
from vehicle.recognition.qr.state_machine import QRStateMachine, State as QRState

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# 结果
# ---------------------------------------------------------------------------

# source 字符串常量
SRC_PATH = "path"
SRC_ARROW = "arrow"
SRC_QR = "qr"

# 箭头方向 → 转向角（度）映射；置信度阈值之上才生效
_ARROW_TO_STEER_DEG = {
    "前": 0.0,
    "左": -22.0,
    "右": 22.0,
}
_ARROW_UNKNOWN_STEER_DEG = 0.0
_ARROW_MIN_CONFIDENCE = 0.5

# 箭头 override 的"单次持续时间"——超过这个时间就让 path 接管（避免卡死）
_ARROW_OVERRIDE_TTL_S = 0.6


@dataclass
class OverrideResult:
    """FrameOverrides.tick 的返回值。"""
    steer_deg: float
    speed: float
    source: str = SRC_PATH      # "path" / "arrow" / "qr"
    arrow: Optional[ArrowResult] = None
    qr_state: Optional[QRState] = None
    qr_text: str = ""
    debug: dict = field(default_factory=dict)


# ---------------------------------------------------------------------------
# 主类
# ---------------------------------------------------------------------------

class FrameOverrides:
    """叠在路径算法之上的两个识别器：箭头 + QR。

    构造时不读摄像头；每帧调一次 ``tick``，内部会按配置的"解码间隔"
    决定是否真去 decode QR（QR 解码 CPU 重，不每帧都跑）。
    """

    def __init__(self, cfg: dict) -> None:
        ov_cfg = cfg.get("overrides", {}) or {}
        # ---- 箭头 ----
        arr_cfg = ov_cfg.get("arrow", {}) or {}
        self.arrow_enabled: bool = bool(arr_cfg.get("enabled", True))
        self.arrow_min_darkness: float = float(arr_cfg.get("min_darkness", 80.0))
        self.arrow_min_conf: float = float(arr_cfg.get("min_confidence", _ARROW_MIN_CONFIDENCE))
        self.arrow_steer_map: dict[str, float] = dict(_ARROW_TO_STEER_DEG)
        custom = arr_cfg.get("steer_map") or {}
        for k, v in custom.items():
            self.arrow_steer_map[k] = float(v)
        self.arrow_ttl_s: float = float(arr_cfg.get("override_ttl_s", _ARROW_OVERRIDE_TTL_S))
        # ---- QR ----
        qr_cfg = ov_cfg.get("qr", {}) or {}
        self.qr_enabled: bool = bool(qr_cfg.get("enabled", True))
        self.qr_decode_every_n: int = max(1, int(qr_cfg.get("decode_every_n", 6)))
        self.policy_timeout_s: float = float(qr_cfg.get("policy_timeout_s", 30.0))
        self.qr_on_state_change: Optional[callable] = None  # 主循环注入：状态变化时打 INFO

        self._sm: Optional[QRStateMachine] = None
        if self.qr_enabled:
            self._sm = QRStateMachine(policy_timeout_s=self.policy_timeout_s)
            if self.qr_on_state_change:
                self._sm.on_state_change(self.qr_on_state_change)

        # 状态
        self._frame_idx: int = 0
        self._last_arrow: Optional[ArrowResult] = None
        self._last_arrow_at: float = 0.0
        self._last_qr_text: str = ""
        self._last_qr_decoded: Optional[DecodedQR] = None
        self._last_qr_at: float = 0.0
        self._last_qr_state: Optional[QRState] = None
        self._stats: dict = {
            "arrow_detects": 0,
            "arrow_overrides": 0,
            "qr_decodes": 0,
            "qr_policies": 0,
        }

    # ---- 公开属性 ----
    @property
    def stats(self) -> dict:
        return dict(self._stats)

    @property
    def last_qr_state(self) -> Optional[QRState]:
        return self._last_qr_state

    def set_state_change_cb(self, cb) -> None:
        """主循环注入：QR 状态变化时回调 (old, new)。"""
        self.qr_on_state_change = cb
        if self._sm is not None:
            self._sm.on_state_change(cb)

    # ---- 主入口 ----
    def tick(
        self,
        frame: np.ndarray,
        path_steer: float,
        path_speed: float,
        pipeline_running: bool,
    ) -> OverrideResult:
        """每帧调一次；返回最终 (steer, speed)。

        ``pipeline_running=False`` 时（即 START 还没发）—— 不做 override，
        直接返回 path 输出，避免误触发。
        """
        self._frame_idx += 1
        now = time.monotonic()

        # 默认：path 输出
        steer, speed, source = path_steer, path_speed, SRC_PATH
        arrow_res: Optional[ArrowResult] = None
        qr_state: Optional[QRState] = None
        qr_text = ""
        debug: dict = {}

        if not pipeline_running:
            return OverrideResult(
                steer_deg=steer, speed=speed, source=source,
                arrow=None, qr_state=None, qr_text="", debug={"skip": "not_running"},
            )

        # 1) QR 状态机：先看它有没有"接管信号"
        if self.qr_enabled and self._sm is not None:
            qr_steer, qr_speed = self._sm.tick(dt=0.0)  # tick 自己用内部时钟
            qr_state = self._sm.state
            qr_text = self._sm.last_policy.text if self._sm.last_policy else ""
            self._last_qr_state = qr_state
            debug["qr_state"] = str(qr_state)
            debug["qr_text"] = qr_text
            # POLICY_ACTIVE 输出非 NaN → 接管
            if qr_state == QRState.POLICY_ACTIVE and not (
                math.isnan(qr_steer) or math.isnan(qr_speed)
            ):
                steer, speed, source = qr_steer, qr_speed, SRC_QR
                self._stats["qr_policies"] += 1
            # 其它状态保持 path 输出

        # 2) 箭头：每帧都检测（便宜，~5ms），但只有"未过期 + 高置信度"才覆盖
        if self.arrow_enabled:
            try:
                res = detect_arrow(frame, min_darkness=self.arrow_min_darkness)
            except Exception as e:  # noqa: BLE001
                log.debug("arrow detect 异常: %s", e)
                res = None
            if res is not None:
                self._last_arrow = res
                self._last_arrow_at = now
                self._stats["arrow_detects"] += 1
                if (
                    res.confidence >= self.arrow_min_conf
                    and res.direction in self.arrow_steer_map
                ):
                    if source != SRC_QR:  # QR 已经接管就不再被箭头覆盖
                        steer = self.arrow_steer_map[res.direction]
                        source = SRC_ARROW
                        self._stats["arrow_overrides"] += 1
                arrow_res = res
        else:
            arrow_res = self._last_arrow if (now - self._last_arrow_at) < self.arrow_ttl_s else None

        # 3) QR 解码：按 decode_every_n 抽样（CPU 重）
        if self.qr_enabled and self._sm is not None and self._frame_idx % self.qr_decode_every_n == 0:
            try:
                decodes: list[DecodedQR] = decode_qr_codes(frame)
            except Exception as e:  # noqa: BLE001
                log.debug("qr decode 异常: %s", e)
                decodes = []
            if decodes:
                self._stats["qr_decodes"] += 1
                self._last_qr_decoded = decodes[0]
                self._last_qr_at = now
                self._sm.on_qr_decoded(decodes[0].text)
                debug["last_qr_text"] = decodes[0].text

        # 暴露给 pipeline 画 overlay 用
        if self._last_qr_decoded is not None and (now - self._last_qr_at) < 1.0:
            debug["last_qr_obj"] = self._last_qr_decoded

        return OverrideResult(
            steer_deg=steer, speed=speed, source=source,
            arrow=arrow_res, qr_state=qr_state, qr_text=qr_text, debug=debug,
        )

    # ---- 外部事件 ----
    def on_start(self) -> None:
        """PC 发了 START —— 启动 QR 扫描。"""
        if self._sm is not None:
            self._sm.start()

    def on_stop(self) -> None:
        """PC 发了 STOP —— QR 状态机回到 IDLE。"""
        if self._sm is not None:
            self._sm.reset()

    def on_ack(self) -> None:
        """PC ack 了 REPORTING。"""
        if self._sm is not None:
            self._sm.ack_report()
