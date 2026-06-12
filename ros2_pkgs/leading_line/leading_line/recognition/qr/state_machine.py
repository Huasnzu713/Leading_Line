# -*- coding: utf-8 -*-
from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Optional

try:
    from .policy import KNOWN_POLICIES, Policy, PolicyParseError, parse_policy
except ImportError:  # 让脚本直跑 (python qr_system/state_machine.py) 也能用
    from policy import KNOWN_POLICIES, Policy, PolicyParseError, parse_policy


class State(str, Enum):
    IDLE = "IDLE"
    SCANNING = "SCANNING"
    DECODED = "DECODED"
    POLICY_ACTIVE = "POLICY_ACTIVE"
    REPORTING = "REPORTING"
    ERROR = "ERROR"


class EventKind(str, Enum):
    START = "START"
    TICK = "TICK"
    QR_FOUND = "QR_FOUND"
    QR_EMPTY = "QR_EMPTY"
    POLICY_DONE = "POLICY_DONE"
    POLICY_TIMEOUT = "POLICY_TIMEOUT"
    RESET = "RESET"
    ACK = "ACK"  # 报告已被下游消费


@dataclass
class Event:
    kind: EventKind
    payload: Any = None


@dataclass
class StateReport:
    """一次状态变化的对外报告。"""
    state: State
    last_policy: Optional[Policy]
    steer_deg: float
    speed: float
    message: str = ""
    timestamp: float = field(default_factory=time.time)


# ---------- 策略 → 控制量 ----------
# 纯查表，不做复杂计算；缺省值由 Policy 字段补。
def _apply_policy(p: Policy) -> tuple[float, float]:
    name = p.name
    if name == "STOP":
        return 0.0, 0.0
    if name in ("TURN_LEFT", "TURN_RIGHT"):
        return float(p.steer_deg), float(p.speed)
    if name in ("SLOW_DOWN", "SPEED_UP", "REVERSE", "CUSTOM", "HOLD"):
        return float(p.steer_deg), float(p.speed)
    if name == "CRUISE":
        # CRUISE：解绑，由引导线算法接管 steer/speed。
        # 状态机输出 NaN 表示"别用我"。
        return float("nan"), float("nan")
    # 兜底：未识别策略名 → 直行低速
    return 0.0, 0.05


# ---------- 状态机本体 ----------
class QRStateMachine:
    """显式转移表 + 副作用函数。"""

    def __init__(self, policy_timeout_s: float = 30.0) -> None:
        self.state: State = State.IDLE
        self.last_policy: Optional[Policy] = None
        self.last_error: Optional[str] = None
        self._policy_started_at: float = 0.0
        self._policy_timeout_s: float = float(policy_timeout_s)
        self._reports: list[StateReport] = []
        self._last_output: tuple[float, float] = (0.0, 0.0)
        self._on_state_change: Optional[Callable[[State, State], None]] = None

    # ---- 观察 / 订阅 ----
    def on_state_change(
        self, cb: Callable[[State, State], None]
    ) -> None:
        """注册状态变化回调（旧→新）。"""
        self._on_state_change = cb

    def consume_reports(self) -> list[StateReport]:
        """弹出已产生的报告（一次性消费）。"""
        out, self._reports = self._reports, []
        return out

    def last_output(self) -> tuple[float, float]:
        """上一 tick 输出的 (steer, speed)。"""
        return self._last_output

    # ---- 事件入口 ----
    def start(self) -> None:
        self._transition(State.SCANNING, "开始扫描")

    def reset(self) -> None:
        self.last_policy = None
        self.last_error = None
        self._last_output = (0.0, 0.0)
        self._transition(State.IDLE, "外部复位")

    def on_qr_decoded(self, text: str) -> None:
        """喂一条 QR 文本。解析失败会上 ERROR。"""
        if self.state not in (State.SCANNING, State.IDLE):
            # 正在执行策略时不打断（保护：避免策略被中途换掉）
            return
        try:
            policy = parse_policy(text)
        except PolicyParseError as e:
            self.last_error = str(e)
            self._transition(State.ERROR, f"解析失败：{e}")
            return

        self.last_policy = policy
        self._transition(State.DECODED, f"已解码：{policy.name}")

    def on_qr_empty(self) -> None:
        """主动上报"扫不到"——可选，主要给日志看。"""
        if self.state == State.SCANNING:
            self._emit_report("扫描中，未检测到二维码")

    def ack_report(self) -> None:
        if self.state == State.REPORTING:
            self._transition(State.IDLE, "报告已确认")

    # ---- 主循环 tick ----
    def tick(self, dt: float) -> tuple[float, float]:
        """推进状态机；返回当前应当下发的 (steer_deg, speed)。

        摄像头帧驱动场景：每帧调一次；dt 是上一帧到本帧的间隔秒。
        """
        if dt < 0:
            dt = 0.0

        if self.state == State.IDLE:
            out = (0.0, 0.0)
        elif self.state == State.SCANNING:
            out = (0.0, 0.0)  # 扫描时不给控制量，保持原状
        elif self.state == State.DECODED:
            # 立刻进入执行态（不卡在 DECODED 等人 ack）
            assert self.last_policy is not None
            self._policy_started_at = time.time()
            self._transition(State.POLICY_ACTIVE, f"执行 {self.last_policy.name}")
            steer, speed = _apply_policy(self.last_policy)
            out = (steer, speed)
        elif self.state == State.POLICY_ACTIVE:
            assert self.last_policy is not None
            steer, speed = _apply_policy(self.last_policy)
            # 0 duration → 一次性，只跑一帧
            if self.last_policy.duration_s <= 0:
                self._transition(State.REPORTING, "策略执行完毕")
            else:
                elapsed = time.time() - self._policy_started_at
                if elapsed >= self.last_policy.duration_s:
                    self._transition(State.REPORTING, "策略执行完毕")
                if elapsed >= self._policy_timeout_s:
                    # 兜底超时：避免永远卡在 ACTIVE
                    self._transition(State.ERROR, "策略超时")
                    steer, speed = 0.0, 0.0
            out = (steer, speed)
        elif self.state == State.REPORTING:
            out = (0.0, 0.0)
        elif self.state == State.ERROR:
            # 错误态：立刻停车，等待外部 reset
            out = (0.0, 0.0)
        else:
            out = (0.0, 0.0)

        self._last_output = out
        return out

    # ---- 内部 ----
    def _transition(self, next_state: State, message: str) -> None:
        prev = self.state
        if prev == next_state:
            return
        self.state = next_state
        self._emit_report(message, state=next_state)
        if self._on_state_change is not None:
            try:
                self._on_state_change(prev, next_state)
            except Exception:
                pass  # 回调异常不影响状态机本身

    def _emit_report(self, message: str, state: Optional[State] = None) -> None:
        self._reports.append(
            StateReport(
                state=state or self.state,
                last_policy=self.last_policy,
                steer_deg=self._last_output[0],
                speed=self._last_output[1],
                message=message,
            )
        )
