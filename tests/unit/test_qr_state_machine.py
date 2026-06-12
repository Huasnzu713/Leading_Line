"""state_machine 单元测试：用合成事件验证状态转移与输出。"""
import sys
import time
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

import pytest

from vehicle.recognition.qr.policy import Policy
from vehicle.recognition.qr.state_machine import QRStateMachine, State


def _fresh(timeout=30.0):
    return QRStateMachine(policy_timeout_s=timeout)


# ---------- 启动 / 复位 ----------
def test_start_goes_to_scanning():
    sm = _fresh()
    assert sm.state == State.IDLE
    sm.start()
    assert sm.state == State.SCANNING
    steer, speed = sm.tick(0.1)
    assert (steer, speed) == (0.0, 0.0)


def test_reset_clears_policy_and_state():
    sm = _fresh()
    sm.start()
    sm.on_qr_decoded('{"policy":"STOP"}')
    sm.tick(0.1)
    assert sm.last_policy is not None
    sm.reset()
    assert sm.state == State.IDLE
    assert sm.last_policy is None


# ---------- 合法 QR → DECODED → POLICY_ACTIVE ----------
def test_valid_qr_runs_one_tick_then_active():
    sm = _fresh()
    sm.start()
    sm.on_qr_decoded('{"policy":"TURN_LEFT","steer_deg":-15,"speed":0.2,"duration_s":1.0}')
    # 第一次 tick：DECODED → POLICY_ACTIVE
    steer, speed = sm.tick(0.1)
    assert sm.state == State.POLICY_ACTIVE
    assert steer == -15.0
    assert speed == 0.2


def test_stop_policy_emits_zero_steer_and_speed():
    sm = _fresh()
    sm.start()
    sm.on_qr_decoded("policy=STOP")
    steer, speed = sm.tick(0.1)
    assert (steer, speed) == (0.0, 0.0)


def test_cruise_emits_nan_to_release_control():
    sm = _fresh()
    sm.start()
    sm.on_qr_decoded("policy=CRUISE")
    steer, speed = sm.tick(0.1)
    assert steer != steer  # NaN check
    assert speed != speed


# ---------- 一次性策略 (duration=0) ----------
def test_zero_duration_runs_one_frame_then_reporting():
    sm = _fresh()
    sm.start()
    sm.on_qr_decoded("policy=STOP")
    sm.tick(0.1)            # DECODED → POLICY_ACTIVE
    steer, speed = sm.tick(0.1)  # POLICY_ACTIVE → REPORTING
    assert sm.state == State.REPORTING
    assert (steer, speed) == (0.0, 0.0)
    sm.ack_report()
    assert sm.state == State.IDLE


# ---------- 解析失败 → ERROR ----------
def test_invalid_qr_goes_to_error():
    sm = _fresh()
    sm.start()
    sm.on_qr_decoded("garbage_no_equals_or_braces")
    assert sm.state == State.ERROR
    assert sm.last_error is not None
    steer, speed = sm.tick(0.1)
    assert (steer, speed) == (0.0, 0.0)  # 错误态 = 停车
    sm.reset()
    assert sm.state == State.IDLE


def test_unknown_policy_goes_to_error():
    sm = _fresh()
    sm.start()
    sm.on_qr_decoded("policy=NUKE_THE_KITCHEN")
    assert sm.state == State.ERROR


# ---------- 活动期不被打断 ----------
def test_active_policy_not_interrupted_by_new_qr():
    sm = _fresh()
    sm.start()
    sm.on_qr_decoded('{"policy":"TURN_LEFT","steer_deg":-10,"speed":0.1,"duration_s":2.0}')
    sm.tick(0.1)  # 进入 ACTIVE
    assert sm.state == State.POLICY_ACTIVE
    sm.on_qr_decoded('{"policy":"TURN_RIGHT","steer_deg":30,"speed":0.5,"duration_s":2.0}')
    sm.tick(0.1)
    # 应该保持 -10 / 0.1（左侧策略），不切换
    assert sm.last_policy.name == "TURN_LEFT"
    assert sm.last_output() == (-10.0, 0.1)


# ---------- 超时 ----------
def test_policy_timeout_goes_to_error(monkeypatch=None):
    sm = _fresh(timeout=0.5)  # 0.5s 兜底超时
    sm.start()
    sm.on_qr_decoded('{"policy":"TURN_LEFT","steer_deg":-10,"speed":0.1,"duration_s":999}')

    # 第一帧：进入 ACTIVE
    sm.tick(0.1)
    assert sm.state == State.POLICY_ACTIVE

    # 等 1 秒（> timeout），再 tick → 触发 POLICY_TIMEOUT → ERROR
    time.sleep(0.6)
    sm.tick(0.1)
    assert sm.state == State.ERROR


# ---------- 报告消费 ----------
def test_reports_drained_after_consume():
    sm = _fresh()
    sm.start()
    sm.on_qr_decoded("policy=STOP")
    sm.tick(0.1)
    reports = sm.consume_reports()
    assert len(reports) > 0
    # 再次消费应该空
    assert sm.consume_reports() == []


# ---------- 直接运行 ----------
if __name__ == "__main__":
    funcs = [v for k, v in globals().items() if k.startswith("test_") and callable(v)]
    failed = 0
    for fn in funcs:
        try:
            fn()
            print(f"  ok: {fn.__name__}")
        except Exception as e:
            print(f"  FAIL: {fn.__name__}: {e}")
            failed += 1
    print(f"--- {len(funcs) - failed}/{len(funcs)} 通过 ---")
    sys.exit(0 if failed == 0 else 1)
