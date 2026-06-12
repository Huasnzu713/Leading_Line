# -*- coding: utf-8 -*-
"""policy 单元测试：覆盖 JSON / key=value 解析、错误路径、policy_to_text 往返。"""
import sys
from pathlib import Path

# 测试在 tests/unit/，算法模块在项目根的 vehicle/recognition/qr/
_PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

import pytest

from vehicle.recognition.qr.policy import (
    Policy, PolicyParseError, parse_policy, policy_to_text, KNOWN_POLICIES,
)


# ---------- JSON 格式 ----------
def test_parse_json_full():
    p = parse_policy('{"policy":"TURN_LEFT","steer_deg":-20,"speed":0.2,"duration_s":1.5}')
    assert p.name == "TURN_LEFT"
    assert p.steer_deg == -20.0
    assert p.speed == 0.2
    assert p.duration_s == 1.5
    assert p.params == {}


def test_parse_json_minimal():
    """只给 policy 字段，其他按缺省。"""
    p = parse_policy('{"policy":"STOP"}')
    assert p.name == "STOP"
    assert p.steer_deg == 0.0
    assert p.speed == 0.0
    assert p.duration_s == 0.0


def test_parse_json_with_params():
    p = parse_policy('{"policy":"CUSTOM","steer_deg":1,"speed":0.1,"params":{"k":"v"}}')
    assert p.name == "CUSTOM"
    assert p.params == {"k": "v"}


# ---------- key=value 格式 ----------
def test_parse_kv_basic():
    p = parse_policy("policy=TURN_RIGHT;steer_deg=15;speed=0.3;duration_s=2.0")
    assert p.name == "TURN_RIGHT"
    assert p.steer_deg == 15
    assert p.speed == 0.3
    assert p.duration_s == 2.0


def test_parse_kv_negative_number():
    p = parse_policy("policy=TURN_LEFT;steer_deg=-30;speed=0.1;duration_s=1")
    assert p.steer_deg == -30
    assert p.duration_s == 1.0  # 无小数点 → 走 int 路径，但 Policy 字段是 float


# ---------- 错误路径 ----------
def test_parse_empty_raises():
    with pytest.raises(PolicyParseError):
        parse_policy("")


def test_parse_no_policy_raises():
    with pytest.raises(PolicyParseError):
        parse_policy("steer_deg=10;speed=0.1")


def test_parse_unknown_policy_raises():
    with pytest.raises(PolicyParseError):
        parse_policy("policy=DO_A_BACKFLIP")


def test_parse_invalid_kv_raises():
    """分号里有片段没有 '='。"""
    with pytest.raises(PolicyParseError):
        parse_policy("policy=STOP;garbage")


def test_parse_json_array_raises():
    with pytest.raises(PolicyParseError):
        parse_policy("[]")


# ---------- 已知策略白名单 ----------
def test_known_policies_nonempty():
    assert "STOP" in KNOWN_POLICIES
    assert "CRUISE" in KNOWN_POLICIES
    assert "TURN_LEFT" in KNOWN_POLICIES


# ---------- policy_to_text 往返 ----------
def test_roundtrip_via_text():
    p = Policy(name="TURN_LEFT", steer_deg=-20, speed=0.2, duration_s=1.5)
    text = policy_to_text(p)
    p2 = parse_policy(text)
    assert p2.name == p.name
    assert p2.steer_deg == p.steer_deg
    assert p2.speed == p.speed
    assert p2.duration_s == p.duration_s


if __name__ == "__main__":
    # 不依赖 pytest：直接当脚本跑也能过
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
