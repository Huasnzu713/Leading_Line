# -*- coding: utf-8 -*-
"""test_ackermann_bridge.py — 单测 ackermann_bridge.MockBridge。"""
from __future__ import annotations

import os
import sys

# 把本包加进 sys.path
_PKG = os.path.join(os.path.dirname(__file__), "..")
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

from leading_line.ackermann_bridge import MockBridge


def test_mock_zero():
    b = MockBridge()
    b.publish(0.0, 0.0)
    assert b.last == (0.0, 0.0)
    assert b.publish_count == 1


def test_mock_positive():
    b = MockBridge(max_speed_mps=0.5, max_steer_deg=30.0)
    b.publish(20.0, 0.5)
    # speed 在 0..1（这里直接 publish(0.5) → 原样存，不做乘法；乘法在 rclpy 端做）
    assert b.last == (20.0, 0.5)


def test_mock_clamp_speed():
    b = MockBridge()
    b.publish(0.0, 2.0)  # 超过 1
    assert b.last[1] == 1.0
    b.publish(0.0, -0.5)  # 负数
    assert b.last[1] == 0.0


def test_mock_clamp_steer():
    b = MockBridge(max_steer_deg=30.0)
    b.publish(60.0, 0.5)
    assert b.last[0] == 30.0
    b.publish(-60.0, 0.5)
    assert b.last[0] == -30.0


def test_mock_stop():
    b = MockBridge()
    b.publish(15.0, 0.7)
    assert b.last == (15.0, 0.7)
    b.stop()
    assert b.last == (0.0, 0.0)


if __name__ == "__main__":
    test_mock_zero()
    test_mock_positive()
    test_mock_clamp_speed()
    test_mock_clamp_steer()
    test_mock_stop()
    print("OK: 5 tests passed")
