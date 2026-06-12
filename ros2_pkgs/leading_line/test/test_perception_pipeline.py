# -*- coding: utf-8 -*-
"""test_perception_pipeline.py — 单测 perception_pipeline.PerceptionPipeline。

无 ROS；纯 cv2 / numpy。给一帧空图 → (0.0, min_speed)。
"""
from __future__ import annotations

import os
import sys

_PKG = os.path.join(os.path.dirname(__file__), "..")
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

import numpy as np

from leading_line.perception_pipeline import PerceptionPipeline


def _minimal_cfg() -> dict:
    return {
        "colors": {
            "road": {"hsv_lower": [100, 50, 50], "hsv_upper": [130, 255, 255]},
            "floor": {"hsv_lower": [0, 8, 15], "hsv_upper": [80, 255, 255]},
        },
        "morphology": {"kernel_size": 5, "opening_iter": 1, "closing_iter": 2},
        "filter": {"min_road_area_px": 0},
        "roi": {"top_ratio": 0.4, "bottom_ratio": 0.95, "left_ratio": 0.0, "right_ratio": 1.0},
        "path": {"num_samples": 20, "smooth_window": 7, "poly_degree": 2},
        "controller": {
            "lookahead_row_from_bottom": 8,
            "max_steer_deg": 30.0,
            "base_speed": 0.30,
            "min_speed": 0.10,
            "curvature_k": 0.6,
        },
        "temporal": {"enabled": True, "alpha": 0.4, "reset_on_no_road": True},
        "overrides": {
            "arrow": {"enabled": False},
            "qr": {"enabled": False},
        },
    }


def test_black_frame_straight():
    cfg = _minimal_cfg()
    p = PerceptionPipeline(cfg)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    result = p.process_frame(frame, pipeline_running=True)
    # 全黑：没有道路像素，controller.decide 返回 (0, min_speed, None)
    assert abs(result["steer_deg"]) < 1e-3
    assert abs(result["speed"] - 0.10) < 1e-3
    assert result["source"] == "path"


def test_running_false_yields_path_output():
    cfg = _minimal_cfg()
    p = PerceptionPipeline(cfg)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    result = p.process_frame(frame, pipeline_running=False)
    # running=False 时 override 不生效，path 输出 (0, min_speed)
    assert abs(result["steer_deg"]) < 1e-3


def test_blue_road_in_lower_half():
    """下半部分填蓝色矩形 → 应该检测到道路，steer 在合理范围。"""
    cfg = _minimal_cfg()
    p = PerceptionPipeline(cfg)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    # HSV (120, 255, 255) → BGR ≈ (255, 0, 0)
    # 在中线画一条蓝色
    cv2 = __import__("cv2")
    cv2.rectangle(frame, (260, 240), (380, 400), (255, 0, 0), thickness=-1)
    result = p.process_frame(frame, pipeline_running=True)
    # 道路偏向右 → steer 应为正（小车右转）or 直行
    assert abs(result["steer_deg"]) < 30.0
    assert 0.0 <= result["speed"] <= 1.0


if __name__ == "__main__":
    test_black_frame_straight()
    test_running_false_yields_path_output()
    test_blue_road_in_lower_half()
    print("OK: 3 tests passed")
