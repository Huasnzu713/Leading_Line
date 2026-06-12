# -*- coding: utf-8 -*-
"""perception_pipeline.py — 算法流水线（颜色分割→路径规划→控制→override）。

无 ROS 依赖，可独立单测；ROS 入口在 perception_node.py 里。
"""
from __future__ import annotations

import logging
from typing import Optional, Tuple

import numpy as np

from leading_line.algo import color_segmenter, controller, path_planner
from leading_line.overrides import FrameOverrides
from leading_line.recognition.qr.state_machine import State as QRState

log = logging.getLogger(__name__)


class PerceptionPipeline:
    """单帧处理：(BGR frame) → (steer_deg, speed, debug_info)。"""

    def __init__(self, cfg: dict, overrides: Optional[FrameOverrides] = None) -> None:
        self.cfg = cfg
        self._overrides = overrides or FrameOverrides(cfg)
        self._smoother = path_planner.PathSmoother(
            alpha=float(cfg.get("temporal", {}).get("alpha", 0.4))
        )

    # ---- 模式切换（外部调用） ----
    def set_mode_cfg(self, effective_cfg: dict) -> None:
        self.cfg = effective_cfg

    # ---- 单帧处理 ----
    def process_frame(self, frame: np.ndarray, pipeline_running: bool = True) -> dict:
        cfg = self.cfg
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
        path = edges["center"]
        if cfg.get("temporal", {}).get("reset_on_no_road", True) and (
            path is None or (path.size and np.all(np.isnan(path[:, 0])))
        ):
            self._smoother.reset()
        path = self._smoother.update(path)
        edges = {**edges, "center": path}

        # override
        ov = self._overrides.tick(
            frame=frame,
            path_steer=steer,
            path_speed=speed,
            pipeline_running=pipeline_running,
        )
        return {
            "road_mask": road_mask,
            "floor_mask": floor_mask,
            "edges": edges,
            "steer_deg": ov.steer_deg,
            "speed": ov.speed,
            "source": ov.source,
            "lookahead": lookahead,
            "qr_state": str(ov.qr_state) if ov.qr_state is not None else None,
            "arrow": ov.arrow,
            "override": ov,
        }
