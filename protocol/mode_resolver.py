# -*- coding: utf-8 -*-
from __future__ import annotations

import copy
from typing import Any

from .constants import ALL_MODES, MODE_BLUE


def list_modes(cfg: dict) -> list[dict]:
    """返回所有可用模式，给 UI 下拉框用。

    每项形如 ``{"name": "blue_path", "label": "蓝色路径模式"}``。
    """
    modes = cfg.get("modes") or {}
    out: list[dict] = []
    for name in ALL_MODES:
        if name in modes:
            out.append({"name": name, "label": modes[name].get("label", name)})
    return out


def select_mode(cfg: dict, mode_name: str) -> tuple[dict, dict]:
    """把 cfg 复制一份，把 mode_name 对应的 colors/visualization 合并到顶层。

    返回 ``(effective_cfg, mode_meta)``：
    - effective_cfg：可直接喂给现有 color_segmenter / path_planner / visualizer
    - mode_meta：包含 ``name`` / ``label`` / ``fallback`` 等元数据

    找不到模式时回退到默认（blue_path），不抛异常，避免 UI 一开始未选模式就崩。
    """
    if not isinstance(cfg, dict):
        raise TypeError(f"cfg 必须是 dict，得到 {type(cfg).__name__}")

    out = copy.deepcopy(cfg)
    modes = out.get("modes") or {}

    requested = mode_name
    fallback = False
    if mode_name not in modes:
        fallback = True
        if MODE_BLUE in modes:
            mode_name = MODE_BLUE
        elif modes:
            mode_name = next(iter(modes))
        else:
            # 连 modes 段都没有，保持原 cfg 不动
            return out, {"name": None, "label": "(no mode)", "fallback": True, "requested": requested}

    block = modes[mode_name]
    meta: dict[str, Any] = {
        "name": mode_name,
        "label": block.get("label", mode_name),
        "fallback": fallback,
        "requested": requested,
    }

    if "colors" in block:
        out["colors"] = copy.deepcopy(block["colors"])
    if "visualization" in block:
        out["visualization"] = copy.deepcopy(block["visualization"])

    return out, meta
