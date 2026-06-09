"""控制量计算：根据平滑后的路径点输出转向角与车速。

约定：
  - 路径点数组是自下而上排列的 (索引 0 = 最靠近小车的那一行)
  - 转向角：右转为正，左转为负
  - 车速：标量，越大越快；具体单位交给上层协议
"""
from __future__ import annotations

import numpy as np


def _curvature(path_xy: np.ndarray) -> float:
    """用首尾三点的二阶差分粗估曲率（绝对值）。值域 0~1 量级。"""
    if path_xy.shape[0] < 3:
        return 0.0
    x0, y0 = path_xy[0]
    x1, y1 = path_xy[len(path_xy) // 2]
    x2, y2 = path_xy[-1]
    # 路径约定"自下而上"（y 递减），用 abs 兼容两种方向
    dy1, dy2 = abs(y1 - y0), abs(y2 - y1)
    dx1, dx2 = (x1 - x0), (x2 - x1)
    # 一阶导
    d1 = dx1 / max(dy1, 1e-6)
    d2 = dx2 / max(dy2, 1e-6)
    # 二阶导（曲率近似）
    ddy = abs(y2 - y0) or 1e-6
    curv = (d2 - d1) / ddy
    return float(abs(curv))


def decide(
    smooth_path_xy: np.ndarray,
    image_width: int,
    cfg: dict,
) -> tuple[float, float, tuple[float, float] | None]:
    """根据路径计算 steer、speed 与"前瞻点"坐标（用于在画面上画小圆点）。

    返回：
        steer_deg:  转向角（度），右正左负
        speed:      车速
        lookahead:  前瞻点 (x, y)，若路径无效则为 None
    """
    lookahead_idx = int(cfg["lookahead_row_from_bottom"])
    max_steer = float(cfg["max_steer_deg"])
    base_speed = float(cfg["base_speed"])
    min_speed = float(cfg["min_speed"])
    curvature_k = float(cfg["curvature_k"])

    # 路径无效：直行 + 减速
    if (
        smooth_path_xy is None
        or smooth_path_xy.size == 0
        or np.all(np.isnan(smooth_path_xy[:, 0]))
    ):
        return 0.0, min_speed, None

    if lookahead_idx < 0:
        lookahead_idx = 0
    if lookahead_idx >= smooth_path_xy.shape[0]:
        lookahead_idx = smooth_path_xy.shape[0] - 1

    lx, ly = smooth_path_xy[lookahead_idx]
    if np.isnan(lx):
        return 0.0, min_speed, None

    # 偏差归一化到 [-1, 1]：图像中心为 0
    center_x = image_width / 2.0
    err = (lx - center_x) / (image_width / 2.0)
    err = float(np.clip(err, -1.0, 1.0))
    steer_deg = err * max_steer

    curv = _curvature(smooth_path_xy)
    speed = max(min_speed, base_speed * (1.0 - curvature_k * curv))
    return steer_deg, speed, (float(lx), float(ly))
