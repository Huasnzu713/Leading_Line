# -*- coding: utf-8 -*-
from __future__ import annotations

import numpy as np
import cv2


def make_masks(
    frame_bgr: np.ndarray,
    road_hsv_lower: list[int],
    road_hsv_upper: list[int],
    floor_hsv_lower: list[int],
    floor_hsv_upper: list[int],
) -> tuple[np.ndarray, np.ndarray]:
    """把 BGR 帧拆成 road_mask 与 floor_mask，用 HSV 颜色空间。

    为什么用 HSV 而不是 RGB 距离：
    RGB 欧氏距离里"浅蓝道路"和"浅黄/棕色谷物"距离可能很近（都在 90 附近），
    单纯调 tolerance 必然顾此失彼。HSV 把颜色拆成"色相 / 饱和度 / 亮度"：
    - 蓝色道路 H ≈ 106-110（OpenCV 0-179 范围）
    - 黄色/棕色谷物 H ≈ 10-40
    二者的 H 区间完全不重叠（中间 [50, 100] 是巨大安全带），
    再加 S 阈值滤掉灰白阴影、V 阈值容许光照变化，就能鲁棒分清。

    返回的掩码是 0/255 的 uint8 图。
    """
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    road_mask = cv2.inRange(
        hsv,
        np.array(road_hsv_lower, dtype=np.uint8),
        np.array(road_hsv_upper, dtype=np.uint8),
    )
    floor_mask = cv2.inRange(
        hsv,
        np.array(floor_hsv_lower, dtype=np.uint8),
        np.array(floor_hsv_upper, dtype=np.uint8),
    )
    return road_mask, floor_mask


def clean_mask(
    mask: np.ndarray,
    kernel_size: int,
    opening_iter: int,
    closing_iter: int,
) -> np.ndarray:
    """形态学开运算去噪 → 闭运算补洞。椭圆核更贴合路面形状。"""
    k = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
    )
    cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=opening_iter)
    cleaned = cv2.morphologyEx(
        cleaned, cv2.MORPH_CLOSE, k, iterations=closing_iter
    )
    return cleaned


def keep_largest_component(mask: np.ndarray, min_area: int = 0) -> np.ndarray:
    """只保留面积最大的连通分量；min_area 是硬门槛。

    用途：颜色分割后，画面里可能同时有"真道路"和"屏边反光/人脸阴影/墙面"等
    零碎的小斑，只用最大块就能把它们全部扔掉，避免路径被异常像素拽歪。
    """
    if mask is None or mask.size == 0:
        return mask
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
        mask, connectivity=8
    )
    if num_labels <= 1:
        return np.zeros_like(mask)
    areas = stats[1:, cv2.CC_STAT_AREA]            # 跳过背景 (label 0)
    largest_local = int(np.argmax(areas))
    if areas[largest_local] < min_area:
        return np.zeros_like(mask)
    out = np.zeros_like(mask)
    out[labels == largest_local + 1] = 255
    return out
