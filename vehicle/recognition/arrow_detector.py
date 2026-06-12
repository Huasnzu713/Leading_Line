# -*- coding: utf-8 -*-
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


# 方向常量 (中文给程序使用, 英文用于在图片上叠加显示, 因为 cv2.putText 不支持中文)
DIRECTION_FORWARD = "前"
DIRECTION_LEFT = "左"
DIRECTION_RIGHT = "右"
DIRECTION_UNKNOWN = "未知"

_EN_LABEL = {
    DIRECTION_FORWARD: "FORWARD",
    DIRECTION_LEFT: "LEFT",
    DIRECTION_RIGHT: "RIGHT",
    DIRECTION_UNKNOWN: "UNKNOWN",
}


@dataclass
class ArrowResult:
    """箭头识别结果."""
    direction: str                   # "前" / "左" / "右" / "未知"
    angle_deg: float                 # 质心 -> 尖端 的角度, 0=右, 90=上, 180/-180=左, -90=下
    tip: Tuple[int, int]             # 尖端像素坐标 (x, y)
    centroid: Tuple[int, int]        # 质心像素坐标 (x, y)
    contour: np.ndarray              # 原始最大轮廓
    confidence: float                # 0~1, 综合"尖端锐利度"与"角度落在某方向中心程度"

    @property
    def en_label(self) -> str:
        return _EN_LABEL.get(self.direction, "UNKNOWN")


# ---------- 内部辅助 ----------

def _preprocess(img: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """返回 (箭头为白 / 背景为黑的二值图, 灰度图)."""
    if img.ndim == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    # Otsu 后若白色像素超过一半, 说明背景被算成了白色, 需要反转让箭头变白
    if cv2.countNonZero(binary) > binary.size / 2:
        binary = cv2.bitwise_not(binary)
    # 闭运算填掉箭头内部的小空洞 / 毛刺
    kernel = np.ones((3, 3), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
    return binary, gray


def _largest_contour(binary: np.ndarray, min_area_frac: float = 0.002) -> Optional[np.ndarray]:
    """返回二值图中面积最大且大于 min_area_frac * 图像面积 的外轮廓."""
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    img_area = binary.shape[0] * binary.shape[1]
    contours = [c for c in contours if cv2.contourArea(c) >= img_area * min_area_frac]
    if not contours:
        return None
    return max(contours, key=cv2.contourArea)


def _hull_polygon(contour: np.ndarray) -> np.ndarray:
    """凸包 + 多边形近似, 返回 (N, 2) 的顶点数组.

    用凸包可以自动剔除箭头颈部的凹陷缺口顶点 (那两个会被误判为尖端).
    """
    hull = cv2.convexHull(contour)
    peri = cv2.arcLength(hull, True)
    # 0.02 倍周长一般能把弧形尖端简化到一个顶点
    approx = cv2.approxPolyDP(hull, max(peri * 0.02, 2.0), True)
    if len(approx) < 3:
        # 退化情况, 用原始凸包
        approx = hull
    return approx.reshape(-1, 2)


def _find_tip(pts: np.ndarray, centroid: np.ndarray) -> Tuple[int, float]:
    """在凸包顶点中找"最尖"的, 返回 (顶点索引, 内角余弦).

    定义"尖锐度" = 顶点处两条相邻边夹角的余弦, 越接近 1 越尖.
    用 (尖锐度 + 1) * 到质心距离 作为综合得分, 防止把次尖的近距离点选成尖端.
    """
    n = len(pts)
    best_idx = 0
    best_score = -float("inf")
    best_cos = -1.0
    for i in range(n):
        prev_pt = pts[(i - 1) % n]
        next_pt = pts[(i + 1) % n]
        v1 = prev_pt - pts[i]
        v2 = next_pt - pts[i]
        nrm = np.linalg.norm(v1) * np.linalg.norm(v2)
        if nrm < 1e-6:
            continue
        cos_a = float(np.dot(v1, v2) / nrm)
        dist = float(np.linalg.norm(pts[i] - centroid))
        score = (cos_a + 1.0) * dist  # 两个因子都恒非负, 兼顾"尖"和"远"
        if score > best_score:
            best_score = score
            best_idx = i
            best_cos = cos_a
    return best_idx, best_cos


def _classify_angle(angle_deg: float) -> Tuple[str, float]:
    """把方向角映射到 前/左/右/未知, 同时给出 0~1 的置信度.

    坐标系: 0° = 右, 90° = 上, ±180° = 左, -90° = 下 (与 atan2(-dy, dx) 一致).
    用 ±45° 的扇区划分; 落到"下"方向 (-135°~-45°) 时返回"未知".
    """
    a = angle_deg
    if 45.0 <= a <= 135.0:
        return DIRECTION_FORWARD, 1.0 - abs(a - 90.0) / 45.0
    if -45.0 <= a <= 45.0:
        return DIRECTION_RIGHT, 1.0 - abs(a) / 45.0
    if a >= 135.0 or a <= -135.0:
        d_to_180 = min(abs(a - 180.0), abs(a + 180.0))
        return DIRECTION_LEFT, 1.0 - d_to_180 / 45.0
    return DIRECTION_UNKNOWN, 0.0


# ---------- 对外接口 ----------

def detect_arrow(img: np.ndarray, min_darkness: float = 80.0) -> Optional[ArrowResult]:
    """识别图中最大的黑色箭头, 返回方向; 找不到合适轮廓或不够黑时返回 None.

    参数
    ----
    img : BGR / 灰度图
    min_darkness : 0~255, 轮廓内平均灰度必须 <= 该值才算"黑色箭头".
        默认 80, 即"中等灰度以上"会被过滤掉; 想严格只接受纯黑可调到 ~50.
    """
    binary, gray = _preprocess(img)
    contour = _largest_contour(binary)
    if contour is None:
        return None

    # 新增: 校验箭头区域必须够黑. 用轮廓填出的掩码求灰度均值, 高于阈值则丢弃.
    #   - 这样红色/绿色等其他颜色箭头(灰度偏高)会被直接拒绝
    #   - 深色阴影/墙面/头发等"形状像箭头但颜色不够黑"的误报也会被压住
    mask = np.zeros(gray.shape, dtype=np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, thickness=-1)
    mean_val = float(cv2.mean(gray, mask=mask)[0])
    if mean_val > min_darkness:
        return None

    M = cv2.moments(contour)
    if M["m00"] == 0:
        return None
    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]
    centroid = np.array([cx, cy])

    pts = _hull_polygon(contour)
    tip_idx, cos_tip = _find_tip(pts, centroid)
    tip = pts[tip_idx]

    dx = float(tip[0]) - cx
    dy = float(tip[1]) - cy
    # 图像坐标 y 向下, 翻转 dy 后 atan2 得到的角度: 0=右, 90=上, ±180=左
    angle_deg = float(np.degrees(np.arctan2(-dy, dx)))

    direction, dir_conf = _classify_angle(angle_deg)
    sharp_conf = max(0.0, (cos_tip + 1.0) / 2.0)
    confidence = float(max(0.0, min(1.0, 0.5 * dir_conf + 0.5 * sharp_conf)))

    return ArrowResult(
        direction=direction,
        angle_deg=angle_deg,
        tip=(int(tip[0]), int(tip[1])),
        centroid=(int(cx), int(cy)),
        contour=contour,
        confidence=confidence,
    )


def annotate(img: np.ndarray, result: ArrowResult) -> np.ndarray:
    """在图上画出轮廓 / 质心 / 尖端 / 方向标签, 返回新图 (BGR)."""
    out = img.copy()
    if out.ndim == 2:
        out = cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)
    cv2.drawContours(out, [result.contour], -1, (0, 255, 0), 2)
    cv2.arrowedLine(out, result.centroid, result.tip, (0, 255, 255), 3, tipLength=0.2)
    cv2.circle(out, result.tip, 8, (0, 0, 255), -1)
    cv2.circle(out, result.centroid, 6, (255, 0, 0), -1)
    label = f"{result.en_label} {int(round(result.angle_deg)):+4d} c={result.confidence:.2f}"
    cv2.putText(out, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(out, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 255, 0), 2, cv2.LINE_AA)
    return out
