# -*- coding: utf-8 -*-
"""QR 解码：把 BGR 图像里的二维码还原成文本。

只用 OpenCV 自带的 cv2.QRCodeDetector（4.x 都带），不引入额外依赖；
它对打印/标准二维码的识别率足够用，对模糊/倾斜/小图会返回空列表。

公共 API：
    decode_qr_codes(frame_bgr)            -> list[DecodedQR]
    decode_qr_codes_first(frame_bgr)      -> DecodedQR | None

返回的 DecodedQR 含有：
    text:      解码文本
    bbox:      4 个角点 (4, 1, 2) int32，OpenCV findContours 风格
    straight:  校正后二维码图 (H, W, 3) uint8 BGR；无二维码时为 None
    confidence: 暂留 1.0（OpenCV 单 QR 路径不返回置信度）
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import cv2
import numpy as np


@dataclass
class DecodedQR:
    """一次解码结果。"""
    text: str
    bbox: np.ndarray            # (4, 1, 2) int32
    straight: Optional[np.ndarray]
    confidence: float = 1.0


def decode_qr_codes(frame_bgr: np.ndarray) -> List[DecodedQR]:
    """从单张 BGR 帧里识别所有二维码。

    OpenCV 的 multi 路径会先 detect，再对每张 try decode。
    失败时返回空 list，不抛异常。
    """
    if frame_bgr is None or frame_bgr.size == 0:
        return []
    if len(frame_bgr.shape) != 3 or frame_bgr.shape[2] != 3:
        return []

    detector = cv2.QRCodeDetector()
    try:
        ok, decoded, points, _ = detector.detectAndDecodeMulti(frame_bgr)
    except cv2.error:
        # detectAndDecodeMulti 在某些退化输入下会抛异常，吞掉当作"没扫到"
        return []

    results: List[DecodedQR] = []
    if not ok or not isinstance(decoded, (list, tuple, np.ndarray)):
        return results

    decoded_list = list(decoded)
    pts_list = points if points is not None else []
    for i, txt in enumerate(decoded_list):
        if not txt:
            continue
        bbox = pts_list[i] if i < len(pts_list) else np.empty((0, 1, 2), np.int32)
        results.append(DecodedQR(text=str(txt), bbox=bbox, straight=None))
    return results


def decode_qr_codes_first(frame_bgr: np.ndarray) -> Optional[DecodedQR]:
    """单二维码便捷调用：找到第一个就返回。"""
    items = decode_qr_codes(frame_bgr)
    return items[0] if items else None


def draw_qr_overlay(
    frame_bgr: np.ndarray,
    decoded: DecodedQR,
    color_bgr: tuple[int, int, int] = (0, 255, 0),
    thickness: int = 2,
) -> np.ndarray:
    """在 frame 上画二维码外框 + 文本标签，返回新图。"""
    out = frame_bgr.copy()
    if decoded.bbox is not None and decoded.bbox.size == 4 * 2:
        pts = decoded.bbox.reshape(-1, 2).astype(np.int32)
        cv2.polylines(out, [pts], isClosed=True, color=color_bgr, thickness=thickness)
        # 文本放第一个角点上方
        x, y = int(pts[0, 0]), int(pts[0, 1])
        label = decoded.text if len(decoded.text) <= 32 else decoded.text[:29] + "..."
        cv2.putText(
            out, label, (x, max(20, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA,
        )
        cv2.putText(
            out, label, (x, max(20, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 1, cv2.LINE_AA,
        )
    return out
