# -*- coding: utf-8 -*-
"""Qt 控件：把 numpy BGR 帧转成 QPixmap 喂给 QLabel。"""
from __future__ import annotations

import numpy as np
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap, QPainter, QColor, QFont


def bgr_to_qimage(frame_bgr: np.ndarray) -> QImage:
    """BGR (H, W, 3) uint8 → QImage（RGB888）。"""
    h, w = frame_bgr.shape[:2]
    # OpenCV 是 BGR，Qt 是 RGB；逐通道换位最简单
    rgb = frame_bgr[:, :, ::-1].copy()  # 反转最后一维
    bytes_per_line = 3 * w
    qimg = QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
    return qimg


def frame_to_pixmap_scaled(frame_bgr: np.ndarray, target_w: int, target_h: int) -> QPixmap:
    """BGR → QPixmap，按目标尺寸做 KeepAspectRatio 缩放。"""
    qimg = bgr_to_qimage(frame_bgr)
    return QPixmap.fromImage(qimg).scaled(
        target_w, target_h,
        Qt.KeepAspectRatio,
        Qt.SmoothTransformation,
    )


def make_placeholder_pixmap(w: int, h: int, text: str = "等待 Jetson 画面...") -> QPixmap:
    """UI 启动但还没收到视频帧时显示的占位图。"""
    pm = QPixmap(w, h)
    pm.fill(Qt.darkGray)
    p = QPainter(pm)
    p.setPen(QColor(220, 220, 220))
    p.setFont(QFont("Microsoft YaHei", 14))
    p.drawText(pm.rect(), Qt.AlignCenter, text)
    p.end()
    return pm
