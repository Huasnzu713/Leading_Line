# -*- coding: utf-8 -*-
from __future__ import annotations

import numpy as np
import cv2


def _draw_road_overlay(
    frame: np.ndarray, road_mask: np.ndarray, color_bgr, alpha: float
) -> np.ndarray:
    overlay = frame.copy()
    overlay[road_mask > 0] = color_bgr
    return cv2.addWeighted(overlay, alpha, frame, 1.0 - alpha, 0)


def _draw_floor_overlay(
    frame: np.ndarray, floor_mask: np.ndarray, color_bgr, alpha: float
) -> np.ndarray:
    """谷地（地面）半透明叠加。alpha=0 时不叠加，直接返回原图。"""
    if alpha <= 0:
        return frame
    overlay = frame.copy()
    overlay[floor_mask > 0] = color_bgr
    return cv2.addWeighted(overlay, alpha, frame, 1.0 - alpha, 0)


def _draw_polyline(
    frame: np.ndarray, points_xy: np.ndarray, color_bgr, thickness: int
) -> np.ndarray:
    if points_xy is None or points_xy.size == 0:
        return frame
    finite = np.all(np.isfinite(points_xy), axis=1)
    pts = points_xy[finite]
    if pts.size < 2:
        return frame
    pts = pts.reshape(-1, 1, 2).astype(np.int32)
    cv2.polylines(frame, [pts], isClosed=False, color=color_bgr, thickness=thickness)
    return frame


def _draw_lookahead(
    frame: np.ndarray, lookahead: tuple[float, float] | None
) -> np.ndarray:
    if lookahead is None:
        return frame
    x, y = int(lookahead[0]), int(lookahead[1])
    cv2.circle(frame, (x, y), 6, (0, 255, 255), -1)  # 黄色实心点
    cv2.circle(frame, (x, y), 8, (0, 0, 0), 1)       # 黑色描边
    return frame


def draw_road_contour(
    frame: np.ndarray, road_mask: np.ndarray, color_bgr, thickness: int
) -> np.ndarray:
    """提取并绘制 road_mask 的外轮廓（道路 vs 谷物的真实分界）。

    替代 path_planner 算的 left/right 折线 —— 折线只覆盖 ROI 内"按行最外侧
    像素"，不闭合、不连续；外轮廓是整个道路区域与谷物的真实边界，闭合完整。

    关键细节：road_mask 若贴到图片边缘，findContours 会把"图像边缘"也当成
    道路的边界（轮廓沿 x=0 / y=0 等延伸），但这段不是"道路 vs 谷物"——
    是道路延伸到画面外。画完轮廓后要把边缘带恢复成原图。
    """
    if road_mask is None or road_mask.size == 0:
        return frame
    h, w = road_mask.shape
    if h < 3 or w < 3:
        return frame

    # 备份原图，画完再把边缘带恢复回去
    original = frame.copy()

    # 加 1 像素 0 padding，findContours 在 padded 里跑（用 RETR_EXTERNAL 拿最外层）
    padded = cv2.copyMakeBorder(
        road_mask, 1, 1, 1, 1, cv2.BORDER_CONSTANT, value=0
    )
    contours, _ = cv2.findContours(
        padded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        return frame
    # 轮廓点从 padded 坐标平移回原图坐标
    shifted = [c - np.array([[1, 1]], dtype=c.dtype) for c in contours]
    cv2.drawContours(frame, shifted, -1, tuple(color_bgr), int(thickness))

    # 擦掉图像边缘带：road_mask 贴边时那部分轮廓是"画面外"不是"道路 vs 谷物"
    band = int(thickness) + 1
    frame[:band, :] = original[:band, :]
    frame[-band:, :] = original[-band:, :]
    frame[:, :band] = original[:, :band]
    frame[:, -band:] = original[:, -band:]
    return frame


def _draw_hud(
    frame: np.ndarray, steer_deg: float, speed: float
) -> np.ndarray:
    text1 = f"Steer: {steer_deg:+6.2f} deg"
    text2 = f"Speed: {speed:5.2f}"
    cv2.putText(frame, text1, (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(frame, text1, (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(frame, text2, (12, 56),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(frame, text2, (12, 56),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
    return frame


def draw(
    frame_bgr: np.ndarray,
    road_mask: np.ndarray,
    edges: dict,
    steer_deg: float,
    speed: float,
    lookahead: tuple[float, float] | None,
    viz_cfg: dict,
    floor_mask: np.ndarray | None = None,
) -> np.ndarray:
    """画：道路区域外轮廓 → 中心引导线 → 前瞻点 → HUD。

    - 道路外轮廓：road_mask 的 findContours，画"道路 vs 谷物"完整分界
    - 中心引导线：path_planner 算的 center 折线，画"行驶路径"（在道路内部）
    - 不再画 path_planner 算的 left/right 折线 —— 它和外轮廓功能重复
    - floor_mask：可选，传了就按 viz_cfg["floor_overlay_*"] 叠加谷物
    """
    out = frame_bgr.copy()
    out = _draw_road_overlay(
        out,
        road_mask,
        tuple(viz_cfg["road_overlay_bgr"]),
        float(viz_cfg["road_overlay_alpha"]),
    )
    # 谷地叠加：可选；alpha=0 或没设 floor_mask 时跳过
    if floor_mask is not None and "floor_overlay_bgr" in viz_cfg:
        out = _draw_floor_overlay(
            out,
            floor_mask,
            tuple(viz_cfg["floor_overlay_bgr"]),
            float(viz_cfg.get("floor_overlay_alpha", 0.0)),
        )
    out = draw_road_contour(
        out,
        road_mask,
        tuple(viz_cfg["edge_color_bgr"]),
        int(viz_cfg["edge_thickness"]),
    )
    out = _draw_polyline(
        out,
        edges.get("center"),
        tuple(viz_cfg["path_color_bgr"]),
        int(viz_cfg["path_thickness"]),
    )
    out = _draw_lookahead(out, lookahead)
    if viz_cfg.get("show_hud", True):
        out = _draw_hud(out, steer_deg, speed)
    return out


def draw_debug(
    frame_bgr: np.ndarray,
    road_mask: np.ndarray,
    floor_mask: np.ndarray,
    edges: dict,
    steer_deg: float,
    speed: float,
    lookahead: tuple[float, float] | None,
    viz_cfg: dict,
) -> np.ndarray:
    """调试模式：2×2 网格同时显示 原图 / 道路掩码 / 地面掩码 / 最终可视化。

    用法：main.py 中按 'd' 键或加 --debug 启动。
    道路掩码用半透明红色叠在原图上，地面掩码用半透明橙色；
    配 draw() 走正常可视化，方便对照"算法到底把什么当成道路"。
    """
    h, w = frame_bgr.shape[:2]

    # 道路掩码：原图 + 半透明红色
    road_vis = frame_bgr.copy()
    road_vis[road_mask > 0] = [0, 0, 255]
    road_vis = cv2.addWeighted(road_vis, 0.4, frame_bgr, 0.6, 0)

    # 地面掩码：原图 + 半透明橙色
    floor_vis = frame_bgr.copy()
    floor_vis[floor_mask > 0] = [0, 165, 255]
    floor_vis = cv2.addWeighted(floor_vis, 0.4, frame_bgr, 0.6, 0)

    # 最终可视化（走 draw()，与正常模式完全一致）
    final = draw(frame_bgr, road_mask, edges, steer_deg, speed, lookahead, viz_cfg, floor_mask)

    # 2×2 网格：上排 原图/道路掩码；下排 地面掩码/最终结果
    top = np.hstack([frame_bgr, road_vis])
    bottom = np.hstack([floor_vis, final])
    combined = np.vstack([top, bottom])

    # 四个角的标注（黑描边 + 白字）
    for (label, x), y in [
        (("Original", 0), 30),
        (("Road Mask", w), 30),
        (("Floor Mask", 0), h + 30),
        (("Result", w), h + 30),
    ]:
        cv2.putText(
            combined, label, (x + 10, y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA,
        )
        cv2.putText(
            combined, label, (x + 10, y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA,
        )
    return combined
