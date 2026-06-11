"""直线道路稳定性测试：把"完美直的"道路以不同角度放置，验证
plan() 输出的 center 仍是直线（不被 polyfit 拗弯）。

中心是控制量的来源，必须严格。左右边缘的"角点换边"现象（条带端点
被切掉时，最左/最右像素在角点处换边，形成折线）是几何必然，跟
polyfit 无关 —— 真车场景下道路不会顶到画面边，不存在。
"""
import sys
import numpy as np
import cv2

sys.path.insert(0, ".")
from jetson.algo import path_planner

cfg = {
    "roi": {"top_ratio": 0.0, "bottom_ratio": 1.0, "left_ratio": 0.0, "right_ratio": 1.0},
    "path": {"num_samples": 20, "smooth_window": 7, "poly_degree": 2},
}


def make_tilted_road(angle_deg: float, H: int = 480, W: int = 640) -> np.ndarray:
    """生成一条穿过画面中心、倾角 angle_deg 的直条带。"""
    cx, cy = W / 2, H / 2
    half_w = 40.0
    a = np.deg2rad(angle_deg)
    nx, ny = -np.sin(a), np.cos(a)        # 法线
    dx, dy = np.cos(a), np.sin(a)         # 方向
    # 条带两端各留 60 px 距离画面边缘，避免"角点换边"
    half_len = min(H, W) / 2 - 60
    p1 = (cx + dx * half_len + nx * half_w, cy + dy * half_len + ny * half_w)
    p2 = (cx - dx * half_len + nx * half_w, cy - dy * half_len + ny * half_w)
    p3 = (cx - dx * half_len - nx * half_w, cy - dy * half_len - ny * half_w)
    p4 = (cx + dx * half_len - nx * half_w, cy + dy * half_len - ny * half_w)
    mask = np.zeros((H, W), dtype=np.uint8)
    cv2.fillPoly(mask, [np.array([p1, p2, p3, p4], dtype=np.int32)], 255)
    return mask


def line_residual_px(path_xy: np.ndarray) -> float:
    """把 path 拟合为一条直线，算最大残差（像素）。残差 ≈ 0 即"严格为直线"."""
    ys = path_xy[:, 1].astype(np.float64)
    xs = path_xy[:, 0].astype(np.float64)
    if not np.all(np.isfinite(xs)):
        return float("inf")
    c = np.polyfit(ys, xs, 1)
    return float(np.max(np.abs(xs - np.polyval(c, ys))))


print("angle(deg)  center resid px")
print("-" * 30)
for ang in [-60, -30, -10, 0, 10, 30, 60, 89]:
    mask = make_tilted_road(ang)
    edges = path_planner.plan(mask, cfg)
    rc = line_residual_px(edges["center"])
    print(f"  {ang:+5d}°     {rc:6.3f}")
    # 中心：必须严格
    assert rc < 0.5, f"angle {ang}°: center 出现非直线残差 {rc:.3f} px"

print("\nOK: 任意角度的直线道路，中心路径都严格是直线（polyfit 不再硬拗）")
