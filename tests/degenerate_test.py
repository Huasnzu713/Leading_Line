"""退化场景回归测试：所有可能让 np.polyfit 炸的情形都跑一遍。"""
import sys
import numpy as np
import cv2

sys.path.insert(0, ".")
import path_planner


def case(name, road_mask, cfg, expect_nonempty=True, allow_nan=False):
    edges = path_planner.plan(road_mask, cfg)
    pts = edges["center"]
    ok = pts.size > 0
    if ok != expect_nonempty:
        raise AssertionError(f"[{name}] expected nonempty={expect_nonempty}, got size={pts.size}")
    if ok and not allow_nan:
        assert np.all(np.isfinite(pts)), f"[{name}] path contains NaN/Inf"
    print(f"  ok: {name} -> {pts.shape if ok else 'empty'}")


cfg = {
    "roi": {"top_ratio": 0.4, "bottom_ratio": 0.95, "left_ratio": 0.0, "right_ratio": 1.0},
    "path": {"num_samples": 20, "smooth_window": 7, "poly_degree": 3},
}

# 1) 完全空掩码 → 所有采样点都 NaN（controller 端会兜底成直行 + 最低速）
case("empty road mask", np.zeros((480, 640), dtype=np.uint8), cfg, allow_nan=True)

# 2) 道路只占 1 行（极窄）
m = np.zeros((480, 640), dtype=np.uint8)
m[200:201, 100:500] = 255
case("single-row road", m, cfg)

# 3) 道路宽度极小（1 像素）
m = np.zeros((480, 640), dtype=np.uint8)
m[200:300, 320:321] = 255
case("1-pixel-wide road", m, cfg)

# 4) ROI 高度为 0
m = np.zeros((480, 640), dtype=np.uint8)
m[200:300, 100:500] = 255
cfg_bad_roi = {**cfg, "roi": {"top_ratio": 0.5, "bottom_ratio": 0.5, "left_ratio": 0, "right_ratio": 1}}
case("zero-height ROI", m, cfg_bad_roi)

# 5) ROI 顶底颠倒 → 我的代码会把区间 clamp 成合法但道路不在新区间内 → 全 NaN
cfg_inverted = {**cfg, "roi": {"top_ratio": 0.9, "bottom_ratio": 0.4, "left_ratio": 0, "right_ratio": 1}}
m = np.zeros((480, 640), dtype=np.uint8)
m[200:300, 100:500] = 255
case("inverted ROI", m, cfg_inverted, allow_nan=True)

# 6) 大面积正常道路
m = np.zeros((480, 640), dtype=np.uint8)
m[200:460, 100:500] = 255
case("normal road", m, cfg)

# 7) 整张图都是道路
case("all-road image", np.full((480, 640), 255, dtype=np.uint8), cfg)

print("all degenerate cases handled without crashing")
