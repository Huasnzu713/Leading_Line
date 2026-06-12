# -*- coding: utf-8 -*-
from __future__ import annotations

import sys
import time
from pathlib import Path

# 测试在 tests/unit/；算法实现已迁入 ros2_pkgs/leading_line/，从那里 import
_PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

_LEADING_LINE_PARENT = _PROJECT_ROOT / "ros2_pkgs" / "leading_line"
if str(_LEADING_LINE_PARENT) not in sys.path:
    sys.path.insert(0, str(_LEADING_LINE_PARENT))

import numpy as np
import cv2
import yaml

from leading_line.algo import color_segmenter, path_planner
from leading_line.algo.path_planner import PathSmoother
from protocol import select_mode


# ---------- 公共 fixture ----------

def _load_effective_cfg(mode: str = "blue_path") -> dict:
    with open(_PROJECT_ROOT / "config.yaml", "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    cfg, _ = select_mode(raw, mode)
    return cfg


def _path_cfg() -> dict:
    return {
        "roi": {"top_ratio": 0.0, "bottom_ratio": 1.0, "left_ratio": 0.0, "right_ratio": 1.0},
        "path": {"num_samples": 20, "smooth_window": 7, "poly_degree": 2},
    }


def _path_cfg_with_roi() -> dict:
    return {
        "roi": {"top_ratio": 0.4, "bottom_ratio": 0.95, "left_ratio": 0.0, "right_ratio": 1.0},
        "path": {"num_samples": 20, "smooth_window": 7, "poly_degree": 3},
    }


def _process(img: np.ndarray, cfg: dict):
    """在测试里复刻 debug.algo_preview 的 process_frame，避开 cv2 弹窗。"""
    road_mask, floor_mask = color_segmenter.make_masks(
        img,
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
    from leading_line.algo import controller
    steer, speed, lookahead = controller.decide(
        edges["center"], img.shape[1], cfg["controller"]
    )
    return road_mask, floor_mask, edges, steer, speed, lookahead


# =========================================================================
# 1. edges: 验证 sample_edges 自洽
# =========================================================================

def test_edges_rect_road():
    """矩形道路：left/right/center 应自洽。"""
    cfg = _path_cfg()
    mask = np.zeros((480, 640), dtype=np.uint8)
    mask[100:400, 200:400] = 255
    edges = path_planner.sample_edges(mask, cfg["roi"], cfg["path"]["num_samples"])
    left, right, center = edges["left"], edges["right"], edges["center"]
    assert np.allclose(left[:, 0], 200, atol=1), f"left should be ~200, got {left[:,0]}"
    assert np.allclose(right[:, 0], 400, atol=1), f"right should be ~400, got {right[:,0]}"
    assert np.allclose(center[:, 0], 300, atol=1), f"center should be ~300, got {center[:,0]}"
    assert np.allclose(center[:, 0], (left[:, 0] + right[:, 0]) / 2, atol=1e-3)
    assert np.all(left[:, 0] <= center[:, 0] + 1e-3)
    assert np.all(center[:, 0] <= right[:, 0] + 1e-3)


def test_edges_trapezoid_road():
    """倾斜梯形道路：自下而上索引 0 是底部（宽），-1 是顶部（窄）。"""
    cfg = _path_cfg()
    mask = np.zeros((480, 640), dtype=np.uint8)
    top_y, bot_y = 100, 460
    pts = np.array([(280, top_y), (360, top_y), (540, bot_y), (100, bot_y)], dtype=np.int32)
    cv2.fillPoly(mask, [pts], 255)
    edges = path_planner.sample_edges(mask, cfg["roi"], cfg["path"]["num_samples"])
    left, right, _ = edges["left"], edges["right"], edges["center"]
    assert abs(left[0, 0] - 100) < 8 and abs(right[0, 0] - 540) < 8, "trap bottom edges wrong"
    assert abs(left[-1, 0] - 280) < 8 and abs(right[-1, 0] - 360) < 8, "trap top edges wrong"


def test_plan_returns_consistent_keys():
    cfg = _path_cfg()
    mask = np.zeros((480, 640), dtype=np.uint8)
    mask[100:460, 100:500] = 255
    result = path_planner.plan(mask, cfg)
    assert set(result.keys()) >= {"left", "right", "center"}
    assert result["left"].shape == result["right"].shape == result["center"].shape


# =========================================================================
# 2. smoke: 跑一张合成图，主流程不掉链子
# =========================================================================

def test_smoke_synth_image():
    cfg = _load_effective_cfg("blue_path")
    img = cv2.imread(str(_PROJECT_ROOT / "tests" / "data" / "synth.png"))
    assert img is not None, "missing tests/data/synth.png"
    road_mask, floor_mask, edges, steer, speed, lookahead = _process(img, cfg)
    path = edges["center"]
    road_pix = int((road_mask > 0).sum())
    floor_pix = int((floor_mask > 0).sum())
    total = img.shape[0] * img.shape[1]
    # 道路应是大面积（占图像相当比例）
    assert road_pix > total * 0.2, f"道路掩码太小 ({road_pix} px)"
    assert floor_pix > 1000, "地面掩码完全没命中"
    # 路径应全在道路中央
    img_cx = img.shape[1] / 2
    assert img.shape[1] * 0.2 < np.nanmin(path[:, 0]) < img.shape[1] * 0.8
    assert abs(np.nanmean(path[:, 0]) - img_cx) < img.shape[1] * 0.2
    # 对称图上 steer 应接近 0
    assert abs(steer) < 5.0, f"对称图上 steer 应当接近 0，实际 {steer}"


# =========================================================================
# 3. straight_road: 任意角度的直线道路，center 严格是直线
# =========================================================================

def _make_tilted_road(angle_deg: float, H: int = 480, W: int = 640) -> np.ndarray:
    cx, cy = W / 2, H / 2
    half_w = 40.0
    a = np.deg2rad(angle_deg)
    nx, ny = -np.sin(a), np.cos(a)
    dx, dy = np.cos(a), np.sin(a)
    half_len = min(H, W) / 2 - 60
    p1 = (cx + dx * half_len + nx * half_w, cy + dy * half_len + ny * half_w)
    p2 = (cx - dx * half_len + nx * half_w, cy - dy * half_len + ny * half_w)
    p3 = (cx - dx * half_len - nx * half_w, cy - dy * half_len - ny * half_w)
    p4 = (cx + dx * half_len - nx * half_w, cy + dy * half_len - ny * half_w)
    mask = np.zeros((H, W), dtype=np.uint8)
    cv2.fillPoly(mask, [np.array([p1, p2, p3, p4], dtype=np.int32)], 255)
    return mask


def _line_residual(path_xy: np.ndarray) -> float:
    ys = path_xy[:, 1].astype(np.float64)
    xs = path_xy[:, 0].astype(np.float64)
    if not np.all(np.isfinite(xs)):
        return float("inf")
    c = np.polyfit(ys, xs, 1)
    return float(np.max(np.abs(xs - np.polyval(c, ys))))


def test_straight_road_strictly_linear_at_all_angles():
    cfg = _path_cfg()
    for ang in [-60, -30, -10, 0, 10, 30, 60, 89]:
        mask = _make_tilted_road(ang)
        edges = path_planner.plan(mask, cfg)
        rc = _line_residual(edges["center"])
        assert rc < 0.5, f"angle {ang}°: center 残差 {rc:.3f} px"


# =========================================================================
# 4. steer_response: 道路偏右 → 正 steer
# =========================================================================

def test_steer_responds_to_offcenter_road():
    cfg = _load_effective_cfg("blue_path")
    H, W = 480, 640
    img = np.full((H, W, 3), 60, dtype=np.uint8)
    # 道路：左侧 1/4 起到右边界（偏右）
    img[int(H * 0.40):int(H * 0.95), W // 4:, :] = (203, 116, 72)
    img[int(H * 0.95):, :, :] = (47, 130, 238)
    _, _, edges, steer, speed, _ = _process(img, cfg)
    path = edges["center"]
    path_cx = float(np.nanmean(path[:, 0]))
    assert path_cx > W / 2, f"路径应偏右，实际 {path_cx}"
    assert steer > 1.0, f"路径偏右应产生正 steer，实际 {steer}"


# =========================================================================
# 5. color_shift: RGB 各 ±15/±20 随机偏移，算法仍稳定
# =========================================================================

def test_color_shift_tolerance():
    cfg = _load_effective_cfg("blue_path")
    H, W = 480, 640
    base = np.zeros((H, W, 3), dtype=np.uint8)
    base[int(H * 0.40):int(H * 0.95), :, :] = (203, 116, 72)
    base[int(H * 0.95):, :, :] = (47, 130, 238)
    rng = np.random.default_rng(42)
    shifts = rng.integers(-20, 21, size=(9, 3))
    for sh in shifts:
        img = np.clip(base.astype(np.int16) + sh, 0, 255).astype(np.uint8)
        road_mask, floor_mask, _, steer, _, _ = _process(img, cfg)
        road_pct = 100 * (road_mask > 0).sum() / (H * W)
        assert road_pct > 20, f"shift {sh}: road pct too low ({road_pct:.1f}%)"
        assert abs(steer) < 3.0, f"shift {sh}: steer {steer} too large"


# =========================================================================
# 6. degenerate: 边界 / 退化输入
# =========================================================================

def _degenerate_case(name, road_mask, cfg, expect_nonempty=True, allow_nan=False):
    edges = path_planner.plan(road_mask, cfg)
    pts = edges["center"]
    ok = pts.size > 0
    if ok != expect_nonempty:
        raise AssertionError(f"[{name}] expected nonempty={expect_nonempty}, got size={pts.size}")
    if ok and not allow_nan:
        assert np.all(np.isfinite(pts)), f"[{name}] path contains NaN/Inf"


def test_degenerate_cases_handled():
    cfg = _path_cfg_with_roi()
    _degenerate_case("empty road mask", np.zeros((480, 640), dtype=np.uint8), cfg, allow_nan=True)

    m = np.zeros((480, 640), dtype=np.uint8)
    m[200:201, 100:500] = 255
    _degenerate_case("single-row road", m, cfg)

    m = np.zeros((480, 640), dtype=np.uint8)
    m[200:300, 320:321] = 255
    _degenerate_case("1-pixel-wide road", m, cfg)

    cfg_bad_roi = {**cfg, "roi": {"top_ratio": 0.5, "bottom_ratio": 0.5, "left_ratio": 0, "right_ratio": 1}}
    m = np.zeros((480, 640), dtype=np.uint8)
    m[200:300, 100:500] = 255
    _degenerate_case("zero-height ROI", m, cfg_bad_roi)

    cfg_inv = {**cfg, "roi": {"top_ratio": 0.9, "bottom_ratio": 0.4, "left_ratio": 0, "right_ratio": 1}}
    m = np.zeros((480, 640), dtype=np.uint8)
    m[200:300, 100:500] = 255
    _degenerate_case("inverted ROI", m, cfg_inv, allow_nan=True)

    m = np.zeros((480, 640), dtype=np.uint8)
    m[200:460, 100:500] = 255
    _degenerate_case("normal road", m, cfg)

    _degenerate_case("all-road image", np.full((480, 640), 255, dtype=np.uint8), cfg)


# =========================================================================
# 7. multi_blob: keep_largest_component + filter_path_outliers 抗屏边反光
# =========================================================================

def test_multi_blob_filter_rejects_spurious_blob():
    cfg = _path_cfg()
    H, W = 480, 640
    road = np.zeros((H, W), dtype=np.uint8)
    road[:, 200:300] = 255  # 真道路：中间竖条
    road[50:120, 450:550] = 255  # 屏边反光：右上一小块

    # 过滤前：路径被小斑拽歪
    raw_path = path_planner.plan(road, cfg)["center"]
    assert np.nanstd(raw_path[:, 0]) > 5, "filter 前应能看到抖动"

    # 过滤后：路径稳定在中线 250 附近
    clean = color_segmenter.keep_largest_component(road)
    assert clean.sum() < road.sum(), "filter should remove the small blob"
    assert (clean[:, 200:300] > 0).all(), "vertical strip should be kept"
    cleaned_path = path_planner.plan(clean, cfg)["center"]
    assert abs(np.nanmean(cleaned_path[:, 0]) - 250) < 5
    assert np.nanstd(cleaned_path[:, 0]) < 2


# =========================================================================
# 8. jitter: EMA 平滑器显著降低跨帧路径抖动
# =========================================================================

def test_jitter_ema_reduces_path_std():
    cfg = _load_effective_cfg("blue_path")
    img = cv2.imread(str(_PROJECT_ROOT / "tests" / "data" / "test_image.png"))
    assert img is not None, "missing tests/data/test_image.png"

    rng = np.random.default_rng(123)
    noisy_frames = [
        np.clip(img.astype(np.int16) + rng.normal(0, 8, img.shape), 0, 255).astype(np.uint8)
        for _ in range(30)
    ]
    for i in range(len(noisy_frames)):
        dx = rng.integers(-1, 2)
        noisy_frames[i] = np.roll(noisy_frames[i], dx, axis=1)

    # 不平滑
    raw_paths = []
    for f in noisy_frames:
        _, _, e, _, _, _ = _process(f, cfg)
        raw_paths.append(e["center"])
    arr = np.array([p[:, 0] for p in raw_paths])
    mid = arr.shape[1] // 3
    raw_std = float(np.std(arr[:, mid:2 * mid]))

    # 加 EMA
    smoother = PathSmoother(alpha=0.4)
    smooth_paths = []
    smooth_la_x = []
    for f in noisy_frames:
        _, _, e, _, _, _ = _process(f, cfg)
        sp = smoother.update(e["center"])
        smooth_paths.append(sp)
        if sp.size and not np.all(np.isnan(sp[:, 0])):
            idx = int(cfg["controller"]["lookahead_row_from_bottom"])
            idx = max(0, min(idx, sp.shape[0] - 1))
            smooth_la_x.append(float(sp[idx, 0]))
    arr = np.array([p[:, 0] for p in smooth_paths])
    smooth_std = float(np.std(arr[:, mid:2 * mid]))

    assert smooth_std < raw_std * 0.7, (
        f"EMA 应该把路径抖动降低 ≥30%，但 raw={raw_std:.3f} smooth={smooth_std:.3f}"
    )
    assert max(smooth_la_x) - min(smooth_la_x) <= 2.5, (
        f"平滑后 lookahead x 跨度 {max(smooth_la_x) - min(smooth_la_x):.1f} 太大"
    )


# =========================================================================
# 直接运行入口
# =========================================================================

def main() -> int:
    funcs = [v for k, v in globals().items() if k.startswith("test_") and callable(v)]
    failed = 0
    for fn in funcs:
        t0 = time.perf_counter()
        try:
            fn()
            dt = (time.perf_counter() - t0) * 1000
            print(f"  ok:   {fn.__name__:<40s} ({dt:6.1f} ms)")
        except Exception as e:
            dt = (time.perf_counter() - t0) * 1000
            print(f"  FAIL: {fn.__name__:<40s} ({dt:6.1f} ms)  {e}")
            failed += 1
    print(f"--- {len(funcs) - failed}/{len(funcs)} 通过 ---")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
