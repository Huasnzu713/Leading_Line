# -*- coding: utf-8 -*-
"""路径规划：在道路掩码上沿多行采样左右边缘的中点，平滑后得到引导线。

所有对外部输入（摄像头帧、道路掩码）的退化情形都做了兜底：
- ROI 非法 / 高度太小 → 直接返回空数组
- 道路掩码空 / 采样行全空 → 返回空数组
- 多项式拟合退化（常数列、点数过少、SVD 不收敛）→ 跳过 polyfit

另提供 PathSmoother 做跨帧指数滑动平均（EMA），专门对付
摄像头噪声/曝光变化导致的逐帧跳动。
"""
from __future__ import annotations

import numpy as np


def _roi_slices(mask: np.ndarray, roi: dict) -> tuple[slice, slice] | None:
    h, w = mask.shape
    top = int(roi["top_ratio"] * h)
    bottom = int(roi["bottom_ratio"] * h)
    left = int(roi["left_ratio"] * w)
    right = int(roi["right_ratio"] * w)
    # 兜底：保证区间合法且至少 2 行高
    top = max(0, min(top, h - 2))
    bottom = max(top + 2, min(bottom, h))
    left = max(0, min(left, w))
    right = max(left + 1, min(right, w))
    return slice(top, bottom), slice(left, right)


def sample_center_points(
    road_mask: np.ndarray, roi: dict, num_samples: int
) -> np.ndarray:
    """便捷包装：只返回中心点。完整功能见 sample_edges。"""
    return sample_edges(road_mask, roi, num_samples)["center"]


def sample_edges(
    road_mask: np.ndarray, roi: dict, num_samples: int
) -> dict:
    """在 ROI 内均匀采 num_samples 桶，每桶用中位行的左右最外侧道路像素。

    实现思路：先扫出 ROI 内所有"有道路像素"的行，再按 y 自下而上
    分成 num_samples 个桶，每桶取中位行的 xs[0]/xs[-1] 当作左右边界。

    返回字典，键 'left' / 'right' / 'center' 都是 (N, 2) 数组；
    该桶没有任何道路行时记 NaN。
    y 坐标是原图坐标（不是 ROI 内坐标），方便后续可视化。
    自下而上排列（索引 0 = 最靠近小车的行）。
    """
    h, w = road_mask.shape
    slices = _roi_slices(road_mask, roi)
    if slices is None:
        empty = np.empty((0, 2), dtype=np.float32)
        return {"left": empty, "right": empty, "center": empty}
    row_slice, col_slice = slices
    y0, y1 = row_slice.start, row_slice.stop
    roi_h = y1 - y0
    if roi_h < 1:
        empty = np.empty((0, 2), dtype=np.float32)
        return {"left": empty, "right": empty, "center": empty}

    num_samples = max(2, int(num_samples))
    sub = road_mask[row_slice, col_slice]
    offset_x = col_slice.start

    has_road = (sub > 0).any(axis=1)
    road_local_ys = np.where(has_road)[0]
    if road_local_ys.size == 0:
        nan_pts = np.full((num_samples, 2), np.nan, dtype=np.float32)
        return {"left": nan_pts, "right": nan_pts, "center": nan_pts}

    road_local_ys = np.sort(road_local_ys)
    if road_local_ys.size < num_samples:
        bucket_assign = np.arange(road_local_ys.size)
        n_out = road_local_ys.size
    else:
        idx = np.arange(road_local_ys.size)
        bucket_assign = (idx * num_samples // road_local_ys.size).astype(int)
        n_out = num_samples

    left = np.full((n_out, 2), np.nan, dtype=np.float32)
    right = np.full((n_out, 2), np.nan, dtype=np.float32)
    for b in range(n_out):
        local_ys_in_bucket = road_local_ys[bucket_assign == b]
        if local_ys_in_bucket.size == 0:
            continue
        mid_local_y = int(np.median(local_ys_in_bucket))
        xs = np.where(sub[mid_local_y] > 0)[0]
        if xs.size == 0:
            continue
        y_img = mid_local_y + y0
        left[b]  = (int(xs[0])  + offset_x, y_img)
        right[b] = (int(xs[-1]) + offset_x, y_img)
    # 自下而上排列
    left  = left[::-1]
    right = right[::-1]
    center = np.full_like(left, np.nan)
    good = ~np.isnan(left[:, 0]) & ~np.isnan(right[:, 0])
    center[good, 0] = (left[good, 0] + right[good, 0]) / 2.0
    center[good, 1] = left[good, 1]
    return {"left": left, "right": right, "center": center}


def filter_path_outliers(
    points: np.ndarray, max_jump_ratio: float = 0.5
) -> np.ndarray:
    """沿"自下而上"方向，剔除 x 跳变过大的采样点。

    思路：相邻两桶的 x 偏移不应该超过图像宽度的 max_jump_ratio（默认 50%）。
    一旦某点偏移过大，把它标 NaN，下游的 _interp_nan 会用相邻有效值补上。
    """
    if points is None or points.size < 3:
        return points
    n, _ = points.shape
    if not np.all(np.isfinite(points[:, 1])):
        return points
    # 用 y 方向跨度估算单像素比例（对默认 ROI 来说就是图像高度）
    y_span = float(np.nanmax(points[:, 1]) - np.nanmin(points[:, 1])) or 1.0
    # 估计图像宽度：用 x 的最大值（这里用 2*max_x 是粗估，但只要 > max_jump 即可）
    img_w = float(np.nanmax(np.abs(points[:, 0])) * 2) or 1.0
    max_jump = img_w * max_jump_ratio
    out = points.copy()
    # 自下而上：索引 0 是最靠近车的；用绝对跳变做判定
    last_x = out[0, 0]
    for i in range(1, n):
        cur_x = out[i, 0]
        if not np.isfinite(cur_x):
            continue
        if abs(cur_x - last_x) > max_jump:
            out[i, 0] = np.nan
        else:
            last_x = cur_x
    return out


def _interp_nan(points: np.ndarray) -> np.ndarray:
    """对 NaN 做线性插值；不足时用有效值外推；全 NaN 时返回原数组。"""
    if points.size == 0 or np.all(np.isnan(points[:, 0])):
        return points
    n = points.shape[0]
    xs = np.arange(n)
    good = ~np.isnan(points[:, 0])
    if good.sum() < 2:
        # 1 个或 0 个有效点：用那一个值铺满（保持 y 的原值不动）
        first = int(np.argmax(good))
        points[:, 0] = points[first, 0]
        return points
    points[~good, 0] = np.interp(xs[~good], xs[good], points[good, 0])
    return points


def _moving_average(points: np.ndarray, window: int) -> np.ndarray:
    if points.size == 0:
        return points
    if window <= 1:
        return points.copy()
    if window % 2 == 0:
        window += 1
    pad = window // 2
    padded = np.pad(points[:, 0], (pad, pad), mode="edge")
    kernel = np.ones(window) / window
    smoothed = np.convolve(padded, kernel, mode="valid")
    out = points.copy()
    out[:, 0] = smoothed
    return out


def _is_degenerate_for_polyfit(xs: np.ndarray, ys: np.ndarray) -> bool:
    """判断是否不适合做多项式拟合（常数 y、点数过少、x 全相等）。"""
    if xs.size < 2:
        return True
    if np.ptp(ys) < 1e-6:           # y 全相等 → 退化为常数拟合
        return True
    if np.ptp(xs) < 1e-6:           # x 全相等 → 拟合无意义
        return True
    # 拟合要求 y 之间至少有 degree+1 个不同值；这里粗略要求至少 2 个
    return False


_LINEAR_RESIDUAL_PX = 0.6


def _is_approximately_linear(xs: np.ndarray, ys: np.ndarray) -> bool:
    """数据是否近似一条直线（直线残差 < 0.6 像素）。

    若近似为直线，强制不做高阶 polyfit —— 否则 np.polyfit(..., 3)
    会在数值噪声下"过拟合出曲线"，把原本直的道路画弯。
    """
    if xs.size < 3:
        return True
    try:
        coeffs = np.polyfit(ys, xs, 1)
    except (np.linalg.LinAlgError, ValueError):
        return True
    residual = np.max(np.abs(xs - np.polyval(coeffs, ys)))
    return bool(residual < _LINEAR_RESIDUAL_PX)


def _polyfit(points: np.ndarray, degree: int) -> np.ndarray:
    """用多项式把 (x, y) 表达为 x = f(y)，再重采样回 num_samples 个点。

    退化情形下保持原 points 不动（不再二次采样），让上层可以照常工作。
    关键：若数据近似为直线，直接返回输入，避免 2 阶以上 polyfit 把直线拗成曲线。

    注意：np.linspace(min, max, n) 永远生成升序 y；本函数上游约定"自下而上"（y 降序），
    所以 polyfit 后要根据原顺序决定是否再翻回来，否则 left/right 边顺序会错乱。
    """
    n = points.shape[0]
    if n < 2:
        return points
    ys = points[:, 1].astype(np.float64)
    xs = points[:, 0].astype(np.float64)
    if _is_degenerate_for_polyfit(xs, ys):
        return points
    if _is_approximately_linear(xs, ys):
        return points  # 已经是直线了，再 polyfit 反而画蛇添足

    degree = max(1, min(int(degree), n - 1))
    try:
        coeffs = np.polyfit(ys, xs, degree)
    except (np.linalg.LinAlgError, ValueError):
        return points
    poly = np.poly1d(coeffs)
    new_ys = np.linspace(ys.min(), ys.max(), n)
    new_xs = poly(new_ys)
    if not np.all(np.isfinite(new_xs)):
        return points
    out = np.stack([new_xs, new_ys], axis=1).astype(np.float32)
    # 保持上游约定的 y 顺序（自下而上 = 降序）
    if points[0, 1] > points[-1, 1]:
        out = out[::-1]
    return out


def smooth_path(points: np.ndarray, window: int, poly_degree: int) -> np.ndarray:
    """滑动平均 → NaN 插值 → 多项式拟合，返回 (N, 2) 平滑点。"""
    if points.size == 0:
        return points
    points = _moving_average(points, window)
    points = _interp_nan(points)
    return _polyfit(points, poly_degree)


def plan(road_mask: np.ndarray, cfg: dict) -> dict:
    """便捷函数：采样 + 异常值过滤 + 平滑一气呵成。

    返回字典：{'left': (N,2), 'right': (N,2), 'center': (N,2)}
    center 字段 = (left+right)/2，与 sample_center_points 等价，
    加上左右两条曲线即可直接画到画面上。
    """
    edges = sample_edges(
        road_mask, cfg["roi"], cfg["path"]["num_samples"]
    )
    cfg_path = cfg["path"]
    for key in ("left", "right", "center"):
        pts = filter_path_outliers(edges[key], max_jump_ratio=0.5)
        edges[key] = smooth_path(pts, cfg_path["smooth_window"], cfg_path["poly_degree"])
    return edges


class PathSmoother:
    """跨帧 EMA 平滑器。专门解决"路径在画面上左右抖动"。

    用法：
        smoother = PathSmoother(alpha=0.4)
        ...
        path = smoother.update(path_planner.plan(road_mask, cfg))

    行为：
        - 第一帧：直接用新值（无历史可比）
        - 后续帧：prev = alpha * new + (1 - alpha) * prev
        - 新值为 NaN 的点保持 prev 不变（避免断流导致整条线飞掉）
        - 形状变了 / 输入为空 → 重置为新值
    """

    def __init__(self, alpha: float = 0.4) -> None:
        if not 0.0 < alpha <= 1.0:
            raise ValueError(f"alpha 必须在 (0, 1] 区间，得到 {alpha}")
        self.alpha = float(alpha)
        self._prev: np.ndarray | None = None

    def reset(self) -> None:
        """外部强制重置历史（例如切场景、离开道路又回来时）。"""
        self._prev = None

    def update(self, path_xy: np.ndarray) -> np.ndarray:
        if path_xy is None or path_xy.size == 0:
            self._prev = None
            return path_xy

        if (
            self._prev is None
            or self._prev.shape != path_xy.shape
        ):
            self._prev = path_xy.astype(np.float32).copy()
            return self._prev.copy()

        prev = self._prev
        new = path_xy.astype(np.float32, copy=True)
        good = ~np.isnan(new[:, 0])
        if good.any():
            prev[good] = self.alpha * new[good] + (1.0 - self.alpha) * prev[good]
        # NaN 点：保持 prev
        self._prev = prev
        return self._prev.copy()
