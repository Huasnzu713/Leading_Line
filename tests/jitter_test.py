"""用 tests/test_image.png 验证时域平滑对路径抖动的抑制效果。

对比两个场景：
  A) 不加平滑器：30 帧带噪，路径 x 在每帧之间跳变 → std 大
  B) 加上 EMA 平滑器：同样 30 帧带噪，路径 x 抖动明显被压住 → std 小

抖动度量：路径"中段 1/3"在所有有效点的 x 标准差（反映"蓝色线左右抖"）。
"""
import sys
import numpy as np
import cv2

sys.path.insert(0, ".")
from main import load_config, process_frame
import path_planner


def path_mid_std(paths):
    """计算所有帧路径中段的 x 跨帧标准差。"""
    arr = np.array([p[:, 0] for p in paths])  # (T, N)
    mid = arr.shape[1] // 3
    center = arr[:, mid:2 * mid]
    return float(np.std(center))


def main():
    cfg = load_config("config.yaml")
    img = cv2.imread("tests/test_image.png")
    assert img is not None, "missing tests/test_image.png"

    # 模拟 30 帧带噪（高斯噪声 σ=8，更接近真实摄像头）
    rng = np.random.default_rng(123)
    noisy_frames = [
        np.clip(img.astype(np.int16) + rng.normal(0, 8, img.shape), 0, 255).astype(np.uint8)
        for _ in range(30)
    ]

    # 抖动更大：道路在 x 方向加 ±1 像素的整体偏移（模拟车体微小晃动）
    for i, f in enumerate(noisy_frames):
        dx = rng.integers(-1, 2)  # -1, 0, +1
        noisy_frames[i] = np.roll(f, dx, axis=1)

    # 1) 不平滑
    raw_paths = []
    raw_lookaheads = []
    for f in noisy_frames:
        _, _, e, _, _, la = process_frame(f, cfg)
        raw_paths.append(e["center"])
        raw_lookaheads.append(la)
    raw_std = path_mid_std(raw_paths)
    raw_la_x = [la[0] for la in raw_lookaheads if la is not None]
    print(f"[no smoothing] path mid x std = {raw_std:.3f}  "
          f"lookahead x range = [{min(raw_la_x)}, {max(raw_la_x)}]")

    # 2) 加 EMA 平滑器 (alpha=0.4)
    smoother = path_planner.PathSmoother(alpha=0.4)
    smooth_paths = []
    smooth_lookaheads = []
    for f in noisy_frames:
        _, _, e, _, _, la = process_frame(f, cfg)
        p = e["center"]
        # 用平滑后的路径重新算 lookahead（保持一致）
        sp = smoother.update(p)
        smooth_paths.append(sp)
        # lookahead 直接用平滑后的路径重新算
        if sp.size and not np.all(np.isnan(sp[:, 0])):
            idx = int(cfg["controller"]["lookahead_row_from_bottom"])
            idx = max(0, min(idx, sp.shape[0] - 1))
            lx = float(sp[idx, 0])
            smooth_lookaheads.append(lx)
        else:
            smooth_lookaheads.append(None)
    smooth_std = path_mid_std(smooth_paths)
    smooth_la_x = [x for x in smooth_lookaheads if x is not None]
    print(f"[EMA alpha=0.4] path mid x std = {smooth_std:.3f}  "
          f"lookahead x range = [{min(smooth_la_x):.1f}, {max(smooth_la_x):.1f}]")

    # 3) 验证：平滑后抖动应该明显小于未平滑
    assert smooth_std < raw_std * 0.7, (
        f"EMA 应该把路径抖动降低 ≥30%，但 raw={raw_std:.3f} smooth={smooth_std:.3f}"
    )
    # 平滑后的 lookahead x 跨度应 ≤ 2 像素
    assert max(smooth_la_x) - min(smooth_la_x) <= 2.5, (
        f"平滑后 lookahead x 跨度 {max(smooth_la_x) - min(smooth_la_x):.1f} 太大"
    )
    print(f"OK: jitter reduced from {raw_std:.3f} to {smooth_std:.3f} "
          f"({100 * (1 - smooth_std / raw_std):.0f}% reduction)")


if __name__ == "__main__":
    main()
