"""色差鲁棒性测试：把合成图的 RGB 各加 ±15 的随机偏移，验证放宽 tolerance 后仍能识别。"""
import sys
import numpy as np
import cv2

sys.path.insert(0, ".")
from main import load_config, process_frame


cfg = load_config("config.yaml")
H, W = 480, 640

base = np.zeros((H, W, 3), dtype=np.uint8)
# 道路在 y=0.4..0.95，x=0..W
base[int(H * 0.40):int(H * 0.95), :, :] = (203, 116, 72)  # 道路 (BGR)
# 地面在 y=0.95..1.0
base[int(H * 0.95):, :, :] = (47, 130, 238)              # 地面 (BGR)

# 9 组偏移：每个通道在 [-20, +20] 随机
rng = np.random.default_rng(42)
shifts = rng.integers(-20, 21, size=(9, 3))

for i, sh in enumerate(shifts):
    img = np.clip(base.astype(np.int16) + sh, 0, 255).astype(np.uint8)
    road_mask, floor_mask, edges, steer, speed, _ = process_frame(img, cfg)
    path = edges["center"]
    road_pct = 100 * (road_mask > 0).sum() / (H * W)
    floor_pct = 100 * (floor_mask > 0).sum() / (H * W)
    sh_str = "(" + ", ".join(f"{int(x):+d}" for x in sh) + ")"
    print(f"shift={sh_str:<14}  road={road_pct:5.1f}%  "
          f"floor={floor_pct:4.1f}%  steer={steer:+.2f}")
    # 道路仍应占相当面积（>20%）
    assert road_pct > 20, f"shift {sh}: road pct too low ({road_pct:.1f}%)"
    # 转向角仍应接近 0（左右对称）
    assert abs(steer) < 3.0, f"shift {sh}: steer {steer} too large"

print("OK: tolerance=80 handles ±20 RGB shifts")
