"""非对称场景：把道路整体左移，验证 steer 给出正方向（右打方向回到中线）的反馈。"""
import sys
import numpy as np
import cv2

sys.path.insert(0, ".")
from main import load_config, process_frame
from protocol import select_mode as _select_mode


cfg = load_config("jetson/config.yaml")
_cfg_raw = cfg; cfg, _ = _select_mode(_cfg_raw, "blue_path")
H, W = 480, 640
img = np.full((H, W, 3), 60, dtype=np.uint8)
# 路面：左侧 1/4 起到右边界，宽 3/4 W
img[int(H * 0.40):int(H * 0.95), W // 4:, :] = (203, 116, 72)  # 道路 (BGR)
img[int(H * 0.95):, :, :] = (47, 130, 238)                     # 地面 (BGR)

_, _, edges, steer, speed, lookahead = process_frame(img, cfg)
path = edges["center"]
img_cx = W / 2
path_cx = float(np.nanmean(path[:, 0]))
print(f"image center x = {img_cx}, path center x = {path_cx:.1f}")
print(f"path is {'left' if path_cx < img_cx else 'right'} of image center")
print(f"steer = {steer:+.2f} deg  speed = {speed:.2f}")
print(f"lookahead = {lookahead}")

# 道路偏右 → 路径中心偏右 → 跟随路径应右打方向 → steer > 0
assert path_cx > img_cx, "路径应该偏右"
assert steer > 1.0, f"路径偏右应产生正的 steer，实际 {steer}"

print("OK: steer responded correctly to off-center road")
