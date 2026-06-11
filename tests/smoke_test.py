"""冒烟测试：不弹窗，直接验证 process_frame 的输出合理性。"""
import sys
import numpy as np
import cv2

sys.path.insert(0, ".")
from main import load_config, process_frame
from protocol import select_mode as _select_mode
from jetson.algo import visualizer


cfg = load_config("jetson/config.yaml")
_cfg_raw = cfg; cfg, _ = _select_mode(_cfg_raw, "blue_path")
img = cv2.imread("tests/synth.png")
assert img is not None, "missing tests/synth.png"

road_mask, floor_mask, edges, steer, speed, lookahead = process_frame(img, cfg)
path = edges["center"]

road_pix = int((road_mask > 0).sum())
floor_pix = int((floor_mask > 0).sum())
total = img.shape[0] * img.shape[1]

print(f"image: {img.shape}, total px = {total}")
print(f"road_mask  : {road_pix:>8d} px  ({100*road_pix/total:5.1f}%)")
print(f"floor_mask : {floor_pix:>8d} px  ({100*floor_pix/total:5.1f}%)")
print(f"steer = {steer:+.2f} deg  (expect near 0 on symmetric road)")
print(f"speed = {speed:.2f}")
print(f"lookahead = {lookahead}")
print(f"path shape = {path.shape}, x range = "
      f"[{np.nanmin(path[:,0]):.1f}, {np.nanmax(path[:,0]):.1f}]")

# 断言：道路应是大面积（占图像相当比例）
assert road_pix > total * 0.2, "道路掩码太小，可能 tolerance 太小"
# 地面应至少有几个像素命中
assert floor_pix > 1000, "地面掩码完全没命中"
# 路径应全在道路中央：x 范围居中
img_cx = img.shape[1] / 2
assert img.shape[1] * 0.2 < np.nanmin(path[:, 0]) < img.shape[1] * 0.8, "路径 x 越界"
assert abs(np.nanmean(path[:, 0]) - img_cx) < img.shape[1] * 0.2, "路径中心偏离图像中心"
# 转向角应接近 0（左右对称）
assert abs(steer) < 5.0, f"对称图上 steer 应当接近 0，实际 {steer}"

# 渲染一张结果图存盘，方便人工目视
vis = visualizer.draw(
    img, road_mask, edges, steer, speed, lookahead, cfg["visualization"]
)
cv2.imwrite("tests/synth_result.png", vis)
print("saved tests/synth_result.png")
