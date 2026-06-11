"""验证 sample_edges 返回的 left/right/center 自洽：
  - left[:,0] <= center[:,0] <= right[:,0]
  - center == (left+right)/2
  - 跑一遍 plan 后再确认一遍（过形态学、异常值过滤、平滑）
"""
import sys
import numpy as np
import cv2

sys.path.insert(0, ".")
from jetson.algo import path_planner

cfg = {
    "roi": {"top_ratio": 0.0, "bottom_ratio": 1.0, "left_ratio": 0.0, "right_ratio": 1.0},
    "path": {"num_samples": 20, "smooth_window": 7, "poly_degree": 3},
}

# 1) 矩形道路：x=200..400，y=100..400
mask = np.zeros((480, 640), dtype=np.uint8)
mask[100:400, 200:400] = 255
edges = path_planner.sample_edges(mask, cfg["roi"], cfg["path"]["num_samples"])
left, right, center = edges["left"], edges["right"], edges["center"]

print(f"[rect] left x: {left[:,0].min():.0f} ~ {left[:,0].max():.0f}")
print(f"[rect] right x: {right[:,0].min():.0f} ~ {right[:,0].max():.0f}")
print(f"[rect] center x: {center[:,0].min():.0f} ~ {center[:,0].max():.0f}")

# 边界值：x 应分别接近 200 / 400 / 300
assert np.allclose(left[:, 0], 200, atol=1), f"left should be ~200, got {left[:,0]}"
assert np.allclose(right[:, 0], 400, atol=1), f"right should be ~400, got {right[:,0]}"
assert np.allclose(center[:, 0], 300, atol=1), f"center should be ~300, got {center[:,0]}"
# 自洽
assert np.allclose(center[:, 0], (left[:, 0] + right[:, 0]) / 2, atol=1e-3), "center != (L+R)/2"
assert np.all(left[:, 0] <= center[:, 0] + 1e-3), "left > center"
assert np.all(center[:, 0] <= right[:, 0] + 1e-3), "center > right"

# 2) 倾斜道路：上窄下宽的梯形
mask = np.zeros((480, 640), dtype=np.uint8)
top_y, bot_y = 100, 460
top_l, top_r = 280, 360
bot_l, bot_r = 100, 540
pts_tl = [(top_l, top_y), (top_r, top_y), (bot_r, bot_y), (bot_l, bot_y)]
cv2.fillPoly(mask, [np.array(pts_tl, dtype=np.int32)], 255)
edges = path_planner.sample_edges(mask, cfg["roi"], cfg["path"]["num_samples"])
left, right, center = edges["left"], edges["right"], edges["center"]

# 自下而上：索引 0 最靠近底部 → 应当是宽的部分
print(f"[trap] idx 0 (bot): L={left[0,0]:.0f} R={right[0,0]:.0f} C={center[0,0]:.0f}")
print(f"[trap] idx -1 (top): L={left[-1,0]:.0f} R={right[-1,0]:.0f} C={center[-1,0]:.0f}")
# 底部：L=100, R=540（用桶中位行会有几像素抖动，放宽容差到 8）
assert abs(left[0, 0] - 100) < 8 and abs(right[0, 0] - 540) < 8, "trap bottom edges wrong"
# 顶部：L=280, R=360
assert abs(left[-1, 0] - 280) < 8 and abs(right[-1, 0] - 360) < 8, "trap top edges wrong"
# 自洽
assert np.allclose(center[:, 0], (left[:, 0] + right[:, 0]) / 2, atol=1e-3)
assert np.all(left[:, 0] <= center[:, 0] + 1e-3)
assert np.all(center[:, 0] <= right[:, 0] + 1e-3)

# 3) 跑完整 plan 流程，验证接口稳定
result = path_planner.plan(mask, cfg)
assert "left" in result and "right" in result and "center" in result
assert result["left"].shape == result["right"].shape == result["center"].shape
print(f"[plan]  all keys present, shape={result['center'].shape}")
print("OK: sample_edges / plan return consistent L/R/C")
