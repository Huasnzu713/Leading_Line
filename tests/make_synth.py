"""生成一张合成图用于冒烟测试：
- 上半部是道路 RGB(72,116,203)，呈梯形（模拟近宽远窄的透视）
- 下半部是谷仓地面 RGB(238,130,47)
- 顶部一小块是灰色背景（应该被忽略，不在 ROI 内）
"""
import numpy as np
import cv2

H, W = 480, 640

# 灰色背景（0..40%）
img = np.full((H, W, 3), 60, dtype=np.uint8)

# 道路：梯形，顶边 [320-60, 320+60] @ y=192, 底边 [0, W] @ y=H
road = np.zeros_like(img)
road_roi_corners = np.array(
    [[0, H], [0, int(H * 0.95)], [W, int(H * 0.95)], [W, H]], dtype=np.int32
)
cv2.fillPoly(road, [road_roi_corners], (72, 116, 203))  # OpenCV 用 BGR
# 但 RGB(72,116,203) 在 BGR 是 (203,116,72)
road = np.zeros_like(img)
cv2.fillPoly(road, [road_roi_corners], (203, 116, 72))

# 路面 + 路面边缘上半部：从 y=0.4H 到 y=0.95H 都算路面
upper_top_y = int(H * 0.40)
img[upper_top_y:, :, :] = (203, 116, 72)  # 道路色 (BGR)
img[int(H * 0.95):, :, :] = (47, 130, 238)  # 地面色 (BGR: 238,130,47 -> 47,130,238)

# 加一点高斯噪声让掩码不会太"干净"
noise = np.random.normal(0, 4, img.shape).astype(np.int16)
img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)

out_path = "tests/synth.png"
import os
os.makedirs("tests", exist_ok=True)
cv2.imwrite(out_path, img)
print("wrote", out_path, "shape", img.shape)
