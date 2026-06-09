"""多连通块场景：模拟"屏边反光 + 真道路"或"人脸阴影 + 真道路"。

验证 keep_largest_component + filter_path_outliers 能让路径稳定地
只跟随最大那块（真道路），不被小斑拽歪。
"""
import sys
import numpy as np
import cv2

sys.path.insert(0, ".")
import color_segmenter
import path_planner


cfg = {
    "roi": {"top_ratio": 0.0, "bottom_ratio": 1.0, "left_ratio": 0.0, "right_ratio": 1.0},
    "path": {"num_samples": 20, "smooth_window": 7, "poly_degree": 3},
}

H, W = 480, 640
# 真道路：中间竖条 (x=200..300)
road = np.zeros((H, W), dtype=np.uint8)
road[:, 200:300] = 255
# 屏边反光 / 阴影：右上一小块
road[50:120, 450:550] = 255

raw_path = path_planner.plan(road, cfg)["center"]
print(f"[no filter]   path x range: {np.nanmin(raw_path[:,0]):.0f} ~ {np.nanmax(raw_path[:,0]):.0f}, "
      f"x std: {np.nanstd(raw_path[:,0]):.1f}")

# 过滤后
clean = color_segmenter.keep_largest_component(road)
print(f"  road mask pixels: {road.sum()//255} -> {clean.sum()//255}  "
      f"(should drop small upper-right blob)")
assert clean.sum() < road.sum(), "filter should remove the small blob"
assert (clean[:, 200:300] > 0).all(), "vertical strip should be kept"

cleaned_path = path_planner.plan(clean, cfg)["center"]
print(f"[largest c.c] path x range: {np.nanmin(cleaned_path[:,0]):.0f} ~ "
      f"{np.nanmax(cleaned_path[:,0]):.0f}, x std: {np.nanstd(cleaned_path[:,0]):.1f}")

# 期望：clean 后路径 x 应集中在 250 附近（道路中线），std 极小
assert abs(np.nanmean(cleaned_path[:, 0]) - 250) < 5, "路径中心应贴道路中线 250"
assert np.nanstd(cleaned_path[:, 0]) < 2, "路径不应再被小斑拽到大幅摆动"
print("OK: largest connected component filter rejects the spurious blob")
