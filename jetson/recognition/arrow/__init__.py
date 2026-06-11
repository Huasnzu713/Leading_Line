"""箭头方向识别（Otsu + 黑色校验 + 凸包多边形近似 + 内角打分）。

入口 `arrow_detector.detect_arrow(frame)` 返回 ArrowResult 或 None。
被 `jetson.overrides.FrameOverrides` 调用。
"""
