# Leading Line — 谷仓场景引导线自动驾驶算法

轻量、可解释、不依赖深度学习的引导线（leading line）算法。  
适用场景：道路颜色固定、地面颜色固定、无其他车辆的受控环境（典型为谷仓内部通道）。

## 技术栈

| 类别 | 选型 | 用途 |
|---|---|---|
| 视觉 | OpenCV ≥ 4.5 (`cv2`) | 颜色分割 / 形态学 / 摄像头 IO / 找轮廓 / 可视化 / QR 解码 |
| 数值 | NumPy ≥ 1.21 | 掩码、采样点、滑动平均 / 多项式拟合 / EMA |
| 配置 | PyYAML ≥ 6.0 | 加载 `config.yaml` |
| QR 生成 | `qrcode` ≥ 7.0 | 仅用于生成测试 QR 样本；解码走 OpenCV 自带 `QRCodeDetector` |
| 语言 | Python 3.10+ | 显式类型注解 `from __future__ import annotations` |

无深度学习框架，无外部硬件依赖。

## 1. 算法流程

```
摄像头帧 (BGR)
   │
   ▼
HSV 颜色分割 (cv2.inRange) ──► 道路掩码 + 地面掩码
   │
   ▼
形态学清理（开 + 闭运算，椭圆核）
   │
   ▼
最大连通块过滤（消除屏边反光 / 墙面等零碎误识别）
   │
   ▼
按 ROI 多行采样：每桶取中位行左右最外侧道路像素的中点
   │
   ▼
异常值剔除 → 滑动平均 → 缺失段插值 → 二次多项式拟合
   │
   ▼
跨帧 EMA 时域平滑（可选，抑制摄像头噪声）
   │
   ▼
前瞻点偏差 → 转向角 + 曲率感知车速
   │
   ▼
可视化：道路外轮廓（红）+ 中心引导线（绿）+ HUD（steer / speed）
```

## 2. 文件结构

| 文件 / 目录 | 作用 |
|---|---|
| `main.py` | 入口，组装主循环，支持摄像头 / 图片 / 视频 |
| `color_segmenter.py` | HSV 颜色分割 + 形态学清理 + 最大连通块过滤 |
| `path_planner.py` | 道路边缘采样 + 中点提取 + 异常值剔除 + 平滑拟合 + 跨帧 EMA |
| `controller.py` | 由路径输出 `(steer_deg, speed)` + 曲率感知降速 |
| `visualizer.py` | 道路外轮廓 + 中心引导线 + 前瞻点 + HUD |
| `config.yaml` | 全部可调参数集中地 |
| `requirements.txt` | Python 依赖清单 |
| `tests/` | 9 个回归测试 + 算法样本图（5 张 PNG） |
| `tests/qr_state_machine_samples/` | QR 系统测试样本（7 张 PNG） |
| `qr_system/` | QR 码驱动的状态机策略系统（见 §8） |
| `qr_system/tests/` | QR 系统的 3 个测试文件 |
| `arrow_recongnize/` | 基于 OpenCV 几何方法的箭头方向识别小工具（见 §9） |
| `arrow_recongnize/samples/` | 自动生成的 9 张示例箭头图（4 方向 × 干净/噪声/旋转） |
| `arrow_recongnize/out/` | 跑 `detect_image.py --save-dir` 产生的标注图 |

## 3. 安装与运行

```bash
pip install -r requirements.txt
pip install qrcode   # 仅 QR 系统需要

# 摄像头模式
python main.py

# 离线跑一张图（结果自动落到 test_result/）
python main.py --source tests/synth.png

# 跑一段视频
python main.py --source path/to/video.mp4

# 调试模式：2x2 网格（原图/道路掩码/地面掩码/结果）
python main.py --source tests/synth.png --debug

# 生成测试 QR 样本到 tests/qr_state_machine_samples/
python qr_system/qr_make_test.py

# test 模式：读图识别
python qr_system/qr_main.py --mode test  --source tests/qr_state_machine_samples/turn_left.png

# camera 模式：实时
python qr_system/qr_main.py --mode camera
```

可视化窗口中按 **ESC** 退出，按 **s** 保存当前帧到 `test_result/`，按 **d** 切换调试模式。

## 4. 配置项速查（`config.yaml`）

- **camera**：设备号、分辨率、帧率
- **colors.road / colors.floor**：`hsv_lower` / `hsv_upper`（HSV 三个通道的上下界，OpenCV H 范围 0-179）
- **roi**：上下左右比例，只在该矩形内做处理
- **morphology**：`kernel_size`、`opening_iter`（去噪）、`closing_iter`（补洞）
- **filter**：`min_road_area_px`，小于此像素数的道路掩码直接当噪声扔掉
- **path**：`num_samples` 采样行数、`smooth_window` 滑动平均窗口、`poly_degree` 多项式阶数
- **controller**：`lookahead_row_from_bottom` 前瞻点、`max_steer_deg`、`base_speed`、`min_speed`、`curvature_k` 曲率降速系数
- **visualization**：`path_color_bgr`（绿）、`edge_color_bgr`（红）、`road_overlay_bgr`（橙，已停用）、`road_overlay_alpha`（=0 表示关闭）、`show_hud`
- **temporal**：`enabled`、`alpha`（EMA 系数）、`reset_on_no_road`（道路消失时重置历史）
- **runtime**：`print_to_stdout`、`window_name`、`exit_key`

> 颜色相关键（`path_color_bgr` / `edge_color_bgr` / `road_overlay_bgr`）以 OpenCV 内部 BGR 顺序记录，与 `colors.*` 下的 HSV 标称值不混淆。

## 5. 调参建议

按以下顺序调参最稳：

1. `colors.road.hsv_lower` / `hsv_upper` — 光照变化大时把 H 区间放宽、S/V 收紧
2. `roi.top_ratio` — 去掉远处干扰
3. `morphology.*` — 抖动 / 空洞多就调
4. `filter.min_road_area_px` — 误识别多就调大
5. `path.smooth_window` 与 `path.poly_degree` — 曲线抖就加窗 / 降阶（已内置"近似直线则跳过 polyfit"保护）
6. `temporal.alpha` — 跨帧抖动大就调小（更平滑但更迟钝）
7. `controller.lookahead_row_from_bottom` — 前瞻太近反应慢、太远反应迟钝

## 6. 测试

```bash
# 算法核心（4 个）
python tests/edges_test.py
python tests/smoke_test.py
python tests/straight_road_test.py
python tests/steer_response_test.py

# 鲁棒性（4 个）
python tests/color_shift_test.py
python tests/degenerate_test.py
python tests/jitter_test.py
python tests/multi_blob_test.py

# 工具
python tests/make_synth.py     # 重新生成合成图 tests/synth.png
```

## 7. 与主控对接

当前 `controller.decide` 只把 steer / speed 打印到控制台与 HUD。  
对接实车时，在 `main.py` 的 `process_frame` 之后串接你的协议（CAN / 串口 / MQTT 等），把同一份 `(steer_deg, speed)` 推下去即可，算法本体的输入输出形状不会变。

## 8. QR 策略下发系统（`qr_system/`）

用二维码做"状态机逻辑策略指定"。扫到码 → 解析为 Policy → 状态机执行 → 输出 `(steer, speed)`。  
支持两种运行模式：

- **camera 模式**：实时从摄像头读流，识别二维码，HUD 显示状态机当前状态与下发值
- **test 模式**：读一张二维码图离线识别，把识别结果画到结果图里

状态机：`IDLE → SCANNING → DECODED → POLICY_ACTIVE → REPORTING → IDLE`（带 `ERROR` 兜底）。  
策略白名单：`STOP` / `CRUISE` / `TURN_LEFT` / `TURN_RIGHT` / `SLOW_DOWN` / `SPEED_UP` / `REVERSE` / `HOLD` / `CUSTOM`。  
`CRUISE` 策略输出 `(NaN, NaN)` 表示"解绑，由引导线算法接管"。

详见 [qr_system/README.md](qr_system/README.md)。

## 9. 箭头方向识别（`arrow_recongnize/`）

轻量、零训练数据的箭头方向识别小工具，基于 OpenCV 几何方法（Otsu 二值化 + 凸包多边形近似 + 内角打分）。  
能识别 **前 / 左 / 右** 三向，给出像素级尖端坐标和 0~1 置信度；摄像头实时模式与图片批量模式都可用。

```bash
cd arrow_recongnize
pip install -r requirements.txt   # 只需 opencv-python、numpy

# 1. 生成示例图到 samples/（已带 9 张，可跳过）
python generate_samples.py

# 2. 对 samples/ 下所有图做识别并打印
python detect_image.py samples/*.png

# 3. 把标注后的图存到 out/
python detect_image.py samples/*.png --save-dir out

# 4. 摄像头实时识别（按 q 退出，按 s 保存当前帧）
python detect_webcam.py
```

示例输出：

```
samples/arrow_up.png:        方向=前    角度=  89.3°  置信度=0.86
samples/arrow_left.png:      方向=左    角度= 179.3°  置信度=0.86
samples/arrow_right.png:     方向=右    角度=   0.7°  置信度=0.86
samples/arrow_up_rot+15.png: 方向=前    角度= 105.3°  置信度=0.69
samples/arrow_down.png:      方向=未知  角度= -90.7°  置信度=0.37
```

完整算法说明、9 张样本的验证结果表、API 字段、已知局限，详见 [arrow_recongnize/README.md](arrow_recongnize/README.md)。

### 与引导线算法对接

`arrow_detector.detect_arrow(frame)` 接受与 `main.py` 同一份摄像头帧 BGR，输出 `(direction, angle_deg, confidence, ...)`。  
典型做法是在 `process_frame` 之前/之后加一段：

```python
import arrow_recongnize.arrow_detector as arrow_detector
arrow = arrow_detector.detect_arrow(frame)
if arrow is not None and arrow.confidence >= 0.5:
    # 用 arrow.direction / angle_deg 覆盖 controller 的输出
    # 例如: 看到"左"则把 steer_deg 替换为 -20，duration 由后续状态机管
    pass
```