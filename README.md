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
| `tests/arrow/` | 箭头方向识别的 12 张测试样本（9 黑 + 3 彩色反例），见 §9 |
| `tests/qr_state_machine_samples/` | QR 系统测试样本（7 张 PNG） |
| `qr_system/` | QR 码驱动的状态机策略系统（见 §8） |
| `qr_system/tests/` | QR 系统的 3 个测试文件 |

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

## 9. 箭头方向识别（历史方案，代码已移除）

轻量、零训练数据的箭头方向识别方案，基于 OpenCV 几何方法（Otsu 二值化 + **黑色校验** + 凸包多边形近似 + 内角打分）。  
**只检测黑色箭头**：找到候选轮廓后会用轮廓内灰度均值再做一次硬过滤（默认 ≤ 80），红/绿/蓝/浅灰等其他颜色直接返回 `None`。能识别 **前 / 左 / 右** 三向，给出像素级尖端坐标和 0~1 置信度。

> 代码已经从 `arrow_recongnize/` 子模块整合进了主目录的依赖与文档体系，但实际运行入口已删除。下面是当时验证过的算法细节、12 张样本的回归结果与 API 形状，作为知识沉淀保留。
>
> 12 张样本已移到 [tests/arrow/](tests/arrow/)（9 张黑箭头 + 3 张彩色反例），可以用任何 OpenCV 工具验证下文算法说明。

### 算法流水线（6 步）

1. **预处理** — 灰度 → 高斯模糊 → Otsu 自适应二值化 → 若背景被算作前景则反转 → 形态学闭运算补洞。
   结果是箭头为白、背景为黑的稳定二值图。
2. **找箭头轮廓** — `cv2.findContours` 取外轮廓，过滤掉小于图像 0.2% 的噪声，取面积最大者。
3. **黑色校验（关键）** — 用轮廓填出掩码，求**轮廓内灰度均值**。
   若均值 > `min_darkness`（默认 80）则直接返回 `None`，
   这一步把红/绿/蓝/浅灰等"形状像但颜色不够黑"的候选全部过滤掉。
4. **求凸包多边形** — `cv2.convexHull` + `cv2.approxPolyDP`，得到大约 5 个顶点的简化多边形。
   *用凸包而不是原始多边形*，是为了**剔除箭头颈部的两个凹陷顶点**——
   那两个顶点在常规多边形近似下也是"尖角"，会和真正的箭头尖端竞争。
5. **定位尖端** — 对凸包每个顶点计算"两条相邻边的夹角余弦"（越接近 1 越尖），
   用 `(cos + 1) × 到质心距离` 综合打分。这样既偏向锐角，又偏向远端，
   避免把短尾巴的尾角选成尖端。
6. **方向分类** — 计算尖端相对质心的角度（图像 y 轴翻转后用 `atan2(-dy, dx)`），
   按 ±45° 扇区映射：
   - `[45°, 135°]` → **前**
   - `[-45°, 45°]` → **右**
   - `[135°, 180°] ∪ [-180°, -135°]` → **左**
   - 其余（朝下） → **未知**（用户只关心三向，朝下不归入任何方向更安全）

置信度 = `0.5 × 角度居中度 + 0.5 × 尖端锐利度`，两者都在 0~1 之间。

`min_darkness` 可在调用时覆盖：

```python
result = detect_arrow(img, min_darkness=50)   # 更严格：只接受深黑
result = detect_arrow(img, min_darkness=120)  # 更宽松：能接深灰
```

### 12 张样本验证结果

| 样本 | 角度 | 识别方向 | 是否正确 |
|---|---|---|---|
| arrow_up | 89.3° | 前 | ✓ |
| arrow_up_noisy | 89.3° | 前 | ✓ |
| arrow_up_rot+15° | 105.3° | 前 | ✓（容差内） |
| arrow_left | 179.3° | 左 | ✓ |
| arrow_left_noisy | 179.3° | 左 | ✓ |
| arrow_right | 0.7° | 右 | ✓ |
| arrow_right_noisy | 0.6° | 右 | ✓ |
| arrow_right_rot-20° | -20.1° | 右 | ✓（容差内） |
| arrow_down | -90.7° | 未知 | ✓（按设计） |
| **arrow_up_red** | — | 未检测到 | ✓（红色被黑色校验拒掉） |
| **arrow_right_red** | — | 未检测到 | ✓（红色被黑色校验拒掉） |
| **arrow_up_gray** | — | 未检测到 | ✓（浅灰被黑色校验拒掉） |

### API 形状

```python
import cv2
from arrow_detector import detect_arrow, annotate

img = cv2.imread("tests/arrow/arrow_up.png")
result = detect_arrow(img)               # 或 detect_arrow(img, min_darkness=50)
if result is not None:
    print(result.direction, result.angle_deg, result.confidence)
    cv2.imwrite("annotated.png", annotate(img, result))
```

`ArrowResult` 字段：

| 字段 | 类型 | 含义 |
|---|---|---|
| `direction` | `str` | "前" / "左" / "右" / "未知" |
| `angle_deg` | `float` | 质心→尖端的角度，0=右，90=上，±180=左，-90=下 |
| `tip` | `(int, int)` | 尖端像素坐标 |
| `centroid` | `(int, int)` | 质心像素坐标 |
| `contour` | `ndarray` | 原始 OpenCV 轮廓 |
| `confidence` | `float` | 0~1 综合置信度 |
| `en_label` | `str` | "FORWARD" / "LEFT" / "RIGHT" / "UNKNOWN"（用于图上叠加） |

### 已知局限

- **只认黑色**：算法刻意只检测"够黑"的箭头（`min_darkness=80`），红/绿/蓝/浅灰等颜色一律拒掉。
  调整 `detect_arrow(img, min_darkness=...)` 阈值可以放宽到深灰或更严到纯黑。
- **多箭头**：当前实现只取最大轮廓，画面中多个箭头时只识别最大的那个。
  如需多目标，把 `_largest_contour` 改成返回所有满足面积阈值的轮廓即可。
- **极度倾斜或视角变形**：算法假设箭头大致正面朝向相机。强透视下尖端可能不在凸包最尖处。
- **复杂背景下的摄像头识别**：依赖 Otsu 二值化能把箭头和背景分开。
  在杂乱场景下建议先把箭头打印在白纸上、或加色彩阈值预筛。
- **朝下的箭头**：被刻意归为"未知"。如需四向支持，在 `_classify_angle` 里加一个 `下` 扇区即可。