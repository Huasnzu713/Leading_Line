# 箭头方向识别 (arrowrecongnize)

基于 OpenCV 几何方法的箭头方向识别小工具，能识别 **前 / 左 / 右** 三个方向，
**只检测黑色箭头**（轮廓内平均灰度需 ≤ 阈值，默认 80；其他颜色直接被拒），
无需训练数据，开箱即用。支持图片文件批量识别和摄像头实时识别。

## 环境要求

- Python ≥ 3.8（已在 3.14 测过）
- 见 [requirements.txt](requirements.txt)：`opencv-python`、`numpy`

```bash
pip install -r requirements.txt
```

## 快速上手

```bash
# 1. 生成示例箭头图（往 samples/ 写 9 张测试图）
python generate_samples.py

# 2. 对所有示例做识别
python detect_image.py samples/*.png

# 3. 同时把标注后的图保存到 out/
python detect_image.py samples/*.png --save-dir out

# 4. 开摄像头实时识别（按 q 退出，按 s 保存当前帧）
python detect_webcam.py
```

示例输出：

```
samples/arrow_left.png:        方向=左    角度= 179.3°  置信度=0.86
samples/arrow_right.png:       方向=右    角度=   0.7°  置信度=0.86
samples/arrow_up.png:          方向=前    角度=  89.3°  置信度=0.86
samples/arrow_up_rot+15.png:   方向=前    角度= 105.3°  置信度=0.69
samples/arrow_down.png:        方向=未知   角度= -90.7°  置信度=0.37
```

## 项目结构

| 文件 | 作用 |
|---|---|
| [arrow_detector.py](arrow_detector.py) | 核心算法：`detect_arrow(img) -> ArrowResult` 和 `annotate(img, result)` |
| [detect_image.py](detect_image.py) | 图片 CLI，支持通配符、批量保存标注图 |
| [detect_webcam.py](detect_webcam.py) | 摄像头实时识别，叠加方向标签和 FPS |
| [generate_samples.py](generate_samples.py) | 自动生成示例箭头图（4 方向 × {干净 / 噪声 / 旋转}） |
| [requirements.txt](requirements.txt) | Python 依赖 |
| [samples/](samples/) | 生成的示例图 |
| [out/](out/) | 标注后的输出图（运行 `--save-dir out` 时生成） |

## 算法说明

整条流水线分 6 步，全部在 [arrow_detector.py](arrow_detector.py) 中实现：

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

## 验证结果

对 12 张生成样本的识别结果（覆盖 4 方向 × {干净 / 噪声 / 旋转} + 3 张彩色反例）：

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

## 已知局限

- **只认黑色**：算法刻意只检测"够黑"的箭头（`min_darkness=80`），红/绿/蓝/浅灰等颜色一律拒掉。
  调整 `detect_arrow(img, min_darkness=...)` 阈值可以放宽到深灰或更严到纯黑。
- **多箭头**：当前实现只取最大轮廓，画面中多个箭头时只识别最大的那个。
  如需多目标，把 `_largest_contour` 改成返回所有满足面积阈值的轮廓即可。
- **极度倾斜或视角变形**：算法假设箭头大致正面朝向相机。强透视下尖端可能不在凸包最尖处。
- **复杂背景下的摄像头识别**：依赖 Otsu 二值化能把箭头和背景分开。
  在杂乱场景下建议先把箭头打印在白纸上、或加色彩阈值预筛。
- **朝下的箭头**：被刻意归为"未知"。如需四向支持，在 `_classify_angle` 里加一个 `下` 扇区即可。

## API 示例

如果你想在自己的代码里调用：

```python
import cv2
from arrow_detector import detect_arrow, annotate

img = cv2.imread("samples/arrow_up.png")
result = detect_arrow(img)
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
