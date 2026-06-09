# Leading Line — 谷仓场景引导线自动驾驶算法

轻量、可解释、不依赖深度学习的引导线（leading line）算法。  
适用场景：道路颜色固定、地面颜色固定、无其他车辆的受控环境（典型为谷仓内部通道）。

## 1. 算法流程

```
摄像头帧 (BGR)
   │
   ▼
颜色分割（RGB 欧氏距离）──► 道路掩码 + 地面掩码
   │
   ▼
形态学清理（开 + 闭运算）
   │
   ▼
按 ROI 多行采样：每行取左右最外侧道路像素的中点
   │
   ▼
滑动平均 → 缺失段插值 → 三次多项式拟合（蓝色曲线）
   │
   ▼
前瞻点偏差 → 转向角 + 曲率感知车速
   │
   ▼
可视化：红色半透明道路叠加 + 蓝色曲线 + HUD（steer / speed）
```

## 2. 文件结构

| 文件 | 作用 |
|---|---|
| `config.yaml` | 全部可调参数集中地 |
| `color_segmenter.py` | RGB 距离阈值的颜色分割 + 形态学清理 |
| `path_planner.py` | 道路边缘采样 + 中点提取 + 平滑拟合 |
| `controller.py` | 由路径输出转向角与车速 |
| `visualizer.py` | 红色道路叠加 + 蓝色曲线 + HUD |
| `main.py` | 入口，组装主循环，支持摄像头/图片/视频 |

## 3. 安装与运行

```bash
pip install -r requirements.txt

# 摄像头模式
python main.py

# 离线跑一张图
python main.py --source tests/synth.png

# 跑一段视频
python main.py --source path/to/video.mp4
```

可视化窗口中按 **ESC** 退出，按 **s** 保存当前帧为 `snapshot.png`。

## 4. 配置项速查（`config.yaml`）

- **camera**：设备号、分辨率、帧率
- **colors.road / colors.floor**：`rgb` 为标称 RGB；`tolerance` 为欧氏距离阈值
- **roi**：上下左右比例，只在该矩形内做处理
- **morphology**：`kernel_size`、`opening_iter`（去噪）、`closing_iter`（补洞）
- **path**：`num_samples` 采样行数、`smooth_window` 滑动平均窗口、`poly_degree` 多项式阶数
- **controller**：`lookahead_row_from_bottom` 前瞻点、`max_steer_deg`、`base_speed`、`min_speed`、`curvature_k` 曲率降速系数
- **visualization**：`path_color_bgr`（蓝）、`road_overlay_bgr`（红）、`road_overlay_alpha` 透明度、`show_hud`
- **runtime**：`print_to_stdout`、`window_name`、`exit_key`

> 颜色相关键（`path_color_bgr` / `road_overlay_bgr`）以 OpenCV 内部 BGR 顺序记录，避免与 `colors.*` 下的 RGB 标称值混淆。

## 5. 调参建议

按以下顺序调参最稳：

1. `colors.road.tolerance` — 光照变化大时调大
2. `roi.top_ratio` — 去掉远处干扰
3. `morphology.*` — 抖动/空洞多就调
4. `path.smooth_window` 与 `path.poly_degree` — 曲线抖就加窗/降阶
5. `controller.lookahead_row_from_bottom` — 前瞻太近反应慢、太远反应迟钝

## 6. 与主控对接

当前 `controller.decide` 只把 steer / speed 打印到控制台与 HUD。  
对接实车时，在 `main.py` 的 `process_frame` 之后串接你的协议（CAN / 串口 / MQTT 等），把同一份 `(steer_deg, speed)` 推下去即可，算法本体的输入输出形状不会变。

## 7. QR 策略下发系统（`qr_system/`）

用二维码做"状态机逻辑策略指定"。扫到码 → 解析为 Policy → 状态机执行 → 输出 `(steer, speed)`。  
支持两种运行模式：

- **camera 模式**：实时从摄像头读流，识别二维码，HUD 显示状态机当前状态与下发值
- **test 模式**：读一张二维码图离线识别，把识别结果画到结果图里

详见 [qr_system/README.md](qr_system/README.md)。

```bash
# 装额外依赖
pip install qrcode

# 生成测试 QR 样本到 tests/qr_state_machine_samples/
python qr_system/qr_make_test.py

# test 模式：读图识别
python qr_system/qr_main.py --mode test  --source tests/qr_state_machine_samples/turn_left.png

# camera 模式：实时
python qr_system/qr_main.py --mode camera
```
