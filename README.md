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
| PC UI | PyQt5 ≥ 5.15 | Qt 监控窗口（视频 + 模式选择 + 启停）|
| 语言 | Python 3.10+ | 显式类型注解 `from __future__ import annotations` |

无深度学习框架，无外部硬件依赖（PC 端只要 OpenCV + Qt；车辆端可选挂 ROS 1）。

## 1. 仓库结构

```
Leading_Line/
├── README.md                      # 本文件
├── requirements.txt
├── config.yaml                    # ★ 车辆端 + 算法端配置（network / camera / modes / 算法段 / ros / overrides / debug）
├── config_pc.yaml                 # PC 端配置（network.vehicle_ip / video_port / cmd_port / modes 副本）
│
├── pc/                            # ★ PC 监控端
│   ├── main.py                    #   入口（python -m pc.main）
│   ├── ui/
│   │   ├── main_window.py         #   Qt 主窗口：左视频 / 右菜单 / 底部状态栏
│   │   └── video_view.py          #   BGR → QPixmap 转换
│   └── comm/
│       ├── command_sender.py      #   PC→车辆 TCP（带重连 + 心跳）
│       └── video_receiver.py      #   车辆→PC UDP（后台线程 + 有界队列）
│
├── vehicle/                       # ★ 车辆端（不绑死 Jetson 硬件）
│   ├── main.py                    #   入口（python -m vehicle.main）
│   ├── pipeline.py                #   主循环：摄像头 → 算法 → override → 渲染 → UDP 推流 + 响应命令
│   ├── overrides.py               #   路径算法之上的覆盖层：箭头 + QR
│   ├── ros_bridge.py              #   算法 → /cmd_vel（mock / ROS 双后端 + sonar/bat 安全闸）
│   ├── algo/                      #   引导线算法核心
│   │   ├── color_segmenter.py     #     HSV 分割 + 形态学 + 最大连通块
│   │   ├── path_planner.py        #     边缘采样 + 中点 + 异常值剔除 + 平滑拟合 + 跨帧 EMA
│   │   ├── controller.py          #     前瞻点偏差 → (steer, speed) + 曲率感知降速
│   │   └── visualizer.py          #     道路外轮廓 + 中心引导线 + 前瞻点 + HUD
│   ├── comm/
│   │   ├── video_sender.py        #     UDP 视频发送（带 seq/ts 头）
│   │   └── command_receiver.py    #     TCP 命令接收（多线程）
│   └── recognition/               #   覆盖层识别
│       ├── arrow_detector.py      #     黑色箭头方向识别（Otsu + 凸包多边形 + 内角打分）
│       └── qr/
│           ├── decoder.py         #     OpenCV QRCodeDetector 包装
│           ├── state_machine.py   #     显式状态机：IDLE → SCANNING → DECODED → POLICY_ACTIVE → REPORTING
│           └── policy.py          #     QR 策略文本解析（JSON / key=value）
│
├── debug/                         # ★ 调试 CLI 集合
│   ├── README.md
│   ├── algo_preview.py            #   单机算法预览（cv2 窗口）
│   ├── arrow_image.py             #   离线箭头识别（图片）
│   ├── arrow_webcam.py            #   实时箭头识别（摄像头）
│   ├── arrow_samples.py           #   生成箭头测试样本
│   ├── qr_preview.py              #   QR 状态机调试（摄像头 / 离线）
│   └── qr_samples.py              #   生成 QR 策略样本
│
├── protocol/                      # ★ 双端共享
│   ├── constants.py               #   模式 / 状态 / 端口常量
│   ├── messages.py                #   TCP 文本命令 + UDP 视频帧协议
│   └── mode_resolver.py           #   cfg["modes"][name] → effective cfg
│
├── tests/                         # ★ 测试
│   ├── unit/
│   │   ├── test_algorithm.py      #   引导线算法 10 个测试（合并原 8 个）
│   │   ├── test_qr_policy.py      #   QR 策略解析
│   │   ├── test_qr_state_machine.py
│   │   ├── test_qr_e2e.py         #   QR 解码 + 状态机端到端
│   │   └── test_comm.py           #   PC ↔ 车辆 socket 双端联通
│   ├── data/                      #   测试样本（不入 pytest 自动发现）
│   │   ├── synth.png              #     合成图（make_synth.py 生成）
│   │   ├── test_image.png         #     抖动测试用
│   │   ├── arrow/                 #     箭头测试样本 + README
│   │   └── qr/qr_state_machine_samples/  # QR 策略样本
│   └── tools/
│       └── make_synth.py          #   合成图生成器
│
└── ros_pkgs/leading_line/         # ROS 1 launch 包
    ├── README.md                  #   ROS 端使用文档
    ├── CMakeLists.txt
    ├── package.xml
    ├── launch/                    #   4 个 launch 文件
    │   ├── leading_line.launch            # 仅起 leading_line 节点
    │   ├── leading_line_with_car.launch   # xcar + realsense + leading_line
    │   ├── leading_line_teleop.launch     # 上面 + 键盘遥控
    │   └── leading_line_pc_monitor.launch # ★ 方式 A：xcar + vehicle/main.py（PC 监控）
    ├── scripts/
    │   ├── node.py                #   ROS 节点：订 image_topic → 算法 → /cmd_vel
    │   ├── pipeline_launcher.py   #   roslaunch 包装器：init_node + vehicle.main()
    │   └── xcar/                  #   zonesion xcar 串口桥（自带，从 mbot 包移植）
    │       ├── xcar_ros.py        #     /cmd_vel ↔ /dev/ttyXCar
    │       ├── xcar_protocol.py   #     zonesion 二进制协议
    │       └── mbot_teleop.py     #     键盘遥控
    └── config/params.yaml         #   ROS 节点参数
```

## 2. 算法流程

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
override 层：箭头方向（高置信度）→ QR 策略（POLICY_ACTIVE）→ 路径算法（兜底）
   │
   ▼
前瞻点偏差 → 转向角 + 曲率感知车速
   │
   ▼
可视化：道路外轮廓（红）+ 中心引导线（绿）+ 前瞻点（黄）+ HUD（steer / speed）
   │
   ▼
JPEG 编码 → UDP 发给 PC；RUNNING 时把 (steer, speed) → RosBridge → /cmd_vel
```

## 3. 安装

```bash
pip install -r requirements.txt
```

**额外 ROS 端依赖**（如果用 `ros_pkgs/leading_line/launch/`）：

```bash
sudo apt install ros-<distro>-realsense2-camera python3-serial
# 或：pip install pyserial
```

## 4. 启动方式

### 4.1 PC 端（Qt 监控 UI）

```bash
python pc/main.py --config config_pc.yaml
# 或：python -m pc.main --config config_pc.yaml
```

- 左：实时视频（30Hz 拉帧）
- 右：模式单选 + 开始/结束按钮 + 状态栏
- 底部：ACK / INFO / 连接状态

### 4.2 车辆端（双端模式主程序）

```bash
python vehicle/main.py --config config.yaml
# 或：python -m vehicle.main --config config.yaml
```

启动后会：
1. 读 `config.yaml`（算法 + network + ros + overrides）
2. 打开 UDP 视频发送（→ PC 的 `network.pc_ip:video_port`）
3. 打开 TCP 命令接收（监听 `0.0.0.0:cmd_port`）
4. 起 `RosBridge`（默认 `mock`；实车改 `"ros"`）
5. 主循环跑：摄像头 → 算法 → override → 渲染 → UDP 发图，响应 TCP 命令

### 4.3 ROS 1 launch（实车部署常用）

```bash
# 编译
cd ~/ros_ws && catkin_make && source devel/setup.bash

# 把项目根的 vehicle/ 和 protocol/ 软链到包内
ln -s /path/to/Leading_Line/vehicle   ~/ros_ws/src/leading_line/vehicle
ln -s /path/to/Leading_Line/protocol ~/ros_ws/src/leading_line/protocol

# 仅跑算法（外部已有摄像头 + 底盘）
roslaunch leading_line leading_line.launch

# 完整启动：xcar + realsense + 算法
roslaunch leading_line leading_line_with_car.launch

# 加键盘遥控
roslaunch leading_line leading_line_teleop.launch

# ★ 方式 A 一键：xcar + vehicle/main.py（PC 监控）—— 最常用
roslaunch leading_line leading_line_pc_monitor.launch
```

详见 [`ros_pkgs/leading_line/README.md`](ros_pkgs/leading_line/README.md)。

### 4.4 调试入口

```bash
# 单机算法预览（cv2 窗口，避开 Qt 和双端）
python debug/algo_preview.py --source tests/data/synth.png

# 箭头 / QR 单项调试
python debug/arrow_image.py tests/data/arrow/arrow_up.png
python debug/arrow_webcam.py
python debug/qr_preview.py --mode test --source tests/data/qr/qr_state_machine_samples/turn_left.png
```

完整列表见 [`debug/README.md`](debug/README.md)。

## 5. 双端架构（PC ↔ 车辆）

```
┌────────────────────────┐         UDP (JPEG 视频)          ┌────────────────────────┐
│        车辆            │  ─────────────────────────►  │          PC             │
│                        │                                 │                        │
│  vehicle/main.py       │                                 │  pc/main.py (Qt)       │
│   └─ pipeline.py       │  ◄─────────────────────────  │   └─ ui/main_window.py │
│       ├─ camera        │         TCP (文本命令)          │       ├─ 视频显示       │
│       ├─ color/planner │            "MODE blue_path"     │       ├─ 模式菜单       │
│       ├─ controller    │            "START" / "STOP"    │       ├─ 开始/结束按钮  │
│       └─ ros_bridge    │            "PING" / "ACK"      │       └─ 状态栏         │
│            │           │                                 │            │           │
│            ▼           │                                 │            ▼           │
│     /cmd_vel (ROS)     │                                 │      CommandSender     │
└────────────────────────┘                                 └────────────────────────┘
```

### 5.1 通信协议

详见 [`protocol/messages.py`](protocol/messages.py)：

- **TCP 命令（PC → 车辆）**：一行一条 UTF-8 文本
  - `MODE <name>`   —— 切换模式（name 必须是 `config.yaml` 里 `modes.*` 的 key）
  - `START`         —— 开始寻路
  - `STOP`          —— 停车
  - `PING`          —— 心跳（PC 算 RTT）
  - `QUIT`          —— 收到后退出主循环
  - `ACK <kind>`    —— PC 对 Jetson 推回的 REPORTING 状态做确认
- **TCP 回包（车辆 → PC）**：`ACK <kind>` / `STATUS <state>` / `INFO <text>` / `PONG <ts>`
- **UDP 视频（车辆 → PC）**：每包 = `[4字节大端长度 N][1字节 seq][8字节 ts_ms][N 字节 JPEG]`
  - 总长 13 + N；seq 0~255 循环；丢包由 PC 静默消化

### 5.2 模式机制

PC UI 上"蓝色路径模式 / 绿色路径模式 / 测试模式"对应 `config.yaml` 里的
`modes.*` 三段配置。每段都包含 `colors`（HSV 阈值）和 `visualization`（BGR 渲染色）。
切换模式时只覆盖这两段，算法其它参数（roi / morphology / path / controller / temporal）保持不变。

| 模式 | `modes.<name>.label` | 道路（path）HSV | 谷地（valley）HSV | 路径线 BGR |
|---|---|---|---|---|
| 蓝色路径模式 | `蓝色路径模式` | H∈[100, 130] S>50 V>50 | H∈[0, 80] S>8 V>15 | 绿 (0,255,0) |
| 绿色路径模式 | `绿色路径模式` | H∈[35, 85] S>50 V>50 | 与蓝色模式相同 | 绿 (0,255,0) |
| 测试模式 | `测试模式` | H 全、S<80、V∈[40,180]（灰） | H 全、S<60、V>200（白） | 灰 (128,128,128) |

> 模式名常量在 `protocol/constants.py`：`MODE_BLUE = "blue_path"`、
> `MODE_GREEN = "green_path"`、`MODE_TEST = "test"`，**双端必须严格一致**。

如何新增一个模式：
1. 在 `config.yaml` 的 `modes` 下加一段，例如 `purple_path:`
2. 在 `protocol/constants.py` 加 `MODE_PURPLE = "purple_path"` 并加到 `ALL_MODES`
3. UI 自动按 `list_modes(cfg)` 列出；车辆自动能解析
4. 重启两端（不重启也行，但 cfg 是启动时读一次）

## 6. 配置项速查

`config.yaml` 段：

- **network**：车辆需要知道 PC 的 IP（`pc_ip`）、视频/命令端口；监听 `bind_host`
- **camera**：设备号、分辨率、帧率
- **colors.road / colors.floor**：`hsv_lower` / `hsv_upper`（HSV 三个通道的上下界，OpenCV H 范围 0-179）
- **modes.\<name\>.colors / visualization**：每个模式自己的颜色 + 渲染色
- **roi**：上下左右比例，只在该矩形内做处理
- **morphology**：`kernel_size`、`opening_iter`（去噪）、`closing_iter`（补洞）
- **filter**：`min_road_area_px`，小于此像素数的道路掩码直接当噪声扔掉
- **path**：`num_samples` 采样行数、`smooth_window` 滑动平均窗口、`poly_degree` 多项式阶数
- **controller**：`lookahead_row_from_bottom` 前瞻点、`max_steer_deg`、`base_speed`、`min_speed`、`curvature_k` 曲率降速系数
- **visualization**（在 modes 内）：`path_color_bgr`（绿）、`edge_color_bgr`（红）、`road_overlay_bgr`（橙，已停用）、`road_overlay_alpha`（=0 表示关闭）、`show_hud`
- **temporal**：`enabled`、`alpha`（EMA 系数）、`reset_on_no_road`（道路消失时重置历史）
- **ros**：`backend`（"mock" / "ros"）、`wheelbase_m`
- **vehicle_runtime**：`jpeg_quality`、`fps_cap`
- **overrides.arrow / overrides.qr**：覆盖层模块开关与参数
- **debug.qr_preview**：`debug/qr_preview.py` 用

`config_pc.yaml` 段：

- **network.vehicle_ip**：PC 要连的车辆 IP；`video_port` / `cmd_port` 与车辆端一致
- **modes.\<name\>**：只需要 `label`（PC UI 显示用），HSV/可视化参数对 PC 不起作用

> 颜色相关键（`path_color_bgr` / `edge_color_bgr` / `road_overlay_bgr`）以 OpenCV 内部 BGR 顺序记录，与 `colors.*` 下的 HSV 标称值不混淆。

## 7. 调参建议

按以下顺序调参最稳：

1. `colors.road.hsv_lower` / `hsv_upper` — 光照变化大时把 H 区间放宽、S/V 收紧
2. `roi.top_ratio` — 去掉远处干扰
3. `morphology.*` — 抖动 / 空洞多就调
4. `filter.min_road_area_px` — 误识别多就调大
5. `path.smooth_window` 与 `path.poly_degree` — 曲线抖就加窗 / 降阶（已内置"近似直线则跳过 polyfit"保护）
6. `temporal.alpha` — 跨帧抖动大就调小（更平滑但更迟钝）
7. `controller.lookahead_row_from_bottom` — 前瞻太近反应慢、太远反应迟钝

## 8. 测试

```bash
# 算法核心 + 鲁棒性（合并到一个文件里，10 个测试）
python tests/unit/test_algorithm.py

# QR 子系统
python tests/unit/test_qr_policy.py
python tests/unit/test_qr_state_machine.py
python tests/unit/test_qr_e2e.py            # 解码 + 状态机端到端

# 双端 socket 联通
python tests/unit/test_comm.py

# 工具
python tests/tools/make_synth.py            # 重新生成合成图 tests/data/synth.png
```

详细覆盖项见各文件 docstring。

## 9. 调试工具

[`debug/`](debug/) 集合了所有"单文件调试入口"：

| 脚本 | 用途 | 替代关系 |
|---|---|---|
| `algo_preview.py` | 单机算法预览（cv2 窗口） | 替代了原根 `main.py` |
| `arrow_image.py` | 离线箭头识别（图片） | 替代了 `recognition/arrow/detect_image.py` |
| `arrow_webcam.py` | 实时箭头识别（摄像头） | 替代了 `recognition/arrow/detect_webcam.py` |
| `arrow_samples.py` | 生成箭头测试样本 | 替代了 `recognition/arrow/generate_samples.py` |
| `qr_preview.py` | QR 状态机调试 | 替代了 `recognition/qr/qr_main.py`（已并入 `config.yaml` 的 `debug.qr_preview` 段） |
| `qr_samples.py` | 生成 QR 策略样本 | 替代了 `recognition/qr/qr_make_test.py` |

详见 [`debug/README.md`](debug/README.md)。

## 10. 安全设计

| 触发 | 反应 |
|---|---|
| 状态 ≠ RUNNING | 持续发 0 |
| 摄像头断流 > `image_timeout_s` | 发 0 |
| 任一 `/xcar/sonarN` < `sonar_stop_m` | 立刻发 0（log warn） |
| `/xcar/sensors` bat < 6.5V | 强制停车（log error） |
| `/xcar/sensors` bat < 7.0V | 打 warn 日志 |
| 节点关闭（Ctrl+C / SIGTERM） | 最后发一次 0 |
| xcar 底盘内置 1s 无 /cmd_vel | 自动停车（兜底）|

安全实现统一在 [`vehicle/ros_bridge.py`](vehicle/ros_bridge.py) 的 `_check_estop` / `_check_battery`，
被 ROS 节点和 UDP pipeline 共享。

## 11. 跨平台兼容

| 平台 | 兼容性 |
|---|---|
| zonesion xcar (4WD 全向) | ✅ 完全适配（默认）|
| 阿克曼底盘（turn_on_wheeltec_robot）| ✅ 仍可用：launch 改成对应底盘包，`linear.y=0` 自动忽略 |
| 纯差速底盘（带 Twist 支持）| ✅ `linear.y=0` 忽略即可 |
| ROS 2（rclpy）| ⚠️ 当前节点用 rospy；要迁到 ROS 2 需要重写 import 和 spin |

## 12. 相关文档

- [`debug/README.md`](debug/README.md) — 调试工具使用说明
- [`ros_pkgs/leading_line/README.md`](ros_pkgs/leading_line/README.md) — ROS 端使用文档
- [`tests/data/arrow/README.md`](tests/data/arrow/README.md) — 箭头测试样本说明
