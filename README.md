# Leading Line — 谷仓场景引导线自动驾驶算法

轻量、可解释、不依赖深度学习的引导线（leading line）算法。  
适用场景：道路颜色固定、地面颜色固定、无其他车辆的受控环境（典型为谷仓内部通道）。

> **2026-06 重构**：运动控制从 ROS1 + Python 桥接迁移到 **ROS2 Humble + 阿克曼架构**。底盘串口驱动用 C++/rclcpp 实现，视觉感知复用 `leading_line.algo`/`leading_line.recognition`（不重写 cv2/QR/箭头）。原 ROS1 串口桥 `xcar_ros.py`/`xcar_protocol.py`/`mbot_teleop.py`、参考源码 `turn_on_wheeltec_robot/`/`wheeltec_robot_rc/`、Python 控制版本 `vehicle/ros_bridge.py` 全部删除；2026-06-12 起 `vehicle/` 整个目录**合并到 `ros2_pkgs/leading_line/leading_line/`，单一来源**。新内容见 [ros2_pkgs/](ros2_pkgs/)。

## 技术栈

| 类别 | 选型 | 用途 |
|---|---|---|
| 视觉 | OpenCV ≥ 4.5 (`cv2`) | 颜色分割 / 形态学 / 摄像头 IO / 找轮廓 / 可视化 / QR 解码 |
| 数值 | NumPy ≥ 1.21 | 掩码、采样点、滑动平均 / 多项式拟合 / EMA |
| 配置 | PyYAML ≥ 6.0 | 加载 `config.yaml` |
| QR 生成 | `qrcode` ≥ 7.0 | 仅用于生成测试 QR 样本；解码走 OpenCV 自带 `QRCodeDetector` |
| PC UI | PyQt5 ≥ 5.15 | Qt 监控窗口（视频 + 模式选择 + 启停）|
| ROS 端 | **ROS2 Humble** + C++ (rclcpp) + Python (rclpy) | 底盘驱动 + 视觉感知 + PC 监控桥 |
| 底盘协议 | zonesion 0x2B-A2 私有协议 | 与 xcar MCU 串口通信（115200）|
| 底盘消息 | `ackermann_msgs/AckermannDriveStamped` | ROS 顶层运动接口（阿克曼标准）|
| 串口 | `libserial-dev` (C++) | 替代原 Python `pyserial` |
| 语言 | Python 3.10+ + C++17 | Python 算法/C++ 底盘 |

无深度学习框架。

## 1. 仓库结构

```
Leading_Line/
├── README.md                      # 本文件
├── requirements.txt
├── config.yaml                    # ★ 车辆端 + 算法端配置（network / camera / modes / 算法段 / ros / overrides / debug）
├── config_pc.yaml                 # PC 端配置（network.vehicle_ip / video_port / cmd_port / modes 副本）
│
├── pc/                            # ★ PC 监控端（Qt UI + TCP/UDP）
│   ├── main.py                    #   入口（python -m pc.main）
│   ├── ui/{main_window.py, video_view.py}
│   └── comm/{command_sender.py, video_receiver.py}
│
├── vehicle/                       # （已删除 — 合并到 ros2_pkgs/leading_line/leading_line/，单一来源）
│   # 历史：原 vehicle/pipeline.py + vehicle/{algo,recognition,overrides.py,comm}/
│   # 现位于 ros2_pkgs/leading_line/leading_line/{pipeline.py,algo/,recognition/,overrides.py,comm/}
│
├── debug/                         # ★ 调试 CLI 集合
│
├── protocol/                      # ★ 双端共享字节协议
│
├── tests/                         # ★ 测试
│   └── unit/                      #   10 个算法测试 + 4 个 QR 测试 + 1 个 PC↔vehicle 协议 e2e
│
└── ros2_pkgs/                     # ★ ROS2 Humble 运动控制（替换原 ros_pkgs/ + turn_on_wheeltec_robot/）
    ├── README.md                  #   详细说明
    ├── build.sh                   #   colcon build 一键
    ├── check_topics.sh            #   启动后跑的话题/服务/参数自检
    ├── mock_serial.py             #   桌面 mock 串口（zonesion 0x2B-A2 注入）
    ├── leading_line_chassis/      #   C++ 底盘驱动：/ackermann_cmd → serial → /odom /imu /battery
    └── leading_line/              #   rclpy 视觉感知 + PC 桥：/image → algo → /ackermann_cmd
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

**额外 ROS2 端依赖**（如果用 `ros2_pkgs/`）：

```bash
sudo apt install -y \
    ros-humble-ackermann-msgs \
    ros-humble-cv-bridge \
    ros-humble-tf2-ros \
    ros-humble-robot-state-publisher \
    ros-humble-usb-cam \
    libserial-dev \
    python3-pyzbar
```

## 4. 启动方式

### 4.1 PC 端（Qt 监控 UI，不变）

```bash
python pc/main.py --config config_pc.yaml
# 或：python -m pc.main --config config_pc.yaml
```

- 左：实时视频（30Hz 拉帧）
- 右：模式单选 + 开始/结束按钮 + 状态栏
- 底部：ACK / INFO / 连接状态

### 4.2 车辆端（CV2 + TCP/UDP 老链路，作为 ROS2 节点运行）

```bash
# ROS2 节点形式（推荐；vehicle_pipeline 在 rclpy 里跑 headless pipeline + 推流 + 命令）
ros2 run leading_line vehicle_pipeline --ros-args -p config_path:=$PWD/config.yaml
```

启动后会：
1. 读 `config.yaml`（算法 + network + ros + overrides）
2. 打开 UDP 视频发送（→ PC 的 `network.pc_ip:video_port`）
3. 打开 TCP 命令接收（监听 `0.0.0.0:cmd_port`）
4. `publish_cmd` 回调里把 (steer, speed) 推到 ackermann_bridge（rclpy 发布到 `/ackermann_cmd`）
5. 主循环跑：摄像头 → 算法 → override → 渲染 → UDP 发图，响应 TCP 命令

### 4.3 ROS2 Humble 全栈（实车部署推荐）

```bash
# 编译
mkdir -p ~/ros2_ws/src
ln -s $PWD/ros2_pkgs/leading_line_chassis ~/ros2_ws/src/
ln -s $PWD/ros2_pkgs/leading_line ~/ros2_ws/src/
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select leading_line_chassis leading_line
source install/setup.bash

# 仅底盘（外部已有视觉）
ros2 launch leading_line_chassis chassis.launch.py

# 仅视觉（外部已有底盘 + 摄像头）
ros2 launch leading_line perception.launch.py

# 全栈：底盘 + USB 摄像头 + 视觉
ros2 launch leading_line full_stack.launch.py \
    usart_port_name:=/dev/ttyXCar \
    camera_backend:=usb_cam

# 全栈 + PC 监控
ros2 launch leading_line full_stack.launch.py \
    usart_port_name:=/dev/ttyXCar \
    camera_backend:=usb_cam \
    use_pc_monitor:=true \
    pc_ip:=192.168.1.100
```

详见 [`ros2_pkgs/README.md`](ros2_pkgs/README.md)。

### 4.4 桌面联调（mock 串口，无 Jetson）

```bash
# 1) 装 socat
sudo apt install socat

# 2) 虚拟串口对
socat -d -d pty,raw,echo=0 pty,raw,echo=0 &
# 假设 /dev/ttyVCar0 ↔ /dev/ttyVCar1

# 3) 启底盘
ros2 launch leading_line_chassis chassis.launch.py usart_port_name:=/dev/ttyVCar0 &

# 4) 另一窗口启 mock 串口注入
python3 ros2_pkgs/mock_serial.py /dev/ttyVCar1

# 5) 启视觉
ros2 launch leading_line perception.launch.py
ros2 service call /leading_line/start_stop std_srvs/srv/SetBool "{data: true}"
ros2 topic echo /ackermann_cmd
ros2 topic hz /odom     # 期望 ~50 Hz
```

### 4.5 调试入口

```bash
# 单机算法预览（cv2 窗口，避开 Qt 和双端）
python debug/algo_preview.py --source tests/data/synth.png

# 箭头 / QR 单项调试
python debug/arrow_image.py tests/data/arrow/arrow_up.png
python debug/arrow_webcam.py
python debug/qr_preview.py --mode test --source tests/data/qr/qr_state_machine_samples/turn_left.png
```

完整列表见 [`debug/README.md`](debug/README.md)。

## 5. 双端架构（PC ↔ 车辆 + ROS2 内部）

```
┌────────────────────────┐         UDP (JPEG 视频)          ┌────────────────────────┐
│        车辆 Jetson     │  ─────────────────────────►  │          PC             │
│  Ubuntu 22.04 + Humble │                                 │   pc/main.py (Qt)      │
│                        │  ◄─────────────────────────  │                         │
│  ┌──────────────────┐  │         TCP (文本命令)          │   └─ ui/main_window.py │
│  │  leading_line/   │  │            "MODE blue_path"     │       ├─ 视频显示       │
│  │   pipeline.py    │  │            "START/STOP/PING"   │       ├─ 模式菜单       │
│  │  ├─ camera       │  │                                 │       └─ 状态栏         │
│  │  ├─ algo/*       │  │                                 │                         │
│  │  └─ publish_cmd  │  │                                 └────────────────────────┘
│  │        │         │  │
│  │        ▼         │  │
│  │ ackermann_bridge │  │        ROS2 内部
│  │  /ackermann_cmd  │  │  ┌──────────────────────────┐
│  └──────────────────┘  │  │  leading_line_chassis     │
│                        │  │  (C++ rclcpp)             │
│                        │  │  - ackermann → (vx,vy,wz) │
│                        │  │  - serial 0x2B-A2         │
│                        │  │  - /odom /imu /battery    │
│                        │  │  - watchdog 500ms         │
│                        │  └─────────────┬──────────────┘
│                        │                │ /dev/ttyXCar
└────────────────────────┘                ▼
                                 zonesion xcar MCU
                                 (阿克曼 4WD 底盘)
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
- **ros**：`backend`（"mock" / "ros"）、`wheelbase_m`（仅 PC 监控链路需要；ROS2 端通过 launch 参数传给底盘）
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

# ROS2 视觉感知（无 ROS 依赖，纯算法）
python ros2_pkgs/leading_line/test/test_perception_pipeline.py
python ros2_pkgs/leading_line/test/test_ackermann_bridge.py

# ROS2 底盘（C++ GTest，需 colcon build 后跑）
colcon test --packages-select leading_line_chassis

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

## 10. ROS2 Humble 阿克曼架构（新）

2026-06 迁移。原 ROS1 + Python 串口桥链路（`ros_pkgs/leading_line/scripts/xcar/{xcar_ros.py, xcar_protocol.py, mbot_teleop.py}`）已全部删除，参考源码 `turn_on_wheeltec_robot/`、`wheeltec_robot_rc/`、Python 控制版本 `vehicle/ros_bridge.py` 同样删除；2026-06-12 起 `vehicle/` 整个目录**合并到 `ros2_pkgs/leading_line/leading_line/`，单一来源**。

新的运动控制栈在 [ros2_pkgs/](ros2_pkgs/) 下：

| 包 | 语言 | 作用 |
|---|---|---|
| `leading_line_chassis` | **C++ / rclcpp** | 订阅 `/ackermann_cmd` (`ackermann_msgs/AckermannDriveStamped`)；按阿克曼自行车模型换算 `(steering, speed) → (vx, vy=0, wz)`；用 zonesion 0x2B-A2 协议打 `/dev/ttyXCar`；50 Hz 主循环回放 `/odom` `/imu` `/battery_voltage` `/xcar/sonar[1-4]` `/xcar/sensors`；watchdog 500 ms 无命令自动发 0；on_shutdown 发 0 |
| `leading_line` | **Python / rclpy** | 视觉感知：订阅 `/image` → 跑 `leading_line.algo` 系列算法 → 发布 `/ackermann_cmd`；提供 `~/start_stop`、`~/set_mode` 服务；可选 `pc_monitor_bridge` 桥接 PC TCP/UDP 监控；同时包含历史 `vehicle/pipeline.py` 改造后的 headless pipeline（`ros2 run leading_line vehicle_pipeline`） |

### 编译 / 运行

```bash
# 编译
mkdir -p ~/ros2_ws/src
ln -s $PWD/ros2_pkgs/leading_line_chassis ~/ros2_ws/src/
ln -s $PWD/ros2_pkgs/leading_line ~/ros2_ws/src/
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select leading_line_chassis leading_line
colcon test  --packages-select leading_line_chassis leading_line

# 全栈
source install/setup.bash
ros2 launch leading_line full_stack.launch.py \
    usart_port_name:=/dev/ttyXCar \
    camera_backend:=usb_cam \
    use_pc_monitor:=true \
    pc_ip:=192.168.1.100
```

### 关键 ROS 接口

| 方向 | 名称 | 类型 | 说明 |
|---|---|---|---|
| sub | `/ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` | 唯一运动输入 |
| pub | `/odom` | `nav_msgs/Odometry` | wheel odometry + TF odom→base_footprint |
| pub | `/imu` | `sensor_msgs/Imu` | zonesion 无 IMU，用 odom 角速度填 |
| pub | `/battery_voltage` | `std_msgs/Float32` | 0.1V 单位的电池电压 |
| pub | `/xcar/sonar[1-4]` | `sensor_msgs/Range` | 4 路超声 |
| pub | `/xcar/sensors` | `std_msgs/Int32MultiArray` | 透传 zonesion 0x03 |
| srv | `/leading_line/start_stop` | `std_srvs/SetBool` | True→RUNNING, False→STOPPED |
| srv | `/leading_line/set_mode` | `std_srvs/SetBool` | data="blue_path"/"green_path"/"test" |

## 11. 安全设计

| 触发 | 反应 | 实现位置 |
|---|---|---|
| 状态 ≠ RUNNING | 持续发 0 | `perception_node._tick` |
| 摄像头断流 > `image_timeout_s` | 发 0 | `perception_node._tick` |
| 任一 `/xcar/sonarN` < `sonar_stop_m` | 立刻发 0（log warn）| `perception_node._check_estop` |
| `/xcar/sensors` bat < `bat_critical_0p1V` (6.5V) | 强制停车（log error）| `perception_node._check_battery` |
| `/xcar/sensors` bat < `bat_warn_0p1V` (7.0V) | 打 warn 日志 | `perception_node._on_sensors` |
| `/ackermann_cmd` 缺失 > `cmd_watchdog_ms` (500ms) | 底盘自动发 0 | `ChassisNode.tick` |
| 节点关闭（Ctrl+C / SIGTERM） | 最后发一次 0 | `rclcpp::on_shutdown` 回调 + dtor |
| 串口 read 异常 / BCC 失败 | 1Hz 日志 + 1s 重连 | `ChassisNode.reconnectSerial` |
| xcar MCU 内置 1s 无下行命令 | 自动停车（兜底）| MCU 固件 |

## 12. 跨平台兼容

| 平台 | 兼容性 |
|---|---|
| zonesion xcar (4WD 阿克曼, 0x2B-A2 协议) | ✅ 完全适配（默认） |
| Wheeltec 24 字节 STM32 协议 | ⚠️ 改 `src/protocol.cpp` 即可（结构风格已照搬 `turn_on_wheeltec_robot`） |
| 纯差速底盘（带 `Twist` 支持）| ⚠️ 加一个 `/cmd_vel → /ackermann_cmd` 桥节点即可（推荐 `twist_to_ackermann`）|
| ros2_control + diff_drive_controller | 🔜 未来重构路径：把 `chassis_node` 拆成 `ros2_control` 硬件接口 |

## 13. 相关文档

- [`debug/README.md`](debug/README.md) — 调试工具使用说明
- [`ros2_pkgs/README.md`](ros2_pkgs/README.md) — **ROS2 端使用文档（主入口）**
- [`ros2_pkgs/leading_line_chassis/README.md`](ros2_pkgs/leading_line_chassis/README.md) — C++ 底盘节点 API
- [`ros2_pkgs/leading_line/README.md`](ros2_pkgs/leading_line/README.md) — rclpy 视觉节点 API
- [`tests/data/arrow/README.md`](tests/data/arrow/README.md) — 箭头测试样本说明
