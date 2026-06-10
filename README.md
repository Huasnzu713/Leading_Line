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
| `protocol/` | **双端共享**：模式常量 + 命令/状态文本协议 + UDP 视频帧协议 + 模式解析器 |
| `protocol/constants.py` | 端口/模式名/状态名常量 |
| `protocol/messages.py` | TCP 文本命令 + UDP 视频帧打包/解包 |
| `protocol/mode_resolver.py` | `select_mode(cfg, name)` 把 modes 段拼成 effective cfg |
| `jetson/` | **Jetson 端**：摄像头 + 算法 + UDP 推流 + TCP 接令 + ROS 桥（见 §10） |
| `jetson/main_jetson.py` | Jetson 端入口（`python -m jetson.main_jetson`） |
| `jetson/pipeline.py` | Jetson 主循环：摄像头 → 算法 → 渲染 → UDP 推流，响应 TCP 命令 |
| `jetson/comm/video_sender.py` | UDP 视频发送 |
| `jetson/comm/command_receiver.py` | TCP 命令接收（多线程） |
| `jetson/ros_bridge.py` | 实车控制接口：`publish_cmd_vel(steer, speed)`，可切 `mock` / `ros` |
| `pc/` | **PC 端**：Qt 监控 UI（见 §11） |
| `pc/main_pc.py` | PC 端入口（`python -m pc.main_pc`） |
| `pc/ui/main_window.py` | Qt 主窗口：左视频 / 右菜单（模式 + 开始/结束）/ 状态栏 |
| `pc/ui/widgets.py` | BGR → QPixmap 的转换 |
| `pc/comm/video_receiver.py` | UDP 视频接收（后台线程） |
| `pc/comm/command_sender.py` | TCP 命令发送（自动重连） |
| `config.yaml` | 模式预设（blue/green/test）+ 算法参数；`main.py` 单机调试仍可用 |
| `config_jetson.yaml` | Jetson 端网络/ROS 段 + 模式预设副本 |
| `config_pc.yaml` | PC 端网络/模式段 |
| `config.yaml` | 全部可调参数集中地 |
| `requirements.txt` | Python 依赖清单 |
| `tests/` | 8 个回归测试 + 算法样本图 |

## 3. 安装与运行

```bash
pip install -r requirements.txt

# 摄像头模式
python main.py

# 离线跑一张图（结果自动落到 test_result/）
python main.py --source tests/synth.png

# 跑一段视频
python main.py --source path/to/video.mp4

# 调试模式：2x2 网格（原图/道路掩码/地面掩码/结果）
python main.py --source tests/synth.png --debug
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


## 8. 模式预设（蓝色 / 绿色 / 测试）

PC UI 上"蓝色路径模式 / 绿色路径模式 / 测试模式"对应 `config.yaml` 里的
`modes.*` 三段配置。每段都包含 `colors`（HSV 阈值）和 `visualization`（BGR 渲染色）。
切换模式时只覆盖这两段，算法其它参数（roi / morphology / path / controller / temporal）保持不变。

| 模式 | `modes.<name>.label` | 道路（path）HSV | 谷地（valley）HSV | 路径线 BGR |
|---|---|---|---|---|
| 蓝色路径模式 | `蓝色路径模式` | H∈[100, 130] S>50 V>50 | H∈[0, 80] S>8 V>15 | 绿 (0,255,0) |
| 绿色路径模式 | `绿色路径模式` | H∈[35, 85] S>50 V>50 | 与蓝色模式相同 | 绿 (0,255,0) |
| 测试模式 | `测试模式` | H 全、S<80、V∈[40,180]（灰） | H 全、S<60、V>200（白） | 灰 (128,128,128) |

> 模式名常量在 `protocol/constants.py`：`MODE_BLUE = "blue_path"`、`MODE_GREEN = "green_path"`、`MODE_TEST = "test"`，**双端必须严格一致**。

如何新增一个模式：

1. 在 `config.yaml` 的 `modes` 下加一段，例如 `purple_path:`
2. 在 `protocol/constants.py` 加 `MODE_PURPLE = "purple_path"` 并加到 `ALL_MODES`
3. UI 自动按 `list_modes(cfg)` 列出；Jetson 自动能解析
4. 重启两端（不重启也行，但 cfg 是启动时读一次）

## 9. 双端架构（Jetson ↔ PC）

完整运行时形态：

```
┌────────────────────────┐         UDP (JPEG 视频)          ┌────────────────────────┐
│       Jetson 小车       │  ─────────────────────────►  │        PC 监控端         │
│                        │                                 │                        │
│  main_jetson.py        │                                 │  main_pc.py (Qt)       │
│   └─ pipeline.py       │  ◄─────────────────────────  │   └─ ui/main_window.py │
│       ├─ camera        │         TCP (文本命令)          │       ├─ 视频显示       │
│       ├─ color/planner │            "MODE blue_path"     │       ├─ 模式菜单       │
│       ├─ controller    │            "START" / "STOP"    │       ├─ 开始/结束按钮  │
│       └─ ros_bridge    │                                 │       └─ 状态栏         │
│            │           │                                 │            │           │
│            ▼           │                                 │            ▼           │
│     /cmd_vel (ROS)     │                                 │      CommandSender     │
└────────────────────────┘                                 └────────────────────────┘
```

### 9.1 通信协议

详见 [protocol/messages.py](protocol/messages.py)：

- **TCP 命令（PC → Jetson）**：一行一条 UTF-8 文本
  - `MODE <name>`   —— 切换模式（name 必须是 `modes.*` 里的 key）
  - `START`         —— 开始寻路
  - `STOP`          —— 停车
  - `PING`          —— 心跳
  - `QUIT`          —— Jetson 收到后退出主循环
- **TCP 回包（Jetson → PC）**：`ACK <kind>` / `STATUS <state>` / `INFO <text>`
- **UDP 视频（Jetson → PC）**：每包 = `[4字节大端长度 N][1字节 seq][8字节 ts_ms][N 字节 JPEG]`
  - 总长 13 + N；seq 0~255 循环；丢包由 PC 静默消化

### 9.2 Jetson 端（[jetson/](jetson/)）

- `jetson/main_jetson.py` —— 入口
- `jetson/pipeline.py` —— 单线程主循环：读摄像头 → 算法 → 渲染（按当前 mode）→ JPEG 编码 → UDP 发图；
  每帧开头先 `cmd_receiver.get(timeout=0)` 拿一条命令（无则继续做算法）
- `jetson/comm/video_sender.py` —— `VideoSender.send(jpeg_bytes)`，单 socket，setblocking(False) 静默丢包
- `jetson/comm/command_receiver.py` —— `socketserver.ThreadingTCPServer`，每条连接一个线程，按行读命令
- `jetson/ros_bridge.py` —— `RosBridge.publish_cmd_vel(steer_deg, speed)`，当前只有 mock 后端
  接入真实 ROS 时改 `ros_bridge.py` 的 `RosBackend` 即可，**算法主循环不需要改一行**

启动：

```bash
# Jetson 端
python -m jetson.main_jetson --config config_jetson.yaml
```

### 9.3 PC 端（[pc/](pc/)）

- `pc/main_pc.py` —— 入口；起 VideoReceiver / CommandSender，打开 Qt 主窗口
- `pc/ui/main_window.py` —— Qt 主窗口（[pc/ui/main_window.py](pc/ui/main_window.py)）
  - 左侧：视频显示（`VideoView`，30Hz `QTimer` 拉帧）
  - 右侧：模式单选组（`ModeGroup`，按 `cfg.modes` 动态生成）+ 开始/结束按钮 + 状态栏
  - 底部 `QStatusBar`：提示 ACK / INFO / 连接状态
- `pc/comm/video_receiver.py` —— 后台线程 `recvfrom` → `decode_video_frame` → cv2 解码 → 有界队列；
  UI 线程从队列取最新帧（丢老帧，保持低延迟）
- `pc/comm/command_sender.py` —— 后台线程长连接 + 自动重连；UI 调 `send_mode` / `send_start` / `send_stop`

启动：

```bash
# PC 端
python -m pc.main_pc --config config_pc.yaml
```

### 9.4 配置互通

`config.yaml` / `config_jetson.yaml` / `config_pc.yaml` 的 `modes` 段**逻辑上一致**；
PC 端只需要 `label` 和模式名（用来显示单选项），HSV/可视化参数对 PC 不起作用。
维护技巧：改 `config.yaml` 的 `modes` 段后，复制粘贴到 `config_jetson.yaml` / `config_pc.yaml` 即可。
如果嫌烦，可以写一个 `merge_configs.py` 做单向合并；本仓库暂未提供。

### 9.5 接入真实 ROS

```python
# jetson/ros_bridge.py 里把 RosBackend 的实现补全即可
# 推荐做法：launch 自己的 ROS 节点，publish 到 /cmd_vel
class RosBackend:
    def __init__(self, cmd_vel_topic="/cmd_vel", wheelbase_m=0.30):
        import rospy
        from geometry_msgs.msg import Twist
        rospy.init_node("leading_line_driver")
        self._pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        ...

    def send(self, cmd):
        msg = Twist()
        msg.linear.x = cmd.linear_mps
        msg.angular.z = cmd.angular_radps
        self._pub.publish(msg)

# 然后在 config_jetson.yaml 里：
ros:
  backend: "ros"      # 由 mock 切到 ros
  wheelbase_m: 0.30
```

`publish_cmd_vel(steer_deg, speed)` 已处理阿克曼转向到角速度的转换（`w = v·tan(steer)/L`），
车体控制 ROS 节点只需要订阅 `/cmd_vel` 并把 `(v, w)` 翻译成电机 PWM / 线控信号。

### 9.6 调试技巧

- 跑通双端最简验证：先启 Jetson，再启 PC；PC 状态栏应显示"已连 Jetson"
- 看不到画面：先看 Jetson 日志的 UDP 丢包数；再在 PC 上 `tcpdump -i any udp port 9000`
- 切模式不生效：检查 PC 状态栏有没有"ACK: MODE"；没有说明命令没发出去（看连接状态）
- 算法只跑一次 main.py 也行：`python main.py --source tests/synth.png`，与 Jetson 端用同一份 `config.yaml`
