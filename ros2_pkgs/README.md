# ros2_pkgs — ROS2 Humble 视觉感知 + 底盘驱动

`ros2_pkgs/` 是把项目里"小车端运动控制"从 ROS1 + Python 桥接迁移到 **ROS2 Humble + 阿克曼架构** 的成果。

## 包

| 包 | 语言 | 类型 | 作用 |
|---|---|---|---|
| `leading_line_chassis/` | C++ | ament_cmake | 底盘驱动：订阅 `/ackermann_cmd`，按阿克曼运动学换算后用 zonesion 0x2B-A2 协议打 `/dev/ttyXCar`；回放 `/odom` `/imu` `/battery_voltage` `/xcar/sonar[1-4]` `/xcar/sensors` |
| `leading_line/` | Python | ament_python | 视觉感知 + PC 监控桥：订阅 `/image` → `leading_line.algo` 系列算法 → `/ackermann_cmd`；附带 `pc_monitor_bridge` 桥接原 PC Qt 监控 |

## 目录树

```
ros2_pkgs/
├── README.md                              # 本文件
├── build.sh                               # 一键 colcon build + test
├── check_topics.sh                        # 启动后跑的话题/服务/参数自检
├── mock_serial.py                         # 桌面 mock 串口：注入假 zonesion 帧
├── leading_line_chassis/
│   ├── package.xml                        # format=3; ackermann_msgs, libserial-dev
│   ├── CMakeLists.txt                     # ament_cmake; C++17
│   ├── src/
│   │   ├── chassis_node.hpp/.cpp          # rclcpp Node；50Hz 主循环；ackermann→twist→serial
│   │   ├── serial_driver.hpp/.cpp         # libserial 包装 + 帧流式重组
│   │   ├── protocol.hpp/.cpp              # zonesion 0x2B-A2 协议（Odom/Sonar/Sensors/Speed）
│   │   ├── ackermann_kinematics.hpp/.cpp  # 阿克曼换算
│   │   └── chassis_orientation.hpp        # yaw→quaternion
│   ├── launch/
│   │   ├── chassis.launch.py              # 底盘节点
│   │   └── chassis_with_robot_description.launch.py
│   ├── urdf/
│   │   ├── mini_akm_robot.urdf            # 移植自 turn_on_wheeltec_robot
│   │   └── senior_akm_robot.urdf
│   ├── config/chassis.yaml
│   ├── test/test_chassis_protocol.cpp     # GTest：encode/decode/BCC
│   └── README.md
└── leading_line/
    ├── package.xml
    ├── setup.py / setup.cfg
    ├── resource/leading_line
    ├── leading_line/
    │   ├── __init__.py
    │   ├── ackermann_bridge.py            # (steer_deg, speed) → AckermannDriveStamped
    │   ├── perception_pipeline.py         # 算法流水线（无 ROS）
    │   ├── perception_node.py             # rclpy Node；/image → /ackermann_cmd
    │   ├── pc_monitor_bridge.py           # PC TCP/UDP ↔ ROS 桥
    │   ├── pipeline.py                    # 主流水线：摄像头 → 算法 → override → 渲染 → UDP 推流 + 响应命令
    │   ├── vehicle_pipeline.py            # ROS 节点包装 pipeline（PC 监控模式）
    │   ├── overrides.py                   # 箭头/QR override 层
    │   ├── algo/                          # 引导线算法核心（HSV 分割 / 路径规划 / 控制 / 可视化）
    │   ├── recognition/                   # 覆盖层识别（arrow_detector + qr/{decoder,state_machine,policy}）
    │   └── comm/                          # TCP 命令接收 + UDP 视频发送（pc_monitor_bridge 复用）
    ├── launch/
    │   ├── perception.launch.py           # 仅视觉
    │   ├── perception_with_camera.launch.py # 视觉 + 摄像头
    │   ├── pc_monitor.launch.py           # 仅 PC 桥
    │   └── full_stack.launch.py           # 底盘 + 摄像头 + 视觉 + PC
    ├── config/perception_params.yaml
    ├── test/
    │   ├── test_ackermann_bridge.py       # MockBridge 单测
    │   └── test_perception_pipeline.py    # perception_pipeline 单测
    └── README.md
```

## 一键构建

```bash
# 1) 工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ln -s /path/to/Leading_Line/ros2_pkgs/leading_line_chassis .
ln -s /path/to/Leading_Line/ros2_pkgs/leading_line .
# 单一来源：`leading_line` 包的 `algo/`/`recognition/`/`comm/`/`pipeline.py`
# 自包含，colcon build 之后 `from leading_line.X import Y` 即可。

# 2) apt 依赖
sudo apt install -y \
    ros-humble-ackermann-msgs \
    ros-humble-cv-bridge \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-robot-state-publisher \
    ros-humble-usb-cam \
    libserial-dev \
    python3-pyzbar

# 3) 编译 + 测
cd ~/ros2_ws
./src/Leading_Line/ros2_pkgs/build.sh
# 或手动：
#   rosdep install --from-paths src --ignore-src -r -y
#   colcon build --packages-select leading_line_chassis leading_line
#   colcon test --packages-select leading_line_chassis leading_line
```

## 一键运行

```bash
source ~/ros2_ws/install/setup.bash

# 1) 仅底盘（外部已有 ackermann 来源）
ros2 launch leading_line_chassis chassis.launch.py

# 2) 仅视觉（外部已有底盘 + 摄像头）
ros2 launch leading_line perception.launch.py

# 3) 全栈：底盘 + 摄像头 + 视觉
ros2 launch leading_line full_stack.launch.py \
    usart_port_name:=/dev/ttyXCar \
    camera_backend:=usb_cam \
    use_pc_monitor:=false

# 4) 桌面 mock 联调
socat -d -d pty,raw,echo=0 pty,raw,echo=0 &
# 假设映射 /dev/ttyVCar0 ↔ /dev/ttyVCar1
ros2 launch leading_line_chassis chassis.launch.py usart_port_name:=/dev/ttyVCar0 &
python3 mock_serial.py /dev/ttyVCar1
ros2 launch leading_line perception.launch.py
./check_topics.sh
```

## 关键设计

| 决策 | 选择 | 原因 |
|---|---|---|
| 消息类型 | `ackermann_msgs/AckermannDriveStamped` | ROS 官方标准；含 `steering_angle` + `speed` 物理量 |
| 字节协议 | zonesion 0x2B-A2 | 当前实际硬件（zonesion xcar 4WD MCU） |
| 阿克曼换算位置 | C++ 底盘节点内部 | 算法节点只发 (steer, speed)；底盘负责物理换算 |
| 视觉算法 | 复用 `leading_line.algo` `leading_line.recognition` `leading_line.overrides` | 避免重写数千行 cv2/QR/箭头 |
| PC ↔ 小车 协议 | TCP 9001 + UDP 9000（`protocol/` 字节层） | 不变；Qt UI 不动 |
| 通信 QoS | `/ackermann_cmd` RELIABLE；`/odom` `/imu` BEST_EFFORT VOLATILE | REP-145 |
| safety | watchdog 500ms；image timeout；sonar 急停；低电 cut-off；`on_shutdown` 发 0 | 跨节点统一 |

## 与旧 ROS1 栈的差异（已删）

| 旧（ROS1, 已删） | 新（ROS2 Humble） |
|---|---|
| `ros_pkgs/leading_line/scripts/node.py` | `leading_line/perception_node.py`（rclpy）|
| `ros_pkgs/leading_line/scripts/xcar/xcar_ros.py`（Python 串口桥）| `leading_line_chassis/chassis_node`（C++ 串口驱动）|
| `ros_pkgs/leading_line/scripts/xcar/xcar_protocol.py`（Python 协议）| `leading_line_chassis/src/protocol.cpp`（C++ 协议）|
| `ros_pkgs/leading_line/scripts/xcar/mbot_teleop.py`（Python 键盘）| 不提供（用户选择不含）|
| `turn_on_wheeltec_robot/`（ROS1 参考源码）| 已删；仅学习其类结构风格 |
| `wheeltec_robot_rc/`（ROS1 手柄）| 已删 |
| `vehicle/ros_bridge.py`（Python rospy 桥）| 已删；`MockBridge` 搬至 `leading_line/ackermann_bridge.py` |
| `vehicle/main.py`（CLI 入口）| 已删；改成 rclpy 节点 `perception_node` / `vehicle_pipeline` |
| `vehicle/pipeline.py` + `vehicle/{algo,recognition,overrides,comm}/`（纯算法库）| 已删并并入 `leading_line/{pipeline.py,algo/,recognition/,overrides.py,comm/}`——**单一来源** |
| `/cmd_vel` (`Twist`) | `/ackermann_cmd` (`AckermannDriveStamped`) |

## 端到端验证

### 1) 编译 + 单元测
```bash
colcon build --packages-select leading_line_chassis leading_line
colcon test --packages-select leading_line_chassis leading_line
```
预期：GTest `test_chassis_protocol` 全过（encode/decode/BCC/partial frame）；pytest `test_ackermann_bridge` + `test_perception_pipeline` 全过；`tests/unit/test_comm.py` 仍 pass（PC ↔ vehicle 协议层未动）。

### 2) 桌面联调（mock 串口）
按上面"一键运行 §4"。

### 3) 真车（Jetson + zonesion MCU + PC UI）
```bash
# 车辆
ros2 launch leading_line full_stack.launch.py \
    usart_port_name:=/dev/ttyXCar \
    use_pc_monitor:=true \
    pc_ip:=192.168.1.100
# PC
python pc/main.py --config config_pc.yaml
```

### 4) 关键断言（自检脚本）
```bash
./check_topics.sh
```
预期：所有 ✓，包括 `/ackermann_cmd` 存在、chassis 参数正常、`/leading_line/start_stop` 可调、TF 树 `odom → base_footprint` 在线。
