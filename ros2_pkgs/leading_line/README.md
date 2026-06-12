# leading_line

ROS2 Humble (Python rclpy) 视觉感知 + PC 监控桥。订阅摄像头图像，跑 `vehicle.algo` 系列算法（颜色分割→路径规划→控制→箭头/QR override），发布 `ackermann_msgs/AckermannDriveStamped` 到 `/ackermann_cmd`，供下游 `leading_line_chassis` 底盘驱动使用。

不重写任何 cv2/QR/箭头识别代码——直接复用项目根的 `vehicle/algo/` `vehicle/recognition/` `vehicle/overrides.py`。

## 节点

| 节点 | 类型 | 作用 |
|---|---|---|
| `perception_node` | rclpy | /image → algo → /ackermann_cmd；提供 ~/start_stop、~/set_mode 服务 |
| `pc_monitor_bridge` | rclpy | 桥接 PC TCP/UDP 监控；订阅 /debug/annotated_image → JPEG → UDP 推给 PC；接收 PC START/STOP/MODE 命令转 ROS 服务 |
| `vehicle_pipeline` | rclpy | 保留原 PC 监控模式（cv2 摄像头 + UDP 推流 + TCP 命令）的 ROS 包装；调用 vehicle.pipeline.Pipeline + ackermann bridge |

## 话题 / 服务

| 方向 | 名称 | 类型 | 说明 |
|---|---|---|---|
| sub | `<image_topic>` | `sensor_msgs/Image` | 默认 `/usb_cam/image_raw` |
| sub | `/xcar/sonar[1-4]` | `sensor_msgs/Range` | 急停距离触发 |
| sub | `/xcar/sensors` | `std_msgs/Int32MultiArray` | 电池状态 |
| pub | `/ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` | 唯一运动输出 |
| pub | `/debug/annotated_image` | `sensor_msgs/Image` | 可选调试标注图 |
| srv | `~/start_stop` | `std_srvs/SetBool` | True→RUNNING, False→STOPPED |
| srv | `~/set_mode` | `std_srvs/SetBool` | data="blue_path"/"green_path"/"test" |

## 参数（perception_node）

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `config_path` | string | `config.yaml` | 算法 YAML 路径（要含 `modes.*`） |
| `image_topic` | string | `/usb_cam/image_raw` | 输入图像话题 |
| `ackermann_topic` | string | `/ackermann_cmd` | 输出话题 |
| `wheelbase_m` | double | 0.30 | 阿克曼等效轴距（与底盘节点一致） |
| `max_steer_deg` | double | 30.0 | 最大前轮转角 |
| `max_speed_mps` | double | 0.5 | 最大前向速度 |
| `min_speed_mps` | double | 0.05 | 最低有效速度 |
| `publish_hz` | double | 30.0 | tick 频率 |
| `default_mode` | string | `blue_path` | 启动模式 |
| `image_timeout_s` | double | 1.0 | 图像断流多久停车 |
| `publish_debug_image` | bool | false | 是否发 `/debug/annotated_image` |
| `sonar_stop_m` | double | 0.30 | 急停距离阈值（<0 禁用） |
| `bat_warn_0p1V` | int | 70 | 告警电压（0.1V） |
| `bat_critical_0p1V` | int | 65 | 强制停车电压（0.1V） |

## 构建

```bash
mkdir -p ~/ros2_ws/src
ln -s /path/to/Leading_Line/ros2_pkgs/leading_line_chassis ~/ros2_ws/src/
ln -s /path/to/Leading_Line/ros2_pkgs/leading_line ~/ros2_ws/src/
# 让 leading_line 找得到 project 根的 vehicle/（cv2 算法）
ln -s /path/to/Leading_Line ~/ros2_ws/src/leading_line_project

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select leading_line leading_line_chassis
colcon test --packages-select leading_line
```

## 运行

```bash
# 仅视觉（外部已有底盘 + 摄像头）
ros2 launch leading_line perception.launch.py

# 视觉 + 摄像头（USB 摄像头）
ros2 launch leading_line perception_with_camera.launch.py \
    camera_backend:=usb_cam image_topic:=/usb_cam/image_raw

# 视觉 + RealSense
ros2 launch leading_line perception_with_camera.launch.py \
    camera_backend:=realsense image_topic:=/camera/color/image_raw

# 仅 PC 监控桥（需要 perception_node 在运行，否则服务不可达）
ros2 launch leading_line pc_monitor.launch.py

# 全栈：底盘 + 摄像头 + 视觉 + PC 监控
ros2 launch leading_line full_stack.launch.py \
    usart_port_name:=/dev/ttyXCar \
    use_pc_monitor:=true \
    pc_ip:=192.168.1.100
```

## 远程控制示例

```bash
# 启动寻路
ros2 service call /perception_node/start_stop std_srvs/srv/SetBool "{data: true}"
# 停车
ros2 service call /perception_node/start_stop std_srvs/srv/SetBool "{data: false}"
# 查看 /ackermann_cmd
ros2 topic echo /ackermann_cmd
```

## 安全设计

| 触发 | 反应 |
|---|---|
| 状态 ≠ RUNNING | 持续发 0 |
| 图像断流 > `image_timeout_s` | 发 0 |
| 任一 `/xcar/sonarN` < `sonar_stop_m` | 立刻发 0（log warn） |
| `/xcar/sensors` bat < `bat_critical_0p1V` (6.5V) | 强制停车（log error） |
| `/xcar/sensors` bat < `bat_warn_0p1V` (7.0V) | 打 warn 日志 |
| 节点关闭（Ctrl+C / SIGTERM） | 最后发一次 0 |

## 关键模块

- `perception_pipeline.py` — 算法流水线（颜色分割→路径规划→控制→override），无 ROS 依赖
- `perception_node.py` — ROS 节点包装 perception_pipeline；加急停/电池/图像 timeout 守护
- `ackermann_bridge.py` — `(steer_deg, speed) → AckermannDriveStamped`；`MockBridge` 给单测用
- `pc_monitor_bridge.py` — PC ↔ ROS TCP/UDP 桥；保留 `CommandReceiver` + `VideoSender`（来自 `vehicle/comm/`）
- `vehicle_pipeline.py` — 原 PC 监控模式（cv2 摄像头 + TCP/UDP）的 ROS 包装，调用 `vehicle.pipeline.Pipeline` + ackermann bridge
- `overrides.py` — 箭头/QR 覆盖层（拷贝自 `vehicle/overrides.py`）
- `algo/` `recognition/` `comm/` — 全部拷贝自 `vehicle/` 同名子包（自包含）

## 与旧 `ros_pkgs/leading_line` 的差异

| 旧（ROS1, 已删） | 新（ROS2 Humble） |
|---|---|
| `node.py` 调 `rospy.init_node` + `RosBridge(backend="ros")` | `perception_node.py` 调 `rclpy.init` + `ackermann_bridge.make_rclpy_bridge` |
| `/cmd_vel` (`Twist`) | `/ackermann_cmd` (`AckermannDriveStamped`) |
| `xcar_ros.py` 把 `Twist` 转 zonesion 0x81 帧 | `leading_line_chassis/chassis_node` 做 ackermann→twist→0x81 |
| `mbot_teleop.py` 键盘发布 `/cmd_vel` | 不提供（用户选择不包含） |
| 多个 launch 散落在 `ros_pkgs/leading_line/launch/` | 统一在 `ros2_pkgs/leading_line/launch/` |
| xcar 串口驱动 Python (`pyserial`) | C++ (`libserial`)，无 Python 操控版本 |
| 视觉算法软链 `vehicle/` 到 catkin 包 | ament_python 自包含（`setup.py` 列子包） |
| `pc_monitor.launch` 串口桥 + cv2 流水线 | `pc_monitor.launch.py` 单独桥 + `vehicle_pipeline` 单独节点 |
