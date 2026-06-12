# leading_line_chassis

ROS2 Humble (C++) 阿克曼底盘驱动包。订阅 `/ackermann_cmd` (`ackermann_msgs/AckermannDriveStamped`)，按阿克曼自行车模型换算为 `(vx, vy=0, wz)` 后用 **zonesion 0x2B-A2 私有协议** 打到 `/dev/ttyXCar` 串口，同时把 MCU 上报回放为 `/odom` `/imu` `/battery_voltage` `/xcar/sonar[1-4]` `/xcar/sensors`，并广播 `odom → base_footprint` TF。

> 结构风格移植自 `turn_on_wheeltec_robot/`（C++ ROS1 参考）；字节协议移植自原 `vehicle/ros_bridge.py` 联动的 zonesion xcar `xcar_protocol.py`（已删）。

## 话题 / 服务

| 方向 | 名称 | 类型 | QoS | 说明 |
|---|---|---|---|---|
| sub | `/ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` | RELIABLE | 唯一运动输入 |
| pub | `/odom` | `nav_msgs/Odometry` | BEST_EFFORT | wheel odometry |
| pub | `/imu` | `sensor_msgs/Imu` | BEST_EFFORT | 角速度来自 odom（zonesion 无 IMU） |
| pub | `/battery_voltage` | `std_msgs/Float32` | RELIABLE | 来自 `/xcar/sensors` bat×0.1V |
| pub | `/xcar/sensors` | `std_msgs/Int32MultiArray` | RELIABLE | 透传 zonesion 0x03 |
| pub | `/xcar/sonar[1-4]` | `sensor_msgs/Range` | BEST_EFFORT | 4 路超声距离 |
| pub | `/ackermann_cmd_echo` | `ackermann_msgs/AckermannDriveStamped` | RELIABLE | clamp 后的实际下发值 |
| TF | `odom → base_footprint` | — | — | 50 Hz |

## 参数

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `usart_port_name` | string | `/dev/ttyXCar` | 串口路径 |
| `serial_baud_rate` | int | 115200 | 9600/19200/38400/57600/115200/230400/460800 |
| `wheelbase_m` | double | 0.30 | 阿克曼等效轴距 |
| `max_steer_deg` | double | 30.0 | 最大前轮转角（>0, ≤60） |
| `max_speed_mps` | double | 0.5 | 最大前向速度 |
| `cmd_watchdog_ms` | int | 500 | 无 `/ackermann_cmd` 超过此时长发 0 |
| `loop_rate_hz` | int | 50 | 主循环频率 |
| `publish_tf` | bool | true | 是否发 TF |
| `odom_frame_id` | string | `odom` | |
| `robot_frame_id` | string | `base_footprint` | |
| `gyro_frame_id` | string | `gyro_link` | |

全部参数可在运行时 `ros2 param set /chassis_node ...` 调整，串口路径 / 波特率 / 阿克曼参数都会被校验。

## 构建

```bash
# 工作空间
mkdir -p ~/ros2_ws/src
ln -s /path/to/Leading_Line/ros2_pkgs/leading_line_chassis ~/ros2_ws/src/
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select leading_line_chassis
colcon test --packages-select leading_line_chassis
```

## 运行

```bash
# 仅底盘
ros2 launch leading_line_chassis chassis.launch.py

# 底盘 + URDF + TF
ros2 launch leading_line_chassis chassis_with_robot_description.launch.py \
    urdf_file:=/abs/path/to/senior_akm_robot.urdf

# 临时改串口
ros2 launch leading_line_chassis chassis.launch.py usart_port_name:=/dev/ttyACM0

# 手动发一条控制指令（用 ackermann_msgs）
ros2 topic pub /ackermann_cmd ackermann_msgs/msg/AckermannDriveStamped \
    "{drive: {steering_angle: 0.2, speed: 0.3}}"
```

## 安全设计

| 触发 | 反应 |
|---|---|
| `/ackermann_cmd` 缺失 > `cmd_watchdog_ms` | 自动发 0 |
| 节点关闭（`rclcpp::on_shutdown` 或析构）| 主动发 0 |
| 串口 read 异常 / 连续 BCC 失败 | 1Hz 日志 + 自动重连（1s 间隔）|
| 串口路径 / 波特率运行时修改 | 立即重新 `open()`，不需重启节点 |
| `steering_angle` / `speed` 入参 | 入参前 `clamp` 到 `±max_steer_rad` / `±max_speed_mps` |

## URDF

`urdf/mini_akm_robot.urdf`（小尺寸）和 `urdf/senior_akm_robot.urdf`（大尺寸）均含：

- `base_footprint → base_link`
- 4 个 `continuous` 轮子（`left/right_wheel_joint` + `left/right_front_joint`）
- `gyro_link`、`imu_link`、`front_axle_link` 占位
- `sonar1/2`、`laser1` 框架（与原 `leading_line_with_car.launch` 静态 TF 对齐）

zonesion 4WD 底盘的"前轮转向"在 MCU 内部完成（不能通过 ROS joint 控制），所以 4 个 wheel 关节都是 `continuous`，没有 `revolute` 转向关节。底盘几何中心（阿克曼转向延长线与后轴交点）即 `base_link` 原点。

## 协议（zonesion 0x2B-A2）

帧格式：

```
[0x2B 0xA2] [seq u8] [type u8] [len u8] [data len bytes] [BCC u8]
```

- `BCC = sum(header + seq + type + len + data) & 0xFF`
- 全部多字节整数字段大端

下行（ROS → MCU）类型：

| type | 含义 | data |
|---|---|---|
| 0x81 | 速度 | 6 字节：vx(int16 BE, mm/s), vy(int16 BE, 0), wz(int16 BE, 1/10000 rad/s) |
| 0x82 | 舵机（暂未实现） | — |
| 0x83 | 夹爪（暂未实现） | — |
| 0x84 | 舵机状态请求 | 空 |

上行（MCU → ROS）类型：

| type | 含义 | data |
|---|---|---|
| 0x01 | 里程计 | 16 字节：pos_x(i32 BE, mm), pos_y(i32 BE, mm), yaw(i32 BE, 1/10000 rad), vx(i16 BE, mm/s), vy(i16 BE, mm/s), wz(i16 BE, 1/10000 rad/s) |
| 0x02 | 4 路超声 | 8 字节：4×i16 BE (cm) |
| 0x03 | 环境传感器 | 13 字节：bat(u8, 0.1V), temp(i16 BE, 0.1°C), humi(i16 BE, %), pressure(i32 BE, 0.1Pa), light(i16 BE, lux), tvoc(i16 BE, ppm), smoke(i16 BE, ppm) |
| 0x04 | 舵机角度（透传忽略） | 6 字节 |

## 已知差异 vs 参考源码

1. **字节协议不同**：参考 `turn_on_wheeltec_robot` 是 24 字节 BCC 协议；本包是 zonesion 0x2B-A2 协议（适配 zonesion xcar 4WD MCU）。结构风格（class、destructor 主动发 0、on_set_parameters_callback）参照参考源码。
2. **IMU**：参考有 Mahony 滤波 + `/imu` 完整数据；本包无 IMU 输入（zonesion MCU 不上报），仅用 wheel odometry 填充 `angular_velocity.z`。
3. **ros2_control**：本包**没有**用 `ros2_control`。如果要对接支持 ros2_control 的底盘，需要新加 `ros2_control` 硬件接口 + URDF `<ros2_control>` 标签。

## 验证

参见仓库根 `README.md` 的"端到端验证"小节，含 mock 串口、真车两步。
