# leading_line — zonesion xcar (4WD 全向) ROS launch 包

把 `jetson/` 下的引导线算法（路径规划 + 箭头 + QR 识别）作为 ROS 节点跑，
订阅摄像头话题、发布 `/cmd_vel` 给 `mbot`（zonesion xcar）底盘。

## 适配平台

| 项 | 值 |
|---|---|
| 车 | zonesion xcar（4WD 全向 / Mecanum）|
| 串口 | `/dev/ttyXCar`, 115200 |
| 协议 | zonesion 自定义二进制帧 `[0x2B,0xA2,no,type,len,data...,checksum]` |
| 底盘 launch | `mbot/launch/serial-node-zonesion.launch`（内部启 `xcar_ros.py`）|
| 摄像头 launch | `mbot/launch/rs_camera.launch`（RealSense）→ `/camera/color/image_raw` |
| 遥控 launch | `mbot/launch/mbot_teleop.launch` |
| 底盘订阅话题 | `/cmd_vel` (`geometry_msgs/Twist`) |
| 底盘发布话题 | `/odom` `/xcar/arm_status` `/xcar/sonar[1-4]` `/xcar/sensors` |

## 文件结构

```
ros_pkgs/leading_line/
├── package.xml
├── CMakeLists.txt
├── README.md
├── launch/
│   ├── leading_line.launch            # 仅起 leading_line 节点
│   ├── leading_line_with_car.launch   # xcar + realsense + leading_line
│   └── leading_line_teleop.launch     # 上面 + mbot 键盘遥控
├── scripts/
│   └── leading_line_node.py           # ROS 节点：image → algo → RosBridge → /cmd_vel
└── config/
    └── params.yaml                    # ROS 节点参数
```

## 运动学：阿克曼风格 × 全向底盘

xcar 是 4WD 全向底盘（`/cmd_vel` 有 `linear.x`, `linear.y`, `angular.z` 三个自由度），
而我们的引导线算法只输出 `(steer_deg, speed)`（前后方向为主）。

转换方式：

```
linear.x     = speed · max_speed_mps
linear.y     = 0                           # 全向平台才有意义；默认 0
angular.z    = v · tan(steer_rad) / L      # 阿克曼近似，等效在 4WD 上开阿克曼车
```

实现在 `jetson/ros_bridge.py:RosBridge.publish_cmd_vel` —— 两端（UDP pipeline
和 ROS node）共用同一份换算，不会出分歧。

## 前置条件

工作空间 `~/ros_ws/src/` 至少要包含：

```
turn_on_wheeltec_robot/   # 旧阿克曼平台（如果同时用）
wheeltec_robot_rc/         # 旧遥控（如果同时用）
mbot/                      # ★ 新 xcar 平台（zonesion）
realsense2_camera/         # RealSense 驱动
usb_cam/                   # 选：USB 摄像头
leading_line/              # ★ 本包
```

并且 `~/ros_ws/src/leading_line/` 下要把项目根的 `jetson/` 和 `protocol/` 软链进来：

```bash
ln -s /path/to/Leading_Line/jetson ~/ros_ws/src/leading_line/jetson
ln -s /path/to/Leading_Line/protocol ~/ros_ws/src/leading_line/protocol
```

## 编译

```bash
cd ~/ros_ws
catkin_make
source devel/setup.bash
```

## 运行

### 1. 单独跑 leading_line（外部已有 xcar+摄像头）
```bash
roslaunch leading_line leading_line.launch
```

### 2. 完整启动：xcar + realsense + 算法（最常用）
```bash
roslaunch leading_line leading_line_with_car.launch
```

### 3. 加键盘遥控（手动 + 自动并存）
```bash
roslaunch leading_line leading_line_teleop.launch
# 另开终端，按 i/j/k/l/m/,. 控制（mbot 标准 teleop）
```

### 4. 不用 realsense，用 USB 摄像头
```bash
roslaunch leading_line leading_line_with_car.launch \
    use_realsense:=false \
    image_topic:=/usb_cam/image_raw
# 自行确认 usb_cam 节点在 launch 之前或同时启动
```

### 5. 调紧急停车距离
```bash
roslaunch leading_line leading_line_with_car.launch sonar_stop_m:=0.50
```

## ROS 话题 / 服务

### 订阅
| 话题 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `<image_topic>` | `sensor_msgs/Image` | `/camera/color/image_raw` | 摄像头原始 BGR 帧 |
| `/xcar/sonar1..4` | `sensor_msgs/Range` | （可选）| 紧急停车距离触发 |

### 发布
| 话题 | 类型 | 说明 |
|---|---|---|
| `<cmd_vel_topic>` | `geometry_msgs/Twist` | xcar 全向 `(linear.x, linear.y, angular.z)` |

### 服务
| 服务 | 类型 | 说明 |
|---|---|---|
| `~start_stop` | `std_srvs/SetBool` | True→RUNNING, False→STOPPED |
| `~set_mode` | `std_srvs/SetBool` | `data="blue_path"` / `"green_path"` / `"test"` |

### 远程调用的例子
```bash
# 启动寻路
rosservice call /leading_line/start_stop "data: true"
# 停车
rosservice call /leading_line/start_stop "data: false"
# 切到绿色路径
rosservice call /leading_line/set_mode "data: 'green_path'"
```

## ROS 参数

通过 `~xxx` 私有参数配置（详见 `config/params.yaml`）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `config_path` | `jetson/config.yaml` | 算法 YAML（要包含 `modes.*`） |
| `image_topic` | `/camera/color/image_raw` | 摄像头话题 |
| `cmd_vel_topic` | `/cmd_vel` | 输出话题 |
| `wheelbase_m` | `0.30` | 等效阿克曼轴距 |
| `max_speed` | `0.30` | 最大线速度（m/s）|
| `min_speed` | `0.05` | 最低有效速度（低于发 0）|
| `publish_hz` | `30.0` | /cmd_vel 频率上限 |
| `default_mode` | `blue_path` | 启动模式 |
| `image_timeout_s` | `1.0` | 摄像头断流多久停车 |
| `sonar_stop_m` | `0.30` | 紧急停车距离阈值（<0 禁用）|
| `sonar_topics` | `["/xcar/sonar1..4"]` | 紧急停车用的超声话题（JSON 字符串）|

## 安全设计

| 触发 | 反应 |
|---|---|
| 状态 ≠ RUNNING | 持续发 0 |
| 摄像头断流 > `image_timeout_s` | 发 0 |
| 任一 `/xcar/sonarN` < `sonar_stop_m` | 立刻发 0（log warn） |
| `/xcar/sensors` bat < 6.5V | 强制停车（log error） |
| `/xcar/sensors` bat < 7.0V | 打 warn 日志 |
| 节点关闭（Ctrl+C / SIGTERM） | 最后发一次 0 |
| xcar 底盘内置 1s 无 /cmd_vel | 自动停车（兜底）|

安全实现统一在 [`jetson/ros_bridge.py:jeston/ros_bridge.py:200-235`](../../../jetson/ros_bridge.py)，
被 ROS 节点和 UDP pipeline 共享。

## 调参建议

| 现象 | 调整 |
|---|---|
| 转弯太急甩尾 | 减小 `controller.max_steer_deg` / `wheelbase_m` 增大 |
| 直线轻微蛇形 | 增大 `temporal.alpha`（更平滑）|
| 路径找不到 | 检查 `config.yaml` 的 HSV；切 `test` 模式看视觉 |
| 命令发不出去 | `rostopic echo /cmd_vel` 看有没有数据 |
| 启动报 "找不到 config" | 改 `~config_path` 为绝对路径，或软链项目根 |
| 频繁紧急停车 | 调大 `sonar_stop_m`，或排查超声数据 |

## 跨平台兼容

| 平台 | 兼容性 |
|---|---|
| zonesion xcar (4WD) | ✅ 完全适配（默认）|
| 阿克曼底盘（turn_on_wheeltec_robot）| ✅ 仍可用：launch 改成 `turn_on_wheeltec_robot/launch/include/base_serial.launch`，`linear.y=0` 自动忽略 |
| 纯差速底盘（带 Twist 支持）| ✅ `linear.y=0` 忽略即可 |
| ROS 2（rclpy）| ⚠️ 当前节点用 rospy；要迁到 ROS 2 需要重写 import 和 spin |

## 相关文件

- 算法核心：[`jetson/algo/`](../../../jetson/algo/)（颜色分割 / 路径规划 / 控制 / 可视化）
- Override 层：[`jetson/overrides.py`](../../../jetson/overrides.py)（箭头 + QR）
- ROS 桥：[`jetson/ros_bridge.py`](../../../jetson/ros_bridge.py)（运动学 + /cmd_vel + 安全）
- xcar 源码：[`src/mbot/scripts/xcar_ros.py`](../../../src/mbot/scripts/xcar_ros.py)（订阅 /cmd_vel 转发到串口）
- xcar 协议：[`src/mbot/scripts/xcar_data.py`](../../../src/mbot/scripts/xcar_data.py)（串口帧解析）