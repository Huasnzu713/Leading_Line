# leading_line — Wheeltec 阿克曼小车 ROS launch 包

把 `jetson/` 下的引导线算法（路径规划 + 箭头 + QR 识别）作为 ROS 节点跑，
订阅摄像头话题、发布 `/cmd_vel` 给 `turn_on_wheeltec_robot` 底盘。

## 文件结构

```
ros_pkgs/leading_line/
├── package.xml
├── CMakeLists.txt
├── README.md
├── launch/
│   ├── leading_line.launch            # 只起 leading_line 节点（外部已起 car+cam）
│   ├── leading_line_with_car.launch   # car + camera + leading_line
│   └── leading_line_teleop.launch     # 上面 + 键盘遥控（手动优先）
├── scripts/
│   └── leading_line_node.py           # ROS 节点：image → algo → /cmd_vel
└── config/
    └── params.yaml                    # 默认 ROS 参数
```

## 前置条件

工作空间至少要包含这些包（其中前两个 Wheeltec 已有，第三个是新加的）：

```
~/ros_ws/src/
├── turn_on_wheeltec_robot/      # 底盘
├── wheeltec_robot_rc/            # 遥控（可选）
└── leading_line/                 # 本包
└── (项目根目录)                       # ← 重要：把 jetson/ 目录也带进工作空间
```

> **重要**：本包要 import `jetson.algo / jetson.recognition / protocol`，
> 这些代码在 **项目根** 的 `jetson/` 等子包，不是 ROS 包。
> 所以工作空间的 `src/` 下需要能直接访问到项目根（或者把 `jetson/ protocol/ main.py`
> 软链 / 拷贝进 `src/leading_line/` 下）。
>
> 最简做法：在 `~/ros_ws/src/` 下 `ln -s /path/to/Leading_Line/jetson leading_line/jetson`
> 和 `ln -s /path/to/Leading_Line/protocol leading_line/protocol`，
> 编译后 PYTHONPATH 自动包含 `src/` 下所有目录。

## 编译

```bash
cd ~/ros_ws
catkin_make
source devel/setup.bash
```

## 运行

### 1. 单独跑 leading_line（外部已起 car + 摄像头）
```bash
roslaunch leading_line leading_line.launch \
    image_topic:=/usb_cam/image_raw \
    default_mode:=blue_path
```

### 2. 完整启动：阿克曼底盘 + 摄像头 + 算法
```bash
# mini_akm 是常见的 Wheeltec 阿克曼车型；其它有 senior_akm/top_akm_bs/top_akm_dl
roslaunch leading_line leading_line_with_car.launch \
    car_mode:=mini_akm \
    camera_mode:=RgbCam \
    default_mode:=blue_path
```

### 3. 加键盘遥控（手动 + 自动并存）
```bash
roslaunch leading_line leading_line_teleop.launch \
    car_mode:=mini_akm \
    camera_mode:=RgbCam
# 另开终端，会进入键盘控制台
# i / , ：前进 / 后退
# j / l ：左转 / 右转
# k ：停车
# q / Ctrl+C ：退出
```

## ROS 话题 / 服务

### 订阅
| 话题 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `<image_topic>` | `sensor_msgs/Image` | `/usb_cam/image_raw` | 摄像头原始 BGR 帧 |

### 发布
| 话题 | 类型 | 说明 |
|---|---|---|
| `<cmd_vel_topic>` | `geometry_msgs/Twist` | 阿克曼 `(linear.x, angular.z)` |

### 服务
| 服务 | 类型 | 说明 |
|---|---|---|
| `~start_stop` | `std_srvs/SetBool` | `req.data=True` → RUNNING；`False` → STOPPED |
| `~set_mode` | `std_srvs/SetBool` | `req.data` = 模式名（`blue_path` / `green_path` / `test`）|

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

通过 `~config_path` 等私有参数配置（详见 `config/params.yaml`）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `config_path` | `jetson/config.yaml` | 算法 YAML（要包含 modes.* 段）|
| `image_topic` | `/usb_cam/image_raw` | 摄像头话题 |
| `cmd_vel_topic` | `/cmd_vel` | 输出话题 |
| `wheelbase_m` | `0.30` | 阿克曼轴距 |
| `max_speed` | `0.30` | 限速 |
| `min_speed` | `0.05` | 最低有效速度（低于发 0）|
| `publish_hz` | `30.0` | /cmd_vel 频率上限 |
| `default_mode` | `blue_path` | 启动模式 |
| `image_timeout_s` | `1.0` | 摄像头断流多久停车 |

命令行覆盖示例：
```bash
roslaunch leading_line leading_line.launch \
    image_topic:=/camera/rgb/image_raw \
    default_mode:=green_path \
    wheelbase_m:=0.32
```

## 安全设计

- **状态机**：`IDLE` / `RUNNING` / `STOPPED`；非 RUNNING 时持续发 0 速度
- **摄像头断流**：超过 `image_timeout_s` 没新帧就发 0
- **底盘安全**：`turn_on_wheeltec_robot` 内置 1s 内无 /cmd_vel 自动停车
- **手动覆盖**：键盘遥控（teleop.launch）和算法并发，按键期间覆盖算法输出

## 阿克曼运动学

`/cmd_vel` 的 `linear.x` 是前进速度（m/s），`angular.z` 是 yaw 角速度（rad/s），
底盘（`turn_on_wheeltec_robot`）内部把 `(v, ω)` 转换为舵机角度 + 轮速。

本节点内部换算：
```
v = clip(speed, 0, max_speed)
w = v · tan(steer_rad) / wheelbase
```

## 调参建议

| 现象 | 调整 |
|---|---|
| 转弯太急甩尾 | 减小 `controller.max_steer_deg` / `publish_hz` |
| 直线轻微蛇形 | 增大 `temporal.alpha`（更平滑）|
| 路径找不到 | 检查 `config.yaml` 的 HSV 范围，或切 `test` 模式看视觉 |
| 命令发不出去 | `rostopic echo /cmd_vel` 看有没有数据 |
| 节点启动报 "找不到 config" | 改 `~config_path` 为绝对路径，或软链项目根 |
