#!/usr/bin/env bash
# build.sh — colcon 编译脚本
# 假设：
#   - 工作空间 ~/ros2_ws/src/  下已放好 leading_line_chassis、leading_line
#   - 车辆端 Ubuntu 22.04 + ROS2 Humble（ros-humble-ackermann-msgs 等已装）

set -euo pipefail

WORKSPACE="${WORKSPACE:-$HOME/ros2_ws}"
PKGS="${PKGS:-leading_line_chassis leading_line}"

cd "$WORKSPACE"

echo "==== rosdep 安装系统依赖 ===="
sudo apt-get update
rosdep install --from-paths src --ignore-src -r -y \
    --rosdistro="${ROS_DISTRO:-humble}"

echo "==== colcon build（释放模式）===="
colcon build \
    --packages-select $PKGS \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --event-handlers console_direct+

echo "==== colcon test ===="
colcon test --packages-select $PKGS

echo "==== 提示 ===="
echo "source $WORKSPACE/install/setup.bash  后即可："
echo "  ros2 launch leading_line_chassis chassis.launch.py"
echo "  ros2 launch leading_line perception.launch.py"
echo "  ros2 launch leading_line full_stack.launch.py use_pc_monitor:=true"
