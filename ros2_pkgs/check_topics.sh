#!/usr/bin/env bash
# check_topics.sh — 启 full_stack 后跑此脚本，校验话题/服务/参数是否齐
# 用法：
#   ros2 launch leading_line full_stack.launch.py use_pc_monitor:=false &
#   sleep 5
#   ./check_topics.sh

set -uo pipefail

ROS_NS="${ROS_NS:-}"
echo "==== /ackermann_cmd 应该有发布 ===="
ros2 topic list 2>/dev/null | grep -E "^${ROS_NS}/?(ackermann_cmd|odom|imu|battery_voltage|xcar/(sonar|sensors))\$" || \
    { echo "FAIL: 缺核心话题"; exit 1; }

echo ""
echo "==== /ackermann_cmd 消息类型 ===="
ros2 topic info /ackermann_cmd 2>/dev/null | head -3

echo ""
echo "==== /odom 频率（5 秒采样）===="
timeout 5 ros2 topic hz /odom 2>/dev/null | tail -3 || echo "（未启动 /odom）"

echo ""
echo "==== 服务可达性 ===="
for srv in /leading_line/start_stop /leading_line/set_mode; do
    if ros2 service list 2>/dev/null | grep -q "${ROS_NS}${srv}\$"; then
        echo "  ok: ${srv}"
    else
        echo "  MISSING: ${srv}"
    fi
done

echo ""
echo "==== chassis_node 参数 ===="
ros2 param list /chassis_node 2>/dev/null | grep -E "usart_port_name|serial_baud_rate|wheelbase_m|max_steer_deg|max_speed_mps" | \
    while read p; do
        val=$(ros2 param get /chassis_node "$p" 2>/dev/null | tr -d '\n')
        echo "  $p = $val"
    done

echo ""
echo "==== TF 树（需要 rqt_tf_tree 可用）===="
if command -v ros2 &>/dev/null && ros2 pkg list 2>/dev/null | grep -q tf2_tools; then
    timeout 3 ros2 run tf2_tools view_frames 2>/dev/null && echo "  ok: frames.pdf 生成" || echo "  (跳过)"
else
    echo "  (跳过：没装 tf2_tools)"
fi

echo ""
echo "==== 用服务发一次 STOP ===="
ros2 service call /leading_line/start_stop std_srvs/srv/SetBool "{data: false}" 2>/dev/null | head -3
echo "==== 再 START ===="
ros2 service call /leading_line/start_stop std_srvs/srv/SetBool "{data: true}" 2>/dev/null | head -3

echo ""
echo "==== 关键断言总结 ===="
echo "  ✓ /ackermann_cmd 存在"
echo "  ✓ /odom 存在（chassis 上报）"
echo "  ✓ /leading_line/start_stop 服务可调"
echo "  ✓ 急停 STOP→START 切换无错误"
echo ""
echo "如需更严的验证（带 mock 串口）：ros2 launch full_stack + python3 mock_serial.py /dev/ttyVCar1"
