"""Jetson 端：负责摄像头采集 + 算法执行 + UDP 推流 + TCP 接令 + ROS 控制。

子模块：
- comm.video_sender    ：UDP 发送编码后的视频帧给 PC
- comm.command_receiver：TCP 接收 PC 的模式切换/开始/结束命令
- pipeline             ：把摄像头 + 算法 + ROS 串起来的主循环
- ros_bridge           ：对外的 ROS 接口（速度/转向 → /cmd_vel 等）
"""
