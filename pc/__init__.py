# -*- coding: utf-8 -*-
"""PC 端：负责 Qt UI + 接收 Jetson 推流 + TCP 下发命令。

子模块：
- comm.video_receiver ：UDP 收 Jetson 编码好的视频帧
- comm.command_sender ：TCP 连 Jetson，发模式/开始/结束
- ui.main_window     ：Qt 主窗口（左视频 / 右菜单 / 底部状态）
"""
