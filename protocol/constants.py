# -*- coding: utf-8 -*-
"""双端通信常量。

- PC 端默认 IP/端口：在局域网里 PC 通常是 192.168.x.x；可在 config 里覆盖
- 模式名常量：PC UI 与 Jetson pipeline 共享，避免拼写漂移
- 状态常量：Jetson 回报给小车的运行状态
"""
from __future__ import annotations

# 默认网络端点（局域网示例值，可在 config.yaml 里覆盖）
DEFAULT_PC_IP: str = "192.168.1.100"
DEFAULT_VIDEO_PORT: int = 9000     # Jetson → PC 的 UDP 视频端口
DEFAULT_CMD_PORT: int = 9001       # PC → Jetson 的 TCP 命令端口

# 模式标识（与 config.yaml 的 modes.*.name 严格对应）
MODE_BLUE: str = "blue_path"       # 蓝色路径模式（保持现状）
MODE_GREEN: str = "green_path"     # 绿色路径模式
MODE_TEST: str = "test"            # 测试模式（灰/白）

ALL_MODES: tuple[str, ...] = (MODE_BLUE, MODE_GREEN, MODE_TEST)

# Jetson 端状态机（与 UI 上"开始/结束"按钮对应）
STATE_IDLE: str = "idle"           # 默认，不寻路
STATE_RUNNING: str = "running"     # 收到 START，正在寻路
STATE_STOPPED: str = "stopped"     # 收到 STOP，安全停车
STATE_ERROR: str = "error"         # 摄像头断开 / 算法异常
