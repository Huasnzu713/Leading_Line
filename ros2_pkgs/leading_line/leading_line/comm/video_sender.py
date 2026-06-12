# -*- coding: utf-8 -*-
from __future__ import annotations

import logging
import socket
import time
from typing import Optional

from protocol import encode_video_frame

log = logging.getLogger(__name__)


class VideoSender:
    """UDP 视频发送端（Jetson 侧）。"""

    def __init__(self, pc_ip: str, pc_port: int) -> None:
        self.pc_ip = pc_ip
        self.pc_port = pc_port
        # SOCK_DGRAM = UDP；SO_SNDBUF 提到 256KB 减少大帧时的丢包
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 256 * 1024)
        self._sock.setblocking(False)
        self._seq = 0
        self._dropped = 0
        self._sent = 0

    def send(self, jpeg: bytes, ts_ms: Optional[int] = None) -> bool:
        """发一帧，返回 True 表示 sendto 成功，False 表示丢包（接收端未就绪等）。"""
        if not jpeg:
            return False
        if ts_ms is None:
            ts_ms = int(time.time() * 1000)
        packet = encode_video_frame(self._seq, ts_ms, jpeg)
        try:
            self._sock.sendto(packet, (self.pc_ip, self.pc_port))
            self._seq = (self._seq + 1) & 0xFF
            self._sent += 1
            return True
        except (BlockingIOError, OSError) as e:
            # 接收端 buffer 满 / 还没启动 / 网线拔了 —— 静默丢包
            self._dropped += 1
            if self._dropped % 100 == 1:
                log.warning("UDP 视频丢包 %d 次 (last err=%s)", self._dropped, e)
            return False

    def stats(self) -> dict:
        return {
            "sent": self._sent,
            "dropped": self._dropped,
            "drop_rate": (self._dropped / max(self._sent + self._dropped, 1)),
        }

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass
