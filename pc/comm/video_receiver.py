"""PC ← Jetson 的 UDP 视频接收器。

设计目标：
- 后台线程跑 recvfrom，把解出来的 BGR 帧塞进一个有界队列
- 队列上限避免 Jetson 推太快时占爆内存；溢出时丢最老的
- 暴露 latest_frame() 给 Qt 定时器拉图；不在主线程里做 cv2.imdecode

线程模型：
- 1 个 daemon 线程：socket → 解析 → 队列
- 主线程：Qt UI timer（30Hz）→ latest_frame() → QImage → QLabel
"""
from __future__ import annotations

import collections
import logging
import socket
import threading
import time
from typing import Optional

import cv2
import numpy as np

from protocol import VideoFrame, decode_video_frame

log = logging.getLogger(__name__)


class VideoReceiver:
    """UDP 视频接收端（PC 侧）。"""

    def __init__(self, port: int, queue_size: int = 3) -> None:
        self.port = int(port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 512 * 1024)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # 1s 超时让 stop() 能及时唤醒
        self._sock.settimeout(1.0)
        self._sock.bind(("0.0.0.0", self.port))
        self._queue: collections.deque = collections.deque(maxlen=int(queue_size))
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._packets = 0
        self._decoded = 0
        self._drops = 0
        self._last_seq: Optional[int] = None
        self._t0 = time.monotonic()

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="video-receiver")
        self._thread.start()
        log.info("UDP 视频接收已启动: 0.0.0.0:%d", self.port)

    def _loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                packet, _addr = self._sock.recvfrom(64 * 1024)
            except socket.timeout:
                continue
            except OSError as e:
                if self._stop_event.is_set():
                    break
                log.warning("recvfrom 异常: %s", e)
                continue
            self._packets += 1
            frame = decode_video_frame(packet)
            if frame is None:
                self._drops += 1
                continue
            # JPEG → BGR（不在主线程里跑，避免卡 UI）
            try:
                arr = np.frombuffer(frame.jpeg, dtype=np.uint8)
                bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            except Exception as e:  # noqa: BLE001
                log.warning("JPEG 解码失败: %s", e)
                self._drops += 1
                continue
            if bgr is None:
                self._drops += 1
                continue
            self._decoded += 1
            if self._last_seq is not None:
                expected = (self._last_seq + 1) & 0xFF
                if frame.seq != expected:
                    # 丢一帧；不致命，只统计
                    pass
            self._last_seq = frame.seq
            with self._lock:
                self._queue.append(bgr)

    def latest_frame(self) -> Optional[np.ndarray]:
        with self._lock:
            if not self._queue:
                return None
            # deque 满了会自动丢老帧；这里取最新的
            return self._queue[-1]

    def stats(self) -> dict:
        elapsed = max(time.monotonic() - self._t0, 1e-6)
        return {
            "packets": self._packets,
            "decoded": self._decoded,
            "drops": self._drops,
            "fps_in": self._decoded / elapsed,
        }

    def close(self) -> None:
        self._stop_event.set()
        try:
            self._sock.close()
        except OSError:
            pass
        if self._thread:
            self._thread.join(timeout=2.0)
        log.info("UDP 视频接收已关闭")
