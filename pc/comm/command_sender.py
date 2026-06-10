"""PC → Jetson 的 TCP 命令发送器。

设计：
- 长连接单 socket：UI 上点"切模式 / 开始 / 结束"是低频操作，复用一条 TCP
- 断线自动重连：UI 后台开个 daemon 线程轮询，连接断开就 sleep 再重连
- send() 是非阻塞的（放入队列），由后台线程实际 send
- 返回状态用回调：on_status(payload) 给 UI 更新

用法::

    sender = CommandSender(jetson_ip, cmd_port)
    sender.start(on_status=lambda s: print("status:", s))
    sender.send_mode("green_path")
    sender.send_start()
    ...
    sender.close()
"""
from __future__ import annotations

import logging
import queue
import socket
import threading
import time
from typing import Callable, Optional

from protocol import (
    STATE_IDLE,
    STATE_RUNNING,
    STATE_STOPPED,
    encode_cmd,
    parse_line,
)

log = logging.getLogger(__name__)


class CommandSender:
    """PC 端 TCP 命令发送器（带自动重连）。"""

    def __init__(self, jetson_ip: str, jetson_port: int, reconnect_interval: float = 1.0) -> None:
        self.ip = jetson_ip
        self.port = int(jetson_port)
        self.reconnect_interval = float(reconnect_interval)
        self._tx: queue.Queue[bytes] = queue.Queue()
        self._stop_event = threading.Event()
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._on_status: Optional[Callable[[str, str], None]] = None
        self._connected = False
        self._send_lock = threading.Lock()

    # ----- 公开 API -----

    def start(self, on_status: Optional[Callable[[str, str], None]] = None) -> None:
        self._on_status = on_status
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="cmd-sender")
        self._thread.start()
        log.info("TCP 命令发送已启动 → %s:%d", self.ip, self.port)

    def send(self, kind: str, payload: str = "") -> None:
        """非阻塞：命令入队由后台线程发出。"""
        self._tx.put(encode_cmd(kind, payload))

    def send_mode(self, mode_name: str) -> None:
        self.send("MODE", mode_name)

    def send_start(self) -> None:
        self.send("START")

    def send_stop(self) -> None:
        self.send("STOP")

    def send_ping(self) -> None:
        self.send("PING")

    def send_quit(self) -> None:
        self.send("QUIT")

    def is_connected(self) -> bool:
        return self._connected

    def close(self) -> None:
        self._stop_event.set()
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self._sock.close()
            except OSError:
                pass
        if self._thread:
            self._thread.join(timeout=2.0)
        log.info("TCP 命令发送已关闭")

    # ----- 后台线程 -----

    def _emit_status(self, kind: str, payload: str = "") -> None:
        if self._on_status is not None:
            try:
                self._on_status(kind, payload)
            except Exception as e:  # noqa: BLE001
                log.warning("on_status 回调异常: %s", e)

    def _connect(self) -> bool:
        try:
            s = socket.create_connection((self.ip, self.port), timeout=2.0)
            s.settimeout(1.0)
            self._sock = s
            self._connected = True
            self._emit_status("connected", f"{self.ip}:{self.port}")
            log.info("已连到 Jetson: %s:%d", self.ip, self.port)
            return True
        except OSError as e:
            log.debug("连 Jetson 失败: %s", e)
            self._connected = False
            return False

    def _loop(self) -> None:
        buf = b""
        while not self._stop_event.is_set():
            if self._sock is None:
                if not self._connect():
                    time.sleep(self.reconnect_interval)
                    continue
            # 先发一帧（不等 recv）：保证 UI 操作低延迟
            try:
                cmd = self._tx.get(timeout=0.05)
            except queue.Empty:
                cmd = None
            if cmd is not None:
                try:
                    with self._send_lock:
                        self._sock.sendall(cmd)
                except OSError as e:
                    log.warning("send 失败: %s", e)
                    self._close_sock()
                    continue
            # 拉回包（非阻塞式轮询）
            try:
                chunk = self._sock.recv(4096)
            except socket.timeout:
                continue
            except OSError as e:
                log.warning("recv 失败: %s", e)
                self._close_sock()
                continue
            if not chunk:
                log.info("Jetson 断开连接")
                self._close_sock()
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                msg = parse_line(raw)
                if msg is None:
                    continue
                self._emit_status(msg.kind, msg.payload)
        self._close_sock()

    def _close_sock(self) -> None:
        self._connected = False
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        self._emit_status("disconnected", "")
