"""PC → Jetson 的 TCP 命令发送器（带自动重连 + 心跳）。

设计：
- 长连接单 socket：UI 上点"切模式 / 开始 / 结束"是低频操作，复用一条 TCP
- 断线自动重连：UI 后台开个 daemon 线程轮询，连接断开就 sleep 再重连
- send() 是非阻塞的（放入队列），由后台线程实际 send
- 返回状态用回调：on_status(kind, payload) 给 UI 更新
- 心跳：enable_ping=True 时每 ping_interval_s 发一次 PING；收到 PONG 后算 RTT 填到 stats

用法::

    sender = CommandSender(jetson_ip, cmd_port, enable_ping=True, ping_interval_s=2.0)
    sender.start(on_status=lambda k, p: print(k, p))
    sender.send_mode("green_path")
    ...
"""
from __future__ import annotations

import logging
import queue
import socket
import threading
import time
from typing import Callable, Optional

from protocol import (
    encode_cmd,
    parse_line,
)

log = logging.getLogger(__name__)


class CommandSender:
    """PC 端 TCP 命令发送器（带自动重连 + 心跳）。"""

    def __init__(
        self,
        jetson_ip: str,
        jetson_port: int,
        reconnect_interval: float = 1.0,
        enable_ping: bool = True,
        ping_interval_s: float = 2.0,
        ping_timeout_s: float = 5.0,
    ) -> None:
        self.ip = jetson_ip
        self.port = int(jetson_port)
        self.reconnect_interval = float(reconnect_interval)
        self.enable_ping = bool(enable_ping)
        self.ping_interval_s = float(ping_interval_s)
        self.ping_timeout_s = float(ping_timeout_s)
        self._tx: queue.Queue[bytes] = queue.Queue()
        self._stop_event = threading.Event()
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._on_status: Optional[Callable[[str, str], None]] = None
        self._connected = False
        self._send_lock = threading.Lock()
        # 心跳状态
        self._last_ping_t: float = 0.0
        self._last_pong_t: float = 0.0
        self._last_rtt_ms: float = -1.0
        self._waiting_pong: bool = False
        # 状态 / 帧率统计（让 UI 可以显示）
        self._last_status_text: str = ""
        self._last_status_t: float = 0.0
        self._lock = threading.Lock()

    # ----- 公开 API -----

    def start(self, on_status: Optional[Callable[[str, str], None]] = None) -> None:
        self._on_status = on_status
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="cmd-sender")
        self._thread.start()
        log.info("TCP 命令发送已启动 → %s:%d (ping=%s, %.1fs)",
                 self.ip, self.port, self.enable_ping, self.ping_interval_s)

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

    def send_ack(self, payload: str = "REPORTING") -> None:
        self.send("ACK", payload)

    def is_connected(self) -> bool:
        return self._connected

    def stats(self) -> dict:
        """给 UI 显示的连接/心跳/状态摘要。"""
        with self._lock:
            now = time.monotonic()
            pong_age = (now - self._last_pong_t) if self._last_pong_t > 0 else None
            status_age = (now - self._last_status_t) if self._last_status_t > 0 else None
            return {
                "connected": self._connected,
                "rtt_ms": self._last_rtt_ms,
                "pong_age_s": pong_age,
                "status_age_s": status_age,
                "status_text": self._last_status_text,
                "ping_due": self.enable_ping and self._connected
                            and (now - self._last_ping_t) >= self.ping_interval_s
                            and not self._waiting_pong,
            }

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
            # 重建连接后立刻发一个 PING，确认对端活
            with self._lock:
                self._last_pong_t = time.monotonic()  # 占位，避免 stats 显示"待定"
                self._last_rtt_ms = -1.0
                self._waiting_pong = False
            self._emit_status("connected", f"{self.ip}:{self.port}")
            log.info("已连到 Jetson: %s:%d", self.ip, self.port)
            return True
        except OSError as e:
            log.debug("连 Jetson 失败: %s", e)
            self._connected = False
            return False

    def _send_ping_if_due(self) -> None:
        if not self.enable_ping or not self._connected:
            return
        now = time.monotonic()
        with self._lock:
            due = (now - self._last_ping_t) >= self.ping_interval_s
            waiting = self._waiting_pong
        if not due or waiting:
            return
        # 检查 PONG 超时：超时则判定 Jetson 没响应，断开重连
        with self._lock:
            if (self._waiting_pong
                    and self._last_ping_t > 0
                    and (now - self._last_ping_t) > self.ping_timeout_s):
                log.warning("PING 超时 (%.1fs)，重置连接", now - self._last_ping_t)
                self._close_sock()
                return
        # 入队一个 PING
        self._tx.put(encode_cmd("PING", str(int(time.time() * 1000))))
        with self._lock:
            self._last_ping_t = now
            self._waiting_pong = True

    def _loop(self) -> None:
        buf = b""
        while not self._stop_event.is_set():
            if self._sock is None:
                if not self._connect():
                    time.sleep(self.reconnect_interval)
                    continue
            # 心跳（PING/PONG 在后台线程自己发）
            self._send_ping_if_due()
            # 拉一帧用户命令
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
                self._handle_message(msg)
        self._close_sock()

    def _handle_message(self, msg) -> None:
        kind = msg.kind.upper()
        # PONG / STATUS 内部记一份，给 UI 读
        now = time.monotonic()
        with self._lock:
            if kind == "PONG":
                self._waiting_pong = False
                self._last_pong_t = now
                if self._last_ping_t > 0:
                    self._last_rtt_ms = (now - self._last_ping_t) * 1000.0
            elif kind == "STATUS":
                self._last_status_text = msg.payload
                self._last_status_t = now
        self._emit_status(kind, msg.payload)

    def _close_sock(self) -> None:
        self._connected = False
        with self._lock:
            self._waiting_pong = False
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        self._emit_status("disconnected", "")
