"""Jetson ← PC 的 TCP 命令接收器。

协议见 protocol/messages.py：
- 一条文本命令一行（"\\n" 结束）
- 命令类型：MODE / START / STOP / PING / QUIT
- 服务器端 accept 多个连接（Jetson 端只服务一个 PC，理论上也只能一个）

线程模型：
- 后台 daemon 线程跑 accept loop，主线程通过队列拿命令
- 关闭时 close() 把 socket 关掉，accept 抛异常 → 线程退出
- 避免每个命令都新开线程：长连接单条 socket 顺序处理
"""
from __future__ import annotations

import logging
import queue
import socket
import socketserver
import threading
from typing import Callable, Optional

from protocol import (
    STATE_IDLE,
    STATE_RUNNING,
    STATE_STOPPED,
    encode_status,
    parse_line,
)

log = logging.getLogger(__name__)


class _CmdHandler(socketserver.BaseRequestHandler):
    """单条 TCP 连接的处理：按行读命令，回 ACK。"""

    # handle() 通过 self.server 拿到外部注入的回调
    def handle(self) -> None:
        srv: "CommandReceiver" = self.server  # type: ignore[assignment]
        sock: socket.socket = self.request
        sock.settimeout(1.0)
        peer = self.client_address
        log.info("PC 已连接: %s:%s", peer[0], peer[1])
        srv._on_connect(peer)
        buf = b""
        try:
            while not srv._stop_event.is_set():
                try:
                    chunk = sock.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    log.info("PC 断开: %s:%s", peer[0], peer[1])
                    break
                buf += chunk
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    cmd = parse_line(raw)
                    if cmd is None:
                        continue
                    log.info("收到命令: %s %r", cmd.kind, cmd.payload)
                    srv._enqueue(cmd)
                    # 回包：让 PC 知道 Jetson 收到了
                    try:
                        sock.sendall(encode_status("ACK", cmd.kind))
                    except OSError:
                        break
        except OSError as e:
            log.warning("TCP 连接异常: %s", e)
        finally:
            srv._on_disconnect(peer)
            try:
                sock.close()
            except OSError:
                pass


class CommandReceiver(socketserver.ThreadingTCPServer):
    """单实例 TCP 服务：监听 0.0.0.0:port，把命令塞进队列。

    用 ThreadingTCPServer 而不是普通 TCPServer：
    - allow_reuse_address 避免重启时端口 TIME_WAIT
    - daemon_threads 让主程序退出时连接线程自动 kill

    用法::

        rx = CommandReceiver(host="0.0.0.0", port=9001)
        rx.start()
        while True:
            cmd = rx.get(timeout=0.1)
            if cmd: handle(cmd)
    """

    allow_reuse_address = True
    daemon_threads = True

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 9001,
        on_connect: Optional[Callable] = None,
        on_disconnect: Optional[Callable] = None,
    ) -> None:
        super().__init__((host, port), _CmdHandler)
        self._queue: queue.Queue = queue.Queue()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._on_connect_cb = on_connect
        self._on_disconnect_cb = on_disconnect

    # ---- 内部回调（被 handler 调用） ----

    def _enqueue(self, cmd) -> None:
        self._queue.put(cmd)

    def _on_connect(self, peer) -> None:
        if self._on_connect_cb:
            try:
                self._on_connect_cb(peer)
            except Exception as e:  # noqa: BLE001
                log.warning("on_connect 回调异常: %s", e)

    def _on_disconnect(self, peer) -> None:
        if self._on_disconnect_cb:
            try:
                self._on_disconnect_cb(peer)
            except Exception as e:  # noqa: BLE001
                log.warning("on_disconnect 回调异常: %s", e)

    # ---- 公开 API ----

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self.serve_forever, daemon=True)
        self._thread.start()
        log.info("TCP 命令服务已启动: %s:%s", self.server_address[0], self.server_address[1])

    def get(self, timeout: float = 0.1):
        """从队列取一条命令；空时返回 None（非阻塞）。"""
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def close(self) -> None:
        self._stop_event.set()
        try:
            self.shutdown()
            self.server_close()
        except Exception:  # noqa: BLE001
            pass
        if self._thread:
            self._thread.join(timeout=2.0)
        log.info("TCP 命令服务已关闭")
