"""Jetson ← PC 的 TCP 命令接收器（兼服务器端文本回推）。

协议见 protocol/messages.py：
- 一条文本命令一行（"\\n" 结束）
- 命令类型：MODE / START / STOP / PING / QUIT / ACK
- 服务器端 accept 多个连接（Jetson 端理论上只服务一个 PC，但允许多个调试时连）

主动回推（Jetson → PC）：
- 收到 ``PING`` → 回 ``PONG <ts>``
- 主循环调 ``push_status`` / ``push_info`` → 广播到所有已连接客户端
- 客户端断线时通过 ``on_disconnect`` 回调通知

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
from typing import Callable, List, Optional, Tuple

from protocol import (
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
        srv._register_client(sock, peer)
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
                    # PING 单独处理：立即回 PONG（不丢进队列）
                    if cmd.kind == "PING":
                        srv._safe_send(sock, encode_status("PONG", cmd.payload))
                        continue
                    srv._enqueue(cmd)
                    # 回包：让 PC 知道 Jetson 收到了
                    srv._safe_send(sock, encode_status("ACK", cmd.kind))
        except OSError as e:
            log.warning("TCP 连接异常: %s", e)
        finally:
            srv._unregister_client(sock)
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
        rx.push_status("RUNNING", "blue_path")  # 主动推
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
        # 已注册客户端（sockets 列表 + lock）
        self._clients_lock = threading.Lock()
        self._clients: List[Tuple[socket.socket, Tuple]] = []

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

    # ---- 客户端注册 / 推送 ----

    def _register_client(self, sock: socket.socket, peer: Tuple) -> None:
        with self._clients_lock:
            self._clients.append((sock, peer))

    def _unregister_client(self, sock: socket.socket) -> None:
        with self._clients_lock:
            self._clients = [(s, p) for (s, p) in self._clients if s is not sock]

    @staticmethod
    def _safe_send(sock: socket.socket, data: bytes) -> None:
        try:
            sock.sendall(data)
        except OSError:
            pass  # 客户端断线，handler 那边会清理

    def has_clients(self) -> bool:
        with self._clients_lock:
            return len(self._clients) > 0

    def client_count(self) -> int:
        with self._clients_lock:
            return len(self._clients)

    def push_status(self, kind: str, payload: str = "") -> int:
        """向所有已连接客户端推一条 STATUS/INFO 文本。

        返回实际成功发送的客户端数。
        """
        if not self.has_clients():
            return 0
        data = encode_status(kind, payload)
        n_ok = 0
        # 拷贝一份再遍历，避免回调里 list 被改
        with self._clients_lock:
            clients = list(self._clients)
        for sock, _peer in clients:
            try:
                sock.sendall(data)
                n_ok += 1
            except OSError:
                # 客户端断了；handler 的 recv 也会抛，最后会 unregister
                pass
        return n_ok

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
        # 主动关所有客户端连接
        with self._clients_lock:
            for sock, _ in self._clients:
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                try:
                    sock.close()
                except OSError:
                    pass
            self._clients.clear()
        if self._thread:
            self._thread.join(timeout=2.0)
        log.info("TCP 命令服务已关闭")
