# -*- coding: utf-8 -*-
"""pc_monitor_bridge.py — PC TCP/UDP ↔ ROS2 service / 视频推流。

桥接协议/ 字节层（不依赖 ROS 1）的 CommandReceiver + VideoSender 到 ROS 2：
  - 收 TCP 命令（START / STOP / MODE / PING / ACK）→ 转发到 ROS 服务
  - 订阅 /debug/annotated_image → JPEG 编码 → UDP 推给 PC
  - 主动把车辆 STATUS（state/mode/fps/...）周期性 UDP 推给 PC
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Optional

import cv2
import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image

from leading_line.comm.command_receiver import CommandReceiver
from leading_line.comm.video_sender import VideoSender
from protocol import STATE_ERROR, STATE_IDLE, STATE_RUNNING, STATE_STOPPED, encode_status

log = logging.getLogger(__name__)


class PcMonitorBridge(Node):
    """ROS2 节点：TCP/UDP ↔ ROS 桥。"""

    def __init__(self) -> None:
        super().__init__("pc_monitor_bridge")

        self.declare_parameter("config_path", "config.yaml")
        self.declare_parameter("debug_image_topic", "/debug/annotated_image")
        self.declare_parameter("cmd_host", "0.0.0.0")
        self.declare_parameter("cmd_port", 9001)
        self.declare_parameter("pc_ip", "192.168.1.100")
        self.declare_parameter("video_port", 9000)
        self.declare_parameter("video_jpeg_quality", 70)
        self.declare_parameter("status_push_period_s", 1.0)
        # 服务名（要调的 perception 服务）
        self.declare_parameter("start_stop_service", "/leading_line/start_stop")
        self.declare_parameter("set_mode_service", "/leading_line/set_mode")

        p = self.get_parameters_by_prefix("")
        self._cfg_path = str(p["config_path"].value)
        self._debug_topic = str(p["debug_image_topic"].value)
        self._cmd_host = str(p["cmd_host"].value)
        self._cmd_port = int(p["cmd_port"].value)
        self._pc_ip = str(p["pc_ip"].value)
        self._video_port = int(p["video_port"].value)
        self._jpeg_quality = int(p["video_jpeg_quality"].value)
        self._status_period = float(p["status_push_period_s"].value)
        self._start_stop_srv = str(p["start_stop_service"].value)
        self._set_mode_srv = str(p["set_mode_service"].value)

        # ---- 网络端 ----
        self._cmd_recv = CommandReceiver(host=self._cmd_host, port=self._cmd_port)
        self._cmd_recv.start()
        self._video = VideoSender(pc_ip=self._pc_ip, pc_port=self._video_port)
        # PC 端地址会在 on_connect 时回填（首次有客户端连接时）
        self._pc_known = False

        # ---- ROS 通信 ----
        self._last_img_lock = threading.Lock()
        self._last_jpeg: Optional[bytes] = None
        self.create_subscription(Image, self._debug_topic, self._on_image, 1)

        # 调 perception 服务
        from std_srvs.srv import SetBool
        self._start_stop_client = self.create_client(SetBool, self._start_stop_srv)
        self._set_mode_client = self.create_client(SetBool, self._set_mode_srv)
        # 状态
        self._state = STATE_IDLE
        self._mode = "blue_path"
        self._last_status_t = 0.0

        # 钩 CommandReceiver 的 on_connect/on_disconnect 拿 PC 地址
        # 用一个小包装：override 已有回调 → 仅记地址
        self._cmd_recv._on_connect_cb = self._on_pc_connect  # noqa: SLF001
        self._cmd_recv._on_disconnect_cb = self._on_pc_disconnect  # noqa: SLF001

        # 30 Hz 推流 + 状态回推
        self.create_timer(1.0 / 30.0, self._tick_video)
        self.create_timer(0.2, self._tick_status)

        self.get_logger().info(
            f"PC 桥就绪: TCP {self._cmd_host}:{self._cmd_port} → UDP {self._pc_ip}:{self._video_port}"
        )

    # ---- 钩子 ----
    def _on_pc_connect(self, peer) -> None:
        ip, port = peer[0], peer[1]
        self.get_logger().info(f"PC 已连接: {ip}:{port}")
        # 切 VideoSender 目标 IP（如果 PC 在不同网段）
        if ip != self._pc_ip:
            self._video.pc_ip = ip
        self._pc_known = True

    def _on_pc_disconnect(self, peer) -> None:
        self.get_logger().info(f"PC 断开: {peer[0]}:{peer[1]}")
        self._pc_known = False

    # ---- 图像 → JPEG ----
    def _on_image(self, msg: Image) -> None:
        try:
            from cv_bridge import CvBridge
            frame = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, buf = cv2.imencode(
                ".jpg", frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality],
            )
            if ok:
                with self._last_img_lock:
                    self._last_jpeg = buf.tobytes()
        except Exception as e:  # noqa: BLE001
            self.get_logger().debug(f"图像编码失败: {e}")

    # ---- 30 Hz 推流 ----
    def _tick_video(self) -> None:
        if not self._pc_known:
            return
        with self._last_img_lock:
            jpeg = self._last_jpeg
        if jpeg:
            self._video.send(jpeg, ts_ms=int(time.time() * 1000))

    # ---- 5 Hz 状态回推 ----
    def _tick_status(self) -> None:
        if not self._pc_known:
            return
        now = time.monotonic()
        if now - self._last_status_t < self._status_period:
            return
        self._last_status_t = now
        status_text = (
            f"state={self._state} mode={self._mode} "
            f"fps=0.0 ros=ackermann clients={self._cmd_recv.client_count()}"
        )
        n = self._cmd_recv.push_status("STATUS", status_text)
        if n:
            self.get_logger().debug(f"STATUS 推 {n} 客户端")

    # ---- 主循环 ----
    def spin_loop(self) -> None:
        """rclpy.spin() 之外的 CommandReceiver 轮询线程。

        在 rclpy.spin() 跑的同时另开一个线程拉 TCP 命令，调 ROS 服务。
        """
        while rclpy.ok() and not self._shutdown:
            cmd = self._cmd_recv.get(timeout=0.1)
            if cmd is None:
                continue
            self._handle_cmd(cmd)

    def _handle_cmd(self, cmd) -> None:
        kind = cmd.kind.upper()
        payload = cmd.payload.strip()
        self.get_logger().info(f"收到 PC 命令: {kind} {payload!r}")
        if kind == "START":
            self._call_service(self._start_stop_client, True)
            self._state = STATE_RUNNING
        elif kind == "STOP":
            self._call_service(self._start_stop_client, False)
            self._state = STATE_STOPPED
        elif kind == "MODE":
            # std_srvs/SetBool 没有 string 字段；这里把 mode 编码到 bool (True)
            # 然后 perception 端的 _on_set_mode 用 str(req.data) 解
            self._call_service(self._set_mode_client, payload)
            self._mode = payload or self._mode
        elif kind == "PING":
            pass  # CommandReceiver 内部已回 PONG
        elif kind == "ACK":
            # 透传到 perception 端，由 overrides.on_ack() 处理
            pass
        elif kind == "INFO":
            self.get_logger().info(f"PC INFO: {payload}")
        elif kind == "QUIT":
            self.get_logger().info("收到 QUIT")
            self._shutdown = True
        else:
            self.get_logger().warning(f"未知命令: {kind} {payload!r}")

    def _call_service(self, client, data) -> None:
        from std_srvs.srv import SetBool, SetBool_Request
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(f"服务 {client.srv_name} 不在线")
            return
        req = SetBool_Request()
        if isinstance(data, bool):
            req.data = data
        else:
            # std_srvs/SetBool.data 是 bool，没法直接传 mode 名字。
            # 妥协：mode 命令当 bool=True 触发，perception 端会用上一次的 mode。
            # TODO 换成 std_srvs/Trigger 或自定义 srv。
            req.data = True
        future = client.call_async(req)
        future.add_done_callback(self._on_srv_done)

    def _on_srv_done(self, future) -> None:
        try:
            resp = future.result()
            self.get_logger().debug(f"服务返回: success={resp.success} msg={resp.message}")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"服务调用异常: {e}")


_shutdown_attr = "_shutdown"


def main(args=None) -> int:
    rclpy.init(args=args)
    node = PcMonitorBridge()
    setattr(node, _shutdown_attr, False)
    spin_thread = threading.Thread(target=node.spin_loop, daemon=True)
    spin_thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        setattr(node, _shutdown_attr, True)
        spin_thread.join(timeout=2.0)
        node._cmd_recv.close()
        node._video.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
