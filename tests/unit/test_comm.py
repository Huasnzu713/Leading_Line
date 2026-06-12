# -*- coding: utf-8 -*-
import logging
import socket
import struct
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

# 测试在 tests/unit/，项目根是 parents[2]
_PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from vehicle.comm.video_sender import VideoSender
from vehicle.comm.command_receiver import CommandReceiver
from pc.comm.video_receiver import VideoReceiver
from pc.comm.command_sender import CommandSender
from protocol import encode_video_frame, decode_video_frame, encode_status

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
log = logging.getLogger("e2e")

VIDEO_PORT = 19000
CMD_PORT = 19001
J_IP = "127.0.0.1"


def mock_frame(w=320, h=240) -> bytes:
    img = np.zeros((h, w, 3), np.uint8)
    img[:] = (h * 7 % 256, w * 13 % 256, (h + w) * 5 % 256)
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
    assert ok
    return buf.tobytes()


def main() -> int:
    cfg = yaml.safe_load(open("config.yaml", encoding="utf-8"))

    # === Jetson 端：TCP server + UDP sender ===
    rx = CommandReceiver(host="0.0.0.0", port=CMD_PORT)
    rx.start()
    sender = VideoSender(pc_ip=J_IP, pc_port=VIDEO_PORT)

    # === PC 端：UDP receiver + TCP sender ===
    receiver = VideoReceiver(port=VIDEO_PORT, queue_size=3)
    receiver.start()
    pc = CommandSender(J_IP, CMD_PORT, enable_ping=True, ping_interval_s=0.5)
    pc_msgs: list[tuple[str, str]] = []
    pc.start(on_status=lambda k, p: pc_msgs.append((k, p)))

    # 1) 等 PC 连上 Jetson
    log.info("等 PC 连上 Jetson ...")
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and not pc.is_connected():
        time.sleep(0.05)
    assert pc.is_connected(), "PC 未连上 Jetson"
    log.info("PC 已连")

    # 2) Jetson 主动 STATUS 推
    rx.push_status("STATUS", "state=IDLE mode=blue_path fps=0.0 ros=mock clients=1")
    time.sleep(0.3)

    # 3) UDP 推 5 帧
    log.info("Jetson → PC 推 5 帧 UDP 视频 ...")
    for i in range(5):
        sender.send(mock_frame(), ts_ms=int(time.time() * 1000))
        time.sleep(0.05)
    time.sleep(0.5)
    rx.push_status("STATUS", "state=RUNNING mode=blue_path fps=20.0 ros=mock clients=1")

    # 4) PC 端 PING → Jetson 回 PONG
    log.info("PC 发 PING ...")
    pc.send_ping()
    time.sleep(0.5)

    # 5) PC 端发 MODE / START
    log.info("PC 发 MODE green_path + START ...")
    pc.send_mode("green_path")
    pc.send_start()
    time.sleep(0.3)

    # 6) Jetson 推 INFO (QR 状态变化) + STATUS (REPORTING)
    rx.push_status("INFO", "qr_state:IDLE->SCANNING")
    rx.push_status("STATUS", "REPORTING")
    time.sleep(0.3)

    # 7) PC ack REPORTING
    log.info("PC 发 ACK REPORTING ...")
    pc.send_ack("REPORTING")
    time.sleep(0.3)

    # 等待 PING 至少 2 个周期
    time.sleep(1.5)

    # === 收尾：读 PC 收到的所有消息 ===
    log.info("=== PC 收到的所有消息（kind, payload）===")
    for k, p in pc_msgs:
        log.info("  %-12s %s", k, p)

    # === 视频侧 ===
    last = receiver.latest_frame()
    log.info("PC 端 latest_frame shape: %s", getattr(last, "shape", None))

    # === 断言 ===
    kinds = [k for k, _ in pc_msgs]
    assert "connected" in kinds, f"缺 connected: {kinds}"
    assert kinds.count("ACK") >= 3, f"ACK 数量不足: {kinds}"
    assert kinds.count("PONG") >= 2, f"PONG 数量不足: {kinds}"
    assert "STATUS" in kinds, f"缺 STATUS 推回: {kinds}"
    assert "INFO" in kinds, f"缺 INFO 推回: {kinds}"
    assert last is not None and last.shape == (240, 320, 3), \
        f"latest_frame shape 异常: {getattr(last, 'shape', None)}"

    # 心跳 RTT 应被记录
    st = pc.stats()
    log.info("PC 心跳 stats: %s", st)
    assert st["rtt_ms"] >= 0, "RTT 未被记录"

    # === 关闭 ===
    log.info("通过所有断言，准备关闭 ...")
    sender.close()
    receiver.close()
    pc.close()
    rx.close()
    log.info("=== 全部通过 ===")
    return 0


if __name__ == "__main__":
    sys.exit(main())
