"""PC 端入口（Qt UI）。

启动后：
1. 读 config_pc.yaml
2. 起 UDP 视频接收（监听 0.0.0.0:video_port）
3. 起 TCP 命令发送（连 jetson_ip:cmd_port）
4. 打开 Qt 主窗口
5. 窗口关闭 → 停通信线程 → 退出

运行::

    python -m pc.main_pc --config config_pc.yaml
"""
from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

import yaml
from PyQt5.QtWidgets import QApplication

from pc.comm.command_sender import CommandSender
from pc.comm.video_receiver import VideoReceiver
from pc.ui.main_window import MainWindow

log = logging.getLogger("pc.main")


def _setup_logging(level: str = "INFO") -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )


def load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="PC 端：Qt 监控 UI")
    p.add_argument("--config", default="config_pc.yaml", help="YAML 配置路径")
    p.add_argument("--log-level", default="INFO", help="DEBUG / INFO / WARNING / ERROR")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    _setup_logging(args.log_level)

    if not Path(args.config).exists():
        log.error("找不到配置文件: %s", args.config)
        return 1

    cfg = load_config(args.config)
    net = cfg.get("network", {})
    jetson_ip = net.get("jetson_ip", "192.168.1.50")
    video_port = int(net.get("video_port", 9000))
    cmd_port = int(net.get("cmd_port", 9001))

    rx = VideoReceiver(port=video_port)
    rx.start()
    tx = CommandSender(jetson_ip=jetson_ip, jetson_port=cmd_port)
    # tx.start() 在 MainWindow 里通过 on_status 回调启动

    app = QApplication.instance() or QApplication(sys.argv)
    win = MainWindow(cfg=cfg, video_receiver=rx, command_sender=tx)
    win.show()

    try:
        rc = app.exec()
    finally:
        log.info("UI 退出，关闭通信")
        try:
            tx.close()
        except Exception:  # noqa: BLE001
            pass
        try:
            rx.close()
        except Exception:  # noqa: BLE001
            pass
    return int(rc)


if __name__ == "__main__":
    sys.exit(main())
