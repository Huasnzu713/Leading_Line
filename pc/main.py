# -*- coding: utf-8 -*-
from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

# 直跑模式兼容：`python pc/main.py` 时 sys.path[0] 是 pc/，找不到 pc.* 包
# 这里把项目根加进去；包模式跑（python -m pc.main）时 ROOT 就是 CWD，已经在 path 上，加了也没副作用
_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

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
    vehicle_ip = net.get("vehicle_ip", "192.168.1.50")
    video_port = int(net.get("video_port", 9000))
    cmd_port = int(net.get("cmd_port", 9001))

    rx = VideoReceiver(port=video_port)
    rx.start()
    tx = CommandSender(jetson_ip=vehicle_ip, jetson_port=cmd_port)
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
