# -*- coding: utf-8 -*-
from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

# 直跑模式兼容：`python vehicle/main.py` 时相对 import 会失效，
# 把项目根加进 sys.path 让绝对 import 也能工作。包模式跑时无副作用。
_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import yaml

from vehicle.comm.command_receiver import CommandReceiver
from vehicle.comm.video_sender import VideoSender
from vehicle.pipeline import Pipeline
from vehicle.ros_bridge import RosBridge

log = logging.getLogger("vehicle.main")


def _setup_logging(level: str = "INFO") -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )


def load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def build_pipeline(cfg: dict) -> Pipeline:
    net = cfg.get("network", {})
    pc_ip = net.get("pc_ip", "192.168.1.100")
    video_port = int(net.get("video_port", 9000))
    cmd_port = int(net.get("cmd_port", 9001))
    bind_host = net.get("bind_host", "0.0.0.0")

    ros_cfg = cfg.get("ros", {})
    ros = RosBridge(
        backend=ros_cfg.get("backend", "mock"),
        wheelbase_m=float(ros_cfg.get("wheelbase_m", 0.30)),
    )

    sender = VideoSender(pc_ip=pc_ip, pc_port=video_port)
    receiver = CommandReceiver(host=bind_host, port=cmd_port)

    runtime = cfg.get("vehicle_runtime", {})
    return Pipeline(
        cfg=cfg,
        video_sender=sender,
        cmd_receiver=receiver,
        ros_bridge=ros,
        jpeg_quality=int(runtime.get("jpeg_quality", 70)),
        fps_cap=float(runtime.get("fps_cap", 30.0)),
    )


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="vehicle 端：摄像头 + 算法 + UDP 推流 + TCP 命令")
    p.add_argument("--config", default="config.yaml", help="YAML 配置路径")
    p.add_argument("--log-level", default="INFO", help="DEBUG / INFO / WARNING / ERROR")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    _setup_logging(args.log_level)

    if not Path(args.config).exists():
        log.error("找不到配置文件: %s", args.config)
        return 1

    cfg = load_config(args.config)
    pipeline = build_pipeline(cfg)
    log.info("vehicle 端启动，PC=%s", cfg.get("network", {}).get("pc_ip", "(default)"))
    pipeline.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
