"""Jetson 端入口。

启动后会做：
1. 读 jetson/config.yaml（算法 + network + ros + overrides 全在里头）
2. 打开 UDP 视频发送（→ PC 的 IP:VIDEO_PORT）
3. 打开 TCP 命令接收（监听 0.0.0.0:CMD_PORT）
4. 起 RosBridge（默认 mock，可改 ros）
5. Pipeline.run() 阻塞跑摄像头 + 算法 + override + 推流 + 响应命令

运行::

    python -m jetson.main_jetson --config jetson/config.yaml
"""
from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

import yaml

from .comm.command_receiver import CommandReceiver
from .comm.video_sender import VideoSender
from .pipeline import Pipeline
from .ros_bridge import RosBridge

log = logging.getLogger("jetson.main")


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

    runtime = cfg.get("jetson_runtime", {})
    return Pipeline(
        cfg=cfg,
        video_sender=sender,
        cmd_receiver=receiver,
        ros_bridge=ros,
        jpeg_quality=int(runtime.get("jpeg_quality", 70)),
        fps_cap=float(runtime.get("fps_cap", 30.0)),
    )


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Jetson 端：摄像头 + 算法 + UDP 推流 + TCP 命令")
    p.add_argument("--config", default="jetson/config.yaml", help="YAML 配置路径")
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
    log.info("Jetson 端启动，PC=%s", cfg.get("network", {}).get("pc_ip", "(default)"))
    pipeline.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
