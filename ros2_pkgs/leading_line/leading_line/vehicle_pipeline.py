# -*- coding: utf-8 -*-
"""vehicle_pipeline.py — ROS2 节点包装 leading_line.pipeline.Pipeline。

保留 PC 监控模式（cv2 摄像头 + UDP 推流 + TCP 命令），运动控制走
ackermann_bridge.publish，发布到 /ackermann_cmd 话题。

用法：
    ros2 run leading_line vehicle_pipeline --ros-args -p config_path:=/abs/config.yaml
"""
from __future__ import annotations

import logging
import sys
import threading
from pathlib import Path

import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from leading_line.ackermann_bridge import make_rclpy_bridge
from leading_line.comm.command_receiver import CommandReceiver
from leading_line.comm.video_sender import VideoSender
from leading_line.pipeline import Pipeline

log = logging.getLogger(__name__)


class VehiclePipelineNode(Node):
    """在 ROS2 节点里跑 leading_line.pipeline.Pipeline。"""

    def __init__(self) -> None:
        super().__init__("vehicle_pipeline")

        self.declare_parameter("config_path", "config.yaml")
        self.declare_parameter("max_steer_deg", 30.0)
        self.declare_parameter("max_speed_mps", 0.5)
        cfg_path = str(self.get_parameter("config_path").value)
        cfg_abs = str(Path(cfg_path).expanduser().resolve())
        with open(cfg_abs, "r", encoding="utf-8") as fp:
            cfg = yaml.safe_load(fp)
        # 桥
        self._bridge = make_rclpy_bridge(
            self,
            max_speed_mps=float(self.get_parameter("max_speed_mps").value),
            max_steer_deg=float(self.get_parameter("max_steer_deg").value),
        )

        net = cfg.get("network", {})
        video_sender = VideoSender(
            pc_ip=net.get("pc_ip", "192.168.1.100"),
            pc_port=int(net.get("video_port", 9000)),
        )
        cmd_receiver = CommandReceiver(
            host=net.get("bind_host", "0.0.0.0"),
            port=int(net.get("cmd_port", 9001)),
        )

        rt = cfg.get("vehicle_runtime", {})
        self._pipeline = Pipeline(
            cfg=cfg,
            video_sender=video_sender,
            cmd_receiver=cmd_receiver,
            publish_cmd=self._bridge.publish,
            backend_name="ackermann",
            jpeg_quality=int(rt.get("jpeg_quality", 70)),
            fps_cap=float(rt.get("fps_cap", 30.0)),
        )
        self.get_logger().info("vehicle_pipeline 启动中…")

    def run(self) -> None:
        try:
            self._pipeline.run()
        finally:
            self._bridge.stop()


def main(args=None) -> int:
    rclpy.init(args=args)
    node = VehiclePipelineNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    # 单独线程跑 pipeline（cv2 主循环 + 串口/TCP 不能和 ROS spin 抢线程）
    t = threading.Thread(target=node.run, daemon=True)
    t.start()
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
