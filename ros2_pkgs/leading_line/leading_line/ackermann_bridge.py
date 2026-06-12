# -*- coding: utf-8 -*-
"""ackermann_bridge.py — (steer_deg, speed) → ackermann_msgs 发布。

同时保留 MockBridge（搬自原 vehicle/ros_bridge.py:42-64），给单元测试用。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple


# ---------------------------------------------------------------------------
# Mock：把每次 (steer_deg, speed) 缓存到 .last，无 ROS 时调试用
# ---------------------------------------------------------------------------

class MockBridge:
    """无 ROS 时的占位实现：把每次发布缓存到 self.last。"""

    def __init__(self, max_speed_mps: float = 0.5, max_steer_deg: float = 30.0) -> None:
        self.max_speed_mps = float(max_speed_mps)
        self.max_steer_deg = float(max_steer_deg)
        self.last: Optional[Tuple[float, float]] = None
        self.publish_count: int = 0

    def publish(self, steer_deg: float, speed: float) -> None:
        steer_deg = max(min(float(steer_deg), self.max_steer_deg), -self.max_steer_deg)
        speed = max(min(float(speed), 1.0), 0.0)
        self.last = (steer_deg, speed)
        self.publish_count += 1

    def stop(self) -> None:
        self.publish(0.0, 0.0)


# ---------------------------------------------------------------------------
# rclpy 真发布：ackermann_msgs/AckermannDriveStamped → /ackermann_cmd
# ---------------------------------------------------------------------------

def make_rclpy_bridge(node, max_speed_mps: float = 0.5, max_steer_deg: float = 30.0,
                      topic: str = "/ackermann_cmd", frame_id: str = "base_footprint"):
    """工厂函数：返回一个 rclpy AckermannBridge 实例，挂在 node 上。

    用工厂而不是直接暴露类，是为了把"是否 import rclpy"放到调用时决定
    （测试 import 该模块时不会拉起 rclpy + cv_bridge）。
    """
    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped

    class _RclpyBridge:
        def __init__(self) -> None:
            self.max_speed_mps = float(max_speed_mps)
            self.max_steer_deg = float(max_steer_deg)
            self.frame_id = str(frame_id)
            self._pub = node.create_publisher(AckermannDriveStamped, topic, 10)
            self._msg_type = AckermannDriveStamped

        def publish(self, steer_deg: float, speed: float) -> None:
            steer_deg = max(min(float(steer_deg), self.max_steer_deg), -self.max_steer_deg)
            speed = max(min(float(speed), 1.0), 0.0)
            msg = self._msg_type()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.drive.steering_angle = math.radians(steer_deg)
            msg.drive.speed = speed * self.max_speed_mps
            self._pub.publish(msg)

        def stop(self) -> None:
            self.publish(0.0, 0.0)

    return _RclpyBridge()
