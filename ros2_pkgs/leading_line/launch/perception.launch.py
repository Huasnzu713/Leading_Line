# -*- coding: utf-8 -*-
"""perception.launch.py — 仅起视觉感知节点。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("config_path", default_value="config.yaml"),
        DeclareLaunchArgument("image_topic", default_value="/usb_cam/image_raw"),
        DeclareLaunchArgument("ackermann_topic", default_value="/ackermann_cmd"),
        DeclareLaunchArgument("max_steer_deg", default_value="30.0"),
        DeclareLaunchArgument("max_speed_mps", default_value="0.5"),
        DeclareLaunchArgument("min_speed_mps", default_value="0.05"),
        DeclareLaunchArgument("publish_hz", default_value="30.0"),
        DeclareLaunchArgument("default_mode", default_value="blue_path"),
        DeclareLaunchArgument("image_timeout_s", default_value="1.0"),
        DeclareLaunchArgument("publish_debug_image", default_value="true"),
        DeclareLaunchArgument("sonar_stop_m", default_value="0.30"),
        DeclareLaunchArgument("bat_warn_0p1V", default_value="70"),
        DeclareLaunchArgument("bat_critical_0p1V", default_value="65"),
    ]

    perception_node = Node(
        package="leading_line",
        executable="perception_node",
        name="perception_node",
        output="screen",
        parameters=[{
            "config_path": LaunchConfiguration("config_path"),
            "image_topic": LaunchConfiguration("image_topic"),
            "ackermann_topic": LaunchConfiguration("ackermann_topic"),
            "max_steer_deg": LaunchConfiguration("max_steer_deg"),
            "max_speed_mps": LaunchConfiguration("max_speed_mps"),
            "min_speed_mps": LaunchConfiguration("min_speed_mps"),
            "publish_hz": LaunchConfiguration("publish_hz"),
            "default_mode": LaunchConfiguration("default_mode"),
            "image_timeout_s": LaunchConfiguration("image_timeout_s"),
            "publish_debug_image": LaunchConfiguration("publish_debug_image"),
            "sonar_stop_m": LaunchConfiguration("sonar_stop_m"),
            "bat_warn_0p1V": LaunchConfiguration("bat_warn_0p1V"),
            "bat_critical_0p1V": LaunchConfiguration("bat_critical_0p1V"),
        }],
    )

    return LaunchDescription(args + [perception_node])
