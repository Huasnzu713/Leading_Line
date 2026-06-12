# -*- coding: utf-8 -*-
"""pc_monitor.launch.py — 仅起 PC 监控桥。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("config_path", default_value="config.yaml"),
        DeclareLaunchArgument("cmd_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("cmd_port", default_value="9001"),
        DeclareLaunchArgument("pc_ip", default_value="192.168.1.100"),
        DeclareLaunchArgument("video_port", default_value="9000"),
        DeclareLaunchArgument("video_jpeg_quality", default_value="70"),
        DeclareLaunchArgument("start_stop_service", default_value="/leading_line/start_stop"),
        DeclareLaunchArgument("set_mode_service", default_value="/leading_line/set_mode"),
    ]

    pc_bridge = Node(
        package="leading_line",
        executable="pc_monitor_bridge",
        name="pc_monitor_bridge",
        output="screen",
        parameters=[{
            "config_path": LaunchConfiguration("config_path"),
            "cmd_host": LaunchConfiguration("cmd_host"),
            "cmd_port": LaunchConfiguration("cmd_port"),
            "pc_ip": LaunchConfiguration("pc_ip"),
            "video_port": LaunchConfiguration("video_port"),
            "video_jpeg_quality": LaunchConfiguration("video_jpeg_quality"),
            "start_stop_service": LaunchConfiguration("start_stop_service"),
            "set_mode_service": LaunchConfiguration("set_mode_service"),
        }],
    )

    return LaunchDescription(args + [pc_bridge])
