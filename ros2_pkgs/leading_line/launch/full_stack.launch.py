# -*- coding: utf-8 -*-
"""full_stack.launch.py — 底盘 + 摄像头 + 视觉 + 可选 PC 监控。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("usart_port_name", default_value="/dev/ttyXCar"),
        DeclareLaunchArgument("serial_baud_rate", default_value="115200"),
        DeclareLaunchArgument("wheelbase_m", default_value="0.30"),
        DeclareLaunchArgument("max_steer_deg", default_value="30.0"),
        DeclareLaunchArgument("max_speed_mps", default_value="0.5"),
        DeclareLaunchArgument("config_path", default_value="config.yaml"),
        DeclareLaunchArgument("camera_backend", default_value="usb_cam"),
        DeclareLaunchArgument("image_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("camera_index", default_value="0"),
        DeclareLaunchArgument("use_pc_monitor", default_value="true"),
        DeclareLaunchArgument("pc_ip", default_value="192.168.1.100"),
        DeclareLaunchArgument("cmd_port", default_value="9001"),
        DeclareLaunchArgument("video_port", default_value="9000"),
    ]

    # 1) 底盘
    chassis = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("leading_line_chassis"), "launch", "chassis.launch.py",
            ])
        ),
        launch_arguments={
            "usart_port_name": LaunchConfiguration("usart_port_name"),
            "serial_baud_rate": LaunchConfiguration("serial_baud_rate"),
            "wheelbase_m": LaunchConfiguration("wheelbase_m"),
            "max_steer_deg": LaunchConfiguration("max_steer_deg"),
            "max_speed_mps": LaunchConfiguration("max_speed_mps"),
        }.items(),
    )

    # 2) 视觉
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("leading_line"), "launch", "perception_with_camera.launch.py",
            ])
        ),
        launch_arguments={
            "config_path": LaunchConfiguration("config_path"),
            "camera_backend": LaunchConfiguration("camera_backend"),
            "image_topic": LaunchConfiguration("image_topic"),
            "camera_index": LaunchConfiguration("camera_index"),
            "max_steer_deg": LaunchConfiguration("max_steer_deg"),
            "max_speed_mps": LaunchConfiguration("max_speed_mps"),
        }.items(),
    )

    # 3) PC 监控（条件包含）
    pc_monitor = IncludeLaunchDescription(
        condition=IfCondition(LaunchConfiguration("use_pc_monitor")),
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("leading_line"), "launch", "pc_monitor.launch.py",
            ])
        ),
        launch_arguments={
            "config_path": LaunchConfiguration("config_path"),
            "pc_ip": LaunchConfiguration("pc_ip"),
            "cmd_port": LaunchConfiguration("cmd_port"),
            "video_port": LaunchConfiguration("video_port"),
            "start_stop_service": "/leading_line/start_stop",
            "set_mode_service": "/leading_line/set_mode",
            "debug_image_topic": "/debug/annotated_image",
        }.items(),
    )

    return LaunchDescription(args + [chassis, perception, pc_monitor])
