# -*- coding: utf-8 -*-
"""chassis.launch.py — 启动底盘驱动节点。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # ---- 参数 ----
    declared = [
        DeclareLaunchArgument(
            "usart_port_name",
            default_value="/dev/ttyXCar",
            description="串口路径（zonesion xcar 默认 /dev/ttyXCar）",
        ),
        DeclareLaunchArgument(
            "serial_baud_rate",
            default_value="115200",
            description="串口波特率",
        ),
        DeclareLaunchArgument(
            "wheelbase_m",
            default_value="0.30",
            description="阿克曼等效轴距（米）",
        ),
        DeclareLaunchArgument(
            "max_steer_deg",
            default_value="30.0",
            description="最大前轮转角（度），>0",
        ),
        DeclareLaunchArgument(
            "max_speed_mps",
            default_value="0.5",
            description="最大前向速度（m/s）",
        ),
        DeclareLaunchArgument(
            "cmd_watchdog_ms",
            default_value="500",
            description="无 /ackermann_cmd 多少 ms 后发 0",
        ),
        DeclareLaunchArgument(
            "loop_rate_hz",
            default_value="50",
            description="主循环频率（Hz）",
        ),
        DeclareLaunchArgument(
            "publish_tf",
            default_value="true",
            description="是否发布 odom→base_footprint TF",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("leading_line_chassis"), "config", "chassis.yaml"]
            ),
            description="YAML 参数文件",
        ),
    ]

    chassis_node = Node(
        package="leading_line_chassis",
        executable="chassis_node",
        name="chassis_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "usart_port_name": LaunchConfiguration("usart_port_name"),
                "serial_baud_rate": LaunchConfiguration("serial_baud_rate"),
                "wheelbase_m": LaunchConfiguration("wheelbase_m"),
                "max_steer_deg": LaunchConfiguration("max_steer_deg"),
                "max_speed_mps": LaunchConfiguration("max_speed_mps"),
                "cmd_watchdog_ms": LaunchConfiguration("cmd_watchdog_ms"),
                "loop_rate_hz": LaunchConfiguration("loop_rate_hz"),
                "publish_tf": LaunchConfiguration("publish_tf"),
            },
        ],
    )

    return LaunchDescription(declared + [chassis_node])
