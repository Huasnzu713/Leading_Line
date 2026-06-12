# -*- coding: utf-8 -*-
"""chassis_with_robot_description.launch.py — 底盘 + URDF + TF。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    urdf_arg = DeclareLaunchArgument(
        "urdf_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("leading_line_chassis"), "urdf", "mini_akm_robot.urdf"]
        ),
        description="URDF 路径（mini_akm_robot.urdf / senior_akm_robot.urdf）",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(["cat ", LaunchConfiguration("urdf_file")]),
                "publish_frequency": 50.0,
            }
        ],
    )

    # 底盘节点（转发到 chassis.launch.py 的内容）
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    chassis_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("leading_line_chassis"), "launch", "chassis.launch.py"]
            )
        ),
    )

    return LaunchDescription([urdf_arg, robot_state_publisher, chassis_include])
