# -*- coding: utf-8 -*-
"""perception_with_camera.launch.py — 摄像头 + 视觉感知。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("config_path", default_value="config.yaml"),
        DeclareLaunchArgument("camera_backend", default_value="none",
                              description="none | usb_cam | realsense | v4l2"),
        DeclareLaunchArgument("image_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("camera_index", default_value="0"),
        DeclareLaunchArgument("max_steer_deg", default_value="30.0"),
        DeclareLaunchArgument("max_speed_mps", default_value="0.5"),
        DeclareLaunchArgument("min_speed_mps", default_value="0.05"),
        DeclareLaunchArgument("publish_hz", default_value="30.0"),
        DeclareLaunchArgument("default_mode", default_value="blue_path"),
        DeclareLaunchArgument("image_timeout_s", default_value="1.0"),
        DeclareLaunchArgument("publish_debug_image", default_value="true"),
    ]

    # 摄像头（可选）
    usb_cam_node = Node(
        condition=IfCondition(LaunchConfiguration("camera_backend") == "usb_cam"),
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        output="screen",
        parameters=[{
            "video_device": LaunchConfiguration("camera_index"),
            "image_width": 640,
            "image_height": 480,
            "framerate": 30.0,
            "pixel_format": "yuyv2rgb",
            "camera_frame_id": "camera_link",
        }],
        remappings=[("/image_raw", LaunchConfiguration("image_topic"))],
    )

    v4l2_node = Node(
        condition=IfCondition(LaunchConfiguration("camera_backend") == "v4l2"),
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="v4l2_camera",
        output="screen",
        parameters=[{
            "video_device": LaunchConfiguration("camera_index"),
            "image_size": [640, 480],
            "camera_frame_id": "camera_link",
        }],
        remappings=[("/image_raw", LaunchConfiguration("image_topic"))],
    )

    perception_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("leading_line"), "launch", "perception.launch.py",
            ])
        ),
        launch_arguments={
            "config_path": LaunchConfiguration("config_path"),
            "image_topic": LaunchConfiguration("image_topic"),
            "max_steer_deg": LaunchConfiguration("max_steer_deg"),
            "max_speed_mps": LaunchConfiguration("max_speed_mps"),
            "min_speed_mps": LaunchConfiguration("min_speed_mps"),
            "publish_hz": LaunchConfiguration("publish_hz"),
            "default_mode": LaunchConfiguration("default_mode"),
            "image_timeout_s": LaunchConfiguration("image_timeout_s"),
            "publish_debug_image": LaunchConfiguration("publish_debug_image"),
        }.items(),
    )

    return LaunchDescription(args + [usb_cam_node, v4l2_node, perception_include])
