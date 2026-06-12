# -*- coding: utf-8 -*-
"""leading_line — ROS2 Humble 视觉感知 + PC 监控桥。

子模块：
- perception_node:  rclpy 节点，/image → algo → /ackermann_cmd
- perception_pipeline: 算法流水线（颜色分割→路径规划→控制→override）
- ackermann_bridge:   (steer_deg, speed) → ackermann_msgs 发布
- vehicle_pipeline:   原 vehicle/pipeline.py 的 ROS 包装，PC 监控模式
- pc_monitor_bridge:  PC TCP/UDP ↔ ROS2 service + /debug/annotated_image
- overrides:          箭头/QR 覆盖层
"""
