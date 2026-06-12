// -*- coding: utf-8 -*-
// 共用小工具：四元数从 yaw 角生成（提取独立头便于 chassis_node.cpp 引用）。
#pragma once

#include <cmath>

#include "geometry_msgs/msg/quaternion.hpp"

namespace leading_line_chassis {

inline geometry_msgs::msg::Quaternion orientationFromYaw(double yaw_rad) {
  const double half = yaw_rad * 0.5;
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half);
  q.w = std::cos(half);
  return q;
}

}  // namespace leading_line_chassis
