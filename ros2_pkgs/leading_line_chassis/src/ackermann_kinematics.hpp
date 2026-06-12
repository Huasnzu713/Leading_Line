#pragma once
// -*- coding: utf-8 -*-
// 阿克曼运动学：(steering_angle_rad, speed_mps, wheelbase_m) → (vx, vy, wz)
// vy 强制 0（阿克曼 4WD 物理上无横向滑动）。
// 实现细节：v·tan(delta)/L；速度极小或角度极小时直接置 0 避免数值抖动。

#include <cstdint>

namespace leading_line_chassis::kinematics {

struct AckermannToTwist {
  double vx_mps;    // 前向速度 m/s
  double vy_mps;    // 横向速度 m/s（阿克曼恒为 0）
  double wz_radps;  // 偏航角速度 rad/s
};

// 阿克曼自行车模型：wz = v * tan(delta) / L
// 入参已假定在外部 clamp 过。
AckermannToTwist ackermannToTwist(double steering_angle_rad, double speed_mps, double wheelbase_m);

}  // namespace leading_line_chassis::kinematics
