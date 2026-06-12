// -*- coding: utf-8 -*-
#include "ackermann_kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace leading_line_chassis::kinematics {

AckermannToTwist ackermannToTwist(double steering_angle_rad, double speed_mps, double wheelbase_m) {
  AckermannToTwist out{};
  out.vx_mps = std::isfinite(speed_mps) ? speed_mps : 0.0;
  out.vy_mps = 0.0;

  // 速度或角度极小 → 角速度 0（避免 tan 数值爆炸）
  constexpr double kVMin = 1e-3;
  constexpr double kDeltaMin = 1e-3;
  if (std::abs(out.vx_mps) < kVMin || std::abs(steering_angle_rad) < kDeltaMin ||
      wheelbase_m <= 0.0) {
    out.wz_radps = 0.0;
  } else {
    out.wz_radps = out.vx_mps * std::tan(steering_angle_rad) / wheelbase_m;
  }
  return out;
}

}  // namespace leading_line_chassis::kinematics
