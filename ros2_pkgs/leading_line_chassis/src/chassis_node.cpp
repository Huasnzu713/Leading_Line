// -*- coding: utf-8 -*-
#include "chassis_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

#include "chassis_orientation.hpp"
#include "rclcpp/logging.hpp"

using std::placeholders::_1;

namespace leading_line_chassis {

namespace {
constexpr double kDegToRad = M_PI / 180.0;

// 参数范围限制
constexpr int kMinBaud = 9600;
constexpr int kMaxBaud = 460800;
constexpr double kMinWheelbase = 0.05;   // 5 cm（防止除零）
constexpr double kMaxSteerDeg = 60.0;    // 物理极限：60° 已非常极端
constexpr double kMinMaxSpeed = 0.01;    // 1 cm/s
constexpr double kDefaultMaxSpeed = 0.5; // m/s
}  // namespace

ChassisNode::ChassisNode(const rclcpp::NodeOptions& options) : rclcpp::Node("chassis_node", options) {
  // ---- 参数声明 ----
  usart_port_name_ = declare_parameter<std::string>("usart_port_name", "/dev/ttyXCar");
  serial_baud_rate_ = declare_parameter<int>("serial_baud_rate", 115200);
  read_timeout_ms_ = declare_parameter<int>("read_timeout_ms", 30);
  cmd_watchdog_ms_ = declare_parameter<int>("cmd_watchdog_ms", 500);
  loop_rate_hz_ = declare_parameter<int>("loop_rate_hz", 50);
  wheelbase_m_ = declare_parameter<double>("wheelbase_m", 0.30);
  max_steer_rad_ = declare_parameter<double>("max_steer_deg", 30.0) * kDegToRad;
  max_speed_mps_ = declare_parameter<double>("max_speed_mps", kDefaultMaxSpeed);
  odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
  robot_frame_id_ = declare_parameter<std::string>("robot_frame_id", "base_footprint");
  gyro_frame_id_ = declare_parameter<std::string>("gyro_frame_id", "gyro_link");
  publish_tf_ = declare_parameter<bool>("publish_tf", true);

  // 参数校验回调
  param_cb_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& params) { return onSetParameters(params); });

  // ---- ROS 通信 ----
  auto qos_best_effort = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
  auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  ackermann_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "/ackermann_cmd", qos_reliable,
      std::bind(&ChassisNode::onAckermannCmd, this, _1));
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", qos_best_effort);
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu", qos_best_effort);
  battery_pub_ = create_publisher<std_msgs::msg::Float32>("/battery_voltage", qos_reliable);
  sensors_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("/xcar/sensors", qos_reliable);
  for (int i = 0; i < 4; ++i) {
    sonar_pubs_[i] = create_publisher<sensor_msgs::msg::Range>(
        "/xcar/sonar" + std::to_string(i + 1), qos_best_effort);
  }
  ackermann_echo_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      "/ackermann_cmd_echo", qos_reliable);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // ---- 串口打开 ----
  serial_ = std::make_unique<serialio::SerialDriver>();
  try {
    serial_->open(usart_port_name_, static_cast<uint32_t>(serial_baud_rate_), read_timeout_ms_);
    RCLCPP_INFO(get_logger(), "串口已打开: %s @ %d", usart_port_name_.c_str(), serial_baud_rate_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "串口打开失败: %s（节点会持续重试）", e.what());
  }

  last_cmd_time_ = std::chrono::steady_clock::now() - std::chrono::milliseconds(cmd_watchdog_ms_ + 1);

  // 50 Hz 主循环
  const auto period = std::chrono::milliseconds(1000 / std::max(1, loop_rate_hz_));
  loop_timer_ = create_wall_timer(period, std::bind(&ChassisNode::tick, this));

  // on_shutdown 主动发 0
  rclcpp::on_shutdown([this]() {
    if (shutdown_started_.exchange(true)) return;
    RCLCPP_INFO(get_logger(), "on_shutdown：发 0 速度");
    publishZeroMotion();
    if (serial_ && serial_->isOpen()) serial_->close();
  });

  RCLCPP_INFO(get_logger(),
              "chassis_node 就绪: port=%s baud=%d wheelbase=%.2fm max_steer=%.1fdeg "
              "max_speed=%.2fm/s loop=%dHz watchdog=%dms",
              usart_port_name_.c_str(), serial_baud_rate_, wheelbase_m_,
              max_steer_rad_ / kDegToRad, max_speed_mps_, loop_rate_hz_, cmd_watchdog_ms_);
}

ChassisNode::~ChassisNode() {
  if (shutdown_started_.exchange(true)) return;
  publishZeroMotion();
  if (serial_ && serial_->isOpen()) serial_->close();
}

rcl_interfaces::msg::SetParametersResult ChassisNode::onSetParameters(
    const std::vector<rclcpp::Parameter>& params) {
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;
  res.reason = "";
  for (const auto& p : params) {
    const auto& name = p.get_name();
    if (name == "usart_port_name") {
      usart_port_name_ = p.as_string();
      try {
        serial_->open(usart_port_name_, static_cast<uint32_t>(serial_baud_rate_), read_timeout_ms_);
        RCLCPP_INFO(get_logger(), "串口切换: %s", usart_port_name_.c_str());
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "切换串口失败: %s", e.what());
      }
    } else if (name == "serial_baud_rate") {
      int v = p.as_int();
      if (v < kMinBaud || v > kMaxBaud) {
        res.successful = false;
        res.reason = "serial_baud_rate 必须在 [9600, 460800]";
        return res;
      }
      serial_baud_rate_ = v;
      try {
        serial_->open(usart_port_name_, static_cast<uint32_t>(serial_baud_rate_), read_timeout_ms_);
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "切换波特率失败: %s", e.what());
      }
    } else if (name == "wheelbase_m") {
      double v = p.as_double();
      if (v < kMinWheelbase) {
        res.successful = false;
        res.reason = "wheelbase_m 必须 > 0.05";
        return res;
      }
      wheelbase_m_ = v;
    } else if (name == "max_steer_deg") {
      double v = p.as_double();
      if (v <= 0.0 || v > kMaxSteerDeg) {
        res.successful = false;
        res.reason = "max_steer_deg 必须在 (0, 60]";
        return res;
      }
      max_steer_rad_ = v * kDegToRad;
    } else if (name == "max_speed_mps") {
      double v = p.as_double();
      if (v < kMinMaxSpeed) {
        res.successful = false;
        res.reason = "max_speed_mps 必须 >= 0.01";
        return res;
      }
      max_speed_mps_ = v;
    } else if (name == "cmd_watchdog_ms") {
      cmd_watchdog_ms_ = std::max(50, p.as_int());
    } else if (name == "loop_rate_hz") {
      loop_rate_hz_ = std::clamp(p.as_int(), 10, 200);
    } else if (name == "odom_frame_id") {
      odom_frame_id_ = p.as_string();
    } else if (name == "robot_frame_id") {
      robot_frame_id_ = p.as_string();
    } else if (name == "gyro_frame_id") {
      gyro_frame_id_ = p.as_string();
    } else if (name == "publish_tf") {
      publish_tf_ = p.as_bool();
    }
  }
  return res;
}

void ChassisNode::onAckermannCmd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
  // 1) clamp 入参
  double delta = std::isfinite(msg->drive.steering_angle) ? msg->drive.steering_angle : 0.0;
  delta = std::clamp(delta, -max_steer_rad_, max_steer_rad_);
  double v = std::isfinite(msg->drive.speed) ? msg->drive.speed : 0.0;
  v = std::clamp(v, -max_speed_mps_, max_speed_mps_);

  // 2) 阿克曼 → (vx, vy=0, wz)
  auto tw = kinematics::ackermannToTwist(delta, v, wheelbase_m_);

  // 3) 编码 + 发
  {
    std::lock_guard<std::mutex> lk(last_cmd_mu_);
    last_vx_mps_ = tw.vx_mps;
    last_wz_radps_ = tw.wz_radps;
    last_cmd_time_ = std::chrono::steady_clock::now();
  }
  if (serial_ && serial_->isOpen()) {
    auto bytes = protocol::encodeSpeedCommand(tw.vx_mps, tw.wz_radps, tx_seq_);
    serial_->sendBytes(bytes);
    tx_seq_ = static_cast<uint8_t>(tx_seq_ + 1);
  }

  // 4) 回放一份（调试 / 上层做闭环用）
  ackermann_msgs::msg::AckermannDriveStamped echo = *msg;
  echo.header.stamp = now();
  echo.header.frame_id = robot_frame_id_;
  echo.drive.steering_angle = delta;
  echo.drive.speed = v;
  ackermann_echo_pub_->publish(echo);
}

void ChassisNode::publishZeroMotion() {
  if (!(serial_ && serial_->isOpen())) return;
  auto bytes = protocol::encodeSpeedCommand(0.0, 0.0, tx_seq_);
  serial_->sendBytes(bytes);
  tx_seq_ = static_cast<uint8_t>(tx_seq_ + 1);
  std::lock_guard<std::mutex> lk(last_cmd_mu_);
  last_vx_mps_ = 0.0;
  last_wz_radps_ = 0.0;
}

void ChassisNode::reconnectSerial() {
  static auto last_attempt = std::chrono::steady_clock::time_point{};
  auto now_tp = std::chrono::steady_clock::now();
  if (now_tp - last_attempt < std::chrono::seconds(1)) return;
  last_attempt = now_tp;
  RCLCPP_WARN_THROTTLE(get_logger(), *this, 1000, "尝试重连串口 %s ...", usart_port_name_.c_str());
  try {
    serial_->open(usart_port_name_, static_cast<uint32_t>(serial_baud_rate_), read_timeout_ms_);
    RCLCPP_INFO(get_logger(), "串口重连成功");
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *this, 5000, "重连失败: %s", e.what());
  }
}

void ChassisNode::tick() {
  if (!serial_ || !serial_->isOpen()) {
    reconnectSerial();
    return;
  }
  // 1) 拉串口 + 解析
  serial_->pollFrames(read_timeout_ms_, [this](const protocol::RawFrame& f) { handleRawFrame(f); });

  // 2) watchdog：超时发 0
  {
    std::lock_guard<std::mutex> lk(last_cmd_mu_);
    auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - last_cmd_time_)
                   .count();
    if (age > cmd_watchdog_ms_ && (last_vx_mps_ != 0.0 || last_wz_radps_ != 0.0)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this, 2000, "watchdog：%ldms 无 /ackermann_cmd，发 0", age);
      // 离开锁再发，避免反向持锁
    }
  }
  if (serial_ && serial_->isOpen()) {
    std::lock_guard<std::mutex> lk(last_cmd_mu_);
    auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now() - last_cmd_time_)
                   .count();
    if (age > cmd_watchdog_ms_ && (last_vx_mps_ != 0.0 || last_wz_radps_ != 0.0)) {
      // 重新发一次 0
      lk.unlock();
      publishZeroMotion();
    }
  }
}

void ChassisNode::handleRawFrame(const protocol::RawFrame& f) {
  switch (f.type) {
    case protocol::kReportOdom:
      if (f.data.size() == sizeof(protocol::OdomFrame)) {
        protocol::OdomFrame odom;
        std::memcpy(&odom, f.data.data(), sizeof(odom));
        handleOdomFrame(odom);
      }
      break;
    case protocol::kReportSonar:
      if (f.data.size() == sizeof(protocol::SonarFrame)) {
        protocol::SonarFrame sonar;
        std::memcpy(&sonar, f.data.data(), sizeof(sonar));
        handleSonarFrame(sonar);
      }
      break;
    case protocol::kReportSensors:
      if (f.data.size() == sizeof(protocol::SensorFrame)) {
        protocol::SensorFrame sensors;
        std::memcpy(&sensors, f.data.data(), sizeof(sensors));
        handleSensorsFrame(sensors);
      }
      break;
    case protocol::kReportServo:
      // 暂不处理
      break;
    default:
      RCLCPP_WARN_THROTTLE(get_logger(), *this, 5000, "未知帧类型 0x%02X", f.type);
      break;
  }
}

void ChassisNode::handleOdomFrame(const protocol::OdomFrame& odom) {
  // zonesion 协议里：pos_x/pos_y 是绝对位置（mm），yaw 是绝对朝向（1/10000 rad）。
  // 我们直接重置本节点的累积量到 MCU 的绝对值，避免双重积分漂移。
  odom_x_ = odom.pos_x_mm / 1000.0;
  odom_y_ = odom.pos_y_mm / 1000.0;
  odom_yaw_ = odom.yaw_1e_4_rad / 10000.0;
  // 速度（也是 MCU 直接给）：m/s, rad/s
  const double vx = odom.speed_x_mmps / 1000.0;
  const double wz = odom.wz_1e_4_rads / 10000.0;

  // ---- /odom ----
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = now();
  odom_msg.header.frame_id = odom_frame_id_;
  odom_msg.child_frame_id = robot_frame_id_;
  odom_msg.pose.pose.position.x = odom_x_;
  odom_msg.pose.pose.position.y = odom_y_;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = orientationFromYaw(odom_yaw_);
  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = odom.speed_y_mmps / 1000.0;
  odom_msg.twist.twist.angular.z = wz;
  // 静止 / 运动 协方差
  static const std::array<double, 36> kOdomPoseMoving = {
      1e-3, 0, 0, 0, 0, 0,  0, 1e-3, 0, 0, 0, 0,  0, 0, 1e6, 0, 0, 0,
      0,    0, 0, 0, 1e6, 0, 0, 0,    0, 0, 0, 0,  0, 0, 0,   0, 0, 1e3,
  };
  static const std::array<double, 36> kOdomPoseStopped = {
      1e-9, 0, 0, 0, 0, 0,  0, 1e-3, 0, 0, 0, 0,  0, 0, 1e6, 0, 0, 0,
      0,    0, 0, 0, 1e6, 0, 0, 0,    0, 0, 0, 0,  0, 0, 0,   0, 0, 1e-9,
  };
  const bool stopped = (std::abs(vx) < 1e-3 && std::abs(wz) < 1e-3);
  std::copy(stopped ? kOdomPoseStopped.begin() : kOdomPoseMoving.begin(),
            stopped ? kOdomPoseStopped.end() : kOdomPoseMoving.end(),
            odom_msg.pose.covariance.begin());
  std::copy(stopped ? kOdomPoseStopped.begin() : kOdomPoseMoving.end(),
            stopped ? kOdomPoseStopped.end() : kOdomPoseMoving.end(),
            odom_msg.twist.covariance.begin());
  odom_pub_->publish(odom_msg);

  // ---- /imu（zonesion 无 IMU，用 odom 的角速度填一下方便下游用） ----
  auto imu_msg = sensor_msgs::msg::Imu();
  imu_msg.header.stamp = odom_msg.header.stamp;
  imu_msg.header.frame_id = gyro_frame_id_;
  imu_msg.orientation = odom_msg.pose.pose.orientation;
  imu_msg.orientation_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6};
  imu_msg.angular_velocity.x = 0.0;
  imu_msg.angular_velocity.y = 0.0;
  imu_msg.angular_velocity.z = wz;
  imu_msg.angular_velocity_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-3};
  imu_msg.linear_acceleration_covariance = {-1, 0, 0, 0, 0, 0, 0, 0, 0};  // 未知
  imu_pub_->publish(imu_msg);

  // ---- TF ----
  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = odom_msg.header.stamp;
    tf_msg.header.frame_id = odom_frame_id_;
    tf_msg.child_frame_id = robot_frame_id_;
    tf_msg.transform.translation.x = odom_x_;
    tf_msg.transform.translation.y = odom_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
  }

  last_odom_stamp_ = odom_msg.header.stamp;
}

void ChassisNode::handleSonarFrame(const protocol::SonarFrame& sonar) {
  const rclcpp::Time stamp = now();
  for (int i = 0; i < 4; ++i) {
    sensor_msgs::msg::Range r;
    r.header.stamp = stamp;
    r.header.frame_id = "sonar" + std::to_string(i + 1);
    r.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    r.field_of_view = 0.3f;
    r.min_range = 0.02f;
    r.max_range = 7.0f;
    // 原 zonesion 单位是 cm
    r.range = static_cast<float>(sonar.dist_cm[i]) / 100.0f;
    sonar_pubs_[i]->publish(r);
  }
}

void ChassisNode::handleSensorsFrame(const protocol::SensorFrame& sensors) {
  // /xcar/sensors：透传原 zonesion 数组
  std_msgs::msg::Int32MultiArray msg;
  msg.data = {
      static_cast<int32_t>(sensors.bat_0p1V),
      sensors.temp_0p1C,
      sensors.humi_pct,
      sensors.pressure_0p1Pa,
      sensors.light_lux,
      sensors.tvoc_ppm,
      sensors.smoke_ppm,
  };
  sensors_pub_->publish(msg);

  // /battery_voltage（伏）
  std_msgs::msg::Float32 bv;
  bv.data = static_cast<float>(sensors.bat_0p1V) / 10.0f;
  battery_pub_->publish(bv);
}

}  // namespace leading_line_chassis

// ---- main ----
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<leading_line_chassis::ChassisNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
