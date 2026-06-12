#pragma once
// -*- coding: utf-8 -*-
// ChassisNode：ROS2 Humble 底盘驱动节点。
//   - 订阅 /ackermann_cmd（ackermann_msgs/AckermannDriveStamped）
//   - 内部按阿克曼换算 → 编码为 zonesion 0x2B-A2 帧 → 写串口
//   - 50 Hz 主循环里拉 MCU 上报，发布 /odom、/imu、/battery_voltage、
//     /xcar/sonar[1-4]、/xcar/sensors，并广播 odom→base_footprint TF
//   - 500ms watchdog：长时间没收到 ackermann_cmd 就发 0
//   - 析构 / on_shutdown 主动发 0

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "ackermann_kinematics.hpp"
#include "protocol.hpp"
#include "serial_driver.hpp"

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace leading_line_chassis {

class ChassisNode : public rclcpp::Node {
 public:
  explicit ChassisNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~ChassisNode() override;

 private:
  // ---- 生命周期 ----
  void onAckermannCmd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult onSetParameters(
      const std::vector<rclcpp::Parameter>& params);
  void tick();  // 50 Hz 主循环
  void reconnectSerial();
  void publishZeroMotion();

  // ---- 处理 MCU 上报 ----
  void handleOdomFrame(const protocol::OdomFrame& odom);
  void handleSonarFrame(const protocol::SonarFrame& sonar);
  void handleSensorsFrame(const protocol::SensorFrame& sensors);
  void handleRawFrame(const protocol::RawFrame& f);

  // ---- 状态量 ----
  // 串口
  std::unique_ptr<serialio::SerialDriver> serial_;
  std::string usart_port_name_;
  int serial_baud_rate_;
  int read_timeout_ms_;
  int cmd_watchdog_ms_;
  int loop_rate_hz_;
  // 阿克曼运动学
  double wheelbase_m_;
  double max_steer_rad_;
  double max_speed_mps_;
  // 帧 ID
  std::string odom_frame_id_;
  std::string robot_frame_id_;
  std::string gyro_frame_id_;
  // 控制量状态（最近一次下发）
  std::mutex last_cmd_mu_;
  std::chrono::steady_clock::time_point last_cmd_time_;
  double last_vx_mps_ = 0.0;
  double last_wz_radps_ = 0.0;
  // 序列号（下行 0x81 帧自增）
  uint8_t tx_seq_ = 0;
  // 屏蔽 publish_tf（调试）
  bool publish_tf_;

  // ---- ROS 通信 ----
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr sensors_pub_;
  std::array<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr, 4> sonar_pubs_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_echo_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // 里程计累积
  double odom_x_ = 0.0;
  double odom_y_ = 0.0;
  double odom_yaw_ = 0.0;
  rclcpp::Time last_odom_stamp_;

  // 周期 timer
  rclcpp::TimerBase::SharedPtr loop_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  // 启动/结束时的兜底 0
  std::atomic<bool> shutdown_started_{false};
};

}  // namespace leading_line_chassis
