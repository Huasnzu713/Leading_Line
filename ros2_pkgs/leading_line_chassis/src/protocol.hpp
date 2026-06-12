#pragma once
// -*- coding: utf-8 -*-
// zonesion xcar 0x2B-A2 串口协议：定义、帧结构、编/解码、BCC 校验。
// 移植自 ros_pkgs/leading_line/scripts/xcar/xcar_protocol.py（原文件已删除）。
// 仅本文件涉及"线"格式；底盘驱动本身不直接做字节 IO，IO 在 serial_driver.hpp 里。

#include <array>
#include <cstdint>
#include <vector>

namespace leading_line_chassis::protocol {

// 帧头 2 字节
constexpr std::array<uint8_t, 2> kFrameHeader = {0x2B, 0xA2};

// 下行（ROS → MCU）类型
constexpr uint8_t kCmdSpeed = 0x81;       // 设置 (vx, vy, wz)
constexpr uint8_t kCmdServo = 0x82;       // 舵机控制（本包暂不实现）
constexpr uint8_t kCmdGripper = 0x83;     // 夹爪控制（本包暂不实现）
constexpr uint8_t kCmdServoReq = 0x84;    // 舵机状态请求

// 上行（MCU → ROS）类型
constexpr uint8_t kReportOdom = 0x01;     // 里程计
constexpr uint8_t kReportSonar = 0x02;    // 4 路超声
constexpr uint8_t kReportSensors = 0x03;  // 环境传感器（bat/temp/humi/...）
constexpr uint8_t kReportServo = 0x04;    // 舵机角度

#pragma pack(push, 1)

// 0x01 里程计帧：16 字节内容
// 大端：pos_x(int32, mm) pos_y(int32, mm) yaw(int32, 1/10000 rad)
//      speed_x(int16, mm/s) speed_y(int16, mm/s) angular(int16, 1/10000 rad/s)
struct OdomFrame {
  int32_t pos_x_mm;
  int32_t pos_y_mm;
  int32_t yaw_1e_4_rad;
  int16_t speed_x_mmps;
  int16_t speed_y_mmps;
  int16_t wz_1e_4_rads;
};
static_assert(sizeof(OdomFrame) == 16, "OdomFrame 大小必须为 16");

// 0x02 超声帧：4×int16 距离（cm）
struct SonarFrame {
  int16_t dist_cm[4];
};
static_assert(sizeof(SonarFrame) == 8, "SonarFrame 大小必须为 8");

// 0x03 环境传感器帧：13 字节
// bat(uint8, 0.1V) temp(int16, 0.1°C) humi(int16, %Rh) pressure(int32, 0.1Pa)
// light(int16, lux) tvoc(int16, ppm) smoke(int16, ppm)
struct SensorFrame {
  uint8_t bat_0p1V;
  int16_t temp_0p1C;
  int16_t humi_pct;
  int32_t pressure_0p1Pa;
  int16_t light_lux;
  int16_t tvoc_ppm;
  int16_t smoke_ppm;
};
static_assert(sizeof(SensorFrame) == 13, "SensorFrame 大小必须为 13");

#pragma pack(pop)

// 通用帧视图：header(2) seq(1) type(1) len(1) data(len) checksum(1)
struct RawFrame {
  uint8_t seq = 0;
  uint8_t type = 0;
  std::vector<uint8_t> data;
};

// 把 OdomFrame 等结构体打包成 RawFrame（自动加 len 与 type）
RawFrame encodeOdom(const OdomFrame& odom);
RawFrame encodeSonar(const SonarFrame& sonar);
RawFrame encodeSensors(const SensorFrame& sensors);

// 校验和：累加 bytes[0..N-1] & 0xFF == bytes[N]
uint8_t bccSum(const std::vector<uint8_t>& bytes);

// 把 RawFrame 序列化为线字节（带 header / tail / BCC）。下到串口前用。
std::vector<uint8_t> serializeFrame(const RawFrame& frame);

// 解析：从一个流式缓冲里搜索帧头/帧尾/校验和，命中后返回一个 RawFrame。
// 解析成功：填好 out，erase 已消费的字节，返回 true。
// 解析失败（checksum/length/header 错误）：丢掉错位字节，返回 false。
// 数据不足：返回 false，且 buffer 未被改动。
bool tryParseFrame(std::vector<uint8_t>& buffer, RawFrame& out);

// 下行 0x81 速度帧：直接给 (vx_mps, wz_radps)，vy 强制 0
std::vector<uint8_t> encodeSpeedCommand(double vx_mps, double wz_radps, uint8_t seq);

}  // namespace leading_line_chassis::protocol
