// -*- coding: utf-8 -*-
#include "protocol.hpp"

#include <cmath>
#include <cstring>
#include <stdexcept>

namespace leading_line_chassis::protocol {

namespace {

template <typename T>
void appendPOD(std::vector<uint8_t>& dst, const T& value) {
  const uint8_t* p = reinterpret_cast<const uint8_t*>(&value);
  dst.insert(dst.end(), p, p + sizeof(T));
}

}  // namespace

uint8_t bccSum(const std::vector<uint8_t>& bytes) {
  uint32_t s = 0;
  for (auto b : bytes) s += b;
  return static_cast<uint8_t>(s & 0xFF);
}

RawFrame encodeOdom(const OdomFrame& odom) {
  RawFrame f;
  f.type = kReportOdom;
  f.data.resize(sizeof(OdomFrame));
  std::memcpy(f.data.data(), &odom, sizeof(OdomFrame));
  return f;
}

RawFrame encodeSonar(const SonarFrame& sonar) {
  RawFrame f;
  f.type = kReportSonar;
  f.data.resize(sizeof(SonarFrame));
  std::memcpy(f.data.data(), &sonar, sizeof(SonarFrame));
  return f;
}

RawFrame encodeSensors(const SensorFrame& sensors) {
  RawFrame f;
  f.type = kReportSensors;
  f.data.resize(sizeof(SensorFrame));
  std::memcpy(f.data.data(), &sensors, sizeof(SensorFrame));
  return f;
}

std::vector<uint8_t> serializeFrame(const RawFrame& frame) {
  std::vector<uint8_t> out;
  out.reserve(2 + 1 + 1 + 1 + frame.data.size() + 1);
  out.push_back(kFrameHeader[0]);
  out.push_back(kFrameHeader[1]);
  out.push_back(frame.seq);
  out.push_back(frame.type);
  const uint8_t len = static_cast<uint8_t>(frame.data.size());
  out.push_back(len);
  out.insert(out.end(), frame.data.begin(), frame.data.end());
  out.push_back(bccSum(out));
  return out;
}

bool tryParseFrame(std::vector<uint8_t>& buffer, RawFrame& out) {
  // 数据太少
  if (buffer.size() < 6) return false;

  // 搜索帧头 0x2B 0xA2
  size_t i = 0;
  while (i + 1 < buffer.size() &&
         !(buffer[i] == kFrameHeader[0] && buffer[i + 1] == kFrameHeader[1])) {
    ++i;
  }
  if (i + 1 >= buffer.size()) {
    // 整个 buffer 都没找到帧头；只保留最后一个字节，下一轮可能拼上
    if (!buffer.empty()) buffer.erase(buffer.begin(), buffer.end() - 1);
    return false;
  }
  // 把帧头对齐到 0
  if (i > 0) buffer.erase(buffer.begin(), buffer.begin() + i);

  if (buffer.size() < 5) return false;  // 还差 type/len
  const uint8_t type = buffer[3];
  const uint8_t len = buffer[4];
  const size_t need = static_cast<size_t>(6) + len;  // header(2)+seq+type+len+data+checksum
  if (buffer.size() < need) return false;  // 等下一轮

  // 校验 type 高位必须为 0（参考原 Python 实现）
  if ((type & 0x80) != 0) {
    buffer.erase(buffer.begin(), buffer.begin() + 2);
    return false;
  }

  // BCC 校验
  uint32_t s = 0;
  for (size_t k = 0; k < 5 + len; ++k) s += buffer[k];
  const uint8_t expected = static_cast<uint8_t>(s & 0xFF);
  if (expected != buffer[5 + len]) {
    buffer.erase(buffer.begin(), buffer.begin() + 2);
    return false;
  }

  out.seq = buffer[2];
  out.type = type;
  out.data.assign(buffer.begin() + 5, buffer.begin() + 5 + len);
  buffer.erase(buffer.begin(), buffer.begin() + need);
  return true;
}

std::vector<uint8_t> encodeSpeedCommand(double vx_mps, double wz_radps, uint8_t seq) {
  // 把 (vx_mps, wz_radps) 编码成 0x81 下行帧。
  // 协议约定：speed_x 放 vx（mm/s），speed_y 固定 0，angular 放 wz（1/10000 rad/s）。
  // vx 量级 ~0.3 m/s，乘 1000 = ~300 mm/s，远小于 int16 限值（±32767），安全。
  int16_t vx_mmps = 0;
  if (std::isfinite(vx_mps)) {
    vx_mmps = static_cast<int16_t>(std::lround(vx_mps * 1000.0));
  }
  int16_t wz_milli = 0;
  if (std::isfinite(wz_radps)) {
    wz_milli = static_cast<int16_t>(std::lround(wz_radps * 10000.0));
  }

  RawFrame f;
  f.seq = seq;
  f.type = kCmdSpeed;
  f.data.resize(6);
  // 大端
  auto be16 = [](uint8_t* dst, int16_t v) {
    dst[0] = static_cast<uint8_t>((static_cast<uint16_t>(v) >> 8) & 0xFF);
    dst[1] = static_cast<uint8_t>(v & 0xFF);
  };
  be16(&f.data[0], vx_mmps);
  be16(&f.data[2], static_cast<int16_t>(0));  // vy
  be16(&f.data[4], wz_milli);
  return serializeFrame(f);
}

}  // namespace leading_line_chassis::protocol
