// -*- coding: utf-8 -*-
// GTest：protocol.hpp 的编/解码、BCC 校验、tryParseFrame 流式重组。
// 不开串口，纯内存测试。

#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "protocol.hpp"

using namespace leading_line_chassis::protocol;

namespace {

// 把 RawFrame 推回流式 buffer，模拟一次"半截帧 + 后续半截"
void append(std::vector<uint8_t>& buf, const std::vector<uint8_t>& bytes) {
  buf.insert(buf.end(), bytes.begin(), bytes.end());
}

}  // namespace

// ---- 0x81 速度命令编码 ----

TEST(EncodeSpeed, Zero) {
  auto bytes = encodeSpeedCommand(0.0, 0.0, 0);
  ASSERT_EQ(bytes.size(), 6u + 6u);  // header(2)+seq+type+len(1)+data(6)+bcc(1) = 11
  EXPECT_EQ(bytes[0], kFrameHeader[0]);
  EXPECT_EQ(bytes[1], kFrameHeader[1]);
  EXPECT_EQ(bytes[2], 0u);
  EXPECT_EQ(bytes[3], kCmdSpeed);
  EXPECT_EQ(bytes[4], 6u);
  // BCC
  uint8_t s = 0;
  for (size_t i = 0; i + 1 < bytes.size(); ++i) s = static_cast<uint8_t>(s + bytes[i]);
  EXPECT_EQ(s, bytes.back());
}

TEST(EncodeSpeed, MaxForwardAndRight) {
  // vx = 0.3 m/s → 300 mm/s；wz = 0.5 rad/s → 5000
  auto bytes = encodeSpeedCommand(0.3, 0.5, 7);
  EXPECT_EQ(bytes[2], 7u);
  // vx_mmps = 300 → 0x012C
  EXPECT_EQ(bytes[5], 0x01);
  EXPECT_EQ(bytes[6], 0x2C);
  // vy = 0
  EXPECT_EQ(bytes[7], 0x00);
  EXPECT_EQ(bytes[8], 0x00);
  // wz_milli = 5000 → 0x1388
  EXPECT_EQ(bytes[9], 0x13);
  EXPECT_EQ(bytes[10], 0x88);
}

TEST(EncodeSpeed, NegativeSpeed) {
  auto bytes = encodeSpeedCommand(-0.1, -0.2, 0);
  // 0.1 m/s × 1000 = 100 mm/s = 0x0064
  EXPECT_EQ(bytes[5], 0x00);
  EXPECT_EQ(bytes[6], 0x64);
  // -0.2 rad/s × 10000 = -2000 = 0xF830
  EXPECT_EQ(bytes[9], 0xF8);
  EXPECT_EQ(bytes[10], 0x30);
}

// ---- tryParseFrame ----

TEST(ParseFrame, OkOdom) {
  // 拼一个合法 odom 帧
  OdomFrame odom{};
  odom.pos_x_mm = 1000;
  odom.pos_y_mm = 2000;
  odom.yaw_1e_4_rad = 12345;
  odom.speed_x_mmps = 30;
  odom.speed_y_mmps = 0;
  odom.wz_1e_4_rads = 50;
  RawFrame f = encodeOdom(odom);
  f.seq = 42;
  auto bytes = serializeFrame(f);

  std::vector<uint8_t> buf = bytes;
  RawFrame out;
  ASSERT_TRUE(tryParseFrame(buf, out));
  EXPECT_EQ(out.type, kReportOdom);
  EXPECT_EQ(out.seq, 42u);
  ASSERT_EQ(out.data.size(), sizeof(OdomFrame));
  OdomFrame got;
  std::memcpy(&got, out.data.data(), sizeof(got));
  EXPECT_EQ(got.pos_x_mm, 1000);
  EXPECT_EQ(got.pos_y_mm, 2000);
  EXPECT_EQ(got.yaw_1e_4_rad, 12345);
  EXPECT_EQ(got.speed_x_mmps, 30);
  EXPECT_EQ(got.wz_1e_4_rads, 50);
  EXPECT_TRUE(buf.empty());
}

TEST(ParseFrame, BadChecksum) {
  OdomFrame odom{};
  odom.pos_x_mm = 0;
  RawFrame f = encodeOdom(odom);
  auto bytes = serializeFrame(f);
  bytes.back() ^= 0xFF;  // 损坏 BCC
  std::vector<uint8_t> buf = bytes;
  RawFrame out;
  EXPECT_FALSE(tryParseFrame(buf, out));
}

TEST(ParseFrame, BadTypeHighBit) {
  // 构造一个 type=0x81 的非法上行帧（高位置 1）
  std::vector<uint8_t> bytes = {
      kFrameHeader[0], kFrameHeader[1], 0, 0x81, 0,
  };
  std::vector<uint8_t> buf = bytes;
  RawFrame out;
  EXPECT_FALSE(tryParseFrame(buf, out));
}

TEST(ParseFrame, SplitAcrossReads) {
  OdomFrame odom{};
  odom.pos_x_mm = 999;
  odom.pos_y_mm = 0;
  odom.yaw_1e_4_rad = 0;
  odom.speed_x_mmps = 0;
  odom.speed_y_mmps = 0;
  odom.wz_1e_4_rads = 0;
  RawFrame f = encodeOdom(odom);
  auto bytes = serializeFrame(f);

  // 第一波只发一半
  std::vector<uint8_t> buf;
  append(buf, std::vector<uint8_t>(bytes.begin(), bytes.begin() + 6));
  RawFrame out;
  EXPECT_FALSE(tryParseFrame(buf, out));

  // 第二波再发剩余
  append(buf, std::vector<uint8_t>(bytes.begin() + 6, bytes.end()));
  EXPECT_TRUE(tryParseFrame(buf, out));
  EXPECT_EQ(out.type, kReportOdom);
  ASSERT_EQ(out.data.size(), sizeof(OdomFrame));
  OdomFrame got;
  std::memcpy(&got, out.data.data(), sizeof(got));
  EXPECT_EQ(got.pos_x_mm, 999);
  EXPECT_TRUE(buf.empty());
}

TEST(ParseFrame, TwoFramesBackToBack) {
  OdomFrame o1{};
  o1.pos_x_mm = 1;
  OdomFrame o2{};
  o2.pos_x_mm = 2;
  auto b1 = serializeFrame(encodeOdom(o1));
  auto b2 = serializeFrame(encodeOdom(o2));
  std::vector<uint8_t> buf;
  append(buf, b1);
  append(buf, b2);

  RawFrame f1, f2;
  ASSERT_TRUE(tryParseFrame(buf, f1));
  ASSERT_TRUE(tryParseFrame(buf, f2));
  EXPECT_EQ(buf.size(), 0u);
  OdomFrame g1, g2;
  std::memcpy(&g1, f1.data.data(), sizeof(g1));
  std::memcpy(&g2, f2.data.data(), sizeof(g2));
  EXPECT_EQ(g1.pos_x_mm, 1);
  EXPECT_EQ(g2.pos_x_mm, 2);
}

TEST(ParseFrame, StrayBytesBeforeHeader) {
  OdomFrame o{};
  o.pos_x_mm = 5;
  auto bytes = serializeFrame(encodeOdom(o));
  std::vector<uint8_t> buf = {0xDE, 0xAD, 0xBE, 0xEF};
  append(buf, bytes);
  RawFrame out;
  EXPECT_TRUE(tryParseFrame(buf, out));
  OdomFrame g;
  std::memcpy(&g, out.data.data(), sizeof(g));
  EXPECT_EQ(g.pos_x_mm, 5);
}

// ---- ackermann_kinematics ----

#include "ackermann_kinematics.hpp"

TEST(Kinematics, Straight) {
  using namespace leading_line_chassis::kinematics;
  auto t = ackermannToTwist(0.0, 0.3, 0.30);
  EXPECT_NEAR(t.vx_mps, 0.3, 1e-9);
  EXPECT_NEAR(t.vy_mps, 0.0, 1e-9);
  EXPECT_NEAR(t.wz_radps, 0.0, 1e-9);
}

TEST(Kinematics, Turn) {
  using namespace leading_line_chassis::kinematics;
  // v=0.3, delta=0.349 rad (20°), L=0.30 → wz = 0.3 * tan(20°) / 0.3 = tan(20°) ≈ 0.36397
  auto t = ackermannToTwist(0.349066, 0.3, 0.30);
  EXPECT_NEAR(t.wz_radps, std::tan(0.349066), 1e-3);
}

TEST(Kinematics, TinyDeltaZeroWz) {
  using namespace leading_line_chassis::kinematics;
  auto t = ackermannToTwist(1e-6, 0.3, 0.30);
  EXPECT_NEAR(t.wz_radps, 0.0, 1e-9);
}

TEST(Kinematics, TinySpeedZeroWz) {
  using namespace leading_line_chassis::kinematics;
  auto t = ackermannToTwist(0.5, 1e-5, 0.30);
  EXPECT_NEAR(t.wz_radps, 0.0, 1e-9);
}

TEST(Kinematics, NaNInputs) {
  using namespace leading_line_chassis::kinematics;
  auto t = ackermannToTwist(std::nan(""), 0.3, 0.30);
  EXPECT_NEAR(t.vx_mps, 0.3, 1e-9);
  EXPECT_NEAR(t.wz_radps, 0.0, 1e-9);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
