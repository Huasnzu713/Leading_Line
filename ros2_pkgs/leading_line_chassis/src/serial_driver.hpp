#pragma once
// -*- coding: utf-8 -*-
// 串口 IO + 帧解析。
// 把 libserial 句柄 + zonesion 0x2B-A2 帧解析封装成一个可注入 chassis_node 的组件。

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "protocol.hpp"

namespace serial {
class Serial;
}

namespace leading_line_chassis::serialio {

struct ParseResult {
  bool ok = false;
  protocol::RawFrame frame;
  std::string error;  // ok=false 时给出原因（用于日志节流）
};

// 线程安全的串口封装：
//  - open() / close() 显式管理（不在构造里 open，构造时只装好配置）
//  - tryRead(timeout_ms) → 拉一坨字节 + 解析成最多 N 个完整帧
//  - sendFrame(...) → 阻塞写一帧
//  - 解析失败的字节会被丢掉，详见 protocol::tryParseFrame
class SerialDriver {
 public:
  SerialDriver();
  ~SerialDriver();

  SerialDriver(const SerialDriver&) = delete;
  SerialDriver& operator=(const SerialDriver&) = delete;

  // 打开串口。失败抛 std::runtime_error。
  void open(const std::string& port, uint32_t baud_rate, int read_timeout_ms = 50);

  // 关闭串口（幂等）。
  void close();

  // 是否已打开
  bool isOpen() const;

  // 拉一坨字节并解析为帧；callback 拿到每个完整帧。返回解析到的帧数。
  // read_timeout_ms 控制 read() 阻塞时长。
  size_t pollFrames(int read_timeout_ms,
                    const std::function<void(const protocol::RawFrame&)>& on_frame);

  // 同步发一帧（带锁，可重入）
  void sendFrame(const protocol::RawFrame& f);

  // 直接发序列化好的字节（speed command 用）
  void sendBytes(const std::vector<uint8_t>& bytes);

  // 串口路径（用于日志）
  const std::string& port() const { return port_; }

  // 已接收的字节数 / 解析成功的帧数 / 解析失败次数（仅做诊断）
  uint64_t bytesRead() const { return bytes_read_; }
  uint64_t framesOk() const { return frames_ok_; }
  uint64_t framesBad() const { return frames_bad_; }

 private:
  std::unique_ptr<serial::Serial> ser_;
  mutable std::mutex mu_;
  std::string port_;
  uint32_t baud_ = 115200;
  std::vector<uint8_t> rx_buf_;
  uint64_t bytes_read_ = 0;
  uint64_t frames_ok_ = 0;
  uint64_t frames_bad_ = 0;
};

}  // namespace leading_line_chassis::serialio
