// -*- coding: utf-8 -*-
#include "serial_driver.hpp"

#include <serial/serial.h>

#include <chrono>
#include <stdexcept>
#include <thread>

namespace leading_line_chassis::serialio {

SerialDriver::SerialDriver() = default;

SerialDriver::~SerialDriver() {
  try {
    close();
  } catch (...) {
    // 析构里不抛
  }
}

void SerialDriver::open(const std::string& port, uint32_t baud_rate, int read_timeout_ms) {
  std::lock_guard<std::mutex> lk(mu_);
  if (ser_ && ser_->isOpen()) {
    ser_->close();
  }
  ser_ = std::make_unique<serial::Serial>();
  ser_->setPort(port);
  ser_->setBaudrate(baud_rate);
  serial::Timeout t;
  t.read_timeout_constant = read_timeout_ms;
  t.read_timeout_multiplier = 0;
  t.write_timeout_constant = 50;
  t.write_timeout_multiplier = 0;
  ser_->setTimeout(t);
  try {
    ser_->open();
  } catch (const std::exception& e) {
    ser_.reset();
    throw std::runtime_error(std::string("SerialDriver::open(") + port + ") failed: " + e.what());
  }
  port_ = port;
  baud_ = baud_rate;
  rx_buf_.clear();
}

void SerialDriver::close() {
  std::lock_guard<std::mutex> lk(mu_);
  if (ser_ && ser_->isOpen()) ser_->close();
  ser_.reset();
  rx_buf_.clear();
}

bool SerialDriver::isOpen() const {
  std::lock_guard<std::mutex> lk(mu_);
  return ser_ && ser_->isOpen();
}

size_t SerialDriver::pollFrames(
    int read_timeout_ms,
    const std::function<void(const protocol::RawFrame&)>& on_frame) {
  std::lock_guard<std::mutex> lk(mu_);
  if (!ser_ || !ser_->isOpen()) return 0;
  size_t n_avail = 0;
  try {
    n_avail = ser_->available();
  } catch (const std::exception&) {
    return 0;
  }
  if (n_avail == 0) {
    // 仍尝试一次小读，方便驱动刷新内部缓冲
    try {
      std::vector<uint8_t> tmp(ser_->read(0));
      if (!tmp.empty()) {
        rx_buf_.insert(rx_buf_.end(), tmp.begin(), tmp.end());
        bytes_read_ += tmp.size();
      }
    } catch (...) {
    }
    return 0;
  }
  try {
    auto chunk = ser_->read(n_avail);
    if (!chunk.empty()) {
      rx_buf_.insert(rx_buf_.end(), chunk.begin(), chunk.end());
      bytes_read_ += chunk.size();
    }
  } catch (const std::exception&) {
    return 0;
  }

  // 边解析边丢坏字节
  size_t parsed = 0;
  while (true) {
    protocol::RawFrame f;
    if (protocol::tryParseFrame(rx_buf_, f)) {
      ++frames_ok_;
      ++parsed;
      on_frame(f);
    } else {
      break;
    }
  }
  // 统计"被丢"的字节：rx_buf 残余 + 已 erase 的字节里含坏字节，
  // 这里简化为：本次循环若解析为 0 且 buffer 在涨，就 +bad（粗略）
  return parsed;
}

void SerialDriver::sendFrame(const protocol::RawFrame& f) {
  sendBytes(protocol::serializeFrame(f));
}

void SerialDriver::sendBytes(const std::vector<uint8_t>& bytes) {
  std::lock_guard<std::mutex> lk(mu_);
  if (!ser_ || !ser_->isOpen()) return;
  try {
    ser_->write(bytes);
  } catch (const std::exception&) {
    // 写失败一般意味着线缆拔了；上层有重连逻辑
  }
}

}  // namespace leading_line_chassis::serialio
