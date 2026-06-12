#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mock_serial.py — 虚拟 zonesion xcar 串口：注入假 odom/sonar/sensors 帧。

用法：
  socat -d -d pty,raw,echo=0 pty,raw,echo=0 &
  # 假设输出 /dev/ttyVCar0 ↔ /dev/ttyVCar1
  ros2 launch leading_line_chassis chassis.launch.py usart_port_name:=/dev/ttyVCar0
  python3 mock_serial.py /dev/ttyVCar1

  # 收到 0x81 速度命令时会回 0x01 里程计帧（vx = 命令里的 vx）
  # 同时按 ~5 Hz 发 0x03 环境帧（bat=11.5V）
"""
from __future__ import annotations

import argparse
import struct
import sys
import time

import serial


# zonesion 0x2B-A2 协议常量
HEADER = b"\x2B\xA2"
TYPE_ODOM = 0x01
TYPE_SONAR = 0x02
TYPE_SENSORS = 0x03
TYPE_CMD_SPEED = 0x81

last_vx_mmps = 0
last_wz_milli = 0
seq = 0


def bcc(buf: bytes) -> int:
    return sum(buf) & 0xFF


def pack_frame(seq: int, type_: int, data: bytes) -> bytes:
    body = HEADER + bytes([seq & 0xFF, type_, len(data)]) + data
    return body + bytes([bcc(body)])


def make_odom(vx_mmps: int, wz_milli: int) -> bytes:
    # pos_x, pos_y, yaw(1/10000 rad), speed_x(mm/s), speed_y(mm/s), wz(1/10000 rad/s)
    data = struct.pack(">iiihhh", 0, 0, 0, vx_mmps, 0, wz_milli)
    return pack_frame(0, TYPE_ODOM, data)


def make_sonar() -> bytes:
    data = struct.pack(">hhhh", 50, 60, 80, 100)  # 4 路距离 cm
    return pack_frame(0, TYPE_SONAR, data)


def make_sensors() -> bytes:
    # bat(uint8, 0.1V), temp(int16, 0.1C), humi(int16), pressure(int32, 0.1Pa),
    # light(int16), tvoc(int16), smoke(int16)
    data = struct.pack(">Bhhhihhh", 115, 230, 55, 10132, 200, 5, 3)
    return pack_frame(0, TYPE_SENSORS, data)


def parse_cmd_speed(frame: bytes) -> None:
    """解析 0x81 速度命令：data[0..5] = vx(int16 BE), vy(int16 BE), wz(int16 BE)"""
    global last_vx_mmps, last_wz_milli
    if len(frame) < 11:
        return
    if frame[3] != TYPE_CMD_SPEED:
        return
    vx = struct.unpack(">h", frame[5:7])[0]
    wz = struct.unpack(">h", frame[9:11])[0]
    last_vx_mmps = vx
    last_wz_milli = wz
    print(f"  RX 0x81: vx={vx/1000.0:+.3f} m/s  wz={wz/10000.0:+.3f} rad/s")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("port", help="虚拟串口路径（chassis 节点的对面）")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--hz", type=float, default=20.0)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.02)
    print(f"打开 mock 串口: {args.port} @ {args.baud}, 推送频率 {args.hz} Hz")
    print("按 Ctrl+C 退出")

    period = 1.0 / args.hz
    last_odom_t = 0.0
    last_sensors_t = 0.0

    try:
        while True:
            loop_t0 = time.monotonic()
            # 1) 读命令帧（如果有）
            if ser.in_waiting > 0:
                buf = ser.read(ser.in_waiting)
                # 简单搜索 0x2B 0xA2 0x81
                for i in range(len(buf) - 10):
                    if buf[i : i + 2] == HEADER and buf[i + 3] == TYPE_CMD_SPEED:
                        end = i + 11
                        if end <= len(buf) and buf[end - 1] == bcc(buf[i : end - 1]):
                            parse_cmd_speed(buf[i:end])

            now = time.monotonic()
            # 2) 50 Hz 发 odom（用最近一次命令的速度）
            if now - last_odom_t >= period:
                ser.write(make_odom(last_vx_mmps, last_wz_milli))
                last_odom_t = now
            # 3) 5 Hz 发 sensors
            if now - last_sensors_t >= 0.2:
                ser.write(make_sensors())
                last_sensors_t = now
            # 4) 5 Hz 发 sonar
            if int(now * 5) != int((now - period) * 5):
                ser.write(make_sonar())

            time.sleep(0.001)
    except KeyboardInterrupt:
        print("\n退出")
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
