# -*- coding: utf-8 -*-
"""双端通信消息格式。

文本命令（PC → Jetson，TCP，一行一条）::

    MODE blue_path
    START
    STOP
    PING

文本状态（Jetson → PC，TCP 响应/心跳）::

    STATUS running
    STATUS idle
    INFO <free text>

视频帧（Jetson → PC，UDP）::

    [4 字节大端长度 N][1 字节 seq][8 字节时间戳 ms][N 字节 JPEG bytes]
    总长 = 13 + N

为什么要用长度前缀：
- 摄像头帧按 JPEG 编码后大小不固定，UDP 是数据报，一次一包即可。
- 长度前缀让接收端能把"长度+负载"作为一个原子包读出来，避免粘包。
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Optional


# ---------- TCP 文本协议 ----------

@dataclass
class CmdMessage:
    """PC → Jetson 的命令。"""
    kind: str          # "MODE" / "START" / "STOP" / "PING" / ...
    payload: str = ""  # MODE 后面的模式名 / INFO 的内容

    def encode(self) -> str:
        return f"{self.kind} {self.payload}".rstrip() + "\n"


@dataclass
class StatusMessage:
    """Jetson → PC 的状态/文本回包。"""
    kind: str          # "STATUS" / "INFO"
    payload: str = ""

    def encode(self) -> str:
        return f"{self.kind} {self.payload}".rstrip() + "\n"


def encode_cmd(kind: str, payload: str = "") -> bytes:
    return CmdMessage(kind=kind, payload=payload).encode().encode("utf-8")


def encode_status(kind: str, payload: str = "") -> bytes:
    return StatusMessage(kind=kind, payload=payload).encode().encode("utf-8")


def parse_line(raw: bytes) -> Optional[CmdMessage]:
    """解析一行 TCP 文本命令；空行/无法解析时返回 None。"""
    try:
        s = raw.decode("utf-8", errors="ignore").strip()
    except Exception:
        return None
    if not s:
        return None
    parts = s.split(maxsplit=1)
    kind = parts[0].upper()
    payload = parts[1] if len(parts) > 1 else ""
    return CmdMessage(kind=kind, payload=payload)


# ---------- UDP 视频帧协议 ----------

_HEADER_FMT = "!I B Q"   # 长度(uint32) | seq(uint8) | ts_ms(uint64) — 13 字节
_HEADER_LEN = struct.calcsize(_HEADER_FMT)   # = 13


@dataclass
class VideoFrame:
    """一帧 JPEG 编码的视频。"""
    seq: int
    ts_ms: int
    jpeg: bytes

    def size_on_wire(self) -> int:
        return _HEADER_LEN + len(self.jpeg)


def encode_video_frame(seq: int, ts_ms: int, jpeg: bytes) -> bytes:
    """打包一帧：4 字节大端长度 + 1 字节 seq + 8 字节 ts + JPEG。"""
    if len(jpeg) > 0xFFFFFFFF:
        raise ValueError(f"JPEG 太大 ({len(jpeg)} bytes)，超出 4 字节长度上限")
    header = struct.pack(_HEADER_FMT, len(jpeg), seq & 0xFF, ts_ms & 0xFFFFFFFFFFFFFFFF)
    return header + jpeg


def decode_video_frame(packet: bytes) -> Optional[VideoFrame]:
    """解一帧；包不完整 / 长度不一致 → 返回 None。

    UDP 一般是单包，但偶尔会发生 IP 分片；这里要求包大小 == header 声明的负载。
    """
    if len(packet) < _HEADER_LEN:
        return None
    jpeg_len, seq, ts_ms = struct.unpack(_HEADER_FMT, packet[:_HEADER_LEN])
    if len(packet) != _HEADER_LEN + jpeg_len:
        return None
    return VideoFrame(seq=seq, ts_ms=ts_ms, jpeg=packet[_HEADER_LEN:])
