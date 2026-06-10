"""共享协议包：双端通信用的消息格式、命令、模式常量。

两端（jetson / pc）都通过该包解析消息，避免字符串协议散落各处。
"""
from .constants import (
    DEFAULT_PC_IP,
    DEFAULT_VIDEO_PORT,
    DEFAULT_CMD_PORT,
    MODE_BLUE,
    MODE_GREEN,
    MODE_TEST,
    ALL_MODES,
    STATE_IDLE,
    STATE_RUNNING,
    STATE_STOPPED,
    STATE_ERROR,
)
from .messages import (
    encode_cmd,
    parse_line,
    CmdMessage,
    StatusMessage,
    VideoFrame,
    encode_status,
    decode_video_frame,
    encode_video_frame,
)
from .mode_resolver import list_modes, select_mode

__all__ = [
    "DEFAULT_PC_IP",
    "DEFAULT_VIDEO_PORT",
    "DEFAULT_CMD_PORT",
    "MODE_BLUE",
    "MODE_GREEN",
    "MODE_TEST",
    "ALL_MODES",
    "STATE_IDLE",
    "STATE_RUNNING",
    "STATE_STOPPED",
    "STATE_ERROR",
    "encode_cmd",
    "parse_line",
    "CmdMessage",
    "StatusMessage",
    "VideoFrame",
    "encode_status",
    "decode_video_frame",
    "encode_video_frame",
    "list_modes",
    "select_mode",
]
