# -*- coding: utf-8 -*-
"""策略解析：把 QR 文本解成 Policy 对象。

支持两种文本格式：
  1) JSON：{"policy": "TURN_LEFT", "steer_deg": -20, "speed": 0.2, "duration_s": 1.5}
  2) 简单 key=value 分号分隔：policy=TURN_LEFT;steer_deg=-20;speed=0.2;duration_s=1.5

policy 字段是必填；其他字段缺失时按缺省值处理：
  steer_deg   = 0.0
  speed       = 0.0
  duration_s  = 0.0  （0 表示"一次性下发、不限时"）
  params      = {}   （自由扩展字段，原样保留）
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field, asdict
from typing import Any, Optional


# 已知策略白名单（不在这里的状态机拒绝执行）。
# 想加新策略：这里加一行，并在 state_machine 里写具体动作。
KNOWN_POLICIES: set[str] = {
    "STOP",            # 立刻停车（speed=0，steer=0）
    "CRUISE",          # 恢复自动巡航（解绑策略，由引导线算法接管）
    "TURN_LEFT",       # 左转指定角度
    "TURN_RIGHT",      # 右转指定角度
    "SLOW_DOWN",       # 减速
    "SPEED_UP",        # 加速
    "REVERSE",         # 倒车
    "HOLD",            # 保持当前 steer/speed，不接管
    "CUSTOM",          # 用户自定义；params 透传
}


@dataclass
class Policy:
    """执行单元。状态机拿到它就按 duration_s 跑完一轮。"""
    name: str                                  # 策略名，对应 KNOWN_POLICIES
    steer_deg: float = 0.0                     # 转向角
    speed: float = 0.0                         # 车速
    duration_s: float = 0.0                    # 持续时间（0 = 一次性）
    params: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return asdict(self)


class PolicyParseError(ValueError):
    """策略文本解析失败。状态机收到这个会上报并切 ERROR。"""


def _from_kv_text(text: str) -> dict[str, str]:
    """把 'a=1;b=2' 拆成 {'a': '1', 'b': '2'}，去空白。"""
    out: dict[str, str] = {}
    for part in text.split(";"):
        part = part.strip()
        if not part:
            continue
        if "=" not in part:
            raise PolicyParseError(f"无法解析片段：{part!r}（缺少 '='）")
        k, v = part.split("=", 1)
        out[k.strip()] = v.strip()
    return out


def _coerce(kv: dict[str, str]) -> dict[str, Any]:
    """把字符串字段类型还原成 float / dict 等。"""
    out: dict[str, Any] = {}
    for k, v in kv.items():
        if k == "params":
            # params=key1:val1,key2:val2 → 简单扁平 dict
            params: dict[str, str] = {}
            for p in v.split(","):
                if ":" in p:
                    pk, pv = p.split(":", 1)
                    params[pk.strip()] = pv.strip()
            out[k] = params
        elif k == "policy" or k == "name":
            out[k] = v
        else:
            try:
                out[k] = float(v) if ("." in v or "e" in v.lower()) else int(v)
            except ValueError:
                out[k] = v
    return out


def parse_policy(text: str) -> Policy:
    """从 QR 文本解析 Policy。失败抛 PolicyParseError。"""
    if not text or not text.strip():
        raise PolicyParseError("QR 文本为空")

    s = text.strip()
    # 1) 试 JSON
    if s.startswith("{"):
        try:
            data = json.loads(s)
        except json.JSONDecodeError as e:
            raise PolicyParseError(f"JSON 解析失败：{e}") from e
        if not isinstance(data, dict):
            raise PolicyParseError("JSON 顶层必须是对象")
        raw = data
    else:
        # 2) 试 key=value 形式
        try:
            raw = _coerce(_from_kv_text(s))
        except PolicyParseError:
            raise

    name = raw.get("policy") or raw.get("name")
    if not name or not isinstance(name, str):
        raise PolicyParseError("缺少 'policy' 字段")
    if name not in KNOWN_POLICIES:
        raise PolicyParseError(
            f"未知策略：{name!r}（允许：{sorted(KNOWN_POLICIES)}）"
        )

    def _f(key: str, default: float = 0.0) -> float:
        v = raw.get(key, default)
        try:
            return float(v)
        except (TypeError, ValueError):
            return default

    return Policy(
        name=name,
        steer_deg=_f("steer_deg"),
        speed=_f("speed"),
        duration_s=_f("duration_s"),
        params=dict(raw.get("params") or {}),
    )


def policy_to_text(p: Policy) -> str:
    """Policy → 简化文本（方便日志打印 / 写回 QR）。"""
    return (
        f"policy={p.name};"
        f"steer_deg={p.steer_deg:.2f};"
        f"speed={p.speed:.2f};"
        f"duration_s={p.duration_s:.2f}"
    )
