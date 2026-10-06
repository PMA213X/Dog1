#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TCP JSON Lines 消息的纯 Python 校验。"""

from __future__ import annotations

import json
import math
from typing import Any, Mapping, Sequence


def encode(payload: Mapping[str, Any]) -> bytes:
    """编码为单行 UTF-8 JSON。"""
    data = json.dumps(payload, ensure_ascii=True, separators=(",", ":"))
    if "\n" in data:
        raise ValueError("消息不能包含换行")
    return data.encode("utf-8") + b"\n"


def decode(line: bytes) -> dict[str, Any]:
    """解码并要求结果为对象。"""
    value = json.loads(line.decode("utf-8"))
    if not isinstance(value, dict):
        raise ValueError("TCP 消息必须是对象")
    return value


def valid_vector(values: Sequence[Any], length: int) -> bool:
    """检查定长有限向量。"""
    if len(values) != length:
        return False
    try:
        return all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError):
        return False


def validate_message(payload: Mapping[str, Any], expected_type: str) -> None:
    """校验环境发给 controller 的消息。"""
    if payload.get("type") != expected_type:
        raise ValueError(f"消息类型错误：{payload.get('type')}")
    if expected_type in {"reset", "act"}:
        if not valid_vector(payload.get("command", []), 3):
            raise ValueError("命令必须是 3 个有限数")
    if expected_type == "act":
        if not valid_vector(payload.get("action", []), 12):
            raise ValueError("动作必须是 12 个有限数")
        if not isinstance(payload.get("jump_request"), bool):
            raise ValueError("jump_request 必须是布尔值")
