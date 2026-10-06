#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RL play 模式的键盘与手柄安全输入适配器。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence


INPUT_TIMEOUT_SECONDS = 0.35
JOYSTICK_DEADZONE = 0.12


@dataclass(frozen=True)
class InputFrame:
    """一次输入采样的命令和边沿。"""

    command: tuple[float, float, float] = (0.0, 0.0, 0.0)
    jump_edge: bool = False
    reset_edge: bool = False
    safe: bool = False
    reason: str = ""


def _finite(values: Sequence[float]) -> bool:
    try:
        return all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError):
        return False


def _axis(value: float) -> float:
    if not math.isfinite(value):
        return 0.0
    if abs(value) < JOYSTICK_DEADZONE:
        return 0.0
    return max(-1.0, min(1.0, value))


class InputAdapter:
    """合并键盘优先、手柄后备的命令，并维护跳跃/复位边沿。"""

    def __init__(self) -> None:
        self._previous_space = False
        self._previous_button_a = False
        self._previous_button_b = False

    def update(
        self,
        *,
        keys: Sequence[int],
        axes: Sequence[float],
        buttons: Sequence[int],
        now: float,
        last_input_at: float,
        disconnected: bool = False,
    ) -> InputFrame:
        """返回限幅命令；非有限、断连或超时直接进入安全态。"""
        codes = tuple(int(value) for value in keys)
        axis_values = tuple(float(value) for value in axes)
        button_values = tuple(int(value) for value in buttons)
        if disconnected or not _finite(axis_values) or not _finite(button_values):
            self._remember_edges(codes, button_values)
            return InputFrame(safe=True, reason="input_invalid")
        if now - last_input_at > INPUT_TIMEOUT_SECONDS:
            self._remember_edges(codes, button_values)
            return InputFrame(safe=True, reason="input_timeout")

        vx = 0.0
        if ord("w") in codes:
            vx += 0.6
        if ord("s") in codes:
            vx -= 0.3
        vy = 0.0
        if ord("a") in codes:
            vy += 0.3
        if ord("d") in codes:
            vy -= 0.3
        wz = 0.0
        if ord("q") in codes:
            wz += 1.0
        if ord("e") in codes:
            wz -= 1.0
        if vx == 0.0 and vy == 0.0 and wz == 0.0 and len(axis_values) >= 3:
            vx = _axis(axis_values[1]) * 0.6
            vy = _axis(axis_values[0]) * 0.3
            wz = _axis(axis_values[2])

        space = ord(" ") in codes
        button_a = bool(button_values[0]) if button_values else False
        button_b = bool(button_values[1]) if len(button_values) > 1 else False
        reset_key = ord("r") in codes
        escape = 0x0100001B in codes or 27 in codes
        jump_edge = (space and not self._previous_space) or (
            button_a and not self._previous_button_a
        )
        reset_edge = (reset_key or (button_b and not self._previous_button_b))
        safe = escape or reset_edge
        reason = "escape" if escape else ("reset" if reset_edge else "")
        frame = InputFrame(
            command=(vx, vy, wz),
            jump_edge=jump_edge and not safe,
            reset_edge=reset_edge,
            safe=safe,
            reason=reason,
        )
        self._remember_edges(codes, button_values)
        return frame

    def _remember_edges(self, codes: Sequence[int], buttons: Sequence[int]) -> None:
        self._previous_space = ord(" ") in codes
        self._previous_button_a = bool(buttons[0]) if buttons else False
        self._previous_button_b = bool(buttons[1]) if len(buttons) > 1 else False
