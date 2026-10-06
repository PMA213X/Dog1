#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""键盘与 Webots Joystick 输入适配。

模块不导入 Webots 包，只约定 Webots ``Keyboard.getKey()`` 与
``Joystick.getAxisValue()`` 的调用接口，因此纯逻辑可以直接单测。
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Dict, Iterable, Optional, Sequence, Tuple

from teleop_types import TeleopCommand, clip_command, finite_or_zero


KEY_MASK = 0x0000FFFF
KEY_SHIFT = 0x00010000
KEY_CONTROL = 0x00020000
KEY_ALT = 0x00040000

KEY_W = ord("w")
KEY_S = ord("s")
KEY_A = ord("a")
KEY_D = ord("d")
KEY_Q = ord("q")
KEY_E = ord("e")
KEY_R = ord("r")
KEY_SPACE = ord(" ")
KEY_ESCAPE = 27

MOTION_KEYS = frozenset((KEY_W, KEY_S, KEY_A, KEY_D, KEY_Q, KEY_E))
KEY_HOLD_SECONDS = 0.16
INPUT_TIMEOUT_SECONDS = 0.35
JOYSTICK_DEADZONE = 0.12
JOYSTICK_AXIS_SCALE = 32767.0
MAX_KEYS_PER_UPDATE = 64


def normalize_key(raw_key: int) -> int:
    """去掉 Webots 修饰位，并把字母统一成小写。"""

    try:
        code = int(raw_key) & KEY_MASK
    except (TypeError, ValueError):
        return 0
    if ord("A") <= code <= ord("Z"):
        code += ord("a") - ord("A")
    return code


def normalize_joystick_axis(value: Any) -> Optional[float]:
    """把 Webots 的 -32768..32767 轴值归一化为 -1..1。

    同时接受测试或适配层已经归一化的 -1..1 值。任何 NaN/Inf 返回 ``None``，
    供上层立即进入安全站立。
    """

    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    if -1.0 <= number <= 1.0:
        normalized = number
    else:
        normalized = number / JOYSTICK_AXIS_SCALE
    return max(-1.0, min(1.0, normalized))


def apply_deadzone(value: float, deadzone: float = JOYSTICK_DEADZONE) -> float:
    """死区内归零，避免手柄静止漂移。"""

    number = finite_or_zero(value)
    return 0.0 if abs(number) < float(deadzone) else number


@dataclass(frozen=True)
class KeyboardFrame:
    """一帧键盘状态。"""

    active_codes: frozenset[int] = frozenset()
    rising_edges: frozenset[int] = frozenset()
    events: Tuple[int, ...] = ()
    latest_sequence: Tuple[Tuple[int, int], ...] = ()

    def active(self, code: int) -> bool:
        return code in self.active_codes

    def rising(self, code: int) -> bool:
        return code in self.rising_edges

    def latest_key(self, codes: Iterable[int]) -> int:
        """同一轴冲突时返回最近采样的仍有效键。"""

        sequence = dict(self.latest_sequence)
        candidates = [code for code in codes if code in self.active_codes]
        return max(candidates, key=lambda code: sequence.get(code, -1)) if candidates else 0


class KeyboardState:
    """把 Webots 键盘事件流整理为短保持、上升沿和同轴冲突结果。"""

    def __init__(
        self,
        *,
        clock: Any = time.monotonic,
        hold_seconds: float = KEY_HOLD_SECONDS,
    ) -> None:
        self._clock = clock
        self._hold_seconds = float(hold_seconds)
        self._held: Dict[int, Tuple[float, int]] = {}
        self._sequence = 0

    def clear(self) -> None:
        """清空全部按键状态，用于 R/故障复位。"""

        self._held.clear()

    def update(self, events: Iterable[int], now: Optional[float] = None) -> KeyboardFrame:
        """吸收本周期事件并返回当前有效键。"""

        timestamp = self._clock() if now is None else float(now)
        event_items = tuple(events)
        expired = [code for code, value in self._held.items() if value[0] <= timestamp]
        for code in expired:
            self._held.pop(code, None)
        previous = set(self._held)
        for raw_key in event_items[:MAX_KEYS_PER_UPDATE]:
            code = normalize_key(raw_key)
            if code == 0:
                continue
            self._sequence += 1
            self._held[code] = (timestamp + self._hold_seconds, self._sequence)

        active = frozenset(self._held)
        rising = frozenset(code for code in active if code not in previous)
        sequence = tuple(sorted((code, value[1]) for code, value in self._held.items()))
        return KeyboardFrame(
            active_codes=active,
            rising_edges=rising,
            events=tuple(normalize_key(key) for key in event_items if normalize_key(key)),
            latest_sequence=sequence,
        )


@dataclass(frozen=True)
class JoystickSample:
    """Webots Joystick 的一帧采样。"""

    connected: bool = False
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    mode_toggle_edge: bool = False
    reset_edge: bool = False
    disconnected_edge: bool = False
    valid: bool = True
    warning: str = ""

    @property
    def velocity(self) -> Tuple[float, float, float]:
        return self.vx, self.vy, self.wz


class WebotsJoystickAdapter:
    """轮询 Webots Joystick，并识别断连与按钮上升沿。

    映射沿用项目遥控约定：轴 0 为 vy、轴 1 为 vx（上推为前）、轴 2 为 wz；
    钮 0 请求模式切换，钮 1 请求复位。
    """

    def __init__(self) -> None:
        self._previous_connected = False
        self._previous_buttons: set[int] = set()
        self._last_warning = ""

    def update(self, joystick: Any) -> JoystickSample:
        if joystick is None:
            disconnected = self._previous_connected
            self._previous_connected = False
            self._previous_buttons.clear()
            return JoystickSample(disconnected_edge=disconnected)

        try:
            connected = bool(joystick.isConnected())
        except Exception as exc:  # Webots 设备异常不得传播到主循环
            connected = False
            self._last_warning = f"Joystick.isConnected 失败: {exc}"

        if not connected:
            disconnected = self._previous_connected
            self._previous_connected = False
            self._previous_buttons.clear()
            return JoystickSample(
                disconnected_edge=disconnected,
                valid=True,
                warning=self._last_warning,
            )

        try:
            axis_count_getter = getattr(joystick, "getNumberOfAxes", None)
            if callable(axis_count_getter) and int(axis_count_getter()) < 3:
                raise ValueError("Joystick 轴数少于 3")
            raw_axes = [joystick.getAxisValue(index) for index in range(3)]
            normalized = [normalize_joystick_axis(value) for value in raw_axes]
            buttons = []
            for _ in range(16):
                pressed = int(joystick.getPressedButton())
                if pressed < 0:
                    break
                buttons.append(pressed)
        except Exception as exc:
            self._last_warning = f"Joystick 读取失败: {exc}"
            self._previous_connected = True
            self._previous_buttons.clear()
            return JoystickSample(
                connected=True,
                valid=False,
                warning=self._last_warning,
            )

        if any(value is None for value in normalized):
            self._previous_connected = True
            return JoystickSample(
                connected=True,
                valid=False,
                warning="Joystick 轴值包含 NaN/Inf",
            )

        axis0 = apply_deadzone(normalized[0] or 0.0)
        axis1 = apply_deadzone(normalized[1] or 0.0)
        axis2 = apply_deadzone(normalized[2] or 0.0)
        vx, vy, wz = clip_command(-axis1 * 0.6, axis0 * 0.3, axis2 * 1.0)
        current_buttons = set(buttons)
        new_buttons = current_buttons - self._previous_buttons
        sample = JoystickSample(
            connected=True,
            vx=vx,
            vy=vy,
            wz=wz,
            mode_toggle_edge=0 in new_buttons,
            reset_edge=1 in new_buttons,
            valid=True,
            warning=self._last_warning,
        )
        self._previous_connected = True
        self._previous_buttons = current_buttons
        return sample


def merge_commands(
    keyboard: Optional[TeleopCommand],
    joystick: Optional[TeleopCommand],
) -> TeleopCommand:
    """合并键盘和手柄命令。

    同一速度分量采用“键盘非零优先”，边沿按逻辑或合并；任一来源无效时
    强制安全站立。
    """

    key_cmd = keyboard or TeleopCommand()
    pad_cmd = joystick or TeleopCommand()
    valid = key_cmd.valid and pad_cmd.valid
    reason = key_cmd.reason if not key_cmd.valid else pad_cmd.reason
    vx, vy, wz = clip_command(
        key_cmd.vx if abs(key_cmd.vx) > 1e-12 else pad_cmd.vx,
        key_cmd.vy if abs(key_cmd.vy) > 1e-12 else pad_cmd.vy,
        key_cmd.wz if abs(key_cmd.wz) > 1e-12 else pad_cmd.wz,
    )
    return TeleopCommand(
        vx=vx,
        vy=vy,
        wz=wz,
        mode_toggle_edge=key_cmd.mode_toggle_edge or pad_cmd.mode_toggle_edge,
        a_edge=key_cmd.a_edge,
        reset_edge=key_cmd.reset_edge or pad_cmd.reset_edge,
        force_stand=(
            key_cmd.force_stand
            or pad_cmd.force_stand
            or not valid
        ),
        valid=valid,
        reason=reason,
        motion_active=key_cmd.motion_active or pad_cmd.motion_active,
    )


def read_webots_keyboard(keyboard: Any, limit: int = MAX_KEYS_PER_UPDATE) -> Tuple[int, ...]:
    """非阻塞读空 Webots 键盘事件队列，不引入启动等待。"""

    if keyboard is None:
        return ()
    events = []
    for _ in range(max(1, int(limit))):
        try:
            key = keyboard.getKey()
        except Exception:
            break
        if key is None or int(key) < 0:
            break
        events.append(int(key))
    return tuple(events)
