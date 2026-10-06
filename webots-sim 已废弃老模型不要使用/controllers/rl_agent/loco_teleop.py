#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""yobogo_loco_jump_v1 本机键盘/手柄输入层。

本模块只负责把输入事件整理为 50 Hz 遥控命令，不依赖 Webots 设备类型，
因此可用 mock 键盘与内存事件流直接自测。速度限幅、死区、跳跃边沿均在
此处统一实现，控制器侧只消费 ``TeleopCommand``。
"""

from __future__ import annotations

import errno
import os
import struct
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, Iterable, Optional, Sequence, Tuple


# 键盘/手柄最终速度都必须裁剪到契约范围。
COMMAND_LIMITS: Dict[str, Tuple[float, float]] = {
    "vx": (-0.3, 0.6),
    "vy": (-0.3, 0.3),
    "wz": (-1.0, 1.0),
}

KEY_W = ord("w")
KEY_S = ord("s")
KEY_A = ord("a")
KEY_D = ord("d")
KEY_Q = ord("q")
KEY_E = ord("e")
KEY_R = ord("r")
KEY_SPACE = 32
KEY_ESC = 27

# Webots Keyboard 的修饰键位（与 Webots Keyboard.hpp 保持一致）。
KEY_MASK = 0x0000FFFF
KEY_SHIFT = 0x00010000

# 50 Hz 输入循环里，Webots 只提供“最近按键事件”。短时保持可避免一次采样后
# 立即归零，同时又能识别松键；Space/R 仍严格按上升沿触发。
KEY_HOLD_SECONDS = 0.16
GAMEPAD_DEADZONE = 0.12
GAMEPAD_PATH = "/dev/input/js0"

# Linux joystick 事件：u32 时间、s16 轴值、u8 类型、u8 轴/钮编号。
_JS_EVENT = struct.Struct("=IhBB")
JS_EVENT_AXIS = 0x01
JS_EVENT_BUTTON = 0x02
JS_EVENT_INIT = 0x80


def clip_command(vx: float, vy: float, wz: float) -> Tuple[float, float, float]:
    """把 [vx, vy, wz] 分量逐一裁剪到契约范围。"""
    return (
        min(COMMAND_LIMITS["vx"][1], max(COMMAND_LIMITS["vx"][0], float(vx))),
        min(COMMAND_LIMITS["vy"][1], max(COMMAND_LIMITS["vy"][0], float(vy))),
        min(COMMAND_LIMITS["wz"][1], max(COMMAND_LIMITS["wz"][0], float(wz))),
    )


def apply_deadzone(value: float, deadzone: float = GAMEPAD_DEADZONE) -> float:
    """应用摇杆死区；死区内归零，不缩放，避免小漂移。"""
    return 0.0 if abs(float(value)) < float(deadzone) else float(value)


@dataclass(frozen=True)
class TeleopCommand:
    """一帧 50 Hz 遥控命令。"""

    vx: float
    vy: float
    wz: float
    jump_edge: bool = False
    reset_edge: bool = False
    estop_edge: bool = False
    sprint: bool = False

    @property
    def velocity(self) -> Tuple[float, float, float]:
        return self.vx, self.vy, self.wz


class KeyboardReader:
    """把 Webots ``getKey()`` 事件流整理为持续按键与一次边沿。"""

    def __init__(
        self,
        keyboard: Any,
        *,
        clock: Callable[[], float] = time.monotonic,
        hold_seconds: float = KEY_HOLD_SECONDS,
    ) -> None:
        self._keyboard = keyboard
        self._clock = clock
        self._hold_seconds = float(hold_seconds)
        self._held: Dict[int, Tuple[float, int]] = {}
        self._sequence = 0
        self._edge_seen: Dict[int, bool] = {
            KEY_SPACE: False,
            KEY_R: False,
            KEY_ESC: False,
        }
        self._shift_until = 0.0

    def _active_codes(self, now: float) -> list[int]:
        expired = [code for code, value in self._held.items() if value[0] <= now]
        for code in expired:
            self._held.pop(code, None)
            if code in self._edge_seen:
                self._edge_seen[code] = False
        return [
            code
            for code, value in sorted(self._held.items(), key=lambda item: item[1][1])
            if value[0] > now
        ]

    def read(self) -> TeleopCommand:
        """读取当前采样周期所有按键并返回合并后的命令。"""
        now = self._clock()
        events: list[int] = []
        while True:
            key = self._keyboard.getKey()
            if key is None or key < 0:
                break
            events.append(int(key))
            # 限制单周期事件数，避免测试 mock 或异常驱动造成死循环。
            if len(events) >= 64:
                break

        jump_edge = False
        reset_edge = False
        estop_edge = False
        for raw_key in events:
            code = raw_key & KEY_MASK
            # Webots 在 Caps Lock/大小写状态下可能给出大写 ASCII，统一为小写。
            if ord("A") <= code <= ord("Z"):
                code += ord("a") - ord("A")
            if raw_key & KEY_SHIFT:
                self._shift_until = now + self._hold_seconds
            if code == 0:
                continue
            self._sequence += 1
            self._held[code] = (now + self._hold_seconds, self._sequence)
            if code in self._edge_seen and not self._edge_seen[code]:
                if code == KEY_SPACE:
                    jump_edge = True
                elif code == KEY_R:
                    reset_edge = True
                elif code == KEY_ESC:
                    estop_edge = True
                self._edge_seen[code] = True

        active = self._active_codes(now)
        active_set = set(active)

        # 同轴冲突时取最后采样键；不同轴可组合。
        if KEY_W in active_set and KEY_S in active_set:
            vx_key = max(
                (KEY_W, KEY_S),
                key=lambda code: self._held[code][1],
            )
        elif KEY_W in active_set:
            vx_key = KEY_W
        elif KEY_S in active_set:
            vx_key = KEY_S
        else:
            vx_key = 0
        if KEY_A in active_set and KEY_D in active_set:
            vy_key = max(
                (KEY_A, KEY_D),
                key=lambda code: self._held[code][1],
            )
        elif KEY_A in active_set:
            vy_key = KEY_A
        elif KEY_D in active_set:
            vy_key = KEY_D
        else:
            vy_key = 0
        if KEY_Q in active_set and KEY_E in active_set:
            wz_key = max(
                (KEY_Q, KEY_E),
                key=lambda code: self._held[code][1],
            )
        elif KEY_Q in active_set:
            wz_key = KEY_Q
        elif KEY_E in active_set:
            wz_key = KEY_E
        else:
            wz_key = 0

        vx = {KEY_W: 0.6, KEY_S: -0.3}.get(vx_key, 0.0)
        vy = {KEY_A: 0.3, KEY_D: -0.3}.get(vy_key, 0.0)
        wz = {KEY_Q: 1.0, KEY_E: -1.0}.get(wz_key, 0.0)
        vx, vy, wz = clip_command(vx, vy, wz)
        return TeleopCommand(
            vx=vx,
            vy=vy,
            wz=wz,
            jump_edge=jump_edge,
            reset_edge=reset_edge,
            estop_edge=estop_edge,
            sprint=now < self._shift_until,
        )


class JoystickReader:
    """Linux ``/dev/input/js0`` 非阻塞读取器，无第三方依赖。"""

    def __init__(
        self,
        path: str = GAMEPAD_PATH,
        *,
        deadzone: float = GAMEPAD_DEADZONE,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.path = path
        self.deadzone = float(deadzone)
        self._clock = clock
        self._fd: Optional[int] = None
        self._axes: Dict[int, float] = {}
        self._buttons: Dict[int, int] = {}
        self._jump_edge = False
        self._reset_edge = False
        self._estop_edge = False
        self.opened = False
        self.warning: Optional[str] = None

    def open(self) -> bool:
        """打开手柄；设备缺失不阻塞键盘路径，只记录一次警告。"""
        try:
            self._fd = os.open(self.path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as exc:
            if exc.errno not in (errno.ENOENT, errno.ENXIO, errno.EACCES):
                self.warning = f"打开手柄 {self.path} 失败: {exc}"
            else:
                self.warning = f"手柄 {self.path} 不可用，仅使用键盘"
            return False
        self.opened = True
        return True

    def close(self) -> None:
        if self._fd is not None:
            try:
                os.close(self._fd)
            finally:
                self._fd = None
                self.opened = False

    def inject_event(self, event_type: int, number: int, value: int) -> None:
        """测试或上游注入事件；与真实 Linux 事件走同一处理逻辑。"""
        self._handle(event_type, number, int(value))

    def _handle(self, event_type: int, number: int, value: int) -> None:
        clean_type = int(event_type) & ~JS_EVENT_INIT
        if clean_type == JS_EVENT_AXIS and number < 8:
            self._axes[number] = max(-1.0, min(1.0, value / 32767.0))
        elif clean_type == JS_EVENT_BUTTON and number < 12:
            previous = self._buttons.get(number, 0)
            current = 1 if value else 0
            self._buttons[number] = current
            if current and not previous:
                if number == 0:
                    self._jump_edge = True
                elif number == 1:
                    self._reset_edge = True
                elif number == 6:
                    self._estop_edge = True

    def _drain_fd(self) -> None:
        if self._fd is None:
            return
        while True:
            try:
                raw = os.read(self._fd, _JS_EVENT.size)
            except BlockingIOError:
                return
            except OSError as exc:
                if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    return
                self.warning = f"读取手柄 {self.path} 失败: {exc}"
                self.close()
                return
            if len(raw) < _JS_EVENT.size:
                return
            _timestamp, value, event_type, number = _JS_EVENT.unpack(raw)
            self._handle(event_type, number, value)

    def read(self) -> TeleopCommand:
        """返回手柄命令；轴值先过 0.12 死区，再映射并限幅。"""
        self._drain_fd()
        axis0 = apply_deadzone(self._axes.get(0, 0.0), self.deadzone)
        axis1 = apply_deadzone(self._axes.get(1, 0.0), self.deadzone)
        axis2 = apply_deadzone(self._axes.get(2, 0.0), self.deadzone)
        # 摇杆上推/右推分别映射为前进/负方向，和键盘 W/E 一致。
        vx, vy, wz = clip_command(-axis1 * 0.6, -axis0 * 0.3, -axis2 * 1.0)
        command = TeleopCommand(
            vx=vx,
            vy=vy,
            wz=wz,
            jump_edge=self._jump_edge,
            reset_edge=self._reset_edge,
            estop_edge=self._estop_edge,
        )
        self._jump_edge = False
        self._reset_edge = False
        self._estop_edge = False
        return command


class TeleopReader:
    """合并键盘与手柄：同分量键盘非零优先，边沿任一来源触发一次。"""

    def __init__(
        self,
        keyboard: Any = None,
        *,
        joystick_path: Optional[str] = GAMEPAD_PATH,
        clock: Callable[[], float] = time.monotonic,
        hold_seconds: float = KEY_HOLD_SECONDS,
    ) -> None:
        self.keyboard = KeyboardReader(
            keyboard, clock=clock, hold_seconds=hold_seconds
        ) if keyboard is not None else None
        self.joystick = JoystickReader(
            joystick_path or GAMEPAD_PATH, clock=clock
        )
        if joystick_path:
            self.joystick.open()

    @staticmethod
    def _prefer_keyboard(
        keyboard_value: float, joystick_value: float
    ) -> float:
        return keyboard_value if abs(keyboard_value) > 1e-12 else joystick_value

    def read(self) -> TeleopCommand:
        key_cmd = (
            self.keyboard.read()
            if self.keyboard is not None
            else TeleopCommand(0.0, 0.0, 0.0)
        )
        pad_cmd = self.joystick.read()
        vx, vy, wz = clip_command(
            self._prefer_keyboard(key_cmd.vx, pad_cmd.vx),
            self._prefer_keyboard(key_cmd.vy, pad_cmd.vy),
            self._prefer_keyboard(key_cmd.wz, pad_cmd.wz),
        )
        estop_edge = key_cmd.estop_edge or pad_cmd.estop_edge
        reset_edge = key_cmd.reset_edge or pad_cmd.reset_edge
        jump_edge = key_cmd.jump_edge or pad_cmd.jump_edge
        if estop_edge:
            vx = vy = wz = 0.0
            reset_edge = False
            jump_edge = False
        elif reset_edge:
            vx = vy = wz = 0.0
            jump_edge = False
        return TeleopCommand(
            vx=vx,
            vy=vy,
            wz=wz,
            jump_edge=jump_edge,
            reset_edge=reset_edge,
            estop_edge=estop_edge,
            sprint=key_cmd.sprint,
        )

    def close(self) -> None:
        self.joystick.close()


def merged_command(commands: Iterable[TeleopCommand]) -> TeleopCommand:
    """测试辅助：按“键盘非零优先、边沿或合并”合并多路命令。"""
    items: Sequence[TeleopCommand] = tuple(commands)
    if not items:
        return TeleopCommand(0.0, 0.0, 0.0)
    estop_edge = any(item.estop_edge for item in items)
    reset_edge = any(item.reset_edge for item in items)
    jump_edge = any(item.jump_edge for item in items)
    if estop_edge:
        return TeleopCommand(
            0.0, 0.0, 0.0, estop_edge=True, sprint=False
        )
    if reset_edge:
        return TeleopCommand(
            0.0, 0.0, 0.0, reset_edge=True, sprint=False
        )
    return TeleopCommand(
        *clip_command(
            next((c.vx for c in items if abs(c.vx) > 1e-12), items[0].vx),
            next((c.vy for c in items if abs(c.vy) > 1e-12), items[0].vy),
            next((c.wz for c in items if abs(c.wz) > 1e-12), items[0].wz),
        ),
        jump_edge=jump_edge,
        sprint=any(c.sprint for c in items),
    )
