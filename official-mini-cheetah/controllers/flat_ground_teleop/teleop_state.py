#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""站立/行走模式状态机。

A 键冲突采用可测试的“模式优先 + 一帧时序”规则：

1. 站立态按 A 的上升沿进入行走，该帧不产生横移；
2. A 继续保持时，从下一帧开始按行走态左移；
3. 行走态按 A 只左移，不退出行走；退出行走统一使用 Space、R 或 Esc。
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Optional

from teleop_types import ControlMode, TeleopCommand


INPUT_TIMEOUT_SECONDS = 0.35


@dataclass(frozen=True)
class StateUpdate:
    """状态机一帧输出。"""

    mode: ControlMode
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    reason: str = ""
    changed: bool = False
    safety_reset: bool = False

    @property
    def velocity(self) -> tuple[float, float, float]:
        return self.vx, self.vy, self.wz


class TeleopStateMachine:
    """维护默认站立、模式切换、A 冲突和输入超时安全规则。"""

    def __init__(
        self,
        *,
        clock: Any = time.monotonic,
        input_timeout: float = INPUT_TIMEOUT_SECONDS,
    ) -> None:
        self._clock = clock
        self._input_timeout = max(0.0, float(input_timeout))
        self.mode = ControlMode.STANDING
        self._last_motion_at = self._clock()

    def reset(self, now: Optional[float] = None) -> StateUpdate:
        """回站立并清空模式/输入时序状态。"""

        timestamp = self._clock() if now is None else float(now)
        changed = self.mode != ControlMode.STANDING
        self.mode = ControlMode.STANDING
        self._last_motion_at = timestamp
        return StateUpdate(
            mode=self.mode,
            reason="reset",
            changed=changed,
            safety_reset=True,
        )

    def update(
        self,
        command: TeleopCommand,
        now: Optional[float] = None,
    ) -> StateUpdate:
        """应用一帧输入并返回模式、速度和安全原因。"""

        timestamp = self._clock() if now is None else float(now)
        previous_mode = self.mode
        motion_active = bool(command.motion_active or not command.zero)
        if motion_active:
            self._last_motion_at = timestamp

        # 安全项优先于所有模式/运动输入。
        if command.force_stand or not command.valid:
            reason = command.reason or "input_invalid"
            self.reset(timestamp)
            return StateUpdate(
                mode=self.mode,
                reason=reason,
                changed=previous_mode != self.mode,
                safety_reset=True,
            )
        if command.reset_edge:
            self.reset(timestamp)
            return StateUpdate(
                mode=self.mode,
                reason="reset",
                changed=previous_mode != self.mode,
                safety_reset=True,
            )
        if (
            self.mode == ControlMode.WALKING
            and not motion_active
            and timestamp - self._last_motion_at > self._input_timeout
        ):
            self.reset(timestamp)
            return StateUpdate(
                mode=self.mode,
                reason="input_timeout",
                changed=previous_mode != self.mode,
                safety_reset=True,
            )

        # Space/手柄 A 钮始终切换模式，优先级高于键盘 A 的移动语义。
        if command.mode_toggle_edge:
            self.mode = (
                ControlMode.WALKING
                if self.mode == ControlMode.STANDING
                else ControlMode.STANDING
            )
            self._last_motion_at = timestamp
            return StateUpdate(
                mode=self.mode,
                reason="mode_toggle",
                changed=previous_mode != self.mode,
            )

        if self.mode == ControlMode.STANDING:
            if command.a_edge:
                self.mode = ControlMode.WALKING
                self._last_motion_at = timestamp
                return StateUpdate(
                    mode=self.mode,
                    reason="a_enter_walking",
                    changed=True,
                )
            # 站立态即使收到 W/S/D/Q/E 或手柄轴也保持全零。
            return StateUpdate(
                mode=self.mode,
                reason="standing_hold",
                changed=False,
            )

        vx, vy, wz = command.velocity
        if command.a_edge:
            reason = "a_lateral"
        else:
            reason = "walking"
        return StateUpdate(
            mode=self.mode,
            vx=vx,
            vy=vy,
            wz=wz,
            reason=reason,
            changed=False,
        )


def is_safe_mode(mode: Any) -> bool:
    """判断模式值是否为可接受的安全枚举。"""

    try:
        return ControlMode(mode) == ControlMode.STANDING
    except (TypeError, ValueError):
        return False
