#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""主动遥控公共类型。

本模块只使用标准库，导入时不需要 Webots 的 ``controller`` 包，便于在
普通 Python 环境中测试输入与状态机。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Tuple


class ControlMode(str, Enum):
    """机器狗控制模式。"""

    STANDING = "standing"
    WALKING = "walking"


# 便于测试和上层代码使用同义名称。
RobotMode = ControlMode


COMMAND_LIMITS: Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]] = (
    (-0.30, 0.60),  # vx：前进为正
    (-0.30, 0.30),  # vy：左移为正
    (-1.00, 1.00),  # wz：左转为正
)


def finite_or_zero(value: float) -> float:
    """把 NaN/Inf 输入转换成零，避免异常数值进入控制链。"""

    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0
    return number if math.isfinite(number) else 0.0


def clip_command(vx: float, vy: float, wz: float) -> Tuple[float, float, float]:
    """对 [vx, vy, wz] 做有限值检查并裁剪到安全遥控范围。"""

    values = (finite_or_zero(vx), finite_or_zero(vy), finite_or_zero(wz))
    return tuple(
        min(limits[1], max(limits[0], value))
        for value, limits in zip(values, COMMAND_LIMITS)
    )  # type: ignore[return-value]


@dataclass(frozen=True)
class TeleopCommand:
    """一帧键盘/手柄合并后的遥控输入。

    ``mode_toggle_edge`` 表示 Space 或手柄 A 钮的切换请求；``a_edge`` 专指
    键盘 A 的上升沿，用于处理“站立时进入行走、行走时横移”的冲突。
    """

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    mode_toggle_edge: bool = False
    a_edge: bool = False
    reset_edge: bool = False
    force_stand: bool = False
    valid: bool = True
    reason: str = ""
    motion_active: bool = False

    def __post_init__(self) -> None:
        raw_values = (self.vx, self.vy, self.wz)
        try:
            finite = all(math.isfinite(float(value)) for value in raw_values)
        except (TypeError, ValueError):
            finite = False
        safe = clip_command(self.vx, self.vy, self.wz)
        object.__setattr__(self, "vx", safe[0])
        object.__setattr__(self, "vy", safe[1])
        object.__setattr__(self, "wz", safe[2])
        if not finite:
            object.__setattr__(self, "valid", False)
            object.__setattr__(self, "force_stand", True)
            object.__setattr__(self, "reason", self.reason or "nonfinite_command")

    @property
    def velocity(self) -> Tuple[float, float, float]:
        """返回统一的 [vx, vy, wz] 命令。"""

        return self.vx, self.vy, self.wz

    @property
    def zero(self) -> bool:
        """判断速度命令是否为空。"""

        return abs(self.vx) < 1e-12 and abs(self.vy) < 1e-12 and abs(self.wz) < 1e-12


Velocity3 = Tuple[float, float, float]
