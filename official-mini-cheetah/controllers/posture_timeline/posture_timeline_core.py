#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""官方 Mini Cheetah 姿态时间线、限幅与几何自检纯模块。

本模块不导入 Webots ``controller`` 包，单元测试可直接在普通 Python 环境
导入。姿态常量和变化率限制统一来自主动控制模块的 ``joint_safety``。
"""

from __future__ import annotations

import math
import sys
from enum import Enum
from pathlib import Path
from typing import Dict, Sequence, Tuple


MODULE_DIR = Path(__file__).resolve().parent
ACTIVE_CONTROLLER_DIR = MODULE_DIR.parent / "flat_ground_teleop"
if str(ACTIVE_CONTROLLER_DIR) not in sys.path:
    sys.path.insert(0, str(ACTIVE_CONTROLLER_DIR))

from joint_safety import (  # noqa: E402
    DEFAULT_CROUCH,
    FLAT_MOTOR_NAMES,
    FLAT_SENSOR_NAMES,
    JOINT_COUNT,
    MAX_TARGET_RATE,
    SLIGHTLY_EXTENDED,
    TARGET_POSITION_LIMITS,
    JointTargets,
    TargetRateLimiter,
    clamp_joint_targets,
    limit_target_rate,
)


DURATION_SECONDS = 16.0
DEFAULT_END_SECONDS = 10.0
EXTENDED_START_SECONDS = 12.0
RETURN_START_SECONDS = 14.0
RETURN_END_SECONDS = 16.0
TRANSITION_SECONDS = 2.0

# 官方 Mini Cheetah 几何：机身半长 0.19 m、abad 侧向偏置 0.062 m、
# hip/knee 连杆分别为 0.209/0.195 m， toe 碰撞球半径 0.015 m。
BODY_HALF_LENGTH = 0.19
BODY_HALF_WIDTH = 0.049
ABAD_LINK_LENGTH = 0.062
HIP_LINK_LENGTH = 0.209
KNEE_LINK_LENGTH = 0.195
TOE_RADIUS = 0.015
SUPPORT_HALF_LENGTH = BODY_HALF_LENGTH + TOE_RADIUS

MOUNTS: Dict[str, Tuple[float, float]] = {
    "fr": (0.19, -0.049),
    "fl": (0.19, 0.049),
    "hr": (-0.19, -0.049),
    "hl": (-0.19, 0.049),
}
SIDE_OFFSETS: Dict[str, float] = {
    "fr": -ABAD_LINK_LENGTH,
    "fl": ABAD_LINK_LENGTH,
    "hr": -ABAD_LINK_LENGTH,
    "hl": ABAD_LINK_LENGTH,
}


class PosturePhase(str, Enum):
    """固定时间线的状态阶段。"""

    DEFAULT_CROUCH = "default_crouch"
    TO_SLIGHTLY_EXTENDED = "to_slightly_extended"
    SLIGHTLY_EXTENDED = "slightly_extended"
    RETURN_TO_DEFAULT = "return_to_default"


def _finite_time(value: float) -> float:
    """把时间转换为有限数；异常时间回退到默认姿态阶段。"""

    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0
    return number if math.isfinite(number) else 0.0


def phase_at(time_seconds: float) -> PosturePhase:
    """返回固定时间线在指定时刻的状态阶段。"""

    now = _finite_time(time_seconds)
    if now < 0.0 or now < DEFAULT_END_SECONDS:
        return PosturePhase.DEFAULT_CROUCH
    if now < EXTENDED_START_SECONDS:
        return PosturePhase.TO_SLIGHTLY_EXTENDED
    if now < RETURN_START_SECONDS:
        return PosturePhase.SLIGHTLY_EXTENDED
    if now < RETURN_END_SECONDS:
        return PosturePhase.RETURN_TO_DEFAULT
    return PosturePhase.DEFAULT_CROUCH


def _smoothstep(value: float) -> float:
    """使用 C1 连续 smoothstep，保证边界一阶导为零。"""

    bounded = max(0.0, min(1.0, value))
    return bounded * bounded * (3.0 - 2.0 * bounded)


def _interpolate(
    start: Sequence[float],
    end: Sequence[float],
    progress: float,
) -> Tuple[float, ...]:
    """在两个 12 路姿态之间做有限值平滑插值。"""

    begin = clamp_joint_targets(start)
    target = clamp_joint_targets(end)
    alpha = _smoothstep(progress)
    return tuple(
        begin[index] + (target[index] - begin[index]) * alpha
        for index in range(JOINT_COUNT)
    )


def target_at(time_seconds: float) -> Tuple[float, ...]:
    """返回指定时刻的 12 路精确时间线目标。"""

    now = _finite_time(time_seconds)
    phase = phase_at(now)
    if phase == PosturePhase.DEFAULT_CROUCH:
        return DEFAULT_CROUCH
    if phase == PosturePhase.SLIGHTLY_EXTENDED:
        return SLIGHTLY_EXTENDED
    if phase == PosturePhase.TO_SLIGHTLY_EXTENDED:
        progress = (now - DEFAULT_END_SECONDS) / TRANSITION_SECONDS
        return _interpolate(DEFAULT_CROUCH, SLIGHTLY_EXTENDED, progress)
    progress = (now - RETURN_START_SECONDS) / TRANSITION_SECONDS
    return _interpolate(SLIGHTLY_EXTENDED, DEFAULT_CROUCH, progress)


def target_velocity_at(time_seconds: float) -> Tuple[float, ...]:
    """返回时间线精确目标的解析变化率；边界处为零。"""

    now = _finite_time(time_seconds)
    phase = phase_at(now)
    if phase not in (
        PosturePhase.TO_SLIGHTLY_EXTENDED,
        PosturePhase.RETURN_TO_DEFAULT,
    ):
        return (0.0,) * JOINT_COUNT
    if phase == PosturePhase.TO_SLIGHTLY_EXTENDED:
        start = DEFAULT_CROUCH
        end = SLIGHTLY_EXTENDED
        local = (now - DEFAULT_END_SECONDS) / TRANSITION_SECONDS
    else:
        start = SLIGHTLY_EXTENDED
        end = DEFAULT_CROUCH
        local = (now - RETURN_START_SECONDS) / TRANSITION_SECONDS
    bounded = max(0.0, min(1.0, local))
    derivative = 6.0 * bounded * (1.0 - bounded) / TRANSITION_SECONDS
    return tuple(
        (end[index] - start[index]) * derivative
        for index in range(JOINT_COUNT)
    )


class PostureTimelineTracker:
    """按控制周期推进时间线并执行显式变化率限制。"""

    def __init__(self) -> None:
        self._limiter = TargetRateLimiter(DEFAULT_CROUCH)
        self.current = DEFAULT_CROUCH

    def sample(self, time_seconds: float, dt: float) -> JointTargets:
        """推进一个控制周期，返回限幅、限速后的姿态目标。"""

        desired = target_at(time_seconds)
        previous = self.current
        self.current = self._limiter.apply(desired, dt)
        try:
            step = float(dt)
        except (TypeError, ValueError):
            step = 0.0
        if not math.isfinite(step) or step <= 0.0:
            velocities = (0.0,) * JOINT_COUNT
        else:
            velocities = tuple(
                (current - before) / step
                for before, current in zip(previous, self.current)
            )
        return JointTargets(self.current, velocities)


def _rotation_x(angle: float) -> Tuple[Tuple[float, ...], ...]:
    """构造绕 +X 的右手旋转矩阵。"""

    cosine = math.cos(angle)
    sine = math.sin(angle)
    return (
        (1.0, 0.0, 0.0),
        (0.0, cosine, -sine),
        (0.0, sine, cosine),
    )


def _rotation_y(angle: float) -> Tuple[Tuple[float, ...], ...]:
    """构造绕 +Y 的右手旋转矩阵。"""

    cosine = math.cos(angle)
    sine = math.sin(angle)
    return (
        (cosine, 0.0, sine),
        (0.0, 1.0, 0.0),
        (-sine, 0.0, cosine),
    )


def _matmul(
    left: Tuple[Tuple[float, ...], ...],
    right: Tuple[Tuple[float, ...], ...],
) -> Tuple[Tuple[float, ...], ...]:
    """计算两个 3x3 矩阵乘积。"""

    return tuple(
        tuple(
            sum(left[row][inner] * right[inner][column] for inner in range(3))
            for column in range(3)
        )
        for row in range(3)
    )


def _matvec(
    matrix: Tuple[Tuple[float, ...], ...],
    vector: Sequence[float],
) -> Tuple[float, float, float]:
    """计算 3x3 矩阵与三维向量乘积。"""

    return tuple(
        sum(matrix[row][column] * float(vector[column]) for column in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def foot_positions(targets: Sequence[float]) -> Dict[str, Tuple[float, float, float]]:
    """按官方轴方向和腿长计算四足 toe 中心在机身坐标系中的位置。"""

    values = clamp_joint_targets(targets)
    result: Dict[str, Tuple[float, float, float]] = {}
    for leg_index, leg in enumerate(("fr", "fl", "hr", "hl")):
        abad, hip, knee = values[leg_index * 3 : leg_index * 3 + 3]
        mount_x, mount_y = MOUNTS[leg]
        abad_rotation = _rotation_x(abad)
        hip_rotation = _matmul(abad_rotation, _rotation_y(-hip))
        knee_rotation = _matmul(hip_rotation, _rotation_y(-knee))

        hip_position = (mount_x, mount_y, 0.0)
        side_vector = _matvec(abad_rotation, (0.0, SIDE_OFFSETS[leg], 0.0))
        hip_position = tuple(
            hip_position[index] + side_vector[index] for index in range(3)
        )
        knee_position = tuple(
            hip_position[index]
            + _matvec(hip_rotation, (0.0, 0.0, -HIP_LINK_LENGTH))[index]
            for index in range(3)
        )
        toe_position = tuple(
            knee_position[index]
            + _matvec(knee_rotation, (0.0, 0.0, -KNEE_LINK_LENGTH))[index]
            for index in range(3)
        )
        result[leg] = toe_position  # type: ignore[assignment]
    return result


def support_half_width(targets: Sequence[float]) -> float:
    """返回给定姿态的足端横向支撑半宽（含 toe 碰撞半径）。"""

    return max(abs(position[1]) for position in foot_positions(targets).values()) + TOE_RADIUS


def support_projection_is_inside(targets: Sequence[float]) -> bool:
    """检查四足投影是否落在机身长度与髋部外撑形成的支撑范围内。"""

    positions = foot_positions(targets)
    half_width = support_half_width(DEFAULT_CROUCH)
    return all(
        abs(position[0]) <= SUPPORT_HALF_LENGTH + 1e-9
        and abs(position[1]) <= half_width + 1e-9
        for position in positions.values()
    )


def knee_interior_angle(targets: Sequence[float]) -> float:
    """返回膝关节内角（弧度）；0 表示直腿锁死。"""

    values = clamp_joint_targets(targets)
    knees = [abs(values[index]) for index in range(2, JOINT_COUNT, 3)]
    return math.pi - max(knees)


__all__ = [
    "DEFAULT_CROUCH",
    "DURATION_SECONDS",
    "FLAT_MOTOR_NAMES",
    "FLAT_SENSOR_NAMES",
    "MAX_TARGET_RATE",
    "PosturePhase",
    "PostureTimelineTracker",
    "SUPPORT_HALF_LENGTH",
    "SLIGHTLY_EXTENDED",
    "TARGET_POSITION_LIMITS",
    "foot_positions",
    "knee_interior_angle",
    "limit_target_rate",
    "phase_at",
    "support_half_width",
    "support_projection_is_inside",
    "target_at",
    "target_velocity_at",
]
