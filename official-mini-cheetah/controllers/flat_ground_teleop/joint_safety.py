#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Mini Cheetah 12 关节 PD、力矩限幅与安全反馈检查。

参数来自官方 Mini Cheetah 控制器：KP=[3,3,3]、KD=[1,0.2,0.2]，
控制指令限幅 15 N·m。生成 world 的 ``RotationalMotor.maxTorque=20`` 是
设备能力字段，不替代本控制器的 15 N·m 指令保护。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple


LEG_NAMES: Tuple[str, ...] = ("fr", "fl", "hr", "hl")
JOINT_NAMES: Tuple[str, ...] = ("abd", "hip", "kn")
JOINT_COUNT = len(LEG_NAMES) * len(JOINT_NAMES)

MOTOR_NAMES: Tuple[Tuple[str, ...], ...] = tuple(
    tuple(f"{leg}_{joint}_motor" for joint in JOINT_NAMES) for leg in LEG_NAMES
)
SENSOR_NAMES: Tuple[Tuple[str, ...], ...] = tuple(
    tuple(f"{leg}_{joint}_sensor" for joint in JOINT_NAMES) for leg in LEG_NAMES
)
FLAT_MOTOR_NAMES: Tuple[str, ...] = tuple(name for row in MOTOR_NAMES for name in row)
FLAT_SENSOR_NAMES: Tuple[str, ...] = tuple(name for row in SENSOR_NAMES for name in row)

# 官方 mini_cheetah_controller.cpp 的关节 PD 参数。
KP: Tuple[float, float, float] = (3.0, 3.0, 3.0)
KD: Tuple[float, float, float] = (1.0, 0.2, 0.2)
MAX_TORQUE = 15.0
WEBOTS_MOTOR_MAX_TORQUE = 20.0

# 官方 Mini Cheetah 轴与腿长：abad 绕 X，hip/knee 绕 -Y；
# 长度来自 MiniCheetah.h（abad 0.062、hip 0.209、knee 0.195 m）。
# abad/hip 限位来自 SpineBoard 软停；官方 knee 未提供软停，因此这里使用
# 官方控制器出现过的折叠/测试上界 2.7 rad 作为姿态软件包络，不冒充机械限位。
TARGET_POSITION_LIMITS: Tuple[Tuple[float, float], ...] = (
    (-1.5, 1.5),   # fr_abd
    (-5.0, 5.0),   # fr_hip
    (-2.7, 2.7),   # fr_kn
    (-1.5, 1.5),   # fl_abd
    (-5.0, 5.0),   # fl_hip
    (-2.7, 2.7),   # fl_kn
    (-1.5, 1.5),   # hr_abd
    (-5.0, 5.0),   # hr_hip
    (-2.7, 2.7),   # hr_kn
    (-1.5, 1.5),   # hl_abd
    (-5.0, 5.0),   # hl_hip
    (-2.7, 2.7),   # hl_kn
)

# 12 路目标的变化率上限；平滑时间线和普通站立控制都复用该限制。
MAX_TARGET_RATE = 0.75

# 参考图的屈膝、髋部外撑站姿。左右腿按关节轴镜像：右腿 abad 为负、
# 左腿 abad 为正；hip/knee 在左右腿使用相同 sagittal 目标。
DEFAULT_CROUCH: Tuple[float, ...] = (
    -0.15, -0.80, 1.60,
     0.15, -0.80, 1.60,
    -0.15, -0.80, 1.60,
     0.15, -0.80, 1.60,
)

# 相对默认姿态更伸展，但膝关节仍保留 1.15 rad 的明显弯折。
SLIGHTLY_EXTENDED: Tuple[float, ...] = (
    -0.15, -0.55, 1.15,
     0.15, -0.55, 1.15,
    -0.15, -0.55, 1.15,
     0.15, -0.55, 1.15,
)

STANDING_TARGETS: Tuple[float, ...] = DEFAULT_CROUCH

# 反馈数值只做基本物理范围检查；异常值立即转为安全站立。
MAX_ABS_JOINT_POSITION = 20.0
MAX_ABS_JOINT_VELOCITY = 100.0
MAX_ABS_TARGET = max(abs(bound) for limits in TARGET_POSITION_LIMITS for bound in limits)


def _number(value: float) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def finite_sequence(values: Iterable[float], length: Optional[int] = None) -> bool:
    """检查序列是否全部有限，且长度符合预期。"""

    try:
        items = tuple(float(value) for value in values)
    except (TypeError, ValueError):
        return False
    if length is not None and len(items) != length:
        return False
    return all(math.isfinite(value) for value in items)


def clamp_torque(torque: float, limit: float = MAX_TORQUE) -> float:
    """把单关节力矩裁剪到正负限幅；非有限值返回零。"""

    value = _number(torque)
    if value is None:
        return 0.0
    bound = abs(_number(limit) or 0.0)
    return max(-bound, min(bound, value))


def joint_pd_torque(
    q_des: float,
    q: float,
    qd_des: float,
    qd: float,
    joint_index: int,
) -> float:
    """计算单关节 PD 力矩并执行安全限幅。"""

    desired = _number(q_des)
    position = _number(q)
    desired_velocity = _number(qd_des)
    velocity = _number(qd)
    if any(value is None for value in (desired, position, desired_velocity, velocity)):
        return 0.0
    try:
        index = int(joint_index)
    except (TypeError, ValueError):
        return 0.0
    if index < 0 or index >= len(KP):
        return 0.0
    torque = KP[index] * (desired - position) + KD[index] * (desired_velocity - velocity)
    return clamp_torque(torque)


def _flatten(values: Sequence[float], expected: int = JOINT_COUNT) -> Tuple[float, ...]:
    try:
        flat = tuple(float(value) for value in values)
    except (TypeError, ValueError):
        return ()
    return flat if len(flat) == expected else ()


def clamp_joint_targets(values: Sequence[float]) -> Tuple[float, ...]:
    """把 12 路位置目标裁剪到显式姿态限位，非有限值回退到默认姿态。"""

    flat = _flatten(values)
    if len(flat) != JOINT_COUNT or not all(math.isfinite(value) for value in flat):
        return DEFAULT_CROUCH
    return tuple(
        max(limits[0], min(limits[1], value))
        for value, limits in zip(flat, TARGET_POSITION_LIMITS)
    )


def limit_target_rate(
    previous: Sequence[float],
    desired: Sequence[float],
    dt: float,
    max_rate: float = MAX_TARGET_RATE,
) -> Tuple[float, ...]:
    """按 ``max_rate`` 限制相邻两帧的 12 路目标变化量。"""

    old = _flatten(previous)
    new = _flatten(desired)
    if len(old) != JOINT_COUNT or len(new) != JOINT_COUNT:
        return DEFAULT_CROUCH
    if not all(math.isfinite(value) for value in old + new):
        return DEFAULT_CROUCH
    try:
        step_dt = float(dt)
        rate = abs(float(max_rate))
    except (TypeError, ValueError):
        return DEFAULT_CROUCH
    if not math.isfinite(step_dt) or step_dt <= 0.0 or not math.isfinite(rate):
        return old
    limit = rate * step_dt
    return tuple(
        max(target - limit, min(target + limit, current))
        for current, target in zip(old, new)
    )


class TargetRateLimiter:
    """维护相邻控制帧的显式位置目标与变化率限制。"""

    def __init__(self, initial: Sequence[float] = DEFAULT_CROUCH) -> None:
        self.current = clamp_joint_targets(initial)

    def apply(
        self,
        desired: Sequence[float],
        dt: float,
    ) -> Tuple[float, ...]:
        """更新并返回经过限幅、限速的 12 路目标。"""

        target = clamp_joint_targets(desired)
        self.current = limit_target_rate(self.current, target, dt)
        self.current = clamp_joint_targets(self.current)
        return self.current


def validate_joint_feedback(
    positions: Sequence[float],
    velocities: Sequence[float],
) -> bool:
    """检查 12 路反馈是否有限且处于保护范围内。"""

    q = _flatten(positions)
    qd = _flatten(velocities)
    if not q or not qd:
        return False
    return (
        all(math.isfinite(value) and abs(value) <= MAX_ABS_JOINT_POSITION for value in q)
        and all(math.isfinite(value) and abs(value) <= MAX_ABS_JOINT_VELOCITY for value in qd)
    )


def compute_pd_torques(
    desired_positions: Sequence[float],
    desired_velocities: Sequence[float],
    measured_positions: Sequence[float],
    measured_velocities: Sequence[float],
) -> Tuple[float, ...]:
    """计算 12 路 PD 力矩；任何一路非有限反馈时全部输出零。"""

    q_des = _flatten(desired_positions)
    qd_des = _flatten(desired_velocities)
    q = _flatten(measured_positions)
    qd = _flatten(measured_velocities)
    if not q_des or not qd_des or not q or not qd:
        return (0.0,) * JOINT_COUNT
    if not validate_joint_feedback(q, qd):
        return (0.0,) * JOINT_COUNT
    return tuple(
        joint_pd_torque(q_des[index], q[index], qd_des[index], qd[index], index % 3)
        for index in range(JOINT_COUNT)
    )


# 兼容更直观的函数名。
compute_joint_torques = compute_pd_torques


@dataclass(frozen=True)
class JointTargets:
    """一帧 12 路期望位置/速度。"""

    positions: Tuple[float, ...] = STANDING_TARGETS
    velocities: Tuple[float, ...] = (0.0,) * JOINT_COUNT

    def __post_init__(self) -> None:
        positions = clamp_joint_targets(self.positions)
        velocities = _flatten(self.velocities)
        object.__setattr__(
            self,
            "positions",
            positions,
        )
        object.__setattr__(
            self,
            "velocities",
            tuple(
                max(-MAX_TARGET_RATE, min(MAX_TARGET_RATE, value))
                if math.isfinite(value)
                else 0.0
                for value in velocities
            )
            if len(velocities) == JOINT_COUNT
            else (0.0,) * JOINT_COUNT,
        )


def standing_targets() -> JointTargets:
    """返回默认站立目标。"""

    return JointTargets()


class MiniCheetahGait:
    """简化速度驱动对角 trot。

    这里只生成有限、限幅的关节期望值，不宣称步态性能；控制器在站立模式
    或任何安全故障下都会绕过该生成器并使用 DEFAULT_CROUCH 站立目标。
    """

    TROT_PERIOD = 0.5
    PHASE_OFFSETS = (0.0, 0.5, 0.5, 0.0)
    HIP_AMPLITUDE_SCALE = 0.22
    KNEE_LIFT_SCALE = 0.04
    ABD_SCALE = 0.12
    YAW_SCALE = 0.08
    MIN_MOTION = 0.02

    def __init__(self) -> None:
        self.phase = 0.0

    def reset(self, phase: float = 0.0) -> None:
        value = _number(phase)
        self.phase = (value or 0.0) % 1.0

    def step(
        self,
        vx: float,
        vy: float,
        wz: float,
        dt: float,
    ) -> JointTargets:
        """推进一个控制周期并返回 12 路目标。"""

        command = tuple(_number(value) for value in (vx, vy, wz))
        time_step = _number(dt)
        if any(value is None for value in command) or time_step is None or time_step <= 0.0:
            self.reset()
            return JointTargets()
        if time_step > 0.1:
            time_step = 0.1
        vx_value, vy_value, wz_value = command
        motion = max(abs(vx_value), abs(vy_value), abs(wz_value))
        if motion < self.MIN_MOTION:
            self.reset()
            return JointTargets()

        self.phase = (self.phase + time_step / self.TROT_PERIOD) % 1.0
        positions: List[float] = []
        velocities: List[float] = []
        hip_amplitude = max(
            -self.HIP_AMPLITUDE_SCALE,
            min(self.HIP_AMPLITUDE_SCALE, vx_value * 0.35),
        )
        knee_lift = min(0.10, 0.025 + abs(vx_value) * self.KNEE_LIFT_SCALE)
        abduction = max(-0.20, min(0.20, vy_value * self.ABD_SCALE))

        for leg_index, leg in enumerate(LEG_NAMES):
            leg_phase = (self.phase + self.PHASE_OFFSETS[leg_index]) % 1.0
            swing = leg_phase >= 0.5
            progress = (leg_phase - 0.5) / 0.5 if swing else leg_phase / 0.5
            hip = hip_amplitude * (1.0 - 2.0 * progress)
            hip_velocity = -2.0 * hip_amplitude / (0.5 * self.TROT_PERIOD)
            # 左右腿反向偏置，使 wz=正对应左转。
            side_sign = -1.0 if leg in ("fr", "hr") else 1.0
            hip += side_sign * wz_value * self.YAW_SCALE

            if swing:
                knee = -knee_lift * math.sin(math.pi * progress)
                knee_velocity = (
                    -knee_lift * math.pi / (0.5 * self.TROT_PERIOD) * math.cos(math.pi * progress)
                )
            else:
                knee = 0.0
                knee_velocity = 0.0

            base_abd, base_hip, base_knee = DEFAULT_CROUCH[
                leg_index * len(JOINT_NAMES) : (leg_index + 1) * len(JOINT_NAMES)
            ]
            positions.extend(
                (
                    base_abd + abduction,
                    base_hip + hip,
                    base_knee + knee,
                )
            )
            velocities.extend((0.0, hip_velocity, knee_velocity))

        return JointTargets(tuple(positions), tuple(velocities))
