#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分阶段平地移动跳跃奖励；所有输入和输出必须有限。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Sequence

from . import contract


@dataclass(frozen=True)
class RewardInputs:
    """单控制周期奖励输入。"""

    phase: str
    roll: float
    pitch: float
    height: float
    command: Sequence[float]
    velocity: Sequence[float]
    action: Sequence[float]
    previous_action: Sequence[float]
    contacts: Sequence[float]
    omega: Sequence[float] = (0.0, 0.0, 0.0)
    joint_positions: Sequence[float] = (0.0,) * contract.ACTION_DIM
    joint_velocities: Sequence[float] = (0.0,) * contract.ACTION_DIM
    previous_joint_velocities: Sequence[float] = (0.0,) * contract.ACTION_DIM
    joint_torques: Sequence[float] = (0.0,) * contract.ACTION_DIM
    foot_velocities: Sequence[float] = (0.0,) * 12
    phase_steps: int = 0
    non_foot_contact: bool = False
    fallen: bool = False
    jump_success_event: bool = False
    jump_landing_event: bool = False
    displacement: Sequence[float] = (0.0, 0.0, 0.0)


WEIGHTS: Dict[str, Dict[str, float]] = {
    phase: {
        "alive": 0.5,
        "posture": -8.0 if phase in {"P0", "P1"} else -5.0,
        "height": -6.0 if phase in {"P0", "P1"} else -4.0,
        "command_tracking": -1.0 if phase in {"P0", "P1"} else -1.5,
        "zero_velocity": -1.0 if phase in {"P0", "P1"} else -0.15,
        "zero_yaw_rate": -0.5 if phase in {"P0", "P1"} else -0.10,
        "horizontal_angular_velocity": -0.5 if phase in {"P0", "P1"} else -0.20,
        "default_pose": -0.30 if phase in {"P0", "P1"} else -0.10,
        "action_rate": -2.0 if phase in {"P0", "P1"} else -0.05,
        "joint_velocity": -0.001 if phase in {"P0", "P1"} else -0.0005,
        "joint_jitter": -0.30 if phase in {"P0", "P1"} else -0.02,
        "torque": -0.002 if phase in {"P0", "P1"} else -0.001,
        "foot_slip": -20.0 if phase in {"P0", "P1"} else -0.8,
        "true_four_foot_contact": (
            3.0 if phase in {"P0", "P1"}
            else 0.8 if phase == "P2"
            else 0.2
        ),
        # P1 非课程阶段采用更强的缺失足惩罚，直接对应四足同时接触 Gate；
        # 课程阶段再由 static_contact_course 权重覆盖，避免部分接触被轻罚。
        "support_gap": (
            -0.35 if phase == "P1"
            else -0.15 if phase == "P0"
            else -0.15 if phase == "P2"
            else -0.05
        ),
        # 课程分项按当前接触足比例给稠密正反馈，避免“到了课程步数
        # 就无条件加分”与 Gate 的四足同时接触目标脱节。
        "static_contact_course": 0.20 if phase == "P1" else 0.0,
        "non_foot_collision": -10.0 if phase in {"P0", "P1"} else -6.0,
        "feet_air_time": 0.02 if phase in {"P5", "P6"} else 0.0,
        # P2 部分接触不能靠 gait 正奖励抵消四足接触奖励，改为轻微惩罚；
        # 后续阶段保持原有步态塑形语义。
        "gait": (
            -0.05 if phase == "P2"
            else 0.05 if phase in {"P3", "P4", "P6"}
            else 0.0
        ),
        "distance": (
            0.50 if phase == "P2"
            else 0.25 if phase in {"P3", "P4", "P6", "P7"}
            else 0.0
        ),
        "command_speed_progress": 1.5 if phase == "P2" else 0.0,
        "command_displacement_progress": 1.5 if phase == "P2" else 0.0,
        "fall": -120.0,
        "jump_success": 8.0 if phase in contract.JUMP_PHASES else 0.0,
        "jump_landing": 4.0 if phase in contract.JUMP_PHASES else 0.0,
    }
    for phase in contract.PHASE_TOTAL_STEPS
}


def _finite_tuple(values: Sequence[float], length: int) -> tuple[float, ...]:
    if len(values) != length:
        raise ValueError(f"期望 {length} 维有限向量")
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError("奖励输入包含 NaN/Inf")
    return result


def _linear_progress(step: int, start: int, end: int) -> float:
    """返回 [start,end] 内的线性进度，窗口外钳制到 0/1。"""
    if end <= start:
        raise ValueError("线性课程窗口必须为正")
    return max(0.0, min(1.0, (step - start) / (end - start)))


def _mix(early: float, late: float, progress: float) -> float:
    """在早期与晚期权重之间连续混合。"""
    if not 0.0 <= progress <= 1.0:
        raise ValueError("混合进度必须在 0 到 1 之间")
    return early + (late - early) * progress


def compute_reward(inputs: RewardInputs) -> tuple[float, Dict[str, float]]:
    """返回总奖励及分项，任何非有限输入直接失败。"""
    phase = contract.normalize_phase(inputs.phase)
    scalars = tuple(
        float(value)
        for value in (inputs.roll, inputs.pitch, inputs.height)
    )
    if not all(math.isfinite(value) for value in scalars):
        raise ValueError("姿态或高度包含 NaN/Inf")
    roll, pitch, height = scalars
    command = _finite_tuple(inputs.command, 3)
    velocity = _finite_tuple(inputs.velocity, 3)
    action = _finite_tuple(inputs.action, contract.ACTION_DIM)
    previous = _finite_tuple(inputs.previous_action, contract.ACTION_DIM)
    contacts = _finite_tuple(inputs.contacts, 4)
    omega = _finite_tuple(inputs.omega, 3)
    joint_positions = _finite_tuple(inputs.joint_positions, contract.ACTION_DIM)
    joint_velocities = _finite_tuple(inputs.joint_velocities, contract.ACTION_DIM)
    previous_joint_velocities = _finite_tuple(
        inputs.previous_joint_velocities,
        contract.ACTION_DIM,
    )
    joint_torques = _finite_tuple(inputs.joint_torques, contract.ACTION_DIM)
    foot_velocities = _finite_tuple(inputs.foot_velocities, 12)
    displacement = _finite_tuple(inputs.displacement, 3)
    weights = WEIGHTS[phase]

    posture = -8.0 * (roll * roll + pitch * pitch)
    height_error = (height - contract.REFERENCE_HEIGHT) ** 2
    command_error = sum(
        (actual - wanted) ** 2
        for actual, wanted in zip(velocity, command)
    )
    command_norm = math.sqrt(sum(value * value for value in command))
    planar_command_norm = math.hypot(command[0], command[1])
    command_speed_progress = 0.0
    command_displacement_progress = 0.0
    if planar_command_norm > 1e-6:
        direction_x = command[0] / planar_command_norm
        direction_y = command[1] / planar_command_norm
        speed_along_command = (
            velocity[0] * direction_x + velocity[1] * direction_y
        )
        displacement_along_command = (
            displacement[0] * direction_x + displacement[1] * direction_y
        )
        command_speed_progress = max(
            0.0,
            min(1.0, speed_along_command / planar_command_norm),
        )
        expected_displacement = (
            planar_command_norm * contract.CONTROL_DT_SECONDS
        )
        command_displacement_progress = max(
            -1.0,
            min(
                1.0,
                displacement_along_command / expected_displacement,
            ),
        )
    zero_velocity = (
        sum(value * value for value in velocity)
        if command_norm <= 1e-6
        else 0.0
    )
    zero_yaw_rate = omega[2] * omega[2] if abs(command[2]) <= 1e-6 else 0.0
    horizontal_angular_velocity = omega[0] * omega[0] + omega[1] * omega[1]
    default_pose = sum(
        (actual - wanted) ** 2
        for actual, wanted in zip(joint_positions, contract.DEFAULT_CROUCH)
    )
    action_rate = sum(
        (current - old) ** 2
        for current, old in zip(action, previous)
    )
    joint_velocity = sum(value * value for value in joint_velocities)
    joint_jitter = sum(
        (current - previous) ** 2
        for current, previous in zip(
            joint_velocities,
            previous_joint_velocities,
        )
    )
    torque = sum(
        (value / contract.MAX_TORQUE) ** 2
        for value in joint_torques
    )
    foot_slip_speeds: list[float] = []
    for index, contact in enumerate(contacts):
        if contact <= 0.0:
            continue
        foot_velocity = foot_velocities[index * 3:index * 3 + 3]
        foot_slip_speeds.append(
            math.hypot(foot_velocity[0], foot_velocity[1])
        )
    # Gate 只要求接触期平均滑移 <= 0.02 m/s；对超出阈值的部分线性
    # 惩罚，避免低于阈值的微小抖动把奖励整体压黑。
    foot_slip = (
        max(0.0, sum(foot_slip_speeds) / len(foot_slip_speeds) - 0.02)
        if foot_slip_speeds
        else 0.0
    )
    non_foot_collision = 1.0 if inputs.non_foot_contact else 0.0
    contact_flags = [1.0 if value >= 0.5 else 0.0 for value in contacts]
    true_four_foot_contact = 1.0 if all(contact_flags) else 0.0
    support_gap = float(4 - int(sum(contact_flags)))
    # P1 接触奖励在 300k 前后 40k 窗口内连续混合；P1 全程保留正的
    # 四足奖励和负的缺足惩罚，不设置会突然消失的硬开关。
    contact_weight = weights["true_four_foot_contact"]
    support_gap_weight = weights["support_gap"]
    static_course_scale = 0.0
    if phase == "P1":
        transition = _linear_progress(
            int(inputs.phase_steps),
            contract.P1_CONTACT_TRANSITION_START,
            contract.P1_CONTACT_TRANSITION_END,
        )
        contact_weight = _mix(
            12.0,
            weights["true_four_foot_contact"],
            transition,
        )
        support_gap_weight = _mix(
            -1.0,
            weights["support_gap"],
            transition,
        )
        # 主奖励先完成主体过渡，静态课程再单独线性淡出，避免静态课程
        # 关闭与主奖励变化在同一步同时发生。
        static_course_scale = 1.0 - _linear_progress(
            int(inputs.phase_steps),
            contract.P1_STATIC_CONTACT_LOCAL_STEPS,
            contract.P1_STATIC_CONTACT_LOCAL_STEPS
            + contract.P1_STATIC_CONTACT_FADE_STEPS,
        )
    static_contact_course = weights["static_contact_course"] * (
        static_course_scale * sum(contact_flags) / 4.0
    )
    fall = weights["fall"] if inputs.fallen else 0.0
    feet_air_time = 1.0 if sum(contacts) <= 0.0 else 0.0
    gait = 1.0 if 2.0 <= sum(contacts) <= 3.0 else 0.0
    if command_norm > 1e-6:
        distance = (
            velocity[0] * command[0] + velocity[1] * command[1]
        ) / command_norm
    else:
        distance = 0.0
    jump_success = 1.0 if inputs.jump_success_event and phase in contract.JUMP_PHASES else 0.0
    jump_landing = 1.0 if inputs.jump_landing_event and phase in contract.JUMP_PHASES else 0.0

    parts = {
        "alive": 0.0 if inputs.fallen else weights["alive"],
        "posture": weights["posture"] * (roll * roll + pitch * pitch),
        "height": weights["height"] * height_error,
        "command_tracking": weights["command_tracking"] * command_error,
        "zero_velocity": weights["zero_velocity"] * zero_velocity,
        "zero_yaw_rate": weights["zero_yaw_rate"] * zero_yaw_rate,
        "horizontal_angular_velocity": weights["horizontal_angular_velocity"] * horizontal_angular_velocity,
        "default_pose": weights["default_pose"] * default_pose,
        "action_rate": weights["action_rate"] * action_rate,
        "joint_velocity": weights["joint_velocity"] * joint_velocity,
        "joint_jitter": weights["joint_jitter"] * joint_jitter,
        "torque": weights["torque"] * torque,
        "foot_slip": weights["foot_slip"] * foot_slip,
        "true_four_foot_contact": contact_weight * true_four_foot_contact,
        "support_gap": support_gap_weight * support_gap,
        "static_contact_course": static_contact_course,
        "non_foot_collision": weights["non_foot_collision"] * non_foot_collision,
        "feet_air_time": weights["feet_air_time"] * feet_air_time,
        "gait": weights["gait"] * gait,
        "distance": weights["distance"] * distance,
        "command_speed_progress": (
            weights["command_speed_progress"] * command_speed_progress
        ),
        "command_displacement_progress": (
            weights["command_displacement_progress"]
            * command_displacement_progress
        ),
        "fall": fall,
        "jump_success": weights["jump_success"] * jump_success,
        "jump_landing": weights["jump_landing"] * jump_landing,
    }
    total = sum(parts.values())
    if not math.isfinite(total) or not all(math.isfinite(value) for value in parts.values()):
        raise ValueError("奖励计算产生 NaN/Inf")
    return float(total), parts
