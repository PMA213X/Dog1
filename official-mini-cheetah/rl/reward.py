#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分阶段平地移动跳跃奖励；所有输入和输出必须有限。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Sequence

from . import contract


RAPID_SOURCE_ACTION_LIMITS: tuple[float, ...] = (2.4, 2.0, 2.0) * 4
RAPID_SOURCE_ACTION_SATURATION_RAMP_START = 0.95
RAPID_FINETUNE_MISSING_FOOT_WEIGHT = -2.00
RAPID_FINETUNE_ACTION_SATURATION_WEIGHT = -0.50
# final Gate 的足滑均值为 0.054 m/s，是 0.02 m/s 门槛的 2.7 倍；训练侧
# 用更严格的 0.015 m/s 起罚线，并把权重从 -20 加强到 -40，保留安全裕量。
RAPID_FINETUNE_FOOT_SLIP_THRESHOLD = 0.015
RAPID_FINETUNE_FOOT_SLIP_WEIGHT = -40.00
# 当前状态没有逐 toe 法向力，因此用“精确 toe 接触 + 高度/姿态支撑”作为
# 等效支撑判据；门槛直接复用 Gate 的 height_p05 和姿态 p95 下界。
EFFECTIVE_SUPPORT_MIN_HEIGHT = 0.25
EFFECTIVE_SUPPORT_MAX_POSTURE = 0.25
NON_FOOT_COLLISION_WEIGHT = -60.00
# 下列权重只作用于 Rapid 微调：final Gate 的 stop 四足比例仅 0.0088、
# forward 速度误差 0.234、平均接触足 2.391、足滑 0.054 m/s、
# height_p05 0.236 m，分别越过各自门槛；除有效支撑过滤和非足大额安全
# 惩罚外，旧 P0～P7 的速度、姿态、步态与课程权重保持不变。
RAPID_FINETUNE_COMMAND_TRACKING_WEIGHT = -8.00
RAPID_FINETUNE_HEIGHT_WEIGHT = -300.00
RAPID_FINETUNE_STOP_CONTACT_WEIGHT = 2.00
RAPID_FINETUNE_STOP_SUPPORT_GAP_WEIGHT = -1.00
RAPID_FINETUNE_MOVING_CONTACT_GUARD_WEIGHT = -2.00
RAPID_FINETUNE_MOVING_CONTACT_TARGET = 2.50


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
    # 仅 Rapid 微调 VecEnv 显式开启；默认关闭以保持旧 checkpoint 和
    # P0～P7 奖励语义完全不变。
    finetune_mode: bool = False
    # 源模型未裁剪动作。若调用方直接传入，compute_reward 会计算饱和
    # 分项；自定义 VecEnv 也可在 env.step 后单独调用纯函数再叠加。
    source_action: Sequence[float] | None = None


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
        # shank、机身等任何非 toe 接触都可能以局部碰地换取接触计数，
        # 必须形成明显高于存活/移动收益的大额惩罚。
        "non_foot_collision": NON_FOOT_COLLISION_WEIGHT,
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
        "source_action_saturation": RAPID_FINETUNE_ACTION_SATURATION_WEIGHT,
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


def source_action_saturation_penalty(
    source_action: Sequence[float],
) -> float:
    """返回 Rapid 源动作接近或越过等效执行边界的非正惩罚。

    `[2.4, 2.0, 2.0]` 与源动作尺度 `0.125/0.25` 相乘后正好得到当前
    `[-1,1]` 动作对应的 `0.30/0.50/0.50 rad` 关节位移。训练末期动作
    饱和率升到 `0.2083`，而旧公式在边界处仍为 0 且越界后按 12 关节平均，
    梯度过弱；从 95% 边界开始线性爬升。这里只惩罚源动作，绝不放宽执行
    层的 `[-1,1]` 裁剪。
    """
    values = _finite_tuple(
        source_action,
        contract.ACTION_DIM,
    )
    limits = RAPID_SOURCE_ACTION_LIMITS
    ramp_width = 1.0 - RAPID_SOURCE_ACTION_SATURATION_RAMP_START
    excess = [
        max(
            0.0,
            abs(value) / limit - RAPID_SOURCE_ACTION_SATURATION_RAMP_START,
        )
        / ramp_width
        for value, limit in zip(values, limits)
    ]
    penalty = (
        RAPID_FINETUNE_ACTION_SATURATION_WEIGHT
        * sum(excess)
        / len(excess)
    )
    if not math.isfinite(penalty):
        raise ValueError("源动作饱和惩罚包含 NaN/Inf")
    return float(penalty)


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
    source_action = (
        None
        if inputs.source_action is None
        else _finite_tuple(
            inputs.source_action,
            contract.ACTION_DIM,
        )
    )
    weights = WEIGHTS[phase]

    posture = -8.0 * (roll * roll + pitch * pitch)
    height_error = (height - contract.REFERENCE_HEIGHT) ** 2
    command_error = sum(
        (actual - wanted) ** 2
        for actual, wanted in zip(velocity, command)
    )
    command_tracking_weight = weights["command_tracking"]
    height_weight = weights["height"]
    foot_slip_weight = weights["foot_slip"]
    if inputs.finetune_mode and phase == "P2":
        # Gate 的 stop/forward 都直接验收速度误差；原 -1.5 在实测
        # stop 0.165 m/s、forward 误差 0.234 m/s 时只有约 -0.04～-0.08，
        # 不足以抵消前进正奖励。-8 使这些失败样本达到约 -0.2～-0.4，
        # 同时横向速度分量会直接压低 forward 的交叉轴漂移。
        command_tracking_weight = RAPID_FINETUNE_COMMAND_TRACKING_WEIGHT
        # Gate height_p05=0.236，低于 0.25；原平方项在 0.035 m 高度差
        # 时几乎不可见，放大系数只在微调分支保护站立高度。
        height_weight = RAPID_FINETUNE_HEIGHT_WEIGHT
        # 仅惩罚超过 Gate 0.02 m/s 的滑移，但实测 0.054 m/s 必须具有
        # 与姿态项同量级的负反馈，不能继续被前进收益抵消。
        foot_slip_weight = RAPID_FINETUNE_FOOT_SLIP_WEIGHT
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
    non_foot_collision = 1.0 if inputs.non_foot_contact else 0.0
    raw_contact_flags = [
        1.0 if value >= 0.5 else 0.0 for value in contacts
    ]
    # 控制器已按缓存的四个精确 shank/toe node_id 过滤；这里再要求机身
    # 处于 Gate 允许的直立高度和姿态内，避免低姿拖行或侧躺蹭地被当成
    # 正常支撑。没有逐 toe 法向力时，这是可由现有状态完整计算的等效判据。
    effective_support = (
        height >= EFFECTIVE_SUPPORT_MIN_HEIGHT
        and abs(roll) <= EFFECTIVE_SUPPORT_MAX_POSTURE
        and abs(pitch) <= EFFECTIVE_SUPPORT_MAX_POSTURE
    )
    contact_flags = (
        raw_contact_flags if effective_support else [0.0] * 4
    )
    # 每个足各自只取其 3 维接触点速度，再在所有有效 toe 上取均值；
    # 任一未接触足不会覆盖其他足，也不会用最后一足速度代替整组。
    foot_slip_speeds: list[float] = []
    for index, contact in enumerate(contact_flags):
        if contact <= 0.0:
            continue
        foot_velocity = foot_velocities[index * 3:index * 3 + 3]
        foot_slip_speeds.append(
            math.hypot(foot_velocity[0], foot_velocity[1])
        )
    foot_slip_threshold = (
        RAPID_FINETUNE_FOOT_SLIP_THRESHOLD
        if inputs.finetune_mode and phase == "P2"
        else 0.02
    )
    foot_slip = (
        max(
            0.0,
            sum(foot_slip_speeds) / len(foot_slip_speeds)
            - foot_slip_threshold,
        )
        if foot_slip_speeds
        else 0.0
    )
    true_four_foot_contact = 1.0 if all(contact_flags) else 0.0
    support_gap = float(4 - int(sum(contact_flags)))
    average_contact_feet = sum(contact_flags)
    finetune_moving = (
        bool(inputs.finetune_mode)
        and phase == "P2"
        and command_norm > 1e-6
    )
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
    # Rapid 微调的零命令只对四个有效 toe 同时支撑给正奖励；移动分支不再
    # 按任意接触足数线性给正分，只保留不足两足和低于 2.5 足的连续惩罚。
    finetune_stop = (
        bool(inputs.finetune_mode)
        and phase == "P2"
        and command_norm <= 1e-6
    )
    if finetune_stop:
        # stop Gate 要求真实四足比例 >=0.85，而 final 仅 0.0088；
        # 旧 P2 的 0.8/-0.15 不足以把已冻结模型的站稳行为保留在
        # 20 s 固定 zero-command episode 内。
        contact_weight = RAPID_FINETUNE_STOP_CONTACT_WEIGHT
        support_gap_weight = RAPID_FINETUNE_STOP_SUPPORT_GAP_WEIGHT
    if finetune_moving:
        contact_weight = 0.0
        support_gap_weight = 0.0
    static_contact_course = weights["static_contact_course"] * (
        static_course_scale * sum(contact_flags) / 4.0
    )
    fall = weights["fall"] if inputs.fallen else 0.0
    feet_air_time = 1.0 if sum(contact_flags) <= 0.0 else 0.0
    gait = (
        0.0
        if finetune_moving
        else 1.0 if 2.0 <= sum(contact_flags) <= 3.0 else 0.0
    )
    if command_norm > 1e-6:
        distance = (
            velocity[0] * command[0] + velocity[1] * command[1]
        ) / command_norm
    else:
        distance = 0.0
    jump_success = 1.0 if inputs.jump_success_event and phase in contract.JUMP_PHASES else 0.0
    jump_landing = 1.0 if inputs.jump_landing_event and phase in contract.JUMP_PHASES else 0.0
    if finetune_moving and average_contact_feet < 2.0:
        finetune_moving_contact = (
            RAPID_FINETUNE_MISSING_FOOT_WEIGHT
            * (4.0 - average_contact_feet)
        )
    else:
        finetune_moving_contact = 0.0
    # moving Gate 要求平均至少 2.5 足；final 为 2.3906。接触分项本身
    # 不再给任何正分，只对低于目标的样本连续扣分，使两足状态明显劣于
    # 三～四足，同时不会鼓励用局部碰地换取接触计数。
    finetune_moving_contact_guard = (
        RAPID_FINETUNE_MOVING_CONTACT_GUARD_WEIGHT
        * max(
            0.0,
            RAPID_FINETUNE_MOVING_CONTACT_TARGET
            - average_contact_feet,
        )
        if finetune_moving
        else 0.0
    )
    source_action_saturation = (
        0.0
        if source_action is None
        else source_action_saturation_penalty(source_action)
    )

    parts = {
        "alive": 0.0 if inputs.fallen else weights["alive"],
        "posture": weights["posture"] * (roll * roll + pitch * pitch),
        "height": height_weight * height_error,
        "command_tracking": command_tracking_weight * command_error,
        "zero_velocity": weights["zero_velocity"] * zero_velocity,
        "zero_yaw_rate": weights["zero_yaw_rate"] * zero_yaw_rate,
        "horizontal_angular_velocity": weights["horizontal_angular_velocity"] * horizontal_angular_velocity,
        "default_pose": weights["default_pose"] * default_pose,
        "action_rate": weights["action_rate"] * action_rate,
        "joint_velocity": weights["joint_velocity"] * joint_velocity,
        "joint_jitter": weights["joint_jitter"] * joint_jitter,
        "torque": weights["torque"] * torque,
        "foot_slip": foot_slip_weight * foot_slip,
        "true_four_foot_contact": contact_weight * true_four_foot_contact,
        "support_gap": support_gap_weight * support_gap,
        "finetune_moving_contact": finetune_moving_contact,
        "finetune_moving_contact_guard": finetune_moving_contact_guard,
        "source_action_saturation": source_action_saturation,
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
