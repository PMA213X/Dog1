#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""官方 Mini Cheetah 平地移动跳跃 RL 的唯一公共契约。"""

from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Dict, Iterable, Sequence, Tuple, Union


CONTRACT_VERSION = "official_mini_cheetah_flat_jump_v1_r3"
PROJECT_ROOT = Path(__file__).resolve().parents[1]
WORLD_PATH = PROJECT_ROOT / "worlds" / "flat_move_jump_rl.wbt"
EVAL_WORLD_PATH = PROJECT_ROOT / "worlds" / "flat_move_jump_rl_eval.wbt"
CHECKPOINT_ROOT = PROJECT_ROOT / "checkpoints" / CONTRACT_VERSION
RUN_ROOT = PROJECT_ROOT / "runs" / CONTRACT_VERSION
LOG_ROOT = PROJECT_ROOT / "logs" / CONTRACT_VERSION

OBS_DIM = 57
ACTION_DIM = 12
ACTION_LOW = -1.0
ACTION_HIGH = 1.0
# P1 Gate 的 action_delta_rms 上限为 0.10；把执行层限幅压到 0.08，
# 保证训练和评估实际执行动作的相邻差不会把该指标推过门槛。
ACTION_RATE_LIMIT = 0.08
ACTION_SCALE = (0.30, 0.50, 0.50) * 4
# 关节目标限速只允许使用经短测验证的三档；0.03 会把 Rapid 目标压成
# 严重滞后，禁止作为默认值回退。环境变量必须继承给 Webots controller。
ACTION_TARGET_RATE_LADDER: Tuple[float, ...] = (0.25, 0.5, 1.0)
ACTION_TARGET_RATE_DEFAULT_LIMIT = 1.0
ACTION_TARGET_RATE_ENV_VAR = "RL_ACTION_TARGET_RATE_LIMIT"
ACTION_TARGET_RATE_TOLERANCE = 1e-9

CONTROL_RATE_HZ = 50
CONTROL_DT_SECONDS = 0.02
WEBOTS_TIMESTEP_MS = 4
PHYSICS_STEPS_PER_CONTROL = 5
BIRTH_HEIGHT = 0.45
# 当前 R2025a world 中 DEFAULT_CROUCH 的实测稳定机身高度。
# RSI、动作零点、奖励高度项和 Gate 必须共用这一权威值。
DEFAULT_STANDING_HEIGHT = 0.2713
REFERENCE_HEIGHT = DEFAULT_STANDING_HEIGHT
RSI_STANCE_HEIGHT = DEFAULT_STANDING_HEIGHT
MAX_EPISODE_STEPS = 1000
P1_EPISODE_STEPS = 500
WORKSPACE_LIMIT = 8.0
LOCAL_ACTIVITY_RADIUS = 3.0
MIN_BASE_HEIGHT = 0.12
MAX_BASE_HEIGHT = 0.60
ROLL_PITCH_LIMIT = math.radians(15.0)
MIN_SUPPORTED_HEIGHT = 0.60 * REFERENCE_HEIGHT
UNSUPPORTED_STEPS_LIMIT = int(1.0 / CONTROL_DT_SECONDS)

DEFAULT_CROUCH: Tuple[float, ...] = (
    -0.15, -0.80, 1.60,
     0.15, -0.80, 1.60,
    -0.15, -0.80, 1.60,
     0.15, -0.80, 1.60,
)
TARGET_POSITION_LIMITS: Tuple[Tuple[float, float], ...] = tuple(
    bound
    for _ in range(4)
    for bound in ((-1.5, 1.5), (-5.0, 5.0), (-2.7, 2.7))
)
KP = (3.0, 3.0, 3.0)
KD = (1.0, 0.2, 0.2)
MAX_TORQUE = 15.0

LEG_NAMES: Tuple[str, ...] = ("fr", "fl", "hr", "hl")
JOINT_NAMES: Tuple[str, ...] = ("abd", "hip", "kn")
MOTOR_NAMES: Tuple[str, ...] = tuple(
    f"{leg}_{joint}_motor"
    for leg in LEG_NAMES
    for joint in JOINT_NAMES
)
SENSOR_NAMES: Tuple[str, ...] = tuple(
    f"{leg}_{joint}_sensor"
    for leg in LEG_NAMES
    for joint in JOINT_NAMES
)

COMMAND_FIELDS: Tuple[str, ...] = ("vx", "vy", "wz")
COMMAND_LIMITS: Dict[str, Tuple[float, float]] = {
    "vx": (-0.3, 0.6),
    "vy": (-0.3, 0.3),
    "wz": (-1.0, 1.0),
}
LOW_SPEED_COMMAND_LIMITS: Dict[str, Tuple[float, float]] = {
    "vx": (-0.10, 0.25),
    "vy": (-0.10, 0.10),
    "wz": (-0.30, 0.30),
}
MOVING_JUMP_COMMAND_LIMITS: Dict[str, Tuple[float, float]] = {
    "vx": (0.10, 0.30),
    "vy": (-0.15, 0.15),
    "wz": (-0.40, 0.40),
}
COMMAND_LOW = tuple(COMMAND_LIMITS[name][0] for name in COMMAND_FIELDS)
COMMAND_HIGH = tuple(COMMAND_LIMITS[name][1] for name in COMMAND_FIELDS)
COMMAND_RESAMPLE_STEPS = 250
ZERO_COMMAND_PROBABILITY = 0.20
# P2 Gate 的 forward 命令固定占 50%，旧均匀采样在短窗口内可能抽不到；
# 用覆盖式循环同时保证 forward、zero、横向与偏航覆盖。
P2_ZERO_COMMAND_PROBABILITY = 0.25
P2_GATE_FORWARD_COMMAND_MIN = 0.18
P2_GATE_FORWARD_COMMAND_MAX = 0.25
P2_COMMAND_STRATUM_CYCLE: Tuple[str, ...] = (
    "gate_forward",
    "zero",
    "lateral_positive",
    "gate_forward",
    "zero",
    "yaw_positive",
    "gate_forward",
    "zero",
    "lateral_negative",
    "gate_forward",
    "zero",
    "yaw_negative",
    "gate_forward",
    "gate_forward",
    "gate_forward",
    "gate_forward",
)

JUMP_SUCCESS_HEIGHT_GAIN = 0.04
JUMP_REQUEST_MIN_STEP = 100
JUMP_REQUEST_MAX_STEP = 200
JUMP_INTERVAL_MIN = 200
JUMP_INTERVAL_MAX = 350

FRICTION_RANGE = (0.9, 1.3)
MASS_SCALE_RANGE = (0.95, 1.05)
DELAY_STEPS_RANGE = (0, 2)
CURRICULUM_FIXED_EPISODES = 10
CURRICULUM_EPISODES = 20

PHASE_TOTAL_STEPS: Dict[str, int] = {
    "P0": 10_000,
    "P1": 400_000,
    "P2": 1_500_000,
    "P3": 4_000_000,
    "P4": 6_500_000,
    "P5": 9_500_000,
    "P6": 14_500_000,
    "P7": 18_000_000,
}
BUFFER_TOTAL_STEPS = 20_000_000
PHASE_NAMES: Dict[str, str] = {
    "P0": "链路冒烟",
    "P1": "固定站立",
    "P2": "低速移动",
    "P3": "完整命令",
    "P4": "域随机鲁棒化",
    "P5": "原地跳",
    "P6": "移动跳",
    "P7": "遥控整合",
}
PHASE_DEFAULT_TARGETS: Dict[str, int] = dict(PHASE_TOTAL_STEPS)
PHASE_ORDER: Tuple[str, ...] = tuple(PHASE_TOTAL_STEPS)
LOW_SPEED_PHASES: Tuple[str, ...] = ("P2",)
COMMAND_PHASES: Tuple[str, ...] = ("P2", "P3", "P4", "P6", "P7")
JUMP_PHASES: Tuple[str, ...] = ("P5", "P6", "P7")
PHASE_RANDOMIZATION_MODES: Dict[str, str] = {
    "P0": "fixed",
    "P1": "fixed",
    "P2": "fixed",
    "P3": "curriculum",
    "P4": "dr1",
    "P5": "full",
    "P6": "full",
    "P7": "full",
}
RANDOMIZATION_LEVELS: Tuple[str, ...] = tuple(f"dr{index}" for index in range(1, 8))
RANDOMIZATION_MODES: Tuple[str, ...] = (
    "fixed",
    "curriculum",
    *RANDOMIZATION_LEVELS,
    "full",
)
RANDOMIZATION_LEVEL_NAMES: Dict[str, str] = {
    "dr1": "reset_pose",
    "dr2": "observation_noise",
    "dr3": "action_delay",
    "dr4": "friction",
    "dr5": "mass",
    "dr6": "motor_strength",
    "dr7": "external_impulse",
}
# 课程奖励在 300k 前后按 40k 窗口连续混合，避免四足奖励与缺足惩罚硬切。
P1_CONTACT_TRANSITION_START = 280_000
P1_CONTACT_TRANSITION_END = 320_000
# 静态课程在 300k 后另用 30k 线性淡出，不与主奖励同时跳变。
P1_STATIC_CONTACT_LOCAL_STEPS = 300_000
P1_STATIC_CONTACT_FADE_STEPS = 30_000
# reset 扰动延后到接触奖励完全稳定 10k 步后启用，错开分布突变。
P1_PERTURBATION_LOCAL_STEPS = 340_000
P1_PERTURBATION_GATE_STEP = 310_000
CHECKPOINT_INTERVAL_STEPS = 20_000
CHECKPOINT_INTERVAL_SECONDS = 1_800.0
GATE_CHECK_INTERVAL_STEPS = 10_000
# P2 每段容纳多个完整 PPO rollout（2048×4=8192），避免 10k 段只更新
# 一次并丢弃尾部 buffer；P0/P1/P4 保持既有 10k 调度。
P2_GATE_CHECK_INTERVAL_STEPS = 50_000
BASE_TRAINING_SEED = 20_261_003
BRIDGE_PORT = 11452
WEBOTS_SUPERVISOR_PORT = 1234
PARALLEL_WORKERS = 4
ROBOT_NAMES: Tuple[str, ...] = tuple(
    f"mini_cheetah_{worker_id}" for worker_id in range(PARALLEL_WORKERS)
)
BIRTH_POSITIONS: Tuple[Tuple[float, float, float], ...] = (
    (-4.0, -4.0, BIRTH_HEIGHT),
    (4.0, -4.0, BIRTH_HEIGHT),
    (-4.0, 4.0, BIRTH_HEIGHT),
    (4.0, 4.0, BIRTH_HEIGHT),
)
EVAL_BIRTH_POSITION: Tuple[float, float, float] = (0.0, 0.0, BIRTH_HEIGHT)


def validate_action_target_rate_limit(value: float | str) -> float:
    """校验限速必须来自 0.25/0.5/1.0 阶梯，拒绝静默回退到 0.03。"""
    try:
        selected = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"ACTION_TARGET_RATE_LIMIT 非法：{value!r}") from exc
    if not math.isfinite(selected):
        raise ValueError("ACTION_TARGET_RATE_LIMIT 必须有限")
    matched = next(
        (
            candidate
            for candidate in ACTION_TARGET_RATE_LADDER
            if math.isclose(
                selected,
                candidate,
                rel_tol=0.0,
                abs_tol=ACTION_TARGET_RATE_TOLERANCE,
            )
        ),
        None,
    )
    if matched is None:
        raise ValueError(
            "ACTION_TARGET_RATE_LIMIT 必须属于阶梯："
            f"{ACTION_TARGET_RATE_LADDER}"
        )
    return float(matched)


def _configured_action_target_rate_limit() -> float:
    """读取启动进程的限速档位；空值使用短测通过的 1.0。"""
    raw = os.environ.get(ACTION_TARGET_RATE_ENV_VAR, "").strip()
    return validate_action_target_rate_limit(
        raw or ACTION_TARGET_RATE_DEFAULT_LIMIT
    )


ACTION_TARGET_RATE_LIMIT = _configured_action_target_rate_limit()


def resolve_action_target_rate_limit(value: float | str | None = None) -> float:
    """返回显式档位或当前进程档位，供 controller/play/env 共用。"""
    if value is None:
        return validate_action_target_rate_limit(ACTION_TARGET_RATE_LIMIT)
    return validate_action_target_rate_limit(value)


def bridge_ports(num_envs: int = PARALLEL_WORKERS) -> Tuple[int, ...]:
    """返回单 Webots world 内各 Robot controller 的连续 TCP 端口。"""
    if num_envs != PARALLEL_WORKERS:
        raise ValueError(f"并行机器人数量必须是 {PARALLEL_WORKERS}")
    return tuple(BRIDGE_PORT + index for index in range(num_envs))

OBS_SLICES: Dict[str, slice] = {
    "q": slice(0, 12),
    "dq": slice(12, 24),
    "rpy": slice(24, 27),
    "v_body": slice(27, 30),
    "prev_action": slice(30, 42),
    "omega_body": slice(42, 45),
    "cmd": slice(45, 48),
    "jump_request": slice(48, 49),
    "jump_phase": slice(49, 50),
    "height": slice(50, 51),
    "contacts": slice(51, 55),
    "terrain_height": slice(55, 56),
    "flat_flag": slice(56, 57),
}

FORBIDDEN_PREFIXES = (
    "yobogo_" + "loco_jump_v1",
    "yobogo_" + "flat_jump_v1",
    "official_mini_cheetah_flat_jump_v1_r2",
)
FORBIDDEN_DIRECTORY = "webots" + "-sim"

CheckpointPath = Union[str, Path]


def normalize_phase(tag: str) -> str:
    """规范化 P0～P7 阶段标签。"""
    value = str(tag).strip().upper()
    if value in PHASE_TOTAL_STEPS:
        return value
    raise ValueError(f"未知阶段 {tag!r}；可选 {list(PHASE_TOTAL_STEPS)}")


def phase_gate_check_interval_steps(phase: str) -> int:
    """返回阶段训练分段/Gate 间隔，不改变阶段总预算。"""
    normalized = normalize_phase(phase)
    if normalized == "P2":
        return P2_GATE_CHECK_INTERVAL_STEPS
    return GATE_CHECK_INTERVAL_STEPS


def phase_training_seed(phase: str, phase_step_offset: int) -> int:
    """把阶段与本地累计步数混入种子，避免每段重复同一条命令序列。"""
    normalized = normalize_phase(phase)
    offset = int(phase_step_offset)
    if offset < 0:
        raise ValueError("阶段步偏移不能为负")
    return (
        BASE_TRAINING_SEED
        + PHASE_ORDER.index(normalized) * 1_000_000
        + offset
    )


def phase_command_limits(tag: str) -> Dict[str, Tuple[float, float]]:
    """返回指定阶段的结构化命令边界。"""
    phase = normalize_phase(tag)
    if phase in LOW_SPEED_PHASES:
        return dict(LOW_SPEED_COMMAND_LIMITS)
    if phase == "P0" or phase == "P1":
        return {
            name: (0.0, 0.0)
            for name in COMMAND_FIELDS
        }
    if phase == "P5":
        return {
            name: (0.0, 0.0)
            for name in COMMAND_FIELDS
        }
    if phase == "P6":
        return dict(MOVING_JUMP_COMMAND_LIMITS)
    return dict(COMMAND_LIMITS)


def finite(values: Iterable[float], length: int | None = None) -> bool:
    """检查序列是否全部有限且长度正确。"""
    try:
        items = tuple(float(value) for value in values)
    except (TypeError, ValueError):
        return False
    if length is not None and len(items) != length:
        return False
    return bool(items) and all(math.isfinite(value) for value in items)


def clip_command(command: Sequence[float]) -> Tuple[float, float, float]:
    """按契约限幅三轴结构化命令。"""
    if not finite(command, 3):
        raise ValueError("命令必须是 3 个有限数")
    values = []
    for index, name in enumerate(COMMAND_FIELDS):
        low, high = COMMAND_LIMITS[name]
        values.append(max(low, min(high, float(command[index]))))
    return values[0], values[1], values[2]


def clip_phase_command(tag: str, command: Sequence[float]) -> Tuple[float, float, float]:
    """按阶段命令边界限幅；站立阶段只允许零命令。"""
    if not finite(command, 3):
        raise ValueError("命令必须是 3 个有限数")
    limits = phase_command_limits(tag)
    values = []
    for index, name in enumerate(COMMAND_FIELDS):
        low, high = limits[name]
        values.append(max(low, min(high, float(command[index]))))
    return values[0], values[1], values[2]


def sanitize_action(action: Sequence[float], previous: Sequence[float] | None = None) -> Tuple[float, ...]:
    """拒绝非有限动作，并执行范围和相邻周期变化率限制。"""
    if not finite(action, ACTION_DIM):
        raise ValueError("动作必须是 12 个有限数")
    values = tuple(float(value) for value in action)
    if previous is None:
        previous_values = (0.0,) * ACTION_DIM
    elif not finite(previous, ACTION_DIM):
        previous_values = (0.0,) * ACTION_DIM
    else:
        previous_values = tuple(float(value) for value in previous)
    return tuple(
        max(
            old - ACTION_RATE_LIMIT,
            min(old + ACTION_RATE_LIMIT, max(ACTION_LOW, min(ACTION_HIGH, value))),
        )
        for old, value in zip(previous_values, values)
    )


def action_to_target(action: Sequence[float]) -> Tuple[float, ...]:
    """把归一化动作映射到屈膝基准附近的受限关节目标。"""
    if not finite(action, ACTION_DIM):
        return DEFAULT_CROUCH
    return tuple(
        max(low, min(high, base + float(value) * scale))
        for value, base, scale, (low, high) in zip(
            action, DEFAULT_CROUCH, ACTION_SCALE, TARGET_POSITION_LIMITS
        )
    )


def limit_joint_targets(
    previous: Sequence[float],
    desired: Sequence[float],
    rate_limit: float | str | None = None,
) -> Tuple[float, ...]:
    """按统一 rad/s 阶梯限制相邻关节目标，并保留关节位置边界。"""
    if not finite(previous, ACTION_DIM) or not finite(desired, ACTION_DIM):
        raise ValueError("关节目标必须是 12 个有限数")
    selected_rate = resolve_action_target_rate_limit(rate_limit)
    max_delta = selected_rate * CONTROL_DT_SECONDS
    limited = tuple(
        max(float(old) - max_delta, min(float(old) + max_delta, float(target)))
        for old, target in zip(previous, desired)
    )
    return tuple(
        max(low, min(high, value))
        for value, (low, high) in zip(limited, TARGET_POSITION_LIMITS)
    )


def target_telemetry(
    desired: Sequence[float],
    executed: Sequence[float],
    *,
    rate_limit: float | str | None = None,
) -> Dict[str, Union[float, Tuple[float, ...]]]:
    """返回目标滞后向量和绝对滞后 mean/RMS/max，供三端统一记录。"""
    if not finite(desired, ACTION_DIM) or not finite(executed, ACTION_DIM):
        raise ValueError("目标遥测必须是 12 个有限数")
    delta = tuple(
        float(actual) - float(wanted)
        for wanted, actual in zip(desired, executed)
    )
    absolute = tuple(abs(value) for value in delta)
    mean = sum(absolute) / ACTION_DIM
    return {
        "desired": tuple(float(value) for value in desired),
        "executed": tuple(float(value) for value in executed),
        "delta": delta,
        "absolute_delta": absolute,
        "mean": mean,
        "rms": math.sqrt(sum(value * value for value in absolute) / ACTION_DIM),
        "max": max(absolute),
        "rate_limit": resolve_action_target_rate_limit(rate_limit),
    }


def action_saturation_rate(action: Sequence[float]) -> float:
    """计算归一化动作被裁到 [-1,1] 的分量比例。"""
    if not finite(action, ACTION_DIM):
        raise ValueError("动作必须是 12 个有限数")
    bound = max(abs(ACTION_LOW), abs(ACTION_HIGH))
    return sum(
        abs(float(value)) >= bound - 1e-6
        for value in action
    ) / ACTION_DIM


def rsi_targets(perturbation: Sequence[float] | None = None) -> Tuple[float, ...]:
    """返回与零动作完全一致的 RSI 关节目标，可叠加原始弧度扰动。"""
    if perturbation is None:
        offsets = (0.0,) * ACTION_DIM
    else:
        offsets = tuple(float(value) for value in perturbation)
        if len(offsets) != ACTION_DIM or not finite(offsets, ACTION_DIM):
            raise ValueError("RSI 扰动必须是 12 个有限数")
    return tuple(
        max(low, min(high, base + offset))
        for base, offset, (low, high) in zip(
            DEFAULT_CROUCH,
            offsets,
            TARGET_POSITION_LIMITS,
        )
    )


def validate_checkpoint_path(path: CheckpointPath) -> Path:
    """只允许新前缀且位于本项目 checkpoint 目录内的文件。"""
    result = Path(path).expanduser()
    lowered_parts = tuple(part.lower() for part in result.parts)
    lowered_name = result.name.lower()
    if any(prefix in lowered_name for prefix in FORBIDDEN_PREFIXES):
        raise ValueError(f"拒绝旧 checkpoint 前缀：{result.name}")
    if FORBIDDEN_DIRECTORY in lowered_parts:
        raise ValueError("拒绝废弃目录中的 checkpoint")
    if not (
        result.name == CONTRACT_VERSION
        or result.name.startswith(f"{CONTRACT_VERSION}_")
        or result.name.startswith(f"{CONTRACT_VERSION}-")
    ):
        raise ValueError(f"checkpoint 前缀不匹配：{result.name}")
    if "official-mini-cheetah" not in lowered_parts:
        raise ValueError("checkpoint 必须位于 official-mini-cheetah 目录")
    return result


def obs_layout_valid() -> bool:
    """验证 57 维切片连续且总和正确。"""
    if sum(item.stop - item.start for item in OBS_SLICES.values()) != OBS_DIM:
        return False
    cursor = 0
    for item in OBS_SLICES.values():
        if item.start != cursor:
            return False
        cursor = item.stop
    return cursor == OBS_DIM


if not obs_layout_valid():
    raise RuntimeError("57 维观测切片错误")
if len(MOTOR_NAMES) != ACTION_DIM or len(SENSOR_NAMES) != ACTION_DIM:
    raise RuntimeError("12 关节设备数量错误")
if len(DEFAULT_CROUCH) != ACTION_DIM or len(ACTION_SCALE) != ACTION_DIM:
    raise RuntimeError("屈膝姿态或动作尺度维度错误")
