#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""平直地面移动与跳跃任务的公共契约。

本模块只定义 57 维观测、12 维动作、命令域、阶段预算和 checkpoint 边界，
不包含训练或 Webots 控制逻辑。
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Sequence, Tuple, Union

CONTRACT_VERSION = "yobogo_flat_jump_v1"
CONTROL_RATE_HZ = 50
CONTROL_DT_SECONDS = 0.02

LEGACY_OBS_DIM = 55
TERRAIN_OBS_DIM = 2
OBS_DIM = 57
ACTION_DIM = 12
ACTION_LOW = -1.0
ACTION_HIGH = 1.0
ACTION_BOUNDS = (ACTION_LOW, ACTION_HIGH)
ACTION_RATE_LIMIT = 0.15

OBS_FIELD_ORDER: Tuple[str, ...] = (
    "q",
    "dq",
    "rpy",
    "v_body",
    "prev_action",
    "omega_body",
    "cmd_vx_vy_wz",
    "jump_request",
    "jump_phase_time",
    "body_height",
    "foot_contact",
    "terrain_height",
    "terrain_flat_flag",
)

OBS_SLICES: Dict[str, slice] = {
    "q": slice(0, 12),
    "dq": slice(12, 24),
    "rpy": slice(24, 27),
    "v_body": slice(27, 30),
    "prev_action": slice(30, 42),
    "omega_body": slice(42, 45),
    "cmd_vx_vy_wz": slice(45, 48),
    "jump_request": slice(48, 49),
    "jump_phase_time": slice(49, 50),
    "body_height": slice(50, 51),
    "foot_contact": slice(51, 55),
    "terrain_height": slice(55, 56),
    "terrain_flat_flag": slice(56, 57),
}

OBS_FIELD_DIMS: Dict[str, int] = {
    name: value.stop - value.start for name, value in OBS_SLICES.items()
}
FOOT_CONTACT_ORDER: Tuple[str, ...] = ("fr", "fl", "hr", "hl")

COMMAND_FIELDS: Tuple[str, ...] = ("vx", "vy", "wz")
COMMAND_LIMITS: Dict[str, Tuple[float, float]] = {
    "vx": (-0.3, 0.6),
    "vy": (-0.3, 0.3),
    "wz": (-1.0, 1.0),
}
COMMAND_LOW = tuple(COMMAND_LIMITS[name][0] for name in COMMAND_FIELDS)
COMMAND_HIGH = tuple(COMMAND_LIMITS[name][1] for name in COMMAND_FIELDS)
COMMAND_RESAMPLE_STEPS = 250
ZERO_COMMAND_PROBABILITY = 0.20

JUMP_REQUEST_STEP_MIN = 100
JUMP_REQUEST_STEP_MAX = 200
JUMP_INTERVAL_MIN = 200
JUMP_INTERVAL_MAX = 350
JUMP_SUCCESS_HEIGHT_GAIN = 0.04
JUMP_LATCH_TIMEOUT_SECONDS = 1.0
JUMP_CONTACT_CLEAR_AFTER_CONTACT_LOSS = True

TCP_STATE_TYPE = "state"
TCP_STATE_REQUIRED_FIELDS: Tuple[str, ...] = (
    "q",
    "dq",
    "rpy",
    "omega",
    "v_body",
    "contacts",
    "height",
    "jump_phase",
    "done",
)
TCP_STATE_ARRAY_LENGTHS: Dict[str, int] = {
    "q": 12,
    "dq": 12,
    "rpy": 3,
    "omega": 3,
    "v_body": 3,
    "contacts": 4,
}
TCP_STATE_SCALAR_FIELDS: Tuple[str, ...] = (
    "height",
    "jump_phase",
    "done",
)

MAX_EPISODE_STEPS = 1000
WORKSPACE_LIMIT = 8.0
MIN_BASE_HEIGHT = 0.12
MAX_BASE_HEIGHT = 0.60
ROLL_PITCH_LIMIT = 0.8

FRICTION_RANGE = (0.9, 1.3)
MASS_SCALE_RANGE = (0.95, 1.05)
DELAY_STEPS_RANGE = (0, 2)
F1_CURRICULUM_EPISODES = 20
F1_CURRICULUM_FIXED_EPISODES = 10
F1_FIXED_RECOVERY_STEPS = 260_000
F1_RECOVERY_TOTAL_STEPS = 350_000

PHASE_TOTAL_STEPS: Dict[str, int] = {
    "F0": 5_000,
    "F1": 350_000,
    "F2": 650_000,
    "F3": 1_200_000,
}
PHASE_NAMES: Dict[str, str] = {
    "F0": "冒烟",
    "F1": "稳定",
    "F2": "移动",
    "F3": "跳跃整合",
}
CHECKPOINT_PREFIX = CONTRACT_VERSION
CHECKPOINT_INTERVAL_STEPS = 50_000
CHECKPOINT_INTERVAL_SECONDS = 1_800.0

CheckpointPath = Union[str, Path]


def normalize_phase(tag: str) -> str:
    """规范化阶段标签。"""
    value = str(tag).strip().upper()
    if value in PHASE_TOTAL_STEPS:
        return value
    raise ValueError(f"未知阶段 {tag!r}；可选 {list(PHASE_TOTAL_STEPS)}")


def clip_command(command: Sequence[float]) -> Tuple[float, float, float]:
    """按契约限幅命令。"""
    if len(command) != len(COMMAND_FIELDS):
        raise ValueError("命令维度必须为 3")
    values = []
    for index, name in enumerate(COMMAND_FIELDS):
        low, high = COMMAND_LIMITS[name]
        value = float(command[index])
        values.append(min(high, max(low, value)))
    return values[0], values[1], values[2]


def clip_jump_phase_time(elapsed_seconds: float) -> float:
    """把跳跃锁存时间裁剪到观测范围。"""
    value = float(elapsed_seconds)
    return min(1.0, max(0.0, value))


def should_clear_jump_latch(
    *,
    jump_latched: bool,
    contacts_restored: bool,
    elapsed_seconds: float,
) -> bool:
    """判断跳跃锁存是否清除。"""
    if not jump_latched:
        return False
    if contacts_restored and JUMP_CONTACT_CLEAR_AFTER_CONTACT_LOSS:
        return True
    return elapsed_seconds >= JUMP_LATCH_TIMEOUT_SECONDS


def validate_checkpoint_path(path: CheckpointPath) -> Path:
    """只允许新前缀 checkpoint，拒绝旧任务文件。"""
    result = Path(path).expanduser()
    name = result.name
    valid = (
        name == CHECKPOINT_PREFIX
        or name.startswith(f"{CHECKPOINT_PREFIX}_")
        or name.startswith(f"{CHECKPOINT_PREFIX}-")
        or name.startswith(f"{CHECKPOINT_PREFIX}.")
    )
    if not valid:
        raise ValueError(
            f"checkpoint 前缀不匹配：{name!r}；"
            f"只允许 {CHECKPOINT_PREFIX}"
        )
    return result


if sum(OBS_FIELD_DIMS.values()) != OBS_DIM:
    raise RuntimeError("57 维观测切片合计错误")
if tuple(OBS_SLICES) != OBS_FIELD_ORDER:
    raise RuntimeError("观测字段顺序错误")
if len(FOOT_CONTACT_ORDER) != OBS_FIELD_DIMS["foot_contact"]:
    raise RuntimeError("足端接触维数错误")
