#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""yobogo_loco_jump_v1 分阶段奖励计算。

本模块只负责把环境状态与阶段标签折算成单步奖励，不依赖 Gymnasium、
Stable-Baselines3 或 Webots，便于单元测试和后续调参。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Mapping, Tuple

import numpy as np

try:
    from . import loco_jump_contract as contract
except ImportError:  # 直接以脚本/同目录模块方式导入
    import loco_jump_contract as contract  # type: ignore[no-redef]


# 标准阶段标签与契约累计目标保持一一对应。
STAGE_BY_TAG: Dict[str, str] = {
    "phase0": "S0_smoke",
    "phase1": "S1_stand",
    "phase2": "S2_command",
    "phase3": "S3_jump",
    "phase4": "S4_mobile_terrain",
}

TAG_BY_STAGE: Dict[str, str] = {stage: tag for tag, stage in STAGE_BY_TAG.items()}


def normalize_stage(tag: str) -> str:
    """把 CLI/环境使用的阶段标签规范化为契约阶段 ID。"""
    raw = str(tag).strip()
    lowered = raw.lower()
    if lowered in STAGE_BY_TAG:
        return STAGE_BY_TAG[lowered]
    # 允许直接传 S0_smoke、S1_stand 等契约名称，大小写不敏感。
    for stage in contract.PHASE_TOTAL_STEPS:
        if lowered == stage.lower():
            return stage
    raise ValueError(
        f"未知阶段 {tag!r}；可选 {list(STAGE_BY_TAG)} 或 {list(contract.PHASE_TOTAL_STEPS)}"
    )


def stage_tag(stage: str) -> str:
    """返回阶段对应的训练 CLI 标签。"""
    return TAG_BY_STAGE[normalize_stage(stage)]


@dataclass(frozen=True)
class RewardInputs:
    """单步奖励所需的最小状态切片。

    ``command_error`` 为速度/偏航跟踪误差范数，``heading_error`` 为相对目标
    偏航角的包裹误差；两者由环境计算并保证为有限浮点数。
    """

    roll: float
    pitch: float
    body_height: float
    action: np.ndarray
    previous_action: np.ndarray
    command_error: float
    heading_error: float
    jump_success_event: bool = False
    terrain_success_event: bool = False
    fallen: bool = False


@dataclass(frozen=True)
class RewardBreakdown:
    """奖励分项与总计，供训练日志与测试检查。"""

    stage: str
    alive: float
    posture: float
    height: float
    smoothness: float
    command: float
    heading: float
    jump: float
    terrain: float
    fall: float

    @property
    def total(self) -> float:
        total = (
            self.alive
            + self.posture
            + self.height
            + self.smoothness
            + self.command
            + self.heading
            + self.jump
            + self.terrain
            + self.fall
        )
        # 奖励不得因异常输入变成 NaN/Inf；异常值统一按 0 处理。
        return float(np.nan_to_num(total, nan=0.0, posinf=0.0, neginf=0.0))

    def as_dict(self) -> Dict[str, float]:
        """返回可直接写入 info/日志的奖励分项。"""
        return {
            "alive": self.alive,
            "posture": self.posture,
            "height": self.height,
            "smoothness": self.smoothness,
            "command": self.command,
            "heading": self.heading,
            "jump": self.jump,
            "terrain": self.terrain,
            "fall": self.fall,
            "total": self.total,
        }


# 阶段权重表：基础项只表达稳定站立，后续阶段逐步叠加命令、跳跃、越障。
# 所有系数均为显式数值，避免依赖全局配置在运行中被别的训练进程改写。
PHASE_REWARD_WEIGHTS: Dict[str, Dict[str, float]] = {
    "S0_smoke": {
        "alive": 1.0,
        "posture": -2.0,
        "height": -2.0,
        "smoothness": -0.05,
        "command": 0.0,
        "heading": 0.0,
        "jump": 0.0,
        "terrain": 0.0,
        "fall": -20.0,
    },
    "S1_stand": {
        "alive": 1.0,
        "posture": -2.0,
        "height": -2.0,
        "smoothness": -0.05,
        "command": 0.0,
        "heading": 0.0,
        "jump": 0.0,
        "terrain": 0.0,
        "fall": -20.0,
    },
    "S2_command": {
        "alive": 1.0,
        "posture": -2.0,
        "height": -2.0,
        "smoothness": -0.05,
        "command": -2.0,
        "heading": -0.5,
        "jump": 0.0,
        "terrain": 0.0,
        "fall": -20.0,
    },
    "S3_jump": {
        "alive": 1.0,
        "posture": -2.0,
        "height": -2.0,
        "smoothness": -0.05,
        "command": -2.0,
        "heading": -0.5,
        "jump": 8.0,
        "terrain": 0.0,
        "fall": -20.0,
    },
    "S4_mobile_terrain": {
        "alive": 1.0,
        "posture": -2.0,
        "height": -2.0,
        "smoothness": -0.05,
        "command": -2.0,
        "heading": -0.5,
        "jump": 8.0,
        "terrain": 10.0,
        "fall": -20.0,
    },
}


def _finite(value: float) -> float:
    """把任意标量转成有限 float，非法值按 0 处理。"""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0
    return float(np.nan_to_num(number, nan=0.0, posinf=0.0, neginf=0.0))


def _phase_offset(height: float) -> float:
    """机身高度相对站立目标 0.26 m 的偏差。"""
    return _finite(height) - 0.26


def compute_reward(stage: str, inputs: RewardInputs) -> RewardBreakdown:
    """按阶段计算有限的单步奖励。

    S0/S1 只奖励存活与姿态稳定；S2 叠加命令跟踪；S3 在此之上奖励跳跃成功；
    S4 再奖励成功越障。跳跃/越障只在事件发生的当步计入，避免连续刷奖励。
    """
    canonical_stage = normalize_stage(stage)
    weights = PHASE_REWARD_WEIGHTS[canonical_stage]

    action = np.asarray(inputs.action, dtype=np.float64).reshape(-1)
    previous_action = np.asarray(inputs.previous_action, dtype=np.float64).reshape(-1)
    if action.size != previous_action.size:
        action_delta = np.nan_to_num(action, nan=0.0, posinf=0.0, neginf=0.0)
        previous_action = np.zeros_like(action_delta)
    else:
        action_delta = np.nan_to_num(
            action - previous_action, nan=0.0, posinf=0.0, neginf=0.0
        )

    alive = weights["alive"]
    posture = weights["posture"] * (
        abs(_finite(inputs.roll)) + abs(_finite(inputs.pitch))
    )
    height = weights["height"] * abs(_phase_offset(inputs.body_height))
    smoothness = weights["smoothness"] * float(np.dot(action_delta, action_delta))
    command = weights["command"] * abs(_finite(inputs.command_error))
    heading = weights["heading"] * abs(_finite(inputs.heading_error))
    jump = weights["jump"] if inputs.jump_success_event else 0.0
    terrain = weights["terrain"] if inputs.terrain_success_event else 0.0
    fall = weights["fall"] if inputs.fallen else 0.0

    return RewardBreakdown(
        stage=canonical_stage,
        alive=float(_finite(alive)),
        posture=float(_finite(posture)),
        height=float(_finite(height)),
        smoothness=float(_finite(smoothness)),
        command=float(_finite(command)),
        heading=float(_finite(heading)),
        jump=float(_finite(jump)),
        terrain=float(_finite(terrain)),
        fall=float(_finite(fall)),
    )


def reward_for_state(
    stage: str,
    state: Mapping[str, object],
    action: np.ndarray,
    previous_action: np.ndarray,
    *,
    command_error: float,
    heading_error: float,
    fallen: bool,
    jump_success_event: bool = False,
    terrain_success_event: bool = False,
) -> Tuple[float, RewardBreakdown]:
    """从 TCP state 的常用字段构造输入并返回 ``(total, breakdown)``。"""
    rpy = np.asarray(state.get("rpy", (0.0, 0.0, 0.0)), dtype=np.float64).reshape(-1)
    height = float(state.get("height", state.get("z", 0.26)))
    inputs = RewardInputs(
        roll=float(rpy[0]) if rpy.size > 0 else 0.0,
        pitch=float(rpy[1]) if rpy.size > 1 else 0.0,
        body_height=height,
        action=np.asarray(action, dtype=np.float32),
        previous_action=np.asarray(previous_action, dtype=np.float32),
        command_error=command_error,
        heading_error=heading_error,
        jump_success_event=bool(jump_success_event),
        terrain_success_event=bool(terrain_success_event),
        fallen=bool(fallen),
    )
    breakdown = compute_reward(stage, inputs)
    return breakdown.total, breakdown

