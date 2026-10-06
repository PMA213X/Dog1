#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""平直地面移动与跳跃任务的分阶段奖励。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Mapping

import numpy as np

try:
    from . import flat_jump_contract as contract
except ImportError:
    import flat_jump_contract as contract  # type: ignore[no-redef]


WEIGHTS: Dict[str, Dict[str, float]] = {
    "F0": {
        "alive": 1.0,
        "posture": -2.0,
        "height": -1.0,
        "contact": 0.1,
        "command": 0.0,
        "velocity": 0.0,
        "heading": 0.0,
        "energy": -0.05,
        "smoothness": -0.10,
        "jump_height": 0.0,
        "jump_success": 0.0,
        "jump_landing": 0.0,
        "fall": -20.0,
    },
    "F1": {
        "alive": 1.0,
        "posture": -2.0,
        "height": -1.0,
        "contact": 0.1,
        "command": -1.0,
        "velocity": 0.0,
        "heading": 0.0,
        "energy": -0.05,
        "smoothness": -0.10,
        "jump_height": 0.0,
        "jump_success": 0.0,
        "jump_landing": 0.0,
        "fall": -100.0,
    },
    "F2": {
        "alive": 1.0,
        "posture": -2.0,
        "height": -1.0,
        "contact": 0.1,
        "command": -1.5,
        "velocity": 1.0,
        "heading": 0.5,
        "energy": -0.05,
        "smoothness": -0.10,
        "jump_height": 0.0,
        "jump_success": 0.0,
        "jump_landing": 0.0,
        "fall": -20.0,
    },
    "F3": {
        "alive": 1.0,
        "posture": -2.0,
        "height": -1.0,
        "contact": 0.1,
        "command": -1.5,
        "velocity": 1.0,
        "heading": 0.5,
        "energy": -0.05,
        "smoothness": -0.10,
        "jump_height": 2.0,
        "jump_success": 8.0,
        "jump_landing": 4.0,
        "fall": -20.0,
    },
}


def _finite(value: float) -> float:
    """把标量转换为有限浮点数。"""
    number = float(value)
    if not np.isfinite(number):
        raise ValueError(f"奖励输入包含 NaN/Inf：{number}")
    return number


@dataclass(frozen=True)
class RewardInputs:
    """奖励所需的最小状态。"""

    phase: str
    roll: float
    pitch: float
    height: float
    command: np.ndarray
    velocity: np.ndarray
    heading_error: float
    contacts: np.ndarray
    action: np.ndarray
    previous_action: np.ndarray
    jump_latched: bool
    jump_peak_gain: float
    jump_success_event: bool
    jump_landing_event: bool
    fallen: bool


@dataclass(frozen=True)
class RewardBreakdown:
    """奖励分项。"""

    values: Dict[str, float]

    @property
    def total(self) -> float:
        """返回有限总奖励。"""
        total = float(sum(self.values.values()))
        if not np.isfinite(total):
            raise ValueError("奖励总计包含 NaN/Inf")
        return total

    def as_dict(self) -> Dict[str, float]:
        """返回可写入日志的分项。"""
        return dict(self.values)


def _velocity_alignment(velocity: np.ndarray, command: np.ndarray) -> float:
    """计算平移和转向方向对齐度。"""
    linear_command = np.asarray(command[:2], dtype=np.float64)
    linear_velocity = np.asarray(velocity[:2], dtype=np.float64)
    command_norm = float(np.linalg.norm(linear_command))
    speed_norm = max(0.6 * command_norm, 0.1)
    if command_norm >= 0.05:
        linear = float(np.dot(linear_velocity, linear_command) / speed_norm)
        linear = float(np.clip(linear, 0.0, 1.0))
    else:
        linear = -float(
            np.clip(np.linalg.norm(linear_velocity) / 0.1, 0.0, 1.0)
        )
    yaw_command = float(command[2])
    yaw_velocity = float(velocity[2])
    if abs(yaw_command) >= 0.1:
        angular = float(np.clip(yaw_velocity * np.sign(yaw_command), 0.0, 1.0))
    else:
        angular = -float(np.clip(abs(yaw_velocity) / 0.1, 0.0, 1.0))
    return 0.5 * (linear + angular)


def compute_reward(inputs: RewardInputs) -> RewardBreakdown:
    """计算单步奖励，任何非有限输入都立即报错。"""
    phase = contract.normalize_phase(inputs.phase)
    weights = WEIGHTS[phase]
    action = np.asarray(inputs.action, dtype=np.float64)
    previous = np.asarray(inputs.previous_action, dtype=np.float64)
    command = np.asarray(inputs.command, dtype=np.float64)
    velocity = np.asarray(inputs.velocity, dtype=np.float64)
    contacts = np.asarray(inputs.contacts, dtype=np.float64)
    if not all(
        np.all(np.isfinite(value))
        for value in (action, previous, command, velocity, contacts)
    ):
        raise ValueError("奖励数组包含 NaN/Inf")

    command_error = float(
        np.linalg.norm(np.asarray([velocity[0], velocity[1], velocity[2]]) - command)
    )
    jump_blocked = bool(inputs.jump_latched)
    values = {
        "alive": _finite(weights["alive"]),
        "posture": weights["posture"]
        * (_finite(inputs.roll) ** 2 + _finite(inputs.pitch) ** 2),
        "height": 0.0
        if jump_blocked
        else weights["height"] * abs(_finite(inputs.height) - 0.26),
        "contact": 0.0
        if jump_blocked
        else weights["contact"] * float(np.sum(contacts)),
        "command": weights["command"] * command_error,
        "velocity": weights["velocity"] * _velocity_alignment(velocity, command),
        "heading": weights["heading"] * float(np.cos(inputs.heading_error)),
        "energy": weights["energy"] * float(np.dot(action, action)),
        "smoothness": weights["smoothness"]
        * float(np.dot(action - previous, action - previous)),
        "jump_height": 0.0,
        "jump_success": 0.0,
        "jump_landing": 0.0,
        "fall": weights["fall"] if inputs.fallen else 0.0,
    }
    if jump_blocked:
        values["jump_height"] = weights["jump_height"] * min(
            max(_finite(inputs.jump_peak_gain), 0.0), 0.15
        )
    if inputs.jump_success_event:
        values["jump_success"] = weights["jump_success"]
    if inputs.jump_landing_event:
        values["jump_landing"] = weights["jump_landing"]
    return RewardBreakdown(
        {name: _finite(value) for name, value in values.items()}
    )


def reward_for_state(
    phase: str,
    state: Mapping[str, object],
    action: np.ndarray,
    previous_action: np.ndarray,
    *,
    command: np.ndarray,
    heading_error: float,
    jump_latched: bool,
    jump_peak_gain: float,
    jump_success_event: bool,
    jump_landing_event: bool,
    fallen: bool,
) -> tuple[float, Dict[str, float]]:
    """从状态构造奖励。"""
    rpy = np.asarray(state["rpy"], dtype=np.float64)
    velocity = np.asarray(state["v_body"], dtype=np.float64)
    omega = np.asarray(state["omega"], dtype=np.float64)
    measured = np.asarray(
        [velocity[0], velocity[1], omega[2]], dtype=np.float64
    )
    result = compute_reward(
        RewardInputs(
            phase=phase,
            roll=float(rpy[0]),
            pitch=float(rpy[1]),
            height=float(state["height"]),
            command=np.asarray(command, dtype=np.float64),
            velocity=measured,
            heading_error=float(heading_error),
            contacts=np.asarray(state["contacts"], dtype=np.float64),
            action=np.asarray(action, dtype=np.float64),
            previous_action=np.asarray(previous_action, dtype=np.float64),
            jump_latched=bool(jump_latched),
            jump_peak_gain=float(jump_peak_gain),
            jump_success_event=bool(jump_success_event),
            jump_landing_event=bool(jump_landing_event),
            fallen=bool(fallen),
        )
    )
    return result.total, result.as_dict()
