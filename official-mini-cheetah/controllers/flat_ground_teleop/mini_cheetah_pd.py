#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""官方 Mini Cheetah PD 控制入口。

实际实现集中在 :mod:`joint_safety`，本文件保留任务要求的模块名，并把
常用常量、设备名和纯函数暴露给控制器及单测。
"""

from __future__ import annotations

from joint_safety import (
    FLAT_MOTOR_NAMES,
    FLAT_SENSOR_NAMES,
    DEFAULT_CROUCH,
    JOINT_COUNT,
    JOINT_NAMES,
    KD,
    KP,
    LEG_NAMES,
    MAX_ABS_JOINT_POSITION,
    MAX_ABS_JOINT_VELOCITY,
    MAX_TARGET_RATE,
    MAX_TORQUE,
    SLIGHTLY_EXTENDED,
    TARGET_POSITION_LIMITS,
    MiniCheetahGait,
    MOTOR_NAMES,
    SENSOR_NAMES,
    STANDING_TARGETS,
    WEBOTS_MOTOR_MAX_TORQUE,
    JointTargets,
    TargetRateLimiter,
    clamp_joint_targets,
    clamp_torque,
    compute_joint_torques,
    compute_pd_torques,
    finite_sequence,
    joint_pd_torque,
    limit_target_rate,
    standing_targets,
    validate_joint_feedback,
)


__all__ = [
    "FLAT_MOTOR_NAMES",
    "FLAT_SENSOR_NAMES",
    "DEFAULT_CROUCH",
    "JOINT_COUNT",
    "JOINT_NAMES",
    "KD",
    "KP",
    "LEG_NAMES",
    "MAX_ABS_JOINT_POSITION",
    "MAX_ABS_JOINT_VELOCITY",
    "MAX_TARGET_RATE",
    "MAX_TORQUE",
    "SLIGHTLY_EXTENDED",
    "TARGET_POSITION_LIMITS",
    "MiniCheetahGait",
    "MOTOR_NAMES",
    "SENSOR_NAMES",
    "STANDING_TARGETS",
    "WEBOTS_MOTOR_MAX_TORQUE",
    "JointTargets",
    "TargetRateLimiter",
    "clamp_joint_targets",
    "clamp_torque",
    "compute_joint_torques",
    "compute_pd_torques",
    "finite_sequence",
    "joint_pd_torque",
    "limit_target_rate",
    "standing_targets",
    "validate_joint_feedback",
]
