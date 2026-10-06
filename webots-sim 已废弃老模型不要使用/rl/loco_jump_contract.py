#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""yobogo_loco_jump_v1 公共契约。

本模块只定义环境、控制器、训练器和监控侧共同遵守的数据契约，不包含
训练逻辑或 Webots 控制逻辑。观测前 42 维保持现有行走环境布局不变，
在其后追加 13 维运动控制与跳跃状态，合计 55 维。
"""

from __future__ import annotations

from os import PathLike
from pathlib import Path
from typing import Dict, Sequence, Tuple, Union

# ---------------------------------------------------------------------------
# 版本与控制周期
# ---------------------------------------------------------------------------
CONTRACT_VERSION: str = "yobogo_loco_jump_v1"
CONTROL_RATE_HZ: int = 50
CONTROL_DT_SECONDS: float = 0.02
# 与项目现有 config.py / walk_env.py 的命名保持兼容。
SIM_DT: float = CONTROL_DT_SECONDS

# ---------------------------------------------------------------------------
# 观测契约：旧 42 维原切片不动 + 新增 13 维
# ---------------------------------------------------------------------------
LEGACY_OBS_DIM: int = 42
ADDED_OBS_DIM: int = 13
OBS_DIM: int = 55

# 按策略网络输入从低索引到高索引排列，顺序不可修改。
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
)

OBS_SLICES: Dict[str, slice] = {
    "q": slice(0, 12),                    # 关节角，rad
    "dq": slice(12, 24),                  # 关节角速度，rad/s
    "rpy": slice(24, 27),                 # roll/pitch/yaw，rad
    "v_body": slice(27, 30),              # 机体系线速度，m/s
    "prev_action": slice(30, 42),         # 上一动作，无量纲 [-1, 1]
    "omega_body": slice(42, 45),          # 机体系角速度，rad/s
    "cmd_vx_vy_wz": slice(45, 48),        # 命令 [vx, vy, wz]
    "jump_request": slice(48, 49),        # 跳跃锁存，0/1
    "jump_phase_time": slice(49, 50),     # 本次跳跃经过时间，秒，clip 到 [0,1]
    "body_height": slice(50, 51),         # 机身高度，m
    "foot_contact": slice(51, 55),        # 足端接触，0/1，顺序见 FOOT_CONTACT_ORDER
}

# 各字段维数与切片一一对应，便于运行期自检。
OBS_FIELD_DIMS: Dict[str, int] = {
    name: obs_slice.stop - obs_slice.start
    for name, obs_slice in OBS_SLICES.items()
}

# 单位/值域说明；该映射同样按 OBS_FIELD_ORDER 排列。
OBS_FIELD_UNITS: Dict[str, str] = {
    "q": "rad",
    "dq": "rad/s",
    "rpy": "rad",
    "v_body": "m/s",
    "prev_action": "dimensionless",
    "omega_body": "rad/s",
    "cmd_vx_vy_wz": "vx,vy:m/s;wz:rad/s",
    "jump_request": "binary",
    "jump_phase_time": "s",
    "body_height": "m",
    "foot_contact": "binary",
}

FOOT_CONTACT_ORDER: Tuple[str, ...] = ("fr", "fl", "hr", "hl")

# ---------------------------------------------------------------------------
# 动作与命令契约
# ---------------------------------------------------------------------------
ACTION_DIM: int = 12
ACTION_LOW: float = -1.0
ACTION_HIGH: float = 1.0
ACTION_BOUNDS: Tuple[float, float] = (ACTION_LOW, ACTION_HIGH)

COMMAND_FIELDS: Tuple[str, ...] = ("vx", "vy", "wz")
COMMAND_LIMITS: Dict[str, Tuple[float, float]] = {
    "vx": (-0.3, 0.6),   # 机体系前进速度，m/s
    "vy": (-0.3, 0.3),   # 机体系侧向速度，m/s
    "wz": (-1.0, 1.0),   # 机体系偏航角速度，rad/s
}
COMMAND_LOW: Tuple[float, float, float] = (
    COMMAND_LIMITS["vx"][0],
    COMMAND_LIMITS["vy"][0],
    COMMAND_LIMITS["wz"][0],
)
COMMAND_HIGH: Tuple[float, float, float] = (
    COMMAND_LIMITS["vx"][1],
    COMMAND_LIMITS["vy"][1],
    COMMAND_LIMITS["wz"][1],
)

JUMP_REQUEST_BOUNDS: Tuple[float, float] = (0.0, 1.0)


def clip_command(command: Sequence[float]) -> Tuple[float, float, float]:
    """按契约限幅并返回 [vx, vy, wz]。"""
    if len(command) != len(COMMAND_FIELDS):
        raise ValueError(
            f"命令维度错误：期望 {len(COMMAND_FIELDS)}，收到 {len(command)}"
        )
    clipped = tuple(
        min(
            COMMAND_LIMITS[field][1],
            max(COMMAND_LIMITS[field][0], float(command[index])),
        )
        for index, field in enumerate(COMMAND_FIELDS)
    )
    return clipped[0], clipped[1], clipped[2]


def validate_jump_request(value: float) -> int:
    """校验跳跃请求只能取 0 或 1。"""
    if value not in JUMP_REQUEST_BOUNDS:
        raise ValueError(
            f"跳跃请求必须为 0 或 1，收到 {value!r}"
        )
    return int(value)


# ---------------------------------------------------------------------------
# 跳跃锁存语义
# ---------------------------------------------------------------------------
JUMP_TRIGGER_NAMES: Tuple[str, ...] = ("Space", "gamepad_A")
JUMP_LATCH_TIMEOUT_SECONDS: float = 1.0
JUMP_PHASE_TIME_CLIP: Tuple[float, float] = (0.0, 1.0)
JUMP_CONTACT_CLEAR_AFTER_CONTACT_LOSS: bool = True


def clip_jump_phase_time(elapsed_seconds: float) -> float:
    """把本次锁存经过秒数裁剪到观测规定的 [0, 1]。"""
    low, high = JUMP_PHASE_TIME_CLIP
    return min(high, max(low, float(elapsed_seconds)))


def should_clear_jump_latch(
    *,
    jump_latched: bool,
    contacts_restored: bool,
    elapsed_seconds: float,
) -> bool:
    """判断当前跳跃锁存是否应清除。

    Space 或手柄 A 的上升沿把锁存置 1；锁存持续到足端恢复接触，
    或本次锁存经过 JUMP_LATCH_TIMEOUT_SECONDS 秒。调用方须先完成
    “离开接触后再恢复接触”的检测，避免触发起跳瞬间立即清除。
    """
    if not jump_latched:
        return False
    if contacts_restored and JUMP_CONTACT_CLEAR_AFTER_CONTACT_LOSS:
        return True
    if elapsed_seconds >= JUMP_LATCH_TIMEOUT_SECONDS:
        return True
    return False


# ---------------------------------------------------------------------------
# TCP state 契约
# ---------------------------------------------------------------------------
TCP_STATE_TYPE: str = "state"
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
    "height",       # 机身高度，m
    "jump_phase",   # 本次锁存经过时间，秒，范围 [0,1]
    "done",         # episode 是否结束，布尔值
)
TCP_STATE_ARRAY_FIELDS: Tuple[str, ...] = tuple(TCP_STATE_ARRAY_LENGTHS)
TCP_STATE_CONTACT_ORDER: Tuple[str, ...] = FOOT_CONTACT_ORDER


# ---------------------------------------------------------------------------
# 阶段总步数配置
# ---------------------------------------------------------------------------
# 下列数值是全进程累计总步数目标，不是各阶段独立步数。
S0_SMOKE_TOTAL_STEPS: int = 5_000
S1_STAND_TOTAL_STEPS: int = 200_000
S2_COMMAND_TOTAL_STEPS: int = 500_000
S3_JUMP_TOTAL_STEPS: int = 800_000
S4_MOBILE_TERRAIN_TOTAL_STEPS: int = 1_200_000

PHASE_TOTAL_STEPS: Dict[str, int] = {
    "S0_smoke": S0_SMOKE_TOTAL_STEPS,
    "S1_stand": S1_STAND_TOTAL_STEPS,
    "S2_command": S2_COMMAND_TOTAL_STEPS,
    "S3_jump": S3_JUMP_TOTAL_STEPS,
    "S4_mobile_terrain": S4_MOBILE_TERRAIN_TOTAL_STEPS,
}
# 便于明确“目标总步数”语义的别名。
PHASE_TARGET_TOTAL_STEPS: Dict[str, int] = PHASE_TOTAL_STEPS

# S0 只冒烟，不计入正式训练预算；正式训练范围为 [800000, 1200000]。
FORMAL_TRAINING_START_STEP: int = 800_000
FORMAL_TRAINING_END_STEP: int = 1_200_000
FORMAL_TRAINING_STEP_RANGE: Tuple[int, int] = (
    FORMAL_TRAINING_START_STEP,
    FORMAL_TRAINING_END_STEP,
)


# ---------------------------------------------------------------------------
# checkpoint 隔离
# ---------------------------------------------------------------------------
CHECKPOINT_PREFIX: str = "yobogo_loco_jump_v1"
# 满足“每 50000 步或 30 分钟”任一条件即应保存。
CHECKPOINT_INTERVAL_STEPS: int = 50_000
CHECKPOINT_INTERVAL_SECONDS: float = 1_800.0

CheckpointPath = Union[str, PathLike[str]]


def _has_checkpoint_prefix(name: str, prefix: str) -> bool:
    """检查文件名是否以完整前缀开头，避免 `v1beta` 误匹配 `v1`。"""
    if not name.startswith(prefix):
        return False
    if len(name) == len(prefix):
        return True
    return name[len(prefix)] in {"_", "-", "."}


def is_isolated_checkpoint(path: CheckpointPath) -> bool:
    """判断路径文件名是否属于 yobogo_loco_jump_v1，旧前缀返回 False。"""
    return _has_checkpoint_prefix(Path(path).name, CHECKPOINT_PREFIX)


def validate_checkpoint_path(path: CheckpointPath) -> Path:
    """校验并返回 checkpoint 路径；旧 checkpoint 一律拒绝。"""
    result = Path(path).expanduser()
    if not _has_checkpoint_prefix(result.name, CHECKPOINT_PREFIX):
        raise ValueError(
            f"checkpoint 前缀不匹配：{result.name!r}；"
            f"只允许 {CHECKPOINT_PREFIX!r}，旧 checkpoint 必须拒绝"
        )
    return result


# ---------------------------------------------------------------------------
# 导入期结构自检
# ---------------------------------------------------------------------------
if sum(OBS_FIELD_DIMS.values()) != OBS_DIM:
    raise RuntimeError("观测字段维数合计与 OBS_DIM 不一致")
if tuple(OBS_SLICES) != OBS_FIELD_ORDER:
    raise RuntimeError("观测切片顺序与 OBS_FIELD_ORDER 不一致")
if len(COMMAND_FIELDS) != 3:
    raise RuntimeError("命令字段必须按 vx/vy/wz 顺序共 3 维")
if len(FOOT_CONTACT_ORDER) != OBS_FIELD_DIMS["foot_contact"]:
    raise RuntimeError("足端接触字段数量与观测切片不一致")
if sorted(TCP_STATE_REQUIRED_FIELDS) != sorted(
    TCP_STATE_ARRAY_FIELDS + TCP_STATE_SCALAR_FIELDS
):
    raise RuntimeError("TCP state 字段分类与必需字段列表不一致")
