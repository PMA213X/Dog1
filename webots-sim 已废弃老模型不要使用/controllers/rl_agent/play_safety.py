#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Webots play 模式的纯逻辑安全层。

本模块不依赖 Webots 设备，也不加载模型，方便用普通 Python 单元测试覆盖：
  1. checkpoint 相对路径的双候选解析与公共契约校验；
  2. reset 后的零动作稳定窗口和四足接触放行条件；
  3. 原始动作绝对限幅、单周期变化率限制和放行诊断。
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass
from os import PathLike
from pathlib import Path
from typing import Any, Callable, Union

import numpy as np


# play_safety 与 rl_agent 同目录；独立导入时补齐公共契约所在目录。
_RL_DIR = Path(__file__).resolve().parents[2] / "rl"
if str(_RL_DIR) not in sys.path:
    sys.path.insert(0, str(_RL_DIR))

if os.environ.get("RL_CONTRACT_VERSION", "").strip() == "flat_v1":
    from flat_jump_contract import validate_checkpoint_path  # noqa: E402
else:
    from loco_jump_contract import validate_checkpoint_path  # noqa: E402


# reset 后至少保持 50 个 50 Hz 零动作周期，最多等待 100 周期。
STABLE_WINDOW_CYCLES = 50
STABLE_CONTACT_CYCLES = 10
STABLE_TIMEOUT_CYCLES = 100

# 机身超过 0.60 m 视为异常离地并停止 play。
PLAY_MAX_BASE_HEIGHT = 0.60

# PlayActionGuard 的固定安全边界。
PLAY_ACTION_ABS_LIMIT = 0.5
PLAY_ACTION_RATE_LIMIT = 0.05
PLAY_RELEASE_REPORT_CYCLES = 10

CHECKPOINT_PATH = Union[str, PathLike[str]]


def _repository_root_from_rl_agent(
    module_file: str | PathLike[str] | None = None,
) -> Path:
    """从 rl_agent.py 的绝对路径推导仓库根目录。"""
    source = (
        Path(module_file).expanduser()
        if module_file is not None
        else Path(__file__).with_name("rl_agent.py")
    ).resolve()
    # 标准布局：仓库/webots-sim/controllers/rl_agent/rl_agent.py。
    # parents[0]=rl_agent、[1]=controllers、[2]=webots-sim、[3]=仓库根。
    if (
        source.name == "rl_agent.py"
        and source.parent.name == "rl_agent"
        and source.parent.parent.name == "controllers"
        and source.parent.parent.parent.name == "webots-sim"
        and len(source.parents) > 3
    ):
        return source.parents[3]
    raise ValueError(
        f"无法从 rl_agent.py 路径推导仓库根目录: {source}"
    )


def resolve_checkpoint_path(
    checkpoint: CHECKPOINT_PATH,
    *,
    cwd: str | PathLike[str] | None = None,
    module_file: str | PathLike[str] | None = None,
) -> Path:
    """解析并校验 play checkpoint，返回存在的绝对文件路径。

    相对路径按“当前目录 → rl_agent.py 所在仓库根目录”的顺序查找；
    两个候选都不存在时，错误信息同时列出完整候选。无论走哪条分支，
    最终都会调用 ``loco_jump_contract.validate_checkpoint_path``。
    """
    if checkpoint is None:
        raise ValueError("checkpoint 路径不能为空")

    raw = Path(checkpoint).expanduser()
    # 先执行公共前缀校验，避免旧 checkpoint 通过路径解析后绕过隔离规则。
    validated_raw = validate_checkpoint_path(raw)
    if validated_raw.is_absolute():
        return validate_checkpoint_path(validated_raw.resolve())

    cwd_base = (
        Path.cwd()
        if cwd is None
        else Path(cwd).expanduser().resolve()
    )
    repo_base = _repository_root_from_rl_agent(module_file)
    current_candidate = validate_checkpoint_path(cwd_base / validated_raw)
    repo_candidate = validate_checkpoint_path(repo_base / validated_raw)

    # 两个候选同时存在时，按任务约定优先当前目录。
    selected = next(
        (
            candidate
            for candidate in (current_candidate, repo_candidate)
            if candidate.is_file()
        ),
        None,
    )
    if selected is None:
        raise FileNotFoundError(
            "checkpoint 不存在，同时列出两个候选：\n"
            f"  当前目录候选: {current_candidate.resolve()}\n"
            f"  仓库根目录候选: {repo_candidate.resolve()}"
        )
    return validate_checkpoint_path(selected.resolve())


@dataclass(frozen=True)
class StabilityResult:
    """reset 后稳定窗口的结果。"""

    released: bool
    reason: str
    cycles: int
    contact_streak: int


def _format_contacts(contacts: np.ndarray) -> str:
    """按契约顺序输出紧凑的 0/1 接触列表。"""
    return np.asarray(contacts).astype(np.int64).tolist().__repr__()


def wait_for_stable_contacts(
    agent: Any,
    teleop: Any,
    *,
    action_dim: int = 12,
    print_func: Callable[..., None] = print,
) -> StabilityResult:
    """执行 reset 后零动作稳定窗口，满足条件后才允许模型动作。

    稳定窗口内每周期都读取并消费遥控输入，但忽略跳跃/复位边沿，命令和
    动作强制清零。最短等待 50 周期；第 50 周期后若仍没有连续 10 周期
    四足接触，则继续只写零动作直到第 100 周期超时。
    """
    zero_action = np.zeros(action_dim, dtype=np.float64)
    contact_streak = 0

    for cycle in range(1, STABLE_TIMEOUT_CYCLES + 1):
        # 每周期必须消费输入，防止按键/手柄事件堆积到放行后突然触发。
        teleop.read()
        agent.current_command = np.zeros(3, dtype=np.float64)
        agent.sprint_mode = False

        # 高度检查放在零动作写入之前，避免异常离地后继续推进任何动作。
        height_before = float(agent._read_base_height())
        if height_before > PLAY_MAX_BASE_HEIGHT:
            print_func(
                "【play-safety】异常离地："
                f"height={height_before:.3f} > 0.600m，停止 play"
            )
            return StabilityResult(
                released=False,
                reason="abnormal_height",
                cycles=cycle,
                contact_streak=contact_streak,
            )

        agent.apply_action(zero_action)
        agent.step_control()

        contacts = np.asarray(agent._read_contacts(), dtype=np.float64)
        height = float(agent._read_base_height())
        if cycle <= STABLE_WINDOW_CYCLES:
            print_func(
                "【play-safety】稳定窗口 "
                f"step={cycle}/{STABLE_WINDOW_CYCLES} "
                f"contacts={_format_contacts(contacts)}"
            )
        else:
            print_func(
                "【play-safety】稳定等待延长 "
                f"step={cycle}/{STABLE_TIMEOUT_CYCLES} "
                f"contacts={_format_contacts(contacts)}"
            )

        if height > PLAY_MAX_BASE_HEIGHT:
            print_func(
                "【play-safety】异常离地："
                f"height={height:.3f} > 0.600m，停止 play"
            )
            return StabilityResult(
                released=False,
                reason="abnormal_height",
                cycles=cycle,
                contact_streak=contact_streak,
            )

        if contacts.tolist() == [1.0, 1.0, 1.0, 1.0]:
            contact_streak += 1
        else:
            contact_streak = 0

        # 50 周期是最短零动作窗口；100 周期是总超时上限。
        if (
            cycle >= STABLE_WINDOW_CYCLES
            and contact_streak >= STABLE_CONTACT_CYCLES
        ):
            return StabilityResult(
                released=True,
                reason="released",
                cycles=cycle,
                contact_streak=contact_streak,
            )

    print_func(
        "【play-safety】稳定接触超时："
        "100 个控制周期内未满足连续 10 周期四足接触，停止 play"
    )
    return StabilityResult(
        released=False,
        reason="timeout",
        cycles=STABLE_TIMEOUT_CYCLES,
        contact_streak=contact_streak,
    )


class PlayActionGuard:
    """play 放行后的动作绝对限幅与单周期变化率护栏。"""

    def __init__(
        self,
        action_dim: int = 12,
        *,
        abs_limit: float = PLAY_ACTION_ABS_LIMIT,
        rate_limit: float = PLAY_ACTION_RATE_LIMIT,
    ) -> None:
        if int(action_dim) <= 0:
            raise ValueError("动作维度必须为正整数")
        if not np.isfinite(abs_limit) or abs_limit <= 0.0:
            raise ValueError("动作绝对限幅必须为正有限值")
        if not np.isfinite(rate_limit) or rate_limit <= 0.0:
            raise ValueError("动作变化率限制必须为正有限值")
        self.action_dim = int(action_dim)
        self.abs_limit = float(abs_limit)
        self.rate_limit = float(rate_limit)
        self._previous_action = np.zeros(self.action_dim, dtype=np.float64)
        self.released_cycles = 0

    @property
    def previous_action(self) -> np.ndarray:
        """返回历史动作副本，调用方不能原地篡改内部状态。"""
        return self._previous_action.copy()

    @property
    def should_report(self) -> bool:
        """放行后的前 10 个周期需要打印诊断。"""
        return self.released_cycles <= PLAY_RELEASE_REPORT_CYCLES

    def reset(self) -> None:
        """清零动作历史和放行周期计数。"""
        self._previous_action = np.zeros(self.action_dim, dtype=np.float64)
        self.released_cycles = 0

    def filter_action(self, raw_action: Any) -> np.ndarray:
        """返回可配置绝对限幅和单周期变化率限制的安全动作。"""
        raw = np.asarray(raw_action, dtype=np.float64)
        if raw.size != self.action_dim:
            raise ValueError(
                f"动作维度错误：期望 {self.action_dim}，收到 {raw.size}"
            )
        raw = raw.reshape(self.action_dim)
        if not np.all(np.isfinite(raw)):
            raise ValueError("play 动作包含非有限数值")

        bounded = np.clip(
            raw,
            -self.abs_limit,
            self.abs_limit,
        )
        safe = np.clip(
            bounded,
            self._previous_action - self.rate_limit,
            self._previous_action + self.rate_limit,
        )
        self._previous_action = safe.copy()
        self.released_cycles += 1
        return safe
