#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Rapid Dict 观测、源动作映射和逐环境历史隔离的 VecEnv 包装。"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence

import numpy as np
from gymnasium import spaces
from stable_baselines3.common.vec_env import VecEnv

from . import contract
from .rapid_policy import (
    JOINT_SIGNS,
    RAPID_MODEL_DIR,
    SOURCE_DEFAULT_ANGLES,
    SOURCE_ACTION_DIM,
    SOURCE_HISTORY_DIM,
    SOURCE_HISTORY_LENGTH,
    SOURCE_OBS_DIM,
    SOURCE_TO_CURRENT_INDICES,
    RapidActionMapper,
    RapidPolicyAdapter,
)
from .reward import source_action_saturation_penalty


# Gate 每个 stop/forward case 连续执行 1000 步；旧 250 步重采样只训练
# 5 s 片段，无法覆盖 final Gate 中随时间累积的 2.87 m 漂移。命令保持
# 一个完整 episode，再在同步 reset 后重新抽样。
COMMAND_RESAMPLE_STEPS = contract.MAX_EPISODE_STEPS
EARLY_FORWARD_MIN = 0.10
EARLY_FORWARD_MAX = 0.18
# 本阶段全程只覆盖 Gate 的 low-speed forward 和 zero；不再在 100k 后
# 扩展到 lateral/yaw 或越界前进，actor 解冻与命令分布互不耦合。
FORWARD_MIN = EARLY_FORWARD_MIN
FORWARD_MAX = EARLY_FORWARD_MAX


class RapidFinetuneVecEnv(VecEnv):
    """底层 57 维/当前动作环境到 630+42 维/源动作策略的统一适配层。"""

    def __init__(
        self,
        base_env: VecEnv,
        *,
        model_dir: Path | str = RAPID_MODEL_DIR,
        command_course: bool = True,
        finetune_mode: bool = True,
        seed: int | None = None,
    ) -> None:
        if base_env.num_envs != contract.PARALLEL_WORKERS:
            raise ValueError("Rapid 微调必须使用四个并行环境")
        if base_env.observation_space.shape != (contract.OBS_DIM,):
            raise ValueError("底层观测必须为 57 维")
        if base_env.action_space.shape != (contract.ACTION_DIM,):
            raise ValueError("底层动作必须为 12 维")
        observation_space = spaces.Dict(
            {
                "current": spaces.Box(
                    low=-np.inf,
                    high=np.inf,
                    shape=(SOURCE_OBS_DIM,),
                    dtype=np.float32,
                ),
                "history": spaces.Box(
                    low=-np.inf,
                    high=np.inf,
                    shape=(SOURCE_HISTORY_DIM,),
                    dtype=np.float32,
                ),
            }
        )
        action_space = spaces.Box(
            low=-100.0,
            high=100.0,
            shape=(SOURCE_ACTION_DIM,),
            dtype=np.float32,
        )
        super().__init__(
            base_env.num_envs,
            observation_space,
            action_space,
        )
        self.base_env = base_env
        self.model_dir = Path(model_dir).resolve()
        self.mapper = RapidActionMapper(self.model_dir)
        # 使用对象新建方式复用 play 的观测构造，不重复加载 TorchScript。
        self.adapter = object.__new__(RapidPolicyAdapter)
        self.adapter.model_dir = self.model_dir
        self.adapter.device = None
        self.adapter._source_defaults = np.asarray(
            SOURCE_DEFAULT_ANGLES,
            dtype=np.float32,
        )
        self.adapter._source_to_current = np.asarray(
            SOURCE_TO_CURRENT_INDICES,
            dtype=np.int64,
        )
        self.adapter._joint_signs = np.asarray(
            JOINT_SIGNS,
            dtype=np.float32,
        )
        self.adapter.action_mapper = self.mapper
        self._histories = [
            [] for _ in range(self.num_envs)
        ]
        self._previous_source_actions = np.zeros(
            (self.num_envs, SOURCE_ACTION_DIM),
            dtype=np.float32,
        )
        self._pending_source_actions: np.ndarray | None = None
        self._command_course = bool(command_course)
        self._finetune_mode = bool(finetune_mode)
        self._command_rng = np.random.default_rng(seed)
        self._command_age = np.zeros(self.num_envs, dtype=np.int64)
        self._last_source_actions = np.zeros(
            (self.num_envs, SOURCE_ACTION_DIM),
            dtype=np.float32,
        )
        self._set_finetune_mode()

    @property
    def envs(self) -> list[Any]:
        """共享底层 worker 列表，供阶段步数配置和诊断复用。"""
        return getattr(self.base_env, "envs", [])

    def _set_finetune_mode(self) -> None:
        workers = getattr(self.base_env, "envs", None)
        if workers is None:
            return
        for worker in workers:
            if self._finetune_mode:
                if hasattr(worker, "set_finetune_mode"):
                    worker.set_finetune_mode(True)
                elif hasattr(worker, "finetune_mode"):
                    worker.finetune_mode = True
                else:
                    raise RuntimeError(
                        "底层环境缺少 finetune_mode 接口，拒绝启动 Rapid 微调"
                    )
            else:
                if hasattr(worker, "set_finetune_mode"):
                    worker.set_finetune_mode(False)
                elif hasattr(worker, "finetune_mode"):
                    worker.finetune_mode = False

    def _sample_command(self, worker_id: int) -> np.ndarray:
        del worker_id
        if self._command_rng.random() < 0.75:
            return np.asarray(
                [
                    self._command_rng.uniform(
                        EARLY_FORWARD_MIN,
                        EARLY_FORWARD_MAX,
                    ),
                    0.0,
                    0.0,
                ],
                dtype=np.float32,
            )
        return np.zeros(3, dtype=np.float32)

    def _apply_command_course(self, worker_ids: Sequence[int] | None = None) -> None:
        if not self._command_course:
            return
        workers = getattr(self.base_env, "envs", None)
        if workers is None:
            raise RuntimeError("底层 VecEnv 缺少 worker 列表")
        selected = range(self.num_envs) if worker_ids is None else worker_ids
        for worker_id in selected:
            command = self._sample_command(int(worker_id))
            worker = workers[int(worker_id)]
            worker._command = command
            worker._command_fixed = True
            worker._command_step = CONTRACT_COMMAND_STEP
            self._command_age[int(worker_id)] = 0

    def _refresh_observation(
        self,
        observations: np.ndarray,
        worker_ids: Sequence[int],
    ) -> None:
        workers = getattr(self.base_env, "envs", None)
        if workers is None:
            return
        for worker_id in worker_ids:
            worker = workers[int(worker_id)]
            if hasattr(worker, "_observation"):
                observations[int(worker_id)] = worker._observation()

    def _encode_one(
        self,
        index: int,
        observation_57: Sequence[float],
        source_action: Sequence[float],
        *,
        mutate_history: bool = True,
    ) -> Dict[str, np.ndarray]:
        """编码单环境；terminal 可只读历史，不推进该 episode 状态。"""
        current = self.adapter.build_source_observation(
            observation_57,
            previous_source_action=source_action,
        )
        history, _warmup = self.adapter.build_history(
            current,
            previous_frames=self._histories[index],
        )
        if mutate_history:
            self._histories[index].append(current)
            if len(self._histories[index]) > SOURCE_HISTORY_LENGTH:
                self._histories[index] = self._histories[index][
                    -SOURCE_HISTORY_LENGTH:
                ]
        return {
            "current": current.astype(np.float32, copy=False),
            "history": history.astype(np.float32, copy=False),
        }

    def _encode(
        self,
        raw_observations: np.ndarray,
        *,
        source_actions: np.ndarray,
        append: bool = True,
    ) -> Dict[str, np.ndarray]:
        encoded = [
            self._encode_one(
                index,
                raw_observations[index],
                source_actions[index],
                mutate_history=append,
            )
            for index in range(self.num_envs)
        ]
        return {
            "current": np.stack(
                [item["current"] for item in encoded]
            ).astype(np.float32),
            "history": np.stack(
                [item["history"] for item in encoded]
            ).astype(np.float32),
        }

    def reset(self) -> Dict[str, np.ndarray]:
        raw = self.base_env.reset()
        self._histories = [[] for _ in range(self.num_envs)]
        self._previous_source_actions.fill(0.0)
        self._last_source_actions.fill(0.0)
        self._command_age.fill(0)
        self._apply_command_course()
        self._refresh_observation(raw, range(self.num_envs))
        return self._encode(
            raw,
            source_actions=self._previous_source_actions,
            append=True,
        )

    def step_async(self, actions: np.ndarray) -> None:
        if self._pending_source_actions is not None:
            raise RuntimeError("已有待完成的 Rapid step")
        values = np.asarray(actions, dtype=np.float32)
        if values.shape != (self.num_envs, SOURCE_ACTION_DIM):
            raise ValueError(
                f"源动作形状必须为 {(self.num_envs, SOURCE_ACTION_DIM)}，收到 {values.shape}"
            )
        if not np.all(np.isfinite(values)):
            raise ValueError("源动作必须全部有限")
        self._pending_source_actions = values.copy()
        mapped = self.mapper.map_batch_to_current(values)
        self.base_env.step_async(mapped)

    def step_wait(self) -> tuple[
        Dict[str, np.ndarray],
        np.ndarray,
        np.ndarray,
        list[Dict[str, Any]],
    ]:
        if self._pending_source_actions is None:
            raise RuntimeError("step_async 尚未调用")
        source_actions = self._pending_source_actions
        self._pending_source_actions = None
        raw, rewards, dones, infos = self.base_env.step_wait()
        rewards = np.asarray(rewards, dtype=np.float32).copy()
        done_ids = [index for index, done in enumerate(dones) if bool(done)]
        for index in range(self.num_envs):
            penalty = float(
                source_action_saturation_penalty(source_actions[index])
            )
            if not math.isfinite(penalty):
                raise RuntimeError("Rapid 动作饱和惩罚非有限")
            infos[index].setdefault(
                "rapid_source_action",
                source_actions[index].copy(),
            )
            infos[index].setdefault(
                "rapid_mapped_action",
                self.mapper.map_source_to_current(source_actions[index]),
            )
            parts = infos[index].get("reward_parts")
            if not isinstance(parts, dict):
                parts = {}
                infos[index]["reward_parts"] = parts
            canonical_penalty = float(
                parts.get("source_action_saturation", 0.0)
            )
            already_applied = (
                abs(canonical_penalty) > 1e-12
                and canonical_penalty * penalty >= 0.0
            )
            if not already_applied:
                rewards[index] += penalty
                parts["source_action_saturation"] = penalty
            parts["rapid_source_action_saturation"] = (
                canonical_penalty if already_applied else penalty
            )

        if not bool(np.any(dones)):
            encoded = self._encode(
                raw,
                source_actions=source_actions,
                append=True,
            )
        else:
            if not bool(np.all(dones)):
                raise RuntimeError("共享 Rapid VecEnv 要求同步终止")
            encoded = {
                "current": np.empty(
                    (self.num_envs, SOURCE_OBS_DIM),
                    dtype=np.float32,
                ),
                "history": np.empty(
                    (self.num_envs, SOURCE_HISTORY_DIM),
                    dtype=np.float32,
                ),
            }
            for index in done_ids:
                terminal_raw = infos[index].get(
                    "terminal_observation",
                    raw[index],
                )
                terminal = self._encode_one(
                    index,
                    terminal_raw,
                    source_actions[index],
                    mutate_history=False,
                )
                encoded["current"][index] = terminal["current"]
                encoded["history"][index] = terminal["history"]
                infos[index]["terminal_observation"] = {
                    "current": terminal["current"].copy(),
                    "history": terminal["history"].copy(),
                }
                self._histories[index].clear()
                self._previous_source_actions[index].fill(0.0)
                self._last_source_actions[index].fill(0.0)
            for index in range(self.num_envs):
                if index not in done_ids:
                    self._previous_source_actions[index] = source_actions[index]
                    self._last_source_actions[index] = source_actions[index]

        # 共享 world 在任一 worker 结束时会同步 reset；刷新命令课程后再编码。
        if done_ids:
            self._command_age.fill(0)
            self._apply_command_course()
            self._refresh_observation(raw, range(self.num_envs))
            encoded = self._encode(
                raw,
                source_actions=self._previous_source_actions,
                append=True,
            )
        else:
            self._command_age += 1
            due = np.flatnonzero(
                self._command_age >= COMMAND_RESAMPLE_STEPS
            )
            if due.size:
                self._apply_command_course(due.tolist())
                self._refresh_observation(raw, due.tolist())
                for index in due.tolist():
                    current = self.adapter.build_source_observation(
                        raw[index],
                        previous_source_action=self._previous_source_actions[index],
                    )
                    self._histories[index][-1] = current
                    encoded["current"][index] = current
        return encoded, rewards, np.asarray(dones, dtype=bool), infos

    def env_method(
        self,
        method_name: str,
        *args: Any,
        indices: Any = None,
        **kwargs: Any,
    ) -> list[Any]:
        return self.base_env.env_method(method_name, *args, indices=indices, **kwargs)

    def get_attr(self, attr_name: str, indices: Any = None) -> list[Any]:
        return self.base_env.get_attr(attr_name, indices=indices)

    def set_attr(self, attr_name: str, value: Any, indices: Any = None) -> None:
        self.base_env.set_attr(attr_name, value, indices=indices)

    def get_images(self) -> Sequence[np.ndarray]:
        return self.base_env.get_images()

    def env_is_wrapped(
        self,
        wrapper_class: Any,
        indices: Any = None,
    ) -> list[bool]:
        return self.base_env.env_is_wrapped(wrapper_class, indices=indices)

    def close(self) -> None:
        self.base_env.close()


# 命令重采样与 contract 常量隔离，避免并行修改期间导入阶段耦合。
CONTRACT_COMMAND_STEP = COMMAND_RESAMPLE_STEPS


__all__ = [
    "COMMAND_RESAMPLE_STEPS",
    "EARLY_FORWARD_MAX",
    "EARLY_FORWARD_MIN",
    "FORWARD_MAX",
    "FORWARD_MIN",
    "RapidFinetuneVecEnv",
]
