#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Rapid 网络的 SB3 PPO 微调策略与分阶段冻结控制。"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence

import numpy as np
import torch as th
from gymnasium import spaces
from stable_baselines3.common.policies import ActorCriticPolicy
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from stable_baselines3 import PPO

from . import contract
from .rapid_policy import (
    RAPID_MODEL_DIR,
    SOURCE_ACTION_DIM,
    SOURCE_HISTORY_DIM,
    SOURCE_LATENT_DIM,
    SOURCE_OBS_DIM,
    verify_asset_manifest,
)


RAPID_TOTAL_STEPS = 500_000
RAPID_STAGE_CRITIC_ONLY = 20_000
RAPID_STAGE_ACTOR_START = 100_000
RAPID_STAGE_ADAPTATION_START = 300_000
RAPID_ACTOR_LR = 1e-5
RAPID_CRITIC_LR = 1e-4
RAPID_ADAPTATION_LR = 1e-6
RAPID_ENT_COEF = 0.003
RAPID_PPO_PARAMS: dict[str, Any] = {
    "n_steps": 512,
    "batch_size": 256,
    "n_epochs": 5,
    "learning_rate": RAPID_ACTOR_LR,
    "gamma": 0.99,
    "gae_lambda": 0.95,
    "clip_range": 0.2,
    # final 训练末期 approx_kl 约 0.008，冻结 smoke 的接触/饱和明显更稳；
    # 对单次更新加 KL 上限，避免为追速度而一次性改写冻结动作分布。
    "target_kl": 0.01,
    "ent_coef": RAPID_ENT_COEF,
}


def rapid_source_std(model_dir: Path | str = RAPID_MODEL_DIR) -> np.ndarray:
    """读取源 checkpoint 的 12 维探索标准差。"""
    verify_asset_manifest(model_dir)
    weights = th.load(
        str(Path(model_dir) / "ac_weights_last.pt"),
        map_location="cpu",
        weights_only=True,
    )
    std = np.asarray(weights["std"], dtype=np.float32)
    if std.shape != (SOURCE_ACTION_DIM,) or not np.all(np.isfinite(std)):
        raise RuntimeError(f"Rapid 源 std 无效：{std}")
    if np.any(std <= 0.0):
        raise RuntimeError("Rapid 源 std 必须为正")
    return std


def _linear_stack(
    dimensions: Sequence[int],
    *,
    final_activation: bool,
) -> th.nn.Sequential:
    """按源 TorchScript 的 Linear/ELU 顺序构造网络。"""
    modules: list[th.nn.Module] = []
    for index, (left, right) in enumerate(
        zip(dimensions[:-1], dimensions[1:])
    ):
        modules.append(th.nn.Linear(left, right))
        if index < len(dimensions) - 2 or final_activation:
            modules.append(th.nn.ELU())
    return th.nn.Sequential(*modules)


def _load_linear(
    module: th.nn.Module,
    weights: Mapping[str, th.Tensor],
    prefix: str,
) -> None:
    """把源 checkpoint 的 Linear 键严格载入本地模块。"""
    with th.no_grad():
        for index, child in enumerate(module):
            if not isinstance(child, th.nn.Linear):
                continue
            weight_key = f"{prefix}.{index}.weight"
            bias_key = f"{prefix}.{index}.bias"
            if weight_key not in weights or bias_key not in weights:
                raise RuntimeError(f"Rapid 权重缺少 {weight_key}")
            child.weight.copy_(weights[weight_key])
            child.bias.copy_(weights[bias_key])


class RapidFeaturesExtractor(BaseFeaturesExtractor):
    """630 维历史 -> 18 维 latent，并与 42 维当前帧拼成 60 维。"""

    def __init__(self, observation_space: spaces.Space, **kwargs: Any) -> None:
        super().__init__(observation_space, features_dim=SOURCE_OBS_DIM + SOURCE_LATENT_DIM, **kwargs)
        self.adaptation = _linear_stack(
            (SOURCE_HISTORY_DIM, 256, 32, SOURCE_LATENT_DIM),
            final_activation=False,
        )

    def forward(self, observation: Mapping[str, th.Tensor]) -> th.Tensor:
        current = observation["current"]
        history = observation["history"]
        if current.shape[-1] != SOURCE_OBS_DIM:
            raise ValueError(f"current 维度错误：{tuple(current.shape)}")
        if history.shape[-1] != SOURCE_HISTORY_DIM:
            raise ValueError(f"history 维度错误：{tuple(history.shape)}")
        latent = self.adaptation(history)
        if latent.shape[-1] != SOURCE_LATENT_DIM:
            raise ValueError(f"latent 维度错误：{tuple(latent.shape)}")
        return th.cat((current, latent), dim=-1)


class RapidActorCriticPolicy(ActorCriticPolicy):
    """保留 Rapid adaptation/actor/critic 结构的自定义 SB3 Policy。"""

    def __init__(
        self,
        observation_space: spaces.Space,
        action_space: spaces.Space,
        lr_schedule: Any,
        *,
        rapid_model_dir: str = str(RAPID_MODEL_DIR),
        **kwargs: Any,
    ) -> None:
        if not isinstance(observation_space, spaces.Dict):
            raise ValueError("Rapid 微调策略要求 Dict 观测空间")
        if set(observation_space.spaces) != {"current", "history"}:
            raise ValueError("Rapid Dict 观测键必须为 current/history")
        if observation_space["current"].shape != (SOURCE_OBS_DIM,):
            raise ValueError("current 必须为 42 维")
        if observation_space["history"].shape != (SOURCE_HISTORY_DIM,):
            raise ValueError("history 必须为 630 维")
        if action_space.shape != (SOURCE_ACTION_DIM,):
            raise ValueError("Rapid 策略动作必须为 12 维")
        self.rapid_model_dir = str(Path(rapid_model_dir).resolve())
        super().__init__(
            observation_space,
            action_space,
            lr_schedule,
            features_extractor_class=RapidFeaturesExtractor,
            features_extractor_kwargs={},
            activation_fn=th.nn.ELU,
            ortho_init=False,
            net_arch=[],
            optimizer_kwargs={"eps": 1e-5},
            **kwargs,
        )
        self.apply_finetune_stage(0, adaptation_enabled=False)

    def _build(self, lr_schedule: Any) -> None:
        del lr_schedule
        # 特征提取器中的 adaptation 在这里严格载入；actor/critic 也使用
        # 源网络的 ELU 结构，避免 SB3 默认 Tanh 改变冻结输出。
        weights = th.load(
            str(Path(self.rapid_model_dir) / "ac_weights_last.pt"),
            map_location="cpu",
            weights_only=True,
        )
        _load_linear(
            self.features_extractor.adaptation,
            weights,
            "adaptation_module",
        )
        self.actor_body = _linear_stack(
            (SOURCE_OBS_DIM + SOURCE_LATENT_DIM, 512, 256, 128, SOURCE_ACTION_DIM),
            final_activation=False,
        )
        self.critic_body = _linear_stack(
            (SOURCE_OBS_DIM + SOURCE_LATENT_DIM, 512, 256, 128, 1),
            final_activation=False,
        )
        _load_linear(self.actor_body, weights, "actor_body")
        _load_linear(self.critic_body, weights, "critic_body")
        self.action_net = th.nn.Identity()
        self.value_net = th.nn.Identity()
        self.log_std = th.nn.Parameter(
            th.log(
                th.as_tensor(
                    np.asarray(weights["std"], dtype=np.float32),
                    dtype=th.float32,
                )
            )
        )
        self.finetune_stage = 0
        self.adaptation_enabled = False
        self.optimizer = th.optim.Adam(
            [
                {
                    "params": list(self.actor_body.parameters())
                    + [self.log_std],
                    "lr": RAPID_ACTOR_LR,
                    "name": "actor",
                },
                {
                    "params": list(self.critic_body.parameters()),
                    "lr": RAPID_CRITIC_LR,
                    "name": "critic",
                },
                {
                    "params": list(
                        self.features_extractor.adaptation.parameters()
                    ),
                    "lr": RAPID_ADAPTATION_LR,
                    "name": "adaptation",
                },
            ],
            eps=1e-5,
        )

    def extract_features(self, obs: Any) -> th.Tensor:
        return self.features_extractor(obs)

    def _latent(self, obs: Any) -> tuple[th.Tensor, th.Tensor]:
        features = self.extract_features(obs)
        return self.actor_body(features), self.critic_body(features)

    def get_distribution(self, obs: Any) -> Any:
        """绕过 SB3 默认 mlp_extractor，直接使用 Rapid actor。"""
        features = self.extract_features(obs)
        return self._get_action_dist_from_latent(self.actor_body(features))

    def predict_values(self, obs: Any) -> th.Tensor:
        """绕过 SB3 默认 mlp_extractor，直接使用 Rapid critic。"""
        features = self.extract_features(obs)
        return self.critic_body(features)

    def forward(
        self,
        obs: Any,
        deterministic: bool = False,
    ) -> tuple[th.Tensor, th.Tensor, th.Tensor]:
        actor_latent, critic_latent = self._latent(obs)
        distribution = self._get_action_dist_from_latent(actor_latent)
        actions = distribution.get_actions(deterministic=deterministic)
        log_prob = distribution.log_prob(actions)
        values = critic_latent
        actions = actions.reshape((-1, *self.action_space.shape))
        return actions, values, log_prob

    def evaluate_actions(
        self,
        obs: Any,
        actions: th.Tensor,
    ) -> tuple[th.Tensor, th.Tensor, th.Tensor | None]:
        actor_latent, critic_latent = self._latent(obs)
        distribution = self._get_action_dist_from_latent(actor_latent)
        log_prob = distribution.log_prob(actions)
        entropy = distribution.entropy()
        return critic_latent, log_prob, entropy

    def apply_finetune_stage(
        self,
        stage: int,
        *,
        adaptation_enabled: bool | None = None,
    ) -> str:
        """切换 critic-only、actor+critic 和可选 adaptation 解冻阶段。"""
        stage = int(stage)
        if stage not in (0, 1, 2):
            raise ValueError(f"未知 Rapid 微调阶段：{stage}")
        if adaptation_enabled is None:
            adaptation_enabled = stage >= 2
        self.finetune_stage = stage
        self.adaptation_enabled = bool(adaptation_enabled)
        critic_only = stage == 0
        self.actor_body.requires_grad_(not critic_only)
        self.log_std.requires_grad_(not critic_only)
        self.critic_body.requires_grad_(True)
        self.features_extractor.adaptation.requires_grad_(
            self.adaptation_enabled
        )
        return (
            f"stage={stage} critic_only={critic_only} "
            f"adaptation_enabled={self.adaptation_enabled}"
        )

    def deterministic_source_action(self, obs: Any) -> th.Tensor:
        """返回冻结初始化使用的确定性源动作，便于与 play 对照。"""
        with th.no_grad():
            features = self.extract_features(obs)
            return self.actor_body(features)


class RapidPPO(PPO):
    """为 actor/critic/adaptation 参数组保持独立学习率的 PPO。"""

    def _update_learning_rate(self, optimizer: th.optim.Optimizer) -> None:
        for group in optimizer.param_groups:
            name = str(group.get("name", ""))
            if name == "actor":
                group["lr"] = RAPID_ACTOR_LR
            elif name == "critic":
                group["lr"] = RAPID_CRITIC_LR
            elif name == "adaptation":
                group["lr"] = (
                    RAPID_ADAPTATION_LR
                    if getattr(self.policy, "adaptation_enabled", False)
                    else 0.0
                )
        if hasattr(self, "logger"):
            for group in optimizer.param_groups:
                name = str(group.get("name", ""))
                if name:
                    self.logger.record(
                        f"rapid/{name}_lr",
                        float(group["lr"]),
                    )


__all__ = [
    "RAPID_TOTAL_STEPS",
    "RAPID_STAGE_CRITIC_ONLY",
    "RAPID_STAGE_ACTOR_START",
    "RAPID_STAGE_ADAPTATION_START",
    "RAPID_ACTOR_LR",
    "RAPID_CRITIC_LR",
    "RAPID_ADAPTATION_LR",
    "RAPID_ENT_COEF",
    "RAPID_PPO_PARAMS",
    "RapidActorCriticPolicy",
    "RapidFeaturesExtractor",
    "RapidPPO",
    "rapid_source_std",
]
