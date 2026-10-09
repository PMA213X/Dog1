"""Rapid Locomotion adaptation/body TorchScript 冻结模型加载与前向。"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import torch

from .adapter import ACTION_DIM, BODY_INPUT_DIM, HISTORY_DIM, LATENT_DIM, OBS_DIM


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_WEIGHTS_DIR = (
    PROJECT_ROOT
    / "external_models"
    / "rapid-locomotion-rl"
    / "runs"
    / "rapid-locomotion"
    / "example"
    / "train"
    / "201852.132488"
    / "checkpoints"
)


def resolve_weight_paths(weights_dir: Path) -> tuple[Path, Path]:
    """优先解析官方 `*_latest.jit`，缺失时给出明确错误而不回退旧 loader。"""
    weights_dir = weights_dir.expanduser().resolve()
    adaptation_path = weights_dir / "adaptation_module_latest.jit"
    body_path = weights_dir / "body_latest.jit"
    missing = [path for path in (adaptation_path, body_path) if not path.is_file()]
    if missing:
        missing_text = "\n".join(str(path) for path in missing)
        raise FileNotFoundError(f"缺少 Rapid TorchScript 权重：\n{missing_text}")
    return adaptation_path, body_path


class RapidLocomotionModel:
    """直接调用 adaptation 与 actor body，只取 actor mean，不做动作采样。"""

    def __init__(
        self,
        weights_dir: Path = DEFAULT_WEIGHTS_DIR,
        device: torch.device | str = "cpu",
    ) -> None:
        self.device = torch.device(device)
        self.adaptation_path, self.body_path = resolve_weight_paths(weights_dir)
        self.adaptation = torch.jit.load(str(self.adaptation_path), map_location=self.device)
        self.body = torch.jit.load(str(self.body_path), map_location=self.device)
        self.adaptation.eval()
        self.body.eval()

    @torch.inference_mode()
    def dummy_forward(self, num_envs: int = 1) -> tuple[torch.Tensor, torch.Tensor]:
        """加载后立即验证 630→18、60→12 的 dummy 前向契约。"""
        if num_envs < 1:
            raise ValueError("num_envs 必须大于 0")
        current = torch.zeros(num_envs, OBS_DIM, dtype=torch.float32, device=self.device)
        history = torch.zeros(num_envs, HISTORY_DIM, dtype=torch.float32, device=self.device)
        return self(current, history)

    @torch.inference_mode()
    def __call__(
        self,
        current_observation: torch.Tensor,
        history: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """执行 630→18 adaptation，拼接 42 维观测后执行 60→12 body。"""
        if current_observation.shape[-1] != OBS_DIM:
            raise ValueError(
                f"current_observation 最后一维必须为 {OBS_DIM}，"
                f"实际为 {tuple(current_observation.shape)}"
            )
        if history.shape[-1] != HISTORY_DIM:
            raise ValueError(
                f"history 最后一维必须为 {HISTORY_DIM}，"
                f"实际为 {tuple(history.shape)}"
            )
        if current_observation.shape[0] != history.shape[0]:
            raise ValueError("current_observation 与 history 批次维度不一致")
        if current_observation.dtype != torch.float32 or history.dtype != torch.float32:
            raise TypeError("Rapid 冻结模型输入必须为 float32")
        if not torch.isfinite(current_observation).all():
            raise ValueError("current_observation 含 NaN/Inf")
        if not torch.isfinite(history).all():
            raise ValueError("history 含 NaN/Inf")

        latent = self.adaptation(history)
        if latent.shape != (current_observation.shape[0], LATENT_DIM):
            raise RuntimeError(f"latent shape 错误：{tuple(latent.shape)}")
        body_input = torch.cat((current_observation, latent), dim=-1)
        if body_input.shape[-1] != BODY_INPUT_DIM:
            raise RuntimeError(f"body input shape 错误：{tuple(body_input.shape)}")
        raw_action = self.body(body_input)
        if raw_action.shape != (current_observation.shape[0], ACTION_DIM):
            raise RuntimeError(f"actor mean shape 错误：{tuple(raw_action.shape)}")
        if not torch.isfinite(latent).all() or not torch.isfinite(raw_action).all():
            raise ValueError("Rapid 冻结模型输出含 NaN/Inf")
        return raw_action, latent
