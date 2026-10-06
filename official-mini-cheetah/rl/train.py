#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""官方 Mini Cheetah 57/12 维 CUDA PPO 训练入口。"""

from __future__ import annotations

import argparse
import math
import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Mapping, Optional, Sequence

from . import contract


GPU_NAME = "NVIDIA GeForce RTX 4060 Laptop GPU"
PPO_PARAMS: dict[str, Any] = {
    "n_steps": 2048,
    "batch_size": 256,
    "n_epochs": 10,
    "learning_rate": 3e-4,
    "gamma": 0.99,
    "gae_lambda": 0.95,
    "clip_range": 0.2,
    "ent_coef": 0.0,
}

TERMINATION_REASONS = (
    "none",
    "time_limit",
    "controller_done",
    "body_contact",
    "joint_limit",
    "tilt",
    "height",
    "workspace",
    "unsupported",
    "unknown",
)


@dataclass
class TelemetryState:
    """跨 callback 步累计的训练可观测状态。"""

    active_episode_returns: dict[int, float] = field(default_factory=dict)
    rollout_episode_returns: list[float] = field(default_factory=list)
    termination_counts: dict[str, int] = field(default_factory=dict)
    contact_samples: int = 0
    four_contact_samples: int = 0


def _termination_reason(info: Mapping[str, Any], done: bool) -> str:
    """从 VecEnv info 中提取稳定 termination reason。"""
    if not done:
        return "none"
    reason = str(info.get("termination_reason") or "")
    if reason:
        return reason
    if info.get("TimeLimit.truncated") or info.get("truncated"):
        return "time_limit"
    if info.get("terminated"):
        return "controller_done"
    return "unknown"


def update_telemetry(
    state: TelemetryState,
    infos: Sequence[Mapping[str, Any]],
    rewards: Sequence[float],
    dones: Sequence[bool],
) -> None:
    """累计 episode return、termination reason 和真实四足接触样本。"""
    for index, info in enumerate(infos):
        worker_id = int(info.get("worker_id", index))
        reward = float(rewards[index]) if index < len(rewards) else 0.0
        state.active_episode_returns[worker_id] = (
            state.active_episode_returns.get(worker_id, 0.0) + reward
        )
        source = str(info.get("foot_contact_source", ""))
        contacts = info.get("foot_contacts")
        if source == "node_id" and contacts is not None:
            try:
                # NumPy ndarray、list、tuple 等可迭代接触数据统一处理。
                values = [float(value) for value in contacts]
            except (TypeError, ValueError):
                values = []
            if len(values) == 4 and all(
                math.isfinite(value) for value in values
            ):
                state.contact_samples += 1
                if all(value >= 0.5 for value in values):
                    state.four_contact_samples += 1
        done = bool(dones[index]) if index < len(dones) else False
        if done:
            reason = _termination_reason(info, True)
            state.termination_counts[reason] = (
                state.termination_counts.get(reason, 0) + 1
            )
        if done:
            state.rollout_episode_returns.append(
                state.active_episode_returns.pop(worker_id, reward)
            )


def record_telemetry(logger: Any, state: TelemetryState) -> None:
    """把训练可观测指标写入 TensorBoard。"""
    returns = state.rollout_episode_returns
    if returns:
        logger.record("rollout/ep_rew_mean", sum(returns) / len(returns))
    elif state.active_episode_returns:
        active = list(state.active_episode_returns.values())
        logger.record(
            "rollout/ep_rew_mean",
            sum(active) / len(active),
        )
    if state.contact_samples:
        logger.record(
            "rollout/true_four_contact_ratio",
            state.four_contact_samples / state.contact_samples,
        )
    for reason in TERMINATION_REASONS:
        logger.record(
            f"termination_reason/{reason}",
            state.termination_counts.get(reason, 0),
        )
    state.rollout_episode_returns.clear()
    state.termination_counts.clear()
    state.contact_samples = 0
    state.four_contact_samples = 0


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """解析参数并完成无副作用校验。"""
    parser = argparse.ArgumentParser(description=contract.CONTRACT_VERSION)
    parser.add_argument("--phase", required=True, choices=tuple(contract.PHASE_TOTAL_STEPS))
    parser.add_argument("--total-steps", type=int, default=None)
    parser.add_argument("--resume", type=str, default=None)
    parser.add_argument("--ckptdir", type=str, default=str(contract.CHECKPOINT_ROOT))
    parser.add_argument("--ckpt-prefix", type=str, default=contract.CONTRACT_VERSION)
    parser.add_argument("--logdir", type=str, default=None)
    parser.add_argument("--bridge-port", type=int, default=contract.BRIDGE_PORT)
    parser.add_argument(
        "--randomization-mode",
        choices=contract.RANDOMIZATION_MODES,
        default="full",
    )
    parser.add_argument("--device", choices=("cuda",), default="cuda")
    parser.add_argument("--checkpoint-interval", type=int, default=contract.CHECKPOINT_INTERVAL_STEPS)
    parser.add_argument("--webots-gui", action="store_true")
    parser.add_argument(
        "--num-envs",
        type=int,
        choices=(contract.PARALLEL_WORKERS,),
        default=contract.PARALLEL_WORKERS,
        help="共享单 world 固定使用四台机器人",
    )
    parser.add_argument("--seed", type=int, default=20261003)
    parser.add_argument("--phase-step-offset", type=int, default=0)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)

    args.phase = contract.normalize_phase(args.phase)
    target = (
        contract.PHASE_DEFAULT_TARGETS[args.phase]
        if args.total_steps is None
        else int(args.total_steps)
    )
    max_target = contract.PHASE_TOTAL_STEPS[args.phase]
    if target <= 0 or target > max_target:
        parser.error(f"累计目标必须位于 (0, {max_target}]")
    if args.checkpoint_interval != contract.CHECKPOINT_INTERVAL_STEPS:
        parser.error(f"checkpoint 间隔必须为 {contract.CHECKPOINT_INTERVAL_STEPS}")
    if args.ckpt_prefix != contract.CONTRACT_VERSION:
        parser.error("checkpoint 前缀不匹配")
    if args.bridge_port != contract.BRIDGE_PORT:
        parser.error("TCP 端口必须为 11452")
    if args.phase_step_offset < 0:
        parser.error("phase-step-offset 不能为负")
    if args.resume:
        args.resume = str(contract.validate_checkpoint_path(args.resume))
    ckptdir = Path(args.ckptdir).expanduser().resolve()
    if "official-mini-cheetah" not in ckptdir.parts:
        parser.error("checkpoint 目录必须位于 official-mini-cheetah")
    if "checkpoints" not in ckptdir.parts:
        parser.error("checkpoint 目录层级错误")
    args.total_steps = target
    args.ckptdir = str(ckptdir)
    if args.logdir is None:
        args.logdir = str(contract.RUN_ROOT / args.phase)
    logdir = Path(args.logdir).expanduser().resolve()
    if "official-mini-cheetah" not in logdir.parts:
        parser.error("日志目录必须位于 official-mini-cheetah")
    args.logdir = str(logdir)
    return args


def configure_phase_step_offset(env: Any, offset: int) -> None:
    """把跨进程累计的阶段步数写入各环境，供课程与 reset 扰动使用。"""
    if offset < 0:
        raise ValueError("阶段步数偏移不能为负")
    workers = getattr(env, "envs", None)
    if workers is None:
        raise ValueError("环境缺少 worker 列表")
    for worker in workers:
        if not hasattr(worker, "_phase_steps"):
            raise ValueError("环境缺少阶段步数计数")
        worker._phase_steps = int(offset)


def validate_cuda() -> None:
    """强制唯一可见设备为 RTX 4060 的 cuda:0。"""
    if os.environ.get("CUDA_VISIBLE_DEVICES") != "0":
        raise SystemExit("CUDA_VISIBLE_DEVICES 必须精确为 0")
    import torch

    if not torch.cuda.is_available():
        raise SystemExit("PyTorch CUDA 不可用")
    if torch.cuda.device_count() != 1:
        raise SystemExit(f"期望唯一 CUDA 设备，收到 {torch.cuda.device_count()}")
    name = torch.cuda.get_device_name(0)
    if name != GPU_NAME:
        raise SystemExit(f"GPU 名称不符：{name}")
    if torch.cuda.current_device() != 0:
        raise SystemExit("当前设备不是 cuda:0")
    tensor = torch.tensor([1.0], device="cuda:0")
    if str(tensor.device) != "cuda:0":
        raise SystemExit(f"cuda:0 张量失败：{tensor.device}")
    print(f"CUDA_OK name={name} device=cuda:0 capability={torch.cuda.get_device_capability(0)}")


def make_callback(target: int, save_dir: Path, interval: int) -> list[Any]:
    """在精确累计目标停止并按步数或时间保存。"""
    from stable_baselines3.common.callbacks import BaseCallback

    class TrainingCallback(BaseCallback):
        def __init__(self) -> None:
            super().__init__(verbose=1)
            self.started = 0.0
            self.last_steps = 0
            self.last_seconds = 0.0
            self.last_heartbeat = 0
            self.telemetry = TelemetryState()

        def _init_callback(self) -> None:
            self.started = time.monotonic()
            self.last_steps = int(self.model.num_timesteps)
            self.last_seconds = 0.0
            self.last_heartbeat = int(self.model.num_timesteps)

        def _on_step(self) -> bool:
            steps = int(self.num_timesteps)
            elapsed = time.monotonic() - self.started
            infos = self.locals.get("infos", [])
            step_rewards = self.locals.get("rewards", [])
            dones = self.locals.get("dones", [])
            update_telemetry(self.telemetry, infos, step_rewards, dones)
            totals: dict[str, list[float]] = {}
            for info in infos:
                if not isinstance(info, dict):
                    continue
                reward_parts = info.get("reward_parts")
                if isinstance(reward_parts, dict):
                    for name, value in reward_parts.items():
                        totals.setdefault(str(name), []).append(float(value))
            transition_rewards = [float(value) for value in step_rewards]
            if transition_rewards:
                self.logger.record(
                    "reward/transition_mean",
                    sum(transition_rewards) / len(transition_rewards),
                )
            for name, values in totals.items():
                self.logger.record(f"reward_parts/{name}", sum(values) / len(values))
            if steps - self.last_heartbeat >= 256:
                print(f"TRAIN_PROGRESS total_timesteps={steps}", flush=True)
                self.last_heartbeat = steps
            due_steps = steps - self.last_steps >= interval
            due_time = elapsed - self.last_seconds >= contract.CHECKPOINT_INTERVAL_SECONDS
            if due_steps or due_time:
                suffix = "steps" if due_steps else "time"
                if due_steps:
                    filename = f"{contract.CONTRACT_VERSION}_{steps}_steps.zip"
                else:
                    filename = f"{contract.CONTRACT_VERSION}_{steps}_time_{int(elapsed)}s.zip"
                target_path = save_dir / filename
                contract.validate_checkpoint_path(target_path)
                self.model.save(str(target_path))
                print(f"CHECKPOINT {target_path}", flush=True)
                self.last_steps = steps
                self.last_seconds = elapsed
            if steps >= target:
                print(f"TRAIN_TARGET_REACHED total_timesteps={steps}", flush=True)
                return False
            return True

        def _on_rollout_end(self) -> None:
            record_telemetry(self.logger, self.telemetry)

    return [TrainingCallback()]


def load_or_create(args: argparse.Namespace, env: Any) -> Any:
    """创建新模型或严格加载同前缀 checkpoint。"""
    from stable_baselines3 import PPO

    overrides = dict(PPO_PARAMS)
    if args.resume:
        path = contract.validate_checkpoint_path(args.resume)
        if not path.is_file():
            raise SystemExit(f"checkpoint 不存在：{path}")
        model = PPO.load(
            str(path),
            env=env,
            device="cuda",
            tensorboard_log=args.logdir,
            **overrides,
        )
        if tuple(model.observation_space.shape) != (contract.OBS_DIM,):
            raise SystemExit("checkpoint 观测维度不是 57")
        if tuple(model.action_space.shape) != (contract.ACTION_DIM,):
            raise SystemExit("checkpoint 动作维度不是 12")
        print(f"MODEL_LOADED path={path} steps={model.num_timesteps}")
        return model
    return PPO(
        "MlpPolicy",
        env,
        policy_kwargs={"net_arch": [256, 256]},
        tensorboard_log=args.logdir,
        device="cuda",
        seed=args.seed,
        verbose=1,
        **overrides,
    )


def train(args: argparse.Namespace) -> Path:
    """执行单阶段训练并返回最终 checkpoint。"""
    validate_cuda()
    from .shared_world_vec_env import SharedWorldVecEnv

    Path(args.logdir).mkdir(parents=True, exist_ok=True)
    save_dir = Path(args.ckptdir)
    save_dir.mkdir(parents=True, exist_ok=True)
    env = SharedWorldVecEnv(
        num_envs=args.num_envs,
        phase=args.phase,
        bridge_port=args.bridge_port,
        randomization_mode=args.randomization_mode,
        render_mode="human" if args.webots_gui else None,
        seed=args.seed,
    )
    try:
        configure_phase_step_offset(env, args.phase_step_offset)
        model = load_or_create(args, env)
        current = int(model.num_timesteps)
        if current > args.total_steps:
            raise SystemExit(f"checkpoint {current} 超过目标 {args.total_steps}")
        remaining = args.total_steps - current
        print(
            f"TRAIN_START phase={args.phase} current={current} "
            f"target={args.total_steps} remaining={remaining} "
            f"num_envs={args.num_envs} ports={list(contract.bridge_ports(args.num_envs))}"
        )
        if remaining:
            model.learn(
                total_timesteps=remaining,
                callback=make_callback(
                    args.total_steps, save_dir, args.checkpoint_interval
                ),
                reset_num_timesteps=not args.resume,
                progress_bar=False,
            )
        final = save_dir / f"{contract.CONTRACT_VERSION}_final.zip"
        contract.validate_checkpoint_path(final)
        model.save(str(final))
        if int(model.num_timesteps) != args.total_steps:
            raise SystemExit(
                f"最终步数错误：{model.num_timesteps} != {args.total_steps}"
            )
        print(f"TRAIN_COMPLETED final={final} steps={model.num_timesteps}")
        return final
    finally:
        env.close()


def print_config(args: argparse.Namespace) -> None:
    """打印无副作用 dry-run 配置。"""
    print("=" * 72)
    print(f"CONTRACT {contract.CONTRACT_VERSION}")
    print(f"PHASE {args.phase} TARGET {args.total_steps}")
    print(f"DIMS obs={contract.OBS_DIM} action={contract.ACTION_DIM} rate={contract.CONTROL_RATE_HZ}Hz")
    print("DEVICE cuda CUDA_VISIBLE_DEVICES=0")
    print(f"GPU {GPU_NAME}")
    print(f"CHECKPOINT_DIR {args.ckptdir}")
    print(f"TENSORBOARD_DIR {args.logdir}")
    print(
        f"PARALLEL_ENVS {args.num_envs} "
        f"PORTS {args.bridge_port}-{args.bridge_port + args.num_envs - 1}"
    )
    print(f"WEBOTS_GUI {args.webots_gui}")
    print(f"PHASE_STEP_OFFSET {args.phase_step_offset}")
    print(f"RESUME {args.resume or 'NONE'}")
    print("STATUS dry-run")
    print("=" * 72)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """入口；dry-run 不创建环境或文件。"""
    args = parse_args(argv)
    if args.dry_run:
        print_config(args)
        return 0
    train(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
