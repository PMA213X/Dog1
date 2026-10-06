#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""平直地面移动与跳跃任务的 CUDA PPO 训练入口。

本入口只允许 ``--device cuda`` 和 ``CUDA_VISIBLE_DEVICES=0``，不提供 CPU
降级路径。``--dry-run`` 不导入训练依赖、不创建环境、不写文件。
"""

from __future__ import annotations

import argparse
import os
import time
from pathlib import Path
from typing import Any, Callable, Optional, Sequence

import flat_jump_contract as contract


PPO_POLICY = "MlpPolicy"
NET_ARCH = [256, 256]
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
F1_PPO_PARAMS: dict[str, Any] = {
    **PPO_PARAMS,
    "learning_rate": 1e-4,
    "clip_range": 0.1,
    "n_epochs": 5,
}
DEFAULT_SEED = 20261003
DEFAULT_BRIDGE_PORT = 11451


def phase_ppo_params(phase: str) -> dict[str, Any]:
    """返回阶段专用 PPO 参数，F1 不向 F2/F3 泄漏。"""
    normalized = contract.normalize_phase(phase)
    if normalized == "F1":
        return dict(F1_PPO_PARAMS)
    return dict(PPO_PARAMS)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """解析训练参数并完成所有无副作用校验。"""
    parser = argparse.ArgumentParser(
        description="yobogo_flat_jump_v1 平地 PPO 训练",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--phase",
        required=True,
        choices=tuple(contract.PHASE_TOTAL_STEPS),
        help="F0～F3 阶段",
    )
    parser.add_argument(
        "--total-steps",
        type=int,
        default=None,
        help="累计目标步数；省略时使用阶段累计目标",
    )
    parser.add_argument(
        "--resume",
        type=str,
        default=None,
        help="从前一阶段同前缀 checkpoint 热启动",
    )
    parser.add_argument(
        "--ckptdir",
        type=str,
        default="checkpoints/yobogo_flat_jump_v1",
    )
    parser.add_argument(
        "--ckpt-prefix",
        type=str,
        default=contract.CHECKPOINT_PREFIX,
    )
    parser.add_argument(
        "--logdir",
        type=str,
        default=None,
        help="TensorBoard 日志目录",
    )
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument(
        "--bridge-port",
        type=int,
        default=DEFAULT_BRIDGE_PORT,
    )
    parser.add_argument(
        "--randomization-mode",
        type=str,
        default="full",
        choices=("fixed", "curriculum", "full"),
        help="平地域随机化模式；最终 F1 验收固定使用 full",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda",
        choices=("cuda",),
        help="固定使用 CUDA，不提供 CPU 降级",
    )
    parser.add_argument(
        "--checkpoint-interval",
        type=int,
        default=contract.CHECKPOINT_INTERVAL_STEPS,
    )
    parser.add_argument(
        "--webots-gui",
        action="store_true",
        help="以本地 GUI 快速模式启动唯一的 Webots 训练窗口",
    )
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)

    phase = contract.normalize_phase(args.phase)
    target = (
        contract.PHASE_TOTAL_STEPS[phase]
        if args.total_steps is None
        else int(args.total_steps)
    )
    if target <= 0 or target > contract.PHASE_TOTAL_STEPS[phase]:
        parser.error(
            f"--total-steps 必须位于 (0, {contract.PHASE_TOTAL_STEPS[phase]}]"
        )
    if args.checkpoint_interval != contract.CHECKPOINT_INTERVAL_STEPS:
        parser.error(
            "--checkpoint-interval 必须为 "
            f"{contract.CHECKPOINT_INTERVAL_STEPS}"
        )
    if args.ckpt_prefix != contract.CHECKPOINT_PREFIX:
        parser.error(
            f"--ckpt-prefix 必须为 {contract.CHECKPOINT_PREFIX}"
        )
    if not 1024 <= args.bridge_port <= 65535:
        parser.error("--bridge-port 必须位于 1024～65535")
    if args.resume:
        args.resume = str(contract.validate_checkpoint_path(args.resume))
    args.phase = phase
    args.total_steps = target
    if args.logdir is None:
        args.logdir = str(
            Path("runs/yobogo_flat_jump_v1") / f"{phase}_{contract.PHASE_NAMES[phase]}"
        )
    return args


def print_config(args: argparse.Namespace) -> None:
    """打印 dry-run 配置。"""
    print("=" * 72)
    print("【配置】yobogo_flat_jump_v1")
    print(f"  阶段              : {args.phase} {contract.PHASE_NAMES[args.phase]}")
    print(f"  累计目标步数      : {args.total_steps}")
    print(f"  观测/动作维度     : {contract.OBS_DIM} / {contract.ACTION_DIM}")
    print("  控制频率          : 50 Hz")
    print("  计算设备          : cuda（CUDA_VISIBLE_DEVICES=0，禁止 CPU）")
    print(f"  checkpoint 前缀   : {args.ckpt_prefix}")
    print(f"  checkpoint 目录   : {args.ckptdir}")
    print(f"  TensorBoard 目录  : {args.logdir}")
    print(f"  TCP 桥端口        : {args.bridge_port}")
    print(f"  地域随机化        : {args.randomization_mode}")
    print(
        "  Webots 窗口       : "
        + ("开启（fast，本地 DISPLAY）" if args.webots_gui else "关闭")
    )
    params = phase_ppo_params(args.phase)
    print(
        "  PPO               : "
        f"lr={params['learning_rate']} clip={params['clip_range']} "
        f"epochs={params['n_epochs']}"
    )
    print(f"  续训 checkpoint   : {args.resume or '（否）'}")
    print("  训练状态          : 未启动（dry-run）")
    print("=" * 72)


def require_training_dependencies() -> None:
    """训练前检查依赖，缺失时直接停止。"""
    try:
        import gymnasium  # noqa: F401
        import stable_baselines3  # noqa: F401
        import torch  # noqa: F401
    except ImportError as exc:
        raise SystemExit(f"【错误】缺少训练依赖：{exc}") from exc


def validate_cuda() -> None:
    """强制校验唯一可见 GPU 为 CUDA 设备 0。"""
    visible = os.environ.get("CUDA_VISIBLE_DEVICES", "")
    if visible != "0":
        raise SystemExit(
            "【错误】CUDA_VISIBLE_DEVICES 必须精确设置为 0，"
            f"当前值={visible!r}"
        )
    import torch

    if not torch.cuda.is_available():
        raise SystemExit("【错误】PyTorch CUDA 不可用")
    if torch.cuda.device_count() != 1:
        raise SystemExit(
            f"【错误】期望恰好 1 个可见 CUDA 设备，收到 {torch.cuda.device_count()}"
        )
    if torch.cuda.current_device() != 0:
        raise SystemExit("【错误】当前 CUDA 设备不是 0")
    print(
        f"【CUDA】device=0 name={torch.cuda.get_device_name(0)} "
        f"capability={torch.cuda.get_device_capability(0)}",
        flush=True,
    )


def make_env_fn(
    phase: str,
    bridge_port: int,
    randomization_mode: str,
    webots_gui: bool = False,
) -> Callable[[], Any]:
    """创建带 Monitor 的单个同步平地环境。"""
    def factory() -> Any:
        from stable_baselines3.common.monitor import Monitor
        from flat_jump_env import FlatGroundJumpEnv

        return Monitor(
            FlatGroundJumpEnv(
                phase=phase,
                bridge_port=bridge_port,
                randomization_mode=randomization_mode,
                render_mode="human" if webots_gui else None,
            )
        )
    return factory


def load_or_create_model(
    args: argparse.Namespace,
    train_env: Any,
) -> Any:
    """创建新 PPO 或加载严格同前缀 checkpoint。"""
    from stable_baselines3 import PPO

    params = phase_ppo_params(args.phase)
    if args.resume:
        resume_path = contract.validate_checkpoint_path(args.resume)
        if not resume_path.is_file():
            raise SystemExit(f"【错误】找不到续训 checkpoint：{resume_path}")
        model = PPO.load(
            str(resume_path),
            env=train_env,
            device="cuda",
            tensorboard_log=args.logdir,
            learning_rate=params["learning_rate"],
            n_steps=params["n_steps"],
            batch_size=params["batch_size"],
            n_epochs=params["n_epochs"],
            gamma=params["gamma"],
            gae_lambda=params["gae_lambda"],
            clip_range=params["clip_range"],
            ent_coef=params["ent_coef"],
        )
        obs_shape = tuple(
            getattr(getattr(train_env, "observation_space", None), "shape", ())
        )
        old_shape = tuple(
            getattr(getattr(model, "observation_space", None), "shape", ())
        )
        if old_shape and obs_shape and old_shape != obs_shape:
            raise SystemExit(
                f"【错误】观测维度不兼容：checkpoint={old_shape} env={obs_shape}"
            )
        print(
            f"【训练】已加载 {resume_path}，累计步数={model.num_timesteps}",
            flush=True,
        )
        return model

    model = PPO(
        policy=PPO_POLICY,
        env=train_env,
        learning_rate=params["learning_rate"],
        n_steps=params["n_steps"],
        batch_size=params["batch_size"],
        n_epochs=params["n_epochs"],
        gamma=params["gamma"],
        gae_lambda=params["gae_lambda"],
        clip_range=params["clip_range"],
        ent_coef=params["ent_coef"],
        policy_kwargs=dict(net_arch=NET_ARCH),
        tensorboard_log=args.logdir,
        device="cuda",
        verbose=1,
        seed=args.seed,
    )
    print(
        f"【训练】新模型 obs={contract.OBS_DIM} "
        f"action={contract.ACTION_DIM} device=cuda "
        f"lr={params['learning_rate']} clip={params['clip_range']} "
        f"epochs={params['n_epochs']}",
        flush=True,
    )
    return model


def make_callbacks(
    target: int,
    save_path: Path,
    prefix: str,
    interval: int,
) -> list[Any]:
    """创建精确停止在累计目标的 checkpoint 回调。"""
    from stable_baselines3.common.callbacks import BaseCallback

    class FlatTrainingCallback(BaseCallback):
        """按目标步数停止，并按步数或墙钟时间保存 checkpoint。"""

        def __init__(self) -> None:
            super().__init__(verbose=1)
            self._started_at = time.monotonic()
            self._last_steps = 0
            self._last_seconds = 0.0
            self._last_heartbeat_steps = 0

        def _init_callback(self) -> None:
            self._started_at = time.monotonic()
            self._last_steps = int(self.model.num_timesteps)
            self._last_seconds = 0.0
            self._last_heartbeat_steps = int(self.model.num_timesteps)

        def _on_training_start(self) -> None:
            save_path.mkdir(parents=True, exist_ok=True)
            self._started_at = time.monotonic()
            self._last_steps = int(self.model.num_timesteps)
            self._last_seconds = 0.0
            self._last_heartbeat_steps = int(self.model.num_timesteps)

        def _on_step(self) -> bool:
            steps = int(self.num_timesteps)
            elapsed = time.monotonic() - self._started_at
            if steps - self._last_heartbeat_steps >= 256:
                print(f"【训练进度】total_timesteps={steps}", flush=True)
                self._last_heartbeat_steps = steps
            due_steps = steps - self._last_steps >= interval
            due_seconds = elapsed - self._last_seconds >= contract.CHECKPOINT_INTERVAL_SECONDS
            if due_steps or due_seconds:
                suffix = "steps" if due_steps else "time"
                if suffix == "steps":
                    filename = f"{prefix}_{steps}_steps.zip"
                else:
                    filename = f"{prefix}_{steps}_time_{int(elapsed)}s.zip"
                target_path = save_path / filename
                contract.validate_checkpoint_path(target_path)
                self.model.save(str(target_path))
                print(f"【checkpoint】{target_path}", flush=True)
                self._last_steps = steps
                self._last_seconds = elapsed
            if steps >= target:
                print(
                    f"【训练】达到累计目标 {target}，停止本阶段",
                    flush=True,
                )
                return False
            return True

    return [FlatTrainingCallback()]


def train(args: argparse.Namespace) -> Path:
    """执行单阶段训练。"""
    require_training_dependencies()
    validate_cuda()
    from stable_baselines3.common.vec_env import DummyVecEnv

    logdir = Path(args.logdir)
    logdir.mkdir(parents=True, exist_ok=True)
    os.environ["RL_WEBOTS_LOG"] = str(logdir / "webots.log")
    ckptdir = Path(args.ckptdir)
    ckptdir.mkdir(parents=True, exist_ok=True)
    train_env = DummyVecEnv(
        [
            make_env_fn(
                args.phase,
                args.bridge_port,
                args.randomization_mode,
                args.webots_gui,
            )
        ]
    )
    model: Any = None
    try:
        model = load_or_create_model(args, train_env)
        current = int(model.num_timesteps)
        if current > args.total_steps:
            raise SystemExit(
                f"【错误】checkpoint 累计步数 {current} 超过目标 {args.total_steps}"
            )
        remaining = args.total_steps - current
        print(
            f"【训练】阶段={args.phase} 累计={current} "
            f"目标={args.total_steps} 本次={remaining}",
            flush=True,
        )
        if remaining == 0:
            final_path = ckptdir / f"{args.ckpt_prefix}_final.zip"
            model.save(str(final_path))
        else:
            callbacks = make_callbacks(
                args.total_steps,
                ckptdir,
                args.ckpt_prefix,
                args.checkpoint_interval,
            )
            started = time.monotonic()
            model.learn(
                total_timesteps=remaining,
                callback=callbacks,
                reset_num_timesteps=not args.resume,
                progress_bar=False,
            )
            elapsed = time.monotonic() - started
            final_path = ckptdir / f"{args.ckpt_prefix}_final.zip"
            model.save(str(final_path))
            print(
                f"【训练】耗时={elapsed:.1f}s；最终模型已保存：{final_path}",
                flush=True,
            )
        if int(model.num_timesteps) != args.total_steps:
            raise SystemExit(
                "【错误】最终步数与累计目标不一致："
                f"{model.num_timesteps} != {args.total_steps}"
            )
        print("【训练】TRAINING_COMPLETED", flush=True)
        return final_path
    finally:
        train_env.close()


def main(argv: Optional[Sequence[str]] = None) -> int:
    """命令行入口。"""
    args = parse_args(argv)
    if args.dry_run:
        print_config(args)
        return 0
    train(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
