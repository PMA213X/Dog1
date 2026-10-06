#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""yobogo_loco_jump_v1 的 PPO 训练入口。

安全边界：
    * ``--dry-run`` 只解析并校验参数，不导入训练依赖、不创建环境、不落盘；
    * ``--resume`` 必须是 ``yobogo_loco_jump_v1`` 新前缀 checkpoint，旧前缀拒绝；
    * checkpoint 固定按 50000 步或 1800 秒任一条件保存；
    * 本任务只执行 dry-run/单元测试，不启动正式训练。
"""

from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence

_RL_DIR = Path(__file__).resolve().parent
if str(_RL_DIR) not in sys.path:
    sys.path.insert(0, str(_RL_DIR))

try:
    from . import loco_jump_contract as contract
    from . import loco_jump_reward as reward_module
except ImportError:  # 直接执行脚本或把 rl 目录加入 sys.path
    import loco_jump_contract as contract  # type: ignore[no-redef]
    import loco_jump_reward as reward_module  # type: ignore[no-redef]

# 第三方训练依赖只在真正训练时导入，确保 --dry-run 在最小环境也能运行。
try:
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.monitor import Monitor
    from stable_baselines3.common.vec_env import DummyVecEnv
except ImportError as _sb3_exc:  # pragma: no cover - dry-run 不需要依赖
    PPO = None  # type: ignore[assignment]
    BaseCallback = object  # type: ignore[assignment,misc]
    Monitor = None  # type: ignore[assignment]
    DummyVecEnv = None  # type: ignore[assignment]
    SB3_IMPORT_ERROR: Optional[ImportError] = _sb3_exc
else:
    SB3_IMPORT_ERROR = None

try:
    from loco_jump_env import QuadrupedLocoJumpEnv
except ImportError as _env_exc:  # pragma: no cover - dry-run 不创建环境
    QuadrupedLocoJumpEnv = None  # type: ignore[assignment]
    ENV_IMPORT_ERROR: Optional[ImportError] = _env_exc
else:
    ENV_IMPORT_ERROR = None

TAGS: tuple[str, ...] = ("phase0", "phase1", "phase2", "phase3", "phase4")
DEVICES: tuple[str, ...] = ("cpu", "cuda")


# ---------------------------------------------------------------------------
# 纯参数与预算校验
# ---------------------------------------------------------------------------
def validate_checkpoint_prefix(prefix: str) -> str:
    """校验 checkpoint 前缀属于新契约命名空间。"""
    candidate = str(prefix).strip()
    if not candidate:
        raise ValueError("checkpoint 前缀不能为空")
    # 用合成文件名复用契约的完整前缀边界规则，拒绝 phase1/ppo_walk 等旧名。
    if not contract.is_isolated_checkpoint(f"{candidate}_probe.zip"):
        raise ValueError(
            f"checkpoint 前缀不兼容：{candidate!r}；"
            f"必须以 {contract.CHECKPOINT_PREFIX!r} 开始"
        )
    return candidate


def validate_resume_path(path: str | Path) -> Path:
    """校验续训 checkpoint；旧前缀一律拒绝，返回规范化路径。"""
    return contract.validate_checkpoint_path(path)


def stage_target_steps(tag: str) -> int:
    """返回阶段对应的累计目标总步数。"""
    stage = reward_module.normalize_stage(tag)
    return int(contract.PHASE_TOTAL_STEPS[stage])


def validate_total_steps(tag: str, total_steps: int) -> int:
    """校验阶段累计预算，保留 S0/S1 管线预算与正式训练预算边界。"""
    stage = reward_module.normalize_stage(tag)
    target = int(contract.PHASE_TOTAL_STEPS[stage])
    value = int(total_steps)
    if value <= 0:
        raise ValueError(f"--total-steps 必须大于 0，收到 {value}")
    if value > target:
        raise ValueError(
            f"--tag {tag} 的累计目标只能到 {target} 步，收到 {value} 步"
        )
    if stage in {"S3_jump", "S4_mobile_terrain"}:
        low, high = contract.FORMAL_TRAINING_STEP_RANGE
        if not low <= value <= high:
            raise ValueError(
                f"正式预算必须位于 [{low}, {high}]，收到 {value} 步"
            )
    return value


def validate_checkpoint_interval(interval: int) -> int:
    """checkpoint 步数间隔锁定为契约的 50000。"""
    value = int(interval)
    if value != contract.CHECKPOINT_INTERVAL_STEPS:
        raise ValueError(
            f"--checkpoint-interval 必须为 "
            f"{contract.CHECKPOINT_INTERVAL_STEPS}，收到 {value}"
        )
    return value


def checkpoint_steps_for_tag(tag: str) -> List[int]:
    """列出阶段内按固定步数应保存的 checkpoint 位置。"""
    target = validate_total_steps(tag, stage_target_steps(tag))
    interval = contract.CHECKPOINT_INTERVAL_STEPS
    return list(range(interval, target + 1, interval))


def checkpoint_should_save(
    current_steps: int,
    elapsed_seconds: float,
    *,
    last_steps: int = 0,
    last_seconds: float = 0.0,
    interval_steps: int = contract.CHECKPOINT_INTERVAL_STEPS,
    interval_seconds: float = contract.CHECKPOINT_INTERVAL_SECONDS,
) -> bool:
    """判断是否达到“50000 步或 1800 秒”任一 checkpoint 条件。"""
    step_due = int(current_steps) - int(last_steps) >= int(interval_steps)
    time_due = float(elapsed_seconds) - float(last_seconds) >= float(interval_seconds)
    return bool(step_due or time_due)


def make_logdir(explicit: Optional[str], tag: str) -> Path:
    """确定 TensorBoard 日志目录；显式路径优先。"""
    if explicit:
        result = Path(explicit)
    else:
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        result = Path(
            f"runs/{contract.CHECKPOINT_PREFIX}_{tag}_{stamp}"
        )
    result.mkdir(parents=True, exist_ok=True)
    return result


# ---------------------------------------------------------------------------
# 命令行
# ---------------------------------------------------------------------------
def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="yobogo_loco_jump_v1 PPO 训练（55 维观测 / 12 维动作）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--total-steps",
        type=int,
        default=None,
        help="累计目标总步数；省略时使用 --tag 对应的阶段累计目标",
    )
    parser.add_argument(
        "--tag",
        type=str,
        default="phase0",
        choices=TAGS,
        help="阶段标签：phase0..phase4",
    )
    parser.add_argument(
        "--ckpt-prefix",
        type=str,
        default=contract.CHECKPOINT_PREFIX,
        help="checkpoint 文件名前缀，必须属于新契约命名空间",
    )
    parser.add_argument(
        "--ckptdir",
        type=str,
        default=f"checkpoints/{contract.CHECKPOINT_PREFIX}",
        help="checkpoint 保存目录",
    )
    parser.add_argument(
        "--logdir",
        type=str,
        default=None,
        help="TensorBoard 日志目录",
    )
    parser.add_argument(
        "--resume",
        type=str,
        default=None,
        metavar="ZIP",
        help="从 yobogo_loco_jump_v1 新前缀 checkpoint 续训",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda",
        choices=DEVICES,
        help="计算设备：cpu 或 cuda",
    )
    parser.add_argument(
        "--checkpoint-interval",
        type=int,
        default=contract.CHECKPOINT_INTERVAL_STEPS,
        help="checkpoint 步数间隔（锁定 50000）",
    )
    parser.add_argument(
        "--eval-interval",
        type=int,
        default=20_000,
        help="评估间隔（步），0 关闭",
    )
    parser.add_argument(
        "--eval-episodes",
        type=int,
        default=10,
        help="每次评估 episode 数",
    )
    parser.add_argument("--seed", type=int, default=None, help="随机种子")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只解析与校验配置，不创建环境、不训练、不写 checkpoint",
    )
    args = parser.parse_args(argv)

    try:
        args.ckpt_prefix = validate_checkpoint_prefix(args.ckpt_prefix)
        if args.resume:
            args.resume = str(validate_resume_path(args.resume))
        if args.total_steps is None:
            args.total_steps = stage_target_steps(args.tag)
        args.total_steps = validate_total_steps(args.tag, args.total_steps)
        args.checkpoint_interval = validate_checkpoint_interval(
            args.checkpoint_interval
        )
        if args.eval_interval < 0:
            raise ValueError("--eval-interval 不能小于 0")
        if args.eval_episodes <= 0:
            raise ValueError("--eval-episodes 必须大于 0")
    except ValueError as exc:
        parser.error(str(exc))
    return args


def print_config(args: argparse.Namespace) -> None:
    """打印 dry-run 配置；不创建任何训练产物。"""
    stage = reward_module.normalize_stage(args.tag)
    target = int(contract.PHASE_TOTAL_STEPS[stage])
    print("=" * 68)
    print(f"【配置】{contract.CHECKPOINT_PREFIX}")
    print("=" * 68)
    print(f"  阶段              : {args.tag} -> {stage}")
    print(f"  累计目标步数      : {args.total_steps} / {target}")
    print(f"  观测/动作维度     : {contract.OBS_DIM} / {contract.ACTION_DIM}")
    print(f"  控制频率          : {contract.CONTROL_RATE_HZ} Hz")
    print(f"  计算设备          : {args.device}")
    print(f"  checkpoint 前缀   : {args.ckpt_prefix}")
    print(
        "  checkpoint 条件   : 每 "
        f"{contract.CHECKPOINT_INTERVAL_STEPS} 步或 "
        f"{contract.CHECKPOINT_INTERVAL_SECONDS:.0f} 秒"
    )
    print(f"  step 保存点       : {checkpoint_steps_for_tag(args.tag)}")
    print(f"  续训 checkpoint   : {args.resume or '（否）'}")
    print("  训练状态          : 未启动（dry-run）")
    print("=" * 68)


# ---------------------------------------------------------------------------
# checkpoint 回调
# ---------------------------------------------------------------------------
if BaseCallback is not object:

    class ContractCheckpointCallback(BaseCallback):  # type: ignore[valid-type]
        """按步数或墙钟时间保存隔离前缀 checkpoint。"""

        def __init__(
            self,
            save_path: str,
            name_prefix: str,
            *,
            interval_steps: int = contract.CHECKPOINT_INTERVAL_STEPS,
            interval_seconds: float = contract.CHECKPOINT_INTERVAL_SECONDS,
            verbose: int = 0,
        ) -> None:
            super().__init__(verbose)
            self.save_path = Path(save_path)
            self.name_prefix = name_prefix
            validate_checkpoint_prefix(self.name_prefix)
            self.interval_steps = validate_checkpoint_interval(interval_steps)
            if float(interval_seconds) != contract.CHECKPOINT_INTERVAL_SECONDS:
                raise ValueError("checkpoint 时间间隔必须为 1800 秒")
            self.interval_seconds = float(interval_seconds)
            # SB3 在 init_callback() 中才注入 model；构造阶段不得读取。
            self._started_at: Optional[float] = None
            self._last_steps: Optional[int] = None
            self._last_seconds = 0.0
            self.save_count = 0

        def _init_callback(self) -> None:
            """model 注入完成后初始化步数基准。"""
            if self.model is None:  # pragma: no cover - init_callback 保证非空
                raise RuntimeError("checkpoint 回调未收到已注入的 model")
            self._started_at = time.monotonic()
            self._last_steps = int(self.model.num_timesteps)
            self._last_seconds = 0.0

        def _on_training_start(self) -> None:
            self.save_path.mkdir(parents=True, exist_ok=True)
            if self.model is None:  # pragma: no cover - SB3 生命周期保证
                raise RuntimeError("checkpoint 回调未收到已注入的 model")
            self._started_at = time.monotonic()
            self._last_steps = int(self.model.num_timesteps)
            self._last_seconds = 0.0

        def _on_step(self) -> bool:
            if self._started_at is None or self._last_steps is None:
                raise RuntimeError("checkpoint 回调尚未完成 SB3 生命周期初始化")
            elapsed = time.monotonic() - self._started_at
            steps = int(self.num_timesteps)
            if not checkpoint_should_save(
                steps,
                elapsed,
                last_steps=self._last_steps,
                last_seconds=self._last_seconds,
                interval_steps=self.interval_steps,
                interval_seconds=self.interval_seconds,
            ):
                return True
            suffix = (
                "steps"
                if steps - self._last_steps >= self.interval_steps
                else "time"
            )
            filename = (
                f"{self.name_prefix}_{steps}_{suffix}"
                f"_{int(elapsed)}s.zip"
                if suffix == "time"
                else f"{self.name_prefix}_{steps}_steps.zip"
            )
            target = self.save_path / filename
            contract.validate_checkpoint_path(target)
            self.model.save(str(target))
            self.save_count += 1
            self._last_steps = steps
            self._last_seconds = elapsed
            if self.verbose:
                print(f"【checkpoint】{target}")
            return True

else:  # pragma: no cover - SB3 缺失时占位

    class ContractCheckpointCallback:  # type: ignore[no-redef]
        """SB3 缺失时的占位实现。"""

        def __init__(self, *args: Any, **kwargs: Any) -> None:
            raise RuntimeError("stable-baselines3 未安装，checkpoint 回调不可用")


# ---------------------------------------------------------------------------
# 评估回调
# ---------------------------------------------------------------------------
if BaseCallback is not object:

    class EvaluateCallback(BaseCallback):  # type: ignore[valid-type]
        """在训练环境之外按固定间隔做确定性评估。"""

        def __init__(
            self,
            eval_env_fn: Callable[[], Any],
            eval_freq: int,
            eval_episodes: int,
            verbose: int = 0,
        ) -> None:
            super().__init__(verbose)
            self.eval_env_fn = eval_env_fn
            self.eval_freq = max(1, int(eval_freq))
            self.eval_episodes = max(1, int(eval_episodes))
            self._env: Any = None
            self._last_eval = -1

        def _on_step(self) -> bool:
            if self.num_timesteps - self._last_eval < self.eval_freq:
                return True
            self._last_eval = self.num_timesteps
            try:
                env = self._env or self.eval_env_fn()
                self._env = env
                rewards: List[float] = []
                lengths: List[int] = []
                for _ in range(self.eval_episodes):
                    reset = env.reset()
                    obs = reset[0] if isinstance(reset, tuple) else reset
                    done = False
                    total = 0.0
                    length = 0
                    while not done:
                        action, _ = self.model.predict(obs, deterministic=True)
                        result = env.step(action)
                        obs, reward, terminated, truncated, info = result
                        done = bool(terminated or truncated)
                        total += float(reward)
                        length += 1
                    rewards.append(total)
                    lengths.append(length)
                print(
                    f"【评估】步数={self.num_timesteps} "
                    f"平均奖励={sum(rewards) / len(rewards):.3f} "
                    f"平均步数={sum(lengths) / len(lengths):.1f}"
                )
            except Exception as exc:  # noqa: BLE001 - Webots 评估失败不应杀训练
                print(f"【评估】失败并关闭后续评估：{exc}")
                self.eval_freq = 10**12
            return True

        def _on_training_end(self) -> None:
            if self._env is not None:
                close = getattr(self._env, "close", None)
                if callable(close):
                    close()
                self._env = None

else:  # pragma: no cover

    class EvaluateCallback:  # type: ignore[no-redef]
        """SB3 缺失时的占位实现。"""

        def __init__(self, *args: Any, **kwargs: Any) -> None:
            raise RuntimeError("stable-baselines3 未安装，评估回调不可用")


# ---------------------------------------------------------------------------
# 依赖、环境与模型
# ---------------------------------------------------------------------------
def require_training_dependencies() -> None:
    """真正训练前检查依赖，给出中文错误。"""
    if SB3_IMPORT_ERROR is not None:
        raise SystemExit(
            "【错误】缺少 stable-baselines3，无法开始训练。\n"
            f"  原始报错：{SB3_IMPORT_ERROR}"
        )
    if ENV_IMPORT_ERROR is not None or QuadrupedLocoJumpEnv is None:
        raise SystemExit(
            "【错误】无法导入 loco_jump_env.QuadrupedLocoJumpEnv。\n"
            f"  原始报错：{ENV_IMPORT_ERROR}"
        )


def validate_cuda_available() -> None:
    """真正使用 cuda 前检查 PyTorch CUDA。"""
    import torch

    if not torch.cuda.is_available():
        raise SystemExit("【错误】--device cuda，但当前 PyTorch 无法使用 CUDA")


def make_env_fn(tag: str) -> Callable[[], Any]:
    """创建带 Monitor 包装的单环境工厂。"""

    def factory() -> Any:
        assert QuadrupedLocoJumpEnv is not None
        assert Monitor is not None
        return Monitor(QuadrupedLocoJumpEnv(tag=tag))

    return factory


def load_or_create_model(
    args: argparse.Namespace,
    train_env: Any,
    logdir: Path,
) -> Any:
    """创建新 PPO 或加载新前缀续训模型；拒绝维度不匹配的热启动。"""
    assert PPO is not None
    if args.resume:
        resume_path = Path(args.resume)
        if not resume_path.is_file():
            raise SystemExit(f"【错误】找不到续训 checkpoint：{resume_path}")
        model = PPO.load(
            str(resume_path),
            env=train_env,
            device=args.device,
            tensorboard_log=str(logdir),
        )
        obs_shape = tuple(
            getattr(getattr(train_env, "observation_space", None), "shape", ())
        )
        old_shape = tuple(
            getattr(getattr(model, "observation_space", None), "shape", ())
        )
        if old_shape and obs_shape and old_shape != obs_shape:
            raise SystemExit(
                f"【错误】续训 checkpoint 观测维度不兼容："
                f"checkpoint={old_shape} env={obs_shape}"
            )
        print(
            f"【训练】已加载新前缀 checkpoint：{resume_path}，"
            f"累计步数={model.num_timesteps}"
        )
        return model

    from config import NET_ARCH, PPO_PARAMS, PPO_POLICY

    model = PPO(
        policy=PPO_POLICY,
        env=train_env,
        learning_rate=PPO_PARAMS["learning_rate"],
        n_steps=PPO_PARAMS["n_steps"],
        batch_size=PPO_PARAMS["batch_size"],
        n_epochs=PPO_PARAMS["n_epochs"],
        gamma=PPO_PARAMS["gamma"],
        gae_lambda=PPO_PARAMS["gae_lambda"],
        clip_range=PPO_PARAMS["clip_range"],
        ent_coef=PPO_PARAMS["ent_coef"],
        policy_kwargs=dict(net_arch=NET_ARCH),
        tensorboard_log=str(logdir),
        device=args.device,
        verbose=1,
        seed=args.seed,
    )
    print(
        f"【训练】已创建新模型：obs={contract.OBS_DIM} "
        f"action={contract.ACTION_DIM} device={args.device}"
    )
    return model


def train(args: argparse.Namespace) -> Path:
    """执行训练；本任务不会调用本函数。"""
    require_training_dependencies()
    if args.device == "cuda":
        validate_cuda_available()
    stage = reward_module.normalize_stage(args.tag)
    # --total-steps 是累计目标；parse_args 已按阶段上限/正式预算校验。
    target = int(args.total_steps)
    if target > int(contract.PHASE_TOTAL_STEPS[stage]):
        raise SystemExit(
            f"【错误】累计目标超过阶段 {args.tag} 的上限 "
            f"{contract.PHASE_TOTAL_STEPS[stage]} 步"
        )
    logdir = make_logdir(args.logdir, args.tag)
    ckptdir = Path(args.ckptdir)
    ckptdir.mkdir(parents=True, exist_ok=True)

    assert DummyVecEnv is not None
    train_env = DummyVecEnv([make_env_fn(args.tag)])
    model = load_or_create_model(args, train_env, logdir)

    callbacks: List[Any] = [
        ContractCheckpointCallback(
            save_path=str(ckptdir),
            name_prefix=args.ckpt_prefix,
            interval_steps=args.checkpoint_interval,
            interval_seconds=contract.CHECKPOINT_INTERVAL_SECONDS,
            verbose=1,
        )
    ]
    if args.eval_interval > 0:
        callbacks.append(
            EvaluateCallback(
                eval_env_fn=make_env_fn(args.tag),
                eval_freq=args.eval_interval,
                eval_episodes=args.eval_episodes,
            )
        )

    if args.resume:
        current = int(model.num_timesteps)
        remaining = max(0, target - current)
    else:
        current = 0
        remaining = target
    if remaining == 0:
        final_path = ckptdir / f"{args.ckpt_prefix}_final.zip"
        model.save(str(final_path))
        print(f"【训练】最终模型已保存：{final_path}")
        return final_path

    print(
        f"【训练】阶段={args.tag} 累计步数={current} "
        f"目标={target} 本次={remaining}"
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
        f"【训练】耗时={elapsed:.1f}s；最终模型已保存：{final_path}"
    )
    train_env.close()
    return final_path


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    if args.dry_run:
        print_config(args)
        return 0
    train(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
