#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PPO 训练入口：用 Stable-Baselines3 训练四足机器狗在 Webots 中行走。

用法示例：
    python train_ppo.py --dry-run                  # 只解析参数，不启动训练
    python train_ppo.py --total-steps 200000       # 正式训练
    python train_ppo.py --resume checkpoints/ppo_walk_100000.zip
    python train_ppo.py --device cpu --total-steps 2048

环境接口由 webots-sim/rl/walk_env.py 提供（另一模块编写）：
    from walk_env import QuadrupedWalkEnv
    obs 42 维 float32，action 12 维 [-1, 1]
"""

from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Optional, Tuple

# 保证同目录下的 config.py / walk_env.py 可以被 import
_RL_DIR = Path(__file__).resolve().parent
if str(_RL_DIR) not in sys.path:
    sys.path.insert(0, str(_RL_DIR))

# ---------------------------------------------------------------------------
# 本地配置
# ---------------------------------------------------------------------------
from config import (  # noqa: E402
    ACTION_DIM,
    ACTION_HIGH,
    ACTION_LOW,
    CKPT_NAME_PREFIX,
    DEFAULT_CKPT_DIR,
    DEFAULT_DEVICE,
    DEFAULT_LOGDIR_PREFIX,
    DEFAULT_TOTAL_STEPS,
    CHECKPOINT_INTERVAL,
    EVAL_EPISODES,
    EVAL_INTERVAL,
    NET_ARCH,
    OBS_DIM,
    OBS_DIM_TURN,
    OBS_DIM_STAIRS,
    PPO_PARAMS,
    PPO_POLICY,
    set_phase,
)

# ---------------------------------------------------------------------------
# 环境接口 import（walk_env.py 若尚未就绪，--dry-run 仍可跑通参数解析）
# ---------------------------------------------------------------------------
try:
    from walk_env import QuadrupedWalkEnv  # noqa: F401
except ImportError as _walk_env_exc:  # pragma: no cover - 取决于另一模块就绪状态
    QuadrupedWalkEnv = None  # type: ignore[assignment,misc]
    WALK_ENV_IMPORT_ERROR: Optional[ImportError] = _walk_env_exc
else:
    WALK_ENV_IMPORT_ERROR = None

# P3 转向环境（可选）
try:
    from walk_env_turn import QuadrupedTurnEnv  # noqa: F401
except ImportError:
    QuadrupedTurnEnv = None  # type: ignore[assignment,misc]

# P4 台阶环境（可选）
try:
    from walk_env_stairs import QuadrupedStairsEnv  # noqa: F401
except ImportError:
    QuadrupedStairsEnv = None  # type: ignore[assignment,misc]

# ---------------------------------------------------------------------------
# 第三方依赖：仅在真正训练时强制要求，便于 --dry-run 无依赖自检
# ---------------------------------------------------------------------------
try:
    import gymnasium as gym
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
    from stable_baselines3.common.monitor import Monitor
    from stable_baselines3.common.vec_env import DummyVecEnv
except ImportError as _sb3_exc:  # pragma: no cover - 无 SB3 时只影响训练路径
    gym = None  # type: ignore[assignment]
    torch = None  # type: ignore[assignment]
    PPO = None  # type: ignore[assignment,misc]
    BaseCallback = object  # type: ignore[assignment,misc]
    CheckpointCallback = None  # type: ignore[assignment,misc]
    Monitor = None  # type: ignore[assignment,misc]
    DummyVecEnv = None  # type: ignore[assignment,misc]
    SB3_IMPORT_ERROR: Optional[ImportError] = _sb3_exc
else:
    SB3_IMPORT_ERROR = None


# ===========================================================================
# 工具函数
# ===========================================================================
def parse_args(argv: Optional[list] = None) -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(
        description="PPO 训练四足机器狗行走（Stable-Baselines3）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--total-steps",
        type=int,
        default=DEFAULT_TOTAL_STEPS,
        help="训练总环境步数",
    )
    parser.add_argument(
        "--logdir",
        type=str,
        default=None,
        help="TensorBoard 日志目录，默认 runs/ppo_walk_<时间戳>",
    )
    parser.add_argument(
        "--ckptdir",
        type=str,
        default=DEFAULT_CKPT_DIR,
        help="checkpoint 保存目录",
    )
    parser.add_argument(
        "--resume",
        type=str,
        default=None,
        metavar="ZIP",
        help="从指定的 .zip 模型继续训练",
    )
    parser.add_argument(
        "--device",
        type=str,
        default=DEFAULT_DEVICE,
        help="计算设备，如 cuda / cpu / cuda:0",
    )
    parser.add_argument(
        "--eval-interval",
        type=int,
        default=EVAL_INTERVAL,
        help="评估间隔（步），0 表示关闭定期评估",
    )
    parser.add_argument(
        "--eval-episodes",
        type=int,
        default=EVAL_EPISODES,
        help="每次评估的 episode 数",
    )
    parser.add_argument(
        "--checkpoint-interval",
        type=int,
        default=CHECKPOINT_INTERVAL,
        help="checkpoint 保存间隔（步）",
    )
    parser.add_argument(
        "--tag",
        type=str,
        default=None,
        help="阶段标签（如 phase1_stand / phase2_walk / phase3_turn），决定奖励权重与日志/ckpt 命名",
    )
    parser.add_argument(
        "--ckpt-prefix",
        type=str,
        default=None,
        help="checkpoint 文件名前缀（默认取 --tag，或 ppo_walk）",
    )
    parser.add_argument(
        "--env",
        type=str,
        default=None,
        choices=["walk", "turn", "stairs"],
        help="环境类型：walk=42 维行走，turn=45 维转向，stairs=51 维台阶；默认按 --tag 自动选择",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="随机种子（可选）",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只解析参数并打印配置，不创建环境、不启动训练",
    )
    return parser.parse_args(argv)


def make_logdir(explicit: Optional[str], tag: Optional[str] = None) -> Path:
    """确定 TensorBoard 日志目录：未指定时用 runs/ppo_walk[_tag]_<时间戳>。"""
    if explicit:
        logdir = Path(explicit)
    else:
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        mid = f"_{tag}" if tag else ""
        logdir = Path(f"{DEFAULT_LOGDIR_PREFIX}{mid}_{stamp}")
    logdir.mkdir(parents=True, exist_ok=True)
    return logdir


def print_config(args: argparse.Namespace) -> None:
    """以中文打印当前生效的超参数 / 路径。"""
    env_kind = resolve_env_kind(args)
    print("=" * 60)
    print("【配置】PPO 训练四足机器狗行走")
    print("=" * 60)
    print(f"  环境类型        : {env_kind}")
    print(f"  观测维度        : {resolve_obs_dim(env_kind)}")
    print(f"  动作维度        : {ACTION_DIM}  (范围 [{ACTION_LOW}, {ACTION_HIGH}])")
    print(f"  策略网络        : {PPO_POLICY}  net_arch={NET_ARCH}")
    for key, value in PPO_PARAMS.items():
        print(f"  {key:<15}: {value}")
    print("-" * 60)
    print(f"  训练总步数      : {args.total_steps}")
    print(f"  计算设备        : {args.device}")
    print(f"  阶段标签        : {args.tag or '（未设置）'}")
    print(f"  日志目录        : {args.logdir or 'runs/ppo_walk[_tag]_<时间戳>'}")
    print(f"  checkpoint 目录 : {args.ckptdir}")
    print(f"  checkpoint 间隔 : {args.checkpoint_interval}")
    print(f"  评估间隔        : {args.eval_interval}  (每 {args.eval_episodes} 集)")
    print(f"  继续训练        : {args.resume or '（否）'}")
    print(f"  随机种子        : {args.seed if args.seed is not None else '（未设置）'}")
    print("=" * 60)


def require_deps() -> None:
    """真正训练前检查依赖是否齐全，缺失时给出中文提示。"""
    if SB3_IMPORT_ERROR is not None:
        raise SystemExit(
            "【错误】缺少依赖 stable-baselines3 / gymnasium，无法开始训练。\n"
            "  请先安装：pip install stable-baselines3 gymnasium torch\n"
            f"  原始报错：{SB3_IMPORT_ERROR}"
        )
    if WALK_ENV_IMPORT_ERROR is not None or QuadrupedWalkEnv is None:
        raise SystemExit(
            "【错误】无法导入 walk_env.QuadrupedWalkEnv，环境文件可能尚未就绪。\n"
            "  请确认 webots-sim/rl/walk_env.py 已存在且可导入。\n"
            f"  原始报错：{WALK_ENV_IMPORT_ERROR}"
        )


def resolve_env_kind(args: argparse.Namespace) -> str:
    """决定用哪个环境：'walk'（42 维）/ 'turn'（45 维）/ 'stairs'（51 维）。"""
    if args.env:
        return args.env
    tag = (args.tag or "").lower()
    if "stair" in tag or "phase4" in tag or "p4" in tag:
        return "stairs"
    if "turn" in tag or "phase3" in tag or "p3" in tag:
        return "turn"
    return "walk"


def resolve_obs_dim(env_kind: str) -> int:
    return {
        "turn": OBS_DIM_TURN,
        "stairs": OBS_DIM_STAIRS,
    }.get(env_kind, OBS_DIM)


def make_env_fn(
    render_mode: Optional[str] = None, env_kind: str = "walk"
) -> Callable[[], Any]:
    """返回一个创建环境的工厂函数（walk / turn / stairs）。"""

    def _factory() -> Any:
        if env_kind == "stairs":
            assert QuadrupedStairsEnv is not None, "walk_env_stairs.QuadrupedStairsEnv 不可用"
            cls = QuadrupedStairsEnv
        elif env_kind == "turn":
            assert QuadrupedTurnEnv is not None, "walk_env_turn.QuadrupedTurnEnv 不可用"
            cls = QuadrupedTurnEnv
        else:
            assert QuadrupedWalkEnv is not None
            cls = QuadrupedWalkEnv
        if render_mode is None:
            try:
                env = cls()
            except TypeError:
                env = cls(render_mode=None)
        else:
            try:
                env = cls(render_mode=render_mode)
            except TypeError:
                # 构造函数不接受 render_mode 时退化为无参构造
                env = cls()
        return Monitor(env)

    return _factory


def self_check_env(env_kind: str = "walk") -> None:
    """创建环境做一次快速自检：reset / step / 维度检查。"""
    obs_dim = resolve_obs_dim(env_kind)
    print(f"【自检】创建环境（{env_kind}）并执行 reset/step ...")
    t0 = time.time()
    if env_kind == "stairs":
        assert QuadrupedStairsEnv is not None
        env = QuadrupedStairsEnv()  # type: ignore[misc]
    elif env_kind == "turn":
        assert QuadrupedTurnEnv is not None
        env = QuadrupedTurnEnv()  # type: ignore[misc]
    else:
        assert QuadrupedWalkEnv is not None
        env = QuadrupedWalkEnv()  # type: ignore[misc]
    try:
        reset_out = env.reset()
        obs = reset_out[0] if isinstance(reset_out, tuple) else reset_out
        if hasattr(obs, "shape"):
            assert obs.shape[-1] == obs_dim, (
                f"观测维度不匹配：期望 {obs_dim}，实际 {obs.shape}"
            )
        action = _dummy_action(env)
        step_out = env.step(action)
        assert len(step_out) == 5, f"step() 应返回 5 元组，实际 {len(step_out)} 元组"
        print("【自检】通过：reset / step 接口正常")
    finally:
        close = getattr(env, "close", None)
        if callable(close):
            close()
    print(f"【自检】耗时 {time.time() - t0:.2f}s")


def _dummy_action(env: Any):
    """生成一个与动作空间匹配的零动作。"""
    space = getattr(env, "action_space", None)
    if space is not None and hasattr(space, "sample"):
        import numpy as np

        return np.zeros(space.shape, dtype=np.float32)
    import numpy as np

    return np.zeros(ACTION_DIM, dtype=np.float32)


# ===========================================================================
# 评估回调：每 EVAL_INTERVAL 步跑 N 个 episode，打印平均奖励 / 存活步数
# ===========================================================================
if BaseCallback is not object:

    class EvaluateCallback(BaseCallback):  # type: ignore[misc,valid-type]
        """定期在独立评估环境上跑若干 episode，中文打印结果。"""

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
            self._eval_env: Any = None
            self._last_eval_step = -1

        def _get_eval_env(self) -> Any:
            if self._eval_env is None:
                self._eval_env = self.eval_env_fn()
            return self._eval_env

        def _on_step(self) -> bool:
            # num_timesteps 在单环境下等于已收集步数
            if self.num_timesteps - self._last_eval_step < self.eval_freq:
                return True
            self._last_eval_step = self.num_timesteps
            self._run_eval()
            return True

        def _run_eval(self) -> None:
            try:
                env = self._get_eval_env()
            except Exception as exc:  # noqa: BLE001 - Webots 可能无法双开
                print(f"【评估】创建评估环境失败，跳过本次评估：{exc}")
                self.eval_freq = 10**12  # 之后不再尝试
                return

            rewards = []
            lengths = []
            for ep in range(self.eval_episodes):
                reset_out = env.reset()
                obs = reset_out[0] if isinstance(reset_out, tuple) else reset_out
                done = False
                ep_reward = 0.0
                ep_len = 0
                while not done:
                    action, _ = self.model.predict(obs, deterministic=True)
                    step_out = env.step(action)
                    if len(step_out) == 5:
                        obs, reward, terminated, truncated, _info = step_out
                        done = bool(terminated) or bool(truncated)
                    else:  # 兼容旧版 gym 接口
                        obs, reward, done, _info = step_out
                        done = bool(done)
                    ep_reward += float(reward)
                    ep_len += 1
                rewards.append(ep_reward)
                lengths.append(ep_len)

            avg_r = sum(rewards) / len(rewards)
            avg_len = sum(lengths) / len(lengths)
            print(
                f"【评估】步数={self.num_timesteps} "
                f"平均奖励={avg_r:.3f} 平均存活步数={avg_len:.1f} "
                f"(共 {self.eval_episodes} 集)"
            )

        def _on_training_end(self) -> None:
            if self._eval_env is not None:
                close = getattr(self._eval_env, "close", None)
                if callable(close):
                    close()
                self._eval_env = None

else:  # pragma: no cover - SB3 缺失时的占位
    class EvaluateCallback:  # type: ignore[no-redef]
        """占位类：stable-baselines3 缺失时不会被真正使用。"""

        def __init__(self, *args: Any, **kwargs: Any) -> None:
            raise RuntimeError("stable-baselines3 未安装，EvaluateCallback 不可用")


# ===========================================================================
# 热启动：obs 维度不一致时扩展首层后拷贝权重
# ===========================================================================
def _expand_first_layer(state_dict, old_sd, prefix: str):
    """把旧网络首层权重复制到新网络：新增的输入列置零。"""
    w_key, b_key = f"{prefix}.weight", f"{prefix}.bias"
    if w_key not in state_dict or w_key not in old_sd:
        return False
    old_w, new_w = old_sd[w_key], state_dict[w_key]
    if old_w.shape == new_w.shape:
        state_dict[w_key] = old_w.clone()
        if b_key in state_dict and b_key in old_sd:
            state_dict[b_key] = old_sd[b_key].clone()
        return True
    if old_w.shape[0] == new_w.shape[0] and old_w.shape[1] < new_w.shape[1]:
        # 输入维变大：右侧新增列置零（新观测通道初始不干扰）
        w = torch.zeros_like(new_w)
        w[:, : old_w.shape[1]] = old_w
        state_dict[w_key] = w
        if b_key in state_dict and b_key in old_sd:
            state_dict[b_key] = old_sd[b_key].clone()
        return True
    return False


def load_resume_model(resume_path: Path, train_env: Any, device: str, logdir: Path):
    """加载旧模型做热启动。

    obs 维度一致 → 直接 PPO.load；
    obs 维度不一致（如 42 → 45）→ 新建模型后逐层拷贝可迁移权重，
    首层输入维扩展处用 0 填充新增通道。
    """
    import torch  # noqa: F811 - 显式导入，函数内也可独立使用
    from stable_baselines3 import PPO as _PPO

    print(f"【训练】从 {resume_path} 继续训练 ...")
    # 先不带 env 加载，读出旧策略参数
    old_model = _PPO.load(str(resume_path), device=device)
    old_obs_dim = int(old_model.observation_space.shape[0])
    new_obs_dim = int(train_env.observation_space.shape[0])
    print(f"【训练】旧模型 obs_dim={old_obs_dim}，新环境 obs_dim={new_obs_dim}")

    if old_obs_dim == new_obs_dim:
        model = _PPO.load(
            str(resume_path),
            env=train_env,
            device=device,
            tensorboard_log=str(logdir),
        )
        print(f"【训练】已加载模型，当前累计步数 = {model.num_timesteps}")
        return model

    # 维度不一致：新建模型 + 权重迁移
    print("【训练】obs 维度不一致，执行首层扩展热启动 ...")
    model = _PPO(
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
        device=device,
        verbose=1,
    )
    old_sd = old_model.policy.state_dict()
    new_sd = model.policy.state_dict()
    copied, expanded = 0, 0
    for key in new_sd:
        if key not in old_sd:
            continue
        if old_sd[key].shape == new_sd[key].shape:
            new_sd[key] = old_sd[key].clone().to(new_sd[key].device)
            copied += 1
        elif key.endswith(".weight") and old_sd[key].dim() == 2 and new_sd[key].dim() == 2:
            # 可能是首层 Linear(obs_dim → hidden)
            prefix = key[: -len(".weight")]
            if _expand_first_layer(new_sd, old_sd, prefix):
                expanded += 1
    model.policy.load_state_dict(new_sd)
    print(
        f"【训练】热启动完成：同形状拷贝 {copied} 个参数张量，"
        f"首层扩展 {expanded} 处（新增输入通道置零）"
    )
    return model


# ===========================================================================
# 训练主流程
# ===========================================================================
def train(args: argparse.Namespace) -> Path:
    """执行训练，返回最终模型保存路径。"""
    require_deps()

    # 环境类型：walk（42 维）/ turn（45 维）/ stairs（51 维）
    env_kind = resolve_env_kind(args)
    obs_dim = resolve_obs_dim(env_kind)
    if env_kind == "turn" and QuadrupedTurnEnv is None:
        raise SystemExit("【错误】需要 walk_env_turn.QuadrupedTurnEnv，但导入失败")
    if env_kind == "stairs" and QuadrupedStairsEnv is None:
        raise SystemExit("【错误】需要 walk_env_stairs.QuadrupedStairsEnv，但导入失败")
    print(f"【训练】环境类型 = {env_kind}，观测维度 = {obs_dim}")

    # 按阶段标签切换奖励权重
    if args.tag:
        weights = set_phase(args.tag)
        print(f"【训练】阶段 = {args.tag}，奖励权重 = {weights}")

    logdir = make_logdir(args.logdir, tag=args.tag)
    ckptdir = Path(args.ckptdir)
    ckptdir.mkdir(parents=True, exist_ok=True)
    print(f"【训练】日志目录 = {logdir}")
    print(f"【训练】checkpoint 目录 = {ckptdir}")

    # checkpoint 文件名前缀：显式 --ckpt-prefix 优先，其次 --tag，最后默认前缀
    name_prefix = (
        args.ckpt_prefix or args.tag or CKPT_NAME_PREFIX
    )

    # 环境自检（--dry-run 不会走到这里）
    self_check_env(env_kind)

    # 训练环境
    train_env = DummyVecEnv([make_env_fn(env_kind=env_kind)])
    print(f"【训练】训练环境已创建，观测维度={obs_dim}，动作维度={ACTION_DIM}")

    # 回调：定期保存 checkpoint + 定期评估
    callbacks = []
    checkpoint_cb = CheckpointCallback(
        save_freq=max(1, args.checkpoint_interval),
        save_path=str(ckptdir),
        name_prefix=name_prefix,
        save_replay_buffer=False,
        save_vecnormalize=False,
    )
    callbacks.append(checkpoint_cb)
    print(
        f"【训练】每 {args.checkpoint_interval} 步保存 checkpoint → "
        f"{ckptdir}/{name_prefix}_<step>.zip"
    )

    if args.eval_interval and args.eval_interval > 0 and args.eval_episodes > 0:
        callbacks.append(
            EvaluateCallback(
                eval_env_fn=make_env_fn(env_kind=env_kind),
                eval_freq=args.eval_interval,
                eval_episodes=args.eval_episodes,
            )
        )
        print(
            f"【训练】每 {args.eval_interval} 步评估一次"
            f"（{args.eval_episodes} 集平均）"
        )
    else:
        print("【训练】已关闭定期评估")

    # 构建 / 加载模型
    if args.resume:
        resume_path = Path(args.resume)
        if not resume_path.is_file():
            raise SystemExit(f"【错误】找不到要继续训练的模型：{resume_path}")
        model = load_resume_model(resume_path, train_env, args.device, logdir)
        print(f"【训练】已加载模型，当前累计步数 = {model.num_timesteps}")
    else:
        print("【训练】创建新 PPO 模型 ...")
        model = PPO(  # type: ignore[misc]
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
            f"【训练】模型已创建：policy={PPO_POLICY}, net_arch={NET_ARCH}, "
            f"device={args.device}"
        )

    # 本次要再跑多少步（resume 时从当前累计步数继续往 total-steps 凑）
    if args.resume:
        remaining = max(0, int(args.total_steps) - int(model.num_timesteps))
        if remaining == 0:
            print(
                f"【训练】当前累计步数 {model.num_timesteps} 已达目标 "
                f"{args.total_steps}，无需继续训练"
            )
            final_path = ckptdir / f"{name_prefix}_final.zip"
            model.save(str(final_path))
            return final_path
    else:
        remaining = int(args.total_steps)

    print(f"【训练】开始训练：本次运行 {remaining} 步（目标总步数 {args.total_steps}）")
    t0 = time.time()
    model.learn(  # type: ignore[union-attr]
        total_timesteps=remaining,
        callback=callbacks,
        reset_num_timesteps=not args.resume,
        progress_bar=False,
    )
    elapsed = time.time() - t0
    fps = remaining / elapsed if elapsed > 0 else 0.0
    print(
        f"【训练】完成：本次 {remaining} 步，耗时 {elapsed/60:.1f} 分钟"
        f"（约 {fps:.1f} steps/s）"
    )
    print(f"【训练】累计总步数 = {model.num_timesteps}")

    # 保存最终模型
    final_path = ckptdir / f"{name_prefix}_final.zip"
    model.save(str(final_path))
    print(f"【训练】最终模型已保存：{final_path}")

    # 关闭环境
    train_env.close()
    return final_path


def main(argv: Optional[list] = None) -> None:
    args = parse_args(argv)
    print_config(args)
    if args.dry_run:
        # dry-run：只做参数解析与配置打印，跳过环境自检与训练
        env_status = "就绪" if QuadrupedWalkEnv is not None else "未就绪（import 失败）"
        sb3_status = "就绪" if SB3_IMPORT_ERROR is None else "未安装"
        print("【dry-run】参数解析成功，不启动训练。")
        print(f"【dry-run】walk_env 状态：{env_status}")
        print(f"【dry-run】stable-baselines3 状态：{sb3_status}")
        return
    train(args)


if __name__ == "__main__":
    main()
