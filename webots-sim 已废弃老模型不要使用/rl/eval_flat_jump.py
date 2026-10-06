#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""yobogo_flat_jump_v1 的确定性评估与阶段验收门。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Optional, Sequence

import numpy as np

import flat_jump_contract as contract


DEFAULT_SEED = 20261001


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """解析评估参数。"""
    parser = argparse.ArgumentParser(description="平地 RL 确定性评估")
    parser.add_argument(
        "--phase",
        required=True,
        choices=tuple(contract.PHASE_TOTAL_STEPS),
    )
    parser.add_argument("--episodes", type=int, default=10)
    parser.add_argument("--max-steps", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument("--gate", action="store_true")
    parser.add_argument(
        "--randomization-mode",
        type=str,
        default="full",
        choices=("fixed", "curriculum", "full"),
        help="地域随机化模式；F1 正式验收必须为 full",
    )
    parser.add_argument("--bridge-port", type=int, default=11452)
    parser.add_argument("--output", type=str, default=None)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    args.phase = contract.normalize_phase(args.phase)
    if args.episodes <= 0 or args.max_steps <= 0:
        parser.error("--episodes 和 --max-steps 必须为正整数")
    if not 1024 <= args.bridge_port <= 65535:
        parser.error("--bridge-port 必须位于 1024～65535")
    if args.gate and args.phase == "F1" and args.randomization_mode != "full":
        parser.error("F1 正式验收必须使用 --randomization-mode full")
    return args


def checkpoint_path() -> Path:
    """返回新前缀 final checkpoint。"""
    path = (
        Path("checkpoints/yobogo_flat_jump_v1")
        / f"{contract.CHECKPOINT_PREFIX}_final.zip"
    )
    return contract.validate_checkpoint_path(path)


def run_episode(
    model: Any,
    env: Any,
    *,
    seed: int,
    max_steps: int,
    command: Optional[Sequence[float]] = None,
) -> dict[str, Any]:
    """运行一集确定性策略并汇总指标。"""
    options: dict[str, Any] = {}
    if command is not None:
        options["command"] = np.asarray(command, dtype=np.float32)
    obs, info = env.reset(seed=seed, options=options)
    reset_initial = {
        "height": float(env._last_state.get("height", 0.0)),
        "roll": float(env._last_state["rpy"][0]),
        "pitch": float(env._last_state["rpy"][1]),
        "contacts": [
            int(value) for value in env._last_state.get("contacts", [])
        ],
        "randomization": dict(info.get("randomization", {})),
        "randomization_mode": info.get("randomization_mode"),
        "curriculum_episode": info.get("curriculum_episode"),
    }
    start_position = np.asarray(
        env._last_state.get("x", 0.0), dtype=np.float64
    ), np.asarray(env._last_state.get("y", 0.0), dtype=np.float64)
    start_yaw = float(env._last_state["rpy"][2])
    total_reward = 0.0
    fallen = bool(info.get("fallen", False))
    jump_success = bool(info.get("jump_success", False))
    command_errors: list[float] = []
    first_action: Optional[list[float]] = None
    steps = 0
    for steps in range(1, max_steps + 1):
        action, _ = model.predict(obs, deterministic=True)
        if first_action is None:
            first_action = [
                float(value)
                for value in np.asarray(action, dtype=np.float64).reshape(-1)
            ]
        obs, reward, terminated, truncated, info = env.step(
            np.asarray(action, dtype=np.float32)
        )
        total_reward += float(reward)
        fallen = fallen or bool(info.get("fallen", False))
        jump_success = jump_success or bool(info.get("jump_success", False))
        command_errors.append(float(info.get("command_error", 0.0)))
        if terminated or truncated:
            break
    final_state = env._last_state
    end_x = float(final_state.get("x", start_position[0]))
    end_y = float(final_state.get("y", start_position[1]))
    end_yaw = float(final_state["rpy"][2])
    yaw_delta = float((end_yaw - start_yaw + np.pi) % (2 * np.pi) - np.pi)
    termination_reason = info.get("termination_reason")
    if termination_reason is None:
        termination_reason = "time_limit" if truncated else (
            "fallen" if fallen else "not_terminated"
        )
    return {
        "seed": seed,
        "steps": steps,
        "reward": total_reward,
        "fallen": fallen,
        "jump_success": jump_success,
        "reset_initial": reset_initial,
        "first_action": first_action,
        "termination_reason": termination_reason,
        "randomization": dict(info.get("randomization", {})),
        "command_error_mean": float(np.mean(command_errors))
        if command_errors
        else None,
        "dx": end_x - float(start_position[0]),
        "dy": end_y - float(start_position[1]),
        "yaw_delta": yaw_delta,
    }


def run_zero_action_baseline(
    env: Any,
    *,
    seed: int,
    max_steps: int,
) -> dict[str, Any]:
    """运行一集零动作 reset 基线，区分出生姿态与策略动作问题。"""
    obs, info = env.reset(
        seed=seed,
        options={"command": np.zeros(3, dtype=np.float32)},
    )
    reset_initial = {
        "height": float(env._last_state.get("height", 0.0)),
        "roll": float(env._last_state["rpy"][0]),
        "pitch": float(env._last_state["rpy"][1]),
        "contacts": [
            int(value) for value in env._last_state.get("contacts", [])
        ],
        "randomization": dict(info.get("randomization", {})),
        "randomization_mode": info.get("randomization_mode"),
        "curriculum_episode": info.get("curriculum_episode"),
    }
    action = np.zeros(env.action_space.shape, dtype=np.float32)
    steps = 0
    fallen = bool(info.get("fallen", False))
    terminated = False
    truncated = False
    for steps in range(1, max_steps + 1):
        obs, _reward, terminated, truncated, info = env.step(action)
        fallen = fallen or bool(info.get("fallen", False))
        if terminated or truncated:
            break
    termination_reason = info.get("termination_reason")
    if termination_reason is None:
        termination_reason = "time_limit" if truncated else (
            "fallen" if fallen else "not_terminated"
        )
    return {
        "seed": seed,
        "steps": steps,
        "fallen": fallen,
        "termination_reason": termination_reason,
        "reset_initial": reset_initial,
        "randomization": dict(info.get("randomization", {})),
    }


def run_f1_gate(model: Any, env: Any, args: argparse.Namespace) -> dict[str, Any]:
    """F1：十集稳定，摔倒率不高于 10%，平均存活至少 950 步。"""
    episodes = [
        run_episode(
            model,
            env,
            seed=args.seed + index,
            max_steps=args.max_steps,
            command=(0.0, 0.0, 0.0),
        )
        for index in range(args.episodes)
    ]
    zero_action_baseline = run_zero_action_baseline(
        env,
        seed=args.seed + 10_000,
        max_steps=args.max_steps,
    )
    fall_rate = float(np.mean([item["fallen"] for item in episodes]))
    mean_steps = float(np.mean([item["steps"] for item in episodes]))
    return {
        "episodes": episodes,
        "zero_action_reset_baseline": zero_action_baseline,
        "fall_rate": fall_rate,
        "mean_steps": mean_steps,
        "passed": fall_rate <= 0.10 and mean_steps >= 950.0,
    }


def run_f2_gate(model: Any, env: Any, args: argparse.Namespace) -> dict[str, Any]:
    """F2：前后、横移和转向六个规范命令均产生正确方向位移。"""
    cases = [
        ("forward", (0.6, 0.0, 0.0), "dx", 1.0),
        ("backward", (-0.3, 0.0, 0.0), "dx", -1.0),
        ("left", (0.0, 0.3, 0.0), "dy", 1.0),
        ("right", (0.0, -0.3, 0.0), "dy", -1.0),
        ("turn_left", (0.0, 0.0, 1.0), "yaw_delta", 1.0),
        ("turn_right", (0.0, 0.0, -1.0), "yaw_delta", -1.0),
    ]
    episodes: dict[str, dict[str, Any]] = {}
    passed = True
    for index, (name, command, metric, sign) in enumerate(cases):
        episode = run_episode(
            model,
            env,
            seed=args.seed + index,
            max_steps=min(args.max_steps, 500),
            command=command,
        )
        episode["metric"] = metric
        episode["metric_value"] = float(episode[metric])
        episode["direction_ok"] = episode["metric_value"] * sign > 0.0
        episode["not_fallen"] = not episode["fallen"]
        episode["case_passed"] = bool(
            episode["direction_ok"] and episode["not_fallen"]
        )
        episodes[name] = episode
        passed = passed and episode["case_passed"]
    return {"episodes": episodes, "passed": passed}


def run_f3_gate(model: Any, env: Any, args: argparse.Namespace) -> dict[str, Any]:
    """F3：十集跳跃整合，摔倒率不高于 10%，跳跃成功率至少 80%。"""
    episodes = [
        run_episode(
            model,
            env,
            seed=args.seed + index,
            max_steps=args.max_steps,
        )
        for index in range(args.episodes)
    ]
    fall_rate = float(np.mean([item["fallen"] for item in episodes]))
    jump_rate = float(np.mean([item["jump_success"] for item in episodes]))
    mean_steps = float(np.mean([item["steps"] for item in episodes]))
    return {
        "episodes": episodes,
        "fall_rate": fall_rate,
        "jump_success_rate": jump_rate,
        "mean_steps": mean_steps,
        "passed": fall_rate <= 0.10 and jump_rate >= 0.80,
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    """执行评估；任何非有限指标或验收失败都返回非零。"""
    args = parse_args(argv)
    checkpoint = checkpoint_path()
    if args.dry_run:
        print(
            f"phase={args.phase} checkpoint={checkpoint} "
            f"episodes={args.episodes} gate={args.gate}"
        )
        return 0
    if not checkpoint.is_file():
        print(f"【错误】checkpoint 不存在：{checkpoint}")
        return 2

    from stable_baselines3 import PPO
    from flat_jump_env import FlatGroundJumpEnv

    model = PPO.load(str(checkpoint), device="cuda")
    env = FlatGroundJumpEnv(
        phase=args.phase,
        bridge_port=args.bridge_port,
        command_resample_steps=100_000,
        randomization_mode=args.randomization_mode,
    )
    try:
        if args.phase == "F1":
            report = run_f1_gate(model, env, args)
        elif args.phase == "F2":
            report = run_f2_gate(model, env, args)
        else:
            report = run_f3_gate(model, env, args)
    finally:
        env.close()

    result = {
        "status": "ok" if report.get("passed") else "failed",
        "phase": args.phase,
        "checkpoint": str(checkpoint.resolve()),
        "deterministic": True,
        "device": "cuda",
        "gate": bool(args.gate),
        "randomization_mode": args.randomization_mode,
        **report,
    }
    serialized = json.dumps(result, ensure_ascii=False, indent=2)
    if not np.isfinite(
        float(report.get("mean_steps", report.get("passed", 0.0)))
    ):
        print("【错误】评估指标包含 NaN/Inf")
        return 3
    if args.output:
        Path(args.output).write_text(serialized + "\n", encoding="utf-8")
    print(serialized)
    if args.gate and not report.get("passed", False):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
