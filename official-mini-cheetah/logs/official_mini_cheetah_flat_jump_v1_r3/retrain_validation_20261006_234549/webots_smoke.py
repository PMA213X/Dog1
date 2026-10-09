#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""真实 Webots 四机 Rapid zero/forward 固定命令冒烟。"""

from __future__ import annotations

import json
import math
import os
import sys
import time
import traceback
from pathlib import Path

import numpy as np
import torch


PROJECT_ROOT = Path(
    "/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/official-mini-cheetah"
)
os.chdir(PROJECT_ROOT)
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from rl import contract  # noqa: E402
from rl.rapid_finetune_policy import RapidActorCriticPolicy  # noqa: E402
from rl.rapid_finetune_vec_env import RapidFinetuneVecEnv  # noqa: E402
from rl.rapid_policy import RAPID_MODEL_DIR  # noqa: E402
from rl.shared_world_vec_env import SharedWorldVecEnv  # noqa: E402


STEPS_PER_CASE = 200
SEED = 20_261_006
RESULT_ROOT = contract.LOG_ROOT / "retrain_validation_20261006_234549"


def finite_array(values: object) -> bool:
    """确认数组全部为有限数。"""
    return bool(np.all(np.isfinite(np.asarray(values, dtype=np.float64))))


def set_case_command(env: RapidFinetuneVecEnv, command: tuple[float, float, float]) -> None:
    """把四个 worker 固定为同一 case 命令并刷新策略观测。"""
    selected = np.asarray(command, dtype=np.float32)
    for worker in env.envs:
        worker._command = selected.copy()
        worker._command_fixed = True
        worker._command_step = contract.COMMAND_RESAMPLE_STEPS
    raw = np.stack([worker._observation() for worker in env.envs]).astype(np.float32)
    env._histories = [[] for _ in range(env.num_envs)]
    env._refresh_observation(raw, range(env.num_envs))
    return env._encode(
        raw,
        source_actions=env._previous_source_actions,
        append=True,
    )


def summarize_case(
    *,
    name: str,
    command: tuple[float, float, float],
    rewards: list[float],
    vx: list[float],
    contacts: list[float],
    zero_contacts: list[int],
    dones: int,
    saturation: list[float],
    target_limits: list[float],
    target_lags: list[float],
    fallen_count: int,
    emergency_stop_count: int,
    termination_reasons: list[str],
    started: float,
) -> dict[str, object]:
    """汇总单个固定命令 case 的运行证据。"""
    return {
        "name": name,
        "command": list(command),
        "steps": STEPS_PER_CASE,
        "robot_steps": STEPS_PER_CASE * contract.PARALLEL_WORKERS,
        "rewards_finite": bool(rewards) and finite_array(rewards),
        "reward_mean": float(np.mean(rewards)),
        "mean_vx": float(np.mean(vx)),
        "mean_vx_abs": float(np.mean(np.abs(vx))),
        "average_contact_feet": float(np.mean(contacts)),
        "zero_contact_ratio": float(np.mean(zero_contacts)),
        "synchronous_done_count": int(dones),
        "source_action_saturation_mean": float(np.mean(saturation)),
        "action_saturation_mean": float(np.mean(saturation)),
        "target_rate_limit_values": sorted(set(target_limits)),
        "target_lag_mean": float(np.mean(target_lags)),
        "target_lag_max": float(np.max(target_lags)),
        "fall_count": int(fallen_count),
        "emergency_stop_count": int(emergency_stop_count),
        "termination_reasons": sorted(set(termination_reasons)),
        "elapsed_seconds": time.monotonic() - started,
    }


def run() -> dict[str, object]:
    """执行两个 case，每个 case 使用四机共享 world 连续推进 1000 vector step。"""
    RESULT_ROOT.mkdir(parents=True, exist_ok=True)
    if os.environ.get("CUDA_VISIBLE_DEVICES") != "0":
        raise RuntimeError("CUDA_VISIBLE_DEVICES 必须精确为 0")
    if os.environ.get("RL_ACTION_TARGET_RATE_LIMIT") != "1.0":
        raise RuntimeError(
            "RL_ACTION_TARGET_RATE_LIMIT 必须精确为 1.0"
        )
    if contract.ACTION_TARGET_RATE_LIMIT != 1.0:
        raise RuntimeError(
            f"契约目标限速必须为 1.0，实际为 "
            f"{contract.ACTION_TARGET_RATE_LIMIT}"
        )
    if not torch.cuda.is_available() or torch.cuda.device_count() != 1:
        raise RuntimeError("CUDA 唯一设备校验失败")
    if torch.cuda.get_device_name(0) != "NVIDIA GeForce RTX 4060 Laptop GPU":
        raise RuntimeError(f"GPU 名称不符：{torch.cuda.get_device_name(0)}")

    base = SharedWorldVecEnv(
        num_envs=4,
        phase="P2",
        bridge_port=11452,
        randomization_mode="fixed",
        render_mode=None,
        seed=SEED,
    )
    env = RapidFinetuneVecEnv(
        base,
        model_dir=RAPID_MODEL_DIR,
        command_course=False,
        finetune_mode=True,
        seed=SEED,
    )
    policy = RapidActorCriticPolicy(
        env.observation_space,
        env.action_space,
        lambda _progress: 0.0,
        rapid_model_dir=str(RAPID_MODEL_DIR),
    ).to("cuda:0")
    policy.eval()
    observations = env.reset()
    summaries: list[dict[str, object]] = []
    try:
        cases = (
            ("zero", (0.0, 0.0, 0.0)),
            ("forward", (0.18, 0.0, 0.0)),
        )
        for case_name, command in cases:
            started = time.monotonic()
            observations = set_case_command(env, command)
            rewards: list[float] = []
            vx: list[float] = []
            contacts: list[float] = []
            zero_contacts: list[int] = []
            saturation: list[float] = []
            target_limits: list[float] = []
            target_lags: list[float] = []
            termination_reasons: list[str] = []
            fallen_count = 0
            emergency_stop_count = 0
            done_count = 0
            for step in range(1, STEPS_PER_CASE + 1):
                if not all(finite_array(observations[key]) for key in ("current", "history")):
                    raise RuntimeError(f"{case_name} step={step} 观测非有限")
                actions, _state = policy.predict(observations, deterministic=True)
                actions = np.asarray(actions, dtype=np.float32)
                if actions.shape != (4, 12) or not finite_array(actions):
                    raise RuntimeError(f"{case_name} step={step} 源动作非有限")
                env.step_async(actions)
                observations, step_rewards, dones, infos = env.step_wait()
                step_rewards = np.asarray(step_rewards, dtype=np.float32)
                if step_rewards.shape != (4,) or not finite_array(step_rewards):
                    raise RuntimeError(f"{case_name} step={step} reward 非有限")
                rewards.extend(step_rewards.tolist())
                if bool(np.any(dones)):
                    if not bool(np.all(dones)):
                        raise RuntimeError(f"{case_name} step={step} 非同步终止")
                    done_count += 1
                for info in infos:
                    state = info.get("state") or {}
                    command_value = info.get("command")
                    if command_value is not None and not np.allclose(
                        np.asarray(command_value, dtype=np.float32),
                        np.asarray(command, dtype=np.float32),
                        atol=1e-6,
                    ):
                        raise RuntimeError(f"{case_name} step={step} 命令漂移")
                    velocity = info.get("v_body")
                    if velocity is None:
                        velocity = state.get("body_linear_velocity")
                    if velocity is None:
                        velocity = state.get("velocity")
                    if velocity is None:
                        velocity_value = float(info.get("mean_vx", 0.0))
                    else:
                        velocity_value = float(np.asarray(velocity).reshape(-1)[0])
                    vx.append(velocity_value)
                    contact_values = info.get("foot_contacts")
                    if contact_values is None:
                        contact_count = 0.0
                    else:
                        contact_count = float(
                            np.sum(np.asarray(contact_values, dtype=np.float32) >= 0.5)
                        )
                    contacts.append(contact_count)
                    zero_contacts.append(int(contact_count == 0.0))
                    mapped = info.get("rapid_mapped_action")
                    if mapped is not None and finite_array(mapped):
                        saturation.append(
                            float(np.mean(np.abs(mapped) >= 0.999999))
                        )
                    target_limit = float(info.get("target_rate_limit", math.nan))
                    target_lag = float(info.get("target_lag_mean", math.nan))
                    if not math.isfinite(target_limit) or not math.isfinite(
                        target_lag
                    ):
                        raise RuntimeError(
                            f"{case_name} step={step} 执行遥测非有限"
                        )
                    target_limits.append(target_limit)
                    target_lags.append(target_lag)
                    if bool(info.get("fallen", False)):
                        fallen_count += 1
                    reason = str(info.get("termination_reason", ""))
                    if reason:
                        termination_reasons.append(reason)
                    if bool(info.get("estop", False)) or bool(
                        info.get("emergency_stop", False)
                    ):
                        emergency_stop_count += 1
                if step % 100 == 0:
                    print(
                        f"SMOKE_PROGRESS case={case_name} step={step} "
                        f"reward_mean={float(np.mean(step_rewards)):.6f} "
                        f"done_count={done_count}",
                        flush=True,
                    )
            if len(rewards) != STEPS_PER_CASE * 4:
                raise RuntimeError(f"{case_name} transition 数量错误")
            if set(target_limits) != {1.0}:
                raise RuntimeError(
                    f"{case_name} 目标限速不是 1.0：{sorted(set(target_limits))}"
                )
            if fallen_count or emergency_stop_count or termination_reasons:
                raise RuntimeError(
                    f"{case_name} 出现跌倒或急停："
                    f"fall={fallen_count} estop={emergency_stop_count} "
                    f"reasons={sorted(set(termination_reasons))}"
                )
            summaries.append(
                summarize_case(
                    name=case_name,
                    command=command,
                    rewards=rewards,
                    vx=vx,
                    contacts=contacts,
                    zero_contacts=zero_contacts,
                    dones=done_count,
                    saturation=saturation or [0.0],
                    target_limits=target_limits,
                    target_lags=target_lags,
                    fallen_count=fallen_count,
                    emergency_stop_count=emergency_stop_count,
                    termination_reasons=termination_reasons,
                    started=started,
                )
            )
        payload = {
            "status": "passed",
            "contract": contract.CONTRACT_VERSION,
            "world": str(contract.WORLD_PATH),
            "num_envs": 4,
            "ports": list(contract.bridge_ports(4)),
            "steps_per_case": STEPS_PER_CASE,
            "action_target_rate_limit": contract.ACTION_TARGET_RATE_LIMIT,
            "cases": summaries,
            "gpu": torch.cuda.get_device_name(0),
        }
    finally:
        env.close()
    result_path = RESULT_ROOT / "result.json"
    result_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False)
        + "\n",
        encoding="utf-8",
    )
    print(
        "SMOKE_SUMMARY "
        + json.dumps(payload, ensure_ascii=False, allow_nan=False),
        flush=True,
    )
    return payload


if __name__ == "__main__":
    try:
        run()
    except Exception:
        traceback.print_exc()
        raise SystemExit(1)
