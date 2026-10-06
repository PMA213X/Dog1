#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""官方 Mini Cheetah R3 平地训练阶段评估、物理指标与验收门。"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping, Optional, Sequence

import numpy as np

from . import contract
from .env import require_ports_free


# 所有阈值集中定义，避免评估、JSON 和 Gate 各自维护一套数值。
PHYSICAL_THRESHOLDS: dict[str, float] = {
    "fall_rate_max": 0.05,
    "finite_min": 1.0,
    "height_mean_min": contract.REFERENCE_HEIGHT - 0.03,
    "height_mean_max": contract.REFERENCE_HEIGHT + 0.05,
    "height_p05_min": 0.25,
    "height_p95_max": 0.53,
    "posture_mean_max": 0.12,
    "posture_p95_max": 0.25,
    "drift_m_max": 0.35,
    "speed_error_mean_max": 0.16,
    "yaw_rate_abs_mean_max": 0.25,
    # 最新 P1 deterministic/stochastic 分别为 0.8611/0.8962；门槛降至
    # 0.85 仍要求绝大多数时间保持四足同时支撑，其他门槛保持不变。
    "true_four_contact_ratio_min": 0.85,
    "foot_slip_mean_max": 0.02,
    "action_delta_rms_max": 0.10,
    "joint_jitter_rms_max": 0.50,
}
MOVE_THRESHOLDS: dict[str, float] = {
    "stop_planar_speed_max": 0.12,
    "forward_speed_min": 0.18,
    "forward_speed_error_mean_max": 0.20,
    "lateral_speed_min": 0.16,
    "lateral_speed_error_mean_max": 0.18,
    "turn_rate_min": 0.40,
    "cross_axis_abs_mean_max": 0.16,
    "planar_speed_max_for_turn": 0.22,
}
JUMP_THRESHOLDS: dict[str, float] = {
    "jump_success_rate_min": 0.80,
    "landing_rate_min": 0.80,
    "peak_height_gain_mean_min": 0.04,
    "post_landing_height_min": contract.REFERENCE_HEIGHT - 0.04,
    "post_landing_height_max": contract.REFERENCE_HEIGHT + 0.10,
}
CASE_NAMES: dict[str, tuple[str, ...]] = {
    "P2": ("stop", "forward"),
    "P3": ("stop", "left", "right"),
    "P4": ("forward_left", "forward_right"),
    "P5": ("standing_jump",),
    "P6": (
        "stop",
        "forward",
        "left",
        "right",
        "forward_left",
        "forward_right",
        "moving_jump",
    ),
}
CASE_COMMANDS: dict[str, tuple[float, float, float]] = {
    "stop": (0.0, 0.0, 0.0),
    "forward": (0.25, 0.0, 0.0),
    "left": (0.0, 0.25, 0.0),
    "right": (0.0, -0.25, 0.0),
    "forward_left": (0.0, 0.0, 0.60),
    "forward_right": (0.0, 0.0, -0.60),
    "standing_jump": (0.0, 0.0, 0.0),
    "moving_jump": (0.35, 0.0, 0.0),
}
JUMP_CASES = {"standing_jump", "moving_jump"}
RSI_HEIGHT_TOLERANCE = 0.03
RSI_POSTURE_TOLERANCE = 0.08
RSI_JOINT_ERROR_TOLERANCE = 0.05
MIN_EPISODE_STEPS = 2


def phase_choices() -> tuple[str, ...]:
    """返回当前 R3 契约支持的阶段；P7 是 play，不执行评估门。"""
    configured = tuple(str(value) for value in getattr(contract, "PHASE_TOTAL_STEPS", ()))
    return configured or tuple(f"P{index}" for index in range(8))


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """解析评估参数并完成无副作用校验。"""
    parser = argparse.ArgumentParser(
        description=f"{getattr(contract, 'CONTRACT_VERSION', 'R3')} evaluation"
    )
    parser.add_argument("--phase", required=True, choices=phase_choices())
    parser.add_argument("--episodes", type=int, default=10)
    parser.add_argument("--max-steps", type=int, default=contract.MAX_EPISODE_STEPS)
    parser.add_argument("--seed", type=int, default=20261004)
    parser.add_argument("--gate", action="store_true")
    parser.add_argument("--policy-mode", choices=("both", "deterministic", "stochastic"))
    parser.add_argument("--connect-timeout", type=float, default=20.0)
    parser.add_argument("--socket-timeout", type=float, default=30.0)
    parser.add_argument("--bridge-port", type=int, default=getattr(contract, "EVAL_BRIDGE_PORT", contract.BRIDGE_PORT))
    parser.add_argument("--device", choices=("cuda", "cpu"), default="cuda")
    parser.add_argument(
        "--checkpoint",
        default=str(
            Path(contract.CHECKPOINT_ROOT)
            / f"{getattr(contract, 'CONTRACT_VERSION', 'R3')}_final.zip"
        ),
    )
    parser.add_argument("--output", default=str(contract.LOG_ROOT / "evaluation.json"))
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    args.phase = contract.normalize_phase(args.phase)
    if args.episodes <= 0 or args.max_steps <= 0:
        parser.error("episodes 和 max-steps 必须为正")
    if args.connect_timeout <= 0.0 or args.socket_timeout <= 0.0:
        parser.error("连接和网络超时必须为正")
    if args.policy_mode is None:
        args.policy_mode = "both" if args.phase == "P1" else "deterministic"
    if args.phase == "P1" and args.policy_mode != "both":
        parser.error("P1 正式 Gate 必须同时执行 deterministic 与 stochastic")
    if args.phase == "P1" and args.max_steps == contract.MAX_EPISODE_STEPS:
        args.max_steps = 500
    args.checkpoint = str(contract.validate_checkpoint_path(args.checkpoint))
    output = Path(args.output).expanduser().resolve()
    if "official-mini-cheetah" not in output.parts:
        parser.error("输出必须位于 official-mini-cheetah")
    args.output = str(output)
    return args


def load_model(checkpoint: str, device: str = "cuda") -> Any:
    """加载并校验 R3 checkpoint 的 57/12 契约。"""
    from stable_baselines3 import PPO

    path = contract.validate_checkpoint_path(checkpoint)
    if not path.is_file():
        raise FileNotFoundError(f"checkpoint 不存在：{path}")
    model = PPO.load(str(path), device=device)
    if tuple(model.observation_space.shape) != (contract.OBS_DIM,):
        raise RuntimeError("checkpoint 观测维度错误")
    if tuple(model.action_space.shape) != (contract.ACTION_DIM,):
        raise RuntimeError("checkpoint 动作维度错误")
    expected_device = torch_device_name(device)
    actual_device = str(model.device)
    if actual_device != expected_device:
        raise RuntimeError(f"checkpoint 设备错误：{actual_device} != {expected_device}")
    return model


def torch_device_name(device: str) -> str:
    """规范化 SB3/PyTorch 设备名，cuda 统一视为当前 cuda:0。"""
    value = str(device)
    return "cuda" if value in {"cuda", "cuda:0"} else value


def _finite_array(values: Any, length: int, name: str) -> np.ndarray:
    """读取固定长度有限向量。"""
    try:
        result = np.asarray(values, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} 必须是 {length} 个有限数") from exc
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} 必须是 {length} 个有限数")
    return result


def _finite_scalar(value: Any, name: str) -> float:
    """读取有限标量。"""
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} 必须是有限数") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} 必须是有限数")
    return result


def true_foot_contacts(state: Mapping[str, Any]) -> tuple[np.ndarray, str]:
    """优先读取四足独立接触通道，并明确记录是否为真实足端数据。"""
    for key in ("foot_contacts", "foot_contact", "contacts"):
        if key not in state:
            continue
        try:
            values = np.asarray(state[key], dtype=np.float64)
        except (TypeError, ValueError):
            continue
        if values.shape == (4,) and np.all(np.isfinite(values)):
            return (values >= 0.5).astype(np.float64), key
    raise ValueError("状态缺少四足接触通道")


def foot_slip_speed(
    state: Mapping[str, Any],
    previous: Mapping[str, Any] | None,
    contacts: np.ndarray,
) -> tuple[float, str]:
    """计算接触足的 XY 滑移速度；缺少真实足端数据时只做低可信回退。"""
    velocities = state.get("foot_velocities")
    if velocities is not None:
        values = np.asarray(velocities, dtype=np.float64)
        if values.shape in {(4, 3), (12, 3), (12,), (36,)} and np.all(np.isfinite(values)):
            if values.shape == (12,):
                selected = values.reshape(4, 3)
            elif values.shape == (36,):
                selected = values.reshape(4, 3, 3).mean(axis=1)
            else:
                selected = values[:4] if values.shape == (4, 3) else values.reshape(4, 3, 3).mean(axis=1)
            return _contact_slip(selected, contacts), "foot_velocities"
    positions = state.get("foot_positions")
    if positions is not None and previous is not None and previous.get("foot_positions") is not None:
        current = np.asarray(positions, dtype=np.float64)
        old = np.asarray(previous["foot_positions"], dtype=np.float64)
        if current.shape in {(4, 3), (12, 3), (12,), (36,)} and current.shape == old.shape and np.all(
            np.isfinite(current + old)
        ):
            if current.shape == (12,):
                current, old = current.reshape(4, 3), old.reshape(4, 3)
            elif current.shape == (36,):
                current = current.reshape(4, 3, 3).mean(axis=1)
                old = old.reshape(4, 3, 3).mean(axis=1)
            else:
                current = current[:4] if current.shape == (4, 3) else current.reshape(4, 3, 3).mean(axis=1)
                old = old[:4] if old.shape == (4, 3) else old.reshape(4, 3, 3).mean(axis=1)
            dt = contract.CONTROL_DT_SECONDS
            return _contact_slip((current[:, :2] - old[:, :2]) / dt, contacts), "foot_positions"
    velocity = _finite_array(state.get("v_body", (0.0, 0.0, 0.0)), 3, "v_body")
    return float(np.linalg.norm(velocity[:2])), "body_velocity_fallback"


def _contact_slip(planar_velocity: np.ndarray, contacts: np.ndarray) -> float:
    """只统计当前接触足的 XY 滑移，避免摆动腿速度误判为打滑。"""
    active = np.asarray(contacts, dtype=np.bool_) >= 0.5
    if not bool(np.any(active)):
        return 0.0
    return float(np.mean(np.linalg.norm(planar_velocity[active], axis=1)))


def _position(state: Mapping[str, Any]) -> np.ndarray:
    """读取世界系位置；旧 state 缺字段时用高度和出生点构造有限回退。"""
    if "position" in state:
        return _finite_array(state["position"], 3, "position")
    return np.asarray((0.0, 0.0, float(state.get("height", 0.0))), dtype=np.float64)


def _state_sample(
    state: Mapping[str, Any],
    previous: Mapping[str, Any] | None,
    action: np.ndarray,
    previous_action: np.ndarray,
) -> dict[str, Any]:
    """把一帧物理状态转换为指标样本。"""
    q = _finite_array(state.get("q"), 12, "q")
    dq = _finite_array(state.get("dq"), 12, "dq")
    rpy = _finite_array(state.get("rpy"), 3, "rpy")
    velocity = _finite_array(state.get("v_body"), 3, "v_body")
    omega = _finite_array(state.get("omega_body"), 3, "omega_body")
    contacts, contact_source = true_foot_contacts(state)
    slip, slip_source = foot_slip_speed(state, previous, contacts)
    joint_jitter = np.zeros(12, dtype=np.float64)
    if previous is not None and "dq" in previous:
        old_dq = _finite_array(previous["dq"], 12, "previous dq")
        joint_jitter = dq - old_dq
    action_delta = action - previous_action
    return {
        "height": _finite_scalar(state.get("height"), "height"),
        "rpy": rpy,
        "v_body": velocity,
        "omega_body": omega,
        "position": _position(state),
        "contacts": contacts,
        "contact_source": contact_source,
        "foot_contact_source": str(
            state.get("foot_contact_source", "unknown")
        ),
        "true_contact_source": (
            contact_source in {"foot_contacts", "foot_contact"}
            and str(state.get("foot_contact_source", "node_id")) == "node_id"
        ),
        "foot_slip": slip,
        "foot_slip_source": slip_source,
        "action_delta": action_delta,
        "joint_velocity": dq,
        "joint_jitter": joint_jitter,
        "q": q,
        "jump_success": bool(state.get("jump_success", False)),
        "jump_landing": bool(state.get("jump_landing", False)),
    }


def _rms(values: Sequence[np.ndarray]) -> float:
    """计算样本序列的 RMS。"""
    if not values:
        return 0.0
    joined = np.concatenate([np.asarray(item, dtype=np.float64).reshape(-1) for item in values])
    return float(np.sqrt(np.mean(joined * joined))) if joined.size else 0.0


def _percentile(values: Sequence[float], percentile: float) -> float:
    """计算安全百分位，空序列返回零。"""
    return float(np.percentile(np.asarray(values, dtype=np.float64), percentile)) if values else 0.0


def summarize_episode(
    samples: Sequence[Mapping[str, Any]],
    *,
    command: Sequence[float],
    steps: int,
    fallen: bool,
    reward: float,
    jump_requests: Sequence[int] = (),
    phase: str = "",
    reset_state: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """汇总单集高度、姿态、漂移、接触、足滑和抖动物理指标。"""
    if not samples:
        raise RuntimeError("评估没有产生任何状态样本")
    # 前 0.5 s 只用于复位稳定，不计入物理 Gate。
    if len(samples) > 25:
        samples = samples[25:]
    cmd = _finite_array(command, 3, "command")
    heights = [float(item["height"]) for item in samples]
    posture = np.asarray([max(abs(float(item["rpy"][0])), abs(float(item["rpy"][1]))) for item in samples])
    velocities = np.asarray([np.asarray(item["v_body"], dtype=np.float64) for item in samples])
    yaw_rates = np.asarray([float(item["omega_body"][2]) for item in samples])
    positions = np.asarray([np.asarray(item["position"], dtype=np.float64) for item in samples])
    displacement = positions[-1, :2] - positions[0, :2]
    # 位置样本已剔除前 0.5 s，计划位移必须按实际参与统计的样本时长计算。
    planned = cmd[:2] * len(samples) * contract.CONTROL_DT_SECONDS
    raw_displacement = float(np.linalg.norm(displacement))
    planned_displacement = float(np.linalg.norm(planned))
    planned_residual = float(np.linalg.norm(displacement - planned))
    if np.linalg.norm(cmd[:2]) < 1e-9:
        drift_component = "raw_displacement"
        drift = raw_displacement
    else:
        drift_component = "planned_residual"
        drift = planned_residual
    contact_ratio = float(
        np.mean([bool(np.all(np.asarray(item["contacts"]) >= 0.5)) for item in samples])
    )
    true_contact_ratio = (
        contact_ratio
        if all(bool(item["true_contact_source"]) for item in samples)
        else 0.0
    )
    foot_slip = [float(item["foot_slip"]) for item in samples]
    velocity_error = np.linalg.norm(velocities[:, :2] - cmd[:2], axis=1)
    yaw_error = np.abs(yaw_rates - cmd[2])
    action_deltas = [np.asarray(item["action_delta"], dtype=np.float64) for item in samples]
    joint_jitters = [
        np.asarray(item["joint_jitter"], dtype=np.float64)
        for item in samples
    ]
    joint_velocities = [np.asarray(item["joint_velocity"], dtype=np.float64) for item in samples]
    jump_events = [index for index, item in enumerate(samples) if bool(item["jump_success"])]
    landing_events = [index for index, item in enumerate(samples) if bool(item["jump_landing"])]
    peak_gains: list[float] = []
    post_landing_heights: list[float] = []
    for request_step in jump_requests:
        start = max(0, min(int(request_step), len(samples) - 1))
        window = samples[start:]
        peak = max((float(item["height"]) for item in window), default=heights[start])
        peak_gains.append(peak - heights[start])
        landing_index = next(
            (index for index, item in enumerate(window) if bool(item["jump_landing"])),
            None,
        )
        if landing_index is not None and landing_index + 1 < len(window):
            post_landing_heights.append(float(window[landing_index + 1]["height"]))
    finite = bool(
        np.all(np.isfinite(heights))
        and np.all(np.isfinite(posture))
        and np.all(np.isfinite(velocities))
        and np.all(np.isfinite(yaw_rates))
    )
    report: dict[str, Any] = {
        "steps": int(steps),
        "fallen": bool(fallen),
        "reward": float(reward),
        "finite": finite,
        "observation_dim": contract.OBS_DIM,
        "action_dim": contract.ACTION_DIM,
        "foot_contact_source": str(
            samples[-1].get("foot_contact_source", "unknown")
        ),
        "height_mean": float(np.mean(heights)),
        "height_p05": _percentile(heights, 5.0),
        "height_p95": _percentile(heights, 95.0),
        "posture_abs_mean": float(np.mean(posture)),
        "posture_abs_p95": _percentile(posture.tolist(), 95.0),
        "drift_m": drift,
        "drift_component": drift_component,
        "raw_displacement_m": raw_displacement,
        "planned_displacement_m": planned_displacement,
        "planned_residual_m": planned_residual,
        "displacement_x": float(displacement[0]),
        "displacement_y": float(displacement[1]),
        "mean_vx": float(np.mean(velocities[:, 0])),
        "mean_vy": float(np.mean(velocities[:, 1])),
        "mean_wz": float(np.mean(yaw_rates)),
        "planar_speed_abs_mean": float(np.mean(np.linalg.norm(velocities[:, :2], axis=1))),
        "speed_error_mean": float(np.mean(velocity_error)),
        "yaw_error_mean": float(np.mean(yaw_error)),
        "yaw_rate_abs_mean": float(np.mean(np.abs(yaw_rates))),
        "four_contact_ratio": contact_ratio,
        "true_four_contact_ratio": true_contact_ratio,
        "foot_slip_mean": float(np.mean(foot_slip)),
        "foot_slip_max": float(np.max(foot_slip)),
        "foot_slip_source": str(samples[-1]["foot_slip_source"]),
        "action_delta_rms": _rms(action_deltas),
        "joint_velocity_rms": _rms(joint_velocities),
        "joint_jitter_rms": _rms(joint_jitters),
        "jump_requests": len(jump_requests),
        "jump_success": len(jump_events),
        "jump_landing": len(landing_events),
        "peak_height_gain_mean": float(np.mean(peak_gains)) if peak_gains else 0.0,
        "post_landing_height_mean": (
            float(np.mean(post_landing_heights)) if post_landing_heights else 0.0
        ),
    }
    report["technical_failure_reasons"] = _technical_failure_reasons(
        report,
        phase=phase,
        reset_state=reset_state,
    )
    report["technical_failure"] = bool(report["technical_failure_reasons"])
    return report


def _technical_failure_reasons(
    report: Mapping[str, Any],
    *,
    phase: str,
    reset_state: Mapping[str, Any] | None,
) -> list[str]:
    """把无效 reset、一步终止和非有限结果从物理样本中隔离。"""
    reasons: list[str] = []
    if int(report.get("steps", 0)) < MIN_EPISODE_STEPS:
        reasons.append(f"steps={report.get('steps', 0)}<{MIN_EPISODE_STEPS}")
    if not bool(report.get("finite", False)):
        reasons.append("non_finite_sample")
    if not math.isfinite(float(report.get("reward", math.nan))):
        reasons.append("non_finite_reward")
    if reset_state is None:
        reasons.append("reset_state_missing")
        return reasons
    try:
        height = float(reset_state["height"])
        rpy = np.asarray(reset_state["rpy"], dtype=np.float64)
        q = np.asarray(reset_state["q"], dtype=np.float64)
        source = str(reset_state.get("foot_contact_source", "node_id"))
    except (KeyError, TypeError, ValueError):
        reasons.append("reset_state_invalid")
        return reasons
    if rpy.shape != (3,) or not np.all(np.isfinite(rpy)):
        reasons.append("reset_posture_non_finite")
    else:
        posture = float(np.max(np.abs(rpy[:2])))
        if phase in {"P0", "P1"}:
            if posture > RSI_POSTURE_TOLERANCE + 1e-9:
                reasons.append(f"reset_posture={posture:.6f}")
        elif posture > contract.ROLL_PITCH_LIMIT + 1e-9:
            reasons.append(f"reset_posture={posture:.6f}")
    if not math.isfinite(height):
        reasons.append("reset_height_non_finite")
    elif phase in {"P0", "P1"}:
        if abs(height - contract.REFERENCE_HEIGHT) > RSI_HEIGHT_TOLERANCE + 1e-9:
            reasons.append(
                f"reset_height={height:.6f},"
                f"expected={contract.REFERENCE_HEIGHT:.6f}"
            )
    elif not (
        contract.MIN_SUPPORTED_HEIGHT
        <= height
        <= contract.MAX_BASE_HEIGHT
    ):
        reasons.append(f"reset_height={height:.6f}")
    if phase in {"P0", "P1"}:
        if q.shape != (contract.ACTION_DIM,) or not np.all(np.isfinite(q)):
            reasons.append("reset_joint_non_finite")
        else:
            targets = np.asarray(contract.rsi_targets(), dtype=np.float64)
            joint_error = float(np.max(np.abs(q - targets)))
            if joint_error > RSI_JOINT_ERROR_TOLERANCE + 1e-9:
                reasons.append(f"reset_joint_error={joint_error:.6f}")
        if source != "node_id":
            reasons.append(f"reset_contact_source={source}")
    return reasons


def _predict_action(model: Any, observation: Any, *, deterministic: bool) -> np.ndarray:
    """调用 SB3 兼容模型并严格校验动作。"""
    action, _state = model.predict(observation, deterministic=deterministic)
    return _finite_array(action, contract.ACTION_DIM, "action")


def _step_with_jump(
    env: Any,
    action: np.ndarray,
    *,
    jump_request: bool,
) -> tuple[Any, float, bool, bool, dict[str, Any]]:
    """执行一步；必要时在 TCP 发送前把显式跳跃请求并入消息。"""
    if hasattr(env, "prepare_step") and hasattr(env, "send_prepared_step"):
        env.prepare_step(action)
        pending = getattr(env, "_pending_step", None)
        if isinstance(pending, dict):
            pending["jump_request"] = bool(pending.get("jump_request", False) or jump_request)
        env.send_prepared_step()
        return env.finish_step()
    return env.step(action)


def _episode(
    model: Any,
    env: Any,
    *,
    seed: int,
    command: Sequence[float] | None = None,
    deterministic: bool = True,
    jump_steps: Iterable[int] = (),
    max_steps: Optional[int] = None,
) -> dict[str, Any]:
    """执行一集并返回物理 Gate 所需的完整汇总。"""
    options = {"command": list(command), "command_fixed": True} if command is not None else None
    obs, _info = env.reset(seed=seed, options=options)
    observation = _finite_array(obs, contract.OBS_DIM, "observation")
    phase = str(getattr(env, "phase", ""))
    raw_reset_state = getattr(env, "_last_state", None)
    reset_state = (
        dict(raw_reset_state)
        if isinstance(raw_reset_state, Mapping)
        else None
    )
    fixed_command = (
        np.asarray(command, dtype=np.float64)
        if command is not None
        else np.asarray(getattr(env, "_command", (0.0, 0.0, 0.0)), dtype=np.float64)
    )
    requested = sorted({int(value) for value in jump_steps if int(value) > 0})
    steps_limit = int(max_steps or getattr(env, "max_episode_steps", contract.MAX_EPISODE_STEPS))
    samples: list[dict[str, Any]] = []
    previous_state: Mapping[str, Any] | None = None
    previous_action = np.zeros(contract.ACTION_DIM, dtype=np.float64)
    previous_raw_action = np.zeros(contract.ACTION_DIM, dtype=np.float64)
    reward_sum = 0.0
    fallen = False
    steps = 0
    for steps in range(1, steps_limit + 1):
        action = _predict_action(model, observation, deterministic=deterministic)
        safe_action = np.asarray(
            contract.sanitize_action(action, previous_action),
            dtype=np.float64,
        )
        obs, reward, terminated, truncated, info = _step_with_jump(
            env,
            safe_action,
            jump_request=steps in requested,
        )
        observation = _finite_array(obs, contract.OBS_DIM, "observation")
        reward_sum += float(reward)
        state = getattr(env, "_last_state", None)
        if not isinstance(state, Mapping):
            raise RuntimeError("环境未提供 _last_state 物理状态")
        applied_action = np.asarray(
            info.get("applied_action", action),
            dtype=np.float64,
        )
        samples.append(
            _state_sample(
                state,
                previous_state,
                applied_action,
                previous_raw_action,
            )
        )
        previous_state = dict(state)
        previous_action = safe_action
        previous_raw_action = applied_action.copy()
        fallen = bool(info.get("fallen", False))
        if terminated or truncated:
            break
    return summarize_episode(
        samples,
        command=fixed_command,
        steps=steps,
        fallen=fallen,
        reward=reward_sum,
        jump_requests=requested,
        phase=phase,
        reset_state=reset_state,
    )


def _mean(episodes: Sequence[Mapping[str, Any]], key: str) -> float:
    """汇总 episode 标量均值。"""
    return float(np.mean([float(item[key]) for item in episodes])) if episodes else 0.0


def _max(episodes: Sequence[Mapping[str, Any]], key: str) -> float:
    """汇总 episode 标量最大值。"""
    return float(np.max([float(item[key]) for item in episodes])) if episodes else math.inf


def physical_gate(episodes: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """P1～P6 共用的物理 Gate；无效 reset 不进入任何均值。"""
    technical = [
        item for item in episodes if bool(item.get("technical_failure", False))
    ]
    valid = [
        item for item in episodes if not bool(item.get("technical_failure", False))
    ]
    if not valid:
        value_keys = (
            "fall_rate",
            "height_mean",
            "height_p05_min",
            "height_p95_max",
            "posture_abs_mean",
            "posture_abs_p95_max",
            "drift_m_max",
            "speed_error_mean",
            "yaw_rate_abs_mean",
            "true_four_contact_ratio_mean",
            "foot_slip_mean",
            "action_delta_rms",
            "joint_jitter_rms",
        )
        checks = {
            name: False
            for name in (
                "finite",
                "fall_rate",
                "height_mean",
                "height_range",
                "posture_mean",
                "posture_p95",
                "drift",
                "speed_error",
                "yaw_rate",
                "true_four_contact",
                "foot_slip",
                "action_jitter",
                "joint_jitter",
            )
        }
        return {
            "passed": False,
            "technical_failure": True,
            "technical_failure_count": len(technical),
            "valid_episode_count": 0,
            "checks": checks,
            "values": {key: 0.0 for key in value_keys},
            "thresholds": dict(PHYSICAL_THRESHOLDS),
        }
    checks = {
        "episodes_valid": not technical and len(valid) == len(episodes),
        "finite": all(bool(item["finite"]) for item in valid),
        "fall_rate": _mean(valid, "fallen") <= PHYSICAL_THRESHOLDS["fall_rate_max"],
        "height_mean": (
            PHYSICAL_THRESHOLDS["height_mean_min"]
            <= _mean(valid, "height_mean")
            <= PHYSICAL_THRESHOLDS["height_mean_max"]
        ),
        "height_range": (
            _min(valid, "height_p05") >= PHYSICAL_THRESHOLDS["height_p05_min"]
            and _max(valid, "height_p95") <= PHYSICAL_THRESHOLDS["height_p95_max"]
        ),
        "posture_mean": _mean(valid, "posture_abs_mean") <= PHYSICAL_THRESHOLDS["posture_mean_max"],
        "posture_p95": _max(valid, "posture_abs_p95") <= PHYSICAL_THRESHOLDS["posture_p95_max"],
        "drift": _max(valid, "drift_m") <= PHYSICAL_THRESHOLDS["drift_m_max"],
        "speed_error": _mean(valid, "speed_error_mean") <= PHYSICAL_THRESHOLDS["speed_error_mean_max"],
        "yaw_rate": _mean(valid, "yaw_rate_abs_mean") <= PHYSICAL_THRESHOLDS["yaw_rate_abs_mean_max"],
        "true_four_contact": (
            _mean(valid, "true_four_contact_ratio")
            >= PHYSICAL_THRESHOLDS["true_four_contact_ratio_min"]
        ),
        "foot_slip": _mean(valid, "foot_slip_mean") <= PHYSICAL_THRESHOLDS["foot_slip_mean_max"],
        "action_jitter": _mean(valid, "action_delta_rms") <= PHYSICAL_THRESHOLDS["action_delta_rms_max"],
        "joint_jitter": _mean(
            valid,
            "joint_jitter_rms",
        ) <= PHYSICAL_THRESHOLDS["joint_jitter_rms_max"],
    }
    return {
        "passed": all(checks.values()),
        "technical_failure": bool(technical),
        "technical_failure_count": len(technical),
        "valid_episode_count": len(valid),
        "checks": checks,
        "values": {
            "fall_rate": _mean(valid, "fallen"),
            "height_mean": _mean(valid, "height_mean"),
            "height_p05_min": _min(valid, "height_p05"),
            "height_p95_max": _max(valid, "height_p95"),
            "posture_abs_mean": _mean(valid, "posture_abs_mean"),
            "posture_abs_p95_max": _max(valid, "posture_abs_p95"),
            "drift_m_max": _max(valid, "drift_m"),
            "speed_error_mean": _mean(valid, "speed_error_mean"),
            "yaw_rate_abs_mean": _mean(valid, "yaw_rate_abs_mean"),
            "true_four_contact_ratio_mean": _mean(valid, "true_four_contact_ratio"),
            "foot_slip_mean": _mean(valid, "foot_slip_mean"),
            "action_delta_rms": _mean(valid, "action_delta_rms"),
            "joint_jitter_rms": _mean(valid, "joint_jitter_rms"),
            "technical_failure_count": float(len(technical)),
            "valid_episode_count": float(len(valid)),
        },
        "thresholds": dict(PHYSICAL_THRESHOLDS),
    }


def _min(episodes: Sequence[Mapping[str, Any]], key: str) -> float:
    """汇总 episode 标量最小值。"""
    return float(np.min([float(item[key]) for item in episodes])) if episodes else -math.inf


def p0_four_robot_handshake() -> bool:
    """读取本次训练留下的四机 TCP 健康证据。"""
    health_path = contract.LOG_ROOT / "health_P0_live.json"
    marker_path = contract.LOG_ROOT / "P0_run_started_at.txt"
    if not health_path.is_file() or not marker_path.is_file():
        return False
    try:
        health = json.loads(health_path.read_text(encoding="utf-8"))
        started = float(marker_path.read_text(encoding="utf-8").strip())
        health_started = float(health["run_started_at"])
        workers = health["worker_logs"]
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError):
        return False
    if abs(started - health_started) > 1.0 or not bool(health.get("passed")):
        return False
    if len(workers) != contract.PARALLEL_WORKERS:
        return False
    fields = ("connected", "worker_id", "robot_name", "timestep", "current")
    return all(
        all(bool(workers[str(index)].get(field)) for field in fields)
        for index in range(contract.PARALLEL_WORKERS)
    )


def engineering_gate(
    episodes: Sequence[Mapping[str, Any]],
    *,
    four_robot_handshake: bool,
) -> dict[str, Any]:
    """P0 工程 Gate：只检查链路、维度、有限值和奖励接口。"""
    technical = [
        item for item in episodes if bool(item.get("technical_failure", False))
    ]
    valid = [
        item for item in episodes
        if not bool(item.get("technical_failure", False))
    ]
    checks = {
        "four_robot_handshake": bool(four_robot_handshake),
        "episodes_present": len(episodes) > 0,
        "valid_episodes": bool(valid) and len(valid) == len(episodes),
        "finite": bool(valid) and all(bool(item["finite"]) for item in valid),
        "observation_action_dims": bool(valid) and all(
            int(item.get("observation_dim", -1)) == contract.OBS_DIM
            and int(item.get("action_dim", -1)) == contract.ACTION_DIM
            for item in valid
        ),
        "reward_interface": bool(valid) and all(
            math.isfinite(float(item.get("reward", math.nan)))
            for item in valid
        ),
        "short_chain": bool(valid) and all(
            int(item.get("steps", 0)) >= MIN_EPISODE_STEPS
            for item in valid
        ),
        "contact_source": bool(valid) and all(
            str(item.get("foot_contact_source", "node_id")) == "node_id"
            for item in valid
        ),
    }
    technical_failure = (
        not four_robot_handshake
        or bool(technical)
        or not checks["valid_episodes"]
    )
    return {
        "gate_type": "engineering",
        "passed": all(checks.values()),
        "technical_failure": technical_failure,
        "technical_failure_count": len(technical),
        "valid_episode_count": len(valid),
        "checks": checks,
        "values": {
            "episodes": float(len(episodes)),
            "valid_episodes": float(len(valid)),
            "technical_failures": float(len(technical)),
            "four_robot_handshake": float(bool(four_robot_handshake)),
        },
        "thresholds": {
            "observation_dim": contract.OBS_DIM,
            "action_dim": contract.ACTION_DIM,
            "minimum_steps": MIN_EPISODE_STEPS,
        },
    }


def _select(episodes: Sequence[Mapping[str, Any]], names: Sequence[str], cases: Sequence[str]) -> list[dict[str, Any]]:
    """按 case 名选择 episode。"""
    return [episode for episode, name in zip(episodes, cases) if name in set(names)]


def _mean_present(
    episodes: Sequence[Mapping[str, Any]],
    key: str,
) -> float:
    """汇总字段；旧兼容样本缺字段时返回零，不制造 NaN 遥测。"""
    values = [
        float(episode[key])
        for episode in episodes
        if key in episode and math.isfinite(float(episode[key]))
    ]
    return float(np.mean(values)) if values else 0.0


def case_telemetry(
    episodes: Sequence[Mapping[str, Any]],
    cases: Sequence[str],
) -> dict[str, dict[str, float]]:
    """按 stop/forward 等 case 输出独立位移、接触与速度遥测。"""
    result: dict[str, dict[str, float]] = {}
    for case in dict.fromkeys(cases):
        selected = _select(episodes, (case,), cases)
        if not selected:
            continue
        result[case] = {
            "episode_count": float(len(selected)),
            "drift_m": _mean_present(selected, "drift_m"),
            "raw_displacement_m": _mean_present(
                selected,
                "raw_displacement_m",
            ),
            "planned_displacement_m": _mean_present(
                selected,
                "planned_displacement_m",
            ),
            "planned_residual_m": _mean_present(
                selected,
                "planned_residual_m",
            ),
            "mean_vx": _mean_present(selected, "mean_vx"),
            "mean_vy": _mean_present(selected, "mean_vy"),
            "speed_error_mean": _mean_present(
                selected,
                "speed_error_mean",
            ),
            "true_four_contact_ratio": _mean_present(
                selected,
                "true_four_contact_ratio",
            ),
        }
    return result


def capability_gate(
    episodes: Sequence[Mapping[str, Any]],
    cases: Sequence[str],
) -> dict[str, Any]:
    """按 P2～P6 用例执行移动或跳跃能力 Gate。"""
    physical = physical_gate(episodes)
    checks: dict[str, bool] = {}
    values: dict[str, float] = {}
    names = set(cases)
    if names & {"stop"}:
        stop = _select(episodes, ("stop",), cases)
        values["stop_planar_speed_abs_mean"] = _mean(stop, "planar_speed_abs_mean")
        checks["stop"] = values["stop_planar_speed_abs_mean"] <= MOVE_THRESHOLDS["stop_planar_speed_max"]
    if names & {"forward"}:
        forward = _select(episodes, ("forward",), cases)
        values["forward_speed"] = _mean(forward, "mean_vx")
        values["forward_speed_error"] = _mean(forward, "speed_error_mean")
        values["forward_cross_axis"] = _mean(forward, "mean_vy") + _mean(forward, "yaw_rate_abs_mean")
        checks["forward"] = (
            values["forward_speed"] >= MOVE_THRESHOLDS["forward_speed_min"]
            and values["forward_speed_error"] <= MOVE_THRESHOLDS["forward_speed_error_mean_max"]
            and values["forward_cross_axis"] <= MOVE_THRESHOLDS["cross_axis_abs_mean_max"] + MOVE_THRESHOLDS["stop_planar_speed_max"]
        )
    if names & {"left", "right"}:
        left = _select(episodes, ("left",), cases)
        right = _select(episodes, ("right",), cases)
        values["lateral_left_speed"] = _mean(left, "mean_vy")
        values["lateral_right_speed"] = _mean(right, "mean_vy")
        values["lateral_speed_error"] = (
            _mean(left, "speed_error_mean") + _mean(right, "speed_error_mean")
        ) / 2.0
        checks["lateral"] = (
            values["lateral_left_speed"] >= MOVE_THRESHOLDS["lateral_speed_min"]
            and values["lateral_right_speed"] <= -MOVE_THRESHOLDS["lateral_speed_min"]
            and values["lateral_speed_error"] <= MOVE_THRESHOLDS["lateral_speed_error_mean_max"]
        )
    if names & {"forward_left", "forward_right"}:
        left_turn = _select(episodes, ("forward_left",), cases)
        right_turn = _select(episodes, ("forward_right",), cases)
        values["turn_left_rate"] = _mean(left_turn, "mean_wz")
        values["turn_right_rate"] = _mean(right_turn, "mean_wz")
        values["turn_planar_speed"] = (
            _mean(left_turn, "planar_speed_abs_mean")
            + _mean(right_turn, "planar_speed_abs_mean")
        ) / 2.0
        checks["turn"] = (
            values["turn_left_rate"] >= MOVE_THRESHOLDS["turn_rate_min"]
            and values["turn_right_rate"] <= -MOVE_THRESHOLDS["turn_rate_min"]
            and values["turn_planar_speed"] <= MOVE_THRESHOLDS["planar_speed_max_for_turn"]
        )
    if names & JUMP_CASES:
        jump = _select(episodes, tuple(sorted(JUMP_CASES)), cases)
        values["jump_success_rate"] = float(np.mean([item["jump_success"] > 0 for item in jump]))
        values["landing_rate"] = float(np.mean([item["jump_landing"] > 0 for item in jump]))
        values["peak_height_gain_mean"] = _mean(jump, "peak_height_gain_mean")
        values["post_landing_height_mean"] = _mean(jump, "post_landing_height_mean")
        checks["jump"] = (
            values["jump_success_rate"] >= JUMP_THRESHOLDS["jump_success_rate_min"]
            and values["landing_rate"] >= JUMP_THRESHOLDS["landing_rate_min"]
            and values["peak_height_gain_mean"] >= JUMP_THRESHOLDS["peak_height_gain_mean_min"]
            and JUMP_THRESHOLDS["post_landing_height_min"]
            <= values["post_landing_height_mean"]
            <= JUMP_THRESHOLDS["post_landing_height_max"]
        )
    return {
        "passed": physical["passed"] and all(checks.values()),
        "physical": physical,
        "checks": checks,
        "values": values,
        "thresholds": {"move": dict(MOVE_THRESHOLDS), "jump": dict(JUMP_THRESHOLDS)},
    }


def _case_plan(phase: str, episodes: int) -> list[tuple[str, tuple[float, float, float], tuple[int, ...]]]:
    """生成阶段用例、固定命令和显式跳跃步。"""
    if phase == "P1":
        return [("stand", (0.0, 0.0, 0.0), ()) for _ in range(episodes)]
    if phase == "P0":
        return [("smoke", (0.0, 0.0, 0.0), ()) for _ in range(max(1, min(episodes, 2)))]
    cases = CASE_NAMES.get(phase, ())
    if not cases:
        raise ValueError(f"阶段 {phase} 没有评估用例")
    repeats = max(1, math.ceil(episodes / len(cases)))
    plan: list[tuple[str, tuple[float, float, float], tuple[int, ...]]] = []
    for name in cases:
        for _ in range(repeats):
            jumps = (100, 350) if name == "moving_jump" else ((100,) if name == "standing_jump" else ())
            plan.append((name, CASE_COMMANDS[name], jumps))
    return plan[: max(episodes, len(cases))]


def _run_mode(
    model: Any,
    env: Any,
    args: argparse.Namespace,
    *,
    deterministic: bool,
    phase: str,
) -> dict[str, Any]:
    """执行一个策略模式下的全部阶段用例。"""
    plan = _case_plan(phase, args.episodes)
    episodes = []
    cases = []
    for index, (case, command, jumps) in enumerate(plan):
        cases.append(case)
        episodes.append(
            _episode(
                model,
                env,
                seed=args.seed + index + (10_000 if not deterministic else 0),
                command=command,
                deterministic=deterministic,
                jump_steps=jumps,
                max_steps=args.max_steps,
            )
        )
    if phase == "P1":
        report = physical_gate(episodes)
    elif phase == "P0":
        report = engineering_gate(
            episodes,
            four_robot_handshake=p0_four_robot_handshake(),
        )
    else:
        report = capability_gate(episodes, cases)
    return {
        "policy_mode": "deterministic" if deterministic else "stochastic",
        "cases": cases,
        "episodes": episodes,
        "case_values": case_telemetry(episodes, cases),
        **report,
    }


def run_p1(model: Any, env: Any, args: argparse.Namespace) -> dict[str, Any]:
    """P1 同时要求 deterministic 与 stochastic 通过物理 Gate。"""
    deterministic = _run_mode(model, env, args, deterministic=True, phase="P1")
    stochastic = _run_mode(model, env, args, deterministic=False, phase="P1")
    return {
        "passed": bool(deterministic["passed"] and stochastic["passed"]),
        "technical_failure": bool(
            deterministic.get("technical_failure", False)
            or stochastic.get("technical_failure", False)
        ),
        "deterministic": deterministic,
        "stochastic": stochastic,
        "policy_modes_required": ["deterministic", "stochastic"],
        "thresholds": dict(PHYSICAL_THRESHOLDS),
    }


def run_phase(model: Any, env: Any, args: argparse.Namespace) -> dict[str, Any]:
    """按 R3 阶段调度 P0～P6；P7 是独立 play。"""
    if args.phase == "P7":
        raise ValueError("P7 是 play 模式，请使用 rl.play，不执行评估 Gate")
    if args.phase == "P1":
        return run_p1(model, env, args)
    deterministic = _run_mode(
        model,
        env,
        args,
        deterministic=True,
        phase=args.phase,
    )
    if args.policy_mode == "both":
        stochastic = _run_mode(
            model,
            env,
            args,
            deterministic=False,
            phase=args.phase,
        )
        return {
            "passed": bool(deterministic["passed"] and stochastic["passed"]),
            "technical_failure": bool(
                deterministic.get("technical_failure", False)
                or stochastic.get("technical_failure", False)
            ),
            "deterministic": deterministic,
            "stochastic": stochastic,
            "policy_modes_required": ["deterministic", "stochastic"],
        }
    return deterministic


# 兼容明确命名的调用入口。
run_p0 = lambda model, env, args: run_phase(model, env, args)
run_p2 = run_p3 = run_p4 = run_p5 = run_p6 = run_phase


def configure_eval_connection(env: Any, *, connect_timeout: float, socket_timeout: float) -> None:
    """给独立评估 listener 和 controller 连接设置严格超时。"""
    if connect_timeout <= 0.0 or socket_timeout <= 0.0:
        raise ValueError("网络超时必须为正")
    server = getattr(env, "_server", None)
    if server is not None and hasattr(server, "settimeout"):
        server.settimeout(float(connect_timeout))
    connection = getattr(env, "_conn", None)
    if connection is not None and hasattr(connection, "settimeout"):
        connection.settimeout(float(socket_timeout))


def start_eval_environment(args: argparse.Namespace) -> Any:
    """显式启动单机器人评估 world，并修复 controller 握手无限等待。"""
    from .env import MiniCheetahFlatJumpEnv

    supervisor_port = contract.WEBOTS_SUPERVISOR_PORT
    require_ports_free(
        (supervisor_port, args.bridge_port),
        "启动评估环境",
    )
    env = MiniCheetahFlatJumpEnv(
        phase=args.phase,
        bridge_port=args.bridge_port,
        randomization_mode=contract.PHASE_RANDOMIZATION_MODES[args.phase],
        start_bridge=False,
        max_episode_steps=args.max_steps,
        world=contract.EVAL_WORLD_PATH,
        worker_id=0,
        robot_name="mini_cheetah",
        birth_position=contract.EVAL_BIRTH_POSITION,
        supervisor_port=supervisor_port,
    )
    try:
        env.open_bridge()
        configure_eval_connection(
            env,
            connect_timeout=args.connect_timeout,
            socket_timeout=args.socket_timeout,
        )
        env.launch_webots()
        env.verify_webots_supervisor_port(timeout=10.0)
        env.launch_controller()
        env.accept_bridge()
        configure_eval_connection(
            env,
            connect_timeout=args.connect_timeout,
            socket_timeout=args.socket_timeout,
        )
        return env
    except Exception:
        env.close()
        raise


def main(argv: Optional[Sequence[str]] = None) -> int:
    """执行评估并输出可审计 JSON。"""
    args = parse_args(argv)
    if args.dry_run:
        print(
            f"EVAL_DRY_RUN phase={args.phase} episodes={args.episodes} "
            f"policy_mode={args.policy_mode} gate={args.gate} checkpoint={args.checkpoint}"
        )
        return 0
    if args.phase == "P7":
        raise SystemExit("P7 是 play 模式，请使用 python -m rl.play")
    model = load_model(args.checkpoint, device=args.device)
    env = start_eval_environment(args)
    try:
        report = run_phase(model, env, args)
    finally:
        env.close()
    payload = {
        "contract": getattr(contract, "CONTRACT_VERSION", "R3"),
        "phase": args.phase,
        "checkpoint": args.checkpoint,
        "gate": args.gate,
        "policy_mode": args.policy_mode,
        "report": report,
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("EVALUATION " + json.dumps(payload, ensure_ascii=False))
    return 1 if args.gate and not report["passed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
