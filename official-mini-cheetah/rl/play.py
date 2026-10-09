#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""R3 checkpoint 的遥控 play 推理管线与独立安全停止。"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence

import numpy as np

from . import contract
from .rapid_policy import RAPID_MODEL_DIR, RapidPolicyAdapter


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONTROLLER_ROOT = PROJECT_ROOT / "controllers" / "rl_agent"
for _path in (str(CONTROLLER_ROOT),):
    if _path not in sys.path:
        sys.path.insert(0, _path)

try:
    from input_adapter import InputAdapter, InputFrame
except ImportError as exc:  # pragma: no cover - Webots 环境缺失时由测试补充路径
    raise RuntimeError("缺少 controllers/rl_agent/input_adapter.py") from exc


def _finite_vector(values: Any, length: int, name: str) -> np.ndarray:
    """读取并校验固定长度有限向量。"""
    try:
        result = np.asarray(values, dtype=np.float32)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} 必须是 {length} 个有限数") from exc
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} 必须是 {length} 个有限数")
    return result


def _contacts(state: Mapping[str, Any]) -> np.ndarray:
    """优先取真实四足接触，再兼容四路 contacts。"""
    for key in ("foot_contacts", "foot_contact", "contacts"):
        if key not in state:
            continue
        try:
            values = np.asarray(state[key], dtype=np.float32)
        except (TypeError, ValueError):
            continue
        if values.shape == (4,) and np.all(np.isfinite(values)):
            return (values >= 0.5).astype(np.float32)
    return np.zeros(4, dtype=np.float32)


class ObservationBuilder:
    """把 controller state、命令和上一动作拼成契约 57 维观测。"""

    def build(
        self,
        state: Mapping[str, Any],
        *,
        command: Sequence[float],
        previous_action: Sequence[float],
        jump_request: bool = False,
    ) -> np.ndarray:
        """严格按 `contract.OBS_SLICES` 拼装观测。"""
        q = _finite_vector(state.get("q"), contract.ACTION_DIM, "q")
        dq = _finite_vector(state.get("dq"), contract.ACTION_DIM, "dq")
        rpy = _finite_vector(state.get("rpy"), 3, "rpy")
        velocity = _finite_vector(state.get("v_body"), 3, "v_body")
        omega = _finite_vector(state.get("omega_body"), 3, "omega_body")
        previous = _finite_vector(previous_action, contract.ACTION_DIM, "previous_action")
        cmd = np.asarray(contract.clip_command(command), dtype=np.float32)
        height_value = float(state.get("height", 0.0))
        if not math.isfinite(height_value):
            raise ValueError("height 必须是有限数")
        jump_phase_value = float(state.get("jump_phase", 0.0))
        if not math.isfinite(jump_phase_value):
            raise ValueError("jump_phase 必须是有限数")
        contacts = _contacts(state)
        values = np.zeros(contract.OBS_DIM, dtype=np.float32)
        values[contract.OBS_SLICES["q"]] = q
        values[contract.OBS_SLICES["dq"]] = dq
        values[contract.OBS_SLICES["rpy"]] = rpy
        values[contract.OBS_SLICES["v_body"]] = velocity
        values[contract.OBS_SLICES["prev_action"]] = previous
        values[contract.OBS_SLICES["omega_body"]] = omega
        values[contract.OBS_SLICES["cmd"]] = cmd
        values[contract.OBS_SLICES["jump_request"]] = 1.0 if jump_request else 0.0
        values[contract.OBS_SLICES["jump_phase"]] = jump_phase_value
        values[contract.OBS_SLICES["height"]] = height_value
        values[contract.OBS_SLICES["contacts"]] = contacts
        values[contract.OBS_SLICES["terrain_height"]] = 0.0
        values[contract.OBS_SLICES["flat_flag"]] = 1.0
        if values.shape != (contract.OBS_DIM,) or not np.all(np.isfinite(values)):
            raise ValueError("57 维观测包含非法值")
        return values


class EmergencyStop:
    """独立急停锁存；不会被超时、零命令或跳跃自动解除。"""

    def __init__(self) -> None:
        self.active = False
        self.reason = ""

    def activate(self, reason: str) -> None:
        """永久激活急停，直到显式 `release()`。"""
        self.active = True
        self.reason = str(reason or "manual_estop")

    def release(self) -> None:
        """仅由用户明确复位流程调用；不作为普通输入超时的副作用。"""
        self.active = False
        self.reason = ""


@dataclass(frozen=True)
class PlayStepResult:
    """一次 play 推理输出。"""

    observation: np.ndarray
    action: np.ndarray
    joint_targets: tuple[float, ...]
    command: tuple[float, float, float]
    safe_stand: bool
    estop: bool
    reason: str
    model_info: Mapping[str, Any] = field(default_factory=dict)
    jump_requested: bool = False
    jump_rejected: bool = False


class PlayPolicy:
    """命令→57 维观测→deterministic PPO→12 关节目标。"""

    def __init__(
        self,
        model: Any,
        *,
        model_type: str = "sb3",
        target_rate_limit: float | str | None = None,
        emergency_stop: Optional[EmergencyStop] = None,
        builder: Optional[ObservationBuilder] = None,
    ) -> None:
        if model_type not in {"sb3", "rapid"}:
            raise ValueError("model_type 必须是 sb3 或 rapid")
        self.model = model
        self.model_type = model_type
        self.target_rate_limit = contract.resolve_action_target_rate_limit(
            target_rate_limit
        )
        self.emergency_stop = emergency_stop or EmergencyStop()
        self.builder = builder or ObservationBuilder()
        self.previous_action = np.zeros(contract.ACTION_DIM, dtype=np.float32)
        self.previous_targets = tuple(float(value) for value in contract.DEFAULT_CROUCH)
        self.last_model_info: dict[str, Any] = {}

    def safe_stand_result(
        self,
        state: Mapping[str, Any],
        *,
        reason: str,
        command: Sequence[float] = (0.0, 0.0, 0.0),
        jump_request: bool = False,
        model_info: Optional[Mapping[str, Any]] = None,
    ) -> PlayStepResult:
        """输出默认站姿，不调用策略网络。"""
        observation = self.builder.build(
            state,
            command=command,
            previous_action=self.previous_action,
            jump_request=jump_request,
        )
        targets = tuple(float(value) for value in contract.DEFAULT_CROUCH)
        self.previous_targets = targets
        self.previous_action = np.zeros(contract.ACTION_DIM, dtype=np.float32)
        info = dict(model_info or self.last_model_info)
        info.update(
            {
                "model_called": False,
                "raw_source_action": [0.0] * contract.ACTION_DIM,
                "mapped_yobo_action": [0.0] * contract.ACTION_DIM,
                "latent": None,
            }
        )
        jump_rejected = jump_request and self.model_type == "rapid"
        if jump_rejected:
            info.update(
                {
                    "jump_request_supported": False,
                    "jump_request_rejected": True,
                    "unsupported_requests": ["jump"],
                }
            )
        return PlayStepResult(
            observation=observation,
            action=np.zeros(contract.ACTION_DIM, dtype=np.float32),
            joint_targets=targets,
            command=(0.0, 0.0, 0.0),
            safe_stand=True,
            estop=self.emergency_stop.active,
            reason=reason,
            model_info=info,
            jump_requested=jump_request,
            jump_rejected=jump_rejected,
        )

    def step(
        self,
        state: Mapping[str, Any],
        *,
        command: Sequence[float],
        jump_request: bool = False,
        input_frame: Optional[InputFrame] = None,
        disconnected: bool = False,
    ) -> PlayStepResult:
        """执行一帧安全 play 推理。"""
        if disconnected:
            self.emergency_stop.activate("input_disconnected")
        if input_frame is not None and input_frame.reason == "escape":
            self.emergency_stop.activate("escape")
        if (
            input_frame is not None
            and input_frame.reason == "reset"
            and self.emergency_stop.active
        ):
            # R 是急停后唯一显式恢复路径；恢复后仍先保持安全站立。
            self.emergency_stop.release()
            return self.safe_stand_result(
                state,
                reason="reset_release",
                command=command,
                jump_request=False,
            )
        if self.emergency_stop.active:
            return self.safe_stand_result(
                state,
                reason=f"estop:{self.emergency_stop.reason}",
                command=command,
                jump_request=False,
            )
        if input_frame is not None and input_frame.safe:
            # 超时/普通 reset 只进入安全站立，不把独立急停锁死。
            return self.safe_stand_result(
                state,
                reason=input_frame.reason or "input_safe",
                command=command,
                jump_request=False,
            )
        requested_command = tuple(float(value) for value in command)
        if (
            self.model_type == "rapid"
            and jump_request
            and all(abs(value) <= 1e-9 for value in requested_command)
        ):
            # Rapid 冻结模型不支持跳跃，零命令也不得借跳跃边沿接管执行。
            return self.safe_stand_result(
                state,
                reason="jump_rejected_unsupported",
                command=requested_command,
                jump_request=True,
                model_info={
                    "jump_request_supported": False,
                    "jump_request_rejected": True,
                    "unsupported_requests": ["jump"],
                },
            )
        if not jump_request and all(abs(value) <= 1e-9 for value in requested_command):
            return self.safe_stand_result(state, reason="zero_command")
        try:
            observation = self.builder.build(
                state,
                command=requested_command,
                previous_action=self.previous_action,
                jump_request=jump_request,
            )
            if self.model_type == "rapid":
                raw, model_info = self.model.predict(
                    observation,
                    deterministic=True,
                    jump_request=jump_request,
                )
            else:
                raw, model_info = self.model.predict(
                    observation,
                    deterministic=True,
                )
            action = np.asarray(
                contract.sanitize_action(raw, self.previous_action),
                dtype=np.float32,
            )
            raw_targets = contract.action_to_target(action)
            targets = self._limit_targets(raw_targets)
        except Exception as exc:
            self.emergency_stop.activate(f"policy_error:{type(exc).__name__}")
            return self.safe_stand_result(
                state,
                reason=self.emergency_stop.reason,
                command=requested_command,
            )
        self.previous_action = action.copy()
        self.previous_targets = targets
        self.last_model_info = dict(model_info or {})
        self.last_model_info["model_called"] = True
        return PlayStepResult(
            observation=observation,
            action=action,
            joint_targets=targets,
            command=requested_command,
            safe_stand=False,
            estop=False,
            reason="policy",
            model_info=self.last_model_info,
            jump_requested=jump_request,
            jump_rejected=bool(
                jump_request
                and (
                    self.model_type == "rapid"
                    or self.last_model_info.get("jump_request_rejected")
                )
            ),
        )

    def _limit_targets(self, desired: Sequence[float]) -> tuple[float, ...]:
        """相邻帧限制目标变化率，防止策略输出突跳。"""
        return contract.limit_joint_targets(
            self.previous_targets,
            desired,
            self.target_rate_limit,
        )


class PlayRuntime:
    """InputAdapter→PlayPolicy 的一帧遥控运行器。"""

    def __init__(
        self,
        model: Any,
        *,
        model_type: str = "sb3",
        target_rate_limit: float | str | None = None,
    ) -> None:
        self.adapter = InputAdapter()
        self.policy = PlayPolicy(
            model,
            model_type=model_type,
            target_rate_limit=target_rate_limit,
        )

    def step(
        self,
        state: Mapping[str, Any],
        *,
        keys: Sequence[int],
        axes: Sequence[float],
        buttons: Sequence[int],
        now: float,
        last_input_at: float,
        disconnected: bool = False,
    ) -> PlayStepResult:
        """读取一帧输入并输出安全关节目标。"""
        frame = self.adapter.update(
            keys=keys,
            axes=axes,
            buttons=buttons,
            now=now,
            last_input_at=last_input_at,
            disconnected=disconnected,
        )
        return self.policy.step(
            state,
            command=frame.command,
            jump_request=frame.jump_edge,
            input_frame=frame,
            disconnected=disconnected,
        )


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """解析 play 参数。"""
    parser = argparse.ArgumentParser(description=f"{getattr(contract, 'CONTRACT_VERSION', 'R3')} play")
    parser.add_argument(
        "--checkpoint",
        default=str(
            Path(contract.CHECKPOINT_ROOT)
            / f"{getattr(contract, 'CONTRACT_VERSION', 'R3')}_final.zip"
        ),
    )
    parser.add_argument(
        "--model-type",
        choices=("sb3", "rapid"),
        default="sb3",
        help="sb3 使用当前 checkpoint；rapid 使用冻结外部模型",
    )
    parser.add_argument(
        "--pretrained-dir",
        default=str(RAPID_MODEL_DIR),
        help="Rapid 固定提交资产目录",
    )
    parser.add_argument(
        "--input",
        choices=("keyboard-joystick",),
        default="keyboard-joystick",
        help="play 输入源",
    )
    parser.add_argument(
        "--world",
        choices=("eval", "train"),
        default="eval",
        help="eval 使用单机器人 flat_move_jump_rl_eval.wbt",
    )
    parser.add_argument("--device", choices=("cuda", "cpu"), default="cuda")
    parser.add_argument("--max-steps", type=int, default=0, help="0 表示不自动停止")
    parser.add_argument(
        "--action-target-rate-limit",
        type=float,
        default=None,
        help="关节目标限速，仅允许 0.25/0.5/1.0 rad/s",
    )
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    if args.max_steps < 0:
        parser.error("max-steps 不能为负")
    if args.action_target_rate_limit is not None:
        try:
            args.action_target_rate_limit = (
                contract.validate_action_target_rate_limit(
                    args.action_target_rate_limit
                )
            )
        except ValueError as exc:
            parser.error(str(exc))
    args.pretrained_dir = str(Path(args.pretrained_dir).resolve())
    if args.model_type == "sb3":
        args.checkpoint = str(contract.validate_checkpoint_path(args.checkpoint))
    elif not Path(args.pretrained_dir).is_dir():
        parser.error(f"pretrained-dir 不存在：{args.pretrained_dir}")
    return args


def load_play_model(checkpoint: str, device: str = "cuda") -> Any:
    """加载 R3 checkpoint；play 只做推理，不创建训练环境。"""
    from .evaluate import load_model

    return load_model(checkpoint, device=device)


def load_rapid_play_model(
    pretrained_dir: Path | str,
    device: str = "cuda",
) -> RapidPolicyAdapter:
    """加载并硬校验 Rapid 冻结策略；只用于推理。"""
    return RapidPolicyAdapter(pretrained_dir, device=device)


class JsonlPlayLogger:
    """创建不覆盖历史的 play 自测 JSONL 日志。"""

    def __init__(self, root: Path | str | None = None) -> None:
        base = Path(root) if root is not None else contract.LOG_ROOT / "play"
        base.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        candidate = base / f"rapid_play_{stamp}.jsonl"
        suffix = 1
        while candidate.exists():
            candidate = base / f"rapid_play_{stamp}_{suffix:02d}.jsonl"
            suffix += 1
        self.path = candidate
        self._handle = candidate.open("x", encoding="utf-8")

    def write(self, record: Mapping[str, Any]) -> None:
        """写入并刷新一帧完整推理与执行证据。"""
        self._handle.write(
            json.dumps(
                record,
                ensure_ascii=False,
                separators=(",", ":"),
                default=_json_default,
            )
            + "\n"
        )
        self._handle.flush()

    def close(self) -> None:
        """幂等关闭日志文件。"""
        if not self._handle.closed:
            self._handle.close()

    def __enter__(self) -> "JsonlPlayLogger":
        return self

    def __exit__(self, *_exc: Any) -> None:
        self.close()


def _json_default(value: Any) -> Any:
    """把 numpy 标量和数组转换为 JSON 可表示值。"""
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f"JSON 不支持的类型：{type(value).__name__}")


def build_play_log_record(
    *,
    step: int,
    result: PlayStepResult,
    transition_info: Mapping[str, Any],
) -> dict[str, Any]:
    """汇总原始动作、映射动作、实际执行动作、限速和安全状态。"""
    info = dict(result.model_info)
    mapped = info.get("mapped_yobo_action")
    mapped_action = (
        np.asarray(mapped, dtype=np.float32)
        if mapped is not None
        else result.action.copy()
    )
    raw_source = info.get("raw_source_action")
    raw_action = (
        np.asarray(raw_source, dtype=np.float32)
        if raw_source is not None
        else result.action.copy()
    )
    applied_raw = transition_info.get("applied_action", result.action)
    applied = np.asarray(applied_raw, dtype=np.float32)
    policy_target = np.asarray(
        contract.action_to_target(result.action),
        dtype=np.float32,
    )
    limited_target = np.asarray(result.joint_targets, dtype=np.float32)
    controller_target_delta: Optional[np.ndarray] = None
    play_control = transition_info.get("play_control")
    if not isinstance(play_control, Mapping):
        play_control = transition_info.get("execution_telemetry")
    if isinstance(play_control, Mapping):
        try:
            desired_targets = np.asarray(
                play_control["desired_targets"],
                dtype=np.float32,
            )
            executed_targets = np.asarray(
                play_control["executed_targets"],
                dtype=np.float32,
            )
            if (
                desired_targets.shape == (contract.ACTION_DIM,)
                and executed_targets.shape == (contract.ACTION_DIM,)
                and np.all(np.isfinite(desired_targets))
                and np.all(np.isfinite(executed_targets))
            ):
                controller_target_delta = executed_targets - desired_targets
        except (KeyError, TypeError, ValueError):
            controller_target_delta = None
    else:
        desired_targets = np.asarray(
            transition_info.get(
                "desired_targets",
                contract.action_to_target(result.action),
            ),
            dtype=np.float32,
        )
        executed_targets = np.asarray(
            transition_info.get(
                "executed_targets",
                desired_targets,
            ),
            dtype=np.float32,
        )
        if (
            desired_targets.shape == (contract.ACTION_DIM,)
            and executed_targets.shape == (contract.ACTION_DIM,)
            and np.all(np.isfinite(desired_targets))
            and np.all(np.isfinite(executed_targets))
        ):
            controller_target_delta = executed_targets - desired_targets
    contacts = transition_info.get(
        "foot_contacts",
        transition_info.get("contacts", [0.0] * 4),
    )
    lag_values = (
        np.abs(controller_target_delta)
        if controller_target_delta is not None
        else np.abs(limited_target - policy_target)
    )
    def optional_vector(name: str, default: tuple[float, ...]) -> list[float]:
        value = transition_info.get(name, default)
        vector = np.asarray(value, dtype=np.float32)
        if (
            vector.shape == (len(default),)
            and np.all(np.isfinite(vector))
        ):
            return vector.tolist()
        return list(default)
    position = optional_vector("position", (0.0, 0.0, 0.0))
    v_body = optional_vector("v_body", (0.0, 0.0, 0.0))
    displacement_world = optional_vector(
        "displacement_world",
        (0.0, 0.0, 0.0),
    )
    displacement_body = optional_vector(
        "displacement_body",
        (0.0, 0.0, 0.0),
    )
    target_rate_limit = float(
        (
            play_control or {}
        ).get(
            "target_rate_limit",
            transition_info.get(
                "target_rate_limit",
                contract.ACTION_TARGET_RATE_LIMIT,
            ),
        )
    )
    if not math.isfinite(target_rate_limit):
        target_rate_limit = contract.ACTION_TARGET_RATE_LIMIT
    return {
        "step": int(step),
        "command": [float(value) for value in result.command],
        "raw_source_action": raw_action,
        "mapped_yobo_action": mapped_action,
        "applied_action": applied,
        "sanitization_delta": result.action - mapped_action,
        "execution_delta": applied - result.action,
        "target_rate_limit_delta": (
            controller_target_delta
            if controller_target_delta is not None
            else limited_target - policy_target
        ),
        "host_target_rate_limit_delta": limited_target - policy_target,
        "desired_targets": (
            desired_targets
            if controller_target_delta is not None
            else policy_target
        ),
        "executed_targets": (
            executed_targets
            if controller_target_delta is not None
            else limited_target
        ),
        "desired_executed_target_delta": (
            controller_target_delta
            if controller_target_delta is not None
            else limited_target - policy_target
        ),
        "target_rate_limit": target_rate_limit,
        "target_lag_mean": float(np.mean(lag_values)),
        "target_lag_rms": float(np.sqrt(np.mean(np.square(lag_values)))),
        "target_lag_max": float(np.max(lag_values)),
        "mapped_action_saturation_rate": contract.action_saturation_rate(
            mapped_action
        ),
        "applied_action_saturation_rate": contract.action_saturation_rate(
            applied
        ),
        "position": position,
        "v_body": v_body,
        "displacement_world": displacement_world,
        "displacement_body": displacement_body,
        "displacement_step_norm": float(
            math.sqrt(sum(value * value for value in displacement_world))
        ),
        "episode_distance_world": float(
            transition_info.get(
                "episode_distance_world",
                transition_info.get("state/distance_world", 0.0),
            )
        ),
        "contacts": np.asarray(contacts, dtype=np.float32),
        "fallen": bool(transition_info.get("fallen", False)),
        "safe_stand": bool(result.safe_stand),
        "emergency_stop": bool(result.estop),
        "reason": str(result.reason),
        "jump_requested": bool(result.jump_requested),
        "jump_rejected": bool(result.jump_rejected),
        "jump_supported": bool(
            info.get("jump_request_supported", True)
        ),
        "observation_42": info.get("observation_42"),
        "latent": info.get("latent"),
        "history_warmup": info.get("history_warmup"),
        "history_frames": info.get("history_frames"),
    }


def play_input_from_state(state: Mapping[str, Any]) -> dict[str, Any]:
    """从 controller state 取一帧真实键盘/手柄输入；缺失时保持安全零输入。"""
    now = time.monotonic()
    payload = state.get("play_input")
    if isinstance(payload, Mapping):
        try:
            keys = [int(value) for value in payload.get("keys", [])]
            axes = [float(value) for value in payload.get("axes", [0.0] * 3)]
            buttons = [
                int(value) for value in payload.get("buttons", [])
            ]
            last_input_at = float(payload.get("last_input_at", now))
            disconnected = bool(payload.get("disconnected", False))
            if len(axes) != 3 or not all(
                math.isfinite(value) for value in axes
            ):
                raise ValueError("手柄轴必须是 3 个有限数")
            if not math.isfinite(last_input_at):
                raise ValueError("last_input_at 必须有限")
            return {
                "keys": keys,
                "axes": axes,
                "buttons": buttons,
                "now": float(payload.get("now", now)),
                "last_input_at": last_input_at,
                "disconnected": disconnected,
            }
        except (TypeError, ValueError):
            pass
    # controller 尚未提供输入能力时采用超时安全值，不允许误接管。
    return {
        "keys": [],
        "axes": [0.0, 0.0, 0.0],
        "buttons": [],
        "now": now,
        "last_input_at": now - 1.0,
        "disconnected": False,
    }


def run_play_session(
    env: Any,
    model: Any,
    input_source: Any,
    *,
    model_type: str = "sb3",
    max_steps: int = 0,
    seed: int = 20261004,
    logger: Optional[JsonlPlayLogger] = None,
) -> dict[str, Any]:
    """用可替换 input_source 执行 play 会话，便于 Webots/controller 集成。"""
    runtime = PlayRuntime(model, model_type=model_type)
    if hasattr(model, "reset"):
        model.reset()
    observation, _info = env.reset(seed=seed, options={"command": (0.0, 0.0, 0.0), "command_fixed": True})
    del observation
    steps = 0
    try:
        while max_steps <= 0 or steps < max_steps:
            sample = input_source()
            result = runtime.step(
                env._last_state,
                **sample,
            )
            # play 的模型输出是关节目标；环境 TCP 契约仍收归一化动作。
            _obs, _reward, terminated, truncated, info = env.step(result.action)
            steps += 1
            if logger is not None:
                log_info = dict(info)
                play_control = env._last_state.get("play_control")
                if play_control is not None:
                    log_info["play_control"] = play_control
                logger.write(
                    build_play_log_record(
                        step=steps,
                        result=result,
                        transition_info=log_info,
                    )
                )
            # 急停后继续锁步发送安全站立，才能由同一会话中的 R 明确恢复；
            # 只有环境终止/截断才结束 TCP 会话。
            if terminated or truncated:
                break
    finally:
        if logger is not None:
            logger.close()
    return {"steps": steps, "estop": runtime.policy.emergency_stop.active, "reason": runtime.policy.emergency_stop.reason}


def main(argv: Optional[Sequence[str]] = None) -> int:
    """play 入口；dry-run 不加载模型、不启动仿真。"""
    args = parse_args(argv)
    selected_rate_limit = contract.resolve_action_target_rate_limit(
        args.action_target_rate_limit
    )
    contract.ACTION_TARGET_RATE_LIMIT = selected_rate_limit
    os.environ[contract.ACTION_TARGET_RATE_ENV_VAR] = format(
        selected_rate_limit,
        ".6g",
    )
    if args.dry_run:
        print(
            "PLAY_DRY_RUN "
            f"model_type={args.model_type} "
            f"checkpoint={args.checkpoint} "
            f"pretrained_dir={args.pretrained_dir} "
            f"input={args.input} world={args.world} "
            f"max_steps={args.max_steps} device={args.device} "
            f"action_target_rate_limit={selected_rate_limit}"
        )
        return 0
    if args.model_type == "rapid":
        model = load_rapid_play_model(
            args.pretrained_dir,
            device=args.device,
        )
        model_label = f"pretrained_dir={args.pretrained_dir}"
    else:
        model = load_play_model(args.checkpoint, device=args.device)
        model_label = f"checkpoint={args.checkpoint}"
    print(f"PLAY_MODEL_READY model_type={args.model_type} {model_label}")
    print("PLAY_INPUT_READY source=keyboard-joystick")

    from .env import MiniCheetahFlatJumpEnv

    # 该环境变量只在 play 进程内生效，使 controller 选择真实输入分支。
    os.environ["RL_PLAY_INTEGRATION"] = "1"
    world = (
        contract.EVAL_WORLD_PATH
        if args.world == "eval"
        else contract.WORLD_PATH
    )
    episode_steps = (
        args.max_steps
        if args.max_steps > 0
        else 1_000_000_000
    )
    env = MiniCheetahFlatJumpEnv(
        phase="P0",
        start_bridge=True,
        render_mode="human",
        world=world,
        max_episode_steps=episode_steps,
    )
    logger = JsonlPlayLogger()
    print(f"PLAY_LOG_READY path={logger.path}")
    try:
        summary = run_play_session(
            env,
            model,
            lambda: play_input_from_state(env._last_state),
            model_type=args.model_type,
            max_steps=args.max_steps,
            logger=logger,
        )
    finally:
        logger.close()
        env.close()
    print(
        "PLAY_SESSION_DONE "
        + json.dumps(summary, ensure_ascii=False, separators=(",", ":"))
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
