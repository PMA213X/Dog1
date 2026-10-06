#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""R3 checkpoint 的遥控 play 推理管线与独立安全停止。"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence

import numpy as np

from . import contract


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


class PlayPolicy:
    """命令→57 维观测→deterministic PPO→12 关节目标。"""

    def __init__(
        self,
        model: Any,
        *,
        emergency_stop: Optional[EmergencyStop] = None,
        builder: Optional[ObservationBuilder] = None,
    ) -> None:
        self.model = model
        self.emergency_stop = emergency_stop or EmergencyStop()
        self.builder = builder or ObservationBuilder()
        self.previous_action = np.zeros(contract.ACTION_DIM, dtype=np.float32)
        self.previous_targets = tuple(float(value) for value in contract.DEFAULT_CROUCH)

    def safe_stand_result(
        self,
        state: Mapping[str, Any],
        *,
        reason: str,
        command: Sequence[float] = (0.0, 0.0, 0.0),
        jump_request: bool = False,
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
        return PlayStepResult(
            observation=observation,
            action=np.zeros(contract.ACTION_DIM, dtype=np.float32),
            joint_targets=targets,
            command=(0.0, 0.0, 0.0),
            safe_stand=True,
            estop=self.emergency_stop.active,
            reason=reason,
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
        if not jump_request and all(abs(value) <= 1e-9 for value in requested_command):
            return self.safe_stand_result(state, reason="zero_command")
        try:
            observation = self.builder.build(
                state,
                command=requested_command,
                previous_action=self.previous_action,
                jump_request=jump_request,
            )
            raw, _ = self.model.predict(observation, deterministic=True)
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
        return PlayStepResult(
            observation=observation,
            action=action,
            joint_targets=targets,
            command=requested_command,
            safe_stand=False,
            estop=False,
            reason="policy",
        )

    def _limit_targets(self, desired: Sequence[float]) -> tuple[float, ...]:
        """相邻帧限制目标变化率，防止策略输出突跳。"""
        max_delta = 0.75 * contract.CONTROL_DT_SECONDS
        result = []
        for old, new in zip(self.previous_targets, desired):
            value = max(old - max_delta, min(old + max_delta, float(new)))
            result.append(value)
        return tuple(result)


class PlayRuntime:
    """InputAdapter→PlayPolicy 的一帧遥控运行器。"""

    def __init__(self, model: Any) -> None:
        self.adapter = InputAdapter()
        self.policy = PlayPolicy(model)

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
    parser.add_argument("--device", choices=("cuda", "cpu"), default="cuda")
    parser.add_argument("--max-steps", type=int, default=0, help="0 表示不自动停止")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    if args.max_steps < 0:
        parser.error("max-steps 不能为负")
    args.checkpoint = str(contract.validate_checkpoint_path(args.checkpoint))
    return args


def load_play_model(checkpoint: str, device: str = "cuda") -> Any:
    """加载 R3 checkpoint；play 只做推理，不创建训练环境。"""
    from .evaluate import load_model

    return load_model(checkpoint, device=device)


def run_play_session(
    env: Any,
    model: Any,
    input_source: Any,
    *,
    max_steps: int = 0,
    seed: int = 20261004,
) -> dict[str, Any]:
    """用可替换 input_source 执行 play 会话，便于 Webots/controller 集成。"""
    runtime = PlayRuntime(model)
    observation, _info = env.reset(seed=seed, options={"command": (0.0, 0.0, 0.0), "command_fixed": True})
    del observation
    steps = 0
    while max_steps <= 0 or steps < max_steps:
        sample = input_source()
        result = runtime.step(
            env._last_state,
            **sample,
        )
        # play 的模型输出是关节目标；环境 TCP 契约仍收归一化动作。
        _obs, _reward, terminated, truncated, _info = env.step(result.action)
        steps += 1
        if result.estop or terminated or truncated:
            break
    return {"steps": steps, "estop": runtime.policy.emergency_stop.active, "reason": runtime.policy.emergency_stop.reason}


def main(argv: Optional[Sequence[str]] = None) -> int:
    """play 入口；dry-run 不加载模型、不启动仿真。"""
    args = parse_args(argv)
    if args.dry_run:
        print(f"PLAY_DRY_RUN checkpoint={args.checkpoint} device={args.device}")
        return 0
    load_play_model(args.checkpoint, device=args.device)
    print("PLAY_MODEL_READY checkpoint=" + args.checkpoint)
    print("PLAY_INPUT_REQUIRED 使用 controllers/rl_agent/input_adapter.py 提供键盘/手柄帧")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
