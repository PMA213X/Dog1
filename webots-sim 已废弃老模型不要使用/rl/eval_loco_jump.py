#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""yobogo_loco_jump_v1 的确定性评估入口。

本脚本只负责“加载策略 → 固定种子评估 → 汇总指标”，不启动训练。环境通过
``--env-id`` 动态解析，避免把未来契约中的类名或工厂函数写死。

正式评估示例（契约环境就绪后）：

    python webots-sim/rl/eval_loco_jump.py \\
        --model checkpoints/yobogo_loco_jump_v1.zip \\
        --env-id loco_jump_env:make_env \\
        --env-arg tag=S4_mobile_terrain \\
        --episodes 5 --seed 20261001 --max-steps 1000 \\
        --output /tmp/yobogo_loco_jump_v1_eval.json

当前没有正式模型时，可先运行 ``--self-test``，它使用内存中的 mock 环境与
mock 策略验证五集评估、指标聚合和 JSON 输出，不加载任何旧 checkpoint。
"""

from __future__ import annotations

import argparse
import importlib
import inspect
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np

# 保证同目录下的 config.py / 环境模块可导入
_RL_DIR = Path(__file__).resolve().parent
if str(_RL_DIR) not in sys.path:
    sys.path.insert(0, str(_RL_DIR))

from config import ACTION_DIM, MAX_EPISODE_STEPS  # noqa: E402

try:
    from stable_baselines3 import PPO
except ImportError as _sb3_exc:  # pragma: no cover
    PPO = None  # type: ignore[assignment,misc]
    SB3_IMPORT_ERROR: Optional[ImportError] = _sb3_exc
else:
    SB3_IMPORT_ERROR = None


# ---------------------------------------------------------------------------
# 参数解析
# ---------------------------------------------------------------------------
def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(
        description="加载 yobogo_loco_jump_v1 checkpoint 做确定性评估（不训练）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--model",
        type=str,
        default=None,
        metavar="ZIP",
        help="正式 PPO checkpoint 路径；使用 --mock-policy 或 --self-test 时可省略",
    )
    parser.add_argument(
        "--env-id",
        type=str,
        default=None,
        metavar="MODULE:SYMBOL",
        help="必填：环境工厂或环境类，格式为 module:symbol；也可传 module.Class",
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=5,
        help="评估 episode 数；正式验收至少 5 集",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=20261001,
        help="首集随机种子；后续每集按 1 递增",
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        default=MAX_EPISODE_STEPS,
        help="单集最大步数上限",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cpu",
        help="PPO 推理设备（评估固定使用 deterministic=True）",
    )
    parser.add_argument(
        "--world",
        type=str,
        default=None,
        help="环境世界文件路径（仅当环境构造器接受 world 参数时传入）",
    )
    parser.add_argument(
        "--bridge-port",
        type=int,
        default=None,
        help="TCP 桥端口（仅当环境构造器接受 bridge_port 参数时传入）",
    )
    parser.add_argument(
        "--env-arg",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="环境构造器附加参数，可重复；VALUE 按 JSON 解析，失败时按字符串",
    )
    parser.add_argument(
        "--reset-option",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="每次 reset 的 options 附加参数，可重复；VALUE 按 JSON 解析",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        metavar="JSON",
        help="评估结果 JSON 路径；省略时只打印结果",
    )
    parser.add_argument(
        "--mock-policy",
        action="store_true",
        help="使用确定性零动作 mock 策略，仅用于链路测试，结果不可验收",
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="运行内存 mock 环境/策略的 5 集自检，不启动 Webots、不读 checkpoint",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只解析并校验参数，不加载模型、不创建环境",
    )
    parser.add_argument(
        "--allow-missing-metrics",
        action="store_true",
        help="允许环境缺少 command_error/jump_success 等字段（仅诊断）",
    )
    args = parser.parse_args(argv)

    if args.episodes <= 0:
        parser.error("--episodes 必须大于 0")
    if args.max_steps <= 0:
        parser.error("--max-steps 必须大于 0")
    if args.self_test and args.episodes < 5:
        parser.error("--self-test 至少需要 5 集，以覆盖正式评估入口")
    if not args.self_test and not args.dry_run and not args.mock_policy and not args.model:
        parser.error("正式评估必须通过 --model 指定 checkpoint，或显式使用 --mock-policy")
    if args.model and args.mock_policy:
        parser.error("--model 与 --mock-policy 不能同时使用")
    if not args.self_test and not args.dry_run and not args.env_id:
        parser.error("必须通过 --env-id 指定未来环境的工厂或类，不能硬编码旧环境")
    for key_values, label in ((args.env_arg, "--env-arg"), (args.reset_option, "--reset-option")):
        for item in key_values:
            if "=" not in item:
                parser.error(f"{label} 需要 KEY=VALUE 格式：{item!r}")
    return args


def parse_key_values(items: Iterable[str], label: str) -> Dict[str, Any]:
    """把 KEY=VALUE 列表转换为字典，优先按 JSON 解析值。"""
    result: Dict[str, Any] = {}
    for item in items:
        key, raw = item.split("=", 1)
        key = key.strip()
        if not key:
            raise ValueError(f"{label} 中键名为空：{item!r}")
        try:
            value = json.loads(raw)
        except json.JSONDecodeError:
            value = raw
        result[key] = value
    return result


# ---------------------------------------------------------------------------
# 动态环境解析
# ---------------------------------------------------------------------------
def resolve_env_symbol(env_id: str) -> Any:
    """解析 module:symbol 或 module.Class；缺省符号时自动寻找本模块环境。"""
    if ":" in env_id:
        module_name, symbol_name = env_id.split(":", 1)
    elif "." in env_id:
        module_name, symbol_name = env_id.rsplit(".", 1)
    else:
        module_name, symbol_name = env_id, ""

    module = importlib.import_module(module_name)
    if symbol_name:
        symbol = getattr(module, symbol_name)
        return symbol

    # 没给符号时优先选择契约文件自己定义的工厂/环境，避免误取 gym 导入项。
    candidate_names = (
        "make_env",
        "make_loco_jump_env",
        "make",
        "LocoJumpEnv",
        "QuadrupedLocoJumpEnv",
        "QuadrupedJumpEnv",
    )
    for name in candidate_names:
        candidate = getattr(module, name, None)
        if candidate is not None:
            return candidate
    candidates = [
        value
        for name, value in vars(module).items()
        if (name.endswith("Env") or name.startswith("make_"))
        and getattr(value, "__module__", None) == module.__name__
    ]
    if len(candidates) == 1:
        return candidates[0]
    raise AttributeError(
        f"无法从 {module_name!r} 自动确定环境符号；请显式传 --env-id {module_name}:SYMBOL"
    )


def _filter_kwargs(
    symbol: Any, requested: Mapping[str, Any]
) -> Tuple[Dict[str, Any], List[str]]:
    """按构造器签名过滤公共参数，支持 **kwargs 的环境接收全部参数。"""
    try:
        signature = inspect.signature(symbol)
    except (TypeError, ValueError):
        return dict(requested), []
    parameters = signature.parameters
    if any(p.kind == inspect.Parameter.VAR_KEYWORD for p in parameters.values()):
        return dict(requested), []
    accepted = {key: value for key, value in requested.items() if key in parameters}
    skipped = sorted(set(requested) - set(accepted))
    return accepted, skipped


def make_env(
    env_id: str,
    *,
    world: Optional[str],
    bridge_port: Optional[int],
    env_args: Mapping[str, Any],
) -> Tuple[Any, List[str]]:
    """创建环境并返回（环境, 未被构造器接收的参数名）。"""
    symbol = resolve_env_symbol(env_id)
    requested: Dict[str, Any] = {
        "render_mode": None,
        "world": world,
        "bridge_port": bridge_port,
        **env_args,
    }
    requested = {key: value for key, value in requested.items() if value is not None}
    filtered, skipped = _filter_kwargs(symbol, requested)
    env = symbol(**filtered)
    if not hasattr(env, "reset") or not hasattr(env, "step"):
        raise TypeError(f"--env-id {env_id!r} 创建的对象不是 Gymnasium 风格环境")
    return env, skipped


# ---------------------------------------------------------------------------
# 策略与空间校验
# ---------------------------------------------------------------------------
class MockDeterministicPolicy:
    """确定性零动作策略，仅用于评估链路自检。"""

    def __init__(self, action_dim: int = ACTION_DIM) -> None:
        self.action_dim = int(action_dim)

    def predict(
        self, obs: np.ndarray, deterministic: bool = True
    ) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        del obs
        if not deterministic:
            raise ValueError("mock 策略只支持确定性模式")
        return np.zeros(self.action_dim, dtype=np.float32), None


def load_policy(
    args: argparse.Namespace, env: Any
) -> Tuple[Any, str, Optional[str]]:
    """加载 PPO 或创建 mock 策略，并校验观测/动作空间。"""
    if args.mock_policy:
        action_space = getattr(env, "action_space", None)
        action_dim = int(np.prod(action_space.shape)) if action_space is not None else ACTION_DIM
        return MockDeterministicPolicy(action_dim), "mock", None
    if SB3_IMPORT_ERROR is not None or PPO is None:
        raise SystemExit(f"【错误】缺少 stable-baselines3：{SB3_IMPORT_ERROR}")
    model_path = Path(args.model or "")
    if not model_path.is_file():
        raise SystemExit(f"【错误】模型不存在：{model_path}")
    try:
        import loco_jump_contract as contract

        # 契约明确拒绝旧 PPO checkpoint；评估入口必须复用同一安全边界。
        model_path = contract.validate_checkpoint_path(model_path)
    except (ImportError, ValueError) as exc:
        raise SystemExit(f"【错误】checkpoint 契约校验失败：{exc}") from exc
    model = PPO.load(str(model_path), device=args.device)

    env_obs_shape = tuple(getattr(getattr(env, "observation_space", None), "shape", ()))
    env_action_shape = tuple(getattr(getattr(env, "action_space", None), "shape", ()))
    expected_obs_shape = (int(contract.OBS_DIM),)
    if env_obs_shape and env_obs_shape != expected_obs_shape:
        raise SystemExit(
            f"【错误】环境观测不符合 yobogo_loco_jump_v1 契约："
            f"env={env_obs_shape} expected={expected_obs_shape}"
        )
    expected_action_shape = (int(contract.ACTION_DIM),)
    if env_action_shape and env_action_shape != expected_action_shape:
        raise SystemExit(
            f"【错误】环境动作不符合契约：env={env_action_shape} "
            f"expected={expected_action_shape}"
        )

    model_obs_shape = tuple(getattr(getattr(model, "observation_space", None), "shape", ()))
    if env_obs_shape and model_obs_shape and env_obs_shape != model_obs_shape:
        raise SystemExit(
            f"【错误】观测空间不兼容：env={env_obs_shape} model={model_obs_shape}"
        )
    model_action_shape = tuple(getattr(getattr(model, "action_space", None), "shape", ()))
    if env_action_shape and model_action_shape and env_action_shape != model_action_shape:
        raise SystemExit(
            f"【错误】动作空间不兼容：env={env_action_shape} model={model_action_shape}"
        )
    return model, "ppo", str(model_path)


# ---------------------------------------------------------------------------
# 指标读取
# ---------------------------------------------------------------------------
def as_float(value: Any) -> Optional[float]:
    """把标量或单元素数组转成 float；非数值返回 None。"""
    if value is None:
        return None
    if isinstance(value, Mapping):
        for key in ("value", "error", "total", "norm"):
            if key in value:
                return as_float(value[key])
        return None
    try:
        arr = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if arr.ndim == 0:
        if not np.isfinite(arr):
            return None
        return float(arr)
    if arr.size == 1:
        scalar = float(arr.reshape(-1)[0])
        return scalar if math.isfinite(scalar) else None
    # 向量命令误差按 L2 范数汇总，便于跨命令维度比较。
    if np.all(np.isfinite(arr)):
        return float(np.linalg.norm(arr))
    return None


def first_value(info: Mapping[str, Any], keys: Sequence[str]) -> Any:
    """按优先级读取 info 字段。"""
    for key in keys:
        if key in info and info[key] is not None:
            return info[key]
    return None


def wrap_angle(angle: float) -> float:
    """把角度包裹到 [-π, π]。"""
    return float((angle + math.pi) % (2.0 * math.pi) - math.pi)


def state_value(state: Mapping[str, Any], key: str, index: Optional[int] = None) -> Optional[float]:
    """安全读取环境内部 state 的标量字段。"""
    value = state.get(key)
    if value is None:
        return None
    if index is None:
        return as_float(value)
    try:
        return as_float(np.asarray(value).reshape(-1)[index])
    except (IndexError, ValueError):
        return None


def fallback_heading_error(info: Mapping[str, Any], state: Mapping[str, Any]) -> Optional[float]:
    """没有 heading_error 时，由 target_yaw / yaw 计算，单位为弧度。"""
    target = first_value(info, ("target_yaw", "target_heading", "yaw_target"))
    if target is None:
        target = state.get("target_yaw", state.get("target_heading"))
    yaw = first_value(info, ("yaw", "heading", "actual_yaw"))
    if yaw is None:
        yaw = state.get("yaw")
    if yaw is None:
        rpy = state.get("rpy")
        if rpy is not None:
            yaw = state_value(state, "rpy", 2)
    try:
        if target is None or yaw is None:
            return None
        return wrap_angle(float(target) - float(yaw))
    except (TypeError, ValueError):
        return None


def fallback_command_error(
    info: Mapping[str, Any], state: Mapping[str, Any]
) -> Optional[float]:
    """缺少 command_error 时，由目标命令与实测机体速度计算 L2 误差。"""
    reference = first_value(
        info,
        ("command", "target_command", "cmd", "command_reference"),
    )
    if reference is None:
        reference = first_value(
            state,
            ("command", "target_command", "cmd", "command_reference"),
        )
    actual = first_value(
        info,
        ("actual_command", "measured_command", "velocity", "v_body"),
    )
    if actual is None:
        actual = state.get("v_body")
    try:
        reference_array = np.asarray(reference, dtype=np.float64).reshape(-1)
        actual_array = np.asarray(actual, dtype=np.float64).reshape(-1)
    except (TypeError, ValueError):
        return None
    if reference_array.size != 3 or actual_array.size < 2:
        return None
    # 契约的 v_body 只含线速度，偏航角速度由 omega 的第三维补齐。
    if actual_array.size == 2:
        omega = first_value(info, ("omega", "omega_body", "body_omega"))
        if omega is None:
            omega = first_value(state, ("omega", "omega_body", "body_omega"))
        try:
            yaw_rate = float(np.asarray(omega, dtype=np.float64).reshape(-1)[2])
        except (TypeError, ValueError, IndexError):
            return None
        actual_array = np.concatenate([actual_array, [yaw_rate]])
    if actual_array.size != 3:
        return None
    if not np.all(np.isfinite(reference_array)) or not np.all(np.isfinite(actual_array)):
        return None
    return float(np.linalg.norm(actual_array - reference_array))


def fallback_fallen(info: Mapping[str, Any], state: Mapping[str, Any]) -> Optional[bool]:
    """没有 fallen 字段时按常用姿态阈值推断摔倒。"""
    value = first_value(info, ("fallen", "is_fallen", "fell", "fall"))
    if value is not None:
        return bool(value)
    reason = first_value(
        info,
        ("termination_reason", "end_reason", "done_reason", "failure_reason"),
    )
    if isinstance(reason, str) and reason.lower() in {"fall", "fallen", "fell"}:
        return True
    roll = first_value(info, ("roll",))
    pitch = first_value(info, ("pitch",))
    height = first_value(info, ("base_z", "z"))
    if roll is None:
        roll = state_value(state, "rpy", 0)
    if pitch is None:
        pitch = state_value(state, "rpy", 1)
    if height is None:
        height = state_value(state, "z")
    if roll is None or pitch is None or height is None:
        return None
    return bool(abs(roll) > 0.8 or abs(pitch) > 0.8 or height < 0.12)


def bool_metric(info: Mapping[str, Any], keys: Sequence[str]) -> Optional[bool]:
    """读取布尔指标，兼容字符串真假值。"""
    value = first_value(info, keys)
    if value is None:
        return None
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"true", "yes", "ok", "success", "1"}:
            return True
        if lowered in {"false", "no", "fail", "failed", "0"}:
            return False
        return None
    try:
        return bool(value)
    except (TypeError, ValueError):
        return None


def collect_episode_metrics(
    env: Any,
    reset_info: Mapping[str, Any],
    step_infos: Sequence[Mapping[str, Any]],
    state_snapshots: Optional[Sequence[Mapping[str, Any]]] = None,
) -> Dict[str, Any]:
    """从 reset/step info 提取单集命令误差、跳跃、摔倒与地形成功指标。"""
    states = list(state_snapshots or [getattr(env, "_last_state", {}) or {}])
    if not states:
        states = [{}]
    infos = [dict(reset_info), *[dict(info) for info in step_infos]]
    command_samples: List[float] = []
    heading_samples: List[float] = []
    command_key = ""
    heading_key = ""
    jump_values: List[bool] = []
    terrain_values: List[bool] = []
    fallen_values: List[bool] = []

    for index, info in enumerate(infos):
        state = states[min(index, len(states) - 1)]
        raw_command = first_value(
            info,
            (
                "command_error",
                "cmd_error",
                "command_error_norm",
                "tracking_error",
                "velocity_error",
            ),
        )
        if raw_command is None:
            raw_command = first_value(
                state,
                ("command_error", "cmd_error", "command_error_norm"),
            )
            if raw_command is not None:
                command_key = "state_command_error"
        if raw_command is None:
            raw_command = first_value(
                info,
                ("heading_error", "yaw_error", "heading_error_rad"),
            )
            if raw_command is not None:
                command_key = "heading_error"
        if raw_command is None:
            raw_command = fallback_command_error(info, state)
            if raw_command is not None:
                command_key = "fallback_velocity_command"
        else:
            if not command_key:
                command_key = "command_error"
        command_value = as_float(raw_command)
        if command_value is not None:
            command_samples.append(command_value)

        raw_heading = first_value(
            info,
            ("heading_error", "heading_error_rad", "yaw_error", "yaw_error_rad"),
        )
        if raw_heading is None:
            raw_heading = fallback_heading_error(info, state)
            if raw_heading is not None:
                heading_key = "fallback_target_yaw"
        else:
            heading_key = "heading_error"
        heading_value = as_float(raw_heading)
        if heading_value is not None:
            heading_samples.append(heading_value)

        jump_value = bool_metric(
            info,
            (
                "jump_success",
                "jumped",
                "jump_succeeded",
                "jump_ok",
                "is_jump_success",
            ),
        )
        if jump_value is not None:
            jump_values.append(jump_value)
        terrain_value = bool_metric(
            info,
            (
                "terrain_success",
                "terrain_ok",
                "obstacle_success",
                "course_success",
            ),
        )
        if terrain_value is not None:
            terrain_values.append(terrain_value)
        fallen_value = fallback_fallen(info, state)
        if fallen_value is not None:
            fallen_values.append(fallen_value)

    # 契约环境若没有直接给出跳跃布尔量，则用“离地→恢复触地”作为成功回退。
    if not jump_values:
        contact_states: List[Optional[np.ndarray]] = []
        for state in states:
            raw_contacts = state.get("contacts")
            try:
                contacts = (
                    np.asarray(raw_contacts, dtype=np.float64).reshape(-1)
                    if raw_contacts is not None else None
                )
            except (TypeError, ValueError):
                contacts = None
            contact_states.append(contacts)
        airborne = any(
            contacts is not None and contacts.size > 0 and float(np.min(contacts)) < 0.5
            for contacts in contact_states
        )
        restored = False
        saw_airborne = False
        for contacts in contact_states:
            if contacts is None:
                continue
            if contacts.size > 0 and float(np.min(contacts)) < 0.5:
                saw_airborne = True
            elif saw_airborne and float(np.max(contacts)) >= 0.5:
                restored = True
                break
        if airborne:
            jump_values.append(restored)

    # 一次 episode 只计一次摔倒，避免同一状态被重复累加。
    fall_events = sum(
        1 for previous, current in zip(fallen_values, fallen_values[1:])
        if not previous and current
    )
    if fallen_values and fallen_values[0]:
        fall_events += 1
    command_array = np.asarray(command_samples, dtype=np.float64)
    heading_array = np.asarray(heading_samples, dtype=np.float64)
    return {
        "command_error_available": bool(command_samples),
        "command_error_final": command_samples[-1] if command_samples else None,
        "command_error_mean": float(np.mean(command_array)) if command_array.size else None,
        "command_error_max": float(np.max(np.abs(command_array))) if command_array.size else None,
        "command_error_source": command_key or None,
        "heading_error_available": bool(heading_samples),
        "heading_error_final": heading_samples[-1] if heading_samples else None,
        "heading_error_mean_abs": (
            float(np.mean(np.abs(heading_array))) if heading_array.size else None
        ),
        "heading_error_source": heading_key or None,
        "jump_success": bool(jump_values and any(jump_values)),
        "jump_success_values": len(jump_values),
        "terrain_success": bool(terrain_values and any(terrain_values))
        if terrain_values else None,
        "terrain_success_values": len(terrain_values),
        "fallen": bool(fallen_values and any(fallen_values)),
        "fall_events": fall_events,
        "steps": len(step_infos),
        "reward": float(sum(float(info.get("reward", 0.0) or 0.0) for info in step_infos)),
        "termination_reason": first_value(
            step_infos[-1] if step_infos else reset_info,
            ("termination_reason", "end_reason", "failure_reason"),
        ),
    }


def aggregate_metrics(episodes: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    """汇总多集指标，显式给出字段可用集数。"""
    def collect(key: str) -> List[float]:
        return [
            float(item[key])
            for item in episodes
            if item.get(key) is not None
        ]

    command_mean = collect("command_error_mean")
    heading_mean = collect("heading_error_mean_abs")
    jump_flags = [bool(item["jump_success"]) for item in episodes]
    terrain_flags = [
        bool(item["terrain_success"])
        for item in episodes
        if item.get("terrain_success") is not None
    ]
    fallen_flags = [bool(item["fallen"]) for item in episodes]
    return {
        "episodes": len(episodes),
        "deterministic": True,
        "command_error_available_episodes": sum(
            bool(item["command_error_available"]) for item in episodes
        ),
        "command_error_mean": float(np.mean(command_mean)) if command_mean else None,
        "command_error_max": float(np.max(collect("command_error_max"))) if collect("command_error_max") else None,
        "heading_error_available_episodes": sum(
            bool(item["heading_error_available"]) for item in episodes
        ),
        "heading_error_mean_abs": float(np.mean(heading_mean)) if heading_mean else None,
        "jump_success_rate": float(np.mean(jump_flags)) if jump_flags else None,
        "jump_success_episodes": int(sum(jump_flags)),
        "terrain_success_available_episodes": len(terrain_flags),
        "terrain_success_rate": float(np.mean(terrain_flags)) if terrain_flags else None,
        "fallen_episodes": int(sum(fallen_flags)),
        "fall_rate": float(np.mean(fallen_flags)) if fallen_flags else None,
        "fall_events": int(sum(int(item["fall_events"]) for item in episodes)),
        "mean_reward": float(np.mean(collect("reward"))) if collect("reward") else None,
        "mean_steps": float(np.mean(collect("steps"))) if collect("steps") else None,
    }


def validate_metric_coverage(report: Mapping[str, Any]) -> List[str]:
    """返回正式评估缺失的关键指标，空列表表示可用于验收。"""
    missing: List[str] = []
    if int(report.get("command_error_available_episodes", 0)) < int(report.get("episodes", 0)):
        missing.append("command_error")
    if report.get("jump_success_rate") is None:
        missing.append("jump_success")
    if report.get("fall_rate") is None:
        missing.append("fallen")
    return missing


# ---------------------------------------------------------------------------
# Mock 自检
# ---------------------------------------------------------------------------
class MockLocoJumpEnv:
    """无 Webots 的最小 Gymnasium 风格环境，仅用于评估链路自检。"""

    def __init__(self, action_dim: int = 3, obs_dim: int = 6, steps: int = 12) -> None:
        from gymnasium import spaces

        self.action_space = spaces.Box(-1.0, 1.0, shape=(action_dim,), dtype=np.float32)
        self.observation_space = spaces.Box(
            -np.inf, np.inf, shape=(obs_dim,), dtype=np.float32
        )
        self._steps = int(steps)
        self._index = 0
        self._episode = 0
        self._last_state: Dict[str, Any] = {}

    def reset(
        self, *, seed: Optional[int] = None, options: Optional[Dict[str, Any]] = None
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        del seed, options
        self._index = 0
        self._episode += 1
        self._last_state = {"x": 0.0, "y": 0.0, "z": 0.25, "rpy": [0.0, 0.0, 0.0]}
        return (
            np.zeros(self.observation_space.shape, dtype=np.float32),
            {"command_error": 0.10, "jump_success": False, "fallen": False},
        )

    def step(
        self, action: np.ndarray
    ) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        del action
        self._index += 1
        command_error = 0.10 / self._index
        self._last_state["x"] = float(self._index * 0.05)
        info: Dict[str, Any] = {
            "command_error": command_error,
            "heading_error": 0.05 / self._index,
            "jump_success": self._index >= 6,
            "terrain_success": self._index >= 6,
            "fallen": False,
            "reward": 1.0,
        }
        done = self._index >= self._steps
        return (
            np.full(self.observation_space.shape, self._index, dtype=np.float32),
            1.0,
            False,
            done,
            info,
        )

    def close(self) -> None:
        return None


def run_self_test(args: argparse.Namespace) -> Dict[str, Any]:
    """运行五集 mock 评估并断言核心聚合结果。"""
    env = MockLocoJumpEnv()
    policy = MockDeterministicPolicy(env.action_space.shape[0])
    result = evaluate(args, env, policy, policy_kind="mock_self_test", model_path=None)
    aggregate = result["aggregate"]
    failures: List[str] = []
    if aggregate["episodes"] < 5:
        failures.append("episode 数少于 5")
    if aggregate["command_error_available_episodes"] != aggregate["episodes"]:
        failures.append("command_error 覆盖不足")
    if aggregate["jump_success_rate"] != 1.0:
        failures.append("jump_success 聚合错误")
    if aggregate["fall_rate"] != 0.0:
        failures.append("fallen 聚合错误")
    if failures:
        raise AssertionError("mock 自检失败：" + "；".join(failures))
    return result


# ---------------------------------------------------------------------------
# 评估主流程
# ---------------------------------------------------------------------------
def _reset_env(
    env: Any, seed: int, reset_options: Mapping[str, Any]
) -> Tuple[Any, Mapping[str, Any]]:
    """兼容标准 reset(seed=..., options=...) 与旧环境的简化签名。"""
    try:
        output = env.reset(seed=seed, options=dict(reset_options))
    except TypeError:
        try:
            output = env.reset(seed=seed)
        except TypeError:
            output = env.reset()
    if isinstance(output, tuple) and len(output) == 2:
        obs, info = output
        return obs, info if isinstance(info, Mapping) else {}
    return output, {}


def evaluate(
    args: argparse.Namespace,
    env: Any,
    policy: Any,
    *,
    policy_kind: str,
    model_path: Optional[str],
) -> Dict[str, Any]:
    """执行确定性评估，返回包含逐集与聚合指标的报告。"""
    reset_options = parse_key_values(args.reset_option, "--reset-option")
    episodes: List[Dict[str, Any]] = []
    started = time.time()
    try:
        for episode_index in range(1, args.episodes + 1):
            seed = args.seed + episode_index - 1
            obs, reset_info = _reset_env(env, seed, reset_options)
            step_infos: List[Mapping[str, Any]] = []
            state_snapshots: List[Mapping[str, Any]] = [
                dict(getattr(env, "_last_state", {}) or {})
            ]
            done = False
            truncated_by_evaluator = False
            while not done:
                action, _ = policy.predict(obs, deterministic=True)
                output = env.step(action)
                if len(output) == 5:
                    obs, _reward, terminated, truncated, info = output
                    done = bool(terminated) or bool(truncated)
                elif len(output) == 4:
                    obs, _reward, done, info = output
                    done = bool(done)
                else:
                    raise RuntimeError(f"环境 step 返回值长度异常：{len(output)}")
                step_infos.append(info if isinstance(info, Mapping) else {})
                state_snapshots.append(dict(getattr(env, "_last_state", {}) or {}))
                if len(step_infos) >= args.max_steps:
                    done = True
                    truncated_by_evaluator = True

            metrics = collect_episode_metrics(
                env, reset_info, step_infos, state_snapshots
            )
            metrics.update(
                {
                    "episode": episode_index,
                    "seed": seed,
                    "evaluator_truncated": truncated_by_evaluator,
                }
            )
            episodes.append(metrics)
            print(
                f"【评估】第 {episode_index}/{args.episodes} 集 "
                f"命令误差={_format_metric(metrics['command_error_mean'])} "
                f"跳跃成功={metrics['jump_success']} "
                f"摔倒={metrics['fallen']} 步数={metrics['steps']}"
            )
    finally:
        close = getattr(env, "close", None)
        if callable(close):
            close()

    aggregate = aggregate_metrics(episodes)
    missing = validate_metric_coverage(aggregate)
    report: Dict[str, Any] = {
        "status": "ok" if not missing else "metrics_incomplete",
        "checkpoint": model_path,
        "policy": policy_kind,
        "mock_result": policy_kind != "ppo",
        "env_id": args.env_id,
        "deterministic": True,
        "seed": args.seed,
        "max_steps": args.max_steps,
        "elapsed_seconds": time.time() - started,
        "missing_metrics": missing,
        "aggregate": aggregate,
        "episodes": episodes,
    }
    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        report["output"] = str(output_path)
    return report


def _format_metric(value: Optional[float]) -> str:
    """把可能缺失的指标格式化为短文本。"""
    return "缺失" if value is None else f"{value:.6g}"


def print_report(report: Mapping[str, Any]) -> None:
    """打印正式评估汇总。"""
    aggregate = report["aggregate"]
    print("-" * 68)
    print("【汇总】yobogo_loco_jump_v1 确定性评估")
    print(f"  状态              : {report['status']}")
    print(f"  模型/策略         : {report['checkpoint'] or report['policy']}")
    print(f"  episode           : {aggregate['episodes']} 集")
    print(
        "  命令误差          : "
        f"{_format_metric(aggregate['command_error_mean'])} "
        f"（{aggregate['command_error_available_episodes']}/{aggregate['episodes']} 集）"
    )
    print(
        "  航向误差绝对均值  : "
        f"{_format_metric(aggregate['heading_error_mean_abs'])} "
        f"（{aggregate['heading_error_available_episodes']}/{aggregate['episodes']} 集）"
    )
    print(
        f"  跳跃成功率        : {aggregate['jump_success_rate']} "
        f"（{aggregate['jump_success_episodes']}/{aggregate['episodes']} 集）"
    )
    print(
        f"  地形成功率        : {aggregate['terrain_success_rate']} "
        f"（可用 {aggregate['terrain_success_available_episodes']} 集）"
    )
    print(
        f"  摔倒率/事件       : {aggregate['fall_rate']} / {aggregate['fall_events']}"
    )
    if report.get("missing_metrics"):
        print(f"  缺失指标          : {', '.join(report['missing_metrics'])}")
    if report.get("mock_result"):
        print("  警告              : mock 结果仅用于链路测试，禁止作为验收依据")
    if report.get("output"):
        print(f"  JSON              : {report['output']}")


def main(argv: Optional[Sequence[str]] = None) -> int:
    """命令行入口，返回进程退出码。"""
    args = parse_args(argv)
    if args.dry_run:
        print("【dry-run】参数解析成功，不加载模型、不创建环境。")
        print(f"  --env-id     : {args.env_id}")
        print(f"  --episodes   : {args.episodes}")
        print(f"  --seed       : {args.seed}")
        print(f"  --max-steps  : {args.max_steps}")
        print(f"  --model      : {args.model or ('mock-policy' if args.mock_policy else '（未指定）')}")
        return 0
    if args.self_test:
        report = run_self_test(args)
        print_report(report)
        return 0

    env_args = parse_key_values(args.env_arg, "--env-arg")
    env, skipped = make_env(
        args.env_id,
        world=args.world,
        bridge_port=args.bridge_port,
        env_args=env_args,
    )
    if skipped:
        print(f"【环境】构造器未接收的附加参数：{', '.join(skipped)}")
    policy, policy_kind, model_path = load_policy(args, env)
    report = evaluate(
        args,
        env,
        policy,
        policy_kind=policy_kind,
        model_path=model_path,
    )
    print_report(report)
    if report["missing_metrics"] and not args.allow_missing_metrics:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
