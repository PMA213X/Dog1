"""单环境 reset、step、站立与随机动作检查。"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import sys

# 运行脚本时不生成任何 __pycache__ 文件。
sys.dont_write_bytecode = True

from _bootstrap import TRAINING_ROOT, ensure_project_on_path


PROJECT_ROOT = ensure_project_on_path()

from isaaclab.app import AppLauncher  # noqa: E402


parser = argparse.ArgumentParser(description="YoboGo 单环境检查")
parser.add_argument(
    "--mode",
    choices=("reset-step", "stand", "random", "all"),
    default="all",
    help="执行的检查模式",
)
parser.add_argument("--reset_steps", type=int, default=10, help="reset/step 有限值检查步数")
parser.add_argument("--stand_steps", type=int, default=500, help="零动作站立检查步数")
parser.add_argument("--random_steps", type=int, default=200, help="随机动作检查步数")
parser.add_argument("--seed", type=int, default=0, help="环境随机种子")
parser.add_argument("--report_dir", type=Path, default=None, help="报告目录；必须位于 training/runs 下")
parser.add_argument("--num_envs", type=int, default=1, help="本脚本固定为 1")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

if min(args_cli.reset_steps, args_cli.stand_steps, args_cli.random_steps) < 1:
    raise ValueError("各检查模式步数必须大于 0")
if args_cli.num_envs not in (None, 1):
    raise ValueError("本脚本只允许 num_envs=1")
if args_cli.report_dir is None:
    run_id = datetime.now(timezone.utc).strftime("%Y-%m-%d_%H-%M-%SZ")
    args_cli.report_dir = TRAINING_ROOT / "runs" / "env-check" / run_id
report_dir = args_cli.report_dir.resolve()
runs_root = (TRAINING_ROOT / "runs").resolve()
if runs_root not in report_dir.parents:
    raise ValueError(f"报告目录必须位于 {runs_root} 下：{report_dir}")
report_dir.mkdir(parents=True, exist_ok=True)

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""以下代码必须在 Isaac Sim 启动后导入。"""

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import isaaclab_tasks  # noqa: F401,E402
import training.envs  # noqa: F401,E402
from isaaclab.utils.math import quat_rotate  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from training.envs.robot_cfg import (  # noqa: E402
    CONTROL_PERIOD_S,
    FOOT_BODY_NAMES,
    JOINT_NAMES,
    SIM_INITIAL_JOINT_POS,
    SIM_INITIAL_ROOT_POS,
    TARGET_JOINT_POS,
    TASK_ID,
    USD_PATH,
)

FOOT_COLLISION_RADIUS_M = 0.0202
FOOT_COLLISION_OFFSET_Z_M = 0.024
BASE_COLLISION_OFFSET_Z_M = 0.01
BASE_COLLISION_HALF_HEIGHT_M = 0.05


def _require(condition: bool, message: str, results: list[dict]) -> None:
    """记录单项检查并在失败时停止后续依赖检查。"""
    if not condition:
        raise AssertionError(message)
    results.append({"name": message, "status": "pass"})


def _finite(name: str, value, results: list[dict]) -> None:
    """递归确认 observation 分组或张量维度非空且全部为有限值。"""
    if isinstance(value, dict):
        if not value:
            _require(False, f"{name} 观测字典非空", results)
        for child_name, child_value in value.items():
            _finite(f"{name}.{child_name}", child_value, results)
        return
    if isinstance(value, (tuple, list)):
        if not value:
            _require(False, f"{name} 观测序列非空", results)
        for index, child_value in enumerate(value):
            _finite(f"{name}[{index}]", child_value, results)
        return
    tensor = torch.as_tensor(value)
    _require(
        tensor.numel() > 0 and bool(torch.isfinite(tensor).all()),
        f"{name} 全部为有限值",
        results,
    )


def _ordered_joints(env) -> None:
    """确认动作项的配置顺序与解析顺序均为固定 12 路。"""
    term = env.unwrapped.action_manager.get_term("joint_pos")
    configured = list(term.cfg.joint_names)
    resolved = list(term._joint_names)
    if configured != JOINT_NAMES:
        raise AssertionError(f"动作配置顺序错误：{configured}")
    if resolved != JOINT_NAMES:
        raise AssertionError(f"动作解析顺序错误：{resolved}")
    if env.unwrapped.action_manager.total_action_dim != 12:
        raise AssertionError("动作维度必须为 12")


def _sim_initial_pose_tensor(device: torch.device) -> torch.Tensor:
    """按 JOINT_NAMES 构造 URDF/Isaac 仿真初态张量。"""
    return torch.tensor(
        [SIM_INITIAL_JOINT_POS[name] for name in JOINT_NAMES],
        device=device,
    )


def _collision_geometry(robot, results: list[dict]) -> None:
    """在 reset 出生姿态断言根高、足端碰撞和机身碰撞几何。"""
    root_z = float(robot.data.root_link_pos_w[0, 2])
    _require(
        abs(root_z - SIM_INITIAL_ROOT_POS[2]) <= 1.0e-5,
        "reset root 出生高度等于 0.26 m",
        results,
    )

    foot_ids, _ = robot.find_bodies(FOOT_BODY_NAMES, preserve_order=True)
    foot_pos = robot.data.body_link_pos_w[0, foot_ids]
    foot_quat = robot.data.body_link_quat_w[0, foot_ids]
    foot_offset = torch.tensor(
        [0.0, 0.0, FOOT_COLLISION_OFFSET_Z_M],
        device=foot_pos.device,
    ).expand(len(foot_ids), -1)
    foot_center_w = quat_rotate(foot_quat, foot_offset) + foot_pos
    foot_collision_min = float(
        (foot_center_w[:, 2] - FOOT_COLLISION_RADIUS_M).min() - root_z
    )
    _require(
        abs(foot_collision_min - (-0.26)) <= 1.0e-4,
        "foot_collision_min ≈ -0.26 m",
        results,
    )

    base_id, _ = robot.find_bodies(["base_link"], preserve_order=True)
    base_pos = robot.data.body_link_pos_w[0, base_id]
    base_quat = robot.data.body_link_quat_w[0, base_id]
    base_offset = torch.tensor(
        [0.0, 0.0, BASE_COLLISION_OFFSET_Z_M],
        device=base_pos.device,
    ).expand(1, -1)
    base_center_w = quat_rotate(base_quat, base_offset) + base_pos
    base_collision_min = float(
        base_center_w[0, 2] - BASE_COLLISION_HALF_HEIGHT_M - root_z
    )
    _require(
        abs(base_collision_min - (-0.04)) <= 1.0e-6,
        "base_collision_min = -0.04 m",
        results,
    )
    results.append(
        {
            "name": "reset 根高与碰撞几何",
            "status": "pass",
            "root_z": root_z,
            "foot_collision_min": foot_collision_min,
            "base_collision_min": base_collision_min,
        }
    )


def _reset_and_step(env, steps: int, results: list[dict]) -> None:
    """检查 reset 后与连续 step 后的观测、奖励和终止信号。"""
    obs, extras = env.reset(seed=args_cli.seed)
    _finite("reset observation", obs, results)
    if "observations" in extras:
        for name, value in extras["observations"].items():
            _finite(f"reset observation {name}", value, results)

    robot = env.unwrapped.scene["robot"]
    joint_ids, resolved_names = robot.find_joints(JOINT_NAMES, preserve_order=True)
    if resolved_names != JOINT_NAMES:
        raise AssertionError(f"资产关节解析顺序错误：{resolved_names}")
    ordered_joint_pos = robot.data.joint_pos[0, joint_ids]
    pose_error = torch.max(
        torch.abs(
            ordered_joint_pos
            - _sim_initial_pose_tensor(robot.data.joint_pos.device)
        )
    )
    _require(
        float(pose_error) <= 1.0e-5,
        "reset 后关节姿态等于 SIM_INITIAL_JOINT_POS",
        results,
    )
    _require(
        ordered_joint_pos.shape[0] == 12,
        "reset 后 12 关节状态齐全",
        results,
    )
    _collision_geometry(robot, results)

    for step in range(steps):
        actions = torch.zeros((1, 12), device=env.unwrapped.device)
        obs, reward, terminated, truncated, info = env.step(actions)
        _finite(f"step {step} observation", obs, results)
        _finite(f"step {step} reward", reward, results)
        _finite(f"step {step} terminated", terminated, results)
        _finite(f"step {step} truncated", truncated, results)
    results.append({"name": f"连续 step {steps} 次", "status": "pass"})


def _stand(env, steps: int, results: list[dict]) -> None:
    """检查零动作站立后的默认目标、机身高度和四足接触。"""
    env.reset(seed=args_cli.seed)
    actions = torch.zeros((1, 12), device=env.unwrapped.device)
    base_height = float("nan")
    for _ in range(steps):
        obs, reward, terminated, truncated, info = env.step(actions)
        _finite("stand observation", obs, results)
        _finite("stand reward", reward, results)
        base_height = float(env.unwrapped.scene["robot"].data.root_pos_w[0, 2])

    robot = env.unwrapped.scene["robot"]
    term = env.unwrapped.action_manager.get_term("joint_pos")
    target_error = torch.max(
        torch.abs(
            term.processed_actions[0]
            - _sim_initial_pose_tensor(term.processed_actions.device)
        )
    )
    _require(
        float(target_error) <= 1.0e-5,
        "站立动作目标等于 SIM_INITIAL_JOINT_POS",
        results,
    )
    _require(base_height > 0.05, "站立机身高度高于足端碰撞容差", results)

    sensor = env.unwrapped.scene.sensors["contact_forces"]
    foot_ids, _ = sensor.find_bodies(FOOT_BODY_NAMES, preserve_order=True)
    # 官方历史张量形状为 (num_envs, history, bodies, 3)。
    forces = sensor.data.net_forces_w_history[:, :, foot_ids, :]
    contact_ratio = (torch.norm(forces, dim=-1) > 1.0).float().mean().item()
    _require(contact_ratio >= 0.95, "站立四足有效接触比例 >= 0.95", results)
    results.append({"name": f"零动作站立 {steps} 次", "status": "pass", "contact_ratio": contact_ratio})


def _random(env, steps: int, results: list[dict]) -> None:
    """检查随机动作输入、动作裁剪与状态有限性。"""
    env.reset(seed=args_cli.seed)
    generator = torch.Generator(device="cpu").manual_seed(args_cli.seed)
    max_base_height = float("-inf")
    min_base_height = float("inf")
    for step in range(steps):
        actions = 2.0 * torch.rand((1, 12), generator=generator) - 1.0
        actions = actions.to(env.unwrapped.device)
        obs, reward, terminated, truncated, info = env.step(actions)
        _finite(f"random step {step} observation", obs, results)
        _finite(f"random step {step} reward", reward, results)
        stored = env.unwrapped.action_manager.action
        _require(
            bool((stored >= -1.0 - 1.0e-6).all() and (stored <= 1.0 + 1.0e-6).all()),
            "随机动作保持在 [-1, 1]",
            results,
        )
        height = float(env.unwrapped.scene["robot"].data.root_pos_w[0, 2])
        max_base_height = max(max_base_height, height)
        min_base_height = min(min_base_height, height)
    _require(min_base_height > 0.05, "随机动作期间机身未触地", results)
    results.append(
        {
            "name": f"随机动作 {steps} 次",
            "status": "pass",
            "base_height_min": min_base_height,
            "base_height_max": max_base_height,
        }
    )


def main() -> int:
    """执行配置、动作顺序及所选单环境模式。"""
    if not USD_PATH.is_file():
        raise FileNotFoundError(f"项目 USD 尚未生成或不在约定路径：{USD_PATH}")

    env_cfg = parse_env_cfg(TASK_ID, device=args_cli.device, num_envs=1)
    control_period = env_cfg.sim.dt * env_cfg.decimation
    results: list[dict] = []
    _require(abs(control_period - CONTROL_PERIOD_S) <= 1.0e-12, "控制周期等于 0.002 s", results)
    _require(env_cfg.scene.robot.spawn.usd_path == str(USD_PATH), "只引用项目 USD", results)

    env = gym.make(TASK_ID, cfg=env_cfg)
    try:
        _ordered_joints(env)
        results.append({"name": "动作/关节固定 12 路顺序", "status": "pass"})

        if args_cli.mode in ("reset-step", "all"):
            _reset_and_step(env, args_cli.reset_steps, results)
        if args_cli.mode in ("stand", "all"):
            _stand(env, args_cli.stand_steps, results)
        if args_cli.mode in ("random", "all"):
            _random(env, args_cli.random_steps, results)
    finally:
        env.close()

    payload = {
        "task": TASK_ID,
        "mode": args_cli.mode,
        "seed": args_cli.seed,
        "usd_path": str(USD_PATH),
        "joint_names": JOINT_NAMES,
        "target_joint_pos": TARGET_JOINT_POS,
        "sim_initial_joint_pos": SIM_INITIAL_JOINT_POS,
        "control_period_s": control_period,
        "status": "pass",
        "checks": results,
    }
    (report_dir / "gate.json").write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    (report_dir / "metrics.json").write_text(
        json.dumps(
            {
                "status": "pass",
                "check_count": len(results),
                "reset_steps": args_cli.reset_steps,
                "stand_steps": args_cli.stand_steps,
                "random_steps": args_cli.random_steps,
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    print(f"report_dir={report_dir}")
    return 0


if __name__ == "__main__":
    try:
        exit_code = main()
    except Exception as exc:
        import traceback

        traceback.print_exc()
        failure = {
            "task": TASK_ID,
            "mode": args_cli.mode,
            "seed": args_cli.seed,
            "status": "fail",
            "error": f"{type(exc).__name__}: {exc}",
        }
        (report_dir / "gate.json").write_text(json.dumps(failure, ensure_ascii=False, indent=2), encoding="utf-8")
        (report_dir / "metrics.json").write_text(json.dumps(failure, ensure_ascii=False, indent=2), encoding="utf-8")
        exit_code = 1
    finally:
        simulation_app.close()
    sys.exit(exit_code)
