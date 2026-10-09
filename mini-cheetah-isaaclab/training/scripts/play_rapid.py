"""Rapid Locomotion 官方 MIT Mini Cheetah 冻结策略的 Isaac Lab 回放入口。"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import sys
import traceback
from uuid import uuid4


# 运行脚本时不生成任何 __pycache__ 文件。
sys.dont_write_bytecode = True

from _bootstrap import PROJECT_ROOT, ensure_project_on_path


ensure_project_on_path()

from isaaclab.app import AppLauncher  # noqa: E402


DEFAULT_TASK = "YoboGo-Velocity-Flat-v0"
COMMAND_TERM_NAME = "base_velocity"
PLAYBACK_RESAMPLING_SECONDS = 1.0e9
DEFAULT_OUTPUT = PROJECT_ROOT / "outputs" / "rapid-locomotion" / "rapid_walk_test.mp4"
LEG_LABELS = ("FR", "FL", "RR", "RL")
JOINT_LABELS = ("haa", "hip", "knee")


parser = argparse.ArgumentParser(
    description="Rapid Locomotion 50 Hz 冻结策略接入与 Isaac Lab 回放"
)
parser.add_argument("--task", default=DEFAULT_TASK, help="Gym 任务名")
parser.add_argument("--num_envs", type=int, default=1, help="并行环境数")
parser.add_argument(
    "--weights-dir",
    type=Path,
    default=None,
    help="含 adaptation_module_latest.jit 与 body_latest.jit 的目录",
)
parser.add_argument(
    "--smoke-steps",
    type=int,
    default=100,
    help="非录制模式的有限控制周期数；每个周期为 0.002 s",
)
parser.add_argument(
    "--walk-speed",
    type=float,
    default=0.15,
    help="YoboGo 前进命令，单位 m/s；0 用于零速 smoke",
)
parser.add_argument(
    "--command-vx",
    type=float,
    default=None,
    help="Rapid 官方域前进命令，范围 [-0.6,0.6] m/s",
)
parser.add_argument(
    "--command-vy",
    type=float,
    default=None,
    help="Rapid 官方域侧向命令，范围 [-0.6,0.6] m/s",
)
parser.add_argument(
    "--command-yaw",
    type=float,
    default=None,
    help="Rapid 官方域偏航命令，范围 [-1,1] rad/s",
)
parser.add_argument(
    "--scenario-name",
    type=str,
    default=None,
    help="场景名称；提供后启用站立阶段与运动阶段指标",
)
parser.add_argument(
    "--settle-steps",
    type=int,
    default=0,
    help="站立稳定阶段控制周期数",
)
parser.add_argument(
    "--motion-steps",
    type=int,
    default=0,
    help="固定命令运动阶段控制周期数；场景验收至少 300",
)
parser.add_argument(
    "--record-video",
    action="store_true",
    help="录制低速行走视频并执行稳定性 Gate",
)
parser.add_argument(
    "--video-steps",
    type=int,
    default=3000,
    help="录制模式控制周期数；3000 步为 6.0 s 仿真时间",
)
parser.add_argument(
    "--video-fps",
    type=int,
    default=25,
    help="录制 MP4 帧率",
)
parser.add_argument(
    "--video-sample-every",
    type=int,
    default=20,
    help="每 20 个 500 Hz 控制周期采样一帧，即 25 Hz",
)
parser.add_argument(
    "--output",
    type=Path,
    default=DEFAULT_OUTPUT,
    help="录制 MP4 输出路径",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

if args_cli.num_envs < 1:
    raise ValueError("num_envs 必须大于 0")
if args_cli.smoke_steps < 1 and not args_cli.record_video:
    raise ValueError("非录制模式的 smoke-steps 必须大于 0")
if not 0.0 <= args_cli.walk_speed <= 0.6:
    raise ValueError("walk-speed 必须位于 [0.0,0.6] m/s")
if args_cli.settle_steps < 0 or args_cli.motion_steps < 0:
    raise ValueError("settle-steps/motion-steps 必须大于或等于 0")
if args_cli.scenario_name and args_cli.motion_steps < 300:
    raise ValueError("场景模式的 motion-steps 必须至少为 300")
if args_cli.motion_steps and args_cli.motion_steps < 300:
    raise ValueError("场景 motion-steps 必须至少为 300")
for name, value in (
    ("command-vx", args_cli.command_vx),
    ("command-vy", args_cli.command_vy),
    ("command-yaw", args_cli.command_yaw),
):
    if value is None:
        continue
    limit = 1.0 if name == "command-yaw" else 0.6
    if abs(value) > limit:
        raise ValueError(f"{name} 必须位于 [-{limit},{limit}]")
if args_cli.video_steps < 1 or args_cli.video_fps < 1 or args_cli.video_sample_every < 1:
    raise ValueError("录制步数、帧率和采样间隔必须大于 0")

video_output_path = args_cli.output.expanduser()
if not video_output_path.is_absolute():
    video_output_path = PROJECT_ROOT / video_output_path
video_output_path = video_output_path.resolve()
summary_output_path = None
video_writer = None
if args_cli.record_video:
    if args_cli.num_envs != 1:
        raise ValueError("录制模式只允许 num_envs=1")
    if video_output_path.suffix.lower() != ".mp4":
        raise ValueError("录制输出必须使用 .mp4 扩展名")
    if video_output_path.exists():
        raise FileExistsError(f"录制输出已存在：{video_output_path}")
    summary_output_path = video_output_path.with_suffix(".summary.json")
    if summary_output_path.exists():
        raise FileExistsError(f"录制摘要已存在：{summary_output_path}")
    args_cli.enable_cameras = True

if args_cli.weights_dir is None:
    from training.external_policy.rapid_locomotion.model import DEFAULT_WEIGHTS_DIR

    weights_dir = DEFAULT_WEIGHTS_DIR
else:
    weights_dir = args_cli.weights_dir.expanduser()
    if not weights_dir.is_absolute():
        weights_dir = PROJECT_ROOT / weights_dir
    weights_dir = weights_dir.resolve()

if args_cli.command_vx is None and args_cli.command_vy is None and args_cli.command_yaw is None:
    command_rapid_requested = [
        min(max(args_cli.walk_speed * 2.0, -0.6), 0.6),
        0.0,
        0.0,
    ]
    command_yobogo_written = [args_cli.walk_speed, 0.0, 0.0]
else:
    command_rapid_requested = [
        min(max(args_cli.command_vx or 0.0, -0.6), 0.6),
        min(max(args_cli.command_vy or 0.0, -0.6), 0.6),
        min(max(args_cli.command_yaw or 0.0, -1.0), 1.0),
    ]
    command_yobogo_written = [
        command_rapid_requested[0] / 2.0,
        command_rapid_requested[1] / 2.0,
        command_rapid_requested[2] / 0.25,
    ]

# 在启动 Isaac Sim 前先验证两个官方 TorchScript 权重和网络 shape 契约。
from training.external_policy.rapid_locomotion.model import (  # noqa: E402
    RapidLocomotionModel,
)

device_name = "cuda:0" if "cuda" in args_cli.device else "cpu"
rapid_model = RapidLocomotionModel(weights_dir=weights_dir, device=device_name)
dummy_action, dummy_latent = rapid_model.dummy_forward(num_envs=args_cli.num_envs)
if tuple(dummy_latent.shape) != (args_cli.num_envs, 18):
    raise RuntimeError(f"JIT latent shape 错误：{tuple(dummy_latent.shape)}")
if tuple(dummy_action.shape) != (args_cli.num_envs, 12):
    raise RuntimeError(f"JIT action shape 错误：{tuple(dummy_action.shape)}")

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""以下代码必须在 Isaac Sim 启动后导入。"""

import gymnasium as gym  # noqa: E402
import imageio.v2 as imageio  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

import isaaclab_tasks  # noqa: F401,E402
from isaaclab.envs import DirectMARLEnv, multi_agent_to_single_agent  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402

import training.envs  # noqa: F401,E402
from training.envs.robot_cfg import (  # noqa: E402
    JOINT_NAMES,
    TASK_ID,
    YOBOGO_MINI_CHEETAH_CFG,
)
from training.external_policy.rapid_locomotion.adapter import (  # noqa: E402
    ACTION_DIM,
    CONTROL_PERIOD_S,
    HISTORY_DIM,
    OBS_DIM,
    RapidLocomotionAdapter,
    map_yobogo_command_to_rapid,
)


def _emit_stage(name: str, pump_gui: bool = True) -> None:
    """输出可核验阶段标记，并在 GUI 模式显式泵一次 Kit 事件循环。"""
    print(f"[STAGE] {name}", flush=True)
    if pump_gui and not args_cli.headless:
        simulation_app.update()


def _resolve_velocity_cfg(env_cfg) -> None:
    """关闭回放期间的 heading、站立抽样和定时随机重采样。"""
    if COMMAND_TERM_NAME not in env_cfg.commands.__dict__:
        raise KeyError(f"环境缺少命令项：{COMMAND_TERM_NAME}")
    velocity_cfg = getattr(env_cfg.commands, COMMAND_TERM_NAME)
    ranges = velocity_cfg.ranges
    velocity_cfg.heading_command = False
    velocity_cfg.rel_heading_envs = 0.0
    velocity_cfg.rel_standing_envs = 0.0
    velocity_cfg.resampling_time_range = (
        PLAYBACK_RESAMPLING_SECONDS,
        PLAYBACK_RESAMPLING_SECONDS,
    )
    ranges.heading = None
    if velocity_cfg.heading_command:
        raise RuntimeError("回放必须关闭 heading command")


def _apply_rapid_replay_actuator_cfg(env_cfg) -> None:
    """只在本次回放实例采用官方 Rapid Kp/Kd 与 URDF 力矩配置。

    不修改既有训练配置，也不把 YoboGo 实机 17/17/26 N·m 配置冒充为
    官方 Rapid 的动态配置。
    """
    actuators = env_cfg.scene.robot.actuators
    if set(actuators) != set(JOINT_NAMES):
        raise RuntimeError(f"执行器关节集合错误：{sorted(actuators)}")
    for joint_name in JOINT_NAMES:
        cfg = actuators[joint_name]
        if list(cfg.joint_names_expr) != [joint_name]:
            raise RuntimeError(f"执行器关节表达式错误：{joint_name}")
        cfg.stiffness = 20.0
        cfg.damping = 0.5
        torque_limit = 18.0 if joint_name.endswith("_abad_joint") else (
            18.0 if joint_name.endswith("_hip_joint") else 26.0
        )
        cfg.effort_limit = torque_limit
        cfg.effort_limit_sim = torque_limit


def _write_command_vector(term, command: list[float]) -> None:
    """把三元 YoboGo 速度命令写入命令项并暂停随机重采样。"""
    if len(command) != 3:
        raise ValueError(f"速度命令必须为 3 维：{command}")
    command = torch.tensor(
        command,
        dtype=term.vel_command_b.dtype,
        device=term.vel_command_b.device,
    )
    command[0].clamp_(
        term.cfg.ranges.lin_vel_x[0],
        term.cfg.ranges.lin_vel_x[1],
    )
    command[1].clamp_(
        term.cfg.ranges.lin_vel_y[0],
        term.cfg.ranges.lin_vel_y[1],
    )
    command[2].clamp_(
        term.cfg.ranges.ang_vel_z[0],
        term.cfg.ranges.ang_vel_z[1],
    )
    term.vel_command_b.copy_(command.unsqueeze(0).expand_as(term.vel_command_b))
    term.time_left.fill_(PLAYBACK_RESAMPLING_SECONDS)


def _write_initial_command(term, walk_speed: float) -> None:
    """在初始观测前写入固定 YoboGo 前进速度命令。"""
    _write_command_vector(term, [walk_speed, 0.0, 0.0])


def _install_command_source(term, command_state: list[float]):
    """在官方命令计算后持续写入可变回放命令并返回原 compute。"""
    original_compute = term.compute

    def compute_with_fixed_command(dt: float) -> None:
        """先执行官方命令逻辑，再覆盖为固定速度并禁止重采样。"""
        original_compute(dt)
        _write_command_vector(term, command_state)

    term.compute = compute_with_fixed_command
    return original_compute


def _install_fixed_command(term, walk_speed: float):
    """兼容旧入口：在官方命令计算后持续写入固定前进命令。"""
    return _install_command_source(term, [walk_speed, 0.0, 0.0])


def _extract_policy_observation(value) -> torch.Tensor:
    """兼容 manager 字典与 Gym 包装层的 48 维 policy 张量提取。"""
    if isinstance(value, torch.Tensor):
        observation = value
    elif isinstance(value, dict) and "policy" in value:
        observation = value["policy"]
    else:
        raise TypeError(f"无法提取 policy observation：{type(value)}")
    if not isinstance(observation, torch.Tensor) or observation.shape[-1] != 48:
        raise RuntimeError(f"YoboGo observation 维度不是 48：{tuple(observation.shape)}")
    return observation


def _quaternion_euler_degrees(quat_wxyz: torch.Tensor) -> tuple[float, float]:
    """按 Isaac 的 wxyz 四元数计算 roll/pitch，单位为度。"""
    w, x, y, z = (float(value) for value in quat_wxyz)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_sin = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_sin)
    return math.degrees(roll), math.degrees(pitch)


def _quaternion_yaw_radians(quat_wxyz: torch.Tensor) -> float:
    """按 Isaac 的 wxyz 四元数计算世界系 yaw（弧度）。"""
    w, x, y, z = (float(value) for value in quat_wxyz)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wrap_angle_radians(angle: float) -> float:
    """把角度包裹到 [-pi, pi]，用于计算净 yaw 变化。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def _build_scenario_metrics(
    start_position: list[float],
    end_position: list[float],
    start_yaw_rad: float,
    end_yaw_rad: float,
    command_rapid: list[float],
    motion_positions: list[list[float]] | None = None,
    motion_yaws: list[float] | None = None,
    mean_body_velocity: list[float] | None = None,
    mean_body_yaw_rate: float | None = None,
) -> dict:
    """计算场景起止位姿、机体轴投影、yaw 变化和方向 Gate。"""
    dx = end_position[0] - start_position[0]
    dy = end_position[1] - start_position[1]
    displacement = math.hypot(dx, dy)
    start_forward_axis = [math.cos(start_yaw_rad), math.sin(start_yaw_rad)]
    start_left_axis = [-math.sin(start_yaw_rad), math.cos(start_yaw_rad)]
    start_forward_projection = dx * start_forward_axis[0] + dy * start_forward_axis[1]
    start_lateral_projection = dx * start_left_axis[0] + dy * start_left_axis[1]
    yaw_change_deg = math.degrees(
        _wrap_angle_radians(end_yaw_rad - start_yaw_rad)
    )
    displacement_threshold = 0.10
    dominance_ratio = 0.60
    yaw_threshold_deg = 10.0
    vx, vy, yaw = command_rapid

    body_forward_projection = start_forward_projection
    body_lateral_projection = start_lateral_projection
    if motion_positions and motion_yaws and len(motion_positions) == len(motion_yaws):
        body_forward_projection = 0.0
        body_lateral_projection = 0.0
        for index in range(1, len(motion_positions)):
            step_dx = motion_positions[index][0] - motion_positions[index - 1][0]
            step_dy = motion_positions[index][1] - motion_positions[index - 1][1]
            step_yaw = motion_yaws[index]
            step_forward = [math.cos(step_yaw), math.sin(step_yaw)]
            step_left = [-math.sin(step_yaw), math.cos(step_yaw)]
            body_forward_projection += (
                step_dx * step_forward[0] + step_dy * step_forward[1]
            )
            body_lateral_projection += (
                step_dx * step_left[0] + step_dy * step_left[1]
            )

    direction_ok = False
    primary_projection = None
    primary_axis = None
    if abs(vx) >= abs(vy) and abs(vx) > 1.0e-6:
        primary_axis = "forward"
        primary_projection = body_forward_projection
        direction_ok = (
            (vx > 0.0 and body_forward_projection > 0.0)
            or (vx < 0.0 and body_forward_projection < 0.0)
        )
        direction_ok = direction_ok and (
            abs(body_forward_projection) >= displacement_threshold
        )
        direction_ok = direction_ok and (
            abs(body_forward_projection)
            >= dominance_ratio * max(displacement, 1.0e-9)
        )
    elif abs(vy) > 1.0e-6:
        primary_axis = "lateral"
        primary_projection = body_lateral_projection
        direction_ok = (
            (vy > 0.0 and body_lateral_projection > 0.0)
            or (vy < 0.0 and body_lateral_projection < 0.0)
        )
        direction_ok = direction_ok and (
            abs(body_lateral_projection) >= displacement_threshold
        )
        direction_ok = direction_ok and (
            abs(body_lateral_projection)
            >= dominance_ratio * max(displacement, 1.0e-9)
        )
    elif abs(yaw) > 1.0e-6:
        primary_axis = "yaw"
        primary_projection = yaw_change_deg
        direction_ok = (
            (yaw > 0.0 and yaw_change_deg > 0.0)
            or (yaw < 0.0 and yaw_change_deg < 0.0)
        )
        direction_ok = direction_ok and abs(yaw_change_deg) >= yaw_threshold_deg

    # 原地转向验收不把平移下限强加给 yaw 场景；转角仍受
    # yaw_threshold_deg 独立约束，平移数值继续完整记录。
    displacement_ok = (
        primary_axis == "yaw" or displacement >= displacement_threshold
    )
    command_ok = (
        -0.6 <= vx <= 0.6
        and -0.6 <= vy <= 0.6
        and -1.0 <= yaw <= 1.0
    )
    return {
        "start_position_world": start_position,
        "end_position_world": end_position,
        "displacement_vector_world": [dx, dy],
        "displacement_m": displacement,
        "start_yaw_deg": math.degrees(start_yaw_rad),
        "end_yaw_deg": math.degrees(end_yaw_rad),
        "yaw_change_deg": yaw_change_deg,
        "start_forward_axis_world": start_forward_axis,
        "start_left_axis_world": start_left_axis,
        "start_forward_projection_m": start_forward_projection,
        "start_lateral_projection_m": start_lateral_projection,
        "body_forward_projection_m": body_forward_projection,
        "body_lateral_projection_m": body_lateral_projection,
        "mean_body_velocity_m_s": mean_body_velocity,
        "mean_body_yaw_rate_rad_s": mean_body_yaw_rate,
        "primary_axis": primary_axis,
        "primary_projection": primary_projection,
        "thresholds": {
            "displacement_m": displacement_threshold,
            "projection_dominance_ratio": dominance_ratio,
            "yaw_change_deg": yaw_threshold_deg,
        },
        "command_ok": command_ok,
        "direction_ok": direction_ok,
        "displacement_ok": displacement_ok,
    }


def _series_stats(values: list[float]) -> dict[str, float]:
    """返回序列的 min/max/RMS/范围与相对均值的运动 RMS。"""
    if not values:
        raise ValueError("统计序列不能为空")
    count = len(values)
    minimum = min(values)
    maximum = max(values)
    rms = math.sqrt(sum(value * value for value in values) / count)
    mean = sum(values) / count
    motion_rms = math.sqrt(
        sum((value - mean) ** 2 for value in values) / count
    )
    return {
        "min": minimum,
        "max": maximum,
        "rms": rms,
        "range": maximum - minimum,
        "motion_rms": motion_rms,
    }


def _build_leg_motion_stats(
    target_history: list[list[float]],
    actual_history: list[list[float]],
    raw_action_history: list[list[float]],
    applied_action_history: list[list[float]],
    position_clip_history: list[list[bool]],
    torque_clip_history: list[list[bool]],
) -> dict:
    """按 FR/FL/RR/RL 与每关节汇总 target/actual/raw/applied 统计。"""
    if not target_history or len(target_history) != len(actual_history):
        raise ValueError("target/actual 历史缺失或长度不一致")
    leg_stats: dict[str, dict] = {}
    for leg_index, leg_label in enumerate(LEG_LABELS):
        joint_stats = {}
        for joint_offset, joint_label in enumerate(JOINT_LABELS):
            column = leg_index * 3 + joint_offset
            target_values = [row[column] for row in target_history]
            actual_values = [row[column] for row in actual_history]
            raw_values = [
                row[column] for row in raw_action_history
            ] if raw_action_history else []
            applied_values = [
                row[column] for row in applied_action_history
            ] if applied_action_history else []
            joint_stats[joint_label] = {
                "target": _series_stats(target_values),
                "actual": _series_stats(actual_values),
                "raw_action": _series_stats(raw_values) if raw_values else None,
                "applied_action": _series_stats(applied_values) if applied_values else None,
                "position_clip_count": sum(
                    int(row[column]) for row in position_clip_history
                ),
                "torque_clip_count": sum(
                    int(row[column]) for row in torque_clip_history
                ),
            }
        leg_stats[leg_label] = {
            "joints": joint_stats,
            "target_range_min": min(
                joint_stats[label]["target"]["range"] for label in JOINT_LABELS
            ),
            "actual_range_min": min(
                joint_stats[label]["actual"]["range"] for label in JOINT_LABELS
            ),
            "actual_motion_rms_min": min(
                joint_stats[label]["actual"]["motion_rms"]
                for label in JOINT_LABELS
            ),
        }
    front_target_range_min = min(
        leg_stats[label]["target_range_min"] for label in ("FR", "FL")
    )
    front_actual_range_min = min(
        leg_stats[label]["actual_range_min"] for label in ("FR", "FL")
    )
    leg_stats["_gate"] = {
        "front_target_range_min": front_target_range_min,
        "front_actual_range_min": front_actual_range_min,
        "front_target_threshold_rad": 0.04,
        "front_actual_threshold_rad": 0.05,
        "ok": front_target_range_min >= 0.04
        and front_actual_range_min >= 0.05,
    }
    return leg_stats


def _close_video_writer(writer) -> None:
    """安全关闭 MP4 写入器；关闭异常不覆盖主流程状态。"""
    if writer is None:
        return
    try:
        writer.close()
    except Exception:
        traceback.print_exc()


def main() -> int:
    """加载两个 JIT 权重并运行零速、低速或录制模式回放。"""
    global video_writer
    _emit_stage("config_start")
    env_cfg = parse_env_cfg(
        args_cli.task,
        device=args_cli.device,
        num_envs=args_cli.num_envs,
        use_fabric=not getattr(args_cli, "disable_fabric", False),
    )
    _resolve_velocity_cfg(env_cfg)
    _apply_rapid_replay_actuator_cfg(env_cfg)
    # 外部策略使用已验证的干净 48 维字段，不叠加本项目训练噪声。
    env_cfg.observations.policy.enable_corruption = False
    if args_cli.record_video:
        env_cfg.seed = 0
        env_cfg.viewer.resolution = (960, 540)
        env_cfg.viewer.origin_type = "world"
        # 从前方侧视，避免机身遮挡 FR/FL；保持固定相机便于复现。
        env_cfg.viewer.eye = (-2.8, 2.8, 1.35)
        env_cfg.viewer.lookat = (0.0, 0.0, 0.25)
    if env_cfg.scene.robot.spawn.usd_path != YOBOGO_MINI_CHEETAH_CFG.spawn.usd_path:
        raise RuntimeError("环境没有引用现有 YoboGo ArticulationCfg USD")
    if abs(env_cfg.sim.dt * env_cfg.decimation - 0.002) > 1.0e-12:
        raise RuntimeError("环境控制周期不是 0.002 s")
    _emit_stage("config_done")

    _emit_stage("environment_create_start")
    render_mode = "rgb_array" if args_cli.record_video else None
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode=render_mode)
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)
    _emit_stage("environment_create_done")

    original_command_compute = None
    term = env.unwrapped.command_manager.get_term(COMMAND_TERM_NAME)
    robot = env.unwrapped.scene["robot"]
    action_term = env.unwrapped.action_manager.get_term("joint_pos")
    joint_ids, resolved_joint_names = robot.find_joints(JOINT_NAMES, preserve_order=True)
    if resolved_joint_names != JOINT_NAMES:
        raise RuntimeError(f"关节顺序错误：{resolved_joint_names}")

    _emit_stage("policy_adapter_setup_start")
    adapter = RapidLocomotionAdapter(
        policy=rapid_model,
        num_envs=args_cli.num_envs,
        device=device_name,
    )
    adapter.reset()
    scenario_mode = args_cli.scenario_name is not None
    # Gymnasium OrderEnforcing 包装层不暴露 manager 的 get_observations()，
    # reset 后直接重算 observation manager，保证观测读到固定命令。
    env.reset(seed=env_cfg.seed)
    command_state = [0.0, 0.0, 0.0] if scenario_mode else command_yobogo_written
    _write_command_vector(term, command_state)
    original_command_compute = _install_command_source(term, command_state)
    obs = _extract_policy_observation(
        env.unwrapped.observation_manager.compute_group("policy")
    )
    _emit_stage("policy_adapter_setup_done")

    if args_cli.record_video:
        # 预热渲染但不写帧，避免首帧黑屏；录制模式首次 RTX 渲染可能
        # 比普通 headless 冒烟更慢，因此保留足够的预热次数。
        warmup_frame = None
        warmup_nonzero = 0
        for warmup_index in range(30):
            warmup_frame = env.render()
            if isinstance(warmup_frame, np.ndarray) and warmup_frame.size:
                warmup_nonzero = int(np.count_nonzero(warmup_frame))
                if warmup_nonzero:
                    break
        if (
            not isinstance(warmup_frame, np.ndarray)
            or warmup_frame.ndim != 3
            or warmup_frame.size == 0
            or warmup_nonzero == 0
        ):
            shape = getattr(warmup_frame, "shape", None)
            frame_max = (
                int(np.max(warmup_frame))
                if isinstance(warmup_frame, np.ndarray) and warmup_frame.size
                else -1
            )
            frame_mean = (
                float(np.mean(warmup_frame))
                if isinstance(warmup_frame, np.ndarray) and warmup_frame.size
                else -1.0
            )
            print(
                f"[RECORD] warmup_failed type={type(warmup_frame)} "
                f"shape={shape} dtype={getattr(warmup_frame, 'dtype', None)} "
                f"nonzero={warmup_nonzero} max={frame_max} mean={frame_mean} "
                f"attempts=30",
                flush=True,
            )
            raise RuntimeError("录制预热未得到有效 RGB 帧")
        video_output_path.parent.mkdir(parents=True, exist_ok=True)
        video_writer = imageio.get_writer(
            str(video_output_path),
            fps=args_cli.video_fps,
            codec="libx264",
            format="FFMPEG",
            pixelformat="yuv420p",
            macro_block_size=1,
            quality=8,
        )

    print(f"[INFO] task={args_cli.task}", flush=True)
    print(f"[INFO] weights_adaptation={rapid_model.adaptation_path}", flush=True)
    print(f"[INFO] weights_body={rapid_model.body_path}", flush=True)
    print(f"[INFO] yobogo_observation_dim=48 rapid_observation_dim={OBS_DIM}", flush=True)
    print(f"[INFO] history_dim={HISTORY_DIM} action_dim={ACTION_DIM}", flush=True)
    print(f"[INFO] inference_period=10 control_period_s=0.002 inference_hz=50", flush=True)
    print("[INFO] rapid_replay_actuator_kp=20 kd=0.5 effort=18/18/26", flush=True)
    print(
        f"[INFO] scenario={args_cli.scenario_name} "
        f"rapid_command_requested={command_rapid_requested} "
        f"settle_steps={args_cli.settle_steps} "
        f"motion_steps={args_cli.motion_steps}",
        flush=True,
    )
    print(f"[INFO] model_dummy_action_shape={tuple(dummy_action.shape)}", flush=True)
    print(f"[INFO] model_dummy_latent_shape={tuple(dummy_latent.shape)}", flush=True)
    _emit_stage("event_loop_start")

    start_xy = robot.data.root_pos_w[0, :2].detach().clone()
    settle_start_position: list[float] | None = None
    settle_start_yaw_rad: float | None = None
    settle_positions: list[list[float]] = []
    settle_yaws: list[float] = []
    settle_min_base_height = math.inf
    settle_max_abs_roll_deg = 0.0
    settle_max_abs_pitch_deg = 0.0
    settle_terminated_steps = 0
    settle_reset_count = 0
    if scenario_mode:
        settle_start_position = (
            robot.data.root_pos_w[0].detach().cpu().tolist()
        )
        settle_start_yaw_rad = _quaternion_yaw_radians(
            robot.data.root_quat_w[0]
        )
        settle_positions.append(settle_start_position)
        settle_yaws.append(settle_start_yaw_rad)
    motion_start_position: list[float] | None = None
    motion_start_yaw_rad: float | None = None
    motion_positions: list[list[float]] = []
    motion_yaws: list[float] = []
    motion_body_velocity_sum = [0.0, 0.0]
    motion_body_yaw_rate_sum = 0.0
    motion_min_base_height = math.inf
    motion_max_abs_roll_deg = 0.0
    motion_max_abs_pitch_deg = 0.0
    motion_terminated_steps = 0
    motion_reset_count = 0
    completed_steps = 0
    inference_steps = 0
    reset_count = 0
    safety_clip_count = 0
    position_clip_count = 0
    torque_clip_count = 0
    large_action_inference_count = 0
    official_clip_inference_count = 0
    ood_guard_count = 0
    max_abs_policy_action = 0.0
    max_abs_rapid_action = 0.0
    min_base_height = math.inf
    max_abs_roll_deg = 0.0
    max_abs_pitch_deg = 0.0
    measured_velocity_sum = 0.0
    terminated_steps = 0
    video_frames_written = 0
    target_history: list[list[float]] = []
    actual_history: list[list[float]] = []
    raw_action_history: list[list[float]] = []
    applied_action_history: list[list[float]] = []
    position_clip_history: list[list[bool]] = []
    torque_clip_history: list[list[bool]] = []
    previous_done = None
    adapter_started_for_motion = False
    step_limit = args_cli.video_steps if args_cli.record_video else args_cli.smoke_steps
    if scenario_mode:
        step_limit = args_cli.settle_steps + args_cli.motion_steps
    running = True

    try:
        while simulation_app.is_running() and running:
            current_phase = (
                "settle"
                if scenario_mode and completed_steps < args_cli.settle_steps
                else "motion"
            )
            if scenario_mode and completed_steps == args_cli.settle_steps:
                motion_start_position = (
                    robot.data.root_pos_w[0].detach().cpu().tolist()
                )
                motion_start_yaw_rad = _quaternion_yaw_radians(
                    robot.data.root_quat_w[0]
                )
                motion_positions.append(motion_start_position)
                motion_yaws.append(motion_start_yaw_rad)
                adapter_started_for_motion = False
                # 场景模式下直接把 Rapid 域命令写入 term；策略观测同时
                # 使用 rapid_command_override，避免 yaw 反变换被 term 范围截断。
                command_state[:] = command_rapid_requested
                _write_command_vector(term, command_state)
            rapid_command_override = None
            if scenario_mode:
                rapid_command_override = torch.tensor(
                    [0.0, 0.0, 0.0]
                    if current_phase == "settle"
                    else command_rapid_requested,
                    dtype=torch.float32,
                    device=device_name,
                ).unsqueeze(0)
            adapter_step = None
            with torch.inference_mode():
                if scenario_mode and current_phase == "settle":
                    # 站立阶段只保持 YoboGo 几何初态，不把冻结策略的
                    # 零命令闭环噪声带入站立或 Rapid 历史；运动开始时
                    # 再重置适配器，以全零历史执行第一次 50 Hz 推理。
                    hold_action = torch.zeros(
                        args_cli.num_envs,
                        ACTION_DIM,
                        dtype=torch.float32,
                        device=device_name,
                    )
                    step_return = env.step(hold_action)
                else:
                    if scenario_mode and not adapter_started_for_motion:
                        adapter.reset()
                        adapter_started_for_motion = True
                    adapter_step = adapter.step(
                        obs,
                        reset_env_ids=previous_done,
                        rapid_command_override=rapid_command_override,
                    )
                    step_return = env.step(adapter_step.yobogo_action)
                if len(step_return) == 5:
                    obs_value, _reward, terminated, truncated, _extras = step_return
                    done = torch.logical_or(terminated, truncated)
                elif len(step_return) == 4:
                    obs_value, _reward, done, _extras = step_return
                else:
                    raise RuntimeError(
                        f"环境 step 返回长度错误：{len(step_return)}"
                    )
                obs = _extract_policy_observation(obs_value)
            if not args_cli.headless:
                simulation_app.update()

            completed_steps += 1
            inference_steps += int(
                adapter_step.did_infer if adapter_step is not None else False
            )
            if adapter_step is not None and adapter_step.did_infer:
                safety_clip_count += int(adapter_step.safety_clipped.sum().item())
                position_clip_count += int(adapter_step.position_clipped.sum().item())
                torque_clip_count += int(adapter_step.torque_clipped.sum().item())
                action_max = float(
                    adapter_step.applied_rapid_action.abs().max().item()
                )
                policy_action_max = float(
                    adapter_step.policy_raw_action.abs().max().item()
                )
                max_abs_rapid_action = max(max_abs_rapid_action, action_max)
                max_abs_policy_action = max(max_abs_policy_action, policy_action_max)
                large_action_inference_count += int(policy_action_max > 10.0)
                official_clip_inference_count += int(policy_action_max > 100.0)
                ood_guard_count += int(
                    (
                        adapter_step.policy_raw_action
                        != adapter_step.applied_rapid_action
                    ).sum().item()
                )
                nonzero_history_rows = int(
                    (
                        adapter_step.flattened_history.reshape(
                            args_cli.num_envs, 15, 42
                        )
                        .ne(0)
                        .any(dim=-1)
                        .sum()
                        .item()
                    )
                )
                print(
                    f"[DIAG_INFER] step={completed_steps} "
                    f"policy_action_max={policy_action_max:.6f} "
                    f"applied_action_max={action_max:.6f} "
                    f"obs_max={float(adapter_step.rapid_observation.abs().max()):.6f} "
                    f"history_nonzero_rows={nonzero_history_rows} "
                    f"position_clip={int(adapter_step.position_clipped.sum())} "
                    f"torque_clip={int(adapter_step.torque_clipped.sum())} "
                    f"action={adapter_step.policy_raw_action[0].detach().cpu().tolist()}",
                    flush=True,
                )
                raw_action_history.append(
                    adapter_step.policy_raw_action[0].detach().cpu().tolist()
                )
                applied_action_history.append(
                    adapter_step.applied_rapid_action[0].detach().cpu().tolist()
                )
                position_clip_history.append(
                    adapter_step.position_clipped[0].detach().cpu().tolist()
                )
                torque_clip_history.append(
                    adapter_step.torque_clipped[0].detach().cpu().tolist()
                )
                applied_target = action_term.processed_actions[0]
                target_error = float(
                    torch.max(
                        torch.abs(applied_target - adapter_step.safe_joint_target[0])
                    ).item()
                )
                current_joint_pos = robot.data.joint_pos[0, joint_ids]
                desired_joint_pos = adapter_step.safe_joint_target[0]
                print(
                    f"[DIAG_TARGET] step={completed_steps} "
                    f"target_error={target_error:.9f} "
                    f"default={robot.data.default_joint_pos[0, joint_ids].detach().cpu().tolist()} "
                    f"desired={desired_joint_pos.detach().cpu().tolist()} "
                    f"applied={applied_target.detach().cpu().tolist()} "
                    f"current={current_joint_pos.detach().cpu().tolist()}",
                    flush=True,
                )

            if adapter_step is not None and current_phase == "motion":
                target_history.append(
                    adapter_step.safe_joint_target[0].detach().cpu().tolist()
                )
                actual_history.append(
                    robot.data.joint_pos[0, joint_ids].detach().cpu().tolist()
                )
            root_pos = robot.data.root_pos_w[0]
            root_quat = robot.data.root_quat_w[0]
            roll_deg, pitch_deg = _quaternion_euler_degrees(root_quat)
            base_height = float(root_pos[2].item())
            # 行走验收按世界系前进速度统计；机体系 x 在 yaw 翻转时会
            # 变号，不能把正常前进误判为负速度。
            measured_velocity = float(robot.data.root_lin_vel_w[0, 0].item())
            min_base_height = min(min_base_height, base_height)
            max_abs_roll_deg = max(max_abs_roll_deg, abs(roll_deg))
            max_abs_pitch_deg = max(max_abs_pitch_deg, abs(pitch_deg))
            measured_velocity_sum += measured_velocity
            terminated_steps += int(bool(done[0].item()))
            if scenario_mode and current_phase == "motion":
                motion_positions.append(root_pos.detach().cpu().tolist())
                motion_yaws.append(_quaternion_yaw_radians(root_quat))
                motion_body_velocity_sum[0] += float(
                    robot.data.root_lin_vel_b[0, 0].item()
                )
                motion_body_velocity_sum[1] += float(
                    robot.data.root_lin_vel_b[0, 1].item()
                )
                motion_body_yaw_rate_sum += float(
                    robot.data.root_ang_vel_b[0, 2].item()
                )
                motion_min_base_height = min(motion_min_base_height, base_height)
                motion_max_abs_roll_deg = max(
                    motion_max_abs_roll_deg, abs(roll_deg)
                )
                motion_max_abs_pitch_deg = max(
                    motion_max_abs_pitch_deg, abs(pitch_deg)
                )
                motion_terminated_steps += int(bool(done[0].item()))
            elif scenario_mode and current_phase == "settle":
                settle_positions.append(root_pos.detach().cpu().tolist())
                settle_yaws.append(_quaternion_yaw_radians(root_quat))
                settle_min_base_height = min(
                    settle_min_base_height, base_height
                )
                settle_max_abs_roll_deg = max(
                    settle_max_abs_roll_deg, abs(roll_deg)
                )
                settle_max_abs_pitch_deg = max(
                    settle_max_abs_pitch_deg, abs(pitch_deg)
                )
                settle_terminated_steps += int(bool(done[0].item()))

            if args_cli.record_video:
                if completed_steps % args_cli.video_sample_every == 0:
                    frame = env.render()
                    if (
                        not isinstance(frame, np.ndarray)
                        or frame.ndim != 3
                        or frame.size == 0
                        or not np.any(frame)
                    ):
                        raise RuntimeError("录制采样未得到有效 RGB 帧")
                    video_writer.append_data(frame)
                    video_frames_written += 1

            reset_ids = done.nonzero(as_tuple=False).reshape(-1)
            previous_done = reset_ids if reset_ids.numel() > 0 else None
            if previous_done is not None:
                reset_count += int(previous_done.numel())
                if scenario_mode and current_phase == "motion":
                    motion_reset_count += int(previous_done.numel())
                elif scenario_mode and current_phase == "settle":
                    settle_reset_count += int(previous_done.numel())

            if completed_steps == 1:
                _emit_stage("first_step_done", pump_gui=False)
            if step_limit > 0 and completed_steps >= step_limit:
                break
            if not torch.isfinite(obs).all():
                raise ValueError("回放观测含 NaN/Inf")
            if not torch.isfinite(robot.data.root_pos_w).all():
                raise ValueError("机器人根状态含 NaN/Inf")

        _emit_stage("video_encoding_start", pump_gui=False)
        _close_video_writer(video_writer)
        video_writer = None
        if args_cli.record_video:
            expected_frames = args_cli.video_steps // args_cli.video_sample_every
            if video_frames_written != expected_frames:
                raise RuntimeError(
                    f"视频帧数错误：expected={expected_frames} actual={video_frames_written}"
                )
            if (
                not video_output_path.is_file()
                or video_output_path.stat().st_size == 0
            ):
                raise RuntimeError(f"MP4 未生成或为空：{video_output_path}")

        end_xy = robot.data.root_pos_w[0, :2].detach().clone()
        displacement = float(torch.linalg.norm(end_xy - start_xy))
        # 用世界系 XY 位移除以仿真时长作为行走速度；机体系或瞬时速度
        # 在 yaw 翻转时会变号，不能据此否定真实前进。
        mean_measured_velocity = displacement / max(
            completed_steps * CONTROL_PERIOD_S, 1.0e-9
        )
        final_command = term.vel_command_b[0].detach().cpu().tolist()
        final_rapid_from_term = (
            command_rapid_requested
            if scenario_mode
            else map_yobogo_command_to_rapid(
                torch.tensor(final_command, dtype=torch.float32)
            ).tolist()
        )
        settle_metrics = None
        scenario_metrics = None
        scenario_gate_errors: list[str] = []
        if scenario_mode:
            if settle_start_position is None or settle_start_yaw_rad is None:
                raise RuntimeError("场景模式未捕获站立阶段起点")
            if motion_start_position is None or motion_start_yaw_rad is None:
                raise RuntimeError("场景模式未捕获运动阶段起点")
            settle_end_position = motion_start_position
            settle_end_yaw_rad = motion_start_yaw_rad
            settle_dx = settle_end_position[0] - settle_start_position[0]
            settle_dy = settle_end_position[1] - settle_start_position[1]
            settle_displacement = math.hypot(settle_dx, settle_dy)
            settle_yaw_change_deg = math.degrees(
                _wrap_angle_radians(settle_end_yaw_rad - settle_start_yaw_rad)
            )
            settle_gate_errors = []
            if settle_min_base_height < 0.15:
                settle_gate_errors.append(
                    f"settle_base_height_min={settle_min_base_height:.6f}<0.15"
                )
            if settle_max_abs_roll_deg >= 45.0:
                settle_gate_errors.append(
                    f"settle_max_abs_roll_deg={settle_max_abs_roll_deg:.3f}>=45"
                )
            if settle_max_abs_pitch_deg >= 45.0:
                settle_gate_errors.append(
                    f"settle_max_abs_pitch_deg={settle_max_abs_pitch_deg:.3f}>=45"
                )
            if abs(settle_yaw_change_deg) >= 10.0:
                settle_gate_errors.append(
                    f"settle_yaw_change_deg={settle_yaw_change_deg:.3f} abs>=10"
                )
            if settle_displacement >= 0.10:
                settle_gate_errors.append(
                    f"settle_displacement={settle_displacement:.6f}>=0.10"
                )
            if settle_terminated_steps > 0:
                settle_gate_errors.append(
                    f"settle_terminated_steps={settle_terminated_steps}>0"
                )
            if settle_reset_count > 0:
                settle_gate_errors.append(
                    f"settle_reset_count={settle_reset_count}>0"
                )
            settle_metrics = {
                "start_position_world": settle_start_position,
                "end_position_world": settle_end_position,
                "displacement_m": settle_displacement,
                "start_yaw_deg": math.degrees(settle_start_yaw_rad),
                "end_yaw_deg": math.degrees(settle_end_yaw_rad),
                "yaw_change_deg": settle_yaw_change_deg,
                "base_height_min_m": settle_min_base_height,
                "max_abs_roll_deg": settle_max_abs_roll_deg,
                "max_abs_pitch_deg": settle_max_abs_pitch_deg,
                "terminated_steps": settle_terminated_steps,
                "reset_count": settle_reset_count,
                "gate_errors": settle_gate_errors,
                "pass": not settle_gate_errors,
            }
            scenario_gate_errors.extend(settle_gate_errors)
            motion_end_position = robot.data.root_pos_w[0].detach().cpu().tolist()
            motion_end_yaw_rad = _quaternion_yaw_radians(
                robot.data.root_quat_w[0]
            )
            scenario_metrics = _build_scenario_metrics(
                motion_start_position,
                motion_end_position,
                motion_start_yaw_rad,
                motion_end_yaw_rad,
                command_rapid_requested,
                motion_positions=motion_positions,
                motion_yaws=motion_yaws,
                mean_body_velocity=[
                    value / max(len(motion_positions) - 1, 1)
                    for value in motion_body_velocity_sum
                ],
                mean_body_yaw_rate=motion_body_yaw_rate_sum
                / max(len(motion_yaws) - 1, 1),
            )
            if not scenario_metrics["command_ok"]:
                scenario_gate_errors.append("command_out_of_rapid_range")
            if not scenario_metrics["direction_ok"]:
                scenario_gate_errors.append("direction_incorrect_or_not_dominant")
            if not scenario_metrics["displacement_ok"]:
                scenario_gate_errors.append(
                    f"displacement={scenario_metrics['displacement_m']:.6f}<0.10"
                )
            if motion_min_base_height < 0.15:
                scenario_gate_errors.append(
                    f"motion_base_height_min={motion_min_base_height:.6f}<0.15"
                )
            if motion_max_abs_roll_deg >= 45.0:
                scenario_gate_errors.append(
                    f"motion_max_abs_roll_deg={motion_max_abs_roll_deg:.3f}>=45"
                )
            if motion_max_abs_pitch_deg >= 45.0:
                scenario_gate_errors.append(
                    f"motion_max_abs_pitch_deg={motion_max_abs_pitch_deg:.3f}>=45"
                )
            if motion_terminated_steps > 0:
                scenario_gate_errors.append(
                    f"motion_terminated_steps={motion_terminated_steps}>0"
                )
            if motion_reset_count > 0:
                scenario_gate_errors.append(
                    f"motion_reset_count={motion_reset_count}>0"
                )
        leg_motion_stats = _build_leg_motion_stats(
            target_history,
            actual_history,
            raw_action_history,
            applied_action_history,
            position_clip_history,
            torque_clip_history,
        )
        stability_errors: list[str] = []
        if not math.isfinite(min_base_height):
            stability_errors.append("base_height_non_finite")
        if min_base_height < 0.15:
            stability_errors.append(f"base_height_min={min_base_height:.6f}<0.15")
        if max_abs_roll_deg >= 45.0:
            stability_errors.append(f"max_abs_roll_deg={max_abs_roll_deg:.3f}>=45")
        if max_abs_pitch_deg >= 45.0:
            stability_errors.append(f"max_abs_pitch_deg={max_abs_pitch_deg:.3f}>=45")
        if terminated_steps > 0:
            stability_errors.append(f"terminated_steps={terminated_steps}>0")
        if args_cli.record_video:
            if mean_measured_velocity < 0.05:
                stability_errors.append(
                    f"mean_velocity={mean_measured_velocity:.6f}<0.05"
                )
            if not leg_motion_stats["_gate"]["ok"]:
                stability_errors.append(
                    "front_leg_motion="
                    f"target_min={leg_motion_stats['_gate']['front_target_range_min']:.6f},"
                    f"actual_min={leg_motion_stats['_gate']['front_actual_range_min']:.6f}"
                )
        expected_command = (
            command_rapid_requested
            if scenario_mode
            else [args_cli.walk_speed, 0.0, 0.0]
        )
        if any(
            abs(actual - expected) > 1.0e-5
            for actual, expected in zip(final_command, expected_command)
        ):
            stability_errors.append(
                f"final_command={final_command} expected={expected_command}"
            )
        if scenario_mode:
            stability_errors.extend(scenario_gate_errors)

        summary = {
            "task": args_cli.task,
            "mode": "record_video" if args_cli.record_video else "smoke",
            "yobogo_walk_speed_m_s": args_cli.walk_speed,
            "rapid_walk_speed_m_s": min(args_cli.walk_speed * 2.0, 0.6),
            "adaptation_weight": str(rapid_model.adaptation_path),
            "body_weight": str(rapid_model.body_path),
            "completed_steps": completed_steps,
            "inference_steps": inference_steps,
            "expected_inference_steps": (
                math.ceil(args_cli.motion_steps / 10)
                if scenario_mode
                else math.ceil(completed_steps / 10)
            ),
            "reset_count": reset_count,
            "safety_clip_count": safety_clip_count,
            "position_clip_count": position_clip_count,
            "torque_clip_count": torque_clip_count,
            "large_action_inference_count": large_action_inference_count,
            "official_clip_inference_count": official_clip_inference_count,
            "max_abs_rapid_action": max_abs_rapid_action,
            "max_abs_policy_action": max_abs_policy_action,
            "ood_guard_count": ood_guard_count,
            "base_height_min_m": min_base_height,
            "max_abs_roll_deg": max_abs_roll_deg,
            "max_abs_pitch_deg": max_abs_pitch_deg,
            "xy_displacement_m": displacement,
            "mean_measured_velocity_m_s": mean_measured_velocity,
            "final_command": final_command,
            "final_rapid_from_term": final_rapid_from_term,
            "terminated_steps": terminated_steps,
            "stability_errors": stability_errors,
            "interface_ok": True,
            "stable_walking": not stability_errors,
            "scenario_name": args_cli.scenario_name,
            "scenario_settle_steps": args_cli.settle_steps,
            "scenario_motion_steps": args_cli.motion_steps,
            "scenario_command_requested_rapid": command_rapid_requested
            if scenario_mode
            else None,
            "scenario_command_policy": command_rapid_requested
            if scenario_mode
            else None,
            "scenario_command_term": final_command
            if scenario_mode
            else None,
            "scenario_command_rapid_final": final_rapid_from_term
            if scenario_mode
            else None,
            "scenario_settle_metrics": settle_metrics,
            "scenario_metrics": scenario_metrics,
            "scenario_gate_errors": scenario_gate_errors,
            "scenario_pass": (not scenario_gate_errors) if scenario_mode else None,
            "video": str(video_output_path) if args_cli.record_video else None,
            "video_frames_written": video_frames_written,
            "leg_motion_stats": leg_motion_stats,
            "scenario_mean_body_velocity": (
                [
                    value / max(len(motion_positions) - 1, 1)
                    for value in motion_body_velocity_sum
                ]
                if scenario_mode and len(motion_positions) > 1
                else None
            ),
            "scenario_mean_body_yaw_rate": (
                motion_body_yaw_rate_sum / max(len(motion_yaws) - 1, 1)
                if scenario_mode and len(motion_yaws) > 1
                else None
            ),
        }
        print(f"[SUMMARY] {json.dumps(summary, ensure_ascii=False)}", flush=True)
        if summary_output_path is not None:
            summary_output_path.write_text(
                json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            leg_stats_path = summary_output_path.with_name(
                summary_output_path.stem + ".leg_stats.json"
            )
            leg_stats_path.write_text(
                json.dumps(leg_motion_stats, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            print(f"[RECORD] leg_stats={leg_stats_path}", flush=True)
            print(f"[RECORD] summary={summary_output_path}", flush=True)
        if args_cli.record_video:
            print(f"[RECORD] video={video_output_path}", flush=True)
        base_inference_expected = (
            math.ceil(args_cli.motion_steps / 10)
            if scenario_mode
            else math.ceil(completed_steps / 10)
        )
        schedule_reset_compensated = False
        if inference_steps != base_inference_expected:
            if reset_count > 0 and abs(inference_steps - base_inference_expected) <= reset_count:
                schedule_reset_compensated = True
                print(
                    f"[SCHEDULE] reset_compensated actual={inference_steps} "
                    f"base_expected={base_inference_expected} resets={reset_count}",
                    flush=True,
                )
            else:
                raise RuntimeError(
                    f"50 Hz 调度错误：actual={inference_steps} "
                    f"expected={base_inference_expected}"
                )
        if schedule_reset_compensated and not reset_count:
            raise RuntimeError(
                f"50 Hz 调度错误：actual={inference_steps} "
                f"expected={base_inference_expected}"
            )
        if stability_errors:
            raise RuntimeError(
                "稳定性 Gate 失败：" + "；".join(stability_errors)
            )
        _emit_stage("event_loop_done")
        return 0
    finally:
        if original_command_compute is not None:
            term.compute = original_command_compute
        _close_video_writer(video_writer)
        video_writer = None
        env.close()


if __name__ == "__main__":
    try:
        exit_code = main()
    except Exception:
        traceback.print_exc()
        exit_code = 1
    finally:
        try:
            simulation_app.close()
        except SystemExit as close_error:
            # SimulationApp 关闭插件时可能主动抛出 SystemExit(0)，
            # 不得覆盖主流程已经记录的真实退出码。
            if exit_code == 0 and close_error.code not in (None, 0):
                exit_code = close_error.code
        except Exception:
            if exit_code == 0:
                traceback.print_exc()
                exit_code = 1
    sys.exit(exit_code)
