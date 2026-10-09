"""YoboGo RSL-RL checkpoint 的视频与逐步遥测对比诊断入口。"""

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


DEFAULT_CHECKPOINT = (
    PROJECT_ROOT
    / "training"
    / "logs"
    / "rsl_rl"
    / "yobogo_velocity_flat"
    / "2026-10-07_23-35-03_repair-v1-systemd_2026-10-07_233458"
    / "model_4799.pt"
)
DEFAULT_TASK = "YoboGo-Velocity-Flat-v0"
COMMAND_TERM_NAME = "base_velocity"
PLAYBACK_RESAMPLING_SECONDS = 1.0e9
SMOKE_COMMAND = (0.15, 0.0, 0.0)


parser = argparse.ArgumentParser(description="YoboGo checkpoint 视频与遥测对比诊断")
parser.add_argument("--task", default=DEFAULT_TASK, help="Gym 任务名")
parser.add_argument(
    "--checkpoint",
    type=Path,
    default=DEFAULT_CHECKPOINT,
    help="RSL-RL checkpoint；默认使用最终 model_4799.pt",
)
parser.add_argument("--num_envs", type=int, default=1, help="并行环境数")
parser.add_argument(
    "--smoke-steps",
    type=int,
    default=0,
    help="有限回放步数；0 表示运行到窗口关闭",
)
parser.add_argument(
    "--no-keyboard",
    action="store_true",
    help="不创建 Se2Keyboard；使用固定速度命令",
)
parser.add_argument(
    "--record-video",
    action="store_true",
    help="录制固定前进命令的机器人行走视频",
)
parser.add_argument(
    "--walk-steps",
    type=int,
    default=400,
    help="录制模式的环境步数；400 步配合 40 FPS 约为 10 秒",
)
parser.add_argument(
    "--walk-speed",
    type=float,
    default=0.3,
    help="录制模式的固定前进速度，单位 m/s",
)
parser.add_argument(
    "--video-fps",
    type=int,
    default=40,
    help="录制视频帧率",
)
parser.add_argument(
    "--output",
    type=Path,
    default=None,
    help="录制视频输出路径；默认写入 outputs/yobogo-walk/",
)
parser.add_argument(
    "--telemetry-output",
    type=Path,
    required=True,
    help="逐步遥测 JSONL 输出路径；汇总 JSON 自动写到同名 *_summary.json",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

if args_cli.num_envs < 1:
    raise ValueError("num_envs 必须大于 0")
if args_cli.smoke_steps < 0:
    raise ValueError("smoke-steps 必须大于或等于 0")
if args_cli.walk_steps < 1:
    raise ValueError("walk-steps 必须大于 0")
if not 0.0 <= args_cli.walk_speed <= 1.0:
    raise ValueError("walk-speed 必须位于 [0.0, 1.0] m/s")
if args_cli.video_fps < 1:
    raise ValueError("video-fps 必须大于 0")
if not args_cli.record_video:
    raise ValueError("checkpoint 对比诊断必须启用 --record-video")

video_output_path = None
video_name_prefix = None
generated_video_path = None
telemetry_output_path = args_cli.telemetry_output.expanduser()
if not telemetry_output_path.is_absolute():
    telemetry_output_path = PROJECT_ROOT / telemetry_output_path
telemetry_output_path = telemetry_output_path.resolve()
if telemetry_output_path.suffix.lower() != ".jsonl":
    raise ValueError("遥测输出必须使用 .jsonl 扩展名")
if telemetry_output_path.exists():
    raise FileExistsError(f"遥测输出已存在：{telemetry_output_path}")
telemetry_summary_path = telemetry_output_path.with_name(
    f"{telemetry_output_path.stem}_summary.json"
)
if telemetry_summary_path.exists():
    raise FileExistsError(f"遥测汇总已存在：{telemetry_summary_path}")
if args_cli.record_video:
    if args_cli.num_envs != 1:
        raise ValueError("录制模式只允许 num_envs=1")
    if args_cli.output is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args_cli.output = PROJECT_ROOT / "outputs" / "yobogo-walk" / f"yobogo_walk_{stamp}.mp4"
    video_output_path = args_cli.output.expanduser()
    if not video_output_path.is_absolute():
        video_output_path = PROJECT_ROOT / video_output_path
    video_output_path = video_output_path.resolve()
    if video_output_path.suffix.lower() != ".mp4":
        raise ValueError("录制输出必须使用 .mp4 扩展名")
    if video_output_path.exists():
        raise FileExistsError(f"录制输出已存在：{video_output_path}")
    video_name_prefix = f"{video_output_path.stem}_{uuid4().hex[:8]}"
    generated_video_path = video_output_path.parent / f"{video_name_prefix}-step-0.mp4"
    args_cli.no_keyboard = True
    args_cli.enable_cameras = True

checkpoint_path = args_cli.checkpoint.expanduser()
if not checkpoint_path.is_absolute():
    checkpoint_path = PROJECT_ROOT / checkpoint_path
checkpoint_path = checkpoint_path.resolve()
if not checkpoint_path.is_file():
    raise FileNotFoundError(f"checkpoint 不存在：{checkpoint_path}")

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""以下代码必须在 Isaac Sim 启动后导入。"""

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402
import torch  # noqa: E402

import isaaclab_tasks  # noqa: F401,E402
from isaaclab.envs import DirectMARLEnv, multi_agent_to_single_agent  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper  # noqa: E402
from isaaclab_tasks.utils import load_cfg_from_registry, parse_env_cfg  # noqa: E402

import training.envs  # noqa: F401,E402
from training.envs.robot_cfg import (  # noqa: E402
    CONTROL_PERIOD_S,
    FOOT_BODY_NAMES,
    JOINT_NAMES,
    TASK_ID,
    YOBOGO_MINI_CHEETAH_CFG,
)


def _emit_stage(name: str, pump_gui: bool = True) -> None:
    """输出可核验阶段标记，并在 GUI 模式显式泵一次 Kit 事件循环。"""
    print(f"[STAGE] {name}", flush=True)
    if pump_gui and not args_cli.headless:
        simulation_app.update()


def _resolve_velocity_cfg(env_cfg):
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
    if velocity_cfg.resampling_time_range != (
        PLAYBACK_RESAMPLING_SECONDS,
        PLAYBACK_RESAMPLING_SECONDS,
    ):
        raise RuntimeError("回放必须停用 10 秒随机重采样")
    return velocity_cfg


def _fixed_command() -> np.ndarray:
    """返回 headless 或禁用键盘时使用的固定速度命令。"""
    return np.asarray(SMOKE_COMMAND, dtype=np.float32)


def _install_command_source(term, command_source):
    """在官方命令计算后写入回放速度，并返回原 compute 方法。"""
    original_compute = term.compute

    def compute_with_source(dt: float) -> None:
        """先执行官方命令逻辑，再覆盖为当前回放命令。"""
        original_compute(dt)
        raw_command = np.asarray(command_source(), dtype=np.float32)
        if raw_command.shape != (3,):
            raise ValueError(f"速度命令必须为 3 维：{raw_command.shape}")

        command = torch.as_tensor(
            raw_command,
            dtype=term.vel_command_b.dtype,
            device=term.vel_command_b.device,
        )
        command = command.clone()
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

    term.compute = compute_with_source
    return original_compute


def _write_initial_command(term, command_source) -> None:
    """在初始观测计算前直接写入命令，覆盖 reset 抽样值。"""
    raw_command = np.asarray(command_source(), dtype=np.float32)
    command = torch.as_tensor(
        raw_command,
        dtype=term.vel_command_b.dtype,
        device=term.vel_command_b.device,
    ).clone()
    command[0].clamp_(term.cfg.ranges.lin_vel_x[0], term.cfg.ranges.lin_vel_x[1])
    command[1].clamp_(term.cfg.ranges.lin_vel_y[0], term.cfg.ranges.lin_vel_y[1])
    command[2].clamp_(term.cfg.ranges.ang_vel_z[0], term.cfg.ranges.ang_vel_z[1])
    term.vel_command_b.copy_(command.unsqueeze(0).expand_as(term.vel_command_b))
    term.time_left.fill_(PLAYBACK_RESAMPLING_SECONDS)


def _scalar(value) -> float:
    """把张量或标量统一转换为 Python 浮点数。"""
    if hasattr(value, "detach"):
        value = value.detach().reshape(-1)[0].cpu()
    return float(value)


def _quaternion_euler_degrees(quat_wxyz) -> tuple[float, float]:
    """按 Isaac 的 wxyz 四元数计算 roll/pitch，单位为度。"""
    w, x, y, z = (float(value) for value in quat_wxyz)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_sin = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_sin)
    return math.degrees(roll), math.degrees(pitch)


def _telemetry_payload(
    *,
    step: int,
    actions: torch.Tensor,
    env,
    term,
    robot,
    action_term,
    joint_ids: list[int],
    foot_ids: list[int],
    done,
    extras,
    reward,
) -> dict:
    """采集单步姿态、接触、动作和关节状态。"""
    root_pos = robot.data.root_pos_w[0].detach().cpu().tolist()
    root_quat = robot.data.root_quat_w[0].detach().cpu().tolist()
    roll_deg, pitch_deg = _quaternion_euler_degrees(root_quat)
    ordered_joint_pos = robot.data.joint_pos[0, joint_ids].detach().cpu().tolist()
    ordered_joint_vel = robot.data.joint_vel[0, joint_ids].detach().cpu().tolist()
    command = term.vel_command_b[0].detach().cpu().tolist()
    raw_action = actions[0].detach().cpu().tolist()
    processed_action = action_term.processed_actions[0].detach().cpu().tolist()

    sensor = env.unwrapped.scene.sensors["contact_forces"]
    history = sensor.data.net_forces_w_history
    if history is not None and history.ndim == 4:
        foot_forces = history[0, 0, foot_ids, :]
    else:
        foot_forces = sensor.data.net_forces_w[0, foot_ids, :]
    foot_force_norms = torch.linalg.vector_norm(foot_forces, dim=-1).detach().cpu().tolist()

    time_outs = extras.get("time_outs") if isinstance(extras, dict) else None
    if time_outs is None:
        terminated_value = done
        truncated_value = torch.zeros_like(done)
    else:
        terminated_value = torch.clamp(done - time_outs, min=0)
        truncated_value = time_outs

    return {
        "step": step,
        "time_s": step * CONTROL_PERIOD_S,
        "command": command,
        "base_position": root_pos,
        "base_quat_wxyz": root_quat,
        "base_roll_deg": roll_deg,
        "base_pitch_deg": pitch_deg,
        "foot_contact_force_n": foot_force_norms,
        "foot_contact": [force > 1.0 for force in foot_force_norms],
        "joint_pos": ordered_joint_pos,
        "joint_vel": ordered_joint_vel,
        "raw_action": raw_action,
        "processed_joint_target": processed_action,
        "reward": _scalar(reward),
        "terminated": _scalar(terminated_value),
        "truncated": _scalar(truncated_value),
    }


def main() -> int:
    """加载指定 checkpoint，录制视频并输出逐步状态遥测。"""
    _emit_stage("config_start")
    env_cfg = parse_env_cfg(
        args_cli.task,
        device=args_cli.device,
        num_envs=args_cli.num_envs,
        use_fabric=not getattr(args_cli, "disable_fabric", False),
    )
    _resolve_velocity_cfg(env_cfg)
    if args_cli.record_video:
        # 录制实例使用固定世界相机与较小分辨率，便于观察机器人实际位移。
        env_cfg.seed = 0
        env_cfg.viewer.resolution = (960, 540)
        env_cfg.viewer.origin_type = "world"
        env_cfg.viewer.eye = (2.8, 2.8, 1.7)
        env_cfg.viewer.lookat = (0.0, 0.0, 0.25)

    if env_cfg.scene.robot.spawn.usd_path != YOBOGO_MINI_CHEETAH_CFG.spawn.usd_path:
        raise RuntimeError("环境没有引用现有 YoboGo ArticulationCfg USD")
    _emit_stage("config_done")

    agent_cfg: RslRlOnPolicyRunnerCfg = load_cfg_from_registry(
        args_cli.task,
        "rsl_rl_cfg_entry_point",
    )
    agent_cfg.seed = 0
    agent_cfg.device = args_cli.device

    _emit_stage("environment_create_start")
    render_mode = "rgb_array" if args_cli.record_video else None
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode=render_mode)
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)
    video_recorder = None
    if args_cli.record_video:
        video_output_path.parent.mkdir(parents=True, exist_ok=True)
        # Gymnasium 强制要求先 reset 才允许调用 render。
        env.reset(seed=env_cfg.seed)
        # 固定预热三次渲染，避免 RecordVideo 首帧只保存渲染器预热黑帧。
        warmup_frame = None
        for _ in range(3):
            warmup_frame = env.render()
        if (
            not isinstance(warmup_frame, np.ndarray)
            or warmup_frame.ndim != 3
            or warmup_frame.size == 0
            or not np.any(warmup_frame)
        ):
            raise RuntimeError("录制预热未得到有效 RGB 帧")
        video_recorder = gym.wrappers.RecordVideo(
            env,
            video_folder=str(video_output_path.parent),
            step_trigger=lambda step: step == 0,
            video_length=args_cli.walk_steps,
            name_prefix=video_name_prefix,
            fps=args_cli.video_fps,
            disable_logger=True,
        )
        env = video_recorder
        _emit_stage("record_setup_done")
    env = RslRlVecEnvWrapper(env)
    _emit_stage("environment_create_done")

    print(f"[INFO] task={args_cli.task}", flush=True)
    print(f"[INFO] checkpoint={checkpoint_path}", flush=True)
    print(f"[INFO] num_envs={args_cli.num_envs}", flush=True)
    print(f"[INFO] heading_command=False", flush=True)
    print(f"[INFO] resampling_time_range={PLAYBACK_RESAMPLING_SECONDS}", flush=True)
    print(f"[DIAG] telemetry={telemetry_output_path}", flush=True)

    runner = OnPolicyRunner(
        env,
        agent_cfg.to_dict(),
        log_dir=None,
        device=agent_cfg.device,
    )
    _emit_stage("checkpoint_load_start")
    runner.load(str(checkpoint_path))
    policy = runner.get_inference_policy(device=env.unwrapped.device)
    print("[INFO] checkpoint_loaded=true", flush=True)
    _emit_stage("checkpoint_load_done")

    keyboard = None
    command_source = None
    term = None
    original_compute = None
    running = True
    telemetry_handle = None
    telemetry_records: list[dict] = []

    try:
        telemetry_output_path.parent.mkdir(parents=True, exist_ok=True)
        telemetry_handle = telemetry_output_path.open("w", encoding="utf-8")
        _emit_stage("keyboard_setup_start")
        if args_cli.record_video:
            walk_command = np.asarray(
                [args_cli.walk_speed, 0.0, 0.0],
                dtype=np.float32,
            )

            def fixed_walk_command() -> np.ndarray:
                """返回录制模式的固定前进速度命令。"""
                return walk_command

            command_source = fixed_walk_command
            print(
                f"[RECORD] walk_speed={args_cli.walk_speed} "
                f"walk_steps={args_cli.walk_steps} fps={args_cli.video_fps}",
                flush=True,
            )
        elif args_cli.no_keyboard:
            command_source = _fixed_command
            print(f"[INFO] keyboard=disabled command={SMOKE_COMMAND}", flush=True)
        else:
            # 官方 devices 包的顶层导入会同时解析 carb.input 注解，
            # 必须先显式加载对应子模块，再导入 Se2Keyboard。
            import carb.input  # noqa: E402
            import omni.appwindow  # noqa: E402
            from isaaclab.devices import Se2Keyboard  # noqa: E402

            keyboard = Se2Keyboard(
                v_x_sensitivity=0.8,
                v_y_sensitivity=0.4,
                omega_z_sensitivity=1.0,
            )
            keyboard.reset()

            def keyboard_command() -> np.ndarray:
                """读取官方 Se2Keyboard 的 [vx, vy, yaw] 命令。"""
                return np.asarray(keyboard.advance(), dtype=np.float32)

            def stop_playback() -> None:
                """按 Esc 停止交互式回放。"""
                nonlocal running
                running = False

            command_source = keyboard_command
            keyboard.add_callback("ESCAPE", stop_playback)
            print(f"[INFO] keyboard={keyboard}", flush=True)
        _emit_stage("keyboard_setup_done")

        term = env.unwrapped.command_manager.get_term(COMMAND_TERM_NAME)
        original_compute = _install_command_source(term, command_source)
        _write_initial_command(term, command_source)

        obs, _ = env.get_observations()
        robot = env.unwrapped.scene["robot"]
        joint_ids, resolved_joint_names = robot.find_joints(JOINT_NAMES, preserve_order=True)
        if resolved_joint_names != JOINT_NAMES:
            raise RuntimeError(f"诊断关节顺序错误：{resolved_joint_names}")
        action_term = env.unwrapped.action_manager.get_term("joint_pos")
        contact_sensor = env.unwrapped.scene.sensors["contact_forces"]
        foot_ids, resolved_foot_names = contact_sensor.find_bodies(
            FOOT_BODY_NAMES, preserve_order=True
        )
        if resolved_foot_names != FOOT_BODY_NAMES:
            raise RuntimeError(f"诊断足端顺序错误：{resolved_foot_names}")
        start_xy = robot.data.root_pos_w[0, :2].detach().clone()
        _emit_stage("event_loop_start")
        completed_steps = 0
        step_limit = args_cli.walk_steps if args_cli.record_video else args_cli.smoke_steps
        while simulation_app.is_running() and running:
            with torch.inference_mode():
                actions = policy(obs)
                obs, reward, done, extras = env.step(actions)
            payload = _telemetry_payload(
                step=completed_steps,
                actions=actions,
                env=env,
                term=term,
                robot=robot,
                action_term=action_term,
                joint_ids=joint_ids,
                foot_ids=foot_ids,
                done=done,
                extras=extras,
                reward=reward,
            )
            telemetry_records.append(payload)
            telemetry_handle.write(
                json.dumps(payload, ensure_ascii=False, allow_nan=False) + "\n"
            )
            telemetry_handle.flush()
            if not args_cli.headless:
                simulation_app.update()
            completed_steps += 1
            if completed_steps == 1:
                _emit_stage("first_step_done", pump_gui=False)
            if step_limit > 0 and completed_steps >= step_limit:
                break

        final_command = term.vel_command_b[0].detach().cpu().tolist()
        end_xy = robot.data.root_pos_w[0, :2].detach().clone()
        displacement = float(torch.linalg.norm(end_xy - start_xy))
        base_heights = [record["base_position"][2] for record in telemetry_records]
        roll_values = [abs(record["base_roll_deg"]) for record in telemetry_records]
        pitch_values = [abs(record["base_pitch_deg"]) for record in telemetry_records]
        foot_contact_counts = [
            sum(bool(record["foot_contact"][foot_index]) for record in telemetry_records)
            for foot_index in range(len(FOOT_BODY_NAMES))
        ]
        summary = {
            "checkpoint": str(checkpoint_path),
            "walk_speed": args_cli.walk_speed,
            "completed_steps": completed_steps,
            "final_command": final_command,
            "xy_displacement_m": displacement,
            "base_height_min_m": min(base_heights),
            "base_height_max_m": max(base_heights),
            "base_height_final_m": base_heights[-1],
            "max_abs_roll_deg": max(roll_values),
            "max_abs_pitch_deg": max(pitch_values),
            "foot_contact_ratio": [
                count / completed_steps for count in foot_contact_counts
            ],
            "terminated_steps": sum(
                bool(record["terminated"]) for record in telemetry_records
            ),
            "truncated_steps": sum(
                bool(record["truncated"]) for record in telemetry_records
            ),
            "max_abs_raw_action": max(
                abs(value)
                for record in telemetry_records
                for value in record["raw_action"]
            ),
            "max_abs_joint_target": max(
                abs(value)
                for record in telemetry_records
                for value in record["processed_joint_target"]
            ),
            "telemetry": str(telemetry_output_path),
        }
        telemetry_summary_path.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        telemetry_handle.flush()
        print(f"[INFO] completed_steps={completed_steps}", flush=True)
        print(f"[INFO] final_command={final_command}", flush=True)
        print(f"[INFO] xy_displacement={displacement:.6f} m", flush=True)
        print(
            f"[DIAG] base_height=[{summary['base_height_min_m']:.6f}, "
            f"{summary['base_height_max_m']:.6f}] "
            f"max_roll={summary['max_abs_roll_deg']:.3f} "
            f"max_pitch={summary['max_abs_pitch_deg']:.3f}",
            flush=True,
        )
        print(f"[DIAG] summary={telemetry_summary_path}", flush=True)

        if args_cli.record_video:
            if video_recorder is None or not video_recorder.recording:
                raise RuntimeError("RecordVideo 没有处于录制状态")
            _emit_stage("video_encoding_start")
            video_recorder.stop_recording()
            if (
                generated_video_path is None
                or not generated_video_path.is_file()
                or generated_video_path.stat().st_size == 0
            ):
                raise RuntimeError("RecordVideo 未生成非空 MP4 文件")
            generated_video_path.replace(video_output_path)
            print(f"[RECORD] video={video_output_path}", flush=True)
            _emit_stage("video_encoding_done")
        return 0
    finally:
        if telemetry_handle is not None:
            telemetry_handle.close()
        if term is not None and original_compute is not None:
            term.compute = original_compute
        if keyboard is not None:
            keyboard.reset()
            keyboard = None
        command_source = None
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
