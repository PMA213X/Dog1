"""YoboGo RSL-RL checkpoint 的官方键盘速度回放入口。"""

from __future__ import annotations

import argparse
from datetime import datetime
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


parser = argparse.ArgumentParser(description="YoboGo Isaac Lab 官方键盘速度回放")
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
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

if args_cli.num_envs < 1:
    raise ValueError("num_envs 必须大于 0")
if args_cli.smoke_steps < 0:
    raise ValueError("smoke-steps 必须大于或等于 0")
if args_cli.walk_steps < 1:
    raise ValueError("walk-steps 必须大于 0")
if not 0.0 < args_cli.walk_speed <= 1.0:
    raise ValueError("walk-speed 必须位于 (0.0, 1.0] m/s")
if args_cli.video_fps < 1:
    raise ValueError("video-fps 必须大于 0")

video_output_path = None
video_name_prefix = None
generated_video_path = None
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
from training.envs.robot_cfg import TASK_ID, YOBOGO_MINI_CHEETAH_CFG  # noqa: E402


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


def main() -> int:
    """加载 YoboGo checkpoint 并运行有限或交互式速度回放。"""
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

    try:
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
        start_xy = robot.data.root_pos_w[0, :2].detach().clone()
        _emit_stage("event_loop_start")
        completed_steps = 0
        step_limit = args_cli.walk_steps if args_cli.record_video else args_cli.smoke_steps
        while simulation_app.is_running() and running:
            with torch.inference_mode():
                actions = policy(obs)
                obs, _, _, _ = env.step(actions)
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
        print(f"[INFO] completed_steps={completed_steps}", flush=True)
        print(f"[INFO] final_command={final_command}", flush=True)
        print(f"[INFO] xy_displacement={displacement:.6f} m", flush=True)

        if args_cli.record_video:
            if displacement < 0.05:
                raise RuntimeError(f"录制位移过小，无法确认机器人行走：{displacement:.6f} m")
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
