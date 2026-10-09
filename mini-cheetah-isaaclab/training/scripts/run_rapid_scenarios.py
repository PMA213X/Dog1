"""顺序运行 Rapid 前/后/左/右/转向场景并汇总视频证据。"""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[2]
ISAAC_LAB = Path("/home/pma213x/IsaacLab/isaaclab.sh")
LOG_ROOT = (
    PROJECT_ROOT
    / "training"
    / "logs"
    / "rapid-locomotion"
    / "2026-10-08"
    / "scenarios"
)
OUTPUT_ROOT = PROJECT_ROOT / "outputs" / "rapid-locomotion" / "scenarios"


SCENARIOS = {
    "forward": (0.6, 0.0, 0.0),
    "backward": (-0.6, 0.0, 0.0),
    "left": (0.0, 0.6, 0.0),
    "right": (0.0, -0.6, 0.0),
    "yaw": (0.0, 0.0, 0.5),
}


def _probe_media(video_path: Path) -> dict:
    """用 ffprobe/ffmpeg 校验单个视频的时长、分辨率、帧率和帧数。"""
    probe = subprocess.run(
        [
            "ffprobe",
            "-v",
            "error",
            "-show_entries",
            "format=duration,size,format_name",
            "-show_entries",
            "stream=index,codec_name,codec_type,width,height,r_frame_rate,nb_frames",
            "-of",
            "json",
            str(video_path),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    probe_data = json.loads(probe.stdout)
    stream = next(
        item
        for item in probe_data.get("streams", [])
        if item.get("codec_type") == "video"
    )
    fmt = probe_data.get("format", {})
    decode = subprocess.run(
        ["ffmpeg", "-v", "error", "-i", str(video_path), "-f", "null", "-"],
        check=True,
        capture_output=True,
        text=True,
    )
    count = subprocess.run(
        [
            "ffprobe",
            "-v",
            "error",
            "-count_frames",
            "-select_streams",
            "v:0",
            "-show_entries",
            "stream=nb_read_frames",
            "-of",
            "csv=p=0",
            str(video_path),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    frame_count = int(count.stdout.strip().splitlines()[-1])
    duration = float(fmt.get("duration", "0"))
    if not 5.0 <= duration <= 8.0:
        raise RuntimeError(f"视频时长不在 5-8 秒：{video_path} {duration}")
    if int(stream.get("width", 0)) <= 0 or int(stream.get("height", 0)) <= 0:
        raise RuntimeError(f"视频分辨率无效：{video_path}")
    if frame_count <= 0:
        raise RuntimeError(f"视频帧数无效：{video_path}")
    return {
        "path": str(video_path),
        "duration_s": duration,
        "format": fmt.get("format_name"),
        "size_bytes": int(fmt.get("size", video_path.stat().st_size)),
        "codec": stream.get("codec_name"),
        "width": int(stream["width"]),
        "height": int(stream["height"]),
        "r_frame_rate": stream.get("r_frame_rate"),
        "nb_frames": frame_count,
        "ffprobe_exit": 0,
        "ffmpeg_decode_exit": decode.returncode,
        "count_frames_exit": count.returncode,
    }


def _jump_support_summary() -> dict:
    """依据官方本地配置给出明确的跳跃支持结论，不生成假跳跃视频。"""
    mini_config = (
        PROJECT_ROOT
        / "external_models"
        / "rapid-locomotion-rl"
        / "mini_gym"
        / "envs"
        / "mini_cheetah"
        / "mini_cheetah_config.py"
    )
    base_config = (
        PROJECT_ROOT
        / "external_models"
        / "rapid-locomotion-rl"
        / "mini_gym"
        / "envs"
        / "base"
        / "legged_robot_config.py"
    )
    mini_text = mini_config.read_text(encoding="utf-8")
    base_text = base_config.read_text(encoding="utf-8")
    return {
        "jump_supported": False,
        "jump_video": None,
        "reason": (
            "官方 Mini Cheetah 配置只提供 vx/vy/yaw 命令；"
            "impulse_height_commands=False，且没有激活跳跃动作/奖励。"
        ),
        "evidence": {
            "mini_config": str(mini_config),
            "mini_has_vx": "_.lin_vel_x" in mini_text,
            "mini_has_vy": "_.lin_vel_y" in mini_text,
            "mini_has_yaw": "_.ang_vel_yaw" in mini_text,
            "impulse_height_commands_false": "impulse_height_commands = False" in base_text,
            "active_jump_reward": "_reward_jump" in base_text and "# def _reward_jump" not in base_text,
        },
    }


def _run_scenario(name: str, command: tuple[float, float, float]) -> dict:
    """运行一个独立场景；Gate 失败仍保留 summary 与媒体证据。"""
    vx, vy, yaw = command
    video_path = OUTPUT_ROOT / f"{name}.mp4"
    summary_path = OUTPUT_ROOT / f"{name}.summary.json"
    leg_stats_path = OUTPUT_ROOT / f"{name}.summary.leg_stats.json"
    for path in (video_path, summary_path, leg_stats_path):
        if path.exists():
            path.unlink()
    LOG_ROOT.mkdir(parents=True, exist_ok=True)
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    log_path = LOG_ROOT / f"{name}.log"
    command_args = [
        str(ISAAC_LAB),
        "-p",
        "training/scripts/play_rapid.py",
        "--headless",
        "--record-video",
        "--scenario-name",
        name,
        "--command-vx",
        str(vx),
        "--command-vy",
        str(vy),
        "--command-yaw",
        str(yaw),
        "--settle-steps",
        "200",
        "--motion-steps",
        "600",
        "--video-steps",
        "800",
        "--video-fps",
        "20",
        "--video-sample-every",
        "8",
        "--output",
        str(video_path),
    ]
    child_env = os.environ.copy()
    venv_bin = "/home/pma213x/.venvs/minicheetah-isaaclab/bin"
    child_env["PATH"] = venv_bin + os.pathsep + child_env.get("PATH", "")
    child_env.setdefault("PYTHONUNBUFFERED", "1")
    child_env.setdefault("NO_COLOR", "1")
    child_env["TERM"] = child_env.get("TERM") or "xterm-256color"
    with log_path.open("w", encoding="utf-8") as log_file:
        result = subprocess.run(
            command_args,
            cwd=PROJECT_ROOT,
            env=child_env,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
    if not summary_path.is_file():
        raise RuntimeError(
            f"场景 {name} 失败且缺少 summary：{log_path}，"
            f"exit={result.returncode}"
        )
    if not video_path.is_file():
        raise RuntimeError(
            f"场景 {name} 失败且缺少视频：{log_path}，"
            f"exit={result.returncode}"
        )
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    if result.returncode != 0 and summary.get("scenario_pass"):
        raise RuntimeError(
            f"场景 {name} summary 已通过但入口退出码为 "
            f"{result.returncode}，日志：{log_path}"
        )
    summary["media"] = _probe_media(video_path)
    summary["log"] = str(log_path)
    summary["leg_stats"] = str(leg_stats_path)
    summary["play_exit_code"] = result.returncode
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return summary


def main() -> int:
    """顺序运行全部方向场景并始终汇总媒体与 Gate 结果。"""
    LOG_ROOT.mkdir(parents=True, exist_ok=True)
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    jump_summary = _jump_support_summary()
    jump_path = OUTPUT_ROOT / "jump_support.json"
    jump_path.write_text(
        json.dumps(jump_summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    results = {}
    hard_errors = {}
    for name, command in SCENARIOS.items():
        print(f"[SCENARIO] start name={name} command={command}", flush=True)
        try:
            results[name] = _run_scenario(name, command)
            if results[name].get("scenario_pass"):
                print(
                    f"[SCENARIO] pass name={name} "
                    f"displacement="
                    f"{results[name]['scenario_metrics']['displacement_m']:.6f} "
                    f"yaw_change="
                    f"{results[name]['scenario_metrics']['yaw_change_deg']:.3f}",
                    flush=True,
                )
            else:
                print(
                    f"[SCENARIO] gate_failed name={name} "
                    f"errors={results[name].get('scenario_gate_errors')}",
                    flush=True,
                )
        except Exception as exc:
            hard_errors[name] = str(exc)
            print(f"[SCENARIO] hard_failed name={name} error={exc}", flush=True)
    aggregate = {
        "task": "YoboGo-Velocity-Flat-v0",
        "scenarios": results,
        "hard_errors": hard_errors,
        "jump_support": jump_summary,
        "all_scenarios_pass": (
            not hard_errors
            and len(results) == len(SCENARIOS)
            and all(item.get("scenario_pass") for item in results.values())
        ),
    }
    aggregate_path = OUTPUT_ROOT / "scenarios_summary.json"
    aggregate_path.write_text(
        json.dumps(aggregate, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"[SCENARIOS] summary={aggregate_path}", flush=True)
    print(f"[SCENARIOS] jump_support={jump_path}", flush=True)
    return 0 if aggregate["all_scenarios_pass"] else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"[SCENARIOS] FAILED {exc}", file=sys.stderr, flush=True)
        raise SystemExit(1)
