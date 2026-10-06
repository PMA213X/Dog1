#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第三人称视角视频生成：用确定性 PPO 策略驱动机器狗，从「后上方跟随相机」录制。

方法说明（对应方案 A-2 / C）：
  在世界文件的 Robot children 里挂一台 DEF TP_CAM 相机（第三人称），由
  Supervisor 控制器 rl_agent 每帧把它摆到世界系「机身后上方、只跟随 yaw」，
  从而看清机身、腿部动作、前进/转向效果与场地环境。帧序列经 OpenCV 写 mp4，
  并同时导出关键帧 PNG。

用法示例：
    # P2 行走
    python play_third_person.py --model checkpoints/ppo_walk_final.zip \
        --video videos/walk_p2_third_person.mp4

    # P3 转向（45 维观测，固定左转目标）
    python play_third_person.py --model checkpoints/p3_turn_200000_steps.zip \
        --turn --heading left --video videos/walk_p3_third_person.mp4

    # P4 台阶（51 维观测，朝 +X 直行爬台阶；世界需用 parkour.wbt）
    python play_third_person.py --model checkpoints/p4_stairs_final.zip \
        --stairs --fixed-heading 0 --world webots-sim/worlds/parkour.wbt \
        --video videos/walk_p4_third_person.mp4

注意：本脚本自带独立世界 .parkour_view.wbt 与独立 TCP 端口（默认 11551），
不覆盖训练用世界，也不影响正在跑的训练进程。

为 yobogo_loco_jump_v1 增加：
  1. ``--env-id`` 动态加载未来 loco_jump 契约环境，不写死环境类名；
  2. ``--env-arg`` / ``--reset-option`` 分别配置构造器与每集任务；
  3. 写盘后默认执行 ffprobe + OpenCV 全帧解码校验，检查分辨率、时长与非黑帧；
  4. ``--mock-frames`` 只生成合成画面，用于正式模型就绪前的短链路测试。

正式第三人称录像模板（三段串行执行，避免桥端口冲突）：

    # 移动
    python play_third_person.py \\
      --model checkpoints/yobogo_loco_jump_v1.zip \\
      --env-id loco_jump_env:make_env --env-arg tag=S2_command \\
      --reset-option 'command=[0.4,0,0]' --reset-option target_yaw=0.0 \\
      --video videos/yobogo_loco_jump_v1_move.mp4 \\
      --episodes 1 --max-steps 500 --min-duration 5

    # 转向
    python play_third_person.py \\
      --model checkpoints/yobogo_loco_jump_v1.zip \\
      --env-id loco_jump_env:make_env --env-arg tag=S2_command \\
      --reset-option 'command=[0,0,0.8]' --reset-option target_yaw=0.0 \\
      --video videos/yobogo_loco_jump_v1_turn.mp4 \\
      --episodes 1 --max-steps 500 --min-duration 5

    # 跳跃
    python play_third_person.py \\
      --model checkpoints/yobogo_loco_jump_v1.zip \\
      --env-id loco_jump_env:make_env --env-arg tag=S3_jump \\
      --reset-option 'command=[0,0,0]' --reset-option jump_request=true \\
      --video videos/yobogo_loco_jump_v1_jump.mp4 \\
      --episodes 1 --max-steps 500 --min-duration 5
"""

from __future__ import annotations

import argparse
import base64
import importlib
import inspect
import json
import math
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

# 保证同目录 config / walk_env / walk_env_turn 可导入
_RL_DIR = Path(__file__).resolve().parent
if str(_RL_DIR) not in sys.path:
    sys.path.insert(0, str(_RL_DIR))

from config import ACTION_DIM, DEFAULT_DEVICE  # noqa: E402

try:
    from stable_baselines3 import PPO
except ImportError as _sb3_exc:  # pragma: no cover
    PPO = None  # type: ignore[assignment,misc]
    SB3_IMPORT_ERROR: Optional[ImportError] = _sb3_exc
else:
    SB3_IMPORT_ERROR = None


# ---------------------------------------------------------------------------
# 路径常量
# ---------------------------------------------------------------------------
WEBOTS_SIM_DIR = _RL_DIR.parent                      # webots-sim/
WORLDS_DIR = WEBOTS_SIM_DIR / "worlds"
DEFAULT_WORLD = WORLDS_DIR / "parkour_dev.wbt"
VIEW_WORLD = WORLDS_DIR / ".parkour_view.wbt"        # 独立世界，不覆盖训练用
REPO_ROOT = WEBOTS_SIM_DIR.parent
DEFAULT_VIDEO = REPO_ROOT / "videos" / "walk_p2_third_person.mp4"
DEFAULT_FRAMES_DIR = REPO_ROOT / "videos" / "frames"
DEFAULT_BRIDGE_PORT = 11551                          # 避开训练默认端口 11451

# 第三人称相机默认机位（与 rl_agent.TP_CAM_* 对应，仅作世界文件初值）
TP_CAM_TRANSLATION = (-2.5, 0.0, 1.15)
TP_CAM_ROTATION = (0.0, 1.0, 0.0, 0.45)   # 绕 +Y 俯视机身


# ===========================================================================
# 世界文件：生成带第三人称相机的独立视图世界
# ===========================================================================
def build_view_world(src_world: Path, dst_world: Path) -> Path:
    """由普通世界生成第三人称视图世界（写入 DEF TP_CAM，controller=rl_agent）。

    与 walk_env._patch_rl_world 的约定一致：controller 改为 rl_agent 并开
    supervisor，这样可用现有 TCP 桥；另外在 Robot children 里插入跟随相机。
    """
    text = src_world.read_text(encoding="utf-8")

    # 1) 控制器 → rl_agent
    if 'controller "rl_agent"' not in text:
        import re
        if 'controller "' in text:
            text = re.sub(r'controller\s+"[^"]*"', 'controller "rl_agent"', text, count=1)
        else:
            raise ValueError(f"世界文件中找不到 controller 字段：{src_world}")
    # 2) Supervisor 权限
    if "supervisor TRUE" not in text:
        text = text.replace(
            'controller "rl_agent"',
            'supervisor TRUE\n  controller "rl_agent"',
            1,
        )

    # 3) 插入第三人称相机（若尚未存在）
    if "DEF TP_CAM" not in text:
        anchor = "    # ---- 机体视觉件"
        tp_block = f"""    # ---- 第三人称跟随相机（play_third_person 专用，不影响物理）----
    DEF TP_CAM Transform {{
      translation {TP_CAM_TRANSLATION[0]} {TP_CAM_TRANSLATION[1]} {TP_CAM_TRANSLATION[2]}
      rotation {TP_CAM_ROTATION[0]} {TP_CAM_ROTATION[1]} {TP_CAM_ROTATION[2]} {TP_CAM_ROTATION[3]}
      children [
        Camera {{
          name "third_camera"
          width 640
          height 480
          fieldOfView 1.0
          noise 0
          motionBlur 0
        }}
      ]
    }}
"""
        if anchor in text:
            text = text.replace(anchor, tp_block + anchor, 1)
        else:
            raise ValueError(
                "世界文件里找不到锚点 '    # ---- 机体视觉件'，无法插入第三人称相机。"
            )

    dst_world.parent.mkdir(parents=True, exist_ok=True)
    dst_world.write_text(text, encoding="utf-8")
    return dst_world


# ===========================================================================
# 帧抓取 / 视频写出
# ===========================================================================
def grab_third_frame(env: Any) -> Optional[np.ndarray]:
    """通过 TCP 桥要一帧第三人称 RGB（不推进仿真）。"""
    try:
        env._send({"type": "get_rgb", "camera": "third"})
        msg = env._recv()
    except Exception:  # noqa: BLE001 - 渲染不可用时安静降级
        return None
    if not isinstance(msg, dict) or msg.get("type") != "rgb":
        return None
    w = int(msg.get("w", 0))
    h = int(msg.get("h", 0))
    b64 = msg.get("data", "")
    if not b64 or w <= 0 or h <= 0:
        return None
    buf = np.frombuffer(base64.b64decode(b64), dtype=np.uint8)
    if buf.size != w * h * 3:
        return None
    return buf.reshape(h, w, 3)


def write_video(frames: List[np.ndarray], out_path: Path, fps: float) -> None:
    """帧序列 → mp4（OpenCV mp4v）。"""
    if not frames:
        raise RuntimeError("没有采集到任何帧，无法写视频")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    import cv2

    first = np.asarray(frames[0])
    height, width = first.shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(out_path), fourcc, float(fps), (width, height))
    if not writer.isOpened():
        raise RuntimeError(f"OpenCV VideoWriter 打开失败：{out_path}")
    for frame in frames:
        arr = np.asarray(frame)
        writer.write(cv2.cvtColor(arr, cv2.COLOR_RGB2BGR))
    writer.release()


def save_keyframes(
    frames: List[np.ndarray], frames_dir: Path, stem: str, n: int = 3
) -> List[Path]:
    """均匀取 n 张关键帧存 PNG，返回路径列表。"""
    frames_dir.mkdir(parents=True, exist_ok=True)
    import cv2

    if not frames:
        return []
    n = max(1, min(n, len(frames)))
    idxs = [int(round(i * (len(frames) - 1) / max(1, n - 1))) for i in range(n)]
    # 去重（帧数少于 n 时）
    seen = set()
    uniq: List[int] = []
    for i in idxs:
        if i not in seen:
            seen.add(i)
            uniq.append(i)
    tags = ["start", "mid", "end"] if len(uniq) == 3 else [f"f{i:02d}" for i in range(len(uniq))]
    paths: List[Path] = []
    for tag, i in zip(tags, uniq):
        p = frames_dir / f"{stem}_{tag}.png"
        cv2.imwrite(str(p), cv2.cvtColor(np.asarray(frames[i]), cv2.COLOR_RGB2BGR))
        paths.append(p)
    return paths


def parse_key_values(items: List[str], label: str) -> Dict[str, Any]:
    """把 KEY=VALUE 列表转换为字典，值优先按 JSON 解析。"""
    result: Dict[str, Any] = {}
    for item in items:
        key, raw = item.split("=", 1)
        key = key.strip()
        if not key:
            raise ValueError(f"{label} 中键名为空：{item!r}")
        try:
            result[key] = json.loads(raw)
        except json.JSONDecodeError:
            result[key] = raw
    return result


def resolve_env_symbol(env_id: str) -> Any:
    """动态解析 module:symbol 或 module.Class 环境入口。"""
    if ":" in env_id:
        module_name, symbol_name = env_id.split(":", 1)
    elif "." in env_id:
        module_name, symbol_name = env_id.rsplit(".", 1)
    else:
        module_name, symbol_name = env_id, ""
    module = importlib.import_module(module_name)
    if symbol_name:
        return getattr(module, symbol_name)

    for name in (
        "make_env",
        "make_loco_jump_env",
        "make",
        "LocoJumpEnv",
        "QuadrupedLocoJumpEnv",
        "QuadrupedJumpEnv",
    ):
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
        f"无法从 {module_name!r} 自动确定环境符号；请显式给 --env-id {module_name}:SYMBOL"
    )


def filter_constructor_kwargs(
    symbol: Any, requested: Dict[str, Any]
) -> Tuple[Dict[str, Any], List[str]]:
    """按环境构造器签名过滤参数，支持显式 **kwargs 工厂。"""
    try:
        signature = inspect.signature(symbol)
    except (TypeError, ValueError):
        return dict(requested), []
    if any(p.kind == inspect.Parameter.VAR_KEYWORD for p in signature.parameters.values()):
        return dict(requested), []
    accepted = {
        key: value for key, value in requested.items()
        if key in signature.parameters
    }
    skipped = sorted(set(requested) - set(accepted))
    return accepted, skipped


def reset_with_options(env: Any, seed: int, options: Dict[str, Any]) -> Tuple[Any, Dict[str, Any]]:
    """带 seed/options reset，并兼容旧环境的简化签名。"""
    try:
        output = env.reset(seed=seed, options=options)
    except TypeError:
        try:
            output = env.reset(seed=seed)
        except TypeError:
            output = env.reset()
    if isinstance(output, tuple) and len(output) == 2:
        obs, info = output
        return obs, info if isinstance(info, dict) else {}
    return output, {}


def make_mock_frames(task: str, count: int, width: int, height: int) -> List[np.ndarray]:
    """生成移动、转向、跳跃三种合成画面，仅用于录像写盘/校验链路测试。"""
    import cv2

    frames: List[np.ndarray] = []
    # 背景带亮度梯度，保证解码后的帧不是纯黑，且相邻帧内容确实变化。
    gradient = np.linspace(24, 96, width, dtype=np.uint8)
    background = np.empty((height, width, 3), dtype=np.uint8)
    background[:] = gradient[None, :, None]
    background[:, :, 1] = np.clip(background[:, :, 1].astype(np.int16) + 24, 0, 255)

    for index in range(count):
        frame = background.copy()
        progress = index / max(1, count - 1)
        if task == "move":
            x = int(60 + progress * (width - 140))
            y = height // 2
        elif task == "turn":
            x = width // 2
            y = height // 2
        else:
            x = int(80 + progress * (width - 160))
            arc = max(0.0, math.sin(progress * math.pi))
            y = int(height * 0.68 - arc * height * 0.34)
        # 简化的机器狗轮廓，用于人工快速确认画面不是空帧。
        cv2.rectangle(frame, (x - 42, y - 22), (x + 42, y + 22), (235, 215, 70), -1)
        cv2.rectangle(frame, (x - 34, y + 20), (x - 18, y + 58), (60, 220, 230), -1)
        cv2.rectangle(frame, (x + 18, y + 20), (x + 34, y + 58), (60, 220, 230), -1)
        if task == "turn":
            angle = int(progress * 360)
            cv2.ellipse(
                frame,
                (width // 2, height // 2),
                (145, 115),
                0,
                -90,
                angle - 90,
                (40, 80, 245),
                5,
            )
        label = f"MOCK {task.upper()} {index + 1}/{count}"
        cv2.putText(
            frame, label, (18, 34), cv2.FONT_HERSHEY_SIMPLEX,
            0.85, (255, 255, 255), 2, cv2.LINE_AA,
        )
        frames.append(frame)
    return frames


def _parse_rate(raw: Any) -> Optional[float]:
    """解析 ffprobe 的有理数帧率。"""
    if raw in (None, "", "N/A", "0/0"):
        return None
    try:
        text = str(raw)
        if "/" in text:
            numerator, denominator = text.split("/", 1)
            denominator_float = float(denominator)
            return float(numerator) / denominator_float if denominator_float else None
        return float(text)
    except (TypeError, ValueError, ZeroDivisionError):
        return None


def verify_video(
    video_path: Path,
    *,
    expected_width: int,
    expected_height: int,
    min_duration: float,
    min_nonblack_ratio: float,
) -> Dict[str, Any]:
    """用 ffprobe + OpenCV 全帧解码校验分辨率、时长与非黑帧。"""
    if not video_path.is_file() or video_path.stat().st_size <= 0:
        raise RuntimeError(f"视频不存在或为空：{video_path}")
    ffprobe = shutil.which("ffprobe")
    if ffprobe is None:
        raise RuntimeError("找不到 ffprobe，无法校验视频容器")
    command = [
        ffprobe, "-v", "error", "-select_streams", "v:0",
        "-show_entries", "stream=codec_name,width,height,r_frame_rate,avg_frame_rate,nb_frames",
        "-show_entries", "format=duration",
        "-of", "json", str(video_path),
    ]
    completed = subprocess.run(
        command, check=False, capture_output=True, text=True, timeout=30
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"ffprobe 校验失败（退出码 {completed.returncode}）：{completed.stderr.strip()}"
        )
    try:
        metadata = json.loads(completed.stdout)
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"ffprobe 输出不是合法 JSON：{exc}") from exc
    streams = metadata.get("streams") or []
    if not streams:
        raise RuntimeError("ffprobe 未找到视频流")
    stream = streams[0]
    ffprobe_width = int(stream.get("width", 0))
    ffprobe_height = int(stream.get("height", 0))
    duration_raw = (metadata.get("format") or {}).get("duration") or stream.get("duration")
    if duration_raw in (None, "", "N/A"):
        raise RuntimeError("ffprobe 未返回视频时长")
    ffprobe_duration = float(duration_raw)
    fps = _parse_rate(stream.get("avg_frame_rate")) or _parse_rate(
        stream.get("r_frame_rate")
    )

    import cv2

    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise RuntimeError(f"OpenCV 无法打开视频：{video_path}")
    decoded = 0
    decoded_width = 0
    decoded_height = 0
    nonblack = 0
    decode_errors = 0
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            decoded += 1
            if frame is None or frame.size == 0:
                decode_errors += 1
                continue
            if decoded == 1:
                decoded_height, decoded_width = frame.shape[:2]
            elif (decoded_height, decoded_width) != frame.shape[:2]:
                decode_errors += 1
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # “非黑帧”要求画面同时具有足够的最大亮度和纹理变化。
            if float(gray.max()) >= 16.0 and float(gray.std()) >= 1.0:
                nonblack += 1
    finally:
        capture.release()

    nonblack_ratio = nonblack / decoded if decoded else 0.0
    resolution_ok = True
    if expected_width > 0 and ffprobe_width != expected_width:
        resolution_ok = False
    if expected_height > 0 and ffprobe_height != expected_height:
        resolution_ok = False
    if (ffprobe_width, ffprobe_height) != (decoded_width, decoded_height):
        resolution_ok = False
    checks = {
        "ffprobe": True,
        "opencv_decode": decoded > 0 and decode_errors == 0,
        "resolution": resolution_ok and decoded_width > 0,
        "duration": ffprobe_duration + 1e-6 >= min_duration,
        "nonblack": nonblack_ratio + 1e-12 >= min_nonblack_ratio,
    }
    result: Dict[str, Any] = {
        **checks,
        "passed": all(checks.values()),
        "codec": stream.get("codec_name"),
        "ffprobe_resolution": [ffprobe_width, ffprobe_height],
        "decoded_resolution": [decoded_width, decoded_height],
        "ffprobe_duration": ffprobe_duration,
        "fps": fps,
        "ffprobe_frames": stream.get("nb_frames"),
        "decoded_frames": decoded,
        "decode_errors": decode_errors,
        "nonblack_frames": nonblack,
        "black_frames": decoded - nonblack,
        "nonblack_ratio": nonblack_ratio,
        "expected_resolution": [expected_width, expected_height],
        "min_duration": min_duration,
        "min_nonblack_ratio": min_nonblack_ratio,
    }
    if not result["passed"]:
        failed = [name for name, passed in checks.items() if not passed]
        raise RuntimeError(f"视频校验失败，未通过项：{', '.join(failed)}；结果={result}")
    return result


# ===========================================================================
# 运行回放
# ===========================================================================
def parse_args(argv: Optional[list] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="生成第三人称视角策略回放视频（Supervisor 跟随相机）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--model", type=str, default=None, metavar="ZIP",
                        help="已训练模型路径（.zip）；--mock-frames 模式可省略")
    parser.add_argument("--video", type=str, default=str(DEFAULT_VIDEO),
                        metavar="OUT.mp4", help="输出视频路径")
    parser.add_argument("--frames-dir", type=str, default=str(DEFAULT_FRAMES_DIR),
                        help="关键帧 PNG 输出目录")
    parser.add_argument("--episodes", type=int, default=1, help="回放 episode 数")
    parser.add_argument("--max-steps", type=int, default=500,
                        help="单集最大步数（50Hz×500=10s 仿真时间）")
    parser.add_argument("--fps", type=float, default=50.0,
                        help="视频帧率；若总时长不足 10s 会自动降帧率补齐")
    parser.add_argument("--min-duration", type=float, default=10.0,
                        help="视频最短时长（秒）")
    parser.add_argument("--keyframes", type=int, default=3, help="导出关键帧数量")
    parser.add_argument("--turn", action="store_true",
                        help="使用 P3 转向环境（45 维观测）")
    parser.add_argument("--stairs", action="store_true",
                        help="使用 P4 台阶环境（51 维观测，世界需用 parkour.wbt）")
    parser.add_argument("--heading", type=str, default="left",
                        choices=["left", "right", "random"],
                        help="P3 目标航向模式（left=+π/2, right=-π/2）")
    parser.add_argument("--fixed-heading", type=float, default=None,
                        help="固定目标航向（弧度）。0=朝 +X 直行（P4 爬台阶用）")
    parser.add_argument("--step-height", type=float, default=None,
                        choices=[0.05, 0.08, 0.12],
                        help="P4 固定台阶高度（m）；不给则每集随机三档")
    parser.add_argument("--device", type=str, default="cpu",
                        help="计算设备（MLP 策略用 cpu 即可）")
    parser.add_argument("--bridge-port", type=int, default=DEFAULT_BRIDGE_PORT,
                        help="TCP 桥端口（避开训练用的 11451）")
    parser.add_argument("--world", type=str, default=str(DEFAULT_WORLD),
                        help="源世界文件")
    parser.add_argument("--stochastic", action="store_true",
                        help="随机采样动作（默认确定性策略）")
    parser.add_argument("--env-id", type=str, default=None,
                        metavar="MODULE:SYMBOL",
                        help="动态环境工厂/类；不给则沿用旧 turn/stairs 分支")
    parser.add_argument("--env-arg", action="append", default=[],
                        metavar="KEY=VALUE",
                        help="环境构造器附加参数，可重复；VALUE 优先按 JSON 解析")
    parser.add_argument("--reset-option", action="append", default=[],
                        metavar="KEY=VALUE",
                        help="每集 reset options 附加参数，可重复")
    parser.add_argument("--seed", type=int, default=20261001,
                        help="首集随机种子；每集按 1 递增")
    parser.add_argument("--mock-frames", type=int, default=0, metavar="N",
                        help="生成 N 张合成帧而不加载模型/Webots，仅用于短链路测试")
    parser.add_argument("--mock-task", choices=["move", "turn", "jump"],
                        default="move", help="合成帧展示的测试任务")
    parser.add_argument("--expected-width", type=int, default=640,
                        help="视频校验期望宽度；0 表示不检查")
    parser.add_argument("--expected-height", type=int, default=480,
                        help="视频校验期望高度；0 表示不检查")
    parser.add_argument("--min-nonblack-ratio", type=float, default=0.50,
                        help="OpenCV 解码后非黑帧最低比例")
    parser.add_argument("--skip-video-verify", action="store_true",
                        help="跳过 ffprobe/OpenCV 视频完整性校验（不推荐）")
    parsed = parser.parse_args(argv)
    if parsed.episodes <= 0 or parsed.max_steps <= 0:
        parser.error("--episodes 和 --max-steps 必须大于 0")
    if parsed.mock_frames < 0:
        parser.error("--mock-frames 不能小于 0")
    if parsed.mock_frames == 0 and not parsed.model:
        parser.error("必须提供 --model，或使用 --mock-frames 做短链路测试")
    if parsed.mock_frames > 0 and parsed.model:
        parser.error("--mock-frames 与 --model 不能同时使用")
    if parsed.min_duration <= 0 or parsed.fps <= 0:
        parser.error("--min-duration 和 --fps 必须大于 0")
    if not 0.0 <= parsed.min_nonblack_ratio <= 1.0:
        parser.error("--min-nonblack-ratio 必须在 [0,1] 内")
    for items, label in ((parsed.env_arg, "--env-arg"),
                         (parsed.reset_option, "--reset-option")):
        for item in items:
            if "=" not in item:
                parser.error(f"{label} 需要 KEY=VALUE 格式：{item!r}")
    return parsed


def make_env(args: argparse.Namespace) -> Any:
    """创建带 rgb_array 能力的环境（Webots 开渲染，供相机出图）。"""
    if args.env_id:
        symbol = resolve_env_symbol(args.env_id)
        requested: Dict[str, Any] = {
            "render_mode": "rgb_array",
            "world": VIEW_WORLD,
            "bridge_port": args.bridge_port,
            **parse_key_values(args.env_arg, "--env-arg"),
        }
        requested = {key: value for key, value in requested.items() if value is not None}
        accepted, skipped = filter_constructor_kwargs(symbol, requested)
        if skipped:
            print(f"【环境】构造器未接收的参数：{', '.join(skipped)}")
        env = symbol(**accepted)
        if not hasattr(env, "reset") or not hasattr(env, "step"):
            raise TypeError(f"--env-id {args.env_id!r} 创建的对象不是 Gymnasium 风格环境")
        return env
    if args.stairs:
        from walk_env_stairs import QuadrupedStairsEnv

        return QuadrupedStairsEnv(
            render_mode="rgb_array",
            world=VIEW_WORLD,
            bridge_port=args.bridge_port,
            heading_mode=args.heading,
            fixed_heading=args.fixed_heading,
            step_height=args.step_height,
        )
    if args.turn:
        from walk_env_turn import QuadrupedTurnEnv

        return QuadrupedTurnEnv(
            render_mode="rgb_array",
            world=VIEW_WORLD,
            bridge_port=args.bridge_port,
            heading_mode=args.heading,
            fixed_heading=args.fixed_heading,
        )
    from walk_env import QuadrupedWalkEnv

    return QuadrupedWalkEnv(
        render_mode="rgb_array",
        world=VIEW_WORLD,
        bridge_port=args.bridge_port,
    )


def wrap_angle(a: float) -> float:
    """角度 wrap 到 [-π, π]。"""
    return float((a + math.pi) % (2.0 * math.pi) - math.pi)


def run(args: argparse.Namespace) -> Dict[str, Any]:
    frames: List[np.ndarray] = []
    ep_stats: List[Dict[str, Any]] = []

    if args.mock_frames > 0:
        # 合成帧只验证“写 mp4 → ffprobe → 解码 → 非黑帧”链路，
        # 不加载模型、不启动 Webots，也不具备任何验收效力。
        frames = make_mock_frames(
            args.mock_task, args.mock_frames, args.expected_width or 640,
            args.expected_height or 480,
        )
        ep_stats.append(
            {
                "episode": 1,
                "reward": 0.0,
                "steps": args.mock_frames,
                "path_length": 0.0,
                "displacement": 0.0,
                "yaw_delta_deg": 0.0,
                "target_yaw_deg": None,
                "command_error": None,
                "jump_success": False,
                "terrain_success": False,
                "fallen": False,
                "mock": True,
            }
        )
        print(
            f"【警告】正在生成 mock-{args.mock_task} 短片段（{args.mock_frames} 帧），"
            "不加载模型/不启动 Webots，结果不可作为验收"
        )
    else:
        if SB3_IMPORT_ERROR is not None or PPO is None:
            raise SystemExit(f"【错误】缺少 stable-baselines3：{SB3_IMPORT_ERROR}")

        model_path = Path(args.model or "")
        if not model_path.is_file():
            raise SystemExit(f"【错误】模型不存在：{model_path}")

        src_world = Path(args.world)
        if not src_world.is_file():
            raise SystemExit(f"【错误】世界文件不存在：{src_world}")

        # 1) 生成独立视图世界（含第三人称相机）
        build_view_world(src_world, VIEW_WORLD)
        print(f"【世界】已生成第三人称视图世界：{VIEW_WORLD}")

        print(f"【模型】加载 {model_path} ...")
        t0 = time.time()
        contract = None
        if args.env_id:
            try:
                import loco_jump_contract as contract_module

                contract = contract_module
                # yobogo_loco_jump_v1 评估明确拒绝旧 PPO checkpoint。
                model_path = contract.validate_checkpoint_path(model_path)
            except (ImportError, ValueError) as exc:
                raise SystemExit(f"【错误】checkpoint 契约校验失败：{exc}") from exc
        model = PPO.load(str(model_path), device=args.device)
        print(f"【模型】加载完成（{time.time() - t0:.1f}s） device={args.device}")

        env = make_env(args)
        if contract is not None:
            obs_shape = tuple(
                getattr(getattr(env, "observation_space", None), "shape", ())
            )
            action_shape = tuple(
                getattr(getattr(env, "action_space", None), "shape", ())
            )
            expected_obs_shape = (int(contract.OBS_DIM),)
            expected_action_shape = (int(contract.ACTION_DIM),)
            if obs_shape and obs_shape != expected_obs_shape:
                raise SystemExit(
                    f"【错误】环境观测不符合契约：env={obs_shape} "
                    f"expected={expected_obs_shape}"
                )
            if action_shape and action_shape != expected_action_shape:
                raise SystemExit(
                    f"【错误】环境动作不符合契约：env={action_shape} "
                    f"expected={expected_action_shape}"
                )
            model_obs_shape = tuple(
                getattr(getattr(model, "observation_space", None), "shape", ())
            )
            model_action_shape = tuple(
                getattr(getattr(model, "action_space", None), "shape", ())
            )
            if model_obs_shape != expected_obs_shape:
                raise SystemExit(
                    f"【错误】模型观测不符合契约：model={model_obs_shape} "
                    f"expected={expected_obs_shape}"
                )
            if model_action_shape != expected_action_shape:
                raise SystemExit(
                    f"【错误】模型动作不符合契约：model={model_action_shape} "
                    f"expected={expected_action_shape}"
                )
        print(
            f"【环境】已启动  env_id={args.env_id or 'legacy'}  "
            f"turn={args.turn}  bridge_port={args.bridge_port}"
        )
        deterministic = not args.stochastic
        reset_options = parse_key_values(args.reset_option, "--reset-option")

        try:
            for ep in range(1, args.episodes + 1):
                obs, reset_info = reset_with_options(
                    env, args.seed + ep - 1, reset_options
                )
                infos: List[Dict[str, Any]] = [dict(reset_info)]
                ep_reward = 0.0
                ep_len = 0
                done = False
                # 轨迹记录：世界系位置与航向
                traj_xy: List[Tuple[float, float]] = []
                traj_yaw: List[float] = []

                def _record_state() -> None:
                    st = getattr(env, "_last_state", {}) or {}
                    try:
                        traj_xy.append((float(st.get("x", 0.0)), float(st.get("y", 0.0))))
                        traj_yaw.append(float(st.get("rpy", [0, 0, 0])[2]))
                    except Exception:  # noqa: BLE001
                        pass

                _record_state()

                while not done:
                    action, _ = model.predict(obs, deterministic=deterministic)
                    step_out = env.step(action)
                    if len(step_out) == 5:
                        obs, reward, terminated, truncated, info = step_out
                        done = bool(terminated) or bool(truncated)
                    else:
                        obs, reward, done, info = step_out
                        done = bool(done)
                    info_dict = info if isinstance(info, dict) else {}
                    infos.append(info_dict)
                    ep_reward += float(reward)
                    ep_len += 1
                    _record_state()

                    frame = grab_third_frame(env)
                    if frame is not None:
                        frames.append(frame)

                    if args.max_steps is not None and ep_len >= args.max_steps:
                        break

                # --- 本集运动学与任务指标 ---
                dist_path = 0.0
                for (x0, y0), (x1, y1) in zip(traj_xy, traj_xy[1:]):
                    dist_path += math.hypot(x1 - x0, y1 - y0)
                disp = 0.0
                if traj_xy:
                    disp = math.hypot(
                        traj_xy[-1][0] - traj_xy[0][0],
                        traj_xy[-1][1] - traj_xy[0][1],
                    )
                yaw_delta = 0.0
                if len(traj_yaw) >= 2:
                    yaw_delta = wrap_angle(traj_yaw[-1] - traj_yaw[0])
                target_yaw = None
                if "target_yaw" in infos[-1]:
                    target_yaw = float(infos[-1]["target_yaw"])
                jump_success = any(
                    bool(item.get(key, False))
                    for item in infos
                    for key in ("jump_success", "jumped", "jump_succeeded")
                )
                terrain_success = any(
                    bool(item.get(key, False))
                    for item in infos
                    for key in ("terrain_success", "terrain_ok", "obstacle_success")
                )
                fallen = any(
                    bool(item.get(key, False))
                    for item in infos
                    for key in ("fallen", "is_fallen", "fell")
                )
                command_error = next(
                    (
                        float(item[key])
                        for item in reversed(infos)
                        for key in ("command_error", "cmd_error", "heading_error", "yaw_error")
                        if key in item and item[key] is not None
                    ),
                    None,
                )

                stats = {
                    "episode": ep,
                    "reward": ep_reward,
                    "steps": ep_len,
                    "path_length": dist_path,
                    "displacement": disp,
                    "yaw_delta_deg": math.degrees(yaw_delta),
                    "target_yaw_deg": (
                        math.degrees(target_yaw) if target_yaw is not None else None
                    ),
                    "command_error": command_error,
                    "jump_success": jump_success,
                    "terrain_success": terrain_success,
                    "fallen": fallen,
                    "mock": False,
                }
                ep_stats.append(stats)
                print(
                    f"【回放】第 {ep}/{args.episodes} 集：奖励={ep_reward:.2f}  "
                    f"步数={ep_len}  路径长={dist_path:.2f}m  位移={disp:.2f}m  "
                    f"转向={stats['yaw_delta_deg']:+.1f}°  命令误差={command_error}  "
                    f"跳跃成功={jump_success}  摔倒={fallen}"
                )
        finally:
            close = getattr(env, "close", None)
            if callable(close):
                close()

    if not frames:
        raise SystemExit("【错误】未采集到任何第三人称帧，无法生成视频")

    # 2) 帧率：保证视频不少于 min_duration 秒
    fps = float(args.fps)
    duration = len(frames) / fps
    if duration < args.min_duration and len(frames) > 0:
        fps = max(1.0, len(frames) / args.min_duration)
        duration = len(frames) / fps
        print(f"【视频】为满足 ≥{args.min_duration}s，帧率调整为 {fps:.2f}")

    video_path = Path(args.video)
    write_video(frames, video_path, fps=fps)
    size_mb = video_path.stat().st_size / 1e6
    h, w = frames[0].shape[:2]
    print(
        f"【视频】已保存：{video_path}  大小={size_mb:.2f}MB  "
        f"分辨率={w}x{h}  帧数={len(frames)}  帧率={fps:.1f}  时长={duration:.1f}s"
    )

    verification: Optional[Dict[str, Any]] = None
    if args.skip_video_verify:
        print("【视频】已按参数跳过 ffprobe/OpenCV 完整性校验")
    else:
        verification = verify_video(
            video_path,
            expected_width=args.expected_width,
            expected_height=args.expected_height,
            min_duration=args.min_duration,
            min_nonblack_ratio=args.min_nonblack_ratio,
        )
        print(
            "【校验】通过  "
            f"ffprobe={verification['ffprobe_resolution']}  "
            f"时长={verification['ffprobe_duration']:.3f}s  "
            f"解码={verification['decoded_frames']}帧  "
            f"非黑帧={verification['nonblack_frames']}/{verification['decoded_frames']} "
            f"({verification['nonblack_ratio']:.1%})"
        )

    # 3) 关键帧 PNG
    stem = video_path.stem
    pngs = save_keyframes(frames, Path(args.frames_dir), stem, n=args.keyframes)
    for p in pngs:
        print(f"【关键帧】{p}")

    return {
        "video": str(video_path),
        "size_mb": size_mb,
        "resolution": (w, h),
        "frames": len(frames),
        "fps": fps,
        "duration": duration,
        "keyframes": [str(p) for p in pngs],
        "episodes": ep_stats,
        "verification": verification,
        "mock": args.mock_frames > 0,
    }


def main(argv: Optional[list] = None) -> None:
    args = parse_args(argv)
    result = run(args)

    # ---- 汇总报告 ----
    print("-" * 60)
    print("【汇总】第三人称视频生成完成")
    eps = result["episodes"]
    if eps:
        avg_path = sum(e["path_length"] for e in eps) / len(eps)
        avg_disp = sum(e["displacement"] for e in eps) / len(eps)
        avg_yaw = sum(e["yaw_delta_deg"] for e in eps) / len(eps)
        total_steps = sum(e["steps"] for e in eps)
        print(f"  总步数        : {total_steps}")
        print(f"  平均路径长度  : {avg_path:.2f} m")
        print(f"  平均直线位移  : {avg_disp:.2f} m")
        print(f"  平均转向角度  : {avg_yaw:+.1f}°")
        for e in eps:
            extra = ""
            if e.get("target_yaw_deg") is not None:
                extra = f"  目标航向={e['target_yaw_deg']:+.1f}°"
            print(
                f"  第 {e['episode']} 集: 步数={e['steps']}  "
                f"路径={e['path_length']:.2f}m  位移={e['displacement']:.2f}m  "
                f"转向={e['yaw_delta_deg']:+.1f}°{extra}"
            )
    print(f"  视频          : {result['video']}  ({result['size_mb']:.2f} MB)")
    print(
        f"  分辨率/时长   : {result['resolution'][0]}x{result['resolution'][1]}"
        f" / {result['duration']:.1f}s @{result['fps']:.1f}fps"
    )
    verification = result.get("verification")
    if verification:
        print(
            "  视频校验      : 通过（ffprobe + OpenCV 解码，非黑帧 "
            f"{verification['nonblack_frames']}/{verification['decoded_frames']}）"
        )
    else:
        print("  视频校验      : 已跳过")
    if result.get("mock"):
        print("  警告          : mock 短片段只验证录像链路，禁止作为模型验收")
    print(f"  关键帧        : {', '.join(result['keyframes'])}")


if __name__ == "__main__":
    main()
