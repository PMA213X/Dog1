#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第三人称视角视频生成：用已训 PPO 策略驱动机器狗，从「后上方跟随相机」录制。

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
"""

from __future__ import annotations

import argparse
import base64
import math
import os
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


# ===========================================================================
# 运行回放
# ===========================================================================
def parse_args(argv: Optional[list] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="生成第三人称视角策略回放视频（Supervisor 跟随相机）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--model", type=str, required=True, metavar="ZIP",
                        help="已训练模型路径（.zip）")
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
    return parser.parse_args(argv)


def make_env(args: argparse.Namespace) -> Any:
    """创建带 rgb_array 能力的环境（Webots 开渲染，供相机出图）。"""
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
    if SB3_IMPORT_ERROR is not None or PPO is None:
        raise SystemExit(f"【错误】缺少 stable-baselines3：{SB3_IMPORT_ERROR}")

    model_path = Path(args.model)
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
    model = PPO.load(str(model_path), device=args.device)
    print(f"【模型】加载完成（{time.time() - t0:.1f}s） device={args.device}")

    env = make_env(args)
    print(f"【环境】已启动  turn={args.turn}  bridge_port={args.bridge_port}")

    deterministic = not args.stochastic
    frames: List[np.ndarray] = []
    ep_stats: List[Dict[str, Any]] = []

    try:
        for ep in range(1, args.episodes + 1):
            if args.turn:
                reset_out = env.reset()
            else:
                reset_out = env.reset()
            obs = reset_out[0] if isinstance(reset_out, tuple) else reset_out

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
                ep_reward += float(reward)
                ep_len += 1
                _record_state()

                frame = grab_third_frame(env)
                if frame is not None:
                    frames.append(frame)

                if args.max_steps is not None and ep_len >= args.max_steps:
                    break

            # --- 本集运动学统计 ---
            dist_path = 0.0
            for (x0, y0), (x1, y1) in zip(traj_xy, traj_xy[1:]):
                dist_path += math.hypot(x1 - x0, y1 - y0)
            disp = 0.0
            if traj_xy:
                disp = math.hypot(traj_xy[-1][0] - traj_xy[0][0],
                                  traj_xy[-1][1] - traj_xy[0][1])
            yaw_delta = 0.0
            if len(traj_yaw) >= 2:
                yaw_delta = wrap_angle(traj_yaw[-1] - traj_yaw[0])
            target_yaw = None
            if isinstance(info, dict) and "target_yaw" in info:
                target_yaw = float(info["target_yaw"])

            stats = {
                "episode": ep,
                "reward": ep_reward,
                "steps": ep_len,
                "path_length": dist_path,
                "displacement": disp,
                "yaw_delta_deg": math.degrees(yaw_delta),
                "target_yaw_deg": (math.degrees(target_yaw)
                                   if target_yaw is not None else None),
                "fallen": bool(info.get("fallen", False)) if isinstance(info, dict) else False,
            }
            ep_stats.append(stats)
            print(
                f"【回放】第 {ep}/{args.episodes} 集：奖励={ep_reward:.2f}  步数={ep_len}  "
                f"路径长={dist_path:.2f}m  位移={disp:.2f}m  "
                f"转向={stats['yaw_delta_deg']:+.1f}°  摔倒={stats['fallen']}"
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
        fps = max(5.0, len(frames) / args.min_duration)
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
    print(f"  关键帧        : {', '.join(result['keyframes'])}")


if __name__ == "__main__":
    main()
