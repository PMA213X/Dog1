#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""加载 PPO 模型，在四足机器狗行走环境中回放 / 演示。

用法示例：
    python play.py --model checkpoints/ppo_walk_final.zip
    python play.py --model checkpoints/ppo_walk_final.zip --episodes 3 --render
    python play.py --model checkpoints/ppo_walk_final.zip --video walk.mp4

环境接口由 webots-sim/rl/walk_env.py 提供（另一模块编写）：
    from walk_env import QuadrupedWalkEnv
    obs 42 维 float32，action 12 维 [-1, 1]
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Any, List, Optional

# 保证同目录下的 config.py / walk_env.py 可以被 import
_RL_DIR = Path(__file__).resolve().parent
if str(_RL_DIR) not in sys.path:
    sys.path.insert(0, str(_RL_DIR))

# ---------------------------------------------------------------------------
# 本地配置
# ---------------------------------------------------------------------------
from config import ACTION_DIM, DEFAULT_DEVICE, OBS_DIM  # noqa: E402

# ---------------------------------------------------------------------------
# 环境接口 import（walk_env.py 若尚未就绪，--dry-run 仍可跑通参数解析）
# ---------------------------------------------------------------------------
try:
    from walk_env import QuadrupedWalkEnv  # noqa: F401
except ImportError as _walk_env_exc:  # pragma: no cover
    QuadrupedWalkEnv = None  # type: ignore[assignment,misc]
    WALK_ENV_IMPORT_ERROR: Optional[ImportError] = _walk_env_exc
else:
    WALK_ENV_IMPORT_ERROR = None

# ---------------------------------------------------------------------------
# 第三方依赖：仅在真正回放时强制要求
# ---------------------------------------------------------------------------
try:
    from stable_baselines3 import PPO
except ImportError as _sb3_exc:  # pragma: no cover
    PPO = None  # type: ignore[assignment,misc]
    SB3_IMPORT_ERROR: Optional[ImportError] = _sb3_exc
else:
    SB3_IMPORT_ERROR = None


# ===========================================================================
# 参数解析
# ===========================================================================
def parse_args(argv: Optional[list] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="加载 PPO 模型回放四足机器狗行走",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--model",
        type=str,
        default=None,
        metavar="ZIP",
        help="已训练模型路径（.zip）",
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=5,
        help="回放的 episode 数量",
    )
    parser.add_argument(
        "--device",
        type=str,
        default=DEFAULT_DEVICE,
        help="计算设备，如 cuda / cpu",
    )
    parser.add_argument(
        "--render",
        action="store_true",
        help="打开渲染窗口（human 模式）",
    )
    parser.add_argument(
        "--video",
        type=str,
        default=None,
        metavar="OUT.mp4",
        help="录制 rgb_array 视频到指定 mp4 文件",
    )
    parser.add_argument(
        "--deterministic",
        action="store_true",
        help="使用确定性动作（默认开启；传 --deterministic 同样开启）",
    )
    parser.add_argument(
        "--stochastic",
        action="store_true",
        help="使用随机采样动作（覆盖默认的确定性策略）",
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        default=None,
        help="单集最大步数上限（默认由环境自身决定）",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只解析参数，不加载模型、不创建环境",
    )
    return parser.parse_args(argv)


# ===========================================================================
# 工具
# ===========================================================================
def require_deps() -> None:
    if SB3_IMPORT_ERROR is not None or PPO is None:
        raise SystemExit(
            "【错误】缺少依赖 stable-baselines3，无法加载模型。\n"
            "  请先安装：pip install stable-baselines3 gymnasium torch\n"
            f"  原始报错：{SB3_IMPORT_ERROR}"
        )
    if WALK_ENV_IMPORT_ERROR is not None or QuadrupedWalkEnv is None:
        raise SystemExit(
            "【错误】无法导入 walk_env.QuadrupedWalkEnv，环境文件可能尚未就绪。\n"
            "  请确认 webots-sim/rl/walk_env.py 已存在且可导入。\n"
            f"  原始报错：{WALK_ENV_IMPORT_ERROR}"
        )


def make_env(render: bool, video: bool) -> Any:
    """创建环境。

    优先尝试带 render_mode 的构造方式：
      - 需要录视频 → render_mode="rgb_array"
      - 需要窗口    → render_mode="human"
      - 否则        → render_mode=None
    构造函数不接受 render_mode 时退化为无参构造。
    """
    assert QuadrupedWalkEnv is not None

    if video:
        mode = "rgb_array"
    elif render:
        mode = "human"
    else:
        mode = None

    if mode is None:
        try:
            return QuadrupedWalkEnv()
        except TypeError:
            return QuadrupedWalkEnv(render_mode=None)

    try:
        return QuadrupedWalkEnv(render_mode=mode)
    except TypeError:
        # 构造函数不支持 render_mode
        return QuadrupedWalkEnv()


def grab_frame(env: Any) -> Optional[Any]:
    """从环境取一帧 RGB 图像；不支持则返回 None。"""
    try:
        frame = env.render()
    except Exception:  # noqa: BLE001 - 渲染不可用时安静降级
        return None
    if frame is None:
        return None
    # 兼容 (H, W, 3) ndarray 与 (frame, extra) 之类返回值
    if isinstance(frame, tuple):
        frame = frame[0]
    return frame


def write_video(frames: List[Any], out_path: str, fps: int = 30) -> None:
    """把帧序列写成 mp4：优先 imageio，退回 OpenCV。"""
    if not frames:
        print("【视频】没有采集到任何帧，跳过录制")
        return

    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)

    try:
        import imageio

        # imageio-ffmpeg 后端
        imageio.mimsave(str(out), frames, fps=fps)
        print(f"【视频】已保存（imageio）：{out}  共 {len(frames)} 帧")
        return
    except Exception as exc:  # noqa: BLE001
        print(f"【视频】imageio 写入失败（{exc}），尝试 OpenCV ...")

    try:
        import cv2
        import numpy as np

        first = np.asarray(frames[0])
        height, width = first.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(out), fourcc, float(fps), (width, height))
        if not writer.isOpened():
            raise RuntimeError("OpenCV VideoWriter 打开失败")
        for frame in frames:
            arr = np.asarray(frame)
            # RGB -> BGR
            writer.write(cv2.cvtColor(arr, cv2.COLOR_RGB2BGR))
        writer.release()
        print(f"【视频】已保存（OpenCV）：{out}  共 {len(frames)} 帧")
    except Exception as exc:  # noqa: BLE001
        raise SystemExit(
            f"【错误】视频写入失败：{exc}\n"
            "  请安装 imageio + imageio-ffmpeg，或 opencv-python"
        )


def extract_forward_distance(info: Any, fallback_total: float) -> float:
    """从 info 里尽量取出「前进距离」。

    walk_env.py 的 info 字典键名以环境实现为准，这里做多键兼容。
    """
    if not isinstance(info, dict):
        return fallback_total

    # 直接给出累计前进距离的键
    for key in (
        "forward_distance",
        "distance",
        "x_pos",
        "x_position",
        "base_x",
        "pos_x",
        "position_x",
        "base_pos",
    ):
        if key in info:
            try:
                value = info[key]
                # base_pos 可能是 (x, y, z) 向量
                if hasattr(value, "__len__") and not isinstance(value, (str, bytes)):
                    value = value[0]
                return float(value)
            except (TypeError, ValueError, IndexError):
                continue

    # 只有前进速度时，由调用方累加（这里返回 fallback）
    return fallback_total


def extract_forward_velocity(info: Any) -> Optional[float]:
    if not isinstance(info, dict):
        return None
    for key in ("forward_vel", "vx", "vel_x", "base_vx", "velocity_x", "lin_vel"):
        if key in info:
            try:
                value = info[key]
                if hasattr(value, "__len__") and not isinstance(value, (str, bytes)):
                    value = value[0]
                return float(value)
            except (TypeError, ValueError, IndexError):
                continue
    return None


# ===========================================================================
# 回放主流程
# ===========================================================================
def play(args: argparse.Namespace) -> None:
    require_deps()

    model_path = Path(args.model) if args.model else None
    if model_path is None or not model_path.is_file():
        raise SystemExit("【错误】请通过 --model 指定已存在的 .zip 模型文件")

    print(f"【回放】加载模型：{model_path}")
    t0 = time.time()
    model = PPO.load(str(model_path), device=args.device)  # type: ignore[union-attr]
    print(f"【回放】模型加载完成（{time.time() - t0:.2f}s），设备={args.device}")

    want_video = bool(args.video)
    env = make_env(render=args.render, video=want_video)
    print(
        f"【回放】环境已创建  render={args.render}  video={args.video or '（否）'}"
    )

    deterministic = not args.stochastic
    frames: List[Any] = []
    all_rewards: List[float] = []
    all_lengths: List[int] = []
    all_distances: List[float] = []

    try:
        for ep in range(1, args.episodes + 1):
            reset_out = env.reset()
            obs = reset_out[0] if isinstance(reset_out, tuple) else reset_out

            ep_reward = 0.0
            ep_len = 0
            done = False
            integrated_dist = 0.0  # info 无距离字段时用速度积分
            last_dist = 0.0

            while not done:
                action, _ = model.predict(obs, deterministic=deterministic)  # type: ignore[union-attr]
                step_out = env.step(action)
                if len(step_out) == 5:
                    obs, reward, terminated, truncated, info = step_out
                    done = bool(terminated) or bool(truncated)
                else:  # 兼容旧版 gym
                    obs, reward, done, info = step_out
                    done = bool(done)

                ep_reward += float(reward)
                ep_len += 1

                # 累计前进距离
                dist = extract_forward_distance(info, fallback_total=last_dist)
                if dist != last_dist or "forward_distance" in (info or {}):
                    last_dist = dist
                vel = extract_forward_velocity(info)
                if vel is not None:
                    # 用 config.SIM_DT 累加会不准（dt 在 walk_env 内），这里仅作估计
                    integrated_dist += vel * 0.02

                # 录制帧
                if want_video:
                    frame = grab_frame(env)
                    if frame is not None:
                        frames.append(frame)

                if args.max_steps is not None and ep_len >= args.max_steps:
                    break

            # 优先用 info 里的距离，否则用速度积分估计
            forward_dist = last_dist if last_dist > 0 else integrated_dist
            all_rewards.append(ep_reward)
            all_lengths.append(ep_len)
            all_distances.append(forward_dist)

            print(
                f"【回放】第 {ep}/{args.episodes} 集："
                f"奖励={ep_reward:.3f}  存活步数={ep_len}  "
                f"前进距离={forward_dist:.3f} m"
            )
    finally:
        close = getattr(env, "close", None)
        if callable(close):
            close()

    n = max(1, len(all_rewards))
    print("-" * 50)
    print(
        f"【回放】汇总：平均奖励={sum(all_rewards)/n:.3f}  "
        f"平均存活步数={sum(all_lengths)/n:.1f}  "
        f"平均前进距离={sum(all_distances)/n:.3f} m  （共 {len(all_rewards)} 集）"
    )

    if want_video:
        write_video(frames, args.video)


def main(argv: Optional[list] = None) -> None:
    args = parse_args(argv)
    if args.dry_run:
        print("【dry-run】参数解析成功，不加载模型、不创建环境。")
        print(f"  --model     : {args.model}")
        print(f"  --episodes  : {args.episodes}")
        print(f"  --render    : {args.render}")
        print(f"  --video     : {args.video}")
        print(f"  --device    : {args.device}")
        env_status = "就绪" if QuadrupedWalkEnv is not None else "未就绪（import 失败）"
        print(f"  walk_env    : {env_status}")
        return

    if not args.model:
        raise SystemExit("【错误】必须通过 --model 指定模型路径（或先使用 --dry-run）")
    play(args)


if __name__ == "__main__":
    main()
