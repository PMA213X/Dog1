#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""QuadrupedTurnEnv：P3 转向训练环境（在 QuadrupedWalkEnv 上增量扩展）。

与 walk_env.py 的差异：
  1. 观测 45 维 = 原 42 维 + [yaw_err, wz, vx]
       yaw_err : 当前航向与目标航向的偏差（wrap 到 [-π, π]）
       wz      : 机体系偏航角速度（rad/s）
       vx      : 机体系前进速度（m/s，与奖励项同源）
  2. 奖励在原行走奖励上叠加转向塑形：
       r = 1.0*alive + 3.0*max(vx, 0) - 2|roll| - 2|pitch|
           - 0.05*||a||² - 10*fallen
           + 2.0*cos(yaw_err)            # 朝向目标
           - 0.5*|wz|                    # 转向平滑
           + 1.0*(|yaw_err| < 0.2)       # 朝向正确奖励
  3. 每个 episode 随机采样目标航向（-π~π），也可固定左/右转。

原 walk_env.py 保持不动，便于 P2 回放与对照。
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import numpy as np

# 保证同目录 config.py / walk_env.py 可导入
_RL_DIR = Path(__file__).resolve().parent
if str(_RL_DIR) not in sys.path:
    sys.path.insert(0, str(_RL_DIR))

from config import OBS_DIM_TURN, REWARD_WEIGHTS  # noqa: E402
from walk_env import QuadrupedWalkEnv  # noqa: E402

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError as _gym_exc:  # pragma: no cover
    raise ImportError(
        f"【错误】缺少 gymnasium，无法使用 QuadrupedTurnEnv：{_gym_exc}"
    ) from _gym_exc


# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------
# 观测末 3 维索引
IDX_YAW_ERR = OBS_DIM_TURN - 3
IDX_WZ = OBS_DIM_TURN - 2
IDX_VX = OBS_DIM_TURN - 1

# 朝向正确奖励阈值（rad）
HEADING_OK_THRESH = 0.2

# 目标航向模式
HEADING_MODE_RANDOM = "random"    # 每 episode 均匀采样 (-π, π]
HEADING_MODE_LEFT = "left"        # 固定左转 +π/2
HEADING_MODE_RIGHT = "right"      # 固定右转 -π/2
HEADING_MODES = (HEADING_MODE_RANDOM, HEADING_MODE_LEFT, HEADING_MODE_RIGHT)


def wrap_angle(a: float) -> float:
    """把角度 wrap 到 [-π, π]。"""
    return float((a + np.pi) % (2.0 * np.pi) - np.pi)


class QuadrupedTurnEnv(QuadrupedWalkEnv):
    """P3 转向环境：45 维观测 + 朝向目标奖励。"""

    metadata = {
        "render_modes": ["human", "rgb_array"],
        "render_fps": 50,
    }

    def __init__(
        self,
        render_mode: Optional[str] = None,
        world: Optional[os.PathLike | str] = None,
        bridge_port: Optional[int] = None,
        webots_timeout: float = 90.0,
        heading_mode: str = HEADING_MODE_RANDOM,
        fixed_heading: Optional[float] = None,
    ) -> None:
        if heading_mode not in HEADING_MODES:
            raise ValueError(
                f"heading_mode 必须是 {HEADING_MODES} 之一，收到 {heading_mode!r}"
            )
        self.heading_mode = heading_mode
        # 优先使用显式指定的目标航向；否则按模式决定
        self._fixed_heading = fixed_heading
        self.target_yaw: float = 0.0
        self._prev_yaw: Optional[float] = None

        super().__init__(
            render_mode=render_mode,
            world=world,
            bridge_port=bridge_port,
            webots_timeout=webots_timeout,
        )

        # 覆盖父类的 42 维观测空间 → 45 维
        high_obs = np.full(OBS_DIM_TURN, np.inf, dtype=np.float32)
        self.observation_space = spaces.Box(-high_obs, high_obs, dtype=np.float32)

    # ------------------------------------------------------------------
    # 目标航向
    # ------------------------------------------------------------------
    def _sample_target_yaw(self) -> float:
        if self._fixed_heading is not None:
            return wrap_angle(float(self._fixed_heading))
        if self.heading_mode == HEADING_MODE_LEFT:
            return wrap_angle(np.pi / 2.0)
        if self.heading_mode == HEADING_MODE_RIGHT:
            return wrap_angle(-np.pi / 2.0)
        # random：均匀采样 (-π, π]
        rng = self.np_random
        return wrap_angle(float(rng.uniform(-np.pi, np.pi)))

    def set_target_yaw(self, yaw: float) -> None:
        """外部指定目标航向（例如评估时固定朝向）。"""
        self.target_yaw = wrap_angle(yaw)

    # ------------------------------------------------------------------
    # 状态 → 观测（45 维）
    # ------------------------------------------------------------------
    @staticmethod
    def _body_vx_from_state(state: Dict[str, Any]) -> float:
        rpy = np.asarray(state["rpy"], dtype=np.float64)
        v_world = np.asarray(state["v"], dtype=np.float64)
        yaw = float(rpy[2])
        return float(np.cos(yaw) * v_world[0] + np.sin(yaw) * v_world[1])

    @staticmethod
    def _wz_from_state(state: Dict[str, Any]) -> float:
        """偏航角速度：优先用 rl_agent 发来的角速度，缺失时用 yaw 差分。"""
        w = state.get("w")
        if w is not None:
            try:
                arr = np.asarray(w, dtype=np.float64)
                if arr.size >= 3:
                    return float(arr[2])
            except (TypeError, ValueError):
                pass
        # 兜底：yaw 数值微分（需上一拍 yaw，由调用方缓存）
        return float("nan")

    def _state_to_obs(self, state: Dict[str, Any]) -> np.ndarray:
        """45 维观测 = 原 42 维 + [yaw_err, wz, vx]。"""
        q = np.asarray(state["q"], dtype=np.float32)          # 12
        dq = np.asarray(state["dq"], dtype=np.float32)        # 12
        rpy = np.asarray(state["rpy"], dtype=np.float32)      # 3
        yaw = float(rpy[2])
        vx = self._body_vx_from_state(state)
        v_world = np.asarray(state["v"], dtype=np.float32)
        vy = float(-np.sin(yaw) * v_world[0] + np.cos(yaw) * v_world[1])
        v_body = np.array([vx, vy, float(v_world[2])], dtype=np.float32)

        yaw_err = wrap_angle(yaw - self.target_yaw)
        wz = self._wz_from_state(state)
        if np.isnan(wz):
            # yaw 差分兜底
            if self._prev_yaw is None:
                wz = 0.0
            else:
                dyaw = wrap_angle(yaw - self._prev_yaw)
                wz = float(dyaw / 0.02)  # SIM_DT
        self._prev_yaw = yaw

        tail = np.array([yaw_err, wz, vx], dtype=np.float32)
        obs = np.concatenate([q, dq, rpy, v_body, self._prev_action, tail])
        obs = obs.astype(np.float32)
        assert obs.shape == (OBS_DIM_TURN,), f"观测维度错误：{obs.shape}"
        return obs

    # ------------------------------------------------------------------
    # 奖励（转向塑形）
    # ------------------------------------------------------------------
    def _compute_reward(
        self, state: Dict[str, Any], action: np.ndarray, fallen: bool
    ) -> float:
        vx = self._body_vx_from_state(state)
        roll = float(state["rpy"][0])
        pitch = float(state["rpy"][1])
        yaw = float(state["rpy"][2])
        yaw_err = wrap_angle(yaw - self.target_yaw)
        wz = self._wz_from_state(state)
        if np.isnan(wz) and self._prev_yaw is not None:
            wz = wrap_angle(yaw - self._prev_yaw) / 0.02
        if np.isnan(wz):
            wz = 0.0

        w = REWARD_WEIGHTS
        r = (
            w["alive_bonus"]
            + w["forward_vel"] * max(vx, 0.0)   # 任务约定：只奖励前进
            + w["roll_penalty"] * abs(roll)
            + w["pitch_penalty"] * abs(pitch)
            + w["action_sq"] * float(np.dot(action, action))
            # ---- P3 转向塑形 ----
            + w["heading_cos"] * float(np.cos(yaw_err))
            + w["yaw_rate_penalty"] * abs(wz)
            + w["heading_bonus"] * (1.0 if abs(yaw_err) < HEADING_OK_THRESH else 0.0)
        )
        if fallen:
            r += w["fall_penalty"]
        return float(r)

    # ------------------------------------------------------------------
    # Gymnasium API
    # ------------------------------------------------------------------
    def reset(
        self, *, seed: Optional[int] = None, options: Optional[Dict[str, Any]] = None
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        # 允许 reset(options={"target_yaw": ...}) 指定目标航向
        if options and "target_yaw" in options:
            self._fixed_heading = float(options["target_yaw"])
        self.target_yaw = self._sample_target_yaw()
        self._prev_yaw = None
        # 父类 reset 会调用本类 _state_to_obs / _build_info（已覆盖）
        return super().reset(seed=seed, options=options)

    def _build_info(
        self, state: Dict[str, Any], reward: float, fallen: bool
    ) -> Dict[str, Any]:
        info = super()._build_info(state, reward=reward, fallen=fallen)
        yaw = float(state["rpy"][2])
        yaw_err = wrap_angle(yaw - self.target_yaw)
        info.update(
            {
                "target_yaw": self.target_yaw,
                "yaw_err": yaw_err,
                "wz": self._wz_from_state(state),
                "heading_ok": bool(abs(yaw_err) < HEADING_OK_THRESH),
            }
        )
        return info


# ===========================================================================
# 冒烟自检
# ===========================================================================
if __name__ == "__main__":
    print("【自检】创建 QuadrupedTurnEnv ...")
    env = QuadrupedTurnEnv()
    try:
        obs, info = env.reset()
        print(
            f"【自检】reset: obs.shape={obs.shape} target_yaw={info.get('target_yaw'):.3f} "
            f"yaw_err={info.get('yaw_err'):.3f}"
        )
        assert obs.shape == (OBS_DIM_TURN,), obs.shape
        total_r = 0.0
        for i in range(50):
            obs, r, term, trunc, info = env.step(env.action_space.sample())
            total_r += r
            if term or trunc:
                print(f"【自检】第 {i+1} 步结束 term={term} trunc={trunc}，自动 reset")
                obs, info = env.reset()
        print(
            f"【自检】50 步OK r={r:.4f} yaw_err={info.get('yaw_err', 0):.3f} "
            f"累计奖励={total_r:.3f}"
        )
    finally:
        env.close()
