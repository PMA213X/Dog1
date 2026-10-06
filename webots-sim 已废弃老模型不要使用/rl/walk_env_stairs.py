#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""QuadrupedStairsEnv：P4 台阶训练环境（在 QuadrupedTurnEnv 上增量扩展）。

与 walk_env_turn.py 的差异：
  1. 观测 51 维 = 原 45 维 + 6 维前下方地形采样
       前视距离 0.1 / 0.25 / 0.4 / 0.6 / 0.9 / 1.2 m
       简化地形：用阶梯函数（piecewise-constant）近似，不用真实高度图。
       台阶高每 episode 从三档 {0.05, 0.08, 0.12} m 随机抽一档，
       让策略对级高鲁棒；踏面深 0.30 m，与 parkour.wbt 的 StraightStairs 一致。
  2. 奖励在转向奖励上叠加爬台阶塑形：
       r = 1.0*alive + 3.0*max(vx,0) - 2|roll| - 2|pitch|
           - 0.05*||a||^2 - 10*fallen
           + 2.0*cos(yaw_err) - 0.5*|wz| + 1.0*(|yaw_err|<0.2)   # 保留 P3 转向
           + 2.0*max(Δz, 0)                                     # 爬升
           + 1.0*(站上台阶)                                       # z 抬升超过级高
           - 3.0*(从台阶掉下)                                     # 从高处跌落
  3. 默认世界 parkour.wbt（含台阶/斜坡/土坑），由 walk_env._patch_rl_world
     在运行时绑定 rl_agent + supervisor 并走 TCP 桥。

原 walk_env.py / walk_env_turn.py 保持不动，便于 P2/P3 回放与对照。
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

# 保证同目录 config.py / walk_env*.py 可导入
_RL_DIR = Path(__file__).resolve().parent
if str(_RL_DIR) not in sys.path:
    sys.path.insert(0, str(_RL_DIR))

from config import OBS_DIM_STAIRS, REWARD_WEIGHTS  # noqa: E402
from walk_env import WORLDS_DIR  # noqa: E402
from walk_env_turn import QuadrupedTurnEnv, wrap_angle  # noqa: E402

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError as _gym_exc:  # pragma: no cover
    raise ImportError(
        f"【错误】缺少 gymnasium，无法使用 QuadrupedStairsEnv：{_gym_exc}"
    ) from _gym_exc


# ---------------------------------------------------------------------------
# 常量：简化地形模型
# ---------------------------------------------------------------------------
# 前视采样距离（m），共 6 点
TERRAIN_DISTS: Tuple[float, ...] = (0.1, 0.25, 0.4, 0.6, 0.9, 1.2)
N_TERRAIN: int = len(TERRAIN_DISTS)

# 台阶高三档（m）：每个 episode 随机抽一档
STEP_HEIGHT_TIERS: Tuple[float, ...] = (0.05, 0.08, 0.12)

# 踏面深（m）与级数：对齐 parkour.wbt 的 StraightStairs stepSize/tread=0.3, nSteps=3
TREAD: float = 0.30
N_STEPS: int = 3

# 台阶起点（世界 x，m）：对齐 parkour.wbt stairs_up translation 1.2
STAIR_X0: float = 1.2
# 高台长（m）：对齐 landing_mid 的 1.2 m 台面
LANDING_LEN: float = 1.2
# 跑道半宽（m）：|y| 超过则视为场外平地
TRACK_HALF_WIDTH: float = 0.5

# 观测末 6 维 = 地形采样
IDX_TERRAIN_START = OBS_DIM_STAIRS - N_TERRAIN


class QuadrupedStairsEnv(QuadrupedTurnEnv):
    """P4 台阶环境：51 维观测 + 转向/爬台阶奖励。"""

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
        heading_mode: str = "random",
        fixed_heading: Optional[float] = None,
        step_height: Optional[float] = None,
    ) -> None:
        # 默认跑酷世界（含台阶/斜坡/土坑）
        if world is None:
            world = WORLDS_DIR / "parkour.wbt"
        # 显式指定级高则固定，否则每 episode 从三档随机抽
        if step_height is not None and step_height not in STEP_HEIGHT_TIERS:
            raise ValueError(
                f"step_height 必须是 {STEP_HEIGHT_TIERS} 之一或 None，收到 {step_height!r}"
            )
        self._fixed_step_height = step_height
        self.step_height: float = STEP_HEIGHT_TIERS[-1]
        self._z0: float = 0.0
        self._prev_z: Optional[float] = None
        self._prev_on_step: bool = False
        self._last_dz: float = 0.0

        super().__init__(
            render_mode=render_mode,
            world=world,
            bridge_port=bridge_port,
            webots_timeout=webots_timeout,
            heading_mode=heading_mode,
            fixed_heading=fixed_heading,
        )

        # 覆盖父类的 45 维观测空间 → 51 维
        high_obs = np.full(OBS_DIM_STAIRS, np.inf, dtype=np.float32)
        self.observation_space = spaces.Box(-high_obs, high_obs, dtype=np.float32)

    # ------------------------------------------------------------------
    # 简化地形：阶梯函数高度场
    # ------------------------------------------------------------------
    def _ground_height(self, px: float, py: float) -> float:
        """世界系 (px, py) 处的地面高度（简化阶梯函数，非真实高度图）。

        沿 +X 跑酷线：平地 → 上台阶 → 高台 → 下台阶 → 平地。
        台阶高 = self.step_height（三档之一），踏面深 TREAD。
        """
        if abs(py) > TRACK_HALF_WIDTH:
            return 0.0
        h = self.step_height
        top = N_STEPS * h
        x = px - STAIR_X0
        if x < 0.0:
            return 0.0
        # 上台阶段：x ∈ (0, N_STEPS*TREAD]
        if x < N_STEPS * TREAD:
            level = int(x / TREAD) + 1
            return float(level * h)
        # 高台段
        if x < N_STEPS * TREAD + LANDING_LEN:
            return float(top)
        # 下台阶段
        d = x - (N_STEPS * TREAD + LANDING_LEN)
        level = N_STEPS - int(d / TREAD)
        return float(max(0.0, level * h))

    def _terrain_samples(self, state: Dict[str, Any]) -> np.ndarray:
        """机器人前下方 6 个点的地面高度（世界 z）。"""
        x = float(state.get("x", 0.0))
        y = float(state.get("y", 0.0))
        yaw = float(np.asarray(state["rpy"], dtype=np.float64)[2])
        fx, fy = np.cos(yaw), np.sin(yaw)
        hs: List[float] = []
        for d in TERRAIN_DISTS:
            hs.append(self._ground_height(x + fx * d, y + fy * d))
        return np.asarray(hs, dtype=np.float32)

    # ------------------------------------------------------------------
    # 状态 → 观测（51 维）
    # ------------------------------------------------------------------
    def _state_to_obs(self, state: Dict[str, Any]) -> np.ndarray:
        """51 维观测 = 原 45 维（转向） + 6 维地形采样。"""
        obs45 = super()._state_to_obs(state)  # 45 维
        terrain = self._terrain_samples(state)  # 6 维
        obs = np.concatenate([obs45, terrain]).astype(np.float32)
        assert obs.shape == (OBS_DIM_STAIRS,), f"观测维度错误：{obs.shape}"
        return obs

    # ------------------------------------------------------------------
    # 奖励（转向 + 爬台阶塑形）
    # ------------------------------------------------------------------
    def _compute_reward(
        self, state: Dict[str, Any], action: np.ndarray, fallen: bool
    ) -> float:
        # 原奖励（alive + 前进 + 姿态 + 能耗 + 转向塑形 + fall）
        r = super()._compute_reward(state, action, fallen)

        z = float(state["z"])
        dz = 0.0 if self._prev_z is None else (z - self._prev_z)
        self._last_dz = dz

        w = REWARD_WEIGHTS
        # 爬升：只奖励向上
        r += w["climb_reward"] * max(dz, 0.0)
        # 站上台阶：基座相对出生高度抬升超过半个级高
        on_step = (z - self._z0) >= 0.5 * self.step_height
        if on_step:
            r += w["on_step_reward"]
        # 从台阶掉下：刚才还在台阶上，现在高度跌落
        if self._prev_on_step and (not on_step) and dz < 0.0:
            r += w["drop_penalty"]

        self._prev_z = z
        self._prev_on_step = on_step
        return float(r)

    # ------------------------------------------------------------------
    # Gymnasium API
    # ------------------------------------------------------------------
    def reset(
        self, *, seed: Optional[int] = None, options: Optional[Dict[str, Any]] = None
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        # 允许 reset(options={"step_height": 0.08}) 固定级高
        if options and "step_height" in options:
            sh = float(options["step_height"])
            if sh not in STEP_HEIGHT_TIERS:
                raise ValueError(
                    f"step_height 必须是 {STEP_HEIGHT_TIERS} 之一，收到 {sh!r}"
                )
            self._fixed_step_height = sh
        # 三档随机抽一档（显式指定则固定）
        if self._fixed_step_height is not None:
            self.step_height = float(self._fixed_step_height)
        else:
            rng = self.np_random
            self.step_height = float(rng.choice(STEP_HEIGHT_TIERS))

        obs, info = super().reset(seed=seed, options=options)
        # 出生高度与爬升记账基准
        state = self._last_state
        self._z0 = float(state.get("z", 0.0))
        self._prev_z = self._z0
        self._prev_on_step = False
        self._last_dz = 0.0
        info.update(
            {
                "step_height": self.step_height,
                "dz": 0.0,
                "climb_z": 0.0,
                "on_step": False,
                "terrain": self._terrain_samples(state).tolist(),
            }
        )
        return obs, info

    def _build_info(
        self, state: Dict[str, Any], reward: float, fallen: bool
    ) -> Dict[str, Any]:
        info = super()._build_info(state, reward=reward, fallen=fallen)
        z = float(state["z"])
        on_step = (z - self._z0) >= 0.5 * self.step_height
        info.update(
            {
                "step_height": self.step_height,
                "dz": self._last_dz,
                "climb_z": z - self._z0,
                "on_step": bool(on_step),
                "terrain": self._terrain_samples(state).tolist(),
            }
        )
        return info


# ===========================================================================
# 冒烟自检
# ===========================================================================
if __name__ == "__main__":
    print("【自检】创建 QuadrupedStairsEnv ...")
    env = QuadrupedStairsEnv()
    try:
        obs, info = env.reset()
        print(
            f"【自检】reset: obs.shape={obs.shape} step_height={info.get('step_height')} "
            f"terrain={np.round(info.get('terrain', []), 3)}"
        )
        assert obs.shape == (OBS_DIM_STAIRS,), obs.shape
        total_r = 0.0
        for i in range(50):
            obs, r, term, trunc, info = env.step(env.action_space.sample())
            total_r += r
            if term or trunc:
                print(f"【自检】第 {i+1} 步结束 term={term} trunc={trunc}，自动 reset")
                obs, info = env.reset()
        print(
            f"【自检】50 步OK r={r:.4f} on_step={info.get('on_step')} "
            f"dz={info.get('dz', 0):.4f} 累计奖励={total_r:.3f}"
        )
    finally:
        env.close()
