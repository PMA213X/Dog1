#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""完全平直地面移动与跳跃的 Gymnasium 环境。

环境复用现有 Webots JSON TCP 锁步桥，只在公共钩子中附加 57 维观测、
命令域、跳跃调度、安全动作和有限域随机化。
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import numpy as np

try:
    from . import flat_jump_contract as contract
    from . import flat_jump_reward as reward_module
    from .loco_jump_env import QuadrupedLocoJumpEnv
except ImportError:
    import flat_jump_contract as contract  # type: ignore[no-redef]
    import flat_jump_reward as reward_module  # type: ignore[no-redef]
    from loco_jump_env import QuadrupedLocoJumpEnv  # type: ignore[no-redef]


_RL_DIR = Path(__file__).resolve().parent
DEFAULT_WORLD = _RL_DIR.parent / "worlds" / "flat_jump_rl.wbt"
CONTACT_ON_THRESHOLD = 0.5


class FlatGroundJumpEnv(QuadrupedLocoJumpEnv):
    """仅覆盖平直无障碍地面的 57 维命令条件化环境。"""

    CONTRACT_VERSION = "flat_v1"

    def __init__(
        self,
        render_mode: Optional[str] = None,
        world: Optional[os.PathLike | str] = None,
        bridge_port: Optional[int] = None,
        webots_timeout: float = 90.0,
        *,
        phase: str = "F0",
        randomization_mode: str = "full",
        start_bridge: bool = True,
        max_episode_steps: int = contract.MAX_EPISODE_STEPS,
        command_resample_steps: int = contract.COMMAND_RESAMPLE_STEPS,
    ) -> None:
        # 基类先用旧标签完成通用字段初始化，随后立即切换到平地阶段。
        super().__init__(
            render_mode=render_mode,
            world=DEFAULT_WORLD if world is None else world,
            bridge_port=bridge_port,
            webots_timeout=webots_timeout,
            tag="phase0",
            start_bridge=False,
            max_episode_steps=max_episode_steps,
            command_resample_steps=command_resample_steps,
            jump_request_step=contract.JUMP_REQUEST_STEP_MIN,
            jump_interval_steps=contract.JUMP_INTERVAL_MIN,
        )
        self.flat_phase = contract.normalize_phase(phase)
        if randomization_mode not in {"fixed", "curriculum", "full"}:
            raise ValueError(
                "randomization_mode 必须是 fixed、curriculum 或 full"
            )
        self.randomization_mode = randomization_mode
        self._curriculum_episode = 0
        high_obs = np.full(contract.OBS_DIM, np.inf, dtype=np.float32)
        self.observation_space = self.observation_space.__class__(
            -high_obs, high_obs, dtype=np.float32
        )
        self._next_jump_step = contract.JUMP_REQUEST_STEP_MIN
        self._jump_landing_event = False
        self._randomization: Dict[str, Any] = {
            "friction": 1.2,
            "mass_scale": 1.0,
            "delay_steps": 0,
        }
        if start_bridge:
            self._start_bridge()

    @property
    def command_enabled(self) -> bool:
        """F2/F3 启用命令域，F0/F1 保持零命令。"""
        return self.flat_phase in {"F2", "F3"}

    @property
    def jump_enabled(self) -> bool:
        """只有 F3 自动请求跳跃。"""
        return self.flat_phase == "F3"

    def sample_command(self) -> np.ndarray:
        """20% 全零停止命令，其余在契约范围内均匀采样。"""
        if not self.command_enabled:
            return np.zeros(3, dtype=np.float32)
        if float(self.np_random.random()) < contract.ZERO_COMMAND_PROBABILITY:
            return np.zeros(3, dtype=np.float32)
        sampled = self.np_random.uniform(
            low=np.asarray(contract.COMMAND_LOW, dtype=np.float32),
            high=np.asarray(contract.COMMAND_HIGH, dtype=np.float32),
        )
        return np.asarray(sampled, dtype=np.float32)

    def request_jump(self) -> bool:
        """锁存一次 F3 跳跃请求。"""
        if not self.jump_enabled:
            return False
        if self._jump_latched:
            return False
        self._jump_latched = True
        self._jump_elapsed = 0.0
        self._jump_had_contact_loss = False
        self._jump_peak_height = self._baseline_height
        return True

    def _maybe_auto_request_jump(self) -> None:
        """按 episode 内随机首点和随机间隔请求跳跃。"""
        if not self.jump_enabled or self._jump_latched:
            return
        if self._ep_steps < self._next_jump_step:
            return
        self.request_jump()
        self._next_jump_step = int(
            self.np_random.integers(
                contract.JUMP_INTERVAL_MIN,
                contract.JUMP_INTERVAL_MAX + 1,
            )
        ) + self._ep_steps

    def _update_jump_latch(self, state: Mapping[str, Any]) -> None:
        """仅在四足离地、达到高度阈值并恢复触地后记录成功和落地。"""
        if not self._jump_latched:
            return
        self._jump_elapsed = contract.clip_jump_phase_time(
            float(state["jump_phase"])
        )
        self._jump_peak_height = max(
            self._jump_peak_height, float(state["height"])
        )
        contacts = np.asarray(state["contacts"], dtype=np.float64)
        airborne = bool(np.all(contacts < CONTACT_ON_THRESHOLD))
        restored_now = bool(np.any(contacts >= CONTACT_ON_THRESHOLD))
        if airborne:
            self._jump_had_contact_loss = True

        if (
            not self._jump_success
            and self._jump_had_contact_loss
            and self._jump_peak_height
            >= self._baseline_height + contract.JUMP_SUCCESS_HEIGHT_GAIN
        ):
            self._jump_success = True
            self._jump_success_event = True

        contacts_restored = bool(
            self._jump_had_contact_loss and restored_now
        )
        if (
            contacts_restored
            and self._jump_success
            and not self._jump_landing_event
        ):
            self._jump_landing_event = True

        clear = contract.should_clear_jump_latch(
            jump_latched=True,
            contacts_restored=contacts_restored,
            elapsed_seconds=self._jump_elapsed,
        )
        if clear:
            self._jump_latched = False
            self._jump_had_contact_loss = False

    @staticmethod
    def _fallen(state: Mapping[str, Any]) -> bool:
        """摔倒、异常高度或越出平地工作区都终止 episode。"""
        roll = float(state["rpy"][0])
        pitch = float(state["rpy"][1])
        height = float(state["height"])
        x = float(state.get("x", 0.0))
        y = float(state.get("y", 0.0))
        return (
            abs(roll) > contract.ROLL_PITCH_LIMIT
            or abs(pitch) > contract.ROLL_PITCH_LIMIT
            or height < contract.MIN_BASE_HEIGHT
            or height > contract.MAX_BASE_HEIGHT
            or abs(x) > contract.WORKSPACE_LIMIT
            or abs(y) > contract.WORKSPACE_LIMIT
        )

    def _state_to_obs(self, state: Mapping[str, Any]) -> np.ndarray:
        """追加地形高度与平坦提示，形成严格 57 维观测。"""
        legacy = super()._state_to_obs(state)
        terrain = np.asarray([0.0, 1.0], dtype=np.float32)
        obs = np.concatenate([legacy, terrain]).astype(np.float32)
        if obs.shape != (contract.OBS_DIM,):
            raise RuntimeError(
                f"平地观测维度错误：期望 {(contract.OBS_DIM,)}，收到 {obs.shape}"
            )
        if not np.all(np.isfinite(obs)):
            raise RuntimeError("平地观测包含 NaN/Inf")
        return obs

    def _validate_state(self, state: Mapping[str, Any]) -> Dict[str, Any]:
        """保留平地验收所需的世界系 x/y 兼容字段。"""
        validated = super()._validate_state(state)
        for field in ("x", "y"):
            if field in state:
                value = float(state[field])
                if not np.isfinite(value):
                    raise ValueError(f"TCP state.{field} 包含 NaN/Inf")
                validated[field] = value
        return validated

    def _reset_payload_extra(self) -> Dict[str, Any]:
        """每次 reset 采样并发送有限域随机化参数。"""
        if self.flat_phase == "F0" or self.randomization_mode == "fixed":
            values = {
                "friction": 1.2,
                "mass_scale": 1.0,
                "delay_steps": 0,
            }
        elif self.randomization_mode == "curriculum":
            episode = self._curriculum_episode
            self._curriculum_episode += 1
            if episode < contract.F1_CURRICULUM_FIXED_EPISODES:
                values = {
                    "friction": 1.2,
                    "mass_scale": 1.0,
                    "delay_steps": 0,
                }
            else:
                ramp_span = (
                    contract.F1_CURRICULUM_EPISODES
                    - contract.F1_CURRICULUM_FIXED_EPISODES
                )
                progress = min(
                    1.0,
                    (
                        episode
                        - contract.F1_CURRICULUM_FIXED_EPISODES
                        + 1
                    )
                    / ramp_span,
                )
                friction_low = 1.2 + progress * (
                    contract.FRICTION_RANGE[0] - 1.2
                )
                friction_high = 1.2 + progress * (
                    contract.FRICTION_RANGE[1] - 1.2
                )
                mass_low = 1.0 + progress * (
                    contract.MASS_SCALE_RANGE[0] - 1.0
                )
                mass_high = 1.0 + progress * (
                    contract.MASS_SCALE_RANGE[1] - 1.0
                )
                delay_max = int(round(progress * contract.DELAY_STEPS_RANGE[1]))
                values = {
                    "friction": float(
                        self.np_random.uniform(friction_low, friction_high)
                    ),
                    "mass_scale": float(
                        self.np_random.uniform(mass_low, mass_high)
                    ),
                    "delay_steps": int(
                        self.np_random.integers(0, delay_max + 1)
                    ),
                }
        else:
            values = {
                "friction": float(
                    self.np_random.uniform(*contract.FRICTION_RANGE)
                ),
                "mass_scale": float(
                    self.np_random.uniform(*contract.MASS_SCALE_RANGE)
                ),
                "delay_steps": int(
                    self.np_random.integers(
                        contract.DELAY_STEPS_RANGE[0],
                        contract.DELAY_STEPS_RANGE[1] + 1,
                    )
                ),
            }
        self._randomization = values
        result_extra = dict(values)
        result_extra["randomization_mode"] = self.randomization_mode
        result_extra["curriculum_episode"] = self._curriculum_episode
        return result_extra

    def _act_payload_extra(self) -> Dict[str, Any]:
        """把本 episode 的电机延迟发送给控制器。"""
        return {"delay_steps": int(self._randomization["delay_steps"])}

    def _sanitize_action(self, action_array: np.ndarray) -> np.ndarray:
        """执行与回放一致的有限值、绝对限幅和单周期变化率限制。"""
        if not np.all(np.isfinite(action_array)):
            raise ValueError("平地动作包含 NaN/Inf")
        bounded = np.clip(
            action_array,
            contract.ACTION_LOW,
            contract.ACTION_HIGH,
        ).astype(np.float32)
        delta = contract.ACTION_RATE_LIMIT
        safe = np.clip(
            bounded,
            self._prev_action.astype(np.float32) - delta,
            self._prev_action.astype(np.float32) + delta,
        ).astype(np.float32)
        if not np.all(np.isfinite(safe)):
            raise ValueError("安全动作包含 NaN/Inf")
        return safe

    def reset(
        self, *, seed: Optional[int] = None, options: Optional[Dict[str, Any]] = None
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """复位并生成随机跳跃调度。"""
        result = super().reset(seed=seed, options=options)
        self._next_jump_step = int(
            self.np_random.integers(
                contract.JUMP_REQUEST_STEP_MIN,
                contract.JUMP_REQUEST_STEP_MAX + 1,
            )
        )
        self._jump_landing_event = False
        result[1]["randomization"] = dict(self._randomization)
        result[1]["randomization_mode"] = self.randomization_mode
        result[1]["curriculum_episode"] = self._curriculum_episode
        result[1]["phase"] = self.flat_phase
        result[1]["reward_breakdown"] = {
            name: 0.0
            for name in reward_module.WEIGHTS[self.flat_phase]
        } | {"total": 0.0}
        return result

    def _compute_reward(
        self,
        state: Mapping[str, Any],
        action: np.ndarray,
        *,
        fallen: bool,
    ) -> Tuple[float, Dict[str, float]]:
        """计算平地阶段奖励并严格拒绝非有限值。"""
        heading_error = self._heading_error(state)
        total, breakdown = reward_module.reward_for_state(
            self.flat_phase,
            state,
            action,
            self._prev_action,
            command=self._command,
            heading_error=heading_error,
            jump_latched=self._jump_latched,
            jump_peak_gain=self._jump_peak_height - self._baseline_height,
            jump_success_event=self._jump_success_event,
            jump_landing_event=self._jump_landing_event,
            fallen=fallen,
        )
        return float(total), breakdown

    def step(
        self, action: np.ndarray
    ) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        """推进一步并在返回前清除一次性事件标志。"""
        obs, reward, terminated, truncated, info = super().step(action)
        self._jump_success_event = False
        self._jump_landing_event = False
        info["phase"] = self.flat_phase
        info["randomization"] = dict(self._randomization)
        info["randomization_mode"] = self.randomization_mode
        info["curriculum_episode"] = self._curriculum_episode
        if terminated:
            info["termination_reason"] = self._termination_reason(
                self._last_state, bool(info.get("fallen", False))
            )
        elif truncated:
            info["termination_reason"] = "time_limit"
        return obs, reward, terminated, truncated, info

    @staticmethod
    def _termination_reason(
        state: Mapping[str, Any],
        fallen: bool,
    ) -> str:
        """返回可读的终止原因，供验收日志区分失败类型。"""
        if not fallen:
            return "state_done"
        roll = float(state.get("rpy", (0.0, 0.0, 0.0))[0])
        pitch = float(state.get("rpy", (0.0, 0.0, 0.0))[1])
        height = float(state.get("height", 0.0))
        x = float(state.get("x", 0.0))
        y = float(state.get("y", 0.0))
        if abs(roll) > contract.ROLL_PITCH_LIMIT or abs(
            pitch
        ) > contract.ROLL_PITCH_LIMIT:
            return "attitude_limit"
        if height < contract.MIN_BASE_HEIGHT:
            return "height_low"
        if height > contract.MAX_BASE_HEIGHT:
            return "height_high"
        if abs(x) > contract.WORKSPACE_LIMIT or abs(y) > contract.WORKSPACE_LIMIT:
            return "workspace_limit"
        return "state_done"


def make_env(
    phase: str = "F0",
    **kwargs: Any,
) -> FlatGroundJumpEnv:
    """创建平地环境的公共工厂。"""
    return FlatGroundJumpEnv(phase=phase, **kwargs)


if __name__ == "__main__":
    print(
        "【自检】平地环境导入通过：obs="
        f"{contract.OBS_DIM} action={contract.ACTION_DIM}"
    )
