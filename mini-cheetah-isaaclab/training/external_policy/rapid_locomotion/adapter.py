"""Rapid Locomotion 与 YoboGo Isaac Lab 观测、动作和 50 Hz 调度适配。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import torch


# Rapid 官方冻结策略的固定张量契约。
OBS_DIM = 42
HISTORY_LEN = 15
HISTORY_DIM = OBS_DIM * HISTORY_LEN
LATENT_DIM = 18
BODY_INPUT_DIM = OBS_DIM + LATENT_DIM
ACTION_DIM = 12

# YoboGo 现有 48 维策略观测顺序：速度 6、重力 3、命令 3、关节 12、
# 关节速度 12、上一动作 12。外部策略只读取其中与 Rapid 对应的字段。
YOBOGO_OBS_DIM = 48
YOBOGO_GRAVITY_SLICE = slice(6, 9)
YOBOGO_COMMAND_SLICE = slice(9, 12)
YOBOGO_JOINT_POS_SLICE = slice(15, 27)
YOBOGO_JOINT_VEL_SLICE = slice(27, 39)

# YoboGo/Isaac 几何初态；由 joint_pos_rel 还原绝对关节角后才减 Rapid 默认角。
YOBOGO_SIM_INITIAL_JOINT_POS = torch.tensor(
    [0.0, -0.785398163, 1.865468294] * 4,
    dtype=torch.float32,
)

# Rapid 官方 Mini Cheetah 默认角，顺序固定为 FR、FL、RR、RL 的
# HAA/hip(knee-to-hip)/knee(calf) 三关节。
RAPID_DEFAULT_JOINT_POS = torch.tensor(
    [
        -0.1, -0.8, 1.62,
        0.1, -0.8, 1.62,
        -0.1, -0.8, 1.62,
        0.1, -0.8, 1.62,
    ],
    dtype=torch.float32,
)

# Rapid 原始动作到官方关节目标的尺度：HAA 为 0.125，hip/knee 为 0.25。
RAPID_ACTION_SCALE = torch.tensor(
    [0.125, 0.25, 0.25] * 4,
    dtype=torch.float32,
)

# XML 中后腿 axis=-1 与 rpy=(0,pi,0) 成对等效；两套环境的关节槽位
# 同为 FR/FL/RR/RL，因此适配层不得再做额外符号反转。
RAPID_TO_YOBOGO_AXIS_SIGN = torch.ones(ACTION_DIM, dtype=torch.float32)

# 命令先按 YoboGo 物理范围裁剪，再乘比例，最后按 Rapid 官方域防御性裁剪。
YOBOGO_COMMAND_PHYSICAL_LIMITS = torch.tensor(
    [[-0.6, 0.6], [-0.6, 0.6], [-1.0, 1.0]],
    dtype=torch.float32,
)
YOBOGO_TO_RAPID_COMMAND_SCALE = torch.tensor([2.0, 2.0, 0.25], dtype=torch.float32)
RAPID_COMMAND_LIMITS = torch.tensor(
    [[-0.6, 0.6], [-0.6, 0.6], [-1.0, 1.0]],
    dtype=torch.float32,
)

# YoboGo Isaac 动作仍为现有 JointPositionAction 的 0.5 倍缩放。
YOBOGO_ACTION_SCALE = 0.5

# 位置限制使用项目 URDF；动态安全配置采用官方 Rapid 回放参数，
# 不把实机 17/17/26 N·m 配置冒充为官方策略的动态配置。
YOBOGO_JOINT_POS_LIMITS = torch.tensor(
    [[-1.5, 1.5], [-5.0, 5.0], [-2.7, 2.7]] * 4,
    dtype=torch.float32,
)
RAPID_REPLAY_TORQUE_LIMITS = torch.tensor(
    [18.0, 18.0, 26.0] * 4, dtype=torch.float32
)
RAPID_REPLAY_PD_KP = 20.0
RAPID_REPLAY_PD_KD = torch.tensor([0.5] * 12, dtype=torch.float32)
# 官方 clip_actions=100 只防溢出，无法防动力学域外引起的闭环放大；
# Rapid 回放增加更保守的 OOD 动作门限，且不删除位置/力矩安全裁剪。
RAPID_REPLAY_RAW_ACTION_LIMIT = 0.25

# 控制周期与外部策略更新周期：500 Hz 控制，每 10 周期推理一次，即 50 Hz。
CONTROL_PERIOD_S = 0.002
INFERENCE_PERIOD_CONTROL_STEPS = 10


class RapidPolicyProtocol(Protocol):
    """外部策略最小协议，便于单元测试使用不加载 Isaac 的假策略。"""

    def __call__(
        self, current_observation: torch.Tensor, history: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """输入 42 维当前观测和 630 维历史，返回 12 维原始动作与 18 维 latent。"""
        ...


def _validate_yobogo_observation(yobogo_observation: torch.Tensor) -> None:
    """校验 48 维 YoboGo 观测的形状、有限值和批次一致性。"""
    if yobogo_observation.ndim != 2 or yobogo_observation.shape[-1] != YOBOGO_OBS_DIM:
        raise ValueError(
            f"YoboGo observation 必须为 (N,{YOBOGO_OBS_DIM})，实际为 "
            f"{tuple(yobogo_observation.shape)}"
        )
    if not torch.isfinite(yobogo_observation).all():
        raise ValueError("YoboGo observation 含 NaN/Inf")


def map_yobogo_command_to_rapid(yobogo_command: torch.Tensor) -> torch.Tensor:
    """把 YoboGo 速度命令换算并裁剪到 Rapid 官方训练域。"""
    if yobogo_command.shape[-1] != 3:
        raise ValueError(f"速度命令必须为 3 维，实际为 {tuple(yobogo_command.shape)}")
    physical_lower = YOBOGO_COMMAND_PHYSICAL_LIMITS[:, 0].to(yobogo_command)
    physical_upper = YOBOGO_COMMAND_PHYSICAL_LIMITS[:, 1].to(yobogo_command)
    physical_command = torch.clamp(
        yobogo_command, min=physical_lower, max=physical_upper
    )
    rapid_command = physical_command * YOBOGO_TO_RAPID_COMMAND_SCALE.to(
        physical_command
    )
    lower = RAPID_COMMAND_LIMITS[:, 0].to(rapid_command)
    upper = RAPID_COMMAND_LIMITS[:, 1].to(rapid_command)
    return torch.clamp(rapid_command, min=lower, max=upper)


def restore_yobogo_joint_positions(yobogo_joint_pos_rel: torch.Tensor) -> torch.Tensor:
    """由 Isaac `joint_pos_rel` 加回 SIM 初态，还原绝对 YoboGo 关节角。"""
    if yobogo_joint_pos_rel.shape[-1] != ACTION_DIM:
        raise ValueError("joint_pos_rel 必须为 12 维")
    return yobogo_joint_pos_rel + YOBOGO_SIM_INITIAL_JOINT_POS.to(yobogo_joint_pos_rel)


def build_rapid_observation(
    yobogo_observation: torch.Tensor,
    previous_rapid_action: torch.Tensor,
    rapid_command_override: torch.Tensor | None = None,
) -> torch.Tensor:
    """按已验证顺序把 48 维 YoboGo 观测转换为 42 维 Rapid 当前观测。"""
    _validate_yobogo_observation(yobogo_observation)
    if previous_rapid_action.shape != (yobogo_observation.shape[0], ACTION_DIM):
        raise ValueError(
            f"previous_rapid_action 必须为 ({yobogo_observation.shape[0]},{ACTION_DIM})，"
            f"实际为 {tuple(previous_rapid_action.shape)}"
        )
    if not torch.isfinite(previous_rapid_action).all():
        raise ValueError("previous_rapid_action 含 NaN/Inf")

    projected_gravity = yobogo_observation[:, YOBOGO_GRAVITY_SLICE]
    if rapid_command_override is None:
        rapid_command = map_yobogo_command_to_rapid(
            yobogo_observation[:, YOBOGO_COMMAND_SLICE]
        )
    else:
        if rapid_command_override.shape != (yobogo_observation.shape[0], 3):
            raise ValueError(
                "rapid_command_override 必须为 (N,3)："
                f"{tuple(rapid_command_override.shape)}"
            )
        if not torch.isfinite(rapid_command_override).all():
            raise ValueError("rapid_command_override 含 NaN/Inf")
        rapid_command = rapid_command_override.to(yobogo_observation)
        rapid_command = torch.cat(
            (
                rapid_command[:, 0:1].clamp(-0.6, 0.6),
                rapid_command[:, 1:2].clamp(-0.6, 0.6),
                rapid_command[:, 2:3].clamp(-1.0, 1.0),
            ),
            dim=-1,
        )
    yobogo_joint_pos = restore_yobogo_joint_positions(
        yobogo_observation[:, YOBOGO_JOINT_POS_SLICE]
    )
    rapid_joint_pos = yobogo_joint_pos * RAPID_TO_YOBOGO_AXIS_SIGN.to(yobogo_joint_pos)
    q_default = rapid_joint_pos - RAPID_DEFAULT_JOINT_POS.to(rapid_joint_pos)

    yobogo_joint_vel = yobogo_observation[:, YOBOGO_JOINT_VEL_SLICE]
    rapid_joint_vel_scaled = (
        yobogo_joint_vel * RAPID_TO_YOBOGO_AXIS_SIGN.to(yobogo_joint_vel) * 0.05
    )

    rapid_observation = torch.cat(
        (
            projected_gravity,
            rapid_command,
            q_default,
            rapid_joint_vel_scaled,
            previous_rapid_action,
        ),
        dim=-1,
    )
    if rapid_observation.shape != (yobogo_observation.shape[0], OBS_DIM):
        raise RuntimeError(
            f"Rapid observation shape 错误：{tuple(rapid_observation.shape)}"
        )
    if not torch.isfinite(rapid_observation).all():
        raise ValueError("Rapid observation 含 NaN/Inf")
    return rapid_observation


def rapid_action_to_yobogo_action(
    rapid_raw_action: torch.Tensor,
    yobogo_observation: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """把 Rapid 原始动作转换为安全的 YoboGo 12 维位置动作。

    返回值依次为 YoboGo 动作、安全目标、合并裁剪、位置裁剪和力矩裁剪。
    历史中的上一动作始终保存 `rapid_raw_action` 原值，不保存此处的 Isaac
    动作，以保持 Rapid 原始动作语义。
    """
    _validate_yobogo_observation(yobogo_observation)
    if rapid_raw_action.shape != (yobogo_observation.shape[0], ACTION_DIM):
        raise ValueError(
            f"Rapid action 必须为 ({yobogo_observation.shape[0]},{ACTION_DIM})，"
            f"实际为 {tuple(rapid_raw_action.shape)}"
        )
    if not torch.isfinite(rapid_raw_action).all():
        raise ValueError("Rapid action 含 NaN/Inf")

    applied_rapid_action = rapid_raw_action.clamp(
        -RAPID_REPLAY_RAW_ACTION_LIMIT, RAPID_REPLAY_RAW_ACTION_LIMIT
    )
    rapid_target = RAPID_DEFAULT_JOINT_POS.to(applied_rapid_action) + (
        RAPID_ACTION_SCALE.to(applied_rapid_action) * applied_rapid_action
    )
    yobogo_target = rapid_target * RAPID_TO_YOBOGO_AXIS_SIGN.to(rapid_target)
    safe_target = yobogo_target.clone()

    # 先执行 URDF 位置硬限位。
    position_lower = YOBOGO_JOINT_POS_LIMITS[:, 0].to(safe_target)
    position_upper = YOBOGO_JOINT_POS_LIMITS[:, 1].to(safe_target)
    before_position = safe_target.clone()
    safe_target.clamp_(min=position_lower, max=position_upper)
    position_clipped = ~torch.isclose(
        before_position, safe_target, atol=1.0e-6, rtol=0.0
    )

    # 再按官方 Rapid 回放 Kp/Kd 和 URDF 力矩反推安全目标区间。
    current_joint_pos = restore_yobogo_joint_positions(
        yobogo_observation[:, YOBOGO_JOINT_POS_SLICE]
    ).to(safe_target)
    current_joint_vel = yobogo_observation[:, YOBOGO_JOINT_VEL_SLICE].to(safe_target)
    kd = RAPID_REPLAY_PD_KD.to(safe_target)
    torque_limit = RAPID_REPLAY_TORQUE_LIMITS.to(safe_target)
    torque_lower = current_joint_pos + (
        kd * current_joint_vel - torque_limit
    ) / RAPID_REPLAY_PD_KP
    torque_upper = current_joint_pos + (
        kd * current_joint_vel + torque_limit
    ) / RAPID_REPLAY_PD_KP
    torque_lower.clamp_(min=position_lower, max=position_upper)
    torque_upper.clamp_(min=position_lower, max=position_upper)

    before_safety = safe_target.clone()
    safe_target.clamp_(min=torque_lower, max=torque_upper)
    torque_clipped = ~torch.isclose(
        before_safety, safe_target, atol=1.0e-6, rtol=0.0
    )
    was_clipped = position_clipped | torque_clipped

    yobogo_action = (safe_target - YOBOGO_SIM_INITIAL_JOINT_POS.to(safe_target)) / (
        YOBOGO_ACTION_SCALE
    )
    if not torch.isfinite(yobogo_action).all():
        raise ValueError("YoboGo action 含 NaN/Inf")
    return (
        yobogo_action,
        safe_target,
        was_clipped,
        position_clipped,
        torque_clipped,
    )


@dataclass
class AdapterStep:
    """一次 500 Hz 控制周期的外部策略输出与诊断信息。"""

    yobogo_action: torch.Tensor
    did_infer: bool
    rapid_raw_action: torch.Tensor
    applied_rapid_action: torch.Tensor
    policy_raw_action: torch.Tensor
    rapid_observation: torch.Tensor | None
    flattened_history: torch.Tensor | None
    safe_joint_target: torch.Tensor
    safety_clipped: torch.Tensor
    position_clipped: torch.Tensor
    torque_clipped: torch.Tensor


class RapidLocomotionAdapter:
    """维护历史、50 Hz 调度、原始动作语义和安全动作转换的冻结策略适配器。"""

    def __init__(
        self,
        policy: RapidPolicyProtocol,
        num_envs: int,
        device: torch.device | str,
        inference_period: int = INFERENCE_PERIOD_CONTROL_STEPS,
    ) -> None:
        if num_envs < 1:
            raise ValueError("num_envs 必须大于 0")
        if inference_period != INFERENCE_PERIOD_CONTROL_STEPS:
            raise ValueError("Rapid 推理周期必须为 10 个 500 Hz 控制周期")
        self.policy = policy
        self.num_envs = num_envs
        self.device = torch.device(device)
        self.inference_period = inference_period
        self.history = torch.zeros(
            num_envs, HISTORY_LEN, OBS_DIM, dtype=torch.float32, device=self.device
        )
        self.previous_rapid_action = torch.zeros(
            num_envs, ACTION_DIM, dtype=torch.float32, device=self.device
        )
        self.held_yobogo_action = torch.zeros(
            num_envs, ACTION_DIM, dtype=torch.float32, device=self.device
        )
        self.held_safe_joint_target = YOBOGO_SIM_INITIAL_JOINT_POS.to(self.device).repeat(
            num_envs, 1
        )
        self.held_safety_clipped = torch.zeros(
            num_envs, ACTION_DIM, dtype=torch.bool, device=self.device
        )
        self.control_cycle = torch.zeros(
            num_envs, dtype=torch.long, device=self.device
        )
        # 官方 HistoryWrapper.reset() 返回全零历史，play.py 首次直接推理；
        # 该标志保证 reset 后首次 50 Hz 调用同样使用全零历史。
        self.history_initialized = torch.zeros(
            num_envs, dtype=torch.bool, device=self.device
        )

    @torch.inference_mode()
    def reset(self, env_ids: torch.Tensor | None = None) -> None:
        """清零指定环境的历史、上一动作、调度计数和保持动作。"""
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        else:
            env_ids = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
            if env_ids.ndim != 1:
                raise ValueError("env_ids 必须为一维")
            if env_ids.numel() == 0:
                return
            if env_ids.min() < 0 or env_ids.max() >= self.num_envs:
                raise IndexError("env_ids 越界")

        self.history[env_ids] = 0.0
        self.previous_rapid_action[env_ids] = 0.0
        self.held_yobogo_action[env_ids] = 0.0
        self.held_safe_joint_target[env_ids] = YOBOGO_SIM_INITIAL_JOINT_POS.to(self.device)
        self.held_safety_clipped[env_ids] = False
        self.control_cycle[env_ids] = 0
        self.history_initialized[env_ids] = False

    @torch.inference_mode()
    def step(
        self,
        yobogo_observation: torch.Tensor,
        reset_env_ids: torch.Tensor | None = None,
        rapid_command_override: torch.Tensor | None = None,
    ) -> AdapterStep:
        """执行一个 500 Hz 控制周期，仅在每第 10 周期更新 50 Hz 策略。"""
        _validate_yobogo_observation(yobogo_observation)
        yobogo_observation = yobogo_observation.to(self.device, copy=True)
        if reset_env_ids is not None and (
            torch.as_tensor(reset_env_ids, dtype=torch.long).numel() > 0
        ):
            self.reset(reset_env_ids)

        infer_mask = torch.remainder(self.control_cycle, self.inference_period) == 0
        if not bool(infer_mask.any()):
            # 非推理周期直接复用上一目标，即在 10 个 500 Hz 周期内保持
            # 50 Hz 关节目标，不额外做会改变训练语义的高频插值。
            self.control_cycle.add_(1)
            return AdapterStep(
                yobogo_action=self.held_yobogo_action.clone(),
                did_infer=False,
                rapid_raw_action=self.previous_rapid_action.clone(),
                applied_rapid_action=self.previous_rapid_action.clamp(
                    -RAPID_REPLAY_RAW_ACTION_LIMIT,
                    RAPID_REPLAY_RAW_ACTION_LIMIT,
                ),
                policy_raw_action=self.previous_rapid_action.clone(),
                rapid_observation=None,
                flattened_history=None,
                safe_joint_target=self.held_safe_joint_target.clone(),
                safety_clipped=self.held_safety_clipped.clone(),
                position_clipped=self.held_safety_clipped.clone(),
                torque_clipped=self.held_safety_clipped.clone(),
            )

        rapid_observation = build_rapid_observation(
            yobogo_observation,
            self.previous_rapid_action,
            rapid_command_override=rapid_command_override,
        )
        first_inference = ~self.history_initialized
        if bool(first_inference.any()):
            # 复现官方 reset 后首次全零历史，不把当前观测提前写入。
            self.history[first_inference] = 0.0
        update_ids = ~first_inference
        if bool(update_ids.any()):
            self.history[update_ids] = torch.cat(
                (self.history[update_ids, 1:], rapid_observation[update_ids, None]),
                dim=1,
            )
        self.history_initialized.fill_(True)
        flattened_history = self.history.reshape(self.num_envs, HISTORY_DIM)
        policy_raw_action, _latent = self.policy(rapid_observation, flattened_history)
        if policy_raw_action.shape != (self.num_envs, ACTION_DIM):
            raise RuntimeError(
                f"Rapid policy action shape 错误：{tuple(policy_raw_action.shape)}"
            )
        if not torch.isfinite(policy_raw_action).all():
            raise ValueError("Rapid policy 输出含 NaN/Inf")
        # history/previous action 始终记录网络原始 Rapid 动作；
        # 仅执行支路经过保守 OOD 门限，之后仍继续位置与力矩安全裁剪。
        rapid_raw_action = policy_raw_action
        applied_rapid_action = policy_raw_action.clamp(
            -RAPID_REPLAY_RAW_ACTION_LIMIT, RAPID_REPLAY_RAW_ACTION_LIMIT
        )

        (
            yobogo_action,
            safe_target,
            was_clipped,
            position_clipped,
            torque_clipped,
        ) = rapid_action_to_yobogo_action(rapid_raw_action, yobogo_observation)
        self.previous_rapid_action.copy_(rapid_raw_action)
        self.held_yobogo_action.copy_(yobogo_action)
        self.held_safe_joint_target.copy_(safe_target)
        self.held_safety_clipped.copy_(was_clipped)
        self.control_cycle.add_(1)
        return AdapterStep(
            yobogo_action=yobogo_action.clone(),
            did_infer=True,
            rapid_raw_action=rapid_raw_action.clone(),
            applied_rapid_action=applied_rapid_action.clone(),
            policy_raw_action=policy_raw_action.clone(),
            rapid_observation=rapid_observation.clone(),
            flattened_history=flattened_history.clone(),
            safe_joint_target=safe_target.clone(),
            safety_clipped=was_clipped.clone(),
            position_clipped=position_clipped.clone(),
            torque_clipped=torque_clipped.clone(),
        )
