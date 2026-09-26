#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PPO 训练 / 回放 的集中配置。

所有超参数、奖励权重、观测与动作维度常量都放在这里，
改动请优先改本文件，避免散落在 train_ppo.py / play.py / walk_env.py 中。

约定的环境接口（webots-sim/rl/walk_env.py 中的 QuadrupedWalkEnv）：
    obs    : float32, shape = (OBS_DIM,)
    action : float32, shape = (ACTION_DIM,), 取值范围 [ACTION_LOW, ACTION_HIGH]
    reset() -> (obs, info)
    step(a) -> (obs, reward, terminated, truncated, info)
"""

from __future__ import annotations

from typing import Any, Dict, List

# ---------------------------------------------------------------------------
# 环境常量：观测 / 动作
# ---------------------------------------------------------------------------
OBS_DIM: int = 42          # 观测向量维度
OBS_DIM_TURN: int = 45     # P3 转向观测维度 = 42 + yaw_err + wz + vx
OBS_DIM_STAIRS: int = 51   # P4 台阶观测维度 = 45 + 6 维前下方地形采样
ACTION_DIM: int = 12       # 动作向量维度（12 个关节）
ACTION_LOW: float = -1.0   # 动作下界
ACTION_HIGH: float = 1.0   # 动作上界

# 单个 episode 最大步数（超过则 truncated）。与 walk_env.py 中的 max_episode_steps 保持一致。
MAX_EPISODE_STEPS: int = 1000

# 仿真步长（秒）。仅作记录/文档用，真正 dt 由 walk_env.py 决定。
SIM_DT: float = 0.02       # 50 Hz 控制频率

# ---------------------------------------------------------------------------
# PPO 超参数（Stable-Baselines3 PPO）
# ---------------------------------------------------------------------------
PPO_POLICY: str = "MlpPolicy"

PPO_PARAMS: Dict[str, Any] = {
    "n_steps": 2048,         # 每次策略更新收集的环境步数
    "batch_size": 256,       # mini-batch 大小
    "n_epochs": 10,          # 每次更新对同一批数据的优化轮数
    "gamma": 0.99,           # 折扣因子
    "gae_lambda": 0.95,      # GAE lambda
    "learning_rate": 3e-4,   # 学习率
    "ent_coef": 0.0,         # 熵正则系数
    "clip_range": 0.2,       # PPO 裁剪范围
}

# 策略 / 价值网络结构
NET_ARCH: List[int] = [256, 256]

# ---------------------------------------------------------------------------
# 训练过程控制
# ---------------------------------------------------------------------------
DEFAULT_TOTAL_STEPS: int = 200_000   # 默认训练总步数
CHECKPOINT_INTERVAL: int = 10_000    # 每多少步存一次 checkpoint
EVAL_INTERVAL: int = 20_000          # 每多少步做一次评估
EVAL_EPISODES: int = 10              # 每次评估跑多少个 episode（取平均）
DEFAULT_DEVICE: str = "cuda"         # 默认计算设备（无 GPU 时可 --device cpu）

# ---------------------------------------------------------------------------
# 奖励权重（walk_env.py 直接 import 使用；两边必须一致）
# 奖励公式：r = alive_bonus + forward_vel*vx
#           + roll_penalty*|roll| + pitch_penalty*|pitch|
#           + action_sq*||a||^2
#           + heading_cos*cos(yaw_err) + yaw_rate_penalty*|wz|
#           + heading_bonus*[|yaw_err|<0.2]                   （P3）
#           + climb_reward*爬升 + on_step_reward*站上台阶指示
#           + drop_penalty*从台阶掉下指示                        （P4）
#           ；摔倒再加 fall_penalty
#
# 分阶段权重（PHASE_REWARD_WEIGHTS），用 set_phase() 切换：
#   P1 站立 : alive+1, -2|roll|, -2|pitch|, -10 fallen
#   P2 行走 : P1 + 3*vx - 0.05*||a||²
#   P3 转向 : P2 + 2*cos(yaw_err) - 0.5*|wz| + 1*(|yaw_err|<0.2)
#   P4 台阶 : P3 + 2*爬升 + 1*站上台阶 - 3*从台阶掉下（stairs）
#             另有 step = P2 + 爬升（无转向，供对照）
# ---------------------------------------------------------------------------
REWARD_WEIGHTS: Dict[str, float] = {
    "alive_bonus": 1.00,       # 每步存活奖励
    "forward_vel": 3.00,       # 前进速度 vx 奖励系数（任务默认 = 3.0）
    "roll_penalty": -2.00,     # |roll| 惩罚系数
    "pitch_penalty": -2.00,    # |pitch| 惩罚系数
    "action_sq": -0.05,        # ||a||^2 动作能耗惩罚系数（任务默认 = -0.05）
    "fall_penalty": -10.00,    # 摔倒（提前终止）惩罚
    # ---- P3 转向（默认关闭）----
    "heading_cos": 0.00,       # cos(yaw_err) 朝向目标奖励（P3 = 2.0）
    "yaw_rate_penalty": 0.00,  # |wz| 转向平滑惩罚（P3 = -0.5）
    "heading_bonus": 0.00,     # |yaw_err|<0.2 指示奖励（P3 = 1.0）
    # ---- P4 台阶（默认关闭）----
    "climb_reward": 0.00,      # 爬升高度奖励（P4 = 2.0）
    "on_step_reward": 0.00,    # 站上台阶指示奖励（P4 = 1.0）
    "drop_penalty": 0.00,      # 从台阶掉下惩罚（P4 = -3.0）
}

# 各阶段完整权重表（set_phase 会先清零 P3/P4 可选项再合并，避免残留）
PHASE_REWARD_WEIGHTS: Dict[str, Dict[str, float]] = {
    # Phase 1 站立：只求不倒
    "stand": {
        "alive_bonus": 1.00,
        "forward_vel": 0.00,
        "roll_penalty": -2.00,
        "pitch_penalty": -2.00,
        "action_sq": 0.00,
        "fall_penalty": -10.00,
    },
    # Phase 2 行走：站立 + 前进速度 + 动作能耗
    "walk": {
        "alive_bonus": 1.00,
        "forward_vel": 3.00,
        "roll_penalty": -2.00,
        "pitch_penalty": -2.00,
        "action_sq": -0.05,
        "fall_penalty": -10.00,
    },
    # Phase 3 转向：行走 + 朝向目标
    # r = 原奖励 + 2.0*cos(yaw_err) - 0.5*|wz| + 1.0*(|yaw_err|<0.2)
    "turn": {
        "alive_bonus": 1.00,
        "forward_vel": 3.00,
        "roll_penalty": -2.00,
        "pitch_penalty": -2.00,
        "action_sq": -0.05,
        "fall_penalty": -10.00,
        "heading_cos": 2.00,
        "yaw_rate_penalty": -0.50,
        "heading_bonus": 1.00,
    },
    # Phase 4 台阶：行走 + 爬升（不含转向塑形，供纯爬台阶对照）
    "step": {
        "alive_bonus": 1.00,
        "forward_vel": 3.00,
        "roll_penalty": -2.00,
        "pitch_penalty": -2.00,
        "action_sq": -0.05,
        "fall_penalty": -10.00,
        "climb_reward": 2.00,
        "on_step_reward": 1.00,
        "drop_penalty": -3.00,
    },
    # Phase 4 台阶（P4 正式）：行走 + 转向 + 爬台阶
    # r = 原奖励 + 2.0*cos(yaw_err) - 0.5*|wz| + 1.0*(|yaw_err|<0.2)
    #     + 2.0*max(Δz,0) + 1.0*站上台阶 - 3.0*从台阶掉下
    "stairs": {
        "alive_bonus": 1.00,
        "forward_vel": 3.00,
        "roll_penalty": -2.00,
        "pitch_penalty": -2.00,
        "action_sq": -0.05,
        "fall_penalty": -10.00,
        "heading_cos": 2.00,
        "yaw_rate_penalty": -0.50,
        "heading_bonus": 1.00,
        "climb_reward": 2.00,
        "on_step_reward": 1.00,
        "drop_penalty": -3.00,
    },
}

# 当前阶段名（train_ppo.py --tag / set_phase 会改写）
CURRENT_PHASE: str = "stand"


def set_phase(phase: str) -> Dict[str, float]:
    """按阶段名切换 REWARD_WEIGHTS（就地更新并返回）。

    phase 取值：stand / walk / turn / step（兼容 phase1_stand 等别名）。
    """
    aliases = {
        # P0 冒烟：权重与 P1 站立一致，仅用于验证训练管线
        "phase0": "stand", "phase0_smoke": "stand", "smoke": "stand",
        "phase1": "stand", "phase1_stand": "stand", "stand": "stand",
        "phase2": "walk", "phase2_walk": "walk", "walk": "walk",
        "phase3": "turn", "phase3_turn": "turn", "turn": "turn",
        "phase4": "stairs", "phase4_step": "step", "step": "step",
        "phase4_stairs": "stairs", "stairs": "stairs", "p4": "stairs",
    }
    key = aliases.get(phase, phase)
    if key not in PHASE_REWARD_WEIGHTS:
        raise ValueError(f"未知阶段：{phase!r}，可选 {list(PHASE_REWARD_WEIGHTS)}")
    global CURRENT_PHASE
    CURRENT_PHASE = key
    for k in (
        "heading_cos", "yaw_rate_penalty", "heading_bonus",
        "climb_reward", "on_step_reward", "drop_penalty",
    ):
        REWARD_WEIGHTS[k] = 0.0
    REWARD_WEIGHTS.update(PHASE_REWARD_WEIGHTS[key])
    return REWARD_WEIGHTS

# ---------------------------------------------------------------------------
# 路径默认值（train_ppo.py / play.py 会用到；命令行可覆盖）
# ---------------------------------------------------------------------------
DEFAULT_LOGDIR_PREFIX: str = "runs/ppo_walk"      # TensorBoard 日志目录前缀
DEFAULT_CKPT_DIR: str = "checkpoints"             # checkpoint 目录
CKPT_NAME_PREFIX: str = "ppo_walk"                # checkpoint 文件名前缀
