"""注册 YoboGo manager-based 训练任务。"""

import gymnasium as gym

from .robot_cfg import ENV_CFG_ENTRY_POINT, RSL_RL_CFG_ENTRY_POINT, TASK_ID


if TASK_ID not in gym.registry:
    gym.register(
        id=TASK_ID,
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": ENV_CFG_ENTRY_POINT,
            "rsl_rl_cfg_entry_point": RSL_RL_CFG_ENTRY_POINT,
        },
    )

