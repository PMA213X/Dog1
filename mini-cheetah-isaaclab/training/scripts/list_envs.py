"""列出并验证本项目的 Gym 环境注册。"""

from __future__ import annotations

import argparse
import sys

# 运行脚本时不生成任何 __pycache__ 文件。
sys.dont_write_bytecode = True

from _bootstrap import ensure_project_on_path


ensure_project_on_path()

from isaaclab.app import AppLauncher  # noqa: E402


parser = argparse.ArgumentParser(description="列出 YoboGo Isaac Lab 任务")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""以下代码必须在 Isaac Sim 启动后导入。"""

import gymnasium as gym  # noqa: E402

import isaaclab_tasks  # noqa: F401,E402
import training.envs  # noqa: F401,E402
from isaaclab_tasks.utils import load_cfg_from_registry  # noqa: E402
from training.envs.robot_cfg import (  # noqa: E402
    CONTROL_PERIOD_S,
    RSL_RL_CFG_ENTRY_POINT,
    TASK_ID,
    USD_PATH,
)


def main() -> int:
    """验证任务、入口与配置周期并打印注册详情。"""
    spec = gym.spec(TASK_ID)
    env_entry = spec.kwargs.get("env_cfg_entry_point")
    agent_entry = spec.kwargs.get("rsl_rl_cfg_entry_point")
    if env_entry != "training.envs.velocity_flat_env_cfg:YoboGoVelocityFlatEnvCfg":
        raise RuntimeError(f"环境入口错误：{env_entry}")
    if agent_entry != RSL_RL_CFG_ENTRY_POINT:
        raise RuntimeError(f"智能体入口错误：{agent_entry}")

    env_cfg = load_cfg_from_registry(TASK_ID, "env_cfg_entry_point")
    period = env_cfg.sim.dt * env_cfg.decimation
    if abs(period - CONTROL_PERIOD_S) > 1.0e-12:
        raise RuntimeError(f"控制周期错误：{period}")

    print(f"task={TASK_ID}")
    print(f"entry_point={spec.entry_point}")
    print(f"env_cfg_entry_point={env_entry}")
    print(f"rsl_rl_cfg_entry_point={agent_entry}")
    print(f"physics_dt={env_cfg.sim.dt}")
    print(f"decimation={env_cfg.decimation}")
    print(f"control_period_s={period}")
    print(f"usd_path={USD_PATH}")
    print(f"usd_status={'present' if USD_PATH.is_file() else 'pending'}")
    return 0


if __name__ == "__main__":
    try:
        exit_code = main()
    except Exception:
        import traceback

        traceback.print_exc()
        exit_code = 1
    finally:
        simulation_app.close()
    sys.exit(exit_code)
