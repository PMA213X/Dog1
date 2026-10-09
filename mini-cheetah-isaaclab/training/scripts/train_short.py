"""YoboGo 100 iteration 内的 RSL-RL 短训练入口。"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
from importlib import metadata
import json
import os
from pathlib import Path
import re
import sys
import traceback
from uuid import uuid4

# 运行脚本时不生成任何 __pycache__ 文件。
sys.dont_write_bytecode = True

from _bootstrap import PROJECT_ROOT, TRAINING_ROOT, ensure_project_on_path


ensure_project_on_path()

from isaaclab.app import AppLauncher  # noqa: E402


parser = argparse.ArgumentParser(description="YoboGo RSL-RL 短训练")
parser.add_argument("--max_iterations", type=int, default=100, help="PPO 迭代数，最大 100")
parser.add_argument("--num_envs", type=int, default=4, help="短训练环境数")
parser.add_argument("--seed", type=int, default=0, help="环境与智能体随机种子")
parser.add_argument("--stage", default="ppo-short", help="训练阶段目录名")
parser.add_argument("--run_id", default=None, help="指定运行 ID；默认生成 UTC 时间与随机后缀")
parser.add_argument("--disable_fabric", action="store_true", default=False, help="关闭 Fabric USD I/O 加速")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

if not 1 <= args_cli.max_iterations <= 100:
    raise ValueError("短训练 max_iterations 必须位于 1..100")
if args_cli.num_envs < 1:
    raise ValueError("num_envs 必须大于 0")
if not re.fullmatch(r"[A-Za-z0-9_.-]+", args_cli.stage):
    raise ValueError("stage 只允许字母、数字、下划线、点和连字符")
if args_cli.run_id is None:
    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%d_%H-%M-%SZ")
    args_cli.run_id = f"{stamp}_{uuid4().hex[:6]}"
if not re.fullmatch(r"[A-Za-z0-9_.-]+", args_cli.run_id):
    raise ValueError("run_id 只允许字母、数字、下划线、点和连字符")

task_name = "YoboGo-Velocity-Flat-v0"
base_dir = TRAINING_ROOT / "checkpoints" / task_name / args_cli.stage / str(args_cli.seed) / args_cli.run_id
checkpoint_dir = base_dir.resolve()
run_dir = (TRAINING_ROOT / "runs" / task_name / args_cli.stage / str(args_cli.seed) / args_cli.run_id).resolve()
log_dir = (TRAINING_ROOT / "logs" / task_name / args_cli.stage / str(args_cli.seed) / args_cli.run_id).resolve()
for output_dir in (checkpoint_dir, run_dir, log_dir):
    training_root = TRAINING_ROOT.resolve()
    if training_root not in output_dir.parents:
        raise ValueError(f"训练产物必须位于 training 下：{output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)

log_path = log_dir / "train.log"


class _Tee:
    """把控制台输出同时写入 training/logs 下的日志文件。"""

    def __init__(self, stream, file_stream):
        self._stream = stream
        self._file = file_stream

    def write(self, data):
        self._stream.write(data)
        self._file.write(data)
        return len(data)

    def flush(self):
        self._stream.flush()
        self._file.flush()


log_file = log_path.open("w", encoding="utf-8")
sys.stdout = _Tee(sys.__stdout__, log_file)
sys.stderr = _Tee(sys.__stderr__, log_file)
os.chdir(TRAINING_ROOT)

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""以下代码必须在 Isaac Sim 启动后导入。"""

from datetime import datetime, timezone  # noqa: E402
import hashlib  # noqa: E402
from importlib import metadata  # noqa: E402
import json  # noqa: E402
from pathlib import Path  # noqa: E402

import gymnasium as gym  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402
import rsl_rl.runners.on_policy_runner as runner_module  # noqa: E402
import torch  # noqa: E402

import isaaclab_tasks  # noqa: F401,E402
from isaaclab.envs import (  # noqa: E402
    DirectMARLEnv,
    ManagerBasedRLEnvCfg,
    multi_agent_to_single_agent,
)
from isaaclab.utils.dict import print_dict  # noqa: E402
from isaaclab.utils.io import dump_pickle, dump_yaml  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper  # noqa: E402
from isaaclab_tasks.utils import load_cfg_from_registry, parse_env_cfg  # noqa: E402

import training.envs  # noqa: F401,E402
from training.envs.robot_cfg import (  # noqa: E402
    CONTROL_PERIOD_S,
    JOINT_NAMES,
    USD_PATH,
)


def _sha256(path: Path) -> str:
    """流式计算文件 SHA256。"""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_json(path: Path, payload: dict) -> None:
    """原子式覆盖写入 JSON 报告。"""
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def _write_run_metadata(env_cfg, agent_cfg) -> None:
    """写入环境锁、数据清单、参数与待验收 Gate。"""
    versions = {}
    for package in ("isaacsim", "isaaclab", "isaaclab-tasks", "isaaclab-rl", "rsl-rl-lib", "gymnasium"):
        try:
            versions[package] = metadata.version(package)
        except metadata.PackageNotFoundError:
            versions[package] = "missing"
    lock_lines = [
        f"python={sys.version.split()[0]}",
        f"torch={torch.__version__}",
        f"torch_cuda={torch.version.cuda}",
        f"cuda_available={torch.cuda.is_available()}",
    ]
    lock_lines.extend(f"{name}={version}" for name, version in versions.items())
    lock_lines.append(f"usd_path={USD_PATH}")
    lock_lines.append(f"usd_status={'present' if USD_PATH.is_file() else 'pending'}")
    (run_dir / "environment-lock.txt").write_text("\n".join(lock_lines) + "\n", encoding="utf-8")

    manifest_inputs = [
        PROJECT_ROOT / "research" / "data-manifest.md",
        PROJECT_ROOT.parent / "YoboGo-control" / "robot-software" / "config" / "initial_jpos_ctrl.yaml",
        PROJECT_ROOT.parent / "YoboGo-control" / "robot-software" / "config" / "mc-mit-ctrl-user-parameters.yaml",
        USD_PATH,
    ]
    manifest_lines = []
    for path in manifest_inputs:
        if path.is_file():
            manifest_lines.append(f"{_sha256(path)}  {path}")
        else:
            manifest_lines.append(f"PENDING  {path}")
    (run_dir / "data-manifest.sha256").write_text("\n".join(manifest_lines) + "\n", encoding="utf-8")

    dump_yaml(str(run_dir / "params" / "env.yaml"), env_cfg)
    dump_yaml(str(run_dir / "params" / "agent.yaml"), agent_cfg)
    dump_pickle(str(run_dir / "params" / "env.pkl"), env_cfg)
    dump_pickle(str(run_dir / "params" / "agent.pkl"), agent_cfg)
    _write_json(
        run_dir / "gate.json",
        {
            "status": "pending",
            "task": task_name,
            "reason": "短训练完成不等于站立/速度 Gate；需由 check_env 与后续评估验收",
        },
    )


def _write_checkpoint_manifest() -> list[dict]:
    """为 checkpoint 目录生成哈希清单。"""
    checkpoints = []
    for path in sorted(checkpoint_dir.glob("model_*.pt")):
        checkpoints.append({"file": path.name, "size": path.stat().st_size, "sha256": _sha256(path)})
    payload = {"task": task_name, "stage": args_cli.stage, "seed": args_cli.seed, "checkpoints": checkpoints}
    _write_json(checkpoint_dir / "checkpoint-manifest.json", payload)
    return checkpoints


def main() -> int:
    """执行 100 iteration 内的 YoboGo PPO 短训练。"""
    if not USD_PATH.is_file():
        raise FileNotFoundError(f"项目 USD 尚未生成或不在约定路径：{USD_PATH}")

    env_cfg: ManagerBasedRLEnvCfg = parse_env_cfg(
        task_name,
        device=args_cli.device,
        num_envs=args_cli.num_envs,
        use_fabric=not args_cli.disable_fabric,
    )
    control_period = env_cfg.sim.dt * env_cfg.decimation
    if abs(control_period - CONTROL_PERIOD_S) > 1.0e-12:
        raise RuntimeError(f"控制周期错误：{control_period}")
    if env_cfg.scene.robot.spawn.usd_path != str(USD_PATH):
        raise RuntimeError("场景 USD 路径错误")

    agent_cfg: RslRlOnPolicyRunnerCfg = load_cfg_from_registry(task_name, "rsl_rl_cfg_entry_point")
    agent_cfg.seed = args_cli.seed
    agent_cfg.device = args_cli.device
    agent_cfg.max_iterations = args_cli.max_iterations
    env_cfg.seed = args_cli.seed
    _write_run_metadata(env_cfg, agent_cfg)

    print(f"[INFO] task={task_name}")
    print(f"[INFO] checkpoint_dir={checkpoint_dir}")
    print(f"[INFO] run_dir={run_dir}")
    print(f"[INFO] log_dir={log_dir}")
    print(f"[INFO] control_period_s={control_period}")
    print(f"[INFO] joint_names={JOINT_NAMES}")
    print_dict(agent_cfg.to_dict(), nesting=4)

    env = gym.make(task_name, cfg=env_cfg)
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)
    env = RslRlVecEnvWrapper(env)

    # 本任务禁止执行 Git；RSL-RL 默认会读取仓库状态，这里显式改为无 Git 产物。
    runner_module.store_code_state = lambda log_dir_path, repositories: []
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=str(run_dir), device=agent_cfg.device)

    original_save = runner.save

    def save_to_checkpoint(path, infos=None):
        """把 RSL-RL 的 model_*.pt 重定向到 training/checkpoints。"""
        target = checkpoint_dir / Path(path).name
        target.parent.mkdir(parents=True, exist_ok=True)
        return original_save(str(target), infos)

    runner.save = save_to_checkpoint

    try:
        runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)
        checkpoints = _write_checkpoint_manifest()
        if not checkpoints:
            raise RuntimeError("训练结束但没有生成 checkpoint")
        _write_json(
            run_dir / "metrics.json",
            {
                "status": "training_completed",
                "task": task_name,
                "seed": args_cli.seed,
                "num_envs": args_cli.num_envs,
                "requested_iterations": args_cli.max_iterations,
                "completed_iteration": runner.current_learning_iteration,
                "total_timesteps": runner.tot_timesteps,
                "checkpoint_count": len(checkpoints),
                "evaluation_status": "pending",
            },
        )
        _write_json(
            run_dir / "gate.json",
            {
                "status": "pending",
                "task": task_name,
                "reason": "训练完成；站立、速度误差、跌倒率与动作饱和度仍待独立评估",
            },
        )
        print(f"[INFO] checkpoints={len(checkpoints)}")
        return 0
    except Exception:
        _write_json(
            run_dir / "metrics.json",
            {"status": "failed", "task": task_name, "error": traceback.format_exc()},
        )
        _write_json(
            run_dir / "gate.json",
            {"status": "failed", "task": task_name, "reason": "PPO 短训练异常"},
        )
        raise
    finally:
        env.close()


if __name__ == "__main__":
    try:
        exit_code = main()
    except Exception:
        traceback.print_exc()
        exit_code = 1
    finally:
        simulation_app.close()
        sys.stdout = sys.__stdout__
        sys.stderr = sys.__stderr__
        log_file.close()
    sys.exit(exit_code)
