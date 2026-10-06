#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""顺序执行 F0～F3 平地训练，并在阶段间校验 checkpoint 与验收门。"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Optional, Sequence

import flat_jump_contract as contract


PROJECT_ROOT = Path(__file__).resolve().parents[2]
RUN_ROOT = PROJECT_ROOT / "logs" / "yobogo_flat_jump_v1"
CHECKPOINT_DIR = PROJECT_ROOT / "checkpoints" / "yobogo_flat_jump_v1"
RUNS_DIR = PROJECT_ROOT / "runs" / "yobogo_flat_jump_v1"
STATUS_PATH = RUN_ROOT / "stage_status.json"
STAGES_LOG = RUN_ROOT / "stages.log"
F0_SEED_PATH = CHECKPOINT_DIR / f"{contract.CHECKPOINT_PREFIX}_F0_5000_steps.zip"
F1_SEED_PATH = CHECKPOINT_DIR / f"{contract.CHECKPOINT_PREFIX}_F1_250000_steps.zip"

STAGE_LOG_DIRS = {
    "F0": "F0_smoke",
    "F1": "F1_recovery",
    "F1A": "F1A_fixed",
    "F1B": "F1B_curriculum",
    "F2": "F2_move",
    "F3": "F3_jump",
}


def log(message: str) -> None:
    """写入阶段执行日志。"""
    line = f"[{time.strftime('%Y-%m-%dT%H:%M:%S%z')}] {message}"
    print(line, flush=True)
    with STAGES_LOG.open("a", encoding="utf-8") as handle:
        handle.write(line + "\n")


def write_status(payload: dict[str, Any]) -> None:
    """原子写入状态文件。"""
    payload = {
        "schema_version": 1,
        "run_id": "yobogo_flat_jump_v1",
        "updated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        **payload,
    }
    temporary = STATUS_PATH.with_suffix(".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(STATUS_PATH)


def stage_command(
    phase: str,
    resume: Optional[Path],
    *,
    total_steps: Optional[int] = None,
    randomization_mode: str = "full",
    log_key: Optional[str] = None,
) -> list[str]:
    """构造单阶段训练命令。"""
    target = (
        total_steps
        if total_steps is not None
        else contract.PHASE_TOTAL_STEPS[phase]
    )
    log_key = log_key or phase
    command = [
        sys.executable,
        str(PROJECT_ROOT / "webots-sim" / "rl" / "train_flat_jump.py"),
        "--phase",
        phase,
        "--total-steps",
        str(target),
        "--device",
        "cuda",
        "--ckptdir",
        str(CHECKPOINT_DIR),
        "--ckpt-prefix",
        contract.CHECKPOINT_PREFIX,
        "--logdir",
        str(RUNS_DIR / STAGE_LOG_DIRS[log_key]),
        "--checkpoint-interval",
        str(contract.CHECKPOINT_INTERVAL_STEPS),
        "--bridge-port",
        "11451",
        "--seed",
        "20261003",
        "--randomization-mode",
        randomization_mode,
        "--webots-gui",
    ]
    if resume is not None:
        command.extend(["--resume", str(resume)])
    return command


def validate_checkpoint(
    phase: str,
    expected_steps: int,
    checkpoint: Optional[Path] = None,
) -> Path:
    """加载 checkpoint 并校验维度、前缀和累计步数。"""
    checkpoint = checkpoint or (
        CHECKPOINT_DIR / f"{contract.CHECKPOINT_PREFIX}_final.zip"
    )
    contract.validate_checkpoint_path(checkpoint)
    if not checkpoint.is_file():
        raise RuntimeError(f"{phase} checkpoint 不存在：{checkpoint}")
    from stable_baselines3 import PPO

    model = PPO.load(str(checkpoint), device="cuda")
    obs_shape = tuple(
        getattr(getattr(model, "observation_space", None), "shape", ())
    )
    action_shape = tuple(
        getattr(getattr(model, "action_space", None), "shape", ())
    )
    if obs_shape != (contract.OBS_DIM,):
        raise RuntimeError(
            f"{phase} checkpoint 观测维度错误：{obs_shape}"
        )
    if action_shape != (contract.ACTION_DIM,):
        raise RuntimeError(
            f"{phase} checkpoint 动作维度错误：{action_shape}"
        )
    if int(model.num_timesteps) != expected_steps:
        raise RuntimeError(
            f"{phase} checkpoint 步数错误："
            f"{model.num_timesteps} != {expected_steps}"
        )
    log(
        f"{phase} checkpoint 重载通过：{checkpoint} "
        f"steps={model.num_timesteps} obs={obs_shape}"
    )
    return checkpoint


def checkpoint_steps(path: Path) -> int:
    """读取 checkpoint 的累计训练步数，用于判断断点是否已越过目标。"""
    contract.validate_checkpoint_path(path)
    if not path.is_file():
        raise RuntimeError(f"checkpoint 不存在：{path}")
    from stable_baselines3 import PPO

    model = PPO.load(str(path), device="cuda")
    return int(model.num_timesteps)


def run_gate(phase: str) -> None:
    """执行阶段验收门；失败即停止，不自动调整训练。"""
    if phase == "F0":
        return
    command = [
        sys.executable,
        str(PROJECT_ROOT / "webots-sim" / "rl" / "eval_flat_jump.py"),
        "--phase",
        phase,
        "--gate",
        "--output",
        str(RUN_ROOT / f"{phase}_gate.json"),
    ]
    environment = os.environ.copy()
    environment["CUDA_VISIBLE_DEVICES"] = "0"
    environment["PYTHONUNBUFFERED"] = "1"
    log(f"{phase} 验收门命令：{' '.join(command)}")
    completed = subprocess.run(
        command,
        cwd=PROJECT_ROOT,
        env=environment,
        check=False,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"{phase} 验收门失败，退出码={completed.returncode}"
        )
    log(f"{phase} 验收门通过")


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """解析执行器参数。"""
    parser = argparse.ArgumentParser(description="平地 F0～F3 顺序训练")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """顺序训练并在任何失败时立即停止。"""
    args = parse_args(argv)
    if args.dry_run:
        for phase in contract.PHASE_TOTAL_STEPS:
            print(" ".join(stage_command(phase, None)))
        return 0
    if os.environ.get("CUDA_VISIBLE_DEVICES") != "0":
        print("【错误】CUDA_VISIBLE_DEVICES 必须为 0", file=sys.stderr)
        return 2
    RUN_ROOT.mkdir(parents=True, exist_ok=True)
    CHECKPOINT_DIR.mkdir(parents=True, exist_ok=True)
    RUNS_DIR.mkdir(parents=True, exist_ok=True)

    final_checkpoint = CHECKPOINT_DIR / f"{contract.CHECKPOINT_PREFIX}_final.zip"
    if not F0_SEED_PATH.exists() and final_checkpoint.is_file():
        validate_checkpoint("F0", contract.PHASE_TOTAL_STEPS["F0"], final_checkpoint)
        shutil.copy2(final_checkpoint, F0_SEED_PATH)
        log(f"保留 F0 checkpoint：{F0_SEED_PATH}")
    if not F1_SEED_PATH.exists() and final_checkpoint.is_file():
        try:
            validate_checkpoint("F1", 250_000, final_checkpoint)
        except RuntimeError:
            pass
        else:
            shutil.copy2(final_checkpoint, F1_SEED_PATH)
            log(f"保留 F1 checkpoint：{F1_SEED_PATH}")

    resume: Optional[Path] = None
    if F0_SEED_PATH.is_file():
        resume = validate_checkpoint(
            "F0",
            contract.PHASE_TOTAL_STEPS["F0"],
            F0_SEED_PATH,
        )
        log(f"复用已有 F0 checkpoint：{resume}")
    f1_resume: Optional[Path] = None
    if F1_SEED_PATH.is_file():
        f1_resume = validate_checkpoint("F1", 250_000, F1_SEED_PATH)
        log(f"复用已有 F1 checkpoint：{f1_resume}")
    write_status(
        {
            "status": "running",
            "current_phase": "F0",
            "completed_phases": [],
            "tensorboard": "http://127.0.0.1:6006/",
        }
    )
    log("开始 F0～F3 顺序训练")
    try:
        for phase, target in contract.PHASE_TOTAL_STEPS.items():
            if phase == "F0" and resume == F0_SEED_PATH:
                log("F0 已有 checkpoint，跳过重复训练")
                write_status(
                    {
                        "status": "gate",
                        "current_phase": "F0",
                        "completed_phases": ["F0"],
                        "target_steps": target,
                        "checkpoint": str(resume),
                    }
                )
                run_gate("F0")
                continue
            if phase == "F1":
                segments = [
                    (
                        "F1A",
                        contract.F1_FIXED_RECOVERY_STEPS,
                        "fixed",
                        f1_resume or resume,
                    ),
                    (
                        "F1B",
                        contract.F1_RECOVERY_TOTAL_STEPS,
                        "curriculum",
                        None,
                    ),
                ]
                segment_resume: Optional[Path] = None
                for segment_name, segment_target, mode, segment_start in segments:
                    if segment_resume is None:
                        segment_resume = segment_start
                    if (
                        segment_resume is not None
                        and int(
                            checkpoint_steps(segment_resume)
                        )
                        >= segment_target
                    ):
                        log(f"{segment_name} 已达到目标，跳过")
                        continue
                    command = stage_command(
                        "F1",
                        segment_resume,
                        total_steps=segment_target,
                        randomization_mode=mode,
                        log_key=segment_name,
                    )
                    train_log = RUN_ROOT / f"{segment_name}_train.log"
                    write_status(
                        {
                            "status": "training",
                            "current_phase": "F1",
                            "segment": segment_name,
                            "completed_phases": ["F0"],
                            "target_steps": segment_target,
                            "train_log": str(train_log),
                            "randomization_mode": mode,
                        }
                    )
                    log(f"{segment_name} 训练启动：{' '.join(command)}")
                    environment = os.environ.copy()
                    environment["CUDA_VISIBLE_DEVICES"] = "0"
                    environment["PYTHONUNBUFFERED"] = "1"
                    with train_log.open("a", encoding="utf-8") as handle:
                        completed = subprocess.run(
                            command,
                            cwd=PROJECT_ROOT,
                            env=environment,
                            stdout=handle,
                            stderr=subprocess.STDOUT,
                            check=False,
                        )
                    if completed.returncode != 0:
                        raise RuntimeError(
                            f"{segment_name} 训练退出码={completed.returncode}"
                        )
                    segment_resume = validate_checkpoint(
                        "F1", segment_target
                    )
                    if segment_name == "F1A" and not F1_SEED_PATH.exists():
                        shutil.copy2(segment_resume, F1_SEED_PATH)
                        log(f"保留 F1 checkpoint：{F1_SEED_PATH}")
                resume = segment_resume
                write_status(
                    {
                        "status": "gate",
                        "current_phase": "F1",
                        "completed_phases": ["F0", "F1"],
                        "target_steps": contract.F1_RECOVERY_TOTAL_STEPS,
                        "checkpoint": str(resume),
                        "randomization_mode": "full",
                    }
                )
                run_gate("F1")
                continue
            command = stage_command(
                phase,
                resume,
                randomization_mode="full",
            )
            train_log = RUN_ROOT / f"{phase}_train.log"
            write_status(
                {
                    "status": "training",
                    "current_phase": phase,
                    "completed_phases": [
                        item
                        for item in contract.PHASE_TOTAL_STEPS
                        if list(contract.PHASE_TOTAL_STEPS).index(item)
                        < list(contract.PHASE_TOTAL_STEPS).index(phase)
                    ],
                    "target_steps": target,
                    "train_log": str(train_log),
                    "randomization_mode": "full",
                }
            )
            log(f"{phase} 训练启动：{' '.join(command)}")
            environment = os.environ.copy()
            environment["CUDA_VISIBLE_DEVICES"] = "0"
            environment["PYTHONUNBUFFERED"] = "1"
            with train_log.open("a", encoding="utf-8") as handle:
                completed = subprocess.run(
                    command,
                    cwd=PROJECT_ROOT,
                    env=environment,
                    stdout=handle,
                    stderr=subprocess.STDOUT,
                    check=False,
                )
            if completed.returncode != 0:
                raise RuntimeError(
                    f"{phase} 训练退出码={completed.returncode}"
                )
            resume = validate_checkpoint(phase, target)
            if phase == "F0" and not F0_SEED_PATH.exists():
                shutil.copy2(resume, F0_SEED_PATH)
                log(f"保留 F0 checkpoint：{F0_SEED_PATH}")
            write_status(
                {
                    "status": "gate",
                    "current_phase": phase,
                    "completed_phases": [
                        item
                        for item in contract.PHASE_TOTAL_STEPS
                        if list(contract.PHASE_TOTAL_STEPS).index(item)
                        <= list(contract.PHASE_TOTAL_STEPS).index(phase)
                    ],
                    "target_steps": target,
                    "checkpoint": str(resume),
                }
            )
            run_gate(phase)
        write_status(
            {
                "status": "completed",
                "current_phase": "F3",
                "completed_phases": list(contract.PHASE_TOTAL_STEPS),
                "total_steps": contract.PHASE_TOTAL_STEPS["F3"],
                "checkpoint": str(resume),
            }
        )
        log("F0～F3 全部完成")
        return 0
    except Exception as exc:
        current_phase = "unknown"
        if STATUS_PATH.exists():
            try:
                current_phase = str(
                    json.loads(STATUS_PATH.read_text(encoding="utf-8")).get(
                        "current_phase",
                        "unknown",
                    )
                )
            except (json.JSONDecodeError, OSError):
                current_phase = "unknown"
        write_status(
            {
                "status": "failed",
                "current_phase": current_phase,
                "error": str(exc),
            }
        )
        log(f"失败即停：{exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
