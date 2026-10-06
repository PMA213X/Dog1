#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""R3 四机器人共享 world 顺序训练、提前 Gate 与预算失败即停。"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Optional, Sequence

from . import contract
from .env import active_tcp_ports, listening_tcp_ports, require_ports_free


STATUS_PATH = contract.LOG_ROOT / "stage_status.json"
STAGES_LOG = contract.LOG_ROOT / "stages.log"
FINAL_CHECKPOINT = contract.CHECKPOINT_ROOT / f"{contract.CONTRACT_VERSION}_final.zip"
GATE_PORT_TIMEOUT_SECONDS = 60.0
GATE_PORT_POLL_SECONDS = 0.25
LOG_DIRS = {
    phase: f"{phase}_parallel"
    for phase in contract.PHASE_ORDER
}
BAD_LOG_PATTERN = re.compile(
    r"(?:\bnan\b|\binf\b|out of memory|\bcuda error\b|traceback)",
    re.IGNORECASE,
)


def log(message: str) -> None:
    """追加阶段执行日志。"""
    line = f"[{time.strftime('%Y-%m-%dT%H:%M:%S%z')}] {message}"
    print(line, flush=True)
    contract.LOG_ROOT.mkdir(parents=True, exist_ok=True)
    with STAGES_LOG.open("a", encoding="utf-8") as handle:
        handle.write(line + "\n")


def write_status(payload: dict[str, Any]) -> None:
    """原子写入阶段状态。"""
    contract.LOG_ROOT.mkdir(parents=True, exist_ok=True)
    value = {
        "schema_version": 3,
        "contract": contract.CONTRACT_VERSION,
        "updated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        **payload,
    }
    temporary = STATUS_PATH.with_suffix(".tmp")
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(STATUS_PATH)


def stage_command(
    phase: str,
    total_steps: int,
    *,
    resume: Path | None = None,
    log_key: str,
    randomization_mode: str = "full",
    phase_step_offset: int = 0,
) -> list[str]:
    """构造固定四 worker 的正式训练命令。"""
    command = [
        sys.executable,
        "-m",
        "rl.train",
        "--phase",
        phase,
        "--total-steps",
        str(total_steps),
        "--device",
        "cuda",
        "--ckptdir",
        str(contract.CHECKPOINT_ROOT),
        "--ckpt-prefix",
        contract.CONTRACT_VERSION,
        "--logdir",
        str(contract.RUN_ROOT / LOG_DIRS[log_key]),
        "--bridge-port",
        str(contract.BRIDGE_PORT),
        "--num-envs",
        str(contract.PARALLEL_WORKERS),
        "--checkpoint-interval",
        str(contract.CHECKPOINT_INTERVAL_STEPS),
        "--randomization-mode",
        randomization_mode or contract.PHASE_RANDOMIZATION_MODES[phase],
        "--seed",
        str(contract.phase_training_seed(phase, phase_step_offset)),
        "--phase-step-offset",
        str(int(phase_step_offset)),
    ]
    if resume is not None:
        command.extend(
            ["--resume", str(contract.validate_checkpoint_path(resume))]
        )
    return command


def checkpoint_steps(path: Path) -> int:
    """加载 checkpoint 并校验 R2 维度。"""
    from stable_baselines3 import PPO

    resolved = contract.validate_checkpoint_path(path)
    if not resolved.is_file():
        raise RuntimeError(f"checkpoint 不存在：{resolved}")
    model = PPO.load(str(resolved), device="cuda")
    if tuple(model.observation_space.shape) != (contract.OBS_DIM,):
        raise RuntimeError("checkpoint 观测维度错误")
    if tuple(model.action_space.shape) != (contract.ACTION_DIM,):
        raise RuntimeError("checkpoint 动作维度错误")
    return int(model.num_timesteps)


def _http_status(url: str) -> str:
    completed = subprocess.run(
        ["curl", "-sS", "-o", "/dev/null", "-w", "%{http_code}", url],
        text=True,
        capture_output=True,
        check=False,
    )
    return completed.stdout.strip()


def _tensorboard_uses_r3_run() -> bool:
    """确认 6007 端口进程确实挂载 R3 命名空间，拒绝历史日志冒充。"""
    completed = subprocess.run(
        ["ps", "-eo", "args="],
        text=True,
        capture_output=True,
        check=False,
    )
    return any(
        "tensorboard" in line
        and contract.CONTRACT_VERSION in line
        and "6007" in line
        for line in completed.stdout.splitlines()
    )


def _process_group_pids(group_id: int) -> set[int]:
    """读取本次训练进程组内的全部 PID。"""
    completed = subprocess.run(
        ["ps", "-eo", "pid=,pgid=,args="],
        text=True,
        capture_output=True,
        check=False,
    )
    pids: set[int] = set()
    for line in completed.stdout.splitlines():
        parts = line.strip().split(maxsplit=2)
        if len(parts) < 2:
            continue
        try:
            pid, pgid = int(parts[0]), int(parts[1])
        except ValueError:
            continue
        if pgid == group_id:
            pids.add(pid)
    return pids


def _port_listeners(group_id: int | None = None) -> list[int]:
    completed = subprocess.run(
        ["ss", "-ltnpH"],
        text=True,
        capture_output=True,
        check=False,
    )
    group_pids = _process_group_pids(group_id) if group_id is not None else set()
    ports: list[int] = []
    for line in completed.stdout.splitlines():
        match = re.search(r":(1145[2-5])\b", line)
        if match is None:
            continue
        if group_id is None:
            ports.append(int(match.group(1)))
            continue
        pid_match = re.search(r"\bpid=(\d+)\b", line)
        if pid_match and int(pid_match.group(1)) in group_pids:
            ports.append(int(match.group(1)))
    return sorted(ports)


def _webots_process_count(group_id: int | None = None) -> int:
    """统计唯一 Webots 主进程数量。"""
    completed = subprocess.run(
        ["ps", "-eo", "pid=,pgid=,args="],
        text=True,
        capture_output=True,
        check=False,
    )
    return sum(
        1
        for line in completed.stdout.splitlines()
        if _line_in_process_group(line, group_id)
        and "/usr/local/webots/webots-controller" not in line
        and "/usr/local/webots/webots" in line
        and "flat_move_jump_rl.wbt" in line
        and "grep" not in line
    )


def _webots_controller_process_count(
    group_id: int | None = None,
    *,
    ports: Sequence[int] | None = None,
    worker_logs: dict[str, Any] | None = None,
) -> int:
    """按当前进程组、端口和握手日志统计四路实际 controller。"""
    completed = subprocess.run(
        ["ps", "-eo", "pid=,pgid=,args="],
        text=True,
        capture_output=True,
        check=False,
    )
    candidates = {
        int(line.strip().split(maxsplit=2)[0])
        for line in completed.stdout.splitlines()
        if _line_in_process_group(line, group_id)
        and _is_rl_controller_process(line)
    }
    if ports is None and worker_logs is None:
        return len(candidates)
    ports_ok = ports is None or tuple(sorted(set(ports))) == contract.bridge_ports()
    logs_ok = worker_logs is None or _worker_controller_logs_ok(worker_logs)
    return len(candidates) if ports_ok and logs_ok else 0


def _is_rl_controller_process(line: str) -> bool:
    """兼容 webots-controller 包装器和实际 python3 rl_agent.py 进程。"""
    if "rl_agent.py" not in line or "grep" in line:
        return False
    return "python" in line or "/usr/local/webots/webots-controller" in line


def _worker_controller_logs_ok(worker_logs: dict[str, Any]) -> bool:
    """校验四路日志均包含当前端口、worker、Robot 名和 timestep。"""
    if len(worker_logs) != contract.PARALLEL_WORKERS:
        return False
    return all(
        all(
            bool(worker_logs.get(str(worker_id), {}).get(field))
            for field in (
                "current",
                "connected",
                "worker_id",
                "robot_name",
                "timestep",
            )
        )
        for worker_id in range(contract.PARALLEL_WORKERS)
    )


def _line_in_process_group(
    line: str,
    group_id: int | None,
) -> bool:
    """判断 ps 行是否属于本次训练进程组。"""
    if group_id is None:
        return True
    parts = line.strip().split(maxsplit=2)
    if len(parts) < 2:
        return False
    try:
        return int(parts[1]) == group_id
    except ValueError:
        return False


def _gui_window_count() -> int:
    completed = subprocess.run(
        ["xwininfo", "-root", "-children"],
        text=True,
        capture_output=True,
        check=False,
    )
    return len(
        {
            line.strip()
            for line in completed.stdout.splitlines()
            if "official-mini-cheetah/worlds/flat_move_jump_rl.wbt" in line
            and "Webots" in line
        }
    )


def _gpu_train_process(train_pid: int) -> bool:
    completed = subprocess.run(
        [
            "nvidia-smi",
            "--query-compute-apps=pid,process_name,used_memory",
            "--format=csv,noheader",
        ],
        text=True,
        capture_output=True,
        check=False,
    )
    return any(
        line.strip().startswith(f"{train_pid},")
        for line in completed.stdout.splitlines()
    )


def _event_nonempty(segment: str, since: float | None = None) -> bool:
    event_dir = contract.RUN_ROOT / LOG_DIRS[segment]
    return any(
        path.is_file()
        and path.stat().st_size > 0
        and (since is None or path.stat().st_mtime >= since - 1.0)
        for path in event_dir.rglob("events.out.tfevents.*")
    )


def _run_started_at(segment: str) -> float | None:
    """读取当前训练段的启动时间标记。"""
    path = contract.LOG_ROOT / f"{segment}_run_started_at.txt"
    if not path.is_file():
        return None
    try:
        return float(path.read_text(encoding="utf-8").strip())
    except (OSError, ValueError):
        return None


def _worker_logs_ok(since: float | None = None) -> dict[str, Any]:
    evidence: dict[str, Any] = {}
    for worker_id, port in enumerate(contract.bridge_ports()):
        path = contract.LOG_ROOT / f"webots_{worker_id}.log"
        text = path.read_text(encoding="utf-8", errors="replace") if path.is_file() else ""
        evidence[str(worker_id)] = {
            "path": str(path),
            "exists": path.is_file(),
            "current": path.is_file()
            and (
                since is None
                or path.stat().st_mtime >= since - 1.0
            ),
            "connected": f"port={port}" in text
            and contract.CONTRACT_VERSION in text,
            "worker_id": f"worker_id={worker_id}" in text,
            "robot_name": f"robot_name={contract.ROBOT_NAMES[worker_id]}" in text,
            "timestep": f"timestep={contract.WEBOTS_TIMESTEP_MS}" in text,
        }
    return evidence


def _no_bad_logs(segment: str, since: float | None = None) -> bool:
    names = (
        f"{segment}_train.log",
        "webots_0.log",
        "webots_1.log",
        "webots_2.log",
        "webots_3.log",
        "webots_shared_world.log",
    )
    for name in names:
        path = contract.LOG_ROOT / name
        if not path.is_file():
            continue
        if since is not None and path.stat().st_mtime < since - 1.0:
            continue
        if BAD_LOG_PATTERN.search(
            path.read_text(encoding="utf-8", errors="replace")
        ):
            return False
    return True


def confirm_live_workers(
    segment: str,
    train_pid: int,
    *,
    run_started_at: float,
) -> dict[str, Any]:
    """训练进行中确认 1 Webots、4 controller、4 TCP/handshake 和 GPU。"""
    deadline = time.monotonic() + 180.0
    last: dict[str, Any] = {}
    while time.monotonic() < deadline:
        if not Path(f"/proc/{train_pid}").exists():
            raise RuntimeError(f"{segment} 训练进程在健康确认前退出")
        worker_logs = _worker_logs_ok(since=run_started_at)
        ports = _port_listeners(group_id=train_pid)
        controller_processes = _webots_controller_process_count(
            group_id=train_pid,
            ports=ports,
            worker_logs=worker_logs,
        )
        last = {
            "segment": segment,
            "checked_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "train_pid": train_pid,
            "process_group": train_pid,
            "ports": ports,
            "webots_processes": _webots_process_count(group_id=train_pid),
            "controller_processes": controller_processes,
            "gui_windows": _gui_window_count(),
            "gpu_train_process": _gpu_train_process(train_pid),
            "worker_logs": worker_logs,
            "event_nonempty": _event_nonempty(segment, since=run_started_at),
            "run_started_at": run_started_at,
        }
        if (
            last["ports"] == list(contract.bridge_ports())
            and last["webots_processes"] == 1
            and last["controller_processes"] == contract.PARALLEL_WORKERS
            and last["gpu_train_process"]
            and all(
                item["connected"]
                and item["current"]
                and item["worker_id"]
                and item["robot_name"]
                and item["timestep"]
                for item in worker_logs.values()
            )
            and last["event_nonempty"]
        ):
            last["passed"] = True
            path = contract.LOG_ROOT / f"health_{segment}_live.json"
            path.write_text(
                json.dumps(last, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            log(f"{segment} 四 worker 实时健康确认通过")
            return last
        time.sleep(2)
    last["passed"] = False
    raise RuntimeError(f"{segment} 四 worker 实时健康确认超时：{last}")


def confirm_post_gate(segment: str, target: int) -> dict[str, Any]:
    """gate 后确认 checkpoint、event、worker 日志、GPU 和 TensorBoard。"""
    steps = checkpoint_steps(FINAL_CHECKPOINT)
    started_at = _run_started_at(segment)
    worker_logs = _worker_logs_ok(since=started_at)
    gpu = subprocess.run(
        ["nvidia-smi", "--query-gpu=index,name", "--format=csv,noheader"],
        text=True,
        capture_output=True,
        check=False,
    ).stdout.strip()
    evidence = {
        "segment": segment,
        "checked_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "checkpoint": str(FINAL_CHECKPOINT),
        "steps": steps,
        "target": target,
        "event_nonempty": _event_nonempty(segment, since=started_at),
        "worker_logs": worker_logs,
        "gpu": gpu,
        "tensorboard_http": _http_status("http://127.0.0.1:6007/"),
        "tensorboard_r3_namespace": _tensorboard_uses_r3_run(),
        "no_bad_logs": _no_bad_logs(segment, since=started_at),
    }
    evidence["passed"] = (
        steps == target
        and evidence["event_nonempty"]
        and all(
            item["connected"]
            and item["current"]
            and item["worker_id"]
            and item["robot_name"]
            and item["timestep"]
            for item in worker_logs.values()
        )
        and "RTX 4060" in gpu
        and evidence["tensorboard_http"] == "200"
        and evidence["tensorboard_r3_namespace"]
        and evidence["no_bad_logs"]
    )
    path = contract.LOG_ROOT / f"health_{segment}_after_gate.json"
    path.write_text(
        json.dumps(evidence, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    if not evidence["passed"]:
        raise RuntimeError(f"{segment} gate 后健康确认失败：{evidence}")
    log(f"{segment} gate 后健康确认通过")
    return evidence


def run_training(command: list[str], phase: str, segment: str) -> None:
    """启动四环境训练并在 worker 活跃时执行健康确认。"""
    train_log = contract.LOG_ROOT / f"{segment}_train.log"
    write_status(
        {
            "status": "training",
            "current_phase": phase,
            "segment": segment,
            "train_log": str(train_log),
            "num_envs": contract.PARALLEL_WORKERS,
            "ports": list(contract.bridge_ports()),
        }
    )
    for worker_id in range(contract.PARALLEL_WORKERS):
        worker_log = contract.LOG_ROOT / f"webots_{worker_id}.log"
        worker_log.write_text("", encoding="utf-8")
    train_log.write_text("", encoding="utf-8")
    shared_world_log = contract.LOG_ROOT / "webots_shared_world.log"
    shared_world_log.write_text("", encoding="utf-8")
    log(f"{segment} 启动：{' '.join(command)}")
    environment = os.environ.copy()
    environment["CUDA_VISIBLE_DEVICES"] = "0"
    environment["DISPLAY"] = environment.get("DISPLAY", ":0")
    environment["PYTHONUNBUFFERED"] = "1"
    contract.LOG_ROOT.mkdir(parents=True, exist_ok=True)
    process: Optional[subprocess.Popen[bytes]] = None
    run_started_at = time.time()
    marker = contract.LOG_ROOT / f"{segment}_run_started_at.txt"
    marker.write_text(str(run_started_at) + "\n", encoding="utf-8")
    try:
        with train_log.open("a", encoding="utf-8") as handle:
            process = subprocess.Popen(
                command,
                cwd=str(contract.PROJECT_ROOT),
                env=environment,
                stdout=handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            confirm_live_workers(
                segment,
                process.pid,
                run_started_at=run_started_at,
            )
            return_code = process.wait()
            _terminate_process_group(process)
        if return_code != 0:
            raise RuntimeError(f"{segment} 训练退出码={return_code}")
    except BaseException:
        if process is not None:
            _terminate_process_group(process)
        raise


def _terminate_process_group(process: subprocess.Popen[bytes]) -> None:
    """终止训练进程组，确保 Webots 和四路 controller 一并回收。"""
    group_id = process.pid
    try:
        os.killpg(group_id, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        pass
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        if not _process_group_pids(group_id):
            return
        time.sleep(0.1)
    try:
        os.killpg(group_id, signal.SIGKILL)
    except ProcessLookupError:
        return
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if not _process_group_pids(group_id):
            return
        time.sleep(0.1)


def wait_for_gate_ports_free(
    ports: Sequence[int],
    context: str,
) -> int:
    """对真实 LISTEN 占用做有界等待，不杀进程且忽略 TIME-WAIT 等残留。"""
    normalized = tuple(dict.fromkeys(int(port) for port in ports))
    started = time.monotonic()
    deadline = started + GATE_PORT_TIMEOUT_SECONDS
    attempts = 0
    last_error = ""
    while True:
        attempts += 1
        try:
            require_ports_free(normalized, context)
        except RuntimeError as exc:
            last_error = str(exc)
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                try:
                    listeners = listening_tcp_ports(normalized)
                except RuntimeError:
                    listeners = []
                try:
                    active = active_tcp_ports(normalized)
                except RuntimeError:
                    active = []
                raise RuntimeError(
                    f"{context}端口等待超时："
                    f"timeout={GATE_PORT_TIMEOUT_SECONDS:.3f}s "
                    f"attempts={attempts} last_error={last_error} "
                    f"listening_ports={list(listeners)} "
                    f"active_residue_ports={list(active)}。"
                    "仅真实 LISTEN 阻断启动，TIME-WAIT/CLOSE-WAIT 等非监听"
                    "残留不阻断；请等待旧 Webots/controller 自然退出后再重试，"
                    "本流程不执行破坏性 kill。"
                ) from exc
            if attempts == 1:
                log(
                    f"{context}检测到 LISTEN 端口占用，"
                    f"开始最多 {GATE_PORT_TIMEOUT_SECONDS:.0f}s 的有界等待"
                )
            time.sleep(
                min(
                    GATE_PORT_POLL_SECONDS,
                    max(0.0, remaining),
                )
            )
            continue
        if attempts > 1:
            elapsed = time.monotonic() - started
            log(
                f"{context}端口已释放并完成安全重试："
                f"attempts={attempts} elapsed={elapsed:.3f}s"
            )
        return attempts


def run_gate(phase: str, attempt: int = 0) -> bool:
    """执行 gate；仅有效 report.passed=false 属于可重试 gate 失败。"""
    wait_for_gate_ports_free(
        (
            contract.WEBOTS_SUPERVISOR_PORT,
            *contract.bridge_ports(),
        ),
        "启动 Gate",
    )
    output_name = f"{phase}_gate_attempt_{attempt}.json"
    output = contract.LOG_ROOT / output_name
    command = [
        sys.executable,
        "-m",
        "rl.evaluate",
        "--phase",
        phase,
        "--gate",
        "--checkpoint",
        str(FINAL_CHECKPOINT),
        "--output",
        str(output),
    ]
    environment = os.environ.copy()
    environment["CUDA_VISIBLE_DEVICES"] = "0"
    environment["PYTHONUNBUFFERED"] = "1"
    log(f"{phase} 验收命令：{' '.join(command)}")
    completed = subprocess.run(
        command,
        cwd=str(contract.PROJECT_ROOT),
        env=environment,
        check=False,
    )
    if completed.returncode == 0:
        log(f"{phase} 验收通过")
        return True
    if not output.is_file():
        raise RuntimeError(
            f"{phase} gate 技术故障，退出码={completed.returncode}，无有效报告"
        )
    try:
        payload = json.loads(output.read_text(encoding="utf-8"))
        passed = bool(payload["report"]["passed"])
    except (json.JSONDecodeError, KeyError, TypeError) as exc:
        raise RuntimeError(f"{phase} gate 技术故障，报告无效：{exc}") from exc
    if bool(payload["report"].get("technical_failure", False)):
        raise RuntimeError(
            f"{phase} gate 技术故障："
            f"{payload['report'].get('technical_failure_reasons') or payload['report'].get('checks')}"
        )
    if passed:
        raise RuntimeError(
            f"{phase} gate 返回非零但 report.passed=true，属于技术故障"
        )
    log(f"{phase} 验收未通过，report.passed=false")
    return False


def run_gate_batch(phase: str) -> bool:
    """连续执行三批 Gate；只有三批全部通过才允许推进。"""
    results = [run_gate(phase, attempt=index) for index in range(3)]
    if not all(results):
        log(f"{phase} 连续 Gate 批次结果={results}")
    return all(results)


def dry_run_commands() -> list[list[str]]:
    """输出 P0～P7 的八条累计训练命令。"""
    commands: list[list[str]] = []
    # P0 是独立起点；污染 checkpoint 只作取证，不进入任何 P0 命令。
    resume: Path | None = None
    for phase in contract.PHASE_ORDER:
        commands.append(
            stage_command(
                phase,
                contract.PHASE_TOTAL_STEPS[phase],
                resume=resume,
                log_key=phase,
                randomization_mode=contract.PHASE_RANDOMIZATION_MODES[phase],
            )
        )
        resume = FINAL_CHECKPOINT
    return commands


def _restart_p0_from_zero_required() -> bool:
    """失败的 P0 必须从零重跑，后续阶段仍按 checkpoint 顺序执行。"""
    if not FINAL_CHECKPOINT.is_file():
        return False
    status_path = contract.LOG_ROOT / "stage_status.json"
    if not status_path.is_file():
        return False
    try:
        status = json.loads(status_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError, TypeError):
        return False
    return str(status.get("status", "")) in {"failed", "training", "running"}


def execute_phase(phase: str, resume: Path | None) -> Path:
    """按固定预算分段训练；Gate 提前通过即返回，预算耗尽仍失败则停止。"""
    budget = contract.PHASE_TOTAL_STEPS[phase]
    current = 0 if resume is None else checkpoint_steps(resume)
    if current > budget:
        raise RuntimeError(f"{phase} checkpoint {current} 超过预算 {budget}")
    if current >= budget:
        if not FINAL_CHECKPOINT.is_file():
            raise RuntimeError(f"{phase} 缺少最终 checkpoint")
        if not run_gate_batch(phase):
            raise RuntimeError(f"{phase} Gate 在预算 {budget} 处失败，按计划停止")
        confirm_post_gate(phase, budget)
        return FINAL_CHECKPOINT

    segment_index = 0
    phase_index = contract.PHASE_ORDER.index(phase)
    previous_phase_budget = (
        contract.PHASE_TOTAL_STEPS[contract.PHASE_ORDER[phase_index - 1]]
        if phase_index > 0
        else 0
    )
    segment_interval = contract.phase_gate_check_interval_steps(phase)
    while current < budget:
        next_target = min(
            budget,
            current + segment_interval,
        )
        remaining_segments = math.ceil(
            (budget - current) / segment_interval
        )
        randomization_mode = contract.PHASE_RANDOMIZATION_MODES[phase]
        if phase == "P4":
            level_index = min(
                len(contract.RANDOMIZATION_LEVELS) - 1,
                segment_index * len(contract.RANDOMIZATION_LEVELS)
                // remaining_segments,
            )
            randomization_mode = contract.RANDOMIZATION_LEVELS[level_index]
        run_training(
            stage_command(
                phase,
                next_target,
                resume=resume,
                log_key=phase,
                randomization_mode=randomization_mode,
                phase_step_offset=max(0, current - previous_phase_budget),
            ),
            phase,
            phase,
        )
        current = checkpoint_steps(FINAL_CHECKPOINT)
        resume = FINAL_CHECKPOINT
        if current != next_target:
            raise RuntimeError(f"{phase} 训练步数 {current} != {next_target}")
        gate_due = (
            current >= budget
            or (
                phase == "P1"
                and next_target >= contract.P1_PERTURBATION_GATE_STEP
            )
            or phase in {"P2", "P3", "P4", "P5", "P6", "P7"}
        )
        gate_passed = run_gate_batch(phase) if gate_due else False
        if not gate_due:
            write_status(
                {
                    "status": "training",
                    "current_phase": phase,
                    "phase_target": budget,
                    "current_steps": current,
                    "checkpoint": str(FINAL_CHECKPOINT),
                }
            )
            segment_index += 1
            continue
        if gate_passed:
            confirm_post_gate(phase, current)
            if current < budget and phase != "P4":
                write_status(
                    {
                        "status": "gate_passed",
                        "current_phase": phase,
                        "phase_target": budget,
                        "passed_steps": current,
                        "checkpoint": str(FINAL_CHECKPOINT),
                    }
                )
                return FINAL_CHECKPOINT
            if current < budget and phase == "P4":
                write_status(
                    {
                        "status": "training",
                        "current_phase": phase,
                        "phase_target": budget,
                        "current_steps": current,
                        "randomization_mode": randomization_mode,
                        "checkpoint": str(FINAL_CHECKPOINT),
                    }
                )
                segment_index += 1
                continue
            write_status(
                {
                    "status": "gate_passed",
                    "current_phase": phase,
                    "phase_target": budget,
                    "passed_steps": current,
                    "checkpoint": str(FINAL_CHECKPOINT),
                }
            )
            return FINAL_CHECKPOINT
        if phase == "P4":
            raise RuntimeError(
                f"P4 DR {randomization_mode} Gate 失败于 {current}，按计划停止"
            )
        if current >= budget:
            raise RuntimeError(
                f"{phase} Gate 在预算 {budget} 处失败，按计划停止且不自动延长"
            )
        write_status(
            {
                "status": "training",
                "current_phase": phase,
                "phase_target": budget,
                "current_steps": current,
                "checkpoint": str(FINAL_CHECKPOINT),
            }
        )
        segment_index += 1
    raise RuntimeError(f"{phase} 未产生有效 Gate 结果")


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """解析 R3 执行器参数。"""
    parser = argparse.ArgumentParser(description=contract.CONTRACT_VERSION)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """执行 P0～P7；任何 Gate 或技术故障立即停止。"""
    args = parse_args(argv)
    if args.dry_run:
        for command in dry_run_commands():
            print(" ".join(command))
        return 0
    if os.environ.get("CUDA_VISIBLE_DEVICES") != "0":
        print("CUDA_VISIBLE_DEVICES 必须为 0", file=sys.stderr)
        return 2
    contract.LOG_ROOT.mkdir(parents=True, exist_ok=True)
    contract.CHECKPOINT_ROOT.mkdir(parents=True, exist_ok=True)
    contract.RUN_ROOT.mkdir(parents=True, exist_ok=True)
    resume = FINAL_CHECKPOINT if FINAL_CHECKPOINT.is_file() else None
    restart_p0 = _restart_p0_from_zero_required()
    current = 0 if resume is None else checkpoint_steps(resume)
    write_status(
        {
            "status": "running",
            "current_phase": "P0",
            "resume_steps": 0 if restart_p0 else current,
            "restart_p0_from_zero": restart_p0,
            "initial_checkpoint": None,
            "num_envs": contract.PARALLEL_WORKERS,
            "ports": list(contract.bridge_ports()),
        }
    )
    log(
        "R3 新训练断点 "
        f"steps={0 if restart_p0 else current} "
        f"restart_p0_from_zero={restart_p0}"
    )
    try:
        completed: list[str] = []
        phase_resume = None if restart_p0 else resume
        for phase in contract.PHASE_ORDER:
            phase_resume = execute_phase(phase, phase_resume)
            completed.append(phase)
            write_status(
                {
                    "status": "phase_passed",
                    "current_phase": phase,
                    "completed_phases": completed,
                    "checkpoint": str(FINAL_CHECKPOINT),
                }
            )
        write_status(
            {
                "status": "completed",
                "current_phase": "P7",
                "completed_phases": completed,
                "total_steps": contract.PHASE_TOTAL_STEPS["P7"],
                "buffer_total_steps": contract.BUFFER_TOTAL_STEPS,
                "checkpoint": str(FINAL_CHECKPOINT),
            }
        )
        log("R3 P0～P7 全部完成")
        return 0
    except Exception as exc:
        write_status({"status": "failed", "error": str(exc)})
        log(f"失败即停：{exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
