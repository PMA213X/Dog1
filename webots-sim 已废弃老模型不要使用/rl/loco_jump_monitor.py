#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""yobogo_loco_jump_v1 本地监控与人工巡检入口。

本文件只负责观察、归档、校验、通知和受控恢复，不修改训练代码。
默认只读巡检；只有显式使用 ``--run`` 才会按命令清单启动训练进程。
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
import zipfile
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

# 固定运行标识，避免与其他训练任务混写日志。
DEFAULT_RUN_ID = "yobogo_loco_jump_v1"
SCHEMA_VERSION = 1

# 训练日志中的累计步数格式：兼容 SB3 竖线表和中文训练日志。
PROGRESS_PATTERNS: Tuple[re.Pattern[str], ...] = (
    re.compile(r"total_timesteps\s*\|\s*([\d,]+)", re.IGNORECASE),
    re.compile(r"total_timesteps\s*[=:]\s*([\d,]+)", re.IGNORECASE),
    re.compile(r"累计总步数\s*[=:]\s*([\d,]+)"),
    re.compile(r"步数\s*[=:]\s*([\d,]+)"),
)

# 日志级致命错误：命中后先终止当前进程，再进入“归档→校验→恢复”。
FATAL_PATTERNS: Tuple[Tuple[str, re.Pattern[str]], ...] = (
    ("nan", re.compile(r"(?<![A-Za-z])nan(?![A-Za-z])", re.IGNORECASE)),
    ("inf", re.compile(r"(?<![A-Za-z])[+-]?inf(?:inity)?(?![A-Za-z])", re.IGNORECASE)),
    (
        "oom",
        re.compile(
            r"out of memory|oom[- ]?killed|cuda.*out of memory|内存不足|内存溢出",
            re.IGNORECASE,
        ),
    ),
    (
        "tcp",
        re.compile(
            r"(?:tcp.*(?:error|failed|refused|reset|timeout|断开|错误|失败))"
            r"|(?:error|failed|refused|reset|timeout|错误|失败).*tcp",
            re.IGNORECASE,
        ),
    ),
)

# 正常完成标记；不同训练入口可以通过命令清单覆盖。
DEFAULT_COMPLETION_MARKERS: Tuple[str, ...] = (
    "最终模型已保存",
    "训练完成",
    "TRAINING_COMPLETED",
    "RECOVERY_COMPLETED",
)

# 默认阶段顺序：累计目标步数与正式训练阶段一一对应。
DEFAULT_STAGES: Tuple[Dict[str, Any], ...] = (
    {
        "id": "S0_smoke",
        "full_name": "S0 冒烟",
        "cumulative_target_steps": 5000,
        "tag": "phase0",
        "log_tag": "S0_smoke",
    },
    {
        "id": "S1_stand",
        "full_name": "S1 站立",
        "cumulative_target_steps": 200000,
        "tag": "phase1",
        "log_tag": "S1_stand",
    },
    {
        "id": "S2_command",
        "full_name": "S2 命令",
        "cumulative_target_steps": 500000,
        "tag": "phase2",
        "log_tag": "S2_command",
    },
    {
        "id": "S3_jump",
        "full_name": "S3 跳跃",
        "cumulative_target_steps": 800000,
        "tag": "phase3",
        "log_tag": "S3_jump",
    },
    {
        "id": "S4_mobile_terrain",
        "full_name": "S4 移动越障",
        "cumulative_target_steps": 1200000,
        "tag": "phase4",
        "log_tag": "S4_mobile_terrain",
    },
)


def now_iso() -> str:
    """返回本地时间 ISO 字符串，便于人工巡检。"""
    return datetime.now().astimezone().isoformat(timespec="seconds")


def compact_time() -> str:
    """返回无冒号的时间片段，用于归档目录名。"""
    return datetime.now().strftime("%Y%m%d-%H%M%S")


def atomic_write_json(path: Path, data: Any) -> None:
    """原子写 JSON，避免监控中断留下半个状态文件。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    tmp.write_text(
        json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(tmp, path)


def append_jsonl(path: Path, data: Dict[str, Any]) -> None:
    """追加一行 JSON；日志写失败不能影响监控主循环。"""
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(data, ensure_ascii=False, sort_keys=True) + "\n")
    except OSError:
        pass


def read_json(path: Path, default: Any) -> Any:
    """读取 JSON，缺失或损坏时返回调用方给定的默认值。"""
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError):
        return default


def tail_text(path: Path, max_bytes: int = 64 * 1024) -> str:
    """读取日志尾部，用于完成标记与归档元数据。"""
    try:
        with path.open("rb") as handle:
            handle.seek(0, os.SEEK_END)
            size = handle.tell()
            handle.seek(max(0, size - max_bytes))
            return handle.read().decode("utf-8", errors="replace")
    except OSError:
        return ""


def sanitize(value: str) -> str:
    """把事件原因转换成可安全落盘的目录片段。"""
    cleaned = re.sub(r"[^0-9A-Za-z_.-]+", "_", value).strip("_")
    return cleaned[:80] or "event"


def pid_alive(pid: Optional[int]) -> bool:
    """只用信号 0 判断 PID 是否存在，不执行 shell。"""
    if not pid or pid <= 0:
        return False
    try:
        os.kill(int(pid), 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    except OSError:
        return False
    return True


def read_cmdline(pid: int) -> List[str]:
    """读取 Linux 进程命令行，用于防止 PID 复用后误操作。"""
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    except OSError:
        return []
    return [part.decode("utf-8", errors="replace") for part in raw.split(b"\0") if part]


def process_matches(pid: int, expected: Sequence[str]) -> bool:
    """校验 PID 仍对应预期命令；命令行不可读时保守认为不匹配。"""
    if not expected:
        return True
    actual = read_cmdline(pid)
    if not actual:
        return False
    expected_list = list(expected)
    actual_list = list(actual)
    if actual_list == expected_list:
        return True
    # Python 启动器可能由 PATH 展开为绝对路径，比较脚本及后续参数。
    if len(actual_list) == len(expected_list):
        return Path(actual_list[0]).name == Path(expected_list[0]).name and actual_list[1:] == expected_list[1:]
    return any(str(expected_list[-1]) == item for item in actual_list)


def terminate_pid_group(pid: Optional[int], timeout: float = 5.0) -> None:
    """先礼貌终止训练进程组，超时后再强制结束。"""
    if not pid_alive(pid):
        return
    assert pid is not None
    try:
        os.killpg(pid, signal.SIGTERM)
    except OSError:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            return
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and pid_alive(pid):
        time.sleep(0.1)
    if pid_alive(pid):
        try:
            os.killpg(pid, signal.SIGKILL)
        except OSError:
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass


def parse_int_progress(text: str) -> Optional[int]:
    """从训练日志中提取最大累计步数。"""
    values: List[int] = []
    for pattern in PROGRESS_PATTERNS:
        for match in pattern.finditer(text):
            raw = match.group(1).replace(",", "")
            if raw.isdigit():
                values.append(int(raw))
    return max(values) if values else None


def checkpoint_step(path: Path) -> Optional[int]:
    """从 checkpoint 文件名提取步数，例如 ..._10000.zip。"""
    match = re.search(r"(\d+)\.zip$", path.name, re.IGNORECASE)
    return int(match.group(1)) if match else None


def validate_checkpoint(path: Path) -> Tuple[bool, str, List[str]]:
    """校验 ZIP 结构与 CRC，返回（是否有效、原因、成员清单）。"""
    try:
        if not path.is_file() or path.stat().st_size <= 0:
            return False, "文件为空或不存在", []
        if not zipfile.is_zipfile(path):
            return False, "不是有效 ZIP", []
        with zipfile.ZipFile(path, "r") as archive:
            names = archive.namelist()
            if not names:
                return False, "ZIP 中没有成员", []
            bad = archive.testzip()
            if bad is not None:
                return False, f"CRC 校验失败：{bad}", names
        return True, "通过", names
    except (OSError, zipfile.BadZipFile) as exc:
        return False, str(exc), []


def parse_value(argv: Sequence[str], flag: str, default: Any = None) -> Any:
    """读取命令数组中的单值参数，不使用 shell。"""
    try:
        index = list(argv).index(flag)
    except ValueError:
        return default
    if index + 1 < len(argv):
        return argv[index + 1]
    return default


def memory_snapshot() -> Dict[str, Any]:
    """读取 /proc/meminfo，避免强依赖 psutil。"""
    info: Dict[str, int] = {}
    try:
        for line in Path("/proc/meminfo").read_text(encoding="ascii", errors="ignore").splitlines():
            key, _, rest = line.partition(":")
            fields = rest.strip().split()
            if fields and fields[0].isdigit():
                info[key] = int(fields[0]) * 1024
    except OSError:
        return {"used_percent": None, "total_gib": None, "available_gib": None}
    total = info.get("MemTotal", 0)
    available = info.get("MemAvailable", info.get("MemFree", 0))
    used = max(0, total - available)
    return {
        "used_percent": round(used * 100.0 / total, 2) if total else None,
        "total_gib": round(total / (1024 ** 3), 2),
        "available_gib": round(available / (1024 ** 3), 2),
    }


def disk_snapshot(path: Path) -> Dict[str, Any]:
    """返回指定路径所在磁盘的可用空间。"""
    try:
        usage = shutil.disk_usage(path)
        return {
            "free_gib": round(usage.free / (1024 ** 3), 2),
            "total_gib": round(usage.total / (1024 ** 3), 2),
            "used_percent": round(usage.used * 100.0 / usage.total, 2) if usage.total else None,
        }
    except OSError as exc:
        return {"free_gib": None, "total_gib": None, "used_percent": None, "error": str(exc)}


class MonitorPaths:
    """集中定义 PID、日志、状态、清单和归档路径。"""

    def __init__(self, project_root: Path, run_root: Path) -> None:
        self.project_root = project_root
        self.run_root = run_root
        self.logs = run_root
        self.pids = run_root / "pids"
        self.archive = run_root / "archive"
        self.train_log = run_root / "train.log"
        self.monitor_log = run_root / "monitor.log"
        self.events_log = run_root / "events.log"
        self.phase_log = run_root / "phase.log"
        self.pid_log = run_root / "pid.log"
        self.state_file = run_root / "state.json"
        self.status_file = run_root / "status.json"
        self.manifest_file = run_root / "manifest.json"
        self.checkpoint_manifest = run_root / "checkpoint_manifest.json"
        self.command_file = run_root / "train_command.json"
        self.monitor_pid = self.pids / "monitor.pid"
        self.watchdog_pid = self.pids / "watchdog.pid"
        self.training_pid = self.pids / "training.pid"
        self.tensorboard_pid = self.pids / "tensorboard.pid"

    def ensure(self) -> None:
        """创建运行目录和 PID 子目录。"""
        self.pids.mkdir(parents=True, exist_ok=True)
        self.archive.mkdir(parents=True, exist_ok=True)

    def convention(self) -> Dict[str, str]:
        """返回供状态页展示的路径约定。"""
        return {
            "run_root": str(self.run_root),
            "train_log": str(self.train_log),
            "monitor_log": str(self.monitor_log),
            "events_log": str(self.events_log),
            "phase_log": str(self.phase_log),
            "pid_log": str(self.pid_log),
            "state": str(self.state_file),
            "status": str(self.status_file),
            "manifest": str(self.manifest_file),
            "checkpoint_manifest": str(self.checkpoint_manifest),
            "archive": str(self.archive),
            "monitor_pid": str(self.monitor_pid),
            "watchdog_pid": str(self.watchdog_pid),
            "training_pid": str(self.training_pid),
            "tensorboard_pid": str(self.tensorboard_pid),
        }


def default_state() -> Dict[str, Any]:
    """给出监控状态文件的最小结构。"""
    return {
        "schema_version": SCHEMA_VERSION,
        "status": "idle",
        "stage_index": 0,
        "attempt": 0,
        "recoveries": 0,
        "failure_history": [],
        "training_pid": None,
        "training_cmdline": [],
        "training_started_at": None,
        "exit_code": None,
        "log_offset": 0,
        "progress_value": None,
        "progress_at": None,
        "last_progress_activity_at": None,
        "last_activity_at": None,
        "activity_fingerprint": "",
        "completion_seen": False,
        "resume_checkpoint": None,
        "archive_paths": [],
        "stop_reason": None,
        "resource_alerts": {},
        "updated_at": None,
    }


def notify_send(title: str, message: str, enabled: bool = True, urgent: str = "critical") -> bool:
    """发送桌面通知；缺少 notify-send 或桌面会话时只返回失败。"""
    if not enabled or not shutil.which("notify-send"):
        return False
    try:
        result = subprocess.run(
            ["notify-send", "--urgency", urgent, title, message],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=3,
            check=False,
        )
        return result.returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


class LocoJumpMonitor:
    """负责启动、巡检、归档、阶段推进和受控恢复的本地监控器。"""

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.paths = MonitorPaths(args.project_root.resolve(), args.run_root.resolve())
        self.paths.ensure()
        self.config = self._load_command_config()
        self.state = self._load_state()
        self.process: Optional[subprocess.Popen[bytes]] = None
        self._checkpoint_cache: Dict[str, Tuple[str, bool, str, List[str]]] = {}
        self._notify_cache: Dict[str, str] = {}
        self._stop_requested = False

    def _load_command_config(self) -> Dict[str, Any]:
        """读取启动脚本写入的分阶段命令清单。"""
        data = read_json(self.paths.command_file, {})
        if not isinstance(data, dict) or not data.get("stages"):
            return {
                "schema_version": SCHEMA_VERSION,
                "run_id": self.args.run_id,
                "stages": list(DEFAULT_STAGES),
                "completion_markers": list(DEFAULT_COMPLETION_MARKERS),
                "resume_flag": "--resume",
                "resume_value": "{checkpoint}",
            }
        return data

    def _load_state(self) -> Dict[str, Any]:
        """读取并补齐状态文件，损坏时以空闲状态重新开始观察。"""
        loaded = read_json(self.paths.state_file, {})
        state = default_state()
        if isinstance(loaded, dict):
            state.update(loaded)
        return state

    def _save_state(self) -> None:
        """持久化状态并同步 PID 文件，便于监控进程重启后续管。"""
        self.state["updated_at"] = now_iso()
        self.state["schema_version"] = SCHEMA_VERSION
        atomic_write_json(self.paths.state_file, self.state)
        if self.state.get("training_pid"):
            self.paths.training_pid.write_text(
                f"{self.state['training_pid']}\n", encoding="ascii"
            )
        elif self.paths.training_pid.exists():
            try:
                self.paths.training_pid.unlink()
            except OSError:
                pass

    def _stage(self) -> Dict[str, Any]:
        """返回当前阶段；索引越界时停在最后阶段。"""
        stages = self.config.get("stages") or list(DEFAULT_STAGES)
        index = max(0, min(int(self.state.get("stage_index", 0)), len(stages) - 1))
        stage = dict(stages[index])
        stage.setdefault("full_name", stage.get("id", f"阶段{index}"))
        stage.setdefault("cumulative_target_steps", 0)
        stage.setdefault("tag", stage.get("log_tag", ""))
        stage.setdefault("log_tag", stage.get("tag", ""))
        stage["index"] = index
        return stage

    def emit(
        self,
        event: str,
        message: str,
        level: str = "INFO",
        **data: Any,
    ) -> None:
        """写结构化事件日志，并同步打印一行便于 monitor.log 阅读。"""
        record = {
            "timestamp": now_iso(),
            "run_id": self.config.get("run_id", self.args.run_id),
            "event": event,
            "level": level,
            "message": message,
            "stage_id": self._stage().get("id"),
            "stage_full_name": self._stage().get("full_name"),
            "stage_log_tag": self._stage().get("log_tag"),
        }
        if data:
            record["data"] = data
        append_jsonl(self.paths.events_log, record)
        print(
            f"[{record['timestamp']}] [{level}] {event}: {message}",
            flush=True,
        )

    def emit_phase(self, action: str, detail: str) -> None:
        """单独记录阶段切换，保证人工巡检可直接读 phase.log。"""
        stage = self._stage()
        append_jsonl(
            self.paths.phase_log,
            {
                "timestamp": now_iso(),
                "action": action,
                "stage_id": stage.get("id"),
                "full_name": stage.get("full_name"),
                "tag": stage.get("tag"),
                "log_tag": stage.get("log_tag"),
                "cumulative_target_steps": stage.get("cumulative_target_steps"),
                "checkpoint_prefix": self.config.get("checkpoint_prefix"),
                "detail": detail,
            },
        )

    def emit_pid(self, action: str, pid: Optional[int], cmdline: Sequence[str] = ()) -> None:
        """记录 PID 启停、退出和归档事件。"""
        append_jsonl(
            self.paths.pid_log,
            {
                "timestamp": now_iso(),
                "action": action,
                "pid": pid,
                "cmdline": list(cmdline),
                "monitor_pid": os.getpid(),
            },
        )

    def notify(self, key: str, title: str, message: str, urgent: str = "critical") -> None:
        """通知去重；短时间重复同一关键事件不会刷屏。"""
        if self.args.no_notify:
            return
        marker = f"{title}|{message}"
        if self._notify_cache.get(key) == marker:
            return
        sent = notify_send(title, message, enabled=True, urgent=urgent)
        self._notify_cache[key] = marker
        self.emit(
            "desktop_notification",
            "桌面通知已发送" if sent else "桌面通知不可用，已保留事件日志",
            level="INFO" if sent else "WARN",
            key=key,
            sent=sent,
            title=title,
            message=message,
        )

    def _write_manifest(self) -> None:
        """写出完整阶段 manifest，明确正式阶段与累计目标。"""
        stages = []
        for stage in self.config.get("stages") or list(DEFAULT_STAGES):
            stages.append(
                {
                    "id": stage.get("id"),
                    "full_name": stage.get("full_name"),
                    "cumulative_target_steps": stage.get("cumulative_target_steps"),
                    "tag": stage.get("tag"),
                    "log_tag": stage.get("log_tag"),
                }
            )
        atomic_write_json(
            self.paths.manifest_file,
            {
                "schema_version": SCHEMA_VERSION,
                "run_id": self.config.get("run_id", self.args.run_id),
                "checkpoint_prefix": self.config.get(
                    "checkpoint_prefix", self.args.run_id
                ),
                "tensorboard": "http://127.0.0.1:6006/",
                "stages": stages,
                "current_stage_index": self.state.get("stage_index", 0),
                "current_stage": self._stage(),
                "paths": self.paths.convention(),
                "thresholds": {
                    "poll_interval_seconds": self.args.poll_interval,
                    "stall_timeout_seconds": self.args.stall_timeout,
                    "startup_grace_seconds": self.args.startup_grace,
                    "failure_window_seconds": self.args.failure_window,
                    "max_recoveries": self.args.max_recoveries,
                    "memory_used_percent": self.args.memory_threshold,
                    "disk_free_gib": self.args.disk_free_threshold,
                    "tmp_free_gib": self.args.tmp_free_threshold,
                },
                "updated_at": now_iso(),
            },
        )

    def _checkpoint_dir(self) -> Path:
        """确定 checkpoint 目录，命令清单优先于当前阶段 argv。"""
        configured = self.config.get("checkpoint_dir")
        if configured:
            return Path(configured)
        stage_argv = self._stage().get("argv") or []
        configured = parse_value(stage_argv, "--ckptdir")
        if configured:
            return Path(str(configured))
        return self.args.project_root / "checkpoints" / self.args.run_id

    def refresh_checkpoint_manifest(self) -> Tuple[List[Dict[str, Any]], Optional[Path]]:
        """生成 checkpoint 清单并返回最佳有效恢复点。"""
        ckpt_dir = self._checkpoint_dir()
        entries: List[Dict[str, Any]] = []
        if ckpt_dir.is_dir():
            for path in sorted(ckpt_dir.glob("*.zip")):
                try:
                    stat = path.stat()
                except OSError:
                    continue
                key = f"{stat.st_size}:{stat.st_mtime_ns}"
                cached = self._checkpoint_cache.get(str(path))
                if cached is None or cached[0] != key:
                    valid, reason, members = validate_checkpoint(path)
                    self._checkpoint_cache[str(path)] = (
                        key,
                        valid,
                        reason,
                        members,
                    )
                    cached = self._checkpoint_cache[str(path)]
                entries.append(
                    {
                        "path": str(path.resolve()),
                        "name": path.name,
                        "step": checkpoint_step(path),
                        "size_bytes": stat.st_size,
                        "mtime": datetime.fromtimestamp(stat.st_mtime)
                        .astimezone()
                        .isoformat(timespec="seconds"),
                        "valid": cached[1],
                        "validation": cached[2],
                        "members": cached[3],
                    }
                )
        entries.sort(
            key=lambda item: (
                item["step"] is None,
                item["step"] if item["step"] is not None else -1,
                item["mtime"],
            )
        )
        valid_entries = [item for item in entries if item["valid"]]
        best_path = (
            Path(valid_entries[-1]["path"]) if valid_entries else None
        )
        atomic_write_json(
            self.paths.checkpoint_manifest,
            {
                "schema_version": SCHEMA_VERSION,
                "run_id": self.config.get("run_id", self.args.run_id),
                "checkpoint_dir": str(ckpt_dir.resolve()) if ckpt_dir.exists() else str(ckpt_dir),
                "count": len(entries),
                "valid_count": len(valid_entries),
                "best_resume_checkpoint": str(best_path) if best_path else None,
                "entries": entries,
                "updated_at": now_iso(),
            },
        )
        return entries, best_path

    def _archive(self, reason: str) -> Optional[Path]:
        """把当前训练日志改名归档，并写入尾部与状态元数据。"""
        stamp = compact_time()
        suffix = sanitize(reason)
        archive_dir = self.paths.archive / f"{stamp}_{self.state.get('attempt', 0)}_{suffix}"
        counter = 1
        while archive_dir.exists():
            archive_dir = self.paths.archive / f"{stamp}_{counter}_{suffix}"
            counter += 1
        archive_dir.mkdir(parents=True, exist_ok=False)
        moved = False
        size = 0
        tail = ""
        if self.paths.train_log.exists():
            try:
                size = self.paths.train_log.stat().st_size
                tail = tail_text(self.paths.train_log, 32 * 1024)
                os.replace(self.paths.train_log, archive_dir / "train.log")
                moved = True
            except OSError as exc:
                self.emit(
                    "archive_partial_failure",
                    f"训练日志归档失败：{exc}",
                    level="ERROR",
                    reason=reason,
                )
        stage = self._stage()
        atomic_write_json(
            archive_dir / "metadata.json",
            {
                "schema_version": SCHEMA_VERSION,
                "run_id": self.config.get("run_id", self.args.run_id),
                "reason": reason,
                "archived_at": now_iso(),
                "stage": stage,
                "exit_code": self.state.get("exit_code"),
                "training_pid": self.state.get("training_pid"),
                "progress_value": self.state.get("progress_value"),
                "log_moved": moved,
                "log_size_bytes": size,
                "log_tail": tail,
                "state": self.state,
            },
        )
        self.state["log_offset"] = 0
        self.state.setdefault("archive_paths", []).append(str(archive_dir))
        self.emit(
            "archived",
            f"已归档训练现场：{archive_dir}",
            reason=reason,
            archive=str(archive_dir),
            log_moved=moved,
        )
        return archive_dir

    def _build_stage_argv(self, resume_checkpoint: Optional[Path]) -> List[str]:
        """生成当前阶段启动命令；恢复时替换旧的 --resume 参数。"""
        stage = self._stage()
        argv = [str(item) for item in stage.get("argv", [])]
        if not argv:
            raise RuntimeError(f"阶段 {stage.get('id')} 缺少 argv")
        resume_flag = str(self.config.get("resume_flag", "--resume"))
        cleaned: List[str] = []
        skip_next = False
        for index, item in enumerate(argv):
            if skip_next:
                skip_next = False
                continue
            if item == resume_flag:
                if index + 1 < len(argv):
                    skip_next = True
                continue
            cleaned.append(item)
        if resume_checkpoint is not None:
            template = str(self.config.get("resume_value", "{checkpoint}"))
            cleaned.extend(
                [
                    resume_flag,
                    template.replace("{checkpoint}", str(resume_checkpoint)),
                ]
            )
        return cleaned

    def launch_stage(self, resume_checkpoint: Optional[Path], reason: str) -> bool:
        """启动当前阶段，并把 PID、阶段和事件信息一次性落盘。"""
        if self.state.get("stop_reason"):
            return False
        if self.paths.train_log.exists() and self.paths.train_log.stat().st_size > 0:
            # 首次启动前也归档旧日志，避免把历史错误误判成新故障。
            if self.state.get("attempt", 0) == 0:
                self._archive("stale_log_before_start")
        argv = self._build_stage_argv(resume_checkpoint)
        self.paths.train_log.parent.mkdir(parents=True, exist_ok=True)
        try:
            log_handle = self.paths.train_log.open("ab", buffering=0)
            self.process = subprocess.Popen(
                argv,
                cwd=str(self.args.project_root),
                stdout=log_handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except OSError as exc:
            self.state["status"] = "stopped"
            self.state["stop_reason"] = "launch_failed"
            self.emit(
                "launch_failed",
                f"训练进程启动失败：{exc}",
                level="ERROR",
                reason=reason,
                argv=argv,
            )
            self.notify(
                "launch_failed",
                f"{self.args.run_id} 启动失败",
                str(exc),
            )
            self._save_state()
            return False
        finally:
            if "log_handle" in locals() and not log_handle.closed:
                log_handle.close()
        stage = self._stage()
        self.state.update(
            {
                "status": "running",
                "attempt": int(self.state.get("attempt", 0)) + 1,
                "training_pid": self.process.pid,
                "training_cmdline": argv,
                "training_started_at": now_iso(),
                "exit_code": None,
                "log_offset": self.paths.train_log.stat().st_size
                if self.paths.train_log.exists()
                else 0,
                "completion_seen": False,
                "last_progress_activity_at": time.time(),
                "resume_checkpoint": str(resume_checkpoint)
                if resume_checkpoint
                else None,
                "last_activity_at": time.time(),
                "activity_fingerprint": self._activity_fingerprint(),
                "stop_reason": None,
            }
        )
        self._save_state()
        self.emit_pid("start", self.process.pid, argv)
        self.emit_phase(
            "start",
            f"{reason}：{stage.get('full_name')}，累计目标 "
            f"{stage.get('cumulative_target_steps')} 步",
        )
        self.emit(
            "training_started",
            f"训练进程已启动 PID={self.process.pid}",
            pid=self.process.pid,
            stage_id=stage.get("id"),
            tag=stage.get("tag"),
            log_tag=stage.get("log_tag"),
            cumulative_target_steps=stage.get("cumulative_target_steps"),
            resume_checkpoint=str(resume_checkpoint) if resume_checkpoint else None,
            argv=argv,
        )
        self._save_status()
        return True

    def _activity_fingerprint(self) -> str:
        """汇总日志、TensorBoard 事件和 checkpoint 的增长指纹。"""
        parts: List[str] = []
        if self.paths.train_log.exists():
            try:
                stat = self.paths.train_log.stat()
                parts.append(f"log:{stat.st_size}:{stat.st_mtime_ns}")
            except OSError:
                pass
        runs_dir = Path(
            self.config.get(
                "runs_dir",
                str(self.args.project_root / "runs" / self.args.run_id),
            )
        )
        if runs_dir.is_dir():
            for path in sorted(runs_dir.rglob("events*")):
                try:
                    stat = path.stat()
                except OSError:
                    continue
                parts.append(f"event:{path}:{stat.st_size}:{stat.st_mtime_ns}")
        ckpt_dir = self._checkpoint_dir()
        if ckpt_dir.is_dir():
            try:
                items = sorted(ckpt_dir.glob("*.zip"))
                newest = max(
                    (path.stat().st_mtime_ns for path in items), default=0
                )
                parts.append(f"ckpt:{len(items)}:{newest}")
            except OSError:
                pass
        return "|".join(parts)

    def scan_log(self) -> Tuple[List[str], Optional[int], bool]:
        """读取新增日志，返回致命类型、新进度和完成标记。"""
        fatal: List[str] = []
        progress: Optional[int] = None
        completion = False
        if not self.paths.train_log.exists():
            return fatal, progress, completion
        try:
            size = self.paths.train_log.stat().st_size
        except OSError:
            return fatal, progress, completion
        offset = int(self.state.get("log_offset", 0))
        if size < offset:
            offset = 0
        if size == offset:
            return fatal, progress, completion
        try:
            with self.paths.train_log.open("rb") as handle:
                handle.seek(offset)
                chunk = handle.read(size - offset)
        except OSError:
            return fatal, progress, completion
        text = chunk.decode("utf-8", errors="replace")
        self.state["log_offset"] = size
        self.state["last_activity_at"] = time.time()
        progress = parse_int_progress(text)
        if progress is not None:
            old = self.state.get("progress_value")
            self.state["progress_value"] = max(
                int(old or 0), int(progress)
            )
            self.state["progress_at"] = now_iso()
            if int(self.state["progress_value"]) > int(old or 0):
                self.state["last_progress_activity_at"] = time.time()
        for name, pattern in FATAL_PATTERNS:
            match = pattern.search(text)
            if match:
                fatal.append(name)
                self.emit(
                    "fatal_log_pattern",
                    f"训练日志命中致命模式 {name}",
                    level="ERROR",
                    pattern=name,
                    sample=match.group(0)[:200],
                )
        markers = self.config.get("completion_markers") or DEFAULT_COMPLETION_MARKERS
        completion = any(str(marker) in text for marker in markers)
        if completion:
            self.state["completion_seen"] = True
        return fatal, progress, completion

    def _resource_snapshot(self) -> Dict[str, Any]:
        """汇总内存、项目磁盘与 /tmp 阈值。"""
        return {
            "memory": memory_snapshot(),
            "project_disk": disk_snapshot(self.args.project_root),
            "tmp": disk_snapshot(Path(self.args.tmp_path)),
            "thresholds": {
                "memory_used_percent": self.args.memory_threshold,
                "project_disk_free_gib": self.args.disk_free_threshold,
                "tmp_free_gib": self.args.tmp_free_threshold,
                "persistent_polls": self.args.resource_persist_polls,
            },
        }

    def check_resources(self) -> Optional[str]:
        """检查资源阈值；连续越界后作为不可自动恢复故障停止。"""
        snapshot = self._resource_snapshot()
        alerts: Dict[str, Any] = self.state.setdefault("resource_alerts", {})
        breaches: List[str] = []
        memory_percent = snapshot["memory"].get("used_percent")
        if memory_percent is not None and memory_percent >= self.args.memory_threshold:
            breaches.append("memory")
        disk_free = snapshot["project_disk"].get("free_gib")
        if disk_free is not None and disk_free <= self.args.disk_free_threshold:
            breaches.append("project_disk")
        tmp_free = snapshot["tmp"].get("free_gib")
        if tmp_free is not None and tmp_free <= self.args.tmp_free_threshold:
            breaches.append("tmp")
        now = time.time()
        for name in breaches:
            item = alerts.setdefault(name, {"first_at": now, "consecutive": 0})
            item["consecutive"] = int(item.get("consecutive", 0)) + 1
            item["last_at"] = now
            item["snapshot"] = snapshot.get(
                "memory" if name == "memory" else name
            )
            if item["consecutive"] == 1:
                self.emit(
                    "resource_threshold_warning",
                    f"资源阈值首次越界：{name}",
                    level="WARN",
                    resource=name,
                    snapshot=item["snapshot"],
                )
                self.notify(
                    f"resource_{name}",
                    f"{self.args.run_id} 资源告警",
                    f"{name} 已越过阈值",
                    urgent="normal",
                )
        for name in list(alerts):
            if name not in breaches:
                alerts.pop(name, None)
        self.state["_resource_snapshot"] = snapshot
        critical = [
            name
            for name in breaches
            if int(alerts.get(name, {}).get("consecutive", 0))
            >= self.args.resource_persist_polls
        ]
        return ",".join(critical) if critical else None

    def _stop_current_process(self) -> None:
        """停止当前训练子进程，退出码未知时保留已有值。"""
        pid = self.state.get("training_pid")
        if self.process is not None and self.process.poll() is not None:
            self.state["exit_code"] = self.process.returncode
        terminate_pid_group(int(pid) if pid else None)
        if self.process is not None:
            try:
                self.process.wait(timeout=3)
                if self.state.get("exit_code") is None:
                    self.state["exit_code"] = self.process.returncode
            except (subprocess.TimeoutError, OSError):
                pass
        if pid:
            self.emit_pid("stop", int(pid))
        self.process = None
        self.state["training_pid"] = None

    def _purge_failure_history(self) -> None:
        """只保留 30 分钟窗口内失败，超窗后恢复新的巡检窗口。"""
        cutoff = time.time() - self.args.failure_window
        history = [
            float(item)
            for item in self.state.get("failure_history", [])
            if float(item) >= cutoff
        ]
        self.state["failure_history"] = history

    def handle_failure(
        self,
        reason: str,
        exit_code: Optional[int],
        detail: str,
    ) -> bool:
        """执行“停止→归档→checkpoint 校验→恢复”，超限则停止。"""
        self._stop_current_process()
        self.state["exit_code"] = exit_code
        self.state["status"] = "recovering"
        self.state["failure_history"] = [
            float(item)
            for item in self.state.get("failure_history", [])
            if float(item) >= time.time() - self.args.failure_window
        ]
        self.state["failure_history"].append(time.time())
        self.emit(
            "training_failed",
            f"训练异常，进入恢复流程：{detail}",
            level="ERROR",
            reason=reason,
            exit_code=exit_code,
            failure_count=len(self.state["failure_history"]),
            failure_window_seconds=self.args.failure_window,
        )
        self.emit(
            reason,
            detail,
            level="ERROR",
            category="failure",
            exit_code=exit_code,
        )
        self.notify(
            "failure",
            f"{self.args.run_id} 训练异常",
            f"{detail}；正在归档并校验 checkpoint",
        )
        self._archive(reason)
        entries, checkpoint = self.refresh_checkpoint_manifest()
        if checkpoint is not None and str(checkpoint) != self.state.get(
            "last_progress_checkpoint"
        ):
            self.state["last_progress_checkpoint"] = str(checkpoint)
            self.state["last_progress_activity_at"] = time.time()
        if checkpoint is None:
            self.state["status"] = "stopped"
            self.state["stop_reason"] = "no_valid_checkpoint"
            self.emit(
                "recovery_blocked",
                "没有可验证的有效 checkpoint，按安全策略停止",
                level="ERROR",
                checkpoint_count=len(entries),
            )
            self.notify(
                "no_checkpoint",
                f"{self.args.run_id} 恢复失败",
                "没有有效 checkpoint，监控已停止",
            )
            self._save_state()
            self._save_status()
            return False
        failures = len(self.state["failure_history"])
        # 前三次失败分别允许第一至第三次恢复；第四次仍在窗口内即停止。
        if (
            int(self.state.get("recoveries", 0)) >= self.args.max_recoveries
            or failures - 1 >= self.args.max_recoveries
        ):
            self.state["status"] = "stopped"
            self.state["stop_reason"] = "consecutive_failure_limit"
            self.emit(
                "recovery_limit",
                f"{self.args.failure_window} 秒内连续失败达到上限，停止自动恢复",
                level="ERROR",
                failure_count=failures,
                max_recoveries=self.args.max_recoveries,
            )
            self.notify(
                "recovery_limit",
                f"{self.args.run_id} 恢复停止",
                f"{self.args.failure_window} 秒内连续失败，已达恢复上限",
            )
            self._save_state()
            self._save_status()
            return False
        if self.launch_stage(Path(checkpoint), f"recovery_{failures}"):
            self.state["recoveries"] = int(self.state.get("recoveries", 0)) + 1
            self._save_state()
            self.emit(
                "recovery_started",
                f"第 {self.state['recoveries']} 次恢复已启动",
                checkpoint=str(checkpoint),
                failure_count=failures,
                max_recoveries=self.args.max_recoveries,
            )
            return True
        return False

    def handle_completion(
        self,
        exit_code: Optional[int],
        detail: str,
    ) -> bool:
        """阶段完成后先校验 checkpoint，再决定推进下一阶段。"""
        self._stop_current_process()
        self.state["exit_code"] = exit_code
        entries, checkpoint = self.refresh_checkpoint_manifest()
        stage = self._stage()
        self.emit(
            "stage_completed",
            f"{stage.get('full_name')} 正常完成",
            exit_code=exit_code,
            detail=detail,
            progress_value=self.state.get("progress_value"),
            cumulative_target_steps=stage.get("cumulative_target_steps"),
            valid_checkpoint=str(checkpoint) if checkpoint else None,
        )
        self._archive("stage_complete")
        stages = self.config.get("stages") or list(DEFAULT_STAGES)
        next_index = int(stage["index"]) + 1
        if next_index >= len(stages):
            self.state["status"] = "completed"
            self.state["stop_reason"] = None
            self.state["training_pid"] = None
            self.emit_phase("complete", "全部阶段完成")
            self.emit(
                "run_completed",
                "yobogo_loco_jump_v1 全部阶段完成",
                valid_checkpoint=str(checkpoint) if checkpoint else None,
                checkpoint_count=len(entries),
            )
            self.notify(
                "completed",
                f"{self.args.run_id} 训练完成",
                "全部阶段已正常结束",
                urgent="normal",
            )
            self._save_state()
            self._save_status()
            return False
        if checkpoint is None:
            self.state["status"] = "stopped"
            self.state["stop_reason"] = "no_checkpoint_for_next_stage"
            self.emit(
                "stage_transition_blocked",
                "当前阶段完成但没有有效 checkpoint，停止阶段推进",
                level="ERROR",
            )
            self._save_state()
            self._save_status()
            return False
        self.state["stage_index"] = next_index
        self.state["failure_history"] = []
        self.state["resume_checkpoint"] = str(checkpoint)
        self.emit_phase(
            "advance",
            f"{stage.get('full_name')} → {stages[next_index].get('full_name')}",
        )
        return self.launch_stage(Path(checkpoint), "stage_advance")

    def _is_success_exit(
        self,
        exit_code: Optional[int],
        completion_seen: bool,
    ) -> bool:
        """只有显式完成标记、累计目标或退出码 0 才按正常退出处理。"""
        if exit_code not in (0, None):
            return False
        if completion_seen:
            return True
        target = int(self._stage().get("cumulative_target_steps") or 0)
        progress = int(self.state.get("progress_value") or 0)
        return bool(target and progress >= target) or exit_code == 0

    def _save_status(self) -> None:
        """把人工巡检需要的状态写成单个 status.json。"""
        stage = self._stage()
        last_activity = self.state.get("last_activity_at")
        resource = self.state.get("_resource_snapshot") or self._resource_snapshot()
        status = {
            "schema_version": SCHEMA_VERSION,
            "run_id": self.config.get("run_id", self.args.run_id),
            "status": self.state.get("status"),
            "stop_reason": self.state.get("stop_reason"),
            "stage": stage,
            "stage_index": self.state.get("stage_index", 0),
            "stage_count": len(self.config.get("stages") or list(DEFAULT_STAGES)),
            "attempt": self.state.get("attempt", 0),
            "recoveries": self.state.get("recoveries", 0),
            "failure_history_count": len(self.state.get("failure_history", [])),
            "training_pid": self.state.get("training_pid"),
            "monitor_pid": os.getpid(),
            "progress_value": self.state.get("progress_value"),
            "progress_at": self.state.get("progress_at"),
            "stalled_for_seconds": round(time.time() - float(last_activity), 1)
            if last_activity
            else None,
            "progress_stalled_for_seconds": round(
                time.time()
                - float(self.state.get("last_progress_activity_at") or time.time()),
                1,
            ),
            "resume_checkpoint": self.state.get("resume_checkpoint"),
            "archives": self.state.get("archive_paths", []),
            "resources": resource,
            "tensorboard": "http://127.0.0.1:6006/",
            "paths": self.paths.convention(),
            "updated_at": now_iso(),
        }
        atomic_write_json(self.paths.status_file, status)

    def inspect_once(self, allow_actions: bool) -> Dict[str, Any]:
        """单次巡检；allow_actions=false 时绝不启动、归档或恢复。"""
        resource_critical = self.check_resources()
        fatal: List[str] = []
        completion = False
        progress: Optional[int] = None
        if self.state.get("status") == "running":
            fatal, progress, completion = self.scan_log()
            fingerprint = self._activity_fingerprint()
            if fingerprint != self.state.get("activity_fingerprint"):
                self.state["activity_fingerprint"] = fingerprint
                self.state["last_activity_at"] = time.time()
        self.refresh_checkpoint_manifest()
        self._write_manifest()
        if not allow_actions:
            self._save_status()
            return {
                "status": self.state.get("status"),
                "fatal": fatal,
                "completion": completion,
                "progress": progress,
                "resource_critical": resource_critical,
            }
        if resource_critical:
            self._stop_current_process()
            self.state["status"] = "stopped"
            self.state["stop_reason"] = f"resource_guard:{resource_critical}"
            self._archive("resource_guard")
            self.emit(
                "resource_guard_stop",
                f"资源阈值持续越界，停止自动恢复：{resource_critical}",
                level="ERROR",
                resources=resource_critical,
            )
            self.notify(
                "resource_guard",
                f"{self.args.run_id} 资源保护停止",
                f"持续越界：{resource_critical}",
            )
            self._save_state()
            self._save_status()
            return {"status": "stopped", "stop_reason": self.state["stop_reason"]}
        return {
            "status": self.state.get("status"),
            "fatal": fatal,
            "completion": completion,
            "progress": progress,
            "resource_critical": None,
        }

    def run(self) -> int:
        """主循环：监控当前进程，异常时最多执行窗口内约定次数恢复。"""
        def request_stop(_signum: int, _frame: Any) -> None:
            """把停止信号转成受控退出，避免留下孤儿训练进程。"""
            self._stop_requested = True

        previous_sigterm = signal.signal(signal.SIGTERM, request_stop)
        previous_sigint = signal.signal(signal.SIGINT, request_stop)
        self.emit(
            "monitor_started",
            "本地监控已启动",
            monitor_pid=os.getpid(),
            mode="run",
            command_file=str(self.paths.command_file),
        )
        self.paths.monitor_pid.write_text(f"{os.getpid()}\n", encoding="ascii")
        self._write_manifest()
        self.refresh_checkpoint_manifest()
        if self.state.get("stop_reason") in {
            "consecutive_failure_limit",
            "no_valid_checkpoint",
            "resource_guard",
        }:
            self.emit(
                "monitor_refused_restart",
                f"状态中已有停止原因 {self.state.get('stop_reason')}，拒绝自动重启",
                level="WARN",
            )
            self._save_status()
            return 2
        if self.state.get("status") in {"idle", None}:
            self.state["status"] = "running"
            self.state["stage_index"] = int(self.state.get("stage_index", 0))
            if not self.launch_stage(None, "initial_start"):
                return 1
        elif self.state.get("status") == "completed":
            self.emit("monitor_completed", "训练已完成，无需再次启动", level="INFO")
            self._save_status()
            return 0
        while True:
            result = self.inspect_once(allow_actions=True)
            if self._stop_requested:
                self._stop_current_process()
                self.state["status"] = "stopped"
                self.state["stop_reason"] = "operator_stop"
                self._archive("operator_stop")
                self.emit(
                    "operator_stop",
                    "收到停止信号，已停止训练并归档现场",
                    level="WARN",
                )
                break
            if self.state.get("stop_reason"):
                break
            if self.state.get("status") not in {"running", "recovering"}:
                break
            pid = self.state.get("training_pid")
            exit_code: Optional[int] = None
            process_exited = False
            if self.process is not None:
                polled = self.process.poll()
                if polled is not None:
                    process_exited = True
                    exit_code = polled
            elif not pid_alive(int(pid) if pid else None):
                process_exited = True
            elif not process_matches(int(pid), self.state.get("training_cmdline", [])):
                process_exited = True
            completion_seen = bool(self.state.get("completion_seen"))
            if process_exited:
                reason = "process_exit"
                if result.get("fatal"):
                    reason = result["fatal"][0]
                if self._is_success_exit(exit_code, completion_seen):
                    if not self.handle_completion(
                        exit_code,
                        "完成标记或累计目标已满足",
                    ):
                        break
                else:
                    if not self.handle_failure(
                        reason,
                        exit_code,
                        f"训练进程退出（exit_code={exit_code}）",
                    ):
                        break
                continue
            if result.get("fatal"):
                reason = result["fatal"][0]
                if not self.handle_failure(
                    reason,
                    None,
                    f"日志出现 {reason}",
                ):
                    break
                continue
            started = self.state.get("training_started_at")
            started_epoch = 0.0
            if started:
                try:
                    started_epoch = datetime.fromisoformat(str(started)).timestamp()
                except ValueError:
                    started_epoch = time.time()
            age = time.time() - started_epoch
            last_progress_activity = float(
                self.state.get("last_progress_activity_at") or time.time()
            )
            if (
                age >= self.args.startup_grace
                and time.time() - last_progress_activity
                >= self.args.stall_timeout
            ):
                if not self.handle_failure(
                    "progress_stalled",
                    None,
                    f"进度已停滞 {int(time.time() - last_progress_activity)} 秒",
                ):
                    break
                continue
            self._save_state()
            self._save_status()
            time.sleep(max(0.1, self.args.poll_interval))
        self._save_state()
        self._save_status()
        self.emit(
            "monitor_stopped",
            f"监控结束，状态={self.state.get('status')}",
            stop_reason=self.state.get("stop_reason"),
        )
        signal.signal(signal.SIGTERM, previous_sigterm)
        signal.signal(signal.SIGINT, previous_sigint)
        try:
            self.paths.monitor_pid.unlink()
        except OSError:
            pass
        if self.state.get("status") == "completed":
            return 0
        if self.state.get("status") == "stopped":
            return 2
        return 1


def build_parser() -> argparse.ArgumentParser:
    """构造命令行参数；无模式时默认只读 dry-run。"""
    here = Path(__file__).resolve()
    project_root = here.parents[2]
    default_run_root = project_root / "logs" / DEFAULT_RUN_ID
    parser = argparse.ArgumentParser(
        description="yobogo_loco_jump_v1 本地训练监控与恢复看门狗",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="只打印计划，不创建或启动任何进程")
    mode.add_argument("--once", action="store_true", help="执行一次只读巡检并写 status.json")
    mode.add_argument("--run", action="store_true", help="启动监控主循环；仅此模式可启动训练")
    mode.add_argument("--status", action="store_true", help="读取并打印现有 status.json")
    parser.add_argument("--run-id", default=DEFAULT_RUN_ID, help="运行标识")
    parser.add_argument("--project-root", type=Path, default=project_root, help="项目根目录")
    parser.add_argument("--run-root", type=Path, default=default_run_root, help="本任务日志根目录")
    parser.add_argument("--command-file", type=Path, default=None, help="训练命令清单 JSON")
    parser.add_argument("--poll-interval", type=float, default=5.0, help="巡检间隔秒")
    parser.add_argument("--stall-timeout", type=float, default=600.0, help="进度停滞秒数")
    parser.add_argument(
        "--startup-grace",
        type=float,
        default=600.0,
        help="启动宽限期秒数",
    )
    parser.add_argument("--failure-window", type=float, default=1800.0, help="连续失败统计窗口秒数")
    parser.add_argument("--max-recoveries", type=int, default=3, help="窗口内最大自动恢复次数")
    parser.add_argument("--memory-threshold", type=float, default=90.0, help="内存使用率百分比阈值")
    parser.add_argument("--disk-free-threshold", type=float, default=5.0, help="项目磁盘剩余 GiB 阈值")
    parser.add_argument("--tmp-free-threshold", type=float, default=2.0, help="/tmp 剩余 GiB 阈值")
    parser.add_argument("--resource-persist-polls", type=int, default=2, help="资源连续越界多少轮后硬停止")
    parser.add_argument("--tmp-path", default="/tmp", help="临时目录检查路径")
    parser.add_argument("--no-notify", action="store_true", help="禁用桌面 notify-send")
    return parser


def print_dry_run(args: argparse.Namespace) -> None:
    """打印安全计划，显式证明默认路径不启动正式训练。"""
    paths = MonitorPaths(args.project_root.resolve(), args.run_root.resolve())
    config = read_json(paths.command_file, {})
    stages = config.get("stages") or list(DEFAULT_STAGES)
    print("【dry-run】yobogo_loco_jump_v1 监控计划")
    print(f"  项目根目录       : {paths.project_root}")
    print(f"  日志根目录       : {paths.run_root}")
    print(f"  TensorBoard      : http://127.0.0.1:6006/")
    print("  训练启动         : 禁止（仅 --run 且命令清单存在时才启动）")
    print("  路径约定：")
    for name, value in paths.convention().items():
        print(f"    {name:<20}: {value}")
    print("  阶段清单：")
    for stage in stages:
        print(
            f"    {stage.get('id')}: {stage.get('full_name')}，"
            f"累计 {stage.get('cumulative_target_steps')} 步，"
            f"tag={stage.get('tag')}，log_tag={stage.get('log_tag')}"
        )
    print(f"  阈值：内存 {args.memory_threshold}%，项目磁盘 "
          f"{args.disk_free_threshold} GiB，/tmp {args.tmp_free_threshold} GiB")
    print(f"  恢复：最多 {args.max_recoveries} 次 / {int(args.failure_window)} 秒窗口")


def main(argv: Optional[Sequence[str]] = None) -> int:
    """入口：除显式 --run 外均不启动训练。"""
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.command_file is not None:
        args.run_root = args.command_file.parent
    if args.status:
        paths = MonitorPaths(args.project_root.resolve(), args.run_root.resolve())
        status = read_json(paths.status_file, {})
        print(json.dumps(status, ensure_ascii=False, indent=2, sort_keys=True))
        return 0 if status else 3
    if args.dry_run or not (args.once or args.run):
        print_dry_run(args)
        return 0
    monitor = LocoJumpMonitor(args)
    if args.once:
        result = monitor.inspect_once(allow_actions=False)
        print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
        return 0
    return monitor.run()


if __name__ == "__main__":
    raise SystemExit(main())
