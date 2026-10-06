#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""yobogo_loco_jump_v1 监控安全与 mock 恢复演练测试。

测试全程只使用临时目录和 mock 进程，绝不启动正式训练。
"""

from __future__ import annotations

import importlib.util
import json
import os
import shlex
import subprocess
import sys
import tempfile
import unittest
import zipfile
from pathlib import Path
from typing import Any, Dict

RL_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = RL_DIR.parents[1]
MONITOR_SCRIPT = RL_DIR / "loco_jump_monitor.py"
WATCHDOG_SCRIPT = RL_DIR / "loco_jump_watchdog.sh"
START_SCRIPT = RL_DIR / "start_loco_jump_training.sh"


def load_monitor_module() -> Any:
    """按文件路径加载监控模块，避免与现有 monitor.py 重名。"""
    spec = importlib.util.spec_from_file_location("loco_jump_monitor_under_test", MONITOR_SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


MONITOR = load_monitor_module()


def make_stage_config(
    root: Path,
    mock_script: Path,
    checkpoint_dir: Path,
    mode: str = "recover_once",
) -> Dict[str, Any]:
    """构造单阶段 mock 命令清单，保持与正式清单相同的核心契约。"""
    return {
        "schema_version": 1,
        "run_id": "mock_loco_jump",
        "project_root": str(root),
        "run_root": str(root / "logs"),
        "checkpoint_dir": str(checkpoint_dir),
        "checkpoint_prefix": "mock_loco_jump",
        "stages": [
            {
                "id": "S0_smoke",
                "full_name": "S0 冒烟",
                "cumulative_target_steps": 1000,
                "tag": "phase0",
                "log_tag": "S0_smoke",
                "argv": [
                    sys.executable,
                    str(mock_script),
                    "--mode",
                    mode,
                    "--ckptdir",
                    str(checkpoint_dir),
                ],
            }
        ],
        "completion_markers": ["RECOVERY_COMPLETED", "最终模型已保存"],
        "resume_flag": "--resume",
        "resume_value": "{checkpoint}",
    }


def write_mock_script(path: Path) -> None:
    """写入首次失败、恢复成功的 mock 训练进程。"""
    path.write_text(
        '''#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Mock 训练：首次创建有效 checkpoint 后退出，带 --resume 时正常完成。"""
import argparse
import sys
import zipfile
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--mode", default="recover_once")
parser.add_argument("--ckptdir", required=True)
parser.add_argument("--resume", default=None)
args = parser.parse_args()
ckpt_dir = Path(args.ckptdir)
ckpt_dir.mkdir(parents=True, exist_ok=True)
checkpoint = ckpt_dir / "mock_1000.zip"

if args.resume is None:
    with zipfile.ZipFile(checkpoint, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("data", "mock-policy")
        archive.writestr("policy.pth", b"mock-tensor")
    print("| rollout/ | total_timesteps | 100 |", flush=True)
    print("MOCK_PROCESS_FAILURE", flush=True)
    raise SystemExit(7)

if not Path(args.resume).is_file() or not zipfile.is_zipfile(Path(args.resume)):
    print("MOCK_BAD_CHECKPOINT", flush=True)
    raise SystemExit(8)
print("MOCK_RESUME checkpoint=" + str(args.resume), flush=True)
print("RECOVERY_COMPLETED", flush=True)
raise SystemExit(0)
''',
        encoding="utf-8",
    )


def write_always_fail_script(path: Path) -> None:
    """写入每次都失败的 mock，用于验证连续失败恢复上限。"""
    path.write_text(
        '''#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Mock 进程：始终创建有效 checkpoint 后非零退出。"""
import argparse
import zipfile
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--mode", default="always_fail")
parser.add_argument("--ckptdir", required=True)
parser.add_argument("--resume", default=None)
args = parser.parse_args()
checkpoint = Path(args.ckptdir) / "mock_1000.zip"
checkpoint.parent.mkdir(parents=True, exist_ok=True)
with zipfile.ZipFile(checkpoint, "w", compression=zipfile.ZIP_DEFLATED) as archive:
    archive.writestr("data", "mock-policy")
print("MOCK_STILL_FAILING resume=" + str(args.resume), flush=True)
raise SystemExit(9)
''',
        encoding="utf-8",
    )


def write_stall_script(path: Path) -> None:
    """写入有有效 checkpoint 但长时间不推进的 mock 进程。"""
    path.write_text(
        '''#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Mock 训练：创建 checkpoint 后保持存活，用于进度停滞演练。"""
import argparse
import time
import zipfile
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--mode", default="stall")
parser.add_argument("--ckptdir", required=True)
parser.add_argument("--resume", default=None)
args = parser.parse_args()
if args.resume is None:
    checkpoint = Path(args.ckptdir) / "mock_1000.zip"
    checkpoint.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(checkpoint, "w") as archive:
        archive.writestr("data", "mock-policy")
    print("MOCK_STALL_READY", flush=True)
    time.sleep(10)
    raise SystemExit(12)
print("RECOVERY_COMPLETED", flush=True)
raise SystemExit(0)
''',
        encoding="utf-8",
    )


def run_monitor(
    project_root: Path,
    run_root: Path,
    command_file: Path,
    extra: list[str] | None = None,
) -> subprocess.CompletedProcess[str]:
    """在临时目录中运行监控主循环，绝不触碰正式训练入口。"""
    command = [
        sys.executable,
        str(MONITOR_SCRIPT),
        "--run",
        "--project-root",
        str(project_root),
        "--run-root",
        str(run_root),
        "--command-file",
        str(command_file),
        "--poll-interval",
        "0.02",
        "--startup-grace",
        "10",
        "--stall-timeout",
        "10",
        "--no-notify",
    ]
    if extra:
        command.extend(extra)
    return subprocess.run(
        command,
        cwd=str(PROJECT_ROOT),
        text=True,
        capture_output=True,
        timeout=20,
        check=False,
    )


class LocoJumpMonitorTests(unittest.TestCase):
    """覆盖安全默认、日志钩子、资源阈值与恢复上限。"""

    def test_default_start_is_dry_run(self) -> None:
        """启动脚本必须打印完整新入口命令，且只读校验 CLI。"""
        result = subprocess.run(
            [str(START_SCRIPT)],
            cwd=str(PROJECT_ROOT),
            text=True,
            capture_output=True,
            timeout=10,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("正式训练         : 未启动", result.stdout)
        self.assertIn("S4_mobile_terrain", result.stdout)
        self.assertNotIn("--start-training", result.stdout)
        self.assertIn("train_loco_jump.py", result.stdout)
        self.assertNotIn("train_ppo.py", result.stdout)
        self.assertIn("checkpoint间隔   : 50000 步", result.stdout)
        self.assertIn("--checkpoint-interval 50000", result.stdout)

        stage_commands = [
            shlex.split(line.strip())
            for line in result.stdout.splitlines()
            if line.strip().startswith("python3 ")
            and "train_loco_jump.py" in line
        ]
        self.assertEqual(len(stage_commands), 5, result.stdout)
        expected = [
            ("phase0", "5000"),
            ("phase1", "200000"),
            ("phase2", "500000"),
            ("phase3", "800000"),
            ("phase4", "1200000"),
        ]
        for argv, (tag, target) in zip(stage_commands, expected, strict=True):
            self.assertIn("--tag", argv)
            self.assertEqual(argv[argv.index("--tag") + 1], tag)
            self.assertEqual(argv[argv.index("--total-steps") + 1], target)
            self.assertEqual(argv[argv.index("--checkpoint-interval") + 1], "50000")
            cli_check = subprocess.run(
                [*argv, "--dry-run"],
                cwd=str(PROJECT_ROOT),
                text=True,
                capture_output=True,
                timeout=10,
                check=False,
            )
            self.assertEqual(cli_check.returncode, 0, cli_check.stderr)

    def test_monitor_default_and_watchdog_dry_run_are_read_only(self) -> None:
        """监控默认模式和 watchdog dry-run 均不得创建运行目录。"""
        with tempfile.TemporaryDirectory() as tmp:
            run_root = Path(tmp) / "logs"
            monitor_result = subprocess.run(
                [
                    sys.executable,
                    str(MONITOR_SCRIPT),
                    "--dry-run",
                    "--project-root",
                    str(Path(tmp)),
                    "--run-root",
                    str(run_root),
                ],
                text=True,
                capture_output=True,
                timeout=10,
                check=False,
            )
            self.assertEqual(monitor_result.returncode, 0, monitor_result.stderr)
            self.assertFalse(run_root.exists())
            watchdog_result = subprocess.run(
                [
                    "bash",
                    str(WATCHDOG_SCRIPT),
                    "dry-run",
                ],
                env={**os.environ, "LOCO_JUMP_RUN_ROOT": str(run_root)},
                text=True,
                capture_output=True,
                timeout=10,
                check=False,
            )
            self.assertEqual(watchdog_result.returncode, 0, watchdog_result.stderr)
            self.assertFalse(run_root.exists())

    def test_progress_and_fatal_log_patterns(self) -> None:
        """PID/阶段之外，步数、NaN/Inf、OOM、TCP 日志必须可识别。"""
        progress = MONITOR.parse_int_progress(
            "| rollout/ | ep_rew_mean=1.0 | total_timesteps | 20,480 | fps=50 |"
        )
        self.assertEqual(progress, 20480)
        samples = {
            "nan": "Value is nan while training",
            "inf": "reward became +inf",
            "oom": "torch.cuda.OutOfMemoryError: CUDA out of memory",
            "tcp": "TCP connection failed: connection refused",
        }
        for expected, sample in samples.items():
            hits = [
                name
                for name, pattern in MONITOR.FATAL_PATTERNS
                if pattern.search(sample)
            ]
            self.assertIn(expected, hits, f"{expected} 未命中：{sample}")

    def test_resource_threshold_once_is_read_only(self) -> None:
        """阈值越界能在只读巡检中报告，但 --once 不触发恢复。"""
        with tempfile.TemporaryDirectory() as tmp:
            project_root = Path(tmp) / "project"
            project_root.mkdir()
            run_root = Path(tmp) / "logs"
            result = subprocess.run(
                [
                    sys.executable,
                    str(MONITOR_SCRIPT),
                    "--once",
                    "--project-root",
                    str(project_root),
                    "--run-root",
                    str(run_root),
                    "--memory-threshold",
                    "0",
                    "--resource-persist-polls",
                    "1",
                    "--no-notify",
                ],
                text=True,
                capture_output=True,
                timeout=10,
                check=False,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            json_start = result.stdout.find("{")
            self.assertGreaterEqual(json_start, 0, result.stdout)
            payload = json.loads(result.stdout[json_start:])
            self.assertEqual(payload["resource_critical"], "memory")
            self.assertTrue((run_root / "status.json").is_file())
            self.assertEqual(list((run_root / "archive").glob("*")), [])

    def test_mock_process_exit_archive_validate_recover(self) -> None:
        """端到端演练：进程退出→日志归档→checkpoint 校验→恢复→完成。"""
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            project_root = root / "project"
            project_root.mkdir()
            run_root = root / "logs"
            checkpoint_dir = root / "checkpoints"
            command_file = run_root / "train_command.json"
            run_root.mkdir(parents=True)
            mock_script = root / "mock_train.py"
            write_mock_script(mock_script)
            command_file.write_text(
                json.dumps(
                    make_stage_config(project_root, mock_script, checkpoint_dir),
                    ensure_ascii=False,
                ),
                encoding="utf-8",
            )

            result = run_monitor(project_root, run_root, command_file)
            self.assertEqual(result.returncode, 0, result.stderr + result.stdout)

            state = json.loads((run_root / "state.json").read_text(encoding="utf-8"))
            status = json.loads((run_root / "status.json").read_text(encoding="utf-8"))
            checkpoint_manifest = json.loads(
                (run_root / "checkpoint_manifest.json").read_text(encoding="utf-8")
            )
            events = [
                json.loads(line)
                for line in (run_root / "events.log").read_text(encoding="utf-8").splitlines()
                if line.strip()
            ]
            event_names = [item["event"] for item in events]

            self.assertEqual(state["status"], "completed")
            self.assertEqual(state["recoveries"], 1)
            self.assertEqual(status["stage"]["id"], "S0_smoke")
            self.assertEqual(status["stage"]["cumulative_target_steps"], 1000)
            self.assertIn("archived", event_names)
            self.assertIn("recovery_started", event_names)
            self.assertIn("stage_completed", event_names)
            self.assertLess(
                event_names.index("archived"),
                event_names.index("recovery_started"),
                "必须先归档再恢复",
            )
            self.assertEqual(checkpoint_manifest["valid_count"], 1)
            self.assertTrue(checkpoint_manifest["best_resume_checkpoint"])
            archives = list((run_root / "archive").glob("*_process_exit"))
            self.assertEqual(len(archives), 1)
            self.assertTrue((archives[0] / "train.log").is_file())
            self.assertTrue((archives[0] / "metadata.json").is_file())
            self.assertIn("MOCK_PROCESS_FAILURE", (archives[0] / "train.log").read_text(encoding="utf-8"))
            self.assertIn("--resume", state["training_cmdline"])
            self.assertTrue((run_root / "phase.log").is_file())
            self.assertTrue((run_root / "pid.log").is_file())

    def test_three_failures_within_window_stop_after_three_recoveries(self) -> None:
        """默认最多 3 次恢复；第 4 次连续失败必须停止而不是无限重启。"""
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            project_root = root / "project"
            project_root.mkdir()
            run_root = root / "logs"
            checkpoint_dir = root / "checkpoints"
            command_file = run_root / "train_command.json"
            run_root.mkdir(parents=True)
            mock_script = root / "always_fail.py"
            write_always_fail_script(mock_script)
            command_file.write_text(
                json.dumps(
                    make_stage_config(
                        project_root,
                        mock_script,
                        checkpoint_dir,
                        mode="always_fail",
                    ),
                    ensure_ascii=False,
                ),
                encoding="utf-8",
            )
            result = run_monitor(
                project_root,
                run_root,
                command_file,
                ["--max-recoveries", "3", "--failure-window", "1800"],
            )
            self.assertEqual(result.returncode, 2, result.stderr + result.stdout)
            state = json.loads((run_root / "state.json").read_text(encoding="utf-8"))
            self.assertEqual(state["status"], "stopped")
            self.assertEqual(state["stop_reason"], "consecutive_failure_limit")
            self.assertEqual(state["recoveries"], 3)
            self.assertEqual(len(state["failure_history"]), 4)
            events = (run_root / "events.log").read_text(encoding="utf-8")
            self.assertIn("recovery_limit", events)

    def test_no_valid_checkpoint_blocks_recovery(self) -> None:
        """没有可验证 checkpoint 时必须停止，不能悄悄从头训练。"""
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            project_root = root / "project"
            project_root.mkdir()
            run_root = root / "logs"
            checkpoint_dir = root / "checkpoints"
            checkpoint_dir.mkdir()
            command_file = run_root / "train_command.json"
            run_root.mkdir(parents=True)
            mock_script = root / "no_checkpoint.py"
            mock_script.write_text(
                "import sys\nprint('NO_CHECKPOINT', flush=True)\nsys.exit(11)\n",
                encoding="utf-8",
            )
            command_file.write_text(
                json.dumps(
                    make_stage_config(project_root, mock_script, checkpoint_dir),
                    ensure_ascii=False,
                ),
                encoding="utf-8",
            )
            result = run_monitor(project_root, run_root, command_file)
            self.assertEqual(result.returncode, 2, result.stderr + result.stdout)
            state = json.loads((run_root / "state.json").read_text(encoding="utf-8"))
            self.assertEqual(state["stop_reason"], "no_valid_checkpoint")

    def test_progress_stall_recovers_from_valid_checkpoint(self) -> None:
        """进度停滞必须被单独记录，并经校验后恢复。"""
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            project_root = root / "project"
            project_root.mkdir()
            run_root = root / "logs"
            checkpoint_dir = root / "checkpoints"
            command_file = run_root / "train_command.json"
            run_root.mkdir(parents=True)
            mock_script = root / "stall_train.py"
            write_stall_script(mock_script)
            command_file.write_text(
                json.dumps(
                    make_stage_config(project_root, mock_script, checkpoint_dir),
                    ensure_ascii=False,
                ),
                encoding="utf-8",
            )
            result = run_monitor(
                project_root,
                run_root,
                command_file,
                ["--startup-grace", "0", "--stall-timeout", "0.1"],
            )
            self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
            events = (run_root / "events.log").read_text(encoding="utf-8")
            self.assertIn('"event": "progress_stalled"', events)
            self.assertIn('"event": "recovery_started"', events)
            state = json.loads((run_root / "state.json").read_text(encoding="utf-8"))
            self.assertEqual(state["status"], "completed")
            self.assertEqual(state["recoveries"], 1)

    def test_resource_guard_stops_without_recovery(self) -> None:
        """资源持续越界必须硬停止，不能用恢复掩盖根因。"""
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            project_root = root / "project"
            project_root.mkdir()
            run_root = root / "logs"
            checkpoint_dir = root / "checkpoints"
            command_file = run_root / "train_command.json"
            run_root.mkdir(parents=True)
            mock_script = root / "stall_train.py"
            write_stall_script(mock_script)
            command_file.write_text(
                json.dumps(
                    make_stage_config(project_root, mock_script, checkpoint_dir),
                    ensure_ascii=False,
                ),
                encoding="utf-8",
            )
            result = run_monitor(
                project_root,
                run_root,
                command_file,
                [
                    "--memory-threshold",
                    "0",
                    "--resource-persist-polls",
                    "1",
                ],
            )
            self.assertEqual(result.returncode, 2, result.stderr + result.stdout)
            state = json.loads((run_root / "state.json").read_text(encoding="utf-8"))
            self.assertEqual(state["status"], "stopped")
            self.assertEqual(state["stop_reason"], "resource_guard:memory")
            self.assertEqual(state["recoveries"], 0)
            events = (run_root / "events.log").read_text(encoding="utf-8")
            self.assertIn('"event": "resource_guard_stop"', events)
            self.assertNotIn('"event": "recovery_started"', events)


if __name__ == "__main__":
    unittest.main(verbosity=2)
