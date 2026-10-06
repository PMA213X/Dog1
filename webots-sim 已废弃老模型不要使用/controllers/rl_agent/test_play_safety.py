#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Webots play 安全层的独立单元测试。

测试不启动 Webots、不加载 checkpoint，也不写入源 world；导入 rl_agent 前
安装最小 controller 桩，确保审计环境无 Webots Python 包时也能执行。
"""

from __future__ import annotations

import contextlib
import io
import os
import re
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

import numpy as np


HERE = Path(__file__).resolve().parent
RL_DIR = HERE.parents[1] / "rl"
for path in (RL_DIR, HERE):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

controller_module = sys.modules.get("controller")
if controller_module is None or not hasattr(controller_module, "Supervisor"):
    controller_module = types.ModuleType("controller")
    controller_module.Supervisor = type("Supervisor", (), {})
    sys.modules["controller"] = controller_module

import play_safety  # noqa: E402
import rl_agent  # noqa: E402


class CheckpointResolutionTests(unittest.TestCase):
    """checkpoint 双候选解析与公共前缀校验。"""

    def test_absolute_path_expanduser_returns_existing_absolute_file(self) -> None:
        with tempfile.TemporaryDirectory(prefix="play-safety-home-") as tmp:
            fake_home = Path(tmp)
            target = (
                fake_home
                / "checkpoints"
                / "yobogo_loco_jump_v1"
                / "yobogo_loco_jump_v1_final.zip"
            )
            target.parent.mkdir(parents=True)
            target.write_bytes("占位 checkpoint".encode("utf-8"))
            relative_to_home = target.relative_to(fake_home).as_posix()

            with mock.patch.dict(os.environ, {"HOME": str(fake_home)}):
                resolved = play_safety.resolve_checkpoint_path(
                    f"~/{relative_to_home}"
                )

            self.assertTrue(resolved.is_absolute())
            self.assertEqual(resolved, target.resolve())
            self.assertTrue(resolved.is_file())

    @staticmethod
    def _synthetic_module_file(repo: Path) -> Path:
        return (
            repo
            / "webots-sim"
            / "controllers"
            / "rl_agent"
            / "rl_agent.py"
        )

    def test_relative_path_prefers_current_directory(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            current_dir = root / "cwd"
            repo = root / "repo"
            relative = Path("checkpoints/yobogo_loco_jump_v1_final.zip")
            current_file = current_dir / relative
            repo_file = repo / relative
            current_file.parent.mkdir(parents=True)
            repo_file.parent.mkdir(parents=True)
            current_file.write_bytes("当前目录".encode("utf-8"))
            repo_file.write_bytes("仓库根目录".encode("utf-8"))

            resolved = play_safety.resolve_checkpoint_path(
                relative,
                cwd=current_dir,
                module_file=self._synthetic_module_file(repo),
            )

            self.assertEqual(resolved, current_file.resolve())

    def test_relative_path_falls_back_to_repository_root(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            current_dir = root / "cwd"
            repo = root / "repo"
            relative = Path("checkpoints/yobogo_loco_jump_v1_final.zip")
            current_dir.mkdir()
            repo_file = repo / relative
            repo_file.parent.mkdir(parents=True)
            repo_file.write_bytes("仓库根目录".encode("utf-8"))

            resolved = play_safety.resolve_checkpoint_path(
                relative,
                cwd=current_dir,
                module_file=self._synthetic_module_file(repo),
            )

            self.assertEqual(resolved, repo_file.resolve())

    def test_missing_path_lists_both_absolute_candidates(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            current_dir = root / "cwd"
            repo = root / "repo"
            relative = Path("checkpoints/yobogo_loco_jump_v1_missing.zip")
            current_dir.mkdir()
            repo.mkdir()

            with self.assertRaises(FileNotFoundError) as caught:
                play_safety.resolve_checkpoint_path(
                    relative,
                    cwd=current_dir,
                    module_file=self._synthetic_module_file(repo),
                )

            message = str(caught.exception)
            self.assertIn(
                str((current_dir / relative).resolve()),
                message,
            )
            self.assertIn(
                str((repo / relative).resolve()),
                message,
            )
            self.assertIn("当前目录候选", message)
            self.assertIn("仓库根目录候选", message)

    def test_resolution_still_calls_contract_validator(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            current_dir = root / "cwd"
            repo = root / "repo"
            relative = Path("checkpoints/yobogo_loco_jump_v1_final.zip")
            repo_file = repo / relative
            repo_file.parent.mkdir(parents=True)
            repo_file.write_bytes("仓库根目录".encode("utf-8"))

            original = play_safety.validate_checkpoint_path
            with mock.patch.object(
                play_safety,
                "validate_checkpoint_path",
                wraps=original,
            ) as validator:
                resolved = play_safety.resolve_checkpoint_path(
                    relative,
                    cwd=current_dir,
                    module_file=self._synthetic_module_file(repo),
                )

            self.assertEqual(resolved, repo_file.resolve())
            self.assertGreaterEqual(validator.call_count, 1)

    def test_old_checkpoint_prefix_is_rejected_before_existence_lookup(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(ValueError) as caught:
                play_safety.resolve_checkpoint_path(
                    "ppo_walk_final.zip",
                    cwd=tmp,
                    module_file=self._synthetic_module_file(Path(tmp)),
                )
            self.assertIn("前缀不匹配", str(caught.exception))


class FakeStabilityTeleop:
    """稳定窗口输入读取计数器。"""

    def __init__(self) -> None:
        self.reads = 0

    def read(self):
        self.reads += 1
        return types.SimpleNamespace(
            velocity=(0.6, 0.3, 1.0),
            jump_edge=True,
            reset_edge=True,
            sprint=True,
        )


class FakeStabilityAgent:
    """仅提供稳定窗口所需的状态与动作记录。"""

    def __init__(
        self,
        contacts_by_cycle,
        height: float = 0.26,
    ) -> None:
        self.contacts_by_cycle = contacts_by_cycle
        self.height = float(height)
        self.control_cycles = 0
        self.applied_actions: list[np.ndarray] = []
        self.current_command = np.array([0.2, -0.1, 0.4])
        self.sprint_mode = True

    def apply_action(self, action: np.ndarray) -> None:
        self.applied_actions.append(np.asarray(action).copy())

    def step_control(self) -> None:
        self.control_cycles += 1

    def _read_contacts(self) -> np.ndarray:
        return np.asarray(
            self.contacts_by_cycle(self.control_cycles),
            dtype=np.float64,
        )

    def _read_base_height(self) -> float:
        return self.height


class StabilityWindowTests(unittest.TestCase):
    """50 周期最短窗口、连续接触、超时与异常离地。"""

    @staticmethod
    def _run(agent, teleop):
        lines: list[str] = []
        result = play_safety.wait_for_stable_contacts(
            agent,
            teleop,
            print_func=lambda message: lines.append(message),
        )
        return result, lines

    def test_minimum_window_consumes_edges_and_writes_only_zero_actions(self) -> None:
        teleop = FakeStabilityTeleop()
        agent = FakeStabilityAgent(
            lambda cycle: (
                [1, 1, 1, 1] if cycle >= 41 else [0, 0, 0, 0]
            ),
        )

        result, lines = self._run(agent, teleop)

        self.assertTrue(result.released)
        self.assertEqual(result.cycles, 50)
        self.assertEqual(result.contact_streak, 10)
        self.assertEqual(teleop.reads, 50)
        self.assertEqual(len(agent.applied_actions), 50)
        self.assertTrue(
            all(action.shape == (12,) for action in agent.applied_actions)
        )
        self.assertTrue(
            all(np.count_nonzero(action) == 0 for action in agent.applied_actions)
        )
        np.testing.assert_array_equal(agent.current_command, np.zeros(3))
        self.assertFalse(agent.sprint_mode)
        self.assertEqual(len(lines), 50)
        self.assertTrue(lines[0].startswith("【play-safety】稳定窗口 step=1/50"))
        self.assertTrue(
            lines[-1].startswith("【play-safety】稳定窗口 step=50/50")
        )

    def test_requires_one_unbroken_ten_cycle_contact_streak(self) -> None:
        def contacts(cycle: int):
            # 41–49 只有 9 个连续接触周期，第 50 周期中断；
            # 51–60 才首次形成连续 10 周期，因此最早在第 60 周期放行。
            if 41 <= cycle <= 49 or 51 <= cycle <= 60:
                return [1, 1, 1, 1]
            return [1, 1, 1, 0]

        teleop = FakeStabilityTeleop()
        agent = FakeStabilityAgent(contacts)

        result, lines = self._run(agent, teleop)

        self.assertTrue(result.released)
        self.assertEqual(result.cycles, 60)
        self.assertEqual(result.contact_streak, 10)
        self.assertEqual(teleop.reads, 60)
        self.assertEqual(len(agent.applied_actions), 60)
        self.assertTrue(
            all(np.count_nonzero(action) == 0 for action in agent.applied_actions)
        )
        self.assertTrue(
            lines[50].startswith("【play-safety】稳定等待延长 step=51/100")
        )

    def test_timeout_at_100_stops_with_fixed_error(self) -> None:
        teleop = FakeStabilityTeleop()
        agent = FakeStabilityAgent(lambda _cycle: [1, 1, 1, 0])

        result, lines = self._run(agent, teleop)

        self.assertFalse(result.released)
        self.assertEqual(result.reason, "timeout")
        self.assertEqual(result.cycles, 100)
        self.assertEqual(teleop.reads, 100)
        self.assertEqual(len(agent.applied_actions), 100)
        self.assertTrue(
            all(np.count_nonzero(action) == 0 for action in agent.applied_actions)
        )
        self.assertEqual(
            lines[-1],
            "【play-safety】稳定接触超时："
            "100 个控制周期内未满足连续 10 周期四足接触，停止 play",
        )

    def test_abnormal_height_stops_before_writing_action(self) -> None:
        teleop = FakeStabilityTeleop()
        agent = FakeStabilityAgent(
            lambda _cycle: [1, 1, 1, 1],
            height=0.601,
        )

        result, lines = self._run(agent, teleop)

        self.assertFalse(result.released)
        self.assertEqual(result.reason, "abnormal_height")
        self.assertEqual(teleop.reads, 1)
        self.assertEqual(agent.applied_actions, [])
        self.assertEqual(
            lines[-1],
            "【play-safety】异常离地：height=0.601 > 0.600m，停止 play",
        )


class PlayActionGuardTests(unittest.TestCase):
    """动作绝对裁剪、变化率、重置与前 10 周期诊断窗口。"""

    def test_absolute_clip_and_single_cycle_rate_limit(self) -> None:
        guard = play_safety.PlayActionGuard(action_dim=12)

        saturated = guard.filter_action(np.full(12, 9.0))
        np.testing.assert_allclose(saturated, np.full(12, 0.05))
        reversed_action = guard.filter_action(np.full(12, -9.0))
        np.testing.assert_allclose(reversed_action, np.full(12, 0.0))

        self.assertLessEqual(np.max(np.abs(saturated)), 0.5)
        self.assertLessEqual(
            float(np.max(np.abs(reversed_action - saturated))),
            0.05 + 1e-12,
        )

        # 已到 +0.5 边界时，超大正动作仍必须被绝对裁剪在 ±0.5 内。
        guard._previous_action = np.full(12, 0.5)
        clipped = guard.filter_action(np.full(12, 100.0))
        np.testing.assert_allclose(clipped, np.full(12, 0.5))

    def test_reset_clears_history_and_report_window(self) -> None:
        guard = play_safety.PlayActionGuard(action_dim=12)
        reports = []
        for _ in range(11):
            guard.filter_action(np.ones(12))
            reports.append(guard.should_report)
        self.assertEqual(reports[:10], [True] * 10)
        self.assertFalse(reports[10])

        guard.reset()
        np.testing.assert_array_equal(guard.previous_action, np.zeros(12))
        self.assertEqual(guard.released_cycles, 0)
        first_after_reset = guard.filter_action(np.ones(12))
        np.testing.assert_allclose(first_after_reset, np.full(12, 0.05))
        self.assertTrue(guard.should_report)

    def test_rejects_wrong_shape_and_non_finite_action(self) -> None:
        guard = play_safety.PlayActionGuard(action_dim=12)
        with self.assertRaises(ValueError):
            guard.filter_action(np.zeros(11))
        invalid = np.zeros(12)
        invalid[3] = np.nan
        with self.assertRaises(ValueError):
            guard.filter_action(invalid)


class FakeKeyboard:
    """记录排空、启停状态的 Webots Keyboard 替身。"""

    def __init__(
        self,
        events: list[int] | None = None,
        *,
        event_log: list[str] | None = None,
    ) -> None:
        self.events = list(events or [])
        self.enabled = False
        self.disabled = False
        self.event_log = event_log if event_log is not None else []

    def enable(self, _timestep: int) -> None:
        self.enabled = True
        self.event_log.append("enable")

    def getKey(self) -> int:
        self.event_log.append("drain")
        return self.events.pop(0) if self.events else -1

    def disable(self) -> None:
        self.disabled = True


class FakeRuntimeRobot:
    """只提供 play 集成测试所需的方法。"""

    def __init__(self, keyboard: FakeKeyboard) -> None:
        self.keyboard = keyboard
        self.quit_codes: list[int] = []

    def getKeyboard(self) -> FakeKeyboard:
        return self.keyboard

    def getTime(self) -> float:
        return 0.0

    def simulationQuit(self, code: int) -> None:
        self.quit_codes.append(code)


class FakeTeleop:
    """按预设返回 play 命令并记录关闭状态。"""

    def __init__(self, commands: list) -> None:
        self.commands = list(commands)
        self.closed = False

    def read(self):
        if self.commands:
            return self.commands.pop(0)
        return types.SimpleNamespace(
            velocity=(0.0, 0.0, 0.0),
            jump_edge=False,
            reset_edge=False,
            estop_edge=False,
            sprint=False,
        )

    def close(self) -> None:
        self.closed = True


def command(
    *,
    reset_edge: bool = False,
    jump_edge: bool = False,
) -> types.SimpleNamespace:
    """构造最小遥控命令对象。"""
    return types.SimpleNamespace(
        velocity=(0.0, 0.0, 0.0),
        jump_edge=jump_edge,
        reset_edge=reset_edge,
        estop_edge=False,
        sprint=False,
    )


class RunPlayIntegrationTests(unittest.TestCase):
    """初始化顺序、两类 reset 排空、固定日志和异常离地停止。"""

    @staticmethod
    def _released_result() -> play_safety.StabilityResult:
        return play_safety.StabilityResult(
            released=True,
            reason="released",
            cycles=50,
            contact_streak=10,
        )

    def test_initialization_order_and_user_reset_drain(self) -> None:
        events: list[str] = []
        keyboard = FakeKeyboard([ord("W"), -1], event_log=events)
        robot = FakeRuntimeRobot(keyboard)
        teleop = FakeTeleop(
            [
                command(reset_edge=True),
                command(),
            ]
        )
        checkpoint = Path("/tmp/play-safety/yobogo_loco_jump_v1_final.zip")

        agent = types.SimpleNamespace(
            robot=robot,
            timestep=4,
            current_command=np.ones(3),
            sprint_mode=True,
            jump_latched=False,
        )

        def load_model():
            events.append("load")
            return object(), checkpoint

        def reset():
            events.append("reset")
            # 用户 R 复位后，下一次直接键盘排空应消费该 R。
            if len([item for item in events if item == "reset"]) == 2:
                keyboard.events.extend([ord("R"), -1])

        def make_teleop(_keyboard):
            events.append("teleop")
            return teleop

        def wait_stable(_agent, _teleop):
            events.append("stable")
            return self._released_result()

        agent.reset = reset
        agent.read_state = lambda: {
            "height": 0.601,
            "contacts": [1, 1, 1, 1],
        }
        agent.state_to_obs = lambda _state: np.zeros(rl_agent.OBS_DIM)
        agent.request_jump = lambda: setattr(agent, "jump_latched", True)

        output = io.StringIO()
        with (
            mock.patch.object(rl_agent, "_load_play_model", side_effect=load_model),
            mock.patch.object(rl_agent, "TeleopReader", side_effect=make_teleop),
            mock.patch.object(
                rl_agent,
                "wait_for_stable_contacts",
                side_effect=wait_stable,
            ),
            contextlib.redirect_stdout(output),
        ):
            rl_agent._run_play(agent)

        self.assertEqual(
            events,
            [
                "load",
                "enable",
                "reset",
                "drain",
                "drain",
                "teleop",
                "stable",
                "reset",
                "drain",
                "drain",
                "stable",
            ],
        )
        self.assertIn(
            "【rl_agent】play 模式："
            "checkpoint=/tmp/play-safety/yobogo_loco_jump_v1_final.zip "
            "rate=50Hz device=cpu",
            output.getvalue(),
        )
        self.assertIn(
            "【play-safety】异常离地："
            "height=0.601 > 0.600m，停止 play",
            output.getvalue(),
        )
        self.assertTrue(keyboard.disabled)
        self.assertTrue(teleop.closed)
        self.assertEqual(robot.quit_codes, [0])

    def test_internal_episode_reset_drains_keyboard(self) -> None:
        events: list[str] = []
        keyboard = FakeKeyboard(event_log=events)
        robot = FakeRuntimeRobot(keyboard)
        teleop = FakeTeleop([command(), command()])
        checkpoint = Path("/tmp/play-safety/yobogo_loco_jump_v1_final.zip")
        state_reads = 0

        agent = types.SimpleNamespace(
            robot=robot,
            timestep=4,
            current_command=np.zeros(3),
            sprint_mode=False,
            jump_latched=False,
            _jump_phase_time=0.0,
        )

        def load_model():
            events.append("load")
            return FakeModel(events), checkpoint

        def reset():
            events.append("reset")

        def read_state():
            nonlocal state_reads
            state_reads += 1
            events.append("state")
            height = 0.26 if state_reads == 1 else 0.601
            return {
                "height": height,
                "contacts": [1, 1, 1, 1],
            }

        def apply_action(_action):
            events.append("apply")

        def wait_stable(_agent, _teleop):
            events.append("stable")
            return self._released_result()

        agent.reset = reset
        agent.read_state = read_state
        agent.state_to_obs = lambda _state: np.zeros(rl_agent.OBS_DIM)
        agent.apply_action = apply_action
        agent.step_control = lambda: None
        agent._read_base_height = lambda: 0.26
        agent._update_jump_latch = lambda _contacts: None
        agent.is_fallen = lambda: False
        agent.request_jump = lambda: None
        agent._read_contacts = lambda: np.ones(4, dtype=np.float64)

        output = io.StringIO()
        with (
            mock.patch.object(rl_agent, "_load_play_model", side_effect=load_model),
            mock.patch.object(rl_agent, "TeleopReader", return_value=teleop),
            mock.patch.object(
                rl_agent,
                "wait_for_stable_contacts",
                side_effect=wait_stable,
            ),
            mock.patch.object(rl_agent, "PLAY_MAX_STEPS", 1),
            contextlib.redirect_stdout(output),
        ):
            rl_agent._run_play(agent)

        self.assertEqual(
            events,
            [
                "load",
                "enable",
                "reset",
                "drain",
                "stable",
                "state",
                "predict",
                "apply",
                "reset",
                "drain",
                "stable",
                "state",
            ],
        )
        self.assertTrue(keyboard.disabled)
        self.assertTrue(teleop.closed)
        self.assertEqual(robot.quit_codes, [0])
        self.assertIn("【play-safety】raw=", output.getvalue())
        self.assertIn(
            "【play-safety】异常离地："
            "height=0.601 > 0.600m，停止 play",
            output.getvalue(),
        )


class FakeModel:
    """只记录确定性预测调用的模型替身。"""

    def __init__(self, events: list[str]) -> None:
        self.events = events

    def predict(self, _obs, deterministic: bool):
        self.events.append("predict")
        self.assert_deterministic = deterministic
        return np.zeros(rl_agent.ACTION_DIM), None


class SourceScopeTests(unittest.TestCase):
    """验证安全层只接入 play，并只读检查 world 的 Viewpoint 数量。"""

    def test_safety_entrypoints_are_not_used_by_tcp_or_smoke(self) -> None:
        source = Path(rl_agent.__file__).read_text(encoding="utf-8")
        tcp_start = source.index("def serve_tcp(")
        play_start = source.index(
            "# play：本机键盘/手柄 + checkpoint 确定性推理"
        )
        smoke_start = source.index("# 冒烟：作为 Webots 控制器运行时")
        tcp_and_train = source[tcp_start:play_start] + source[smoke_start:]
        self.assertNotIn("PlayActionGuard", tcp_and_train)
        self.assertNotIn("wait_for_stable_contacts", tcp_and_train)
        self.assertNotIn("resolve_checkpoint_path", tcp_and_train)

    def test_parkour_dev_world_has_exactly_one_viewpoint(self) -> None:
        world = HERE.parents[1] / "worlds" / "parkour_dev.wbt"
        source = world.read_text(encoding="utf-8")
        pattern = re.compile(
            r"^[ \t]*(?:DEF[ \t]+[A-Za-z0-9_]+[ \t]+)?"
            r"Viewpoint[ \t]*\{",
            re.MULTILINE,
        )
        self.assertEqual(len(pattern.findall(source)), 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
