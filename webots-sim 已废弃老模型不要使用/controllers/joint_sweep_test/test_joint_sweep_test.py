#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""关节扫掠控制器的离线单元测试。

测试只使用假电机和假位置传感器，不启动 Webots，不加载 RL 模型，也不
访问 checkpoint 或网络。
"""

from __future__ import annotations

import ast
import re
import unittest
from pathlib import Path

import joint_sweep_test as sweep


HERE = Path(__file__).resolve().parent
CONTROLLER_PATH = HERE / "joint_sweep_test.py"

MIN_POSITION = -0.10
MAX_POSITION = 0.10
TIME_STEP_MS = 100
TIME_STEP_SECONDS = TIME_STEP_MS / 1000.0


class FakePositionSensor:
    """可选择跟随电机目标的假位置传感器。"""

    def __init__(self, value: float, *, follows_motor: bool = True) -> None:
        self.value = float(value)
        self.follows_motor = follows_motor
        self.enabled_at_ms: int | None = None

    def enable(self, time_step_ms: int) -> None:
        self.enabled_at_ms = int(time_step_ms)

    def getValue(self) -> float:
        return self.value


class FakeMotor:
    """记录全部目标角的假电机。"""

    def __init__(
        self,
        sensor: FakePositionSensor,
        minimum: float = MIN_POSITION,
        maximum: float = MAX_POSITION,
    ) -> None:
        self.sensor = sensor
        self.minimum = float(minimum)
        self.maximum = float(maximum)
        self.targets: list[float] = []

    def getMinPosition(self) -> float:
        return self.minimum

    def getMaxPosition(self) -> float:
        return self.maximum

    def setPosition(self, target: float) -> None:
        target = float(target)
        self.targets.append(target)
        if self.sensor.follows_motor:
            self.sensor.value = target


class FakeKeyboard:
    """仅提供 Webots Keyboard 所需接口的假键盘。"""

    def __init__(self) -> None:
        self.enabled_at_ms: int | None = None
        self.key = -1

    def enable(self, time_step_ms: int) -> None:
        self.enabled_at_ms = int(time_step_ms)

    def getKey(self) -> int:
        key, self.key = self.key, -1
        return key


class FakeRobot:
    """提供控制器构造所需设备查询的假 Robot。"""

    def __init__(
        self,
        *,
        initial_positions: dict[str, float] | None = None,
        follows_motor: dict[str, bool] | None = None,
        minimum: float = MIN_POSITION,
        maximum: float = MAX_POSITION,
    ) -> None:
        initial_positions = initial_positions or {}
        follows_motor = follows_motor or {}
        self.time_step_ms = TIME_STEP_MS
        self.simulation_time = 0.0
        self.keyboard = FakeKeyboard()
        self.devices: dict[str, object] = {}

        for motor_name, sensor_name in zip(
            sweep.MOTOR_NAMES,
            sweep.SENSOR_NAMES,
        ):
            sensor = FakePositionSensor(
                initial_positions.get(motor_name, 0.0),
                follows_motor=follows_motor.get(motor_name, True),
            )
            motor = FakeMotor(sensor, minimum=minimum, maximum=maximum)
            self.devices[motor_name] = motor
            self.devices[sensor_name] = sensor

    def getBasicTimeStep(self) -> int:
        return self.time_step_ms

    def getDevice(self, name: str):
        return self.devices.get(name)

    def getKeyboard(self) -> FakeKeyboard:
        return self.keyboard

    def step(self, time_step_ms: int) -> int:
        self.simulation_time += float(time_step_ms) / 1000.0
        return 0

    def getTime(self) -> float:
        return self.simulation_time

    def motor(self, name: str) -> FakeMotor:
        device = self.devices[name]
        assert isinstance(device, FakeMotor)
        return device

    def sensor(self, name: str) -> FakePositionSensor:
        if name.endswith("_motor"):
            name = name.removesuffix("_motor") + "_sensor"
        device = self.devices[name]
        assert isinstance(device, FakePositionSensor)
        return device


class ControllerHarness:
    """构造控制器并提供确定性仿真推进。"""

    def __init__(self, **robot_options) -> None:
        self.logs: list[str] = []
        self.robot = FakeRobot(**robot_options)
        self.controller = sweep.JointSweepController(
            self.robot,
            self.robot.keyboard,
            time_step_ms=TIME_STEP_MS,
            output=self.logs.append,
        )
        self.now = 0.0

    def tick(self, *, key=None) -> str:
        return self.controller.tick(self.now, key)

    def advance(self, seconds: float, *, key=None) -> str:
        self.now += float(seconds)
        return self.controller.tick(self.now, key)

    def run_to_completion(self, *, maximum_ticks: int = 5000) -> str:
        status = self.tick()
        for _ in range(maximum_ticks):
            if status != sweep.STATUS_RUNNING:
                return status
            status = self.advance(TIME_STEP_SECONDS)
        self.fail_if_needed("控制器未在最大步数内完成")
        return status

    def fail_if_needed(self, message: str) -> None:
        raise AssertionError(message)


def stage_lines(logs: list[str], stage: str, joint_name: str) -> list[str]:
    """筛选指定关节、指定阶段的日志。"""

    prefix = f"[{stage}] 关节={joint_name} "
    return [line for line in logs if line.startswith(prefix)]


def log_value(line: str, field: str) -> float:
    """读取日志行中的数值字段。"""

    match = re.search(rf"{field}=(-?\d+(?:\.\d+)?)", line)
    if match is None:
        raise AssertionError(f"日志缺少字段 {field}: {line}")
    return float(match.group(1))


class DefinitionTests(unittest.TestCase):
    """固定顺序、路径和速度常量。"""

    def test_joint_order_is_fr_fl_hr_hl_and_abd_hip_kn(self) -> None:
        self.assertEqual(sweep.LEG_ORDER, ("fr", "fl", "hr", "hl"))
        self.assertEqual(sweep.JOINT_ORDER, ("abd", "hip", "kn"))
        self.assertEqual(
            sweep.MOTOR_NAMES,
            (
                "fr_abd_motor",
                "fr_hip_motor",
                "fr_kn_motor",
                "fl_abd_motor",
                "fl_hip_motor",
                "fl_kn_motor",
                "hr_abd_motor",
                "hr_hip_motor",
                "hr_kn_motor",
                "hl_abd_motor",
                "hl_hip_motor",
                "hl_kn_motor",
            ),
        )
        self.assertEqual(
            sweep.SENSOR_NAMES,
            tuple(name.replace("_motor", "_sensor") for name in sweep.MOTOR_NAMES),
        )

    def test_path_and_phase_durations_match_approved_plan(self) -> None:
        self.assertEqual(
            sweep.build_joint_path(MIN_POSITION, MAX_POSITION),
            (0.0, MIN_POSITION, 0.0, MAX_POSITION, 0.0),
        )
        phases = sweep.build_motion_phases(MIN_POSITION, MAX_POSITION)
        self.assertEqual(len(phases), 4)
        self.assertEqual(
            [(phase.start, phase.end) for phase in phases],
            [
                (0.0, MIN_POSITION),
                (MIN_POSITION, 0.0),
                (0.0, MAX_POSITION),
                (MAX_POSITION, 0.0),
            ],
        )
        self.assertEqual(
            [phase.duration_seconds for phase in phases],
            [0.5, 0.5, 0.5, 0.5],
        )
        self.assertEqual(
            [phase.hold_endpoint for phase in phases],
            [True, False, True, False],
        )
        self.assertEqual(sweep.SWEEP_SPEED_RAD_S, 0.20)
        self.assertEqual(sweep.ENDPOINT_HOLD_SECONDS, 0.5)
        self.assertEqual(sweep.POSITION_ERROR_LIMIT_RAD, 0.05)
        self.assertEqual(sweep.POSITION_ERROR_TIMEOUT_SECONDS, 10.0)

    def test_controller_source_has_no_forbidden_runtime_dependencies(self) -> None:
        tree = ast.parse(CONTROLLER_PATH.read_text(encoding="utf-8"))
        imported_modules: set[str] = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported_modules.update(
                    alias.name.split(".", 1)[0] for alias in node.names
                )
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported_modules.add(node.module.split(".", 1)[0])

        forbidden = {
            "stable_baselines3",
            "numpy",
            "socket",
            "loco_jump_contract",
            "play_safety",
            "rl_agent",
        }
        self.assertTrue(forbidden.isdisjoint(imported_modules))


class SweepSequenceTests(unittest.TestCase):
    """完整扫掠的顺序、路径、日志、端点保持和回零。"""

    @classmethod
    def setUpClass(cls) -> None:
        cls.harness = ControllerHarness()
        cls.status = cls.harness.run_to_completion()
        cls.logs = cls.harness.logs

    def test_all_twelve_joints_are_covered_in_approved_order(self) -> None:
        started_names = []
        for line in self.logs:
            match = re.match(r"\[开始] 关节=(\S+)", line)
            if match is not None and match.group(1) != "全部回零":
                started_names.append(match.group(1))

        expected = [
            name
            for name in sweep.MOTOR_NAMES
            for _ in range(4)
        ]
        self.assertEqual(started_names, expected)
        for name in sweep.MOTOR_NAMES:
            self.assertEqual(len(stage_lines(self.logs, "开始", name)), 4)
            self.assertEqual(len(stage_lines(self.logs, "到位", name)), 4)
            self.assertEqual(len(stage_lines(self.logs, "结束", name)), 4)

    def test_each_joint_follows_zero_min_zero_max_zero(self) -> None:
        for name in sweep.MOTOR_NAMES:
            starts = [
                log_value(line, "目标")
                for line in stage_lines(self.logs, "开始", name)
            ]
            ends = [
                log_value(line, "目标")
                for line in stage_lines(self.logs, "结束", name)
            ]
            self.assertEqual(
                starts,
                [0.0, MIN_POSITION, 0.0, MAX_POSITION],
                name,
            )
            self.assertEqual(
                ends,
                [MIN_POSITION, 0.0, MAX_POSITION, 0.0],
                name,
            )
            arrivals = stage_lines(self.logs, "到位", name)
            self.assertEqual(
                [log_value(line, "目标") for line in arrivals],
                [MIN_POSITION, 0.0, MAX_POSITION, 0.0],
                name,
            )
            self.assertTrue(
                all("目标=" in line and "实际=" in line for line in arrivals)
            )

    def test_command_rate_never_exceeds_approved_speed(self) -> None:
        for name in sweep.MOTOR_NAMES:
            targets = self.harness.robot.motor(name).targets
            differences = [
                abs(current - previous)
                for previous, current in zip(targets, targets[1:])
            ]
            maximum_step = sweep.SWEEP_SPEED_RAD_S * TIME_STEP_SECONDS + 1e-9
            self.assertLessEqual(max(differences), maximum_step, name)

    def test_min_and_max_are_each_held_for_half_second(self) -> None:
        for name in sweep.MOTOR_NAMES:
            hold_starts = stage_lines(self.logs, "端点保持开始", name)
            hold_ends = stage_lines(self.logs, "端点保持结束", name)
            self.assertEqual(len(hold_starts), 2, name)
            self.assertEqual(len(hold_ends), 2, name)
            self.assertEqual(
                [log_value(line, "目标") for line in hold_starts],
                [MIN_POSITION, MAX_POSITION],
                name,
            )
            for start_line, end_line in zip(hold_starts, hold_ends):
                duration = (
                    log_value(end_line, "时间")
                    - log_value(start_line, "时间")
                )
                self.assertAlmostEqual(duration, 0.5, places=9, msg=name)

    def test_completion_returns_all_twelve_motors_to_zero_and_prints(self) -> None:
        self.assertEqual(self.status, sweep.STATUS_COMPLETED)
        for name in sweep.MOTOR_NAMES:
            self.assertEqual(self.harness.robot.motor(name).targets[-1], 0.0)
            self.assertEqual(self.harness.robot.sensor(name).value, 0.0)
        self.assertTrue(
            any(
                "关节扫掠完成，已全部回零" in line
                for line in self.logs
            )
        )
        zero_starts = stage_lines(self.logs, "开始", "全部回零")
        zero_arrivals = stage_lines(self.logs, "到位", "全部回零")
        zero_ends = stage_lines(self.logs, "结束", "全部回零")
        self.assertEqual(len(zero_starts), 1)
        self.assertEqual(len(zero_arrivals), 1)
        self.assertEqual(len(zero_ends), 1)
        for line in zero_starts + zero_arrivals + zero_ends:
            self.assertIn("目标=", line)
            self.assertIn("实际=", line)


class HoldAndStopTests(unittest.TestCase):
    """非当前关节保持、位置误差超时和 Esc/R 停止保持。"""

    def test_non_current_joints_keep_initial_actual_positions(self) -> None:
        initial = {
            "fr_abd_motor": 0.0,
            "fr_hip_motor": 0.07,
            "fr_kn_motor": -0.08,
            "fl_abd_motor": 0.06,
            "fl_hip_motor": -0.07,
            "fl_kn_motor": 0.05,
        }
        harness = ControllerHarness(initial_positions=initial)
        harness.tick()
        harness.advance(0.4)

        for name, expected in initial.items():
            if name == "fr_abd_motor":
                continue
            motor = harness.robot.motor(name)
            self.assertEqual(motor.targets, [expected], name)
            self.assertEqual(harness.robot.sensor(name).value, expected, name)

    def test_error_over_threshold_for_ten_seconds_stops_and_holds(self) -> None:
        stuck_name = sweep.MOTOR_NAMES[0]
        harness = ControllerHarness(
            follows_motor={stuck_name: False},
        )
        harness.tick()
        error_started = None
        status = sweep.STATUS_RUNNING
        for step in range(1, 130):
            now = step * TIME_STEP_SECONDS
            status = harness.advance(TIME_STEP_SECONDS)
            active_target = harness.robot.motor(stuck_name).targets[-1]
            active_actual = harness.robot.sensor(stuck_name).value
            if (
                error_started is None
                and abs(active_target - active_actual)
                > sweep.POSITION_ERROR_LIMIT_RAD
            ):
                error_started = now
            if status != sweep.STATUS_RUNNING:
                break

        self.assertEqual(status, sweep.STATUS_TIMEOUT)
        self.assertIsNotNone(error_started)
        self.assertAlmostEqual(
            harness.now - error_started,
            sweep.POSITION_ERROR_TIMEOUT_SECONDS,
            places=9,
        )
        self.assertEqual(
            harness.controller.stop_reason,
            "位置误差超时",
        )
        self.assertIsNotNone(harness.controller.hold_positions)
        self.assertTrue(
            any(
                "[停止并保持]" in line and "原因=位置误差超时" in line
                for line in harness.logs
            )
        )

        lengths = {
            name: len(harness.robot.motor(name).targets)
            for name in sweep.MOTOR_NAMES
        }
        self.assertEqual(harness.advance(TIME_STEP_SECONDS), status)
        self.assertEqual(
            lengths,
            {
                name: len(harness.robot.motor(name).targets)
                for name in sweep.MOTOR_NAMES
            },
        )

    def test_escape_immediately_stops_and_holds_current_positions(self) -> None:
        self._assert_stop_key(sweep.ESCAPE_KEY, "Esc/R 停止")

    def test_uppercase_r_immediately_stops_and_holds_current_positions(self) -> None:
        self._assert_stop_key(ord("R"), "Esc/R 停止")

    def test_lowercase_r_is_also_accepted(self) -> None:
        self._assert_stop_key(ord("r"), "Esc/R 停止")

    def _assert_stop_key(self, key: int, expected_reason: str) -> None:
        harness = ControllerHarness()
        harness.tick()
        harness.advance(0.2)
        status = harness.advance(0.1, key=key)

        self.assertEqual(status, sweep.STATUS_STOP_KEY)
        self.assertEqual(harness.controller.stop_reason, expected_reason)
        self.assertTrue(
            any(
                "[停止并保持]" in line and f"原因={expected_reason}" in line
                for line in harness.logs
            )
        )
        lengths = {
            name: len(harness.robot.motor(name).targets)
            for name in sweep.MOTOR_NAMES
        }
        self.assertEqual(harness.advance(0.1), status)
        self.assertEqual(
            lengths,
            {
                name: len(harness.robot.motor(name).targets)
                for name in sweep.MOTOR_NAMES
            },
        )
        for name in sweep.MOTOR_NAMES:
            self.assertEqual(
                harness.robot.motor(name).targets[-1],
                harness.robot.sensor(name).value,
                name,
            )


if __name__ == "__main__":
    unittest.main()
