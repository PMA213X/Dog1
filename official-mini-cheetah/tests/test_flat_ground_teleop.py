"""官方 Mini Cheetah 主动控制的独立静态与纯模块测试。"""

from __future__ import annotations

import importlib.abc
import math
import re
import subprocess
import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONTROLLER_DIR = PROJECT_ROOT / "controllers/flat_ground_teleop"
if str(CONTROLLER_DIR) not in sys.path:
    sys.path.insert(0, str(CONTROLLER_DIR))

from flat_ground_teleop import FlatGroundTeleopController  # noqa: E402
from joint_safety import (  # noqa: E402
    FLAT_MOTOR_NAMES,
    FLAT_SENSOR_NAMES,
    JOINT_COUNT,
    KD,
    KP,
    MAX_TORQUE,
    TARGET_POSITION_LIMITS,
    MiniCheetahGait,
    WEBOTS_MOTOR_MAX_TORQUE,
    compute_pd_torques,
    validate_joint_feedback,
)
from teleop_input import (  # noqa: E402
    KEY_A,
    KEY_D,
    KEY_E,
    KEY_ESCAPE,
    KEY_Q,
    KEY_R,
    KEY_S,
    KEY_SPACE,
    KEY_W,
    WebotsJoystickAdapter,
    merge_commands,
)
from teleop_state import TeleopStateMachine  # noqa: E402
from teleop_types import ControlMode, TeleopCommand  # noqa: E402


ACTIVE_WORLD = PROJECT_ROOT / "worlds/flat_ground_teleop.wbt"
PASSIVE_WORLD = PROJECT_ROOT / "worlds/official_flat_ground_test.wbt"
ACTIVE_SOURCE = CONTROLLER_DIR / "flat_ground_teleop.py"


def _robot_block(text: str) -> str:
    """提取 name=mini_cheetah 的主 Robot 节点。"""

    match = re.search(r"name\s+\"mini_cheetah\"", text)
    if match is None:
        raise AssertionError("缺少 mini_cheetah Robot")
    start = text.rfind("Robot {", 0, match.start())
    if start < 0:
        raise AssertionError("mini_cheetah 不在 Robot 节点内")
    depth = 0
    in_string = False
    escaped = False
    for index in range(start, len(text)):
        char = text[index]
        if in_string:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                in_string = False
        elif char == '"':
            in_string = True
        elif char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return text[start : index + 1]
    raise AssertionError("Robot 节点未闭合")


def _model_tokens(robot: str) -> tuple[str, ...]:
    """移除 controller 与姿态控制专用关节弹簧字段后比较官方模型。"""

    clean = re.sub(r"controller\s+\"[^\"]*\"", "", robot)
    clean = re.sub(r"controllerArgs\s*\[[^\]]*\]", "", clean)
    clean = re.sub(r"(?m)^\s*springConstant\s+[^\n#]+", "", clean)
    clean = re.sub(r"translation 0 0 0\.40", "", clean)
    clean = re.sub(r"translation 0 0 0\.45", "", clean)
    return tuple(
        re.findall(
            r"\"(?:\\.|[^\"\\])*\"|[A-Za-z_][A-Za-z0-9_]*|"
            r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?|[{}\[\]]",
            clean,
        )
    )


class _Motor:
    def __init__(self) -> None:
        self.positions: list[float] = []
        self.torques: list[float] = []

    def setPosition(self, value: float) -> None:
        self.positions.append(float(value))

    def setTorque(self, value: float) -> None:
        self.torques.append(float(value))


class _Sensor:
    def __init__(self, value: float = 0.0) -> None:
        self.value = float(value)
        self.enabled: list[int] = []

    def enable(self, timestep: int) -> None:
        self.enabled.append(int(timestep))

    def getValue(self) -> float:
        return self.value


class _Keyboard:
    def __init__(self, events: list[int]) -> None:
        self.events = list(events)

    def getKey(self) -> int:
        return self.events.pop(0) if self.events else -1


class _Joystick:
    def __init__(self, axes: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
        self.axes = list(axes)
        self.connected = True
        self.button = -1

    def isConnected(self) -> bool:
        return self.connected

    def getAxisValue(self, index: int) -> float:
        return self.axes[index]

    def getPressedButton(self) -> int:
        return self.button


class _Robot:
    def __init__(self) -> None:
        self.time = 0.0
        self.motors = {name: _Motor() for name in FLAT_MOTOR_NAMES}
        self.sensors = {name: _Sensor() for name in FLAT_SENSOR_NAMES}
        self.keyboard = _Keyboard([])
        self.joystick = _Joystick()

    def getBasicTimeStep(self) -> int:
        return 4

    def getTime(self) -> float:
        return self.time

    def getDevice(self, name: str):
        return self.motors.get(name, self.sensors.get(name))

    def getKeyboard(self):
        return self.keyboard

    def getJoystick(self):
        return self.joystick


class WorldIsolationTests(unittest.TestCase):
    """主动 world、被动 world 与模型不变性。"""

    def test_worlds_are_isolated(self) -> None:
        active = ACTIVE_WORLD.read_text(encoding="utf-8")
        passive = PASSIVE_WORLD.read_text(encoding="utf-8")
        self.assertIn('controller "flat_ground_teleop"', active)
        self.assertNotIn("acceptance_supervisor", active)
        self.assertNotIn("supervisor", active)
        self.assertNotIn("flat_ground_teleop", passive)
        self.assertIn("acceptance_supervisor", passive)
        self.assertIn("supervisor TRUE", passive)
        source = ACTIVE_SOURCE.read_text(encoding="utf-8")
        self.assertNotIn("acceptance_supervisor", source)
        self.assertNotIn("acceptance_summary", source)
        self.assertNotIn("acceptance_samples", source)

    def test_active_and_passive_have_complete_12_axis_models(self) -> None:
        for path in (ACTIVE_WORLD, PASSIVE_WORLD):
            with self.subTest(path=path.name):
                robot = _robot_block(path.read_text(encoding="utf-8"))
                self.assertEqual(robot.count("HingeJoint {"), 12)
                self.assertEqual(robot.count("endPoint Solid"), 12)
                self.assertEqual(robot.count("RotationalMotor {"), 12)
                self.assertEqual(robot.count("PositionSensor {"), 12)
                axes = re.findall(r"(?m)^\s*axis\s+([^\n#]+)", robot)
                self.assertEqual(len(axes), 12)
                normalized = [tuple(float(v) for v in axis.split()) for axis in axes]
                self.assertEqual(sum(abs(axis[0]) > 0 for axis in normalized), 4)
                self.assertEqual(sum(abs(axis[1]) > 0 for axis in normalized), 8)
                self.assertTrue(all(abs(axis[2]) < 1e-12 for axis in normalized))

    def test_robot_model_differs_only_by_controller_and_active_spring(self) -> None:
        active = _robot_block(ACTIVE_WORLD.read_text(encoding="utf-8"))
        passive = _robot_block(PASSIVE_WORLD.read_text(encoding="utf-8"))
        self.assertEqual(_model_tokens(active), _model_tokens(passive))
        self.assertEqual(active.count("springConstant 0"), 12)
        self.assertEqual(passive.count("springConstant 100"), 12)
        self.assertIn("translation 0 0 0.45", active)


class ImportAndDeviceTests(unittest.TestCase):
    """无 Webots 导入与设备完整性。"""

    def test_all_controller_modules_import_without_webots(self) -> None:
        program = r"""
import importlib.abc, sys
class Blocker(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname == 'controller' or fullname.startswith('controller.'):
            raise ImportError('Webots intentionally unavailable')
        return None
sys.meta_path.insert(0, Blocker())
for name in ('teleop_types', 'teleop_input', 'joint_safety', 'teleop_state',
             'mini_cheetah_pd', 'flat_ground_teleop'):
    __import__(name)
"""
        result = subprocess.run(
            [sys.executable, "-B", "-c", program],
            cwd=CONTROLLER_DIR,
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_twelve_devices_are_complete_and_configured(self) -> None:
        robot = _Robot()
        controller = FlatGroundTeleopController(robot)
        self.assertTrue(controller.initialize_devices())
        self.assertEqual(len(controller.motors), JOINT_COUNT)
        self.assertEqual(len(controller.sensors), JOINT_COUNT)
        self.assertEqual(controller.missing_devices, [])
        self.assertTrue(all(motor.positions == [float("inf")] for motor in robot.motors.values()))
        self.assertTrue(all(sensor.enabled == [4] for sensor in robot.sensors.values()))
        self.assertEqual(len(set(FLAT_MOTOR_NAMES)), 12)
        self.assertEqual(len(set(FLAT_SENSOR_NAMES)), 12)


class PdAndSafetyTests(unittest.TestCase):
    """12 路 PD、限幅与反馈安全。"""

    def test_pd_gains_and_limits_cover_all_joints(self) -> None:
        self.assertEqual(KP, (3.0, 3.0, 3.0))
        self.assertEqual(KD, (1.0, 0.2, 0.2))
        self.assertEqual(MAX_TORQUE, 15.0)
        self.assertEqual(WEBOTS_MOTOR_MAX_TORQUE, 20.0)
        desired = (0.5,) * JOINT_COUNT
        torques = compute_pd_torques(desired, (0.0,) * 12, (0.0,) * 12, (0.0,) * 12)
        self.assertEqual(len(torques), JOINT_COUNT)
        self.assertTrue(all(-MAX_TORQUE <= value <= MAX_TORQUE for value in torques))
        self.assertTrue(all(value > 0.0 for value in torques))

    def test_nonfinite_feedback_forces_zero_torques(self) -> None:
        bad = [0.0] * JOINT_COUNT
        bad[7] = math.nan
        self.assertFalse(validate_joint_feedback(bad, [0.0] * JOINT_COUNT))
        self.assertEqual(
            compute_pd_torques([0.2] * 12, [0.0] * 12, bad, [0.0] * 12),
            (0.0,) * JOINT_COUNT,
        )

    def test_gait_targets_are_finite_and_bounded(self) -> None:
        gait = MiniCheetahGait()
        targets = gait.step(0.6, 0.3, 1.0, 0.004)
        self.assertEqual(len(targets.positions), JOINT_COUNT)
        self.assertEqual(len(targets.velocities), JOINT_COUNT)
        self.assertTrue(all(math.isfinite(value) for value in targets.positions))
        self.assertTrue(
            all(
                lower <= value <= upper
                for value, (lower, upper) in zip(targets.positions, TARGET_POSITION_LIMITS)
            )
        )


class InputAndModeTests(unittest.TestCase):
    """键盘映射、模式冲突与安全回站立。"""

    @staticmethod
    def _command(events: list[int], *, joystick: _Joystick | None = None):
        robot = _Robot()
        robot.keyboard = _Keyboard(events)
        robot.joystick = joystick or _Joystick()
        controller = FlatGroundTeleopController(robot)
        controller.keyboard = robot.keyboard
        controller.joystick = robot.joystick
        return controller._read_input()

    def test_default_is_standing_and_motion_is_ignored(self) -> None:
        machine = TeleopStateMachine(clock=lambda: 0.0)
        self.assertEqual(machine.mode, ControlMode.STANDING)
        update = machine.update(TeleopCommand(vx=0.6, motion_active=True), 0.0)
        self.assertEqual(update.mode, ControlMode.STANDING)
        self.assertEqual(update.velocity, (0.0, 0.0, 0.0))

    def test_w_s_a_d_q_e_signs_and_same_axis_latest_wins(self) -> None:
        cases = (
            ([KEY_W], (0.6, 0.0, 0.0)),
            ([KEY_S], (-0.3, 0.0, 0.0)),
            ([KEY_A], (0.0, 0.3, 0.0)),
            ([KEY_D], (0.0, -0.3, 0.0)),
            ([KEY_Q], (0.0, 0.0, 1.0)),
            ([KEY_E], (0.0, 0.0, -1.0)),
            ([KEY_W, KEY_S], (-0.3, 0.0, 0.0)),
            ([KEY_A, KEY_D], (0.0, -0.3, 0.0)),
            ([KEY_Q, KEY_E], (0.0, 0.0, -1.0)),
        )
        for events, expected in cases:
            with self.subTest(events=events):
                self.assertEqual(self._command(events).velocity, expected)

    def test_space_and_a_conflict_rule_is_explicit(self) -> None:
        machine = TeleopStateMachine(clock=lambda: 0.0)
        simultaneous = machine.update(
            TeleopCommand(mode_toggle_edge=True, a_edge=True, vy=0.3, motion_active=True),
            0.0,
        )
        self.assertEqual(simultaneous.mode, ControlMode.WALKING)
        self.assertEqual(simultaneous.reason, "mode_toggle")

        machine = TeleopStateMachine(clock=lambda: 0.0)
        first = machine.update(TeleopCommand(a_edge=True, vy=0.3, motion_active=True), 0.0)
        self.assertEqual(first.mode, ControlMode.WALKING)
        self.assertEqual(first.velocity, (0.0, 0.0, 0.0))
        second = machine.update(TeleopCommand(vy=0.3, motion_active=True), 0.0)
        self.assertEqual(second.velocity, (0.0, 0.3, 0.0))
        lateral = machine.update(
            TeleopCommand(a_edge=True, vy=0.3, motion_active=True), 0.0
        )
        self.assertEqual(lateral.reason, "a_lateral")
        self.assertEqual(lateral.velocity, (0.0, 0.3, 0.0))

    def test_r_escape_and_timeout_return_to_standing(self) -> None:
        machine = TeleopStateMachine(clock=lambda: 0.0, input_timeout=0.35)
        machine.update(TeleopCommand(mode_toggle_edge=True, motion_active=True), 0.0)
        self.assertEqual(machine.update(TeleopCommand(reset_edge=True), 0.0).mode, ControlMode.STANDING)
        machine.update(TeleopCommand(mode_toggle_edge=True, motion_active=True), 0.0)
        self.assertEqual(machine.update(TeleopCommand(force_stand=True), 0.0).mode, ControlMode.STANDING)
        machine.update(TeleopCommand(mode_toggle_edge=True, motion_active=True), 0.0)
        timed_out = machine.update(TeleopCommand(), 0.36)
        self.assertEqual(timed_out.mode, ControlMode.STANDING)
        self.assertEqual(timed_out.reason, "input_timeout")
        self.assertEqual(self._command([KEY_R]).force_stand, True)
        self.assertEqual(self._command([KEY_ESCAPE]).force_stand, True)

    def test_gamepad_deadzone_disconnect_nan_and_mapping(self) -> None:
        pad = WebotsJoystickAdapter()
        dead = _Joystick((100.0, -100.0, 100.0))
        self.assertEqual(pad.update(dead).velocity, (0.0, 0.0, 0.0))
        mapped = _Joystick((32767.0, -32767.0, 32767.0))
        self.assertEqual(pad.update(mapped).velocity, (0.6, 0.3, 1.0))

        invalid_pad = _Joystick((math.nan, 0.0, 0.0))
        invalid = pad.update(invalid_pad)
        self.assertFalse(invalid.valid)
        merged = merge_commands(TeleopCommand(), TeleopCommand(valid=False, force_stand=True))
        self.assertTrue(merged.force_stand)

        connected = _Joystick()
        pad.update(connected)
        connected.connected = False
        disconnected = pad.update(connected)
        self.assertTrue(disconnected.disconnected_edge)
        self.assertTrue(merge_commands(TeleopCommand(), TeleopCommand(force_stand=True)).force_stand)


if __name__ == "__main__":
    unittest.main(verbosity=2)
