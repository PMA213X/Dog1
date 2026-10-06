#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""flat_ground_teleop 纯模块单元测试；不需要 Webots 运行时。"""

from __future__ import annotations

import math
import os
import sys
import unittest


MODULE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if MODULE_DIR not in sys.path:
    sys.path.insert(0, MODULE_DIR)

from joint_safety import (  # noqa: E402
    FLAT_MOTOR_NAMES,
    FLAT_SENSOR_NAMES,
    MAX_TORQUE,
    compute_pd_torques,
    joint_pd_torque,
)
from teleop_input import (  # noqa: E402
    KEY_A,
    KEY_D,
    KEY_R,
    KEY_S,
    KEY_SPACE,
    KEY_W,
    KeyboardState,
    WebotsJoystickAdapter,
    merge_commands,
    normalize_joystick_axis,
)
from teleop_state import TeleopStateMachine  # noqa: E402
from teleop_types import ControlMode, TeleopCommand  # noqa: E402


class FakeJoystick:
    def __init__(self) -> None:
        self.connected = True
        self.axes = [0, 0, 0]
        self.button = -1

    def isConnected(self) -> bool:
        return self.connected

    def getAxisValue(self, index: int):
        return self.axes[index]

    def getPressedButton(self) -> int:
        return self.button


class TestJointSafety(unittest.TestCase):
    def test_device_names_are_exactly_twelve_each(self) -> None:
        self.assertEqual(len(FLAT_MOTOR_NAMES), 12)
        self.assertEqual(len(FLAT_SENSOR_NAMES), 12)
        self.assertEqual(FLAT_MOTOR_NAMES[0], "fr_abd_motor")
        self.assertEqual(FLAT_MOTOR_NAMES[-1], "hl_kn_motor")
        self.assertEqual(FLAT_SENSOR_NAMES[0], "fr_abd_sensor")
        self.assertEqual(FLAT_SENSOR_NAMES[-1], "hl_kn_sensor")

    def test_pd_and_torque_limit(self) -> None:
        self.assertAlmostEqual(joint_pd_torque(1.0, 0.0, 0.0, 0.0, 0), 3.0)
        self.assertAlmostEqual(joint_pd_torque(100.0, 0.0, 0.0, 0.0, 1), MAX_TORQUE)
        self.assertAlmostEqual(joint_pd_torque(-100.0, 0.0, 0.0, 0.0, 2), -MAX_TORQUE)

    def test_nonfinite_feedback_outputs_zero(self) -> None:
        positions = [0.0] * 12
        velocities = [0.0] * 12
        positions[4] = math.nan
        self.assertEqual(
            compute_pd_torques(positions, velocities, positions, velocities),
            (0.0,) * 12,
        )


class TestTeleopState(unittest.TestCase):
    def setUp(self) -> None:
        self.machine = TeleopStateMachine(clock=lambda: 0.0, input_timeout=0.2)

    def test_default_and_a_timing_rule(self) -> None:
        self.assertEqual(self.machine.mode, ControlMode.STANDING)
        first = self.machine.update(
            TeleopCommand(vy=0.3, a_edge=True, motion_active=True), 0.0
        )
        self.assertEqual(first.mode, ControlMode.WALKING)
        self.assertEqual(first.velocity, (0.0, 0.0, 0.0))
        second = self.machine.update(TeleopCommand(vy=0.3, motion_active=True), 0.004)
        self.assertEqual(second.mode, ControlMode.WALKING)
        self.assertEqual(second.velocity, (0.0, 0.3, 0.0))

    def test_a_is_lateral_only_in_walking(self) -> None:
        self.machine.update(TeleopCommand(mode_toggle_edge=True), 0.0)
        update = self.machine.update(
            TeleopCommand(vy=0.3, a_edge=True, motion_active=True), 0.004
        )
        self.assertEqual(update.mode, ControlMode.WALKING)
        self.assertEqual(update.velocity, (0.0, 0.3, 0.0))

    def test_space_toggles_both_ways_and_reset_is_safe(self) -> None:
        entered = self.machine.update(TeleopCommand(mode_toggle_edge=True), 0.0)
        self.assertEqual(entered.mode, ControlMode.WALKING)
        left = self.machine.update(TeleopCommand(mode_toggle_edge=True), 0.004)
        self.assertEqual(left.mode, ControlMode.STANDING)
        reset = self.machine.update(
            TeleopCommand(vx=0.6, reset_edge=True, motion_active=True), 0.008
        )
        self.assertEqual(reset.mode, ControlMode.STANDING)
        self.assertEqual(reset.velocity, (0.0, 0.0, 0.0))
        self.assertTrue(reset.safety_reset)

    def test_input_timeout_returns_standing(self) -> None:
        self.machine.update(TeleopCommand(mode_toggle_edge=True), 0.0)
        update = self.machine.update(TeleopCommand(), 0.3)
        self.assertEqual(update.mode, ControlMode.STANDING)
        self.assertEqual(update.reason, "input_timeout")


class TestInputAdapters(unittest.TestCase):
    def test_keyboard_hold_conflict_and_edges(self) -> None:
        keyboard = KeyboardState(clock=lambda: 0.0)
        first = keyboard.update([ord("w"), ord("s"), KEY_SPACE], 0.0)
        self.assertEqual(first.latest_key((KEY_W, KEY_S)), ord("s"))
        self.assertTrue(first.rising(KEY_SPACE))
        second = keyboard.update([], 0.01)
        self.assertTrue(second.active(ord("w")))
        self.assertFalse(second.rising(KEY_SPACE))
        expired = keyboard.update([], 1.0)
        self.assertFalse(expired.active(ord("w")))

    def test_joystick_mapping_disconnect_and_nonfinite(self) -> None:
        joystick = FakeJoystick()
        adapter = WebotsJoystickAdapter()
        joystick.axes = [32767, -32767, 16384]
        sample = adapter.update(joystick)
        self.assertTrue(sample.connected)
        self.assertGreater(sample.vx, 0.0)
        self.assertGreater(sample.vy, 0.0)
        self.assertGreater(sample.wz, 0.0)
        joystick.connected = False
        disconnected = adapter.update(joystick)
        self.assertTrue(disconnected.disconnected_edge)
        self.assertEqual(disconnected.velocity, (0.0, 0.0, 0.0))
        self.assertIsNone(normalize_joystick_axis(math.inf))

    def test_merge_prefers_nonzero_keyboard(self) -> None:
        merged = merge_commands(
            TeleopCommand(vx=0.6, vy=0.3, mode_toggle_edge=True),
            TeleopCommand(vx=-0.3, vy=-0.3, reset_edge=True),
        )
        self.assertEqual(merged.velocity, (0.6, 0.3, 0.0))
        self.assertTrue(merged.mode_toggle_edge)
        self.assertTrue(merged.reset_edge)


if __name__ == "__main__":
    unittest.main()
