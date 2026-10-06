#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""yobogo_loco_jump_v1 控制器状态与本机遥控的独立持久化测试。

直接运行：
    python3 webots-sim/controllers/rl_agent/test_loco_teleop.py

测试不启动 Webots，也不加载 checkpoint。导入 rl_agent 前安装最小 controller
桩，确保在无 Webots Python 环境、无 WEBOTS_HOME 的审计环境中也能稳定运行。
"""

from __future__ import annotations

import math
import sys
import types
import unittest
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
RL_DIR = HERE.parents[1] / "rl"
for path in (RL_DIR, HERE):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

# 单元测试只需要 Supervisor 类型名，不实例化 Webots Supervisor。
controller_module = sys.modules.get("controller")
if controller_module is None or not hasattr(controller_module, "Supervisor"):
    controller_module = types.ModuleType("controller")
    controller_module.Supervisor = type("Supervisor", (), {})
    sys.modules["controller"] = controller_module

import rl_agent  # noqa: E402
from loco_jump_contract import (  # noqa: E402
    OBS_DIM,
    OBS_SLICES,
    TCP_STATE_REQUIRED_FIELDS,
)
from loco_teleop import (  # noqa: E402
    KEY_ESC,
    KEY_SHIFT,
    JoystickReader,
    KeyboardReader,
    TeleopCommand,
    clip_command,
    merged_command,
)


class MockTimeRobot:
    """只提供控制器状态读取所需的仿真时间。"""

    def __init__(self, time_value: float = 0.0) -> None:
        self.time_value = float(time_value)

    def getTime(self) -> float:
        return self.time_value


class MockRobotNode:
    """提供世界系速度与机身位置。"""

    def getVelocity(self) -> list[float]:
        return [1.0, 2.0, 3.0, 0.1, 0.2, 0.3]

    def getPosition(self) -> list[float]:
        return [0.0, 0.0, 0.26]


class MockKeyboard:
    """按预设事件序列响应 Webots Keyboard.getKey()。"""

    def __init__(self, events: list[int]) -> None:
        self.events = list(events)

    def getKey(self) -> int:
        return self.events.pop(0) if self.events else -1


class MockMotor:
    """记录 setPosition() 的最终目标角。"""

    def __init__(self) -> None:
        self.target: float | None = None

    def setPosition(self, value: float) -> None:
        self.target = float(value)


def make_agent(time_value: float = 0.0) -> rl_agent.RlAgent:
    """构造不依赖 Webots 实例的 RlAgent 测试夹具。"""
    agent = rl_agent.RlAgent.__new__(rl_agent.RlAgent)
    agent.robot = MockTimeRobot(time_value)
    agent.robot_node = MockRobotNode()
    agent._prev_q = np.zeros(rl_agent.ACTION_DIM, dtype=np.float64)
    agent._prev_ctrl_time = float(time_value)
    agent._prev_yaw = 0.0
    agent._prev_ctrl_time_w = float(time_value)
    agent.last_action = np.zeros(rl_agent.ACTION_DIM, dtype=np.float64)
    agent.current_command = np.array([0.1, -0.2, 0.3], dtype=np.float64)
    agent.sprint_mode = False
    agent.jump_latched = False
    agent._jump_started_at = None
    agent._jump_contact_lost = False
    agent._jump_phase_time = 0.0
    agent._read_joint_positions = lambda: (
        np.arange(rl_agent.ACTION_DIM, dtype=np.float64) * 0.1
    )
    agent._read_rpy = lambda: np.array(
        [0.0, 0.0, math.pi / 2.0], dtype=np.float64
    )
    agent._read_linear_velocity = lambda: np.array(
        [1.0, 2.0, 3.0], dtype=np.float64
    )
    agent._read_contacts = lambda: np.ones(4, dtype=np.float64)
    return agent


class ControllerHeaderTests(unittest.TestCase):
    """文件头必须反映 55 维 loco_jump，并明确旧 TCP 兼容范围。"""

    def test_header_uses_55_dim_contract(self) -> None:
        doc = rl_agent.__doc__ or ""
        self.assertIn("yobogo_loco_jump_v1", doc)
        self.assertIn("55 维观测", doc)
        self.assertNotIn("拼成 42 维观测", doc)
        self.assertIn("旧 TCP 兼容说明", doc)
        self.assertIn("12 维 [-1,1]", doc)


class StateAndObservationTests(unittest.TestCase):
    """TCP state 字段和 55 维观测切片。"""

    def test_tcp_state_fields_and_lengths(self) -> None:
        state = make_agent().read_state()
        for field in TCP_STATE_REQUIRED_FIELDS:
            self.assertIn(field, state)
        expected_lengths = {
            "q": 12,
            "dq": 12,
            "rpy": 3,
            "omega": 3,
            "v_body": 3,
            "contacts": 4,
        }
        for field, length in expected_lengths.items():
            self.assertEqual(len(state[field]), length, field)
        self.assertFalse(state["done"])
        self.assertEqual(state["height"], state["body_height"])
        self.assertEqual(state["jump_phase"], state["jump_phase_time"])
        self.assertIn("v", state)
        self.assertIn("z", state)

    def test_observation_is_contract_ordered_55_dim(self) -> None:
        agent = make_agent()
        state = agent.read_state()
        obs = agent.state_to_obs(state)
        self.assertEqual(obs.shape, (OBS_DIM,))
        np.testing.assert_allclose(
            obs[OBS_SLICES["q"]], np.asarray(state["q"], dtype=np.float32)
        )
        np.testing.assert_allclose(
            obs[OBS_SLICES["rpy"]], np.asarray(state["rpy"], dtype=np.float32)
        )
        np.testing.assert_allclose(
            obs[OBS_SLICES["v_body"]],
            np.asarray(state["v_body"], dtype=np.float32),
        )
        np.testing.assert_allclose(
            obs[OBS_SLICES["prev_action"]], agent.last_action.astype(np.float32)
        )
        np.testing.assert_allclose(
            obs[OBS_SLICES["cmd_vx_vy_wz"]],
            agent.current_command.astype(np.float32),
        )
        np.testing.assert_allclose(
            obs[OBS_SLICES["foot_contact"]],
            np.asarray(state["contacts"], dtype=np.float32),
        )
        self.assertEqual(float(obs[OBS_SLICES["jump_request"]][0]), 0.0)
        self.assertAlmostEqual(
            float(obs[OBS_SLICES["body_height"]][0]), 0.26, places=5
        )


class KeyboardTests(unittest.TestCase):
    """键盘 W/S/A/D/Q/E、Shift、Space 与 R。"""

    def test_motion_keys(self) -> None:
        cases = (
            ([ord("W")], "vx", 0.6),
            ([ord("S")], "vx", -0.3),
            ([ord("A")], "vy", 0.3),
            ([ord("D")], "vy", -0.3),
            ([ord("Q")], "wz", 1.0),
            ([ord("E")], "wz", -1.0),
        )
        for events, field, expected in cases:
            with self.subTest(events=events):
                command = KeyboardReader(MockKeyboard(events)).read()
                self.assertEqual(getattr(command, field), expected)
                self.assertEqual(
                    clip_command(command.vx, command.vy, command.wz),
                    command.velocity,
                )

    def test_shift_space_and_reset_edges(self) -> None:
        reader = KeyboardReader(
            MockKeyboard(
                [
                    KEY_SHIFT | ord("W"),
                    ord(" "),
                    ord("R"),
                ]
            )
        )
        first = reader.read()
        self.assertEqual(first.vx, 0.6)
        self.assertTrue(first.sprint)
        self.assertTrue(first.jump_edge)
        self.assertTrue(first.reset_edge)

        second = reader.read()
        self.assertFalse(second.jump_edge)
        self.assertFalse(second.reset_edge)

    def test_escape_is_edge_triggered_estop(self) -> None:
        reader = KeyboardReader(MockKeyboard([KEY_ESC]))
        first = reader.read()
        self.assertTrue(first.estop_edge)
        self.assertEqual(first.velocity, (0.0, 0.0, 0.0))
        second = reader.read()
        self.assertFalse(second.estop_edge)


class JoystickTests(unittest.TestCase):
    """手柄摇杆、0.12 死区和 A/B 上升沿。"""

    @staticmethod
    def make_joystick() -> JoystickReader:
        return JoystickReader(path="/nonexistent-yobogo-test-js0")

    def test_stick_deadzone_and_mapping(self) -> None:
        joystick = self.make_joystick()
        joystick.inject_event(1, 0, 100)
        joystick.inject_event(1, 1, -100)
        joystick.inject_event(1, 2, 100)
        self.assertEqual(joystick.read().velocity, (0.0, 0.0, 0.0))

        joystick.inject_event(1, 0, 32767)
        joystick.inject_event(1, 1, -32767)
        joystick.inject_event(1, 2, 32767)
        command = joystick.read()
        self.assertEqual(command.velocity, (0.6, -0.3, -1.0))
        self.assertEqual(
            clip_command(command.vx, command.vy, command.wz),
            command.velocity,
        )

    def test_jump_and_reset_button_edges(self) -> None:
        joystick = self.make_joystick()
        joystick.inject_event(2, 0, 1)
        joystick.inject_event(2, 1, 1)
        first = joystick.read()
        self.assertTrue(first.jump_edge)
        self.assertTrue(first.reset_edge)

        second = joystick.read()
        self.assertFalse(second.jump_edge)
        self.assertFalse(second.reset_edge)

    def test_button_six_is_estop_edge(self) -> None:
        joystick = self.make_joystick()
        joystick.inject_event(2, 6, 1)
        first = joystick.read()
        self.assertTrue(first.estop_edge)
        second = joystick.read()
        self.assertFalse(second.estop_edge)


class MergePriorityTests(unittest.TestCase):
    """急停、复位、跳跃和速度分量的确定优先级。"""

    def test_estop_overrides_reset_jump_and_motion(self) -> None:
        merged = merged_command(
            [
                TeleopCommand(0.6, 0.3, 1.0, jump_edge=True),
                TeleopCommand(0.0, 0.0, 0.0, reset_edge=True),
                TeleopCommand(0.0, 0.0, 0.0, estop_edge=True),
            ]
        )
        self.assertTrue(merged.estop_edge)
        self.assertFalse(merged.reset_edge)
        self.assertFalse(merged.jump_edge)
        self.assertEqual(merged.velocity, (0.0, 0.0, 0.0))

    def test_keyboard_nonzero_component_has_priority(self) -> None:
        merged = merged_command(
            [
                TeleopCommand(0.6, 0.0, -1.0),
                TeleopCommand(-0.3, 0.3, 1.0),
            ]
        )
        self.assertEqual(merged.velocity, (0.6, 0.3, -1.0))


class JumpLatchTests(unittest.TestCase):
    """足端恢复接触和 1 秒超时两条清除路径。"""

    def test_contact_loss_then_restore_clears_latch(self) -> None:
        agent = make_agent()
        agent.request_jump()
        self.assertTrue(agent.jump_latched)

        agent._update_jump_latch(np.ones(4), now=0.5)
        self.assertTrue(agent.jump_latched)
        agent._update_jump_latch(np.array([0, 1, 1, 1]), now=0.55)
        self.assertTrue(agent.jump_latched)
        agent._update_jump_latch(np.zeros(4), now=0.6)
        self.assertTrue(agent.jump_latched)
        agent._update_jump_latch(np.ones(4), now=0.7)
        self.assertFalse(agent.jump_latched)
        self.assertAlmostEqual(agent._jump_phase_time, 0.7)

    def test_one_second_timeout_clears_latch(self) -> None:
        agent = make_agent()
        agent.request_jump()
        agent._update_jump_latch(np.ones(4), now=0.999)
        self.assertTrue(agent.jump_latched)
        agent._update_jump_latch(np.ones(4), now=1.0)
        self.assertFalse(agent.jump_latched)
        self.assertEqual(agent._jump_phase_time, 1.0)


class ActionMappingTests(unittest.TestCase):
    """12 维动作限幅、q_des 公式和命令限幅。"""

    def test_action_clip_and_q_des_formula(self) -> None:
        agent = rl_agent.RlAgent.__new__(rl_agent.RlAgent)
        agent.motors = [
            MockMotor() for _ in range(rl_agent.ACTION_DIM)
        ]
        agent.q_stand = np.zeros(rl_agent.ACTION_DIM, dtype=np.float64)
        agent.action_scale = rl_agent.ACTION_SCALE.copy()

        action = np.full(rl_agent.ACTION_DIM, 9.0, dtype=np.float64)
        action[1] = -9.0
        agent.apply_action(action)

        np.testing.assert_allclose(agent.last_action[0], 1.0)
        np.testing.assert_allclose(agent.last_action[1], -1.0)
        expected = agent.q_stand + agent.last_action * agent.action_scale
        actual = np.array(
            [motor.target for motor in agent.motors], dtype=np.float64
        )
        np.testing.assert_allclose(actual, expected, atol=1e-12)
        self.assertAlmostEqual(agent.motors[0].target or 0.0, 0.3)
        self.assertAlmostEqual(agent.motors[1].target or 0.0, -0.5)

    def test_command_limits(self) -> None:
        self.assertEqual(
            clip_command(9.0, -9.0, 9.0),
            (0.6, -0.3, 1.0),
        )
        self.assertEqual(
            clip_command(-9.0, 9.0, -9.0),
            (-0.3, 0.3, -1.0),
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
