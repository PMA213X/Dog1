#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""无 Webots 的环境、TCP 和输入测试。"""

from __future__ import annotations

import math
import socket
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
CONTROLLERS = ROOT / "controllers"
for path in (ROOT, CONTROLLERS / "rl_agent"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import rl.contract as contract
from rl.env import MiniCheetahFlatJumpEnv, require_ports_free
from input_adapter import InputAdapter
from rl_agent import (
    RlAgentController,
    action_to_stance_targets,
    connection_failure_report,
    smooth_action_targets,
)
from tcp_protocol import decode, encode, validate_message


def state(**changes: object) -> dict[str, object]:
    value: dict[str, object] = {
        "q": [0.0] * 12,
        "dq": [0.0] * 12,
        "rpy": [0.0, 0.0, 0.0],
        "v_body": [0.0, 0.0, 0.0],
        "omega_body": [0.0, 0.0, 0.0],
        "contacts": [1.0, 0.0, 1.0, 0.0],
        "foot_contacts": [1.0, 0.0, 1.0, 0.0],
        "joint_torques": [0.0] * 12,
        "foot_velocities": [0.0] * 12,
        "non_foot_contact": False,
        "body_contact": False,
        "height": 0.45,
        "position": [0.0, 0.0, 0.45],
        "jump_phase": 0.0,
        "jump_success": False,
        "jump_landing": False,
        "done": False,
    }
    value.update(changes)
    return value


class MockEnv(MiniCheetahFlatJumpEnv):
    def __init__(self, **kwargs: object) -> None:
        super().__init__(start_bridge=False, **kwargs)  # type: ignore[arg-type]
        self._conn = object()
        self.sent: list[dict[str, object]] = []
        self.next_state = state()

    def _send(self, payload: object) -> None:
        self.sent.append(dict(payload))  # type: ignore[arg-type]

    def _recv(self) -> dict[str, object]:
        return dict(self.next_state)


class EnvironmentTests(unittest.TestCase):
    def test_target_port_conflict_is_rejected(self) -> None:
        completed = type(
            "Completed",
            (),
            {
                "stdout": (
                    "LISTEN 0 50 *:1234 0.0.0.0:* "
                    'users:(("webots-bin",pid=9001,fd=21))'
                ),
                "stderr": "",
                "returncode": 0,
            },
        )()
        with patch("rl.env.subprocess.run", return_value=completed):
            with self.assertRaisesRegex(RuntimeError, "1234"):
                require_ports_free(
                    (contract.WEBOTS_SUPERVISOR_PORT,),
                    "启动评估环境",
                )

    def test_time_wait_does_not_block_bind(self) -> None:
        completed = type(
            "Completed",
            (),
            {
                "stdout": (
                    "TIME-WAIT 0 0 127.0.0.1:11452 127.0.0.1:54321 "
                    'users:(("python3",pid=9002,fd=44))'
                ),
                "stderr": "",
                "returncode": 0,
            },
        )()
        with patch("rl.env.subprocess.run", return_value=completed):
            require_ports_free(
                contract.bridge_ports(),
                "启动 Gate",
            )

    def test_webots_and_controller_share_explicit_supervisor_port(self) -> None:
        commands: list[list[str]] = []

        class FakeProcess:
            pid = 91002

            def poll(self) -> int | None:
                return 0

        def fake_popen(command: list[str], **kwargs: object) -> FakeProcess:
            del kwargs
            commands.append(list(command))
            return FakeProcess()

        with tempfile.TemporaryDirectory() as temporary:
            with patch.object(contract, "LOG_ROOT", Path(temporary)):
                env = MiniCheetahFlatJumpEnv(
                    phase="P0",
                    bridge_port=11999,
                    start_bridge=False,
                    supervisor_port=contract.WEBOTS_SUPERVISOR_PORT,
                )
                env.open_bridge()
                with patch("rl.env.subprocess.Popen", side_effect=fake_popen):
                    env.launch_webots()
                    env.launch_controller()
                env._webots = None
                env._controllers.clear()
                env.close()
        expected = f"--port={contract.WEBOTS_SUPERVISOR_PORT}"
        self.assertEqual(len(commands), 2)
        self.assertIn(expected, commands[0])
        self.assertIn(expected, commands[1])

    def test_webots_port_fallback_fails_before_controller(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            log_root = Path(temporary)
            log_path = log_root / "webots_eval_world.log"
            log_path.write_text(
                "WARNING: Could not listen to extern controllers and robot "
                "windows on port 1234. Port 1234 is already in use. "
                "Using port 1235 instead.\n",
                encoding="utf-8",
            )
            env = MiniCheetahFlatJumpEnv(
                phase="P0",
                bridge_port=11998,
                start_bridge=False,
                world=contract.EVAL_WORLD_PATH,
                robot_name="mini_cheetah",
                supervisor_port=contract.WEBOTS_SUPERVISOR_PORT,
            )
            env._webots = type(
                "FakeWebots",
                (),
                {"poll": lambda self: None},
            )()
            env._webots_log_offset = 0
            with (
                patch.object(contract, "LOG_ROOT", log_root),
                patch(
                    "rl.env.listening_tcp_ports",
                    return_value=[contract.WEBOTS_SUPERVISOR_PORT],
                ),
            ):
                with self.assertRaisesRegex(
                    RuntimeError,
                    "webots_supervisor_port_mismatch",
                ):
                    env.verify_webots_supervisor_port(timeout=0.1)

    def test_reset_and_step_are_57_12_finite(self) -> None:
        env = MockEnv(phase="P3", randomization_mode="fixed")
        obs, info = env.reset(seed=1)
        self.assertEqual(obs.shape, (57,))
        self.assertTrue(np.all(np.isfinite(obs)))
        self.assertEqual(obs[56], 1.0)
        env.next_state = state(
            position=list(contract.BIRTH_POSITIONS[env.worker_id])
        )
        result = env.step(np.zeros(12))
        self.assertEqual(result[0].shape, (57,))
        self.assertTrue(math.isfinite(result[1]))
        self.assertFalse(result[2])
        self.assertFalse(result[3])
        env.close()

    def test_finish_reset_timeout_is_explicit_worker_context(self) -> None:
        env = MockEnv(phase="P0", randomization_mode="fixed")
        env.prepare_reset(seed=41)
        env.send_prepared_reset()

        def timeout_recv() -> dict[str, object]:
            raise socket.timeout("timed out")

        env._recv = timeout_recv  # type: ignore[method-assign]
        with self.assertRaisesRegex(
            TimeoutError,
            r"worker_id=0.*timeout=180\.0",
        ):
            env.finish_reset()
        env.close()

    def test_fall_terminates_and_reward_is_negative(self) -> None:
        env = MockEnv(phase="P1", randomization_mode="fixed")
        env.reset(seed=2)
        env.next_state = state(height=0.10)
        _, reward, terminated, truncated, info = env.step(np.zeros(12))
        self.assertTrue(terminated)
        self.assertFalse(truncated)
        self.assertTrue(info["fallen"])
        self.assertLess(reward, 0.0)
        env.close()

    def test_fixed_and_curriculum_randomization(self) -> None:
        fixed = MockEnv(phase="P1", randomization_mode="fixed")
        _, info = fixed.reset(seed=3)
        self.assertEqual(
            info["randomization"],
            {
                "friction": 1.2,
                "mass_scale": 1.0,
                "delay_steps": 0,
                "observation_noise": 0.0,
                "motor_strength": 1.0,
                "external_impulse": [0.0, 0.0, 0.0],
                "level": 0,
            },
        )
        fixed.close()
        curriculum = MockEnv(phase="P3", randomization_mode="curriculum")
        for _ in range(contract.CURRICULUM_FIXED_EPISODES):
            _, info = curriculum.reset(seed=4)
            self.assertEqual(info["randomization"]["friction"], 1.2)
        _, info = curriculum.reset(seed=24)
        self.assertTrue(
            contract.FRICTION_RANGE[0]
            <= info["randomization"]["friction"]
            <= contract.FRICTION_RANGE[1]
        )
        curriculum.close()

    def test_command_domain_by_phase(self) -> None:
        stable = MockEnv(phase="P1", randomization_mode="fixed")
        stable.reset(seed=5)
        np.testing.assert_array_equal(stable._command, np.zeros(3))
        stable.close()
        moving = MockEnv(phase="P3", randomization_mode="fixed")
        moving.reset(seed=6, options={"command": [9, -9, 9]})
        np.testing.assert_allclose(moving._command, [0.6, -0.3, 1.0])
        moving.close()
        low_speed = MockEnv(phase="P2", randomization_mode="fixed")
        low_speed.reset(seed=7, options={"command": [9, -9, 9]})
        np.testing.assert_allclose(low_speed._command, [0.25, -0.10, 0.30])
        low_speed.close()
        stationary_jump = MockEnv(phase="P5", randomization_mode="fixed")
        stationary_jump.reset(seed=8, options={"command": [9, -9, 9]})
        np.testing.assert_array_equal(stationary_jump._command, np.zeros(3))
        stationary_jump.close()

    def test_p2_stratified_commands_cover_gate_forward_and_other_modes(self) -> None:
        cycle = contract.P2_COMMAND_STRATUM_CYCLE
        env = MockEnv(phase="P2", randomization_mode="fixed")
        env.reset(seed=44)
        samples = [env._command.copy()]
        samples.extend(
            env.sample_command()
            for _ in range(len(cycle) - 1)
        )
        forward = [
            value for value in samples
            if value[1] == 0.0
            and value[2] == 0.0
            and contract.P2_GATE_FORWARD_COMMAND_MIN
            <= value[0]
            <= contract.P2_GATE_FORWARD_COMMAND_MAX
        ]
        zero = [
            value for value in samples
            if np.count_nonzero(value) == 0
        ]
        lateral = [
            value for value in samples
            if value[0] == 0.0 and value[2] == 0.0 and value[1] != 0.0
        ]
        yaw = [
            value for value in samples
            if value[0] == 0.0 and value[1] == 0.0 and value[2] != 0.0
        ]
        self.assertEqual(len(forward), 8)
        self.assertEqual(len(zero), 4)
        self.assertEqual(len(lateral), 2)
        self.assertEqual(len(yaw), 2)
        self.assertEqual(env._p2_command_slot, 0)
        env.close()

        other = MockEnv(phase="P2", randomization_mode="fixed")
        other.reset(seed=45)
        other_samples = [other._command.copy()]
        other_samples.extend(
            other.sample_command()
            for _ in range(len(cycle) - 1)
        )
        self.assertFalse(
            all(
                np.allclose(left, right)
                for left, right in zip(samples, other_samples)
            )
        )
        other.close()

        stationary = MockEnv(phase="P1", randomization_mode="fixed")
        stationary.reset(seed=46)
        for _ in range(len(cycle)):
            np.testing.assert_array_equal(
                stationary.sample_command(),
                np.zeros(3),
            )
        stationary.close()

    def test_reset_uses_worker_birth_position(self) -> None:
        for worker_id, birth in enumerate(contract.BIRTH_POSITIONS):
            env = MockEnv(phase="P1", worker_id=worker_id)
            env.reset(seed=10 + worker_id)
            reset_message = next(
                message for message in env.sent if message["type"] == "reset"
            )
            self.assertEqual(reset_message["birth_position"], list(birth))
            env.close()

    def test_fall_boundary_is_local_to_each_birth_position(self) -> None:
        for worker_id, birth in enumerate(contract.BIRTH_POSITIONS):
            env = MockEnv(phase="P1", worker_id=worker_id)
            env.reset(seed=20 + worker_id)
            inside = state(
                position=[
                    birth[0] + contract.LOCAL_ACTIVITY_RADIUS,
                    birth[1] - contract.LOCAL_ACTIVITY_RADIUS,
                    birth[2],
                ]
            )
            self.assertFalse(env._fallen(inside))
            outside = state(
                position=[
                    birth[0] + contract.LOCAL_ACTIVITY_RADIUS + 0.01,
                    birth[1],
                    birth[2],
                ]
            )
            self.assertTrue(env._fallen(outside))
            env.close()

    def test_eval_world_uses_single_robot_contract(self) -> None:
        env = MiniCheetahFlatJumpEnv(
            phase="P3",
            start_bridge=False,
            world=contract.EVAL_WORLD_PATH,
            worker_id=0,
        )
        self.assertEqual(env.robot_name, "mini_cheetah")
        self.assertEqual(env.birth_position, contract.EVAL_BIRTH_POSITION)
        env.close()

    def test_p1_rsi_and_delayed_reward_action(self) -> None:
        env = MockEnv(phase="P1", randomization_mode="fixed")
        env.reset(seed=30)
        reset_message = next(
            message for message in env.sent if message["type"] == "reset"
        )
        self.assertEqual(reset_message["reset_style"], "rsi")
        self.assertEqual(reset_message["perturbation"], [0.0] * 12)
        np.testing.assert_array_equal(env._previous_action, np.zeros(12))
        np.testing.assert_array_equal(env._last_policy_action, np.zeros(12))
        self.assertEqual(
            action_to_stance_targets([0.0] * 12),
            contract.rsi_targets(),
        )
        env._delayed_actions = [np.zeros(12, dtype=np.float32)]
        message = env.prepare_step(np.ones(12, dtype=np.float32))
        np.testing.assert_array_equal(message["action"], np.zeros(12))
        env.send_prepared_step()
        _, reward, _, _, info = env.finish_step()
        self.assertEqual(info["reward_parts"]["action_rate"], 0.0)
        self.assertTrue(math.isfinite(reward))
        env.close()

    def test_p1_episode_truncates_at_gate_duration(self) -> None:
        env = MockEnv(phase="P1", randomization_mode="fixed")
        env.reset(seed=31)
        birth = list(contract.BIRTH_POSITIONS[env.worker_id])
        env._episode_steps = contract.P1_EPISODE_STEPS - 1
        env.next_state = state(
            height=contract.REFERENCE_HEIGHT,
            position=birth,
            contacts=[1.0] * 4,
            foot_contacts=[1.0] * 4,
        )
        _, _, terminated, truncated, _ = env.step(np.zeros(12))
        self.assertFalse(terminated)
        self.assertTrue(truncated)
        env.close()

    def test_action_target_rate_limit_helper(self) -> None:
        previous = contract.DEFAULT_CROUCH
        desired = contract.action_to_target([0.5] * contract.ACTION_DIM)
        first = smooth_action_targets(previous, desired)
        bound = contract.ACTION_TARGET_RATE_LIMIT * contract.CONTROL_DT_SECONDS
        self.assertLessEqual(
            max(abs(actual - base) for actual, base in zip(first, previous)),
            bound + 1e-9,
        )
        self.assertTrue(
            all(
                low <= value <= high
                for value, (low, high) in zip(first, contract.TARGET_POSITION_LIMITS)
            )
        )
        current = first
        for _ in range(10):
            next_target = smooth_action_targets(current, desired)
            self.assertLessEqual(
                max(
                    abs(actual - base)
                    for actual, base in zip(next_target, current)
                ),
                bound + 1e-9,
            )
            current = next_target
        with self.assertRaises(ValueError):
            smooth_action_targets([0.0] * 11, [0.0] * 12)
        with self.assertRaises(ValueError):
            smooth_action_targets([0.0] * 12, [math.nan] + [0.0] * 11)

    def test_set_targets_bypasses_action_rate_limiter(self) -> None:
        class Motor:
            def __init__(self) -> None:
                self.target = None

            def setPosition(self, value: float) -> None:
                self.target = float(value)

        controller = object.__new__(RlAgentController)
        controller.motors = [Motor() for _ in range(contract.ACTION_DIM)]
        raw_targets = [
            limits[0] if index % 2 == 0 else limits[1]
            for index, limits in enumerate(contract.TARGET_POSITION_LIMITS)
        ]
        controller.set_targets(raw_targets)
        self.assertEqual(
            [motor.target for motor in controller.motors],
            raw_targets,
        )

    def test_true_foot_contacts_remain_independent(self) -> None:
        env = MockEnv(phase="P1", randomization_mode="fixed")
        env.reset(seed=32)
        env.next_state = state(
            foot_contacts=[1.0, 0.0, 0.0, 1.0],
            contacts=[1.0, 0.0, 0.0, 1.0],
            position=list(contract.BIRTH_POSITIONS[env.worker_id]),
        )
        _, _, terminated, _, info = env.step(np.zeros(12))
        self.assertFalse(terminated)
        np.testing.assert_array_equal(
            info["foot_contacts"],
            [1.0, 0.0, 0.0, 1.0],
        )
        env.close()

    def test_p1_fixed_rsi_then_local_perturbation(self) -> None:
        env = MockEnv(phase="P1", randomization_mode="fixed")
        env.reset(seed=33)
        initial = next(
            message for message in env.sent if message["type"] == "reset"
        )
        self.assertEqual(initial["perturbation"], [0.0] * 12)
        env._phase_steps = contract.P1_PERTURBATION_LOCAL_STEPS
        env.reset(seed=34)
        perturbed = [
            message
            for message in env.sent
            if message["type"] == "reset"
        ][-1]
        self.assertTrue(
            any(abs(value) > 0.0 for value in perturbed["perturbation"])
        )
        env.close()

    def test_first_reset_reads_external_impulse_from_protocol_mapping(self) -> None:
        """首次 reset 必须从 reset.randomization 读取 DR7 冲量。"""
        from contextlib import redirect_stdout
        import io

        class Field:
            def __init__(self) -> None:
                self.values: list[list[float]] = []

            def setSFVec3f(self, value: Sequence[float]) -> None:
                self.values.append([float(item) for item in value])

            def setSFRotation(self, value: Sequence[float]) -> None:
                self.values.append([float(item) for item in value])

        class Node:
            def __init__(self) -> None:
                self.translation = Field()
                self.rotation = Field()
                self.velocities: list[list[float]] = []
                self.reset_count = 0

            def getField(self, name: str) -> Field:
                if name == "translation":
                    return self.translation
                if name == "rotation":
                    return self.rotation
                raise AssertionError(name)

            def resetPhysics(self) -> None:
                self.reset_count += 1

            def setVelocity(self, value: Sequence[float]) -> None:
                self.velocities.append([float(item) for item in value])

            def getPosition(self) -> list[float]:
                return [0.0, 0.0, contract.REFERENCE_HEIGHT]

            @staticmethod
            def getOrientation() -> list[float]:
                return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        class Motor:
            def __init__(self) -> None:
                self.target = 0.0
                self.torque = 0.0
                self.available = 0.0

            def setAvailableTorque(self, value: float) -> None:
                self.available = float(value)

            def setPosition(self, value: float) -> None:
                self.target = float(value)

            def getTargetPosition(self) -> float:
                return self.target

            def getAvailableTorque(self) -> float:
                return self.available

        class Sensor:
            def __init__(self, index: int) -> None:
                self.index = index

            def getValue(self) -> float:
                return contract.rsi_targets()[self.index]

        class Robot:
            @staticmethod
            def getRoot() -> object:
                raise RuntimeError("无需 Webots 字段即可覆盖首次 reset")

            @staticmethod
            def step(timestep: int) -> int:
                self_assert = 4
                assert timestep == self_assert
                return 0

        controller = object.__new__(RlAgentController)
        node = Node()
        controller.robot = Robot()
        controller.node = node
        controller.spawn_position = contract.BIRTH_POSITIONS[0]
        controller.motors = [Motor() for _ in range(contract.ACTION_DIM)]
        controller.sensors = [
            Sensor(index) for index in range(contract.ACTION_DIM)
        ]
        controller._motor_strength = 1.0
        controller.timestep = contract.WEBOTS_TIMESTEP_MS
        controller.initialization_error = ""
        controller.capability_errors = {}
        controller._resolver_context = ""
        controller._resolve_foot_shank_nodes = lambda: False
        controller._initialize_metric_capabilities()
        self.assertEqual(controller.initialization_error, "")
        capability_reports = "\n".join(controller.capability_error_reports())
        self.assertIn("torque_sampling_failed", capability_reports)
        self.assertIn("foot_shank_resolution_failed", capability_reports)
        with redirect_stdout(io.StringIO()):
            controller.reset_robot(
                {
                    "friction": 1.2,
                    "mass_scale": 1.0,
                    "external_impulse": [0.1, -0.2, 0.3],
                },
                reset_style="rsi",
                perturbation=[0.0] * contract.ACTION_DIM,
            )
        self.assertEqual(node.reset_count, 2)
        self.assertEqual(
            node.velocities[0],
            [0.1, -0.2, 0.3, 0.0, 0.0, 0.0],
        )
        self.assertTrue(
            all(value == [0.0] * 6 for value in node.velocities[1:])
        )
        self.assertAlmostEqual(
            node.translation.values[-1][2],
            contract.REFERENCE_HEIGHT,
        )

    def test_specific_initialization_and_connection_errors_are_visible(self) -> None:
        controller = RlAgentController(object())
        controller._fail_initialization(
            "motor_device_missing",
            "fr_abd_motor",
        )
        self.assertEqual(
            controller.initialization_failure_report(),
            "RL_CONTROLLER_ERROR motor_device_missing: fr_abd_motor",
        )
        self.assertIn(
            "RL_CONTROLLER_ERROR webots_connection_failed:",
            connection_failure_report(ConnectionError("bridge refused")),
        )

        controller.capability_errors = {}
        try:
            raise RuntimeError("torque sampling period mismatch")
        except RuntimeError as exc:
            controller._record_capability_error(
                "torque_sampling_failed",
                "扭矩反馈能力不可用",
                exc,
            )
        controller._record_capability_error(
            "foot_shank_resolution_failed",
            "未找到四个具名 shank 节点",
        )
        reports = "\n".join(controller.capability_error_reports())
        self.assertIn("RL_CAPABILITY_ERROR torque_sampling_failed", reports)
        self.assertIn("torque sampling period mismatch", reports)
        self.assertIn("Traceback", reports)
        self.assertIn(
            "RL_CAPABILITY_ERROR foot_shank_resolution_failed",
            reports,
        )

    def test_capability_missing_contact_is_zeroed_before_reward(self) -> None:
        env = MockEnv(phase="P1", randomization_mode="fixed")
        env.reset(seed=34)
        env.next_state = state(
            foot_contacts=[1.0, 1.0, 1.0, 1.0],
            contacts=[1.0, 1.0, 1.0, 1.0],
            foot_contact_source="capability_missing_fail_closed",
            position=list(contract.BIRTH_POSITIONS[env.worker_id]),
        )
        _, reward, _, _, info = env.step(np.zeros(12))
        np.testing.assert_array_equal(info["foot_contacts"], [0.0] * 4)
        self.assertEqual(
            info["foot_contact_source"],
            "capability_missing_fail_closed",
        )
        self.assertEqual(info["reward_parts"]["true_four_foot_contact"], 0.0)
        self.assertTrue(math.isfinite(reward))
        env.close()

    def test_true_contact_requires_toe_region_and_rejects_shank_box(
        self,
    ) -> None:
        class Shank:
            def __init__(self, node_id: int, velocity: list[float]) -> None:
                self.node_id = node_id
                self.velocity = velocity

            def getId(self) -> int:
                return self.node_id

            def getPosition(self) -> list[float]:
                return [0.0, 0.0, 0.0]

            @staticmethod
            def getOrientation() -> list[float]:
                return [
                    1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0,
                ]

            def getVelocity(self) -> list[float]:
                return self.velocity

        class Contact:
            def __init__(self, node_id: int, point: list[float]) -> None:
                self.node_id = node_id
                self.point = point

            def getNodeId(self) -> int:
                return self.node_id

            def getPoint(self) -> list[float]:
                return self.point

        class BodyNode:
            @staticmethod
            def getOrientation() -> list[float]:
                return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

            @staticmethod
            def getPosition() -> list[float]:
                return [0.0, 0.0, contract.REFERENCE_HEIGHT]

        controller = object.__new__(RlAgentController)
        controller.node = BodyNode()
        shanks = {
            index: Shank(101 + index, [0.01 * (index + 1), 0.0, 0.0, 0.0, 0.0, 0.0])
            for index in range(4)
        }
        controller._foot_shank_nodes = dict(shanks)
        controller._foot_shank_node_ids = {
            index: node.getId() for index, node in shanks.items()
        }
        controller._previous_foot_points = [None] * 4
        controller.contact_points = lambda: [
            Contact(101, [0.0, 0.0, -0.09]),
            Contact(999, [0.18, -0.10, 0.0]),
        ]
        contacts, velocities, non_foot, body, source = (
            controller.foot_contacts_and_slip()
        )
        self.assertEqual(contacts, [1.0, 0.0, 0.0, 0.0])
        self.assertTrue(non_foot)
        self.assertTrue(body)
        self.assertEqual(source, "node_id")
        self.assertAlmostEqual(velocities[0], 0.01)

        # 同一 shank node 上的 Box 接触不得伪装成 toe。
        controller._previous_foot_points = [None] * 4
        controller.contact_points = lambda: [
            Contact(101, [0.015, 0.0, -0.02]),
        ]
        contacts, _, non_foot, _, source = controller.foot_contacts_and_slip()
        self.assertEqual(contacts, [0.0] * 4)
        self.assertTrue(non_foot)
        self.assertEqual(source, "unmapped_fail_closed")

        controller._foot_shank_node_ids = {}
        controller._foot_shank_nodes = {}
        controller._previous_foot_points = [None] * 4
        contacts, _, non_foot, _, source = controller.foot_contacts_and_slip()
        self.assertEqual(contacts, [0.0] * 4)
        self.assertTrue(non_foot)
        self.assertEqual(source, "unmapped_fail_closed")

        controller.contact_capability = False
        contacts, velocities, non_foot, _, source = (
            controller.foot_contacts_and_slip()
        )
        self.assertEqual(contacts, [0.0] * 4)
        self.assertEqual(velocities, [0.0] * 12)
        self.assertFalse(non_foot)
        self.assertEqual(source, "capability_missing_fail_closed")

    def test_resolver_maps_each_knee_motor_to_named_shank_node(self) -> None:
        class Field:
            def __init__(self, node: object | None = None) -> None:
                self.node = node

            @staticmethod
            def getSFString() -> str:
                return ""

            def getSFNode(self) -> object:
                assert self.node is not None
                return self.node

        class Shank:
            def __init__(self, node_id: int, name: str) -> None:
                self.node_id = node_id
                self._name = name

            def getId(self) -> int:
                return self.node_id

            def getField(self, name: str) -> Field | None:
                return Field() if name == "name" else None

            @property
            def name(self) -> str:
                return self._name

            def get_name_for_test(self) -> str:
                return self._name

        class NamedShank(Shank):
            def __init__(self, node_id: int, leg: str) -> None:
                super().__init__(node_id, f"{leg}_shank_link")
                self._name = f"{leg}_shank_link"

            def getField(self, name: str) -> Field | None:
                if name == "name":
                    return NamedNameField(self._name)
                return None

        class NamedNameField:
            def __init__(self, value: str) -> None:
                self.value = value

            @staticmethod
            def getSFString() -> str:
                return ""

            def getSFString(self) -> str:  # type: ignore[no-redef]
                return self.value

        class Joint:
            def __init__(self, shank: object) -> None:
                self.shank = shank

            @staticmethod
            def getField(name: str) -> Field | None:
                return None

            def getField(self, name: str) -> Field | None:  # type: ignore[no-redef]
                return Field(self.shank) if name == "endPoint" else None

            @staticmethod
            def getParentNode() -> None:
                return None

        class Motor:
            def __init__(self, index: int) -> None:
                self.index = index
                self._tag = 1000 + index

        shanks = [
            NamedShank(201 + index, leg)
            for index, leg in enumerate(contract.LEG_NAMES)
        ]

        class Robot:
            @staticmethod
            def getFromDevice(tag: int) -> Joint:
                if not isinstance(tag, int):
                    raise TypeError("getFromDevice 只接受整数 device tag")
                return Joint(shanks[(tag - 1000) // 3])

        controller = object.__new__(RlAgentController)
        controller.robot = Robot()
        controller.motors = [Motor(index) for index in range(12)]
        self.assertTrue(controller._resolve_foot_shank_nodes())
        self.assertEqual(
            controller._foot_shank_node_ids,
            {0: 201, 1: 202, 2: 203, 3: 204},
        )
        self.assertEqual(
            [controller._device_tag(motor) for motor in controller.motors[2::3]],
            [1002, 1005, 1008, 1011],
        )
        with self.assertRaises(TypeError):
            controller._device_tag(object())

    def test_rsi_reset_rejects_height_posture_and_joint_error_boundaries(
        self,
    ) -> None:
        class Node:
            def __init__(self) -> None:
                self.height = contract.REFERENCE_HEIGHT
                self.orientation = [
                    1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0,
                ]

            def getPosition(self) -> list[float]:
                return [0.0, 0.0, self.height]

            def getOrientation(self) -> list[float]:
                return list(self.orientation)

        class Sensor:
            def __init__(self, index: int, values: list[float]) -> None:
                self.index = index
                self.values = values

            def getValue(self) -> float:
                return self.values[self.index]

        controller = object.__new__(RlAgentController)
        controller.node = Node()
        targets = list(contract.rsi_targets())
        positions = list(targets)
        controller.sensors = [
            Sensor(index, positions) for index in range(contract.ACTION_DIM)
        ]
        controller._validate_rsi_state(
            targets,
            stage="rsi_valid",
            check_height=True,
        )

        controller.node.height = contract.REFERENCE_HEIGHT + 0.031
        with self.assertRaisesRegex(RuntimeError, "height="):
            controller._validate_rsi_state(
                targets,
                stage="rsi_height_boundary",
                check_height=True,
            )
        controller.node.height = contract.REFERENCE_HEIGHT

        tilted = math.sin(0.081)
        controller.node.orientation = [
            1.0, 0.0, 0.0,
            0.0, math.cos(0.081), -tilted,
            0.0, tilted, math.cos(0.081),
        ]
        with self.assertRaisesRegex(RuntimeError, "posture="):
            controller._validate_rsi_state(
                targets,
                stage="rsi_posture_boundary",
                check_height=True,
            )
        controller.node.orientation = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]

        positions[0] += 0.051
        with self.assertRaisesRegex(RuntimeError, "joint_error="):
            controller._validate_rsi_state(
                targets,
                stage="rsi_joint_boundary",
                check_height=True,
            )

    def test_state_uses_nonzero_torque_feedback_sample(self) -> None:
        class Motor:
            def __init__(self, torque: float) -> None:
                self.torque = torque

            def getTorqueFeedback(self) -> float:
                return self.torque

            @staticmethod
            def getTargetPosition() -> float:
                return 0.0

        class Node:
            @staticmethod
            def getOrientation() -> list[float]:
                return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

            @staticmethod
            def getVelocity() -> list[float]:
                return [0.0] * 6

            @staticmethod
            def getPosition() -> list[float]:
                return [0.0, 0.0, contract.REFERENCE_HEIGHT]

        class Gyro:
            @staticmethod
            def getValues() -> list[float]:
                return [0.0, 0.0, 0.0]

        class Robot:
            @staticmethod
            def getTime() -> float:
                return 0.0

        controller = object.__new__(RlAgentController)
        controller.read_feedback = lambda: (
            contract.DEFAULT_CROUCH,
            (0.0,) * 12,
            True,
        )
        controller.node = Node()
        controller.gyro = Gyro()
        controller.robot = Robot()
        controller.motors = [
            Motor(0.5 + index * 0.1) for index in range(contract.ACTION_DIM)
        ]
        controller.foot_contacts_and_slip = lambda: (
            [1.0] * 4,
            [0.0] * 12,
            False,
            False,
            "node_id",
        )
        controller._jump_started = 0.0
        controller._jump_latched = False
        controller._jump_success_event = False
        controller._jump_landing_event = False
        controller._torque_diagnostic_printed = True
        sample = controller.state()
        self.assertEqual(sample["torque_source"], "torque_feedback")
        self.assertTrue(all(value > 0.0 for value in sample["joint_torques"]))

    def test_friction_readback_reports_toe_values_and_unknown_errors(self) -> None:
        import io
        import json
        from contextlib import redirect_stdout

        class FloatField:
            def __init__(self, value: float, *, fail_read: bool = False) -> None:
                self.value = float(value)
                self.fail_read = fail_read

            @staticmethod
            def getCount() -> int:
                return 1

            def getMFFloat(self, index: int) -> float:
                if self.fail_read:
                    raise RuntimeError("coulombFriction read unsupported")
                if index != 0:
                    raise IndexError(index)
                return self.value

            def setMFFloat(self, index: int, value: float) -> None:
                if index != 0:
                    raise IndexError(index)
                self.value = float(value)

        class StringField:
            def __init__(self, value: str) -> None:
                self.value = value

            @staticmethod
            def getSFString() -> str:
                return ""

            def getSFString(self) -> str:  # type: ignore[no-redef]
                return self.value

        class Contact:
            def __init__(self, material1: str, material2: str, friction: float, *, fail_read: bool = False) -> None:
                self.fields = {
                    "material1": StringField(material1),
                    "material2": StringField(material2),
                    "coulombFriction": FloatField(friction, fail_read=fail_read),
                }

            @staticmethod
            def getField(name: str) -> object:
                return None

            def getField(self, name: str) -> object:  # type: ignore[no-redef]
                if name not in self.fields:
                    raise KeyError(name)
                return self.fields[name]

        class Contacts:
            def __init__(self, nodes: list[Contact]) -> None:
                self.nodes = nodes

            @staticmethod
            def getCount() -> int:
                return 0

            def getCount(self) -> int:  # type: ignore[no-redef]
                return len(self.nodes)

            @staticmethod
            def getMFNode(index: int) -> Contact:
                raise IndexError(index)

            def getMFNode(self, index: int) -> Contact:  # type: ignore[no-redef]
                return self.nodes[index]

        class Field:
            def __init__(self, value: object) -> None:
                self.value = value

            @staticmethod
            def getSFNode() -> object:
                return None

            def getSFNode(self) -> object:  # type: ignore[no-redef]
                return self.value

        class Root:
            def __init__(self, world_info: object) -> None:
                self.world_info = world_info

            @staticmethod
            def getField(name: str) -> object:
                return None

            def getField(self, name: str) -> object:  # type: ignore[no-redef]
                if name != "worldInfo":
                    raise KeyError(name)
                return Field(self.world_info)

        class Robot:
            def __init__(self, root: Root) -> None:
                self.root = root

            @staticmethod
            def getRoot() -> object:
                return None

            def getRoot(self) -> object:  # type: ignore[no-redef]
                return self.root

        class MassField:
            def __init__(self) -> None:
                self.value = 3.3

            @staticmethod
            def getSFFloat() -> float:
                return 0.0

            def getSFFloat(self) -> float:  # type: ignore[no-redef]
                return self.value

            @staticmethod
            def setSFFloat(value: float) -> None:
                raise RuntimeError("mass write unsupported")

            def setSFFloat(self, value: float) -> None:  # type: ignore[no-redef]
                self.value = float(value)

        class Physics:
            def __init__(self, mass: MassField) -> None:
                self.mass = mass

            @staticmethod
            def getField(name: str) -> object:
                return None

            def getField(self, name: str) -> object:  # type: ignore[no-redef]
                if name != "mass":
                    raise KeyError(name)
                return self.mass

        class PhysicsField:
            def __init__(self, physics: Physics) -> None:
                self.physics = physics

            @staticmethod
            def getSFNode() -> object:
                return None

            def getSFNode(self) -> object:  # type: ignore[no-redef]
                return self.physics

        class Node:
            def __init__(self, physics: Physics) -> None:
                self.physics = physics

            @staticmethod
            def getField(name: str) -> object:
                return None

            def getField(self, name: str) -> object:  # type: ignore[no-redef]
                if name != "physics":
                    raise KeyError(name)
                return PhysicsField(self.physics)

        toe_names = ["fr_toe", "fl_toe", "hr_toe", "hl_toe"]
        contacts = Contacts(
            [Contact(name, "default", 0.9) for name in toe_names]
            + [Contact("default", "default", 0.8)]
        )
        world_info = type("WorldInfo", (), {})()
        world_info.getField = lambda name: contacts if name == "contactProperties" else (_ for _ in ()).throw(KeyError(name))
        controller = object.__new__(RlAgentController)
        controller.robot = Robot(Root(world_info))
        controller.node = Node(Physics(MassField()))

        buffer = io.StringIO()
        with redirect_stdout(buffer):
            controller.apply_randomization(
                {"friction": 1.2, "mass_scale": 1.0, "motor_strength": 1.0}
            )
        output = buffer.getvalue()
        readback_lines = [
            json.loads(line.split(" ", 1)[1])
            for line in output.splitlines()
            if line.startswith("RL_FRICTION_READBACK ")
        ]
        self.assertEqual(len(readback_lines), 1)
        report = readback_lines[0]
        self.assertEqual(report["status"], "OK")
        self.assertEqual(len(report["before"]["items"]), 4)
        self.assertEqual(len(report["after"]["items"]), 4)
        self.assertTrue(all(item["material1"] in toe_names for item in report["before"]["items"]))
        self.assertTrue(all(item["material2"] == "default" for item in report["after"]["items"]))
        self.assertTrue(all(item["readback"] == 0.9 for item in report["before"]["items"]))
        self.assertTrue(all(item["readback"] == 1.2 for item in report["after"]["items"]))
        self.assertTrue(all(item["match"] for item in report["after"]["items"]))

        # 读 API 不可用时必须输出 UNKNOWN 和异常，不得静默视为成功。
        bad_contacts = Contacts(
            [Contact(name, "default", 1.2, fail_read=True) for name in toe_names]
        )
        bad_world_info = type("WorldInfo", (), {})()
        bad_world_info.getField = lambda name: bad_contacts if name == "contactProperties" else (_ for _ in ()).throw(KeyError(name))
        controller.robot = Robot(Root(bad_world_info))
        buffer = io.StringIO()
        with redirect_stdout(buffer):
            controller.apply_randomization(
                {"friction": 1.2, "mass_scale": 1.0, "motor_strength": 1.0}
            )
        output = buffer.getvalue()
        self.assertIn("RL_FRICTION_READBACK_ERROR", output)
        error_report = json.loads(
            next(
                line.split(" ", 1)[1]
                for line in output.splitlines()
                if line.startswith("RL_FRICTION_READBACK_ERROR ")
            )
        )
        self.assertEqual(error_report["status"], "UNKNOWN")
        self.assertTrue(error_report["errors"])
        self.assertTrue(
            any(
                item.get("error_type") == "RuntimeError"
                for item in error_report["errors"]
            )
        )


class ProtocolAndInputTests(unittest.TestCase):
    def test_tcp_roundtrip_and_validation(self) -> None:
        payload = {"type": "act", "action": [0.0] * 12, "command": [0.0] * 3, "jump_request": False}
        self.assertEqual(decode(encode(payload)), payload)
        validate_message(payload, "act")
        with self.assertRaises(ValueError):
            validate_message({"type": "act", "action": [math.nan], "command": [], "jump_request": False}, "act")

    def test_keyboard_and_safety(self) -> None:
        adapter = InputAdapter()
        frame = adapter.update(keys=[ord("w")], axes=[0, 0, 0], buttons=[0, 0], now=1.0, last_input_at=1.0)
        self.assertEqual(frame.command[0], 0.6)
        jump1 = adapter.update(keys=[ord(" ")], axes=[0, 0, 0], buttons=[0, 0], now=1.1, last_input_at=1.1)
        jump2 = adapter.update(keys=[ord(" ")], axes=[0, 0, 0], buttons=[0, 0], now=1.2, last_input_at=1.2)
        self.assertTrue(jump1.jump_edge)
        self.assertFalse(jump2.jump_edge)
        timeout = adapter.update(keys=[], axes=[0, 0, 0], buttons=[0, 0], now=2.0, last_input_at=1.0)
        self.assertTrue(timeout.safe)


if __name__ == "__main__":
    unittest.main(verbosity=2)
