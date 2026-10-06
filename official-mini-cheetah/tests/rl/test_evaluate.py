#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""R3 物理指标、能力 Gate、双策略模式与评估超时测试。"""

from __future__ import annotations

import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rl import contract
from rl.evaluate import (
    _episode,
    _state_sample,
    case_telemetry,
    capability_gate,
    configure_eval_connection,
    engineering_gate,
    physical_gate,
    p0_four_robot_handshake,
    run_p1,
    start_eval_environment,
)


def state(**changes: object) -> dict[str, object]:
    """构造包含真实足端通道的完整物理状态。"""
    value: dict[str, object] = {
        "q": [0.0] * 12,
        "dq": [0.0] * 12,
        "rpy": [0.0, 0.0, 0.0],
        "v_body": [0.0, 0.0, 0.0],
        "omega_body": [0.0, 0.0, 0.0],
        "foot_contacts": [1.0] * 4,
        "foot_velocities": [[0.0, 0.0, 0.0]] * 4,
        "height": contract.REFERENCE_HEIGHT,
        "position": [0.0, 0.0, contract.REFERENCE_HEIGHT],
        "jump_phase": 0.0,
        "jump_success": False,
        "jump_landing": False,
        "done": False,
    }
    value.update(changes)
    return value


class RecordingModel:
    """记录 deterministic 参数的 12 维策略替身。"""

    def __init__(self) -> None:
        self.calls: list[bool] = []

    def predict(self, observation: np.ndarray, deterministic: bool) -> tuple[np.ndarray, None]:
        self.assertEqual_shape(observation)
        self.calls.append(bool(deterministic))
        return np.zeros(contract.ACTION_DIM, dtype=np.float32), None

    @staticmethod
    def assertEqual_shape(observation: np.ndarray) -> None:
        if observation.shape != (contract.OBS_DIM,):
            raise AssertionError("观测维度错误")


class FakeEnv:
    """无 Webots 的固定物理轨迹环境。"""

    max_episode_steps = 4

    def __init__(self, *, moving: tuple[float, float, float] | None = None) -> None:
        self.index = 0
        self.moving = moving
        self._last_state: dict[str, object] = {}
        self._command = np.zeros(3, dtype=np.float32)
        self.prepared_jump: list[bool] = []

    def reset(self, *, seed: int, options: object) -> tuple[np.ndarray, dict[str, object]]:
        del seed, options
        self.index = 0
        self._command = np.asarray(self.moving or (0.0, 0.0, 0.0), dtype=np.float32)
        self._last_state = state()
        return np.zeros(contract.OBS_DIM, dtype=np.float32), {}

    def prepare_step(self, action: np.ndarray) -> None:
        del action
        self._pending_step = {"type": "act", "jump_request": False}

    def send_prepared_step(self) -> None:
        self.prepared_jump.append(bool(self._pending_step["jump_request"]))

    def finish_step(self) -> tuple[np.ndarray, float, bool, bool, dict[str, object]]:
        return self.step(np.zeros(contract.ACTION_DIM, dtype=np.float32))

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict[str, object]]:
        del action
        self.index += 1
        velocity = self.moving or (0.0, 0.0, 0.0)
        self._last_state = state(
            v_body=list(velocity),
            omega_body=[0.0, 0.0, float(velocity[2])],
            position=[
                float(velocity[0]) * self.index * contract.CONTROL_DT_SECONDS,
                float(velocity[1]) * self.index * contract.CONTROL_DT_SECONDS,
                contract.REFERENCE_HEIGHT,
            ],
        )
        truncated = self.index >= self.max_episode_steps
        return (
            np.zeros(contract.OBS_DIM, dtype=np.float32),
            0.0,
            False,
            truncated,
            {},
        )


def episode(**overrides: object) -> dict[str, object]:
    """构造通过 P1 物理 Gate 的 episode 汇总。"""
    value: dict[str, object] = {
        "finite": True,
        "fallen": False,
        "height_mean": contract.REFERENCE_HEIGHT,
        "height_p05": contract.REFERENCE_HEIGHT - 0.01,
        "height_p95": contract.REFERENCE_HEIGHT + 0.01,
        "posture_abs_mean": 0.03,
        "posture_abs_p95": 0.08,
        "drift_m": 0.02,
        "speed_error_mean": 0.02,
        "yaw_rate_abs_mean": 0.02,
        "true_four_contact_ratio": 1.0,
        "foot_slip_mean": 0.02,
        "action_delta_rms": 0.01,
        "joint_jitter_rms": 0.10,
        "mean_vx": 0.0,
        "mean_vy": 0.0,
        "mean_wz": 0.0,
        "planar_speed_abs_mean": 0.01,
        "jump_success": 0,
        "jump_landing": 0,
        "peak_height_gain_mean": 0.0,
        "post_landing_height_mean": contract.REFERENCE_HEIGHT,
        "steps": 100,
        "reward": -1.0,
        "technical_failure": False,
        "technical_failure_reasons": [],
        "foot_contact_source": "node_id",
        "observation_dim": contract.OBS_DIM,
        "action_dim": contract.ACTION_DIM,
    }
    value.update(overrides)
    return value


class EvaluationMetricTests(unittest.TestCase):
    def test_episode_records_real_contact_slip_and_jitter(self) -> None:
        report = _episode(
            RecordingModel(),
            FakeEnv(),
            seed=1,
            command=(0.0, 0.0, 0.0),
            deterministic=True,
        )
        self.assertEqual(report["steps"], 4)
        self.assertEqual(report["true_four_contact_ratio"], 1.0)
        self.assertEqual(report["foot_slip_source"], "foot_velocities")
        self.assertAlmostEqual(report["foot_slip_mean"], 0.0)
        self.assertAlmostEqual(report["action_delta_rms"], 0.0)
        self.assertAlmostEqual(report["joint_jitter_rms"], 0.0)
        self.assertTrue(report["finite"])

    def test_raw_displacement_and_planned_residual_are_separate(self) -> None:
        moving = _episode(
            RecordingModel(),
            FakeEnv(moving=(0.25, 0.0, 0.0)),
            seed=2,
            command=(0.25, 0.0, 0.0),
            deterministic=True,
        )
        self.assertEqual(moving["drift_component"], "planned_residual")
        self.assertAlmostEqual(
            moving["planned_displacement_m"],
            moving["planned_residual_m"] + moving["raw_displacement_m"],
            delta=0.02,
        )
        self.assertIn("raw_displacement_m", moving)
        self.assertIn("planned_residual_m", moving)

        stopped = _episode(
            RecordingModel(),
            FakeEnv(),
            seed=3,
            command=(0.0, 0.0, 0.0),
            deterministic=True,
        )
        self.assertEqual(stopped["drift_component"], "raw_displacement")
        self.assertEqual(stopped["drift_m"], stopped["raw_displacement_m"])

        telemetry = case_telemetry(
            [
                episode(
                    raw_displacement_m=5.04,
                    planned_displacement_m=10.0,
                    planned_residual_m=4.96,
                    drift_m=4.96,
                ),
                episode(
                    raw_displacement_m=0.01,
                    planned_displacement_m=0.0,
                    planned_residual_m=0.01,
                    drift_m=0.01,
                ),
            ],
            ["forward", "stop"],
        )
        self.assertEqual(
            telemetry["forward"]["raw_displacement_m"],
            5.04,
        )
        self.assertEqual(
            telemetry["forward"]["planned_residual_m"],
            4.96,
        )
        self.assertEqual(telemetry["stop"]["drift_m"], 0.01)

    def test_height_action_and_joint_jitter_use_contract_units(self) -> None:
        previous = state(dq=[0.10] * 12, height=contract.REFERENCE_HEIGHT)
        current = state(dq=[0.14] * 12, height=contract.REFERENCE_HEIGHT)
        sample = _state_sample(
            current,
            previous,
            np.full(12, 0.02, dtype=np.float64),
            np.zeros(12, dtype=np.float64),
        )
        self.assertAlmostEqual(sample["height"], contract.REFERENCE_HEIGHT)
        np.testing.assert_allclose(sample["action_delta"], 0.02)
        np.testing.assert_allclose(sample["joint_jitter"], 0.04)
        report = physical_gate(
            [
                episode(
                    height_mean=contract.REFERENCE_HEIGHT,
                    height_p05=contract.REFERENCE_HEIGHT - 0.01,
                    height_p95=contract.REFERENCE_HEIGHT + 0.01,
                    action_delta_rms=0.02,
                    joint_jitter_rms=0.04,
                )
            ]
        )
        self.assertTrue(report["passed"], report)

    def test_aggregate_contact_is_not_true_four_foot_contact(self) -> None:
        class AggregateEnv(FakeEnv):
            def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict[str, object]]:
                del action
                self.index += 1
                base = state(contacts=[1.0] * 4)
                base.pop("foot_contacts", None)
                self._last_state = base
                return (
                    np.zeros(contract.OBS_DIM, dtype=np.float32),
                    0.0,
                    False,
                    self.index >= self.max_episode_steps,
                    {},
                )

        report = _episode(
            RecordingModel(),
            AggregateEnv(),
            seed=1,
            command=(0.0, 0.0, 0.0),
            deterministic=True,
        )
        self.assertEqual(report["four_contact_ratio"], 1.0)
        self.assertEqual(report["true_four_contact_ratio"], 0.0)
        self.assertFalse(physical_gate([report])["passed"])

    def test_explicit_jump_request_is_injected_before_tcp_send(self) -> None:
        env = FakeEnv()
        report = _episode(
            RecordingModel(),
            env,
            seed=1,
            command=(0.0, 0.0, 0.0),
            deterministic=True,
            jump_steps=(2,),
            max_steps=3,
        )
        self.assertEqual(env.prepared_jump, [False, True, False])
        self.assertEqual(report["jump_requests"], 1)

    def test_p1_requires_deterministic_and_stochastic(self) -> None:
        model = RecordingModel()
        args = types.SimpleNamespace(episodes=1, seed=1, max_steps=2)
        report = run_p1(model, FakeEnv(), args)
        self.assertTrue(report["passed"])
        self.assertEqual(model.calls, [True, True, False, False])
        self.assertEqual(report["policy_modes_required"], ["deterministic", "stochastic"])

    def test_p0_fixed_randomization_for_p0_and_p1(self) -> None:
        class FakeEvalEnv:
            instances: list["FakeEvalEnv"] = []

            def __init__(self, **kwargs: object) -> None:
                self.randomization_mode = kwargs["randomization_mode"]
                self.phase = kwargs["phase"]
                self.__class__.instances.append(self)

            @staticmethod
            def open_bridge() -> None:
                return None

            @staticmethod
            def launch_webots() -> None:
                return None

            @staticmethod
            def verify_webots_supervisor_port(*, timeout: float) -> None:
                del timeout

            @staticmethod
            def launch_controller() -> None:
                return None

            @staticmethod
            def accept_bridge() -> None:
                return None

            @staticmethod
            def close() -> None:
                return None

        for phase in ("P0", "P1"):
            args = types.SimpleNamespace(
                phase=phase,
                bridge_port=contract.BRIDGE_PORT,
                max_steps=10,
                connect_timeout=1.0,
                socket_timeout=2.0,
            )
            with (
                patch("rl.evaluate.require_ports_free"),
                patch("rl.env.MiniCheetahFlatJumpEnv", FakeEvalEnv),
            ):
                start_eval_environment(args)
            created = FakeEvalEnv.instances[-1]
            self.assertEqual(created.phase, phase)
            self.assertEqual(created.randomization_mode, "fixed")

    def test_short_or_invalid_reset_is_technical_and_excluded(self) -> None:
        class ShortResetEnv(FakeEnv):
            phase = "P0"

            def reset(self, *, seed: int, options: object) -> tuple[np.ndarray, dict[str, object]]:
                super().reset(seed=seed, options=options)
                self._last_state = state(
                    height=1.543,
                    position=[0.0, 0.0, 1.543],
                    q=list(contract.rsi_targets()),
                    foot_contact_source="node_id",
                )
                return np.zeros(contract.OBS_DIM, dtype=np.float32), {}

            def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict[str, object]]:
                self.index += 1
                self._last_state = state(
                    height=1.655,
                    position=[0.0, 0.0, 1.655],
                    q=list(contract.rsi_targets()),
                    foot_contact_source="node_id",
                )
                return (
                    np.zeros(contract.OBS_DIM, dtype=np.float32),
                    -1.0,
                    True,
                    False,
                    {"fallen": True},
                )

        invalid = _episode(
            RecordingModel(),
            ShortResetEnv(),
            seed=1,
            command=(0.0, 0.0, 0.0),
            deterministic=True,
        )
        self.assertTrue(invalid["technical_failure"])
        self.assertTrue(
            any("reset_height" in reason for reason in invalid["technical_failure_reasons"])
        )
        self.assertTrue(
            any(reason.startswith("steps=") for reason in invalid["technical_failure_reasons"])
        )
        valid = episode()
        report = physical_gate([invalid, valid])
        self.assertFalse(report["passed"])
        self.assertTrue(report["technical_failure"])
        self.assertEqual(report["technical_failure_count"], 1)
        self.assertEqual(report["valid_episode_count"], 1)
        self.assertEqual(report["values"]["height_mean"], valid["height_mean"])

    def test_p0_engineering_gate_is_not_p1_physical_gate(self) -> None:
        engineering = engineering_gate(
            [episode(height_mean=1.655, height_p05=1.655, height_p95=1.655)],
            four_robot_handshake=True,
        )
        self.assertEqual(engineering["gate_type"], "engineering")
        self.assertTrue(engineering["passed"], engineering)
        physical = physical_gate(
            [episode(height_mean=1.655, height_p05=1.655, height_p95=1.655)]
        )
        self.assertFalse(physical["passed"])
        self.assertFalse(physical["checks"]["height_mean"])
        missing_handshake = engineering_gate([episode()], four_robot_handshake=False)
        self.assertFalse(missing_handshake["passed"])
        self.assertTrue(missing_handshake["technical_failure"])

    def test_p0_handshake_requires_matching_four_worker_evidence(self) -> None:
        import json
        import tempfile

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            workers = {
                str(index): {
                    "connected": True,
                    "worker_id": True,
                    "robot_name": True,
                    "timestep": True,
                    "current": True,
                }
                for index in range(contract.PARALLEL_WORKERS)
            }
            health = {"passed": True, "run_started_at": 100.0, "worker_logs": workers}
            (root / "health_P0_live.json").write_text(
                json.dumps(health), encoding="utf-8"
            )
            (root / "P0_run_started_at.txt").write_text("100\n", encoding="utf-8")
            with patch("rl.evaluate.contract.LOG_ROOT", root):
                self.assertTrue(p0_four_robot_handshake())
                (root / "P0_run_started_at.txt").write_text("200\n", encoding="utf-8")
                self.assertFalse(p0_four_robot_handshake())

    def test_move_and_jump_capability_gates(self) -> None:
        cases = ["stop", "forward", "left", "right", "forward_left", "forward_right", "moving_jump"]
        episodes = [
            episode(mean_vx=0.0, mean_vy=0.0, mean_wz=0.0, planar_speed_abs_mean=0.01),
            episode(mean_vx=0.32, speed_error_mean=0.06),
            episode(mean_vy=0.22, speed_error_mean=0.05),
            episode(mean_vy=-0.22, speed_error_mean=0.05),
            episode(mean_vx=0.28, mean_wz=0.55, planar_speed_abs_mean=0.20),
            episode(mean_vx=0.28, mean_wz=-0.55, planar_speed_abs_mean=0.20),
            episode(
                mean_vx=0.32,
                jump_success=1,
                jump_landing=1,
                peak_height_gain_mean=0.055,
                post_landing_height_mean=contract.REFERENCE_HEIGHT,
            ),
        ]
        report = capability_gate(episodes, cases)
        self.assertTrue(report["passed"], report)
        failed = capability_gate(
            [episode(mean_vx=0.0, jump_success=0, jump_landing=0)],
            ["moving_jump"],
        )
        self.assertFalse(failed["passed"])

    def test_eval_network_timeouts_are_explicit(self) -> None:
        class Socket:
            def __init__(self) -> None:
                self.values: list[float] = []

            def settimeout(self, value: float) -> None:
                self.values.append(float(value))

        env = types.SimpleNamespace(_server=Socket(), _conn=Socket())
        configure_eval_connection(env, connect_timeout=2.0, socket_timeout=3.0)
        self.assertEqual(env._server.values, [2.0])
        self.assertEqual(env._conn.values, [3.0])
        with self.assertRaises(ValueError):
            configure_eval_connection(env, connect_timeout=0.0, socket_timeout=3.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
