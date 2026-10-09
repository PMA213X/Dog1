#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""R3 物理指标、能力 Gate、双策略模式与评估超时测试。"""

from __future__ import annotations

import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch

from gymnasium import spaces
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rl import contract
from rl.evaluate import (
    RAPID_FINETUNE_THRESHOLDS,
    RapidEvaluationContext,
    _episode,
    _predict_action,
    _state_sample,
    baseline_evaluation_due,
    case_telemetry,
    capability_gate,
    compare_rapid_finetune_to_baseline,
    configure_eval_connection,
    engineering_gate,
    is_rapid_model,
    load_model,
    physical_gate,
    p0_four_robot_handshake,
    rapid_finetune_gate,
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

    device = "cpu"
    observation_space = spaces.Box(
        low=-np.inf,
        high=np.inf,
        shape=(contract.OBS_DIM,),
        dtype=np.float32,
    )
    action_space = spaces.Box(
        low=-1.0,
        high=1.0,
        shape=(contract.ACTION_DIM,),
        dtype=np.float32,
    )

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


class RapidRecordingModel:
    """返回 source Box(-100,100) 动作并记录 Dict 观测的替身。"""

    device = "cpu"

    observation_space = spaces.Dict(
        {
            "current": spaces.Box(
                low=-np.inf,
                high=np.inf,
                shape=(42,),
                dtype=np.float32,
            ),
            "history": spaces.Box(
                low=-np.inf,
                high=np.inf,
                shape=(630,),
                dtype=np.float32,
            ),
        }
    )
    action_space = spaces.Box(
        low=-100.0,
        high=100.0,
        shape=(contract.ACTION_DIM,),
        dtype=np.float32,
    )

    def __init__(self, source_action: Sequence[float] | None = None) -> None:
        self.source_action = np.asarray(
            source_action if source_action is not None else [100.0] * 12,
            dtype=np.float32,
        )
        self.observations: list[dict[str, np.ndarray]] = []
        self.deterministic: list[bool] = []

    def predict(
        self,
        observation: dict[str, np.ndarray],
        deterministic: bool,
    ) -> tuple[np.ndarray, None]:
        if set(observation) != {"current", "history"}:
            raise AssertionError("Rapid 模型观测必须是 current/history Dict")
        if observation["current"].shape != (42,):
            raise AssertionError("Rapid current 维度错误")
        if observation["history"].shape != (630,):
            raise AssertionError("Rapid history 维度错误")
        self.observations.append(
            {key: np.asarray(value).copy() for key, value in observation.items()}
        )
        self.deterministic.append(bool(deterministic))
        return self.source_action.copy(), None


class FakeEnv:
    """无 Webots 的固定物理轨迹环境。"""

    max_episode_steps = 4

    def __init__(self, *, moving: tuple[float, float, float] | None = None) -> None:
        self.index = 0
        self.moving = moving
        self._last_state: dict[str, object] = {}
        self._command = np.zeros(3, dtype=np.float32)
        self.prepared_jump: list[bool] = []
        self.received_actions: list[np.ndarray] = []

    def reset(self, *, seed: int, options: object) -> tuple[np.ndarray, dict[str, object]]:
        del seed, options
        self.index = 0
        self._command = np.asarray(self.moving or (0.0, 0.0, 0.0), dtype=np.float32)
        self._last_state = state()
        return np.zeros(contract.OBS_DIM, dtype=np.float32), {}

    def prepare_step(self, action: np.ndarray) -> None:
        self._pending_step = {
            "type": "act",
            "jump_request": False,
            "action": np.asarray(action, dtype=np.float64).copy(),
        }

    def send_prepared_step(self) -> None:
        self.prepared_jump.append(bool(self._pending_step["jump_request"]))

    def finish_step(self) -> tuple[np.ndarray, float, bool, bool, dict[str, object]]:
        return self.step(self._pending_step["action"])

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict[str, object]]:
        self.received_actions.append(np.asarray(action, dtype=np.float64).copy())
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

    def test_rapid_predict_uses_core_mapper_and_history(self) -> None:
        source = np.asarray(
            [1.0, 2.0, 2.0] * 4,
            dtype=np.float32,
        )
        model = RapidRecordingModel(source)
        context = RapidEvaluationContext()
        observation = np.zeros(contract.OBS_DIM, dtype=np.float32)
        first = _predict_action(
            model,
            observation,
            deterministic=True,
            rapid_context=context,
        )
        expected = np.clip(
            source
            * np.asarray((0.125, 0.25, 0.25) * 4, dtype=np.float32)
            / np.asarray(contract.ACTION_SCALE, dtype=np.float32),
            -1.0,
            1.0,
        )
        np.testing.assert_allclose(first, expected, atol=1e-6)
        self.assertTrue(np.all(np.abs(first) <= 1.0))
        self.assertEqual(len(context.adapter._history), 1)
        np.testing.assert_allclose(
            context.last_source_action,
            source,
            atol=0.0,
        )
        second = _predict_action(
            model,
            observation,
            deterministic=True,
            rapid_context=context,
        )
        np.testing.assert_allclose(second, first, atol=1e-6)
        self.assertEqual(len(context.adapter._history), 2)
        np.testing.assert_allclose(
            model.observations[1]["current"][-12:],
            source,
            atol=0.0,
        )
        self.assertTrue(all(model.deterministic))

    def test_rapid_episode_sends_mapped_action_not_source_action(self) -> None:
        model = RapidRecordingModel([100.0] * contract.ACTION_DIM)
        env = FakeEnv()
        report = _episode(
            model,
            env,
            seed=1,
            command=(0.0, 0.0, 0.0),
            deterministic=True,
            max_steps=3,
            rapid_context=RapidEvaluationContext(),
        )
        self.assertEqual(report["steps"], 3)
        self.assertTrue(env.received_actions)
        self.assertTrue(
            all(
                np.all(np.abs(action) <= 1.0 + 1e-9)
                for action in env.received_actions
            )
        )
        self.assertTrue(
            any(np.any(np.abs(action) < 100.0) for action in env.received_actions)
        )
        self.assertFalse(np.any(np.abs(env.received_actions[0]) > 1.0))
        self.assertTrue(is_rapid_model(model))

    def test_load_model_accepts_rapid_and_rejects_legacy_in_rapid_gate(self) -> None:
        rapid_model = RapidRecordingModel()
        legacy_model = RecordingModel()
        checkpoint = str(
            contract.CHECKPOINT_ROOT
            / f"{contract.CONTRACT_VERSION}_final.zip"
        )
        with patch("stable_baselines3.PPO.load", return_value=rapid_model):
            loaded = load_model(
                checkpoint,
                device="cpu",
                gate_mode="rapid-finetune",
            )
        self.assertIs(loaded, rapid_model)
        with patch("stable_baselines3.PPO.load", return_value=legacy_model):
            with self.assertRaises(RuntimeError):
                load_model(
                    checkpoint,
                    device="cpu",
                    gate_mode="rapid-finetune",
                )
        with patch("stable_baselines3.PPO.load", return_value=legacy_model):
            loaded_legacy = load_model(checkpoint, device="cpu")
        self.assertIs(loaded_legacy, legacy_model)

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

    def test_case_telemetry_records_position_velocity_and_contacts(self) -> None:
        telemetry = case_telemetry(
            [
                episode(
                    position_x=1.2,
                    position_y=-0.3,
                    position_z=contract.REFERENCE_HEIGHT,
                    mean_vx=0.21,
                    mean_vy=0.01,
                    mean_vz=0.0,
                    raw_displacement_m=0.8,
                    average_contact_feet=2.75,
                    zero_contact_ratio=0.005,
                )
            ],
            ["forward"],
        )
        self.assertAlmostEqual(telemetry["forward"]["position_x"], 1.2)
        self.assertAlmostEqual(telemetry["forward"]["position_y"], -0.3)
        self.assertAlmostEqual(
            telemetry["forward"]["position_z"],
            contract.REFERENCE_HEIGHT,
        )
        self.assertAlmostEqual(telemetry["forward"]["mean_vx"], 0.21)
        self.assertAlmostEqual(telemetry["forward"]["mean_vy"], 0.01)
        self.assertAlmostEqual(telemetry["forward"]["mean_vz"], 0.0)
        self.assertAlmostEqual(
            telemetry["forward"]["actual_displacement_m"],
            0.8,
        )
        self.assertAlmostEqual(
            telemetry["forward"]["average_contact_feet"],
            2.75,
        )
        self.assertAlmostEqual(
            telemetry["forward"]["zero_contact_ratio"],
            0.005,
        )

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

    def test_rapid_finetune_stop_and_forward_gate(self) -> None:
        stop = episode(
            true_four_contact_ratio=1.0,
            average_contact_feet=4.0,
            zero_contact_ratio=0.0,
            planar_speed_abs_mean=0.01,
            mean_vx=0.0,
            speed_error_mean=0.01,
        )
        forward = episode(
            # 移动时只要求平均接触足，不要求四足同时落地。
            true_four_contact_ratio=0.0,
            average_contact_feet=2.60,
            zero_contact_ratio=0.005,
            planar_speed_abs_mean=0.22,
            mean_vx=0.22,
            mean_vy=0.01,
            speed_error_mean=0.07,
        )
        report = rapid_finetune_gate(
            [stop, forward],
            ["stop", "forward"],
        )
        self.assertTrue(report["passed"], report)
        self.assertEqual(report["gate_mode"], "rapid-finetune")
        self.assertTrue(report["checks"]["stop_four_contact"])
        self.assertTrue(report["checks"]["moving_contact"])
        self.assertTrue(report["checks"]["moving_zero_contact"])
        self.assertTrue(report["checks"]["forward_speed"])
        self.assertEqual(
            report["values"]["forward_average_contact_feet"],
            2.60,
        )
        self.assertEqual(
            report["values"]["forward_zero_contact_ratio"],
            0.005,
        )
        self.assertEqual(
            report["thresholds"]["moving_average_contact_feet_min"],
            RAPID_FINETUNE_THRESHOLDS["moving_average_contact_feet_min"],
        )

    def test_rapid_finetune_gate_rejects_case_threshold_misses(self) -> None:
        weak_stop = episode(
            true_four_contact_ratio=0.84,
            average_contact_feet=3.6,
            zero_contact_ratio=0.0,
            planar_speed_abs_mean=0.13,
        )
        weak_forward = episode(
            true_four_contact_ratio=1.0,
            average_contact_feet=2.40,
            zero_contact_ratio=0.02,
            planar_speed_abs_mean=0.17,
            mean_vx=0.17,
            speed_error_mean=0.21,
        )
        report = rapid_finetune_gate(
            [weak_stop, weak_forward],
            ["stop", "forward"],
        )
        self.assertFalse(report["passed"])
        self.assertFalse(report["checks"]["stop_four_contact"])
        self.assertFalse(report["checks"]["stop_speed"])
        self.assertFalse(report["checks"]["moving_contact"])
        self.assertFalse(report["checks"]["moving_zero_contact"])
        self.assertFalse(report["checks"]["forward_speed"])
        self.assertFalse(report["checks"]["forward_speed_error"])

    def test_rapid_finetune_requires_stop_and_forward(self) -> None:
        with self.assertRaises(ValueError):
            rapid_finetune_gate([episode()], ["stop"])

    def test_fixed_50k_baseline_comparison_hook(self) -> None:
        self.assertFalse(baseline_evaluation_due(50_000, "legacy"))
        self.assertFalse(baseline_evaluation_due(49_999, "rapid-finetune"))
        self.assertTrue(baseline_evaluation_due(50_000, "rapid-finetune"))
        self.assertTrue(baseline_evaluation_due(100_000, "rapid-finetune"))
        current = {
            "case_values": {
                "stop": {
                    "true_four_contact_ratio": 0.90,
                },
                "forward": {
                    "mean_vx": 0.24,
                    "average_contact_feet": 2.7,
                },
            }
        }
        frozen = {
            "case_values": {
                "stop": {
                    "true_four_contact_ratio": 0.95,
                },
                "forward": {
                    "mean_vx": 0.20,
                    "average_contact_feet": 3.0,
                },
            }
        }
        comparison = compare_rapid_finetune_to_baseline(
            current,
            frozen,
        )
        self.assertAlmostEqual(
            comparison["forward_speed_delta"],
            0.04,
        )
        self.assertAlmostEqual(
            comparison["forward_speed_improvement_ratio"],
            0.20,
        )
        self.assertTrue(comparison["improved"])
        self.assertEqual(comparison["baseline_interval_steps"], 50_000)

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
