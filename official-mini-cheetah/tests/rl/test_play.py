#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""R3 play 观测、确定性推理、安全站立和独立急停测试。"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
CONTROLLER_ROOT = ROOT / "controllers" / "rl_agent"
for path in (ROOT, CONTROLLER_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from input_adapter import InputAdapter, InputFrame
from rl import contract
from rl.play import EmergencyStop, ObservationBuilder, PlayPolicy, PlayRuntime


def state(**changes: object) -> dict[str, object]:
    """构造 play controller state。"""
    value: dict[str, object] = {
        "q": [0.1] * 12,
        "dq": [0.2] * 12,
        "rpy": [0.01, -0.02, 0.03],
        "v_body": [0.1, 0.2, 0.3],
        "omega_body": [0.4, 0.5, 0.6],
        "foot_contacts": [1.0] * 4,
        "height": contract.REFERENCE_HEIGHT,
        "jump_phase": 0.25,
    }
    value.update(changes)
    return value


class RecordingModel:
    """记录推理模式与观测，并返回有限动作。"""

    def __init__(self) -> None:
        self.calls: list[tuple[np.ndarray, bool]] = []

    def predict(self, observation: np.ndarray, deterministic: bool) -> tuple[np.ndarray, None]:
        self.calls.append((observation.copy(), bool(deterministic)))
        return np.full(contract.ACTION_DIM, 0.25, dtype=np.float32), None


class PlayTests(unittest.TestCase):
    def test_observation_is_57_dimensional_contract_layout(self) -> None:
        observation = ObservationBuilder().build(
            state(),
            command=(0.6, -0.3, 1.0),
            previous_action=np.arange(12, dtype=np.float32),
            jump_request=True,
        )
        self.assertEqual(observation.shape, (contract.OBS_DIM,))
        np.testing.assert_allclose(observation[contract.OBS_SLICES["cmd"]], [0.6, -0.3, 1.0])
        self.assertEqual(observation[contract.OBS_SLICES["jump_request"]][0], 1.0)
        self.assertEqual(observation[contract.OBS_SLICES["contacts"]].tolist(), [1.0] * 4)
        self.assertTrue(np.all(np.isfinite(observation)))

    def test_nonzero_command_uses_deterministic_prediction_and_joint_targets(self) -> None:
        model = RecordingModel()
        policy = PlayPolicy(model)
        result = policy.step(state(), command=(0.4, 0.0, 0.0))
        self.assertFalse(result.safe_stand)
        self.assertTrue(model.calls[0][1])
        self.assertEqual(result.action.shape, (contract.ACTION_DIM,))
        self.assertEqual(len(result.joint_targets), contract.ACTION_DIM)
        self.assertTrue(np.all(np.isfinite(result.joint_targets)))
        self.assertNotEqual(result.joint_targets, contract.DEFAULT_CROUCH)

    def test_zero_command_and_timeout_are_safe_stand_without_estop(self) -> None:
        model = RecordingModel()
        policy = PlayPolicy(model)
        zero = policy.step(state(), command=(0.0, 0.0, 0.0))
        timeout = policy.step(
            state(),
            command=(0.6, 0.0, 0.0),
            input_frame=InputFrame(safe=True, reason="input_timeout"),
        )
        self.assertTrue(zero.safe_stand)
        self.assertTrue(timeout.safe_stand)
        self.assertEqual(timeout.reason, "input_timeout")
        self.assertFalse(policy.emergency_stop.active)
        self.assertEqual(zero.joint_targets, contract.DEFAULT_CROUCH)
        self.assertEqual(timeout.joint_targets, contract.DEFAULT_CROUCH)
        self.assertFalse(model.calls)

    def test_jump_from_zero_command_does_not_force_safe_stand(self) -> None:
        result = PlayPolicy(RecordingModel()).step(
            state(),
            command=(0.0, 0.0, 0.0),
            jump_request=True,
        )
        self.assertFalse(result.safe_stand)

    def test_estop_is_latched_independent_until_explicit_release(self) -> None:
        model = RecordingModel()
        policy = PlayPolicy(model, emergency_stop=EmergencyStop())
        first = policy.step(
            state(),
            command=(0.6, 0.0, 0.0),
            input_frame=InputFrame(safe=True, reason="escape"),
        )
        second = policy.step(state(), command=(0.6, 0.0, 0.0))
        self.assertTrue(first.estop)
        self.assertTrue(second.estop)
        self.assertTrue(second.safe_stand)
        self.assertTrue(policy.emergency_stop.active)
        policy.emergency_stop.release()
        self.assertFalse(policy.emergency_stop.active)

    def test_input_adapter_timeout_reaches_play_safe_stand(self) -> None:
        model = RecordingModel()
        runtime = PlayRuntime(model)
        runtime.adapter = InputAdapter()
        result = runtime.step(
            state(),
            keys=[],
            axes=[0.0, 0.0, 0.0],
            buttons=[0, 0],
            now=2.0,
            last_input_at=1.0,
        )
        self.assertTrue(result.safe_stand)
        self.assertEqual(result.reason, "input_timeout")
        self.assertFalse(result.estop)


if __name__ == "__main__":
    unittest.main(verbosity=2)
