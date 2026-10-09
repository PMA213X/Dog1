from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
CONTROLLER_DIR = ROOT / "controllers" / "rl_agent"
if str(CONTROLLER_DIR) not in sys.path:
    sys.path.insert(0, str(CONTROLLER_DIR))

from rl import contract
from rl.play import PlayPolicy, build_play_log_record
from rl_agent import smooth_action_targets


class RateTelemetryTests(unittest.TestCase):
    """限速阶梯、统一目标遥测和非有限保护。"""

    def test_rate_ladder_is_exactly_configured_and_rejects_old_default(self) -> None:
        self.assertEqual(contract.ACTION_TARGET_RATE_LADDER, (0.25, 0.5, 1.0))
        self.assertEqual(contract.ACTION_TARGET_RATE_LIMIT, 1.0)
        for value in contract.ACTION_TARGET_RATE_LADDER:
            self.assertEqual(
                contract.validate_action_target_rate_limit(value),
                value,
            )
        with self.assertRaises(ValueError):
            contract.validate_action_target_rate_limit(0.03)
        with self.assertRaises(ValueError):
            contract.validate_action_target_rate_limit(math.nan)

    def test_contract_controller_and_play_use_same_target_rate_semantics(self) -> None:
        previous = contract.DEFAULT_CROUCH
        desired = tuple(
            low + 0.1 if index % 2 == 0 else high - 0.1
            for index, (low, high) in enumerate(
                contract.TARGET_POSITION_LIMITS
            )
        )
        policy = PlayPolicy(object())
        for rate in contract.ACTION_TARGET_RATE_LADDER:
            expected = contract.limit_joint_targets(
                previous,
                desired,
                rate,
            )
            controller = smooth_action_targets(
                previous,
                desired,
                rate,
            )
            policy.previous_targets = previous
            policy.target_rate_limit = rate
            play = policy._limit_targets(desired)
            np.testing.assert_allclose(expected, controller, atol=0.0, rtol=0.0)
            np.testing.assert_allclose(expected, play, atol=0.0, rtol=0.0)
            allowed = rate * contract.CONTROL_DT_SECONDS
            self.assertLessEqual(
                max(
                    abs(left - right)
                    for left, right in zip(expected, previous)
                ),
                allowed + 1e-9,
            )

    def test_target_telemetry_and_saturation_are_finite(self) -> None:
        desired = contract.DEFAULT_CROUCH
        executed = tuple(
            value + (0.02 if index < 6 else -0.05)
            for index, value in enumerate(desired)
        )
        telemetry = contract.target_telemetry(desired, executed, rate_limit=0.5)
        self.assertEqual(len(telemetry["delta"]), contract.ACTION_DIM)
        self.assertAlmostEqual(telemetry["mean"], 0.035)
        self.assertAlmostEqual(telemetry["rms"], math.sqrt((6 * 0.0004 + 6 * 0.0025) / 12))
        self.assertAlmostEqual(telemetry["max"], 0.05)
        self.assertEqual(contract.action_saturation_rate([1.0] * 11 + [0.0]), 11 / 12)
        with self.assertRaises(ValueError):
            contract.target_telemetry(desired, [math.inf] * 12)
        with self.assertRaises(ValueError):
            contract.limit_joint_targets(desired, [math.nan] * 12)

    def test_play_log_exposes_execution_position_velocity_and_displacement(self) -> None:
        result = PlayPolicy(object()).safe_stand_result(
            {
                "q": contract.DEFAULT_CROUCH,
                "dq": [0.0] * 12,
                "rpy": [0.0] * 3,
                "v_body": [0.0] * 3,
                "omega_body": [0.0] * 3,
                "height": contract.DEFAULT_STANDING_HEIGHT,
                "contacts": [1.0] * 4,
            },
            reason="zero_command",
        )
        info = build_play_log_record(
            step=1,
            result=result,
            transition_info={
                "applied_action": np.zeros(12, dtype=np.float32),
                "desired_targets": contract.DEFAULT_CROUCH,
                "executed_targets": contract.DEFAULT_CROUCH,
                "target_rate_limit": 0.5,
                "position": [1.0, -2.0, 0.3],
                "v_body": [0.1, 0.0, -0.01],
                "displacement_world": [0.02, 0.01, 0.0],
                "displacement_body": [0.02, 0.01, 0.0],
                "episode_distance_world": 1.5,
            },
        )
        self.assertEqual(info["target_rate_limit"], 0.5)
        np.testing.assert_allclose(
            info["position"],
            [1.0, -2.0, 0.3],
            atol=1e-7,
        )
        np.testing.assert_allclose(
            info["v_body"],
            [0.1, 0.0, -0.01],
            atol=1e-7,
        )
        self.assertAlmostEqual(
            info["displacement_step_norm"],
            math.sqrt(0.0005),
            places=7,
        )
        self.assertEqual(info["episode_distance_world"], 1.5)
        self.assertEqual(info["target_lag_mean"], 0.0)
        self.assertEqual(info["mapped_action_saturation_rate"], 0.0)

    def test_environment_exposes_finetune_mode_without_starting_bridge(self) -> None:
        from rl.env import MiniCheetahFlatJumpEnv

        default_env = MiniCheetahFlatJumpEnv(start_bridge=False)
        finetune_env = MiniCheetahFlatJumpEnv(
            start_bridge=False,
            finetune_mode=True,
        )
        self.assertFalse(default_env.finetune_mode)
        self.assertTrue(finetune_env.finetune_mode)


if __name__ == "__main__":
    unittest.main()
