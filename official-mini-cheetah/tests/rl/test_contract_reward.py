#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""新契约与奖励纯测试。"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import rl.contract as contract
from rl.evaluate import PHYSICAL_THRESHOLDS
from rl.reward import RewardInputs, compute_reward


class ContractTests(unittest.TestCase):
    def test_contract_dimensions_and_schedule(self) -> None:
        self.assertEqual(contract.OBS_DIM, 57)
        self.assertEqual(contract.ACTION_DIM, 12)
        self.assertEqual(contract.CONTROL_RATE_HZ, 50)
        self.assertEqual(contract.BRIDGE_PORT, 11452)
        self.assertEqual(contract.ACTION_RATE_LIMIT, 0.08)
        self.assertEqual(contract.ACTION_TARGET_RATE_LIMIT, 0.03)
        self.assertAlmostEqual(
            contract.ACTION_TARGET_RATE_LIMIT
            * contract.CONTROL_DT_SECONDS,
            0.0006,
        )
        self.assertEqual(contract.P1_EPISODE_STEPS, 500)
        self.assertEqual(contract.P1_CONTACT_TRANSITION_START, 280_000)
        self.assertEqual(contract.P1_CONTACT_TRANSITION_END, 320_000)
        self.assertEqual(contract.P1_STATIC_CONTACT_LOCAL_STEPS, 300_000)
        self.assertEqual(contract.P1_STATIC_CONTACT_FADE_STEPS, 30_000)
        self.assertEqual(contract.P1_PERTURBATION_LOCAL_STEPS, 340_000)
        self.assertEqual(contract.P1_PERTURBATION_GATE_STEP, 310_000)
        self.assertEqual(
            contract.phase_gate_check_interval_steps("P2"),
            50_000,
        )
        for phase in ("P0", "P1", "P4", "P5"):
            self.assertEqual(
                contract.phase_gate_check_interval_steps(phase),
                10_000,
            )
        cycle = contract.P2_COMMAND_STRATUM_CYCLE
        self.assertEqual(len(cycle), 16)
        self.assertEqual(cycle.count("gate_forward"), 8)
        self.assertEqual(cycle.count("zero"), 4)
        self.assertEqual(cycle.count("lateral_positive"), 1)
        self.assertEqual(cycle.count("lateral_negative"), 1)
        self.assertEqual(cycle.count("yaw_positive"), 1)
        self.assertEqual(cycle.count("yaw_negative"), 1)
        self.assertNotEqual(
            contract.phase_training_seed("P2", 0),
            contract.phase_training_seed("P2", 50_000),
        )
        self.assertNotEqual(
            contract.phase_training_seed("P1", 0),
            contract.phase_training_seed("P2", 0),
        )
        with self.assertRaises(ValueError):
            contract.phase_training_seed("P2", -1)
        self.assertEqual(
            PHYSICAL_THRESHOLDS["true_four_contact_ratio_min"],
            0.85,
        )
        self.assertEqual(
            contract.PHASE_TOTAL_STEPS,
            {
                "P0": 10_000,
                "P1": 400_000,
                "P2": 1_500_000,
                "P3": 4_000_000,
                "P4": 6_500_000,
                "P5": 9_500_000,
                "P6": 14_500_000,
                "P7": 18_000_000,
            },
        )
        self.assertEqual(contract.PHASE_DEFAULT_TARGETS["P1"], 400_000)
        self.assertEqual(
            contract.BUFFER_TOTAL_STEPS,
            20_000_000,
        )
        self.assertEqual(contract.CONTRACT_VERSION, "official_mini_cheetah_flat_jump_v1_r3")
        self.assertEqual(contract.RANDOMIZATION_LEVELS, tuple(f"dr{i}" for i in range(1, 8)))
        self.assertEqual(contract.PARALLEL_WORKERS, 4)
        self.assertEqual(contract.bridge_ports(), (11452, 11453, 11454, 11455))
        with self.assertRaises(ValueError):
            contract.bridge_ports(2)
        self.assertEqual(contract.WEBOTS_SUPERVISOR_PORT, 1234)
        self.assertEqual(
            contract.ROBOT_NAMES,
            (
                "mini_cheetah_0",
                "mini_cheetah_1",
                "mini_cheetah_2",
                "mini_cheetah_3",
            ),
        )
        self.assertEqual(
            contract.BIRTH_POSITIONS,
            (
                (-4.0, -4.0, 0.45),
                (4.0, -4.0, 0.45),
                (-4.0, 4.0, 0.45),
                (4.0, 4.0, 0.45),
            ),
        )
        self.assertEqual(contract.LOCAL_ACTIVITY_RADIUS, 3.0)
        self.assertEqual(contract.EVAL_BIRTH_POSITION, (0.0, 0.0, 0.45))
        self.assertTrue(contract.obs_layout_valid())
        self.assertEqual(len(contract.MOTOR_NAMES), 12)
        self.assertEqual(len(contract.SENSOR_NAMES), 12)
        self.assertEqual(contract.BIRTH_HEIGHT, 0.45)
        self.assertAlmostEqual(contract.REFERENCE_HEIGHT, 0.2713, places=4)
        self.assertEqual(contract.RSI_STANCE_HEIGHT, contract.REFERENCE_HEIGHT)

    def test_checkpoint_isolation(self) -> None:
        allowed = contract.validate_checkpoint_path(
            ROOT / "checkpoints" / contract.CONTRACT_VERSION / f"{contract.CONTRACT_VERSION}_final.zip"
        )
        self.assertIn("official-mini-cheetah", allowed.parts)
        with self.assertRaises(ValueError):
            contract.validate_checkpoint_path(
                ROOT.parent / "checkpoints" / f"{contract.CONTRACT_VERSION}_final.zip"
            )
        with self.assertRaises(ValueError):
            contract.validate_checkpoint_path(
                ROOT / "checkpoints" / ("yobogo_" + "flat_jump_v1_final.zip")
            )
        with self.assertRaises(ValueError):
            contract.validate_checkpoint_path(
                ROOT
                / "checkpoints"
                / "official_mini_cheetah_flat_jump_v1_r2"
                / "official_mini_cheetah_flat_jump_v1_r2_450000_steps.zip"
            )

    def test_action_safety_and_target(self) -> None:
        safe = contract.sanitize_action([9.0] * 12, [0.0] * 12)
        self.assertTrue(all(value <= contract.ACTION_RATE_LIMIT + 1e-9 for value in safe))
        with self.assertRaises(ValueError):
            contract.sanitize_action([math.nan] + [0.0] * 11)
        target = contract.action_to_target([0.0] * 12)
        self.assertEqual(target, contract.rsi_targets())
        self.assertEqual(contract.rsi_targets(), contract.DEFAULT_CROUCH)
        scaled = contract.action_to_target([1.0] * 12)
        self.assertAlmostEqual(
            scaled[0],
            contract.DEFAULT_CROUCH[0] + contract.ACTION_SCALE[0],
        )
        self.assertTrue(
            all(low <= value <= high for value, (low, high) in zip(target, contract.TARGET_POSITION_LIMITS))
        )

    def test_command_limits(self) -> None:
        self.assertEqual(contract.clip_command([9, -9, 9]), (0.6, -0.3, 1.0))
        self.assertEqual(
            contract.phase_command_limits("P2"),
            {"vx": (-0.10, 0.25), "vy": (-0.10, 0.10), "wz": (-0.30, 0.30)},
        )
        self.assertEqual(
            contract.clip_phase_command("P5", [0.4, 0.2, 1.0]),
            (0.0, 0.0, 0.0),
        )


def inputs(phase: str, **changes: object) -> RewardInputs:
    values = {
        "phase": phase,
        "roll": 0.0,
        "pitch": 0.0,
        "height": 0.45,
        "command": np.zeros(3),
        "velocity": np.zeros(3),
        "action": np.zeros(12),
        "previous_action": np.zeros(12),
        "contacts": np.ones(4),
        "fallen": False,
        "jump_success_event": False,
        "jump_landing_event": False,
    }
    values.update(changes)
    return RewardInputs(**values)  # type: ignore[arg-type]


class RewardTests(unittest.TestCase):
    def test_p0_and_p1_expose_alive_and_true_four_foot_contact(self) -> None:
        for phase in ("P0", "P1"):
            parts = compute_reward(inputs(phase))[1]
            self.assertGreater(parts["alive"], 0.0)
            self.assertGreater(parts["true_four_foot_contact"], 0.0)
        partial = compute_reward(
            inputs("P1", contacts=[1.0, 1.0, 0.0, 0.0])
        )[1]
        self.assertEqual(partial["true_four_foot_contact"], 0.0)
        self.assertEqual(partial["support_gap"], -2.0)
        full = compute_reward(inputs("P1"))[1]
        self.assertEqual(full["support_gap"], 0.0)
        self.assertGreater(full["true_four_foot_contact"], 0.0)
        torque = compute_reward(
            inputs("P1", joint_torques=[5.0] * contract.ACTION_DIM)
        )[1]
        self.assertLess(torque["torque"], 0.0)
        no_contact = compute_reward(
            inputs("P1", contacts=[0.0] * 4)
        )[1]
        self.assertEqual(no_contact["true_four_foot_contact"], 0.0)

    def test_p1_contact_course_blends_without_hard_switch(self) -> None:
        before = compute_reward(
            inputs(
                "P1",
                phase_steps=contract.P1_CONTACT_TRANSITION_START - 1,
            )
        )[1]
        start = compute_reward(
            inputs(
                "P1",
                phase_steps=contract.P1_CONTACT_TRANSITION_START,
            )
        )[1]
        midpoint = compute_reward(
            inputs(
                "P1",
                phase_steps=(
                    contract.P1_CONTACT_TRANSITION_START
                    + contract.P1_CONTACT_TRANSITION_END
                ) // 2,
            )
        )[1]
        end = compute_reward(
            inputs(
                "P1",
                phase_steps=contract.P1_CONTACT_TRANSITION_END,
            )
        )[1]
        self.assertAlmostEqual(before["true_four_foot_contact"], 12.0)
        self.assertAlmostEqual(start["true_four_foot_contact"], 12.0)
        self.assertAlmostEqual(midpoint["true_four_foot_contact"], 7.5)
        self.assertAlmostEqual(end["true_four_foot_contact"], 3.0)

        partial_before = compute_reward(
            inputs(
                "P1",
                contacts=[1.0, 1.0, 0.0, 0.0],
                phase_steps=contract.P1_CONTACT_TRANSITION_START - 1,
            )
        )[1]
        partial_midpoint = compute_reward(
            inputs(
                "P1",
                contacts=[1.0, 1.0, 0.0, 0.0],
                phase_steps=(
                    contract.P1_CONTACT_TRANSITION_START
                    + contract.P1_CONTACT_TRANSITION_END
                ) // 2,
            )
        )[1]
        partial_end = compute_reward(
            inputs(
                "P1",
                contacts=[1.0, 1.0, 0.0, 0.0],
                phase_steps=contract.P1_CONTACT_TRANSITION_END,
            )
        )[1]
        self.assertAlmostEqual(partial_before["support_gap"], -2.0)
        self.assertAlmostEqual(partial_midpoint["support_gap"], -1.35)
        self.assertAlmostEqual(partial_end["support_gap"], -0.7)

        # 静态课程在 300k 后独立淡出，关闭前后逐步连续。
        static_start = compute_reward(
            inputs(
                "P1",
                phase_steps=contract.P1_STATIC_CONTACT_LOCAL_STEPS,
            )
        )[1]
        static_half = compute_reward(
            inputs(
                "P1",
                phase_steps=(
                    contract.P1_STATIC_CONTACT_LOCAL_STEPS
                    + contract.P1_STATIC_CONTACT_FADE_STEPS // 2
                ),
            )
        )[1]
        static_end = compute_reward(
            inputs(
                "P1",
                phase_steps=(
                    contract.P1_STATIC_CONTACT_LOCAL_STEPS
                    + contract.P1_STATIC_CONTACT_FADE_STEPS
                ),
            )
        )[1]
        self.assertAlmostEqual(static_start["static_contact_course"], 0.2)
        self.assertAlmostEqual(static_half["static_contact_course"], 0.1)
        self.assertEqual(static_end["static_contact_course"], 0.0)
        self.assertEqual(
            contract.P1_CONTACT_TRANSITION_END,
            contract.P1_STATIC_CONTACT_LOCAL_STEPS
            + contract.P1_STATIC_CONTACT_FADE_STEPS
            - 10_000,
        )
        self.assertGreater(
            contract.P1_PERTURBATION_LOCAL_STEPS,
            contract.P1_STATIC_CONTACT_LOCAL_STEPS
            + contract.P1_STATIC_CONTACT_FADE_STEPS,
        )

    def test_p2_progress_rewards_movement_and_contact_balance(self) -> None:
        forward_command = [0.25, 0.0, 0.0]
        moving = compute_reward(
            inputs(
                "P2",
                command=forward_command,
                velocity=[0.18, 0.0, 0.0],
                displacement=[0.0036, 0.0, 0.0],
            )
        )[1]
        standing = compute_reward(
            inputs(
                "P2",
                command=forward_command,
                velocity=[0.0, 0.0, 0.0],
                displacement=[0.0, 0.0, 0.0],
            )
        )[1]
        self.assertAlmostEqual(
            moving["command_speed_progress"],
            1.08,
            places=6,
        )
        self.assertAlmostEqual(
            moving["command_displacement_progress"],
            1.08,
            places=6,
        )
        self.assertGreater(
            sum(moving.values()) - sum(standing.values()),
            2.0,
        )
        partial = compute_reward(
            inputs(
                "P2",
                command=forward_command,
                velocity=[0.18, 0.0, 0.0],
                displacement=[0.0036, 0.0, 0.0],
                contacts=[1.0, 1.0, 1.0, 0.0],
            )
        )[1]
        self.assertEqual(partial["true_four_foot_contact"], 0.0)
        self.assertAlmostEqual(partial["support_gap"], -0.15)
        self.assertAlmostEqual(partial["gait"], -0.05)
        later = compute_reward(
            inputs(
                "P3",
                command=forward_command,
                velocity=[0.18, 0.0, 0.0],
                displacement=[0.0036, 0.0, 0.0],
            )
        )[1]
        self.assertEqual(later["command_speed_progress"], 0.0)
        self.assertEqual(later["command_displacement_progress"], 0.0)
        self.assertAlmostEqual(later["gait"], 0.0)

    def test_gate_failure_terms_are_penalized_more_in_p1(self) -> None:
        fast_feet = [0.2] * 12
        p1_slip = compute_reward(
            inputs(
                "P1",
                contacts=[1.0] * 4,
                foot_velocities=fast_feet,
            )
        )[1]["foot_slip"]
        p3_slip = compute_reward(
            inputs(
                "P3",
                contacts=[1.0] * 4,
                foot_velocities=fast_feet,
            )
        )[1]["foot_slip"]
        self.assertLess(p1_slip, p3_slip)

        p1_action = compute_reward(
            inputs(
                "P1",
                action=[0.2] * 12,
                previous_action=[0.0] * 12,
            )
        )[1]["action_rate"]
        p3_action = compute_reward(
            inputs(
                "P3",
                action=[0.2] * 12,
                previous_action=[0.0] * 12,
            )
        )[1]["action_rate"]
        self.assertLess(p1_action, p3_action)

    def test_joint_jitter_uses_previous_joint_velocity(self) -> None:
        still = compute_reward(
            inputs(
                "P1",
                joint_velocities=[0.2] * contract.ACTION_DIM,
                previous_joint_velocities=[0.2] * contract.ACTION_DIM,
            )
        )[1]
        jitter = compute_reward(
            inputs(
                "P1",
                joint_velocities=[0.4] * contract.ACTION_DIM,
                previous_joint_velocities=[0.2] * contract.ACTION_DIM,
            )
        )[1]
        self.assertEqual(still["joint_jitter"], 0.0)
        self.assertLess(jitter["joint_jitter"], 0.0)

    def test_height_reward_is_zero_at_reference_and_negative_away(self) -> None:
        at_reference = compute_reward(
            inputs("P1", height=contract.REFERENCE_HEIGHT)
        )[1]
        above = compute_reward(
            inputs(
                "P1",
                height=contract.REFERENCE_HEIGHT + 0.05,
            )
        )[1]
        below = compute_reward(
            inputs(
                "P1",
                height=contract.REFERENCE_HEIGHT - 0.05,
            )
        )[1]
        self.assertEqual(at_reference["height"], 0.0)
        self.assertLess(above["height"], 0.0)
        self.assertLess(below["height"], 0.0)

    def test_default_crouch_rsi_zero_action_and_reward_align(self) -> None:
        zero_action_target = contract.action_to_target(
            [0.0] * contract.ACTION_DIM
        )
        rsi_target = contract.rsi_targets()
        reward_parts = compute_reward(
            inputs(
                "P1",
                height=contract.REFERENCE_HEIGHT,
                joint_positions=contract.DEFAULT_CROUCH,
            )
        )[1]
        self.assertEqual(zero_action_target, contract.DEFAULT_CROUCH)
        self.assertEqual(rsi_target, contract.DEFAULT_CROUCH)
        self.assertEqual(reward_parts["default_pose"], 0.0)

    def test_stage_weights_and_finite(self) -> None:
        self.assertEqual(compute_reward(inputs("P1"))[1]["fall"], 0.0)
        self.assertEqual(contract.__dict__.get("unused"), None)
        fall = compute_reward(inputs("P1", fallen=True))[0]
        self.assertLess(fall, -100.0)
        moving = compute_reward(
            inputs("P3", command=[0.4, 0, 0], velocity=[0.4, 0, 0])
        )[0]
        self.assertTrue(math.isfinite(moving))

    def test_p1_zero_speed_term_is_negative(self) -> None:
        still = compute_reward(
            inputs("P1", command=[0, 0, 0], velocity=[0, 0, 0])
        )[1]
        moving = compute_reward(
            inputs("P1", command=[0, 0, 0], velocity=[0.4, 0, 0])
        )[1]
        self.assertLessEqual(still["command_tracking"], 0.0)
        self.assertLess(moving["command_tracking"], 0.0)
        self.assertLess(
            sum(moving.values()),
            sum(still.values()),
        )

    def test_true_four_foot_contact_drives_foot_slip(self) -> None:
        fast_feet = [0.2] * 12
        first_only = compute_reward(
            inputs(
                "P1",
                contacts=[1.0, 0.0, 0.0, 0.0],
                foot_velocities=fast_feet,
            )
        )[1]
        fourth_only = compute_reward(
            inputs(
                "P1",
                contacts=[0.0, 0.0, 0.0, 1.0],
                foot_velocities=fast_feet,
            )
        )[1]
        self.assertLess(first_only["foot_slip"], 0.0)
        self.assertEqual(
            first_only["foot_slip"],
            fourth_only["foot_slip"],
        )
        no_contact = compute_reward(
            inputs("P1", contacts=[0.0] * 4, foot_velocities=fast_feet)
        )[1]
        self.assertEqual(no_contact["foot_slip"], 0.0)

    def test_jump_events_only_in_jump_phases(self) -> None:
        moving = compute_reward(
            inputs("P3", jump_success_event=True, jump_landing_event=True)
        )[1]
        jumping = compute_reward(
            inputs("P5", jump_success_event=True, jump_landing_event=True)
        )[1]
        self.assertEqual(moving["jump_success"], 0.0)
        self.assertEqual(jumping["jump_success"], 8.0)
        self.assertEqual(jumping["jump_landing"], 4.0)

    def test_nonfinite_fails(self) -> None:
        with self.assertRaises(ValueError):
            compute_reward(inputs("P3", height=math.nan))
        with self.assertRaises(ValueError):
            compute_reward(inputs("P3", velocity=[math.inf, 0, 0]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
