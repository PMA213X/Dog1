#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""yobogo_loco_jump_v1 分阶段奖励单元测试。"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np


RL_DIR = Path(__file__).resolve().parents[1]
if str(RL_DIR) not in sys.path:
    sys.path.insert(0, str(RL_DIR))

from loco_jump_reward import (  # noqa: E402
    PHASE_REWARD_WEIGHTS,
    RewardInputs,
    compute_reward,
    normalize_stage,
    reward_for_state,
    stage_tag,
)


ALL_STAGES = (
    "S0_smoke",
    "S1_stand",
    "S2_command",
    "S3_jump",
    "S4_mobile_terrain",
)


def make_inputs(**overrides) -> RewardInputs:
    """构造默认稳定状态，测试再按需要覆盖字段。"""
    values = {
        "roll": 0.0,
        "pitch": 0.0,
        "body_height": 0.26,
        "action": np.zeros(12, dtype=np.float32),
        "previous_action": np.zeros(12, dtype=np.float32),
        "command_error": 0.0,
        "heading_error": 0.0,
        "jump_success_event": False,
        "terrain_success_event": False,
        "fallen": False,
    }
    values.update(overrides)
    return RewardInputs(**values)


class LocoJumpRewardFinitenessTest(unittest.TestCase):
    """验证所有阶段在正常、极端和非法输入下都返回有限奖励。"""

    def test_all_stages_are_finite_for_normal_and_extreme_inputs(self) -> None:
        cases = (
            make_inputs(),
            make_inputs(
                roll=0.7,
                pitch=-0.8,
                body_height=0.05,
                action=np.linspace(-1.0, 1.0, 12, dtype=np.float32),
                previous_action=np.linspace(1.0, -1.0, 12, dtype=np.float32),
                command_error=12.5,
                heading_error=math.pi,
                jump_success_event=True,
                terrain_success_event=True,
                fallen=True,
            ),
            make_inputs(
                roll=float("nan"),
                pitch=float("inf"),
                body_height=float("-inf"),
                action=np.full(12, float("nan"), dtype=np.float32),
                previous_action=np.full(12, float("inf"), dtype=np.float32),
                command_error=float("nan"),
                heading_error=float("-inf"),
                jump_success_event=True,
                terrain_success_event=True,
                fallen=True,
            ),
        )
        for stage in ALL_STAGES:
            for case_index, inputs in enumerate(cases):
                with self.subTest(stage=stage, case=case_index):
                    breakdown = compute_reward(stage, inputs)
                    self.assertTrue(math.isfinite(breakdown.total))
                    for name, value in breakdown.as_dict().items():
                        self.assertTrue(
                            math.isfinite(value),
                            f"{stage}/{name} 非有限：{value!r}",
                        )
                    self.assertAlmostEqual(
                        breakdown.total,
                        sum(
                            (
                                breakdown.alive,
                                breakdown.posture,
                                breakdown.height,
                                breakdown.smoothness,
                                breakdown.command,
                                breakdown.heading,
                                breakdown.jump,
                                breakdown.terrain,
                                breakdown.fall,
                            )
                        ),
                        places=12,
                    )

    def test_mismatched_action_shapes_remain_finite(self) -> None:
        for stage in ALL_STAGES:
            with self.subTest(stage=stage):
                inputs = make_inputs(
                    action=np.ones(12, dtype=np.float32),
                    previous_action=np.zeros(6, dtype=np.float32),
                )
                breakdown = compute_reward(stage, inputs)
                self.assertTrue(math.isfinite(breakdown.total))
                self.assertTrue(math.isfinite(breakdown.smoothness))


class LocoJumpRewardStageDifferenceTest(unittest.TestCase):
    """验证命令、跳跃和越障项按阶段逐步启用。"""

    def test_command_error_only_shapes_command_stage_and_later(self) -> None:
        baseline = compute_reward("S0_smoke", make_inputs())
        with_error = compute_reward(
            "S0_smoke",
            make_inputs(command_error=0.75),
        )
        self.assertEqual(with_error.command, 0.0)
        self.assertEqual(with_error.total, baseline.total)

        command_rewards = []
        for stage in ("S2_command", "S3_jump", "S4_mobile_terrain"):
            breakdown = compute_reward(
                stage,
                make_inputs(command_error=0.75),
            )
            self.assertLess(breakdown.command, 0.0)
            self.assertAlmostEqual(breakdown.command, -1.5, places=12)
            command_rewards.append(breakdown.command)
        self.assertEqual(len(set(command_rewards)), 1)

    def test_heading_error_only_shapes_command_stage_and_later(self) -> None:
        self.assertEqual(
            compute_reward("S1_stand", make_inputs(heading_error=0.4)).heading,
            0.0,
        )
        for stage in ("S2_command", "S3_jump", "S4_mobile_terrain"):
            with self.subTest(stage=stage):
                breakdown = compute_reward(
                    stage,
                    make_inputs(heading_error=0.4),
                )
                self.assertAlmostEqual(breakdown.heading, -0.2, places=12)

    def test_jump_event_only_shapes_jump_stage_and_later(self) -> None:
        for stage in ("S0_smoke", "S1_stand", "S2_command"):
            with self.subTest(stage=stage):
                breakdown = compute_reward(
                    stage,
                    make_inputs(jump_success_event=True),
                )
                self.assertEqual(breakdown.jump, 0.0)
        for stage in ("S3_jump", "S4_mobile_terrain"):
            with self.subTest(stage=stage):
                breakdown = compute_reward(
                    stage,
                    make_inputs(jump_success_event=True),
                )
                self.assertGreater(breakdown.jump, 0.0)
                self.assertAlmostEqual(breakdown.jump, 8.0, places=12)

    def test_terrain_event_only_shapes_mobile_terrain_stage(self) -> None:
        for stage in ALL_STAGES[:-1]:
            with self.subTest(stage=stage):
                breakdown = compute_reward(
                    stage,
                    make_inputs(terrain_success_event=True),
                )
                self.assertEqual(breakdown.terrain, 0.0)

        breakdown = compute_reward(
            "S4_mobile_terrain",
            make_inputs(terrain_success_event=True),
        )
        self.assertGreater(breakdown.terrain, 0.0)
        self.assertAlmostEqual(breakdown.terrain, 10.0, places=12)

    def test_posture_height_smoothness_and_fall_terms_are_active(self) -> None:
        action = np.zeros(12, dtype=np.float32)
        previous_action = np.zeros(12, dtype=np.float32)
        action[0] = 0.5
        previous_action[0] = -0.5
        inputs = make_inputs(
            roll=0.2,
            pitch=-0.3,
            body_height=0.36,
            action=action,
            previous_action=previous_action,
            fallen=True,
        )
        breakdown = compute_reward("S0_smoke", inputs)
        self.assertAlmostEqual(breakdown.posture, -1.0, places=12)
        self.assertAlmostEqual(breakdown.height, -0.2, places=12)
        self.assertAlmostEqual(breakdown.smoothness, -0.05, places=12)
        self.assertEqual(breakdown.fall, -20.0)

    def test_stage_weights_are_monotonic_in_features(self) -> None:
        self.assertEqual(
            tuple(PHASE_REWARD_WEIGHTS),
            ALL_STAGES,
        )
        self.assertEqual(PHASE_REWARD_WEIGHTS["S0_smoke"], PHASE_REWARD_WEIGHTS["S1_stand"])
        self.assertEqual(PHASE_REWARD_WEIGHTS["S2_command"]["command"], -2.0)
        self.assertEqual(PHASE_REWARD_WEIGHTS["S2_command"]["jump"], 0.0)
        self.assertEqual(PHASE_REWARD_WEIGHTS["S3_jump"]["jump"], 8.0)
        self.assertEqual(PHASE_REWARD_WEIGHTS["S3_jump"]["terrain"], 0.0)
        self.assertEqual(PHASE_REWARD_WEIGHTS["S4_mobile_terrain"]["terrain"], 10.0)


class LocoJumpRewardInterfaceTest(unittest.TestCase):
    """验证阶段标签与 TCP state 奖励适配接口。"""

    def test_stage_normalization_and_tags(self) -> None:
        self.assertEqual(normalize_stage("phase0"), "S0_smoke")
        self.assertEqual(normalize_stage("PHASE1"), "S1_stand")
        self.assertEqual(normalize_stage("s2_command"), "S2_command")
        self.assertEqual(normalize_stage("S4_mobile_terrain"), "S4_mobile_terrain")
        self.assertEqual(
            [stage_tag(stage) for stage in ALL_STAGES],
            ["phase0", "phase1", "phase2", "phase3", "phase4"],
        )
        with self.assertRaises(ValueError):
            normalize_stage("phase9")

    def test_reward_for_state_uses_tcp_fields(self) -> None:
        state = {
            "rpy": [0.1, -0.2, 1.2],
            "height": 0.31,
        }
        total, breakdown = reward_for_state(
            "phase4",
            state,
            np.zeros(12, dtype=np.float32),
            np.zeros(12, dtype=np.float32),
            command_error=0.25,
            heading_error=0.5,
            fallen=False,
            jump_success_event=True,
            terrain_success_event=True,
        )
        self.assertTrue(math.isfinite(total))
        self.assertEqual(breakdown.stage, "S4_mobile_terrain")
        self.assertAlmostEqual(breakdown.posture, -0.6, places=12)
        self.assertAlmostEqual(breakdown.height, -0.1, places=12)
        self.assertAlmostEqual(breakdown.command, -0.5, places=12)
        self.assertAlmostEqual(breakdown.heading, -0.25, places=12)
        self.assertAlmostEqual(breakdown.jump, 8.0, places=12)
        self.assertAlmostEqual(breakdown.terrain, 10.0, places=12)
        self.assertAlmostEqual(total, breakdown.total, places=12)


if __name__ == "__main__":
    unittest.main()
