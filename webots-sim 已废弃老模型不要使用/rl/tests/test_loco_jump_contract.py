#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""yobogo_loco_jump_v1 公共契约单元测试。"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np


RL_DIR = Path(__file__).resolve().parents[1]
if str(RL_DIR) not in sys.path:
    sys.path.insert(0, str(RL_DIR))

import loco_jump_contract as contract  # noqa: E402


class LocoJumpContractDimensionsTest(unittest.TestCase):
    """验证观测、动作和关节目标映射契约。"""

    def test_observation_dimension_and_slices(self) -> None:
        self.assertEqual(contract.OBS_DIM, 55)
        self.assertEqual(contract.LEGACY_OBS_DIM, 42)
        self.assertEqual(contract.ADDED_OBS_DIM, 13)
        self.assertEqual(
            contract.OBS_FIELD_ORDER,
            (
                "q",
                "dq",
                "rpy",
                "v_body",
                "prev_action",
                "omega_body",
                "cmd_vx_vy_wz",
                "jump_request",
                "jump_phase_time",
                "body_height",
                "foot_contact",
            ),
        )
        self.assertEqual(tuple(contract.OBS_SLICES), contract.OBS_FIELD_ORDER)
        self.assertEqual(sum(contract.OBS_FIELD_DIMS.values()), contract.OBS_DIM)

        cursor = 0
        for field in contract.OBS_FIELD_ORDER:
            field_slice = contract.OBS_SLICES[field]
            self.assertEqual(field_slice.start, cursor)
            self.assertEqual(
                field_slice.stop - field_slice.start,
                contract.OBS_FIELD_DIMS[field],
            )
            cursor = field_slice.stop
        self.assertEqual(cursor, contract.OBS_DIM)

        self.assertEqual(contract.OBS_SLICES["prev_action"], slice(30, 42))
        self.assertEqual(contract.OBS_SLICES["omega_body"], slice(42, 45))
        self.assertEqual(contract.OBS_SLICES["cmd_vx_vy_wz"], slice(45, 48))
        self.assertEqual(contract.OBS_SLICES["jump_request"], slice(48, 49))
        self.assertEqual(contract.OBS_SLICES["jump_phase_time"], slice(49, 50))
        self.assertEqual(contract.OBS_SLICES["body_height"], slice(50, 51))
        self.assertEqual(contract.OBS_SLICES["foot_contact"], slice(51, 55))
        self.assertEqual(contract.FOOT_CONTACT_ORDER, ("fr", "fl", "hr", "hl"))

    def test_action_dimension_bounds_and_q_des_formula(self) -> None:
        self.assertEqual(contract.ACTION_DIM, 12)
        self.assertEqual(contract.ACTION_BOUNDS, (-1.0, 1.0))

        q_stand = np.array(
            [
                -0.10,
                -0.20,
                -0.30,
                -0.40,
                -0.50,
                -0.60,
                -0.70,
                -0.80,
                -0.90,
                -1.00,
                -1.10,
                -1.20,
            ],
            dtype=np.float64,
        )
        action = np.array(
            [
                1.0,
                -1.0,
                0.5,
                -0.5,
                0.25,
                -0.25,
                0.75,
                -0.75,
                1.0,
                -1.0,
                0.1,
                -0.1,
            ],
            dtype=np.float64,
        )
        expected_scale = np.tile(np.array([0.3, 0.5, 0.5]), 4)
        expected_q_des = q_stand + action * expected_scale

        mapper = self._resolve_q_des_mapper()
        actual_q_des = np.asarray(mapper(action, q_stand), dtype=np.float64)
        # 实现侧按 float32 传递到控制器，验证公式时允许单精度舍入误差。
        np.testing.assert_allclose(actual_q_des, expected_q_des, rtol=0.0, atol=2e-7)
        self.assertEqual(actual_q_des.shape, (contract.ACTION_DIM,))
        self.assertTrue(np.all(np.isfinite(actual_q_des)))

    @staticmethod
    def _resolve_q_des_mapper():
        """解析实现侧公开的 q_des 映射函数，保持测试只验证契约行为。"""
        module_names = ("loco_jump_env", "loco_jump_contract")
        function_names = ("action_to_q_des", "compute_q_des", "map_action_to_q_des")
        for module_name in module_names:
            try:
                module = __import__(module_name)
            except ImportError:
                continue
            for function_name in function_names:
                candidate = getattr(module, function_name, None)
                if candidate is not None:
                    return candidate
        raise AssertionError(
            "缺少可测试的 q_des 映射函数："
            "loco_jump_env.action_to_q_des(action, q_stand)"
        )


class LocoJumpContractCommandsTest(unittest.TestCase):
    """验证命令顺序、限幅和跳跃请求值域。"""

    def test_command_limits_and_clipping(self) -> None:
        self.assertEqual(contract.COMMAND_FIELDS, ("vx", "vy", "wz"))
        self.assertEqual(contract.COMMAND_LOW, (-0.3, -0.3, -1.0))
        self.assertEqual(contract.COMMAND_HIGH, (0.6, 0.3, 1.0))

        self.assertEqual(
            contract.clip_command((-10.0, 10.0, -10.0)),
            (-0.3, 0.3, -1.0),
        )
        self.assertEqual(
            contract.clip_command((10.0, -10.0, 10.0)),
            (0.6, -0.3, 1.0),
        )
        self.assertEqual(
            contract.clip_command((0.1, -0.2, 0.3)),
            (0.1, -0.2, 0.3),
        )

        with self.assertRaises(ValueError):
            contract.clip_command((0.0, 0.0))
        with self.assertRaises(ValueError):
            contract.clip_command((0.0, 0.0, 0.0, 0.0))

    def test_jump_request_and_latch_semantics(self) -> None:
        self.assertEqual(contract.JUMP_REQUEST_BOUNDS, (0.0, 1.0))
        self.assertEqual(contract.validate_jump_request(0), 0)
        self.assertEqual(contract.validate_jump_request(1), 1)
        for invalid in (-1, 2, 0.5, float("nan")):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ValueError):
                    contract.validate_jump_request(invalid)

        self.assertEqual(contract.clip_jump_phase_time(-0.2), 0.0)
        self.assertEqual(contract.clip_jump_phase_time(0.4), 0.4)
        self.assertEqual(contract.clip_jump_phase_time(1.2), 1.0)
        self.assertTrue(
            contract.should_clear_jump_latch(
                jump_latched=True,
                contacts_restored=True,
                elapsed_seconds=0.1,
            )
        )
        self.assertTrue(
            contract.should_clear_jump_latch(
                jump_latched=True,
                contacts_restored=False,
                elapsed_seconds=contract.JUMP_LATCH_TIMEOUT_SECONDS,
            )
        )
        self.assertFalse(
            contract.should_clear_jump_latch(
                jump_latched=True,
                contacts_restored=False,
                elapsed_seconds=0.99,
            )
        )
        self.assertFalse(
            contract.should_clear_jump_latch(
                jump_latched=False,
                contacts_restored=True,
                elapsed_seconds=2.0,
            )
        )


class LocoJumpContractScheduleTest(unittest.TestCase):
    """验证阶段累计预算、正式训练范围和 checkpoint 隔离。"""

    def test_phase_targets_are_cumulative(self) -> None:
        self.assertEqual(
            contract.PHASE_TOTAL_STEPS,
            {
                "S0_smoke": 5_000,
                "S1_stand": 200_000,
                "S2_command": 500_000,
                "S3_jump": 800_000,
                "S4_mobile_terrain": 1_200_000,
            },
        )
        self.assertIs(
            contract.PHASE_TARGET_TOTAL_STEPS,
            contract.PHASE_TOTAL_STEPS,
        )
        targets = tuple(contract.PHASE_TOTAL_STEPS.values())
        self.assertEqual(targets, (5_000, 200_000, 500_000, 800_000, 1_200_000))
        self.assertTrue(
            all(left < right for left, right in zip(targets, targets[1:]))
        )

        self.assertEqual(contract.FORMAL_TRAINING_STEP_RANGE, (800_000, 1_200_000))
        self.assertEqual(contract.FORMAL_TRAINING_START_STEP, 800_000)
        self.assertEqual(contract.FORMAL_TRAINING_END_STEP, 1_200_000)

    def test_checkpoint_interval_and_prefix_isolation(self) -> None:
        self.assertEqual(contract.CHECKPOINT_INTERVAL_STEPS, 50_000)
        self.assertEqual(contract.CHECKPOINT_INTERVAL_SECONDS, 1_800.0)
        self.assertTrue(math.isfinite(contract.CHECKPOINT_INTERVAL_SECONDS))
        self.assertEqual(contract.CHECKPOINT_PREFIX, "yobogo_loco_jump_v1")

        accepted = (
            "yobogo_loco_jump_v1",
            "yobogo_loco_jump_v1_50000.zip",
            "yobogo_loco_jump_v1-50000.zip",
            "yobogo_loco_jump_v1.50000.zip",
            "/tmp/checkpoints/yobogo_loco_jump_v1_final.zip",
        )
        rejected = (
            "ppo_walk_final.zip",
            "phase4_stairs_final.zip",
            "yobogo_loco_jump_v1beta_50000.zip",
            "yobogo_loco_jump_v10_50000.zip",
            "old_yobogo_loco_jump_v1_50000.zip",
        )
        for path in accepted:
            with self.subTest(path=path):
                self.assertTrue(contract.is_isolated_checkpoint(path))
                validated = contract.validate_checkpoint_path(path)
                self.assertEqual(validated.name, Path(path).name)
        for path in rejected:
            with self.subTest(path=path):
                self.assertFalse(contract.is_isolated_checkpoint(path))
                with self.assertRaises(ValueError):
                    contract.validate_checkpoint_path(path)


class LocoJumpTcpContractTest(unittest.TestCase):
    """验证 TCP state 必需字段及数组长度。"""

    def test_tcp_state_fields(self) -> None:
        self.assertEqual(contract.TCP_STATE_TYPE, "state")
        self.assertEqual(
            contract.TCP_STATE_REQUIRED_FIELDS,
            (
                "q",
                "dq",
                "rpy",
                "omega",
                "v_body",
                "contacts",
                "height",
                "jump_phase",
                "done",
            ),
        )
        self.assertEqual(
            contract.TCP_STATE_ARRAY_LENGTHS,
            {
                "q": 12,
                "dq": 12,
                "rpy": 3,
                "omega": 3,
                "v_body": 3,
                "contacts": 4,
            },
        )
        self.assertEqual(
            contract.TCP_STATE_SCALAR_FIELDS,
            ("height", "jump_phase", "done"),
        )
        self.assertEqual(
            contract.TCP_STATE_CONTACT_ORDER,
            contract.FOOT_CONTACT_ORDER,
        )


if __name__ == "__main__":
    unittest.main()
