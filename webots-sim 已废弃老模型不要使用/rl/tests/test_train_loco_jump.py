#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""yobogo_loco_jump_v1 训练入口纯参数和 dry-run 单元测试。"""

from __future__ import annotations

import contextlib
import io
import math
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


RL_DIR = Path(__file__).resolve().parents[1]
if str(RL_DIR) not in sys.path:
    sys.path.insert(0, str(RL_DIR))

import loco_jump_contract as contract  # noqa: E402
import train_loco_jump as training  # noqa: E402


class TrainLocoJumpBudgetTest(unittest.TestCase):
    """验证阶段累计预算、正式训练范围和 checkpoint 保存条件。"""

    def test_stage_target_steps_matches_contract(self) -> None:
        expected = {
            "phase0": 5_000,
            "phase1": 200_000,
            "phase2": 500_000,
            "phase3": 800_000,
            "phase4": 1_200_000,
        }
        self.assertEqual(training.TAGS, tuple(expected))
        for tag, target in expected.items():
            with self.subTest(tag=tag):
                self.assertEqual(training.stage_target_steps(tag), target)

    def test_total_steps_validate_stage_cap_and_formal_range(self) -> None:
        for tag, target in (
            ("phase0", 5_000),
            ("phase1", 200_000),
            ("phase2", 500_000),
            ("phase3", 800_000),
            ("phase4", 1_200_000),
        ):
            with self.subTest(tag=tag):
                self.assertEqual(training.validate_total_steps(tag, target), target)
                with self.assertRaises(ValueError):
                    training.validate_total_steps(tag, 0)
                with self.assertRaises(ValueError):
                    training.validate_total_steps(tag, target + 1)

        for value in (800_000, 1_000_000, 1_200_000):
            self.assertEqual(training.validate_total_steps("phase4", value), value)
        for value in (799_999, 1_200_001):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    training.validate_total_steps("phase4", value)
        with self.assertRaises(ValueError):
            training.validate_total_steps("phase3", 799_999)

    def test_checkpoint_steps_are_isolated_per_stage(self) -> None:
        self.assertEqual(training.checkpoint_steps_for_tag("phase0"), [])
        self.assertEqual(
            training.checkpoint_steps_for_tag("phase1"),
            [50_000, 100_000, 150_000, 200_000],
        )
        self.assertEqual(
            training.checkpoint_steps_for_tag("phase2"),
            [50_000, 100_000, 150_000, 200_000, 250_000,
             300_000, 350_000, 400_000, 450_000, 500_000],
        )
        self.assertEqual(
            training.checkpoint_steps_for_tag("phase3")[-1],
            contract.S3_JUMP_TOTAL_STEPS,
        )
        self.assertEqual(
            training.checkpoint_steps_for_tag("phase4")[-1],
            contract.S4_MOBILE_TERRAIN_TOTAL_STEPS,
        )
        for tag in training.TAGS:
            steps = training.checkpoint_steps_for_tag(tag)
            self.assertTrue(all(step % contract.CHECKPOINT_INTERVAL_STEPS == 0 for step in steps))
            self.assertTrue(all(step <= training.stage_target_steps(tag) for step in steps))

    def test_checkpoint_interval_and_should_save_use_or_condition(self) -> None:
        self.assertEqual(
            training.validate_checkpoint_interval(contract.CHECKPOINT_INTERVAL_STEPS),
            50_000,
        )
        for invalid in (0, 49_999, 50_001, -50_000):
            with self.subTest(interval=invalid):
                with self.assertRaises(ValueError):
                    training.validate_checkpoint_interval(invalid)

        self.assertFalse(training.checkpoint_should_save(49_999, 1_799.0))
        self.assertTrue(training.checkpoint_should_save(50_000, 0.0))
        self.assertTrue(training.checkpoint_should_save(0, 1_800.0))
        self.assertTrue(training.checkpoint_should_save(50_000, 1_800.0))
        self.assertFalse(
            training.checkpoint_should_save(
                50_000,
                0.0,
                last_steps=10_000,
                last_seconds=0.0,
            )
        )
        self.assertTrue(
            training.checkpoint_should_save(
                20_000,
                1_801.0,
                last_steps=10_000,
                last_seconds=0.0,
            )
        )


class TrainLocoJumpCheckpointPrefixTest(unittest.TestCase):
    """验证新旧 checkpoint 前缀隔离。"""

    def test_checkpoint_prefix_accepts_only_new_namespace(self) -> None:
        accepted = (
            "yobogo_loco_jump_v1",
            "yobogo_loco_jump_v1_phase4",
            "yobogo_loco_jump_v1-run",
            "yobogo_loco_jump_v1.20261001",
        )
        rejected = (
            "",
            "  ",
            "ppo_walk",
            "phase0_smoke",
            "p4_stairs",
            "yobogo_loco_jump_v1beta",
            "yobogo_loco_jump_v10",
            "yobogo_loco_jump",
        )
        for prefix in accepted:
            with self.subTest(prefix=prefix):
                self.assertEqual(training.validate_checkpoint_prefix(prefix), prefix)
        for prefix in rejected:
            with self.subTest(prefix=prefix):
                with self.assertRaises(ValueError):
                    training.validate_checkpoint_prefix(prefix)

    def test_resume_path_rejects_old_prefix(self) -> None:
        accepted = (
            "yobogo_loco_jump_v1_final.zip",
            "checkpoints/yobogo_loco_jump_v1_50000_steps.zip",
        )
        rejected = (
            "ppo_walk_final.zip",
            "checkpoints/phase1_stand_final.zip",
            "yobogo_loco_jump_v1beta_final.zip",
            "yobogo_loco_jump_v10_final.zip",
        )
        for path in accepted:
            with self.subTest(path=path):
                self.assertEqual(training.validate_resume_path(path), Path(path))
        for path in rejected:
            with self.subTest(path=path):
                with self.assertRaises(ValueError):
                    training.validate_resume_path(path)


class TrainLocoJumpCheckpointCallbackTest(unittest.TestCase):
    """验证 checkpoint 回调严格遵守 SB3 的 model 注入生命周期。"""

    def test_constructor_does_not_access_model(self) -> None:
        callback = training.ContractCheckpointCallback(
            save_path="/tmp/yobogo_loco_jump_v1_callback",
            name_prefix="yobogo_loco_jump_v1",
        )
        self.assertFalse(hasattr(callback, "model"))
        self.assertIsNone(callback._last_steps)
        self.assertIsNone(callback._started_at)
        self.assertEqual(callback.save_count, 0)

    def test_constructor_rejects_old_prefix(self) -> None:
        for prefix in ("ppo_walk", "phase0_smoke", "yobogo_loco_jump_v1beta"):
            with self.subTest(prefix=prefix):
                with self.assertRaises(ValueError):
                    training.ContractCheckpointCallback(
                        save_path="/tmp/yobogo_loco_jump_v1_callback",
                        name_prefix=prefix,
                    )

    def test_model_injection_then_save_at_contract_step_interval(self) -> None:
        class FakeModel:
            """只记录 save() 调用，不产生真实 checkpoint。"""

            def __init__(self, num_timesteps: int = 0) -> None:
                self.num_timesteps = num_timesteps
                self.saved: list[str] = []

            def save(self, path: str) -> None:
                self.saved.append(path)

        with tempfile.TemporaryDirectory() as tmp:
            model = FakeModel(num_timesteps=0)
            callback = training.ContractCheckpointCallback(
                save_path=tmp,
                name_prefix="yobogo_loco_jump_v1",
            )
            callback.init_callback(model)
            self.assertIs(callback.model, model)
            self.assertEqual(callback._last_steps, 0)
            self.assertIsNotNone(callback._started_at)

            callback._on_training_start()
            # SB3 的 BaseCallback.on_step() 会从 model 回读累计步数。
            model.num_timesteps = 50_000
            self.assertTrue(callback.on_step())  # 第 50000 步触发
            self.assertEqual(callback.num_timesteps, 50_000)
            self.assertEqual(callback.save_count, 1)
            self.assertEqual(len(model.saved), 1)
            saved_path = Path(model.saved[0])
            self.assertEqual(saved_path.parent, Path(tmp))
            self.assertEqual(
                saved_path.name,
                "yobogo_loco_jump_v1_50000_steps.zip",
            )
            self.assertTrue(contract.is_isolated_checkpoint(saved_path))


class TrainLocoJumpCliDryRunTest(unittest.TestCase):
    """验证 CLI 参数覆盖和 dry-run 不进入训练路径。"""

    def test_parse_args_accepts_phase_tags_and_devices(self) -> None:
        for tag in training.TAGS:
            for device in training.DEVICES:
                with self.subTest(tag=tag, device=device):
                    args = training.parse_args(
                        [
                            "--dry-run",
                            "--tag",
                            tag,
                            "--device",
                            device,
                            "--ckpt-prefix",
                            "yobogo_loco_jump_v1_test",
                        ]
                    )
                    self.assertTrue(args.dry_run)
                    self.assertEqual(args.tag, tag)
                    self.assertEqual(args.device, device)
                    self.assertEqual(args.total_steps, training.stage_target_steps(tag))
                    self.assertEqual(
                        args.checkpoint_interval,
                        contract.CHECKPOINT_INTERVAL_STEPS,
                    )

    def test_parse_args_rejects_old_prefix_old_resume_and_bad_budget(self) -> None:
        invalid_args = (
            ["--dry-run", "--ckpt-prefix", "ppo_walk"],
            ["--dry-run", "--resume", "checkpoints/ppo_walk_final.zip"],
            ["--dry-run", "--resume", "checkpoints/yobogo_loco_jump_v1beta_final.zip"],
            ["--dry-run", "--tag", "phase3", "--total-steps", "799999"],
            ["--dry-run", "--tag", "phase4", "--total-steps", "1200001"],
            ["--dry-run", "--checkpoint-interval", "20000"],
            ["--dry-run", "--device", "gpu"],
            ["--dry-run", "--tag", "phase5"],
            ["--dry-run", "--eval-interval", "-1"],
            ["--dry-run", "--eval-episodes", "0"],
        )
        for argv in invalid_args:
            with self.subTest(argv=argv):
                with self.assertRaises(SystemExit):
                    training.parse_args(argv)

    def test_main_dry_run_does_not_call_train_or_create_outputs(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            logdir = Path(tmp) / "logs"
            ckptdir = Path(tmp) / "checkpoints"
            output = io.StringIO()
            with mock.patch.object(
                training,
                "train",
                side_effect=AssertionError("dry-run 不得进入 train()"),
            ) as train_mock:
                with contextlib.redirect_stdout(output):
                    result = training.main(
                        [
                            "--dry-run",
                            "--tag",
                            "phase4",
                            "--device",
                            "cpu",
                            "--ckpt-prefix",
                            "yobogo_loco_jump_v1_dry",
                            "--ckptdir",
                            str(ckptdir),
                            "--logdir",
                            str(logdir),
                        ]
                    )
            self.assertEqual(result, 0)
            train_mock.assert_not_called()
            self.assertFalse(logdir.exists())
            self.assertFalse(ckptdir.exists())
            self.assertIn("未启动（dry-run）", output.getvalue())
            self.assertIn("yobogo_loco_jump_v1_dry", output.getvalue())
            self.assertIn("1200000", output.getvalue())

    def test_main_dry_run_accepts_resume_without_loading_checkpoint(self) -> None:
        with mock.patch.object(
            training,
            "train",
            side_effect=AssertionError("dry-run 不得加载模型或训练"),
        ) as train_mock:
            result = training.main(
                [
                    "--dry-run",
                    "--tag",
                    "phase3",
                    "--device",
                    "cuda",
                    "--resume",
                    "checkpoints/yobogo_loco_jump_v1_50000_steps.zip",
                ]
            )
        self.assertEqual(result, 0)
        train_mock.assert_not_called()


if __name__ == "__main__":
    unittest.main()
