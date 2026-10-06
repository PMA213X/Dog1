#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""平直地面 RL 契约、奖励、环境、入口和 world 的离线测试。"""

from __future__ import annotations

import unittest
import sys
from pathlib import Path

import numpy as np

RL_DIR = Path(__file__).resolve().parents[1]
if str(RL_DIR) not in sys.path:
    sys.path.insert(0, str(RL_DIR))

import flat_jump_contract as contract
import flat_jump_reward as reward_module
from flat_jump_env import FlatGroundJumpEnv
from train_flat_jump import parse_args as parse_train_args, phase_ppo_params
from eval_flat_jump import parse_args as parse_eval_args
from run_flat_jump_stages import parse_args as parse_stage_args, stage_command


PROJECT_ROOT = Path(__file__).resolve().parents[3]
FLAT_WORLD = PROJECT_ROOT / "webots-sim" / "worlds" / "flat_jump_rl.wbt"


def make_state(**overrides: object) -> dict[str, object]:
    """构造通过旧公共状态校验的合成 TCP 状态。"""
    state: dict[str, object] = {
        "type": "state",
        "q": [0.0] * 12,
        "dq": [0.0] * 12,
        "rpy": [0.0, 0.0, 0.0],
        "omega": [0.0, 0.0, 0.0],
        "v_body": [0.0, 0.0, 0.0],
        "contacts": [1, 1, 1, 1],
        "height": 0.26,
        "jump_phase": 0.0,
        "done": False,
        "x": 0.0,
        "y": 0.0,
    }
    state.update(overrides)
    return state


class ContractTests(unittest.TestCase):
    """57 维观测、命令、阶段和 checkpoint 边界。"""

    def test_observation_layout_is_57(self) -> None:
        self.assertEqual(contract.OBS_DIM, 57)
        self.assertEqual(contract.ACTION_DIM, 12)
        self.assertEqual(
            sum(
                item.stop - item.start
                for item in contract.OBS_SLICES.values()
            ),
            57,
        )
        self.assertEqual(
            contract.OBS_SLICES["terrain_height"], slice(55, 56)
        )
        self.assertEqual(
            contract.OBS_SLICES["terrain_flat_flag"], slice(56, 57)
        )

    def test_phase_budget_and_checkpoint_prefix(self) -> None:
        self.assertEqual(
            contract.PHASE_TOTAL_STEPS,
            {"F0": 5000, "F1": 350000, "F2": 650000, "F3": 1200000},
        )
        self.assertEqual(contract.F1_FIXED_RECOVERY_STEPS, 260000)
        self.assertEqual(contract.F1_RECOVERY_TOTAL_STEPS, 350000)
        accepted = contract.validate_checkpoint_path(
            "checkpoints/yobogo_flat_jump_v1_final.zip"
        )
        self.assertEqual(accepted.name, "yobogo_flat_jump_v1_final.zip")
        with self.assertRaises(ValueError):
            contract.validate_checkpoint_path(
                "checkpoints/yobogo_loco_jump_v1_final.zip"
            )

    def test_command_limits(self) -> None:
        self.assertEqual(
            contract.clip_command((9.0, -9.0, 9.0)),
            (0.6, -0.3, 1.0),
        )
        self.assertEqual(
            contract.clip_command((-9.0, 9.0, -9.0)),
            (-0.3, 0.3, -1.0),
        )


class RewardTests(unittest.TestCase):
    """奖励有限性、阶段开关与非有限值失败。"""

    @staticmethod
    def inputs(phase: str, **changes: object) -> reward_module.RewardInputs:
        values: dict[str, object] = {
            "phase": phase,
            "roll": 0.0,
            "pitch": 0.0,
            "height": 0.26,
            "command": np.zeros(3),
            "velocity": np.zeros(3),
            "heading_error": 0.0,
            "contacts": np.ones(4),
            "action": np.zeros(12),
            "previous_action": np.zeros(12),
            "jump_latched": False,
            "jump_peak_gain": 0.0,
            "jump_success_event": False,
            "jump_landing_event": False,
            "fallen": False,
        }
        values.update(changes)
        return reward_module.RewardInputs(**values)  # type: ignore[arg-type]

    def test_phase_weight_switches(self) -> None:
        stable = reward_module.compute_reward(self.inputs("F1"))
        moving = reward_module.compute_reward(
            self.inputs(
                "F2",
                command=np.array([0.6, 0.0, 0.0]),
                velocity=np.array([0.3, 0.0, 0.0]),
            )
        )
        self.assertLess(moving.values["command"], stable.values["command"])
        self.assertGreater(moving.values["velocity"], stable.values["velocity"])
        self.assertEqual(stable.values["jump_success"], 0.0)
        self.assertEqual(reward_module.WEIGHTS["F1"]["fall"], -100.0)
        self.assertEqual(reward_module.WEIGHTS["F1"]["command"], -1.0)
        self.assertEqual(reward_module.WEIGHTS["F2"]["command"], -1.5)
        self.assertEqual(reward_module.WEIGHTS["F3"]["command"], -1.5)

    def test_f1_twenty_step_fall_is_negative_and_standing_is_positive(self) -> None:
        fall_total = 0.0
        for step in range(20):
            fall_total += reward_module.compute_reward(
                self.inputs("F1", fallen=step == 19)
            ).total
        stable_total = sum(
            reward_module.compute_reward(self.inputs("F1")).total
            for _ in range(20)
        )
        self.assertLess(fall_total, 0.0)
        self.assertGreater(stable_total, 0.0)

    def test_jump_and_fall_events(self) -> None:
        result = reward_module.compute_reward(
            self.inputs(
                "F3",
                jump_latched=True,
                jump_peak_gain=0.10,
                jump_success_event=True,
                jump_landing_event=True,
                fallen=True,
            )
        )
        self.assertGreater(result.values["jump_height"], 0.0)
        self.assertEqual(result.values["jump_success"], 8.0)
        self.assertEqual(result.values["jump_landing"], 4.0)
        self.assertEqual(result.values["fall"], -20.0)
        self.assertTrue(np.isfinite(result.total))

    def test_nonfinite_reward_input_fails(self) -> None:
        with self.assertRaises(ValueError):
            reward_module.compute_reward(
                self.inputs("F3", height=float("nan"))
            )


class EnvironmentTests(unittest.TestCase):
    """无 Webots 的平地环境纯逻辑测试。"""

    @classmethod
    def setUpClass(cls) -> None:
        cls.env = FlatGroundJumpEnv(phase="F2", start_bridge=False)
        cls.env._send = lambda payload: None  # type: ignore[assignment]
        cls.env._recv = lambda: make_state()  # type: ignore[assignment]

    @classmethod
    def tearDownClass(cls) -> None:
        cls.env.close()

    def test_reset_observation_and_command(self) -> None:
        obs, info = self.env.reset(seed=7, options={"command": [9, -9, 9]})
        self.assertEqual(obs.shape, (57,))
        self.assertTrue(np.all(np.isfinite(obs)))
        self.assertEqual(float(obs[55]), 0.0)
        self.assertEqual(float(obs[56]), 1.0)
        np.testing.assert_allclose(
            obs[45:48], np.array([0.6, -0.3, 1.0], dtype=np.float32)
        )
        self.assertEqual(info["phase"], "F2")
        self.assertIn(
            info["randomization"]["delay_steps"],
            range(contract.DELAY_STEPS_RANGE[0], contract.DELAY_STEPS_RANGE[1] + 1),
        )

    def test_action_is_finite_clipped_and_rate_limited(self) -> None:
        self.env.reset(seed=8, options={"command": [0.0, 0.0, 0.0]})
        safe = self.env._sanitize_action(np.full(12, 9.0))
        self.assertTrue(np.all(safe <= contract.ACTION_RATE_LIMIT + 1e-7))
        self.assertTrue(np.all(safe >= -contract.ACTION_RATE_LIMIT - 1e-7))
        with self.assertRaises(ValueError):
            self.env._sanitize_action(np.full(12, np.nan))

    def test_termination_boundaries(self) -> None:
        self.assertTrue(
            FlatGroundJumpEnv._fallen(make_state(x=8.01))
        )
        self.assertTrue(
            FlatGroundJumpEnv._fallen(make_state(y=-8.01))
        )
        self.assertTrue(
            FlatGroundJumpEnv._fallen(make_state(height=0.61))
        )
        self.assertFalse(FlatGroundJumpEnv._fallen(make_state()))

    def test_fixed_and_full_randomization_boundaries(self) -> None:
        fixed = FlatGroundJumpEnv(
            phase="F1",
            randomization_mode="fixed",
            start_bridge=False,
        )
        fixed._send = lambda payload: None  # type: ignore[assignment]
        fixed._recv = lambda: make_state()  # type: ignore[assignment]
        _, info = fixed.reset(seed=11)
        self.assertEqual(info["randomization"], {
            "friction": 1.2,
            "mass_scale": 1.0,
            "delay_steps": 0,
        })

        full = FlatGroundJumpEnv(
            phase="F1",
            randomization_mode="full",
            start_bridge=False,
        )
        full._send = lambda payload: None  # type: ignore[assignment]
        full._recv = lambda: make_state()  # type: ignore[assignment]
        _, info = full.reset(seed=12)
        values = info["randomization"]
        self.assertTrue(contract.FRICTION_RANGE[0] <= values["friction"] <= contract.FRICTION_RANGE[1])
        self.assertTrue(contract.MASS_SCALE_RANGE[0] <= values["mass_scale"] <= contract.MASS_SCALE_RANGE[1])
        self.assertIn(values["delay_steps"], range(3))
        self.assertEqual(info["randomization_mode"], "full")

    def test_curriculum_gradually_reaches_full_range(self) -> None:
        env = FlatGroundJumpEnv(
            phase="F1",
            randomization_mode="curriculum",
            start_bridge=False,
        )
        env._send = lambda payload: None  # type: ignore[assignment]
        env._recv = lambda: make_state()  # type: ignore[assignment]
        for index in range(contract.F1_CURRICULUM_EPISODES):
            _, info = env.reset(seed=20 + index)
            if index < contract.F1_CURRICULUM_FIXED_EPISODES:
                self.assertEqual(info["randomization"]["friction"], 1.2)
                self.assertEqual(info["randomization"]["delay_steps"], 0)
        values = info["randomization"]
        self.assertTrue(contract.FRICTION_RANGE[0] <= values["friction"] <= contract.FRICTION_RANGE[1])
        self.assertTrue(contract.MASS_SCALE_RANGE[0] <= values["mass_scale"] <= contract.MASS_SCALE_RANGE[1])
        self.assertIn(values["delay_steps"], range(3))


class CliTests(unittest.TestCase):
    """训练、评估和阶段执行器 dry-run 参数。"""

    def test_training_dry_run_is_cuda_only(self) -> None:
        args = parse_train_args(["--phase", "F3", "--dry-run"])
        self.assertEqual(args.device, "cuda")
        self.assertEqual(args.total_steps, 1200000)
        with self.assertRaises(SystemExit):
            parse_train_args(
                ["--phase", "F3", "--device", "cpu", "--dry-run"]
            )

    def test_f1_only_uses_recovery_ppo_params(self) -> None:
        f1 = phase_ppo_params("F1")
        f2 = phase_ppo_params("F2")
        f3 = phase_ppo_params("F3")
        self.assertEqual(f1["learning_rate"], 1e-4)
        self.assertEqual(f1["clip_range"], 0.1)
        self.assertEqual(f1["n_epochs"], 5)
        self.assertEqual(f2["learning_rate"], 3e-4)
        self.assertEqual(f2["clip_range"], 0.2)
        self.assertEqual(f2["n_epochs"], 10)
        self.assertEqual(f3, f2)

    def test_stage_commands_are_sequential_and_resume(self) -> None:
        first = stage_command("F0", None)
        second = stage_command("F1", Path("checkpoint.zip"))
        self.assertIn("cuda", first)
        self.assertIn("1200000", stage_command("F3", None))
        self.assertIn("--resume", second)
        self.assertIn("--webots-gui", second)
        self.assertEqual(parse_stage_args(["--dry-run"]).dry_run, True)

    def test_evaluation_dry_run(self) -> None:
        args = parse_eval_args(["--phase", "F2", "--dry-run"])
        self.assertEqual(args.phase, "F2")
        self.assertTrue(args.dry_run)
        self.assertEqual(args.randomization_mode, "full")
        with self.assertRaises(SystemExit):
            parse_eval_args([
                "--phase",
                "F1",
                "--gate",
                "--randomization-mode",
                "fixed",
                "--dry-run",
            ])


class WorldTests(unittest.TestCase):
    """平地 world 静态边界。"""

    @classmethod
    def setUpClass(cls) -> None:
        cls.text = FLAT_WORLD.read_text(encoding="utf-8")

    def test_only_flat_floor_and_yobogo_robot(self) -> None:
        self.assertTrue(FLAT_WORLD.is_file())
        self.assertIn('name "floor"', self.text)
        self.assertIn("size 20 20 0.1", self.text)
        self.assertNotIn('name "hurdle"', self.text)
        self.assertNotIn("stairs", self.text.lower())
        self.assertNotIn("ElevationGrid", self.text)
        self.assertEqual(self.text.count("Robot {"), 1)
        self.assertIn('controller "rl_agent"', self.text)
        self.assertIn("supervisor TRUE", self.text)
        self.assertIn("translation 0 0 0.26", self.text)


if __name__ == "__main__":
    unittest.main(verbosity=2)
