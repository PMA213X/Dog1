#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Rapid 自定义 SB3 微调策略与 Dict VecEnv 测试。"""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

import gymnasium as gym
import numpy as np
import torch as th
from gymnasium import spaces
from stable_baselines3.common.vec_env import DummyVecEnv

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rl import contract
from rl.rapid_finetune_policy import (
    RAPID_PPO_PARAMS,
    RAPID_ACTOR_LR,
    RAPID_ADAPTATION_LR,
    RAPID_CRITIC_LR,
    RAPID_STAGE_ACTOR_START,
    RapidActorCriticPolicy,
    RapidPPO,
)
from rl.rapid_finetune_vec_env import (
    COMMAND_RESAMPLE_STEPS,
    RapidFinetuneVecEnv,
)
from rl.rapid_policy import RapidActionMapper, RapidPolicyAdapter
from rl.reward import source_action_saturation_penalty
from rl.train import parse_args


def _observation_spaces() -> spaces.Dict:
    return spaces.Dict(
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


class FakeWorker:
    """只提供命令课程与观测刷新所需的最小 worker。"""

    def __init__(self, worker_id: int) -> None:
        self.worker_id = worker_id
        self._phase_steps = 0
        self.finetune_mode = False
        self._command = np.zeros(3, dtype=np.float32)
        self._command_fixed = False
        self._command_step = 0
        self.observation = np.zeros(contract.OBS_DIM, dtype=np.float32)

    def _observation(self) -> np.ndarray:
        return self.observation.copy()


class FakeSharedVecEnv:
    """模拟共享 world 的同步终止与 terminal observation 契约。"""

    def __init__(self) -> None:
        self.num_envs = contract.PARALLEL_WORKERS
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(contract.OBS_DIM,),
            dtype=np.float32,
        )
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(contract.ACTION_DIM,),
            dtype=np.float32,
        )
        self.envs = [FakeWorker(index) for index in range(self.num_envs)]
        self.received: list[np.ndarray] = []
        self.done_once = False
        self.preexisting_saturation = 0.0
        self.terminal_value = 9.0
        self.fresh_value = 1.0
        self.step_value = 2.0

    def reset(self) -> np.ndarray:
        observations = []
        for worker in self.envs:
            worker.observation.fill(0.0)
            observations.append(worker._observation())
        return np.stack(observations).astype(np.float32)

    def step_async(self, actions: np.ndarray) -> None:
        self.received.append(np.asarray(actions, dtype=np.float32).copy())

    def step_wait(
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[dict]]:
        if self.done_once:
            terminal = np.full(
                (self.num_envs, contract.OBS_DIM),
                self.terminal_value,
                dtype=np.float32,
            )
            fresh = np.full(
                (self.num_envs, contract.OBS_DIM),
                self.fresh_value,
                dtype=np.float32,
            )
            dones = np.ones(self.num_envs, dtype=bool)
            infos = [
                {
                    "terminal_observation": terminal[index].copy(),
                    "reward_parts": {
                        "source_action_saturation": (
                            self.preexisting_saturation
                        ),
                    },
                }
                for index in range(self.num_envs)
            ]
            for index, worker in enumerate(self.envs):
                worker.observation = fresh[index].copy()
            self.done_once = False
            rewards = np.ones(self.num_envs, np.float32) + float(
                self.preexisting_saturation
            )
            return fresh, rewards, dones, infos
        rewards = np.ones(self.num_envs, np.float32) + float(
            self.preexisting_saturation
        )
        infos = [
            {
                "reward_parts": {
                    "source_action_saturation": (
                        self.preexisting_saturation
                    ),
                },
            }
            for _ in range(self.num_envs)
        ]
        observations = np.full(
            (self.num_envs, contract.OBS_DIM),
            self.step_value,
            dtype=np.float32,
        )
        return (
            observations,
            rewards,
            np.zeros(self.num_envs, dtype=bool),
            infos,
        )

    def close(self) -> None:
        return None


class DummyPolicyEnv(gym.Env):
    """PPO save/load 测试使用的无仿真 Dict 环境。"""

    def __init__(self) -> None:
        self.observation_space = _observation_spaces()
        self.action_space = spaces.Box(
            low=-100.0,
            high=100.0,
            shape=(contract.ACTION_DIM,),
            dtype=np.float32,
        )

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict | None = None,
    ) -> tuple[dict[str, np.ndarray], dict]:
        del seed, options
        return {
            "current": np.zeros(42, dtype=np.float32),
            "history": np.zeros(630, dtype=np.float32),
        }, {}

    def step(
        self,
        action: np.ndarray,
    ) -> tuple[dict[str, np.ndarray], float, bool, bool, dict]:
        del action
        return {
            "current": np.zeros(42, dtype=np.float32),
            "history": np.zeros(630, dtype=np.float32),
        }, 0.0, False, False, {}


class RapidFinetunePolicyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.observation_space = _observation_spaces()
        cls.action_space = spaces.Box(
            low=-100.0,
            high=100.0,
            shape=(contract.ACTION_DIM,),
            dtype=np.float32,
        )
        cls.policy = RapidActorCriticPolicy(
            cls.observation_space,
            cls.action_space,
            lambda _progress: RAPID_ACTOR_LR,
        )

    def test_rapid_cli_defaults_to_500k_fixed_p2(self) -> None:
        args = parse_args(
            ["--phase", "P2", "--init-from", "rapid", "--dry-run"]
        )
        self.assertEqual(args.total_steps, 500_000)
        self.assertEqual(args.randomization_mode, "fixed")
        self.assertTrue(Path(args.pretrained_dir).is_dir())
        with self.assertRaises(SystemExit):
            parse_args(
                ["--phase", "P1", "--init-from", "rapid", "--dry-run"]
            )

    def test_frozen_output_matches_play_adapter(self) -> None:
        adapter = RapidPolicyAdapter()
        adapter.reset()
        observation = np.zeros(contract.OBS_DIM, dtype=np.float32)
        observation[contract.OBS_SLICES["q"]] = contract.DEFAULT_CROUCH
        observation[contract.OBS_SLICES["cmd"]] = (0.14, 0.0, 0.0)
        _mapped, info = adapter.predict(observation, deterministic=True)
        model_observation = {
            "current": np.asarray(
                info["observation_42"],
                dtype=np.float32,
            )[None],
            "history": np.asarray(
                info["history_630"],
                dtype=np.float32,
            )[None],
        }
        with th.no_grad():
            action = self.policy.deterministic_source_action(
                {
                    key: th.from_numpy(value)
                    for key, value in model_observation.items()
                }
            )[0].numpy()
        expected = np.asarray(info["raw_source_action"], dtype=np.float32)
        self.assertLessEqual(float(np.max(np.abs(action - expected))), 1e-5)

    def test_stage_freezes_critic_actor_and_adaptation(self) -> None:
        self.policy.apply_finetune_stage(0, adaptation_enabled=False)
        self.assertFalse(
            all(
                parameter.requires_grad
                for parameter in self.policy.actor_body.parameters()
            )
        )
        self.assertFalse(
            all(
                parameter.requires_grad
                for parameter in self.policy.features_extractor.adaptation.parameters()
            )
        )
        self.assertTrue(
            all(
                parameter.requires_grad
                for parameter in self.policy.critic_body.parameters()
            )
        )
        self.policy.apply_finetune_stage(1, adaptation_enabled=False)
        self.assertTrue(
            all(
                parameter.requires_grad
                for parameter in self.policy.actor_body.parameters()
            )
        )
        self.assertFalse(self.policy.adaptation_enabled)
        self.policy.apply_finetune_stage(2, adaptation_enabled=True)
        self.assertTrue(self.policy.adaptation_enabled)
        self.assertTrue(
            all(
                parameter.requires_grad
                for parameter in self.policy.features_extractor.adaptation.parameters()
            )
        )

    def test_optimizer_has_independent_learning_rates(self) -> None:
        groups = {
            str(group["name"]): float(group["lr"])
            for group in self.policy.optimizer.param_groups
        }
        self.assertEqual(groups["actor"], RAPID_ACTOR_LR)
        self.assertEqual(groups["critic"], RAPID_CRITIC_LR)
        self.assertEqual(groups["adaptation"], RAPID_ADAPTATION_LR)
        self.assertEqual(RAPID_STAGE_ACTOR_START, 100_000)
        self.assertEqual(RAPID_PPO_PARAMS["target_kl"], 0.01)

    def test_ppo_save_and_load_round_trip(self) -> None:
        env = DummyVecEnv([DummyPolicyEnv for _ in range(4)])
        model = RapidPPO(
            RapidActorCriticPolicy,
            env,
            n_steps=8,
            batch_size=4,
            n_epochs=1,
            learning_rate=RAPID_ACTOR_LR,
            ent_coef=0.003,
            device="cpu",
            verbose=0,
        )
        model.learn(total_timesteps=8)
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "rapid.zip")
            model.save(path)
            loaded = RapidPPO.load(
                path,
                env=env,
                device="cpu",
                **RAPID_PPO_PARAMS,
            )
        observation = env.reset()
        action, _state = loaded.predict(observation, deterministic=True)
        self.assertEqual(action.shape, (4, contract.ACTION_DIM))
        self.assertTrue(np.all(np.isfinite(action)))
        env.close()


class RapidFinetuneVecEnvTests(unittest.TestCase):
    def setUp(self) -> None:
        self.base = FakeSharedVecEnv()
        self.env = RapidFinetuneVecEnv(
            self.base,  # type: ignore[arg-type]
            command_course=False,
            seed=1234,
        )

    def tearDown(self) -> None:
        self.env.close()

    def test_action_mapping_history_and_saturating_reward(self) -> None:
        observation = self.env.reset()
        self.assertTrue(
            all(worker.finetune_mode for worker in self.base.envs)
        )
        self.assertEqual(observation["current"].shape, (4, 42))
        self.assertEqual(observation["history"].shape, (4, 630))
        np.testing.assert_array_equal(
            observation["current"][0, 30:42],
            np.zeros(12, dtype=np.float32),
        )
        source = np.tile((4.8, 4.0, 4.0), 4).astype(np.float32)
        self.env.step_async(np.stack([source] * 4))
        np.testing.assert_allclose(
            self.base.received[-1],
            np.ones((4, 12), dtype=np.float32),
            atol=1e-6,
        )
        _obs, rewards, dones, infos = self.env.step_wait()
        self.assertFalse(np.any(dones))
        self.assertTrue(all(reward < 1.0 for reward in rewards))
        self.assertTrue(
            all(
                info["reward_parts"]["rapid_source_action_saturation"] < 0.0
                for info in infos
            )
        )
        np.testing.assert_allclose(
            _obs["current"][0, 30:42],
            source,
            atol=1e-6,
        )

    def test_saturation_penalty_is_added_exactly_once(self) -> None:
        self.env.reset()
        source = np.tile((4.8, 4.0, 4.0), 4).astype(np.float32)
        expected = float(source_action_saturation_penalty(source))
        self.assertLess(expected, 0.0)
        self.env.step_async(np.stack([source] * 4))
        _obs, rewards, _dones, infos = self.env.step_wait()
        np.testing.assert_allclose(rewards, 1.0 + expected)
        self.assertAlmostEqual(
            infos[0]["reward_parts"]["source_action_saturation"],
            expected,
        )

        self.base.preexisting_saturation = expected
        self.env.step_async(np.stack([source] * 4))
        _obs, rewards, _dones, infos = self.env.step_wait()
        np.testing.assert_allclose(rewards, 1.0 + expected)
        self.assertAlmostEqual(
            infos[0]["reward_parts"]["source_action_saturation"],
            expected,
        )

    def test_finetune_flag_can_be_disabled_for_legacy_wrapper(self) -> None:
        env = RapidFinetuneVecEnv(
            self.base,  # type: ignore[arg-type]
            command_course=False,
            finetune_mode=False,
            seed=1234,
        )
        env.reset()
        self.assertFalse(
            any(worker.finetune_mode for worker in self.base.envs)
        )
        env.close()

    def test_command_course_stays_low_speed_forward_and_zero(self) -> None:
        """100k 解冻后也不得扩展 lateral、yaw 或越界 forward。"""
        env = RapidFinetuneVecEnv(
            self.base,  # type: ignore[arg-type]
            command_course=True,
            seed=20261006,
        )
        try:
            for worker in self.base.envs:
                worker._phase_steps = 0
            samples_before = np.stack(
                [env._sample_command(0) for _ in range(8_000)]
            )
            forward = samples_before[:, 0] > 0.0
            zero = np.all(samples_before == 0.0, axis=1)
            self.assertTrue(np.all(samples_before[:, 1:] == 0.0))
            self.assertAlmostEqual(
                float(np.mean(forward)),
                0.75,
                delta=0.03,
            )
            self.assertAlmostEqual(float(np.mean(zero)), 0.25, delta=0.03)
            self.assertGreaterEqual(
                float(samples_before[forward, 0].min()),
                0.10,
            )
            self.assertLessEqual(
                float(samples_before[forward, 0].max()),
                0.18,
            )

            for worker in self.base.envs:
                worker._phase_steps = 100_000
            samples_after = np.stack(
                [env._sample_command(0) for _ in range(8_000)]
            )
            forward = samples_after[:, 0] > 0.0
            zero = np.all(samples_after == 0.0, axis=1)
            self.assertTrue(np.all(samples_after[:, 1:] == 0.0))
            self.assertAlmostEqual(
                float(np.mean(forward)),
                0.75,
                delta=0.03,
            )
            self.assertAlmostEqual(float(np.mean(zero)), 0.25, delta=0.03)
            self.assertGreaterEqual(
                float(samples_after[forward, 0].min()),
                0.10,
            )
            self.assertLessEqual(
                float(samples_after[forward, 0].max()),
                0.18,
            )
        finally:
            env.close()

    def test_command_holds_for_full_gate_episode(self) -> None:
        """命令不再在 5 s 时切换，必须覆盖 1000 步 Gate 漂移窗口。"""
        self.assertEqual(COMMAND_RESAMPLE_STEPS, contract.MAX_EPISODE_STEPS)
        env = RapidFinetuneVecEnv(
            self.base,  # type: ignore[arg-type]
            command_course=True,
            seed=20261006,
        )
        try:
            env.reset()
            commands = [
                worker._command.copy() for worker in self.base.envs
            ]
            env.step_async(
                np.zeros((self.base.num_envs, contract.ACTION_DIM))
            )
            _obs, _reward, _dones, _infos = env.step_wait()
            self.assertTrue(
                all(
                    np.array_equal(worker._command, expected)
                    for worker, expected in zip(self.base.envs, commands)
                )
            )
            self.assertTrue(np.all(env._command_age == 1))
        finally:
            env.close()

    def test_terminal_observation_uses_terminal_raw_and_clears_history(self) -> None:
        self.env.reset()
        for _step in range(14):
            self.env.step_async(
                np.zeros((4, contract.ACTION_DIM), dtype=np.float32)
            )
            _obs, _reward, dones, _infos = self.env.step_wait()
            self.assertFalse(np.any(dones))
        self.base.done_once = True
        self.env.step_async(
            np.ones((4, contract.ACTION_DIM), dtype=np.float32)
        )
        fresh, _reward, dones, infos = self.env.step_wait()
        self.assertTrue(np.all(dones))
        for index, info in enumerate(infos):
            terminal = info["terminal_observation"]
            self.assertAlmostEqual(
                float(terminal["current"][18]),
                0.45,
                places=4,
            )
            self.assertAlmostEqual(
                float(fresh["current"][index][18]),
                0.05,
                places=4,
            )
            self.assertEqual(
                float(fresh["current"][index][30]),
                0.0,
            )
        self.assertTrue(
            all(len(history) == 1 for history in self.env._histories)
        )
        self.assertTrue(
            all(
                np.allclose(history[0], fresh["current"][index])
                for index, history in enumerate(self.env._histories)
            )
        )

    def test_shared_mapping_facade_matches_adapter(self) -> None:
        mapper = RapidActionMapper()
        adapter = RapidPolicyAdapter()
        source = np.linspace(-5.0, 5.0, 12, dtype=np.float32)
        np.testing.assert_allclose(
            mapper.map_source_to_current(source),
            adapter.map_source_action_to_current(source),
            atol=0.0,
        )


if __name__ == "__main__":
    unittest.main()
