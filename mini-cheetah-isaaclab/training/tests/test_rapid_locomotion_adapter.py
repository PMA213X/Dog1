"""Rapid Locomotion 冻结策略适配器的独立单元测试。"""

from __future__ import annotations

import unittest

import torch

from training.external_policy.rapid_locomotion.adapter import (
    ACTION_DIM,
    HISTORY_DIM,
    HISTORY_LEN,
    OBS_DIM,
    RAPID_DEFAULT_JOINT_POS,
    RAPID_REPLAY_RAW_ACTION_LIMIT,
    RapidLocomotionAdapter,
    build_rapid_observation,
    map_yobogo_command_to_rapid,
    rapid_action_to_yobogo_action,
)
from training.external_policy.rapid_locomotion.model import (
    DEFAULT_WEIGHTS_DIR,
    RapidLocomotionModel,
)


def make_yobogo_observation(num_envs: int = 1) -> torch.Tensor:
    """构造可精确核对各字段的 48 维 YoboGo 观测。"""
    observation = torch.zeros(num_envs, 48, dtype=torch.float32)
    observation[:, 6:9] = torch.tensor([0.1, 0.2, -0.9])
    # YoboGo 命令换算后应为 Rapid 命令 [0.2,0.1,0.2]。
    observation[:, 9:12] = torch.tensor([0.1, 0.05, 0.8])
    observation[:, 15:27] = 0.0
    observation[:, 27:39] = torch.arange(12, dtype=torch.float32)
    return observation


class FakeRapidPolicy:
    """记录调用次数与入参的最小假策略。"""

    def __init__(self) -> None:
        self.calls = 0
        self.captured_current: list[torch.Tensor] = []
        self.captured_history: list[torch.Tensor] = []

    def __call__(
        self, current_observation: torch.Tensor, history: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        self.calls += 1
        self.captured_current.append(current_observation.clone())
        self.captured_history.append(history.clone())
        raw_action = torch.full_like(current_observation[:, :ACTION_DIM], float(self.calls))
        return raw_action, torch.zeros(current_observation.shape[0], 18)


class RapidObservationTests(unittest.TestCase):
    """覆盖 48→42 字段、命令、默认角、速度和原始上一动作。"""

    def test_yobogo_48_to_rapid_42(self) -> None:
        observation = make_yobogo_observation()
        previous_action = torch.arange(12, dtype=torch.float32).unsqueeze(0)
        rapid_observation = build_rapid_observation(observation, previous_action)

        self.assertEqual(tuple(rapid_observation.shape), (1, OBS_DIM))
        torch.testing.assert_close(rapid_observation[0, 0:3], observation[0, 6:9])
        torch.testing.assert_close(
            rapid_observation[0, 3:6], torch.tensor([0.2, 0.1, 0.2])
        )
        # joint_pos_rel=0 还原为 YoboGo SIM 初态，再减 Rapid 默认角；
        # 后腿 axis 与 rpy 成对等效，不做额外符号反转。
        expected_q_default = torch.tensor(
            [0.0, -0.785398163, 1.865468294] * 4
        ) - RAPID_DEFAULT_JOINT_POS
        torch.testing.assert_close(rapid_observation[0, 6:18], expected_q_default)
        expected_joint_vel = torch.arange(12, dtype=torch.float32) * 0.05
        torch.testing.assert_close(rapid_observation[0, 18:30], expected_joint_vel)
        torch.testing.assert_close(rapid_observation[0, 30:42], previous_action[0])

    def test_observation_rejects_wrong_shape(self) -> None:
        with self.assertRaisesRegex(ValueError, "YoboGo observation"):
            build_rapid_observation(
                torch.zeros(1, 42, dtype=torch.float32),
                torch.zeros(1, ACTION_DIM, dtype=torch.float32),
            )

    def test_rear_haa_axis_sign(self) -> None:
        observation = make_yobogo_observation()
        observation[0, 15 + 6] = 0.2
        observation[0, 15 + 9] = -0.2
        rapid_observation = build_rapid_observation(
            observation, torch.zeros(1, ACTION_DIM)
        )
        # 槽位直接同序映射，0.2/-0.2 映射为 0.2/-0.2。
        self.assertAlmostEqual(float(rapid_observation[0, 6 + 6]), 0.3, places=6)
        self.assertAlmostEqual(float(rapid_observation[0, 6 + 9]), -0.3, places=6)

    def test_command_clips_before_scale(self) -> None:
        command = torch.tensor([[10.0, -10.0, 10.0]])
        rapid_command = map_yobogo_command_to_rapid(command)
        torch.testing.assert_close(rapid_command, torch.tensor([[0.6, -0.6, 0.25]]))


class RapidActionTests(unittest.TestCase):
    """覆盖默认角、动作尺度、后腿 HAA 符号和安全裁剪。"""

    def test_zero_raw_action_targets_rapid_defaults(self) -> None:
        observation = make_yobogo_observation()
        yobogo_action, safe_target, safety_clipped, *_ = rapid_action_to_yobogo_action(
            torch.zeros(1, ACTION_DIM), observation
        )
        expected_rapid_target = RAPID_DEFAULT_JOINT_POS.clone()
        torch.testing.assert_close(safe_target[0], expected_rapid_target)
        torch.testing.assert_close(yobogo_action[0] * 0.5 + torch.tensor(
            [0.0, -0.785398163, 1.865468294] * 4
        ), expected_rapid_target)
        self.assertFalse(bool(safety_clipped.any()))

    def test_action_scale_and_safety_clip(self) -> None:
        observation = make_yobogo_observation()
        extreme_action = torch.full((1, ACTION_DIM), 100.0)
        yobogo_action, safe_target, _safety_clipped, *_ = rapid_action_to_yobogo_action(
            extreme_action, observation
        )
        self.assertTrue(bool(torch.isfinite(yobogo_action).all()))
        self.assertTrue(bool((safe_target >= -5.0 - 1.0e-6).all()))
        self.assertTrue(bool((safe_target <= 5.0 + 1.0e-6).all()))
        # OOD 原始动作先收敛到 ±2，再按官方 Kp/Kd/力矩计算安全目标。
        current_pos = torch.tensor(
            [0.0, -0.785398163, 1.865468294] * 4, dtype=torch.float32
        )
        current_vel = torch.arange(12, dtype=torch.float32)
        torque = 20.0 * (safe_target[0] - current_pos) - 0.5 * current_vel
        self.assertTrue(bool((torque[0::3].abs() <= 18.0 + 1.0e-5).all()))
        self.assertTrue(bool((torque[1::3].abs() <= 18.0 + 1.0e-5).all()))
        self.assertTrue(bool((torque[2::3].abs() <= 26.0 + 1.0e-5).all()))

    def test_yobogo_action_formula(self) -> None:
        observation = make_yobogo_observation()
        # 使用 OOD 门限内的动作，避免测试公式时触发额外安全收敛。
        rapid_action = torch.full((1, ACTION_DIM), 0.2)
        yobogo_action, *_ = rapid_action_to_yobogo_action(rapid_action, observation)
        yobogo_default = torch.tensor(
            [0.0, -0.785398163, 1.865468294] * 4
        )
        expected = (
            2.0 * (RAPID_DEFAULT_JOINT_POS - yobogo_default)
            + 2.0 * torch.tensor([0.125, 0.25, 0.25] * 4) * rapid_action[0]
        )
        torch.testing.assert_close(yobogo_action[0], expected)

    def test_history_keeps_raw_policy_action(self) -> None:
        class ConstantPolicy:
            def __call__(self, current, history):
                return (
                    torch.full((current.shape[0], ACTION_DIM), 20.0),
                    torch.zeros(current.shape[0], 18),
                )

        adapter = RapidLocomotionAdapter(ConstantPolicy(), 1, "cpu")
        step = adapter.step(make_yobogo_observation())
        self.assertTrue(
            torch.equal(step.policy_raw_action, torch.full((1, ACTION_DIM), 20.0))
        )
        self.assertTrue(
            torch.equal(
                step.applied_rapid_action,
                torch.full(
                    (1, ACTION_DIM), RAPID_REPLAY_RAW_ACTION_LIMIT
                ),
            )
        )
        self.assertTrue(
            torch.equal(
                adapter.previous_rapid_action,
                torch.full((1, ACTION_DIM), 20.0),
            )
        )


class RapidScheduleTests(unittest.TestCase):
    """覆盖 reset 清零、630 维历史和每 10 个控制周期一次推理。"""

    def test_reset_clears_all_state(self) -> None:
        policy = FakeRapidPolicy()
        adapter = RapidLocomotionAdapter(policy, num_envs=2, device="cpu")
        adapter.step(make_yobogo_observation(2))
        adapter.step(make_yobogo_observation(2))
        adapter.reset()
        self.assertTrue(bool((adapter.history == 0).all()))
        self.assertTrue(bool((adapter.previous_rapid_action == 0).all()))
        self.assertTrue(bool((adapter.held_yobogo_action == 0).all()))
        self.assertTrue(bool((adapter.control_cycle == 0).all()))

    def test_10_cycle_schedule_and_history_shape(self) -> None:
        policy = FakeRapidPolicy()
        adapter = RapidLocomotionAdapter(policy, num_envs=1, device="cpu")
        inference_flags: list[bool] = []
        held_actions: list[torch.Tensor] = []
        for _ in range(21):
            step = adapter.step(make_yobogo_observation())
            inference_flags.append(step.did_infer)
            held_actions.append(step.yobogo_action.clone())

        self.assertEqual(policy.calls, 3)
        self.assertEqual(
            inference_flags,
            [True] + [False] * 9 + [True] + [False] * 9 + [True],
        )
        self.assertEqual(tuple(adapter.history.shape), (1, HISTORY_LEN, OBS_DIM))
        self.assertEqual(tuple(adapter.history.reshape(1, HISTORY_DIM).shape), (1, HISTORY_DIM))
        self.assertTrue(torch.equal(held_actions[1], held_actions[9]))
        self.assertTrue(torch.equal(held_actions[11], held_actions[19]))
        # 复现官方 play.py：reset 后首次推理历史全零。
        self.assertTrue(bool((policy.captured_history[0] == 0).all()))
        self.assertEqual(int((policy.captured_history[1].reshape(15, 42) != 0).any(dim=-1).sum()), 1)
        # 第二次推理时 previous action 仍是第一次原始动作。
        torch.testing.assert_close(
            policy.captured_current[1][0, 30:42],
            torch.ones(ACTION_DIM, dtype=torch.float32),
        )

    def test_partial_reset_only_clears_selected_env(self) -> None:
        policy = FakeRapidPolicy()
        adapter = RapidLocomotionAdapter(policy, num_envs=2, device="cpu")
        for _ in range(11):
            adapter.step(make_yobogo_observation(2))
        adapter.reset(torch.tensor([1]))
        self.assertFalse(bool((adapter.history[0] == 0).all()))
        self.assertTrue(bool((adapter.history[1] == 0).all()))
        self.assertEqual(int(adapter.control_cycle[0]), 11)
        self.assertEqual(int(adapter.control_cycle[1]), 0)


class RapidModelTests(unittest.TestCase):
    """覆盖官方 JIT 路径、加载、630→18 和 60→12 dummy forward。"""

    def test_real_jit_load_and_dummy_forward(self) -> None:
        model = RapidLocomotionModel(weights_dir=DEFAULT_WEIGHTS_DIR, device="cpu")
        raw_action, latent = model.dummy_forward(num_envs=2)
        self.assertEqual(tuple(latent.shape), (2, 18))
        self.assertEqual(tuple(raw_action.shape), (2, ACTION_DIM))
        self.assertTrue(bool(torch.isfinite(raw_action).all()))
        self.assertTrue(model.adaptation_path.name == "adaptation_module_latest.jit")
        self.assertTrue(model.body_path.name == "body_latest.jit")


if __name__ == "__main__":
    unittest.main()
