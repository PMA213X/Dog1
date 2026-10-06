#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""yobogo_loco_jump_v1 环境契约单元测试（mock TCP state，不启动 Webots）。"""

from __future__ import annotations

import math
import sys
import unittest
from collections import deque
from typing import Any, Deque, Dict, Iterable, List

import numpy as np
from pathlib import Path


RL_DIR = Path(__file__).resolve().parents[1]
if str(RL_DIR) not in sys.path:
    sys.path.insert(0, str(RL_DIR))

import loco_jump_contract as contract  # noqa: E402
from loco_jump_env import (  # noqa: E402
    ACTION_SCALE,
    ACTION_SCALE_LEG,
    LocoJumpEnv,
    Q_STAND,
    QuadrupedLocoJumpEnv,
    action_to_q_des,
    make_env,
)


def state(
    *,
    q: Iterable[float] | None = None,
    dq: Iterable[float] | None = None,
    rpy: Iterable[float] | None = None,
    omega: Iterable[float] | None = None,
    v_body: Iterable[float] | None = None,
    contacts: Iterable[float] | None = None,
    height: float = 0.26,
    jump_phase: float = 0.0,
    done: bool = False,
) -> Dict[str, Any]:
    """构造符合公共契约的 TCP state。"""
    return {
        "type": "state",
        "q": list(q if q is not None else np.zeros(12)),
        "dq": list(dq if dq is not None else np.zeros(12)),
        "rpy": list(rpy if rpy is not None else np.zeros(3)),
        "omega": list(omega if omega is not None else np.zeros(3)),
        "v_body": list(v_body if v_body is not None else np.zeros(3)),
        "contacts": list(contacts if contacts is not None else np.ones(4)),
        "height": float(height),
        "jump_phase": float(jump_phase),
        "done": bool(done),
    }


class MockLocoJumpEnv(QuadrupedLocoJumpEnv):
    """只在内存中回放 mock state 的环境，不建立 socket 或 Webots 进程。"""

    def __init__(self, states: Iterable[Dict[str, Any]], **kwargs: Any) -> None:
        self.sent: List[Dict[str, Any]] = []
        self._mock_states: Deque[Dict[str, Any]] = deque(states)
        kwargs["start_bridge"] = False
        super().__init__(**kwargs)

    def _send(self, obj: Dict[str, Any]) -> None:
        self.sent.append(dict(obj))

    def _recv(self) -> Dict[str, Any]:
        if not self._mock_states:
            raise AssertionError("mock state 不足，测试未准备足够的 reset/step 响应")
        return self._mock_states.popleft()


class LocoJumpActionContractTest(unittest.TestCase):
    """验证 12 维动作和 q_des 映射公式。"""

    def test_action_scale_and_q_des_formula(self) -> None:
        self.assertEqual(ACTION_SCALE_LEG, (0.3, 0.5, 0.5))
        np.testing.assert_allclose(
            ACTION_SCALE,
            np.tile(np.asarray(ACTION_SCALE_LEG, dtype=np.float32), 4),
        )
        self.assertEqual(Q_STAND.shape, (contract.ACTION_DIM,))

        q_stand = np.linspace(-1.0, 1.0, contract.ACTION_DIM, dtype=np.float32)
        action = np.linspace(1.0, -1.0, contract.ACTION_DIM, dtype=np.float32)
        expected = q_stand + action * np.tile(
            np.asarray(ACTION_SCALE_LEG, dtype=np.float32), 4
        )
        actual = action_to_q_des(action, q_stand)
        np.testing.assert_allclose(actual, expected, rtol=0.0, atol=1e-7)
        self.assertEqual(actual.shape, (contract.ACTION_DIM,))
        self.assertTrue(np.all(np.isfinite(actual)))

    def test_action_mapping_rejects_invalid_inputs(self) -> None:
        with self.assertRaises(ValueError):
            action_to_q_des(np.zeros(11))
        with self.assertRaises(ValueError):
            action_to_q_des(np.zeros(12), np.zeros(11))
        with self.assertRaises(ValueError):
            action_to_q_des(np.full(12, np.nan))
        with self.assertRaises(ValueError):
            action_to_q_des(np.zeros(12), np.full(12, np.inf))


class MockLocoJumpEnvInterfaceTest(unittest.TestCase):
    """用 mock state 进行多步 reset/step 接口验收。"""

    @staticmethod
    def make_env(
        states: Iterable[Dict[str, Any]],
        *,
        tag: str = "phase2",
        **kwargs: Any,
    ) -> MockLocoJumpEnv:
        return MockLocoJumpEnv(
            states,
            tag=tag,
            max_episode_steps=kwargs.pop("max_episode_steps", 20),
            **kwargs,
        )

    def test_factory_and_spaces_without_starting_bridge(self) -> None:
        env = make_env(tag="phase0", start_bridge=False)
        try:
            self.assertIsInstance(env, LocoJumpEnv)
            self.assertEqual(env.observation_space.shape, (contract.OBS_DIM,))
            self.assertEqual(env.action_space.shape, (contract.ACTION_DIM,))
            self.assertEqual(env.action_space.low.tolist(), [-1.0] * 12)
            self.assertEqual(env.action_space.high.tolist(), [1.0] * 12)
        finally:
            env.close()

    def test_multi_step_reset_and_step_shapes_and_info_keys(self) -> None:
        states = [
            state(q=np.linspace(0.0, 0.11, 12), height=0.26),
            state(
                q=np.linspace(0.1, 0.21, 12),
                rpy=(0.01, -0.02, 0.03),
                v_body=(0.1, 0.0, 0.0),
                height=0.27,
                jump_phase=0.02,
            ),
            state(
                q=np.linspace(0.2, 0.31, 12),
                rpy=(0.02, -0.01, 0.06),
                v_body=(0.2, 0.0, 0.0),
                height=0.28,
                jump_phase=0.04,
            ),
            state(
                q=np.linspace(0.3, 0.41, 12),
                rpy=(0.03, -0.02, 0.09),
                v_body=(0.3, 0.0, 0.0),
                height=0.29,
                jump_phase=0.06,
            ),
        ]
        env = self.make_env(states, tag="phase2")
        try:
            reset_obs, reset_info = env.reset(seed=7)
            self.assertEqual(reset_obs.shape, (contract.OBS_DIM,))
            self.assertEqual(reset_obs.dtype, np.float32)
            self.assertTrue(np.all(np.isfinite(reset_obs)))

            required_info_keys = {
                "command_error",
                "jump_success",
                "fallen",
                "heading_error",
                "terrain_success",
            }
            self.assertTrue(required_info_keys.issubset(reset_info))

            for step_index, action in enumerate(
                (
                    np.linspace(-0.9, 0.9, 12, dtype=np.float32),
                    np.linspace(0.8, -0.8, 12, dtype=np.float32),
                    np.zeros(12, dtype=np.float32),
                ),
                start=1,
            ):
                obs, reward, terminated, truncated, info = env.step(action)
                self.assertEqual(obs.shape, (contract.OBS_DIM,))
                self.assertEqual(obs.dtype, np.float32)
                self.assertTrue(np.all(np.isfinite(obs)))
                self.assertTrue(math.isfinite(float(reward)))
                self.assertIsInstance(terminated, bool)
                self.assertIsInstance(truncated, bool)
                self.assertTrue(required_info_keys.issubset(info))
                for key in required_info_keys:
                    if key in {"jump_success", "fallen", "terrain_success"}:
                        self.assertIsInstance(info[key], bool)
                    else:
                        self.assertTrue(math.isfinite(float(info[key])))
                self.assertEqual(info["episode_steps"], step_index)

                act_message = env.sent[-1]
                self.assertEqual(act_message["type"], "act")
                self.assertEqual(len(act_message["a"]), contract.ACTION_DIM)
                self.assertEqual(len(act_message["q_des"]), contract.ACTION_DIM)
                np.testing.assert_allclose(
                    np.asarray(act_message["q_des"], dtype=np.float32),
                    action_to_q_des(
                        np.clip(action, contract.ACTION_LOW, contract.ACTION_HIGH)
                    ),
                    rtol=0.0,
                    atol=1e-7,
                )

            # 第二步起，prev_action 切片来自上一拍动作。
            np.testing.assert_allclose(
                obs[contract.OBS_SLICES["prev_action"]],
                np.linspace(0.8, -0.8, 12, dtype=np.float32),
                rtol=0.0,
                atol=1e-7,
            )
            self.assertEqual(
                obs[contract.OBS_SLICES["q"]].shape,
                (contract.OBS_FIELD_DIMS["q"],),
            )
        finally:
            env.close()

    def test_observation_contract_slices_from_mock_state(self) -> None:
        q = np.linspace(0.1, 1.2, 12, dtype=np.float32)
        dq = np.linspace(-1.2, -0.1, 12, dtype=np.float32)
        rpy = np.array([0.11, -0.22, 0.33], dtype=np.float32)
        v_body = np.array([0.44, -0.55, 0.66], dtype=np.float32)
        omega = np.array([0.77, -0.88, 0.99], dtype=np.float32)
        contacts = np.array([1.0, 0.0, 1.0, 0.0], dtype=np.float32)
        env = self.make_env(
            [
                state(
                    q=q,
                    dq=dq,
                    rpy=rpy,
                    v_body=v_body,
                    omega=omega,
                    contacts=contacts,
                    height=0.31,
                    jump_phase=0.37,
                )
            ],
            tag="phase4",
        )
        try:
            obs, info = env.reset(seed=3, options={"command": (2.0, -2.0, 2.0)})
            self.assertTrue(np.array_equal(obs[contract.OBS_SLICES["q"]], q))
            self.assertTrue(np.array_equal(obs[contract.OBS_SLICES["dq"]], dq))
            self.assertTrue(np.array_equal(obs[contract.OBS_SLICES["rpy"]], rpy))
            self.assertTrue(
                np.array_equal(obs[contract.OBS_SLICES["v_body"]], v_body)
            )
            self.assertTrue(
                np.array_equal(obs[contract.OBS_SLICES["omega_body"]], omega)
            )
            self.assertTrue(
                np.array_equal(obs[contract.OBS_SLICES["foot_contact"]], contacts)
            )
            self.assertEqual(obs[contract.OBS_SLICES["body_height"]][0], 0.31)
            self.assertEqual(obs[contract.OBS_SLICES["jump_phase_time"]][0], 0.37)
            self.assertEqual(obs[contract.OBS_SLICES["jump_request"]][0], 0.0)
            np.testing.assert_allclose(
                obs[contract.OBS_SLICES["cmd_vx_vy_wz"]],
                np.array([0.6, -0.3, 1.0], dtype=np.float32),
            )
            np.testing.assert_allclose(
                info["command"],
                [0.6, -0.3, 1.0],
                rtol=0.0,
                atol=1e-7,
            )
        finally:
            env.close()

    def test_action_clipping_is_applied_to_sent_action_and_q_des(self) -> None:
        env = self.make_env([state(), state()], tag="phase1")
        try:
            env.reset(seed=1)
            action = np.array(
                [-2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0, -2.0, 1.5, -1.5],
                dtype=np.float32,
            )
            _, _, _, _, _ = env.step(action)
            sent = env.sent[-1]
            np.testing.assert_allclose(
                sent["a"],
                np.clip(action, contract.ACTION_LOW, contract.ACTION_HIGH),
                rtol=0.0,
                atol=0.0,
            )
            expected_q_des = action_to_q_des(
                np.clip(action, contract.ACTION_LOW, contract.ACTION_HIGH)
            )
            np.testing.assert_allclose(sent["q_des"], expected_q_des, rtol=0.0, atol=1e-7)
        finally:
            env.close()

    def test_invalid_action_is_rejected(self) -> None:
        env = self.make_env([state(), state(), state()], tag="phase1")
        try:
            env.reset(seed=1)
            with self.assertRaises(ValueError):
                env.step(np.zeros(11, dtype=np.float32))
            with self.assertRaises(ValueError):
                env.step(np.full(12, np.nan, dtype=np.float32))
        finally:
            env.close()


class LocoJumpCommandAndJumpTest(unittest.TestCase):
    """验证命令限幅、阶段门控和跳跃信息不触碰真实进程。"""

    def test_command_limits_and_phase_gate(self) -> None:
        env = MockLocoJumpEnv(
            [state(), state()],
            tag="phase2",
            start_bridge=False,
        )
        try:
            _, info = env.reset(seed=11, options={"command": (9.0, -9.0, 9.0)})
            np.testing.assert_allclose(
                info["command"],
                [0.6, -0.3, 1.0],
                rtol=0.0,
                atol=1e-7,
            )
            np.testing.assert_allclose(
                env.command,
                [0.6, -0.3, 1.0],
                rtol=0.0,
                atol=1e-7,
            )
            self.assertEqual(env._target_yaw, 0.0)
        finally:
            env.close()

        phase0 = MockLocoJumpEnv([state(), state()], tag="phase0", start_bridge=False)
        try:
            _, info = phase0.reset(seed=11, options={"command": (9.0, -9.0, 9.0)})
            np.testing.assert_allclose(info["command"], [0.0, 0.0, 0.0])
            self.assertFalse(phase0.command_enabled)
            with self.assertRaises(RuntimeError):
                phase0.request_jump()
        finally:
            phase0.close()

    def test_jump_latch_and_success_info_without_webots(self) -> None:
        env = MockLocoJumpEnv(
            [
                state(contacts=(1.0, 1.0, 1.0, 1.0), height=0.26, jump_phase=0.0),
                state(contacts=(0.0, 0.0, 0.0, 0.0), height=0.31, jump_phase=0.04),
                state(contacts=(1.0, 1.0, 1.0, 1.0), height=0.27, jump_phase=0.08),
            ],
            tag="phase4",
            start_bridge=False,
            jump_request_step=0,
        )
        try:
            _, reset_info = env.reset(seed=5, options={"jump_request": True})
            self.assertTrue(reset_info["jump_latched"])
            _, _, _, _, first_info = env.step(np.zeros(12, dtype=np.float32))
            self.assertTrue(first_info["jump_latched"])
            self.assertTrue(first_info["jump_success"])
            self.assertTrue(math.isfinite(first_info["heading_error"]))
            self.assertTrue(math.isfinite(first_info["command_error"]))
            self.assertFalse(first_info["terrain_success"])

            _, _, _, _, second_info = env.step(np.zeros(12, dtype=np.float32))
            self.assertTrue(second_info["jump_success"])
            self.assertTrue(second_info["terrain_success"])
            self.assertFalse(second_info["jump_latched"])
        finally:
            env.close()


if __name__ == "__main__":
    unittest.main()
