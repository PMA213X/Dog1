#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""R3 play 观测、确定性推理、安全站立和独立急停测试。"""

from __future__ import annotations

import io
import json
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest import mock

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
CONTROLLER_ROOT = ROOT / "controllers" / "rl_agent"
for path in (ROOT, CONTROLLER_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from input_adapter import InputAdapter, InputFrame
from rl import contract
from rl.play import (
    EmergencyStop,
    JsonlPlayLogger,
    ObservationBuilder,
    PlayPolicy,
    PlayRuntime,
    load_rapid_play_model,
    main,
    parse_args,
    play_input_from_state,
    run_play_session,
)


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


class RecordingRapidModel:
    """记录 Rapid 专用 predict 参数并返回可审计 info。"""

    def __init__(self) -> None:
        self.calls: list[tuple[np.ndarray, bool, bool]] = []
        self.resets = 0

    def reset(self) -> None:
        self.resets += 1

    def predict(
        self,
        observation: np.ndarray,
        deterministic: bool,
        *,
        jump_request: bool = False,
    ) -> tuple[np.ndarray, dict[str, object]]:
        self.calls.append(
            (observation.copy(), bool(deterministic), bool(jump_request))
        )
        raw = np.full(contract.ACTION_DIM, 0.25, dtype=np.float32)
        mapped = np.full(contract.ACTION_DIM, 0.125, dtype=np.float32)
        return mapped, {
            "raw_source_action": raw.tolist(),
            "mapped_yobo_action": mapped.tolist(),
            "latent": [0.1] * 18,
            "observation_42": [0.0] * 42,
            "history_warmup": False,
            "history_frames": 15,
            "jump_request_supported": False,
            "jump_request_rejected": bool(jump_request),
            "unsupported_requests": ["jump"] if jump_request else [],
        }


class FakePlayEnv:
    """最小 TCP 环境替身，验证 max-steps、JSONL 和实际动作记录。"""

    def __init__(self) -> None:
        self._last_state = state()
        self.resets: list[dict[str, object]] = []
        self.actions: list[np.ndarray] = []

    def reset(self, *, seed: int, options: dict[str, object]):
        self.resets.append({"seed": seed, "options": options})
        self._last_state = state()
        return np.zeros(contract.OBS_DIM, dtype=np.float32), {}

    def step(self, action: np.ndarray):
        applied = np.asarray(action, dtype=np.float32) * 0.5
        self.actions.append(applied.copy())
        info = {
            "applied_action": applied,
            "foot_contacts": [1.0] * 4,
        }
        return np.zeros(contract.OBS_DIM, dtype=np.float32), 0.0, False, False, info


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

    def test_reset_releases_estop_to_safe_stand_without_policy_call(self) -> None:
        model = RecordingModel()
        policy = PlayPolicy(model)
        policy.step(
            state(),
            command=(0.6, 0.0, 0.0),
            input_frame=InputFrame(safe=True, reason="escape"),
        )
        result = policy.step(
            state(),
            command=(0.6, 0.0, 0.0),
            input_frame=InputFrame(safe=True, reason="reset"),
        )
        self.assertTrue(result.safe_stand)
        self.assertEqual(result.reason, "reset_release")
        self.assertFalse(result.estop)
        self.assertFalse(policy.emergency_stop.active)
        self.assertFalse(model.calls)

    def test_rapid_jump_is_rejected_and_not_used_for_zero_command(self) -> None:
        model = RecordingRapidModel()
        policy = PlayPolicy(model, model_type="rapid")
        zero_jump = policy.step(
            state(),
            command=(0.0, 0.0, 0.0),
            jump_request=True,
        )
        self.assertTrue(zero_jump.safe_stand)
        self.assertTrue(zero_jump.jump_requested)
        self.assertTrue(zero_jump.jump_rejected)
        self.assertFalse(model.calls)

        moving_jump = policy.step(
            state(),
            command=(0.2, 0.0, 0.0),
            jump_request=True,
        )
        self.assertFalse(moving_jump.safe_stand)
        self.assertTrue(moving_jump.jump_rejected)
        self.assertTrue(model.calls[0][2])
        self.assertFalse(
            moving_jump.model_info["jump_request_supported"]
        )

    def test_parse_rapid_cli_and_dry_run_do_not_load_sb3_checkpoint(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            args = parse_args(
                [
                    "--model-type",
                    "rapid",
                    "--pretrained-dir",
                    directory,
                    "--input",
                    "keyboard-joystick",
                    "--world",
                    "eval",
                    "--max-steps",
                    "7",
                    "--dry-run",
                ]
            )
            self.assertEqual(args.model_type, "rapid")
            self.assertEqual(args.input, "keyboard-joystick")
            self.assertEqual(args.world, "eval")
            self.assertEqual(args.max_steps, 7)
            output = io.StringIO()
            with (
                mock.patch("rl.play.load_play_model") as sb3_loader,
                mock.patch("rl.play.load_rapid_play_model") as rapid_loader,
                redirect_stdout(output),
            ):
                code = main(argv=[
                    "--model-type",
                    "rapid",
                    "--pretrained-dir",
                    directory,
                    "--max-steps",
                    "7",
                    "--dry-run",
                ])
            self.assertEqual(code, 0)
            sb3_loader.assert_not_called()
            rapid_loader.assert_not_called()
            self.assertIn("model_type=rapid", output.getvalue())
            self.assertIn("max_steps=7", output.getvalue())

    def test_rapid_loader_uses_fixed_adapter(self) -> None:
        sentinel = object()
        with mock.patch("rl.play.RapidPolicyAdapter", return_value=sentinel):
            result = load_rapid_play_model("/tmp/rapid-test", device="cpu")
        self.assertIs(result, sentinel)

    def test_main_passes_created_logger_into_session(self) -> None:
        """防止真实入口创建 JSONL 后却漏传给 run_play_session。"""
        with tempfile.TemporaryDirectory() as directory:
            logger = mock.Mock()
            env = mock.Mock()
            with (
                mock.patch(
                    "rl.play.load_rapid_play_model",
                    return_value=object(),
                ),
                mock.patch(
                    "rl.play.JsonlPlayLogger",
                    return_value=logger,
                ),
                mock.patch(
                    "rl.play.run_play_session",
                    return_value={
                        "steps": 1,
                        "estop": False,
                        "reason": "",
                    },
                ) as session,
                mock.patch(
                    "rl.play.MiniCheetahFlatJumpEnv",
                    create=True,
                    return_value=env,
                ),
            ):
                # main 在函数体内延迟导入环境类，因此同时补丁其真实来源。
                with mock.patch(
                    "rl.env.MiniCheetahFlatJumpEnv",
                    return_value=env,
                ):
                    code = main(
                        argv=[
                            "--model-type",
                            "rapid",
                            "--pretrained-dir",
                            directory,
                            "--max-steps",
                            "1",
                        ]
                    )
            self.assertEqual(code, 0)
            self.assertIs(
                session.call_args.kwargs["logger"],
                logger,
            )
            logger.close.assert_called_with()
            env.close.assert_called_with()

    def test_play_input_missing_from_controller_fails_closed(self) -> None:
        payload = play_input_from_state({})
        self.assertEqual(payload["keys"], [])
        self.assertEqual(payload["axes"], [0.0, 0.0, 0.0])
        self.assertTrue(
            payload["last_input_at"] < payload["now"] - 0.35
        )
        self.assertFalse(payload["disconnected"])

    def test_estop_session_continues_until_explicit_reset(self) -> None:
        env = FakePlayEnv()
        model = RecordingModel()
        samples = iter(
            [
                {
                    "keys": [0x0100001B],
                    "axes": [0.0, 0.0, 0.0],
                    "buttons": [],
                    "now": 10.0,
                    "last_input_at": 10.0,
                    "disconnected": False,
                },
                {
                    "keys": [ord("r")],
                    "axes": [0.0, 0.0, 0.0],
                    "buttons": [],
                    "now": 10.1,
                    "last_input_at": 10.1,
                    "disconnected": False,
                },
            ]
        )
        summary = run_play_session(
            env,
            model,
            lambda: next(samples),
            max_steps=2,
        )
        self.assertEqual(summary["steps"], 2)
        self.assertFalse(summary["estop"])
        self.assertEqual(summary["reason"], "")

    def test_max_steps_and_jsonl_record_complete_execution_delta(self) -> None:
        env = FakePlayEnv()
        model = RecordingRapidModel()
        with tempfile.TemporaryDirectory() as directory:
            logger = JsonlPlayLogger(directory)
            summary = run_play_session(
                env,
                model,
                lambda: {
                    "keys": [ord("w")],
                    "axes": [0.0, 0.0, 0.0],
                    "buttons": [],
                    "now": 10.0,
                    "last_input_at": 10.0,
                    "disconnected": False,
                },
                model_type="rapid",
                max_steps=3,
                logger=logger,
            )
            self.assertEqual(summary["steps"], 3)
            self.assertEqual(model.resets, 1)
            self.assertEqual(len(env.actions), 3)
            path = logger.path
            records = [
                json.loads(line)
                for line in path.read_text(encoding="utf-8").splitlines()
            ]
        self.assertEqual(len(records), 3)
        first = records[0]
        self.assertEqual(first["step"], 1)
        self.assertEqual(len(first["raw_source_action"]), 12)
        self.assertEqual(len(first["mapped_yobo_action"]), 12)
        self.assertEqual(len(first["applied_action"]), 12)
        self.assertEqual(len(first["execution_delta"]), 12)
        self.assertEqual(len(first["target_rate_limit_delta"]), 12)
        self.assertEqual(first["contacts"], [1.0] * 4)
        self.assertFalse(first["safe_stand"])
        self.assertFalse(first["emergency_stop"])
        self.assertEqual(first["history_frames"], 15)


if __name__ == "__main__":
    unittest.main(verbosity=2)
