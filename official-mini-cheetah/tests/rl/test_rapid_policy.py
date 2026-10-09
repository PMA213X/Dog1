#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Rapid Mini Cheetah 冻结策略适配测试。"""

from __future__ import annotations

import hashlib
import json
import sys
import unittest
import warnings
from pathlib import Path

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rl import contract
from rl.rapid_policy import (
    CURRENT_ACTION_SCALES,
    JOINT_MAPPING,
    RAPID_MODEL_DIR,
    SOURCE_ACTION_SCALES,
    SOURCE_COMMIT,
    SOURCE_DEFAULT_ANGLES,
    SOURCE_HISTORY_DIM,
    SOURCE_HISTORY_LENGTH,
    SOURCE_OBS_DIM,
    SOURCE_TO_CURRENT_INDICES,
    RapidPolicyAdapter,
    projected_gravity_from_rpy,
    validate_joint_mapping,
    verify_asset_manifest,
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class RapidPolicyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        warnings.filterwarnings(
            "ignore",
            message=".*torch.jit.load.*",
            category=FutureWarning,
        )
        cls.adapter = RapidPolicyAdapter()

    @staticmethod
    def _observation(
        *,
        rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
        command: tuple[float, float, float] = (0.0, 0.0, 0.0),
        q: np.ndarray | None = None,
        dq: np.ndarray | None = None,
        previous_action: np.ndarray | None = None,
    ) -> np.ndarray:
        observation = np.zeros(contract.OBS_DIM, dtype=np.float32)
        observation[contract.OBS_SLICES["rpy"]] = rpy
        observation[contract.OBS_SLICES["cmd"]] = command
        observation[contract.OBS_SLICES["q"]] = (
            np.asarray(contract.DEFAULT_CROUCH, dtype=np.float32)
            if q is None
            else np.asarray(q, dtype=np.float32)
        )
        observation[contract.OBS_SLICES["dq"]] = (
            np.zeros(contract.ACTION_DIM, dtype=np.float32)
            if dq is None
            else np.asarray(dq, dtype=np.float32)
        )
        if previous_action is not None:
            observation[contract.OBS_SLICES["prev_action"]] = np.asarray(
                previous_action,
                dtype=np.float32,
            )
        return observation

    def test_manifest_and_checkpoint_hashes(self) -> None:
        manifest = json.loads(
            (RAPID_MODEL_DIR / "manifest.json").read_text(encoding="utf-8")
        )
        self.assertEqual(manifest["commit"], SOURCE_COMMIT)
        self.assertEqual(manifest["license"], "MIT")
        checkpoint_names = (
            "ac_weights_last.pt",
            "body_latest.jit",
            "adaptation_module_latest.jit",
            "parameters.pkl",
        )
        for name in checkpoint_names:
            self.assertIn(name, manifest["files"])
            self.assertEqual(
                _sha256(RAPID_MODEL_DIR / name),
                manifest["files"][name]["sha256"],
            )
        verified = verify_asset_manifest()
        self.assertEqual(
            set(verified),
            {
                "ac_weights_last.pt",
                "body_latest.jit",
                "adaptation_module_latest.jit",
                "parameters.pkl",
                "mini_cheetah.urdf",
                "LICENSE",
            },
        )

    def test_real_asset_shapes_and_network_outputs(self) -> None:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", FutureWarning)
            adaptation = torch.jit.load(
                str(RAPID_MODEL_DIR / "adaptation_module_latest.jit"),
                map_location="cpu",
            ).eval()
            body = torch.jit.load(
                str(RAPID_MODEL_DIR / "body_latest.jit"),
                map_location="cpu",
            ).eval()
            weights = torch.load(
                str(RAPID_MODEL_DIR / "ac_weights_last.pt"),
                map_location="cpu",
                weights_only=True,
            )
        self.assertEqual(
            tuple(weights["adaptation_module.0.weight"].shape),
            (256, SOURCE_HISTORY_DIM),
        )
        self.assertEqual(
            tuple(weights["actor_body.0.weight"].shape),
            (512, SOURCE_OBS_DIM + 18),
        )
        self.assertEqual(
            tuple(weights["actor_body.6.weight"].shape),
            (12, 128),
        )
        with torch.inference_mode():
            latent = adaptation(torch.zeros(1, SOURCE_HISTORY_DIM))
            action = body(torch.zeros(1, SOURCE_OBS_DIM + 18))
        self.assertEqual(tuple(latent.shape), (1, 18))
        self.assertEqual(tuple(action.shape), (1, contract.ACTION_DIM))
        self.assertTrue(bool(torch.isfinite(latent).all()))
        self.assertTrue(bool(torch.isfinite(action).all()))

    def test_joint_mapping_is_explicit_and_invertible(self) -> None:
        mapping = validate_joint_mapping()
        self.assertEqual(mapping["source_commit"], SOURCE_COMMIT)
        self.assertEqual(mapping["source_to_current_indices"], tuple(range(12)))
        self.assertEqual(mapping["joint_signs"], (1,) * 12)
        self.assertEqual(
            tuple(item[0] for item in JOINT_MAPPING),
            (
                "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
            ),
        )
        self.assertEqual(
            tuple(item[1] for item in JOINT_MAPPING),
            (
                "fr_abd", "fr_hip", "fr_kn",
                "fl_abd", "fl_hip", "fl_kn",
                "hr_abd", "hr_hip", "hr_kn",
                "hl_abd", "hl_hip", "hl_kn",
            ),
        )
        self.assertTrue(all(item[3] == 1 for item in JOINT_MAPPING))
        current = np.arange(12, dtype=np.float32)
        source = self.adapter._map_current_to_source(current)
        recovered = self.adapter._map_source_to_current(source)
        np.testing.assert_array_equal(recovered, current)
        np.testing.assert_array_equal(
            source[np.asarray(SOURCE_TO_CURRENT_INDICES)],
            current,
        )

    def test_forty_two_observation_fields_and_scales(self) -> None:
        rpy = (0.3, -0.2, 0.7)
        command = (0.4, -0.25, 1.5)
        q = np.asarray(SOURCE_DEFAULT_ANGLES, dtype=np.float32)
        q_offset = np.linspace(-0.4, 0.4, 12, dtype=np.float32)
        q = q + q_offset
        dq = np.linspace(-2.0, 2.0, 12, dtype=np.float32)
        previous = np.linspace(-1.0, 1.0, 12, dtype=np.float32)
        observation = self._observation(
            rpy=rpy,
            command=command,
            q=q,
            dq=dq,
            previous_action=previous,
        )
        result = self.adapter.build_source_observation(
            observation,
            previous_source_action=previous,
        )
        self.assertEqual(result.shape, (SOURCE_OBS_DIM,))
        np.testing.assert_allclose(
            result[0:3],
            projected_gravity_from_rpy(rpy),
            rtol=0.0,
            atol=1e-6,
        )
        np.testing.assert_allclose(
            result[3:6],
            np.asarray(command) * np.asarray((2.0, 2.0, 0.25)),
            rtol=0.0,
            atol=1e-6,
        )
        np.testing.assert_allclose(
            result[6:18],
            q_offset,
            rtol=0.0,
            atol=1e-6,
        )
        np.testing.assert_allclose(
            result[18:30],
            dq * 0.05,
            rtol=0.0,
            atol=1e-6,
        )
        np.testing.assert_allclose(
            result[30:42],
            previous,
            rtol=0.0,
            atol=1e-6,
        )

        clipped = self._observation(q=np.full(12, 1000.0))
        clipped_result = self.adapter.build_source_observation(clipped)
        self.assertTrue(np.all(np.abs(clipped_result) <= 100.0))

    def test_projected_gravity_zero_and_tilt(self) -> None:
        np.testing.assert_allclose(
            projected_gravity_from_rpy((0.0, 0.0, 1.23)),
            (0.0, 0.0, -1.0),
            atol=1e-7,
        )
        roll, pitch = 0.4, -0.3
        expected = (
            np.sin(pitch),
            -np.cos(pitch) * np.sin(roll),
            -np.cos(pitch) * np.cos(roll),
        )
        np.testing.assert_allclose(
            projected_gravity_from_rpy((roll, pitch, 2.0)),
            expected,
            atol=1e-7,
        )

    def test_history_fifteen_frames_and_first_frame_padding(self) -> None:
        first = np.full(SOURCE_OBS_DIM, -2.0, dtype=np.float32)
        history, warmup = self.adapter.build_history(first)
        self.assertEqual(history.shape, (SOURCE_HISTORY_DIM,))
        self.assertTrue(warmup)
        np.testing.assert_array_equal(
            history.reshape(SOURCE_HISTORY_LENGTH, SOURCE_OBS_DIM),
            np.tile(first, (SOURCE_HISTORY_LENGTH, 1)),
        )

        frames = [
            np.full(SOURCE_OBS_DIM, float(index), dtype=np.float32)
            for index in range(SOURCE_HISTORY_LENGTH - 1)
        ]
        current = np.full(SOURCE_OBS_DIM, 14.0, dtype=np.float32)
        history, warmup = self.adapter.build_history(
            current,
            previous_frames=frames,
        )
        self.assertFalse(warmup)
        np.testing.assert_array_equal(
            history.reshape(SOURCE_HISTORY_LENGTH, SOURCE_OBS_DIM),
            np.stack([*frames, current]),
        )

        too_many = [
            np.full(SOURCE_OBS_DIM, float(index), dtype=np.float32)
            for index in range(SOURCE_HISTORY_LENGTH + 4)
        ]
        history, warmup = self.adapter.build_history(
            current,
            previous_frames=too_many,
        )
        self.assertFalse(warmup)
        np.testing.assert_array_equal(
            history.reshape(SOURCE_HISTORY_LENGTH, SOURCE_OBS_DIM),
            np.stack([*too_many[-(SOURCE_HISTORY_LENGTH - 1):], current]),
        )

    def test_action_scale_conversion_is_reversible_and_clipped(self) -> None:
        source = np.ones(12, dtype=np.float32)
        mapped = self.adapter.map_source_action_to_current(source)
        self.assertEqual(mapped.shape, (12,))
        self.assertTrue(np.all(np.abs(mapped) <= 1.0))
        np.testing.assert_allclose(
            mapped * np.asarray(CURRENT_ACTION_SCALES),
            source * np.asarray(SOURCE_ACTION_SCALES),
            rtol=0.0,
            atol=1e-6,
        )
        expected = np.tile(
            (0.125 / 0.30, 0.25 / 0.50, 0.25 / 0.50),
            4,
        )
        np.testing.assert_allclose(mapped, expected, rtol=0.0, atol=1e-6)

        clipped = self.adapter.map_source_action_to_current(
            np.full(12, 1000.0, dtype=np.float32)
        )
        np.testing.assert_array_equal(clipped, np.ones(12, dtype=np.float32))

    def test_predict_returns_twelve_finite_actions_and_info(self) -> None:
        self.adapter.reset()
        observation = self._observation(
            command=(0.2, 0.0, 0.1),
            q=np.asarray(contract.DEFAULT_CROUCH, dtype=np.float32),
        )
        action, info = self.adapter.predict(observation, deterministic=True)
        self.assertEqual(action.shape, (contract.ACTION_DIM,))
        self.assertTrue(np.all(np.isfinite(action)))
        self.assertTrue(np.all(np.abs(action) <= 1.0))
        self.assertEqual(len(info["raw_source_action"]), 12)
        self.assertEqual(len(info["mapped_yobo_action"]), 12)
        self.assertEqual(len(info["latent"]), 18)
        self.assertEqual(len(info["observation_42"]), 42)
        self.assertEqual(len(info["history_630"]), 630)
        self.assertTrue(info["history_warmup"])
        self.assertEqual(info["history_frames"], 1)
        self.assertEqual(info["source_commit"], SOURCE_COMMIT)
        json.dumps(info)

        for step in range(1, SOURCE_HISTORY_LENGTH):
            _, info = self.adapter.predict(observation)
            self.assertEqual(info["history_frames"], step + 1)
            self.assertEqual(
                info["history_warmup"],
                step + 1 < SOURCE_HISTORY_LENGTH,
            )
        self.adapter.reset()
        self.assertEqual(self.adapter._history, [])
        np.testing.assert_array_equal(
            self.adapter._previous_source_action,
            np.zeros(12, dtype=np.float32),
        )

    def test_non_finite_inputs_and_outputs_are_rejected(self) -> None:
        observation = self._observation()
        observation[5] = np.nan
        with self.assertRaisesRegex(ValueError, "57 维观测"):
            self.adapter.predict(observation)

        valid = self._observation()
        with self.assertRaisesRegex(ValueError, "源动作"):
            self.adapter.map_source_action_to_current(
                np.full(12, np.nan, dtype=np.float32)
            )

        self.adapter.reset()
        original_body = self.adapter.body

        class NonFiniteBody:
            def __call__(self, _value: torch.Tensor) -> torch.Tensor:
                return torch.full((1, 12), float("nan"))

        self.adapter.body = NonFiniteBody()
        try:
            with self.assertRaisesRegex(RuntimeError, "非有限 latent 或动作"):
                self.adapter.predict(valid)
        finally:
            self.adapter.body = original_body
        self.assertEqual(self.adapter._history, [])

    def test_jump_request_is_recorded_as_unsupported(self) -> None:
        self.adapter.reset()
        _action, info = self.adapter.predict(
            self._observation(),
            jump_request=True,
        )
        self.assertFalse(info["jump_request_supported"])
        self.assertTrue(info["jump_request_rejected"])
        self.assertEqual(info["unsupported_requests"], ["jump"])


if __name__ == "__main__":
    unittest.main()
