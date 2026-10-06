#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""训练遥测与阶段步数课程测试。"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rl.train import (
    TelemetryState,
    configure_phase_step_offset,
    parse_args,
    record_telemetry,
    update_telemetry,
)


class RecordingLogger:
    """记录 TensorBoard scalar 写入的最小替身。"""

    def __init__(self) -> None:
        self.values: dict[str, float] = {}

    def record(self, name: str, value: float) -> None:
        self.values[name] = float(value)


class TrainingTelemetryTests(unittest.TestCase):
    def test_parse_and_configure_phase_step_offset(self) -> None:
        args = parse_args(
            ["--phase", "P1", "--total-steps", "20000", "--dry-run"]
        )
        self.assertEqual(args.phase_step_offset, 0)
        workers = [SimpleNamespace(_phase_steps=0) for _ in range(4)]
        env = SimpleNamespace(envs=workers)
        configure_phase_step_offset(env, 100_000)
        self.assertEqual(
            [worker._phase_steps for worker in workers],
            [100_000] * 4,
        )
        with self.assertRaises(ValueError):
            configure_phase_step_offset(env, -1)

    def test_telemetry_records_episode_contact_and_termination(self) -> None:
        state = TelemetryState()
        update_telemetry(
            state,
            [
                {
                    "worker_id": 0,
                    "foot_contact_source": "node_id",
                    "foot_contacts": [1.0, 1.0, 1.0, 1.0],
                },
                {
                    "worker_id": 1,
                    "foot_contact_source": "node_id",
                    "foot_contacts": [1.0, 0.0, 0.0, 0.0],
                    "TimeLimit.truncated": True,
                },
            ],
            [2.0, 3.0],
            [False, True],
        )
        update_telemetry(
            state,
            [
                {
                    "worker_id": 0,
                    "foot_contact_source": "capability_missing_fail_closed",
                    "foot_contacts": [1.0, 1.0, 1.0, 1.0],
                }
            ],
            [4.0],
            [True],
        )
        logger = RecordingLogger()
        record_telemetry(logger, state)
        self.assertAlmostEqual(logger.values["rollout/ep_rew_mean"], 4.5)
        self.assertAlmostEqual(
            logger.values["rollout/true_four_contact_ratio"],
            0.5,
        )
        self.assertEqual(logger.values["termination_reason/time_limit"], 1.0)
        self.assertEqual(logger.values["termination_reason/unknown"], 1.0)
        self.assertEqual(logger.values["termination_reason/none"], 0.0)
        self.assertEqual(state.contact_samples, 0)
        self.assertEqual(state.four_contact_samples, 0)

    def test_telemetry_accepts_array_like_contacts(self) -> None:
        contact_values = [
            np.array([1.0, 1.0, 1.0, 1.0]),
            [1.0, 1.0, 1.0, 1.0],
            (1.0, 1.0, 1.0, 1.0),
        ]
        state = TelemetryState()
        update_telemetry(
            state,
            [
                {
                    "worker_id": index,
                    "foot_contact_source": "node_id",
                    "foot_contacts": contacts,
                }
                for index, contacts in enumerate(contact_values)
            ],
            [1.0, 2.0, 3.0],
            [False, False, False],
        )
        self.assertEqual(state.contact_samples, 3)
        self.assertEqual(state.four_contact_samples, 3)
        logger = RecordingLogger()
        record_telemetry(logger, state)
        self.assertAlmostEqual(
            logger.values["rollout/true_four_contact_ratio"],
            1.0,
        )

    def test_non_done_transitions_do_not_count_as_terminations(self) -> None:
        state = TelemetryState()
        update_telemetry(
            state,
            [{"worker_id": 0, "termination_reason": "height"}],
            [1.0],
            [False],
        )
        logger = RecordingLogger()
        record_telemetry(logger, state)
        self.assertEqual(
            sum(
                value
                for name, value in logger.values.items()
                if name.startswith("termination_reason/")
            ),
            0.0,
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
