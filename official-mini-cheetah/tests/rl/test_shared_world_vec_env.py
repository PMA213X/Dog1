#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""共享单 world VecEnv 的锁步、reset 与关闭测试。"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import rl.contract as contract
from rl.shared_world_vec_env import SharedWorldVecEnv


class FakeProcess:
    """仅用于验证共享 Webots 关闭顺序的假进程。"""

    def __init__(self, events: list[str]) -> None:
        self.events = events
        self.terminated = False

    def poll(self) -> None:
        return None if not self.terminated else 0

    def terminate(self) -> None:
        self.terminated = True
        self.events.append("webots_terminate")

    def wait(self, timeout: float) -> int:
        del timeout
        return 0

    def kill(self) -> None:
        self.terminated = True
        self.events.append("webots_kill")


class FakeWorker:
    """记录批量 prepare/send/finish 顺序的最小环境替身。"""

    def __init__(self, worker_id: int, events: list[str]) -> None:
        self.worker_id = worker_id
        self.events = events
        self.done_ids: set[int] = set()
        self.closed = False
        self._webots = None
        self._owns_simulator = False
        self.reset_count = 0
        self.step_count = 0
        self.friction_values: list[float] = []

    def open_bridge(self) -> None:
        self.events.append(f"open_{self.worker_id}")

    def launch_webots(self) -> None:
        self.events.append("webots_launch")
        self._webots = FakeProcess(self.events)

    def launch_controller(self) -> None:
        self.events.append(f"controller_{self.worker_id}")

    def accept_bridge(self) -> None:
        self.events.append(f"handshake_{self.worker_id}")

    def prepare_reset(
        self,
        *,
        seed: int | None,
        shared_friction: float,
    ) -> dict[str, object]:
        self.events.append(f"reset_prepare_{self.worker_id}")
        self.friction_values.append(float(shared_friction))
        return {"seed": seed, "shared_friction": shared_friction}

    def send_prepared_reset(self) -> None:
        self.events.append(f"reset_send_{self.worker_id}")

    def finish_reset(self) -> tuple[np.ndarray, dict[str, object]]:
        self.events.append(f"reset_finish_{self.worker_id}")
        self.reset_count += 1
        observation = np.full(contract.OBS_DIM, 900.0 + self.worker_id, np.float32)
        return observation, {
            "worker_id": self.worker_id,
            "reset_count": self.reset_count,
        }

    def prepare_step(self, action: np.ndarray) -> dict[str, object]:
        del action
        self.events.append(f"step_prepare_{self.worker_id}")
        return {"type": "act"}

    def send_prepared_step(self) -> None:
        self.events.append(f"step_send_{self.worker_id}")

    def finish_step(self) -> tuple[np.ndarray, float, bool, bool, dict[str, object]]:
        self.events.append(f"step_finish_{self.worker_id}")
        self.step_count += 1
        observation = np.full(
            contract.OBS_DIM, 100.0 * self.step_count + self.worker_id, np.float32
        )
        return (
            observation,
            1.5,
            self.worker_id in self.done_ids,
            False,
            {"worker_id": self.worker_id, "fallen": self.worker_id in self.done_ids},
        )

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        self.events.append(f"env_close_{self.worker_id}")

    def request_exit(self) -> None:
        self.events.append(f"exit_{self.worker_id}")


def make_fake_vec(
    events: list[str],
    workers: list[FakeWorker] | None = None,
    *,
    start_runtime: bool = False,
) -> SharedWorldVecEnv:
    """创建不启动真实 Webots 的共享 VecEnv。"""
    pool = workers or []

    def factory(worker_id: int) -> FakeWorker:
        if pool:
            return pool[worker_id]
        return FakeWorker(worker_id, events)

    return SharedWorldVecEnv(  # type: ignore[arg-type]
        env_factory=factory,
        start_runtime=start_runtime,
        seed=1234,
    )


class SharedWorldVecEnvTests(unittest.TestCase):
    def test_startup_order_is_listener_webots_controller_handshake(self) -> None:
        events: list[str] = []
        vec = make_fake_vec(events, start_runtime=True)
        self.assertEqual(
            events,
            [
                "open_0", "open_1", "open_2", "open_3",
                "webots_launch",
                "controller_0", "controller_1", "controller_2", "controller_3",
                "handshake_0", "handshake_1", "handshake_2", "handshake_3",
            ],
        )
        vec.close()

    def test_step_strict_shape_and_send_all_before_receive(self) -> None:
        events: list[str] = []
        vec = make_fake_vec(events)
        vec.reset()
        events.clear()
        with self.assertRaises(ValueError):
            vec.step_async(np.zeros((3, contract.ACTION_DIM), dtype=np.float32))
        self.assertEqual(events, [])

        vec.step_async(np.zeros((4, contract.ACTION_DIM), dtype=np.float32))
        self.assertFalse(any("step_finish_" in event for event in events))
        self.assertEqual(
            [event for event in events if event.startswith("step_prepare_")],
            [f"step_prepare_{index}" for index in range(4)],
        )
        self.assertEqual(
            [event for event in events if event.startswith("step_send_")],
            [f"step_send_{index}" for index in range(4)],
        )
        events.clear()
        vec.step_wait()
        self.assertEqual(
            [event for event in events if event.startswith("step_finish_")],
            [f"step_finish_{index}" for index in range(4)],
        )
        vec.close()

    def test_any_done_forces_uniform_reset_and_terminal_info(self) -> None:
        events: list[str] = []
        workers = [FakeWorker(index, events) for index in range(4)]
        vec = make_fake_vec(events, workers)
        vec.reset()
        events.clear()
        workers[1].done_ids = {1}
        vec.step_async(np.zeros((4, contract.ACTION_DIM), dtype=np.float32))
        events.clear()
        observations, rewards, dones, infos = vec.step_wait()

        np.testing.assert_array_equal(dones, [True, True, True, True])
        np.testing.assert_allclose(rewards, [1.5] * 4)
        np.testing.assert_allclose(observations[:, 0], [900.0, 901.0, 902.0, 903.0])
        self.assertTrue(infos[1]["terminated"])
        self.assertNotIn("TimeLimit.truncated", infos[1])
        for index in (0, 2, 3):
            self.assertTrue(infos[index]["truncated"])
            self.assertFalse(infos[index]["terminated"])
            self.assertTrue(infos[index]["TimeLimit.truncated"])
            self.assertIn("terminal_observation", infos[index])
            self.assertEqual(
                infos[index]["terminal_observation"][0],
                100.0 * workers[index].step_count + index,
            )
        finishes = [event for event in events if event.startswith("step_finish_")]
        reset_prepares = [
            event for event in events if event.startswith("reset_prepare_")
        ]
        self.assertEqual(len(finishes), 4)
        self.assertEqual(len(reset_prepares), 4)
        self.assertLess(
            max(index for index, value in enumerate(events) if value.startswith("step_finish_")),
            min(index for index, value in enumerate(events) if value.startswith("reset_prepare_")),
        )
        vec.close()

    def test_reset_uses_one_shared_friction(self) -> None:
        events: list[str] = []
        workers = [FakeWorker(index, events) for index in range(4)]
        vec = make_fake_vec(events, workers)
        vec.reset()
        first = workers[0].friction_values[-1]
        self.assertEqual(
            [worker.friction_values[-1] for worker in workers],
            [first] * 4,
        )
        self.assertTrue(contract.FRICTION_RANGE[0] <= first <= contract.FRICTION_RANGE[1])
        vec.reset()
        second = workers[0].friction_values[-1]
        self.assertEqual([worker.friction_values[-1] for worker in workers], [second] * 4)
        vec.close()

    def test_close_is_idempotent_and_stops_controllers_before_webots(self) -> None:
        events: list[str] = []
        vec = make_fake_vec(events, start_runtime=True)
        vec.close()
        first = list(events)
        vec.close()
        self.assertEqual(events, first)
        close_indexes = [
            events.index(f"env_close_{index}") for index in range(4)
        ]
        self.assertLess(max(close_indexes), events.index("webots_terminate"))
        exit_indexes = [events.index(f"exit_{index}") for index in range(4)]
        self.assertLess(max(exit_indexes), min(close_indexes))
        self.assertIsNone(vec._webots)

    def test_hello_requires_all_lockstep_fields(self) -> None:
        hello = {
            "type": "hello",
            "contract": contract.CONTRACT_VERSION,
            "worker_id": 3,
            "robot_name": contract.ROBOT_NAMES[3],
            "timestep": contract.WEBOTS_TIMESTEP_MS,
        }
        SharedWorldVecEnv.validate_hello(
            hello,
            worker_id=3,
            robot_name=contract.ROBOT_NAMES[3],
        )
        with self.assertRaises(RuntimeError):
            SharedWorldVecEnv.validate_hello(
                {**hello, "timestep": 8},
                worker_id=3,
                robot_name=contract.ROBOT_NAMES[3],
            )


if __name__ == "__main__":
    unittest.main(verbosity=2)
