#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单 Webots world 内四台 Mini Cheetah 的同步 SB3 VecEnv。"""

from __future__ import annotations

import subprocess
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, Optional, Sequence

import numpy as np
from gymnasium import spaces
from stable_baselines3.common.vec_env import VecEnv

from . import contract
from .env import MiniCheetahFlatJumpEnv


class SharedWorldVecEnv(VecEnv):
    """统一管理一个 Webots、四路 TCP 和四台机器人锁步生命周期。"""

    def __init__(
        self,
        *,
        num_envs: int = contract.PARALLEL_WORKERS,
        phase: str = "P0",
        bridge_port: int = contract.BRIDGE_PORT,
        randomization_mode: str = "full",
        render_mode: Optional[str] = None,
        max_episode_steps: int = contract.MAX_EPISODE_STEPS,
        world: Path | str | None = None,
        seed: int | None = None,
        env_factory: Callable[[int], MiniCheetahFlatJumpEnv] | None = None,
        start_runtime: bool = True,
    ) -> None:
        if num_envs != contract.PARALLEL_WORKERS:
            raise ValueError(f"共享 world 必须使用 {contract.PARALLEL_WORKERS} 个环境")
        if tuple(contract.bridge_ports(num_envs)) != tuple(
            bridge_port + index for index in range(num_envs)
        ):
            raise ValueError(
                f"TCP 端口必须是 {list(contract.bridge_ports(num_envs))}"
            )
        observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(contract.OBS_DIM,),
            dtype=np.float32,
        )
        action_space = spaces.Box(
            low=contract.ACTION_LOW,
            high=contract.ACTION_HIGH,
            shape=(contract.ACTION_DIM,),
            dtype=np.float32,
        )
        super().__init__(num_envs, observation_space, action_space)
        self.phase = contract.normalize_phase(phase)
        self.bridge_port = int(bridge_port)
        self.randomization_mode = randomization_mode
        self.render_mode = render_mode
        self.max_episode_steps = int(max_episode_steps)
        self.world_path = Path(world) if world else contract.WORLD_PATH
        self.seed_value = seed
        self._closed = False
        self._async_actions = False
        self._reset_counter = 0
        self._friction_rng = np.random.default_rng(seed)
        self._webots: Optional[subprocess.Popen[bytes]] = None
        factory = env_factory or self._default_env_factory
        self.envs = [factory(worker_id) for worker_id in range(self.num_envs)]
        if start_runtime:
            try:
                self._start_runtime()
            except Exception:
                self.close()
                raise

    def _default_env_factory(self, worker_id: int) -> MiniCheetahFlatJumpEnv:
        """创建只等待连接、不自行启动仿真的单 worker 环境。"""
        return MiniCheetahFlatJumpEnv(
            phase=self.phase,
            bridge_port=self.bridge_port + worker_id,
            worker_id=worker_id,
            randomization_mode=self.randomization_mode,
            render_mode=self.render_mode,
            start_bridge=False,
            max_episode_steps=self.max_episode_steps,
            world=self.world_path,
        )

    def _start_runtime(self) -> None:
        """按 listener、Webots、controller、四路握手顺序启动并整体回滚。"""
        for env in self.envs:
            env.open_bridge()
        self.envs[0].launch_webots()
        self._webots = self.envs[0]._webots
        self.envs[0]._webots = None
        for env in self.envs:
            env.launch_controller()
        for env in self.envs:
            env.accept_bridge()

    @staticmethod
    def validate_hello(
        hello: Dict[str, Any],
        *,
        worker_id: int,
        robot_name: str,
    ) -> None:
        """校验 controller hello 的五个锁步关键字段。"""
        if hello.get("type") != "hello":
            raise RuntimeError(f"webots_connection_failed: TCP 握手失败：{hello}")
        if hello.get("contract") != contract.CONTRACT_VERSION:
            raise RuntimeError(f"webots_connection_failed: 契约不匹配：{hello}")
        if int(hello.get("worker_id", -1)) != worker_id:
            raise RuntimeError(f"webots_connection_failed: worker ID 不匹配：{hello}")
        if hello.get("robot_name") != robot_name:
            raise RuntimeError(f"webots_connection_failed: robot_name 不匹配：{hello}")
        if int(hello.get("timestep", -1)) != contract.WEBOTS_TIMESTEP_MS:
            raise RuntimeError(f"webots_connection_failed: timestep 不匹配：{hello}")

    def reset(self) -> np.ndarray:
        """四路统一 reset；摩擦全局一次采样，质量和延迟各自独立。"""
        if self._closed:
            raise RuntimeError("VecEnv 已关闭")
        self._async_actions = False
        friction = float(
            self._friction_rng.uniform(*contract.FRICTION_RANGE)
        )
        seed = (
            None
            if self.seed_value is None or self._reset_counter > 0
            else int(self.seed_value)
        )
        for worker_id, env in enumerate(self.envs):
            worker_seed = None if seed is None else seed + worker_id
            env.prepare_reset(
                seed=worker_seed,
                shared_friction=friction,
            )
        self._reset_counter += 1
        for env in self.envs:
            env.send_prepared_reset()
        observations, infos = [], []
        for env in self.envs:
            observation, info = env.finish_reset()
            observations.append(observation)
            infos.append(info)
        return np.stack(observations).astype(np.float32)

    def step_async(self, actions: np.ndarray) -> None:
        """严格校验批量动作，先准备并发送四路消息，不接收响应。"""
        if self._closed:
            raise RuntimeError("VecEnv 已关闭")
        if self._async_actions:
            raise RuntimeError("已有待完成的 step")
        values = np.asarray(actions)
        expected = (self.num_envs, contract.ACTION_DIM)
        if values.shape != expected:
            raise ValueError(f"动作形状必须是 {expected}，收到 {values.shape}")
        if not np.issubdtype(values.dtype, np.number):
            raise TypeError("动作必须是数值数组")
        if not np.all(np.isfinite(values)):
            raise ValueError("动作必须全部有限")
        for worker_id, env in enumerate(self.envs):
            env.prepare_step(values[worker_id])
        for env in self.envs:
            env.send_prepared_step()
        self._async_actions = True

    def step_wait(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[Dict[str, Any]]]:
        """先接收四路 transition，再执行统一同步 reset。"""
        if not self._async_actions:
            raise RuntimeError("step_async 尚未调用")
        results = [env.finish_step() for env in self.envs]
        self._async_actions = False
        observations = np.stack([item[0] for item in results]).astype(np.float32)
        rewards = np.asarray([item[1] for item in results], dtype=np.float32)
        terminated = np.asarray([item[2] for item in results], dtype=bool)
        truncated = np.asarray([item[3] for item in results], dtype=bool)
        infos = [dict(item[4]) for item in results]
        dones = terminated | truncated
        if not bool(np.any(dones)):
            return observations, rewards, dones, infos

        for index, done in enumerate(dones):
            if not done:
                continue
            infos[index].setdefault(
                "terminal_observation", observations[index].copy()
            )
            if terminated[index]:
                infos[index]["terminated"] = True
            elif truncated[index]:
                infos[index]["TimeLimit.truncated"] = True

        forced = ~dones
        for index in np.flatnonzero(forced):
            infos[int(index)]["truncated"] = True
            infos[int(index)]["terminated"] = False
            infos[int(index)]["TimeLimit.truncated"] = True
            infos[int(index)]["terminal_observation"] = observations[index].copy()
            truncated[index] = True
        dones = terminated | truncated
        fresh = self.reset()
        return fresh, rewards, dones, infos

    def close(self) -> None:
        """幂等关闭四路连接、controller 和唯一 Webots，无孤儿进程。"""
        if self._closed:
            return
        self._closed = True
        self._async_actions = False
        for env in self.envs:
            try:
                env.request_exit()
            except Exception:
                pass
        for env in self.envs:
            try:
                env.close()
            except Exception:
                pass
        if self._webots is not None:
            try:
                if self._webots.poll() is None:
                    self._webots.terminate()
                    self._webots.wait(timeout=10)
            except Exception:
                try:
                    self._webots.kill()
                except Exception:
                    pass
            self._webots = None

    def env_method(
        self,
        method_name: str,
        *args: Any,
        indices: Any = None,
        **kwargs: Any,
    ) -> list[Any]:
        """在指定单环境上调用方法。"""
        selected = self._indices(indices)
        return [
            getattr(self.envs[index])(method_name)(*args, **kwargs)
            for index in selected
        ]

    def get_attr(self, attr_name: str, indices: Any = None) -> list[Any]:
        """读取指定单环境属性。"""
        return [getattr(self.envs[index], attr_name) for index in self._indices(indices)]

    def set_attr(
        self,
        attr_name: str,
        value: Any,
        indices: Any = None,
    ) -> None:
        """设置指定单环境属性。"""
        for index in self._indices(indices):
            setattr(self.envs[index], attr_name, value)

    def _indices(self, indices: Any) -> list[int]:
        if indices is None:
            return list(range(self.num_envs))
        if isinstance(indices, (int, np.integer)):
            return [int(indices)]
        return [int(index) for index in indices]

    def get_images(self) -> Sequence[np.ndarray]:
        """共享 headless world 没有独立可截取图像。"""
        return []

    def env_is_wrapped(
        self,
        wrapper_class: Any,
        indices: Any = None,
    ) -> list[bool]:
        """四个单环境均不返回额外 Gym wrapper 包装状态。"""
        del wrapper_class
        return [False for _ in self._indices(indices)]
