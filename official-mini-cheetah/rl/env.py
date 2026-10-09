#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""官方 Mini Cheetah 平地移动跳跃 Gymnasium 环境与 TCP 锁步桥。"""

from __future__ import annotations

import json
import math
import os
import re
import socket
import subprocess
import time
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from . import contract
from .reward import RewardInputs, compute_reward


TCP_RESPONSE_TIMEOUT_SECONDS = 180.0


def _tcp_ports(ports: Sequence[int], *, listening_only: bool) -> list[int]:
    """读取目标 TCP 端口的监听或连接状态。"""
    command = ["ss", "-H", "-ltnp" if listening_only else "-tanp"]
    completed = subprocess.run(
        command,
        text=True,
        capture_output=True,
        check=False,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"端口状态检查失败：ss 退出码={completed.returncode} "
            f"{completed.stderr.strip()}"
        )
    expected = {int(port) for port in ports}
    conflicts: set[int] = set()
    for line in completed.stdout.splitlines():
        fields = line.split()
        if len(fields) < 5:
            continue
        if listening_only and fields[0].upper() != "LISTEN":
            continue
        for endpoint in fields[3:5]:
            match = re.search(r":(\d+)$", endpoint)
            if match is None:
                continue
            port = int(match.group(1))
            if port in expected:
                conflicts.add(port)
    return sorted(conflicts)


def active_tcp_ports(ports: Sequence[int]) -> list[int]:
    """返回存在监听或连接的指定端口。"""
    return _tcp_ports(ports, listening_only=False)


def listening_tcp_ports(ports: Sequence[int]) -> list[int]:
    """返回正在监听的指定端口。"""
    return _tcp_ports(ports, listening_only=True)


def require_ports_free(ports: Sequence[int], context: str) -> None:
    """LISTEN 端口占用时立即失败；TCP 残留状态不阻断 bind。"""
    normalized = tuple(dict.fromkeys(int(port) for port in ports))
    conflicts = listening_tcp_ports(normalized)
    if conflicts:
        raise RuntimeError(
            f"{context}失败：端口仍被占用："
            f"{','.join(str(port) for port in conflicts)}"
        )


class MiniCheetahFlatJumpEnv(gym.Env):
    """57 维观测、12 维动作、50 Hz 的唯一 RL 环境。"""

    metadata = {"render_modes": ["human"], "render_fps": contract.CONTROL_RATE_HZ}

    def __init__(
        self,
        phase: str = "P0",
        *,
        bridge_port: int = contract.BRIDGE_PORT,
        randomization_mode: str = "full",
        render_mode: Optional[str] = None,
        start_bridge: bool = True,
        max_episode_steps: int = contract.MAX_EPISODE_STEPS,
        world: Path | str | None = None,
        worker_id: int = 0,
        robot_name: str | None = None,
        birth_position: Sequence[float] | None = None,
        supervisor_port: int = contract.WEBOTS_SUPERVISOR_PORT,
        finetune_mode: bool = False,
    ) -> None:
        super().__init__()
        self.phase = contract.normalize_phase(phase)
        if randomization_mode not in contract.RANDOMIZATION_MODES:
            raise ValueError(
                f"randomization_mode 必须是 {list(contract.RANDOMIZATION_MODES)}"
            )
        self.randomization_mode = randomization_mode
        self.render_mode = render_mode
        self.bridge_port = int(bridge_port)
        self.world_path = (Path(world) if world else contract.WORLD_PATH).resolve()
        self.worker_id = int(worker_id)
        self.supervisor_port = int(supervisor_port)
        self.finetune_mode = bool(finetune_mode)
        if not 1 <= self.supervisor_port <= 65535:
            raise ValueError("supervisor_port 必须位于 1～65535")
        if not 0 <= self.worker_id < contract.PARALLEL_WORKERS:
            raise ValueError(f"worker_id 必须位于 0～{contract.PARALLEL_WORKERS - 1}")
        is_eval_world = self.world_path == contract.EVAL_WORLD_PATH.resolve()
        expected_robot_name = (
            "mini_cheetah" if is_eval_world else contract.ROBOT_NAMES[self.worker_id]
        )
        self.robot_name = robot_name or expected_robot_name
        if self.robot_name != expected_robot_name:
            raise ValueError(
                f"robot_name 必须是 {expected_robot_name}"
            )
        selected_birth = (
            (
                contract.EVAL_BIRTH_POSITION
                if is_eval_world
                else contract.BIRTH_POSITIONS[self.worker_id]
            )
            if birth_position is None
            else tuple(float(value) for value in birth_position)
        )
        if len(selected_birth) != 3 or not all(
            math.isfinite(value) for value in selected_birth
        ):
            raise ValueError("birth_position 必须是 3 个有限数")
        self.birth_position = selected_birth
        self.max_episode_steps = int(max_episode_steps)
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(contract.OBS_DIM,),
            dtype=np.float32,
        )
        self.action_space = spaces.Box(
            low=contract.ACTION_LOW,
            high=contract.ACTION_HIGH,
            shape=(contract.ACTION_DIM,),
            dtype=np.float32,
        )

        self._server: Optional[socket.socket] = None
        self._conn: Optional[socket.socket] = None
        self._buffer = b""
        self._webots: Optional[subprocess.Popen[bytes]] = None
        self._webots_log_offset = 0
        self._controllers: list[subprocess.Popen[bytes]] = []
        self._owns_simulator = start_bridge
        self._pending_reset: Dict[str, Any] | None = None
        self._pending_step: Dict[str, Any] | None = None
        self._reset_sent = False
        self._step_sent = False
        self._exit_sent = False
        self._episode_steps = 0
        self._phase_steps = 0
        self._episode_count = 0
        self._command_step = contract.COMMAND_RESAMPLE_STEPS
        self._command = np.zeros(3, dtype=np.float32)
        self._command_fixed = False
        self._p2_command_slot = 0
        self._previous_action = np.zeros(contract.ACTION_DIM, dtype=np.float32)
        self._last_policy_action = np.zeros(contract.ACTION_DIM, dtype=np.float32)
        self._delayed_actions: list[np.ndarray] = []
        self._pending_input_action: np.ndarray | None = None
        self._pending_input_saturation_rate = 0.0
        self._episode_lag_absolute_sum = 0.0
        self._episode_lag_squared_sum = 0.0
        self._episode_lag_max = 0.0
        self._episode_saturation_sum = 0.0
        self._episode_telemetry_steps = 0
        self._episode_distance_world = 0.0
        self._unsupported_steps = 0
        self._termination_reason = ""
        self._jump_request_step = int(
            np.random.default_rng().integers(
                contract.JUMP_REQUEST_MIN_STEP,
                contract.JUMP_REQUEST_MAX_STEP + 1,
            )
        )
        self._next_jump_step = self._jump_request_step
        self._jump_latched = False
        self._last_state: Dict[str, Any] = {}
        self._randomization = {
            "friction": 1.2,
            "mass_scale": 1.0,
            "delay_steps": 0,
            "observation_noise": 0.0,
            "motor_strength": 1.0,
            "external_impulse": [0.0, 0.0, 0.0],
        }
        self._closed = False
        if start_bridge:
            self._start_bridge()

    # ------------------------------------------------------------------
    # TCP 与 Webots 生命周期
    # ------------------------------------------------------------------
    def open_bridge(self) -> None:
        """仅为当前 Robot controller 建立独立 TCP 监听端口。"""
        if not self.world_path.is_file():
            raise FileNotFoundError(f"world 不存在：{self.world_path}")
        if "official-mini-cheetah" not in self.world_path.parts:
            raise ValueError("world 必须位于 official-mini-cheetah 目录")
        if self._server is not None:
            raise RuntimeError("TCP bridge 已监听")
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind(("127.0.0.1", self.bridge_port))
        self._server.listen(1)
        self._server.settimeout(90.0)

    def launch_webots(self) -> None:
        """启动单个 Webots 实例；调用前必须先为每个 Robot 打开 TCP 端口。"""
        if self._server is None:
            raise RuntimeError("必须先监听 TCP 端口")
        if not self.world_path.is_file():
            raise FileNotFoundError(f"world 不存在：{self.world_path}")
        if "official-mini-cheetah" not in self.world_path.parts:
            raise ValueError("world 必须位于 official-mini-cheetah 目录")
        if self._webots is not None:
            raise RuntimeError("Webots 已启动")
        environment = os.environ.copy()
        environment["RL_BRIDGE_PORT"] = str(self.bridge_port)
        environment["RL_BRIDGE_HOST"] = "127.0.0.1"
        environment["RL_CONTRACT_VERSION"] = contract.CONTRACT_VERSION
        environment["RL_AGENT_MODE"] = "train"
        environment["RL_WORKER_ID"] = str(self.worker_id)
        environment["RL_ROBOT_NAME"] = self.robot_name
        environment["WEBOTS_HOME"] = environment.get(
            "WEBOTS_HOME", "/usr/local/webots"
        )
        command = [
            "/usr/local/webots/webots",
            f"--port={self.supervisor_port}",
            "--stdout",
            "--stderr",
        ]
        if self.render_mode == "human":
            command.append("--mode=fast")
        else:
            command.extend(["--batch", "--mode=fast", "--no-rendering"])
        command.append(str(self.world_path))

        log_name = (
            "webots_shared_world.log"
            if self.world_path == contract.WORLD_PATH.resolve()
            else "webots_eval_world.log"
        )
        log_path = contract.LOG_ROOT / log_name
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self._webots_log_offset = (
            log_path.stat().st_size if log_path.exists() else 0
        )
        handle = log_path.open("a", encoding="utf-8")
        try:
            self._webots = subprocess.Popen(
                command,
                cwd=str(contract.PROJECT_ROOT),
                env=environment,
                stdout=handle,
                stderr=subprocess.STDOUT,
            )
        finally:
            handle.close()

    def launch_controller(self) -> None:
        """以外部控制器方式启动当前 Robot，并记录独立健康日志。"""
        if not self.world_path.is_file():
            raise FileNotFoundError(f"world 不存在：{self.world_path}")
        controller_file = (
            contract.PROJECT_ROOT / "controllers" / "rl_agent" / "rl_agent.py"
        )
        environment = os.environ.copy()
        environment["RL_BRIDGE_PORT"] = str(self.bridge_port)
        environment["RL_BRIDGE_HOST"] = "127.0.0.1"
        environment["RL_CONTRACT_VERSION"] = contract.CONTRACT_VERSION
        environment["RL_AGENT_MODE"] = "train"
        environment["RL_WORKER_ID"] = str(self.worker_id)
        environment["RL_ROBOT_NAME"] = self.robot_name
        environment["WEBOTS_HOME"] = environment.get(
            "WEBOTS_HOME", "/usr/local/webots"
        )
        command = [
            "/usr/local/webots/webots-controller",
            f"--port={self.supervisor_port}",
            f"--robot-name={self.robot_name}",
            str(controller_file),
        ]
        contract.LOG_ROOT.mkdir(parents=True, exist_ok=True)
        log_path = contract.LOG_ROOT / f"webots_{self.worker_id}.log"
        handle = log_path.open("a", encoding="utf-8")
        try:
            process = subprocess.Popen(
                command,
                cwd=str(contract.PROJECT_ROOT),
                env=environment,
                stdout=handle,
                stderr=subprocess.STDOUT,
            )
        finally:
            handle.close()
        self._controllers.append(process)

    def verify_webots_supervisor_port(self, *, timeout: float = 10.0) -> None:
        """确认 Webots 未静默回退端口且指定 Supervisor 端口已监听。"""
        if timeout <= 0.0:
            raise ValueError("端口验证超时必须为正")
        if self._webots is None:
            raise RuntimeError("Webots 未启动，无法验证 Supervisor 端口")
        log_name = (
            "webots_shared_world.log"
            if self.world_path == contract.WORLD_PATH.resolve()
            else "webots_eval_world.log"
        )
        log_path = contract.LOG_ROOT / log_name
        deadline = time.monotonic() + timeout
        while True:
            if log_path.is_file():
                size = log_path.stat().st_size
                if size >= self._webots_log_offset:
                    with log_path.open("r", encoding="utf-8") as handle:
                        handle.seek(self._webots_log_offset)
                        text = handle.read()
                    fallback = re.search(r"Using port\s+(\d+)\s+instead", text)
                    if fallback is not None:
                        actual_port = int(fallback.group(1))
                        if actual_port != self.supervisor_port:
                            raise RuntimeError(
                                "webots_supervisor_port_mismatch: "
                                f"requested={self.supervisor_port} "
                                f"actual={actual_port}"
                            )
            if self.supervisor_port in listening_tcp_ports(
                (self.supervisor_port,)
            ):
                return
            if self._webots.poll() is not None:
                raise RuntimeError(
                    "webots_supervisor_port_missing: "
                    f"Webots 已退出，端口={self.supervisor_port}"
                )
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "webots_supervisor_port_timeout: "
                    f"端口未监听={self.supervisor_port}"
                )
            time.sleep(0.1)

    def accept_bridge(self) -> None:
        """接收本 worker 对应 Robot controller 的握手。"""
        if self._server is None:
            raise RuntimeError("TCP bridge 未监听")
        if self._webots is None and self._owns_simulator:
            raise RuntimeError("Webots 未启动")
        try:
            connection, _address = self._server.accept()
        except socket.timeout as exc:
            self.close()
            raise TimeoutError(
                "webots_connection_failed: 等待 RL controller 连接超时"
            ) from exc
        self._conn = connection
        # controller 的锁步 socket 同样使用 180 s；reset 的两阶段 RSI
        # 受界为 1500 个 4 ms 物理步，避免环境先于 controller 超时。
        self._conn.settimeout(TCP_RESPONSE_TIMEOUT_SECONDS)
        hello = self._recv()
        if hello.get("type") != "hello":
            self.close()
            raise RuntimeError(f"webots_connection_failed: TCP 握手失败：{hello}")
        if hello.get("contract") != contract.CONTRACT_VERSION:
            self.close()
            raise RuntimeError(f"webots_connection_failed: 契约不匹配：{hello}")
        if int(hello.get("worker_id", -1)) != self.worker_id:
            self.close()
            raise RuntimeError(
                f"webots_connection_failed: worker ID 不匹配：{hello}"
            )
        if hello.get("robot_name") != self.robot_name:
            self.close()
            raise RuntimeError(
                f"webots_connection_failed: robot_name 不匹配：{hello}"
            )
        if int(hello.get("timestep", -1)) != contract.WEBOTS_TIMESTEP_MS:
            self.close()
            raise RuntimeError(
                f"webots_connection_failed: timestep 不匹配：{hello}"
            )

    def _start_bridge(self) -> None:
        """独立环境兼容入口：监听、启动 world/controller 并接收握手。"""
        self.open_bridge()
        self.launch_webots()
        self.launch_controller()
        self.accept_bridge()

    def _send(self, payload: Mapping[str, Any]) -> None:
        if self._conn is None:
            raise RuntimeError("TCP 未连接")
        data = json.dumps(payload, ensure_ascii=True, separators=(",", ":")).encode()
        self._conn.sendall(data + b"\n")

    def request_exit(self) -> None:
        """通知 controller 退出；共享 VecEnv 会先向四路全部发送。"""
        if self._conn is None or self._exit_sent:
            return
        self._send({"type": "exit"})
        self._exit_sent = True

    def _recv(self) -> Dict[str, Any]:
        if self._conn is None:
            raise RuntimeError("TCP 未连接")
        while b"\n" not in self._buffer:
            chunk = self._conn.recv(65536)
            if not chunk:
                raise ConnectionError("controller 断开")
            self._buffer += chunk
        line, self._buffer = self._buffer.split(b"\n", 1)
        value = json.loads(line.decode("utf-8"))
        if not isinstance(value, dict):
            raise ValueError("TCP 消息必须是对象")
        return value

    def close(self) -> None:
        """幂等关闭 TCP、Webots 和监听端口。"""
        if self._closed:
            return
        self._closed = True
        if self._conn is not None:
            self.request_exit()
            try:
                self._conn.close()
            except Exception:
                pass
            self._conn = None
        if self._server is not None:
            try:
                self._server.close()
            except Exception:
                pass
            self._server = None
        for process in self._controllers:
            try:
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=5)
            except Exception:
                try:
                    process.kill()
                except Exception:
                    pass
        self._controllers.clear()
        if self._webots is not None:
            try:
                self._webots.terminate()
                self._webots.wait(timeout=10)
            except Exception:
                try:
                    self._webots.kill()
                except Exception:
                    pass
            self._webots = None

    # ------------------------------------------------------------------
    # 命令、随机化和观测
    # ------------------------------------------------------------------
    def sample_command(self) -> np.ndarray:
        """站立阶段固定零命令，移动阶段按阶段边界采样结构化命令。"""
        if self.phase not in contract.COMMAND_PHASES:
            return np.zeros(3, dtype=np.float32)
        if self.phase == "P2":
            return self._sample_p2_stratified_command()
        zero_probability = (
            contract.ZERO_COMMAND_PROBABILITY
        )
        if self.np_random.random() < zero_probability:
            return np.zeros(3, dtype=np.float32)
        limits = contract.phase_command_limits(self.phase)
        low = [limits[name][0] for name in contract.COMMAND_FIELDS]
        high = [limits[name][1] for name in contract.COMMAND_FIELDS]
        sampled = self.np_random.uniform(low, high)
        return np.asarray(
            contract.clip_phase_command(self.phase, sampled),
            dtype=np.float32,
        )

    def _sample_p2_stratified_command(self) -> np.ndarray:
        """P2 按固定比例轮换覆盖式命令，保证短窗口有足量 Gate forward。"""
        cycle = contract.P2_COMMAND_STRATUM_CYCLE
        stratum = cycle[self._p2_command_slot % len(cycle)]
        self._p2_command_slot = (
            self._p2_command_slot + 1
        ) % len(cycle)
        if stratum == "zero":
            return np.zeros(3, dtype=np.float32)
        if stratum == "gate_forward":
            vx = float(
                self.np_random.uniform(
                    contract.P2_GATE_FORWARD_COMMAND_MIN,
                    contract.P2_GATE_FORWARD_COMMAND_MAX,
                )
            )
            return np.asarray([vx, 0.0, 0.0], dtype=np.float32)
        limits = contract.phase_command_limits("P2")
        if stratum.startswith("lateral_"):
            high = limits["vy"][1]
            sign = 1.0 if stratum.endswith("positive") else -1.0
            return np.asarray(
                [0.0, float(self.np_random.uniform(0.0, high)) * sign, 0.0],
                dtype=np.float32,
            )
        if stratum.startswith("yaw_"):
            high = limits["wz"][1]
            sign = 1.0 if stratum.endswith("positive") else -1.0
            return np.asarray(
                [0.0, 0.0, float(self.np_random.uniform(0.0, high)) * sign],
                dtype=np.float32,
            )
        raise ValueError(f"未知 P2 命令分层：{stratum}")

    def _sample_randomization(self) -> Dict[str, float | int]:
        mode = self.randomization_mode
        if mode == "curriculum":
            mode = (
                "fixed"
                if self._episode_count <= contract.CURRICULUM_FIXED_EPISODES
                else "dr7"
            )
        if mode == "full":
            mode = "dr7"
        level = (
            0
            if mode == "fixed"
            else int(mode[2:])
            if mode in contract.RANDOMIZATION_LEVELS
            else 0
        )
        friction = 1.2
        mass = 1.0
        delay = 0
        observation_noise = 0.0
        motor_strength = 1.0
        external_impulse = [0.0, 0.0, 0.0]
        if level >= 4:
            friction = float(self.np_random.uniform(*contract.FRICTION_RANGE))
        if level >= 5:
            mass = float(self.np_random.uniform(*contract.MASS_SCALE_RANGE))
        if level >= 3:
            delay = int(
                self.np_random.integers(
                    contract.DELAY_STEPS_RANGE[0],
                    contract.DELAY_STEPS_RANGE[1] + 1,
                )
            )
        if level >= 2:
            observation_noise = 0.005
        if level >= 6:
            motor_strength = float(self.np_random.uniform(0.95, 1.05))
        if level >= 7:
            external_impulse = [
                float(value)
                for value in self.np_random.uniform(-1.5, 1.5, 3)
            ]
        return {
            "friction": round(friction, 6),
            "mass_scale": round(mass, 6),
            "delay_steps": delay,
            "observation_noise": observation_noise,
            "motor_strength": round(motor_strength, 6),
            "external_impulse": external_impulse,
            "level": level,
        }

    @staticmethod
    def _finite(values: Sequence[Any], length: int) -> np.ndarray:
        result = np.asarray(values, dtype=np.float32)
        if result.shape != (length,) or not np.all(np.isfinite(result)):
            raise ValueError(f"状态必须是 {length} 个有限数")
        return result

    def _validate_state(self, state: Mapping[str, Any]) -> Dict[str, Any]:
        required = (
            "q", "dq", "rpy", "v_body", "omega_body", "contacts",
            "height", "jump_phase", "jump_success", "jump_landing",
            "done",
        )
        missing = [key for key in required if key not in state]
        if missing:
            raise ValueError(f"状态缺少字段：{missing}")
        result: Dict[str, Any] = dict(state)
        result["q"] = self._finite(state["q"], 12)
        result["dq"] = self._finite(state["dq"], 12)
        result["rpy"] = self._finite(state["rpy"], 3)
        result["v_body"] = self._finite(state["v_body"], 3)
        result["omega_body"] = self._finite(state["omega_body"], 3)
        contacts_source = state.get("foot_contacts", state["contacts"])
        result["contacts"] = self._finite(contacts_source, 4)
        result["joint_torques"] = self._finite(
            state.get("joint_torques", [0.0] * contract.ACTION_DIM),
            contract.ACTION_DIM,
        )
        result["torque_source"] = str(state.get("torque_source", "unknown"))
        result["foot_velocities"] = self._finite(
            state.get("foot_velocities", [0.0] * 12),
            12,
        )
        result["non_foot_contact"] = bool(state.get("non_foot_contact", False))
        result["body_contact"] = bool(state.get("body_contact", False))
        result["foot_contact_source"] = str(
            state.get("foot_contact_source", "node_id")
        )
        if result["foot_contact_source"] != "node_id":
            result["contacts"] = np.zeros(4, dtype=np.float32)
            result["foot_contacts"] = np.zeros(4, dtype=np.float32)
        for key in ("height", "jump_phase"):
            value = float(state[key])
            if not math.isfinite(value):
                raise ValueError(f"{key} 非有限")
            result[key] = value
        result["jump_success"] = bool(state["jump_success"])
        result["jump_landing"] = bool(state["jump_landing"])
        result["done"] = bool(state["done"])
        return result

    def _observation(self) -> np.ndarray:
        state = self._last_state
        values = np.concatenate(
            [
                state["q"],
                state["dq"],
                state["rpy"],
                state["v_body"],
                self._previous_action,
                state["omega_body"],
                self._command,
                np.asarray(
                    [1.0 if self._jump_latched else 0.0, state["jump_phase"]],
                    dtype=np.float32,
                ),
                np.asarray([state["height"]], dtype=np.float32),
                state["contacts"],
                np.asarray([0.0, 1.0], dtype=np.float32),
            ]
        ).astype(np.float32)
        observation_noise = float(
            self._randomization.get("observation_noise", 0.0)
        )
        if observation_noise > 0.0:
            values = values + self.np_random.normal(
                0.0, observation_noise, contract.OBS_DIM
            ).astype(np.float32)
        if values.shape != (contract.OBS_DIM,) or not np.all(np.isfinite(values)):
            raise ValueError("观测维度或有限性错误")
        return values

    def _fallen(self, state: Mapping[str, Any]) -> bool:
        """按 R3 终止规则判断失败，并保留首个原因供验收记录。"""
        roll, pitch, _yaw = (float(value) for value in state["rpy"])
        x, y, _z = self._position_from_state(state)
        height = float(state["height"])
        if bool(state.get("body_contact", False)):
            self._termination_reason = "body_contact"
            return True
        if any(
            value < low - 1e-3 or value > high + 1e-3
            for value, (low, high) in zip(
                state["q"], contract.TARGET_POSITION_LIMITS
            )
        ):
            self._termination_reason = "joint_limit"
            return True
        if abs(roll) > contract.ROLL_PITCH_LIMIT or abs(pitch) > contract.ROLL_PITCH_LIMIT:
            self._termination_reason = "tilt"
            return True
        if height < contract.MIN_SUPPORTED_HEIGHT or height > contract.MAX_BASE_HEIGHT:
            self._termination_reason = "height"
            return True
        if (
            abs(x - self.birth_position[0]) > contract.LOCAL_ACTIVITY_RADIUS
            or abs(y - self.birth_position[1]) > contract.LOCAL_ACTIVITY_RADIUS
        ):
            self._termination_reason = "workspace"
            return True
        if sum(float(value) for value in state["contacts"]) <= 0.0:
            self._unsupported_steps += 1
        else:
            self._unsupported_steps = 0
        if self._unsupported_steps >= contract.UNSUPPORTED_STEPS_LIMIT:
            self._termination_reason = "unsupported"
            return True
        return False

    @staticmethod
    def _position_from_state(state: Mapping[str, Any]) -> tuple[float, float, float]:
        values = state.get("position", (0.0, 0.0, float(state["height"])))
        return tuple(float(value) for value in values)  # type: ignore[return-value]

    # ------------------------------------------------------------------
    # Gymnasium API
    # ------------------------------------------------------------------
    def prepare_reset(
        self,
        *,
        seed: int | None = None,
        options: Dict[str, Any] | None = None,
        shared_friction: float | None = None,
    ) -> Dict[str, Any]:
        """准备本 worker 的 reset 消息，但不读取任何网络响应。"""
        if self._closed:
            raise RuntimeError("环境已关闭")
        if self._pending_reset is not None:
            raise RuntimeError("已有待完成的 reset")
        super().reset(seed=seed)
        options = options or {}
        self._episode_count += 1
        self._episode_steps = 0
        self._previous_action = np.zeros(contract.ACTION_DIM, dtype=np.float32)
        self._last_policy_action = np.zeros(contract.ACTION_DIM, dtype=np.float32)
        self._delayed_actions = []
        self._pending_input_action = None
        self._pending_input_saturation_rate = 0.0
        self._episode_lag_absolute_sum = 0.0
        self._episode_lag_squared_sum = 0.0
        self._episode_lag_max = 0.0
        self._episode_saturation_sum = 0.0
        self._episode_telemetry_steps = 0
        self._episode_distance_world = 0.0
        self._unsupported_steps = 0
        self._termination_reason = ""
        self._randomization = self._sample_randomization()
        randomized = int(self._randomization.get("level", 0)) >= 4
        if shared_friction is not None and randomized:
            self._randomization["friction"] = round(float(shared_friction), 6)
        if "command" in options:
            self._command = np.asarray(
                contract.clip_phase_command(self.phase, options["command"]),
                dtype=np.float32,
            )
            self._command_fixed = bool(options.get("command_fixed", True))
        else:
            self._command_fixed = False
            self._command_step = contract.COMMAND_RESAMPLE_STEPS
            self._command = self.sample_command()
        delay = int(self._randomization["delay_steps"])
        self._delayed_actions = [
            np.zeros(contract.ACTION_DIM, dtype=np.float32)
            for _ in range(delay)
        ]
        reset_style = "rsi" if self.phase in {"P0", "P1"} else "settle"
        perturbation = np.zeros(contract.ACTION_DIM, dtype=np.float32)
        randomization_level = int(self._randomization.get("level", 0))
        if (
            self.phase == "P1"
            and self._phase_steps >= contract.P1_PERTURBATION_LOCAL_STEPS
        ) or randomization_level >= 1:
            perturbation = self.np_random.uniform(
                -0.03, 0.03, contract.ACTION_DIM
            ).astype(np.float32)
        self._jump_latched = False
        self._next_jump_step = int(
            self.np_random.integers(
                contract.JUMP_REQUEST_MIN_STEP,
                contract.JUMP_REQUEST_MAX_STEP + 1,
            )
        )
        self._pending_reset = {
            "type": "reset",
            "command": [float(value) for value in self._command],
            "randomization": dict(self._randomization),
            "birth_position": [float(value) for value in self.birth_position],
            "reset_style": reset_style,
            "perturbation": [float(value) for value in perturbation],
        }
        self._reset_sent = False
        return dict(self._pending_reset)

    def send_prepared_reset(self) -> None:
        """发送已准备的 reset；共享 VecEnv 会先完成四路发送。"""
        if self._pending_reset is None or self._reset_sent:
            raise RuntimeError("没有待发送的 reset")
        self._send(self._pending_reset)
        self._reset_sent = True

    def finish_reset(self) -> tuple[np.ndarray, Dict[str, Any]]:
        """接收并校验 reset 后状态。"""
        if self._pending_reset is None or not self._reset_sent:
            raise RuntimeError("reset 尚未发送")
        started = time.monotonic()
        try:
            raw_state = self._recv()
        except socket.timeout as exc:
            raise TimeoutError(
                "reset response timeout: "
                f"worker_id={self.worker_id}, "
                f"timeout={TCP_RESPONSE_TIMEOUT_SECONDS:.1f}s, "
                f"elapsed={time.monotonic() - started:.3f}s"
            ) from exc
        except ConnectionError as exc:
            raise ConnectionError(
                "reset response disconnected: "
                f"worker_id={self.worker_id}, "
                f"elapsed={time.monotonic() - started:.3f}s"
            ) from exc
        self._last_state = self._validate_state(raw_state)
        self._pending_reset = None
        self._reset_sent = False
        observation = self._observation()
        info = {
            "phase": self.phase,
            "worker_id": self.worker_id,
            "robot_name": self.robot_name,
            "randomization": dict(self._randomization),
            "randomization_mode": self.randomization_mode,
            "command": self._command.copy(),
        }
        return observation, info

    def reset(
        self,
        *,
        seed: int | None = None,
        options: Dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, Dict[str, Any]]:
        """单环境兼容入口：准备、发送并接收一次 reset。"""
        self.prepare_reset(seed=seed, options=options)
        self.send_prepared_reset()
        return self.finish_reset()

    def prepare_step(
        self, action: Sequence[float]
    ) -> Dict[str, Any]:
        """准备一个 act 消息，但不发送或接收。"""
        if self._closed or self._conn is None:
            raise RuntimeError("环境未连接")
        if self._pending_step is not None:
            raise RuntimeError("已有待完成的 step")
        previous_action = self._previous_action.copy()
        input_action = np.asarray(action, dtype=np.float32)
        safe = np.asarray(
            contract.sanitize_action(action, self._last_policy_action),
            dtype=np.float32,
        )
        self._pending_input_action = input_action.copy()
        self._pending_input_saturation_rate = contract.action_saturation_rate(
            input_action
        )
        self._delayed_actions.append(safe)
        delayed = self._delayed_actions.pop(0)

        if not self._command_fixed:
            self._episode_steps += 1
            self._phase_steps += 1
            if self._episode_steps >= self._command_step:
                self._command = self.sample_command()
                self._command_step += contract.COMMAND_RESAMPLE_STEPS
        else:
            self._episode_steps += 1
            self._phase_steps += 1

        jump_request = False
        if self.phase in contract.JUMP_PHASES and not self._jump_latched:
            if self._episode_steps >= self._next_jump_step:
                self._jump_latched = True
                jump_request = True
                self._next_jump_step = self._episode_steps + int(
                    self.np_random.integers(
                        contract.JUMP_INTERVAL_MIN,
                        contract.JUMP_INTERVAL_MAX + 1,
                    )
                )
        if jump_request:
            self._jump_latched = True

        self._pending_step = {
            "type": "act",
            "action": [float(value) for value in delayed],
            "command": [float(value) for value in self._command],
            "jump_request": jump_request,
        }
        self._pending_previous_action = previous_action
        self._pending_safe_action = safe
        self._pending_delayed_action = delayed.copy()
        self._step_sent = False
        return dict(self._pending_step)

    def send_prepared_step(self) -> None:
        """发送已准备的 act；共享 VecEnv 会先完成四路发送。"""
        if self._pending_step is None or self._step_sent:
            raise RuntimeError("没有待发送的 step")
        self._send(self._pending_step)
        self._step_sent = True

    def finish_step(self) -> tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        """接收状态并计算单环境 transition。"""
        if self._pending_step is None or not self._step_sent:
            raise RuntimeError("step 尚未发送")
        previous_action = self._pending_previous_action
        delayed = self._pending_delayed_action
        safe = self._pending_safe_action
        state = self._validate_state(self._recv())
        previous_state = self._last_state
        self._pending_step = None
        self._step_sent = False
        self._last_state = state
        self._previous_action = delayed
        self._last_policy_action = safe
        if state["jump_landing"] and self._jump_latched:
            self._jump_latched = False

        fallen = self._fallen(state)
        terminated = bool(state["done"] or fallen)
        episode_limit = (
            contract.P1_EPISODE_STEPS
            if self.phase == "P1"
            else self.max_episode_steps
        )
        truncated = self._episode_steps >= episode_limit
        previous_position = previous_state.get(
            "position",
            state["position"],
        )
        world_displacement = [
            float(current) - float(previous)
            for current, previous in zip(
                state["position"],
                previous_position,
            )
        ]
        if len(world_displacement) != 3:
            world_displacement = [0.0, 0.0, 0.0]
        # 命令是机体系，世界系位移按当前偏航角转回机体系后再做进度奖励。
        yaw = float(state["rpy"][2])
        cosine_yaw = math.cos(yaw)
        sine_yaw = math.sin(yaw)
        local_displacement = (
            cosine_yaw * world_displacement[0]
            + sine_yaw * world_displacement[1],
            -sine_yaw * world_displacement[0]
            + cosine_yaw * world_displacement[1],
            world_displacement[2],
        )
        previous_joint_velocities = previous_state.get(
            "dq",
            [0.0] * contract.ACTION_DIM,
        )
        execution_telemetry = state.get("execution_telemetry")
        if isinstance(execution_telemetry, Mapping):
            desired_targets = execution_telemetry.get(
                "desired_targets",
                contract.action_to_target(delayed),
            )
            executed_targets = execution_telemetry.get(
                "executed_targets",
                desired_targets,
            )
        else:
            desired_targets = contract.action_to_target(delayed)
            executed_targets = desired_targets
        target_telemetry = contract.target_telemetry(
            desired_targets,
            executed_targets,
        )
        input_saturation_rate = (
            self._pending_input_saturation_rate
            if self._pending_input_action is not None
            else contract.action_saturation_rate(delayed)
        )
        lag_mean = float(target_telemetry["mean"])
        lag_rms = float(target_telemetry["rms"])
        lag_max = float(target_telemetry["max"])
        self._episode_lag_absolute_sum += lag_mean
        self._episode_lag_squared_sum += lag_rms * lag_rms
        self._episode_lag_max = max(self._episode_lag_max, lag_max)
        self._episode_saturation_sum += float(input_saturation_rate)
        self._episode_telemetry_steps += 1
        telemetry_steps = max(1, self._episode_telemetry_steps)
        reward, reward_parts = compute_reward(
            RewardInputs(
                phase=self.phase,
                roll=float(state["rpy"][0]),
                pitch=float(state["rpy"][1]),
                height=float(state["height"]),
                command=self._command,
                velocity=state["v_body"],
                action=delayed,
                previous_action=previous_action,
                contacts=state["contacts"],
                omega=state["omega_body"],
                joint_positions=state["q"],
                joint_velocities=state["dq"],
                previous_joint_velocities=previous_joint_velocities,
                joint_torques=state["joint_torques"],
                foot_velocities=state["foot_velocities"],
                phase_steps=self._phase_steps,
                non_foot_contact=state["non_foot_contact"],
                fallen=fallen,
                jump_success_event=bool(state["jump_success"]),
                jump_landing_event=bool(state["jump_landing"]),
                displacement=local_displacement,
                finetune_mode=self.finetune_mode,
            )
        )
        observation = self._observation()
        info = {
            "phase": self.phase,
            "worker_id": self.worker_id,
            "robot_name": self.robot_name,
            "fallen": fallen,
            "termination_reason": self._termination_reason if fallen else "",
            "reward_parts": reward_parts,
            "command": self._command.copy(),
            "randomization": dict(self._randomization),
            "jump_success_event": bool(state["jump_success"]),
            "jump_landing_event": bool(state["jump_landing"]),
            "body_contact": state["body_contact"],
            "non_foot_contact": state["non_foot_contact"],
            "foot_contacts": state["contacts"].copy(),
            "foot_contact_source": state["foot_contact_source"],
            "torque_source": state["torque_source"],
            "applied_action": delayed.copy(),
            "unsupported_steps": self._unsupported_steps,
            "position": np.asarray(state["position"], dtype=np.float32),
            "v_body": state["v_body"].copy(),
            "displacement_world": np.asarray(
                world_displacement,
                dtype=np.float32,
            ),
            "displacement_body": np.asarray(
                local_displacement,
                dtype=np.float32,
            ),
            "displacement_step_norm": float(
                math.sqrt(
                    sum(float(value) * float(value) for value in world_displacement)
                )
            ),
            "desired_targets": np.asarray(
                target_telemetry["desired"],
                dtype=np.float32,
            ),
            "executed_targets": np.asarray(
                target_telemetry["executed"],
                dtype=np.float32,
            ),
            "desired_executed_target_delta": np.asarray(
                target_telemetry["delta"],
                dtype=np.float32,
            ),
            "target_rate_limit": float(target_telemetry["rate_limit"]),
            "target_lag_mean": lag_mean,
            "target_lag_rms": lag_rms,
            "target_lag_max": lag_max,
            "episode_target_lag_mean": (
                self._episode_lag_absolute_sum / telemetry_steps
            ),
            "episode_target_lag_rms": math.sqrt(
                self._episode_lag_squared_sum / telemetry_steps
            ),
            "episode_target_lag_max": self._episode_lag_max,
            "action_saturation_rate": float(input_saturation_rate),
            "episode_action_saturation_rate": (
                self._episode_saturation_sum / telemetry_steps
            ),
            "execution/target_rate_limit": float(
                target_telemetry["rate_limit"]
            ),
            "execution/target_lag_mean": lag_mean,
            "execution/target_lag_rms": lag_rms,
            "execution/target_lag_max": lag_max,
            "execution/action_saturation_rate": float(
                input_saturation_rate
            ),
            "state/position_x": float(state["position"][0]),
            "state/position_y": float(state["position"][1]),
            "state/position_z": float(state["position"][2]),
            "state/v_body_x": float(state["v_body"][0]),
            "state/v_body_y": float(state["v_body"][1]),
            "state/v_body_z": float(state["v_body"][2]),
            "state/displacement_step_norm": float(
                math.sqrt(
                    sum(float(value) * float(value) for value in world_displacement)
                )
            ),
        }
        self._episode_distance_world += float(
            math.sqrt(
                sum(float(value) * float(value) for value in world_displacement)
            )
        )
        self._episode_telemetry_steps = telemetry_steps
        info["episode_distance_world"] = self._episode_distance_world
        info["state/distance_world"] = self._episode_distance_world
        self._pending_input_action = None
        self._pending_input_saturation_rate = 0.0
        return observation, reward, terminated, truncated, info

    def step(
        self, action: Sequence[float]
    ) -> tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        """单环境兼容入口：准备、发送并接收一个动作周期。"""
        self.prepare_step(action)
        self.send_prepared_step()
        return self.finish_step()
