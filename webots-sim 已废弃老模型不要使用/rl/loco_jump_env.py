#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""yobogo_loco_jump_v1 的 Webots Gymnasium 训练环境。

观测严格遵守 ``loco_jump_contract``：旧 42 维顺序与 walk_env 保持一致，随后
追加角速度 3 维、命令 3 维、跳跃请求/经过时间/机身高度/足端接触 6 维，合计
55 维。动作仍为 12 维 ``[-1, 1]``，关节目标为
``q_des = q_stand + a * [0.3, 0.5, 0.5]``（逐腿重复）。

环境通过 JSON 行 TCP 桥与 rl_agent 交换数据；单测可传
``start_bridge=False`` 并覆写 ``_send/_recv``，不会启动 Webots。
"""

from __future__ import annotations

import base64
import json
import os
import re
import socket
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError as _gym_exc:  # pragma: no cover - 缺依赖时给出中文提示
    raise ImportError(
        "【错误】缺少 gymnasium，无法使用 QuadrupedLocoJumpEnv。\n"
        "  请先安装：pip install gymnasium\n"
        f"  原始报错：{_gym_exc}"
    ) from _gym_exc

_RL_DIR = Path(__file__).resolve().parent
if str(_RL_DIR) not in sys.path:
    sys.path.insert(0, str(_RL_DIR))

try:
    from . import loco_jump_contract as contract
    from . import loco_jump_reward as reward_module
except ImportError:  # 直接执行脚本或把 rl 目录加入 sys.path
    import loco_jump_contract as contract  # type: ignore[no-redef]
    import loco_jump_reward as reward_module  # type: ignore[no-redef]

# ---------------------------------------------------------------------------
# 路径、常量与公开动作映射
# ---------------------------------------------------------------------------
WEBOTS_SIM_DIR = _RL_DIR.parent
WORLDS_DIR = WEBOTS_SIM_DIR / "worlds"
DEFAULT_WORLD = WORLDS_DIR / "parkour_dev.wbt"
DEFAULT_BRIDGE_PORT = 11451
BRIDGE_HOST = "127.0.0.1"

LEG_ORDER: Tuple[str, ...] = ("fr", "fl", "hr", "hl")
JOINT_ORDER: Tuple[str, ...] = ("abd", "hip", "kn")
MOTOR_NAMES: Tuple[str, ...] = tuple(
    f"{leg}_{joint}_motor" for leg in LEG_ORDER for joint in JOINT_ORDER
)
SENSOR_NAMES: Tuple[str, ...] = tuple(
    f"{leg}_{joint}_sensor" for leg in LEG_ORDER for joint in JOINT_ORDER
)

Q_STAND_LEG: Tuple[float, float, float] = (0.0, 0.0, 0.0)
Q_STAND: np.ndarray = np.tile(
    np.asarray(Q_STAND_LEG, dtype=np.float32), len(LEG_ORDER)
)
ACTION_SCALE_LEG: Tuple[float, float, float] = (0.3, 0.5, 0.5)
ACTION_SCALE: np.ndarray = np.tile(
    np.asarray(ACTION_SCALE_LEG, dtype=np.float32), len(LEG_ORDER)
)

MAX_ROLL: float = 0.8
MAX_PITCH: float = 0.8
MIN_BASE_HEIGHT: float = 0.12
MAX_EPISODE_STEPS: int = 1000
DEFAULT_COMMAND_RESAMPLE_STEPS: int = 250
DEFAULT_JUMP_REQUEST_STEP: int = 100
DEFAULT_JUMP_INTERVAL_STEPS: int = 250
JUMP_SUCCESS_HEIGHT_GAIN: float = 0.04
CONTACT_ON_THRESHOLD: float = 0.5

__all__ = [
    "ACTION_SCALE",
    "ACTION_SCALE_LEG",
    "LocoJumpEnv",
    "QuadrupedLocoJumpEnv",
    "Q_STAND",
    "Q_STAND_LEG",
    "action_to_q_des",
    "make_env",
]


def action_to_q_des(
    action: Sequence[float],
    q_stand: Sequence[float] = Q_STAND,
) -> np.ndarray:
    """把 12 维归一化动作映射为 12 维关节目标角。

    公式与控制器保持一致：逐腿按 ``(abd, hip, kn)`` 重复
    ``[0.3, 0.5, 0.5]`` 缩放。函数不擅自裁剪动作；环境在 ``step()``
    入口会先按契约裁剪到 ``[-1, 1]``。
    """
    action_array = np.asarray(action, dtype=np.float64).reshape(-1)
    stand_array = np.asarray(q_stand, dtype=np.float64).reshape(-1)
    if action_array.size != contract.ACTION_DIM:
        raise ValueError(
            f"动作维度错误：期望 {contract.ACTION_DIM}，收到 {action_array.size}"
        )
    if stand_array.size != contract.ACTION_DIM:
        raise ValueError(
            f"站立关节角维度错误：期望 {contract.ACTION_DIM}，收到 {stand_array.size}"
        )
    if not np.all(np.isfinite(action_array)) or not np.all(np.isfinite(stand_array)):
        raise ValueError("动作或站立关节角包含 NaN/Inf")
    scale = np.asarray(ACTION_SCALE, dtype=np.float64)
    return stand_array + action_array * scale


def _find_webots() -> str:
    """查找 Webots 可执行文件。"""
    import shutil

    candidates = []
    home = os.environ.get("WEBOTS_HOME")
    if home:
        candidates.append(os.path.join(home, "webots"))
    candidates.extend(
        ["/usr/local/webots/webots", "/usr/local/bin/webots"]
    )
    for candidate in candidates:
        if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            return candidate
    resolved = shutil.which("webots")
    if resolved:
        return resolved
    raise FileNotFoundError(
        "【错误】找不到 Webots 可执行文件。请安装 Webots 或设置 WEBOTS_HOME。"
    )


def _generated_world_path(src_world: Path) -> Path:
    """由源世界名推导运行期 RL 世界路径。"""
    return WORLDS_DIR / f".{src_world.stem}_rl.wbt"


def _patch_rl_world(src_world: Path, dst_world: Path) -> Path:
    """生成只使用 rl_agent 控制器且带 Supervisor 权限的运行期世界。"""
    text = src_world.read_text(encoding="utf-8")
    if 'controller "rl_agent"' not in text:
        if 'controller "' not in text:
            raise ValueError(f"世界文件中找不到 controller 字段：{src_world}")
        text = re.sub(
            r'controller\s+"[^"]*"',
            'controller "rl_agent"',
            text,
            count=1,
        )
    if "supervisor TRUE" not in text:
        text = text.replace(
            'controller "rl_agent"',
            'supervisor TRUE\n  controller "rl_agent"',
            1,
        )
    dst_world.write_text(text, encoding="utf-8")
    return dst_world


class QuadrupedLocoJumpEnv(gym.Env):
    """Webots 单实例运动控制 + 跳跃 Gymnasium 环境。"""

    metadata = {
        "render_modes": ["human", "rgb_array"],
        "render_fps": int(1.0 / contract.CONTROL_DT_SECONDS),
    }

    def __init__(
        self,
        render_mode: Optional[str] = None,
        world: Optional[os.PathLike | str] = None,
        bridge_port: Optional[int] = None,
        webots_timeout: float = 90.0,
        *,
        tag: str = "phase0",
        start_bridge: bool = True,
        max_episode_steps: int = MAX_EPISODE_STEPS,
        command_resample_steps: int = DEFAULT_COMMAND_RESAMPLE_STEPS,
        jump_request_step: int = DEFAULT_JUMP_REQUEST_STEP,
        jump_interval_steps: int = DEFAULT_JUMP_INTERVAL_STEPS,
    ) -> None:
        super().__init__()
        self.render_mode = render_mode
        self.world_path = Path(world) if world is not None else DEFAULT_WORLD
        self.webots_timeout = float(webots_timeout)
        self.stage = reward_module.normalize_stage(tag)
        self.tag = reward_module.stage_tag(self.stage)
        self.max_episode_steps = int(max_episode_steps)
        self.command_resample_steps = max(1, int(command_resample_steps))
        self.jump_request_step = max(0, int(jump_request_step))
        self.jump_interval_steps = max(1, int(jump_interval_steps))

        high_obs = np.full(contract.OBS_DIM, np.inf, dtype=np.float32)
        self.observation_space = spaces.Box(
            -high_obs, high_obs, dtype=np.float32
        )
        self.action_space = spaces.Box(
            low=np.full(contract.ACTION_DIM, contract.ACTION_LOW, dtype=np.float32),
            high=np.full(contract.ACTION_DIM, contract.ACTION_HIGH, dtype=np.float32),
            dtype=np.float32,
        )

        self._webots_proc: Optional[subprocess.Popen] = None
        self._server_sock: Optional[socket.socket] = None
        self._conn: Optional[socket.socket] = None
        self._bridge_port = int(
            bridge_port
            if bridge_port is not None
            else os.environ.get("RL_BRIDGE_PORT", DEFAULT_BRIDGE_PORT)
        )

        self._ep_steps = 0
        self._prev_action = np.zeros(contract.ACTION_DIM, dtype=np.float32)
        self._last_state: Dict[str, Any] = {}
        self._closed = False

        # 命令、目标偏航与 episode 记账。
        self._command = np.zeros(3, dtype=np.float32)
        self._target_yaw = 0.0
        self._baseline_height = 0.26

        # 跳跃锁存状态；清理严格使用契约 helper，避免触发起跳即清零。
        self._jump_latched = False
        self._jump_elapsed = 0.0
        self._jump_had_contact_loss = False
        self._jump_peak_height = 0.0
        self._jump_success = False
        self._jump_success_event = False
        self._terrain_success = False
        self._terrain_success_event = False

        if start_bridge:
            self._start_bridge()

    # ------------------------------------------------------------------
    # Webots / TCP 桥生命周期
    # ------------------------------------------------------------------
    def _start_bridge(self) -> None:
        """生成 RL 世界、监听端口并等待控制器客户端连入。"""
        webots_bin = _find_webots()
        if not self.world_path.is_file():
            raise FileNotFoundError(f"世界文件不存在：{self.world_path}")
        world = _patch_rl_world(
            self.world_path, _generated_world_path(self.world_path)
        )

        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.bind((BRIDGE_HOST, self._bridge_port))
        self._server_sock.listen(1)
        self._server_sock.settimeout(self.webots_timeout)

        env = os.environ.copy()
        env["RL_BRIDGE_PORT"] = str(self._bridge_port)
        env["RL_BRIDGE_HOST"] = BRIDGE_HOST
        env["RL_CONTRACT_VERSION"] = str(
            getattr(self, "CONTRACT_VERSION", "")
        )
        home = env.get("WEBOTS_HOME", "/usr/local/webots")
        env["WEBOTS_HOME"] = home
        controller_python = Path(home) / "lib" / "controller" / "python"
        if controller_python.is_dir():
            existing = env.get("PYTHONPATH", "")
            env["PYTHONPATH"] = str(controller_python) + (
                os.pathsep + existing if existing else ""
            )

        cmd = ["--batch", "--minimize", "--mode=fast", "--no-rendering"]
        if self.render_mode == "human":
            cmd = ["--mode=fast"]
        elif self.render_mode == "rgb_array":
            cmd = ["--batch", "--minimize", "--mode=fast"]
        log_path = os.environ.get("RL_WEBOTS_LOG")
        if log_path:
            cmd.extend(["--stdout", "--stderr"])
        cmd = [webots_bin, *cmd, str(world)]
        if log_path:
            Path(log_path).parent.mkdir(parents=True, exist_ok=True)
            log_handle = Path(log_path).open("a", encoding="utf-8")
            self._webots_proc = subprocess.Popen(
                cmd,
                env=env,
                cwd=str(WEBOTS_SIM_DIR),
                stdout=log_handle,
                stderr=subprocess.STDOUT,
            )
            log_handle.close()
        else:
            self._webots_proc = subprocess.Popen(
                cmd,
                env=env,
                cwd=str(WEBOTS_SIM_DIR),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

        try:
            self._conn, _addr = self._server_sock.accept()
        except socket.timeout as exc:
            self.close()
            raise TimeoutError(
                "【错误】等待 rl_agent 控制器连接超时。"
            ) from exc
        self._conn.settimeout(self.webots_timeout)
        hello = self._recv()
        if hello.get("type") != "hello":
            self.close()
            raise RuntimeError(f"【错误】桥接握手失败，收到：{hello}")

    def _send(self, obj: Dict[str, Any]) -> None:
        """发送一行 JSON。"""
        if self._conn is None:
            raise RuntimeError("TCP 桥未连接；mock 测试请覆写 _send()")
        payload = json.dumps(obj, ensure_ascii=False, separators=(",", ":"))
        self._conn.sendall(payload.encode("utf-8") + b"\n")

    def _recv(self) -> Dict[str, Any]:
        """接收一行 JSON。"""
        if self._conn is None:
            raise RuntimeError("TCP 桥未连接；mock 测试请覆写 _recv()")
        buffer = bytearray()
        while True:
            chunk = self._conn.recv(65536)
            if not chunk:
                raise ConnectionError("TCP 桥连接已关闭")
            buffer.extend(chunk)
            newline = buffer.find(b"\n")
            if newline >= 0:
                break
        payload = bytes(buffer[:newline])
        try:
            message = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"TCP 状态不是合法 JSON：{payload!r}") from exc
        if not isinstance(message, dict):
            raise RuntimeError(f"TCP 状态必须是 JSON 对象，收到：{type(message)}")
        return message

    # ------------------------------------------------------------------
    # 契约状态解析与观测
    # ------------------------------------------------------------------
    @staticmethod
    def _array_state(
        state: Mapping[str, Any], key: str, expected_size: int
    ) -> np.ndarray:
        if key not in state:
            raise ValueError(f"TCP state 缺少字段：{key}")
        try:
            array = np.asarray(state[key], dtype=np.float32).reshape(-1)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"TCP state 字段 {key} 不是数值数组") from exc
        if array.size != expected_size:
            raise ValueError(
                f"TCP state 字段 {key} 维度错误：期望 {expected_size}，收到 {array.size}"
            )
        if not np.all(np.isfinite(array)):
            raise ValueError(f"TCP state 字段 {key} 包含 NaN/Inf")
        return array

    def _validate_state(self, state: Mapping[str, Any]) -> Dict[str, Any]:
        if state.get("type") != "state":
            raise RuntimeError(f"收到意外 TCP 消息：{state}")
        missing = [
            key
            for key in contract.TCP_STATE_REQUIRED_FIELDS
            if key not in state
        ]
        if missing:
            raise ValueError(f"TCP state 缺少字段：{missing}")

        validated: Dict[str, Any] = {"type": "state"}
        for key, size in contract.TCP_STATE_ARRAY_LENGTHS.items():
            validated[key] = self._array_state(state, key, size)
        for key in contract.TCP_STATE_SCALAR_FIELDS:
            value = state[key]
            if key == "done":
                try:
                    validated[key] = bool(value)
                except (TypeError, ValueError) as exc:
                    raise ValueError(f"TCP state.done 不是布尔值：{value!r}") from exc
                continue
            try:
                scalar = float(value)
            except (TypeError, ValueError) as exc:
                raise ValueError(f"TCP state.{key} 不是标量：{value!r}") from exc
            if not np.isfinite(scalar):
                raise ValueError(f"TCP state.{key} 不是有限值")
            validated[key] = scalar
        return validated

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return float((angle + np.pi) % (2.0 * np.pi) - np.pi)

    def _state_to_obs(self, state: Mapping[str, Any]) -> np.ndarray:
        q = np.asarray(state["q"], dtype=np.float32)
        dq = np.asarray(state["dq"], dtype=np.float32)
        rpy = np.asarray(state["rpy"], dtype=np.float32)
        v_body = np.asarray(state["v_body"], dtype=np.float32)
        omega = np.asarray(state["omega"], dtype=np.float32)
        contacts = np.asarray(state["contacts"], dtype=np.float32)
        jump_phase_time = contract.clip_jump_phase_time(
            float(state["jump_phase"])
        )

        # 前 42 维必须与旧 walk_env 完全相同；随后 13 维按契约顺序拼接。
        legacy = np.concatenate(
            [q, dq, rpy, v_body, self._prev_action]
        ).astype(np.float32)
        added = np.concatenate(
            [
                omega,
                self._command,
                np.asarray(
                    [1.0 if self._jump_latched else 0.0], dtype=np.float32
                ),
                np.asarray([jump_phase_time], dtype=np.float32),
                np.asarray([float(state["height"])], dtype=np.float32),
                contacts,
            ]
        ).astype(np.float32)
        obs = np.concatenate([legacy, added]).astype(np.float32)
        if obs.shape != (contract.OBS_DIM,):
            raise RuntimeError(
                f"观测维度错误：期望 {(contract.OBS_DIM,)}，收到 {obs.shape}"
            )
        if not np.all(np.isfinite(obs)):
            raise RuntimeError("观测包含 NaN/Inf")
        return obs

    def _reset_payload_extra(self) -> Dict[str, Any]:
        """子类在 reset 消息中附加的字段；旧环境保持为空。"""
        return {}

    def _act_payload_extra(self) -> Dict[str, Any]:
        """子类在动作消息中附加的字段；旧环境保持为空。"""
        return {}

    def _sanitize_action(self, action_array: np.ndarray) -> np.ndarray:
        """裁剪动作；子类可增加与回放一致的安全限制。"""
        return np.clip(
            action_array,
            contract.ACTION_LOW,
            contract.ACTION_HIGH,
        ).astype(np.float32)

    # ------------------------------------------------------------------
    # 命令随机化、目标偏航与跳跃锁存
    # ------------------------------------------------------------------
    @property
    def command(self) -> np.ndarray:
        """当前命令 ``[vx, vy, wz]`` 的只读副本。"""
        return self._command.copy()

    @property
    def jump_latched(self) -> bool:
        return self._jump_latched

    @property
    def jump_elapsed(self) -> float:
        return float(self._jump_elapsed)

    @property
    def jump_enabled(self) -> bool:
        return self.stage in {"S3_jump", "S4_mobile_terrain"}

    @property
    def command_enabled(self) -> bool:
        return self.stage in {
            "S2_command",
            "S3_jump",
            "S4_mobile_terrain",
        }

    def sample_command(self) -> np.ndarray:
        """按契约上下限均匀采样当前阶段命令。"""
        if not self.command_enabled:
            return np.zeros(3, dtype=np.float32)
        low = np.asarray(contract.COMMAND_LOW, dtype=np.float32)
        high = np.asarray(contract.COMMAND_HIGH, dtype=np.float32)
        sampled = self.np_random.uniform(low=low, high=high)
        return np.asarray(sampled, dtype=np.float32)

    def _set_command(self, command: Sequence[float]) -> None:
        if not self.command_enabled:
            self._command[:] = 0.0
            return
        clipped = contract.clip_command(command)
        self._command[:] = clipped

    def request_jump(self) -> bool:
        """请求一次跳跃并锁存；未锁存时返回 True。"""
        if not self.jump_enabled:
            raise RuntimeError(
                f"阶段 {self.stage} 不允许请求跳跃；仅 S3_jump/S4_mobile_terrain 可请求"
            )
        if self._jump_latched:
            return False
        self._jump_latched = True
        self._jump_elapsed = 0.0
        self._jump_had_contact_loss = False
        self._jump_peak_height = self._baseline_height
        return True

    def _maybe_auto_request_jump(self) -> None:
        if not self.jump_enabled or self._jump_latched:
            return
        first = self.jump_request_step
        interval = self.jump_interval_steps
        due_first = first > 0 and self._ep_steps == first
        due_repeat = (
            first >= 0
            and self._ep_steps > first
            and (self._ep_steps - first) % interval == 0
        )
        if due_first or due_repeat:
            self.request_jump()

    def _update_jump_latch(self, state: Mapping[str, Any]) -> None:
        if not self._jump_latched:
            return
        elapsed = contract.clip_jump_phase_time(float(state["jump_phase"]))
        self._jump_elapsed = float(elapsed)
        self._jump_peak_height = max(
            self._jump_peak_height, float(state["height"])
        )

        contacts = np.asarray(state["contacts"], dtype=np.float64)
        airborne = bool(np.all(contacts < CONTACT_ON_THRESHOLD))
        restored_now = bool(np.any(contacts >= CONTACT_ON_THRESHOLD))
        if airborne:
            self._jump_had_contact_loss = True
        contacts_restored = bool(
            self._jump_had_contact_loss and restored_now
        )

        # 高度增益达到阈值即判定成功，并在本 episode 中锁存为 true。
        if not self._jump_success and (
            self._jump_peak_height
            >= self._baseline_height + JUMP_SUCCESS_HEIGHT_GAIN
        ):
            self._jump_success = True
            self._jump_success_event = True

        if (
            self.stage == "S4_mobile_terrain"
            and not self._terrain_success
            and self._jump_success
            and contacts_restored
        ):
            self._terrain_success = True
            self._terrain_success_event = True

        clear = contract.should_clear_jump_latch(
            jump_latched=True,
            contacts_restored=contacts_restored,
            elapsed_seconds=max(self._jump_elapsed, self._jump_elapsed),
        )
        if clear:
            self._jump_latched = False
            self._jump_had_contact_loss = False

    def _command_error(self, state: Mapping[str, Any]) -> float:
        measured = np.asarray(
            [
                float(state["v_body"][0]),
                float(state["v_body"][1]),
                float(state["omega"][2]),
            ],
            dtype=np.float64,
        )
        desired = np.asarray(self._command, dtype=np.float64)
        return float(np.linalg.norm(measured - desired))

    def _heading_error(self, state: Mapping[str, Any]) -> float:
        return self._wrap_angle(float(state["rpy"][2]) - self._target_yaw)

    @staticmethod
    def _fallen(state: Mapping[str, Any]) -> bool:
        roll = float(state["rpy"][0])
        pitch = float(state["rpy"][1])
        height = float(state["height"])
        return (
            abs(roll) > MAX_ROLL
            or abs(pitch) > MAX_PITCH
            or height < MIN_BASE_HEIGHT
        )

    # ------------------------------------------------------------------
    # 信息与奖励
    # ------------------------------------------------------------------
    def _build_info(
        self,
        state: Mapping[str, Any],
        *,
        reward: float,
        fallen: bool,
    ) -> Dict[str, Any]:
        command_error = self._command_error(state)
        heading_error = self._heading_error(state)
        contacts = np.asarray(state["contacts"], dtype=np.float32)
        return {
            # info 使用 Python float，避免 JSON/测试读取时出现 float32 舍入。
            "command": np.asarray(self._command, dtype=np.float64).tolist(),
            "command_error": float(command_error),
            "heading_error": float(heading_error),
            "jump_success": bool(self._jump_success),
            "fallen": bool(fallen),
            "terrain_success": bool(self._terrain_success),
            "jump_latched": bool(self._jump_latched),
            "jump_phase_time": contract.clip_jump_phase_time(
                float(state["jump_phase"])
            ),
            "target_yaw": float(self._target_yaw),
            "base_height": float(state["height"]),
            "contacts": contacts.copy(),
            "episode_steps": int(self._ep_steps),
            "reward": float(reward),
        }

    def _compute_reward(
        self,
        state: Mapping[str, Any],
        action: np.ndarray,
        *,
        fallen: bool,
    ) -> Tuple[float, Dict[str, float]]:
        command_error = self._command_error(state)
        heading_error = self._heading_error(state)
        total, breakdown = reward_module.reward_for_state(
            self.stage,
            state,
            action,
            self._prev_action,
            command_error=command_error,
            heading_error=heading_error,
            fallen=fallen,
            jump_success_event=self._jump_success_event,
            terrain_success_event=self._terrain_success_event,
        )
        return float(total), breakdown.as_dict()

    # ------------------------------------------------------------------
    # Gymnasium API
    # ------------------------------------------------------------------
    def reset(
        self, *, seed: Optional[int] = None, options: Optional[Dict[str, Any]] = None
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        super().reset(seed=seed)
        options = dict(options or {})

        # 先根据阶段决定是否允许命令；reset 响应之后再确定目标偏航。
        requested_command = options.get("command")
        if requested_command is None or not self.command_enabled:
            self._set_command(self.sample_command())
        else:
            self._set_command(np.asarray(requested_command, dtype=np.float32))

        reset_payload = {
            "type": "reset",
            "command": self._command.tolist(),
            "jump_request": 0,
        }
        reset_payload.update(self._reset_payload_extra())
        self._send(reset_payload)
        raw_state = self._recv()
        state = self._validate_state(raw_state)
        self._last_state = state

        self._ep_steps = 0
        self._prev_action = np.zeros(contract.ACTION_DIM, dtype=np.float32)
        self._baseline_height = float(state["height"])
        if "target_yaw" in options:
            self._target_yaw = float(options["target_yaw"])
        else:
            self._target_yaw = float(state["rpy"][2])
        self._jump_latched = False
        self._jump_elapsed = 0.0
        self._jump_had_contact_loss = False
        self._jump_peak_height = self._baseline_height
        self._jump_success = False
        self._jump_success_event = False
        self._terrain_success = False
        self._terrain_success_event = False
        if bool(options.get("jump_request", False)) and self.jump_enabled:
            self.request_jump()

        obs = self._state_to_obs(state)
        info = self._build_info(state, reward=0.0, fallen=False)
        info["reward_breakdown"] = {
            "alive": 0.0,
            "posture": 0.0,
            "height": 0.0,
            "smoothness": 0.0,
            "command": 0.0,
            "heading": 0.0,
            "jump": 0.0,
            "terrain": 0.0,
            "fall": 0.0,
            "total": 0.0,
        }
        return obs, info

    def step(
        self, action: np.ndarray
    ) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        action_array = np.asarray(action, dtype=np.float32).reshape(-1)
        if action_array.size != contract.ACTION_DIM:
            raise ValueError(
                f"动作维度错误：期望 {contract.ACTION_DIM}，收到 {action_array.size}"
            )
        if not np.all(np.isfinite(action_array)):
            raise ValueError("动作包含 NaN/Inf")
        action_array = self._sanitize_action(action_array)
        q_des = action_to_q_des(action_array)

        # 命令周期重采样、自动跳跃请求与锁存都在发出动作前完成。
        if (
            self.command_enabled
            and self._ep_steps > 0
            and self._ep_steps % self.command_resample_steps == 0
        ):
            self._set_command(self.sample_command())
        self._maybe_auto_request_jump()
        jump_request = int(self._jump_latched)

        act_payload = {
            "type": "act",
            "a": action_array.tolist(),
            "q_des": q_des.tolist(),
            "command": self._command.tolist(),
            "jump_request": jump_request,
        }
        act_payload.update(self._act_payload_extra())
        self._send(act_payload)
        raw_state = self._recv()
        state = self._validate_state(raw_state)
        self._last_state = state
        self._ep_steps += 1

        # 目标偏航按本周期命令积分，再对返回 yaw 求包裹误差。
        self._target_yaw = self._wrap_angle(
            self._target_yaw
            + float(self._command[2]) * contract.CONTROL_DT_SECONDS
        )
        self._update_jump_latch(state)

        fallen = self._fallen(state)
        reward, reward_breakdown = self._compute_reward(
            state, action_array, fallen=fallen
        )
        terminated = bool(state["done"]) or fallen
        truncated = bool(
            not terminated and self._ep_steps >= self.max_episode_steps
        )

        info = self._build_info(
            state, reward=reward, fallen=fallen
        )
        info["reward_breakdown"] = reward_breakdown
        obs = self._state_to_obs(state)

        # 事件型奖励只在发生的那一步为 True，episode 成员标志保持锁存。
        self._jump_success_event = False
        self._terrain_success_event = False
        self._prev_action = action_array.copy()
        return obs, reward, terminated, truncated, info

    def render(self) -> Optional[np.ndarray]:
        """向控制器请求 RGB 帧；human 模式由 Webots 自己渲染。"""
        if self.render_mode != "rgb_array":
            return None
        try:
            self._send({"type": "get_rgb"})
            msg = self._recv()
        except (OSError, ConnectionError, RuntimeError):
            return None
        if msg.get("type") != "rgb":
            return None
        width = int(msg.get("w", 0))
        height = int(msg.get("h", 0))
        encoded = str(msg.get("data", ""))
        if width <= 0 or height <= 0 or not encoded:
            return None
        buffer = np.frombuffer(base64.b64decode(encoded), dtype=np.uint8)
        if buffer.size != width * height * 3:
            return None
        return buffer.reshape(height, width, 3)

    def close(self) -> None:
        """关闭连接与 Webots 子进程；mock 模式可重复调用。"""
        if self._closed:
            return
        self._closed = True
        for attribute in ("_conn", "_server_sock"):
            sock = getattr(self, attribute, None)
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
                setattr(self, attribute, None)
        if self._webots_proc is not None:
            try:
                self._webots_proc.terminate()
                self._webots_proc.wait(timeout=10)
            except Exception:  # noqa: BLE001 - 清理路径不向外抛异常
                try:
                    self._webots_proc.kill()
                except Exception:
                    pass
            self._webots_proc = None


# 两个类名都导出，便于监控/评估脚本按候选名解析。
LocoJumpEnv = QuadrupedLocoJumpEnv


def make_env(
    *,
    tag: str = "phase0",
    render_mode: Optional[str] = None,
    world: Optional[os.PathLike | str] = None,
    bridge_port: Optional[int] = None,
    start_bridge: bool = True,
    **kwargs: Any,
) -> QuadrupedLocoJumpEnv:
    """创建 ``yobogo_loco_jump_v1`` 环境的公共工厂。"""
    return QuadrupedLocoJumpEnv(
        tag=tag,
        render_mode=render_mode,
        world=world,
        bridge_port=bridge_port,
        start_bridge=start_bridge,
        **kwargs,
    )


if __name__ == "__main__":
    print(
        "【自检】导入通过：obs="
        f"{contract.OBS_DIM} action={contract.ACTION_DIM} "
        f"stage targets={contract.PHASE_TOTAL_STEPS}"
    )
    print(f"【自检】动作映射示例={action_to_q_des(np.ones(contract.ACTION_DIM))[:3]}")
