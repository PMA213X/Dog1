#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""QuadrupedWalkEnv：Webots 四足机器狗行走的 Gymnasium 环境。

接口约定（与 config.py / train_ppo.py / play.py 保持一致）：
    obs    : float32, shape (42,)
             12 关节角 + 12 关节角速度 + roll/pitch/yaw + 3 机体系线速度 + 12 上一动作
    action : float32, shape (12,), 取值 [-1, 1]
             q_des = q_stand + a * scale   （scale: abd 0.3, hip 0.5, knee 0.5）
    reset() -> (obs, info)
    step(a) -> (obs, reward, terminated, truncated, info)

奖励（walk_env.py 为准，权重见 config.REWARD_WEIGHTS）：
    +1 存活 + 3*vx - 2*|roll| - 2*|pitch| - 0.05*||a||^2 ，摔倒再 -10
终止：|roll|>0.8 或 |pitch|>0.8 或 base z<0.12 ；超过 MAX_EPISODE_STEPS 则 truncated。

后端：Webots Supervisor 控制器 webots-sim/controllers/rl_agent/rl_agent.py，
通过本进程内的 TCP 桥（JSON 行协议）交换状态/动作。本环境启动 Webots
子进程并扮演 TCP 服务端，rl_agent 作为客户端连入。

单环境说明：Webots 多实例并行较困难（GPU/端口/资源），当前为**单环境**，
对应 train_ppo.py 中 DummyVecEnv([make_env_fn()]) + n_steps=2048。
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
from typing import Any, Dict, Optional, Tuple

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError as _gym_exc:  # pragma: no cover - 无 gymnasium 时给出中文提示
    raise ImportError(
        "【错误】缺少 gymnasium，无法使用 QuadrupedWalkEnv。\n"
        "  请先安装：pip install gymnasium\n"
        f"  原始报错：{_gym_exc}"
    ) from _gym_exc

# 保证同目录 config.py 可导入
_RL_DIR = Path(__file__).resolve().parent
if str(_RL_DIR) not in sys.path:
    sys.path.insert(0, str(_RL_DIR))

from config import (  # noqa: E402
    ACTION_DIM,
    ACTION_HIGH,
    ACTION_LOW,
    MAX_EPISODE_STEPS,
    OBS_DIM,
    REWARD_WEIGHTS,
)

# ---------------------------------------------------------------------------
# 路径与常量
# ---------------------------------------------------------------------------
WEBOTS_SIM_DIR = _RL_DIR.parent                      # webots-sim/
WORLDS_DIR = WEBOTS_SIM_DIR / "worlds"
DEFAULT_WORLD = WORLDS_DIR / "parkour_dev.wbt"       # 平地 + 矮箱开发世界
# 运行时生成的 RL 世界（controller=rl_agent + supervisor TRUE），放在 worlds/ 下
# 以便 Webots 能按项目结构找到 controllers/rl_agent/
# 命名规则：.{源世界 stem}_rl.wbt，多世界（parkour_dev / parkour）互不覆盖
GENERATED_WORLD = WORLDS_DIR / ".parkour_dev_rl.wbt"


def _generated_world_path(src_world: Path) -> Path:
    """由源世界名推导 RL 生成世界路径（隐藏文件，避免与手维护世界冲突）。"""
    return WORLDS_DIR / f".{src_world.stem}_rl.wbt"

# 关节顺序：fr / fl / hr / hl ，每条腿 abd → hip → kn
# （与 gen_yobogo_robot.py 生成的电机/传感器命名一致）
LEG_ORDER: Tuple[str, ...] = ("fr", "fl", "hr", "hl")
JOINT_ORDER: Tuple[str, ...] = ("abd", "hip", "kn")
MOTOR_NAMES: Tuple[str, ...] = tuple(
    f"{leg}_{j}_motor" for leg in LEG_ORDER for j in JOINT_ORDER
)
SENSOR_NAMES: Tuple[str, ...] = tuple(
    f"{leg}_{j}_sensor" for leg in LEG_ORDER for j in JOINT_ORDER
)

# 站立关节角（abd, hip, kn）：本机型模型零位即站立（实测 z=0.26 稳定）
# 实际映射 q_des = q_stand + a*scale 由 rl_agent.py 执行，此处常量保持同步
Q_STAND_LEG: Tuple[float, float, float] = (0.0, 0.0, 0.0)
Q_STAND: np.ndarray = np.tile(np.array(Q_STAND_LEG, dtype=np.float64), 4)

# 动作缩放：abd 0.3 rad, hip 0.5 rad, knee 0.5 rad
ACTION_SCALE_LEG: Tuple[float, float, float] = (0.3, 0.5, 0.5)
ACTION_SCALE: np.ndarray = np.tile(np.array(ACTION_SCALE_LEG, dtype=np.float64), 4)

# 姿态 / 高度终止阈值
MAX_ROLL: float = 0.8          # |roll| 超过则判摔倒
MAX_PITCH: float = 0.8         # |pitch| 超过则判摔倒
MIN_BASE_Z: float = 0.12       # 机身高度低于则判摔倒

# 控制周期（秒）。rl_agent 侧按 basicTimeStep=4ms 走 5 步凑 20ms
SIM_DT: float = 0.02

# TCP 桥默认端口（可用环境变量 RL_BRIDGE_PORT 覆盖）
DEFAULT_BRIDGE_PORT: int = 11451
BRIDGE_HOST: str = "127.0.0.1"


# ===========================================================================
# 工具函数
# ===========================================================================
def _find_webots() -> str:
    """查找 Webots 可执行文件。"""
    import shutil

    candidates = []
    home = os.environ.get("WEBOTS_HOME")
    if home:
        candidates.append(os.path.join(home, "webots"))
    candidates += ["/usr/local/webots/webots", "/usr/local/bin/webots"]
    for cand in candidates:
        if os.path.isfile(cand) and os.access(cand, os.X_OK):
            return cand
    resolved = shutil.which("webots")
    if resolved:
        return resolved
    raise FileNotFoundError(
        "【错误】找不到 Webots 可执行文件。请安装 Webots 或设置 WEBOTS_HOME 环境变量。\n"
        "  例如：export WEBOTS_HOME=/usr/local/webots"
    )


def _patch_rl_world(src_world: Path, dst_world: Path) -> Path:
    """由普通世界生成 RL 世界：controller 改为 rl_agent 并开启 supervisor。

    运行时生成，避免手工维护两份世界文件产生漂移。
    """
    text = src_world.read_text(encoding="utf-8")
    # 控制器换为 rl_agent
    if 'controller "rl_agent"' not in text:
        if 'controller "' in text:
            text = re.sub(
                r'controller\s+"[^"]*"',
                'controller "rl_agent"',
                text,
                count=1,
            )
        else:
            raise ValueError(f"世界文件中找不到 controller 字段：{src_world}")
    # Supervisor 权限（simulationReset / getVelocity 需要）
    if "supervisor TRUE" not in text:
        text = text.replace(
            'controller "rl_agent"',
            'supervisor TRUE\n  controller "rl_agent"',
            1,
        )
    dst_world.write_text(text, encoding="utf-8")
    return dst_world


# ===========================================================================
# Gymnasium 环境
# ===========================================================================
class QuadrupedWalkEnv(gym.Env):
    """Webots 四足行走环境（单实例 TCP 桥接 Supervisor 控制器）。"""

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": int(1.0 / SIM_DT)}

    def __init__(
        self,
        render_mode: Optional[str] = None,
        world: Optional[os.PathLike | str] = None,
        bridge_port: Optional[int] = None,
        webots_timeout: float = 90.0,
    ) -> None:
        super().__init__()
        self.render_mode = render_mode
        self.world_path = Path(world) if world is not None else DEFAULT_WORLD
        self.webots_timeout = webots_timeout

        # 观测 / 动作空间（与 config.py 常量一致）
        high_obs = np.full(OBS_DIM, np.inf, dtype=np.float32)
        self.observation_space = spaces.Box(-high_obs, high_obs, dtype=np.float32)
        self.action_space = spaces.Box(
            low=np.full(ACTION_DIM, ACTION_LOW, dtype=np.float32),
            high=np.full(ACTION_DIM, ACTION_HIGH, dtype=np.float32),
            dtype=np.float32,
        )

        # 进程 / 连接
        self._webots_proc: Optional[subprocess.Popen] = None
        self._server_sock: Optional[socket.socket] = None
        self._conn: Optional[socket.socket] = None
        self._bridge_port = int(
            bridge_port if bridge_port is not None
            else os.environ.get("RL_BRIDGE_PORT", DEFAULT_BRIDGE_PORT)
        )

        # episode 记账
        self._ep_steps = 0
        self._prev_action = np.zeros(ACTION_DIM, dtype=np.float32)
        self._last_state: Dict[str, Any] = {}
        self._closed = False

        # 启动 Webots 并建立桥接
        self._start_bridge()

    # ------------------------------------------------------------------
    # Webots / TCP 桥生命周期
    # ------------------------------------------------------------------
    def _start_bridge(self) -> None:
        """生成 RL 世界、监听端口、启动 Webots，等待 rl_agent 连入。"""
        webots_bin = _find_webots()
        world = _patch_rl_world(self.world_path, _generated_world_path(self.world_path))

        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.bind((BRIDGE_HOST, self._bridge_port))
        self._server_sock.listen(1)
        self._server_sock.settimeout(self.webots_timeout)

        # rl_agent 通过环境变量得知端口（Webots 子进程继承环境）
        env = os.environ.copy()
        env["RL_BRIDGE_PORT"] = str(self._bridge_port)
        env["RL_BRIDGE_HOST"] = BRIDGE_HOST
        home = env.get("WEBOTS_HOME", "/usr/local/webots")
        env["WEBOTS_HOME"] = home
        # Webots 自带 controller python 库路径（rl_agent 需要）
        ctrl_py = Path(home) / "lib" / "controller" / "python"
        if ctrl_py.is_dir():
            env["PYTHONPATH"] = str(ctrl_py) + (
                os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else ""
            )

        cmd = [webots_bin, "--batch", "--minimize", "--mode=fast", "--no-rendering"]
        if self.render_mode == "human":
            # 有界面时去掉 batch/minimize/no-rendering
            cmd = [webots_bin, "--mode=fast"]
        elif self.render_mode == "rgb_array":
            # 录视频需要相机出图：--no-rendering 会禁用 Camera 传感器，
            # 这里去掉该选项，保留 batch/minimize 仍为无窗口运行
            cmd = [webots_bin, "--batch", "--minimize", "--mode=fast"]
        cmd.append(str(world))

        self._webots_proc = subprocess.Popen(
            cmd,
            env=env,
            cwd=str(WEBOTS_SIM_DIR),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        # 等待 rl_agent 连入
        try:
            self._conn, _addr = self._server_sock.accept()
        except socket.timeout as exc:
            self.close()
            raise TimeoutError(
                "【错误】等待 Webots 控制器 rl_agent 连接超时。\n"
                "  请确认 webots-sim/controllers/rl_agent/rl_agent.py 存在，"
                "且 Webots 能正常启动。"
            ) from exc
        self._conn.settimeout(self.webots_timeout)
        hello = self._recv()
        if hello.get("type") != "hello":
            self.close()
            raise RuntimeError(f"【错误】桥接握手失败，收到：{hello}")

    def _send(self, obj: Dict[str, Any]) -> None:
        """发送一行 JSON。"""
        assert self._conn is not None
        data = (json.dumps(obj) + "\n").encode("utf-8")
        self._conn.sendall(data)

    def _recv(self) -> Dict[str, Any]:
        """接收一行 JSON。"""
        assert self._conn is not None
        buf = b""
        while not buf.endswith(b"\n"):
            chunk = self._conn.recv(4096)
            if not chunk:
                raise ConnectionError("【错误】Webots 桥接连接被关闭")
            buf += chunk
        return json.loads(buf.decode("utf-8"))

    # ------------------------------------------------------------------
    # 状态 → 观测
    # ------------------------------------------------------------------
    def _state_to_obs(self, state: Dict[str, Any]) -> np.ndarray:
        """把 rl_agent 发来的原始状态拼成 42 维观测。"""
        q = np.asarray(state["q"], dtype=np.float32)          # 12
        dq = np.asarray(state["dq"], dtype=np.float32)        # 12
        rpy = np.asarray(state["rpy"], dtype=np.float32)      # 3
        v_world = np.asarray(state["v"], dtype=np.float32)    # 3 世界系线速度
        # 机体系速度：把世界系速度转到 body（只用 yaw）
        yaw = float(rpy[2])
        vx = np.cos(yaw) * v_world[0] + np.sin(yaw) * v_world[1]
        vy = -np.sin(yaw) * v_world[0] + np.cos(yaw) * v_world[1]
        vz = v_world[2]
        v_body = np.array([vx, vy, vz], dtype=np.float32)
        obs = np.concatenate([q, dq, rpy, v_body, self._prev_action]).astype(np.float32)
        assert obs.shape == (OBS_DIM,), f"观测维度错误：{obs.shape}"
        return obs

    def _body_vx(self, state: Dict[str, Any]) -> float:
        """机体系前向速度 vx（奖励用）。"""
        rpy = np.asarray(state["rpy"], dtype=np.float64)
        v_world = np.asarray(state["v"], dtype=np.float64)
        yaw = float(rpy[2])
        return float(np.cos(yaw) * v_world[0] + np.sin(yaw) * v_world[1])

    def _compute_reward(
        self, state: Dict[str, Any], action: np.ndarray, fallen: bool
    ) -> float:
        """按约定公式计算奖励（权重来自 config.REWARD_WEIGHTS）。"""
        vx = self._body_vx(state)
        roll = float(state["rpy"][0])
        pitch = float(state["rpy"][1])
        w = REWARD_WEIGHTS
        r = (
            w["alive_bonus"]
            + w["forward_vel"] * vx
            + w["roll_penalty"] * abs(roll)
            + w["pitch_penalty"] * abs(pitch)
            + w["action_sq"] * float(np.dot(action, action))
        )
        if fallen:
            r += w["fall_penalty"]
        return float(r)

    # ------------------------------------------------------------------
    # Gymnasium API
    # ------------------------------------------------------------------
    def reset(
        self, *, seed: Optional[int] = None, options: Optional[Dict[str, Any]] = None
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        super().reset(seed=seed)
        self._send({"type": "reset"})
        state = self._recv()
        if state.get("type") != "state":
            raise RuntimeError(f"【错误】reset 收到意外消息：{state}")
        self._last_state = state
        self._ep_steps = 0
        self._prev_action = np.zeros(ACTION_DIM, dtype=np.float32)
        obs = self._state_to_obs(state)
        info = self._build_info(state, reward=0.0, fallen=False)
        return obs, info

    def step(
        self, action: np.ndarray
    ) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        action = np.clip(
            np.asarray(action, dtype=np.float32).reshape(-1),
            ACTION_LOW,
            ACTION_HIGH,
        )
        self._send({"type": "act", "a": action.tolist()})
        state = self._recv()
        if state.get("type") != "state":
            raise RuntimeError(f"【错误】step 收到意外消息：{state}")
        self._last_state = state
        self._ep_steps += 1

        # 摔倒判定
        roll = float(state["rpy"][0])
        pitch = float(state["rpy"][1])
        z = float(state["z"])
        fallen = bool(abs(roll) > MAX_ROLL or abs(pitch) > MAX_PITCH or z < MIN_BASE_Z)

        reward = self._compute_reward(state, action, fallen)
        terminated = fallen
        truncated = bool(self._ep_steps >= MAX_EPISODE_STEPS)

        self._prev_action = action.copy()
        obs = self._state_to_obs(state)
        info = self._build_info(state, reward=reward, fallen=fallen)
        return obs, reward, terminated, truncated, info

    def _build_info(
        self, state: Dict[str, Any], reward: float, fallen: bool
    ) -> Dict[str, Any]:
        """info 里放回放/调试常用字段（play.py 会多键兼容读取）。"""
        vx = self._body_vx(state)
        z = float(state["z"])
        return {
            "vx": vx,
            "forward_velocity": vx,          # play.py 多键兼容
            "forward_distance": float(state.get("x", 0.0)),  # 世界 x 作前进距离近似
            "base_z": z,
            "roll": float(state["rpy"][0]),
            "pitch": float(state["rpy"][1]),
            "yaw": float(state["rpy"][2]),
            "fallen": fallen,
            "reward": reward,
            "episode_steps": self._ep_steps,
        }

    def render(self) -> Optional[np.ndarray]:
        """rgb_array：向 rl_agent 要一帧相机画面；human：Webots 自带窗口。"""
        if self.render_mode != "rgb_array":
            return None
        try:
            self._send({"type": "get_rgb"})
            msg = self._recv()
        except (OSError, ConnectionError):
            return None
        if msg.get("type") != "rgb":
            return None
        w = int(msg.get("w", 0))
        h = int(msg.get("h", 0))
        b64 = msg.get("data", "")
        if not b64 or w <= 0 or h <= 0:
            return None
        buf = np.frombuffer(base64.b64decode(b64), dtype=np.uint8)
        if buf.size != w * h * 3:
            return None
        return buf.reshape(h, w, 3)

    def close(self) -> None:
        """关闭连接与 Webots 子进程。"""
        if self._closed:
            return
        self._closed = True
        if self._conn is not None:
            try:
                self._conn.close()
            except OSError:
                pass
            self._conn = None
        if self._server_sock is not None:
            try:
                self._server_sock.close()
            except OSError:
                pass
            self._server_sock = None
        if self._webots_proc is not None:
            try:
                self._webots_proc.terminate()
                self._webots_proc.wait(timeout=10)
            except Exception:  # noqa: BLE001 - 清理路径不抛异常
                try:
                    self._webots_proc.kill()
                except Exception:
                    pass
            self._webots_proc = None


# ===========================================================================
# 冒烟自检：python walk_env.py
# ===========================================================================
if __name__ == "__main__":
    print("【自检】创建 QuadrupedWalkEnv ...")
    env = QuadrupedWalkEnv()
    try:
        obs, info = env.reset()
        print(f"【自检】reset: obs.shape={obs.shape} info_keys={sorted(info)}")
        assert obs.shape == (OBS_DIM,), obs.shape
        total_r = 0.0
        for i in range(50):
            obs, r, term, trunc, info = env.step(env.action_space.sample())
            total_r += r
            if term or trunc:
                print(f"【自检】第 {i+1} 步结束 term={term} trunc={trunc}，自动 reset")
                obs, info = env.reset()
        print(f"【自检】50 步OK r={r:.4f} term={term} trunc={trunc} 累计奖励={total_r:.3f}")
    finally:
        env.close()
