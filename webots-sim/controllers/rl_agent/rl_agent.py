#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Webots Supervisor 控制器：供 RL 环境（walk_env.py）驱动 YoboGo 机器狗。

本文件是 Webots 侧的机器人接口层，职责：
  1. 读取 12 关节角/角速度、IMU（rpy）、机体线速度、上一动作，拼成 42 维观测；
  2. 把 12 维动作 [-1,1] 映射为关节目标角 q_des = q_stand + a * scale 并写电机；
  3. 提供 episode 复位与摔倒检测；
  4. 通过 TCP 桥（JSON 行协议）与 walk_env.py 锁步交互。

观测布局（42 维）：
    [ 0:12] 关节角 q（rad，顺序 fr/fl/hr/hl × abd/hip/kn）
    [12:24] 关节角速度 qd（rad/s，由关节角差分估计）
    [24:27] 机体 rpy（rad，来自 InertialUnit，顺序 roll/pitch/yaw）
    [27:30] 机体线速度（m/s，世界系，来自 Supervisor getVelocity 前 3 维）
    [30:42] 上一时刻动作 a（无量纲，[-1,1]）

TCP 命令（walk_env → 本控制器，每行一条 JSON）：
    {"type": "reset"}                 复位 episode，返回初始状态
    {"type": "act", "a": [12 个数]}   施加动作并推进一个控制周期（20ms），返回新状态
    {"type": "get_rgb"}               返回一帧相机画面（base64 RGB，默认前视相机）
    {"type": "get_rgb", "camera": "third"}
                                    返回第三人称跟随相机画面（世界系后上方，跟 yaw）
    {"type": "exit"}                  退出

状态消息（本控制器 → walk_env）：
    {"type": "state", "q": [12], "dq": [12], "rpy": [3], "v": [3], "z": 0.26, "x": 0.0}

单独运行（作为 Webots 控制器，无 TCP）：`python3 rl_agent.py` 冒烟 50 步后退出。
带 TCP 桥运行：设置环境变量 RL_BRIDGE_PORT（由 walk_env.py 注入）。
"""

from __future__ import annotations

import base64
import json
import os
import socket
import sys
import time

import numpy as np

try:
    from controller import Supervisor
except ImportError:  # pragma: no cover - 独立调试兜底
    _home = os.environ.get("WEBOTS_HOME", "/usr/local/webots")
    sys.path.insert(0, os.path.join(_home, "lib", "controller", "python"))
    from controller import Supervisor

# ---------------------------------------------------------------------------
# 常量：关节顺序 / 维度 / 站立角 / 动作缩放
# ---------------------------------------------------------------------------
# 腿序：前右 fr、前左 fl、后右 hr、后左 hl；每条腿 abd(外展)/hip(髋)/kn(膝)
LEGS = ("fr", "fl", "hr", "hl")
JOINTS = ("abd", "hip", "kn")

MOTOR_NAMES = [f"{leg}_{joint}_motor" for leg in LEGS for joint in JOINTS]
SENSOR_NAMES = [f"{leg}_{joint}_sensor" for leg in LEGS for joint in JOINTS]

ACTION_DIM = 12
OBS_DIM = 42          # 12 关节角 + 12 角速度 + 3 rpy + 3 线速度 + 12 上一动作

# 站立角：模型零位即站立，全 0（冒烟实测 z=0.26 稳定）
Q_STAND = np.zeros(ACTION_DIM, dtype=np.float64)

# 动作缩放：abd 0.3 rad，hip/kn 0.5 rad
_JOINT_SCALE = {"abd": 0.3, "hip": 0.5, "kn": 0.5}
ACTION_SCALE = np.array(
    [_JOINT_SCALE[j] for _leg in LEGS for j in JOINTS], dtype=np.float64
)

# 初始位姿（与 parkour_dev.wbt 中 Robot 的 translation/rotation 一致）
INIT_TRANSLATION = [0.0, 0.0, 0.26]
INIT_ROTATION = [0.0, 0.0, 1.0, 0.0]   # 绕 z 轴 0 rad，即水平朝前

# 摔倒判定阈值（与 walk_env.py 一致）
ROLL_PITCH_LIMIT = 0.8    # rad，|roll| 或 |pitch| 超过即视为摔倒
Z_LIMIT = 0.12            # m，机体高度低于此值视为摔倒

# 一个 RL 控制周期 = 20ms（walk_env.SIM_DT）；basicTimeStep=4ms → 5 个仿真步
CTRL_PERIOD_MS = 20.0

# TCP 桥（walk_env.py 注入 RL_BRIDGE_PORT 时启用；否则跑内置冒烟）
BRIDGE_HOST = os.environ.get("RL_BRIDGE_HOST", "127.0.0.1")
BRIDGE_PORT = os.environ.get("RL_BRIDGE_PORT")  # None → 冒烟模式

# ---------------------------------------------------------------------------
# 第三人称跟随相机（可选，仅当世界里存在 DEF TP_CAM 时生效）
# 相机挂在机器人 children 下的 Transform 上，由 Supervisor 每帧把它摆到
# 世界系「机身后上方」，只跟随 yaw（保持水平），从而看到机身姿态/腿部动作。
# ---------------------------------------------------------------------------
TP_CAM_DEF = "TP_CAM"          # 世界文件里 Transform 的 DEF 名
TP_CAM_DEVICE = "third_camera"  # Camera 的 name 字段
TP_CAM_BACK = 2.5               # 机身之后距离（m）
TP_CAM_UP = 1.15                # 机身上方高度（m）
TP_CAM_LOOK_UP = 0.10           # 注视点相对机身抬高量（m），让机身居中偏下


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """欧拉角（ZYX，Webots InertialUnit 约定）→ 体系→世界 旋转矩阵。"""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    return rz @ ry @ rx


def _look_at_matrix(eye: np.ndarray, target: np.ndarray,
                    up: np.ndarray | None = None) -> np.ndarray:
    """look-at 旋转矩阵（列 = 相机局部轴在世界系方向）。

    约定与 Webots Camera 一致：+X 为光轴（看向 target），+Z 近似朝上。
    """
    if up is None:
        up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    x = np.asarray(target, dtype=np.float64) - np.asarray(eye, dtype=np.float64)
    n = np.linalg.norm(x)
    if n < 1e-9:
        x = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    else:
        x = x / n
    y = np.cross(up, x)
    ny = np.linalg.norm(y)
    if ny < 1e-8:                      # 视线与 up 平行，换参考轴
        y = np.cross(np.array([0.0, 1.0, 0.0]), x)
        ny = np.linalg.norm(y)
    y = y / max(ny, 1e-12)
    z = np.cross(x, y)
    z = z / max(np.linalg.norm(z), 1e-12)
    return np.column_stack([x, y, z])


def _matrix_to_axis_angle(R: np.ndarray) -> list:
    """旋转矩阵 → Webots SFRotation [ax, ay, az, angle]。"""
    R = np.asarray(R, dtype=np.float64)
    tr = float(np.trace(R))
    cos_a = float(np.clip((tr - 1.0) / 2.0, -1.0, 1.0))
    angle = float(np.arccos(cos_a))
    if angle < 1e-8:
        return [0.0, 0.0, 1.0, 0.0]
    if abs(np.pi - angle) < 1e-6:
        # 接近 180°：从 R+I 的最大列取轴
        M = R + np.eye(3)
        col = int(np.argmax(np.sum(M * M, axis=0)))
        axis = M[:, col]
        axis = axis / max(np.linalg.norm(axis), 1e-12)
        return [float(axis[0]), float(axis[1]), float(axis[2]), float(angle)]
    axis = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1],
    ], dtype=np.float64)
    axis = axis / max(np.linalg.norm(axis), 1e-12)
    return [float(axis[0]), float(axis[1]), float(axis[2]), float(angle)]


def _bgra_to_rgb(src: bytes, w: int, h: int) -> bytes:
    """Webots 相机 BGRA → 紧凑 RGB 字节（numpy 向量化，避免逐像素 Python 循环）。"""
    arr = np.frombuffer(src, dtype=np.uint8)
    if arr.size != w * h * 4:
        return b""
    arr = arr.reshape(h, w, 4)
    return arr[:, :, [2, 1, 0]].tobytes()


class RlAgent:
    """YoboGo 机器狗的 Supervisor 接口封装。"""

    def __init__(self) -> None:
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())   # 4 ms
        self.dt = self.timestep / 1000.0                     # 秒
        # 一个控制周期包含的仿真步数（4ms × 5 = 20ms）
        self.ctrl_steps = max(1, int(round(CTRL_PERIOD_MS / self.timestep)))

        # 12 个电机（位置控制模式）
        self.motors = []
        for name in MOTOR_NAMES:
            motor = self.robot.getDevice(name)
            if motor is None:
                raise RuntimeError(f"找不到电机: {name}")
            self.motors.append(motor)

        # 12 个关节位置传感器
        self.sensors = []
        for name in SENSOR_NAMES:
            sensor = self.robot.getDevice(name)
            if sensor is None:
                raise RuntimeError(f"找不到位置传感器: {name}")
            self.sensors.append(sensor)

        # IMU
        self.imu = self.robot.getDevice("inertial unit")
        self.gyro = self.robot.getDevice("gyro")
        self.accelerometer = self.robot.getDevice("accelerometer")
        if self.imu is None:
            raise RuntimeError("找不到 InertialUnit: 'inertial unit'")

        # 相机（可选，get_rgb 用）：前视 + 第三人称跟随
        self.camera = self.robot.getDevice("front_camera")
        self.tp_camera = self.robot.getDevice(TP_CAM_DEVICE)
        # 第三人称相机挂点（世界里 DEF TP_CAM 的 Transform，无则退化为固定机位）
        self.tp_cam_node = None
        try:
            self.tp_cam_node = self.robot.getFromDef(TP_CAM_DEF)
        except Exception:
            self.tp_cam_node = None

        # 对外可见的站立角 / 动作缩放（与 walk_env.ACTION_SCALE_LEG 同步）
        self.q_stand = Q_STAND.copy()
        self.action_scale = ACTION_SCALE.copy()

        # 机体节点（Supervisor 才能拿到，用于读位姿/线速度）
        self.robot_node = self.robot.getSelf()
        if self.robot_node is None:
            raise RuntimeError("getSelf() 返回空，确认当前控制器是 Supervisor")

        # 12 个 HingeJoint 节点（reset 时用 setJointPosition 瞬移关节角）
        self.joint_nodes = self._collect_joint_nodes(self.robot_node)

        self._enable_devices()

        # 上一时刻状态
        self.last_action = np.zeros(ACTION_DIM, dtype=np.float64)
        self._prev_q = self._read_joint_positions()
        # 控制周期时刻戳（关节差分用 20ms，而不是 4ms）
        self._prev_ctrl_time = self.robot.getTime()

    # ------------------------------------------------------------------
    # 设备使能（reset 后设备可能失效，可重复调用）
    # ------------------------------------------------------------------
    def _enable_devices(self) -> None:
        for sensor in self.sensors:
            sensor.enable(self.timestep)
        if self.imu is not None:
            self.imu.enable(self.timestep)
        if self.gyro is not None:
            self.gyro.enable(self.timestep)
        if self.accelerometer is not None:
            self.accelerometer.enable(self.timestep)
        if self.camera is not None:
            try:
                self.camera.enable(self.timestep)
            except Exception:
                pass
        if getattr(self, "tp_camera", None) is not None:
            try:
                self.tp_camera.enable(self.timestep)
            except Exception:
                pass

    # ------------------------------------------------------------------
    # 内部读取
    # ------------------------------------------------------------------
    def _collect_joint_nodes(self, root):
        """深度优先收集 root 子树里的 HingeJoint 节点（顺序与 MOTOR_NAMES 一致）。

        关节嵌套在各腿 Solid 的 children 里，链式关节还挂在 endPoint 上，
        因此这里遍历节点的所有字段（SFNode/MFNode）而不是只看 children。
        """
        joints = []
        visited = set()

        def walk(node):
            if node is None:
                return
            nid = node.getId()
            if nid in visited:
                return
            visited.add(nid)
            if node.getBaseTypeName() in ("HingeJoint", "SliderJoint", "BallJoint"):
                joints.append(node)
            for fi in range(node.getNumberOfFields()):
                field = node.getFieldByIndex(fi)
                ftype = field.getTypeName()
                if ftype.endswith("SFNode"):
                    walk(field.getSFNode())
                elif ftype.endswith("MFNode"):
                    for j in range(field.getCount()):
                        walk(field.getMFNode(j))

        walk(root)
        return joints

    def _read_joint_positions(self) -> np.ndarray:
        return np.array([s.getValue() for s in self.sensors], dtype=np.float64)

    def _read_rpy(self) -> np.ndarray:
        return np.array(self.imu.getRollPitchYaw(), dtype=np.float64)

    def _read_linear_velocity(self) -> np.ndarray:
        # Supervisor 读机体速度：[vx, vy, vz, wx, wy, wz]，取线速度前 3 维
        return np.array(self.robot_node.getVelocity()[:3], dtype=np.float64)

    def _read_base_height(self) -> float:
        return float(self.robot_node.getPosition()[2])

    # ------------------------------------------------------------------
    # 对外接口
    # ------------------------------------------------------------------
    def get_obs(self) -> np.ndarray:
        """返回 42 维观测：关节角/角速度/rpy/线速度/上一动作。"""
        q = self._read_joint_positions()
        # 关节角速度由位置差分估计（Motor.getVelocity() 返回的是目标速度，不可用）
        t = self.robot.getTime()
        dt = max(1e-4, t - self._prev_ctrl_time)
        qd = (q - self._prev_q) / dt if self._prev_q is not None else np.zeros_like(q)
        self._prev_q = q.copy()
        self._prev_ctrl_time = t

        rpy = self._read_rpy()
        v_lin = self._read_linear_velocity()

        obs = np.concatenate([q, qd, rpy, v_lin, self.last_action]).astype(np.float32)
        assert obs.shape == (OBS_DIM,), f"观测维度异常: {obs.shape}"
        return obs

    def read_state(self) -> dict:
        """返回 TCP 协议用的原始状态（walk_env 侧拼观测/算奖励）。"""
        q = self._read_joint_positions()
        t = self.robot.getTime()
        dt = max(1e-4, t - self._prev_ctrl_time)
        dq = (q - self._prev_q) / dt if self._prev_q is not None else np.zeros_like(q)
        self._prev_q = q.copy()
        self._prev_ctrl_time = t
        rpy = self._read_rpy()
        v = self._read_linear_velocity()
        # 角速度 [wx,wy,wz]：Supervisor getVelocity 后 3 维；缺失时用 yaw 差分兜底
        vel6 = np.array(self.robot_node.getVelocity(), dtype=np.float64)
        if vel6.size >= 6:
            w_ang = vel6[3:6]
        else:
            yaw = float(rpy[2])
            dyaw = yaw - getattr(self, "_prev_yaw", yaw)
            dyaw = (dyaw + np.pi) % (2 * np.pi) - np.pi
            dt = max(1e-4, t - getattr(self, "_prev_ctrl_time_w", t - 0.02))
            self._prev_yaw = yaw
            self._prev_ctrl_time_w = t
            w_ang = np.array([0.0, 0.0, dyaw / dt], dtype=np.float64)
        pos = np.array(self.robot_node.getPosition(), dtype=np.float64)
        return {
            "type": "state",
            "q": q.tolist(),
            "dq": dq.tolist(),
            "rpy": rpy.tolist(),
            "v": v.tolist(),
            "w": w_ang.tolist(),
            "z": float(pos[2]),
            "x": float(pos[0]),
            "y": float(pos[1]),
        }

    def apply_action(self, a: np.ndarray) -> None:
        """把 12 维动作写入电机：q_des = q_stand + clip(a) * scale。"""
        a = np.clip(np.asarray(a, dtype=np.float64).reshape(ACTION_DIM), -1.0, 1.0)
        q_des = self.q_stand + a * self.action_scale
        for motor, q in zip(self.motors, q_des):
            motor.setPosition(float(q))
        self.last_action = a.copy()

    def reset(self) -> np.ndarray:
        """复位仿真到初始站立状态，返回初始观测。

        采用 resetPhysics + 显式重设 translation/rotation 的方式：
        不整世界 reload，设备句柄保持有效，适合 RL 高频 episode 切换。
        """
        # 1) 清物理场（速度/力等）
        self.robot.simulationResetPhysics()

        # 2) 机体拉回初始位姿
        self.robot_node.getField("translation").setSFVec3f(list(INIT_TRANSLATION))
        self.robot_node.getField("rotation").setSFRotation(list(INIT_ROTATION))
        self.robot_node.setVelocity([0.0] * 6)

        # 3) 关节角瞬移回站立角（必须在 HingeJoint 节点上调用，不是 Robot 节点）
        for node, q in zip(self.joint_nodes, Q_STAND):
            node.setJointPosition(float(q), 1)
        # 4) 电机目标角也写站立角
        for motor, q in zip(self.motors, Q_STAND):
            motor.setPosition(float(q))

        # 5) 内部状态复位
        self.last_action = np.zeros(ACTION_DIM, dtype=np.float64)
        self._enable_devices()
        self.step()
        self._prev_q = self._read_joint_positions()
        self._prev_ctrl_time = self.robot.getTime()
        return self.get_obs()

    def step(self) -> None:
        """推进一个基本仿真步（4 ms）。"""
        self.robot.step(self.timestep)

    def step_control(self) -> None:
        """推进一个 RL 控制周期（20 ms = ctrl_steps 个仿真步）。"""
        for _ in range(self.ctrl_steps):
            if self.robot.step(self.timestep) == -1:
                break

    def is_fallen(self) -> bool:
        """|roll|>0.8 或 |pitch>0.8 或 z<0.12 视为摔倒。"""
        roll, pitch, _ = self._read_rpy()
        z = self._read_base_height()
        return abs(roll) > ROLL_PITCH_LIMIT or abs(pitch) > ROLL_PITCH_LIMIT or z < Z_LIMIT

    def _update_tp_cam_pose(self) -> None:
        """把 DEF TP_CAM 摆到「世界系机身后上方」，只跟随 yaw、保持水平。

        相机是 Robot 的后代节点，位姿在机体坐标系里；这里反解出机体局部
        translation/rotation 再写回，使相机在世界系稳定跟随（不受机身 roll/pitch
        牵动），从而能看清机身姿态与腿部动作。
        """
        node = self.tp_cam_node
        if node is None:
            return
        try:
            pos = np.array(self.robot_node.getPosition(), dtype=np.float64)
            roll, pitch, yaw = (float(v) for v in self._read_rpy())
            # 目标世界位姿：机身后方 TP_CAM_BACK、上方 TP_CAM_UP，看向机身
            heading = np.array([np.cos(yaw), np.sin(yaw), 0.0], dtype=np.float64)
            eye = pos + heading * (-TP_CAM_BACK) + np.array([0.0, 0.0, TP_CAM_UP])
            target = pos + np.array([0.0, 0.0, TP_CAM_LOOK_UP])
            R_wc = _look_at_matrix(eye, target)

            # 机体→世界旋转（IMU rpy，ZYX）
            R_bw = _rpy_to_matrix(roll, pitch, yaw)
            R_local = R_bw.T @ R_wc                       # 世界目标姿态 → 机体局部
            t_local = R_bw.T @ (eye - pos)

            node.getField("translation").setSFVec3f([float(v) for v in t_local])
            node.getField("rotation").setSFRotation(_matrix_to_axis_angle(R_local))
        except Exception:
            # 跟随失败时保持世界文件里的默认机位，不影响出图
            pass

    def get_rgb(self, which: str = "front") -> dict:
        """返回一帧相机 RGB（base64），供 play 录视频。

        which='front' → 前视相机 front_camera；
        which='third' → 第三人称跟随相机 third_camera（先摆好跟随位姿）。
        """
        if which == "third":
            self._update_tp_cam_pose()
            cam = getattr(self, "tp_camera", None)
        else:
            cam = self.camera
        if cam is None:
            return {"type": "rgb", "w": 0, "h": 0, "data": ""}
        w = cam.getWidth()
        h = cam.getHeight()
        raw = cam.getImage()  # Webots 返回 BGRA
        if raw is None:
            return {"type": "rgb", "w": 0, "h": 0, "data": ""}
        src = raw if isinstance(raw, (bytes, bytearray)) else bytes(raw)
        rgb = _bgra_to_rgb(bytes(src), w, h)
        if not rgb:
            return {"type": "rgb", "w": 0, "h": 0, "data": ""}
        return {
            "type": "rgb",
            "w": w,
            "h": h,
            "data": base64.b64encode(rgb).decode("ascii"),
        }

    # 便捷接口：walk_env 计算前进距离/奖励可能用到
    def get_base_position(self) -> np.ndarray:
        """返回机体位置 [x, y, z]（世界系）。"""
        return np.array(self.robot_node.getPosition(), dtype=np.float64)


# ---------------------------------------------------------------------------
# TCP 桥主循环（walk_env.py 注入 RL_BRIDGE_PORT 时启用）
# ---------------------------------------------------------------------------
def serve_tcp(agent: RlAgent) -> None:
    """连入 walk_env 的 TCP 服务端，按命令锁步推进仿真。"""
    port = int(BRIDGE_PORT)  # type: ignore[arg-type]
    sock = None
    last_exc = None
    for _ in range(60):
        try:
            sock = socket.create_connection((BRIDGE_HOST, port), timeout=5.0)
            break
        except OSError as exc:
            last_exc = exc
            time.sleep(0.5)
    if sock is None:
        raise ConnectionError(f"无法连接 walk_env 桥 {BRIDGE_HOST}:{port}: {last_exc}")
    sock.settimeout(180.0)
    print(f"【rl_agent】已连接 walk_env {BRIDGE_HOST}:{port}")

    def send(obj: dict) -> None:
        sock.sendall((json.dumps(obj) + "\n").encode("utf-8"))  # type: ignore[union-attr]

    def recv() -> dict:
        buf = b""
        while not buf.endswith(b"\n"):
            chunk = sock.recv(4096)  # type: ignore[union-attr]
            if not chunk:
                raise ConnectionError("walk_env 断开连接")
            buf += chunk
        return json.loads(buf.decode("utf-8"))

    send({"type": "hello", "timestep": agent.timestep})
    while True:
        try:
            msg = recv()
        except ConnectionError:
            print("【rl_agent】连接关闭，退出")
            break
        mtype = msg.get("type")
        if mtype == "reset":
            agent.reset()
            send(agent.read_state())
        elif mtype == "act":
            a = np.asarray(msg.get("a", []), dtype=np.float64)
            agent.apply_action(a)
            agent.step_control()      # 一个控制周期 20ms
            send(agent.read_state())
        elif mtype == "get_rgb":
            # camera: "front"(默认) / "third"（第三人称跟随相机）
            send(agent.get_rgb(str(msg.get("camera", "front"))))
        elif mtype == "exit":
            print("【rl_agent】收到 exit，退出")
            break
        else:
            send({"type": "error", "msg": f"未知命令 {mtype}"})
    try:
        sock.close()
    except OSError:
        pass


# ---------------------------------------------------------------------------
# 冒烟：作为 Webots 控制器运行时打印 50 步观测
# ---------------------------------------------------------------------------
def _smoke(n_steps: int = 50) -> None:
    agent = RlAgent()
    print(f"[smoke] timestep={agent.timestep} ms, obs_dim={OBS_DIM}, action_dim={ACTION_DIM}", flush=True)
    for i in range(n_steps):
        obs = agent.get_obs()
        agent.apply_action(np.zeros(ACTION_DIM))   # 保持站立
        agent.step()
        roll, pitch, _ = agent._read_rpy()
        z = agent._read_base_height()
        if i % 10 == 0 or i == n_steps - 1:
            print(
                f"[smoke] step={i:3d} obs.shape={obs.shape} "
                f"roll={roll:+.4f} pitch={pitch:+.4f} z={z:.4f} fallen={agent.is_fallen()}",
                flush=True,
            )
    print(f"[smoke] 完成 {n_steps} 步，最终 obs.shape={obs.shape}", flush=True)
    # 顺带验证 reset()
    obs0 = agent.reset()
    roll, pitch, _ = agent._read_rpy()
    z = agent._read_base_height()
    print(
        f"[smoke] reset 后 obs.shape={obs0.shape} roll={roll:+.4f} "
        f"pitch={pitch:+.4f} z={z:.4f} fallen={agent.is_fallen()}",
        flush=True,
    )
    # 再推进几步让 Webots 把 stdout 转发干净，否则 simulationQuit 会截断末尾输出
    for _ in range(25):
        if agent.robot.step(agent.timestep) == -1:
            break
    sys.stdout.flush()
    sys.stderr.flush()
    agent.robot.simulationQuit(0)


if __name__ == "__main__":
    if BRIDGE_PORT:
        serve_tcp(RlAgent())
    else:
        _smoke(50)
