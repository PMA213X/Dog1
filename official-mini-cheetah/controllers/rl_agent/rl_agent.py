#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""官方 Mini Cheetah 50 Hz TCP RL controller。"""

from __future__ import annotations

import json
import math
import os
import socket
import sys
import time
import traceback
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence


OFFICIAL_ROOT = Path(__file__).resolve().parents[2]
RL_ROOT = OFFICIAL_ROOT / "rl"
JOINT_ROOT = OFFICIAL_ROOT / "controllers" / "flat_ground_teleop"
for path in (str(OFFICIAL_ROOT), str(RL_ROOT), str(JOINT_ROOT)):
    if path not in sys.path:
        sys.path.insert(0, path)

import contract  # noqa: E402
from joint_safety import (  # noqa: E402
    DEFAULT_CROUCH,
    FLAT_MOTOR_NAMES,
    FLAT_SENSOR_NAMES,
    clamp_joint_targets,
    joint_pd_torque,
    MAX_TORQUE,
    SLIGHTLY_EXTENDED,
    validate_joint_feedback,
)
from tcp_protocol import decode, encode, validate_message  # noqa: E402


PHYSICS_DT = contract.WEBOTS_TIMESTEP_MS / 1000.0
CONTROL_DT = contract.CONTROL_DT_SECONDS
POSITION_CONTROL_TORQUE = 20.0
RESET_SETTLE_STEPS = 125
RESET_HOLD_STEPS = 50
RSI_STAGE_A_MAX_STEPS = 500
RSI_STAGE_B_MAX_STEPS = 1000
RSI_JOINT_ERROR_TOLERANCE = 0.05
RSI_HEIGHT_TOLERANCE = 0.03
RSI_POSTURE_TOLERANCE = 0.08
TOE_CONTACT_MATERIALS = frozenset(
    {"fr_toe", "fl_toe", "hr_toe", "hl_toe"}
)
ROBOT_SPAWNS: Dict[int, tuple[float, float, float]] = {
    0: (-4.0, -4.0, contract.BIRTH_HEIGHT),
    1: (4.0, -4.0, contract.BIRTH_HEIGHT),
    2: (-4.0, 4.0, contract.BIRTH_HEIGHT),
    3: (4.0, 4.0, contract.BIRTH_HEIGHT),
}


def controller_option(name: str, environment_name: str) -> str | None:
    """读取 external controller 命令行参数，其次读取环境变量。"""
    prefix = f"--{name}="
    for index, argument in enumerate(sys.argv[1:]):
        if argument.startswith(prefix):
            return argument[len(prefix):]
        if argument == f"--{name}" and index + 2 < len(sys.argv):
            return sys.argv[index + 2]
    return os.environ.get(environment_name)


def worker_id_from_robot_name(robot_name: str) -> int | None:
    """从 mini_cheetah_N 形式的 Robot 名称解析 worker 编号。"""
    prefix = "mini_cheetah_"
    if not robot_name.startswith(prefix):
        return None
    suffix = robot_name[len(prefix):]
    return int(suffix) if suffix.isdigit() else None


def matrix_to_rpy(matrix: Sequence[float]) -> tuple[float, float, float]:
    """把 Webots 3×3 姿态矩阵转换为 roll/pitch/yaw。"""
    if len(matrix) != 9 or not all(math.isfinite(float(v)) for v in matrix):
        raise ValueError("姿态矩阵必须是 9 个有限数")
    r00, r01, r02, r10, r11, r12, r20, r21, r22 = (
        float(value) for value in matrix
    )
    pitch = math.asin(max(-1.0, min(1.0, -r20)))
    roll = math.atan2(r21, r22)
    yaw = math.atan2(r10, r00)
    return roll, pitch, yaw


def world_to_body(vector: Sequence[float], matrix: Sequence[float]) -> tuple[float, float, float]:
    """把世界系向量旋转到机体系。"""
    if len(vector) != 3 or len(matrix) != 9:
        raise ValueError("向量或姿态矩阵维度错误")
    vx, vy, vz = (float(value) for value in vector)
    m = tuple(float(value) for value in matrix)
    return (
        m[0] * vx + m[3] * vy + m[6] * vz,
        m[1] * vx + m[4] * vy + m[7] * vz,
        m[2] * vx + m[5] * vy + m[8] * vz,
    )


def action_to_stance_targets(action: Sequence[float]) -> tuple[float, ...]:
    """把动作映射到仍明显弯膝、且满足 0.30 m 姿态门槛的训练基准。"""
    if (
        len(action) != contract.ACTION_DIM
        or not all(math.isfinite(float(value)) for value in action)
    ):
        return DEFAULT_CROUCH
    return tuple(
        max(low, min(high, base + float(value) * scale))
        for value, base, scale, (low, high) in zip(
            action,
            DEFAULT_CROUCH,
            contract.ACTION_SCALE,
            contract.TARGET_POSITION_LIMITS,
        )
    )


def smooth_action_targets(
    previous: Sequence[float],
    desired: Sequence[float],
) -> tuple[float, ...]:
    """限制策略动作映射后的相邻目标变化；不用于 reset/RSI。"""
    if len(previous) != contract.ACTION_DIM or len(desired) != contract.ACTION_DIM:
        raise ValueError("关节目标必须为 12 维")
    previous_values = tuple(float(value) for value in previous)
    desired_values = tuple(float(value) for value in desired)
    if not all(math.isfinite(value) for value in previous_values + desired_values):
        raise ValueError("关节目标必须为有限值")
    limit = contract.ACTION_TARGET_RATE_LIMIT * CONTROL_DT
    limited = tuple(
        max(current - limit, min(current + limit, target))
        for current, target in zip(previous_values, desired_values)
    )
    return clamp_joint_targets(limited)


def connection_failure_report(error: BaseException) -> str:
    """输出连接失败的稳定错误码并保留异常类型与消息。"""
    return (
        "RL_CONTROLLER_ERROR webots_connection_failed: "
        f"{type(error).__name__}: {error}"
    )


class RlAgentController:
    """设备、TCP、屈膝 PD 和 57 维状态的唯一控制器实现。"""

    def __init__(self, robot: Any) -> None:
        self.robot = robot
        self.timestep = contract.WEBOTS_TIMESTEP_MS
        self.robot_name = "mini_cheetah"
        self.worker_id = 0
        self.spawn_position = ROBOT_SPAWNS[self.worker_id]
        self.bridge_port = contract.BRIDGE_PORT
        self.motors: list[Any] = []
        self.sensors: list[Any] = []
        self.gyro: Any = None
        self.node: Any = None
        self._previous_positions = [None] * contract.ACTION_DIM
        self._previous_foot_points: list[tuple[float, float, float] | None] = [
            None
        ] * 4
        self._motor_strength = 1.0
        self._foot_shank_nodes: dict[int, Any] = {}
        self._foot_shank_node_ids: dict[int, int] = {}
        self._torque_diagnostic_printed = False
        self.initialization_error = ""
        self.initialization_detail = ""
        self.initialization_context = ""
        self.capability_errors: Dict[str, str] = {}
        self.torque_capability = False
        self.contact_capability = False
        self._resolver_context = ""
        self._command = [0.0, 0.0, 0.0]
        self._jump_latched = False
        self._jump_lost_contact = False
        self._jump_baseline = contract.REFERENCE_HEIGHT
        self._jump_peak = contract.REFERENCE_HEIGHT
        self._jump_started = 0.0
        self._jump_success_event = False
        self._jump_landing_event = False
        self._socket: socket.socket | None = None
        self._buffer = b""
        self._control_cycle = 0
        self._last_control_diagnostics: Dict[str, Any] = {}
        self._action_targets: tuple[float, ...] | None = None

    def _fail_initialization(
        self,
        reason: str,
        detail: str = "",
        cause: BaseException | None = None,
    ) -> bool:
        """记录致命初始化分支及原始异常上下文。"""
        if not hasattr(self, "initialization_error"):
            self.initialization_error = ""
            self.initialization_detail = ""
            self.initialization_context = ""
        self.initialization_error = reason
        self.initialization_detail = detail
        if cause is not None:
            self.initialization_context = (
                f"{type(cause).__name__}: {cause}\n"
                + traceback.format_exc()
            )
        return False

    def initialization_failure_report(self) -> str:
        """输出可定位的致命初始化失败原因。"""
        if not hasattr(self, "initialization_error"):
            self.initialization_error = ""
            self.initialization_detail = ""
            self.initialization_context = ""
        reason = self.initialization_error or "device_initialization_failed"
        detail = self.initialization_detail or reason
        return f"RL_CONTROLLER_ERROR {reason}: {detail}"

    def _record_capability_error(
        self,
        reason: str,
        detail: str = "",
        cause: BaseException | None = None,
    ) -> None:
        """记录不阻断 RSI 的指标能力错误和异常上下文。"""
        if not hasattr(self, "capability_errors"):
            self.capability_errors = {}
        context = detail or reason
        if cause is not None:
            context += f" | {type(cause).__name__}: {cause}"
            if traceback.format_exc().strip() != "NoneType: None":
                context += "\n" + traceback.format_exc()
        self.capability_errors[reason] = context

    def capability_error_reports(self) -> list[str]:
        """输出指标能力缺失；这些错误不会阻止屈膝 reset。"""
        if not hasattr(self, "capability_errors"):
            self.capability_errors = {}
        return [
            f"RL_CAPABILITY_ERROR {reason}: {context}"
            for reason, context in self.capability_errors.items()
        ]

    def _initialize_metric_capabilities(self) -> None:
        """初始化 torque/contact 指标能力；失败只降级，不阻断 RSI。"""
        torque_ok = True
        for motor in self.motors:
            try:
                if not hasattr(motor, "enableTorqueFeedback") or not hasattr(
                    motor, "getTorqueFeedbackSamplingPeriod"
                ):
                    raise RuntimeError("Webots Motor 缺少 torque feedback 接口")
                motor.enableTorqueFeedback(self.timestep)
                sampling_period = int(motor.getTorqueFeedbackSamplingPeriod())
                if sampling_period != self.timestep:
                    raise RuntimeError(
                        f"torque feedback sampling period={sampling_period}, "
                        f"expected={self.timestep}"
                    )
            except Exception as exc:
                torque_ok = False
                self._record_capability_error(
                    "torque_sampling_failed",
                    "扭矩反馈能力不可用，将使用明确的 PD 替代数据源",
                    exc,
                )
        self.torque_capability = torque_ok

        contact_tracking_ok = True
        try:
            self.node.enableContactPointsTracking(self.timestep, True)
        except Exception as exc:
            contact_tracking_ok = False
            self._record_capability_error(
                "contact_tracking_failed",
                "接触点跟踪能力不可用",
                exc,
            )
        try:
            resolver_ok = self._resolve_foot_shank_nodes()
        except Exception as exc:
            resolver_ok = False
            self._record_capability_error(
                "foot_shank_resolution_failed",
                "小腿节点解析能力不可用",
                exc,
            )
        if not resolver_ok:
            if "foot_shank_resolution_failed" not in self.capability_errors:
                self._record_capability_error(
                    "foot_shank_resolution_failed",
                    self._resolver_context
                    or "无法解析四个膝关节对应的具名 shank 节点",
                )
        self.contact_capability = contact_tracking_ok and resolver_ok

    def initialize(self) -> bool:
        """校验同步配置，并获取 12 电机、12 传感器、IMU 和接触跟踪。"""
        self.initialization_error = ""
        self.initialization_detail = ""
        self.initialization_context = ""
        self.capability_errors = {}
        self.torque_capability = False
        self.contact_capability = False
        self._resolver_context = ""
        if int(self.robot.getBasicTimeStep()) != self.timestep:
            return self._fail_initialization(
                "timestep_mismatch",
                f"expected={self.timestep}",
            )
        self.node = self.robot.getSelf()
        if self.node is None:
            return self._fail_initialization("self_node_missing")
        name_field = self.node.getField("name")
        if name_field is None:
            return self._fail_initialization("robot_name_missing")
        self.robot_name = str(name_field.getSFString())
        configured_worker_id = controller_option("worker-id", "RL_WORKER_ID")
        if configured_worker_id is not None:
            self.worker_id = int(configured_worker_id)
        else:
            parsed_worker_id = worker_id_from_robot_name(self.robot_name)
            self.worker_id = (
                parsed_worker_id if parsed_worker_id is not None else 0
            )
        if self.worker_id not in ROBOT_SPAWNS:
            return self._fail_initialization(
                "worker_id_invalid",
                f"worker_id={self.worker_id}",
            )
        self.spawn_position = ROBOT_SPAWNS[self.worker_id]
        configured_port = controller_option("bridge-port", "RL_BRIDGE_PORT")
        self.bridge_port = (
            int(configured_port)
            if configured_port is not None
            else contract.BRIDGE_PORT + self.worker_id
        )
        synchronization_field = self.node.getField("synchronization")
        if (
            synchronization_field is None
            or not synchronization_field.getSFBool()
        ):
            return self._fail_initialization("synchronization_failed")
        for name in FLAT_MOTOR_NAMES:
            motor = self.robot.getDevice(name)
            if motor is None:
                return self._fail_initialization(
                    "motor_device_missing",
                    name,
                )
            # 位置保持使用 world 已声明的官方 20 N·m 设备能力；
            # 仅显式 setTorque() 分支仍受 MAX_TORQUE=15 N·m 限制。
            motor.setAvailableTorque(POSITION_CONTROL_TORQUE)
            self.motors.append(motor)
        for name in FLAT_SENSOR_NAMES:
            sensor = self.robot.getDevice(name)
            if sensor is None:
                return self._fail_initialization(
                    "sensor_device_missing",
                    name,
                )
            sensor.enable(self.timestep)
            self.sensors.append(sensor)
        self.gyro = self.robot.getDevice("gyro")
        if self.gyro is not None:
            self.gyro.enable(self.timestep)
        self._initialize_metric_capabilities()
        if not (
            len(self.motors) == contract.ACTION_DIM
            and len(self.sensors) == contract.ACTION_DIM
            and self.node is not None
        ):
            return self._fail_initialization(
                "device_count_mismatch",
                f"motors={len(self.motors)} sensors={len(self.sensors)}",
            )
        return True

    @staticmethod
    def _node_name(node: Any) -> str:
        """读取节点 name 字段；协议节点缺失时返回空串。"""
        try:
            field = node.getField("name")
            return "" if field is None else str(field.getSFString())
        except Exception:
            return ""

    @staticmethod
    def _device_tag(device: Any) -> int:
        """只把当前 Webots Python 接受的整数 device tag 交给 Supervisor。"""
        if isinstance(device, int) and not isinstance(device, bool):
            return device
        tag = getattr(device, "_tag", None)
        if isinstance(tag, int) and not isinstance(tag, bool):
            return tag
        getter = getattr(device, "getTag", None)
        if callable(getter):
            value = getter()
            if isinstance(value, int) and not isinstance(value, bool):
                return value
        raise TypeError(
            "Webots getFromDevice 只接受 device tag；"
            f"收到 {type(device).__name__}"
        )

    def _resolve_shank_node(self, device: Any, expected_name: str) -> Any:
        """按整数 device tag 回溯，并以精确 shank 名称校验唯一目标。"""
        try:
            tag = self._device_tag(device)
            node = self.robot.getFromDevice(tag)
        except Exception as exc:
            self._resolver_context = (
                f"getFromDevice(device_tag) failed: {type(exc).__name__}: {exc}"
            )
            return None
        if node is None:
            self._resolver_context = f"device tag={tag} 无法映射节点"
            return None
        for depth in range(8):
            current = node
            # HingeJoint 的 endPoint 是膝电机到小腿的最短、最稳定路径。
            try:
                endpoint_field = current.getField("endPoint")
                endpoint = (
                    endpoint_field.getSFNode()
                    if endpoint_field is not None
                    else None
                )
                if endpoint is not None:
                    current = endpoint
            except Exception as exc:
                self._resolver_context = (
                    f"endPoint traversal failed: "
                    f"{type(exc).__name__}: {exc}"
                )
            name = self._node_name(current)
            if name == expected_name:
                return current
            try:
                node = current.getParentNode()
            except Exception as exc:
                self._resolver_context = (
                    f"getParentNode failed: {type(exc).__name__}: {exc}"
                )
                return None
        self._resolver_context = (
            f"device tag={tag} 八级回溯后仍未找到精确节点 {expected_name}"
        )
        return None

    def _resolve_foot_shank_nodes(self) -> bool:
        """按四个精确 shank 名称和唯一 node_id 建立接触映射。"""
        self._foot_shank_nodes = {}
        self._foot_shank_node_ids = {}
        resolved_by_name: dict[str, Any] = {}
        knee_motor_indexes = (2, 5, 8, 11)
        for foot_index, motor_index in enumerate(knee_motor_indexes):
            if motor_index >= len(self.motors):
                self._resolver_context = f"缺少膝电机索引 {motor_index}"
                return False
            leg = contract.LEG_NAMES[foot_index]
            expected_name = f"{leg}_shank_link"
            node = self._resolve_shank_node(
                self.motors[motor_index],
                expected_name,
            )
            if node is None:
                return False
            if expected_name in resolved_by_name:
                self._resolver_context = f"重复解析 {expected_name}"
                return False
            resolved_by_name[expected_name] = node
        if set(resolved_by_name) != {
            f"{leg}_shank_link" for leg in contract.LEG_NAMES
        }:
            self._resolver_context = "四个 shank 名称集合不完整"
            return False
        for foot_index, leg in enumerate(contract.LEG_NAMES):
            node = resolved_by_name[f"{leg}_shank_link"]
            try:
                node_id = int(node.getId())
            except Exception as exc:
                self._resolver_context = (
                    f"shank node_id 读取失败: {type(exc).__name__}: {exc}"
                )
                return False
            if node_id <= 0 or node_id in self._foot_shank_node_ids.values():
                self._resolver_context = (
                    f"shank node_id 非唯一或无效: {node_id}"
                )
                return False
            self._foot_shank_nodes[foot_index] = node
            self._foot_shank_node_ids[foot_index] = node_id
        return len(self._foot_shank_nodes) == 4

    def read_feedback(self) -> tuple[tuple[float, ...], tuple[float, ...], bool]:
        """读取 12 路位置，并以控制周期估计速度。"""
        positions: list[float] = []
        velocities: list[float] = []
        for index, sensor in enumerate(self.sensors):
            try:
                position = float(sensor.getValue())
            except Exception:
                position = 0.0
            previous = self._previous_positions[index]
            velocity = 0.0 if previous is None else (position - previous) / CONTROL_DT
            positions.append(position)
            velocities.append(velocity)
            self._previous_positions[index] = position
        valid = validate_joint_feedback(positions, velocities)
        return tuple(positions), tuple(velocities), valid

    def read_physics_feedback(self) -> tuple[tuple[float, ...], tuple[float, ...]]:
        """PD 控制内部按 4 ms 读取关节反馈。"""
        positions: list[float] = []
        velocities: list[float] = []
        for index, sensor in enumerate(self.sensors):
            position = float(sensor.getValue())
            previous = self._previous_positions[index]
            velocity = 0.0 if previous is None else (position - previous) / PHYSICS_DT
            positions.append(position)
            velocities.append(velocity)
            self._previous_positions[index] = position
        return tuple(positions), tuple(velocities)

    def set_targets(self, targets: Sequence[float]) -> None:
        """切换到显式位置目标；用于出生和复位沉降。"""
        for motor, target in zip(self.motors, targets):
            motor.setPosition(float(target))

    def enable_torque_control(self) -> None:
        """使用 Webots 默认位置控制器，开放官方 20 N·m 设备能力。"""
        for motor in self.motors:
            motor.setAvailableTorque(
                POSITION_CONTROL_TORQUE * self._motor_strength
            )
            motor.setPosition(float("inf"))

    def apply_pd_action(self, action: Sequence[float]) -> bool:
        """以 50 Hz 执行一个动作周期并返回反馈是否有效。"""
        desired_targets = action_to_stance_targets(action)
        previous_targets = self._action_targets
        if previous_targets is None:
            try:
                current_targets = tuple(
                    float(motor.getTargetPosition())
                    for motor in self.motors
                )
                if len(current_targets) == contract.ACTION_DIM and all(
                    math.isfinite(value) for value in current_targets
                ):
                    previous_targets = current_targets
            except Exception:
                previous_targets = None
            if previous_targets is None:
                previous_targets = DEFAULT_CROUCH
        # 动作映射后再做目标限速；RSI/reset 的 set_targets 不经过此路径。
        targets = smooth_action_targets(previous_targets, desired_targets)
        self._action_targets = targets
        # 先提交明确目标，再读取反馈；避免切换到 position=inf 后无目标运行。
        self.set_targets(targets)
        positions, velocities, valid = self.read_feedback()
        target_positions = tuple(
            float(motor.getTargetPosition()) for motor in self.motors
        )
        available_torque = tuple(
            float(motor.getAvailableTorque()) for motor in self.motors
        )
        height_before = float(self.node.getPosition()[2])
        diagnostics = {
            "cycle": self._control_cycle,
            "feedback_valid": bool(valid),
            "target_min": min(target_positions),
            "target_max": max(target_positions),
            "targets_equal_requested": all(
                abs(actual - requested) <= 1e-9
                for actual, requested in zip(target_positions, targets)
            ),
            "available_torque_min": min(available_torque),
            "available_torque_max": max(available_torque),
            "height_before": height_before,
            "used_position_target": bool(valid),
            "used_zero_torque_branch": not valid,
            "position_infinite": any(
                not math.isfinite(value) for value in target_positions
            ),
            "target_rate_limit": contract.ACTION_TARGET_RATE_LIMIT,
            "target_delta_max": max(
                abs(actual - previous)
                for actual, previous in zip(target_positions, previous_targets)
            ),
            "desired_target_delta_max": max(
                abs(actual - previous)
                for actual, previous in zip(desired_targets, previous_targets)
            ),
        }
        for _ in range(5):
            if valid:
                self.set_targets(targets)
            else:
                for motor in self.motors:
                    motor.setTorque(0.0)
            if self.robot.step(4) < 0:
                return False
        diagnostics["height_after"] = float(self.node.getPosition()[2])
        self._last_control_diagnostics = diagnostics
        if self._control_cycle == 0:
            print(
                "RL_CONTROL_DIAGNOSTIC "
                + json.dumps(diagnostics, ensure_ascii=True),
                flush=True,
            )
        self._control_cycle += 1
        return valid

    def _world_info_node(self) -> Any:
        """定位 WorldInfo；兼容 root 特殊字段与 children 首节点两种 API。"""
        root = self.robot.getRoot()
        try:
            world_info_field = root.getField("worldInfo")
            if world_info_field is not None:
                world_info = world_info_field.getSFNode()
                if world_info is not None:
                    return world_info
        except Exception:
            pass
        children = root.getField("children")
        if children is None:
            raise RuntimeError("root 缺少 children 字段")
        for index in range(int(children.getCount())):
            node = children.getMFNode(index)
            if node is None:
                continue
            try:
                if str(node.getTypeName()) == "WorldInfo":
                    return node
            except Exception:
                pass
            if node.getField("contactProperties") is not None:
                return node
        raise RuntimeError("未找到 WorldInfo/contactProperties 节点")

    def _read_toe_friction_snapshot(
        self,
        stage: str,
        target: float,
    ) -> Dict[str, Any]:
        """读取四个 toe 的摩擦配置；读取异常显式记录为 UNKNOWN。"""
        report: Dict[str, Any] = {
            "stage": stage,
            "target": float(target),
            "status": "UNKNOWN",
            "items": [],
            "errors": [],
        }
        try:
            world_info = self._world_info_node()
            contacts = world_info.getField("contactProperties")
            count = int(contacts.getCount())
        except Exception as exc:
            report["errors"].append(
                {
                    "scope": "contact_properties",
                    "error_type": type(exc).__name__,
                    "error": str(exc),
                }
            )
            return report

        for index in range(count):
            try:
                node = contacts.getMFNode(index)
                material1 = str(node.getField("material1").getSFString())
            except Exception as exc:
                report["errors"].append(
                    {
                        "scope": "material1",
                        "contact_index": index,
                        "error_type": type(exc).__name__,
                        "error": str(exc),
                    }
                )
                continue
            if material1 not in TOE_CONTACT_MATERIALS:
                continue

            item: Dict[str, Any] = {
                "contact_index": index,
                "material1": material1,
                "material2": "UNKNOWN",
                "target": float(target),
                "readback": "UNKNOWN",
                "match": False,
                "status": "UNKNOWN",
            }
            try:
                item["material2"] = str(
                    node.getField("material2").getSFString()
                )
            except Exception as exc:
                report["errors"].append(
                    {
                        "scope": "material2",
                        "contact_index": index,
                        "error_type": type(exc).__name__,
                        "error": str(exc),
                    }
                )
            try:
                friction_field = node.getField("coulombFriction")
                count = int(friction_field.getCount())
                if count < 1:
                    raise ValueError(f"coulombFriction 元素数={count}")
                values = [
                    float(friction_field.getMFFloat(index))
                    for index in range(count)
                ]
                if not all(math.isfinite(value) for value in values):
                    raise ValueError(f"coulombFriction 非有限：{values}")
                readback = values[0]
                if not math.isfinite(readback):
                    raise ValueError(f"coulombFriction 非有限：{readback}")
                item["count"] = count
                item["values"] = values
                item["readback"] = readback
                item["match"] = math.isclose(
                    readback,
                    float(target),
                    rel_tol=0.0,
                    abs_tol=1e-9,
                )
            except Exception as exc:
                report["errors"].append(
                    {
                        "scope": "coulombFriction",
                        "contact_index": index,
                        "error_type": type(exc).__name__,
                        "error": str(exc),
                    }
                )
            if item["material2"] != "UNKNOWN" and item[
                "readback"
            ] != "UNKNOWN":
                item["status"] = "OK"
            report["items"].append(item)

        if (
            not report["errors"]
            and len(report["items"]) == len(TOE_CONTACT_MATERIALS)
            and all(item["status"] == "OK" for item in report["items"])
        ):
            report["status"] = (
                "OK"
                if all(bool(item["match"]) for item in report["items"])
                else "MISMATCH"
            )
        elif len(report["items"]) != len(TOE_CONTACT_MATERIALS):
            report["errors"].append(
                {
                    "scope": "toe_count",
                    "expected": len(TOE_CONTACT_MATERIALS),
                    "actual": len(report["items"]),
                }
            )
        return report

    @staticmethod
    def _emit_friction_readback(
        before: Mapping[str, Any],
        after: Mapping[str, Any],
        write_error: str | None = None,
    ) -> None:
        """输出写入前后 readback；异常不会被静默吞掉。"""
        errors = [
            *before.get("errors", []),
            *after.get("errors", []),
        ]
        if write_error is not None:
            errors.append(
                {
                    "scope": "write",
                    "error": write_error,
                }
            )
        if write_error is not None:
            status = "WRITE_ERROR"
        elif errors:
            status = "UNKNOWN"
        elif after.get("status") == "MISMATCH":
            status = "MISMATCH"
        else:
            status = str(after.get("status", "UNKNOWN"))
        report = {
            "target": before.get("target", after.get("target")),
            "status": status,
            "before": dict(before),
            "after": dict(after),
            "errors": errors,
        }
        print(
            "RL_FRICTION_READBACK "
            + json.dumps(report, ensure_ascii=True),
            flush=True,
        )
        if errors:
            print(
                "RL_FRICTION_READBACK_ERROR "
                + json.dumps(
                    {"status": status, "errors": errors},
                    ensure_ascii=True,
                ),
                flush=True,
            )

    def apply_randomization(self, values: Mapping[str, Any]) -> None:
        """尽力在 Supervisor 可写字段上设置摩擦和质量倍率并 readback。"""
        try:
            friction = float(values.get("friction", 1.2))
            mass_scale = float(values.get("mass_scale", 1.0))
            if not (math.isfinite(friction) and math.isfinite(mass_scale)):
                raise ValueError
            before = self._read_toe_friction_snapshot(
                "before_write",
                friction,
            )
            write_error: str | None = None
            try:
                world_info = self._world_info_node()
                contacts = world_info.getField("contactProperties")
                for index in range(contacts.getCount()):
                    node = contacts.getMFNode(index)
                    field = node.getField("coulombFriction")
                    # R2025a 的 Field Python API 使用 MFFloat 访问器；
                    # 旧的 set1Float 在该运行时不存在。
                    field.setMFFloat(0, friction)
            except Exception as exc:
                write_error = (
                    f"{type(exc).__name__}: {exc}"
                )
            after = self._read_toe_friction_snapshot(
                "after_write",
                friction,
            )
            self._emit_friction_readback(before, after, write_error)
            if write_error is not None:
                return
            physics_field = self.node.getField("physics")
            physics = physics_field.getSFNode()
            mass_field = physics.getField("mass")
            if not hasattr(self, "_base_mass"):
                self._base_mass = float(mass_field.getSFFloat())
            mass_field.setSFFloat(self._base_mass * mass_scale)
            motor_strength = float(values.get("motor_strength", 1.0))
            if math.isfinite(motor_strength):
                self._motor_strength = max(0.5, min(1.5, motor_strength))
        except Exception as exc:
            # Webots 字段权限差异不阻断训练；延迟随机化始终由环境执行。
            # 摩擦 readback 异常已在上方显式输出，不能静默吞掉。
            print(
                "RL_RANDOMIZATION_WRITE_ERROR "
                + json.dumps(
                    {
                        "error_type": type(exc).__name__,
                        "error": str(exc),
                    },
                    ensure_ascii=True,
                ),
                flush=True,
            )
            return

    @staticmethod
    def _max_joint_error(
        targets: Sequence[float],
        positions: Sequence[float],
    ) -> float:
        """返回 RSI 目标与关节反馈的最大绝对误差。"""
        if len(targets) != contract.ACTION_DIM or len(positions) != len(targets):
            return math.inf
        errors = [
            abs(float(target) - float(position))
            for target, position in zip(targets, positions)
        ]
        return max(errors) if all(math.isfinite(value) for value in errors) else math.inf

    def _validate_rsi_state(
        self,
        targets: Sequence[float],
        *,
        stage: str,
        check_height: bool,
    ) -> None:
        """验收首个 RSI 状态；任一越界均作为技术失败抛出。"""
        try:
            positions = [float(sensor.getValue()) for sensor in self.sensors]
            height = float(self.node.getPosition()[2])
            roll, pitch, _yaw = matrix_to_rpy(self.node.getOrientation())
        except Exception as exc:
            raise RuntimeError(
                f"RL_RESET_TECHNICAL_FAILURE {stage}_read_failed: "
                f"{type(exc).__name__}: {exc}"
            ) from exc
        joint_error = self._max_joint_error(targets, positions)
        posture = max(abs(roll), abs(pitch))
        failures: list[str] = []
        if not math.isfinite(height):
            failures.append(f"height={height}")
        elif check_height and abs(height - contract.REFERENCE_HEIGHT) > (
            RSI_HEIGHT_TOLERANCE + 1e-9
        ):
            failures.append(
                f"height={height:.6f}, "
                f"expected={contract.REFERENCE_HEIGHT:.6f}±"
                f"{RSI_HEIGHT_TOLERANCE:.3f}"
            )
        if not math.isfinite(posture) or posture > RSI_POSTURE_TOLERANCE + 1e-9:
            failures.append(f"posture={posture:.6f}")
        if not math.isfinite(joint_error) or joint_error > (
            RSI_JOINT_ERROR_TOLERANCE + 1e-9
        ):
            failures.append(f"joint_error={joint_error:.6f}")
        if failures:
            print(
                "RL_RSI_STAGE_DIAGNOSTIC "
                + json.dumps(
                    {
                        "stage": stage,
                        "height": height,
                        "rpy": [roll, pitch, 0.0],
                        "orientation": list(self.node.getOrientation()),
                        "joint_error": joint_error,
                        "targets": list(targets),
                        "positions": positions,
                        "failures": failures,
                    },
                    ensure_ascii=True,
                ),
                flush=True,
            )
            raise RuntimeError(
                "RL_RESET_TECHNICAL_FAILURE "
                f"{stage}: " + "; ".join(failures)
            )

    def _hold_rsi_stage(
        self,
        targets: Sequence[float],
        *,
        stage: str,
        max_steps: int,
        check_height: bool,
        pin_initial_pose: bool = False,
    ) -> None:
        """按固定步数稳定 RSI 阶段，保证四台同步 controller 同时收口。"""
        for index in range(max_steps):
            if pin_initial_pose:
                self._set_rsi_initial_pose()
            self.set_targets(targets)
            if self.robot.step(4) < 0:
                raise RuntimeError(f"{stage} 期间 Webots 退出")
            if pin_initial_pose:
                # 第一阶段只负责出生姿态与关节收敛；接地后的实际姿态
                # 仍由第二阶段独立验收，不能用固定姿态掩盖接地故障。
                self._set_rsi_initial_pose()
            if index + 1 < max_steps:
                continue
            try:
                self._validate_rsi_state(
                    targets,
                    stage=f"{stage}_{index}",
                    check_height=check_height,
                )
            except RuntimeError as exc:
                raise RuntimeError(
                    f"RL_RESET_TECHNICAL_FAILURE {stage}: {exc}"
                ) from exc
            return
        raise RuntimeError(
            f"RL_RESET_TECHNICAL_FAILURE {stage}: 固定步数循环未执行"
        )

    def _set_rsi_initial_pose(self) -> None:
        """固定第一阶段的出生平移、姿态和零速度，供关节目标收敛。"""
        self.node.getField("translation").setSFVec3f(
            [
                self.spawn_position[0],
                self.spawn_position[1],
                contract.BIRTH_HEIGHT,
            ]
        )
        self.node.getField("rotation").setSFRotation([0.0, 0.0, 1.0, 0.0])
        try:
            self.node.setVelocity([0.0] * 6)
        except Exception:
            # 旧测试替身或 Webots 权限差异不改变字段固定逻辑。
            pass

    def reset_robot(
        self,
        randomization: Mapping[str, Any],
        *,
        reset_style: str = "settle",
        perturbation: Sequence[float] | None = None,
    ) -> None:
        """按 RSI 或沉降模式复位，并保持固定站立初态。"""
        self.apply_randomization(randomization)
        z_position = (
            contract.REFERENCE_HEIGHT
            if reset_style == "rsi"
            else contract.BIRTH_HEIGHT
        )
        self.node.getField("translation").setSFVec3f(
            [self.spawn_position[0], self.spawn_position[1], z_position]
        )
        self.node.getField("rotation").setSFRotation(
            [0.0, 0.0, 1.0, 0.0]
        )
        self.node.resetPhysics()
        external_impulse = randomization.get("external_impulse", (0.0, 0.0, 0.0))
        if (
            isinstance(external_impulse, (list, tuple))
            and len(external_impulse) == 3
            and all(math.isfinite(float(value)) for value in external_impulse)
        ):
            impulse = [float(value) for value in external_impulse]
            if any(abs(value) > 0.0 for value in impulse):
                self.node.setVelocity(
                    [impulse[0], impulse[1], impulse[2], 0.0, 0.0, 0.0]
                )
        self._previous_positions = [None] * contract.ACTION_DIM
        self._previous_foot_points = [None] * 4
        self._torque_diagnostic_printed = False
        if reset_style == "rsi":
            rsi_targets = contract.rsi_targets(perturbation)
            # 第一阶段在出生高度空载收敛屈膝目标，避免冷启动直腿把机身顶起。
            self.set_targets(rsi_targets)
            self._hold_rsi_stage(
                rsi_targets,
                stage="rsi_stage_a",
                max_steps=RSI_STAGE_A_MAX_STEPS,
                check_height=False,
                pin_initial_pose=True,
            )
            # 第二阶段把已收敛的屈膝模型放回参考高度并完成接地验收。
            self.node.getField("translation").setSFVec3f(
                [
                    self.spawn_position[0],
                    self.spawn_position[1],
                    contract.REFERENCE_HEIGHT,
                ]
            )
            self.node.resetPhysics()
            self.enable_torque_control()
            self.set_targets(rsi_targets)
            self._hold_rsi_stage(
                rsi_targets,
                stage="rsi_stage_b",
                max_steps=RSI_STAGE_B_MAX_STEPS,
                check_height=True,
            )
            self._validate_rsi_state(
                rsi_targets,
                stage="rsi_first_state",
                check_height=True,
            )
        else:
            self.set_targets(SLIGHTLY_EXTENDED)
            for _ in range(RESET_SETTLE_STEPS):
                if self.robot.step(4) < 0:
                    raise RuntimeError("复位期间 Webots 退出")
            self.enable_torque_control()
            # position=inf 后立即恢复明确屈膝目标，并稳定一个 0.2 s 保持窗口。
            self.set_targets(SLIGHTLY_EXTENDED)
            for _ in range(RESET_HOLD_STEPS):
                if self.robot.step(4) < 0:
                    raise RuntimeError("屈膝保持期间 Webots 退出")
        reset_positions = tuple(
            float(sensor.getValue()) for sensor in self.sensors
        )
        reset_targets = tuple(
            float(motor.getTargetPosition()) for motor in self.motors
        )
        reset_torque = tuple(
            float(motor.getAvailableTorque()) for motor in self.motors
        )
        print(
            "RL_RESET_DIAGNOSTIC "
            + json.dumps(
                {
                    "height": float(self.node.getPosition()[2]),
                    "target_min": min(reset_targets),
                    "target_max": max(reset_targets),
                    "target_errors": [
                        abs(target - position)
                        for target, position in zip(reset_targets, reset_positions)
                    ],
                    "available_torque_min": min(reset_torque),
                    "available_torque_max": max(reset_torque),
                    "position_infinite": any(
                        not math.isfinite(value) for value in reset_targets
                    ),
                },
                ensure_ascii=True,
            ),
            flush=True,
        )
        if all(math.isfinite(value) for value in reset_targets):
            self._action_targets = tuple(reset_targets)
        else:
            self._action_targets = None
        self._command = [0.0, 0.0, 0.0]
        self._jump_latched = False
        self._jump_lost_contact = False
        self._jump_success_event = False
        self._jump_landing_event = False
        # 固化复位后的关节基线，使自然摔倒恢复后的首个状态速度为零且有限。
        self._previous_positions = list(reset_positions)
        self._control_cycle = 0
        self._last_control_diagnostics = {
            "reset_height": float(self.node.getPosition()[2]),
                    "reset_target_min": min(
                float(motor.getTargetPosition()) for motor in self.motors
            ),
            "reset_target_max": max(
                float(motor.getTargetPosition()) for motor in self.motors
            ),
            "reset_available_torque_min": min(
                float(motor.getAvailableTorque()) for motor in self.motors
            ),
            "reset_position_infinite": any(
                not math.isfinite(float(motor.getTargetPosition()))
                for motor in self.motors
            ),
        }

    def contact_points(self) -> list[Any]:
        """返回当前 Robot 子树接触点。"""
        if not getattr(self, "contact_capability", True):
            return []
        try:
            return list(self.node.getContactPoints(True))
        except Exception as exc:
            self.contact_capability = False
            self._record_capability_error(
                "contact_read_failed",
                "接触点读取失败，接触指标 fail closed",
                exc,
            )
            print("\n".join(self.capability_error_reports()), flush=True)
            return []

    def _contact_node_foot_index(self, contact: Any) -> tuple[int | None, Any]:
        """只接受预先解析并缓存的小腿 node_id，不做象限猜测。"""
        try:
            contact_node_id = int(contact.getNodeId())
        except Exception:
            return None, None
        for index, shank_node_id in self._foot_shank_node_ids.items():
            if contact_node_id == shank_node_id:
                return index, self._foot_shank_nodes.get(index)
        return None, None

    def foot_contacts_and_slip(
        self,
    ) -> tuple[list[float], list[float], bool, bool, str]:
        """返回四足独立接触、12 维足端速度及非足/机身碰撞标记。"""
        if not getattr(self, "contact_capability", True):
            return (
                [0.0] * 4,
                [0.0] * 12,
                False,
                False,
                "capability_missing_fail_closed",
            )
        matrix = tuple(float(value) for value in self.node.getOrientation())
        robot_position = tuple(float(value) for value in self.node.getPosition())
        foot_contacts = [0.0] * 4
        foot_velocities = [0.0] * 12
        current_points: list[tuple[float, float, float] | None] = [None] * 4
        non_foot_contact = False
        body_contact = False
        contact_seen = False
        mapped_active = 0
        velocity_valid_active = 0
        contacts = self.contact_points()
        if not getattr(self, "contact_capability", True):
            return (
                [0.0] * 4,
                [0.0] * 12,
                False,
                False,
                "capability_missing_fail_closed",
            )
        for contact in contacts:
            contact_seen = True
            point = tuple(float(value) for value in contact.getPoint())
            if len(point) != 3 or not all(math.isfinite(value) for value in point):
                continue
            relative = tuple(point[axis] - robot_position[axis] for axis in range(3))
            local_point = world_to_body(relative, matrix)
            index, contact_node = self._contact_node_foot_index(contact)
            if index is None:
                non_foot_contact = True
                if (
                    point[2] <= 0.08
                    and abs(local_point[0]) <= 0.28
                    and abs(local_point[1]) <= 0.18
                ):
                    body_contact = True
                continue
            mapped_active += 1
            foot_contacts[index] = 1.0
            previous = self._previous_foot_points[index]
            if contact_node is not None:
                try:
                    node_position = tuple(
                        float(value) for value in contact_node.getPosition()
                    )
                    velocity_six = tuple(
                        float(value) for value in contact_node.getVelocity()
                    )
                    relative = tuple(
                        point[axis] - node_position[axis] for axis in range(3)
                    )
                    angular = velocity_six[3:6]
                    toe_velocity = [
                        velocity_six[axis]
                        + angular[(axis + 1) % 3] * relative[(axis + 2) % 3]
                        - angular[(axis + 2) % 3] * relative[(axis + 1) % 3]
                        for axis in range(3)
                    ]
                    foot_velocities[index * 3:index * 3 + 3] = toe_velocity
                    velocity_valid_active += 1
                except Exception:
                    pass
            current_points[index] = point
        for index in range(4):
            if current_points[index] is None:
                self._previous_foot_points[index] = None
                for axis in range(3):
                    foot_velocities[index * 3 + axis] = 0.0
            else:
                self._previous_foot_points[index] = current_points[index]
        if contact_seen and mapped_active == 0:
            contact_source = "unmapped_fail_closed"
        elif mapped_active > 0 and velocity_valid_active != mapped_active:
            contact_source = "foot_velocity_fail_closed"
        else:
            contact_source = "node_id"
        return (
            foot_contacts,
            foot_velocities,
            non_foot_contact,
            body_contact,
            contact_source,
        )

    def contact_count(self) -> int:
        """返回当前接触点数量。"""
        return len(self.contact_points())

    def state(self) -> Dict[str, Any]:
        """组装环境所需的 57 维基础状态。"""
        positions, velocities, valid = self.read_feedback()
        if not valid:
            raise RuntimeError("关节反馈非有限")
        matrix = tuple(float(value) for value in self.node.getOrientation())
        roll, pitch, yaw = matrix_to_rpy(matrix)
        velocity_six_axis = tuple(
            float(value) for value in self.node.getVelocity()
        )
        # Webots Supervisor 返回线速度与角速度共 6 维；机体系平移只取前 3 维。
        world_velocity = velocity_six_axis[:3]
        v_body = world_to_body(world_velocity, matrix)
        if self.gyro is not None:
            omega = tuple(float(value) for value in self.gyro.getValues())
        else:
            omega = (0.0, 0.0, 0.0)
        position = tuple(float(value) for value in self.node.getPosition())
        foot_contacts, foot_velocities, non_foot_contact, body_contact, contact_source = (
            self.foot_contacts_and_slip()
        )
        joint_torques: list[float] = []
        for index, motor in enumerate(self.motors):
            try:
                torque = float(motor.getTorqueFeedback())
            except Exception:
                torque = joint_pd_torque(
                    float(motor.getTargetPosition()),
                    positions[index],
                    0.0,
                    velocities[index],
                    index % 3,
                )
            joint_torques.append(torque if math.isfinite(torque) else 0.0)
        torque_capability = getattr(self, "torque_capability", True)
        torque_source = (
            "torque_feedback"
            if torque_capability
            else "capability_missing_pd_fallback"
        )
        if not torque_capability or not any(
            abs(value) > 1e-9 for value in joint_torques
        ):
            fallback = [
                joint_pd_torque(
                    float(motor.getTargetPosition()),
                    positions[index],
                    0.0,
                    velocities[index],
                    index % 3,
                )
                for index, motor in enumerate(self.motors)
            ]
            if torque_capability and any(
                abs(value) > 1e-9 for value in fallback
            ):
                joint_torques = fallback
                torque_source = "pd_fallback"
            elif not torque_capability:
                joint_torques = fallback
        self._last_torque_sample = tuple(joint_torques)
        if not self._torque_diagnostic_printed:
            print(
                "RL_TORQUE_DIAGNOSTIC "
                + json.dumps(
                    {
                        "source": torque_source,
                        "values": [round(value, 6) for value in joint_torques],
                        "min": min(joint_torques),
                        "max": max(joint_torques),
                    },
                    ensure_ascii=True,
                ),
                flush=True,
            )
            self._torque_diagnostic_printed = True
        height = position[2]
        jump_phase = min(
            1.0,
            max(0.0, (self.robot.getTime() - self._jump_started) if self._jump_latched else 0.0),
        )
        values = (
            positions
            + velocities
            + (roll, pitch, yaw)
            + v_body
            + omega
            + (height, jump_phase)
            + tuple(joint_torques)
            + tuple(foot_velocities)
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise RuntimeError("状态包含 NaN/Inf")
        return {
            "q": list(positions),
            "dq": list(velocities),
            "rpy": [roll, pitch, yaw],
            "v_body": list(v_body),
            "omega_body": list(omega),
            "contacts": foot_contacts,
            "foot_contacts": foot_contacts,
            "foot_velocities": foot_velocities,
            "joint_torques": joint_torques,
            "torque_source": torque_source,
            "non_foot_contact": non_foot_contact,
            "body_contact": body_contact,
            "foot_contact_source": contact_source,
            "height": height,
            "position": list(position),
            "jump_phase": jump_phase,
            "jump_latched": self._jump_latched,
            "jump_success": self._jump_success_event,
            "jump_landing": self._jump_landing_event,
            "done": False,
        }

    def update_jump(self, requested: bool, height: float, contacts: int) -> None:
        """维护二进制跳跃锁存、峰值与成功/落地事件。"""
        self._jump_success_event = False
        self._jump_landing_event = False
        if requested and not self._jump_latched:
            self._jump_latched = True
            self._jump_lost_contact = False
            self._jump_baseline = height
            self._jump_peak = height
            self._jump_started = self.robot.getTime()
        if not self._jump_latched:
            return
        if contacts == 0:
            self._jump_lost_contact = True
        self._jump_peak = max(self._jump_peak, height)
        if self._jump_lost_contact and contacts > 0:
            gain = self._jump_peak - self._jump_baseline
            self._jump_success_event = gain >= contract.JUMP_SUCCESS_HEIGHT_GAIN
            self._jump_landing_event = True
            self._jump_latched = False
            self._jump_lost_contact = False

    # ------------------------------------------------------------------
    # TCP
    # ------------------------------------------------------------------
    def connect(self) -> None:
        """连接环境服务器并发送契约 hello。"""
        host = os.environ.get("RL_BRIDGE_HOST", "127.0.0.1")
        last_error: Exception | None = None
        for _ in range(60):
            try:
                self._socket = socket.create_connection(
                    (host, self.bridge_port), timeout=5.0
                )
                break
            except OSError as exc:
                last_error = exc
                time.sleep(0.5)
        if self._socket is None:
            raise ConnectionError(f"无法连接环境：{last_error}") from last_error
        self._socket.settimeout(180.0)
        try:
            self.send(
                {
                    "type": "hello",
                    "contract": contract.CONTRACT_VERSION,
                    "worker_id": self.worker_id,
                    "robot_name": self.robot_name,
                    "timestep": self.timestep,
                }
            )
        except (OSError, ConnectionError) as exc:
            raise ConnectionError(
                f"hello 发送失败 host={host} port={self.bridge_port}"
            ) from exc
        print(
            f"RL_CONTROLLER_CONNECTED host={host} port={self.bridge_port} "
            f"worker_id={self.worker_id} robot_name={self.robot_name} "
            f"timestep={self.timestep} contract={contract.CONTRACT_VERSION}",
            flush=True,
        )

    def send(self, payload: Mapping[str, Any]) -> None:
        if self._socket is None:
            raise ConnectionError("TCP 未连接")
        self._socket.sendall(encode(payload))

    def receive(self) -> Dict[str, Any]:
        if self._socket is None:
            raise ConnectionError("TCP 未连接")
        while b"\n" not in self._buffer:
            chunk = self._socket.recv(65536)
            if not chunk:
                raise ConnectionError("环境断开")
            self._buffer += chunk
        line, self._buffer = self._buffer.split(b"\n", 1)
        return decode(line)

    def tcp_loop(self) -> int:
        """锁步处理 reset/act/get_rgb/exit。"""
        self.connect()
        while True:
            message = self.receive()
            kind = str(message.get("type", ""))
            if kind == "exit":
                return 0
            if kind == "reset":
                validate_message(message, "reset")
                birth_position = message.get("birth_position")
                if birth_position is not None:
                    if len(birth_position) != 3 or not all(
                        math.isfinite(float(value)) for value in birth_position
                    ):
                        raise ValueError("reset 出生点必须是 3 个有限数")
                    self.spawn_position = tuple(
                        float(value) for value in birth_position
                    )
                self.reset_robot(
                    message.get("randomization", {}),
                    reset_style=str(message.get("reset_style", "settle")),
                    perturbation=message.get("perturbation"),
                )
                self._command = [float(value) for value in message["command"]]
                self.send(self.state())
                continue
            if kind == "act":
                validate_message(message, "act")
                self._command = [float(value) for value in message["command"]]
                valid = self.apply_pd_action(message["action"])
                state = self.state()
                self.update_jump(
                    bool(message["jump_request"]),
                    float(state["height"]),
                    int(sum(state["contacts"])),
                )
                state["jump_success"] = self._jump_success_event
                state["jump_landing"] = self._jump_landing_event
                state["jump_phase"] = min(
                    1.0,
                    max(
                        0.0,
                        (self.robot.getTime() - self._jump_started)
                        if self._jump_latched
                        else 0.0,
                    ),
                )
                state["done"] = not valid
                self.send(state)
                continue
            if kind == "get_rgb":
                self.send({"type": "rgb", "data": ""})
                continue
            raise ValueError(f"未知消息类型：{kind}")

    # ------------------------------------------------------------------
    # 无 TCP 的 headless 冒烟与屈膝沉降
    # ------------------------------------------------------------------
    def run_standalone(self, mode: str) -> int:
        """执行 50 步冒烟或 3 秒屈膝沉降并退出仿真。"""
        self.reset_robot({"friction": 1.2, "mass_scale": 1.0, "delay_steps": 0})
        # 两种模式都走同一动作周期，保持 reset/act 之外无额外仿真步调用点。
        if mode != "smoke":
            self.set_targets(SLIGHTLY_EXTENDED)
        cycles = 50 if mode == "smoke" else 150
        heights: list[float] = []
        contacts_seen = 0
        finite_samples = 0
        for cycle in range(cycles):
            if not self.apply_pd_action((0.0,) * contract.ACTION_DIM):
                break
            state = self.state()
            if cycle >= 5:
                heights.append(float(state["height"]))
            contacts_seen += int(sum(state["contacts"]) > 0)
            finite_samples += 1
            if cycle % 10 == 0:
                print(
                    f"RL_STANDALONE step={cycle} height={state['height']:.4f} "
                    f"rpy={state['rpy']} contacts={sum(state['contacts'])}",
                    flush=True,
                )
        span = max(heights) - min(heights) if heights else float("inf")
        final = self.state()
        tilt = max(abs(final["rpy"][0]), abs(final["rpy"][1]))
        passed = (
            finite_samples == cycles
            and contacts_seen > 0
            and 0.30 <= final["height"] <= 0.60
            and tilt <= 0.30
            and (mode == "smoke" or span <= 0.05)
        )
        result = {
            "status": "PASS" if passed else "FAIL",
            "mode": mode,
            "steps": finite_samples,
            "height": final["height"],
            "height_span": span,
            "tilt": tilt,
            "contact_samples": contacts_seen,
            "obs_dim": contract.OBS_DIM,
            "action_dim": contract.ACTION_DIM,
        }
        result_path = contract.LOG_ROOT / f"standalone_{mode}_result.json"
        result_path.parent.mkdir(parents=True, exist_ok=True)
        result_path.write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(
            f"RL_{mode.upper()}_RESULT " + json.dumps(result, ensure_ascii=True),
            flush=True,
        )
        try:
            self.robot.simulationSetMode(self.robot.SIMULATION_MODE_PAUSE)
        except Exception:
            pass
        try:
            self.robot.simulationQuit(0 if passed else 1)
        except Exception:
            pass
        return 0 if passed else 1


def requested_mode() -> str:
    """解析 controllerArgs 或环境变量模式。"""
    arguments = [value.lower() for value in sys.argv[1:]]
    explicit = next(
        (
            value
            for value in arguments
            if value in {"auto", "train", "smoke", "settle", "play"}
        ),
        "",
    )
    environment = os.environ.get("RL_AGENT_MODE", "").strip().lower()
    if environment in {"train", "smoke", "settle", "play"}:
        return environment
    if explicit in {"train", "smoke", "settle", "play"}:
        return explicit
    if controller_option("bridge-port", "RL_BRIDGE_PORT") is not None:
        return "train"
    return "smoke"


def main() -> int:
    """Webots controller 入口。"""
    try:
        from controller import Supervisor
    except ImportError:
        print("RL_CONTROLLER_ERROR missing_webots_controller")
        return 2
    try:
        robot = Supervisor()
        controller = RlAgentController(robot)
        try:
            initialized = controller.initialize()
        except Exception as exc:
            controller._fail_initialization(
                "initialization_exception",
                f"{type(exc).__name__}: {exc}",
                exc,
            )
            initialized = False
        if not initialized:
            print(controller.initialization_failure_report(), flush=True)
            if controller.initialization_context:
                print(
                    "RL_CONTROLLER_ERROR_CONTEXT\n"
                    + controller.initialization_context,
                    flush=True,
                )
            for report in controller.capability_error_reports():
                print(report, flush=True)
            return 1
        for report in controller.capability_error_reports():
            print(report, flush=True)
        mode = requested_mode()
        if mode == "train":
            try:
                return controller.tcp_loop()
            except (ConnectionError, TimeoutError, OSError) as exc:
                print(connection_failure_report(exc), flush=True)
                traceback.print_exc()
                return 1
        if mode in {"smoke", "settle"}:
            return controller.run_standalone(mode)
        print(f"RL_CONTROLLER_ERROR unsupported_mode={mode}")
        return 2
    except Exception as exc:
        print(f"RL_CONTROLLER_ERROR {type(exc).__name__}: {exc}", flush=True)
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
