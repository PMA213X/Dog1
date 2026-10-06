#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""官方 Mini Cheetah 固定姿态时间线 Webots 控制器。

控制器只向 12 个电机写入经限幅、限速的 PD 力矩，并把状态打印到标准输出；
不读取或写入被动验收 artifacts。测试 world 的主 Robot 开启只读 Supervisor
能力，仅用于读取自身位姿、接触点和几何穿地状态，不绑定验收 Supervisor。
"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple


MODULE_DIR = os.path.dirname(os.path.abspath(__file__))
if MODULE_DIR not in sys.path:
    sys.path.insert(0, MODULE_DIR)

from posture_timeline_core import (  # noqa: E402
    DEFAULT_CROUCH,
    DURATION_SECONDS,
    FLAT_MOTOR_NAMES,
    FLAT_SENSOR_NAMES,
    MAX_TARGET_RATE,
    PosturePhase,
    PostureTimelineTracker,
    SUPPORT_HALF_LENGTH,
    SLIGHTLY_EXTENDED,
    TARGET_POSITION_LIMITS,
    foot_positions,
    knee_interior_angle,
    phase_at,
    support_half_width,
    support_projection_is_inside,
)
from joint_safety import (  # noqa: E402
    JOINT_COUNT,
    MAX_TORQUE,
    compute_pd_torques,
    validate_joint_feedback,
)


DEFAULT_TIMESTEP_MS = 4
STATUS_PERIOD_SECONDS = 0.5
PENETRATION_LIMIT = 0.010
# 前 3 秒只作为出生间隙与四足落地沉降，不计入默认姿态稳定跨度。
SETTLE_SECONDS = 3.0
STABILITY_HEIGHT_TOLERANCE = 0.030
STABILITY_TILT_TOLERANCE = 0.35
FINAL_TARGET_TOLERANCE = 0.25
RESULT_LOG_PATH = (
    Path(__file__).resolve().parents[3]
    / "logs/posture_timeline_20261003/posture_result.log"
)


def _finite(values: Sequence[float]) -> bool:
    """检查序列是否全部为有限数。"""

    try:
        return all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError):
        return False


def _rotation_x(angle: float) -> List[List[float]]:
    """构造绕 X 轴的右手旋转矩阵。"""

    cosine = math.cos(angle)
    sine = math.sin(angle)
    return [[1.0, 0.0, 0.0], [0.0, cosine, -sine], [0.0, sine, cosine]]


def _rotation_y(angle: float) -> List[List[float]]:
    """构造绕 Y 轴的右手旋转矩阵。"""

    cosine = math.cos(angle)
    sine = math.sin(angle)
    return [[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]]


def _matmul(left: List[List[float]], right: List[List[float]]) -> List[List[float]]:
    """计算两个 3x3 矩阵乘积。"""

    return [
        [
            sum(left[row][inner] * right[inner][column] for inner in range(3))
            for column in range(3)
        ]
        for row in range(3)
    ]


def _matvec(matrix: List[List[float]], vector: Sequence[float]) -> List[float]:
    """计算 3x3 矩阵与三维向量乘积。"""

    return [
        sum(matrix[row][column] * float(vector[column]) for column in range(3))
        for row in range(3)
    ]


def _world_point(
    origin: Sequence[float],
    rotation: List[List[float]],
    local_point: Sequence[float],
    root_position: Sequence[float],
    root_rotation: List[List[float]],
) -> List[float]:
    """把本体系点变换到世界坐标。"""

    rotated = _matvec(rotation, local_point)
    local_world = [float(origin[index]) + rotated[index] for index in range(3)]
    root_rotated = _matvec(root_rotation, local_world)
    return [
        float(root_position[index]) + root_rotated[index] for index in range(3)
    ]


def _box_min_z(
    center: Sequence[float],
    rotation: List[List[float]],
    half_size: Sequence[float],
    root_position: Sequence[float],
    root_rotation: List[List[float]],
) -> float:
    """计算任意姿态 Box 的世界最低 z。"""

    minimum = math.inf
    for sx in (-float(half_size[0]), float(half_size[0])):
        for sy in (-float(half_size[1]), float(half_size[1])):
            for sz in (-float(half_size[2]), float(half_size[2])):
                point = _world_point(
                    center,
                    rotation,
                    (sx, sy, sz),
                    root_position,
                    root_rotation,
                )
                minimum = min(minimum, point[2])
    return minimum


def lowest_geometry_z(
    targets: Sequence[float],
    root_position: Sequence[float],
    root_orientation: Sequence[float],
) -> Optional[float]:
    """按官方碰撞几何计算当前姿态的最低世界 z。"""

    if len(root_position) != 3 or len(root_orientation) != 9:
        return None
    if not _finite(tuple(root_position) + tuple(root_orientation)):
        return None
    root_rotation = [
        [float(root_orientation[row + column]) for column in range(3)]
        for row in range(0, 9, 3)
    ]
    minimum = _box_min_z(
        (0.0, 0.0, 0.0),
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        (0.19, 0.05, 0.03),
        root_position,
        root_rotation,
    )
    mounts = {
        "fr": (0.19, -0.049, 0.0),
        "fl": (0.19, 0.049, 0.0),
        "hr": (-0.19, -0.049, 0.0),
        "hl": (-0.19, 0.049, 0.0),
    }
    side = {"fr": -0.062, "fl": 0.062, "hr": -0.062, "hl": 0.062}
    values = list(targets)
    if len(values) != JOINT_COUNT or not _finite(values):
        return None

    for leg_index, leg in enumerate(("fr", "fl", "hr", "hl")):
        abad, hip, knee = values[leg_index * 3 : leg_index * 3 + 3]
        r_abad = _rotation_x(abad)
        # Webots 的 hip/knee 轴是 -Y，因此正角对应标准 Ry(-q)。
        r_hip = _matmul(r_abad, _rotation_y(-hip))
        r_knee = _matmul(r_hip, _rotation_y(-knee))
        mount = mounts[leg]
        hip_position = [
            mount[index] + _matvec(r_abad, (0.0, side[leg], 0.0))[index]
            for index in range(3)
        ]
        minimum = min(
            minimum,
            _box_min_z(
                hip_position,
                r_abad,
                (0.015, 0.062, 0.015),
                root_position,
                root_rotation,
            ),
        )
        thigh_center = [
            hip_position[index]
            + _matvec(r_hip, (0.0, 0.0, -0.1045))[index]
            for index in range(3)
        ]
        minimum = min(
            minimum,
            _box_min_z(
                thigh_center,
                r_hip,
                (0.02, 0.02, 0.1045),
                root_position,
                root_rotation,
            ),
        )
        knee_origin = [
            hip_position[index]
            + _matvec(r_hip, (0.0, 0.0, -0.209))[index]
            for index in range(3)
        ]
        shank_center = [
            knee_origin[index]
            + _matvec(r_knee, (0.0, 0.0, -0.09))[index]
            for index in range(3)
        ]
        minimum = min(
            minimum,
            _box_min_z(
                shank_center,
                r_knee,
                (0.015, 0.015, 0.09),
                root_position,
                root_rotation,
            ),
        )
        sphere_bottom = _world_point(
            shank_center,
            r_knee,
            (0.0, 0.0, -0.015),
            root_position,
            root_rotation,
        )
        minimum = min(minimum, sphere_bottom[2])
    return minimum


def _phase_name(phase: PosturePhase) -> str:
    """返回适合日志的阶段名称。"""

    return phase.value


class PostureTimelineController:
    """固定时间线姿态控制器与只读状态记录。"""

    def __init__(self, robot: Any, *, clock: Optional[Any] = None) -> None:
        self.robot = robot
        try:
            self.timestep = int(robot.getBasicTimeStep()) or DEFAULT_TIMESTEP_MS
        except Exception:
            self.timestep = DEFAULT_TIMESTEP_MS
        if self.timestep <= 0:
            self.timestep = DEFAULT_TIMESTEP_MS
        self.dt = self.timestep / 1000.0
        self._clock = clock or (lambda: float(self.robot.getTime()))
        self.tracker = PostureTimelineTracker()
        self.motors: List[Any] = []
        self.sensors: List[Any] = []
        self.missing_devices: List[str] = []
        self.device_fault = False
        self.feedback_fault = False
        self.robot_node: Any = None
        self._previous_positions: List[Optional[float]] = [None] * JOINT_COUNT
        self._last_status_at = -float("inf")
        self._last_phase: Optional[PosturePhase] = None
        self._samples = 0
        self._finite_samples = 0
        self._contact_samples = 0
        self._max_penetration = 0.0
        self._phase_heights: Dict[str, List[float]] = {
            "default_crouch": [],
            "slightly_extended": [],
            "return_to_default": [],
        }
        self._phase_tilts: Dict[str, List[float]] = {
            "default_crouch": [],
            "slightly_extended": [],
            "return_to_default": [],
        }
        self._final_targets: Tuple[float, ...] = DEFAULT_CROUCH
        self._final_feedback: Tuple[float, ...] = DEFAULT_CROUCH
        self._final_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._result_printed = False

    def initialize_devices(self) -> bool:
        """获取 12 电机、12 传感器；任何缺失都记录为失败。"""

        get_device = getattr(self.robot, "getDevice", None)
        if not callable(get_device):
            self.device_fault = True
            return False
        for name in FLAT_MOTOR_NAMES:
            try:
                motor = get_device(name)
            except Exception:
                motor = None
            self.motors.append(motor)
            if motor is None:
                self.missing_devices.append(name)
        for name in FLAT_SENSOR_NAMES:
            try:
                sensor = get_device(name)
            except Exception:
                sensor = None
            self.sensors.append(sensor)
            if sensor is None:
                self.missing_devices.append(name)
        for motor in self.motors:
            if motor is None:
                continue
            try:
                motor.setPosition(float("inf"))
            except Exception:
                self.device_fault = True
        for sensor in self.sensors:
            if sensor is None:
                continue
            try:
                sensor.enable(self.timestep)
            except Exception:
                self.device_fault = True
        if self.missing_devices:
            self.device_fault = True
        try:
            self.robot_node = self.robot.getSelf()
        except Exception:
            self.robot_node = None
        if self.robot_node is None:
            self.device_fault = True
        else:
            try:
                self.robot_node.enableContactPointsTracking(self.timestep, True)
            except Exception:
                self.device_fault = True
        return not self.device_fault

    def _read_feedback(self) -> Tuple[Tuple[float, ...], Tuple[float, ...], bool]:
        """读取 12 路关节反馈并做有限性检查。"""

        positions: List[float] = []
        velocities: List[float] = []
        valid = len(self.sensors) == JOINT_COUNT
        for index, sensor in enumerate(self.sensors):
            if sensor is None:
                positions.append(0.0)
                velocities.append(0.0)
                valid = False
                continue
            try:
                position = float(sensor.getValue())
            except Exception:
                positions.append(0.0)
                velocities.append(0.0)
                valid = False
                continue
            if not math.isfinite(position) or abs(position) > 20.0:
                positions.append(0.0)
                velocities.append(0.0)
                valid = False
                continue
            previous = self._previous_positions[index]
            velocity = 0.0 if previous is None else (position - previous) / self.dt
            if not math.isfinite(velocity) or abs(velocity) > 100.0:
                positions.append(0.0)
                velocities.append(0.0)
                valid = False
                continue
            positions.append(position)
            velocities.append(velocity)
            self._previous_positions[index] = position
        if not validate_joint_feedback(positions, velocities):
            valid = False
        self.feedback_fault = not valid
        return tuple(positions), tuple(velocities), valid

    def _apply_torques(self, torques: Sequence[float]) -> None:
        """把 12 路 PD 力矩限幅后写入电机。"""

        for index, motor in enumerate(self.motors):
            if motor is None:
                continue
            torque = max(-MAX_TORQUE, min(MAX_TORQUE, float(torques[index])))
            try:
                motor.setTorque(torque)
            except Exception:
                self.device_fault = True

    def _apply_position_targets(self, positions: Sequence[float]) -> None:
        """把限幅、限速后的关节位置交给 Webots 位置控制器。"""

        for motor, target in zip(self.motors, positions):
            if motor is None:
                continue
            try:
                motor.setPosition(float(target))
            except Exception:
                self.device_fault = True

    def _sample_status(self, now: float) -> None:
        """打印周期状态和阶段边界状态，不写文件。"""

        phase = phase_at(now)
        if phase == self._last_phase and now - self._last_status_at < STATUS_PERIOD_SECONDS:
            return
        self._last_status_at = now
        self._last_phase = phase
        if self.robot_node is None:
            return
        try:
            position = tuple(float(value) for value in self.robot_node.getPosition())
            orientation = tuple(float(value) for value in self.robot_node.getOrientation())
            velocity = tuple(float(value) for value in self.robot_node.getVelocity())
            contacts = self.robot_node.getContactPoints(True)
            contact_count = len(contacts)
        except Exception:
            position = ()
            orientation = ()
            velocity = ()
            contact_count = 0
        # 穿地必须按实际关节反馈计算；目标姿态只描述期望，不代表当前碰撞体。
        geometry_z = lowest_geometry_z(
            self._final_feedback,
            position,
            orientation,
        )
        finite = (
            _finite(position)
            and _finite(orientation)
            and _finite(velocity)
            and geometry_z is not None
            and math.isfinite(geometry_z)
        )
        if not finite or geometry_z is None:
            return
        tilt = math.acos(max(-1.0, min(1.0, orientation[4])))
        if now >= SETTLE_SECONDS and now < 10.0:
            self._phase_heights["default_crouch"].append(position[2])
            self._phase_tilts["default_crouch"].append(tilt)
        elif 12.0 <= now < 14.0:
            self._phase_heights["slightly_extended"].append(position[2])
            self._phase_tilts["slightly_extended"].append(tilt)
        elif now >= 14.0:
            self._phase_heights["return_to_default"].append(position[2])
            self._phase_tilts["return_to_default"].append(tilt)
        if contact_count:
            self._contact_samples += 1
        self._max_penetration = max(self._max_penetration, -float(geometry_z))
        self._samples += 1
        self._finite_samples += 1
        self._final_targets = tuple(self.tracker.current)
        self._final_position = tuple(position)  # type: ignore[assignment]
        payload: Dict[str, Any] = {
            "time_s": round(now, 4),
            "phase": _phase_name(phase),
            "position": list(position),
            "orientation": list(orientation),
            "velocity": list(velocity),
            "joint_targets": list(self.tracker.current),
            "joint_feedback": list(self._final_feedback),
            "geometry_min_z": geometry_z,
            "contact_count": contact_count,
            "finite": finite,
            "support_inside": support_projection_is_inside(self.tracker.current),
            "knee_interior_rad": knee_interior_angle(self.tracker.current),
        }
        print("POSTURE_STATUS " + json.dumps(payload, ensure_ascii=False), flush=True)

    def control_step(self) -> None:
        """执行一个固定时间线控制周期。"""

        now = float(self._clock())
        targets = self.tracker.sample(now, self.dt)
        positions, velocities, feedback_valid = self._read_feedback()
        if not feedback_valid:
            self.feedback_fault = True
            self._apply_torques((0.0,) * JOINT_COUNT)
            return
        self._final_feedback = tuple(positions)
        # 时间线测试使用 Webots 位置控制，确保 16 秒内能回到显式目标；
        # 反馈异常时仍立即切换到零力矩安全状态。
        self._apply_position_targets(targets.positions)
        self._sample_status(now)

    def _result(self, now: float) -> Dict[str, Any]:
        """构造最终机器可读结果。"""

        phase_heights = {
            name: values for name, values in self._phase_heights.items() if values
        }
        default_span = (
            max(phase_heights.get("default_crouch", [0.0]))
            - min(phase_heights.get("default_crouch", [0.0]))
        )
        extended_span = (
            max(phase_heights.get("slightly_extended", [0.0]))
            - min(phase_heights.get("slightly_extended", [0.0]))
        )
        final_error = max(
            abs(target - feedback)
            for target, feedback in zip(self._final_targets, self._final_feedback)
        )
        reasons: List[str] = []
        if now + 1e-9 < DURATION_SECONDS:
            reasons.append("未达到16仿真秒")
        if self.device_fault or self.missing_devices:
            reasons.append("设备缺失或读写失败")
        if self.feedback_fault or self._finite_samples != self._samples:
            reasons.append("存在非有限反馈")
        if self._samples == 0 or self._contact_samples == 0:
            reasons.append("没有有效状态或接触采样")
        if self._max_penetration > PENETRATION_LIMIT:
            reasons.append(f"穿地超过{PENETRATION_LIMIT:.3f}m")
        if not phase_heights.get("default_crouch"):
            reasons.append("默认阶段没有稳定采样")
        elif default_span > STABILITY_HEIGHT_TOLERANCE:
            reasons.append("默认阶段机身高度不稳定")
        if not phase_heights.get("slightly_extended"):
            reasons.append("伸展阶段没有稳定采样")
        elif extended_span > STABILITY_HEIGHT_TOLERANCE:
            reasons.append("伸展阶段机身高度不稳定")
        max_tilt = max(
            (max(values) for values in self._phase_tilts.values() if values),
            default=0.0,
        )
        if max_tilt > STABILITY_TILT_TOLERANCE:
            reasons.append("机身倾角超过稳定阈值")
        if final_error > FINAL_TARGET_TOLERANCE:
            reasons.append("返回阶段未恢复默认目标")
        if not phase_heights.get("return_to_default"):
            reasons.append("返回阶段没有稳定采样")
        default_reference = (
            sum(phase_heights.get("default_crouch", [0.0]))
            / len(phase_heights.get("default_crouch", [1.0]))
        )
        if abs(self._final_position[2] - default_reference) > STABILITY_HEIGHT_TOLERANCE:
            reasons.append("返回阶段未恢复默认机身高度")
        if not support_projection_is_inside(self._final_targets):
            reasons.append("足端投影超出支撑范围")
        if knee_interior_angle(self._final_targets) <= math.radians(15.0):
            reasons.append("默认姿态接近直腿锁死")
        return {
            "status": "PASS" if not reasons else "FAIL",
            "duration_target_s": DURATION_SECONDS,
            "duration_observed_s": now,
            "sample_count": self._samples,
            "finite_sample_count": self._finite_samples,
            "contact_sample_count": self._contact_samples,
            "max_geometry_penetration": self._max_penetration,
            "default_height_span": default_span,
            "extended_height_span": extended_span,
            "max_tilt_rad": max_tilt,
            "final_targets": list(self._final_targets),
            "final_feedback": list(self._final_feedback),
            "final_position": list(self._final_position),
            "final_target_error": final_error,
            "failure_reasons": reasons,
            "joint_count": JOINT_COUNT,
            "support_half_length": SUPPORT_HALF_LENGTH,
            "support_half_width": support_half_width(self._final_targets),
        }

    def finish(self, now: float) -> int:
        """打印最终结果并退出仿真。"""

        if self._result_printed:
            return 0
        self._result_printed = True
        result = self._result(now)
        # ASCII JSON 可避免 Webots 控制器在非 UTF-8 stdout 上丢失最终结果。
        result_line = "POSTURE_RESULT " + json.dumps(result, ensure_ascii=True)
        # 先写独立结果日志和 stderr，再尝试 stdout；Webots 在 simulationQuit
        # 前可能关闭或丢弃 stdout 缓冲。
        try:
            RESULT_LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
            descriptor = os.open(
                RESULT_LOG_PATH,
                os.O_WRONLY | os.O_CREAT | os.O_TRUNC,
                0o644,
            )
            try:
                os.write(descriptor, (result_line + "\n").encode("utf-8"))
            finally:
                os.close(descriptor)
        except OSError:
            pass
        try:
            os.write(2, (result_line + "\n").encode("utf-8"))
        except OSError:
            pass
        try:
            print(result_line, flush=True)
        except (BrokenPipeError, OSError):
            pass
        status = 0 if result["status"] == "PASS" else 1
        try:
            self.robot.simulationSetMode(self.robot.SIMULATION_MODE_PAUSE)
        except Exception:
            pass
        try:
            self.robot.simulationQuit(status)
        except Exception:
            pass
        return status

    def run(self) -> int:
        """执行固定 16 秒时间线并返回进程状态。"""

        if not self.initialize_devices():
            return self.finish(float(self._clock()))
        try:
            if self.robot.step(self.timestep) < 0:
                return self.finish(float(self._clock()))
        except Exception:
            return self.finish(float(self._clock()))
        while True:
            try:
                result = self.robot.step(self.timestep)
            except Exception:
                return self.finish(float(self._clock()))
            if result < 0:
                return self.finish(float(self._clock()))
            now = float(self._clock())
            self.control_step()
            if now + 1e-9 >= DURATION_SECONDS:
                return self.finish(now)


def main() -> int:
    """Webots 控制器入口。"""

    try:
        from controller import Supervisor  # type: ignore
    except ImportError:
        print("POSTURE_RESULT {\"status\":\"FAIL\",\"failure_reasons\":[\"缺少Webots controller包\"]}")
        return 2
    try:
        robot = Supervisor()
        return PostureTimelineController(robot).run()
    except Exception as exc:
        print(
            "POSTURE_RESULT "
            + json.dumps(
                {"status": "FAIL", "failure_reasons": [str(exc)]},
                ensure_ascii=True,
            ),
            flush=True,
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
