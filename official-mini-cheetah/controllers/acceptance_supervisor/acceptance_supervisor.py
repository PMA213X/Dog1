#!/usr/bin/env python3
"""只读记录官方 Mini Cheetah 的 10 秒平地被动验收。

本控制器只运行在独立 supervisor Robot 节点上，不获取或调用机器人电机接口，
因此不会向 Mini Cheetah 发送任何运动指令。
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Iterable

from controller import Supervisor


DURATION_SECONDS = 10.0
GEOMETRY_PENETRATION_LIMIT = 0.02
PENETRATION_WARNING_LIMIT = 0.01
PERSISTENT_PENETRATION_LIMIT_SECONDS = 0.05
FINAL_LINEAR_SPEED_LIMIT = 0.15
FINAL_ANGULAR_SPEED_LIMIT = 1.0
EXPECTED_JOINT_NAMES = tuple(
    f"{leg}_{joint}_motor"
    for leg in ("fr", "fl", "hr", "hl")
    for joint in ("abd", "hip", "kn")
)

ARTIFACT_ROOT = Path(__file__).resolve().parents[2] / "artifacts"
SAMPLE_LOG = ARTIFACT_ROOT / "acceptance_samples.jsonl"
SUMMARY_LOG = ARTIFACT_ROOT / "acceptance_summary.json"


def _finite(values: Iterable[float]) -> bool:
    """判断全部数值均为有限数，拒绝 NaN 和无穷大。"""

    return all(math.isfinite(float(value)) for value in values)


def _child_nodes(node: Any) -> list[Any]:
    """读取模型节点的子节点；只调用 Supervisor 只读字段接口。"""

    result: list[Any] = []
    for field_name in ("children", "device", "endPoint"):
        field = node.getField(field_name)
        if field is None:
            continue
        type_name = field.getTypeName()
        if type_name == "MFNode":
            for index in range(field.getCount()):
                child = field.getMFNode(index)
                if child is not None:
                    result.append(child)
        elif type_name == "SFNode":
            child = field.getSFNode()
            if child is not None:
                result.append(child)
    return result


def _walk_nodes(root: Any) -> list[Any]:
    """深度优先遍历 Robot 的模型树，用于发现 12 个 endPoint 关节。"""

    visited: set[int] = set()
    ordered: list[Any] = []

    def visit(node: Any) -> None:
        if node is None:
            return
        node_id = node.getId()
        if node_id in visited:
            return
        visited.add(node_id)
        ordered.append(node)
        for child in _child_nodes(node):
            visit(child)

    visit(root)
    return ordered


def _read_joint_records(robot_node: Any) -> list[dict[str, Any]]:
    """读取 12 个 HingeJoint 的动态 position 和设备名。"""

    records: list[dict[str, Any]] = []
    for node in _walk_nodes(robot_node):
        if node.getTypeName() != "HingeJoint":
            continue
        position_field = node.getField("position")
        position = (
            float(position_field.getSFFloat())
            if position_field is not None
            else None
        )
        device_field = node.getField("device")
        motor_name = None
        sensor_name = None
        if device_field is not None:
            for index in range(device_field.getCount()):
                device = device_field.getMFNode(index)
                if device is None:
                    continue
                name_field = device.getField("name")
                if name_field is None:
                    continue
                device_name = name_field.getSFString()
                if device.getTypeName() == "RotationalMotor":
                    motor_name = device_name
                elif device.getTypeName() == "PositionSensor":
                    sensor_name = device_name
        records.append(
            {
                "motor_name": motor_name,
                "sensor_name": sensor_name,
                "position": position,
            }
        )
    return records


def _joint_values(records: list[dict[str, Any]]) -> dict[str, float | None]:
    """按官方 motor 名称返回关节角字典。"""

    return {
        str(record["motor_name"]): record["position"]
        for record in records
        if record["motor_name"] is not None
    }


def _rotation_x(angle: float) -> list[list[float]]:
    """构造绕 X 轴的右手旋转矩阵。"""

    c = math.cos(angle)
    s = math.sin(angle)
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]


def _rotation_y(angle: float) -> list[list[float]]:
    """构造绕 Y 轴的右手旋转矩阵。"""

    c = math.cos(angle)
    s = math.sin(angle)
    return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]


def _matmul(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    """计算 3x3 矩阵乘积。"""

    return [
        [
            sum(left[row][k] * right[k][column] for k in range(3))
            for column in range(3)
        ]
        for row in range(3)
    ]


def _matvec(matrix: list[list[float]], vector: list[float]) -> list[float]:
    """计算 3x3 矩阵与三维向量乘积。"""

    return [sum(matrix[row][k] * vector[k] for k in range(3)) for row in range(3)]


def _world_point(
    origin: list[float],
    rotation: list[list[float]],
    local_point: list[float],
    root_position: list[float],
    root_rotation: list[list[float]],
) -> list[float]:
    """将 Robot 本体系点变换到世界坐标。"""

    rotated = _matvec(rotation, local_point)
    local_world = [origin[index] + rotated[index] for index in range(3)]
    root_rotated = _matvec(root_rotation, local_world)
    return [
        root_position[index] + root_rotated[index] for index in range(3)
    ]


def _box_min_z(
    center: list[float],
    rotation: list[list[float]],
    half_size: list[float],
    root_position: list[float],
    root_rotation: list[list[float]],
) -> float:
    """计算任意姿态 Box 的世界最低 z。"""

    minimum = math.inf
    for sx in (-half_size[0], half_size[0]):
        for sy in (-half_size[1], half_size[1]):
            for sz in (-half_size[2], half_size[2]):
                point = _world_point(
                    center,
                    rotation,
                    [sx, sy, sz],
                    root_position,
                    root_rotation,
                )
                minimum = min(minimum, point[2])
    return minimum


def _lowest_geometry_z(
    joint_values: dict[str, float | None],
    root_position: list[float],
    root_orientation: list[float],
) -> float | None:
    """按官方 World 显式几何计算当前姿态的最低 z。"""

    if len(joint_values) != 12 or not _finite(root_position + root_orientation):
        return None
    root_rotation = [list(root_orientation[row : row + 3]) for row in range(0, 9, 3)]
    minimum = _box_min_z(
        [0.0, 0.0, 0.0],
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        [0.19, 0.05, 0.03],
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

    for leg, mount in mounts.items():
        q_abad = joint_values.get(f"{leg}_abd_motor")
        q_hip = joint_values.get(f"{leg}_hip_motor")
        q_knee = joint_values.get(f"{leg}_kn_motor")
        if q_abad is None or q_hip is None or q_knee is None:
            return None
        r_abad = _rotation_x(float(q_abad))
        r_hip = _rotation_y(float(q_hip))
        r_knee = _rotation_y(float(q_knee))
        abad_rotation = r_abad
        hip_rotation = _matmul(r_abad, r_hip)
        knee_rotation = _matmul(hip_rotation, r_knee)

        hip_local = [0.0, side[leg], 0.0]
        hip_world_local = [
            mount[index] + _matvec(abad_rotation, hip_local)[index]
            for index in range(3)
        ]
        abad_center = hip_world_local
        minimum = min(
            minimum,
            _box_min_z(
                abad_center,
                abad_rotation,
                [0.015, 0.062, 0.015],
                root_position,
                root_rotation,
            ),
        )

        hip_thigh_offset = _matvec(hip_rotation, [0.0, 0.0, -0.1045])
        thigh_center = [
            hip_world_local[index] + hip_thigh_offset[index] for index in range(3)
        ]
        minimum = min(
            minimum,
            _box_min_z(
                thigh_center,
                hip_rotation,
                [0.02, 0.02, 0.1045],
                root_position,
                root_rotation,
            ),
        )

        knee_offset = _matvec(hip_rotation, [0.0, 0.0, -0.209])
        knee_origin = [
            hip_world_local[index] + knee_offset[index] for index in range(3)
        ]
        shank_offset = _matvec(knee_rotation, [0.0, 0.0, -0.09])
        shank_center = [
            knee_origin[index] + shank_offset[index] for index in range(3)
        ]
        minimum = min(
            minimum,
            _box_min_z(
                shank_center,
                knee_rotation,
                [0.015, 0.015, 0.09],
                root_position,
                root_rotation,
            ),
        )
        sphere_bottom = _world_point(
            shank_center,
            knee_rotation,
            [0.0, 0.0, -0.015],
            root_position,
            root_rotation,
        )
        minimum = min(minimum, sphere_bottom[2])
    return minimum


def _contact_sample(robot_node: Any) -> tuple[int, float | None]:
    """读取 Robot 及后代接触点数量和最低接触 z。"""

    points = robot_node.getContactPoints(True)
    if not points:
        return 0, None
    values = [float(value) for point in points for value in point.getPoint()]
    minimum = min(float(point.getPoint()[2]) for point in points)
    if not _finite(values):
        return len(points), math.nan
    return len(points), minimum


def _flatten_joint_values(values: dict[str, float | None]) -> list[float | None]:
    """按官方 12 关节顺序展开关节角。"""

    return [values.get(name) for name in EXPECTED_JOINT_NAMES]


def _write_summary(summary: dict[str, Any]) -> None:
    """写出机器可读 JSON 摘要并输出明确运行时标记。"""

    ARTIFACT_ROOT.mkdir(parents=True, exist_ok=True)
    SUMMARY_LOG.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    compact = json.dumps(summary, ensure_ascii=False, sort_keys=True)
    print(f"ACCEPTANCE_RESULT {compact}", flush=True)


def _finish(supervisor: Supervisor, summary: dict[str, Any]) -> None:
    """结束记录后暂停并退出仿真，失败返回非零退出状态。"""

    _write_summary(summary)
    status = 0 if summary["status"] == "PASS" else 1
    try:
        supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
    except Exception as error:  # pragma: no cover - 依赖 Webots 运行时
        print(f"WARN: 暂停仿真失败，改为直接结束：{error}", flush=True)
    supervisor.simulationQuit(status)


def run() -> int:
    """执行 10 秒只读验收并返回进程状态。"""

    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())
    ARTIFACT_ROOT.mkdir(parents=True, exist_ok=True)
    robot_node = supervisor.getFromDef("MINI_CHEETAH")
    floor_node = supervisor.getFromDef("FLAT_FLOOR")
    failure_reasons: list[str] = []

    if robot_node is None:
        failure_reasons.append("MINI_CHEETAH 节点缺失")
    if floor_node is None:
        failure_reasons.append("FLAT_FLOOR 节点缺失")
    if failure_reasons:
        _finish(
            supervisor,
            {
                "status": "FAIL",
                "failure_reasons": failure_reasons,
                "sample_count": 0,
            },
        )
        return 1

    initial_records = _read_joint_records(robot_node)
    if len(initial_records) != 12:
        failure_reasons.append(
            f"旧 child 语法下只发现 {len(initial_records)} 个 HingeJoint；停止"
        )
        _finish(
            supervisor,
            {
                "status": "FAIL",
                "failure_reasons": failure_reasons,
                "joint_count": len(initial_records),
                "sample_count": 0,
            },
        )
        return 1
    if {record["motor_name"] for record in initial_records} != set(
        EXPECTED_JOINT_NAMES
    ):
        failure_reasons.append("12 个 motor 名称与官方模型不一致；停止")
        _finish(
            supervisor,
            {
                "status": "FAIL",
                "failure_reasons": failure_reasons,
                "joint_count": len(initial_records),
                "sample_count": 0,
            },
        )
        return 1

    robot_node.enableContactPointsTracking(timestep, True)
    nan_count = 0
    samples = 0
    contact_samples = 0
    min_initial_geometry_z = math.inf
    min_geometry_z = math.inf
    min_contact_z = math.inf
    max_penetration = 0.0
    penetration_over_limit_samples = 0
    max_persistent_penetration_seconds = 0.0
    penetration_over_limit_started_at: float | None = None
    final_sample: dict[str, Any] = {}
    reached_duration = False
    ended_early = False

    SAMPLE_LOG.write_text("", encoding="utf-8")
    with SAMPLE_LOG.open("a", encoding="utf-8") as log_file:
        while True:
            records = _read_joint_records(robot_node)
            values = _joint_values(records)
            flat_values = _flatten_joint_values(values)
            position = robot_node.getPosition()
            orientation = robot_node.getOrientation()
            velocity = robot_node.getVelocity()
            contact_count, contact_min_z = _contact_sample(robot_node)
            geometry_z = _lowest_geometry_z(values, position, orientation)
            sample_time = float(supervisor.getTime())
            all_values = list(velocity) + list(position) + list(orientation)
            all_values.extend(float(v) for v in flat_values if v is not None)
            if contact_min_z is not None:
                all_values.append(contact_min_z)
            if geometry_z is not None:
                all_values.append(geometry_z)
            finite = _finite(all_values)
            if not finite or len(records) != 12 or geometry_z is None:
                nan_count += 1

            if samples == 0 and geometry_z is not None:
                min_initial_geometry_z = geometry_z
            if geometry_z is not None:
                min_geometry_z = min(min_geometry_z, geometry_z)
                max_penetration = max(max_penetration, -geometry_z)
                if geometry_z < -PENETRATION_WARNING_LIMIT:
                    penetration_over_limit_samples += 1
                    if penetration_over_limit_started_at is None:
                        penetration_over_limit_started_at = sample_time
                    duration = (
                        sample_time
                        - penetration_over_limit_started_at
                        + timestep / 1000.0
                    )
                    max_persistent_penetration_seconds = max(
                        max_persistent_penetration_seconds,
                        duration,
                    )
                elif penetration_over_limit_started_at is not None:
                    duration = (
                        sample_time
                        - penetration_over_limit_started_at
                        + timestep / 1000.0
                    )
                    max_persistent_penetration_seconds = max(
                        max_persistent_penetration_seconds,
                        duration,
                    )
                    penetration_over_limit_started_at = None
            if contact_count > 0:
                contact_samples += 1
                if contact_min_z is not None and math.isfinite(contact_min_z):
                    min_contact_z = min(min_contact_z, contact_min_z)

            sample = {
                "time_s": sample_time,
                "position": position,
                "orientation": orientation,
                "velocity": velocity,
                "joint_values": flat_values,
                "joint_names": list(EXPECTED_JOINT_NAMES),
                "geometry_min_z": geometry_z,
                "contact_count": contact_count,
                "contact_min_z": contact_min_z,
                "finite": finite,
            }
            log_file.write(json.dumps(sample, ensure_ascii=False) + "\n")
            log_file.flush()
            final_sample = sample
            samples += 1

            if sample_time + 1e-9 >= DURATION_SECONDS:
                reached_duration = True
                break
            if supervisor.step(timestep) == -1:
                ended_early = True
                break

    if penetration_over_limit_started_at is not None:
        duration = (
            float(final_sample.get("time_s", 0.0))
            - penetration_over_limit_started_at
            + timestep / 1000.0
        )
        max_persistent_penetration_seconds = max(
            max_persistent_penetration_seconds,
            duration,
        )

    final_velocity = final_sample.get("velocity", [math.nan] * 6)
    final_linear_speed = math.sqrt(
        float(final_velocity[0]) ** 2
        + float(final_velocity[1]) ** 2
        + float(final_velocity[2]) ** 2
    )
    final_angular_speed = math.sqrt(
        float(final_velocity[3]) ** 2
        + float(final_velocity[4]) ** 2
        + float(final_velocity[5]) ** 2
    )

    if ended_early:
        failure_reasons.append("仿真在 10 秒前结束")
    if not reached_duration:
        failure_reasons.append("未达到 10 仿真秒")
    if nan_count:
        failure_reasons.append(f"存在 {nan_count} 个 NaN/异常样本")
    if contact_samples == 0:
        failure_reasons.append("10 秒内没有检测到机器人接触")
    if min_initial_geometry_z < -1e-6:
        failure_reasons.append(
            f"初始最低几何穿地：{min_initial_geometry_z:.9f} m"
        )
    if max_penetration > GEOMETRY_PENETRATION_LIMIT:
        failure_reasons.append(
            f"最低几何穿地超过 {GEOMETRY_PENETRATION_LIMIT:.3f} m："
            f"{max_penetration:.9f} m"
        )
    if max_persistent_penetration_seconds > PERSISTENT_PENETRATION_LIMIT_SECONDS:
        failure_reasons.append(
            f"超过 10 mm 的穿地持续 "
            f"{max_persistent_penetration_seconds:.3f} s"
        )
    if final_linear_speed > FINAL_LINEAR_SPEED_LIMIT:
        failure_reasons.append(
            f"末态线速度未稳定：{final_linear_speed:.6f} m/s"
        )
    if final_angular_speed > FINAL_ANGULAR_SPEED_LIMIT:
        failure_reasons.append(
            f"末态角速度未稳定：{final_angular_speed:.6f} rad/s"
        )

    status = "PASS" if not failure_reasons else "FAIL"
    summary = {
        "status": status,
        "duration_target_s": DURATION_SECONDS,
        "duration_observed_s": float(supervisor.getTime()),
        "sample_count": samples,
        "joint_count": len(final_sample.get("joint_values", [])),
        "nan_anomaly_samples": nan_count,
        "contact_samples": contact_samples,
        "initial_geometry_min_z": min_initial_geometry_z,
        "geometry_min_z": min_geometry_z,
        "max_geometry_penetration": max_penetration,
        "penetration_over_10mm_samples": penetration_over_limit_samples,
        "max_persistent_penetration_seconds": max_persistent_penetration_seconds,
        "contact_min_z": min_contact_z,
        "final_position": final_sample.get("position"),
        "final_velocity": final_sample.get("velocity"),
        "final_linear_speed": final_linear_speed,
        "final_angular_speed": final_angular_speed,
        "final_joint_values": final_sample.get("joint_values"),
        "failure_reasons": failure_reasons,
        "sample_log": str(SAMPLE_LOG),
        "summary_log": str(SUMMARY_LOG),
        "robot_controller_commands": 0,
    }
    _finish(supervisor, summary)
    return 0 if status == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(run())
