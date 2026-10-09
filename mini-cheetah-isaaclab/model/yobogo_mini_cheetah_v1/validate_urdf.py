#!/usr/bin/env python3
"""对 YoboGo Mini Cheetah URDF 做可复现的静态验收。"""

from __future__ import annotations

import argparse
import hashlib
import math
import re
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np

DEFAULT_URDF = Path(__file__).with_name("yobogo_mini_cheetah.urdf")
MASS_SCALE = 9.0 / 8.292
FOOT_COLLISION_RADIUS_M = 0.0202
FOOT_MASS_KG = 0.01 * MASS_SCALE
FOOT_DERIVED_INERTIA_KG_M2 = (
    0.4 * FOOT_MASS_KG * FOOT_COLLISION_RADIUS_M**2
)

EXPECTED_JOINTS = [
    f"leg{slot}_{joint}_joint"
    for slot in range(4)
    for joint in ("abad", "hip", "knee")
]
EXPECTED_FIXED_JOINTS = [
    f"leg{slot}_shank_to_foot_joint" for slot in range(4)
]
EXPECTED_LINKS = (
    ["base_link"]
    + [
        f"leg{slot}_{kind}_link"
        for slot in range(4)
        for kind in ("hip", "thigh", "shank", "foot")
    ]
)
EXPECTED_MASS = {
    "base": 3.3,
    "hip": 0.54,
    "thigh": 0.634,
    "shank": 0.064,
    "foot": 0.01,
}
EXPECTED_INERTIA = {
    "base": (0.011253, 0.0, 0.0, 0.362030, 0.0, 0.042673),
    "hip": (0.000381, 0.000058, 0.00000045, 0.000560, 0.00000095, 0.000444),
    "thigh": (0.001983, 0.000245, 0.000013, 0.002103, 0.0000015, 0.000408),
    "shank": (0.000245, 0.0, 0.0, 0.000248, 0.0, 0.000006),
    # MIT 源 foot 惯量全零；验收期望使用实心球派生正定值。
    "foot": (
        FOOT_DERIVED_INERTIA_KG_M2,
        0.0,
        0.0,
        FOOT_DERIVED_INERTIA_KG_M2,
        0.0,
        FOOT_DERIVED_INERTIA_KG_M2,
    ),
}
EXPECTED_MESH_SHA256 = {
    "mini_abad.dae": "6a34448654e342b6b71e71a1ca263e327fba8f4f60fd3e91aab0f8afed03683c",
    "mini_body.dae": "ba877c099da9a76f973f4c955260e8fc95224ea45b12fbbc1eca0ab2dfd55343",
    "mini_lower_link.dae": "e18ac026e8febee7e1d933235ff1a147268fa9009c8a46d2cc92012c53c50c91",
    "mini_upper_link.dae": "88a55f7c1de47945b0f51e990ace895e5fb3e46b51b0e5fade6d683249a8958e",
}
EXPECTED_TARGET = [
    -0.6, -1.0, 2.7,
    0.6, -1.0, 2.7,
    -0.6, -1.0, 2.7,
    0.6, -1.0, 2.7,
]
EXPECTED_SIM_INITIAL = [
    0.0, -0.785398163, 1.865468294,
    0.0, -0.785398163, 1.865468294,
    0.0, -0.785398163, 1.865468294,
    0.0, -0.785398163, 1.865468294,
]
EXPECTED_SIM_ROOT_Z_M = 0.26
EXPECTED_FOOT_COLLISION_MIN_M = -0.26
EXPECTED_BASE_COLLISION_MIN_M = -0.04
FOOT_COLLISION_OFFSET_XYZ = (0.0, 0.0, 0.024)
EXPECTED_KINEMATICS = {
    0: {
        "haa": ((0.14775, -0.049, 0.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        "hip": ((0.055, -0.019, 0.0), (0.0, 0.0, 0.0)),
        "knee": (0.0, -0.049, -0.2085),
    },
    1: {
        "haa": ((0.14775, 0.049, 0.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        "hip": ((0.055, 0.019, 0.0), (0.0, 0.0, 0.0)),
        "knee": (0.0, 0.049, -0.2085),
    },
    2: {
        "haa": ((-0.14775, -0.049, 0.0), (0.0, math.pi, 0.0), (-1.0, 0.0, 0.0)),
        "hip": ((0.055, -0.019, 0.0), (0.0, math.pi, 0.0)),
        "knee": (0.0, -0.049, -0.2085),
    },
    3: {
        "haa": ((-0.14775, 0.049, 0.0), (0.0, math.pi, 0.0), (-1.0, 0.0, 0.0)),
        "hip": ((0.055, 0.019, 0.0), (0.0, math.pi, 0.0)),
        "knee": (0.0, 0.049, -0.2085),
    },
}


def close_sequence(
    actual: list[float] | tuple[float, ...],
    expected: list[float] | tuple[float, ...],
    tolerance: float = 1e-9,
) -> bool:
    return len(actual) == len(expected) and all(
        math.isclose(left, right, rel_tol=0.0, abs_tol=tolerance)
        for left, right in zip(actual, expected)
    )


def vector(element: ET.Element | None) -> tuple[float, ...]:
    if element is None or "xyz" not in element.attrib:
        return (0.0, 0.0, 0.0)
    return tuple(float(value) for value in element.get("xyz", "").split())


def inertia_tuple(element: ET.Element) -> tuple[float, ...]:
    names = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
    return tuple(float(element.get(name, "nan")) for name in names)


def rotation_from_rpy(rpy: tuple[float, ...]) -> np.ndarray:
    """按 URDF 固定轴 RPY 顺序构造旋转矩阵 Rz(yaw)Ry(pitch)Rx(roll)。"""
    roll, pitch, yaw = rpy
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def rotation_about_axis(axis: tuple[float, ...], angle: float) -> np.ndarray:
    """按轴角公式构造关节旋转矩阵。"""
    axis_array = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(axis_array))
    if norm == 0.0:
        raise ValueError("URDF 关节 axis 不能为零向量")
    x, y, z = axis_array / norm
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return (
        np.eye(3)
        + math.sin(angle) * skew
        + (1.0 - math.cos(angle)) * (skew @ skew)
    )


def homogeneous(
    rotation: np.ndarray, translation: tuple[float, ...]
) -> np.ndarray:
    """构造 4x4 齐次变换。"""
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return matrix


def link_transforms(
    root: ET.Element, joint_positions: dict[str, float]
) -> dict[str, np.ndarray]:
    """按 URDF 拓扑计算 base_link 到各 link 的几何变换。"""
    transforms = {"base_link": np.eye(4, dtype=float)}
    for joint in root.findall("joint"):
        parent_name = joint.find("parent").get("link")
        child_name = joint.find("child").get("link")
        if parent_name not in transforms:
            raise ValueError(f"关节 {joint.get('name')} 的父 link 尚未计算")
        origin = joint.find("origin")
        xyz = vector(origin)
        rpy = tuple(float(value) for value in origin.get("rpy", "0 0 0").split())
        origin_transform = homogeneous(rotation_from_rpy(rpy), xyz)
        if joint.get("type") == "revolute":
            axis = vector(joint.find("axis"))
            angle = joint_positions[joint.get("name", "")]
            joint_transform = homogeneous(rotation_about_axis(axis, angle), (0, 0, 0))
        else:
            joint_transform = np.eye(4, dtype=float)
        transforms[child_name] = transforms[parent_name] @ origin_transform @ joint_transform
    return transforms


def flat_joint_positions(flat: list[float]) -> dict[str, float]:
    """把 12 元 flat 向量映射到固定关节名。"""
    if len(flat) != len(EXPECTED_JOINTS):
        return {}
    return dict(zip(EXPECTED_JOINTS, flat))


def foot_collision_min(
    root: ET.Element, joint_positions: dict[str, float]
) -> float:
    """计算四足球碰撞最低点相对 base_link 的 z 值。"""
    transforms = link_transforms(root, joint_positions)
    minima = []
    for slot in range(4):
        link = next(
            item
            for item in root.findall("link")
            if item.get("name") == f"leg{slot}_foot_link"
        )
        collision = link.find("collision")
        origin = collision.find("origin")
        offset = vector(origin)
        radius = float(collision.find("geometry").find("sphere").get("radius"))
        center_w = transforms[f"leg{slot}_foot_link"] @ np.append(offset, 1.0)
        minima.append(float(center_w[2] - radius))
    return min(minima)


def base_collision_min(
    root: ET.Element, joint_positions: dict[str, float]
) -> float:
    """计算 base box 八个角点的最低 z，相对 base_link 原点。"""
    transforms = link_transforms(root, joint_positions)
    link = next(item for item in root.findall("link") if item.get("name") == "base_link")
    collision = link.find("collision")
    origin = collision.find("origin")
    offset = np.asarray(vector(origin), dtype=float)
    size = np.asarray(
        [float(value) for value in collision.find("geometry").find("box").get("size").split()],
        dtype=float,
    )
    half = size / 2.0
    corners = np.array(
        [
            [sx * half[0], sy * half[1], sz * half[2]]
            for sx in (-1.0, 1.0)
            for sy in (-1.0, 1.0)
            for sz in (-1.0, 1.0)
        ]
    )
    local_corners = corners + offset
    homogeneous_corners = np.column_stack(
        (local_corners, np.ones(len(local_corners), dtype=float))
    )
    world_corners = (transforms["base_link"] @ homogeneous_corners.T).T
    return float(np.min(world_corners[:, 2]))


def link_kind(link_name: str) -> str:
    if link_name == "base_link":
        return "base"
    for kind in ("hip", "thigh", "shank", "foot"):
        if link_name.endswith(f"_{kind}_link"):
            return kind
    raise ValueError(f"未知 link 类型：{link_name}")


def metadata_from_comments(root: ET.Element) -> str:
    return "\n".join(
        child.text or ""
        for child in root
        if child.tag is ET.Comment
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("urdf", nargs="?", type=Path, default=DEFAULT_URDF)
    parser.add_argument(
        "--strict",
        "--strict-inertia",
        dest="strict_inertia",
        action="store_true",
        help="执行严格验收（兼容旧 --strict-inertia 名称）",
    )
    args = parser.parse_args()

    urdf_path = args.urdf.resolve()
    passed: list[str] = []
    warnings: list[str] = []
    errors: list[str] = []

    def check(name: str, condition: bool, detail: str) -> None:
        if condition:
            passed.append(f"{name}: {detail}")
        else:
            errors.append(f"{name}: {detail}")

    try:
        parser_target = ET.TreeBuilder(insert_comments=True)
        tree = ET.parse(urdf_path, ET.XMLParser(target=parser_target))
    except (OSError, ET.ParseError) as exc:
        print(f"[FAIL] XML 解析失败: {exc}")
        return 2
    root = tree.getroot()
    check("XML", root.tag == "robot", "XML 解析成功且根元素为 robot")
    check(
        "机器人名",
        root.get("name") == "yobogo_mini_cheetah_v1",
        "robot name=yobogo_mini_cheetah_v1",
    )

    metadata = metadata_from_comments(root)
    links = root.findall("link")
    joints = root.findall("joint")
    revolute_joints = [joint for joint in joints if joint.get("type") == "revolute"]
    fixed_joints = [joint for joint in joints if joint.get("type") == "fixed"]
    link_names = [link.get("name", "") for link in links]
    revolute_names = [joint.get("name", "") for joint in revolute_joints]
    fixed_names = [joint.get("name", "") for joint in fixed_joints]

    check(
        "结构计数",
        len(links) == 17 and len(joints) == 16,
        f"17 links / 16 joints（12 驱动 + 4 固定），实际 {len(links)}/{len(joints)}",
    )
    check(
        "驱动关节命名与顺序",
        revolute_names == EXPECTED_JOINTS,
        "leg0 FR → leg1 FL → leg2 RR → leg3 RL，每腿 abad/hip/knee",
    )
    check(
        "足端固定关节",
        fixed_names == EXPECTED_FIXED_JOINTS,
        "4 个 shank_to_foot fixed joint",
    )
    check(
        "link 命名",
        link_names == EXPECTED_LINKS,
        "base_link 与 legN_hip/thigh/shank/foot_link 顺序正确",
    )
    check(
        "链路拓扑",
        len({joint.find("child").get("link") for joint in joints}) == 16
        and {joint.find("parent").get("link") for joint in fixed_joints}
        == {f"leg{slot}_shank_link" for slot in range(4)},
        "每条腿为 base → hip → thigh → shank → foot，子 link 唯一",
    )

    expected_total_mass = 9.0
    actual_total_mass = 0.0
    inertia_errors = []
    foot_pd_count = 0
    for link in links:
        name = link.get("name", "")
        kind = link_kind(name)
        inertial = link.find("inertial")
        if inertial is None or inertial.find("mass") is None or inertial.find("inertia") is None:
            errors.append(f"惯量: {name} 缺少 inertial/mass/inertia")
            continue
        mass = float(inertial.find("mass").get("value"))
        actual_total_mass += mass
        expected_mass = EXPECTED_MASS[kind] * MASS_SCALE
        if not math.isclose(mass, expected_mass, rel_tol=0.0, abs_tol=2e-11):
            inertia_errors.append(f"{name} mass={mass} expected={expected_mass}")

        tensor = inertia_tuple(inertial.find("inertia"))
        if kind == "foot":
            expected_tensor = EXPECTED_INERTIA[kind]
        else:
            expected_tensor = tuple(
                value * MASS_SCALE for value in EXPECTED_INERTIA[kind]
            )
        # 生成器按 12 位有效数字落盘，允许 1e-12 以内的序列化舍入误差。
        if not close_sequence(tensor, expected_tensor, tolerance=1e-12):
            inertia_errors.append(f"{name} inertia={tensor} expected={expected_tensor}")
        matrix = np.array(
            [
                [tensor[0], tensor[1], tensor[2]],
                [tensor[1], tensor[3], tensor[4]],
                [tensor[2], tensor[4], tensor[5]],
            ],
            dtype=float,
        )
        eigenvalues = np.linalg.eigvalsh(matrix)
        if kind == "foot":
            foot_pd_count += int(float(np.min(eigenvalues)) > 0.0)
        elif float(np.min(eigenvalues)) <= 0.0:
            inertia_errors.append(
                f"{name} 非正定，特征值={eigenvalues.tolist()}"
            )
    foot_metadata_ok = all(
        token in metadata
        for token in (
            "foot_inertia_derivation=solid_sphere_i=2/5*m*r^2",
            f"foot_inertia_mass_kg={format(FOOT_MASS_KG, '.12g')}",
            f"foot_inertia_radius_m={format(FOOT_COLLISION_RADIUS_M, '.12g')}",
            f"foot_inertia_diagonal_kg_m2="
            f"{format(FOOT_DERIVED_INERTIA_KG_M2, '.12g')}",
            "not MIT original inertia; not a guessed epsilon",
        )
    )
    if foot_pd_count != 4:
        inertia_errors.append(
            f"期望 4 个 foot 惯量严格正定，实际 {foot_pd_count}"
        )
    if not foot_metadata_ok:
        inertia_errors.append("foot 派生惯量 metadata 缺失或数值不匹配")
    check(
        "逐 link 质量与惯量正定性",
        not inertia_errors,
        "17 个 link 质量缩放正确；foot 使用可信质量和 MIT 球半径按 "
        "2/5*m*r^2 派生，4 个 foot 严格正定"
        if not inertia_errors
        else "; ".join(inertia_errors),
    )
    check(
        "总质量",
        math.isclose(actual_total_mass, expected_total_mass, rel_tol=0.0, abs_tol=2e-11),
        f"合计 {actual_total_mass:.12f} kg（目标 9 kg，差 {actual_total_mass - expected_total_mass:+.3e}）",
    )
    for slot in range(4):
        expected = EXPECTED_KINEMATICS[slot]
        abad = next(j for j in revolute_joints if j.get("name") == f"leg{slot}_abad_joint")
        hip = next(j for j in revolute_joints if j.get("name") == f"leg{slot}_hip_joint")
        knee = next(j for j in revolute_joints if j.get("name") == f"leg{slot}_knee_joint")
        abad_origin = abad.find("origin")
        hip_origin = hip.find("origin")
        knee_origin = knee.find("origin")
        abad_rpy = abad_origin.get("rpy", "0 0 0")
        hip_rpy = hip_origin.get("rpy", "0 0 0")
        expected_haa_xyz, expected_haa_rpy, expected_haa_axis = expected["haa"]
        expected_hip_xyz, expected_hip_rpy = expected["hip"]
        check(
            f"leg{slot} 运动学",
            close_sequence(vector(abad_origin), expected_haa_xyz)
            and close_sequence(
                tuple(float(v) for v in abad_rpy.split()),
                expected_haa_rpy,
                tolerance=1e-9,
            )
            and close_sequence(
                vector(abad.find("axis")),
                expected_haa_axis,
            )
            and close_sequence(vector(hip_origin), expected_hip_xyz)
            and close_sequence(
                tuple(float(v) for v in hip_rpy.split()),
                expected_hip_rpy,
                tolerance=1e-9,
            )
            and close_sequence(vector(knee_origin), expected["knee"])
            and close_sequence(vector(knee.find("axis")), (0.0, -1.0, 0.0)),
            f"ORCAgym 固定提交 HAA/HFE/KFE 原点、轴与后腿父坐标旋转",
        )

    expected_limits = {
        "abad": (-1.5, 1.5, 17.0, 41.0),
        "hip": (-5.0, 5.0, 17.0, 41.0),
        "knee": (-2.7, 2.7, 26.0, 26.8),
    }
    for joint in revolute_joints:
        kind = joint.get("name", "").split("_", 2)[1]
        limit = joint.find("limit")
        actual = (
            float(limit.get("lower")),
            float(limit.get("upper")),
            float(limit.get("effort")),
            float(limit.get("velocity")),
        )
        check(
            f"{joint.get('name')} 限位/力矩/速度",
            close_sequence(actual, expected_limits[kind]),
            "软件候选限位、项目力矩 17/17/26、MIT 速度参考",
        )

    mesh_nodes = root.findall(".//mesh")
    mesh_filenames = [node.get("filename", "") for node in mesh_nodes]
    mesh_set = set(mesh_filenames)
    check(
        "visual 网格引用",
        len(mesh_nodes) == 13
        and mesh_set
        == {
            "../visual_dae/mini_body.dae",
            "../visual_dae/mini_abad.dae",
            "../visual_dae/mini_upper_link.dae",
            "../visual_dae/mini_lower_link.dae",
        },
        f"13 个 mesh 实例、4 个唯一 DAE，实际实例 {len(mesh_nodes)}、唯一 {len(mesh_set)}",
    )
    mesh_results = []
    for filename in sorted({Path(path).name for path in mesh_filenames}):
        mesh_path = (urdf_path.parent / ".." / "visual_dae" / filename).resolve()
        if not mesh_path.is_file():
            mesh_results.append(f"{filename}: 不存在")
            continue
        digest = hashlib.sha256(mesh_path.read_bytes()).hexdigest()
        if digest != EXPECTED_MESH_SHA256.get(filename):
            mesh_results.append(f"{filename}: SHA256 不匹配")
    check(
        "网格路径与 SHA256",
        not mesh_results,
        "4 个 DAE 均可从 ../visual_dae 解析且 SHA256 匹配"
        if not mesh_results
        else "; ".join(mesh_results),
    )

    collisions = root.findall(".//collision")
    collision_shapes = [
        child.tag
        for collision in collisions
        for child in collision.find("geometry")
    ]
    check(
        "碰撞候选",
        len(collisions) == 17
        and collision_shapes.count("box") == 1
        and collision_shapes.count("cylinder") == 12
        and collision_shapes.count("sphere") == 4,
        "MIT 候选：base box、4 hip/thigh/shank cylinders、4 foot spheres",
    )

    foot_alignment_errors = []
    for slot in range(4):
        foot = next(
            item
            for item in links
            if item.get("name") == f"leg{slot}_foot_link"
        )
        visual_origin = vector(foot.find("visual").find("origin"))
        collision_origin = vector(foot.find("collision").find("origin"))
        if not close_sequence(visual_origin, FOOT_COLLISION_OFFSET_XYZ):
            foot_alignment_errors.append(
                f"leg{slot} visual={visual_origin}"
            )
        if not close_sequence(collision_origin, FOOT_COLLISION_OFFSET_XYZ):
            foot_alignment_errors.append(
                f"leg{slot} collision={collision_origin}"
            )
    check(
        "足端 visual/collision 对齐",
        not foot_alignment_errors,
        "4 个 foot visual 与 collision origin 均为 (0,0,0.024)"
        if not foot_alignment_errors
        else "; ".join(foot_alignment_errors),
    )

    target_match = re.search(
        r"target_joint_positions_flat=\[([^\]]+)\]",
        metadata,
    )
    target_values = (
        [float(value) for value in target_match.group(1).split(",")]
        if target_match
        else []
    )
    check(
        "实机 target_jpos",
        close_sequence(target_values, EXPECTED_TARGET),
        "YoboGo 控制侧 [-0.6/-0.6,-1,2.7] 与 [0.6,-1,2.7] 按腿拆分登记",
    )
    sim_match = re.search(
        r"sim_initial_joint_positions_flat=\[([^\]]+)\]",
        metadata,
    )
    sim_values = (
        [float(value) for value in sim_match.group(1).split(",")]
        if sim_match
        else []
    )
    check(
        "URDF/Isaac 仿真初态",
        close_sequence(sim_values, EXPECTED_SIM_INITIAL, tolerance=1e-9),
        "每腿 [0,-0.785398163,1.865468294]，与实机 target_jpos 拆分",
    )
    check(
        "初态拆分语义",
        "target_joint_semantics=YoboGo control-side logical coordinates; "
        "not direct MIT URDF geometric angles" in metadata
        and "sim_initial_joint_semantics=ORCAgym geometry stance for foot "
        "collision minimum -0.26 m relative root" in metadata
        and target_values != sim_values,
        "target_jpos 不得冒充 ORCAgym 几何初态",
    )
    check(
        "仿真 root 出生高度",
        "sim_initial_root_xyz=0,0,0.26" in metadata
        and math.isclose(EXPECTED_SIM_ROOT_Z_M, 0.26, abs_tol=0.0),
        "root 出生高度固定为 0.26 m",
    )

    geometry_errors = []
    try:
        sim_foot_min = foot_collision_min(
            root, flat_joint_positions(sim_values)
        )
        sim_base_min = base_collision_min(
            root, flat_joint_positions(sim_values)
        )
        if not math.isclose(
            sim_foot_min,
            EXPECTED_FOOT_COLLISION_MIN_M,
            rel_tol=0.0,
            abs_tol=2.0e-6,
        ):
            geometry_errors.append(
                f"foot_collision_min={sim_foot_min:.12f}"
            )
        if not math.isclose(
            sim_base_min,
            EXPECTED_BASE_COLLISION_MIN_M,
            rel_tol=0.0,
            abs_tol=1.0e-9,
        ):
            geometry_errors.append(
                f"base_collision_min={sim_base_min:.12f}"
            )
    except (KeyError, TypeError, ValueError) as exc:
        geometry_errors.append(f"FK 异常: {type(exc).__name__}: {exc}")
    check(
        "初态碰撞几何",
        not geometry_errors,
        "foot_collision_min≈-0.260000 m、base_collision_min=-0.040000 m"
        if not geometry_errors
        else "; ".join(geometry_errors),
    )
    check(
        "PD 与 500 Hz",
        "control_period_s=0.002" in metadata
        and "control_frequency_hz=500" in metadata
        and "pd_kp=3,3,3" in metadata
        and "pd_kd=1,0.2,0.2" in metadata,
        "YoboGo Kp=[3,3,3]、Kd=[1,0.2,0.2]、周期 0.002 s",
    )
    check(
        "腿映射",
        "leg_mapping=leg0:FR,leg1:FL,leg2:RR,leg3:RL" in metadata,
        "YoboGo 数组槽位与物理腿固定映射",
    )
    check(
        "来源边界",
        "5aaff694ae5f1c31e08040d59787f2cca4c5cfe0" in metadata
        and "visual_source=local ../visual_dae four DAE only" in metadata
        and "official-mini-cheetah" not in urdf_path.read_text(encoding="utf-8")
        and "mini_vision" not in urdf_path.read_text(encoding="utf-8")
        and all(
            filename.startswith("../visual_dae/")
            for filename in mesh_filenames
        ),
        "动力学固定到 ORCAgym 提交；visual 只引用本地 4 DAE；未引用旧资产路径",
    )
    check(
        "机械限位边界",
        "mechanical hard stop unknown" in metadata
        and "knee [-2.7,2.7] is local initial/safety candidate only" in metadata,
        "abad/hip 标为软件参考，knee 标为本地初始/安全候选，机械 hard stop 未知",
    )
    check(
        "CAD 碰撞边界",
        "CAD not split into links" in metadata,
        "碰撞为 MIT 保守候选，CAD 尚未分件",
    )

    for message in passed:
        print(f"[PASS] {message}")
    for message in warnings:
        print(f"[WARN] {message}")
    for message in errors:
        print(f"[FAIL] {message}")
    print(
        f"汇总: {len(passed)} 项通过, {len(warnings)} 项警告, "
        f"{len(errors)} 项失败"
    )
    if errors:
        return 1
    print("结论: 静态结构验收通过；4 个 foot 派生惯量均严格正定")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
