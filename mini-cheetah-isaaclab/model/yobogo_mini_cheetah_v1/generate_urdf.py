#!/usr/bin/env python3
"""生成 YoboGo-10S Mini Cheetah 形态的静态 URDF 资产。"""

from __future__ import annotations

import math
from pathlib import Path
from xml.etree import ElementTree as ET

URDF_PATH = Path(__file__).with_name("yobogo_mini_cheetah.urdf")
SOURCE_TOTAL_MASS_KG = 8.292
TARGET_TOTAL_MASS_KG = 9.0
MASS_SCALE = TARGET_TOTAL_MASS_KG / SOURCE_TOTAL_MASS_KG
FOOT_COLLISION_RADIUS_M = 0.0202
FOOT_MASS_KG = 0.01 * MASS_SCALE
# 源 foot 惯量全零；用可信的缩放后质量与 MIT 保守球半径按实心球公式
# 派生 PhysX 可接受的正定惯量。该值不是 MIT 原始惯量，也不是猜测 epsilon。
FOOT_DERIVED_INERTIA_KG_M2 = 0.4 * FOOT_MASS_KG * FOOT_COLLISION_RADIUS_M**2

# ORCAgym 固定提交的逐 link 原始数据；质量与惯量统一乘 MASS_SCALE。
SOURCE_MASS = {
    "base": 3.3,
    "hip": 0.54,
    "thigh": 0.634,
    "shank": 0.064,
    "foot": 0.01,
}
SOURCE_INERTIA = {
    # 分量顺序：ixx、ixy、ixz、iyy、iyz、izz。
    "base": (0.011253, 0.0, 0.0, 0.362030, 0.0, 0.042673),
    "hip": (0.000381, 0.000058, 0.00000045, 0.000560, 0.00000095, 0.000444),
    "thigh": (0.001983, 0.000245, 0.000013, 0.002103, 0.0000015, 0.000408),
    "shank": (0.000245, 0.0, 0.0, 0.000248, 0.0, 0.000006),
    # 源文件足端惯量全零；实际输出改用下方实心球派生值。
    "foot": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
}

LEGS = (
    {
        "slot": 0,
        "physical": "FR",
        "source": "rf",
        "haa_xyz": (0.14775, -0.049, 0.0),
        "haa_rpy": (0.0, 0.0, 0.0),
        "haa_axis": (1.0, 0.0, 0.0),
        "hfe_xyz": (0.055, -0.019, 0.0),
        "hfe_rpy": (0.0, 0.0, 0.0),
        "kfe_xyz": (0.0, -0.049, -0.2085),
        "side_y": -1.0,
    },
    {
        "slot": 1,
        "physical": "FL",
        "source": "lf",
        "haa_xyz": (0.14775, 0.049, 0.0),
        "haa_rpy": (0.0, 0.0, 0.0),
        "haa_axis": (1.0, 0.0, 0.0),
        "hfe_xyz": (0.055, 0.019, 0.0),
        "hfe_rpy": (0.0, 0.0, 0.0),
        "kfe_xyz": (0.0, 0.049, -0.2085),
        "side_y": 1.0,
    },
    {
        "slot": 2,
        "physical": "RR",
        "source": "rh",
        "haa_xyz": (-0.14775, -0.049, 0.0),
        "haa_rpy": (0.0, math.pi, 0.0),
        "haa_axis": (-1.0, 0.0, 0.0),
        "hfe_xyz": (0.055, -0.019, 0.0),
        "hfe_rpy": (0.0, math.pi, 0.0),
        "kfe_xyz": (0.0, -0.049, -0.2085),
        "side_y": -1.0,
    },
    {
        "slot": 3,
        "physical": "RL",
        "source": "lh",
        "haa_xyz": (-0.14775, 0.049, 0.0),
        "haa_rpy": (0.0, math.pi, 0.0),
        "haa_axis": (-1.0, 0.0, 0.0),
        "hfe_xyz": (0.055, 0.019, 0.0),
        "hfe_rpy": (0.0, math.pi, 0.0),
        "kfe_xyz": (0.0, 0.049, -0.2085),
        "side_y": 1.0,
    },
)

TARGET_POSITIONS = {
    0: (-0.6, -1.0, 2.7),
    1: (0.6, -1.0, 2.7),
    2: (-0.6, -1.0, 2.7),
    3: (0.6, -1.0, 2.7),
}
# 实机 target_jpos 是控制侧逻辑坐标，不能直接作为 ORCAgym 运动学几何角。
# 仿真初态固定为可让四足碰撞最低点相对 root 等于 -0.26 m 的几何站姿。
SIM_INITIAL_POSITIONS = {
    slot: (0.0, -0.785398163, 1.865468294) for slot in range(4)
}
SIM_INITIAL_ROOT_XYZ = (0.0, 0.0, 0.26)
FOOT_COLLISION_OFFSET_XYZ = (0.0, 0.0, 0.024)
JOINT_LIMITS = {
    "abad": {"lower": -1.5, "upper": 1.5, "velocity": 41.0},
    "hip": {"lower": -5.0, "upper": 5.0, "velocity": 41.0},
    "knee": {"lower": -2.7, "upper": 2.7, "velocity": 26.8},
}
PROJECT_EFFORT = {"abad": 17.0, "hip": 17.0, "knee": 26.0}
PROJECT_KP = {"abad": 3.0, "hip": 3.0, "knee": 3.0}
PROJECT_KD = {"abad": 1.0, "hip": 0.2, "knee": 0.2}


def format_number(value: float) -> str:
    """输出紧凑且可复算的十进制数值。"""
    return "0" if value == 0.0 else format(value, ".12g")


def add_origin(
    parent: ET.Element,
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float] | None = None,
) -> None:
    attrs = {"xyz": " ".join(format_number(value) for value in xyz)}
    if rpy is not None and any(rpy):
        attrs["rpy"] = " ".join(format_number(value) for value in rpy)
    ET.SubElement(parent, "origin", attrs)


def add_inertial(link: ET.Element, kind: str, side_y: float = 0.0) -> None:
    """按统一缩放比例写入逐 link 质量和惯量。"""
    inertial = ET.SubElement(link, "inertial")
    if kind == "base":
        com = (0.0, 0.0, 0.0)
    elif kind == "hip":
        com = (0.055, -0.036 if side_y < 0 else 0.036, 0.0)
    elif kind == "thigh":
        com = (0.0, -0.016 if side_y < 0 else 0.016, -0.02)
    elif kind == "shank":
        com = (0.0, 0.0, -0.061)
    else:
        com = (0.0, 0.0, 0.0)
    add_origin(inertial, com)
    ET.SubElement(
        inertial,
        "mass",
        {"value": format_number(SOURCE_MASS[kind] * MASS_SCALE)},
    )
    names = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
    if kind == "foot":
        output_inertia = (
            FOOT_DERIVED_INERTIA_KG_M2,
            0.0,
            0.0,
            FOOT_DERIVED_INERTIA_KG_M2,
            0.0,
            FOOT_DERIVED_INERTIA_KG_M2,
        )
    else:
        output_inertia = tuple(
            value * MASS_SCALE for value in SOURCE_INERTIA[kind]
        )
    ET.SubElement(
        inertial,
        "inertia",
        {
            name: format_number(value)
            for name, value in zip(names, output_inertia)
        },
    )


def add_mesh_visual(
    link: ET.Element,
    filename: str,
    rpy: tuple[float, float, float] | None = None,
) -> None:
    visual = ET.SubElement(link, "visual")
    add_origin(visual, (0.0, 0.0, 0.0), rpy)
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "mesh", {"filename": filename})


def add_sphere_visual(link: ET.Element) -> None:
    visual = ET.SubElement(link, "visual")
    add_origin(visual, FOOT_COLLISION_OFFSET_XYZ)
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "sphere", {"radius": "0.0202"})
    ET.SubElement(visual, "material", {"name": "foot_gray"})


def add_shape_collision(
    link: ET.Element,
    shape: str,
    dimensions: dict[str, str],
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float] | None = None,
) -> None:
    collision = ET.SubElement(link, "collision")
    add_origin(collision, xyz, rpy)
    geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(geometry, shape, dimensions)


def add_base_link(robot: ET.Element) -> None:
    link = ET.SubElement(robot, "link", {"name": "base_link"})
    add_inertial(link, "base")
    add_mesh_visual(link, "../visual_dae/mini_body.dae")
    add_shape_collision(
        link,
        "box",
        {"size": "0.30 0.20 0.10"},
        (0.0, 0.0, 0.01),
    )


def add_leg_links(robot: ET.Element, leg: dict[str, object]) -> None:
    slot = int(leg["slot"])
    side_y = float(leg["side_y"])
    prefix = f"leg{slot}"

    hip = ET.SubElement(robot, "link", {"name": f"{prefix}_hip_link"})
    add_inertial(hip, "hip", side_y)
    # DAE 几何沿 +Y；右侧翻转、左侧保持，使外伸方向一致。
    hip_rpy = (math.pi, 0.0, 0.0) if side_y < 0 else (0.0, 0.0, 0.0)
    add_mesh_visual(hip, "../visual_dae/mini_abad.dae", hip_rpy)
    add_shape_collision(
        hip,
        "cylinder",
        {"length": "0.025", "radius": "0.05"},
        (0.052, -0.02 if side_y < 0 else 0.02, 0.0),
        (math.pi / 2.0, 0.0, 0.0),
    )

    thigh = ET.SubElement(robot, "link", {"name": f"{prefix}_thigh_link"})
    add_inertial(thigh, "thigh", side_y)
    # DAE 长轴为 -X；绕 Y 旋转 -90 度，与膝关节的 -Z 运动学方向重合。
    add_mesh_visual(
        thigh,
        "../visual_dae/mini_upper_link.dae",
        (0.0, -math.pi / 2.0, 0.0),
    )
    add_shape_collision(
        thigh,
        "cylinder",
        {"length": "0.17", "radius": "0.015"},
        (0.0, 0.05 * side_y, -0.092),
    )

    shank = ET.SubElement(robot, "link", {"name": f"{prefix}_shank_link"})
    add_inertial(shank, "shank", side_y)
    # DAE 长轴为 +Z；绕 Y 旋转 180 度，与小腿的 -Z 方向重合。
    add_mesh_visual(
        shank,
        "../visual_dae/mini_lower_link.dae",
        (0.0, math.pi, 0.0),
    )
    add_shape_collision(
        shank,
        "cylinder",
        {"length": "0.10", "radius": "0.010"},
        (0.0, 0.0, -0.1),
    )

    foot = ET.SubElement(robot, "link", {"name": f"{prefix}_foot_link"})
    add_inertial(foot, "foot", side_y)
    add_sphere_visual(foot)
    add_shape_collision(
        foot,
        "sphere",
        {"radius": "0.0202"},
        FOOT_COLLISION_OFFSET_XYZ,
    )


def add_joint(
    robot: ET.Element,
    name: str,
    joint_type: str,
    parent: str,
    child: str,
    xyz: tuple[float, float, float],
    axis: tuple[float, float, float] | None = None,
    rpy: tuple[float, float, float] | None = None,
    joint_kind: str | None = None,
) -> None:
    joint = ET.SubElement(robot, "joint", {"name": name, "type": joint_type})
    add_origin(joint, xyz, rpy)
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    if joint_type != "revolute":
        return
    assert axis is not None and joint_kind is not None
    limit = JOINT_LIMITS[joint_kind]
    ET.SubElement(joint, "axis", {"xyz": " ".join(format_number(v) for v in axis)})
    ET.SubElement(
        joint,
        "limit",
        {
            "lower": format_number(limit["lower"]),
            "upper": format_number(limit["upper"]),
            "effort": format_number(PROJECT_EFFORT[joint_kind]),
            "velocity": format_number(limit["velocity"]),
        },
    )


def metadata_comment() -> ET.Comment:
    target_flat = [
        value for slot in range(4) for value in TARGET_POSITIONS[slot]
    ]
    sim_initial_flat = [
        value
        for slot in range(4)
        for value in SIM_INITIAL_POSITIONS[slot]
    ]
    return ET.Comment(
        "\n"
        "YoboGo-10S 资产元数据（静态验收口径）\n"
        "leg_mapping=leg0:FR,leg1:FL,leg2:RR,leg3:RL\n"
        "joint_order=leg0_abad_joint,leg0_hip_joint,leg0_knee_joint,"
        "leg1_abad_joint,leg1_hip_joint,leg1_knee_joint,"
        "leg2_abad_joint,leg2_hip_joint,leg2_knee_joint,"
        "leg3_abad_joint,leg3_hip_joint,leg3_knee_joint\n"
        "target_joint_positions_flat=["
        + ",".join(format_number(value) for value in target_flat)
        + "]\n"
        "sim_initial_joint_positions_flat=["
        + ",".join(format_number(value) for value in sim_initial_flat)
        + "]\n"
        f"sim_initial_root_xyz="
        + ",".join(format_number(value) for value in SIM_INITIAL_ROOT_XYZ)
        + "\n"
        "target_joint_semantics=YoboGo control-side logical coordinates; "
        "not direct MIT URDF geometric angles\n"
        "sim_initial_joint_semantics=ORCAgym geometry stance for foot "
        "collision minimum -0.26 m relative root\n"
        "foot_visual_collision_origin_xyz="
        + ",".join(
            format_number(value) for value in FOOT_COLLISION_OFFSET_XYZ
        )
        + "\n"
        "control_period_s=0.002\n"
        "control_frequency_hz=500\n"
        "pd_kp=3,3,3\n"
        "pd_kd=1,0.2,0.2\n"
        "effort_limit=17,17,26\n"
        "source_total_mass_kg=8.292\n"
        "target_total_mass_kg=9\n"
        f"mass_scale=9/8.292={format_number(MASS_SCALE)}\n"
        "dynamics_source=MIT ORCAgym commit "
        "5aaff694ae5f1c31e08040d59787f2cca4c5cfe0\n"
        "visual_source=local ../visual_dae four DAE only\n"
        "control_source=YoboGo-control local files\n"
        "softstop_source=MIT Cheetah-Software SpineBoard reference\n"
        "limit_semantics=URDF limit is software/safety candidate, "
        "mechanical hard stop unknown\n"
        "knee_limit_semantics=knee [-2.7,2.7] is local initial/safety "
        "candidate only\n"
        "collision_status=MIT conservative candidates, CAD not split "
        "into links\n"
        "source_foot_inertia=all four MIT source tensors are zero\n"
        "foot_inertia_derivation=solid_sphere_i=2/5*m*r^2\n"
        f"foot_inertia_mass_kg={format_number(FOOT_MASS_KG)}\n"
        f"foot_inertia_radius_m={format_number(FOOT_COLLISION_RADIUS_M)}\n"
        f"foot_inertia_diagonal_kg_m2="
        f"{format_number(FOOT_DERIVED_INERTIA_KG_M2)}\n"
        "foot_inertia_status=derived from trusted foot mass and MIT "
        "collision sphere radius for PhysX positive definiteness; "
        "not MIT original inertia; not a guessed epsilon\n"
    )


def build_robot() -> ET.Element:
    robot = ET.Element("robot", {"name": "yobogo_mini_cheetah_v1"})
    robot.append(metadata_comment())
    material = ET.SubElement(robot, "material", {"name": "foot_gray"})
    ET.SubElement(material, "color", {"rgba": "0.35 0.35 0.35 1"})

    add_base_link(robot)
    for leg in LEGS:
        add_leg_links(robot, leg)

    for leg in LEGS:
        slot = int(leg["slot"])
        prefix = f"leg{slot}"
        robot.append(
            ET.Comment(
                f" {leg['physical']} / YoboGo {prefix} / "
                f"ORCAgym {leg['source']} "
            )
        )
        add_joint(
            robot,
            f"{prefix}_abad_joint",
            "revolute",
            "base_link",
            f"{prefix}_hip_link",
            leg["haa_xyz"],  # type: ignore[arg-type]
            leg["haa_axis"],  # type: ignore[arg-type]
            leg["haa_rpy"],  # type: ignore[arg-type]
            "abad",
        )
        add_joint(
            robot,
            f"{prefix}_hip_joint",
            "revolute",
            f"{prefix}_hip_link",
            f"{prefix}_thigh_link",
            leg["hfe_xyz"],  # type: ignore[arg-type]
            (0.0, -1.0, 0.0),
            leg["hfe_rpy"],  # type: ignore[arg-type]
            "hip",
        )
        add_joint(
            robot,
            f"{prefix}_knee_joint",
            "revolute",
            f"{prefix}_thigh_link",
            f"{prefix}_shank_link",
            leg["kfe_xyz"],  # type: ignore[arg-type]
            (0.0, -1.0, 0.0),
            (0.0, 0.0, 0.0),
            "knee",
        )
        add_joint(
            robot,
            f"{prefix}_shank_to_foot_joint",
            "fixed",
            f"{prefix}_shank_link",
            f"{prefix}_foot_link",
            (0.0, 0.0, -0.22),
        )
    return robot


def main() -> None:
    robot = build_robot()
    ET.indent(robot, space="  ")
    ET.ElementTree(robot).write(
        URDF_PATH,
        encoding="utf-8",
        xml_declaration=True,
        short_empty_elements=True,
    )
    print(f"已生成 {URDF_PATH}")


if __name__ == "__main__":
    main()
