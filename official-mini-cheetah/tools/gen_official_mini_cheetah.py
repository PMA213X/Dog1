#!/usr/bin/env python3
"""从官方 Mini Cheetah world 提取并自检 Robot 文本。

本脚本只读取参考文件，默认将生成后的 Robot 节点打印到标准输出。
当传入 --output 时才写入指定的新文件，不会修改任何参考文件。
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PROJECT_ROOT.parent
DEFAULT_SOURCE_WORLD = REPOSITORY_ROOT / "webots-sim/worlds/mini_cheetah.wbt"
MINI_CHEETAH_HEADER = (
    REPOSITORY_ROOT
    / "Cheetah-Software/common/include/Dynamics/MiniCheetah.h"
)
MINI_CHEETAH_URDF = REPOSITORY_ROOT / "webots-sim/urdf/mini_cheetah.urdf"

EXPECTED_LEGS = ("fr", "fl", "hr", "hl")
EXPECTED_JOINTS = ("abd", "hip", "kn")


@dataclass(frozen=True)
class ValidationReport:
    """记录一次模型自检的精简结论。"""

    official_robot_joints: int
    official_robot_motors: int
    official_robot_sensors: int
    official_robot_abad_axes: int
    official_robot_pitch_axes: int
    urdf_joints: int
    cpp_body_mass: float
    urdf_body_mass: float
    endpoint_count: int
    max_anchor_endpoint_error: float
    max_zero_world_error: float

    def as_dict(self) -> dict[str, object]:
        """返回可直接序列化的自检摘要。"""

        return {
            "official_robot_joints": self.official_robot_joints,
            "official_robot_motors": self.official_robot_motors,
            "official_robot_sensors": self.official_robot_sensors,
            "official_robot_abad_axes": self.official_robot_abad_axes,
            "official_robot_pitch_axes": self.official_robot_pitch_axes,
            "urdf_joints": self.urdf_joints,
            "cpp_body_mass": self.cpp_body_mass,
            "urdf_body_mass": self.urdf_body_mass,
            "endpoint_count": self.endpoint_count,
            "max_anchor_endpoint_error": self.max_anchor_endpoint_error,
            "max_zero_world_error": self.max_zero_world_error,
        }


def _require(condition: bool, message: str) -> None:
    """条件不满足时立即终止，避免生成降精度模型。"""

    if not condition:
        raise AssertionError(message)


def _extract_braced_block(text: str, start_pattern: str) -> str:
    """提取从首个匹配行开始、花括号完全配对的 VRML 节点。"""

    match = re.search(start_pattern, text, flags=re.MULTILINE)
    _require(match is not None, f"未找到节点：{start_pattern}")
    start = match.start()
    depth = 0
    in_string = False
    escaped = False
    in_line_comment = False
    in_block_comment = False

    for index in range(start, len(text)):
        char = text[index]
        next_char = text[index + 1] if index + 1 < len(text) else ""
        if in_line_comment:
            if char == "\n":
                in_line_comment = False
            continue
        if in_block_comment:
            if char == "*" and next_char == "/":
                in_block_comment = False
            continue
        if in_string:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                in_string = False
            continue
        if char == "#":
            in_line_comment = True
            continue
        if char == "/" and next_char == "*":
            in_block_comment = True
            continue
        if char == '"':
            in_string = True
            continue
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            _require(depth >= 0, "节点花括号提前闭合")
            if depth == 0:
                return text[start : index + 1]

    raise AssertionError(f"节点未闭合：{start_pattern}")


def _extract_braced_block_at(text: str, start: int) -> str:
    """从指定左花括号位置提取完整节点，便于无重叠文本变换。"""

    depth = 0
    in_string = False
    escaped = False
    in_line_comment = False
    in_block_comment = False
    for index in range(start, len(text)):
        char = text[index]
        next_char = text[index + 1] if index + 1 < len(text) else ""
        if in_line_comment:
            if char == "\n":
                in_line_comment = False
            continue
        if in_block_comment:
            if char == "*" and next_char == "/":
                in_block_comment = False
            continue
        if in_string:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                in_string = False
            continue
        if char == "#":
            in_line_comment = True
            continue
        if char == "/" and next_char == "*":
            in_block_comment = True
            continue
        if char == '"':
            in_string = True
            continue
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            _require(depth >= 0, "节点花括号提前闭合")
            if depth == 0:
                return text[start : index + 1]
    raise AssertionError("指定节点未闭合")


def _parse_tuple(text: str) -> tuple[float, float, float]:
    """解析 VRML 三维向量。"""

    values = tuple(float(value) for value in text.split())
    _require(len(values) == 3, f"三维向量长度错误：{text}")
    return values  # type: ignore[return-value]


def _vec_add(
    left: tuple[float, float, float],
    right: tuple[float, float, float],
) -> tuple[float, float, float]:
    """逐分量相加。"""

    return tuple(left[index] + right[index] for index in range(3))  # type: ignore[return-value]


def _joint_blocks(robot: str, legacy: bool = False) -> list[dict[str, object]]:
    """读取全部 HingeJoint 的 motor、anchor 与直接子 Solid 平移。"""

    blocks: list[dict[str, object]] = []
    for match in re.finditer(r"(?m)^(\s*)HingeJoint \{", robot):
        block = _extract_braced_block_at(robot, match.start())
        motor_match = re.search(
            r'name "([a-z]{2}_(?:abd|hip|kn)_motor)"',
            block,
        )
        anchor_match = re.search(
            r"(?m)^\s*anchor\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)",
            block,
        )
        _require(motor_match is not None, "HingeJoint 缺少官方 motor 名")
        _require(anchor_match is not None, "HingeJoint 缺少 anchor")
        child_pattern = (
            r"child Solid\s*\{\s*translation\s+"
            r"([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)"
            if legacy
            else r"endPoint Solid\s*\{\s*translation\s+"
            r"([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)"
        )
        child_match = re.search(child_pattern, block)
        _require(
            child_match is not None,
            "HingeJoint 缺少直接子 Solid translation",
        )
        blocks.append(
            {
                "start": match.start(),
                "block": block,
                "motor": motor_match.group(1),
                "anchor": _parse_tuple(anchor_match.group(0).split("anchor", 1)[1]),
                "child_translation": _parse_tuple(
                    " ".join(child_match.group(index) for index in (1, 2, 3))
                ),
                "indent": match.group(1),
            }
        )
    _require(len(blocks) == 12, f"HingeJoint 数量错误：{len(blocks)}")
    _require(
        len({str(item["motor"]) for item in blocks}) == 12,
        "HingeJoint motor 名重复",
    )
    return blocks


def _expected_endpoint_translations(
    legacy_blocks: list[dict[str, object]],
) -> dict[str, tuple[float, float, float]]:
    """按旧 child 相对关节 anchor 的语义换算 R2025a endPoint 平移。"""

    expected: dict[str, tuple[float, float, float]] = {}
    for item in legacy_blocks:
        anchor = item["anchor"]
        child = item["child_translation"]
        _require(isinstance(anchor, tuple) and isinstance(child, tuple), "向量类型错误")
        expected[str(item["motor"])] = _vec_add(anchor, child)
    return expected


def _convert_child_to_endpoint(robot: str) -> str:
    """将官方旧 child Solid 精确转换为 R2025a endPoint Solid。"""

    legacy_blocks = _joint_blocks(robot, legacy=True)
    expected_endpoints = _expected_endpoint_translations(legacy_blocks)
    locations: list[tuple[int, str]] = []
    for motor in expected_endpoints:
        marker = f'name "{motor}"'
        motor_index = robot.find(marker)
        _require(motor_index >= 0, f"缺少 motor marker：{motor}")
        locations.append((motor_index, motor))
    # 从最深、最靠后的关节向前处理，避免嵌套块互相覆盖。
    for motor_index, motor in sorted(locations, reverse=True):
        child_match = re.search(
            r"child Solid(\s*\{\s*translation\s+)"
            r"([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)",
            robot[motor_index:],
        )
        _require(child_match is not None, f"缺少旧 child Solid：{motor}")
        absolute_start = motor_index + child_match.start()
        absolute_end = motor_index + child_match.end()
        endpoint = expected_endpoints[motor]
        replacement = (
            "endPoint Solid"
            + child_match.group(1)
            + " ".join(f"{value:.12g}" for value in endpoint)
            + "  # R2025a：相对父 Pose，保留官方零位几何"
        )
        robot = robot[:absolute_start] + replacement + robot[absolute_end:]

    robot, stop_count = re.subn(
        r"\n([ \t]*)stopSpringDamper 100 1",
        r"\n\1springConstant 100",
        robot,
    )
    _require(stop_count == 12, f"旧 stopSpringDamper 迁移数量错误：{stop_count}")
    _require("child Solid" not in robot, "转换后仍存在 child Solid")
    _require(robot.count("endPoint Solid") == 12, "endPoint Solid 数量不是 12")
    _require("stopSpringDamper" not in robot, "仍存在旧 stopSpringDamper")
    _require(robot.count("springConstant 100") == 12, "springConstant 迁移数量不是 12")
    return robot


def _find_closing_bracket(text: str, start: int) -> int:
    """找到从 start 开始的方括号闭合位置，忽略字符串和注释。"""

    depth = 0
    in_string = False
    escaped = False
    in_line_comment = False
    in_block_comment = False
    for index in range(start, len(text)):
        char = text[index]
        next_char = text[index + 1] if index + 1 < len(text) else ""
        if in_line_comment:
            if char == "\n":
                in_line_comment = False
            continue
        if in_block_comment:
            if char == "*" and next_char == "/":
                in_block_comment = False
            continue
        if in_string:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                in_string = False
            continue
        if char == "#":
            in_line_comment = True
            continue
        if char == "/" and next_char == "*":
            in_block_comment = True
            continue
        if char == '"':
            in_string = True
            continue
        if char == "[":
            depth += 1
        elif char == "]":
            depth -= 1
            if depth == 0:
                return index
    raise AssertionError("children 方括号未闭合")


def _move_nested_joints_into_children(robot: str) -> str:
    """把旧 Solid 直接子 HingeJoint 移入 R2025a children 列表。"""

    moved = 0
    for leg in EXPECTED_LEGS:
        for kind in ("abad", "abd", "thigh", "shank"):
            marker = f'name "{leg}_{kind}_link"'
            name_index = robot.find(marker)
            if name_index < 0:
                continue
            endpoint_start = robot.rfind("endPoint Solid {", 0, name_index)
            _require(endpoint_start >= 0, f"找不到 endPoint：{marker}")
            endpoint_block = _extract_braced_block_at(
                robot,
                endpoint_start + len("endPoint ") ,
            )
            # _extract_braced_block_at expects the opening brace position.
            endpoint_brace = endpoint_start + len("endPoint Solid ")
            endpoint_block = _extract_braced_block_at(robot, endpoint_brace)
            children_start = endpoint_block.find("children [")
            _require(children_start >= 0, f"endPoint 缺少 children：{marker}")
            children_open = endpoint_block.find("[", children_start)
            children_close = _find_closing_bracket(endpoint_block, children_open)
            direct_joint_match = re.search(
                r"(?m)^[ \t]*HingeJoint \{",
                endpoint_block[children_close + 1 :],
            )
            if direct_joint_match is None:
                continue
            joint_start = children_close + 1 + direct_joint_match.start()
            joint_brace = children_close + 1 + direct_joint_match.start(
            ) + direct_joint_match.group(0).find("{")
            joint_block = _extract_braced_block_at(endpoint_block, joint_brace)
            # 提取结果从左花括号开始，结束位置必须按花括号块本身计算；
            # 同时保留完整的 “HingeJoint …” 节点文本，移入 children 后
            # 才能继续被 R2025a 解析器识别。
            joint_end = joint_brace + len(joint_block)
            joint_text = endpoint_block[joint_start:joint_end]
            endpoint_without_joint = (
                endpoint_block[:joint_start] + endpoint_block[joint_end:]
            )
            new_children_close = endpoint_without_joint.find(
                "children [",
                0,
            )
            new_children_open = endpoint_without_joint.find(
                "[",
                new_children_close,
            )
            new_children_close = _find_closing_bracket(
                endpoint_without_joint,
                new_children_open,
            )
            endpoint_with_joint = (
                endpoint_without_joint[:new_children_close]
                + "\n"
                + joint_text
                + "\n"
                + endpoint_without_joint[new_children_close:]
            )
            # endpoint_start 指向 “endPoint Solid ” 前缀；替换时保留该前缀，
            # 只替换其后的花括号块，避免丢失顶层 endPoint 字段名。
            endpoint_brace_start = endpoint_start + len("endPoint Solid ")
            robot = (
                robot[:endpoint_brace_start]
                + endpoint_with_joint
                + robot[endpoint_brace_start + len(endpoint_block):]
            )
            moved += 1
    _require(moved == 8, f"移入 children 的 HingeJoint 数量错误：{moved}")
    return robot


def _format_vector(values: tuple[float, ...]) -> str:
    """按稳定精度输出 VRML 向量。"""

    return " ".join(f"{value:.12g}" for value in values)


def _rpy_to_axis_angle(
    rpy: tuple[float, float, float],
) -> tuple[float, float, float, float]:
    """把 URDF RPY 转成 Webots rotation 轴角。"""

    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    _require(norm > 1e-15, "URDF RPY 无法转换为轴角")
    qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
    if qw < 0.0:
        qx, qy, qz, qw = -qx, -qy, -qz, -qw
    sin_half = math.sqrt(max(0.0, 1.0 - qw * qw))
    if sin_half <= 1e-12:
        return 0.0, 0.0, 1.0, 0.0
    return qx / sin_half, qy / sin_half, qz / sin_half, 2.0 * math.acos(qw)


def _official_visual_data() -> dict[str, dict[str, object]]:
    """读取官方 URDF 的 mesh、visual origin 与 toe 固定关节。"""

    root = ET.parse(MINI_CHEETAH_URDF).getroot()
    links = {link.get("name"): link for link in root.findall("link")}
    expected_links = ["body"]
    for leg in EXPECTED_LEGS:
        expected_links.extend((f"abduct_{leg}", f"thigh_{leg}", f"shank_{leg}"))
    _require(set(expected_links).issubset(links), "URDF 缺少官方外观 link")

    result: dict[str, dict[str, object]] = {}
    for link_name in expected_links:
        visual = links[link_name].find("visual")
        _require(visual is not None, f"URDF link 缺少 visual：{link_name}")
        origin = visual.find("origin")
        xyz = _parse_tuple(origin.get("xyz", "0 0 0"))
        rpy = _parse_tuple(origin.get("rpy", "0 0 0"))
        mesh = visual.find("geometry/mesh")
        if mesh is None:
            sphere = visual.find("geometry/sphere")
            _require(sphere is not None, f"URDF visual 几何缺失：{link_name}")
            radius = float(sphere.get("radius", "nan"))
            _require(radius == 0.015, f"官方 toe 半径异常：{radius}")
            result[link_name] = {
                "kind": "sphere",
                "xyz": xyz,
                "rpy": rpy,
                "radius": radius,
            }
            continue
        filename = mesh.get("filename", "")
        mesh_path = (MINI_CHEETAH_URDF.parent / filename).resolve()
        _require(mesh_path.is_file(), f"官方 mesh 资源不存在：{filename}")
        _require(
            mesh_path.parent == (MINI_CHEETAH_URDF.parent / "meshes").resolve(),
            f"官方 mesh 不在预期资源目录：{filename}",
        )
        relative = mesh_path.relative_to(REPOSITORY_ROOT)
        world_relative = Path("../..") / relative
        result[link_name] = {
            "kind": "mesh",
            "xyz": xyz,
            "rpy": rpy,
            "path": mesh_path,
            "url": world_relative.as_posix(),
        }

    toe_joint = next(
        joint
        for joint in root.findall("joint")
        if joint.get("name") == "toe_fr_joint" and joint.get("type") == "fixed"
    )
    toe_origin = toe_joint.find("origin")
    _require(toe_origin is not None, "URDF 缺少官方 toe 固定关节")
    result["toe"] = {
        "kind": "sphere",
        "xyz": _parse_tuple(toe_origin.get("xyz", "0 0 0")),
        "rpy": (0.0, 0.0, 0.0),
        "radius": 0.015,
    }
    _require(
        all(data["kind"] == "mesh" for name, data in result.items() if name != "toe"),
        "官方 link visual 不是 mesh",
    )
    return result


def _mesh_pose(
    visual: dict[str, object],
    frame_offset: tuple[float, float, float],
    indent: str,
) -> str:
    """生成相对当前 Solid frame 的官方 mesh Pose。"""

    xyz = visual["xyz"]
    rpy = visual["rpy"]
    _require(
        isinstance(xyz, tuple) and isinstance(rpy, tuple),
        "官方 visual origin 类型错误",
    )
    relative_xyz = tuple(
        xyz[index] - frame_offset[index] for index in range(3)
    )
    axis_x, axis_y, axis_z, angle = _rpy_to_axis_angle(rpy)  # type: ignore[arg-type]
    url = str(visual["url"])
    return (
        f"{indent}Pose {{\n"
        f"{indent}  translation {_format_vector(relative_xyz)}\n"
        f"{indent}  rotation {axis_x:.12g} {axis_y:.12g} {axis_z:.12g} {angle:.12g}\n"
        f"{indent}  children [\n"
        f"{indent}    Shape {{\n"
        f"{indent}      castShadows FALSE\n"
        f"{indent}      appearance Appearance {{\n"
        f"{indent}        material Material {{\n"
        f"{indent}          diffuseColor 0.8 0.8 0.8\n"
        f"{indent}        }}\n"
        f"{indent}      }}\n"
        f"{indent}      geometry Mesh {{\n"
        f"{indent}        url [\n"
        f'{indent}          "{url}"\n'
        f"{indent}        ]\n"
        f"{indent}      }}\n"
        f"{indent}    }}\n"
        f"{indent}  ]\n"
        f"{indent}}}"
    )


def _toe_shape(
    visual: dict[str, object],
    frame_offset: tuple[float, float, float],
    indent: str,
) -> str:
    """按官方固定关节位置生成 toe 球体。"""

    xyz = visual["xyz"]
    _require(isinstance(xyz, tuple), "官方 toe origin 类型错误")
    relative_xyz = tuple(
        xyz[index] - frame_offset[index] for index in range(3)
    )
    radius = float(visual["radius"])
    return (
        f"{indent}Pose {{\n"
        f"{indent}  translation {_format_vector(relative_xyz)}\n"
        f"{indent}  rotation 0 0 1 0\n"
        f"{indent}  children [\n"
        f"{indent}    Shape {{\n"
        f"{indent}      appearance Appearance {{\n"
        f"{indent}        material Material {{\n"
        f"{indent}          diffuseColor 0.2 0.2 0.2\n"
        f"{indent}        }}\n"
        f"{indent}      }}\n"
        f"{indent}      geometry Sphere {{\n"
        f"{indent}        radius {radius:.12g}\n"
        f"{indent}        subdivision 4\n"
        f"{indent}      }}\n"
        f"{indent}    }}\n"
        f"{indent}  ]\n"
        f"{indent}}}"
    )


def _replace_children_block(
    robot: str,
    marker: str,
    content: str,
) -> str:
    """替换 marker 前最近一个 children 列表，保留同 Solid 的其他字段。"""

    marker_index = robot.find(marker)
    _require(marker_index >= 0, f"缺少 Solid 标记：{marker}")
    children_start = robot.rfind("children [", 0, marker_index)
    _require(children_start >= 0, f"Solid 缺少 children：{marker}")
    line_start = robot.rfind("\n", 0, children_start) + 1
    indent_match = re.match(r"[ \t]*", robot[line_start:])
    indent = indent_match.group(0) if indent_match else ""
    children_open = robot.find("[", children_start)
    children_close = _find_closing_bracket(robot, children_open)
    return (
        robot[:line_start]
        + f"{indent}children [\n"
        + content
        + f"\n{indent}]"
        + robot[children_close + 1 :]
    )


def _replace_official_visual_shapes(robot: str) -> str:
    """用官方 URDF DAE mesh 替换 world 中的简化盒/球外观。"""

    visuals = _official_visual_data()
    legacy_blocks = _joint_blocks(robot, legacy=True)
    by_motor = {str(item["motor"]): item for item in legacy_blocks}

    body_marker = "# ---- Body visual ----"
    body_start = robot.find(body_marker)
    body_end = robot.find("# ---- Front Right Leg ----", body_start)
    _require(body_start >= 0 and body_end > body_start, "机身外观区段缺失")
    body_node = _mesh_pose(visuals["body"], (0.0, 0.0, 0.0), "    ")
    robot = (
        robot[: body_start + len(body_marker)]
        + "\n"
        + body_node
        + "\n"
        + robot[body_end:]
    )

    for leg in EXPECTED_LEGS:
        frame_offsets = {
            f"{leg}_abd_link": by_motor[f"{leg}_abd_motor"]["child_translation"],
            f"{leg}_thigh_link": by_motor[f"{leg}_hip_motor"]["child_translation"],
            f"{leg}_shank_link": by_motor[f"{leg}_kn_motor"]["child_translation"],
        }
        urdf_names = {
            f"{leg}_abd_link": f"abduct_{leg}",
            f"{leg}_thigh_link": f"thigh_{leg}",
            f"{leg}_shank_link": f"shank_{leg}",
        }
        for world_name, urdf_name in urdf_names.items():
            offset = frame_offsets[world_name]
            _require(
                isinstance(offset, tuple),
                f"官方 frame offset 类型错误：{world_name}",
            )
            base_indent = "        "
            if world_name.endswith("_thigh_link"):
                base_indent = "            "
            elif world_name.endswith("_shank_link"):
                base_indent = "                "
            content = _mesh_pose(visuals[urdf_name], offset, base_indent + "  ")
            if world_name.endswith("_shank_link"):
                content += "\n" + _toe_shape(
                    visuals["toe"],
                    offset,
                    base_indent + "  ",
                )
            robot = _replace_children_block(
                robot,
                f'name "{world_name}"',
                content,
            )

    _require(robot.count("geometry Mesh {") == 13, "官方 mesh 数量不是 13")
    _require(robot.count("mini_abad.dae") == 4, "官方 abad mesh 数量不是 4")
    _require(robot.count("mini_upper_link.dae") == 4, "官方 thigh mesh 数量不是 4")
    _require(robot.count("mini_lower_link.dae") == 4, "官方 shank mesh 数量不是 4")
    _require(robot.count("geometry Sphere {") == 4, "官方 toe 球数量不是 4")
    return robot


def _add_bounding_objects(robot: str) -> str:
    """为机身与 12 个连杆补入与官方视觉一致的碰撞体。"""

    root_pattern = re.compile(r"(?m)^(  \])\n(  physics Physics \{)")
    robot, root_count = root_pattern.subn(
        r"\1\n"
        "  boundingObject Transform {\n"
        "    children [\n"
        "      Box { size 0.38 0.1 0.06 }\n"
        "    ]\n"
        "  }\n"
        r"\2",
        robot,
        count=1,
    )
    _require(root_count == 1, "机身 boundingObject 插入失败")

    link_pattern = re.compile(
        r"(?m)^(\s*)name \"([a-z]{2})_(abad|abd|thigh|shank)_link\"\n"
        r"(?=\s*physics Physics \{)"
    )

    def insert_link(match: re.Match[str]) -> str:
        indent = match.group(1)
        kind = match.group(3)
        if kind in ("abad", "abd"):
            geometry = "Box { size 0.03 0.124 0.03 }"
        elif kind == "thigh":
            geometry = "Box { size 0.04 0.04 0.209 }"
        else:
            geometry = (
                "Box { size 0.03 0.03 0.18 }\n"
                f"{indent}  Sphere {{ radius 0.015 }}"
            )
        # R2025a 对 boundingObject 内的 Pose 只接受一个子节点；小腿需要
        # 同时保留箱体和 toe 球体，因此用 Group 承载多个碰撞几何。
        return (
            f"{match.group(0)}"
            f"{indent}boundingObject Group {{\n"
            f"{indent}  children [\n"
            f"{indent}    {geometry}\n"
            f"{indent}  ]\n"
            f"{indent}}}\n"
        )

    robot, link_count = link_pattern.subn(insert_link, robot)
    _require(link_count == 12, f"连杆 boundingObject 数量错误：{link_count}")
    _require(robot.count("boundingObject") == 13, "boundingObject 总数不是 13")
    return robot


def _convert_inertia_matrices(robot: str) -> str:
    """把旧 3x3 惯量矩阵转为 R2025a 的两向量格式。"""

    positions = [
        match.start()
        for match in re.finditer(r"inertiaMatrix \[", robot)
    ]
    replacements: list[tuple[int, int, str]] = []
    for bracket_start in positions:
        open_bracket = robot.find("[", bracket_start)
        _require(open_bracket > bracket_start, "inertiaMatrix 缺少左括号")
        close = robot.find("]", open_bracket)
        _require(close > bracket_start, "inertiaMatrix 未闭合")
        body = robot[open_bracket : close + 1]
        numbers = [
            float(value)
            for value in re.findall(
                r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?",
                body,
            )
        ]
        _require(len(numbers) == 9, f"inertiaMatrix 数量错误：{len(numbers)}")
        ixx, ixy, ixz, iyy, iyz, izz, _, _, _ = (
            numbers[0],
            numbers[1],
            numbers[2],
            numbers[3],
            numbers[4],
            numbers[5],
            numbers[6],
            numbers[7],
            numbers[8],
        )
        # 旧矩阵按行给出；R2025a 需要 [I11 I22 I33] 与 [I12 I13 I23]。
        i11, i22, i33 = numbers[0], numbers[4], numbers[8]
        i12, i13, i23 = numbers[1], numbers[2], numbers[5]
        _require(
            all(value > 0 for value in (i11, i22, i33)),
            "R2025a 主惯量必须为正",
        )
        converted = (
            "inertiaMatrix [\n"
            f"      {i11:.12g} {i22:.12g} {i33:.12g}\n"
            f"      {i12:.12g} {i13:.12g} {i23:.12g}\n"
            "    ]"
        )
        del ixx, ixy, ixz, iyy, iyz, izz
        replacements.append((bracket_start, close + 1, converted))
    for start, end, converted in sorted(replacements, reverse=True):
        robot = robot[:start] + converted + robot[end:]
    _require(robot.count("inertiaMatrix [") == 13, "惯量矩阵数量不是 13")
    _require("4.5e-07,\n" not in robot, "仍存在旧三行惯量格式")
    return robot


def _clean_legacy_fields(robot: str) -> str:
    """清理 R2025a 已移除字段，并保留等价接触/阻尼语义。"""

    robot, coordinate_count = re.subn(
        r"[ \t]*coordinateSystem \"NUE\"\n",
        "      # R2025a 无 coordinateSystem 字段；本验收不依赖 IMU 坐标系。\n",
        robot,
        count=1,
    )
    _require(coordinate_count == 1, "旧 coordinateSystem 清理失败")

    friction_match = re.search(r"[ \t]*frictionMaterial ContactProperties \{", robot)
    _require(friction_match is not None, "旧 frictionMaterial 未找到")
    start = friction_match.start()
    block = _extract_braced_block_at(robot, friction_match.end() - 1)
    end = friction_match.end() - 1 + len(block)
    indent_match = re.match(r"[ \t]*", friction_match.group(0))
    indent = indent_match.group(0) if indent_match else ""
    robot = (
        robot[:start]
        + f"{indent}# 旧 frictionMaterial 已迁移到 WorldInfo.contactProperties。\n"
        + robot[end:]
    )
    _require(
        re.search(r"(?m)^\s*frictionMaterial", robot) is None,
        "旧 frictionMaterial 字段仍存在",
    )
    _require("dampingConstant 1.0" in robot, "关节阻尼被意外删除")
    return robot


def _zero_world_positions(
    blocks: list[dict[str, object]],
    endpoint_translations: dict[str, tuple[float, float, float]],
    legacy: bool = False,
) -> dict[str, tuple[float, float, float]]:
    """计算零位下 12 个关节轴的世界坐标。"""

    by_motor = {str(item["motor"]): item for item in blocks}
    result: dict[str, tuple[float, float, float]] = {}
    for leg in EXPECTED_LEGS:
        parent_origin = (0.0, 0.0, 0.0)
        for joint in EXPECTED_JOINTS:
            motor = f"{leg}_{joint}_motor"
            item = by_motor[motor]
            anchor = item["anchor"]
            _require(isinstance(anchor, tuple), "anchor 类型错误")
            joint_world = _vec_add(parent_origin, anchor)
            result[motor] = joint_world
            if legacy:
                child = item["child_translation"]
                _require(isinstance(child, tuple), "child 类型错误")
                endpoint_origin = _vec_add(joint_world, child)
            else:
                endpoint_origin = _vec_add(
                    parent_origin,
                    endpoint_translations[motor],
                )
            parent_origin = endpoint_origin
    return result


def _urdf_zero_world_positions(path: Path) -> dict[str, tuple[float, float, float]]:
    """从 URDF 三个关节原点链式计算零位世界坐标。"""

    root = ET.parse(path).getroot()
    joints = {
        joint.get("name"): _parse_tuple(joint.find("origin").get("xyz"))
        for joint in root.findall("joint")
        if joint.get("type") != "fixed" and joint.find("origin") is not None
    }
    result: dict[str, tuple[float, float, float]] = {}
    for leg in EXPECTED_LEGS:
        origin = (0.0, 0.0, 0.0)
        names = (
            f"torso_to_abduct_{leg}_j",
            f"abduct_{leg}_to_thigh_{leg}_j",
            f"thigh_{leg}_to_knee_{leg}_j",
        )
        for joint_name, motor in zip(
            names,
            (f"{leg}_abd_motor", f"{leg}_hip_motor", f"{leg}_kn_motor"),
        ):
            origin = _vec_add(origin, joints[joint_name])
            result[motor] = origin
    return result


def _max_vector_error(
    left: dict[str, tuple[float, float, float]],
    right: dict[str, tuple[float, float, float]],
) -> float:
    """计算两组关节世界坐标的最大欧氏误差。"""

    _require(set(left) == set(right), "关节坐标键集合不一致")
    return max(
        math.dist(left[motor], right[motor])
        for motor in left
    )


def _active_cpp_assignment(source: str, field: str) -> float:
    """读取 MiniCheetah.h 中未注释的数值赋值。"""

    pattern = rf"^\s*cheetah\.{re.escape(field)}\s*=\s*([^;]+);"
    match = re.search(pattern, source, flags=re.MULTILINE)
    _require(match is not None, f"C++ 模型缺少字段：{field}")
    expression = match.group(1).strip()
    allowed = set("0123456789.+*/() eE-")
    _require(
        all(token in allowed for token in expression),
        f"C++ 字段不是受支持的数值表达式：{field}={expression}",
    )
    value = float(eval(expression, {"__builtins__": {}}, {}))  # noqa: S307
    return value


def _assert_close(actual: float, expected: float, label: str) -> None:
    """以严格容差核对官方常量。"""

    _require(
        abs(actual - expected) <= 1e-12,
        f"{label} 不一致：实际 {actual}，期望 {expected}",
    )


def _validate_cpp_model(path: Path) -> float:
    """断言 C++ 动力学构造中的核心尺寸、质量与惯量来源字段。"""

    source = path.read_text(encoding="utf-8")
    expected_scalars = {
        "_bodyMass": 3.3,
        "_bodyLength": 0.38,
        "_bodyWidth": 0.098,
        "_bodyHeight": 0.1,
        "_abadGearRatio": 6.0,
        "_hipGearRatio": 6.0,
        "_kneeGearRatio": 9.33,
        "_abadLinkLength": 0.062,
        "_hipLinkLength": 0.209,
        "_kneeLinkY_offset": 0.004,
        "_kneeLinkLength": 0.195,
        "_maxLegLength": 0.409,
    }
    for field, expected in expected_scalars.items():
        _assert_close(_active_cpp_assignment(source, field), expected, field)

    required_fragments = (
        "inertia parameters of all bodies are",
        "determined from CAD.",
        "abadRotationalInertia << 381, 58, 0.45, 58, 560, 0.95, 0.45, 0.95, 444;",
        "hipRotationalInertia << 1983, 245, 13, 245, 2103, 1.5, 13, 1.5, 408;",
        "kneeRotationalInertiaRotated << 6, 0, 0, 0, 248, 0, 0, 0, 245;",
        "bodyRotationalInertia << 11253, 0, 0, 0, 36203, 0, 0, 0, 42673;",
        "Vec3<T> abadCOM(0, 0.036, 0);",
        "Vec3<T> hipCOM(0, 0.016, -0.02);",
        "Vec3<T> kneeCOM(0, 0, -0.061);",
    )
    for fragment in required_fragments:
        _require(fragment in source, f"C++ 模型核心内容缺失：{fragment}")
    return _active_cpp_assignment(source, "_bodyMass")


def _parse_inertia(element: ET.Element) -> tuple[float, ...]:
    """读取 URDF 惯量六分量。"""

    keys = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
    return tuple(float(element.get(key, "nan")) for key in keys)


def _validate_urdf(path: Path) -> tuple[int, float]:
    """断言 URDF 拓扑、安装点、质量与惯量。"""

    root = ET.parse(path).getroot()
    links = {link.get("name"): link for link in root.findall("link")}
    joints = root.findall("joint")
    active_joints = [joint for joint in joints if joint.get("type") != "fixed"]
    _require(len(active_joints) == 12, "URDF 主动关节数不是 12")

    expected_joint_axes = {
        f"torso_to_abduct_{leg}_j": "1 0 0" for leg in EXPECTED_LEGS
    }
    for leg in EXPECTED_LEGS:
        expected_joint_axes[f"abduct_{leg}_to_thigh_{leg}_j"] = "0 -1 0"
        expected_joint_axes[f"thigh_{leg}_to_knee_{leg}_j"] = "0 -1 0"
    for joint in active_joints:
        name = joint.get("name", "")
        axis = joint.find("axis")
        _require(name in expected_joint_axes, f"URDF 出现未知关节：{name}")
        _require(axis is not None, f"URDF 关节缺少 axis：{name}")
        _require(
            axis.get("xyz") == expected_joint_axes[name],
            f"URDF 关节轴不一致：{name}",
        )

    expected_origins = {
        "fr": ((0.19, -0.049, 0.0), (0.0, -0.062, 0.0), (0.0, 0.0, -0.209)),
        "fl": ((0.19, 0.049, 0.0), (0.0, 0.062, 0.0), (0.0, 0.0, -0.209)),
        "hr": ((-0.19, -0.049, 0.0), (0.0, -0.062, 0.0), (0.0, 0.0, -0.209)),
        "hl": ((-0.19, 0.049, 0.0), (0.0, 0.062, 0.0), (0.0, 0.0, -0.209)),
    }
    for leg, expected in expected_origins.items():
        names = (
            f"torso_to_abduct_{leg}_j",
            f"abduct_{leg}_to_thigh_{leg}_j",
            f"thigh_{leg}_to_knee_{leg}_j",
        )
        for name, expected_xyz in zip(names, expected):
            joint = next(item for item in joints if item.get("name") == name)
            origin = joint.find("origin")
            _require(origin is not None, f"URDF 关节缺少 origin：{name}")
            actual_xyz = tuple(float(v) for v in origin.get("xyz", "").split())
            _require(actual_xyz == expected_xyz, f"URDF 关节位置不一致：{name}")

    body = links["body"]
    body_mass = float(body.find("inertial/mass").get("value"))
    _assert_close(body_mass, 3.3, "URDF body mass")
    body_inertia = _parse_inertia(body.find("inertial/inertia"))
    _require(
        body_inertia == (0.011253, 0.0, 0.0, 0.036203, 0.0, 0.042673),
        f"URDF body 惯量不一致：{body_inertia}",
    )

    expected_link_data = {
        "abduct_fr": (0.54, (0.000381, 0.000058, 0.00000045, 0.000560, 0.00000095, 0.000444)),
        "thigh_fr": (0.634, (0.001983, 0.000245, 0.000013, 0.002103, 0.0000015, 0.000408)),
        "shank_fr": (0.064, (0.000245, 0.0, 0.0, 0.000248, 0.0, 0.000006)),
    }
    for link_name, (expected_mass, expected_inertia) in expected_link_data.items():
        link = links[link_name]
        actual_mass = float(link.find("inertial/mass").get("value"))
        actual_inertia = _parse_inertia(link.find("inertial/inertia"))
        _assert_close(actual_mass, expected_mass, f"URDF {link_name} mass")
        _require(
            actual_inertia == expected_inertia,
            f"URDF {link_name} 惯量不一致：{actual_inertia}",
        )

    toe_masses = [
        float(links[f"toe_{leg}"].find("inertial/mass").get("value"))
        for leg in EXPECTED_LEGS
    ]
    _require(
        all(abs(mass - 0.15) <= 1e-12 for mass in toe_masses),
        f"URDF toe 质量不一致：{toe_masses}",
    )
    return len(active_joints), body_mass


def _validate_official_robot(
    robot: str,
    *,
    generated: bool = False,
) -> tuple[int, int, int, int, int]:
    """断言官方 Robot 文本的关节、设备、轴、尺寸、质量和惯量。"""

    counts = {
        "HingeJoint": robot.count("HingeJoint {"),
        "RotationalMotor": robot.count("RotationalMotor {"),
        "PositionSensor": robot.count("PositionSensor {"),
        "abad_axis": robot.count("axis 1 0 0"),
        "pitch_axis": robot.count("axis 0 -1 0"),
    }
    _require(counts["HingeJoint"] == 12, "官方 Robot HingeJoint 不是 12")
    _require(counts["RotationalMotor"] == 12, "官方 Robot 电机不是 12")
    _require(counts["PositionSensor"] == 12, "官方 Robot 传感器不是 12")
    _require(counts["abad_axis"] == 4, "官方 Robot abad 轴不是 4")
    _require(counts["pitch_axis"] == 8, "官方 Robot hip/knee 轴不是 8")

    for leg in EXPECTED_LEGS:
        for joint in EXPECTED_JOINTS:
            _require(
                robot.count(f'name "{leg}_{joint}_motor"') == 1,
                f"官方 Robot 电机名重复或缺失：{leg}_{joint}",
            )
            _require(
                robot.count(f'name "{leg}_{joint}_sensor"') == 1,
                f"官方 Robot 传感器名重复或缺失：{leg}_{joint}",
            )

    common_literals = (
        "mass 3.3",
        "mass 0.54",
        "mass 0.634",
        "mass 0.214",
        "anchor 0.19 -0.049 0",
        "anchor 0.19 0.049 0",
        "anchor -0.19 -0.049 0",
        "anchor -0.19 0.049 0",
        "anchor 0 0 -0.1045",
    )
    for literal in common_literals:
        _require(literal in robot, f"官方 Robot 关键常量缺失：{literal}")

    if generated:
        _require(robot.count("geometry Mesh {") == 13, "生成结果官方 mesh 不是 13")
        _require(robot.count("mini_body.dae") == 1, "生成结果 body mesh 数量错误")
        _require(robot.count("mini_abad.dae") == 4, "生成结果 abad mesh 数量错误")
        _require(
            robot.count("mini_upper_link.dae") == 4,
            "生成结果 thigh mesh 数量错误",
        )
        _require(
            robot.count("mini_lower_link.dae") == 4,
            "生成结果 shank mesh 数量错误",
        )
        _require(robot.count("geometry Sphere {") == 4, "生成结果 toe 球数量错误")
        _require(
            robot.count('diffuseColor 0.2 0.2 0.2') == 4,
            "生成结果官方 toe 颜色数量错误",
        )
        generated_literals = (
            "Box { size 0.38 0.1 0.06 }",
            "size 0.04 0.04 0.209",
            "size 0.03 0.03 0.18",
            "radius 0.015",
        )
        for literal in generated_literals:
            _require(literal in robot, f"生成结果碰撞常量缺失：{literal}")
    else:
        source_literals = (
            "Box {\n      size 0.38 0.1 0.06\n    }",
            "size 0.04 0.04 0.209",
            "size 0.03 0.03 0.18",
            "radius 0.015",
        )
        for literal in source_literals:
            _require(literal in robot, f"官方源 Robot 外观常量缺失：{literal}")

    inertia_alternatives = (
        ("0.011253, 0, 0,", ("0.011253", "0.036203", "0.042673")),
        (
            "3.81e-04, 5.8e-05, 4.5e-07,",
            ("0.000381", "5.8e-05", "4.5e-07"),
        ),
        (
            "1.983e-03, 2.45e-04, 1.3e-05,",
            ("0.001983", "0.000245", "1.3e-05"),
        ),
        (
            "2.70e-04, 0, 0,",
            ("0.00027", "0.000273", "3.1e-05"),
        ),
    )
    for old_literal, new_literals in inertia_alternatives:
        _require(
            old_literal in robot or all(part in robot for part in new_literals),
            f"惯量常量缺失：{old_literal} / {new_literals}",
        )

    endpoint_alternatives = (
        ("translation 0 -0.062 0", "translation 0.19 -0.111 0"),
        ("translation 0 0.062 0", "translation 0.19 0.111 0"),
    )
    for old_literal, new_literal in endpoint_alternatives:
        _require(
            old_literal in robot or new_literal in robot,
            f"侧向平移常量缺失：{old_literal} / {new_literal}",
        )

    return (
        counts["HingeJoint"],
        counts["RotationalMotor"],
        counts["PositionSensor"],
        counts["abad_axis"],
        counts["pitch_axis"],
    )


def _validate_conversion(
    generated: str,
    source_world: str,
) -> tuple[int, float, float]:
    """核对 anchor、endPoint、旧零位与 URDF 零位，误差必须小于 1 mm。"""

    official_robot = _extract_braced_block(source_world, r"^Robot \{$")
    legacy_blocks = _joint_blocks(official_robot, legacy=True)
    generated_blocks = _joint_blocks(generated, legacy=False)
    expected_endpoints = _expected_endpoint_translations(legacy_blocks)
    generated_endpoints = {
        str(item["motor"]): item["child_translation"]
        for item in generated_blocks
    }
    _require(
        set(generated_endpoints) == set(expected_endpoints),
        "生成器 endPoint motor 集合不一致",
    )
    endpoint_error = max(
        math.dist(generated_endpoints[motor], expected_endpoints[motor])
        for motor in expected_endpoints
    )
    legacy_zero = _zero_world_positions(
        legacy_blocks,
        expected_endpoints,
        legacy=True,
    )
    generated_zero = _zero_world_positions(
        generated_blocks,
        generated_endpoints,
        legacy=False,
    )
    urdf_zero = _urdf_zero_world_positions(MINI_CHEETAH_URDF)
    zero_error = max(
        _max_vector_error(generated_zero, legacy_zero),
        _max_vector_error(generated_zero, urdf_zero),
    )
    _require(
        endpoint_error < 0.001,
        f"endPoint 与官方换算误差超过 1 mm：{endpoint_error:.9f} m",
    )
    _require(
        zero_error < 0.001,
        f"零位世界坐标误差超过 1 mm：{zero_error:.9f} m",
    )
    return len(generated_blocks), endpoint_error, zero_error


def _transform_official_robot() -> tuple[str, str]:
    """提取官方 Robot 并执行全部 R2025a 兼容转换。"""

    source_world = DEFAULT_SOURCE_WORLD.read_text(encoding="utf-8")
    robot = _extract_braced_block(source_world, r"^Robot \{$")
    robot = _replace_official_visual_shapes(robot)
    robot = _convert_child_to_endpoint(robot)
    robot = _move_nested_joints_into_children(robot)
    robot = _clean_legacy_fields(robot)
    robot = _add_bounding_objects(robot)
    robot = _convert_inertia_matrices(robot)
    source_translation_count = len(
        re.findall(r"(?m)^  translation 0 0 0\.38$", source_world),
    )
    _require(
        source_translation_count == 1,
        "官方源 Robot 根 translation 不是 0.38",
    )
    robot, translation_count = re.subn(
        r"(?m)^(  translation )0 0 0\.38$",
        r"\g<1>0 0 0.40",
        robot,
        count=1,
    )
    _require(
        translation_count == 1,
        "验收场景根 translation 覆写为 0.40 失败",
    )
    robot = robot.replace(
        "Robot {\n  translation 0 0 0.40\n",
        "DEF MINI_CHEETAH Robot {\n"
        "  translation 0 0 0.40\n"
        "  rotation 0 0 1 0\n",
        1,
    )
    robot = _insert_explicit_joint_positions(robot)
    robot, controller_count = re.subn(
        r'(?m)^  controller "mini_cheetah_controller"$',
        '  controller "<none>"',
        robot,
        count=1,
    )
    _require(controller_count == 1, "官方 Robot controller 禁用失败")
    return robot, source_world


def validate_models() -> ValidationReport:
    """对官方 world、C++ 构造、URDF 和转换结果执行交叉自检。"""

    for path in (DEFAULT_SOURCE_WORLD, MINI_CHEETAH_HEADER, MINI_CHEETAH_URDF):
        _require(path.is_file(), f"参考文件不存在：{path}")

    source_world = DEFAULT_SOURCE_WORLD.read_text(encoding="utf-8")
    official_robot = _extract_braced_block(source_world, r"^Robot \{$")
    official_counts = _validate_official_robot(official_robot, generated=False)
    cpp_body_mass = _validate_cpp_model(MINI_CHEETAH_HEADER)
    urdf_joints, urdf_body_mass = _validate_urdf(MINI_CHEETAH_URDF)
    generated, _ = _transform_official_robot()
    endpoint_count, endpoint_error, zero_error = _validate_conversion(
        generated,
        source_world,
    )
    _assert_close(cpp_body_mass, 3.3, "C++ body mass")
    _assert_close(urdf_body_mass, 3.3, "URDF body mass")
    return ValidationReport(
        *official_counts,
        urdf_joints,
        cpp_body_mass,
        urdf_body_mass,
        endpoint_count,
        endpoint_error,
        zero_error,
    )


def _insert_explicit_joint_positions(robot: str) -> str:
    """为全部 HingeJointParameters 显式写入 position 0。"""

    pattern = re.compile(
        r"(?P<indent>[ \t]*)jointParameters HingeJointParameters \{\n"
    )
    inserted: list[str] = []
    last = 0
    count = 0
    for match in pattern.finditer(robot):
        indent = match.group("indent")
        inserted.append(robot[last : match.end()])
        inserted.append(f"{indent}  position 0\n")
        last = match.end()
        count += 1
    _require(count == 12, f"显式 position 插入数量错误：{count}")
    inserted.append(robot[last:])
    return "".join(inserted)


def generate_robot() -> str:
    """生成验收 world 所需的官方 Robot 文本。"""

    validate_models()
    robot, source_world = _transform_official_robot()
    _require(
        robot.count('controller "<none>"') == 1,
        "生成结果控制器字段数量错误",
    )

    generated_counts = _validate_official_robot(robot, generated=True)
    _require(generated_counts[0] == 12, "生成结果 HingeJoint 数量变化")
    endpoint_count, endpoint_error, zero_error = _validate_conversion(
        robot,
        source_world,
    )
    _require(endpoint_count == 12, "生成结果 endPoint 数量错误")
    _require(endpoint_error < 0.001, "生成结果 endPoint 转换误差超限")
    _require(zero_error < 0.001, "生成结果零位坐标误差超限")
    _require(
        robot.count("position 0") == 12,
        f"生成结果 position 0 数量错误：{robot.count('position 0')}",
    )
    _require("yobogo" not in robot.lower(), "生成结果混入 YoboGo 模型")
    return robot


def build_parser() -> argparse.ArgumentParser:
    """构建命令行参数。"""

    parser = argparse.ArgumentParser(
        description="生成官方 Mini Cheetah 验收 Robot 文本并执行自检。"
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="将 Robot 文本写入指定新文件；省略时打印到标准输出。",
    )
    parser.add_argument(
        "--report-json",
        action="store_true",
        help="额外在标准错误输出 JSON 自检摘要。",
    )
    return parser


def main() -> int:
    """命令行入口。"""

    args = build_parser().parse_args()
    try:
        robot = generate_robot()
        report = validate_models().as_dict()
    except (AssertionError, OSError, ET.ParseError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1

    if args.report_json:
        import json

        print(json.dumps({"status": "PASS", **report}, sort_keys=True), file=sys.stderr)

    if args.output is None:
        print(robot)
        return 0

    output = args.output.resolve()
    _require(
        PROJECT_ROOT in output.parents,
        f"输出必须位于本验收目录内：{output}",
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(robot + "\n", encoding="utf-8")
    print(f"PASS: 已生成 {output}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
