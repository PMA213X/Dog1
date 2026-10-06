#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""yobogo_terrain_test.wbt 的静态契约测试。"""

from __future__ import annotations

import math
import re
import unittest
from pathlib import Path


WEBOTS_DIR = Path(__file__).resolve().parents[2]
WORLD_PATH = WEBOTS_DIR / "worlds" / "yobogo_terrain_test.wbt"
TEMPLATE_PATH = WEBOTS_DIR / "worlds" / "parkour_dev.wbt"


def _without_comments(text: str) -> str:
    """移除行注释，保留节点和字符串内容供计数。"""

    return "\n".join(line.split("#", 1)[0] for line in text.splitlines())


def _top_level_blocks(text: str) -> list[tuple[str, str]]:
    """按大括号深度提取顶层节点，兼容 WBT 的普通节点写法。"""

    blocks: list[tuple[str, str]] = []
    current_name: str | None = None
    current_lines: list[str] = []
    depth = 0

    for line in text.splitlines():
        code = line.split("#", 1)[0]
        if current_name is None and depth == 0:
            match = re.match(
                r"\s*(?:DEF\s+\w+\s+)?([A-Za-z]\w*)\s*\{",
                code,
            )
            if match:
                current_name = match.group(1)
                current_lines = [line]
        elif current_name is not None:
            current_lines.append(line)

        if current_name is not None:
            depth += code.count("{") - code.count("}")
            if depth == 0:
                blocks.append((current_name, "\n".join(current_lines) + "\n"))
                current_name = None
                current_lines = []

    return blocks


def _block_by_name(blocks: list[tuple[str, str]], node_name: str) -> str:
    """返回指定顶层节点文本；缺失时给出明确错误。"""

    for name, block in blocks:
        if name == node_name:
            return block
    raise AssertionError(f"缺少顶层节点：{node_name}")


def _first_float_vector(block: str, field: str) -> tuple[float, ...]:
    """读取节点中首个指定字段的数值向量。"""

    match = re.search(
        rf"(?m)^\s*{re.escape(field)}\s+([-\d.eE+]+(?:\s+[-\d.eE+]+)*)",
        block,
    )
    if not match:
        raise AssertionError(f"缺少字段：{field}")
    return tuple(float(value) for value in match.group(1).split())


def _first_geometry(block: str) -> tuple[str, tuple[float, ...]]:
    """读取首个 Box/Cylinder 几何及其尺寸。"""

    match = re.search(
        r"\b(Box|Cylinder)\s*\{([^{}]*)\}",
        block,
        flags=re.DOTALL,
    )
    if not match:
        raise AssertionError("地形缺少内置几何")
    geometry_name = match.group(1)
    body = match.group(2)
    if geometry_name == "Box":
        size = _first_float_vector(body, "size")
        if len(size) != 3:
            raise AssertionError(f"Box size 长度错误：{size}")
        return geometry_name, size

    radius = _first_float_vector(body, "radius")
    height = _first_float_vector(body, "height")
    return geometry_name, (radius[0], radius[0], height[0])


def _rotation_matrix(block: str) -> list[list[float]] | None:
    """将首个轴角 rotation 转为 3x3 矩阵；无 rotation 时返回 None。"""

    match = re.search(
        r"(?m)^\s*rotation\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+"
        r"([-\d.eE+]+)\s+([-\d.eE+]+)",
        block,
    )
    if not match:
        return None
    x, y, z, angle = (float(value) for value in match.groups())
    length = math.sqrt(x * x + y * y + z * z)
    if length == 0:
        raise AssertionError("rotation 轴长度不能为零")
    x, y, z = x / length, y / length, z / length
    cosine = math.cos(angle)
    sine = math.sin(angle)
    one_minus = 1.0 - cosine
    return [
        [
            cosine + x * x * one_minus,
            x * y * one_minus - z * sine,
            x * z * one_minus + y * sine,
        ],
        [
            y * x * one_minus + z * sine,
            cosine + y * y * one_minus,
            y * z * one_minus - x * sine,
        ],
        [
            z * x * one_minus - y * sine,
            z * y * one_minus + x * sine,
            cosine + z * z * one_minus,
        ],
    ]


def _top_height(block: str) -> float:
    """按平移、几何尺寸和可选旋转计算单个地形块最高点。"""

    translation = _first_float_vector(block, "translation")
    geometry_name, size = _first_geometry(block)
    if len(translation) != 3:
        raise AssertionError(f"translation 长度错误：{translation}")

    matrix = _rotation_matrix(block)
    if matrix is None:
        return translation[2] + size[2] / 2.0

    highest = -math.inf
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            for z_sign in (-1.0, 1.0):
                local = (
                    x_sign * size[0] / 2.0,
                    y_sign * size[1] / 2.0,
                    z_sign * size[2] / 2.0,
                )
                world_z = (
                    matrix[2][0] * local[0]
                    + matrix[2][1] * local[1]
                    + matrix[2][2] * local[2]
                    + translation[2]
                )
                highest = max(highest, world_z)
    return highest


class YobogoTerrainWorldTest(unittest.TestCase):
    """验证新世界保留 YoboGo 契约并使用规定的低障碍地形。"""

    @classmethod
    def setUpClass(cls) -> None:
        cls.world = WORLD_PATH.read_text(encoding="utf-8")
        cls.template = TEMPLATE_PATH.read_text(encoding="utf-8")
        cls.clean_world = _without_comments(cls.world)
        cls.blocks = _top_level_blocks(cls.world)
        cls.template_blocks = _top_level_blocks(cls.template)

    def test_header_world_info_and_viewpoint_are_preserved(self) -> None:
        self.assertEqual(
            self.world.splitlines()[0],
            "#VRML_SIM R2025a utf8",
        )
        world_info = _block_by_name(self.blocks, "WorldInfo")
        template_world_info = _block_by_name(self.template_blocks, "WorldInfo")
        self.assertEqual(world_info, template_world_info)
        self.assertRegex(world_info, r"(?m)^  basicTimeStep 4\s*$")

        viewpoints = [
            block
            for name, block in self.blocks
            if name == "Viewpoint"
        ]
        self.assertEqual(len(viewpoints), 1)
        self.assertEqual(
            len(
                re.findall(
                    r"(?m)^\s*(?:DEF\s+\w+\s+)?Viewpoint\s*\{",
                    self.clean_world,
                )
            ),
            1,
        )
        self.assertIn("DEF VP_FOLLOW Viewpoint", viewpoints[0])
        self.assertEqual(
            viewpoints[0],
            _block_by_name(self.template_blocks, "Viewpoint"),
        )

    def test_robot_contract_is_unchanged(self) -> None:
        robot = _block_by_name(self.blocks, "Robot")
        template_robot = _block_by_name(self.template_blocks, "Robot")
        self.assertEqual(robot, template_robot)
        self.assertRegex(robot, r'(?m)^  translation 0 0 0\.26\s*$')
        self.assertRegex(robot, r"(?m)^  rotation 0 0 1 0\s*$")
        self.assertRegex(robot, r'(?m)^  controller "rl_agent"\s*$')
        self.assertRegex(robot, r"(?m)^  supervisor TRUE\s*$")

        count_pattern = lambda name: len(
            re.findall(
                rf"(?m)^\s*(?:DEF\s+\w+\s+)?{name}\s*\{{",
                self.clean_world,
            )
        )
        self.assertEqual(count_pattern("RotationalMotor"), 12)
        self.assertEqual(count_pattern("PositionSensor"), 12)
        self.assertEqual(count_pattern("TouchSensor"), 4)
        self.assertEqual(count_pattern("InertialUnit"), 1)

        foot_names = set(
            re.findall(r'name\s+"((?:fr|fl|hr|hl)_foot_touch)"', self.clean_world)
        )
        self.assertEqual(
            foot_names,
            {
                "fr_foot_touch",
                "fl_foot_touch",
                "hr_foot_touch",
                "hl_foot_touch",
            },
        )
        controllers = re.findall(r'(?m)^\s*controller\s+"([^"]+)"', self.clean_world)
        self.assertEqual(controllers, ["rl_agent"])

    def test_no_external_resources_or_absolute_paths(self) -> None:
        self.assertNotRegex(self.world, r"(?i)\bEXTERNPROTO\b")
        self.assertNotRegex(self.world, r"(?i)\bhttps?://")
        self.assertNotRegex(self.world, r"\b[A-Za-z]:[\\/]")
        self.assertNotRegex(
            self.world,
            r"(?<![\w:])/(?:[A-Za-z0-9_.-]+)(?:/[A-Za-z0-9_.-]+)*",
        )

    def test_terrain_blocks_are_bounded_builtin_geometry(self) -> None:
        robot_index = next(
            index
            for index, (name, _) in enumerate(self.blocks)
            if name == "Robot"
        )
        terrain_blocks = [
            block
            for name, block in self.blocks[:robot_index]
            if name == "Solid"
        ]
        names = set()
        allowed_nodes = {
            "Solid",
            "Shape",
            "PBRAppearance",
            "Box",
            "Cylinder",
            "Transform",
        }
        for block in terrain_blocks:
            name_match = re.search(r'(?m)^\s*name\s+"([^"]+)"', block)
            self.assertIsNotNone(name_match)
            names.add(name_match.group(1))
            self.assertIn("boundingObject", block)
            node_names = set(
                re.findall(
                    r"(?m)^\s*(?:DEF\s+\w+\s+)?([A-Za-z]\w*)\s*\{",
                    _without_comments(block),
                )
            )
            self.assertTrue(node_names <= allowed_nodes, node_names)
            self.assertNotRegex(block, r"\b(?:Mesh|Plane|IndexedFaceSet|PROTO)\b")

        expected_names = {
            "floor",
            "hurdle",
            "stair_08",
            "stair_16",
            "stair_24",
            "ramp",
            "rough_pad_04",
            "rough_pad_06",
            "rough_pad_08",
            "rough_pad_05",
            "rough_pad_07",
            "stop_zone",
        }
        self.assertEqual(names, expected_names)

        heights = {}
        for block in terrain_blocks:
            name = re.search(r'(?m)^\s*name\s+"([^"]+)"', block).group(1)
            heights[name] = _top_height(block)

        self.assertAlmostEqual(heights["floor"], 0.0, places=6)
        self.assertAlmostEqual(heights["hurdle"], 0.10, places=6)
        self.assertAlmostEqual(heights["stair_08"], 0.08, places=6)
        self.assertAlmostEqual(heights["stair_16"], 0.16, places=6)
        self.assertAlmostEqual(heights["stair_24"], 0.24, places=6)
        self.assertAlmostEqual(heights["ramp"], 0.1976, places=3)
        self.assertAlmostEqual(heights["rough_pad_04"], 0.04, places=6)
        self.assertAlmostEqual(heights["rough_pad_06"], 0.06, places=6)
        self.assertAlmostEqual(heights["rough_pad_08"], 0.08, places=6)
        self.assertAlmostEqual(heights["rough_pad_05"], 0.05, places=6)
        self.assertAlmostEqual(heights["rough_pad_07"], 0.07, places=6)
        self.assertAlmostEqual(heights["stop_zone"], 0.0, places=6)
        self.assertLessEqual(max(heights.values()), 0.24 + 1e-9)

        translations = {}
        for block in terrain_blocks:
            name = re.search(r'(?m)^\s*name\s+"([^"]+)"', block).group(1)
            translations[name] = _first_float_vector(block, "translation")
        self.assertAlmostEqual(translations["hurdle"][0], 1.5, places=6)
        self.assertEqual(
            [translations[name][0] for name in ("stair_08", "stair_16", "stair_24")],
            [2.85, 3.15, 3.45],
        )
        self.assertAlmostEqual(translations["ramp"][0], 5.0, places=6)
        self.assertEqual(
            [
                translations[name][0]
                for name in (
                    "rough_pad_04",
                    "rough_pad_06",
                    "rough_pad_08",
                    "rough_pad_05",
                    "rough_pad_07",
                )
            ],
            [6.7, 7.0, 7.3, 7.6, 7.9],
        )
        self.assertAlmostEqual(translations["stop_zone"][0], 9.0, places=6)

        # 出生区只有基础地面和 1.5m 矮障碍，其余障碍从 2.7m 后开始。
        early_names = {
            name
            for name, translation in translations.items()
            if name != "floor" and translation[0] < 2.0
        }
        self.assertEqual(early_names, {"hurdle"})


if __name__ == "__main__":
    unittest.main()
