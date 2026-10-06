#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""joint_sweep_test.wbt 的静态契约测试。"""

from __future__ import annotations

import re
import unittest
from pathlib import Path


WEBOTS_DIR = Path(__file__).resolve().parents[2]
WORLD_PATH = WEBOTS_DIR / "worlds" / "joint_sweep_test.wbt"
TEMPLATE_PATH = WEBOTS_DIR / "worlds" / "parkour_dev.wbt"


def _without_comments(text: str) -> str:
    """移除行注释，保留节点和字符串内容供计数。"""

    return "\n".join(line.split("#", 1)[0] for line in text.splitlines())


def _top_level_blocks(text: str) -> list[tuple[str, str]]:
    """按大括号深度提取顶层节点，兼容带 DEF 名称的节点。"""

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


def _contact_properties_blocks(world_info: str) -> list[tuple[str, str]]:
    """提取 WorldInfo.contactProperties 中的全部 ContactProperties 节点。"""

    match = re.search(
        r"(?ms)^  contactProperties \[\n(.*?)^  \]\s*$",
        world_info,
    )
    if not match:
        raise AssertionError("缺少 WorldInfo.contactProperties 字段")
    return _top_level_blocks(match.group(1))


def _block_by_name(blocks: list[tuple[str, str]], node_name: str) -> str:
    """返回指定顶层节点文本；缺失时给出明确错误。"""

    for name, block in blocks:
        if name == node_name:
            return block
    raise AssertionError(f"缺少顶层节点：{node_name}")


def _first_float_vector(block: str, field: str) -> tuple[float, ...]:
    """读取节点中首个指定字段的数值向量，兼容可选方括号。"""

    match = re.search(
        rf"(?m)^\s*{re.escape(field)}\s+(?:\[)?"
        rf"\s*([-\d.eE+]+(?:\s+[-\d.eE+]+)*)\s*(?:\])?",
        block,
    )
    if not match:
        raise AssertionError(f"缺少字段：{field}")
    return tuple(float(value) for value in match.group(1).split())


def _first_string(block: str, field: str) -> str:
    """读取节点中首个指定字段的字符串值。"""

    match = re.search(
        rf'(?m)^\s*{re.escape(field)}\s+"([^"]+)"',
        _without_comments(block),
    )
    if not match:
        raise AssertionError(f"缺少字段：{field}")
    return match.group(1)


def _count_nodes(block: str, node_name: str) -> int:
    """统计指定 Webots 节点数量，忽略行注释。"""

    return len(
        re.findall(
            rf"(?m)^\s*(?:DEF\s+\w+\s+)?{re.escape(node_name)}\s*\{{",
            _without_comments(block),
        )
    )


class JointSweepWorldTest(unittest.TestCase):
    """验证仰躺关节扫描世界的设备、姿态和平地契约。"""

    @classmethod
    def setUpClass(cls) -> None:
        cls.world = WORLD_PATH.read_text(encoding="utf-8")
        cls.template = TEMPLATE_PATH.read_text(encoding="utf-8")
        cls.clean_world = _without_comments(cls.world)
        cls.blocks = _top_level_blocks(cls.world)
        cls.template_blocks = _top_level_blocks(cls.template)

    def test_header_world_info_and_single_viewpoint_are_preserved(self) -> None:
        """核对 R2025a 头、4 ms 步长、WorldInfo 兜底外契约和唯一视角。"""

        self.assertEqual(
            self.world.splitlines()[0],
            "#VRML_SIM R2025a utf8",
        )
        world_info = _block_by_name(self.blocks, "WorldInfo")
        template_world_info = _block_by_name(self.template_blocks, "WorldInfo")
        preserved_world_info = world_info
        for name, contact_block in _contact_properties_blocks(world_info):
            if name != "ContactProperties":
                continue
            if (
                _first_string(contact_block, "material1") == "default"
                and _first_string(contact_block, "material2") == "default"
            ):
                preserved_world_info = preserved_world_info.replace(
                    contact_block,
                    "",
                    1,
                )
        self.assertEqual(preserved_world_info, template_world_info)
        self.assertRegex(world_info, r"(?m)^  basicTimeStep 4\s*$")
        self.assertIn("contactProperties [", world_info)

        viewpoints = [
            block
            for name, block in self.blocks
            if name == "Viewpoint"
        ]
        self.assertEqual(len(viewpoints), 1)
        self.assertEqual(_count_nodes(self.clean_world, "Viewpoint"), 1)
        self.assertIn("DEF VP_FOLLOW Viewpoint", viewpoints[0])
        self.assertEqual(
            viewpoints[0],
            _block_by_name(self.template_blocks, "Viewpoint"),
        )

    def test_default_contact_properties_are_conservative(self) -> None:
        """核对 default/default 显式采用零回弹和 body/default 邻近参数。"""

        world_info = _block_by_name(self.blocks, "WorldInfo")
        default_contacts = [
            contact_block
            for name, contact_block in _contact_properties_blocks(world_info)
            if name == "ContactProperties"
            and _first_string(contact_block, "material1") == "default"
            and _first_string(contact_block, "material2") == "default"
        ]
        self.assertEqual(len(default_contacts), 1)
        default_contact = default_contacts[0]
        self.assertEqual(
            _first_float_vector(default_contact, "coulombFriction"),
            (0.8,),
        )
        self.assertEqual(
            _first_float_vector(default_contact, "bounce"),
            (0.0,),
        )
        self.assertEqual(
            _first_float_vector(default_contact, "bounceVelocity"),
            (0.0,),
        )
        self.assertEqual(
            _first_float_vector(default_contact, "softERP"),
            (0.2,),
        )
        self.assertEqual(
            _first_float_vector(default_contact, "softCFM"),
            (0.001,),
        )

    def test_robot_pose_controller_and_supervisor_are_expected(self) -> None:
        """核对规定姿态、控制器及禁用内部自碰撞后的机器人契约。"""

        robot = _block_by_name(self.blocks, "Robot")
        template_robot = _block_by_name(self.template_blocks, "Robot")
        expected_robot, translation_count = re.subn(
            r"(?m)^  translation 0 0 0\.26$",
            "  translation 0 0 0.15",
            template_robot,
            count=1,
        )
        expected_robot, rotation_count = re.subn(
            r"(?m)^  rotation 0 0 1 0$",
            "  rotation 1 0 0 1.5708",
            expected_robot,
            count=1,
        )
        expected_robot, controller_count = re.subn(
            r'(?m)^  controller "rl_agent"$',
            '  controller "joint_sweep_test"',
            expected_robot,
            count=1,
        )
        expected_robot, self_collision_count = re.subn(
            r"(?m)^  selfCollision TRUE$",
            "  selfCollision FALSE",
            expected_robot,
            count=1,
        )
        self.assertEqual(
            (
                translation_count,
                rotation_count,
                controller_count,
                self_collision_count,
            ),
            (1, 1, 1, 1),
        )
        self.assertEqual(robot, expected_robot)
        translation = _first_float_vector(robot, "translation")
        self.assertEqual(len(translation), 3)
        self.assertGreaterEqual(translation[2], 0.15)
        self.assertEqual(
            _first_float_vector(robot, "rotation"),
            (1.0, 0.0, 0.0, 1.5708),
        )
        self.assertRegex(
            robot,
            r"(?m)^  rotation 1 0 0 1\.5708\s*$",
        )
        self.assertRegex(
            robot,
            r'(?m)^  controller "joint_sweep_test"\s*$',
        )
        self.assertRegex(robot, r"(?m)^  supervisor TRUE\s*$")
        # A/B 已证实 TRUE 会让仰躺机器人触发内部接触并低空弹射。
        # 本测试世界禁用内部自碰撞，因此不验证机器人各部件间的自碰撞。
        self.assertRegex(robot, r"(?m)^  selfCollision FALSE\s*$")
        self.assertNotRegex(robot, r"(?m)^  selfCollision TRUE\s*$")
        self.assertEqual(
            _count_nodes(self.clean_world, "Robot"),
            1,
        )

    def test_robot_has_twelve_motors_sensors_and_four_feet(self) -> None:
        """核对 12 电机、12 关节传感器、4 足端和机体惯性传感器。"""

        robot = _block_by_name(self.blocks, "Robot")
        expected_counts = {
            "RotationalMotor": 12,
            "PositionSensor": 12,
            "TouchSensor": 4,
            "InertialUnit": 1,
            "Gyro": 1,
            "Accelerometer": 1,
        }
        for node_name, expected_count in expected_counts.items():
            with self.subTest(node=node_name):
                self.assertEqual(
                    _count_nodes(robot, node_name),
                    expected_count,
                )

        motor_names = set(
            re.findall(r'name\s+"((?:fr|fl|hr|hl)_(?:abd|hip|kn)_motor)"', robot)
        )
        sensor_names = set(
            re.findall(r'name\s+"((?:fr|fl|hr|hl)_(?:abd|hip|kn)_sensor)"', robot)
        )
        foot_names = set(
            re.findall(r'name\s+"((?:fr|fl|hr|hl)_foot_touch)"', robot)
        )
        self.assertEqual(len(motor_names), 12)
        self.assertEqual(len(sensor_names), 12)
        self.assertEqual(
            foot_names,
            {
                "fr_foot_touch",
                "fl_foot_touch",
                "hr_foot_touch",
                "hl_foot_touch",
            },
        )

    def test_no_rl_or_remote_resources_are_referenced(self) -> None:
        """禁止 RL 环境引用、外部协议和远程资源。"""

        forbidden_patterns = {
            "RL 环境引用": r"(?i)\b(?:RL_AGENT|checkpoint|rl_agent)\b",
            "外部协议声明": r"(?i)\bEXTERNPROTO\b",
            "远程 URL": r"(?i)\b(?:https?|ftp|file|data)://",
            "Windows 绝对路径": r"[A-Za-z]:[\\/]",
            "Unix 绝对文件路径": (
                r"(?<![\w:])/(?:[A-Za-z0-9_.-]+)"
                r"(?:/[A-Za-z0-9_.-]+)+"
            ),
        }
        for description, pattern in forbidden_patterns.items():
            with self.subTest(禁止项=description):
                self.assertNotRegex(self.world, pattern)

    def test_spawn_area_is_a_single_flat_floor(self) -> None:
        """确认出生区只有上表面位于 z=0 的静态平地。"""

        solids = [
            block
            for name, block in self.blocks
            if name == "Solid"
        ]
        self.assertEqual(len(solids), 1)
        floor = solids[0]
        self.assertRegex(floor, r'(?m)^\s*name\s+"floor"\s*$')
        self.assertNotRegex(floor, r"(?m)^\s*physics\s+Physics\s*\{")
        self.assertIn("boundingObject Box", floor)
        self.assertNotIn("hurdle", self.world, "测试世界不得包含障碍")

        translation = _first_float_vector(floor, "translation")
        geometry = re.search(
            r"\bBox\s*\{([^{}]*)\}",
            floor,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(geometry)
        size = _first_float_vector(geometry.group(1), "size")
        self.assertEqual(len(translation), 3)
        self.assertEqual(len(size), 3)
        self.assertAlmostEqual(translation[2] + size[2] / 2.0, 0.0)

    def test_world_delimiters_are_balanced(self) -> None:
        """静态核对大括号和方括号成对且不发生负深度。"""

        stack: list[str] = []
        closing_to_opening = {"}": "{", "]": "["}
        for character in _without_comments(self.world):
            if character in "{[":
                stack.append(character)
            elif character in closing_to_opening:
                self.assertTrue(
                    stack,
                    f"出现多余的闭合符号：{character}",
                )
                self.assertEqual(
                    stack.pop(),
                    closing_to_opening[character],
                    f"括号类型不匹配：{character}",
                )
        self.assertEqual(stack, [], "存在未闭合的括号")


if __name__ == "__main__":
    unittest.main()
