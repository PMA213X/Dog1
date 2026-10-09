#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""world 静态隔离与训练 CLI 测试。"""

from __future__ import annotations

import re
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
PROJECT = ROOT.parent
for path in (ROOT,):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import rl.contract as contract
from rl.evaluate import parse_args as parse_eval_args
from rl.run_stages import (
    _port_listeners,
    _terminate_process_group,
    _webots_controller_process_count,
    _webots_process_count,
    dry_run_commands,
    run_gate,
    run_gate_batch,
    run_training,
    stage_command,
)
from rl.train import parse_args as parse_train_args


class WorldStaticTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.world = (ROOT / "worlds" / "flat_move_jump_rl.wbt").read_text(encoding="utf-8")
        cls.eval_world = (
            ROOT / "worlds" / "flat_move_jump_rl_eval.wbt"
        ).read_text(encoding="utf-8")
        cls.template = (ROOT / "worlds" / "posture_timeline_test.wbt").read_text(encoding="utf-8")

    @staticmethod
    def _robot_blocks(text: str) -> list[str]:
        blocks: list[str] = []
        cursor = 0
        while True:
            match = re.search(r"DEF\s+\S+\s+Robot\s+\{", text[cursor:])
            if match is None:
                return blocks
            start = cursor + match.start()
            opening = text.index("{", start)
            depth = 0
            for index in range(opening, len(text)):
                if text[index] == "{":
                    depth += 1
                elif text[index] == "}":
                    depth -= 1
                    if depth == 0:
                        blocks.append(text[start:index + 1])
                        cursor = index + 1
                        break
            else:
                raise AssertionError("Robot 花括号未闭合")

    @staticmethod
    def _node_block(text: str, marker: str) -> str:
        """按花括号提取 marker 起始的单个节点块。"""
        start = text.index(marker)
        opening = text.index("{", start)
        depth = 0
        for index in range(opening, len(text)):
            if text[index] == "{":
                depth += 1
            elif text[index] == "}":
                depth -= 1
                if depth == 0:
                    return text[start:index + 1]
        raise AssertionError(f"{marker} 花括号未闭合")

    @staticmethod
    def _contact_properties(text: str) -> str:
        match = re.search(r"contactProperties\s*\[(.*?)\n\s*\]", text, re.S)
        if match is None:
            raise AssertionError("缺少 contactProperties 块")
        return re.sub(r"\s+", " ", match.group(1)).strip()

    def test_new_world_is_flat_and_isolated(self) -> None:
        self.assertIn("basicTimeStep 4", self.world)
        self.assertIn("optimalThreadCount 1", self.world)
        self.assertIn('name "floor"', self.world)
        self.assertIn("size 20 20 0.1", self.world)
        robot_blocks = self._robot_blocks(self.world)
        self.assertEqual(len(robot_blocks), 4)
        self.assertEqual(self.world.count('controller "<extern>"'), 4)
        self.assertEqual(self.world.count("controllerArgs []"), 4)
        self.assertEqual(self.world.count("synchronization TRUE"), 4)
        self.assertEqual(self.world.count("supervisor TRUE"), 4)
        for worker_id, birth in enumerate(contract.BIRTH_POSITIONS):
            self.assertIn(f'name "mini_cheetah_{worker_id}"', robot_blocks[worker_id])
            self.assertIn(
                "translation "
                + " ".join(str(int(value)) if value.is_integer() else str(value) for value in birth),
                robot_blocks[worker_id],
            )
        self.assertNotIn('controller "rl_agent"', self.world)
        self.assertNotIn("acceptance_supervisor", self.world)
        self.assertNotIn(contract.FORBIDDEN_DIRECTORY, self.world)
        self.assertIn("geometry Mesh", self.world)
        self.assertNotIn("ElevationGrid", self.world)
        self.assertNotIn("stairs", self.world.lower())
        self.assertNotIn("hurdle", self.world.lower())

    def test_floor_visual_grid_is_identical_and_non_physical(self) -> None:
        """两个 world 使用同一视觉网格，且不改变平地物理与摩擦语义。"""
        floor = self._node_block(self.world, "DEF FLAT_FLOOR Solid {")
        eval_floor = self._node_block(self.eval_world, "DEF FLAT_FLOOR Solid {")
        self.assertEqual(floor, eval_floor)
        self.assertIn("translation 0 0 -0.05", floor)
        self.assertIn("baseColor 0.22 0.32 0.38", floor)
        self.assertIn("geometry IndexedLineSet", floor)
        self.assertIn("castShadows FALSE", floor)
        self.assertIn("diffuseColor 0.1 0.78 0.95", floor)
        self.assertIn("emissiveColor 0.02 0.22 0.3", floor)

        # 物理面仍是原 20 m × 20 m × 0.1 m Box；网格只位于 children。
        self.assertEqual(floor.count("geometry Box"), 1)
        self.assertEqual(floor.count("boundingObject Box"), 1)
        self.assertEqual(floor.count("size 20 20 0.1"), 2)
        bounding = self._node_block(floor, "boundingObject Box {")
        self.assertIn("size 20 20 0.1", bounding)
        self.assertNotIn("Shape", bounding)
        self.assertNotIn("IndexedLineSet", bounding)
        self.assertNotIn("physics Physics", floor)
        self.assertNotIn("contactMaterial", floor)

        point_array = re.search(r"point\s*\[(.*?)\]", floor, re.S)
        index_array = re.search(r"coordIndex\s*\[(.*?)\]", floor, re.S)
        self.assertIsNotNone(point_array)
        self.assertIsNotNone(index_array)
        number = re.compile(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?")
        point_numbers = number.findall(point_array.group(1))
        points = [
            tuple(float(value) for value in point_numbers[index:index + 3])
            for index in range(0, len(point_numbers), 3)
        ]
        indices = [int(value) for value in number.findall(index_array.group(1))]
        self.assertEqual(len(points), 84)
        self.assertTrue(all(point[2] == 0.0502 for point in points))
        self.assertEqual(len(indices), 126)
        self.assertEqual(indices.count(-1), 42)
        self.assertTrue(all(value == -1 or 0 <= value < 84 for value in indices))
        self.assertTrue(all(indices[index + 2] == -1 for index in range(0, 126, 3)))

        line_pairs = [
            (points[indices[index]], points[indices[index + 1]])
            for index in range(0, 126, 3)
        ]
        horizontal = [pair for pair in line_pairs if pair[0][1] == pair[1][1]]
        vertical = [pair for pair in line_pairs if pair[0][0] == pair[1][0]]
        self.assertEqual(len(horizontal), 21)
        self.assertEqual(len(vertical), 21)
        self.assertEqual({pair[0][1] for pair in horizontal}, set(range(-10, 11)))
        self.assertEqual({pair[0][0] for pair in vertical}, set(range(-10, 11)))

        template_contact = self._contact_properties(self.template)
        self.assertEqual(self._contact_properties(self.world), template_contact)
        self.assertEqual(self._contact_properties(self.eval_world), template_contact)
        self.assertEqual(template_contact.count("coulombFriction [ 0.9 ]"), 4)
        self.assertEqual(template_contact.count("coulombFriction [ 0.8 ]"), 2)

    def test_local_mesh_assets_are_official_only(self) -> None:
        urls = re.findall(r'url\s*\[\s*"([^"]+)"\s*\]', self.world)
        self.assertEqual(len(urls), 4 * 13)
        counts: dict[str, int] = {}
        for url in urls:
            self.assertTrue(url.startswith("../assets/meshes/"), url)
            self.assertNotIn(contract.FORBIDDEN_DIRECTORY, url)
            asset = (ROOT / "worlds" / url).resolve()
            self.assertTrue(asset.is_file(), asset)
            counts[asset.name] = counts.get(asset.name, 0) + 1
        self.assertEqual(
            counts,
            {
                "mini_body.dae": 4,
                "mini_abad.dae": 16,
                "mini_upper_link.dae": 16,
                "mini_lower_link.dae": 16,
            },
        )

    @staticmethod
    def _physical_signature(text: str) -> dict[str, list[str]]:
        return {
            "mass": re.findall(r"\bmass\s+([0-9.eE+-]+)", text),
            "center_of_mass": re.findall(
                r"centerOfMass\s+([-\d.eE+]+\s+[-\d.eE+]+\s+[-\d.eE+]+)",
                text,
            ),
            "inertia": [
                re.sub(r"\s+", " ", block).strip()
                for block in re.findall(r"inertiaMatrix\s*\[(.*?)\]", text, re.S)
            ],
            "anchors": re.findall(
                r"anchor\s+([-\d.eE+]+\s+[-\d.eE+]+\s+[-\d.eE+]+)",
                text,
            ),
            "axes": re.findall(
                r"axis\s+([-\d.eE+]+\s+[-\d.eE+]+\s+[-\d.eE+]+)",
                text,
            ),
            "initial_positions": re.findall(
                r"jointParameters HingeJointParameters\s*\{\s*position\s+([-\d.eE+]+)",
                text,
                re.S,
            ),
            "motors": re.findall(
                r'name "((?:fr|fl|hr|hl)_(?:abd|hip|kn)_motor)"',
                text,
            ),
            "sensors": re.findall(
                r'name "((?:fr|fl|hr|hl)_(?:abd|hip|kn)_sensor)"',
                text,
            ),
            "max_torque": re.findall(r"maxTorque\s+([0-9.]+)", text),
            "spring": re.findall(r"springConstant\s+([0-9.]+)", text),
            "damping": re.findall(r"dampingConstant\s+([0-9.]+)", text),
            "contact_materials": re.findall(r'contactMaterial\s+"([^"]+)"', text),
            "diffuse_colors": re.findall(r"diffuseColor\s+([0-9. ]+)", text),
            "bounding_count": [str(text.count("boundingObject"))],
        }

    def test_physical_fields_match_posture_template(self) -> None:
        template_robot = self._robot_blocks(self.template)[0]
        expected = self._physical_signature(template_robot)
        for robot in self._robot_blocks(self.world):
            actual = self._physical_signature(robot)
            # 小腿由 fr_*_toe 改为 default 是本次接触语义修复的预期差异；
            # 其余动力学字段必须仍与未修改模板一致。
            expected_without_materials = dict(expected)
            actual_without_materials = dict(actual)
            expected_without_materials.pop("contact_materials")
            actual_without_materials.pop("contact_materials")
            self.assertEqual(actual_without_materials, expected_without_materials)
            self.assertEqual(actual["contact_materials"][:4], ["default"] * 4)
            self.assertEqual(actual["contact_materials"][4:], ["body"])

    def test_shank_box_uses_default_and_toe_collision_is_offset(self) -> None:
        """小腿 Box 不得冒充 toe；足端球碰撞固定在局部末端。"""
        robot_blocks = self._robot_blocks(self.world)
        self.assertEqual(len(robot_blocks), 4)
        for robot in robot_blocks:
            for leg in ("fr", "fl", "hr", "hl"):
                shank_name = f'name "{leg}_shank_link"'
                shank_start = robot.index(shank_name)
                bounding_start = robot.index(
                    "boundingObject Group {",
                    shank_start,
                )
                bounding_open = robot.index("{", bounding_start)
                depth = 0
                for index in range(bounding_open, len(robot)):
                    if robot[index] == "{":
                        depth += 1
                    elif robot[index] == "}":
                        depth -= 1
                        if depth == 0:
                            bounding_end = index + 1
                            break
                else:
                    raise AssertionError(f"{leg} shank boundingObject 未闭合")
                bounding = robot[bounding_start:bounding_end]
                self.assertIn("Box { size 0.03 0.03 0.18 }", bounding)
                self.assertIn("Transform {", bounding)
                self.assertIn("translation 0 0 -0.09", bounding)
                self.assertIn("Sphere { radius 0.015 }", bounding)

                physics_start = robot.index("physics Physics {", shank_start)
                next_name = robot.find('name "', physics_start)
                shank_tail = robot[physics_start:next_name]
                self.assertIn('contactMaterial "default"', shank_tail)
                self.assertNotIn(f'contactMaterial "{leg}_toe"', shank_tail)

        # WorldInfo 仍保留四个 toe 的零回弹摩擦配置，地面安全语义不变。
        world_info = self._node_block(self.eval_world, "WorldInfo {")
        for leg in ("fr", "fl", "hr", "hl"):
            self.assertIn(f'material1 "{leg}_toe"', world_info)
        self.assertEqual(
            self.eval_world.count('contactMaterial "default"'),
            4,
        )

    def test_twelve_devices_and_axes(self) -> None:
        motors = re.findall(r'name "((?:fr|fl|hr|hl)_(?:abd|hip|kn)_motor)"', self.world)
        sensors = re.findall(r'name "((?:fr|fl|hr|hl)_(?:abd|hip|kn)_sensor)"', self.world)
        self.assertEqual(len(motors), 48)
        self.assertEqual(len(sensors), 48)
        self.assertEqual(len(set(motors)), 12)
        self.assertEqual(self.world.count("axis 1 0 0"), 16)
        self.assertEqual(self.world.count("axis 0 -1 0"), 32)
        self.assertEqual(self.world.count("maxTorque 20.0"), 48)
        self.assertEqual(self.world.count("springConstant 0"), 48)

    def test_eval_world_remains_single_robot(self) -> None:
        self.assertIn("optimalThreadCount 1", self.eval_world)
        self.assertIn("basicTimeStep 4", self.eval_world)
        self.assertIn('name "floor"', self.eval_world)
        self.assertIn("size 20 20 0.1", self.eval_world)
        robots = self._robot_blocks(self.eval_world)
        self.assertEqual(len(robots), 1)
        self.assertIn('name "mini_cheetah"', robots[0])
        self.assertIn("translation 0 0 0.45", robots[0])
        self.assertEqual(self.eval_world.count('controller "<extern>"'), 1)
        self.assertEqual(self.eval_world.count("controllerArgs []"), 1)
        self.assertEqual(self.eval_world.count("synchronization TRUE"), 1)
        self.assertEqual(self.eval_world.count("supervisor TRUE"), 1)
        self.assertEqual(
            contract.EVAL_WORLD_PATH,
            ROOT / "worlds" / "flat_move_jump_rl_eval.wbt",
        )

    def test_template_remains_unchanged(self) -> None:
        self.assertIn('controller "posture_timeline"', self.template)
        self.assertIn("controllerArgs []", self.template)
        self.assertNotIn('controller "rl_agent"', self.template)

    def test_new_source_has_no_deprecated_import_path(self) -> None:
        forbidden = contract.FORBIDDEN_DIRECTORY
        for path in list((ROOT / "rl").glob("*.py")) + list((ROOT / "controllers" / "rl_agent").glob("*.py")):
            text = path.read_text(encoding="utf-8")
            self.assertNotIn(forbidden, text, path)
            if path.name != "contract.py":
                for prefix in contract.FORBIDDEN_PREFIXES:
                    self.assertNotIn(prefix, text, path)


class ControllerStaticTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.controller = (
            ROOT / "controllers" / "rl_agent" / "rl_agent.py"
        ).read_text(encoding="utf-8")

    def test_hello_contains_all_lockstep_fields(self) -> None:
        self.assertIn('"contract": contract.CONTRACT_VERSION', self.controller)
        self.assertIn('"worker_id": self.worker_id', self.controller)
        self.assertIn('"robot_name": self.robot_name', self.controller)
        self.assertIn('"timestep": self.timestep', self.controller)
        self.assertIn("self.timestep = contract.WEBOTS_TIMESTEP_MS", self.controller)

    def test_robot_step_is_limited_to_reset_and_act_barriers(self) -> None:
        self.assertEqual(self.controller.count("robot.step("), 4)
        self.assertIn("RESET_SETTLE_STEPS = 125", self.controller)
        self.assertIn("RESET_HOLD_STEPS = 50", self.controller)
        self.assertIn("for _ in range(5):", self.controller)
        self.assertIn('if self.robot.step(4) < 0:', self.controller)

    def test_reset_consumes_worker_birth_position(self) -> None:
        self.assertIn("birth_position = message.get(", self.controller)
        self.assertIn("self.spawn_position = tuple(", self.controller)
        self.assertIn('self.node.getField("translation").setSFVec3f(', self.controller)

    def test_controller_uses_default_pose_true_contact_and_torque_feedback(self) -> None:
        self.assertIn("motor.enableTorqueFeedback(self.timestep)", self.controller)
        self.assertIn(
            "motor.getTorqueFeedbackSamplingPeriod()",
            self.controller,
        )
        self.assertIn("_contact_node_foot_index(contact)", self.controller)
        self.assertIn("TOE_CENTER_LOCAL", self.controller)
        self.assertIn("TOE_COLLISION_RADIUS", self.controller)
        self.assertIn("shank Box", self.controller)
        self.assertIn('_resolve_foot_shank_nodes()', self.controller)
        self.assertIn("self.robot.getFromDevice(tag)", self.controller)
        self.assertIn("_device_tag(device)", self.controller)
        self.assertIn('current.getField("endPoint")', self.controller)
        self.assertIn('expected_name = f"{leg}_shank_link"', self.controller)
        self.assertNotIn("_foot_index(local_point)", self.controller)
        action_block = self.controller.split(
            "def action_to_stance_targets", 1
        )[1].split("class RlAgentController", 1)[0]
        self.assertIn("DEFAULT_CROUCH", action_block)
        self.assertNotIn("SLIGHTLY_EXTENDED", action_block)

    def test_controller_error_reports_are_wired_into_main(self) -> None:
        self.assertIn("initialization_failure_report()", self.controller)
        self.assertIn("capability_error_reports()", self.controller)
        self.assertIn("connection_failure_report(exc)", self.controller)
        self.assertIn('"RL_CONTROLLER_ERROR webots_connection_failed: "', self.controller)
        self.assertIn('"RL_CAPABILITY_ERROR {reason}: {context}"', self.controller)


class CliTests(unittest.TestCase):
    def test_train_requires_cuda_and_official_paths(self) -> None:
        args = parse_train_args(["--phase", "P7", "--dry-run"])
        self.assertEqual(args.device, "cuda")
        self.assertEqual(args.total_steps, 18_000_000)
        self.assertEqual(args.num_envs, 4)
        self.assertEqual(args.bridge_port, 11452)
        with self.assertRaises(SystemExit):
            parse_train_args(["--phase", "P7", "--device", "cpu", "--dry-run"])
        with self.assertRaises(SystemExit):
            parse_train_args(["--phase", "P7", "--bridge-port", "11451", "--dry-run"])

    def test_stage_commands_are_official_only(self) -> None:
        command = stage_command(
            "P1", 400_000,
            resume=contract.CHECKPOINT_ROOT / f"{contract.CONTRACT_VERSION}_final.zip",
            log_key="P1",
            randomization_mode="fixed",
        )
        joined = " ".join(command)
        self.assertIn("--device cuda", joined)
        self.assertIn("--bridge-port 11452", joined)
        self.assertIn(contract.CONTRACT_VERSION, joined)
        self.assertNotIn("--webots-gui", joined)
        self.assertIn("--num-envs 4", joined)
        self.assertIn("--randomization-mode fixed", joined)
        self.assertIn(str(contract.RUN_ROOT / "P1_parallel"), joined)
        self.assertNotIn(contract.FORBIDDEN_DIRECTORY, joined)

    def test_stage_seed_changes_with_phase_and_local_steps(self) -> None:
        first = stage_command(
            "P2",
            450_000,
            log_key="P2",
            randomization_mode="fixed",
            phase_step_offset=0,
        )
        second = stage_command(
            "P2",
            500_000,
            log_key="P2",
            randomization_mode="fixed",
            phase_step_offset=50_000,
        )
        first_seed = first[first.index("--seed") + 1]
        second_seed = second[second.index("--seed") + 1]
        self.assertNotEqual(first_seed, second_seed)
        self.assertEqual(
            int(second_seed),
            contract.phase_training_seed("P2", 50_000),
        )
        p1 = stage_command(
            "P1",
            20_000,
            log_key="P1",
            randomization_mode="fixed",
            phase_step_offset=0,
        )
        self.assertNotEqual(
            first_seed,
            p1[p1.index("--seed") + 1],
        )
        self.assertEqual(
            contract.phase_gate_check_interval_steps("P2"),
            50_000,
        )
        self.assertEqual(
            contract.phase_gate_check_interval_steps("P4"),
            contract.GATE_CHECK_INTERVAL_STEPS,
        )

    def test_dry_run_contains_eight_r3_stages(self) -> None:
        missing_checkpoint = (
            contract.CHECKPOINT_ROOT
            / f"{contract.CONTRACT_VERSION}_missing_for_dry_run.zip"
        )
        with patch("rl.run_stages.FINAL_CHECKPOINT", missing_checkpoint):
            commands = dry_run_commands()
        self.assertEqual(len(commands), 8)
        targets = [
            int(command[command.index("--total-steps") + 1])
            for command in commands
        ]
        self.assertEqual(
            targets,
            list(contract.PHASE_TOTAL_STEPS.values()),
        )
        self.assertNotIn("--resume", " ".join(commands[0]))
        self.assertEqual(
            commands[3][commands[3].index("--randomization-mode") + 1],
            "curriculum",
        )
        self.assertEqual(
            commands[4][commands[4].index("--randomization-mode") + 1],
            "dr1",
        )
        for command in commands:
            self.assertIn("--num-envs 4", " ".join(command))
            self.assertIn("--bridge-port 11452", " ".join(command))

    def test_gate_requires_three_consecutive_batches(self) -> None:
        with patch("rl.run_stages.run_gate", side_effect=[True, True, True]) as gate:
            self.assertTrue(run_gate_batch("P1"))
            self.assertEqual(gate.call_count, 3)
        with patch("rl.run_stages.run_gate", side_effect=[True, False, True]) as gate:
            self.assertFalse(run_gate_batch("P1"))
            self.assertEqual(gate.call_count, 3)

    def test_training_main_exit_still_reaps_residual_process_group(self) -> None:
        class ExitedProcess:
            pid = 737905

            def wait(self, timeout: float | None = None) -> int:
                del timeout
                return 0

        process = ExitedProcess()
        with (
            patch("rl.run_stages.os.killpg") as killpg,
            patch(
                "rl.run_stages._process_group_pids",
                side_effect=[{9001}, set()],
            ) as group_pids,
            patch("rl.run_stages.time.sleep"),
        ):
            _terminate_process_group(process)  # type: ignore[arg-type]
        killpg.assert_called_once()
        self.assertEqual(group_pids.call_count, 2)

    def test_successful_run_training_invokes_group_reaper(self) -> None:
        class SuccessfulProcess:
            pid = 91001

            def wait(self, timeout: float | None = None) -> int:
                del timeout
                return 0

        process = SuccessfulProcess()
        with tempfile.TemporaryDirectory() as temporary:
            log_root = Path(temporary)
            with (
                patch("rl.run_stages.contract.LOG_ROOT", log_root),
                patch("rl.run_stages.log"),
                patch("rl.run_stages.write_status"),
                patch("rl.run_stages.subprocess.Popen", return_value=process),
                patch("rl.run_stages.confirm_live_workers"),
                patch("rl.run_stages._terminate_process_group") as reaper,
            ):
                run_training(["python3", "-m", "rl.train"], "P0", "P0")
            reaper.assert_called_once_with(process)

    def test_gate_waits_for_release_then_starts(self) -> None:
        class FakeClock:
            def __init__(self) -> None:
                self.now = 0.0

            def monotonic(self) -> float:
                return self.now

            def sleep(self, seconds: float) -> None:
                self.now += seconds

        clock = FakeClock()
        completed = type(
            "Completed",
            (),
            {"returncode": 0, "stdout": "", "stderr": ""},
        )()
        with (
            patch(
                "rl.run_stages.require_ports_free",
                side_effect=[
                    RuntimeError("启动 Gate失败：端口仍被占用：1234"),
                    None,
                ],
            ) as check_ports,
            patch(
                "rl.run_stages.time.monotonic",
                side_effect=clock.monotonic,
            ),
            patch("rl.run_stages.time.sleep", side_effect=clock.sleep) as sleep,
            patch("rl.run_stages.log"),
            patch(
                "rl.run_stages.subprocess.run",
                return_value=completed,
            ) as run,
        ):
            self.assertTrue(run_gate("P2"))
        self.assertEqual(check_ports.call_count, 2)
        self.assertEqual(
            check_ports.call_args.args[0],
            (contract.WEBOTS_SUPERVISOR_PORT, *contract.bridge_ports()),
        )
        sleep.assert_called_once()
        run.assert_called_once()

    def test_gate_real_listener_timeout_is_diagnostic(self) -> None:
        class FakeClock:
            def __init__(self) -> None:
                self.now = 0.0

            def monotonic(self) -> float:
                return self.now

            def sleep(self, seconds: float) -> None:
                self.now += seconds

        clock = FakeClock()
        gate_commands: list[list[str]] = []
        diagnostic_commands: list[list[str]] = []

        def fake_run(command: list[str], **kwargs: object) -> object:
            del kwargs
            diagnostic_prefixes = {
                ("ss", "-H", "-ltnp"),
                ("ss", "-H", "-tanp"),
            }
            if tuple(command) in diagnostic_prefixes:
                diagnostic_commands.append(list(command))
                return type(
                    "Completed",
                    (),
                    {"returncode": 0, "stdout": "", "stderr": ""},
                )()
            gate_commands.append(list(command))
            return type(
                "Completed",
                (),
                {"returncode": 0, "stdout": "", "stderr": ""},
            )()

        with (
            patch(
                "rl.run_stages.GATE_PORT_TIMEOUT_SECONDS",
                1.0,
            ),
            patch(
                "rl.run_stages.require_ports_free",
                side_effect=RuntimeError(
                    "启动 Gate失败：端口仍被占用：1234"
                ),
            ) as check_ports,
            patch(
                "rl.run_stages.time.monotonic",
                side_effect=clock.monotonic,
            ),
            patch("rl.run_stages.time.sleep", side_effect=clock.sleep),
            patch("rl.run_stages.log"),
            patch("rl.run_stages.subprocess.run", side_effect=fake_run) as run,
        ):
            with self.assertRaisesRegex(
                RuntimeError,
                "端口等待超时.*TIME-WAIT.*不执行破坏性 kill",
            ):
                run_gate("P2")
        self.assertEqual(check_ports.call_count, 5)
        # 端口诊断允许读取 ss；真正 Gate 子进程必须尚未启动。
        self.assertEqual(
            diagnostic_commands,
            [
                ["ss", "-H", "-ltnp"],
                ["ss", "-H", "-tanp"],
            ],
        )
        self.assertEqual(gate_commands, [])
        self.assertEqual(run.call_count, 2)

    def test_eval_dry_run(self) -> None:
        args = parse_eval_args(["--phase", "P3", "--gate", "--dry-run"])
        self.assertEqual(args.phase, "P3")
        self.assertTrue(args.gate)

    def test_live_health_counts_one_world_and_four_controllers(self) -> None:
        output = "\n".join(
            [
                "101 100 /bin/bash /usr/local/webots/webots --port=1234 worlds/flat_move_jump_rl.wbt",
                "102 100 /usr/local/webots/bin/webots-bin --port=1234 worlds/flat_move_jump_rl.wbt",
                *[
                    f"{200 + index} 100 python3 -u rl_agent.py"
                    for index in range(4)
                ],
                "900 900 /bin/bash /usr/local/webots/webots --port=1234 worlds/flat_move_jump_rl.wbt",
                "901 900 python3 -u rl_agent.py",
            ]
        )
        completed = type(
            "Completed",
            (),
            {"stdout": output, "returncode": 0},
        )()
        with patch("rl.run_stages.subprocess.run", return_value=completed):
            self.assertEqual(_webots_process_count(group_id=100), 1)
            self.assertEqual(
                _webots_controller_process_count(group_id=100),
                4,
            )
            self.assertEqual(
                _webots_controller_process_count(group_id=900),
                1,
            )
            worker_logs = {
                str(index): {
                    "current": True,
                    "connected": True,
                    "worker_id": True,
                    "robot_name": True,
                    "timestep": True,
                }
                for index in range(4)
            }
            self.assertEqual(
                _webots_controller_process_count(
                    group_id=100,
                    ports=contract.bridge_ports(),
                    worker_logs=worker_logs,
                ),
                4,
            )
            self.assertEqual(
                _webots_controller_process_count(
                    group_id=100,
                    ports=(11452, 11453),
                    worker_logs=worker_logs,
                ),
                0,
            )

    def test_live_health_ports_are_bound_to_current_process_group(self) -> None:
        ss_output = "\n".join(
            [
                *[
                    f"LISTEN 0 4096 127.0.0.1:{port} 0.0.0.0:* "
                    f'users:(("python3",pid=100,fd=8))'
                    for port in contract.bridge_ports()
                ],
                "LISTEN 0 4096 127.0.0.1:11452 0.0.0.0:* "
                'users:(("python3",pid=900,fd=8))',
            ]
        )
        ps_output = "100 100 python3 -m rl.train\n900 900 python3 -m stale\n"

        def fake_run(command: list[str], **kwargs: object) -> object:
            del kwargs
            stdout = ss_output if command[0] == "ss" else ps_output
            return type(
                "Completed",
                (),
                {"stdout": stdout, "returncode": 0},
            )()

        with patch("rl.run_stages.subprocess.run", side_effect=fake_run):
            self.assertEqual(
                _port_listeners(group_id=100),
                list(contract.bridge_ports()),
            )


if __name__ == "__main__":
    unittest.main(verbosity=2)
