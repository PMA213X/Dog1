"""官方 Mini Cheetah 默认姿态与固定时间线的单元、静态测试。"""

from __future__ import annotations

import math
import re
import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PROJECT_ROOT.parent
ACTIVE_CONTROLLER_DIR = PROJECT_ROOT / "controllers/flat_ground_teleop"
TIMELINE_CONTROLLER_DIR = PROJECT_ROOT / "controllers/posture_timeline"
for path in (ACTIVE_CONTROLLER_DIR, TIMELINE_CONTROLLER_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from joint_safety import (  # noqa: E402
    DEFAULT_CROUCH,
    FLAT_MOTOR_NAMES,
    FLAT_SENSOR_NAMES,
    JOINT_COUNT,
    MAX_TARGET_RATE,
    SLIGHTLY_EXTENDED,
    TARGET_POSITION_LIMITS,
    JointTargets,
    TargetRateLimiter,
    MiniCheetahGait,
    standing_targets,
)
from posture_timeline_core import (  # noqa: E402
    DURATION_SECONDS,
    PosturePhase,
    PostureTimelineTracker,
    SUPPORT_HALF_LENGTH,
    foot_positions,
    knee_interior_angle,
    phase_at,
    support_projection_is_inside,
    target_at,
    target_velocity_at,
)
from posture_timeline import lowest_geometry_z  # noqa: E402


PASSIVE_WORLD = PROJECT_ROOT / "worlds/official_flat_ground_test.wbt"
ACTIVE_WORLD = PROJECT_ROOT / "worlds/flat_ground_teleop.wbt"
TIMELINE_WORLD = PROJECT_ROOT / "worlds/posture_timeline_test.wbt"
TIMELINE_CONTROLLER = TIMELINE_CONTROLLER_DIR / "posture_timeline.py"
TIMELINE_CORE = TIMELINE_CONTROLLER_DIR / "posture_timeline_core.py"


def _robot_block(text: str) -> str:
    """提取 name=mini_cheetah 的主 Robot 节点。"""

    match = re.search(r'name\s+"mini_cheetah"', text)
    if match is None:
        raise AssertionError("缺少 mini_cheetah Robot")
    start = text.rfind("Robot {", 0, match.start())
    depth = 0
    in_string = False
    escaped = False
    for index in range(start, len(text)):
        char = text[index]
        if in_string:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                in_string = False
        elif char == '"':
            in_string = True
        elif char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return text[start : index + 1]
    raise AssertionError("Robot 节点未闭合")


def _model_tokens(robot: str) -> tuple[str, ...]:
    """移除 controller、只读 supervisor 和姿态专用关节弹簧字段后比较模型。"""

    clean = re.sub(r"controller\s+\"[^\"]*\"", "", robot)
    clean = re.sub(r"controllerArgs\s*\[[^\]]*\]", "", clean)
    clean = re.sub(r"supervisor\s+TRUE", "", clean)
    clean = re.sub(r"(?m)^\s*springConstant\s+[^\n#]+", "", clean)
    clean = re.sub(r"translation 0 0 0\.40", "", clean)
    clean = re.sub(r"translation 0 0 0\.45", "", clean)
    return tuple(
        re.findall(
            r"\"(?:\\.|[^\"\\])*\"|[A-Za-z_][A-Za-z0-9_]*|"
            r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?|[{}\[\]]",
            clean,
        )
    )


class PostureTargetTests(unittest.TestCase):
    """默认姿态、伸展姿态和几何推导。"""

    def test_targets_are_finite_bounded_and_mirrored(self) -> None:
        for name, target in (
            ("DEFAULT_CROUCH", DEFAULT_CROUCH),
            ("SLIGHTLY_EXTENDED", SLIGHTLY_EXTENDED),
        ):
            with self.subTest(name=name):
                self.assertEqual(len(target), JOINT_COUNT)
                self.assertTrue(all(math.isfinite(value) for value in target))
                self.assertTrue(
                    all(
                        lower <= value <= upper
                        for value, (lower, upper) in zip(target, TARGET_POSITION_LIMITS)
                    )
                )
                # fr/fl 与 hr/hl 严格左右镜像：abad 反号，hip/knee 同值。
                for right, left in ((0, 3), (6, 9)):
                    self.assertEqual(target[right], -target[left])
                    self.assertEqual(target[right + 1], target[left + 1])
                    self.assertEqual(target[right + 2], target[left + 2])
                # 前后腿保持同一 sagittal 姿态。
                self.assertEqual(target[0:3], target[6:9])
                self.assertEqual(target[3:6], target[9:12])

    def test_slightly_extended_is_extended_but_not_locked(self) -> None:
        self.assertLess(abs(SLIGHTLY_EXTENDED[1]), abs(DEFAULT_CROUCH[1]))
        self.assertLess(abs(SLIGHTLY_EXTENDED[2]), abs(DEFAULT_CROUCH[2]))
        self.assertGreaterEqual(abs(SLIGHTLY_EXTENDED[2]), 1.0)
        self.assertGreater(knee_interior_angle(DEFAULT_CROUCH), math.radians(70.0))
        self.assertGreater(knee_interior_angle(SLIGHTLY_EXTENDED), math.radians(90.0))
        self.assertLess(knee_interior_angle(SLIGHTLY_EXTENDED), math.pi)

    def test_support_projection_uses_official_geometry(self) -> None:
        for target in (DEFAULT_CROUCH, SLIGHTLY_EXTENDED):
            positions = foot_positions(target)
            self.assertEqual(set(positions), {"fr", "fl", "hr", "hl"})
            self.assertTrue(support_projection_is_inside(target))
            self.assertTrue(
                all(abs(position[0]) <= SUPPORT_HALF_LENGTH + 1e-9 for position in positions.values())
            )

    def test_passive_world_still_uses_none_controller(self) -> None:
        self.assertIn('controller "<none>"', PASSIVE_WORLD.read_text(encoding="utf-8"))

    def test_spawn_height_has_geometry_clearance(self) -> None:
        timeline = TIMELINE_WORLD.read_text(encoding="utf-8")
        self.assertIn("translation 0 0 0.45", timeline)
        identity = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        zeros = [0.0] * JOINT_COUNT
        zero_gap = lowest_geometry_z(zeros, (0.0, 0.0, 0.45), identity)
        default_gap = lowest_geometry_z(
            DEFAULT_CROUCH,
            (0.0, 0.0, 0.45),
            identity,
        )
        self.assertIsNotNone(zero_gap)
        self.assertIsNotNone(default_gap)
        self.assertGreaterEqual(zero_gap, 0.05)
        self.assertGreaterEqual(default_gap, 0.15)


class TimelineTests(unittest.TestCase):
    """固定时间线、状态边界、连续性和变化率限制。"""

    def test_boundaries_are_exact(self) -> None:
        expected = (
            (0.0, PosturePhase.DEFAULT_CROUCH, DEFAULT_CROUCH),
            (10.0, PosturePhase.TO_SLIGHTLY_EXTENDED, DEFAULT_CROUCH),
            (12.0, PosturePhase.SLIGHTLY_EXTENDED, SLIGHTLY_EXTENDED),
            (14.0, PosturePhase.RETURN_TO_DEFAULT, SLIGHTLY_EXTENDED),
            (16.0, PosturePhase.DEFAULT_CROUCH, DEFAULT_CROUCH),
        )
        for now, phase, target in expected:
            with self.subTest(now=now):
                self.assertEqual(phase_at(now), phase)
                self.assertEqual(target_at(now), target)
                self.assertEqual(target_velocity_at(now), (0.0,) * JOINT_COUNT)

    def test_transition_is_continuous_and_smooth(self) -> None:
        previous = target_at(0.0)
        maximum_step = 0.0
        for index in range(1, int(DURATION_SECONDS * 1000) + 1):
            current = target_at(index / 1000.0)
            maximum_step = max(
                maximum_step,
                max(abs(left - right) for left, right in zip(previous, current)),
            )
            previous = current
        self.assertLess(maximum_step, 0.001)
        # 边界前后目标不应跳变。
        for boundary in (10.0, 12.0, 14.0, 16.0):
            before = target_at(boundary - 0.001)
            after = target_at(boundary + 0.001)
            self.assertLess(
                max(abs(left - right) for left, right in zip(before, after)),
                0.002,
            )

    def test_rate_limiter_and_tracker_enforce_change_limit(self) -> None:
        limiter = TargetRateLimiter(DEFAULT_CROUCH)
        previous = limiter.current
        for index in range(1, 4001):
            current = limiter.apply(target_at(index * 0.004), 0.004)
            self.assertTrue(
                all(
                    abs(next_value - before) <= MAX_TARGET_RATE * 0.004 + 1e-12
                    for before, next_value in zip(previous, current)
                )
            )
            previous = current
        tracker = PostureTimelineTracker()
        previous = tracker.current
        for index in range(1, 4001):
            result = tracker.sample(index * 0.004, 0.004)
            self.assertEqual(len(result.positions), JOINT_COUNT)
            self.assertEqual(len(result.velocities), JOINT_COUNT)
            self.assertTrue(all(math.isfinite(value) for value in result.positions))
            self.assertTrue(
                all(
                    abs(next_value - before) <= MAX_TARGET_RATE * 0.004 + 1e-12
                    for before, next_value in zip(previous, result.positions)
                )
            )
            previous = result.positions

    def test_active_default_and_gait_use_crouch_baseline(self) -> None:
        self.assertEqual(standing_targets().positions, DEFAULT_CROUCH)
        self.assertEqual(JointTargets().positions, DEFAULT_CROUCH)
        gait = MiniCheetahGait()
        self.assertEqual(gait.step(0.0, 0.0, 0.0, 0.004).positions, DEFAULT_CROUCH)
        moving = gait.step(0.6, 0.3, 1.0, 0.004)
        self.assertEqual(len(moving.positions), JOINT_COUNT)
        self.assertTrue(
            all(
                lower <= value <= upper
                for value, (lower, upper) in zip(moving.positions, TARGET_POSITION_LIMITS)
            )
        )


class WorldStaticTests(unittest.TestCase):
    """独立测试 world 的模型、隔离和无 artifacts 约束。"""

    def test_timeline_world_is_isolated(self) -> None:
        timeline = TIMELINE_WORLD.read_text(encoding="utf-8")
        passive = PASSIVE_WORLD.read_text(encoding="utf-8")
        active = ACTIVE_WORLD.read_text(encoding="utf-8")
        self.assertIn('controller "posture_timeline"', timeline)
        self.assertIn("supervisor TRUE", timeline)
        self.assertNotIn("acceptance_supervisor", timeline)
        self.assertNotIn("DEF ACCEPTANCE_SUPERVISOR", timeline)
        self.assertNotIn("acceptance_summary", timeline)
        self.assertNotIn("acceptance_samples", timeline)
        self.assertIn('controller "<none>"', passive)
        self.assertIn("acceptance_supervisor", passive)
        self.assertIn('controller "flat_ground_teleop"', active)
        self.assertNotIn("acceptance_supervisor", active)
        self.assertEqual(timeline.count("springConstant 0"), 12)
        self.assertEqual(active.count("springConstant 0"), 12)
        self.assertEqual(passive.count("springConstant 100"), 12)

    def test_timeline_world_keeps_12_12_12_and_axes(self) -> None:
        robot = _robot_block(TIMELINE_WORLD.read_text(encoding="utf-8"))
        self.assertEqual(robot.count("HingeJoint {"), 12)
        self.assertEqual(robot.count("endPoint Solid"), 12)
        self.assertEqual(robot.count("RotationalMotor {"), 12)
        self.assertEqual(robot.count("PositionSensor {"), 12)
        axes = re.findall(r"(?m)^\s*axis\s+([^\n#]+)", robot)
        self.assertEqual(len(axes), 12)
        normalized = [tuple(float(value) for value in axis.split()) for axis in axes]
        self.assertEqual(sum(abs(axis[0]) > 0 for axis in normalized), 4)
        self.assertEqual(sum(abs(axis[1]) > 0 for axis in normalized), 8)
        self.assertTrue(all(abs(axis[2]) < 1e-12 for axis in normalized))
        self.assertEqual(len(set(FLAT_MOTOR_NAMES)), 12)
        self.assertEqual(len(set(FLAT_SENSOR_NAMES)), 12)

    def test_timeline_world_preserves_official_model(self) -> None:
        timeline_robot = _robot_block(TIMELINE_WORLD.read_text(encoding="utf-8"))
        passive_robot = _robot_block(PASSIVE_WORLD.read_text(encoding="utf-8"))
        self.assertEqual(_model_tokens(timeline_robot), _model_tokens(passive_robot))
        for token in (
            "mini_body.dae",
            "mini_abad.dae",
            "mini_upper_link.dae",
            "mini_lower_link.dae",
            "mass 3.3",
            "mass 0.634",
            "mass 0.54",
            "mass 0.214",
            "inertiaMatrix",
            "boundingObject",
        ):
            self.assertIn(token, timeline_robot)

    def test_controller_does_not_write_passive_artifacts(self) -> None:
        source = TIMELINE_CONTROLLER.read_text(encoding="utf-8")
        core = TIMELINE_CORE.read_text(encoding="utf-8")
        for text in (source, core):
            self.assertNotIn("ARTIFACT_ROOT", text)
            self.assertNotIn("acceptance_summary", text)
            self.assertNotIn("acceptance_samples", text)
            self.assertNotIn("write_text", text)
            self.assertIsNone(re.search(r"(?<!os\.)open\(", text))
        self.assertIn("POSTURE_STATUS", source)
        self.assertIn("POSTURE_RESULT", source)
        self.assertIn("simulationQuit", source)

    def test_geometry_logger_rejects_nonfinite_pose(self) -> None:
        identity = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        value = lowest_geometry_z(DEFAULT_CROUCH, (0.0, 0.0, 0.40), identity)
        self.assertIsNotNone(value)
        self.assertTrue(math.isfinite(value))
        self.assertIsNone(
            lowest_geometry_z(DEFAULT_CROUCH, (math.nan, 0.0, 0.40), identity)
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
