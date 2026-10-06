"""官方 Mini Cheetah 平地被动验收的静态单元测试。"""

from __future__ import annotations

import hashlib
import importlib.util
import math
import re
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PROJECT_ROOT.parent
WORLD_PATH = PROJECT_ROOT / "worlds/official_flat_ground_test.wbt"
GENERATOR_PATH = PROJECT_ROOT / "tools/gen_official_mini_cheetah.py"
SUPERVISOR_PATH = (
    PROJECT_ROOT
    / "controllers/acceptance_supervisor/acceptance_supervisor.py"
)
REFERENCE_PATHS = (
    REPOSITORY_ROOT / "webots-sim/worlds/mini_cheetah.wbt",
    REPOSITORY_ROOT / "Cheetah-Software/common/include/Dynamics/MiniCheetah.h",
    REPOSITORY_ROOT / "webots-sim/urdf/mini_cheetah.urdf",
)


def _sha256(path: Path) -> str:
    """计算文件摘要，用于确认测试不改动参考文件。"""

    return hashlib.sha256(path.read_bytes()).hexdigest()


def _extract_block(text: str, start_pattern: str) -> str:
    """提取花括号配对的 VRML 节点。"""

    match = re.search(start_pattern, text, flags=re.MULTILINE)
    if match is None:
        raise AssertionError(f"未找到节点：{start_pattern}")
    depth = 0
    in_string = False
    for index in range(match.start(), len(text)):
        char = text[index]
        if char == '"':
            in_string = not in_string
        elif not in_string and char == "{":
            depth += 1
        elif not in_string and char == "}":
            depth -= 1
            if depth == 0:
                return text[match.start() : index + 1]
    raise AssertionError(f"节点未闭合：{start_pattern}")


def _extract_block_at(text: str, start: int) -> str:
    """按左花括号位置提取完整 VRML 节点。"""

    depth = 0
    in_string = False
    escaped = False
    in_comment = False
    for index in range(start, len(text)):
        char = text[index]
        if in_comment:
            if char == "\n":
                in_comment = False
            continue
        if in_string:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                in_string = False
            continue
        if char == '"':
            in_string = True
        elif char == "#":
            in_comment = True
        elif char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return text[start : index + 1]
    raise AssertionError("节点花括号未闭合")


def _field(text: str, name: str) -> tuple[float, ...] | None:
    """读取单个数值字段；字段缺失时返回 None。"""

    match = re.search(rf"(?m)^\s*{re.escape(name)}\s+([^\n#]+)", text)
    if match is None:
        return None
    return tuple(float(value) for value in match.group(1).split())


def _floats(text: str) -> tuple[float, ...]:
    """提取节点中的全部十进制数值。"""

    return tuple(
        float(value)
        for value in re.findall(
            r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?",
            text,
        )
    )


def _joint_block(robot: str, motor: str) -> str:
    """按 motor 名提取所属 HingeJoint 节点。"""

    motor_index = robot.find(f'name "{motor}"')
    if motor_index < 0:
        raise AssertionError(f"缺少 motor：{motor}")
    joint_index = robot.rfind("HingeJoint {", 0, motor_index)
    if joint_index < 0:
        raise AssertionError(f"motor 不在 HingeJoint 中：{motor}")
    return _extract_block_at(robot, joint_index + len("HingeJoint "))


def _endpoint_block(robot: str, motor: str) -> str:
    """提取 motor 对应的 child/endPoint Solid 节点。"""

    joint = _joint_block(robot, motor)
    match = re.search(r"(?:child|endPoint) Solid\s*\{", joint)
    if match is None:
        raise AssertionError(f"缺少 Solid：{motor}")
    return _extract_block_at(joint, match.end() - 1)


def _physics_signature(
    block: str,
) -> tuple[float, tuple[float, ...], tuple[float, ...]]:
    """解析 mass、centerOfMass 和 R2025a 六值 inertiaMatrix。"""

    mass_match = re.search(r"mass\s+([-\d.eE+]+)", block)
    com_match = re.search(r"centerOfMass\s+([^\n]+)", block)
    matrix_match = re.search(r"inertiaMatrix\s*\[([\s\S]*?)\]", block)
    if mass_match is None or com_match is None or matrix_match is None:
        raise AssertionError("Physics 字段缺失")
    mass = float(mass_match.group(1))
    com = tuple(float(value) for value in com_match.group(1).split())
    values = _floats(matrix_match.group(1))
    if len(values) == 9:
        values = (
            values[0],
            values[4],
            values[8],
            values[1],
            values[2],
            values[5],
        )
    if len(values) != 6:
        raise AssertionError(f"inertiaMatrix 数量错误：{len(values)}")
    return mass, com, values


def _physics_for_link(
    robot: str,
    motor: str,
    link: str,
) -> tuple[float, tuple[float, ...], tuple[float, ...]]:
    """提取 endpoint 内指定 link 的 Physics。"""

    endpoint = _endpoint_block(robot, motor)
    name_index = endpoint.find(f'name "{link}"')
    if name_index < 0:
        raise AssertionError(f"endpoint 缺少 link：{link}")
    physics_index = endpoint.find("physics Physics {", name_index)
    if physics_index < 0:
        raise AssertionError(f"link 缺少 Physics：{link}")
    physics = _extract_block_at(
        endpoint,
        physics_index + len("physics Physics "),
    )
    return _physics_signature(physics)


def _root_physics(
    robot: str,
) -> tuple[float, tuple[float, ...], tuple[float, ...]]:
    """提取 Robot 根 Physics。"""

    physics_index = robot.rfind("physics Physics {")
    if physics_index < 0:
        raise AssertionError("Robot 缺少根 Physics")
    return _physics_signature(
        _extract_block_at(robot, physics_index + len("physics Physics ")),
    )


def _geometry_signature(
    text: str,
) -> tuple[tuple[str, tuple[float, ...]], ...]:
    """提取 Box/Sphere collision 资源的类型和尺寸。"""

    result: list[tuple[str, tuple[float, ...]]] = []
    for match in re.finditer(r"\b(Box|Sphere)\s*\{([^{}]*)\}", text):
        kind = match.group(1)
        body = match.group(2)
        field_name = "size" if kind == "Box" else "radius"
        value = _field(body, field_name)
        if value is None:
            raise AssertionError(f"{kind} 缺少 {field_name}")
        result.append((kind, value))
    return tuple(sorted(result))


def _link_collision_signature(
    robot: str,
    motor: str,
    link: str,
    *,
    generated: bool,
) -> tuple[tuple[str, tuple[float, ...]], ...]:
    """比较官方源几何与生成结果 boundingObject。"""

    endpoint = _endpoint_block(robot, motor)
    name_index = endpoint.find(f'name "{link}"')
    if generated:
        bounding_index = endpoint.find("boundingObject", name_index)
        if bounding_index < 0:
            raise AssertionError(f"生成 link 缺少 boundingObject：{link}")
        region = _extract_block_at(endpoint, endpoint.find("{", bounding_index))
    else:
        nested_index = endpoint.find("HingeJoint {", name_index)
        region = endpoint if nested_index < 0 else endpoint[:nested_index]
    return _geometry_signature(region)


def _root_collision_signature(
    robot: str,
    *,
    generated: bool,
) -> tuple[tuple[str, tuple[float, ...]], ...]:
    """比较机身根 collision 几何。"""

    if generated:
        bounding_index = robot.rfind("boundingObject")
        if bounding_index < 0:
            raise AssertionError("生成机身缺少 boundingObject")
        region = _extract_block_at(robot, robot.find("{", bounding_index))
    else:
        first_joint = robot.find("HingeJoint {")
        region = robot if first_joint < 0 else robot[:first_joint]
    return _geometry_signature(region)


def _axis_angle_to_matrix(
    rotation: tuple[float, ...],
) -> tuple[tuple[float, float, float], ...]:
    """把 Webots 轴角转换为 3x3 矩阵。"""

    x, y, z, angle = rotation
    norm = math.sqrt(x * x + y * y + z * z)
    if norm < 1e-15 or abs(angle) < 1e-15:
        return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    x, y, z = x / norm, y / norm, z / norm
    cosine = math.cos(angle)
    sine = math.sin(angle)
    complement = 1.0 - cosine
    return (
        (
            cosine + x * x * complement,
            x * y * complement - z * sine,
            x * z * complement + y * sine,
        ),
        (
            y * x * complement + z * sine,
            cosine + y * y * complement,
            y * z * complement - x * sine,
        ),
        (
            z * x * complement - y * sine,
            z * y * complement + x * sine,
            cosine + z * z * complement,
        ),
    )


def _rpy_to_axis_angle(rpy: tuple[float, ...]) -> tuple[float, ...]:
    """把 URDF RPY 转为 Webots 轴角。"""

    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    if qw < 0.0:
        qx, qy, qz, qw = -qx, -qy, -qz, -qw
    sin_half = math.sqrt(max(0.0, 1.0 - qw * qw))
    if sin_half <= 1e-12:
        return (0.0, 0.0, 1.0, 0.0)
    return (qx / sin_half, qy / sin_half, qz / sin_half, 2.0 * math.acos(qw))


def _matrix_error(
    left: tuple[tuple[float, float, float], ...],
    right: tuple[tuple[float, float, float], ...],
) -> float:
    """计算两个旋转矩阵的最大元素误差。"""

    return max(
        abs(left[row][column] - right[row][column])
        for row in range(3)
        for column in range(3)
    )


def _urdf_link_frames(urdf_path: Path) -> dict[str, tuple[float, ...]]:
    """按 URDF 活动关节链计算 link 零位坐标。"""

    root = ET.parse(urdf_path).getroot()
    active = [
        joint
        for joint in root.findall("joint")
        if joint.get("type") != "fixed"
    ]
    parent_joint = {
        joint.find("child").get("link"): joint
        for joint in active
    }
    frames: dict[str, tuple[float, ...]] = {"body": (0.0, 0.0, 0.0)}

    def resolve(link: str) -> tuple[float, ...]:
        if link in frames:
            return frames[link]
        joint = parent_joint[link]
        parent = resolve(joint.find("parent").get("link"))
        origin = tuple(
            float(value)
            for value in joint.find("origin").get("xyz", "0 0 0").split()
        )
        frames[link] = tuple(
            parent[index] + origin[index]
            for index in range(3)
        )
        return frames[link]

    for link in root.findall("link"):
        name = link.get("name")
        if not name.startswith("toe_"):
            resolve(name)
    return frames


def _strict_visual_compare(world_robot: str, urdf_path: Path) -> None:
    """逐个比较 13 个 DAE visual 的 URL、translation 和 rpy。"""

    mesh_records: list[tuple[tuple[float, ...], tuple[float, ...], str]] = []
    for match in re.finditer(r"(?m)^([ \t]*)Pose \{", world_robot):
        brace_index = match.start() + len(match.group(1)) + len("Pose ")
        pose = _extract_block_at(world_robot, brace_index)
        if "geometry Mesh" not in pose:
            continue
        translation = _field(pose, "translation")
        rotation = _field(pose, "rotation")
        url_match = re.search(r'url\s*\[\s*"([^"]+)"', pose)
        if translation is None or rotation is None or url_match is None:
            raise AssertionError("DAE Pose 字段缺失")
        mesh_records.append((translation, rotation, url_match.group(1)))
    if len(mesh_records) != 13:
        raise AssertionError(f"DAE visual 数量不是 13：{len(mesh_records)}")

    urdf = ET.parse(urdf_path).getroot()
    links = {link.get("name"): link for link in urdf.findall("link")}
    frames = _urdf_link_frames(urdf_path)
    expected_names = ["body"]
    for leg in ("fr", "fl", "hr", "hl"):
        expected_names.extend(
            (f"abduct_{leg}", f"thigh_{leg}", f"shank_{leg}"),
        )

    endpoint_frames: dict[str, tuple[float, ...]] = {}
    for leg in ("fr", "fl", "hr", "hl"):
        parent = (0.0, 0.0, 0.0)
        for kind, motor_kind in (("abd", "abd"), ("hip", "hip"), ("kn", "kn")):
            endpoint = _endpoint_block(
                world_robot,
                f"{leg}_{motor_kind}_motor",
            )
            translation = _field(endpoint, "translation")
            if translation is None:
                raise AssertionError("endpoint translation 缺失")
            parent = tuple(
                parent[index] + translation[index]
                for index in range(3)
            )
            endpoint_frames[f"{leg}_{kind}"] = parent

    for index, link_name in enumerate(expected_names):
        visual = links[link_name].find("visual")
        origin = visual.find("origin")
        xyz = tuple(
            float(value)
            for value in origin.get("xyz", "0 0 0").split()
        )
        rpy = tuple(
            float(value)
            for value in origin.get("rpy", "0 0 0").split()
        )
        mesh = visual.find("geometry/mesh")
        if mesh is None:
            raise AssertionError(f"URDF visual 不是 mesh：{link_name}")
        expected_url = "../../webots-sim/urdf/" + mesh.get("filename")
        if index == 0:
            endpoint = (0.0, 0.0, 0.0)
        else:
            leg = ("fr", "fl", "hr", "hl")[(index - 1) // 3]
            kind = ("abd", "hip", "kn")[(index - 1) % 3]
            endpoint = endpoint_frames[f"{leg}_{kind}"]
        link_frame = frames[link_name]
        expected_translation = tuple(
            link_frame[value] + xyz[value] - endpoint[value]
            for value in range(3)
        )
        actual_translation, actual_rotation, actual_url = mesh_records[index]
        if actual_url != expected_url:
            raise AssertionError(
                f"DAE URL 不一致：{actual_url} != {expected_url}",
            )
        for value in range(3):
            if not math.isclose(
                actual_translation[value],
                expected_translation[value],
                rel_tol=0.0,
                abs_tol=1e-12,
            ):
                raise AssertionError(
                    f"DAE translation 不一致：{link_name}",
                )
        actual_matrix = _axis_angle_to_matrix(actual_rotation)
        expected_matrix = _axis_angle_to_matrix(_rpy_to_axis_angle(rpy))
        if _matrix_error(actual_matrix, expected_matrix) > 1e-8:
            raise AssertionError(f"DAE rpy 不一致：{link_name}")

    for _, _, url in mesh_records:
        mesh_path = (WORLD_PATH.parent / url).resolve()
        if not mesh_path.is_file():
            raise AssertionError(f"DAE 资源不存在：{mesh_path}")
        root = ET.parse(mesh_path).getroot()
        namespace = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
        unit = root.find(".//c:asset/c:unit", namespace)
        up_axis = root.findtext(".//c:asset/c:up_axis", namespaces=namespace)
        if unit is None or not math.isclose(float(unit.get("meter")), 1.0):
            raise AssertionError(f"DAE 单位不是米：{mesh_path}")
        if up_axis != "Z_UP":
            raise AssertionError(f"DAE up_axis 不是 Z_UP：{mesh_path}")


def _strict_model_compare(world_robot: str, source_robot: str) -> None:
    """逐腿比较 WBT 权威动力学/关节/collision 字段。"""

    link_by_motor = {"abd": "abd", "hip": "thigh", "kn": "shank"}
    for leg in ("fr", "fl", "hr", "hl"):
        for motor_kind, link_kind in link_by_motor.items():
            motor = f"{leg}_{motor_kind}_motor"
            link = f"{leg}_{link_kind}_link"
            source_joint = _joint_block(source_robot, motor)
            world_joint = _joint_block(world_robot, motor)
            for field in ("anchor", "axis", "dampingConstant"):
                source_value = _field(source_joint, field)
                world_value = _field(world_joint, field)
                if source_value != world_value:
                    raise AssertionError(
                        f"{motor} {field} 不一致："
                        f"{world_value} != {source_value}",
                    )
            source_stop = _field(source_joint, "stopSpringDamper")
            world_spring = _field(world_joint, "springConstant")
            world_damping = _field(world_joint, "dampingConstant")
            if source_stop is None or world_spring is None or world_damping is None:
                raise AssertionError(f"{motor} stop/spring 字段缺失")
            if source_stop != (world_spring[0], world_damping[0]):
                raise AssertionError(f"{motor} stop 语义不一致")
            for field in ("minStop", "maxStop"):
                if _field(source_joint, field) != _field(world_joint, field):
                    raise AssertionError(f"{motor} {field} 不一致")
            source_motor = _extract_block_at(
                source_joint,
                source_joint.find("RotationalMotor {")
                + len("RotationalMotor "),
            )
            world_motor = _extract_block_at(
                world_joint,
                world_joint.find("RotationalMotor {")
                + len("RotationalMotor "),
            )
            for field in ("minPosition", "maxPosition"):
                if _field(source_motor, field) != _field(world_motor, field):
                    raise AssertionError(f"{motor} {field} 不一致")
            source_physics = _physics_for_link(source_robot, motor, link)
            world_physics = _physics_for_link(world_robot, motor, link)
            if source_physics != world_physics:
                raise AssertionError(f"{link} Physics 不一致")
            source_collision = _link_collision_signature(
                source_robot,
                motor,
                link,
                generated=False,
            )
            world_collision = _link_collision_signature(
                world_robot,
                motor,
                link,
                generated=True,
            )
            if source_collision != world_collision:
                raise AssertionError(f"{link} collision 不一致")

    if _root_physics(source_robot) != _root_physics(world_robot):
        raise AssertionError("机身 Physics 不一致")
    if _root_collision_signature(
        source_robot,
        generated=False,
    ) != _root_collision_signature(world_robot, generated=True):
        raise AssertionError("机身 collision 不一致")


def _load_generator():
    """加载生成器模块供测试复用，避免命令行副作用。"""

    spec = importlib.util.spec_from_file_location(
        "gen_official_mini_cheetah_under_test", GENERATOR_PATH
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("无法加载生成器")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class OfficialFlatGroundStaticTests(unittest.TestCase):
    """检查 world、生成器与只读 supervisor。"""

    @classmethod
    def setUpClass(cls) -> None:
        """记录参考文件摘要和待测源码。"""

        cls.reference_hashes = {path: _sha256(path) for path in REFERENCE_PATHS}
        cls.world = WORLD_PATH.read_text(encoding="utf-8")
        cls.generator = _load_generator()
        cls.supervisor_source = SUPERVISOR_PATH.read_text(encoding="utf-8")

    @classmethod
    def tearDownClass(cls) -> None:
        """确认全部静态测试均未修改参考文件。"""

        for path, expected in cls.reference_hashes.items():
            if _sha256(path) != expected:
                raise AssertionError(f"测试修改了参考文件：{path}")

    def test_generator_default_output_matches_world_robot(self) -> None:
        """生成器默认文本必须与 world 中 Robot 完全一致。"""

        generated = self.generator.generate_robot()
        world_robot = _extract_block(self.world, r"^DEF MINI_CHEETAH Robot \{$")
        self.assertEqual(generated, world_robot)
        report = self.generator.validate_models()
        self.assertEqual(report.official_robot_joints, 12)
        self.assertEqual(report.official_robot_motors, 12)
        self.assertEqual(report.official_robot_sensors, 12)
        self.assertEqual(report.official_robot_abad_axes, 4)
        self.assertEqual(report.official_robot_pitch_axes, 8)
        self.assertEqual(report.urdf_joints, 12)
        self.assertEqual(report.endpoint_count, 12)
        self.assertLess(report.max_anchor_endpoint_error, 0.001)
        self.assertLess(report.max_zero_world_error, 0.001)

    def test_robot_initial_pose_and_controller(self) -> None:
        """检查 Robot 初始位姿和控制器彻底禁用。"""

        self.assertIn("DEF MINI_CHEETAH Robot {", self.world)
        source_world = self.generator.DEFAULT_SOURCE_WORLD.read_text(
            encoding="utf-8",
        )
        source_robot = _extract_block(source_world, r"^Robot \{$")
        world_robot = _extract_block(
            self.world,
            r"^DEF MINI_CHEETAH Robot \{$",
        )
        self.assertIn("translation 0 0 0.38", source_robot)
        self.assertIn("translation 0 0 0.40", world_robot)
        self.assertNotIn("translation 0 0 0.38", world_robot)
        self.assertIn("rotation 0 0 1 0", self.world)
        self.assertIn('controller "<none>"', self.world)
        self.assertIn("controllerArgs []", self.world)
        self.assertIn("selfCollision TRUE", self.world)
        self.assertEqual(self.world.count('controller "<none>"'), 1)
        self.assertNotIn("mini_cheetah_controller", self.world)
        self.assertNotIn("yobogo", self.world.lower())

    def test_twelve_joint_structure_and_axes(self) -> None:
        """检查 12 关节、设备、轴和显式零位。"""

        self.assertEqual(self.world.count("HingeJoint {"), 12)
        self.assertEqual(self.world.count("endPoint Solid"), 12)
        self.assertEqual(self.world.count("child Solid"), 0)
        self.assertEqual(self.world.count("RotationalMotor {"), 12)
        self.assertEqual(self.world.count("PositionSensor {"), 12)
        self.assertEqual(self.world.count("axis 1 0 0"), 4)
        self.assertEqual(self.world.count("axis 0 -1 0"), 8)
        self.assertEqual(self.world.count("position 0"), 12)
        self.assertEqual(self.world.count("dampingConstant 1.0"), 12)
        source_world = self.generator.DEFAULT_SOURCE_WORLD.read_text(
            encoding="utf-8",
        )
        source_robot = _extract_block(source_world, r"^Robot \{$")
        self.assertEqual(source_robot.count("stopSpringDamper 100 1"), 12)
        self.assertNotIn("stopSpringDamper", self.world)
        self.assertEqual(self.world.count("springConstant 100"), 12)
        self.assertNotIn('coordinateSystem "NUE"', self.world)
        self.assertNotIn("frictionMaterial ContactProperties", self.world)
        for leg in ("fr", "fl", "hr", "hl"):
            for joint in ("abd", "hip", "kn"):
                self.assertEqual(
                    self.world.count(f'name "{leg}_{joint}_motor"'), 1
                )
                self.assertEqual(
                    self.world.count(f'name "{leg}_{joint}_sensor"'), 1
                )

    def test_static_flat_floor_and_scene_nodes(self) -> None:
        """检查唯一静态平地和必需场景节点。"""

        floor = _extract_block(self.world, r"^DEF FLAT_FLOOR Solid \{$")
        self.assertIn('name "floor"', floor)
        self.assertIn("translation 0 0 -0.05", floor)
        self.assertIn("size 20 20 0.1", floor)
        self.assertIn("boundingObject Box", floor)
        self.assertNotIn("physics Physics", floor)
        self.assertRegex(
            floor,
            r"translation 0 0 -0\.05[\s\S]*size 20 20 0\.1",
        )
        self.assertEqual(self.world.count("Viewpoint {"), 1)
        self.assertEqual(self.world.count("Background {"), 1)
        self.assertEqual(self.world.count("DirectionalLight {"), 1)
        self.assertEqual(
            len(re.findall(r"(?m)^DEF FLAT_FLOOR Solid \{", self.world)), 1
        )
        for obstacle in ("hurdle", "stairs", "ramp", "pit", "box_platform"):
            self.assertNotRegex(
                self.world,
                rf'(?m)^\s*name\s+"{obstacle}[^"]*"\s*$',
            )

    def test_toe_contacts_are_zero_bounce(self) -> None:
        """检查四条腿材料均与地面零回弹。"""

        world_info = _extract_block(self.world, r"^WorldInfo \{$")
        for material in ("fr_toe", "fl_toe", "hr_toe", "hl_toe"):
            pattern = (
                rf'material1 "{material}"[\s\S]*?material2 "default"'
                rf"[\s\S]*?bounce 0\s+bounceVelocity 0"
            )
            self.assertRegex(world_info, pattern)

    def test_official_constants_are_present(self) -> None:
        """检查官方尺寸、质量和惯量没有在生成过程中丢失。"""

        source_world = self.generator.DEFAULT_SOURCE_WORLD.read_text(
            encoding="utf-8",
        )
        source_robot = _extract_block(source_world, r"^Robot \{$")
        world_robot = _extract_block(
            self.world,
            r"^DEF MINI_CHEETAH Robot \{$",
        )
        _strict_model_compare(world_robot, source_robot)
        _strict_visual_compare(
            world_robot,
            REPOSITORY_ROOT / "webots-sim/urdf/mini_cheetah.urdf",
        )

        required = (
            "Box { size 0.38 0.1 0.06 }",
            "mass 3.3",
            "mass 0.54",
            "mass 0.634",
            "mass 0.214",
            "0.011253 0.036203 0.042673",
            "0.000381 0.00056 0.000444",
            "0.001983 0.002103 0.000408",
            "0.00027 0.000273 3.1e-05",
            "anchor 0.19 -0.049 0",
            "anchor 0.19 0.049 0",
            "anchor -0.19 -0.049 0",
            "anchor -0.19 0.049 0",
            "translation 0.19 -0.111 0",
            "translation 0.19 0.111 0",
            "translation -0.19 -0.111 0",
            "translation -0.19 0.111 0",
            "size 0.04 0.04 0.209",
            "size 0.03 0.03 0.18",
            "radius 0.015",
        )
        for literal in required:
            with self.subTest(literal=literal):
                self.assertIn(literal, self.world)
        self.assertEqual(self.world.count("geometry Mesh {"), 13)
        self.assertEqual(self.world.count("mini_body.dae"), 1)
        self.assertEqual(self.world.count("mini_abad.dae"), 4)
        self.assertEqual(self.world.count("mini_upper_link.dae"), 4)
        self.assertEqual(self.world.count("mini_lower_link.dae"), 4)
        self.assertEqual(self.world.count("geometry Sphere {"), 4)
        self.assertEqual(self.world.count("diffuseColor 0.2 0.2 0.2"), 4)
        robot_block = _extract_block(self.world, r"^DEF MINI_CHEETAH Robot \{$")
        self.assertNotIn("PBRAppearance", robot_block)
        matrix_blocks = re.findall(
            r"inertiaMatrix \[([\s\S]*?)\]",
            self.world,
        )
        self.assertEqual(len(matrix_blocks), 13)
        for block in matrix_blocks:
            numbers = re.findall(
                r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?",
                block,
            )
            with self.subTest(block=block):
                self.assertEqual(len(numbers), 6)

    def test_anchor_endpoint_and_zero_world_coordinates(self) -> None:
        """逐关节确认 endPoint 换算和零位坐标误差小于 1 mm。"""

        generated = self.generator.generate_robot()
        source = self.generator.DEFAULT_SOURCE_WORLD.read_text(encoding="utf-8")
        count, endpoint_error, zero_error = self.generator._validate_conversion(
            generated,
            source,
        )
        self.assertEqual(count, 12)
        self.assertLess(endpoint_error, 0.001)
        self.assertLess(zero_error, 0.001)

    def test_supervisor_is_read_only(self) -> None:
        """静态确认 supervisor 没有任何电机控制调用。"""

        compile(
            self.supervisor_source,
            str(SUPERVISOR_PATH),
            "exec",
        )
        forbidden = (
            "wb_motor_",
            ".setTorque(",
            ".setVelocity(",
            ".setPosition(",
            ".setForce(",
            ".setJointPosition(",
            ".addForce(",
            ".addTorque(",
            "setVelocity(",
        )
        for token in forbidden:
            with self.subTest(token=token):
                self.assertNotIn(token, self.supervisor_source)
        self.assertIn("ACCEPTANCE_RESULT", self.supervisor_source)
        self.assertIn("DURATION_SECONDS = 10.0", self.supervisor_source)
        self.assertIn("EXPECTED_JOINT_NAMES", self.supervisor_source)

    def test_supervisor_node_and_artifacts(self) -> None:
        """检查 world 绑定独立 supervisor 且日志路径固定。"""

        supervisor = _extract_block(
            self.world,
            r"^DEF ACCEPTANCE_SUPERVISOR Robot \{$",
        )
        self.assertIn("supervisor TRUE", supervisor)
        self.assertIn('controller "acceptance_supervisor"', self.world)
        self.assertNotIn("RotationalMotor", supervisor)
        self.assertNotIn("PositionSensor", supervisor)
        self.assertIn("acceptance_samples.jsonl", self.supervisor_source)
        self.assertIn("acceptance_summary.json", self.supervisor_source)
        self.assertTrue((PROJECT_ROOT / "artifacts").is_dir())


if __name__ == "__main__":
    unittest.main()
