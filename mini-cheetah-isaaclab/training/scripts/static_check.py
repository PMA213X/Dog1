"""训练代码的无 Isaac 启动静态检查。"""

from __future__ import annotations

import ast
from pathlib import Path
import sys

# 静态检查不生成任何 __pycache__ 文件。
sys.dont_write_bytecode = True

from _bootstrap import TRAINING_ROOT, ensure_project_on_path


ensure_project_on_path()


def _literal_assignment(tree: ast.Module, name: str):
    """读取模块顶层的字面量赋值，避免导入 Isaac Lab。"""
    for node in tree.body:
        if isinstance(node, ast.Assign):
            if any(isinstance(target, ast.Name) and target.id == name for target in node.targets):
                return ast.literal_eval(node.value)
    raise AssertionError(f"缺少顶层字面量：{name}")


def _read(path: Path) -> tuple[str, ast.Module]:
    """读取并编译单个 Python 文件，但不执行导入。"""
    source = path.read_text(encoding="utf-8")
    tree = ast.parse(source, filename=str(path))
    compile(source, str(path), "exec")
    return source, tree


def main() -> int:
    """执行语法、资产、顺序、周期、PD、力矩和注册入口检查。"""
    failures: list[str] = []
    python_files = sorted(TRAINING_ROOT.rglob("*.py"))
    if not python_files:
        failures.append("training 下没有 Python 文件")

    parsed: dict[Path, tuple[str, ast.Module]] = {}
    for path in python_files:
        try:
            parsed[path] = _read(path)
        except Exception as exc:  # noqa: BLE001 - 静态检查需要汇总全部错误
            failures.append(f"{path}: {type(exc).__name__}: {exc}")

    robot_path = TRAINING_ROOT / "envs" / "robot_cfg.py"
    env_path = TRAINING_ROOT / "envs" / "velocity_flat_env_cfg.py"
    reg_path = TRAINING_ROOT / "envs" / "__init__.py"
    agent_path = TRAINING_ROOT / "agents" / "rsl_rl_ppo_cfg.py"
    check_path = TRAINING_ROOT / "scripts" / "check_env.py"
    for required in (robot_path, env_path, reg_path, agent_path, check_path):
        if required not in parsed:
            failures.append(f"关键文件未通过语法检查：{required}")

    if not failures:
        robot_source, robot_tree = parsed[robot_path]
        env_source, _ = parsed[env_path]
        reg_source, _ = parsed[reg_path]
        agent_source, _ = parsed[agent_path]
        check_source, _ = parsed[check_path]

        expected_joints = [
            "leg0_abad_joint",
            "leg0_hip_joint",
            "leg0_knee_joint",
            "leg1_abad_joint",
            "leg1_hip_joint",
            "leg1_knee_joint",
            "leg2_abad_joint",
            "leg2_hip_joint",
            "leg2_knee_joint",
            "leg3_abad_joint",
            "leg3_hip_joint",
            "leg3_knee_joint",
        ]
        expected_target = {
            "leg0_abad_joint": -0.6,
            "leg0_hip_joint": -1.0,
            "leg0_knee_joint": 2.7,
            "leg1_abad_joint": 0.6,
            "leg1_hip_joint": -1.0,
            "leg1_knee_joint": 2.7,
            "leg2_abad_joint": -0.6,
            "leg2_hip_joint": -1.0,
            "leg2_knee_joint": 2.7,
            "leg3_abad_joint": 0.6,
            "leg3_hip_joint": -1.0,
            "leg3_knee_joint": 2.7,
        }
        expected_sim_pose = {
            joint_name: value
            for slot in range(4)
            for joint_name, value in (
                (f"leg{slot}_abad_joint", 0.0),
                (f"leg{slot}_hip_joint", -0.785398163),
                (f"leg{slot}_knee_joint", 1.865468294),
            )
        }
        checks = [
            (
                _literal_assignment(robot_tree, "LEG_NAMES")
                == ("FR", "FL", "RR", "RL"),
                "四腿映射必须为 FR/FL/RR/RL",
            ),
            (
                _literal_assignment(robot_tree, "JOINT_NAMES") == expected_joints,
                "12 关节顺序必须为 leg0..3、abad/hip/knee",
            ),
            (
                _literal_assignment(robot_tree, "TARGET_JOINT_POS")
                == expected_target,
                "TARGET_JOINT_POS 必须与 YoboGo target_jpos 完全一致",
            ),
            (
                _literal_assignment(robot_tree, "SIM_INITIAL_JOINT_POS")
                == expected_sim_pose,
                "SIM 初态必须为 [0,-0.785398163,1.865468294] × 4",
            ),
            (
                _literal_assignment(robot_tree, "SIM_INITIAL_ROOT_POS")
                == (0.0, 0.0, 0.26),
                "仿真 root 出生高度必须为 0.26 m",
            ),
            (
                _literal_assignment(robot_tree, "TARGET_JOINT_POS")
                != _literal_assignment(robot_tree, "SIM_INITIAL_JOINT_POS"),
                "实机 target_jpos 与 URDF/Isaac 仿真初态必须拆分",
            ),
            (
                abs(
                    _literal_assignment(robot_tree, "PHYSICS_DT")
                    * _literal_assignment(robot_tree, "DECIMATION")
                    - _literal_assignment(robot_tree, "CONTROL_PERIOD_S")
                )
                <= 1.0e-12,
                "physics_dt × decimation 必须等于 0.002 s",
            ),
            (
                _literal_assignment(robot_tree, "PD_KP") == 3.0
                and _literal_assignment(robot_tree, "PD_KD")
                == {"abad": 1.0, "hip": 0.2, "knee": 0.2},
                "PD 必须为 Kp=3、Kd=1/0.2/0.2",
            ),
            (
                _literal_assignment(robot_tree, "TORQUE_LIMIT")
                == {"abad": 17.0, "hip": 17.0, "knee": 26.0},
                "力矩限制必须为 17/17/26 N·m",
            ),
            (
                'USD_PATH = PROJECT_ROOT / "model" / "yobogo_mini_cheetah_v1" / "yobogo_mini_cheetah.usd"'
                in robot_source,
                "ArticulationCfg 只能引用项目 USD 路径",
            ),
            (
                env_source.count("preserve_order=True") >= 5,
                "动作与关节观测必须显式 preserve_order",
            ),
            (
                'self.events.base_external_force_torque.params["asset_cfg"].body_names'
                in env_source
                and '"base_link"' in env_source,
                "官方外力事件必须绑定项目根链接 base_link",
            ),
            (
                'joint_names=list(JOINT_NAMES)' in env_source
                and 'use_default_offset=True' in env_source,
                "JointPositionActionCfg 必须使用固定顺序和默认姿态偏置",
            ),
            (
                "TASK_ID" in reg_source and "gym.register" in reg_source,
                "必须注册本项目 Gym 任务",
            ),
            (
                "max_iterations = 100" in agent_source,
                "RSL-RL 短训练入口必须限制为 100 iteration",
            ),
            (
                "robot.find_joints(JOINT_NAMES, preserve_order=True)" in check_source
                and "net_forces_w_history[:, :, foot_ids, :]" in check_source,
                "单环境检查必须按固定关节顺序与四维接触历史索引",
            ),
            (
                "foot_collision_min" in check_source
                and "base_collision_min" in check_source
                and "_sim_initial_pose_tensor" in check_source,
                "动态检查必须断言 SIM 初态和足端/机身碰撞几何",
            ),
        ]
        for condition, message in checks:
            if not condition:
                failures.append(message)

        task_id = _literal_assignment(robot_tree, "TASK_ID")
        if task_id != "YoboGo-Velocity-Flat-v0":
            failures.append("TASK_ID 必须为 YoboGo-Velocity-Flat-v0")
        if "id=TASK_ID" not in reg_source:
            failures.append("gym.register 必须使用 TASK_ID")

        forbidden_tokens = (
            "official-mini-cheetah/",
            "ISAAC_NUCLEUS_DIR",
            "Nucleus",
            "Robots/",
        )
        for token in forbidden_tokens:
            if token in robot_source or token in env_source:
                failures.append(f"发现禁用资产引用：{token}")

    if failures:
        print("STATIC_CHECK_FAIL")
        for failure in failures:
            print(f"- {failure}")
        return 1

    print(f"STATIC_CHECK_OK files={len(python_files)}")
    print(f"training_root={TRAINING_ROOT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
