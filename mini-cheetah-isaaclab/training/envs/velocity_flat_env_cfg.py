"""官方 manager-based 平地速度任务的 YoboGo 配置。"""

from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import (
    LocomotionVelocityRoughEnvCfg,
)

import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp

from .robot_cfg import (
    CONTROL_PERIOD_S,
    DECIMATION,
    FOOT_BODY_NAMES,
    SIM_INITIAL_JOINT_POS,
    TARGET_JOINT_POS,
    JOINT_NAMES,
    PHYSICS_DT,
    THIGH_BODY_NAMES,
    USD_PATH,
    YOBOGO_MINI_CHEETAH_CFG,
)


def _ordered_joint_entity(name: str = "robot") -> SceneEntityCfg:
    """构造保持 leg0→leg3、abad→hip→knee 顺序的关节实体。"""
    return SceneEntityCfg(name, joint_names=list(JOINT_NAMES), preserve_order=True)


@configclass
class YoboGoVelocityFlatEnvCfg(LocomotionVelocityRoughEnvCfg):
    """YoboGo-10S 平地速度跟踪环境。"""

    def __post_init__(self):
        """套用官方平地速度骨架并写入本项目固定契约。"""
        super().__post_init__()

        # 只引用本项目 USD，不引用任何官方或其他机器人资产。
        self.scene.robot = YOBOGO_MINI_CHEETAH_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        if self.scene.robot.spawn.usd_path != str(USD_PATH):
            raise RuntimeError("场景 USD 路径必须指向项目资产")

        # 首版短训练固定小规模，避免在资产验收前占用大量显存。
        self.scene.num_envs = 4
        self.scene.env_spacing = 2.5

        # 官方平地速度任务：关闭高程图、地形课程并切到平面。
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None
        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        self.curriculum.terrain_levels = None

        # 固定 500 Hz 控制周期。
        self.decimation = DECIMATION
        self.sim.dt = PHYSICS_DT
        if abs(self.sim.dt * self.decimation - CONTROL_PERIOD_S) > 1.0e-12:
            raise RuntimeError("物理步长与 decimation 不满足 0.002 s 契约")
        self.sim.render_interval = self.decimation
        self.sim.physics_material = self.scene.terrain.physics_material
        if self.scene.contact_forces is not None:
            self.scene.contact_forces.update_period = self.sim.dt

        # 动作维度、顺序与初始目标严格跟随 YoboGo 的 12 路数组。
        self.actions.joint_pos = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=list(JOINT_NAMES),
            preserve_order=True,
            scale=0.5,
            use_default_offset=True,
        )

        # 本体观测显式锁定同一顺序，避免依赖 USD 内部关节排序。
        self.observations.policy.joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": _ordered_joint_entity()},
        )
        self.observations.policy.joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={"asset_cfg": _ordered_joint_entity()},
        )

        # 禁止随机改变 9 kg 总质量口径。
        self.events.add_base_mass = None

        # 官方骨架沿用其机器人根链接名 base；本项目根链接已确认为 base_link。
        self.events.base_external_force_torque.params["asset_cfg"].body_names = (
            "base_link"
        )

        # reset 事件按 ArticulationCfg 默认姿态缩放；系数固定为 1，
        # 因此精确回到 URDF/Isaac 几何 SIM 初态，而不是实机 target_jpos。
        self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
        self.events.reset_robot_joints.params["velocity_range"] = (0.0, 0.0)

        # 关节姿态/速度相关奖励与惩罚同样锁定顺序。
        self.rewards.dof_torques_l2.params["asset_cfg"] = _ordered_joint_entity()
        self.rewards.dof_acc_l2.params["asset_cfg"] = _ordered_joint_entity()
        self.rewards.dof_pos_limits.params["asset_cfg"] = _ordered_joint_entity()

        # 使用项目 link 命名匹配足端与大腿接触。
        self.rewards.feet_air_time.params["sensor_cfg"] = SceneEntityCfg(
            "contact_forces",
            body_names=list(FOOT_BODY_NAMES),
            preserve_order=True,
        )
        self.rewards.undesired_contacts.params["sensor_cfg"] = SceneEntityCfg(
            "contact_forces",
            body_names=list(THIGH_BODY_NAMES),
            preserve_order=True,
        )
        self.terminations.base_contact.params["sensor_cfg"] = SceneEntityCfg(
            "contact_forces",
            body_names=["base_link"],
            preserve_order=True,
        )

        # 官方平地任务权重。
        self.rewards.flat_orientation_l2.weight = -5.0
        self.rewards.dof_torques_l2.weight = -2.5e-5
        self.rewards.feet_air_time.weight = 0.5

        if list(TARGET_JOINT_POS) != JOINT_NAMES:
            raise RuntimeError("target_jpos 与动作顺序不一致")
        if list(SIM_INITIAL_JOINT_POS) != JOINT_NAMES:
            raise RuntimeError("SIM 初态与动作顺序不一致")
        if TARGET_JOINT_POS == SIM_INITIAL_JOINT_POS:
            raise RuntimeError("实机 target_jpos 与仿真初态必须拆分")
