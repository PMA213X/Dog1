"""YoboGo-10S 项目资产与执行器配置。"""

from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg


# 训练工作区根目录与唯一允许引用的项目 USD。
PROJECT_ROOT = Path(__file__).resolve().parents[2]
USD_PATH = PROJECT_ROOT / "model" / "yobogo_mini_cheetah_v1" / "yobogo_mini_cheetah.usd"

# 环境与智能体入口名称。
TASK_ID = "YoboGo-Velocity-Flat-v0"
ENV_CFG_ENTRY_POINT = "training.envs.velocity_flat_env_cfg:YoboGoVelocityFlatEnvCfg"
RSL_RL_CFG_ENTRY_POINT = "training.agents.rsl_rl_ppo_cfg:YoboGoFlatPPORunnerCfg"

# 物理步长与控制周期：0.0004 s × 5 = 0.002 s，即 500 Hz。
PHYSICS_DT = 0.0004
DECIMATION = 5
CONTROL_PERIOD_S = 0.002

# YoboGo 控制数组固定顺序：leg0=FR、leg1=FL、leg2=RR、leg3=RL。
LEG_NAMES = ("FR", "FL", "RR", "RL")
JOINT_NAMES = [
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
FOOT_BODY_NAMES = [
    "leg0_foot_link",
    "leg1_foot_link",
    "leg2_foot_link",
    "leg3_foot_link",
]
THIGH_BODY_NAMES = [
    "leg0_thigh_link",
    "leg1_thigh_link",
    "leg2_thigh_link",
    "leg3_thigh_link",
]

# 来源：YoboGo-control/robot-software/config/initial_jpos_ctrl.yaml。
# 这是实机控制侧逻辑坐标，下游 rt_spi 还会做逐腿符号/零偏换算，
# 不能直接当作 MIT ORCAgym URDF 的几何关节角。
TARGET_JOINT_POS = {
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

# 保持 ORCAgym 的 axis/rpy 不变，采用已由只读 FK+collision 诊断确认的
# 几何站姿：每腿 HAA=0、HFE=-π/4、KFE=1.865468294 rad。
SIM_INITIAL_JOINT_POS = {
    "leg0_abad_joint": 0.0,
    "leg0_hip_joint": -0.785398163,
    "leg0_knee_joint": 1.865468294,
    "leg1_abad_joint": 0.0,
    "leg1_hip_joint": -0.785398163,
    "leg1_knee_joint": 1.865468294,
    "leg2_abad_joint": 0.0,
    "leg2_hip_joint": -0.785398163,
    "leg2_knee_joint": 1.865468294,
    "leg3_abad_joint": 0.0,
    "leg3_hip_joint": -0.785398163,
    "leg3_knee_joint": 1.865468294,
}
# root 出生高度与四足碰撞最低点 -0.26 m 对齐，确保出生即足端触地。
SIM_INITIAL_ROOT_POS = (0.0, 0.0, 0.26)

# 来源：YoboGo-control/robot-software/config/mc-mit-ctrl-user-parameters.yaml。
PD_KP = 3.0
PD_KD = {"abad": 1.0, "hip": 0.2, "knee": 0.2}
TORQUE_LIMIT = {"abad": 17.0, "hip": 17.0, "knee": 26.0}


def _joint_type(joint_name: str) -> str:
    """返回关节类型，用于施加逐类型 PD 与力矩限制。"""
    if joint_name.endswith("_abad_joint"):
        return "abad"
    if joint_name.endswith("_hip_joint"):
        return "hip"
    if joint_name.endswith("_knee_joint"):
        return "knee"
    raise ValueError(f"未知关节名称：{joint_name}")


def _build_actuators() -> dict[str, ImplicitActuatorCfg]:
    """按 JOINT_NAMES 的 12 路顺序生成逐关节隐式 PD 执行器。"""
    actuators: dict[str, ImplicitActuatorCfg] = {}
    for joint_name in JOINT_NAMES:
        joint_type = _joint_type(joint_name)
        torque_limit = TORQUE_LIMIT[joint_type]
        actuators[joint_name] = ImplicitActuatorCfg(
            joint_names_expr=[joint_name],
            effort_limit=torque_limit,
            effort_limit_sim=torque_limit,
            stiffness=PD_KP,
            damping=PD_KD[joint_type],
        )
    return actuators


def _validate_contract() -> None:
    """在配置导入阶段立即拦截关节顺序、周期和姿态契约错误。"""
    if len(JOINT_NAMES) != 12 or len(set(JOINT_NAMES)) != 12:
        raise RuntimeError("12 个驱动关节必须唯一且完整")
    if list(TARGET_JOINT_POS) != JOINT_NAMES:
        raise RuntimeError("target_jpos 字典顺序必须等于 JOINT_NAMES")
    if list(SIM_INITIAL_JOINT_POS) != JOINT_NAMES:
        raise RuntimeError("SIM 初态字典顺序必须等于 JOINT_NAMES")
    if TARGET_JOINT_POS == SIM_INITIAL_JOINT_POS:
        raise RuntimeError("实机 target_jpos 与 URDF/Isaac 仿真初态必须拆分")
    if len(SIM_INITIAL_ROOT_POS) != 3 or SIM_INITIAL_ROOT_POS[2] != 0.26:
        raise RuntimeError("仿真 root 出生高度必须为 0.26 m")
    if LEG_NAMES != ("FR", "FL", "RR", "RL"):
        raise RuntimeError("四腿映射必须为 FR/FL/RR/RL")
    if abs(PHYSICS_DT * DECIMATION - CONTROL_PERIOD_S) > 1.0e-12:
        raise RuntimeError("physics_dt × decimation 必须等于 0.002 s")
    if TORQUE_LIMIT != {"abad": 17.0, "hip": 17.0, "knee": 26.0}:
        raise RuntimeError("关节力矩限制必须为 17/17/26 N·m")


_validate_contract()


YOBOGO_MINI_CHEETAH_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(USD_PATH),
        activate_contact_sensors=True,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=SIM_INITIAL_ROOT_POS,
        joint_pos=dict(SIM_INITIAL_JOINT_POS),
        joint_vel={".*": 0.0},
    ),
    actuators=_build_actuators(),
)
