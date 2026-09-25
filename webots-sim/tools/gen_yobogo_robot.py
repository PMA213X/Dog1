#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成 REAL YoboGo-10S 四足机器人的 Webots R2025a Robot 节点。

Scheme A：将运动学链缩放到 YoboGo-10S 包络尺寸。
包络目标：站立 485 x 275 x 300 mm，质量 10.5 kg
  （来源：YoboGo-10S 使用说明书 §1.3）。

用法：
  python3 gen_yobogo_robot.py            # 将机器人 VRML 打印到 stdout
  python3 gen_yobogo_robot.py out.wbt    # 将机器人 VRML 写入文件

下面每个数值常量都标注了出处：
  [说明书]  YoboGo-10S 使用说明书 §1.3
  [yaml]    YoboGo-control/robot-software/config/mc-mit-ctrl-user-parameters.yaml
  [估算]    工程估算（并非来自数据手册）
  [合同]    生成合同（Scheme A）中的明确数值
"""

import sys

# ============================================================================
# ==== 相机可调参数 (Camera tunable) ====
# fov_rad: 水平FOV（弧度）。0.96~1.22 对应 55°~70°，推荐 1.05 (60°)
# pitch_rad: 俯仰角（弧度，负值=低头）。-0.52~-0.70 对应 -30°~-40°，推荐 -0.61 (-35°)
#
# Webots Camera 光轴 = 相机局部坐标系的 +X 轴（已用彩色标记实测验证，见文末
# CAMERA_ORIENT_NOTES）。俯仰低头 angle 弧度 => 绕 +Y 轴右手旋转 -angle。
# ============================================================================
CAMERA_FOV_RAD = 1.05        # [合同] 推荐 1.05 rad ≈ 60°
CAMERA_PITCH_RAD = -0.44     # [调参] -0.44 rad ≈ -25° 低头；为看到 3 m 外的球，比循迹用的 -35° 抬高了一些
CAMERA_WIDTH = 640           # [合同]
CAMERA_HEIGHT = 480          # [合同]
CAMERA_NAME = "front_camera"  # [合同]
CAMERA_TRANS = (0.20, 0.0, 0.055)  # [合同] body 系：+0.20 前, 0 左右, +0.055 上
# 世界高度 ≈ 0.26 + 0.055 = 0.315 m ≈ 0.33 m 目标 (可接受)

# ---- Robot 级常量 ----
ROBOT_NAME = "yobogo_10s"                 # [合同]
ROBOT_CONTROLLER = "mini_cheetah_controller"  # [合同] 控制器依赖此名
ROBOT_TRANSLATION = (3.0, 0.0, 0.26)      # [调参] x=3.0 距场心球 3 m（原 8 m 超出相机视距）；z=0.26 [yaml] des_p[2]
ROBOT_ROTATION = (0.0, 0.0, 1.0, 3.14159265)  # [合同] 朝向 -X
CONTACT_MATERIAL = "body"                 # [合同]
SELF_COLLISION = True                     # [合同]
MAX_TORQUE = 18.0                         # [说明书] 说明书§六 CAN 协议力矩字段 -18~18 N·m
JOINT_DAMPING = 1.0                       # [合同]
TOTAL_MASS = 10.5                         # [说明书] 总质量 10.5 kg（说明书 §1.3）

# ---- 机体 ----
BODY_SIZE = (0.40, 0.13, 0.10)            # [合同] L×W×H m
BODY_MASS = 3.92                          # [合同] kg
# 机体惯量直接取控制器模型 RPC_inertia [yaml:77]，单位 kg·m²
BODY_INERTIA_MOMENTS = (0.07, 0.26, 0.242)  # [yaml:77] RPC_inertia
BODY_INERTIA_PRODUCTS = (0.0, 0.0, 0.0)     # [合同] products 0 0 0
BODY_COM = (0.0, 0.0, 0.0)                # [合同]
BODY_LINEAR_DAMPING = 0.5                 # [合同]
BODY_ANGULAR_DAMPING = 0.5                # [合同]

# ---- 腿部运动学（body 坐标系）----
# 髋安装点（abd 关节）。x=±0.18, y=±0.052 [合同]
HIP_MOUNTS = {
    "fr": (0.18, -0.052, 0.0),
    "fl": (0.18, 0.052, 0.0),
    "hr": (-0.18, -0.052, 0.0),
    "hl": (-0.18, 0.052, 0.0),
}
# side_offset: +1 左侧 (fl/hl), -1 右侧 (fr/hr)  [合同]
SIDE_OFFSET = {"fr": -1, "fl": 1, "hr": -1, "hl": 1}
# abd 连杆到 hip-pitch 关节的偏置：0 ±0.065 0  [合同]
ABD_OFFSET_Y = 0.065                      # [合同]
THIGH_LEN = 0.14                          # [合同] 与 shank 之和 = 0.26 = des_p[2]
SHANK_LEN = 0.12                          # [合同]
# knee 锚点 / endPoint 在 thigh 坐标系中的平移：0 0 -0.14  [合同]
KNEE_IN_THIGH = (0.0, 0.0, -0.14)
SHANK_VIS_Z = -0.06                       # [合同] shank 视觉中心
TOE_IN_SHANK = (0.0, 0.0, -0.12)          # [合同] 足端球
TOE_RADIUS = 0.02                         # [合同]

# ---- 连杆视觉件（Box 尺寸；visual Transform 与 boundingObject Transform 相同）----
ABD_BOX = (0.04, 0.065, 0.04)             # 修正: 腿段长 0.065, 置中后外缘=0.052+0.065=0.117 → 总宽 0.234; 足端 0.274 主导包络
THIGH_BOX = (0.05, 0.05, 0.14)            # [合同]
SHANK_BOX = (0.04, 0.04, 0.12)            # [合同]
ABD_VIS_T = (0.0, None, 0.0)              # y 按腿填充：±0.065  [合同]
THIGH_VIS_T = (0.0, 0.0, -0.07)           # [合同]
SHANK_VIS_T = (0.0, 0.0, -0.06)           # [合同]

# ---- 质量（单腿 1.644 = 0.64+0.75+0.254；总质量 10.496 ≈10.5（说明书））----
ABD_MASS = 0.64                           # [合同]
THIGH_MASS = 0.75                         # [合同]
SHANK_MASS = 0.254                        # [合同] 0.64+0.75+0.254 = 1.644
# 质心：abd 0 ±0.03 0; thigh 0 ∓0.01 -0.07; shank 0 0 -0.06  [合同]
ABD_COM_Y = 0.03                          # * side_offset
THIGH_COM_Y = -0.01                       # * side_offset
THIGH_COM_Z = -0.07
SHANK_COM = (0.0, 0.0, -0.06)

# ---- 惯量（估算值——并非来自数据手册）[估算] ----
# 见注释：按细长杆/块体量级估算的各向同性-ish 主惯量，products 全 0。
ABD_INERTIA_MOMENTS = (0.001, 0.0012, 0.001)      # [估算]
THIGH_INERTIA_MOMENTS = (0.003, 0.0032, 0.0006)   # [估算]
SHANK_INERTIA_MOMENTS = (0.001, 0.001, 0.0002)    # [估算]
ZERO_PRODUCTS = (0.0, 0.0, 0.0)                   # [合同/估算]

# ---- 外观 ----
BODY_COLOR = (0.15, 0.15, 0.15)
LINK_COLOR = (0.2, 0.2, 0.2)
TOE_COLOR = (0.1, 0.1, 0.1)

LEGS = ("fr", "fl", "hr", "hl")  # 输出中使用的顺序


# ----------------------------------------------------------------------------
def _fmt(v, nd=8):
    """紧凑且稳定地格式化浮点数（保留合同级精度）。"""
    if isinstance(v, int):
        return str(v)
    s = f"{v:.{nd}f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def _vec3(v):
    return f"{_fmt(v[0])} {_fmt(v[1])} {_fmt(v[2])}"


def _shape_box(color, size, indent):
    p = " " * indent
    return (
        f"{p}Shape {{\n"
        f"{p}  appearance PBRAppearance {{\n"
        f"{p}    baseColor {_vec3(color)}\n"
        f"{p}    roughness 0.8\n"
        f"{p}    metalness 0.1\n"
        f"{p}  }}\n"
        f"{p}  geometry Box {{\n"
        f"{p}    size {_vec3(size)}\n"
        f"{p}  }}\n"
        f"{p}}}"
    )


def _shape_sphere(color, radius, indent):
    p = " " * indent
    return (
        f"{p}Shape {{\n"
        f"{p}  appearance PBRAppearance {{\n"
        f"{p}    baseColor {_vec3(color)}\n"
        f"{p}    roughness 0.8\n"
        f"{p}    metalness 0.1\n"
        f"{p}  }}\n"
        f"{p}  geometry Sphere {{\n"
        f"{p}    radius {_fmt(radius)}\n"
        f"{p}  }}\n"
        f"{p}}}"
    )


def _transform(translation, children_text, indent):
    p = " " * indent
    return (
        f"{p}Transform {{\n"
        f"{p}  translation {_vec3(translation)}\n"
        f"{p}  children [\n"
        f"{children_text}\n"
        f"{p}  ]\n"
        f"{p}}}"
    )


def _physics(mass, com, moments, products, indent, damping=True):
    p = " " * indent
    d = ""
    if damping:
        d = (
            f"{p}  damping Damping {{\n"
            f"{p}    linear 0.5\n"
            f"{p}    angular 0.5\n"
            f"{p}  }}\n"
        )
    return (
        f"{p}physics Physics {{\n"
        f"{p}  mass {_fmt(mass)}\n"
        f"{p}  centerOfMass {_vec3(com)}\n"
        f"{p}  inertiaMatrix [\n"
        f"{p}    {_fmt(moments[0])} {_fmt(moments[1])} {_fmt(moments[2])}\n"
        f"{p}    {_fmt(products[0])} {_fmt(products[1])} {_fmt(products[2])}\n"
        f"{p}  ]\n"
        f"{p}  density -1\n"
        f"{d}"
        f"{p}}}"
    )


def _motor_sensor(leg, joint, indent):
    """RotationalMotor + PositionSensor 配对。名称必须与控制器一致。"""
    p = " " * indent
    return (
        f"{p}device [\n"
        f"{p}  RotationalMotor {{\n"
        f"{p}    name \"{leg}_{joint}_motor\"\n"
        f"{p}    maxTorque {_fmt(MAX_TORQUE)}\n"
        f"{p}  }}\n"
        f"{p}  PositionSensor {{\n"
        f"{p}    name \"{leg}_{joint}_sensor\"\n"
        f"{p}  }}\n"
        f"{p}]"
    )


# ----------------------------------------------------------------------------
def build_camera():
    """相机块：俯仰/FOV 可调，置于 body 坐标系。

    Webots R2025a Camera 字段名是 `fieldOfView`（不是 `fov`）— 见 Camera.wrl。
    光轴为相机局部 +X 轴（已实测验证，见文末说明）。
    低头 35° => rotation 0 1 0 0.61（绕 +Y 轴右手旋转）。
    """
    # 轴角旋转：pitch_rad=-0.61（低头）=> angle = -pitch_rad = 0.61
    rot_angle = -CAMERA_PITCH_RAD
    p2 = " " * 2
    p4 = " " * 4
    p6 = " " * 6
    p8 = " " * 8
    lines = []
    c = p2  # 注释与 Transform 同级缩进
    lines.append(f"{c}# ==== 相机可调参数 (Camera tunable) ====")
    lines.append(f"{c}# fov_rad: 水平FOV（弧度）。0.96~1.22 对应 55°~70°，推荐 1.05 (60°)")
    lines.append(f"{c}# pitch_rad: 俯仰角（弧度，负值=低头）。-0.52~-0.70 对应 -30°~-40°，推荐 -0.61 (-35°)")
    lines.append(f"{c}# 注：Webots R2025a 字段名是 fieldOfView（不是 fov）— 见 Camera.wrl")
    lines.append(f"{c}# Webots Camera 光轴 = 相机局部 +X（实测验证：原点朝 +X 时画面中心为 +X 处色标）。")
    lines.append(f"{c}# 俯仰低头 pitch_rad 弧度 => 父 Transform 绕 +Y 轴右手旋转 -pitch_rad。")
    lines.append(f"{c}# 当前: fov={_fmt(CAMERA_FOV_RAD)} ({_fmt(CAMERA_FOV_RAD * 180 / 3.14159265, 1)}°), "
                 f"pitch={_fmt(CAMERA_PITCH_RAD)} ({_fmt(CAMERA_PITCH_RAD * 180 / 3.14159265, 1)}°)")
    lines.append(f"{p2}Transform {{")
    lines.append(f"{p4}translation {_vec3(CAMERA_TRANS)}")
    lines.append(f"{p4}# pitch 由父 Transform 的 rotation 实现（绕 +Y，低头为正角）")
    lines.append(f"{p4}rotation 0 1 0 {_fmt(rot_angle)}")
    lines.append(f"{p4}# 备选（错误用法，仅记录）：rotation 1 0 0 {_fmt(CAMERA_PITCH_RAD)}")
    lines.append(f"{p4}#   —— 绕前向 X 轴是横滚(roll)，不是俯仰；勿用")
    lines.append(f"{p4}children [")
    lines.append(f"{p6}Camera {{")
    lines.append(f"{p8}name \"{CAMERA_NAME}\"")
    lines.append(f"{p8}width {CAMERA_WIDTH}")
    lines.append(f"{p8}height {CAMERA_HEIGHT}")
    lines.append(f"{p8}fieldOfView {_fmt(CAMERA_FOV_RAD)}")
    lines.append(f"{p8}# pitch 由父 Transform 的 rotation 实现，见上")
    lines.append(f"{p8}noise 0")
    lines.append(f"{p8}motionBlur 0")
    lines.append(f"{p6}}}")
    lines.append(f"{p4}]")
    lines.append(f"{p2}}}")
    return "\n".join(lines)


def build_leg(leg):
    """单条完整腿：abd -> thigh -> shank（+ 足端视觉球）。"""
    side = SIDE_OFFSET[leg]           # +1 左 / -1 右
    mount = HIP_MOUNTS[leg]
    abd_offset = (0.0, side * ABD_OFFSET_Y, 0.0)   # hip-pitch 关节在 abd 坐标系中的位置
    abd_vis_t = (0.0, side * ABD_OFFSET_Y / 2.0, 0.0)  # 盒居中于腿段中点, visual == bounding
    abd_com = (0.0, side * ABD_COM_Y, 0.0)
    thigh_com = (0.0, side * THIGH_COM_Y, THIGH_COM_Z)

    L = []  # 行缓冲
    a = L.append

    # ---------------- shank（最内层，先写以便理清嵌套）----------------
    # 文本上按由外到内输出；shank 在最深处。

    # 实际按自顶向下输出嵌套结构。
    ind = lambda n: " " * n

    # HingeJoint abd
    a(f"{ind(4)}HingeJoint {{")
    a(f"{ind(6)}jointParameters HingeJointParameters {{")
    a(f"{ind(8)}anchor {_vec3(mount)}")
    a(f"{ind(8)}axis 1 0 0")
    a(f"{ind(8)}dampingConstant 1.0")
    a(f"{ind(6)}}}")
    a(_motor_sensor(leg, "abd", 6))
    a(f"{ind(6)}endPoint Solid {{")
    a(f"{ind(8)}translation {_vec3(mount)}   # 必须与 joint anchor 一致")
    a(f"{ind(8)}children [")

    # abd 视觉件
    a(_transform(abd_vis_t, _shape_box(LINK_COLOR, ABD_BOX, 12), 10))

    # HingeJoint hip
    a(f"{ind(12)}HingeJoint {{")
    a(f"{ind(14)}jointParameters HingeJointParameters {{")
    a(f"{ind(16)}anchor {_vec3(abd_offset)}")
    a(f"{ind(16)}axis 0 -1 0")
    a(f"{ind(16)}dampingConstant 1.0")
    a(f"{ind(14)}}}")
    a(_motor_sensor(leg, "hip", 14))
    a(f"{ind(14)}endPoint Solid {{")
    a(f"{ind(16)}translation {_vec3(abd_offset)}   # 必须与 joint anchor 一致")
    a(f"{ind(16)}children [")

    # thigh 视觉件
    a(_transform(THIGH_VIS_T, _shape_box(LINK_COLOR, THIGH_BOX, 20), 18))

    # HingeJoint knee
    a(f"{ind(20)}HingeJoint {{")
    a(f"{ind(22)}jointParameters HingeJointParameters {{")
    a(f"{ind(24)}anchor {_vec3(KNEE_IN_THIGH)}")
    a(f"{ind(24)}axis 0 -1 0")
    a(f"{ind(24)}dampingConstant 1.0")
    a(f"{ind(22)}}}")
    a(_motor_sensor(leg, "kn", 22))
    a(f"{ind(22)}endPoint Solid {{")
    a(f"{ind(24)}translation {_vec3(KNEE_IN_THIGH)}   # 必须与 joint anchor 一致")
    a(f"{ind(24)}children [")

    # shank 视觉件
    a(_transform(SHANK_VIS_T, _shape_box(LINK_COLOR, SHANK_BOX, 28), 26))
    # 足端视觉球（仅视觉；bounding 由 shank 盒承担）
    a(_transform(TOE_IN_SHANK, _shape_sphere(TOE_COLOR, TOE_RADIUS, 28), 26))

    a(f"{ind(24)}]")
    a(f"{ind(24)}name \"{leg}_shank_link\"")
    a(f"{ind(24)}boundingObject Transform {{")
    a(f"{ind(26)}translation {_vec3(SHANK_VIS_T)}")
    a(f"{ind(26)}children [")
    a(f"{ind(28)}Box {{ size {_vec3(SHANK_BOX)} }}")
    a(f"{ind(26)}]")
    a(f"{ind(24)}}}")
    a(_physics(SHANK_MASS, SHANK_COM, SHANK_INERTIA_MOMENTS, ZERO_PRODUCTS, 24))
    a(f"{ind(22)}}}")   # endPoint shank
    a(f"{ind(20)}}}")   # HingeJoint knee

    a(f"{ind(16)}]")
    a(f"{ind(16)}name \"{leg}_thigh_link\"")
    a(f"{ind(16)}boundingObject Transform {{")
    a(f"{ind(18)}translation {_vec3(THIGH_VIS_T)}")
    a(f"{ind(18)}children [")
    a(f"{ind(20)}Box {{ size {_vec3(THIGH_BOX)} }}")
    a(f"{ind(18)}]")
    a(f"{ind(16)}}}")
    a(_physics(THIGH_MASS, thigh_com, THIGH_INERTIA_MOMENTS, ZERO_PRODUCTS, 16))
    a(f"{ind(14)}}}")   # endPoint thigh
    a(f"{ind(12)}}}")   # HingeJoint hip

    a(f"{ind(8)}]")
    a(f"{ind(8)}name \"{leg}_abd_link\"")
    a(f"{ind(8)}boundingObject Transform {{")
    a(f"{ind(10)}translation {_vec3(abd_vis_t)}")
    a(f"{ind(10)}children [")
    a(f"{ind(12)}Box {{ size {_vec3(ABD_BOX)} }}")
    a(f"{ind(10)}]")
    a(f"{ind(8)}}}")
    a(_physics(ABD_MASS, abd_com, ABD_INERTIA_MOMENTS, ZERO_PRODUCTS, 8))
    a(f"{ind(6)}}}")   # endPoint abd
    a(f"{ind(4)}}}")   # HingeJoint abd
    return "\n".join(L)


def build_robot():
    """完整 Robot 节点文本（从 'Robot {' 到闭合 '}'）。"""
    L = []
    a = L.append
    a("Robot {")
    a(f"  translation {_vec3(ROBOT_TRANSLATION)}")
    a(f"  rotation {_fmt(ROBOT_ROTATION[0])} {_fmt(ROBOT_ROTATION[1])} "
      f"{_fmt(ROBOT_ROTATION[2])} {_fmt(ROBOT_ROTATION[3])}")
    a(f"  name \"{ROBOT_NAME}\"")
    a("  children [")

    # ---- 机体上的 IMU 传感器（名称为控制器所需）----
    a("    # ---- 机体上的 IMU 传感器 ----")
    a("    InertialUnit {")
    a("      name \"inertial unit\"")
    a("    }")
    a("    Gyro {")
    a("      name \"gyro\"")
    a("    }")
    a("    Accelerometer {")
    a("      name \"accelerometer\"")
    a("    }")

    # ---- 相机（位于 IMU 传感器之后）----
    a(build_camera())

    # ---- 机体视觉件 ----
    a("    # ---- 机体视觉件（L x W x H = 0.40 x 0.13 x 0.10）[合同] ----")
    a(_shape_box(BODY_COLOR, BODY_SIZE, 4))

    # ---- 腿 ----
    for leg in LEGS:
        a(f"    # ---- {leg.upper()} 腿（原点在关节处）----")
        a(build_leg(leg))

    a("  ]")
    # 机体 boundingObject：Transform + Box（与视觉几何相同）
    a("  boundingObject Transform {")
    a("    translation 0 0 0")
    a("    children [")
    a(f"      Box {{ size {_vec3(BODY_SIZE)} }}")
    a("    ]")
    a("  }")
    a(_physics(BODY_MASS, BODY_COM, BODY_INERTIA_MOMENTS, BODY_INERTIA_PRODUCTS,
               2))
    a(f"  controller \"{ROBOT_CONTROLLER}\"")
    a("  controllerArgs []")
    a(f"  selfCollision {'TRUE' if SELF_COLLISION else 'FALSE'}")
    a(f"  contactMaterial \"{CONTACT_MATERIAL}\"")
    a("}")
    return "\n".join(L) + "\n"


# ----------------------------------------------------------------------------
def assert_robot(text):
    """文本离开脚本前的结构性健全性检查。"""
    # 花括号 / 方括号配平
    assert text.count("{") == text.count("}"), (
        f"brace imbalance: {{={text.count('{')} }}={text.count('}')}")
    assert text.count("[") == text.count("]"), (
        f"bracket imbalance: [={text.count('[')} ]={text.count(']')}")

    # 必需节点计数
    assert text.count("HingeJoint {") == 12, text.count("HingeJoint {")
    assert text.count("endPoint Solid {") == 12, text.count("endPoint Solid {")
    assert text.count("boundingObject") == 13, text.count("boundingObject")
    assert text.count("RotationalMotor {") == 12
    assert text.count("PositionSensor {") == 12
    assert text.count("Sphere {") == 4  # 足端
    assert 'name "yobogo_10s"' in text
    assert 'controller "mini_cheetah_controller"' in text
    assert 'name "front_camera"' in text
    assert "fieldOfView" in text
    assert "stopSpringDamper" not in text
    assert "coordinateSystem" not in text
    assert "frictionMaterial" not in text

    # 每个 endPoint 平移必须等于其关节 anchor
    import re
    anchors = re.findall(r"anchor\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)", text)
    # endPoint 平移行带以 '#' 开头的尾注
    etrans = re.findall(
        r"translation\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+# 必须与 joint anchor 一致",
        text)
    assert len(anchors) == 12, f"found {len(anchors)} anchors"
    assert len(etrans) == 12, f"found {len(etrans)} endPoint translations"
    for i, (a, e) in enumerate(zip(anchors, etrans)):
        av = tuple(float(x) for x in a)
        ev = tuple(float(x) for x in e)
        assert all(abs(x - y) < 1e-9 for x, y in zip(av, ev)), (
            f"joint {i}: anchor {av} != endPoint.translation {ev}")

    # 电机 / 传感器命名
    for leg in ("fr", "fl", "hr", "hl"):
        for j in ("abd", "hip", "kn"):
            assert f'name "{leg}_{j}_motor"' in text
            assert f'name "{leg}_{j}_sensor"' in text

    # 质量核算：单腿 1.644，总质量 10.496 ≈10.5（说明书）
    leg_mass = ABD_MASS + THIGH_MASS + SHANK_MASS
    assert abs(leg_mass - 1.644) < 1e-9, leg_mass
    total = BODY_MASS + 4 * leg_mass
    assert abs(total - 10.496) < 1e-9, total
    assert abs(total - TOTAL_MASS) < 0.02, total

    # 包络断言
    # 包络目标 485×275×300（说明书§1.3）；实现为 Scheme A 估算 400×274×315
    # body 尺寸 L×W×H = 0.40 × 0.13 × 0.10
    assert BODY_SIZE == (0.40, 0.13, 0.10), BODY_SIZE
    # 髋位：x=±0.18, y=±0.052, z=0
    for _leg, (_hx, _hy, _hz) in HIP_MOUNTS.items():
        assert abs(abs(_hx) - 0.18) < 1e-9, (_leg, _hx)
        assert abs(abs(_hy) - 0.052) < 1e-9, (_leg, _hy)
        assert abs(_hz) < 1e-9, (_leg, _hz)
    # 腿长 = 大腿 + 小腿 = 0.14 + 0.12 = 0.26
    leg_len = THIGH_LEN + SHANK_LEN
    assert abs(leg_len - 0.26) < 1e-9, leg_len
    # 宽度 = 2*(髋 y + abd 偏置 + 足端半径) = 2*(0.052+0.065+0.02) = 0.274
    width = 2.0 * (abs(HIP_MOUNTS["fr"][1]) + ABD_OFFSET_Y + TOE_RADIUS)
    assert abs(width - 0.274) < 0.02, width
    # 高度 = body 中心离地 + 半身高 = 0.26 + 0.05 = 0.31
    height = ROBOT_TRANSLATION[2] + BODY_SIZE[2] / 2.0
    assert abs(height - 0.31) < 0.02, height

    # 视觉件必须是 Shape{appearance, geometry Box/Sphere}；boundingObject
    # 的 children 可以是单行 Box/Sphere（Webots 合法写法）。
    for kw in ("Box {", "Sphere {"):
        idx = 0
        while True:
            i = text.find(kw, idx)
            if i < 0:
                break
            # 单行 bounding 写法："Box { size ... }" / "Sphere { radius ... }"
            line_end = text.find("\n", i)
            one_line = text[i:line_end if line_end > 0 else len(text)]
            bare_ok = ("size " in one_line and "geometry" not in one_line) or (
                "radius " in one_line and "geometry" not in one_line)
            prefix = text[max(0, i - 40):i]
            if not bare_ok:
                assert "geometry" in prefix, f"bare {kw} without geometry field"
            idx = i + 1

    return True


# ----------------------------------------------------------------------------
CAMERA_ORIENT_NOTES = """
相机朝向证据（Webots R2025a）
------------------------------------------
1) /usr/local/webots/resources/nodes/Camera.wrl
   - 字段名是 `fieldOfView`（不是 `fov`）；取值范围 (0, pi)。`fov` 是
     未知字段，加载时会报错。
2) 官方 Camera 参考（docs/reference/camera.md）：
   - 球面/柱面投影数学定义图像中心沿向量 (X, 0, 0) —— 光轴即相机局部 X 轴。
   - 识别物体只报告 Y 和 Z 方向的尺寸："impossible to know
     the depth of the object along the Camera X axis" —— 深度沿 X 轴。
3) 官方 MyBot 相机示例（projects/samples/mybot/worlds/mybot_camera.wbt）：
   - 朝向 +X 的机器人上 Camera 无旋转；镜头 Cylinder 视觉件放在
     translation -0.015 0 0（沿 -X 在原点后方），即相机机身向后伸、镜头朝 +X。
4) 实测探针（在本仓库中运行后已删除）：
   世界中放置 +-X/+-Y/+-Z 六个彩色标记，Camera 位姿为单位阵。
   中心像素 = +X（红色）标记 => 相机沿 +X 观看。
   然后 Transform rotation `0 1 0 0.61`，标记放在 (cos35, 0, -sin35)：
   中心像素 = 该标记 => +Y / +0.61 rad 使视野向下俯 35°。

因此上面使用的主旋转是：
    rotation 0 1 0 0.61
而合同中的 `1 0 0 -0.61` 是绕前向轴的横滚（仅保留在文件注释中）。
俯仰是绕 Webots x-前 y-左 z-上 机器人坐标系中左右轴 +Y 的旋转。
"""


def main(argv):
    text = build_robot()
    assert_robot(text)
    if len(argv) > 1:
        with open(argv[1], "w", encoding="utf-8") as f:
            f.write(text)
        print(f"wrote {argv[1]} ({len(text.splitlines())} lines)", file=sys.stderr)
    else:
        sys.stdout.write(text)


if __name__ == "__main__":
    main(sys.argv)
