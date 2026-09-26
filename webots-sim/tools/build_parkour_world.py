#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成 webots-sim/worlds/parkour.wbt — YoboGo-10S 障碍跑酷世界。

障碍布局来源（仅借布局思路与官方 PROTO 摆法，不拷贝仓库内容）：
  DDTRobot/ddt_ros2_control — simulation/webots_bridge/worlds/stairs.wbt
  https://github.com/DDTRobot/ddt_ros2_control
  参考快照：webots-sim/protos/parkour/src_stairs_reference.wbt
  原版用 StraightStairs / Ramp30deg / StraightStairsLanding / WoodenBox 金字塔。
  该仓库无 LICENSE，因此只重摆官方 PROTO、按 YoboGo 尺度改参数，不整仓拷贝。

YoboGo-10S 改造依据：站立高 0.30 m，说明书实机爬坡 ≤30°。
  台阶级高 0.12 m（原 0.23）、踏面 0.3 m；斜坡改 15°/25° 两段；
  另补窄道 0.4 m、土坑 0.35 m、起点平台 2×2 m（原版没有）。

用法：python3 tools/build_parkour_world.py
"""

import math
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "worlds", "parkour.wbt")
GEN = os.path.join(ROOT, "tools", "gen_yobogo_robot.py")

# ----------------------------------------------------------------------------
# 障碍布局常量（YoboGo 尺度）
# ----------------------------------------------------------------------------
STEP_RISE = 0.12          # 台阶级高 0.08~0.15
STEP_DEPTH = 0.30         # 踏面深
STEP_WIDTH = 0.80         # 台阶宽
STAIR_A_N = 3             # 台阶 A：3 级上
STAIR_B_N = 3             # 台阶 B：3 级下（借原版 up/down 对摆）
LANDING_H = 0.345         # 高台顶面 ≈ 台阶顶 0.36
LANDING_X = 2.5
LANDING_SX = 1.2

BOX_SIDE = 0.4            # 箱台 0.4×0.4×0.2（可跳上）
BOX_H = 0.2

PIT_GAP = 0.35            # 土坑/沟宽（可跳过）
PIT_LEDGE_H = 0.15
PIT_LEDGE_SX = 1.0

NARROW_W = 0.4            # 窄道宽
NARROW_L = 2.0            # 窄道长
NARROW_WALL_H = 0.15

RAMP15_DEG = 15.0
RAMP25_DEG = 25.0
RAMP_W = 0.8              # 斜坡宽
RAMP_T = 0.08             # 斜坡板厚
RAMP15_L = 1.2            # 单侧坡长
RAMP25_L = 1.0

WALL_H = 0.3              # 黑色挡板高
ARENA_X = 7.0             # 场地中心 x
ARENA_SX = 18.0
ARENA_SY = 8.0


def fmt(v: float, nd: int = 4) -> str:
    s = f"{v:.{nd}f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def vec3(v):
    return f"{fmt(v[0])} {fmt(v[1])} {fmt(v[2])}"


def tent_ramp_solid(name, cx, cy, angle_deg, slope_len, width, thick, color):
    """对称帐篷式斜坡（两块倾斜板在峰顶相接），做法借自官方 Ramp30deg.proto。

    每块斜板：Box(size=slope_len × width × thick) 绕 -Y 旋转 angle。
    板心高度 = (L/2)·sinθ + (T/2)·cosθ，使低端触地。
    """
    th = math.radians(angle_deg)
    half_base = slope_len * math.cos(th)
    cz = (slope_len / 2.0) * math.sin(th) + (thick / 2.0) * math.cos(th)
    lx = cx - (slope_len / 2.0) * math.cos(th)   # 左坡板心 x（峰在 cx）
    rx = cx + (slope_len / 2.0) * math.cos(th)
    size = f"{fmt(slope_len)} {fmt(width)} {fmt(thick)}"
    c = color
    return f"""Solid {{
  translation {fmt(cx)} {fmt(cy)} 0
  name "{name}"
  children [
    # 左坡（-X 低 → 峰）：rotation 0 -1 0 +{fmt(th, 6)}
    Pose {{
      translation {fmt(lx - cx)} 0 {fmt(cz)}
      rotation 0 -1 0 {fmt(th, 6)}
      children [
        Shape {{
          appearance PBRAppearance {{ baseColor {c} roughness 0.6 metalness 0 }}
          geometry Box {{ size {size} }}
        }}
      ]
    }}
    # 右坡（峰 → +X 低）：rotation 0 -1 0 -{fmt(th, 6)}
    Pose {{
      translation {fmt(rx - cx)} 0 {fmt(cz)}
      rotation 0 -1 0 {fmt(-th, 6)}
      children [
        Shape {{
          appearance PBRAppearance {{ baseColor {c} roughness 0.6 metalness 0 }}
          geometry Box {{ size {size} }}
        }}
      ]
    }}
  ]
  boundingObject Group {{
    children [
      Pose {{
        translation {fmt(lx - cx)} 0 {fmt(cz)}
        rotation 0 -1 0 {fmt(th, 6)}
        children [ Box {{ size {size} }} ]
      }}
      Pose {{
        translation {fmt(rx - cx)} 0 {fmt(cz)}
        rotation 0 -1 0 {fmt(-th, 6)}
        children [ Box {{ size {size} }} ]
      }}
    ]
  }}
}}"""


def solid_box(name, cx, cy, cz, sx, sy, sz, color, massless=True):
    """实心障碍盒（static，无 physics，避免被撞飞）。"""
    phys = ""
    if not massless:
        phys = f"  physics Physics {{ mass 1 }}\n"
    return f"""Solid {{
  translation {fmt(cx)} {fmt(cy)} {fmt(cz)}
  name "{name}"
  children [
    Shape {{
      appearance PBRAppearance {{ baseColor {color} roughness 0.7 metalness 0 }}
      geometry Box {{ size {fmt(sx)} {fmt(sy)} {fmt(sz)} }}
    }}
  ]
  boundingObject Box {{ size {fmt(sx)} {fmt(sy)} {fmt(sz)} }}
{phys}}}"""


def wooden_box(name, cx, cy, cz, sx, sy, sz):
    return f"""WoodenBox {{
  translation {fmt(cx)} {fmt(cy)} {fmt(cz)}
  name "{name}"
  size {fmt(sx)} {fmt(sy)} {fmt(sz)}
}}"""


def build_robot_segment():
    """调用 gen_yobogo_robot.py，并改写出生位姿与控制器绑定。"""
    raw = subprocess.check_output([sys.executable, GEN], text=True)
    raw = raw.replace(
        "  translation 3 0 0.26",
        "  translation 0 0 0.26   # 起点平台中心，站立高 0.26（面向 +X）",
    )
    raw = raw.replace(
        "  rotation 0 0 1 3.14159265",
        "  rotation 0 0 1 0   # 朝向 +X，对准障碍路线",
    )
    raw = raw.replace(
        '  controller "mini_cheetah_controller"',
        '  controller "manual_control"',
    )
    assert 'controller "manual_control"' in raw
    assert "translation 0 0 0.26" in raw
    assert "rotation 0 0 1 0" in raw
    return raw.rstrip() + "\n"


def build_world():
    L = []
    a = L.append

    a("#VRML_SIM R2025a utf8")
    a("# =============================================================================")
    a("# YoboGo-10S 障碍跑酷世界 (parkour)")
    a("# =============================================================================")
    a("# 障碍布局来源（借用开源布局思路 + 官方 PROTO 重摆，未整仓拷贝）：")
    a("#   DDTRobot/ddt_ros2_control")
    a("#   https://github.com/DDTRobot/ddt_ros2_control")
    a("#   文件: simulation/webots_bridge/worlds/stairs.wbt")
    a("#   Raw:  https://raw.githubusercontent.com/DDTRobot/ddt_ros2_control/main/simulation/webots_bridge/worlds/stairs.wbt")
    a("#   本地参考快照: ../protos/parkour/src_stairs_reference.wbt")
    a("#   该仓库无 LICENSE —— 只借 StraightStairs×2 / 斜坡 / Landing / WoodenBox 金字塔的")
    a("#   布局思路，障碍实体全部改用 cyberbotics 官方 PROTO 或基础 Box 重新参数化摆放。")
    a("#")
    a("# 按 YoboGo-10S 改造的尺寸（站立高 0.30 m，说明书实机爬坡 ≤30°）：")
    a("#   台阶   : 级高 0.12 m（原版 0.23 偏高）、踏面 0.30 m、宽 0.80 m，3 级上 + 3 级下")
    a("#   箱台   : 0.4×0.4×0.2 m（可跳上）")
    a("#   斜坡   : 15° 与 25° 两段帐篷式（原版 Ramp30deg 固定 30°，改为两段更低缓）")
    a("#   窄道   : 宽 0.4 m × 长 2.0 m（原版无，按需求补）")
    a("#   土坑   : 宽 0.35 m 沟，两侧台地高 0.15 m（原版无，按需求补）")
    a("#   起点   : 2×2 m 平地，机器人 translation 0 0 0.26 朝 +X")
    a("#   挡板   : 场地四周黑色 Box 高 0.3 m")
    a("#")
    a("# 路线（+X 前进）：起点 → 台阶上 → 高台 → 台阶下 → 箱台 → 土坑 → 窄道 →")
    a("#                 15° 斜坡 → 25° 斜坡 → WoodenBox 金字塔 → 终点")
    a("# =============================================================================")
    a("")
    a("# ---- 官方 PROTO（cyberbotics/webots R2025a，与 stairs.wbt 同源、版本升级）----")
    a('EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"')
    a('EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"')
    a('EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/floors/protos/RectangleArena.proto"')
    a('EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/stairs/protos/StraightStairs.proto"')
    a('EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/stairs/protos/StraightStairsLanding.proto"')
    a('EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/factory/containers/protos/WoodenBox.proto"')
    a("")
    a("WorldInfo {")
    a("  info [")
    a('    "YoboGo-10S parkour obstacle course"')
    a('    "布局借自 DDTRobot/ddt_ros2_control stairs.wbt，尺寸按 YoboGo 狗高 0.30 m 重摆"')
    a('    "控制器: manual_control（键盘/手柄手动控制）"')
    a("  ]")
    a('  title "YoboGo Parkour"')
    a("  basicTimeStep 4")
    a("  FPS 60")
    a("}")
    a("")
    a("# ==== 观察视角（GUI: 视图 → 视角 切换）====")
    a("# 1) 第三人称·全景：看清整条障碍路线")
    a("Viewpoint {")
    a("  orientation -0.35 0.2 0.1 1.1")
    a(f"  position -3 -10 8")
    a('  follow "OFF"')
    a("}")
    a("# 2) 第三人称·跟随机器人：镜头随狗走，便于观察步态")
    a("Viewpoint {")
    a("  position 0 -2.2 1.0")
    a('  follow "ON"')
    a('  followType "Mounted Shot"')
    a("  followSmoothness 2")
    a("}")
    a("# 3) 起点近景：看清出发姿态与第一段台阶")
    a("Viewpoint {")
    a("  orientation -0.25 0.5 0 0.9")
    a("  position -1.6 -1.8 1.1")
    a('  follow "OFF"')
    a("}")
    a("# 4) 终点俯视：金字塔与终点区")
    a("Viewpoint {")
    a("  orientation -0.9 0.15 0 0.6")
    a("  position 13 -1 7")
    a('  follow "OFF"')
    a("}")
    a("# 5) 侧面低角度：看台阶 / 斜坡剖面")
    a("Viewpoint {")
    a("  orientation -0.15 0.9 0 1.3")
    a("  position 6 -6 1.4")
    a('  follow "OFF"')
    a("}")
    a("")
    a("# 注意：Background 与 TexturedBackground 二选一，同时出现会 WARNING")
    a("TexturedBackground {")
    a("}")
    a("TexturedBackgroundLight {")
    a("}")
    a("")
    a("# ---- 灯光：DirectionalLight 只能挂在 world 顶层（不能进 Group/Transform）----")
    a("DirectionalLight {")
    a("  direction -0.4 0.25 -0.9")
    a("  intensity 0.85")
    a("  color 1 1 1")
    a("  castShadows TRUE")
    a("  on TRUE")
    a("}")
    a("DirectionalLight {")
    a("  direction 0.35 -0.2 -0.85")
    a("  intensity 0.35")
    a("  color 0.95 0.95 1")
    a("  castShadows FALSE")
    a("  on TRUE")
    a("}")
    a("")
    a("# ---- 场地：矩形地面 + 四周黑色挡板（高 0.3 m，防走出）----")
    a("# 注：RectangleArena 墙体即挡板；appearance 用纯色，避免贴图下载 WARNING")
    a("RectangleArena {")
    a(f"  translation {fmt(ARENA_X)} 0 0")
    a(f"  floorSize {fmt(ARENA_SX)} {fmt(ARENA_SY)}")
    a("  floorTileSize 1 1")
    a("  floorAppearance PBRAppearance { baseColor 0.55 0.55 0.55 roughness 0.9 metalness 0 }")
    a("  wallThickness 0.08")
    a(f"  wallHeight {fmt(WALL_H)}")
    a("  wallAppearance PBRAppearance { baseColor 0.05 0.05 0.05 roughness 0.9 metalness 0 }")
    a("}")
    a("")
    a("# ---- 起点平台 2×2 m 平地（视觉标记，碰撞由地面承担）----")
    a("Transform {")
    a("  translation 0 0 0.003")
    a("  children [")
    a("    Shape {")
    a("      appearance PBRAppearance { baseColor 0.2 0.6 0.35 roughness 0.8 metalness 0 }")
    a("      geometry Box { size 2 2 0.006 }")
    a("    }")
    a("  ]")
    a("}")
    a("")
    a("# ---- [1] 台阶 A：3 级上（借 stairs.wbt 的 StraightStairs 摆法，级高改 0.12）----")
    a("# stepSize = 踏面深 0.30 × 宽 0.80 × 板厚 0.03；stepRise=0.12 → 顶高 0.36")
    a("# appearance 纯色（默认 VarnishedPine 需下载贴图）")
    a("StraightStairs {")
    a("  translation 1.2 0 0")
    a(f"  stepSize {fmt(STEP_DEPTH)} {fmt(STEP_WIDTH)} 0.03")
    a(f"  stepRise {fmt(STEP_RISE)}")
    a(f"  nSteps {STAIR_A_N}")
    a('  name "stairs_up"')
    a("  stepAppearance PBRAppearance { baseColor 0.62 0.5 0.38 roughness 0.7 metalness 0 }")
    a("  stringerAppearance PBRAppearance { baseColor 0.45 0.35 0.28 roughness 0.7 metalness 0 }")
    a("  leftRail []")
    a("  rightRail []")
    a("  startingStairs FALSE")
    a("}")
    a("")
    a("# ---- [2] 高台：借 stairs.wbt 的 StraightStairsLanding，高度改 0.345 ----")
    a("# 顶面 ≈ 0.36，与台阶 A 顶齐平；台面 1.2 × 0.8")
    a("StraightStairsLanding {")
    a(f"  translation {fmt(LANDING_X)} 0 0")
    a(f"  landingSize {fmt(LANDING_SX)} {fmt(STEP_WIDTH)} 0.03")
    a(f"  height {fmt(LANDING_H)}")
    a('  name "landing_mid"')
    a("  appearance PBRAppearance { baseColor 0.55 0.42 0.32 roughness 0.7 metalness 0 }")
    a("  floorAppearance PBRAppearance { baseColor 0.6 0.48 0.36 roughness 0.75 metalness 0 }")
    a("  stringerLeft FALSE")
    a("  stringerRight FALSE")
    a("  stringerBack FALSE")
    a("}")
    a("")
    a("# ---- [3] 台阶 B：3 级下（借 stairs.wbt 的 up/down 对摆，rotation π）----")
    a("StraightStairs {")
    a("  translation 3.85 0 0")
    a("  rotation 0 0 1 3.14159265")
    a(f"  stepSize {fmt(STEP_DEPTH)} {fmt(STEP_WIDTH)} 0.03")
    a(f"  stepRise {fmt(STEP_RISE)}")
    a(f"  nSteps {STAIR_B_N}")
    a('  name "stairs_down"')
    a("  stepAppearance PBRAppearance { baseColor 0.62 0.5 0.38 roughness 0.7 metalness 0 }")
    a("  stringerAppearance PBRAppearance { baseColor 0.45 0.35 0.28 roughness 0.7 metalness 0 }")
    a("  leftRail []")
    a("  rightRail []")
    a("  startingStairs FALSE")
    a("}")
    a("")
    a("# ---- [4] 箱台 0.4×0.4×0.2（可跳上；借 stairs.wbt WoodenBox 金字塔的盒体）----")
    a(wooden_box("box_platform", 4.8, 0, BOX_H / 2.0, BOX_SIDE, BOX_SIDE, BOX_H))
    a("")
    a("# ---- [5] 土坑/沟 宽 0.35（两侧台地，可跳过；按 YoboGo 需求新增）----")
    a("# 台地顶高 0.15，中间留 0.35 空隙")
    a(solid_box("pit_ledge_1", 5.7, 0, PIT_LEDGE_H / 2.0, PIT_LEDGE_SX, 1.0, PIT_LEDGE_H,
                "0.55 0.45 0.3"))
    a(solid_box("pit_ledge_2", 5.7 + PIT_LEDGE_SX / 2 + PIT_GAP + PIT_LEDGE_SX / 2, 0,
                PIT_LEDGE_H / 2.0, PIT_LEDGE_SX, 1.0, PIT_LEDGE_H, "0.55 0.45 0.3"))
    a("# 沟底暗色垫（视觉，低于地面）")
    a(solid_box("pit_bottom", 5.7 + PIT_LEDGE_SX / 2 + PIT_GAP / 2, 0, -0.08,
                PIT_GAP + 0.02, 1.0, 0.06, "0.15 0.12 0.1"))
    a("")
    a("# ---- [6] 窄道 宽 0.4 × 长 2.0（两侧挡条；按 YoboGo 需求新增）----")
    a(solid_box("narrow_wall_L", 8.2, NARROW_W / 2 + 0.1, NARROW_WALL_H / 2,
                NARROW_L, 0.2, NARROW_WALL_H, "0.75 0.7 0.65"))
    a(solid_box("narrow_wall_R", 8.2, -(NARROW_W / 2 + 0.1), NARROW_WALL_H / 2,
                NARROW_L, 0.2, NARROW_WALL_H, "0.75 0.7 0.65"))
    a("")
    a("# ---- [7] 斜坡 15°（帐篷式两段，借 Ramp30deg 的倾斜板做法，角度改 15°）----")
    a(tent_ramp_solid("ramp_15deg", 10.2, 0, RAMP15_DEG, RAMP15_L, RAMP_W, RAMP_T,
                      "0.6 0.5 0.4"))
    a("")
    a("# ---- [8] 斜坡 25°（角度改 25°，说明书实机 ≤30°）----")
    a(tent_ramp_solid("ramp_25deg", 12.0, 0, RAMP25_DEG, RAMP25_L, RAMP_W, RAMP_T,
                      "0.55 0.4 0.3"))
    a("")
    a("# ---- [9] WoodenBox 金字塔（借 stairs.wbt 金字塔摆法，盒体改 0.4×0.4×0.2）----")
    a("# 底层 3 盒 + 中层 2 盒 + 顶层 1 盒 + 引导盒 1 = 7 盒")
    a(wooden_box("pyr_base_1", 13.4, -0.4, BOX_H / 2, BOX_SIDE, BOX_SIDE, BOX_H))
    a(wooden_box("pyr_base_2", 13.4, 0, BOX_H / 2, BOX_SIDE, BOX_SIDE, BOX_H))
    a(wooden_box("pyr_base_3", 13.4, 0.4, BOX_H / 2, BOX_SIDE, BOX_SIDE, BOX_H))
    a(wooden_box("pyr_mid_1", 13.4, -0.2, BOX_H * 1.5, BOX_SIDE, BOX_SIDE, BOX_H))
    a(wooden_box("pyr_mid_2", 13.4, 0.2, BOX_H * 1.5, BOX_SIDE, BOX_SIDE, BOX_H))
    a(wooden_box("pyr_top", 13.4, 0, BOX_H * 2.5, BOX_SIDE, BOX_SIDE, BOX_H))
    a(wooden_box("pyr_step", 12.9, 0, BOX_H / 2, BOX_SIDE, BOX_SIDE, BOX_H))
    a("")
    a("# ---- [10] 终点平台 ----")
    a(solid_box("finish_pad", 14.6, 0, 0.01, 1.2, 1.2, 0.02, "0.85 0.75 0.2"))
    a("")
    a("# ---- 机器人：REAL YoboGo-10S（由 tools/gen_yobogo_robot.py 生成）----")
    a("# 出生点在起点平台中心 translation 0 0 0.26，面向 +X（rotation 0 0 1 0）")
    a("# 控制器绑定 manual_control")
    a("")
    robot = build_robot_segment()
    L.append(robot)

    text = "\n".join(L)
    if not text.endswith("\n"):
        text += "\n"
    return text


def main():
    text = build_world()
    # 结构健全性检查
    assert text.startswith("#VRML_SIM R2025a utf8")
    assert text.count("{") == text.count("}"), (
        f"brace imbalance {{={text.count('{')} }}={text.count('}')}")
    assert text.count("[") == text.count("]"), "bracket imbalance"
    assert "basicTimeStep 4" in text
    assert 'controller "manual_control"' in text
    assert "translation 0 0 0.26" in text
    assert text.count("DirectionalLight {") == 2
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as f:
        f.write(text)
    n = text.count("\n")
    print(f"wrote {OUT} ({n} lines)", file=sys.stderr)


if __name__ == "__main__":
    main()
