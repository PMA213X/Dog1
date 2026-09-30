# Webots 仿真使用指南

四足机器人 Webots 仿真项目：PD 站立 + trot 步态，C++ 控制器，无需 ROS。源世界包含 Mini Cheetah 步态试验台、MSL 比赛场地和跑酷/RL 世界；YoboGo-10S 机器人段由 `gen_yobogo_robot.py` 作为唯一模型真源生成，2026-09-30 的课件驱动重构见 [第 9 节](#9-yobogo-10s-课件驱动重构)。

- **Webots 版本**：R2025a（apt 包 `webots 2025a`）
- **安装路径**：`/usr/local/webots`
- **项目路径**：仓库根目录 `webots-sim/`
- **控制器**：`mini_cheetah_controller`（334 行 C++，已编译，仿真可运行）
- **世界文件**：`mini_cheetah.wbt`、`msl_match.wbt`、`parkour.wbt`、`parkour_dev.wbt`；三个 YoboGo 源 world 见 [第 9 节](#9-yobogo-10s-课件驱动重构)

---

## 目录

1. [安装方法](#1-安装方法)
2. [项目结构](#2-项目结构)
3. [运行方法](#3-运行方法)
4. [控制器说明（PD + trot）](#4-控制器说明pd--trot)
5. [按键操作](#5-按键操作)
6. [中文界面设置](#6-中文界面设置)
7. [常见问题](#7-常见问题)
8. [MSL 比赛场地](#8-msl-比赛场地msl_matchwbt)
9. [YoboGo-10S 课件驱动重构](#9-yobogo-10s-课件驱动重构)

---

## 1. 安装方法

本项目使用 **apt 源安装**（Cyberbotics 官方 apt 仓库，需参考 Cyberbotics 官方 Wiki 添加源）：

```bash
# 添加 Cyberbotics apt 源后（详见 cyberbotics.com 官方 Wiki 的 Ubuntu 安装说明）
sudo apt update
sudo apt install webots
```

验证安装：

```bash
dpkg -l | grep webots
# ii  webots  2025a  amd64  Mobile robot simulation software

/usr/local/webots/webots --version
# Webots version: R2025a
```

> **备选方式**：Cyberbotics 也提供 `.deb` 包和 tarball。tarball 解压即用，但本项目统一使用 apt 版本（路径 `/usr/local/webots`，与控制器 Makefile 的默认 `WEBOTS_HOME` 一致）。
>
> **注意**：Webots 体积较大，请确认磁盘空间充足（本机根分区紧张时需先清理或改用外置盘 tarball 方案）。

---

## 2. 项目结构

```
webots-sim/
├── worlds/
│   ├── mini_cheetah.wbt            # 世界文件（#VRML_SIM R2025a，步态试验台）
│   ├── msl_match.wbt               # MSL 比赛场地世界（见第 8 节）
│   ├── parkour.wbt                 # 跑酷源世界
│   └── parkour_dev.wbt             # RL 开发源世界
├── controllers/
│   └── mini_cheetah_controller/
│       ├── mini_cheetah_controller.cpp   # 控制器源码（334 行）
│       ├── Makefile                      # Webots 标准 Makefile.include
│       ├── mini_cheetah_controller        # 编译产物（可执行）
│       └── build/release/                # 中间产物（.o / .d）
├── urdf/
│   ├── mini_cheetah.urdf           # Mini Cheetah URDF（参考模型）
│   └── meshes/                     # 网格（.dae，4 个连杆）
├── protos/                         # 自定义 PROTO（MSL 场地，见第 8 节）
│   ├── MslField.proto              # 场地地毯 + 全部标线
│   ├── MslGoal.proto               # 球门框架 + 球网
│   ├── MslBall.proto               # FIFA 5 号球
│   └── MslEnvironment.proto        # 安全挡板 + 旗杆
└── tools/
    └── gen_yobogo_robot.py         # YoboGo-10S 机器人段生成脚本（见第 9 节）
```

### 2.1 世界文件要点

- 头部声明：`#VRML_SIM R2025a utf8`
- `basicTimeStep 4` —— 4ms 步长，对应控制器 250Hz 控制周期
- 机器人使用 **Webots 内置节点**（`Robot` + 内置 `RotationalMotor` / `PositionSensor` / `Gyro` / `Accelerometer` / `InertialUnit`），**不依赖外部 PROTO**，克隆仓库后可直接打开
- 关节命名：`{fr,fl,hr,hl}_{abd,hip,kn}_motor` / 对应 `_sensor`；`controller "mini_cheetah_controller"` 绑定本项目控制器
- 初始高度：`mini_cheetah.wbt` 为 `translation 0 0 0.38`；YoboGo 源 world 默认出生位姿由生成器提供，`msl_match.wbt` 当前为 `translation 3 0 0.26`，跑酷世界为 `translation 0 0 0.26`，见 [第 9 节](#9-yobogo-10s-课件驱动重构)

### 2.2 URDF 说明

`urdf/mini_cheetah.urdf` 为模型参数参考（机体 mass=3.3kg，连杆质量、腿长等），网格文件一并提供；当前世界文件直接用内置节点建模，URDF 暂未被加载。

---

## 3. 运行方法

### 3.1 打开仿真

```bash
# 方式一：命令行（推荐）
/usr/local/webots/webots webots-sim/worlds/mini_cheetah.wbt

# 方式二：GUI 内 File → Open World... 选择 webots-sim/worlds/mini_cheetah.wbt
```

首次打开时 Webots 会**自动调用控制器 Makefile 编译**；也可手动编译：

```bash
cd webots-sim/controllers/mini_cheetah_controller
export WEBOTS_HOME=/usr/local/webots   # apt 安装时的默认路径
make
```

### 3.2 运行与停止

- 点击工具栏 **Real-time / 播放** 按钮开始仿真（默认启动模式 Real-time）
- 仿真开始后控制器进入 **站立模式**，Console 打印：

  ```
  === Mini Cheetah Controller ===
  Control modes:
    'S' : Standing (default)
    'T' : Trot gait
    'R' : Reset to standing
  ===============================
  ```

- 每 500ms 打印一次 IMU（roll/pitch/yaw）与 FR 腿关节角调试信息
- 点击 **暂停/停止** 结束；`Ctrl+S` 保存世界

### 3.3 在仿真世界中交互

- 视角：鼠标左键旋转 / 滚轮缩放 / 右键平移；`Viewpoint` 已开启 `follow` 跟随机器人
- 键盘输入需**先点击 3D 视图窗口获得焦点**，再按 `S` / `T` / `R`

---

## 4. 控制器说明（PD + trot）

源码：`webots-sim/controllers/mini_cheetah_controller/mini_cheetah_controller.cpp`

参考：`docs/features/textbook-ch9-13.md`（MPC/WBC 框架）、`docs/features/robot-software-analysis.md`（关节 PD 增益）。

### 4.1 基本结构

| 项目 | 值 |
|------|-----|
| 控制周期 | 4ms（250Hz，与 `basicTimeStep` 一致） |
| 腿 / 关节 | 4 腿 × 3 关节（abd 外展 / hip 髋 / kn 膝），共 12 电机 |
| 腿索引 | FR=0, FL=1, HR=2, HL=3 |
| 传感器 | 关节位置传感器（1ms 采样，速度由差分估计）+ Gyro + Accelerometer + InertialUnit（四元数） |
| 力矩限幅 | `MAX_TORQUE = 15.0` N·m |
| 模式变量 | `control_mode`：0=站立，1=trot |

### 4.2 关节 PD

```
torque = KP[j] * (q_des - q) + KD[j] * (qd_des - qd)
```

| 增益 | abd | hip | kn |
|------|-----|-----|----|
| KP | 3.0 | 3.0 | 3.0 |
| KD | 1.0 | 0.2 | 0.2 |

### 4.3 站立模式（control_standing）

所有 12 关节的指令为 `STANDING_DES = {0, 0, 0}`，期望速度 0，纯 PD 回位；控制器逻辑本轮不改。旧模型曾把“指令 0”实现为大腿、小腿共线的直腿，2026-09-30 重构通过模型 frame/endpoint 零位让“模型关节 0”对应自然屈膝站姿，不能继续把该值解释为实机编码器零位。

### 4.4 trot 步态（control_trot）

对角小跑：**FR+HL 同相位，FL+HR 同相位**。

| 参数 | 值 |
|------|-----|
| `TROT_PERIOD` | 0.5 s（每周期） |
| 支撑相 | 前 50%（0.25s），髋关节前后扫动 ±0.15 rad |
| 摆动相 | 后 50%，钟形轨迹抬腿 |
| `TROT_SWING_HEIGHT` | 0.04 m |
| 相位偏移 `TROT_PHASE` | FR=0.0, FL=0.5, HR=0.5, HL=0.0 |
| abd 关节 | 全程保持 0（简化 trot 无侧向运动） |
| 膝关节摆动 | `swing_knee = -LIFT / L_KNEE * sin(π * p)`，`L_KNEE = 0.18` |

腿长常量（正运动学参考）：`L_ABD=0.062`、`L_HIP=0.209`、`L_KNEE=0.18`；髋部位置 ±0.19m（前后）× ±0.111m（左右）。这些是**控制器源码内**的 Mini Cheetah 尺寸常量；`msl_match.wbt` 的 YoboGo-10S 世界模型当前为大腿 0.15 / 小腿 0.15 m（见 [第 9 节](#9-yobogo-10s-实机模型与相机)），trot 中的 `swing_knee` 仍按旧控制器常量计算，尚未同步新几何。

### 4.5 坐标系

世界文件注释：**X=前，Y=左，Z=上**；IMU 四元数按 NUE 坐标系转 roll/pitch/yaw。

---

## 5. 按键操作

在 3D 视图获得焦点后：

| 按键 | 功能 | 控制台输出 |
|------|------|-----------|
| `S` | 切换到站立模式 | `Mode: STANDING` |
| `T` | 切换到 trot 步态（重置步态相位计时） | `Mode: TROT` |
| `R` | 复位到站立（重置相位） | `Mode: RESET to STANDING` |

启动时默认站立模式（`control_mode = 0`）。

---

## 6. 中文界面设置

### 方法一：配置文件（已设置）

编辑（或创建）：

```
~/.config/Cyberbotics/Webots-R2025a.conf
```

在 `[%General]` 段设置：

```ini
[%General]
language=zh_CN
```

重启 Webots 后界面为简体中文。

### 方法二：GUI 菜单

`Settings`（设置）→ `Preferences...`（首选项）→ `General` → `Language`（语言）选择 `简体中文 (zh_CN)` → 重启生效。

> 配置文件名中的 `R2025a` 随版本变化（本机另有旧版 `Webots-R2023b.conf`），版本升级后需对新版本的 conf 重新设置。

---

## 7. 常见问题

### 7.1 控制器没有运行 / 报找不到设备

- 现象：Console 出现 `WARNING: Motor not found: xxx_motor`
- 原因：控制器二进制与世界文件不匹配，或未编译
- 处理：在 `controllers/mini_cheetah_controller/` 下执行 `make`（确保 `WEBOTS_HOME=/usr/local/webots`），然后重新打开世界文件
- 补充：`mini_cheetah.wbt` 中的机器人模型仍保留 Gazebo 风格语法（R2025a 可能因此报找不到设备）；R2025a 兼容的语法修复仅在 `msl_match.wbt` 中，详见 [第 8 节](#8-msl-比赛场地msl_matchwbt)

### 7.2 Makefile 报找不到 `Makefile.include`

```bash
export WEBOTS_HOME=/usr/local/webots   # apt 安装路径
make
```

### 7.3 机器人瘫倒 / 一动不动

- 确认已点击播放、3D 视图有焦点
- 站立模式期望角全为 0；若初始姿态偏差大，先按 `R` 复位
- 电机力矩被限幅 15N·m，属正常保护

### 7.4 键盘无反应

- 先用鼠标点击 3D 视图窗口（焦点在编辑器/Console 时键盘输入不进控制器）
- 100ms 采样周期，快速单击可能漏读，按住稍许

### 7.5 图形渲染问题（黑屏 / 卡顿）

- Webots R2025a 需要 OpenGL 3.3+，确认显卡驱动（NVIDIA 用专有驱动）
- 可在 `Settings → Preferences → OpenGL` 调低 GTAO / 纹理质量，或关闭 `rendering`
- 远程/虚拟显示环境渲染受限时，优先本机桌面运行

### 7.6 中文不生效

- 确认改的是 **`Webots-R2025a.conf`**（不是旧版 R2023b 的 conf）
- `language=zh_CN` 必须在 `[%General]` 段
- 修改后需完全退出并重启 Webots

### 7.7 与实机代码的关系

本仿真验证的是 PD/步态算法思路，控制器为独立实现；实机运动控制链路（robot-software 的 MPC+WBC、SPI 到 SPIne 板、STM32 FOC）见 `docs/architecture/system-overview.md` 与 `docs/api/robot-communication.md`。

---

## 8. MSL 比赛场地（msl_match.wbt）

新增世界 `webots-sim/worlds/msl_match.wbt`（Webots R2025a），按 `docs/features/robocup-midsize-rules.md`（MSL Rulebook 2025 v26.0）渲染官方 MSL 比赛场地，并搭载 inline **YoboGo-10S 实机规格机器人模型**（含前置相机，见 [第 9 节](#9-yobogo-10s-实机模型与相机)）。

### 8.1 打开方式

```bash
/usr/local/webots/webots webots-sim/worlds/msl_match.wbt
```

### 8.2 场景元素与关键尺寸

| 元素 | 数量 | 说明 |
|------|------|------|
| 场地 | 1 | 22 m × 14 m 绿色地毯 |
| 标线 | — | 白色，宽 0.125 m（边线、球门线、中线、中圈、中点、球门区、罚球区、罚球点、角弧） |
| 球门 | 2 | 白色门柱 + 球网 |
| 足球 | 1 | FIFA 5 号球，橙色 |
| 安全挡板 | 1 圈 | 黑色，24 m × 16 m 围合 |
| 旗杆 | 6 | 4 角 + 中线与边线交点 2 处 |
| 灯光 | 2 | DirectionalLight（写在 world 文件顶层） |
| 机器人 | 1 | inline YoboGo-10S（`yobogo_10s`），出生点 `3 0 0.26`，绕 Z 轴旋转 180°（面向 -X，朝向场地中心），见 [第 9 节](#9-yobogo-10s-实机模型与相机) |

**场地标线**：

| 标线 / 标记 | 尺寸 |
|-------------|------|
| 标线宽度 | 0.125 m |
| 中圈 | 半径 2 m |
| 球门区 | 深 0.75 m × 宽 3.94 m |
| 罚球区 | 深 2.25 m × 宽 6.94 m |
| 罚球点 | 3.6 m |
| 角弧 | 半径 0.75 m |

**球门**：

| 项目 | 值 |
|------|-----|
| 内宽 | 2.44 m |
| 横梁下沿高度 | 2.0 m |
| 深度 | 0.5 m |
| 球网底部外伸 | 0.40 m |

**足球 / 挡板 / 旗杆**：

| 项目 | 值 |
|------|-----|
| 足球 | FIFA 5 号球，半径 0.11 m，质量 0.43 kg，橙色 |
| 安全挡板 | 黑色，高 0.10 m，围合 24 m × 16 m（距边线 ≥1 m） |
| 旗杆 | 高 1.5 m，红 / 黄旗，共 6 根 |

### 8.3 PROTO 文件

world 文件通过 `EXTERNPROTO "../protos/X.proto"` 引用下列自定义 PROTO：

| PROTO | 行数 | 职责 |
|-------|------|------|
| `MslField.proto` | 1443 | 绿色地毯 + 全部白色标线 |
| `MslGoal.proto` | 611 | 球门框架 + 球网；局部坐标系开口朝 -X |
| `MslBall.proto` | 44 | FIFA 5 号球 |
| `MslEnvironment.proto` | 263 | 安全挡板 + 旗杆（灯光不在此 PROTO 内） |

### 8.4 坐标系

- **X=前，Y=左，Z=上**（与 `mini_cheetah.wbt` 一致）
- 场地中心为原点，两球门位于 `x = ±11`（22 m 场长的一半）

### 8.5 与 mini_cheetah.wbt 的差异

| 对比项 | `mini_cheetah.wbt` | `msl_match.wbt` |
|--------|--------------------|-----------------|
| 定位 | 站立 / trot 算法的空白试验台 | 完整 MSL 比赛场地场景 |
| 场景内容 | 无场地元素 | 场地、标线、球门、球、挡板、旗杆、灯光 |
| 自定义 PROTO | 无 | 4 个（见 8.3） |
| 机器人模型 | Mini Cheetah 盒模型（Gazebo 风格语法） | YoboGo-10S 实机规格模型 + 前置相机（R2025a 兼容语法，见第 9 节） |

机器人模型在 `msl_match.wbt` 中由 `webots-sim/tools/gen_yobogo_robot.py` 生成，详见 [第 9 节](#9-yobogo-10s-课件驱动重构)。**`mini_cheetah.wbt` 仍保留旧 Mini Cheetah 盒模型与 Gazebo 风格语法，不参与本轮 YoboGo 重构。**

### 8.6 注意事项

- **灯光必须写在 world 顶层**：Webots 禁止在 `Group` / `Pose` 子节点中放置 `DirectionalLight`，因此两盏灯直接写在 `msl_match.wbt` 顶层，而不是 `MslEnvironment.proto` 内。
- **旋转朝向**：`rotation 0 0 1 π` 表示绕 Z 轴转 180°（面向 -X）；若误写为 `0 1 0 π`，会把机器人 / 球门上下颠倒。

---

## 9. YoboGo-10S 实机模型与相机

`msl_match.wbt` 中的机器人已由 **Mini Cheetah 盒模型** 重建为 **YoboGo-10S 实机规格模型**（方案 A：按包络缩放运动学链），并新增前置相机 `front_camera`。机器人段由脚本生成，可重新生成覆盖；`mini_cheetah.wbt` 仍保留旧盒模型。

2026-09-30 进一步依据《四足仿生机器人基本原理及开发教程》实施**等效三关节重构**，已修复旧模型 `thigh=0.14 m + shank=0.12 m` 恰好等于站立高 `0.26 m` 导致的零位直腿、腿长比例失真和足端由小腿盒边触地问题。

> **状态说明（2026-09-30 验收完成）**：生成器、三个源 world、静态一致性检查和 Webots 批处理加载均已完成，实际参数与本节的 `0.15/0.15 m`、约 `120°`、工程暂定限位、足端碰撞及四连杆外观一致。精确腿长、四连杆杆长、真实机械限位和数值零位仍为**待实机/CAD确认**项。

### 9.1 等效三关节与零位

- **拓扑保持不变**：每腿仍是 `abd`（横滚/侧摆）→ `hip`（髋俯仰）→ `kn`（膝俯仰），共 12 个 `HingeJoint`、12 个 `RotationalMotor`、12 个 `PositionSensor`。
- **默认几何**：工程标定值 `thigh = shank = 0.15 m`，配合约 `120°` 膝部内角，使约 `0.26 m` 站立高度来自屈膝折线，而不是两段腿共线。
- **零位概念分开**：
  - **模型关节零位**：Webots 中 `motor target = 0` 对应自然屈膝站姿，通过固定 frame/endpoint 偏置实现；
  - **实机编码器零位**：课件给出“髋横向水平、大腿纵向水平、小腿摆到限位”的定性步骤，但未给数值角，不得与模型零位混写；
  - **自然站立角**：控制器或 RL 围绕模型零位给定的运行姿态，本轮不修改控制器代码。
- **坐标约定不变**：`X=前、Y=左、Z=上`；课件坐标系必须先转换到 Webots 轴向，不直接照搬。
- **设备接口不变**：`{fr,fl,hr,hl}_{abd,hip,kn}_motor` / `_sensor` 的腿序、关节序和控制器绑定均不变。

### 9.2 参数来源分级

文档与生成器必须使用同一套来源词：**课件原文**、**说明书原文**、**现有代码**、**图片比例测量**、**工程估算**、**工程暂定**、**待实机/CAD确认**。下表是本轮采用和必须保留的来源边界：

| 参数 | 数值 / 结论 | 来源等级 | 说明 |
|------|-------------|---------|------|
| 整机包络 | 485×275×300 mm | **说明书原文** | 可用于包络检查 |
| 整机质量 | 约 10.5 kg | **说明书原文** | 可用于总质量约束 |
| 自由度 | 整机 12 DOF，每腿 3 DOF | **说明书原文 / 课件原文** | 两处一致 |
| 关节链 | 横滚 + 两俯仰：`abd → hip → kn` | **课件原文** | 课件第2、4、7、9章及结构图支持拓扑 |
| 膝部结构 | 平行四边形/连杆膝、足底柔性弧面 | **课件原文** | 结构约束；精确杆长未给出 |
| 足端检测 | 仿真在小腿末端使用 `Touch Sensor` | **课件原文** | 新增设备不接入现有 RL 观测 |
| 定性零位 | 髋横向水平、大腿纵向水平、小腿摆到限位 | **课件原文** | 只有姿态描述，没有数值角 |
| 站立高度 0.26 m | `des_p[2]` / 旧模型出生高 | **现有代码** | 仅作操作参考，不是机械尺寸或关节限位 |
| `thigh/shank` | **0.15 / 0.15 m** | **工程暂定** | 为使 `0.26 m` 屈膝站立而采用的暂定值 |
| 膝部内角 | 约 **120°** | **工程暂定** | 定性屈膝目标，不是课件原文限位 |
| 髋安装点 | `(±0.18, ±0.052, 0)` m | **现有代码 + 工程估算** | 先沿用并重新按 485×275×300 mm 包络检查 |
| 关节限位 | `abd ±0.6`、`hip ±1.0`、`kn ±1.0 rad` | **工程暂定** | 只用于安全软/硬限位，不能冒充实机机械限位 |
| 结构比例 | 屈膝方向、四连杆大致比例、足底形状 | **图片比例测量** | 仅作定性核对，不从截图反推精确杆长或转角 |
| 腿段、四连杆杆长、真实机械限位、数值零位偏置 | 未确认 | **待实机/CAD确认** | 课件未给出，禁止编造 |
| Mini Cheetah 的 `0.062/0.209/0.18 m`、减速比 | 不采用 | **禁止移植** | `buildMiniCheetah()` 参数不属于 YoboGo-10S |

### 9.3 机械限位、碰撞与四连杆

- 12 个关节全部显式写入 `minPosition/maxPosition` 和 `minStop/maxStop`；暂定 `abd = ±0.6 rad`、`hip = ±1.0 rad`、`kn = ±1.0 rad`，最大速度保持 `10 rad/s`。
- 上述限位必须在代码和文档中标注为**工程标定暂定值**；课件没有提供三关节数值机械限位，不能把 CAN 字段范围或 Mini Cheetah softstop 当作实机范围。
- 足端球已纳入 `boundingObject`，避免只有视觉球而由小腿 Box 底边触地；四条腿使用 `yobogo_foot` 独立接触材质，并配置摩擦、零回弹与软接触顺应。
- 已启用 4 个接触设备 `fr/fl/hr/hl_foot_touch`，不进入现有 `OBS_DIM=42/45/51` 观测。
- 四连杆/平行四边形只作为跟随 `hip`、`kn` 和小腿运动的**外观件**；不建立闭链、不增加被动关节，驱动关节总数仍为 12。

以下保留 2026-09-25 初版模型记录，仅作历史对照；其中 `0.14/0.12 m` 和“模型站立姿态”描述已被 9.1–9.3 的本轮重构结果取代。

#### 9.3.1 2026-09-25 初版模型变更说明

- **几何 / 质量**：按说明书包络 **485×275×300 mm**、总质量 **10.5 kg** 缩放运动学链；站立机身高取实机控制器参数 **0.26 m**
- **动力学**：机身惯量直接取控制器模型 `RPC_inertia`；关节力矩上限对齐 CAN 协议 **±18 N·m**
- **相机**：机身前上方新增 `Camera`（640×480），供视觉 / 巡线等算法接入
- **关节命名不变**：`{fr,fl,hr,hl}_{abd,hip,kn}_motor` / 对应 `_sensor`，仍绑定 `mini_cheetah_controller`
- **机器人名**：`yobogo_10s`（2026-09-25 初版记录出生点 `translation 8 0 0.26`；2026-09-30 生成器当前默认为 `translation 3 0 0.26`，以第 9.5 节实际输出为准）

#### 9.3.2 2026-09-25 初版参数溯源（已被 9.2 取代）

可信度标记：**A** 说明书 / 官方文档 · **B** 实机 yaml / 代码 · **C** 估算（按包络缩放，非实测）

| 参数 | 值 | 来源 | 可信度 |
|------|-----|------|--------|
| 包络（站立） | 485×275×300 mm | `YoboGo-control/YoboGo-10S使用说明书(开源).docx` §1.3 | A |
| 总质量 | 10.5 kg | 同上 §1.3 | A |
| 关节力矩上限 | 18 N·m | 同上 §六 CAN 协议（±18 N·m） | A |
| 站立机身高 | 0.26 m | `YoboGo-control/robot-software/config/mc-mit-ctrl-user-parameters.yaml` `des_p[2]` | B |
| 机身惯量（主惯量） | [0.07, 0.26, 0.242] kg·m² | 同 yaml `RPC_inertia` | B |
| 机身质量 RPC_mass | 9 kg | 同 yaml `RPC_mass` | B |
| 关节 Kp | [3, 3, 3] | 同 yaml `Kp_joint` | B |
| 关节 Kd | [1, 0.2, 0.2] | 同 yaml `Kd_joint` | B |
| 抬腿高度 Swing_traj_height | 0.07 m | 同 yaml `Swing_traj_height` | B |
| 期望速度上限 des_dp_max | [1.0, 0.5, 0] | 同 yaml `des_dp_max` | B |
| 步态周期 gait_period_time | 0.5 s | 同 yaml `gait_period_time` | B |
| 相机分辨率 | 640×480 | `YoboGo-control/track1.6/track/main.cpp:24` | B |
| 视觉仅用上半图 | rows 0–149 | 同 `main.cpp:168`（`i < frame.rows/2.0`，作用于 :203 缩放后的 400×300 图） | B |
| 目标中线 goalAverage | 200 | 同 `main.cpp:201`（同义默认 `average=200` 见 `:138`） | B |
| 机身盒尺寸 | 0.40×0.13×0.10 m | 按包络缩放估算 | C |
| 髋安装位置 | (±0.18, ±0.052) m | 按包络缩放估算 | C |
| abd 偏置 | ±0.065 m | 按包络缩放估算 | C |
| 大腿 / 小腿长 | 0.14 / 0.12 m（和 = 0.26 = 站立高） | 按包络缩放估算 | C |
| 足端球半径 | 0.02 m | 按包络缩放估算 | C |
| 质量分配 | body 3.92 + 4 腿×1.644 ≈ 10.5 kg | 按总质量估算分配 | C |
| 腿段惯量 | 见生成脚本常量（abd / thigh / shank） | 细长杆量级估算，products 全 0 | C |
| 相机 `fieldOfView` | 1.05 rad（≈60°） | 视觉需求推断 | C（推断） |
| 相机俯仰 | -35°（`rotation 0 1 0 0.61`） | 视觉需求推断 | C（推断） |
| 相机安装位 | `translation 0.2 0 0.055`（前上） | 布局约定 | C |

### 9.4 相机参数与可调范围

Webots R2025a 中 Camera 的视场角字段名是 **`fieldOfView`**（**不是** `fov`），取值范围 (0, π)，单位弧度；光轴为相机局部 **+X** 轴（实测验证：+Y / 正角度绕 Y 轴右手旋转 = 低头）。

| 参数 | 字段 | 当前值 | 可调范围 | 说明 |
|------|------|--------|----------|------|
| 水平视场角 | `fieldOfView` | 1.05（≈60°） | 0.96–1.22（≈55°–70°） | 推断值，可按视觉算法需求调整 |
| 俯仰角 | `rotation 0 1 0 θ` | θ=0.44（低头约 25°，当前调参值） | 初始验收建议 θ=0.52–0.70（低头 30°–40°） | 绕 +Y 右手旋转，正值 = 低头；当前为看到 3 m 外球体而抬高，已超出初始建议范围 |
| 安装位置 | `translation` | `0.2 0 0.055` | — | 机身坐标系：+X 前、+Z 上 |
| 分辨率 | `width` / `height` | 640×480 | — | 与实机视觉代码一致 |
| 设备名 | `name` | `front_camera` | — | 控制器按此名取像 |

> 相机世界高度 ≈ 0.26 + 0.055 ≈ 0.315 m。修改俯仰或 FOV 时，可直接改 world 中 `Camera` 节点，或改生成脚本顶部 `CAMERA_*` 常量后重新生成（见 9.4）。

### 9.5 重新生成机器人段

`gen_yobogo_robot.py` 是机器人段唯一模型真源；不带文件参数时，它把机器人 VRML 打印到标准输出。`build_parkour_world.py` 会调用生成器构建 `parkour.wbt`，并改写出生位姿与 controller。

> **危险命令修正**：生成器带文件参数时会**整体覆盖该输出文件**。禁止把 `msl_match.wbt`、`parkour.wbt` 或 `parkour_dev.wbt` 直接作为生成器输出参数，否则会丢失整个 world。仓库当前没有可复用的“只替换 Robot 段”命令；本轮已由受控替换流程更新三个源 world。后续重新生成前，应先补非破坏性的 Robot 段替换工具或沿用已验证流程，再逐 world 检查括号、设备计数和 controller/出生位姿差异。

同时检查 `tools/build_parkour_world.py` 对生成器默认出生位姿字符串的依赖。**禁止手改**运行时生成的 `.parkour*_rl.wbt`、`.parkour_view*.wbt`；`mini_cheetah.wbt` 保持不动。

| 验收项 | 要求 | 当前文档状态 |
|--------|------|--------------|
| 静态计数 | 12 关节 / 12 电机 / 12 传感器，名称与顺序不变 | **PASS**：三 world 均 12/12/12 |
| 锚点 | 每个 `endPoint.translation == anchor` | **PASS**：生成器自检 + 静态检查 |
| 限位 | 12 关节均有软限位与硬限位 | **PASS**：每 world 各 12 组硬/软限位 |
| 动力学 | 质量总和、惯量正定、质心和包络检查通过 | **PASS**：生成器自检；总质量 10.496 kg |
| 姿态 | 模型零位四腿对称屈膝，约 120° 膝部内角，站立高接近 0.26 m | **PASS（静态几何）**：膝内角 120°、下落 0.2598 m |
| 接触 | 足端球/弧面触地，不是小腿盒体边缘 | **PASS（结构 + 高度）**：4 个球碰撞体，最低点约为 z=0 |
| 四连杆 | 8 组平行外观，仅视觉，不新增关节 | **PASS** |
| World 一致性 | 三个源 world 核心机器人段一致 | **PASS**：出生/controller 不同；`parkour_dev` 另有既有 `supervisor TRUE` |
| Webots 加载 | 三世界均 0 ERROR、0 Motor/Sensor not found | **PASS**：退出码 0；仅既有重复 `Viewpoint` 警告 |
| RL 冒烟 | `parkour_dev` 50 步，42 维观测 / 12 维动作 | **PASS** |
| 兼容边界 | RL、TCP、控制器源码未改，维度和设备名不变 | **PASS**：源码未改；42/12 已冒烟，45/51 为静态常量 |

### 9.6 与旧模型的差异

| 对比项 | 旧 YoboGo 几何 | 2026-09-30 重构后 |
|--------|----------------|----------------------|
| 腿段 | 0.14 / 0.12 m，和恰好 0.26 m | **0.15 / 0.15 m + 约 120° 屈膝** |
| 模型零位 | 大腿、小腿共线，视觉直腿 | **自然屈膝站姿** |
| 关节限位 | 未启用位置限位 | **软限位 + 硬限位，暂定安全范围** |
| 足端碰撞 | Sphere 仅视觉，小腿 Box 触地 | **足端球/弧面参与碰撞** |
| 四连杆 | 无明显结构表达 | **仅外观件，跟随三驱动关节** |
| RL 接口 | 42/45/51 维观测、12 维动作 | **维度不变，但旧 checkpoint 不兼容** |

> 本轮不修改 RL、观测、动作、奖励或 C++ 控制器。几何、零位、碰撞和惯量变化后，旧 checkpoint 只作为历史模型资产，不作为新模型的回放、热启动或正式验收依据。
>
> **控制器遗留**：`mini_cheetah_controller`、`manual_control` 仍使用旧直腿/Mini Cheetah 常量，新模型运行时可能频繁触及暂定限位；旧 P1–P4 checkpoint 必须重新训练后才能作为新模型结果。
