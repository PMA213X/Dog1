# Webots 仿真使用指南

四足机器人 Webots 仿真项目：PD 站立 + trot 步态，C++ 控制器，无需 ROS。包含两个世界：步态试验台（Mini Cheetah 盒模型）与 MSL 标准比赛场地（**YoboGo-10S 实机规格模型** + 前置相机，见 [第 9 节](#9-yobogo-10s-实机模型与相机)）。

- **Webots 版本**：R2025a（apt 包 `webots 2025a`）
- **安装路径**：`/usr/local/webots`
- **项目路径**：仓库根目录 `webots-sim/`
- **控制器**：`mini_cheetah_controller`（334 行 C++，已编译，仿真可运行）
- **世界文件**：`mini_cheetah.wbt`（原步态试验台）与 `msl_match.wbt`（MSL 比赛场地，见 [第 8 节](#8-msl-比赛场地msl_matchwbt)）

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
9. [YoboGo-10S 实机模型与相机](#9-yobogo-10s-实机模型与相机)

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
│   └── msl_match.wbt               # MSL 比赛场地世界（见第 8 节）
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
- 初始高度：`mini_cheetah.wbt` 为 `translation 0 0 0.38`；`msl_match.wbt` 的 YoboGo-10S 为 `translation 8 0 0.26`（站立机身高，见 [第 9 节](#9-yobogo-10s-实机模型与相机)）

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

所有 12 关节的期望角为中立位 `STANDING_DES = {0, 0, 0}`（本模型中 0 位腿竖直向下），期望速度 0，纯 PD 回位。

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

腿长常量（正运动学参考）：`L_ABD=0.062`、`L_HIP=0.209`、`L_KNEE=0.18`；髋部位置 ±0.19m（前后）× ±0.111m（左右）。这些是**控制器源码内**的 Mini Cheetah 尺寸常量；`msl_match.wbt` 的 YoboGo-10S 世界模型几何与之不同（大腿 0.14 / 小腿 0.12 等，见 [第 9 节](#9-yobogo-10s-实机模型与相机)），trot 中的 `swing_knee` 仍按控制器常量计算。

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
| 机器人 | 1 | inline YoboGo-10S（`yobogo_10s`），出生点 `8 0 0.26`，绕 Z 轴旋转 180°（面向 -X，朝向场地中心），见 [第 9 节](#9-yobogo-10s-实机模型与相机) |

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

机器人模型在 `msl_match.wbt` 中已重建为 **YoboGo-10S 实机规格模型**（由 `webots-sim/tools/gen_yobogo_robot.py` 生成，详见 [第 9 节](#9-yobogo-10s-实机模型与相机)），采用 R2025a 兼容语法（`child Solid` → `endPoint Solid`、appearance 包入 `Shape {}`、`inertiaMatrix` 两行写法、带 `boundingObject` 等）。12 个电机 / 传感器全部找到，机器人可稳定站立。**`mini_cheetah.wbt` 仍保留旧 Mini Cheetah 盒模型与 Gazebo 风格语法，未同步重建。**

### 8.6 注意事项

- **灯光必须写在 world 顶层**：Webots 禁止在 `Group` / `Pose` 子节点中放置 `DirectionalLight`，因此两盏灯直接写在 `msl_match.wbt` 顶层，而不是 `MslEnvironment.proto` 内。
- **旋转朝向**：`rotation 0 0 1 π` 表示绕 Z 轴转 180°（面向 -X）；若误写为 `0 1 0 π`，会把机器人 / 球门上下颠倒。

---

## 9. YoboGo-10S 实机模型与相机

`msl_match.wbt` 中的机器人已由 **Mini Cheetah 盒模型** 重建为 **YoboGo-10S 实机规格模型**（方案 A：按包络缩放运动学链），并新增前置相机 `front_camera`。机器人段由脚本生成，可重新生成覆盖；`mini_cheetah.wbt` 仍保留旧盒模型。

### 9.1 模型变更说明

- **几何 / 质量**：按说明书包络 **485×275×300 mm**、总质量 **10.5 kg** 缩放运动学链；站立机身高取实机控制器参数 **0.26 m**
- **动力学**：机身惯量直接取控制器模型 `RPC_inertia`；关节力矩上限对齐 CAN 协议 **±18 N·m**
- **相机**：机身前上方新增 `Camera`（640×480），供视觉 / 巡线等算法接入
- **关节命名不变**：`{fr,fl,hr,hl}_{abd,hip,kn}_motor` / 对应 `_sensor`，仍绑定 `mini_cheetah_controller`
- **机器人名**：`yobogo_10s`（出生点 `translation 8 0 0.26`，绕 Z 轴 180° 朝向场地中心）

### 9.2 参数溯源

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

### 9.3 相机参数与可调范围

Webots R2025a 中 Camera 的视场角字段名是 **`fieldOfView`**（**不是** `fov`），取值范围 (0, π)，单位弧度；光轴为相机局部 **+X** 轴（实测验证：+Y / 正角度绕 Y 轴右手旋转 = 低头）。

| 参数 | 字段 | 当前值 | 可调范围 | 说明 |
|------|------|--------|----------|------|
| 水平视场角 | `fieldOfView` | 1.05（≈60°） | 0.96–1.22（≈55°–70°） | 推断值，可按视觉算法需求调整 |
| 俯仰角 | `rotation 0 1 0 θ` | θ=0.61（低头 35°） | θ=0.52–0.70（低头 30°–40°） | 绕 +Y 右手旋转，正值 = 低头 |
| 安装位置 | `translation` | `0.2 0 0.055` | — | 机身坐标系：+X 前、+Z 上 |
| 分辨率 | `width` / `height` | 640×480 | — | 与实机视觉代码一致 |
| 设备名 | `name` | `front_camera` | — | 控制器按此名取像 |

> 相机世界高度 ≈ 0.26 + 0.055 ≈ 0.315 m。修改俯仰或 FOV 时，可直接改 world 中 `Camera` 节点，或改生成脚本顶部 `CAMERA_*` 常量后重新生成（见 9.4）。

### 9.4 重新生成机器人段

```bash
python3 webots-sim/tools/gen_yobogo_robot.py <输出>
# 不带参数时打印到 stdout
```

脚本内每个数值常量均带来源标注（`[说明书]` / `[yaml]` / `[估算]` / `[合同]`），并内置自检（总质量 ≈10.5 kg、`fieldOfView` 字段名等）；修改参数后重新生成即可覆盖 world 中的机器人段。

### 9.5 与旧模型的差异

| 对比项 | 旧（Mini Cheetah 盒模型） | 新（YoboGo-10S） |
|--------|--------------------------|------------------|
| 总质量 | 8.85 kg | **10.5 kg** |
| 出生点 z | 0.38 m（`translation 0 0 0.38`） | **0.26 m**（`translation 8 0 0.26`） |
| 电机 `maxTorque` | 20 N·m | **18 N·m** |
| 相机 | 无 | `front_camera`，640×480 |
| 参数来源 | URDF / 估算 | 说明书 + 实机 yaml + 估算（见 9.2） |

> 控制器软件限幅 `MAX_TORQUE = 15.0` N·m 不变（见 4.1），低于电机能力 18 N·m，属正常保护。
