# Webots 仿真使用指南

Mini Cheetah 四足机器人 Webots 仿真项目：PD 站立 + trot 步态，C++ 控制器，无需 ROS。

- **Webots 版本**：R2025a（apt 包 `webots 2025a`）
- **安装路径**：`/usr/local/webots`
- **项目路径**：仓库根目录 `webots-sim/`
- **控制器**：`mini_cheetah_controller`（334 行 C++，已编译，仿真可运行）

---

## 目录

1. [安装方法](#1-安装方法)
2. [项目结构](#2-项目结构)
3. [运行方法](#3-运行方法)
4. [控制器说明（PD + trot）](#4-控制器说明pd--trot)
5. [按键操作](#5-按键操作)
6. [中文界面设置](#6-中文界面设置)
7. [常见问题](#7-常见问题)

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
│   └── mini_cheetah.wbt            # 世界文件（#VRML_SIM R2025a）
├── controllers/
│   └── mini_cheetah_controller/
│       ├── mini_cheetah_controller.cpp   # 控制器源码（334 行）
│       ├── Makefile                      # Webots 标准 Makefile.include
│       ├── mini_cheetah_controller        # 编译产物（可执行）
│       └── build/release/                # 中间产物（.o / .d）
├── urdf/
│   ├── mini_cheetah.urdf           # Mini Cheetah URDF（参考模型）
│   └── meshes/                     # 网格（.dae，4 个连杆）
└── protos/                         # 自定义 PROTO（当前为空）
```

### 2.1 世界文件要点

- 头部声明：`#VRML_SIM R2025a utf8`
- `basicTimeStep 4` —— 4ms 步长，对应控制器 250Hz 控制周期
- 机器人使用 **Webots 内置节点**（`Robot` + 内置 `RotationalMotor` / `PositionSensor` / `Gyro` / `Accelerometer` / `InertialUnit`），**不依赖外部 PROTO**，克隆仓库后可直接打开
- 关节命名：`{fr,fl,hr,hl}_{abd,hip,kn}_motor` / 对应 `_sensor`；`controller "mini_cheetah_controller"` 绑定本项目控制器
- 初始高度 `translation 0 0 0.38`

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

腿长常量（正运动学参考）：`L_ABD=0.062`、`L_HIP=0.209`、`L_KNEE=0.18`；髋部位置 ±0.19m（前后）× ±0.111m（左右）。

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
