# 系统架构 — YoboGo-10S 机器狗

2026 RoboCup 中国机器人大赛中型组机器狗项目的整体架构：三端协作、通信拓扑、软件模块关系、硬件规格与数据流。

> 关联文档：
> - `docs/api/robot-communication.md` — 通信协议细节（端口 / LCM / SPI）
> - `docs/features/robot-software-analysis.md` — 运动控制代码分析
> - `docs/features/track1.6-analysis.md` — 视觉循迹代码分析
> - `docs/features/network-debug-notes.md` — 网络连接与调试
> - `docs/features/webots-sim-guide.md` — 仿真验证环境

---

## 目录

1. [三端架构](#1-三端架构)
2. [通信拓扑](#2-通信拓扑)
3. [软件模块关系](#3-软件模块关系)
4. [硬件规格汇总](#4-硬件规格汇总)
5. [数据流图](#5-数据流图)

---

## 1. 三端架构

```
┌─────────────────────────────────────────────────────────────────────────┐
│ 端 1：上位机（开发/比赛主机，10.0.0.30）                                  │
│                                                                         │
│  · socketServer     Qt GUI，颜色阈值远程调试（收 30015/16/17，发 8000）    │
│  · 比赛代码 ./main   比赛策略（收 10020 目标点，发 20001 运动指令，需 sudo） │
│  · 本项目开发机      scp 部署代码、SSH 运维、Webots 仿真                   │
└──────────────────────────────┬──────────────────────────────────────────┘
                               │ 以太网（10.0.0.0/24，UDP/TCP/SSH）
┌──────────────────────────────┴──────────────────────────────────────────┐
│ 端 2：机器人端计算机（UP Board，10.0.0.34）                               │
│                                                                         │
│  · 视觉程序（track1.6 / /home/dog/dogvision）                            │
│      摄像头采集 → HSV 颜色识别 → 二值化 → 目标点/循迹决策                  │
│  · 运动控制程序（robot-software，MIT Mini Cheetah 架构）                   │
│      状态估计 + MPC/WBC → 12 关节 PD/力矩指令                             │
│  · 手柄遥控（Qt QGamepad / SBUS）、LCM 参数调试                           │
└──────────────────────────────┬──────────────────────────────────────────┘
                               │ SPI（/dev/spidev2.0 & 2.1，132B/包，6MHz）
┌──────────────────────────────┴──────────────────────────────────────────┐
│ 端 3：运动控制板（SPIne 板 + STM32F446 电机驱动板）                        │
│                                                                         │
│  · SPIne 板    x86 ↔ MCU 桥接（SPI 1kHz）                                │
│  · STM32F446   180MHz，FOC 矢量控制 40kHz，DRV8323 驱动器，               │
│                AS5047P 14 位磁编码器，CAN bus 1Mbps（板间）               │
│  · 12 无刷电机 4 腿 × 3 关节（abad / hip / knee）                          │
│  · 安全机制    运动指令 1.5s 超时自动停机；力矩限幅；遥控 SwE 急停          │
└─────────────────────────────────────────────────────────────────────────┘
```

**职责划分**：

| 端 | 角色 | 关键 IP / 接口 |
|----|------|----------------|
| 上位机 | 策略决策 + 调试工具 + 开发环境 | `10.0.0.30`，UDP 对外 |
| 机器人端计算机 | 感知（视觉）+ 高层运动控制（MPC/WBC） | `10.0.0.34`，摄像头 / SPI / 手柄 |
| 运动控制板 | 实时电机控制（FOC）+ 安全保护 | SPI / CAN，1kHz+ 实时环 |

---

## 2. 通信拓扑

### 2.1 跨端通信（以太网 UDP/TCP，10.0.0.0/24）

| 链路 | 协议/端口 | 说明 |
|------|-----------|------|
| 视觉 → 上位机 | UDP 30015/30016/30017 | 原始图 / 二值图 / 阈值反馈（调阈值用） |
| 上位机 → 视觉 | UDP 8000（+30014） | 模式切换单字节指令、颜色选择、8 字节阈值、保存 |
| 视觉 → 比赛代码 | UDP 10020 | 目标点坐标 |
| 比赛代码 → 运动控制板 | UDP 20001 | 逗号分隔运动指令（1.5s 超时停机） |
| 开发机 → 机器人 | SSH 22 / SCP | `user@10.0.0.34`（密码 123456）部署与运维 |
| 调试端 ↔ robot-software | TCP 1986 | 上位机 socket 服务（`SERV_PORT`） |

### 2.2 机器人端内部通信

| 链路 | 协议 | 说明 |
|------|------|------|
| 进程间组件 | **LCM** 组播 `239.255.76.67:7667` | `interface` / `leg_control_*` / `state_estimator` / `hw_vectornav` / 可视化等通道 |
| UP Board ↔ SPIne 板 | **SPI** `/dev/spidev2.0 & 2.1`，SPI_MODE_0，6MHz，132B/包 | 关节位置/速度 + KP/KD/力矩前馈；XOR 校验 |
| SPIne ↔ 电机驱动 | **CAN bus** 1Mbps | STM32 板间总线 |
| 遥控器 | Qt QGamepad（USB/UART）、SBUS | SwE 三档：急停(passive) / 站立 / 平衡 |
| 传感器 | VectorNav / Lord MicroStrain IMU、RealSense D435/T265 | IMU 数据经 LCM `hw_vectornav` 分发 |

详细协议见 `docs/api/robot-communication.md`。

### 2.3 物理网络

```
上位机 eno1 (10.0.0.30, robot 连接, never-default)
   │  网线（直连或经 5 口交换机）
   ▼
机器狗外壳 RJ45 → UP Board (10.0.0.34)

※ 原生 Ubuntu 直连存在物理层协商问题时用交换机中转，
  WiFi 与有线分属不同子网共存，见 docs/features/network-debug-notes.md
```

---

## 3. 软件模块关系

### 3.1 仓库结构

```
Dog1/                                    # 项目根
├── YoboGo-control/                      # 开源仓库（RRRexyz/YoboGo-control）
│   ├── robot-software/                  # 运动控制（MIT Mini Cheetah 分支）
│   │   ├── robot/src/                   #   RobotRunner / HardwareBridge / rt_* 驱动
│   │   ├── robot/include/rt/            #   rt_spi.h 等接口定义
│   │   ├── config/                      #   simulator-defaults.yaml 等参数
│   │   └── lcm-types/                   #   LCM 消息类型定义
│   ├── track1.6/                        # 视觉循迹（Qt 5.10 + OpenCV 5 + LCM）
│   │   └── track/{colorgroup,udputil,lcmutil}.cpp
│   └── YoboGo-10S使用说明书(开源).docx
├── webots-sim/                          # Webots R2025a 仿真（PD + trot 控制器）
├── 2026robocup中型组比赛资料10/           # 比赛资料
│   ├── socketServer/                    #   Qt 阈值调试 GUI
│   ├── 比赛代码/                        #   策略代码（本地为空，见资料 PDF）
│   ├── 运动控制代码/                     #   STM32 端代码（本地为空）
│   └── 中国机器人大赛调试说明文档.pdf
├── 四足仿生机器人基本原理及开发教程（…）/  # 13 章配套教材（PPTX）
├── Cheetah-Software/                    # MIT 官方 Cheetah-Software（参考）
├── quadruped_ctrl/                      # PyBullet 方案（依赖 ROS，暂弃）
├── scripts/                             # 连接/探测/传输脚本 ×4
└── docs/                                # 本文档体系
    ├── changes/    修改日志
    ├── features/   功能/专项文档
    ├── api/        通信协议 API
    └── architecture/ 系统架构（本文件）
```

### 3.2 运行时模块依赖

```
                 ┌──────────────┐   LCM interface    ┌──────────────────┐
                 │ 手柄/调试上位机 │ ───────────────▶ │  HardwareBridge   │
                 └──────────────┘                    │  (参数/手柄接入)   │
                                                     └────────┬─────────┘
                                                              ▼
┌────────────┐  视觉目标   ┌──────────────┐  关节指令  ┌──────────────────┐
│  摄像头     │ ─────────▶ │  决策/策略层   │ ────────▶ │   RobotRunner     │
└────────────┘            │ track1.6 /   │           │ 状态估计+控制器    │
                          │ 比赛代码      │ ◀──────── │ (MPC+WBC / PD)    │
                          └──────────────┘  state估   └────────┬─────────┘
                                                              ▼ SPI
                                                       ┌──────────────────┐
                                                       │ SPIne → STM32FOC │
                                                       │ → 12 电机         │
                                                       └──────────────────┘
```

- **robot-software**：`RobotRunner` 主循环组装状态估计（IMU/腿运动学）与控制器输出；`HardwareBridge` 按运行模式选择 SPI（MiniCheetah 硬件）/ EtherCAT（Cheetah3）/ 仿真桥；7 种通信接口（SPI / EtherCAT / TCP / LCM / SBUS / USB Joystick / 串口）
- **track1.6**：`colorGroup`（颜色识别 + UDP 图像推送）+ `UdpUtil`（8000 指令接收）+ `lcmutil`（LCM 运动控制），6 种运行模式（循迹/住户区/限高/停止交付/上楼梯/障碍）
- **仿真**：Webots 控制器独立实现 PD+trot 用于算法验证；robot-software 内置仿真走 `./robot m s`（SimulationBridge 共享内存），二者互补
- **控制模式切换**（robot-software 仿真）：6=JOINT_PD（电机上电）→ 3=STAND_UP（站立）→ 4=LOCOMOTION（trot）

---

## 4. 硬件规格汇总

### 4.1 整机：YoboGo-10S（山东优宝特 Yobotics）

| 项目 | 规格 |
|------|------|
| 型号 | YoboGo-10S（Y10S/Y10） |
| 尺寸 / 重量 | 485 × 275 × 300 mm，10 kg ± 2 kg |
| 自由度 | 12（4 腿 × 3 关节） |
| 结构材料 | 铝合金 |
| 电池 | 18650 锂电池，6.4 Ah，续航 ≥ 1.5h，40A 放电 |
| 供电保护 | 限流拨码开关（15A/20A/25A/无限流），峰值保护 15A；电机供电 12~24V |
| 操作系统 | Linux（Ubuntu） |
| 开源性 | 硬件 + 软件完全开源 |

### 4.2 计算与控制

| 模块 | 规格 |
|------|------|
| 主控计算机 | **UP Board**：Intel Atom X5-Z8350，4GB RAM，Ubuntu；IP `10.0.0.34`；网口 RTL8111/8168 |
| 运动控制 MCU | **STM32F446**：180MHz，FOC 矢量控制环 40kHz |
| 电机驱动 | DRV8323 栅极驱动 + AS5047P 14 位磁编码器 |
| 桥接 | SPIne 板：x86↔MCU，SPI 1kHz（132B/包） |
| 板间总线 | CAN bus 1Mbps |
| 可选算力 | Jetson TX2（256 CUDA 核心，视觉）；`ssh tx2@192.168.0.145` |

### 4.3 传感器 / 外设

| 设备 | 说明 |
|------|------|
| IMU | VectorNav + Lord MicroStrain（双 IMU，rt_vectornav / 串口端口 0） |
| 相机 | RealSense D435 深度相机（1280×720@90fps）、板载摄像头（视觉循迹） |
| 位姿 | RealSense T265（可选） |
| 遥控 | AT9s / Taranis X7（SBUS）、Qt QGamepad USB 手柄；SwE 开关三档（急停/站立/平衡） |
| 开发机网卡 | HP OMEN：Realtek RTL8111/8168（r8168-dkms 8.055.00） |

### 4.4 控制算法栈

| 层 | 算法 / 参数 |
|----|-------------|
| 高层步态 | MPC（模型预测控制，qpOASES QP 求解，~1kHz）+ WBC（零空间投影全身控制） |
| 关节层 | PD（仿真默认 KP=[3,3,3]、KD=[1,0.2,0.2]，限幅 15N·m；硬件力矩限 17/17/26 N·m） |
| 底层 | FOC 电流环 40kHz |
| 遥控档位 | passive(急停) → stand_up → balance |

---

## 5. 数据流图

### 5.1 比赛运行数据流（端到端）

```
摄像头图像
   │
   ▼
视觉程序 (10.0.0.34, track1.6 / dogvision)
   │  HSV 颜色阈值二值化 ──── 阈值由 socketServer 经 UDP 8000 在线调节
   │
   ├── UDP 30015/30016/30017 ──▶ 上位机 socketServer（显示原图/二值图/同步滑块）
   │
   ▼ 目标点坐标
   │  UDP 10020
   ▼
比赛代码 ./main (10.0.0.30, 比赛策略 team1/team2)
   │  策略计算：目标点 → 动作决策
   │  UDP 20001（"0.1,0.3,1.0\n" 逗号分隔，1.5s 超时停机）
   ▼
运动控制板 (STM32F446)
   │  指令 → 关节期望 → FOC 电流环 40kHz
   ▼
12 无刷电机 → 机器狗运动
```

### 5.2 高层运动控制数据流（robot-software，独立链路）

```
IMU (VectorNav/Lord) ──┐
关节编码器 (经 SPI 回读) ─┼─▶ 状态估计 (state_estimator, LCM 发布)
遥控器 (SBUS/QGamepad) ─┘              │
                                       ▼
                        ┌── RobotRunner 主循环 ──┐
                        │  控制模式: passive/stand/locomotion │
                        │  MPC (qpOASES ~1kHz) + WBC          │
                        │  关节 PD + 力矩前馈                   │
                        └──────────┬──────────────┘
                                   │ spi_command_t (4腿×3关节: q_des/qd_des/kp/kd/tau_ff)
                                   ▼ SPI 132B/包 (XOR 校验, 字节翻转)
                        SPIne 板 ── CAN ──▶ STM32 FOC ──▶ 电机
                                   ▲
                                   └── spine_data_t: q/qd/tau 回读（闭环）
```

### 5.3 仿真数据流（Webots，算法验证）

```
mini_cheetah.wbt (250Hz, basicTimeStep=4ms)
   │  关节 PositionSensor + Gyro/Accelerometer/InertialUnit
   ▼
mini_cheetah_controller (PD 站立 / trot 对角步态, 键盘 S/T/R)
   │  wb_motor_set_torque (限幅 15N·m)
   ▼
Webots 物理引擎 → 机器人运动
```

---

*文档版本：2026-09-22。硬件与协议细节以 `docs/api/robot-communication.md` 和源码为准。*
