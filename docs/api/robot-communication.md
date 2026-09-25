# 机器人通信协议 API

YoboGo-10S 机器狗系统的通信协议汇总：UDP 端口分配、socketServer 指令协议、LCM 消息、SPI 数据包、SSH 连接信息、运动指令格式。

依据源码：

- `YoboGo-control/track1.6/track/{colorgroup.cpp, udputil.cpp, lcmutil.cpp}`
- `2026robocup中型组比赛资料10/socketServer/widget.cpp`
- `YoboGo-control/robot-software/robot/src/rt/{rt_spi.cpp, HardwareBridge.cpp, rt_socket.h}`
- `YoboGo-control/robot-software/robot/include/rt/rt_spi.h`

---

## 目录

1. [UDP 端口分配表](#1-udp-端口分配表)
2. [socketServer 指令协议（UDP 8000）](#2-socketserver-指令协议udp-8000)
3. [LCM 通信协议](#3-lcm-通信协议)
4. [SPI 通信协议（UP Board ↔ SPIne 板）](#4-spi-通信协议up-board--spine-板)
5. [SSH 连接信息](#5-ssh-连接信息)
6. [运动指令格式（UDP 20001）](#6-运动指令格式udp-20001)
7. [TCP 服务端口（robot-software）](#7-tcp-服务端口robot-software)

---

## 1. UDP 端口分配表

| 端口 | 协议 | 方向 | 用途 | 源码位置 |
|------|------|------|------|----------|
| **30015** | UDP | 机器人 → 上位机(10.0.0.30) | 发送摄像头**原始图像**（JPEG 编码，接收端 `imdecode` 显示） | `colorgroup.cpp:223` / socketServer `udpSocket` |
| **30016** | UDP | 机器人 → 上位机 | 发送**二值化图像**（颜色阈值处理结果） | `colorgroup.cpp:228` / socketServer `udpSocket2` |
| **30017** | UDP | 机器人 → 上位机 | 发送**颜色阈值反馈**（6 字节，用于滑块同步） | `colorgroup.cpp:279` / socketServer `udpSocket3` |
| **8000** | UDP | 上位机 → 机器人(10.0.0.34) | **控制指令**：颜色选择 / 阈值设置 / 保存 / 运行模式切换 | `udputil.cpp:11`（`SERV_PORT`） |
| **30014** | UDP | 上位机 → 机器人 | 颜色阈值接收端口（绑定本机 10.0.0.34） | `colorgroup.cpp:90` |
| **10020** | UDP | 视觉(10.0.0.34) → 比赛代码(10.0.0.30) | **目标点坐标**：视觉处理程序向比赛策略程序发送目标点 | 比赛代码 `strategy.cpp`（资料 PDF 说明） |
| **20001** | UDP | 比赛代码(10.0.0.30) → 运动控制板(STM32) | **运动控制指令** | 比赛代码 `communication.cpp` / 运动控制代码（资料 PDF 说明） |

### 1.1 通信拓扑

```
                    ┌─────────────────────────────────────────────┐
                    │              上位机 10.0.0.30               │
                    │  socketServer        比赛代码 ./main (sudo)  │
                    └──────┬───────────────────▲──────────┬───────┘
             UDP 30015/16/17│(图像/阈值)        │10020      │20001
                            ▼                   │          │(运动指令)
                    ┌───────────────────────────┴──────────┴───────┐
                    │         机器人端 10.0.0.34 (UP Board)        │
                    │  视觉程序 track1.6 / /home/dog/dogvision     │
                    │  运动控制 robot-software                     │
                    └──────────────────────┬───────────────────────┘
                                           │ SPI (132B/包)
                    ┌──────────────────────┴───────────────────────┐
                    │        SPIne 板 ↔ STM32F446 (FOC 电机驱动)   │
                    └──────────────────────────────────────────────┘

  上位机 ──UDP 8000（单字节模式指令 / 8字节阈值）──▶ 机器人端 10.0.0.34
```

### 1.2 运行模式切换指令（UDP 8000，单字节）

track1.6 `UdpUtil` 接收上位机单字节指令（ASCII）：

| 字节 | 模式 |
|------|------|
| `'0'` (0x30) | limitHeight 限高（代码中已注释，不生效） |
| `'1'` (0x31) | residence 住户区 |
| `'2'` (0x32) | stop 停止/交付 |
| `'3'` (0x33) | stop 停止 |

---

## 2. socketServer 指令协议（UDP 8000）

上位机 socketServer 向 `10.0.0.34:8000` 发送的报文格式：

| 指令类型 | 字节 0 | 字节 1 | 字节 2~7 | 长度 |
|----------|--------|--------|----------|------|
| **颜色选择** | `0` | 颜色索引 (0-7) | — | 2 字节 |
| **阈值设置** | `1`（mode） | low1 | low2, low3, high1, high2, high3 | 8 字节 |
| **保存参数** | `2` | — | — | 1 字节 |

颜色索引（8 种可识别颜色）：

| 索引 | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|------|---|---|---|---|---|---|---|---|
| 颜色 | yellow | white | blue | violet | brown | green | red | orange |

阈值参数为 HSV 三通道高低阈值（各 0~255），保存到机器人端 `colorGroup.txt`。

**回传数据**：

- 30015/30016：JPEG 图像字节流（接收端 `cv::imdecode` → `imshow`）
- 30017：6 字节当前阈值，驱动上位机滑块同步

---

## 3. LCM 通信协议

LCM（Lightweight Communications and Marshalling）用于机器人端**进程间/组件间**通信，以及上位机参数调试。

### 3.1 传输参数

| 项目 | 值 |
|------|-----|
| 组播地址 | `239.255.76.67`（LCM 默认组） |
| 端口 | `7667`（track1.6 `lcmutil.cpp:17`；LCM 默认） |
| TTL | `getLcmUrl(255)` |
| 拔网线独立运行 | `sudo ifconfig lo multicast` + `sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo` |

### 3.2 主要通道（robot-software）

| 通道名 | 类型 | 发布方 → 订阅方 | 内容 |
|--------|------|-----------------|------|
| `interface` | `gamepad_lcmt` | 上位机 → HardwareBridge | 手柄/游戏杆输入 |
| `interface_request` | `control_parameter_request_lcmt` | 调试端 → HardwareBridge | 控制参数修改请求 |
| `interface_response` | `control_parameter_response_lcmt` | HardwareBridge → 调试端 | 参数修改响应 |
| `leg_control_command` | `leg_control_command_lcmt` | RobotRunner | 12 关节期望位置/速度/增益/前馈 |
| `leg_control_data` | `leg_control_data_lcm` | RobotRunner | 关节实际状态反馈 |
| `state_estimator` | `state_estimator_lcmt` | 状态估计器 | 机体位姿/速度估计 |
| `hw_vectornav` | `vectornav_lcmt` | rt_vectornav | IMU（四元数/角速度/加速度） |
| `main_cheetah_visualization` | `cheetah_visualization_lcmt` | HardwareBridge（60Hz） | 可视化数据 |
| `ecat_cmd` / `ecat_data` | — | Cheetah3 EtherCAT 桥 | EtherCAT 电机总线（本机为 MiniCheetah/SPI 分支，不使用） |

参数加载方式：`./robot m r f`（从文件加载参数）或默认走 LCM（`l`）。

---

## 4. SPI 通信协议（UP Board ↔ SPIne 板）

源码：`robot/src/rt/rt_spi.cpp`、`robot/include/rt/rt_spi.h`。UP Board（x86）通过 SPI 与 SPIne 板通信，SPIne 板桥接到 STM32 电机驱动（详见 `docs/architecture/system-overview.md`）。

### 4.1 链路参数

| 项目 | 值 |
|------|-----|
| 设备节点 | `/dev/spidev2.0`、`/dev/spidev2.1`（两条 SPI 链，各挂 1 块 spine 板，每板驱动 2 条腿） |
| 模式 | `SPI_MODE_0` |
| 位宽 | 8 bit/word |
| 时钟 | 6 MHz（`spi_speed = 6000000`） |
| 帧长 | 66 × 2 字节 = **132 字节/包**（`K_WORDS_PER_MESSAGE = 66`） |
| 字节序 | 总线上传输时每 16 位字**大小端翻转**（发送 `tx_buf[i] = (w>>8)\|(w<<8)`，接收对称翻转） |
| 校验 | XOR 校验和（`xor_checksum`，按 32 位字异或） |

### 4.2 命令包 spine_cmd_t（x86 → SPIne，132 字节）

```c
typedef struct {
  float   q_des_abad[2];    // 外展关节期望角 (rad)
  float   q_des_hip[2];     // 髋关节期望角
  float   q_des_knee[2];    // 膝关节期望角
  float   qd_des_abad[2];   // 期望角速度 (rad/s)
  float   qd_des_hip[2];
  float   qd_des_knee[2];
  float   kp_abad[2];       // 位置刚度
  float   kp_hip[2];
  float   kp_knee[2];
  float   kd_abad[2];       // 阻尼
  float   kd_hip[2];
  float   kd_knee[2];
  float   tau_abad_ff[2];   // 力矩前馈 (N·m)
  float   tau_hip_ff[2];
  float   tau_knee_ff[2];
  int32_t flags[2];         // bit0=使能力矩限幅, bit1=弱力矩模式(wimp)
  int32_t checksum;         // = XOR 前 32 个 uint32 字
} spine_cmd_t;              // 15×2×4 + 2×4 + 4 = 132 字节
```

每条 SPI 链承载 2 条腿（`spi_board * 2` 选择 leg_0），两条链合计 4 腿 12 关节。

### 4.3 数据包 spine_data_t（SPIne → x86，84 字节有效）

```c
typedef struct {
  float   q_abad[2];        // 关节实际位置 (rad)
  float   q_hip[2];
  float   q_knee[2];
  float   qd_abad[2];       // 关节实际速度
  float   qd_hip[2];
  float   qd_knee[2];
  int32_t flags[2];         // 板端状态标志
  int32_t checksum;         // = XOR 前 14 个 uint32 字
  float   tau_abad[2];      // 实际力矩
  float   tau_hip[2];
  float   tau_knee[2];
} spine_data_t;             // 6×2×4 + 2×4 + 4 + 3×2×4 = 84 字节
```

接收时只回读 42 个 16 位字（84 字节），并对 `spine_data` 做 14 字 XOR 校验；校验失败打印 `SPI ERROR BAD CHECKSUM`。

### 4.4 坐标/零位换算（发往 spine 前）

代码在 `spi_to_spine()` 中按腿做符号与零位补偿（单位：rad）：

```c
q_abad' = q_abad * abad_side_sign + abad_offset     // side_sign: {-1,-1,1,1}
q_hip'  = q_hip  * hip_side_sign  + hip_offset      // side_sign: {-1,1,-1,1}
q_knee' = q_knee / knee_side_sign + knee_offset     // side_sign: {±0.6429}
```

零位：`hip_offset = ±(π-8°)/2、±(π-6°)/2`，`knee_offset = ±K_KNEE_OFFSET_POS(4.35 rad)`。

### 4.5 力矩限幅（板端由 flags 决定）

| 状态 | abad / hip / knee 上限 (N·m) |
|------|------------------------------|
| 未使能（flags bit0=0） | 0 / 0 / 0（无力矩） |
| 正常使能 | 17 / 17 / 26 |
| 弱力矩（bit1=1） | 6 / 6 / 6 |

---

## 5. SSH 连接信息

| 项目 | 值 |
|------|-----|
| 机器狗主机 | `10.0.0.34` |
| 登录命令 | `ssh user@10.0.0.34` |
| 用户名 / 密码 | `user` / `123456` |
| 主机名 | `Robot`（登录横幅 `user@Robot`） |
| 本机前置 IP | `10.0.0.30`（`robot` 连接，见 `docs/features/network-debug-notes.md`） |
| 文件传输 | `scp -r <目录> user@10.0.0.34:/home/user/`（失败时加 `sudo`） |
| 默认 SSH 端口 | 22（`No route to host` 属网络层问题，非端口问题） |
| 视觉程序路径 | `/home/dog/dogvision` |
| TX2 中转（如配备） | `ssh tx2@192.168.0.145` 再跳 UP Board |

标准顺序：`ping 10.0.0.34` → `ssh` → `scp`（ping 不通不要 scp）。

---

## 6. 运动指令格式（UDP 20001）

比赛代码 → 运动控制板（STM32）的运动指令：

| 项目 | 规格 |
|------|------|
| 传输层 | UDP，目标 `10.0.0.34:20001`（目标 IP 在 `communication.cpp` 中配置） |
| 编码 | **ASCII 数值字符串，逗号分隔，以 `\n` 结尾** |
| 示例 | `0.1,0.0,0.0\n`（左轮/右轮速度等分量）、`0.1,0.3,1.0\n`、`1.0,1.0,1.0\n`（特殊命令） |
| **安全超时** | **1.5 秒**内未收到新指令 → 运动控制板**自动停止电机**（防失控） |

配套的 IP 修改点（调试文档要求两处同步改）：

1. `strategy.cpp`（搜 `10.0.0`）：目标 `10.0.0.34`（视觉机）、监听 `10.0.0.30`（上位机）
2. `communication.cpp`（搜 `10.0.0`）：目标 `10.0.0.34`（运动控制板）、监听 `10.0.0.30`

---

## 7. TCP 服务端口（robot-software）

| 端口 | 协议 | 用途 | 源码 |
|------|------|------|------|
| **1986** | TCP | robot-software 对外状态/控制 socket 服务（上位机或外部客户端连接） | `rt_socket.h`（`SERV_PORT`） |

硬件运行模式（需 sudo）：

```bash
./robot m r f    # hardware 模式，f=从文件加载参数（l/缺省=LCM）
./robot m s      # 仿真模式（SimulationBridge 共享内存）
```

---

## 附：端口速查

```
上位机(10.0.0.30)                          机器人端(10.0.0.34) / 运动控制板
─────────────────                          ────────────────────────────────
socketServer ◀── UDP 30015 原始图像 ──┐
socketServer ◀── UDP 30016 二值图像 ──┼── 视觉程序 (track1.6 / dogvision)
socketServer ◀── UDP 30017 阈值反馈 ──┘
socketServer ──▶ UDP 8000  指令 ──────── UdpUtil（模式切换 / 阈值 / 保存）
比赛代码     ◀── UDP 10020 目标点 ────── 视觉处理
比赛代码     ──▶ UDP 20001 运动指令 ──── STM32 运动控制板（1.5s 超时停机）
调试/参数    ◀──▶ LCM 239.255.76.67:7667  robot-software 各组件
上位机       ◀──▶ TCP 1986               robot-software socket 服务
user@10.0.0.34:22  SSH（密码 123456）
```
