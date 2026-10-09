# YoboGo-10S 机器人分层备份方案

> 文档日期：2026-10-09  
> 当前状态：**只读盘点与目标目录准备已完成；实际文件复制尚未开始**  
> 文档性质：已核实事实、分层备份计划与后续验收要求  
> 本地备份父目录：`/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/robot_backups`

本文档只记录已经核实的机器人与本机事实，以及尚未执行的备份步骤。凡标注
“已完成”的内容可以视为既有事实；标注“计划”的内容尚未执行；标注
“待远端核验”或“待验证”的内容不得当作结论使用。

---

## 1. 目的与范围

### 1.1 目的

1. 在本地 `10.0.0.30` 上建立 YoboGo-10S 机器人 `10.0.0.34` 的可恢复备份。
2. 先保护与行走直接相关的 MIT/Cheetah 源码、配置、启动脚本和运行环境元数据，
   再逐步覆盖完整用户目录、系统配置和全系统归档。
3. 将“源端统计、传输、目标端统计、哈希与归档完整性”分开记录，
   避免把计划、部分复制或未经校验的文件描述为成功备份。
4. 保留 Unix 属主、权限、ACL、xattr 等恢复所需元数据，并明确热复制风险。

### 1.2 范围

- **已纳入事实范围**：本机与机器人网络连通性、只读软硬件盘点、本地工具版本、
  本地目标目录结构与目标分区容量。
- **计划范围**：核心代码、`/home/user`、`/etc`、系统与硬件元数据、
  指定系统目录以及整系统 `tar + zstd` 归档的复制和校验。
- **不在本轮文档任务内**：执行 SSH、`rsync`、`tar` 复制、`sudo`、
  Git 写操作、机器人重启/关机、关闭 WiFi、修改机器人配置或修改备份文件。

### 1.3 状态标记

| 标记 | 含义 |
| --- | --- |
| **已完成** | 已有明确只读检查或目录操作证据 |
| **计划** | 方案中安排执行，但本任务尚未执行 |
| **待远端核验** | 当前证据不足，必须在远端只读检查后补全 |
| **待验证** | 已有操作但尚未完成完整性、可恢复性或一致性验收 |

---

## 2. 已确认事实

### 2.1 网络与登录 — 已完成

- 本地上位机通过交换机直连机器人：`10.0.0.30/24 -> 10.0.0.34/24`。
- `ping 4/4`、`0% loss`。
- SSH 交互登录成功，远端主机名为 `Robot`、用户为 `user`。
- WiFi 接口 `wlo1` 保持连接并保留默认路由；**本方案不会建议关闭 WiFi**。

### 2.2 本地备份位置与目录 — 已完成

- 本地备份父目录：
  `/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/robot_backups`
- 本次独立时间戳目录：
  `/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/robot_backups/backup_2026-10-09_120612`
- 上述目录内已创建以下子目录，所有 `mkdir` 均为退出码 `0`：
  - `system/`
  - `home/`
  - `etc/`
  - `opt_var/`
  - `ros_workspace/`
  - `hardware_interfaces/`
  - `metadata/`
  - `logs/`

**当前进度**：仅完成独立时间戳目录及子目录创建，**当前尚未执行实际文件复制**。
本次尚未成批复制时，可继续使用上述已创建容器；一旦复制中断后重新开始，
或启动新的备份轮次，必须新建 `backup_YYYY-MM-DD_HHMMSS/`，
不得复用已经中断或已经完成的旧时间戳目录。

### 2.3 容量与本地工具 — 已完成

- 本地目标分区 `/dev/nvme0n1p2`：总容量 `1.7T`，已用 `1.3T`，可用 `432G`。
- 本地根分区：可用 `51G`。
- 本地工具：
  - `rsync 3.4.1`
  - GNU `tar 1.35`
  - `zstd 1.5.7`
  - OpenSSH `10.2p1`
  - `git 2.53.0`
- `7z` 不存在，不把 7z 作为备份或恢复依赖。
- 远端 `rsync`、`tar`、`zstd` 的具体版本为**待远端核验**；上述版本只代表本地工具。

### 2.4 只读盘点概览 — 已完成

- 操作系统：Ubuntu `16.04.6`。
- 内核：`4.4.86-rt99`，架构 `x86_64`。
- CPU：Intel Atom `x5-Z8350`，4 核。
- 内存：`1.4GiB`，无 swap。
- 机器人系统盘：`9.8GiB`，已用 `8.0GiB`，剩余 `1.3GiB`。
- 全系统逻辑估算约 `9.2GiB`，整盘未压缩约 `29GiB`。
- 未发现 ROS1/ROS2、Docker、`package.xml`、ROS workspace、launch、
  URDF 或 Xacro。
- `/home/user/robot-software` 没有 Git 元数据，无法确认版本历史或提交对应关系。

---

## 3. 远端硬件与接口

### 3.1 计算与存储平台 — 已完成

| 项目 | 已确认内容 |
| --- | --- |
| CPU | Intel Atom `x5-Z8350`，4 核 |
| 内存 | `1.4GiB`，无 swap |
| 系统盘 | `9.8GiB`，已用 `8.0GiB`，剩余 `1.3GiB` |
| 网卡 | Realtek `RTL8111/8168` |
| 显示 | Intel `i915` |
| USB | `xHCI` / `DWC3` |
| 串口 | CH340：`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` -> `/dev/ttyUSB0` |
| 串口权限 | `root:dialout`；`user` 不在 `dialout` 组 |
| I2C | `/dev/i2c-0` 至 `/dev/i2c-6`，均为 `root-only` |
| GPIO | `gpiochip0`、`gpiochip318`、`gpiochip373`、`gpiochip397`、`gpiochip456` |
| CAN | 未发现 CAN 设备 |
| IIO | 未发现 IIO 设备 |
| 已加载模块 | `ch341`、`usbserial`、`ti_adc081c`、`industrialio` |

### 3.2 权限边界 — 已完成

- `user` 属于 `sudo` 组，但没有 `NOPASSWD`。
- `/root`、`/etc/shadow` 以及部分 `/etc`、`/var/lib` 内容需要 root。
- 当前只读盘点中的不可读项：`home` 4 项、`etc` 87 项、
  `var/lib` 23 项，以及 `/root` 整目录。
- 计划中的 root 内容复制必须使用安全、交互式授权；
  任何授权材料都不得写入文档、日志或备份清单。

---

## 4. 软件栈与行走代码

### 4.1 核心程序 — 已完成

- 核心控制程序位于 `/home/user/robot-software`，大小 `532MiB`。
- 其中 `build` 为 `522MiB`。
- 关键源码与配置目录为 `robot`、`actor_model`、`config`。
- 运行进程为 `./mit_ctrl m r f`，`PID=1818`。
- 该进程由 `/etc/rc.local` 通过 `/home/user/robot-software/build/run_mc.sh` 启动。

### 4.2 行走配置与二进制依赖 — 已完成

- 关键配置：
  - `/home/user/robot-software/config/cheetah-3-defaults.yaml`
  - `/home/user/robot-software/config/mini-cheetah-defaults.yaml`
  - `/home/user/robot-software/config/mc-mit-ctrl-user-parameters.yaml`
- 关键二进制/库：
  - `mit_ctrl`
  - `libbiomimetics.so`
  - `libWBC_state.so`
  - `libJCQP.so`
  - `libfootstep_planner.so`

### 4.3 其他程序与目录容量 — 已完成

- 其他程序：`track_run`（`129MiB`）、`track_build.tar.gz`（`51.9MB`）、
  `opencv_test`、`motion_test`、`cpp`、`listener`、
  `receive_gamepad`、`send-message`。
- 容量：
  - `/home/user`：`733MiB`
  - `/opt`：`4KiB`
  - `/srv`：`4KiB`
  - `/usr/local`：`13MiB`
  - `/etc`：`13MiB`
  - `/var/lib`：`291MiB`
  - `/usr`：`3.9GiB`
  - `/lib/modules`：`3.4GiB`
  - `/lib/firmware`：`235MiB`
  - `/boot`：`452MiB`
  - `/var`：`421MiB`

---

## 5. 动作、运动、遥控与传感器接口及限制

本章节基于机器人端源码、配置、运行进程、监听端口和设备权限的只读调查。
以下带 `/home/user/`、`/etc/`、`/dev/` 或 `/proc/` 的路径均为**机器人端绝对路径**。
“已完成”表示已有直接证据；“待远端核验”表示当前证据仍不足，不作猜测。

### 5.1 运动控制入口、状态与模式 — 已完成

- 运行链路：
  `PID 1804 rc.local` -> `PID 1807 run_mc.sh` -> `PID 1817 sudo`
  -> `PID 1818 ./mit_ctrl m r f`。
- `/etc/rc.local` 从 `/home/user/robot-software/build/` 启动控制程序。
- 启动参数语义来自
  `/home/user/robot-software/robot/src/main_helper.cpp`：
  - `m`：Mini Cheetah；
  - `r`：真实机器人；
  - `f`：从文件加载参数；不指定 `f` 时从 LCM 加载。
- 默认配置来自 `/home/user/robot-software/config/mini-cheetah-defaults.yaml`：
  `control_mode: 0`、`cheater_mode: 0`、`use_rc: 1`。
- 当前 `mit_ctrl` 以 root 运行，因此能够访问受限制的串口和 SPI 设备。

已确认动作/状态值来自
`/home/user/robot-software/robot/include/rt/rt_rc_interface.h`：

```text
OFF=0
STAND_UP=1
READY=2
QP_STAND=3
BACKFLIP_PRE=4
BACKFLIP=5
VISION=6
LOCOMOTION=11
RECOVERY_STAND=12
FRONT_JUMP=13
ROLL_OVER=14
ZOOM_SHOW=15
DANCE=16
TWIST=17
LANDING_BUFFER=18
STAND_DOWN=19
TWO_LEG_STANCE_PRE=20
TWO_LEG_STANCE=21
WAIT=31
RL_JOINT_PD=53
```

状态值和枚举名已确认；各状态在本机上的实时迁移条件和当前运行状态为
**待远端核验**。

### 5.2 命令、状态与网络通道 — 已完成，部分生效状态未确认

#### LCM

证据来自 `/home/user/robot-software/robot/src/HardwareBridge.cpp`、
`RobotRunner.cpp`：

| 方向 | channel | 内容 | 状态 |
| --- | --- | --- | --- |
| 订阅 | `interface` | 遥控/接口命令 | 已完成 |
| 订阅 | `interface_request` | 参数或接口请求 | 已完成 |
| 发布 | `interface_response` | 接口响应 | 已完成 |
| 发布 | `leg_control_command` | 腿部控制命令 | 已完成 |
| 发布 | `leg_control_data` | 腿部控制数据 | 已完成 |
| 发布 | `state_estimator` | 状态估计 | 已完成 |
| 发布 | `main_cheetah_visualization` | 可视化状态 | 已完成 |
| 发布 | `hw_vectornav` | VectorNav IMU | 已完成 |
| 发布 | `microstrain` | Microstrain/Lord IMU 数据 | 已完成 |
| 发布 | `spi_data` | SPI 关节反馈 | 已完成 |
| 发布 | `spi_command` | SPI 关节命令 | 已完成 |

- LCM URL 构造调用为 `getLcmUrl(255)`；这不能直接解释为固定 UDP 端口。
- 运行时 `UDP` 监听包含多个 LCM 端口，其中 `0.0.0.0:7667`
  重复出现 9 次；非 root 无法把全部 socket inode 归属到 `PID 1818`。
- 各 channel 的发布/订阅频率除下文已列频率外，其余为
  **待远端核验**。

#### TCP 控制/状态端口

- `robot/include/rt/rt_socket.h` 与 `robot/src/rt/rt_socket.cpp`
  定义服务器监听 `0.0.0.0:1986`，当前运行时为 `LISTEN`。
- 命令帧 `Client_Command_Message` 字段：
  `start`、`roll`、`pitch`、`yaw`、`velocity_x`、`velocity_y`、
  `omega_z`、`mode`、`step_height`、`gait`、
  `body_height_variation`、`offset_x`、`offset_y`、`end`。
- 回帧 `ReplyMessage` 包含位置、姿态、速度、12 个关节角、mode、
  voltage、gait；回帧周期为 50 ms。
- `recv()` 已执行，但帧校验、回帧和命令赋值位于注释块，
  `rt_rc_interface.cpp` 中 TCP 命令应用也已注释。
  **TCP 是否实际控制机器人：未确认**。

#### 进程参数

- `./mit_ctrl m r f` 是已确认的当前入口。
- `f` 决定从文件加载；三个 YAML 的加载优先级、运行时覆盖顺序和
  LCM 参数服务关系为 **待远端核验**。

### 5.3 控制频率与周期 — 已完成

| 接口/任务 | 周期或频率 | 证据 |
| --- | --- | --- |
| 主控制循环 | `controller_dt=0.002`，500 Hz | `/home/user/robot-software/config/mini-cheetah-defaults.yaml`、`RobotRunner.cpp` |
| 关节 SPI | 0.002 s，500 Hz | `HardwareBridge.cpp`、`rt_spi.cpp` |
| 可视化发布 | 0.0167 s，约 60 Hz | `RobotRunner.cpp` |
| TCP 回帧 | 50 ms | `rt_socket.cpp` |
| SBUS 读取任务 | 0.005 s，200 Hz | `rt_sbus.cpp` |
| Linux 手柄轮询 | 0.01 s，100 Hz | `HardwareBridge.cpp` |
| VectorNav | 输出配置 800/4，200 Hz | `rt_vectornav.cpp` |
| Microstrain/Lord | 1 kHz 调用 `updateLCM` 并发布 `microstrain` | `HardwareBridge.cpp` |

实际调度抖动、队列积压和端到端命令延迟为 **待远端核验**。

### 5.4 关节、速度、力矩与步态限制 — 已完成可确认项

#### 初始化关节位置

来自 `/home/user/robot-software/config/initial_jpos_ctrl.yaml`：

```text
target_jpos=[-0.6,-1.0,2.7, 0.6,-1.0,2.7, ...]
mid_jpos=[-1.8,0,2.7,1.8,0,2.7,-1.7,0.5,0.5,1.7,0.5,0.5]
```

`target_jpos` 当前只取得前两腿完整数值，其余元素为
**待远端核验**，不能补写推测值。

#### 期望位姿、速度和步态约束

来自 `/home/user/robot-software/config/mc-mit-ctrl-user-parameters.yaml`：

| 参数 | 数值 |
| --- | --- |
| `des_p` | `[0,0,0.26]` |
| `des_theta_max` | `[0,0.4,0]` |
| `des_dp_max` | `[1.0,0.5,0]` |
| `des_dtheta_max` | `[0,0,3]` |
| `gait_period_time` | `0.5` |
| `gait_max_leg_angle` | `15` |
| `gait_max_stance_time` | `0.25` s |
| `gait_min_stance_time` | `0.1` s |
| `Swing_traj_height` | `0.07` |
| `Swing_step_offset` | `[0,0.05,-0.003]` |
| `gait_type` | `4` |
| `gait_switching_phase` | `0.5` |
| `gait_override` | `4` |

以上是配置中的期望约束，不等同于物理硬限位。

#### 力矩

- `RobotRunner.cpp` 调用 `setMaxTorqueCheetah3(208.5)`；
  该调用路径对 Mini Cheetah 也会执行，但对 Mini Cheetah 的实际生效
  情况**未确认**。
- `rt_spi.cpp` 的 `fake_spine_control` 路径使用
  `max_torque={17,17,26} Nm`；
  是否覆盖真实 SPI 控制路径**未确认**。

#### 控制增益与求解器

同一 YAML 中已确认的主要值：

- `Kp_joint=[3,3,3]`、`Kd_joint=[1,0.2,0.2]`；
- `Kp_body=[40,40,100]`、`Kd_body=[10,10,10]`；
- `Kp_foot=[300,300,300]`、`Kd_foot=[80,80,80]`；
- `Kp_ori=[80,80,100]`、`Kd_ori=[10,10,10]`；
- `Swing_Kp_cartesian=[350,350,75]`、
  `Swing_Kd_cartesian=[5.5,5.5,5.5]`、
  `Swing_Kd_joint=[0.2,0.2,0.2]`；
- `cmpc_gait=9`、`jcqp_max_iter=10000`、`use_jcqp=0`、
  `use_wbc=1`、`stance_legs=4`、`RPC_mu=0.5`。

`mini-cheetah-defaults.yaml` 中还确认：
`stand_kd_cartesian=[2.5,2.5,2.5]`、
`stand_kp_cartesian=[50,50,50]`、
`kpCOM=[50,50,50]`、`kdCOM=[10,10,10]`、
`kpBase=[300,200,100]`、`kdBase=[20,10,10]`。

#### 未发现或未确认的限制

- 在现有 `config/*.yaml` 与源码检索中**未发现**明确的关节速度限制键、
  电流硬限或对应数值。
- 只发现 `des_*_max`、`Swing_traj_height` 等期望约束，
  **未发现**可直接确认的物理关节位置、机身姿态或足端硬限位。
- TCP `mode`、`gait` 等外部字段与状态机的实际绑定关系为
  **待远端核验**。

### 5.5 超时与安全保护 — 部分已完成

- 急停条件已确认：`rc_control.mode==0 && use_rc` 时，
  四腿命令清零并调用 `Estop()`。
- 使能时序已确认：前 10 次控制循环禁用；第 20～30、40～50 次
  控制循环之间禁用，其余使能。
- 全源码关键词检索只命中 SBUS 读包重试和 TCP `SO_RCVTIMEO` 注释；
  `rt_socket.cpp` 中定义过 5 s 超时，但该设置已被注释。
- **未发现运动命令 watchdog、遥控失联急停、网络断流保护或
  手柄 deadman**。
- 过流、过温、过压、姿态/跌倒阈值、故障复位和 `rc.local`
  自动重启策略仍为 **待远端核验**。

### 5.6 遥控接口与映射 — 已完成，部分程序用途未确认

#### SBUS / Taranis / AT9S

- 真机 SBUS 串口：`/dev/ttyS4`。
- 仿真分支 SBUS 串口：`/dev/ttyUSB0`。
- 波特率 `100000`，读取周期 0.005 s（200 Hz）。
- 通道：`ch0..3` 为摇杆，`ch4..9` 为开关，`ch10/11` 为旋钮。
- Taranis 原始标定：
  - 左右摇杆 L LR：`372/1000/1752`；
  - L FB：`268/1000/1589`；
  - R FB：`411/1000/1721`；
  - R LR：`294/1012/1636`。
- AT9S 映射：
  - SWE 急停：上=`OFF`、中=恢复站立、下=运行；
  - SWA=`QP_STAND`/运动；
  - SWG=动作；
  - SWC/SWD 组合选择步态；
  - 低速步态编号 `9/3/6`，高速步态编号 `5/1/2`；
  - `v_scale=1.5`、`w_scale=3.0`、deadband `0.1`、
    `step_height=varB+1.0`。
- Taranis 模式：
  - SWE 上=`OFF`、中=恢复、下=运动；
  - SWA/SWC/SWD 组合选择 `QP_STAND` 与 9 种步态；
  - deadband `0.1`；
  - `v_scale=knobs[0]*1.5+2.0`（0.5～3.5）、
    `w_scale=2*v_scale`。
- SBUS 代码位于 `/home/user/robot-software/robot/src/rt/rt_sbus.cpp`
  与 `rt_rc_interface.cpp`。

#### Linux 手柄

- 设备：`/dev/input/js0`，轮询 0.01 s（100 Hz）。
- 当前只读盘点确认 **`/dev/input/js0` 不存在**。
- type1：button `0..5` 对应 A/B/X/Y/LB/RB。
- type2：axes `0/1/3/4` 为摇杆，`2/5` 为扳机，
  `6/7` 为方向键；原始值除以 `32767` 归一化。
- 收到 LCM `interface` 后，`joystickReceiveLock=0`，
  此后本地 `/dev/input/js0` 不再更新 `_gamepadCommand`。

#### 其他遥控程序与网络通道

- 全仓库检索**未发现键盘遥控源码或映射**。
- 文件盘点清单中出现过 `receive_gamepad` 名称，
  但接口调查**未找到其源码、LCM/UDP 通道或运行进程**；
  其用途为 **待远端核验**。
- `/home/user/robot-software/build/listener` 与
  `/home/user/robot-software/build/send-message` 是未 stripped ELF，
  当前未运行；只能确认二进制内含 `LCM`，源码、channel 和用途
  **待远端核验**。
- TCP `1986` 的命令字段已确认，但是否实际控制未确认；
  不能把它描述为有效遥控入口。

### 5.7 IMU、编码器、足端与底层 I/O — 已完成可确认项

| 接口/数据流 | 设备或通道 | 已确认参数 | 未确认项 |
| --- | --- | --- | --- |
| VectorNav IMU | `/dev/ttyS0`，LCM `hw_vectornav` | 输出 800/4=200 Hz，由 `mit_ctrl` 直接读取并回调写入 `_vectorNavData` | 型号、坐标系、单位和完整消息字段待远端核验 |
| Microstrain/Lord IMU | 串口索引 0，LCM `microstrain` | `tryInit(0,921600)` 或 `tryInit(0,460800)`；1 kHz 调用 `updateLCM` | 具体型号、实际选中的波特率分支待远端核验 |
| 关节 SPI | `/dev/spidev2.0`、`/dev/spidev2.1`，LCM `spi_data`/`spi_command` | 500 Hz，`SPI_MODE_0`、8 bit、6 MHz | 真机板卡固件和逐字段物理单位待远端核验 |
| 关节编码器/力矩 | SPI 数据 `q/qd/tau/flags` | 由 `mit_ctrl` 经 SPI 直接读取 | 12 轴到腿/关节的映射与故障 flags 语义待远端核验 |
| 状态估计 | VectorNav 姿态 + 关节/SPI 数据 | `mit_ctrl` 内部计算并发布 `state_estimator` | 噪声模型、零位与故障降级待远端核验 |
| 足端接触 | 无独立设备节点；`ContactEstimator` | 默认接触相位 `0.5` | 独立足端力传感器未确认；足端力日志代码被注释 |
| TCP 状态转发 | `ReplyMessage` | 50 ms 周期，由 `mit_ctrl` 内部线程发送 | 不是 ROS 或外部转发程序 |

底层设备与权限：

| 设备/接口 | 路径/节点 | 权限/状态 |
| --- | --- | --- |
| CH340 串口 | `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` -> `/dev/ttyUSB0` | `root:dialout 660` |
| VectorNav 串口 | `/dev/ttyS0` | `root:dialout 660` |
| SBUS 真机串口 | `/dev/ttyS4` | `root:dialout 660` |
| 仿真 SBUS 串口 | `/dev/ttyUSB0` | `root:dialout 660` |
| 关节 SPI | `/dev/spidev2.0`、`/dev/spidev2.1` | `root:root 600` |
| I2C | `/dev/i2c-0..6` | `root-only` |
| GPIO | `gpiochip0/318/373/397/456` | chip 已确认；line 权限与用途待远端核验 |
| Linux 手柄 | `/dev/input/js0` | 当前不存在 |

- `user` 属于 `user,adm,sudo,plugdev,...`，但**不在 `dialout`**；
  当前 `mit_ctrl` 以 root 运行，因此能访问上述设备。
- 已加载 `ch341`、`usbserial`、`ti_adc081c`、`industrialio`；
  但未发现 CAN 和 IIO 设备节点。
- T265 订阅和显示代码已被注释；源码检索未找到摄像头/OpenCV、
  ADC、GPIO、温度接口的活动实现。
- CH340 实际连接板卡、I2C 地址/芯片、GPIO line/方向、
  ADC 通道和数据频率均为 **待远端核验**。
- 编码器、足端力、摄像头和温度传感器是否存在独立物理通道，
  除上表已确认项外均为 **待远端核验**。

接口证据应保存至新备份时间戳目录的 `hardware_interfaces/`；
不得只保存结论而不保留原始命令输出和权限错误。

## 6. 目标目录结构

### 6.1 本次已创建结构 — 已完成

```text
/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/robot_backups/backup_2026-10-09_120612/
├── system/                 # 计划：整系统归档
├── home/                   # 计划：/home/user 及核心代码
├── etc/                    # 计划：/etc 与启动/服务配置
├── opt_var/                # 计划：/opt /srv /usr/local /var/lib 等
├── ros_workspace/          # 已确认远端无 ROS workspace；保留盘点结果
├── hardware_interfaces/    # 计划：运动、遥控、IMU、I/O 等接口清单
├── metadata/               # 计划：环境、包、服务、进程、网络、哈希清单
└── logs/                   # 计划：命令日志、错误日志、校验日志
```

除目录本身已创建外，上表中的“计划”内容**尚未复制**。

### 6.2 新备份命名规则

- 每个新备份必须使用新的 `backup_YYYY-MM-DD_HHMMSS/`。
- 不覆盖旧备份，不把不同轮次混入同一时间戳目录。
- 时间戳应在创建目录时由本地时钟生成，并在
  `metadata/` 中同时记录开始时间、结束时间和时区。
- 中断后的重新复制应另建时间戳，不把不完整旧目录标成成功备份。

---

## 7. 分层备份清单

| 层级 | 内容 | 计划目标 | 说明 | 状态 |
| --- | --- | --- | --- | --- |
| L0 元数据 | OS、内核、CPU、内存、磁盘、挂载、包、服务、进程、网络、硬件、权限错误 | `metadata/` | 不依赖大文件复制，先建立环境快照 | 计划 |
| L1 核心代码/配置 | `/home/user/robot-software` 的源码、`config`、`build/run_mc.sh`、`/etc/rc.local` | `home/`、`etc/` | 优先保障 MIT/Cheetah 可分析、可重建 | 计划 |
| L2 用户目录 | `/home/user` 完整副本 | `home/` | 包含 `robot-software`、其他程序与用户数据 | 计划 |
| L3 系统配置 | `/etc`、包清单、服务、进程、网络和硬件元数据 | `etc/`、`metadata/` | root 项需安全交互式 `sudo` | 计划 |
| L4 系统分区层 | `/opt`、`/srv`、`/usr/local`、`/var/lib`、`/usr`、`/lib`、`/boot` | `opt_var/`、`system/` | root 项需安全交互式 `sudo` | 计划 |
| L5 全系统 | 整系统流式 `tar + zstd` | `system/rootfs.tar.zst` | 保留属主、权限、ACL、xattr 等元数据 | 计划 |
| L6 恢复演练 | 解压抽样、清单比对、核心程序恢复检查 | `metadata/`、`logs/` | 不是“文件已复制”的替代品 | 待验证 |

### 7.1 核心优先级

1. `/home/user/robot-software` 的源码和 `config/`。
2. `/home/user/robot-software/build/run_mc.sh`、其他启动脚本和
   `/etc/rc.local`。
3. 三个已确认行走配置 YAML，以及运行进程和链接库清单。
4. `/home/user` 完整副本。
5. `/etc`、包/服务/进程/网络/硬件元数据。
6. 指定系统目录和整系统归档。

### 7.2 整系统归档原则

- 建议在远端以 `tar` 流式输出，经 SSH 交给本地 `zstd` 压缩为
  `tar.zst`，避免在仅有 `1.3GiB` 可用空间的机器人系统盘上生成大文件。
- 归档只排除伪文件系统 `/proc`、`/sys`、`/dev`、`/run`
  以及执行前明确列出并记录的临时目录；不得静默扩大排除范围。
- `/proc` 中的进程内存、`/sys`、`/dev`、`/run` 中的运行时对象
  不是可恢复的普通文件。
- Unix socket、进程内存和临时文件不适合直接复制；对整系统归档遇到的
  非普通文件必须记录为错误或不一致，不得静默忽略。
- 如果本地备份父目录所在文件系统为 NTFS，则该文件系统不适合作为
  带 Unix 权限的最终唯一副本。核心目录可额外解压为普通目录便于浏览，
  但整系统仍应保存 `tar.zst`，以归档方式保留属主、权限、ACL 与 xattr。
- NTFS 支持情况、目标挂载参数和权限映射为**待验证**，不能把“目录可创建”
  等同于“Unix 元数据可完整恢复”。

---

## 8. 执行步骤

以下命令只是执行计划，**本任务未运行其中任何 SSH、复制、归档或 `sudo` 命令**。
执行时不得把授权材料写入命令历史、日志、文档或清单。

### 8.1 预检与新时间戳

```bash
# 使用已核实的本地绝对路径；不得从相对路径推断备份位置
BACKUP_ROOT="/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/robot_backups"
BACKUP_DIR="${BACKUP_ROOT}/backup_$(date +%Y-%m-%d_%H%M%S)"

# 新轮次创建独立时间戳目录；中断重跑时不要复用旧目录
mkdir -p \
  "${BACKUP_DIR}/system" \
  "${BACKUP_DIR}/home" \
  "${BACKUP_DIR}/etc" \
  "${BACKUP_DIR}/opt_var" \
  "${BACKUP_DIR}/ros_workspace" \
  "${BACKUP_DIR}/hardware_interfaces" \
  "${BACKUP_DIR}/metadata" \
  "${BACKUP_DIR}/logs"

# 再次确认连通、默认路由和源/目标容量；WiFi 保持连接
ping -c 4 10.0.0.34
ip route
df -h /dev/nvme0n1p2
df -h /
```

如果继续使用已创建的
`/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/robot_backups/backup_2026-10-09_120612`，
应先确认它仍只有空目录、没有任何半成品复制；否则新建时间戳。

### 8.2 远端只读预检 — 计划

```bash
# 仅核验工具、身份、空间和文件数量；不写远端文件
ssh -o BatchMode=yes user@10.0.0.34 \
  'hostname; uname -a; df -h /; command -v rsync tar zstd; du -sh /home/user'
```

记录每条命令的退出码。`BatchMode` 若因未配置密钥而失败，
应由用户交互登录；不得用文档中的授权材料绕过。

### 8.3 L1 核心代码与配置 — 计划

```bash
# 远端 rsync 版本和能力尚未核验；确认后再执行
rsync -aHAXv --numeric-ids --partial --info=progress2 \
  user@10.0.0.34:/home/user/robot-software/robot/ \
  "${BACKUP_DIR}/home/user/robot-software/robot/"

rsync -aHAXv --numeric-ids --partial --info=progress2 \
  user@10.0.0.34:/home/user/robot-software/actor_model/ \
  "${BACKUP_DIR}/home/user/robot-software/actor_model/"

rsync -aHAXv --numeric-ids --partial --info=progress2 \
  user@10.0.0.34:/home/user/robot-software/config/ \
  "${BACKUP_DIR}/home/user/robot-software/config/"
```

`build/run_mc.sh` 与 `/etc/rc.local` 是否可由 `user` 读取为**待远端核验**。
如需 root，先进行交互式授权并把错误输出保存到
`"${BACKUP_DIR}/logs/"`，不得伪造成功。

### 8.4 L2 完整 `/home/user` — 计划

```bash
rsync -aHAXv --numeric-ids --partial --info=progress2 \
  --exclude='/proc/**' \
  --exclude='/sys/**' \
  --exclude='/dev/**' \
  --exclude='/run/**' \
  user@10.0.0.34:/home/user/ \
  "${BACKUP_DIR}/home/user/"
```

在复制前应确认远端没有把伪文件系统挂载到 `/home/user` 下；
如果实际挂载不同，必须先记录挂载表并调整排除项，不能机械套用。

### 8.5 L3 `/etc` 与环境元数据 — 计划

```bash
# 先尝试普通用户可读部分；不可读项必须记录，不能默认跳过
rsync -aHAXv --numeric-ids --partial --info=progress2 \
  user@10.0.0.34:/etc/ \
  "${BACKUP_DIR}/etc/"

# 需要 root 的残余项通过安全交互式 sudo 单独处理
# 该步骤不提供、不记录、不回显任何授权材料
```

元数据至少包括：

- `/etc/os-release`、内核、CPU、内存、磁盘与挂载；
- 包清单、服务单元、运行进程及启动链；
- IP 地址、路由、DNS、接口与监听端口；
- PCI、USB、串口、I2C、GPIO、CAN/IIO 探测结果；
- `du`、文件数、权限错误清单和工具版本。

### 8.6 L4 指定系统目录 — 计划

按 `/opt`、`/srv`、`/usr/local`、`/var/lib`、`/usr`、`/lib`、
`/boot` 分别记录源端 `du`、文件数和复制退出码，再复制到
`"${BACKUP_DIR}/opt_var/"` 或 `system/` 的明确子目录。
root 项只能通过安全交互式授权处理，不能因为权限错误而把结果标为成功。

### 8.7 L5 全系统流式归档 — 计划

```bash
# 模板：执行前先列出并记录临时目录排除项
# 该命令尚未执行；必须人工确认 SSH、sudo、tar 和管道退出码
ssh -o BatchMode=yes user@10.0.0.34 \
  'sudo -p "远端交互式授权: " tar \
    --numeric-owner --acls --xattrs \
    --exclude=/proc --exclude=/sys --exclude=/dev --exclude=/run \
    --exclude=/tmp --exclude=/var/tmp \
    -cpf - /' \
  | zstd -T0 -19 -o "${BACKUP_DIR}/system/rootfs.tar.zst"
```

正式执行前必须：

1. 确认远端 `tar` 与本地 `zstd` 版本和参数支持；
2. 确认 `/tmp`、`/var/tmp` 是本轮明确批准的临时目录排除项；
3. 开启 `pipefail` 或等效检查，避免只凭压缩端退出码判断成功；
4. 保留远端 `tar` 警告、文件变化与权限错误；
5. 不在机器人系统盘写入整盘临时归档。

### 8.8 热复制记录 — 计划

- 记录复制开始/结束时间、`mit_ctrl` 的 PID/命令行/进程启动时间，
  以及复制期间发生变化的文件。
- `mit_ctrl` 正在运行时，控制状态、日志、网络缓冲和部分文件可能变化；
  热复制只能标记为“热备份”，不能宣称为原子一致快照。
- 若用户后续明确批准停机窗口，应另行制定停机归档步骤；
  本轮不自动重启、关机或停止控制进程。

---

## 9. 校验与恢复

### 9.1 容量与文件数 — 计划

每个层级都要同时记录：

1. 远端源目录 `du -sh` 与文件数；
2. 本地目标目录 `du -sh` 与文件数；
3. 复制命令退出码；
4. 权限错误、缺失文件和符号链接目标错误。

```bash
# 文件数在复制完成后于本地生成；变量来自第 8.1 节
find "${BACKUP_DIR}" -xdev -type f | wc -l
du -sh "${BACKUP_DIR}"
```

源端与目标端文件数可能因权限、伪文件系统和特殊文件而不同；
差异必须逐项解释，不得只比较总容量。

### 9.2 SHA-256 清单 — 计划

```bash
# 生成本地普通文件哈希清单
(
  cd "${BACKUP_DIR}" &&
  find . -xdev -type f -print0 |
    LC_ALL=C sort -z |
    xargs -0 -r sha256sum
) > "${BACKUP_DIR}/metadata/sha256-local.txt"
```

- 对远端可读范围生成对应的 `sha256sum` 清单。
- 核对远端与本地清单时，要单列因 root 权限而无法读取的路径。
- 不能把只对本地可读文件生成的清单描述为“全系统已校验”。

### 9.3 归档完整性 — 计划

```bash
# zstd 压缩流完整性
zstd -t "${BACKUP_DIR}/system/rootfs.tar.zst"

# tar 成员可读性检查；不解压到生产路径
zstd -dc "${BACKUP_DIR}/system/rootfs.tar.zst" |
  tar -tf - >/dev/null
```

两项均应为退出码 `0`，并保留 stderr。随后将归档成员数、
远端普通文件数、权限错误和排除目录清单写入
`metadata/archive-verification.txt`。

### 9.4 热复制一致性 — 待验证

- 比较复制前后 `mit_ctrl` 配置和关键文件哈希；
- 记录复制过程中被修改或消失的文件；
- 对日志、PID 文件、socket、临时文件单列风险；
- 哈希一致只能说明被检查文件一致，不能证明运行时内存、
  网络状态或跨文件事务一致。

### 9.5 恢复验收 — 待验证

1. 在隔离、非生产恢复目录或备用磁盘上先做只读解压演练。
2. 检查目录属主、组、权限、ACL、xattr、硬链接和符号链接。
3. 核对三个行走 YAML、`run_mc.sh`、`rc.local`、`mit_ctrl`
   及已记录二进制库是否存在且哈希匹配。
4. 恢复 `/etc` 时先比较差异，禁止直接覆盖运行中的机器人。
5. 整系统恢复只在独立环境验证；任何机器人重启或生产覆盖必须由用户明确批准。
6. 恢复演练日志与失败项必须写入 `logs/` 和 `metadata/`。

---

## 10. 敏感信息与安全

1. **绝不把 SSH 登录授权材料或 sudo 授权材料写入本文档、
   命令、脚本、日志、清单或 Git。**
2. 备份可能包含 `/etc/shadow`、SSH 私钥、服务凭据、Token、
   网络配置和用户数据；本地备份目录应按敏感数据对待。
3. 任何包含上述内容的备份都禁止提交到 Git，也不得放入公开日志。
4. root 内容只能通过安全交互式授权读取；失败时记录“权限缺失”，
   不得降级为“备份成功”。
5. 不关闭 WiFi。WiFi 负责外网连接，网线负责机器人连接，两者并行保留。
6. 不执行破坏性磁盘操作，不按设备编号猜测目标磁盘。
7. 不自动执行 Git `add`、`commit`、`push`、`merge` 或 `rebase`。

---

## 11. 限制与待办

### 11.1 当前限制

- **当前尚未执行实际文件复制**；本目录目前只有已创建的结构。
- 当前没有本地 `du`、文件数、SHA-256、`zstd -t` 或 `tar -tf`
  的验收结果，因为复制和归档尚未开始。
- 热复制运行中的 `mit_ctrl` 可能不一致，无法提供原子快照保证。
- `/root`、`/etc/shadow` 和部分 `/etc`、`/var/lib` 需要 root；
  当前只读盘点存在明确不可读项。
- `/proc`、`/sys`、`/dev`、`/run`、Unix socket、进程内存和
  临时文件不适合作为普通文件直接复制。
- `/home/user/robot-software` 没有 Git 元数据，无法验证源码版本历史。
- 机器人系统盘仅剩 `1.3GiB`，不能在源盘生成整盘未压缩副本。
- `ros_workspace/` 是为盘点结论保留的目录；当前没有发现 ROS workspace。
- NTFS 是否为备份父目录所在文件系统及其 Unix 元数据兼容性为待验证。

### 11.2 待办

- [ ] 继续补全第 5 节明确标出的“待远端核验”字段：状态迁移、
      LCM 参数覆盖、TCP 生效路径、物理硬限、失联保护、
      `receive_gamepad`/`listener`/`send-message` 用途、
      CH340/I2C/GPIO/ADC 具体数据通道，以及独立足端与温度传感器证据。
- [ ] 确认远端 `rsync`、`tar`、`zstd` 能力与交互式 root 读取范围。
- [ ] 按 L0-L5 分层执行复制，并为每个新轮次创建新时间戳目录。
- [ ] 生成远端/本地 `du`、文件数和 SHA-256 清单。
- [ ] 完成 `zstd -t`、`tar -tf`、成员数和权限元数据检查。
- [ ] 记录热复制期间的文件变化与 `mit_ctrl` 运行状态。
- [ ] 在隔离环境完成核心目录与整系统恢复演练。
- [ ] 将所有失败、缺失权限和未覆盖路径写入 `logs/` 与 `metadata/`。

---

## 12. 结论

本方案已完成网络连通、软硬件只读盘点、本地工具与容量确认，以及本次独立
时间戳目录和八个子目录的创建。当前最重要的状态边界是：

> **当前尚未执行实际文件复制；所有分层复制、哈希、归档完整性、
> 接口远端核验与恢复演练均仍处于计划或待验证状态。**

后续执行必须继续使用新的 `backup_YYYY-MM-DD_HHMMSS/` 隔离每一轮备份，
保留源端与目标端证据，绝不把权限缺失、热复制或未校验结果描述为成功。
