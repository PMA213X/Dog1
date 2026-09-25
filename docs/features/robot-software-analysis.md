# YoboGo-control Robot-Software 代码结构分析报告

> 分析日期：2026年  
> 源码路径：`YoboGo-control/robot-software/`

---

## 1. 完整目录树

```
robot-software/
├── actor_model/                          # 强化学习Actor模型数据
│   ├── actor_5000.txt                    # Actor网络权重(5000步)
│   ├── estimator_5000.txt                # Estimator网络权重(5000步)
│   ├── mean5000.csv                      # 均值归一化数据
│   └── var5000.csv                       # 方差归一化数据
│
├── config/                               # 配置文件目录
│   ├── c3-jpos-user-parameters.yaml      # Cheetah 3关节位置用户参数
│   ├── cheetah-3-defaults.yaml           # Cheetah 3默认机器人参数
│   ├── default-terrain.yaml              # 仿真环境地形配置(平面、斜坡、台阶等)
│   ├── default-user-parameters-file.yaml # 用户参数文件索引
│   ├── default-user.yaml                 # 默认用户参数
│   ├── front_jump_data.dat               # 前跳动作轨迹数据
│   ├── front_jump_extend_legs.dat        # 前跳-伸腿数据
│   ├── front_jump_pitchup.dat            # 前跳-抬头数据
│   ├── front_jump_pitchup_v2.dat         # 前跳-抬头v2
│   ├── front_jump_pitchup_v2_too_much.dat
│   ├── front_jump_v3.dat                 # 前跳v3
│   ├── front_jump_v4.dat                 # 前跳v4
│   ├── front_jump_v5.dat                 # 前跳v5
│   ├── heightmap.txt                     # 高度图
│   ├── heightmap_demo.txt                # 演示高度图
│   ├── initial_jpos_ctrl.yaml            # 初始关节位置控制参数
│   ├── macroConfig.h                     # 宏定义配置(控制LCM发布开关)
│   ├── mc-mit-ctrl-user-parameters.yaml  # Mini Cheetah MIT控制器用户参数(核心)
│   ├── mc_flip.dat                       # 翻转动作数据
│   ├── mini-cheetah-defaults.yaml        # Mini Cheetah默认机器人参数
│   ├── no-parameters.yaml                # 空参数文件
│   ├── simulator-defaults.yaml           # 仿真器默认参数
│   └── test-yaml.yaml                    # 测试YAML
│
├── config_network_lcm.sh                 # 网络/LCM多播配置脚本
│
└── robot/                                # 核心机器人软件
    ├── CMakeLists.txt                    # CMake构建文件
    │
    ├── include/                          # 头文件
    │   ├── HardwareBridge.h              # 硬件桥接接口(Mini Cheetah / Cheetah 3)
    │   ├── JPosInitializer.h             # 关节位置初始化控制器
    │   ├── main_helper.h                 # 主函数辅助入口
    │   ├── RobotController.h             # 机器人控制器基类(抽象接口)
    │   ├── RobotRunner.h                 # 机器人运行框架(控制循环核心)
    │   ├── SimulationBridge.h            # 仿真桥接接口
    │   └── rt/                           # 实时硬件接口
    │       ├── rt_ethercat.h             # EtherCAT通信接口(Cheetah 3)
    │       ├── rt_rc_interface.h         # 遥控器(RC)控制接口
    │       ├── rt_sbus.h                 # SBUS协议遥控器通信
    │       ├── rt_serial.h               # 串口通信
    │       ├── rt_socket.h               # TCP Socket通信
    │       ├── rt_spi.h                  # SPI通信(Spine Board)
    │       └── rt_vectornav.h            # VectorNav IMU通信
    │
    └── src/                              # 源文件
        ├── HardwareBridge.cpp            # 硬件桥接实现
        ├── JPosInitializer.cpp           # 关节位置初始化实现
        ├── main_helper.cpp               # 主函数入口实现
        ├── RobotRunner.cpp               # 控制循环主框架实现
        ├── SimulationBridge.cpp          # 仿真桥接实现
        └── rt/                           # 实时接口实现
            ├── rt_ethercat.cpp           # EtherCAT (SOEM)通信实现
            ├── rt_rc_interface.cpp       # 遥控器控制逻辑实现
            ├── rt_sbus.cpp               # SBUS协议解码实现
            ├── rt_serial.cpp             # 串口配置实现
            ├── rt_socket.cpp             # TCP Socket服务器实现
            ├── rt_spi.cpp                # SPI驱动实现
            └── rt_vectornav.cpp          # VectorNav IMU驱动实现
```

---

## 2. 关键文件功能说明

### 2.1 构建系统

#### `robot/CMakeLists.txt`
- **功能**：CMake 构建配置文件
- **构建产物**：`robot` 共享库 (`add_library(robot SHARED ...)`)
- **包含路径**：
  - 项目自身头文件 (`include/`)
  - 公共库 (`common/include/`)
  - 第三方库：ParamHandler、VectorNav SDK、SOEM(EtherCAT)、Lord IMU
  - LCM类型定义 (`lcm-types/cpp`)
  - 系统LCM (`/usr/local/include/lcm/`)
- **链接库**：
  - `biomimetics` — 机器人动力学核心库
  - `pthread` — POSIX线程
  - `lcm` — Lightweight Communications and Marshalling
  - `inih` — INI文件解析
  - `dynacore_param_handler` — 动态参数处理
  - `lord_imu` — Lord MicroStrain IMU驱动
  - `soem` — Simple Open EtherCAT Master
  - `libvnc`、`rt` — Linux实时扩展

### 2.2 主入口

#### `robot/src/main_helper.cpp` + `robot/include/main_helper.h`
- **功能**：机器人程序的主入口函数
- **命令行格式**：`robot [robot-id] [sim-or-robot] [parameters-from-file]`
  - `robot-id`：`3` = Cheetah 3，`m` = Mini Cheetah
  - `sim-or-robot`：`s` = 仿真模式，`r` = 真实硬件模式
  - `param-file`：`f` = 从文件加载参数，不指定 = 通过LCM从网络加载
- **关键逻辑**：
  - 根据参数选择 `SimulationBridge`（仿真）或 `HardwareBridge`（硬件）
  - Mini Cheetah 硬件模式启动时会执行 `/home/user/music/script/start_music.sh`（启动音乐）
  - 根据机器人类型创建对应的 `MiniCheetahHardwareBridge` 或 `Cheetah3HardwareBridge`

### 2.3 硬件桥接

#### `robot/include/HardwareBridge.h` + `robot/src/HardwareBridge.cpp`
- **功能**：机器人代码与硬件之间的接口层
- **三个核心类**：
  1. **`HardwareBridge`**（基类）
     - 初始化实时调度器 (`SCHED_FIFO`, 优先级49)
     - 锁定内存防止页错误 (`mlockall`)
     - 处理LCM接口消息（手柄、参数设置）
     - SBUS遥控器数据接收
     - USB手柄数据接收 (`/dev/input/js0`)
     - 可视化数据发布
  2. **`MiniCheetahHardwareBridge`**（Mini Cheetah专用）
     - SPI通信（2ms/500Hz周期）
     - Lord MicroStrain IMU（串口0, 460800/921600波特率）
     - VectorNav IMU初始化
     - 参数加载（从文件或LCM网络）
     - TCP线程初始化
     - USB手柄遥控器接口
  3. **`Cheetah3HardwareBridge`**（Cheetah 3专用）
     - EtherCAT通信（1ms/1000Hz周期）
     - VectorNav IMU
     - SBUS遥控器

#### `robot/include/SimulationBridge.h` + `robot/src/SimulationBridge.cpp`
- **功能**：仿真环境桥接接口
- 通过共享内存与仿真器通信 (`DEVELOPMENT_SIMULATOR_SHARED_MEMORY_NAME`)
- 支持SBUS遥控器在仿真中使用
- 也初始化TCP线程

### 2.4 控制框架

#### `robot/include/RobotRunner.h` + `robot/src/RobotRunner.cpp`
- **功能**：机器人控制循环的核心框架
- **控制循环周期**：2ms (500Hz)
- **核心流程**：
  1. 状态估计器运行 (`_stateEstimator->run()`)
  2. 将位置、姿态、关节角度、速度等数据填入TCP回复消息 (`replymessage`)
  3. 腿部控制器更新 (`setupStep`)
  4. E-Stop检测与安全保护
  5. 用户控制器运行 (`_robot_ctrl->runController()`)
  6. 可视化数据更新
  7. 命令下发 (`finalizeStep`) — 通过SPI(Mini Cheetah)或EtherCAT(Cheetah 3)
  8. LCM数据发布（腿控制命令、腿数据、状态估计）
- **初始化的组件**：
  - 四足机器人模型 (`Quadruped<float>`)
  - 浮基动力学模型 (`FloatingBaseModel<float>`)
  - 腿控制器 (`LegController<float>`)
  - 状态估计容器 (`StateEstimatorContainer<float>`)
  - T265状态估计器（Intel RealSense T265视觉里程计）
  - 期望状态命令 (`DesiredStateCommand<float>`)
  - 关节位置初始化器 (`JPosInitializer<float>`)

#### `robot/include/RobotController.h`
- **功能**：用户控制器的抽象基类接口
- **纯虚函数**：
  - `initializeController()` — 初始化控制器
  - `runController()` — 每个控制周期调用
  - `updateVisualization()` — 更新可视化数据
  - `getUserControlParameters()` — 获取用户控制参数
- **保护成员**：四足模型、腿控制器、状态估计器、T265估计器、驱动命令、控制参数等

### 2.5 通信接口

#### SPI 通信 (`rt_spi.h/cpp`)
- **功能**：与Spine Board（脊柱板/电机驱动板）通信
- **设备**：`/dev/spidev2.0`, `/dev/spidev2.1`
- **模式**：SPI_MODE_0, 8bit, 6MHz
- **数据结构**：
  - `spine_cmd_t`：发送给电机的指令（关节位置、速度、力矩前馈、增益）
  - `spine_data_t`：从电机接收的数据（实际关节位置、速度、力矩）
- **处理**：每条SPI消息包含2条腿的数据（共2个SPI板，4条腿）
- **特殊处理**：关节偏移量、正负号映射（左右腿差异）

#### EtherCAT 通信 (`rt_ethercat.h/cpp`)
- **功能**：Cheetah 3 与TI关节控制板的工业以太网通信
- **网络适配器**：`enp2s0`
- **使用SOEM库**（Simple Open EtherCAT Master）
- **从站数量**：4个（对应4条腿）
- **运行周期**：1ms (1000Hz)
- **通信内容**：
  - 发送：位置/速度目标、PD增益、力矩前馈、使能信号
  - 接收：实际位置/速度/力矩、控制板状态

#### TCP Socket 通信 (`rt_socket.h/cpp`)
- **功能**：与上位机/外部客户端的TCP通信
- **端口**：1986 (`SERV_PORT`)
- **模式**：TCP服务器模式，机器人作为服务端
- **两个线程**：
  1. `TCPThread` — 接收客户端指令
  2. `TCPSendThread` — 以50Hz (20ms)周期发送机器人状态数据
- **数据帧格式**：
  - **接收** (`Client_Command_Message`)：roll/pitch/yaw、速度、模式、步态高度、步态类型等
  - **发送** (`ReplyMessage`)：位置(xyz)、姿态(rpy)、速度、关节角度(4腿x3关节)、模式、步态
  - 帧头：`0xAAAAAAAA`，帧尾：`0xFFFFFFFF`

#### LCM 通信 (Lightweight Communications and Marshalling)
- **功能**：进程间通信框架，用于多模块间数据传递
- **多播地址配置**：通过 `getLcmUrl(255)` 获取（TTL=255）
- **LCM通道**：
  | 通道名 | 方向 | 内容 |
  |--------|------|------|
  | `interface` | 接收 | 手柄游戏控制命令 |
  | `interface_request` | 接收 | 控制参数设置请求 |
  | `interface_response` | 发送 | 控制参数设置响应 |
  | `spi_data` | 发送 | SPI电机数据 |
  | `spi_command` | 发送 | SPI电机命令 |
  | `microstrain` | 发送 | Lord IMU数据 |
  | `hw_vectornav` | 发送 | VectorNav IMU数据 |
  | `main_cheetah_visualization` | 发送 | 可视化数据 |
  | `leg_control_command` | 发送 | 腿控制命令 |
  | `leg_control_data` | 发送 | 腿传感器数据 |
  | `state_estimator` | 发送 | 状态估计数据 |
  | `ecat_cmd` | 发送 | EtherCAT命令(Cheetah 3) |
  | `ecat_data` | 发送 | EtherCAT数据(Cheetah 3) |

#### SBUS 遥控器通信 (`rt_sbus.h/cpp`)
- **功能**：Futaba/乐迪 AT9S 遥控器SBUS协议通信
- **串口设备**：
  - 仿真模式：`/dev/ttyUSB0`
  - 真实机器人：`/dev/ttyS4`
- **波特率**：100000 (SBUS标准)
- **支持的遥控器**：
  - Taranis X7
  - AT9S（乐迪）
- **遥控器功能映射**：
  - SWE三档开关：OFF / 恢复站立 / 运行
  - SWA开关：QP站立 / 运动模式
  - SWC/SWD开关组合：选择6种步态（trot、slow trot、walk、flying trot、bound、pronk）
  - SWG三档开关：正常运动 / 翻转动作 / 表演模式
  - 摇杆：速度、转向、姿态控制

#### USB 手柄通信
- **设备**：`/dev/input/js0`
- **映射**：A/B/X/Y按钮、左右摇杆、扳机键、肩键、方向键
- **优先级**：当LCM手柄数据未接收时，读取USB手柄数据

#### 串口通信 (`rt_serial.h/cpp`)
- **功能**：串口配置工具，为SBUS和其他串口设备提供波特率配置
- **支持自定义波特率**（通过 `BOTHER` 标志）

### 2.6 传感器接口

#### VectorNav IMU (`rt_vectornav.h/cpp`)
- **设备**：VectorNav VN-100/110 IMU
- **串口**：`/dev/ttyS0`
- **波特率**：115200
- **输出频率**：200Hz（800Hz/4除频）
- **数据**：四元数、角速度、加速度
- **模式**：相对航向模式

#### Lord MicroStrain IMU
- **驱动库**：`lord_imu`
- **串口**：`/dev/ttyS0`（端口0）
- **波特率**：460800（默认）或 921600（lordIMU宏）
- **运行方式**：独立线程持续读取
- **数据**：四元数、角速度、加速度
- **优先级**：`#define USE_MICROSTRAIN` 开启时替代VectorNav

#### T265 视觉里程计
- **设备**：Intel RealSense T265
- **用途**：视觉定位辅助（代码中有注释掉的T265 LCM接口）
- **状态估计器**：`_t265stateEstimator` 作为独立的状态估计容器

---

## 3. 所有IP地址和网络配置

### 3.1 硬编码IP地址

| IP地址 | 所在文件 | 用途 |
|--------|----------|------|
| `10.0.0.30` | `track1.6/track/colorgroup.cpp` (`goalIp`) | 视觉追踪目标机器人IP |
| `10.0.0.34` | `track1.6/track/colorgroup.cpp` (`nativeIp`) | 本机IP |
| `10.0.0.34` | `track1.6/send.sh` | SCP传输目标IP |

### 3.2 网络配置

| 配置项 | 值 | 所在文件 |
|--------|-----|----------|
| TCP服务器端口 | `1986` | `rt_socket.h` (`SERV_PORT`) |
| LCM多播TTL | `255` | 多处 `getLcmUrl(255)` |
| LCM多播网段 | `224.0.0.0/4` | `config_network_lcm.sh` |
| 注释掉的路由 | `10.0.0.0/8` | `config_network_lcm.sh` (第21行) |

### 3.3 网络接口名称

| 名称 | 描述 | 用于 |
|------|------|------|
| `enp0s25` | Thinkpad有线网卡 | LCM配置 |
| `enxa0cec808fb18` | Cynergy USB网卡 | LCM配置 |
| `wlp2s0` | XPS WiFi | LCM配置 |
| `enp1s0` | MC（Mini Cheetah机载电脑）有线网卡 | LCM配置 |
| `enxa0cec80424d3` | MC USB网卡 | LCM配置 |
| `enp2s0` | EtherCAT网络适配器 | EtherCAT通信 |
| `enx70886b885732` | dhkim网卡 | LCM配置 |
| `enx70886b887f40` | dhmac网卡 | LCM配置 |

### 3.4 SCP远程部署信息

```bash
# track1.6/send.sh 中的部署命令：
scp -r ./build/ user@10.0.0.34:/home/user/track2025
```
- **远程用户名**：`user`
- **远程IP**：`10.0.0.34`
- **远程路径**：`/home/user/track2025`

---

## 4. 编译和运行步骤

### 4.1 构建系统

本项目使用 CMake 构建。`robot-software/robot/CMakeLists.txt` 作为子目录构建文件，需要上级目录的 `CMakeLists.txt` 提供项目级配置。

**依赖库**：
- LCM (Lightweight Communications and Marshalling)
- SOEM (Simple Open EtherCAT Master)
- VectorNav SDK
- Lord IMU SDK
- ParamHandler
- Eigen3
- biomimetics（内部库）
- dynacore_param_handler（内部库）
- libvnc
- pthread
- inih

### 4.2 编译步骤

```bash
# 假设在项目根目录
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### 4.3 运行命令

#### Mini Cheetah 仿真模式
```bash
./robot m s
# robot-id: m = Mini Cheetah
# sim-or-robot: s = simulation
```

#### Mini Cheetah 真实硬件（参数从文件加载）
```bash
sudo ./robot m r f
# 需要root权限（mlockall + 实时调度）
# f = 从YAML文件加载参数
# 参数文件：config/mini-cheetah-defaults.yaml, config/mc-mit-ctrl-user-parameters.yaml
```

#### Mini Cheetah 真实硬件（参数从LCM网络加载）
```bash
sudo ./robot m r
# 通过LCM网络等待GUI界面发送参数
```

#### Cheetah 3 仿真模式
```bash
./robot 3 s
```

#### Cheetah 3 真实硬件
```bash
sudo ./robot 3 r
```

### 4.4 网络配置

在运行前需先配置LCM多播网络：

```bash
# 配置网络接口的多播支持
./config_network_lcm.sh mc        # Mini Cheetah机载电脑(enp1s0)
./config_network_lcm.sh mc-usb    # Mini Cheetah USB网卡
./config_network_lcm.sh mc-top    # Mini Cheetah顶部接口
./config_network_lcm.sh thinkpad  # Thinkpad笔记本
./config_network_lcm.sh xps-wifi  # XPS WiFi
./config_network_lcm.sh -I eth0   # 通用接口名

# 实际执行的命令（以mc为例）：
# sudo ifconfig enp1s0 multicast
# sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev enp1s0
```

### 4.5 远程部署（track1.6视觉模块）

```bash
cd track1.6
./send.sh
# 将编译好的track程序通过SCP部署到 10.0.0.34
```

---

## 5. 代码中提到的硬件接口汇总

### 5.1 通信接口

| 接口 | 协议/标准 | 设备文件 | 用途 | 机器人型号 |
|------|-----------|----------|------|-----------|
| SPI | Linux SPI (`spidev`) | `/dev/spidev2.0`, `/dev/spidev2.1` | 与Spine Board通信控制电机 | Mini Cheetah |
| EtherCAT | SOEM | `enp2s0` 网卡 | 与TI关节板通信 | Cheetah 3 |
| Serial (SBUS) | SBUS | `/dev/ttyS4` (机器人), `/dev/ttyUSB0` (仿真) | 遥控器通信 | 通用 |
| Serial (IMU) | UART | `/dev/ttyS0` | VectorNav IMU | 通用 |
| Serial (Lord) | UART | 端口0 | Lord MicroStrain IMU | Mini Cheetah |
| TCP Socket | TCP/IP | 端口1986 | 上位机/外部客户端通信 | 通用 |
| LCM | UDP多播 | 224.0.0.0/4 网段 | 模块间进程通信 | 通用 |
| USB Joystick | HID | `/dev/input/js0` | USB游戏手柄 | Mini Cheetah |

### 5.2 传感器

| 传感器 | 型号/类型 | 用途 |
|--------|-----------|------|
| IMU | VectorNav VN-100/110 | 姿态估计（四元数、角速度、加速度） |
| IMU | Lord MicroStrain (Microstrain) | 备用/主IMU（替代VectorNav） |
| 视觉里程计 | Intel RealSense T265 | 视觉定位辅助 |
| 足端力传感器 | 内置 | 地面接触力估计 |
| 关节编码器 | 绝对编码器 | 关节角度测量 |

### 5.3 执行器

| 执行器 | 详情 |
|--------|------|
| 电机驱动 | Mini Cheetah: 通过SPI与Spine Board通信 |
| 电机驱动 | Cheetah 3: 通过EtherCAT与TI控制板通信 |
| 关节结构 | 每腿3自由度：abad(侧摆)、hip(髋)、knee(膝) |
| 关节数量 | 4腿 x 3关节 = 12个电机 |

### 5.4 遥控设备

| 设备 | 型号 | 协议 |
|------|------|------|
| 遥控器 | 乐迪 AT9S | SBUS |
| 遥控器 | FrSky Taranis X7 | SBUS |
| USB手柄 | 通用游戏手柄 | Linux HID (`/dev/input/js0`) |

### 5.5 机载计算平台

| 平台 | 网络接口 | 说明 |
|------|----------|------|
| Mini Cheetah 机载电脑 | `enp1s0`, USB网卡 | 运行机器人控制软件，用户路径 `/home/user/` |

---

## 6. 控制模式说明

代码中定义了以下控制模式（`RC_mode`命名空间）：

| 模式ID | 名称 | 说明 |
|--------|------|------|
| 0 | OFF | 紧急停止 |
| 1 | STAND_UP | 站立 |
| 2 | READY | 准备 |
| 3 | QP_STAND | 准静态站立（可控姿态） |
| 4 | BACKFLIP_PRE | 后空翻准备 |
| 5 | BACKFLIP | 后空翻 |
| 6 | VISION | 视觉控制模式 |
| 11 | LOCOMOTION | 运动模式（主要模式） |
| 12 | RECOVERY_STAND | 恢复站立 |
| 13 | FRONT_JUMP | 前跳 |
| 14 | ROLL_OVER | 翻滚 |
| 15 | ZOOM_SHOW | 表演模式 |
| 16 | DANCE | 舞蹈 |
| 17 | TWIST | 扭转 |
| 18 | LANDING_BUFFER | 着陆缓冲 |
| 19 | STAND_DOWN | 蹲下 |
| 20/21 | TWO_LEG_STANCE | 双腿站立 |
| 31 | WAIT | 等待 |
| 53 | RL_JOINT_PD | 强化学习关节PD控制 |

### 步态类型（通过遥控器拨码选择）

| 步态ID | 名称 | 描述 |
|--------|------|------|
| 1 | Bound | 疾驰 |
| 2 | Pronk | 蹦跳 |
| 3 | Slow Trot | 慢速对角步态 |
| 5 | Flying Trot | 飞奔对角步态 |
| 6 | Walk | 行走 |
| 9 | Trot | 标准对角步态 |

---

## 7. 系统架构总结

```
┌─────────────────────────────────────────────────────┐
│                    上位机/客户端                       │
│     (TCP端口1986接收状态，发送控制指令)                  │
└──────────────────────┬──────────────────────────────┘
                       │ TCP Socket
                       ▼
┌─────────────────────────────────────────────────────┐
│              rt_socket (TCP Server)                  │
│    Client_Command_Message ←→ ReplyMessage             │
└──────────────────────┬──────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────┐
│               main_helper → HardwareBridge           │
│                    │                                  │
│    ┌───────────────┼───────────────┐                  │
│    │               │               │                  │
│    ▼               ▼               ▼                  │
│  RobotRunner   rt_rc_interface  rt_sbus/Joystick      │
│  (控制循环)     (遥控器映射)    (遥控器数据采集)        │
│    │                                                  │
│    ├── StateEstimator (状态估计)                       │
│    ├── LegController  (腿部控制)                       │
│    ├── DesiredStateCommand (期望状态)                  │
│    └── RobotController (用户控制算法)                   │
│         ├── MPC控制器                                  │
│         ├── WBC控制器                                  │
│         └── MIT控制器                                  │
└──────────────────────┬──────────────────────────────┘
                       │
          ┌────────────┼────────────┐
          ▼            ▼            ▼
    ┌──────────┐ ┌──────────┐ ┌──────────┐
    │ SPI驱动  │ │EtherCAT  │ │   LCM    │
    │(Mini Cheetah) │ (Cheetah 3) │ │(多播通信)│
    └────┬─────┘ └────┬─────┘ └──────────┘
         │            │
         ▼            ▼
    ┌──────────┐ ┌──────────┐
    │Spine Board│ │TI关节板  │
    │ (4条腿)  │ │ (4条腿)  │
    └──────────┘ └──────────┘
```

---

## 8. 重要配置参数

### Mini Cheetah 默认参数 (`mini-cheetah-defaults.yaml`)
- 控制周期：`0.002s` (500Hz)
- 站立KP (笛卡尔)：`[50, 50, 50]`
- 站立KD (笛卡尔)：`[2.5, 2.5, 2.5]`
- COM控制增益：KP=`[50,50,50]`, KD=`[10,10,10]`
- 基座控制增益：KP=`[300,200,100]`, KD=`[20,10,10]`
- 使用遥控器：`1`（启用）

### MIT控制器用户参数 (`mc-mit-ctrl-user-parameters.yaml`)
- 关节Kp：`[3, 3, 3]`
- 关节Kd：`[1, 0.2, 0.2]`
- 机身Kp：`[40, 40, 100]`，Kd：`[10, 10, 10]`
- 足端Kp：`[300, 300, 300]`，Kd：`[80, 80, 80]`
- 姿态Kp：`[80, 80, 100]`，Kd：`[10, 10, 10]`
- 摆动腿轨迹高度：`0.07m`
- 使用WBC：`1`（启用）
- 默认步态：`9`（Trot）
- 期望高度：`0.26m`

### 仿真器参数 (`simulator-defaults.yaml`)
- 动力学周期：`0.001s` (1000Hz)
- 高层控制周期：`0.002s` (500Hz)
- 低层控制周期：`0.0002s` (5000Hz)

---

*报告完成*
