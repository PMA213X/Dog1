# YoboGo-10S 官方使用说明书 — 完整摘要报告

> 来源：`YoboGo-10S使用说明书(开源).docx`
> 提取时间：2026-07-22

---

## 目录

1. [机器人基本信息](#1-机器人基本信息)
2. [物理连接方式（重点）](#2-物理连接方式重点)
3. [开机步骤（详细）](#3-开机步骤详细)
4. [网络配置](#4-网络配置)
5. [UP Board 相关信息](#5-up-board-相关信息)
6. [遥控器使用方法](#6-遥控器使用方法)
7. [软件运行方法](#7-软件运行方法)
8. [硬件接口和端口](#8-硬件接口和端口)
9. [所有命令汇总](#9-所有命令汇总)
10. [常见问题和故障排除](#10-常见问题和故障排除)
11. [质量保证与售后](#11-质量保证与售后)

---

## 1. 机器人基本信息

| 项目 | 参数 |
|------|------|
| 型号 | YoboGo |
| 尺寸 | 约 485×275×300mm |
| 重量 | 约 10.5 kg |
| 自由度 | 12（每腿3个自由度） |
| 本体结构 | 准直驱行星减速结构 |
| 轴承 | 普通深沟球轴承 |
| 平台 | Linux |
| 编程语言 | C/C++ |
| 保护模式 | 过流保护、急停保护、过热保护 |
| 材质 | 铝合金 |

**产品特点**：轻量型、高动态性能四足机器人，配备步态规划、腿足控制、环境感知技术。硬件结构、顶层软件、底层软件和原理图全部开源。

---

## 2. 物理连接方式（重点）

### 2.1 以太网连接（主要连接方式）

这是与机器人通信的**主要方式**：

```
你的电脑（PC）  <--网线-->  YoboGo 的 UP Board
IP: 10.0.0.x               IP: 10.0.0.34
子网掩码: 255.255.255.0     子网掩码: 255.255.255.0
网关: 10.0.0.1              网关: 10.0.0.1
```

**操作步骤**：
1. 用网线将你的电脑直接连接到 YoboGo 的 UP Board 网口
2. 在电脑上配置有线网络（Network Connections → 新建 Ethernet 连接）：
   - Address: `10.0.0.3`（或同网段任意地址，如 10.0.0.100）
   - Netmask: `255.255.255.0`
   - Gateway: `10.0.0.1`
3. 配置完成后即可通过 SSH 连接到 UP Board

### 2.2 SSH 连接

```bash
ssh user@10.0.0.34
# 密码：123456
```

### 2.3 USB/串口连接（电机调试用）

- 电机调试通过串口连接，波特率设置为 **921600**
- 输入输出设置为 **ASCII**
- 用于电机配置（编码器校准、零位设置等）

### 2.4 WiFi

说明书中**未提及 WiFi 连接方式**，机器人主要依赖以太网通信。

---

## 3. 开机步骤（详细）

### 3.1 控制器（遥控器）初始位置

拨钮初始状态：
- **拨钮 B、C、D** → 处于**上方**
- **拨钮 A、E、F、H** → 处于**下方**
- **拨钮 G** → 处于**中间**

### 3.2 启动步骤

1. **摆放机器人到初始位置**
   - 将 YoboGo 摆动到初始位置（腿折叠姿态）
   - ⚠️ **必须确认线没有缠绕**，不会干涉启动
   - 错误示例可能导致异常

2. **按下两个电源按钮**
   - 左边按钮 = **电机开关**（控制电机和电机控制板供电）
   - 右边按钮 = **总开关**（控制所有部件供电）
   - 只关闭电机开关 → 电机断电，但控制板仍运行
   - 关闭总开关 → 所有部件断电

3. **等待 1-2 分钟**
   - YoboGo 自动上电并进入 **passive 状态**（电机输出力为 0）

4. **站立（拨钮 E → 中间）**
   - 向上拨动拨钮 E 至中间
   - 等待 YoboGo 自动进入站立姿态

5. **平衡站立（拨钮 E → 最上）**
   - 向上拨动拨钮 E 至最上
   - YoboGo 处于平衡站立状态
   - 推动摇杆可执行姿态展示功能

6. **进入运动模式（拨钮 A → 向上）**
   - 向上拨动拨钮 A
   - 机器人进入运动状态
   - 拨动按钮 C 和 D 可切换步态

### 3.3 关机 / 紧急操作

| 操作 | 方法 |
|------|------|
| 退出运动模式 | 向下拨动拨钮 A |
| 紧急停止（急停） | 将拨钮 E 拨到**最下方**（passive 模式，电机输出力为 0） |
| 恢复站立模式 | 拨动拨钮 E 至中间 |
| 关闭电源 | 先将 E 键拨到最下方 → 再关闭电源开关 |
| 突发情况/起火 | E 键拨到最下方 → 关闭电源 → 灭火 |

> ⚠️ 在突发状况下，**切勿直接接触 YoboGo**，应先将 E 键拨到最下方再来关闭电源。

---

## 4. 网络配置

### 4.1 IP 地址信息

| 设备 | IP 地址 | 子网掩码 | 网关 |
|------|---------|----------|------|
| YoboGo（UP Board） | `10.0.0.34` | `255.255.255.0` | `10.0.0.1` |
| 客户端（你的电脑） | `10.0.0.x`（x≠34） | `255.255.255.0` | `10.0.0.1` |

### 4.2 通信协议

- 机器人内部使用 **LCM（Lightweight Communications and Marshalling）** 通信
- LCM 基于 **UDP** 协议进行双端通信
- 两端主机 IP 必须在同一网段下
- CAN 通信波特率：**1M**，标准帧模式

### 4.3 电脑端网络配置步骤

1. 打开 Network Connections
2. 添加一个新的 Ethernet 连接
3. 设置：
   - Address: `10.0.0.3`
   - Netmask: `255.255.255.0`
   - Gateway: `10.0.0.1`

---

## 5. UP Board 相关信息

### 5.1 基本情况

- UP Board 安装在 YoboGo 上，是**算法的运算载体**
- 出厂时**已配置好**
- 运行 **Ubuntu** 系统，使用 **RT（实时）内核**

### 5.2 关键接口

- **SPI 接口**：应有两个（`ls /dev/spidev*` 验证）
- **以太网口**：用于与外部电脑通信
- **USB 口**：用于串口调试电机等

### 5.3 SSH 登录

```bash
ssh user@10.0.0.34
# 密码：123456
```

### 5.4 程序自动启动

- YoboGo 有**程序自动启动功能**
- 在发送新代码前，需要先停止正在运行的程序

### 5.5 UP Board 环境配置（如需自行配置）

> ⚠️ 自行配置需具备较高开发与调试能力，由此产生的问题不在售后服务范围内。

#### 编译内核步骤

```bash
# 1. 更新库
sudo apt-get update
sudo apt-get upgrade

# 2. 安装依赖
sudo apt-get install libncurses5-dev
sudo apt-get install libssl-dev

# 3. 下载配置好的内核压缩包（U盘中），解压后
cp .config kernel
cd kernel
make menuconfig  # 不修改任何东西，save 保持默认选项后退出

# 4. 编译安装内核（过程较长）
make -j4
sudo make modules_install -j4
sudo make install -j4
sudo update-grub

# 5. reboot，在 Ubuntu 启动时选择 advanced option → RT 内核

# 6. 验证内核
uname -a  # 确认是否为 RT 内核

# 7. 修改 grub 默认启动顺序
sudo gedit /etc/default/grub
# 将 GRUB_DEFAULT 改为：GRUB_DEFAULT = "1> 6"
sudo update-grub
reboot

# 8. 验证 SPI 接口
ls /dev/spidev*  # 应该有两个
```

---

## 6. 遥控器使用方法

### 6.1 兼容型号

- **Logitech F710**（用户可自行购买）

### 6.2 控制器各部件功能

| 编号 | 部件 | 功能 |
|------|------|------|
| 1 | 摇杆 | 控制机器人姿态/运动方向 |
| 2 | SwA 二段开关 | 运动模式开关（上=运动，下=停止运动） |
| 3 | SwB 二段开关 | 预留功能 |
| 4 | SwC 三段开关 | 与 SwD 共同决定步态类型 |
| 5 | SwD 二段开关 | 与 SwC 共同决定步态类型 |
| 6 | SwE 三段开关 | **核心安全开关**（下=passive/急停，中=站立，上=平衡站立） |
| 7 | SwF 二段开关 | 预留功能 |
| 8 | Vrb 旋钮开关 | 控制步高（范围 0-0.2m） |
| - | SwG | 中间位置 |
| - | SwH | 初始在下方 |

### 6.3 操作流程总结

```
开机 → E拨中间(站立) → E拨最上(平衡) → A拨上(运动) → C/D切换步态
         ↓                                    ↓
      随时可急停：E拨最下                 退出运动：A拨下
```

### 6.4 步态切换

通过 SwC（三段）和 SwD（二段）组合切换四种步态：
- **Trot（小跑）**
- **Flying-Trot（快跑）**
- **Walk（行走）**
- **Bound（跑跳）**

### 6.5 运动能力

- 抗外力扰动平衡控制
- 平整路面：Trot、Flying-Trot、Walk、Bound
- 高难度动作：跌倒爬起、后空翻
- 复杂地形：最高约30度斜坡、草丛、碎石路

---

## 7. 软件运行方法

### 7.1 环境依赖安装

```bash
# 安装依赖
sudo apt install mesa-common-dev freeglut3-dev coinor-libipopt-dev \
    libblas-dev liblapack-dev gfortran liblapack-dev \
    coinor-libipopt-dev cmake gcc build-essential libglib2.0-dev

# 安装 openjdk
sudo apt-get update
sudo apt-get install openjdk-8-jdk

# 安装 git
sudo apt install git
```

### 7.2 安装 lcm1.3.1

```bash
# 解压 lcm1.3.1 包后进入目录
./configure     # 确保 java support is Enabled
make
sudo make install
sudo ldconfig
```

### 7.3 安装 eigen

```bash
# 解压后进入目录
mkdir build     # 如已有 build 文件夹需先删除
cd build
cmake ..
make install
```

### 7.4 安装 Qt 5.10

```bash
chmod a+x qt-opensource-linux-x64-5.10.0.run
./qt-opensource-linux-x64-5.10.0.run
# 安装到用户主目录下，目录只保留 Qt，不要后面的版本号
```

### 7.5 代码编译

```bash
cd 源码文件目录/scripts
./make_types.sh          # 可能看到 rm: cannot remove... 报错，这是正常的

cd ..
mkdir mc-build
cd mc-build
cmake -DMINI_CHEETAH_BUILD=TRUE ..
make -j4                 # 数字与电脑线程数有关，根据实际情况选择
```

### 7.6 运行仿真

```bash
# 终端 1：打开仿真控制板
cd 源码文件目录/mc-build
./sim/sim

# 在仿真界面中选择：Mini Cheetah 和 Simulator

# 终端 2：运行控制程序
cd 源码文件目录/mc-build
./user/MIT_Controller/mit_ctrl m s
```

**仿真中的控制模式切换**（修改参数文件）：
- `use_rc` 改为 `0`（转换仿真控制，1=控制器控制）
- `control_mode = 6` → recovery stand（恢复站立）
- `control_mode = 3` → balance stand（平衡站立）
- `control_mode = 4` → locomotion（运动模式，默认 Trot 步态）

### 7.7 运行真实机器人

```bash
# ① 停止 YoboGo 内已有程序
ssh user@10.0.0.34        # 密码：123456
ps -ef | grep mit
sudo kill <进程ID>

# ② 发送代码至 YoboGo
cd 源码文件目录/mc-build/
../scripts/send_to_mini_cheetah.sh ./user/MIT_Controller/mit_ctrl

# ③ 开启新终端，SSH 登录 UP Board
ssh user@10.0.0.34        # 密码：123456

# ④ 打开 UP-Board 文件夹
cd ./robot-software/build

# ⑤ 运行程序
./run_mc.sh ./mit_ctrl
```

**连接真实机器人后的仿真界面**：
- 选择 **Robot**（而非 Simulator）
- 机器人位于地面下方一般为正确位置
- 可观察图形界面测试零位、俯仰角等
- IMU 未开启时身体姿势为平躺，可通过鼠标拖动地面转动方向查看

---

## 8. 硬件接口和端口

### 8.1 机器人本体

| 接口 | 说明 |
|------|------|
| 总电源开关 | 右侧按钮，控制所有部件供电 |
| 电机开关 | 左侧按钮，控制电机和电机控制板供电 |
| 急停按钮 | 遥控器 SwE 拨到最下方 |
| 以太网口 | UP Board 上，用于外部通信 |
| 电源板限流拨码开关 | 红色模块上的键1和键2 |

### 8.2 限流拨码开关设置

| 键1 | 键2 | 限流值 | 适用场景 |
|-----|-----|--------|----------|
| 下 | 下 | 15A | 普通 Trot（1.5m/s以下） |
| 上 | 下 | 20A | 高速跑、Bound、Gallop |
| 下 | 上 | 25A | 高速跑、Bound、Gallop |
| 上 | 上 | 无限流 | 类前跳、后空翻 |

### 8.3 电机接口

| 接口 | 说明 |
|------|------|
| 电机电源接口 | 48NM电机为 36V |
| 电机 CAN 接口 | CAN 通信，波特率 1M |
| 电机调试口1 | 串口调试，波特率 921600，ASCII 模式 |
| 关节驱动板 | 最大输出电流 40A，最大输入电压 24V |
| 双绝对式编码器 | 专利技术，可获取 360 度角度信息 |

### 8.4 UP Board 接口

| 接口 | 说明 |
|------|------|
| SPI 接口 | 两个（`/dev/spidev*`） |
| 以太网口 | 与外部电脑通信 |
| USB 口 | 串口调试等 |

---

## 9. 所有命令汇总

### 9.1 SSH 连接

```bash
ssh user@10.0.0.34
# 密码：123456
```

### 9.2 停止机器人程序

```bash
ps -ef | grep mit
sudo kill <进程ID>
```

### 9.3 发送代码到机器人

```bash
../scripts/send_to_mini_cheetah.sh ./user/MIT_Controller/mit_ctrl
```

### 9.4 在 UP Board 上运行程序

```bash
cd ./robot-software/build
./run_mc.sh ./mit_ctrl
```

### 9.5 仿真运行

```bash
./sim/sim
./user/MIT_Controller/mit_ctrl m s
```

### 9.6 编译命令

```bash
cd scripts && ./make_types.sh
mkdir mc-build && cd mc-build
cmake -DMINI_CHEETAH_BUILD=TRUE ..
make -j4
```

### 9.7 依赖安装

```bash
sudo apt install mesa-common-dev freeglut3-dev coinor-libipopt-dev \
    libblas-dev liblapack-dev gfortran liblapack-dev \
    coinor-libipopt-dev cmake gcc build-essential libglib2.0-dev

sudo apt-get install openjdk-8-jdk
sudo apt install git
sudo apt-get install libncurses5-dev libssl-dev
```

### 9.8 LCM 安装

```bash
./configure
make
sudo make install
sudo ldconfig
```

### 9.9 内核编译

```bash
sudo apt-get update && sudo apt-get upgrade
cp .config kernel && cd kernel
make menuconfig
make -j4
sudo make modules_install -j4
sudo make install -j4
sudo update-grub
uname -a                           # 验证 RT 内核
ls /dev/spidev*                    # 验证 SPI 接口
```

### 9.10 修改 grub 启动顺序

```bash
sudo gedit /etc/default/grub
# GRUB_DEFAULT = "1> 6"
sudo update-grub
reboot
```

### 9.11 电机调试命令

通过串口（波特率 921600）发送：

| 命令 | 功能 |
|------|------|
| `m` | 电机驱动模式 |
| `c` | 校准编码器 |
| `s` | 设置电机通信相关参数 |
| `e` | 打印当前编码器信息 |
| `z` | 以当前角度作为零位 |
| `p` | 打印当前角度信息 |
| `r` | 取霍尔最小值 |
| `u` | 确认（在霍尔校准流程中使用） |
| `t` | 取霍尔中间值 |

**编码器校准完整流程**：`c → r → u → t → u → z → z → z`（横杠处需断电重新上电）

### 9.12 LCM 生成消息头文件

```bash
lcm-gen -x example_t.lcm
```

---

## 10. 常见问题和故障排除

| 异常情况 | 处理方法 |
|----------|----------|
| 手柄操作无反应 | SwE 开关滑到最下方 passive 模式 → 关机重启。若无效，联系售后。 |
| YoboGo 自行运动不受控制 | SwE 开关滑到 passive 模式 → 关闭电源 |
| YoboGo 运行一段时间自行停止 | 检查限流拨码开关设置是否合适 |
| 路径走偏，手柄无法纠正 | SwE → passive → 重启 |
| YoboGo 摔倒后静止不动 | SwE → passive → 关机重启 |
| YoboGo 摔倒后仍在摆动 | SwE → passive → 关机重启 |
| YoboGo 起火 | SwE 拨到 passive 模式 → 关闭电源 → 灭火 |

### 通用故障处理流程

```
异常发生 → 1. SwE 拨到最下方（passive/急停）
           → 2. 关闭电源开关
           → 3. 检查线路、初始位置
           → 4. 重新开机
```

---

## 11. 质量保证与售后

- 质保期（非人为原因）：**1-3 个月**
- 质保范围：制造工艺、材料引起的设备损坏
- 质保外：提供永久成本价维修服务
- 客户错误操作、不当修理/改造、不可抗力 → 收取成本维修费用
- 提供免费用户操作培训
- 提供售后服务技术支持

---

## ⚠️ 重要安全提醒

1. **开机前**：仔细检查遥控器与 YoboGo 的初始位置，防止绕线
2. **操作时**：不要猛推控制摇杆，防止未定义行为
3. **失控时**：先让 YoboGo 处于 passive 状态（SwE 最下方），再断电
4. **运行中**：不要将手或身体部位伸入关节运动空间
5. **存储时**：关闭主电源，用防尘布遮盖，禁止淋水，每月充电
6. **搬运时**：关闭主电源，急停按钮和电源按钮保持关闭状态
