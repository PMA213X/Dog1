# track1.6 代码结构分析报告

**生成时间**: 2024
**源码路径**: `/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/YoboGo-control/track1.6/`

---

## 1. 完整目录树

```
track1.6/
├── send.sh                          # 部署脚本（打包 + SCP 远程传输）
├── 更新日志.txt                      # 版本更新历史记录
└── track/                           # 核心源码目录
    ├── .qmake.stash                 # qmake 缓存文件
    ├── track.pro                    # Qt 工程文件（构建系统）
    ├── track.pro.user               # Qt Creator 用户配置
    ├── track.pro.user.36f98e2       # Qt Creator 用户配置副本
    ├── Makefile                     # qmake 生成的 Makefile
    ├── main.cpp                     # 主程序入口（574 行）
    ├── colorgroup.cpp               # 颜色阈值管理 + UDP 图像发送（343 行）
    ├── colorgroup.h                 # 颜色组类定义（63 行）
    ├── lcmutil.cpp                  # LCM 通信封装（45 行）
    ├── lcmutil.h                    # LCM 工具类定义（16 行）
    ├── mythread.cpp                 # 定时/状态机线程（66 行）
    ├── mythread.h                   # 线程类 + 模式枚举定义（32 行）
    ├── udputil.cpp                  # UDP 接收服务器（71 行）
    ├── udputil.h                    # UDP 工具类定义（19 行）
    ├── robot_control_lcmt.lcm       # LCM 消息类型定义（8 行）
    ├── robot_control_lcmt.hpp       # LCM 自动生成的 C++ 头文件（v1，当前使用）
    ├── robot_control_lcmt2.hpp      # LCM 自动生成的 C++ 头文件（v2，备用）
    └── form.ui                      # Qt UI 文件（颜色阈值调节界面）
```

---

## 2. 各关键文件功能说明

### 2.1 main.cpp — 主程序入口

| 功能 | 说明 |
|------|------|
| **摄像头采集** | 自动检测 `/dev/video0~3`，支持热插拔自动重连 |
| **图像处理** | 灰度化 → 中值滤波 → 阈值二值化(130~255) → 赛道中线提取 |
| **赛道循迹** | `getAverage()` 函数：提取左右边缘 → 计算中线均值 → 偏差控制 |
| **黑色条带识别** | `recognizeBlackStripe()` 递归算法判断长/短条带 |
| **运动控制** | 根据中线偏差计算 `v_des[1]`(横移) 和 `v_des[2]`(转向) |
| **UDP 接收** | 通过 `UdpUtil` 线程接收上位机控制指令 |
| **LCM 发送** | 通过 `lcmUtil` 将运动参数发送至机器狗下位机 |
| **模式切换** | 根据 UDP 收到的指令码切换运行模式 |

**图像处理流程**:
```
原始图像(640x480) → resize(400x300) → 中值滤波(5x5) → 灰度转换
→ inRange(130,255) 二值化 → 边缘检测 → 中线计算 → PID 控制
```

**运动参数**:
- `gait_type`: 3=运动, 4=停止
- `step_height`: 步高(默认 0.03~0.04)
- `stand_height`: 站立高度(默认 0.3，限高区降至 0.12~0.2)
- `v_des[0]`: 前进速度(默认 0.15)
- `v_des[1]`: 横移速度(自动计算)
- `v_des[2]`: 转向速度(自动计算)
- `rpy_des[0/1/2]`: 横滚/俯仰/偏航角

### 2.2 colorgroup.cpp/h — 颜色阈值管理

| 功能 | 说明 |
|------|------|
| **颜色阈值存储** | 支持 7 种颜色：黄色、蓝色、紫色、棕色、绿色、红色、白色 |
| **文件读写** | 从 `colorGroup.txt` 读取/写入 HSV 阈值 |
| **UDP 图像发送** | 将原图和二值化图像通过 UDP 发送到上位机 |
| **UDP 阈值发送** | 将当前颜色阈值发送到上位机 |
| **图像显示** | 可在本地 GUI 显示或远程 UDP 传输 |

**颜色定义宏**:
```cpp
#define yellow 0
#define white  1
#define blue   2
#define violet 3
#define brown  4
#define green  5
#define red    6
```

**默认颜色阈值** (Scalar 三通道):
| 颜色 | Min | Max |
|------|-----|-----|
| 棕色 | (79,74,118) | (255,255,255) |
| 黄色 | (0,0,0) | (255,255,255) |
| 绿色 | (0,0,0) | (98,116,114) |
| 紫色 | (12,123,79) | (93,200,149) |
| 蓝色 | (158,109,0) | (255,209,84) |
| 红色 | (158,109,0) | (255,209,84) |
| 白色 | (10,10,10) | (255,255,255) |

### 2.3 lcmutil.cpp/h — LCM 通信

| 功能 | 说明 |
|------|------|
| **LCM 初始化** | 使用 UDP 组播 `udpm://239.255.76.67:7667?ttl=1` |
| **消息发布** | 通过 `"voice_lcm"` 频道发布运动控制指令 |
| **数据封装** | 将 v_des、gait_type、step_height、stand_height、rpy_des 打包为 `robot_control_lcmt` 结构体 |

### 2.4 udputil.cpp/h — UDP 接收服务

| 功能 | 说明 |
|------|------|
| **监听端口** | 8000 |
| **绑定地址** | `INADDR_ANY`（监听所有网卡） |
| **协议** | UDP，接收上位机控制指令 |
| **指令处理** | 接收到的第一个字节存入 `receivedFlag`，并回传确认 |

**指令码映射**:
| 接收字节(ASCII) | 十进制 | 模式 |
|-----------------|--------|------|
| '0' | 48 | limitHeight（已注释） |
| '1' | 49 | residence（住户区） |
| '2' | 50 | stop（停止/交付） |
| '3' | 51 | stop（停止/交付） |

### 2.5 mythread.cpp/h — 状态机线程

定义了系统的运行模式枚举：

```cpp
enum Mode {
    residence   = 0,  // 住户区模式
    track       = 1,  // 循迹模式（默认）
    upstair     = 2,  // 上楼梯模式
    limitHeight = 3,  // 限高模式
    obstruct    = 4,  // 障碍模式
    stop        = 5   // 停止/交付模式
};
```

**模式持续时间**:
| 模式 | 持续时间 | 说明 |
|------|----------|------|
| residence | 2.5s | 然后切回 track |
| upstair | 7s | 然后切回 track |
| limitHeight | 1s → 6s → 1s | 低姿(0.12) → 恢复(0.2) → 切回 track |
| stop | 1.5s + 3s | 等待 → 倾身交付 → 切回 track |
| obstruct | 即时 | 直接切回 track |

### 2.6 robot_control_lcmt.lcm — LCM 消息定义

```c
struct robot_control_lcmt {
    int32_t control_mode;       // 控制模式
    int32_t gait_type;          // 步态类型（3=运动，4=停止）
    float   v_des[3];           // 期望速度 [前进, 横移, 旋转]
    float   step_height_lcm;    // 步高
    float   stand_height_lcm;   // 站立高度
    float   rpy_des[3];         // 期望姿态 [横滚, 俯仰, 偏航]
}
```

**注意**: `robot_control_lcmt2.hpp` 是早期版本，缺少 `stand_height_lcm` 字段，hash 值不同，当前未使用。

### 2.7 form.ui — 颜色阈值调节界面

Qt Designer UI 文件，包含：
- 6 个 QSlider（min1~3, max1~3）用于调节颜色阈值的 3 个通道
- 8 个 QLineEdit 显示当前值
- 1 个 QPushButton("save") 保存阈值

### 2.8 send.sh — 部署脚本

```bash
#!/bin/bash
./build/linuxdeployqt ./build/track -appimage   # 打包为 AppImage
rm ./build/colorGroup.txt                        # 删除颜色配置
scp -r ./build/ user@10.0.0.34:/home/user/track2025  # SCP 到目标机
```

---

## 3. 所有 IP 地址和网络配置

### 3.1 IP 地址

| IP 地址 | 用途 | 定义位置 |
|---------|------|----------|
| `10.0.0.30` | 目标机（上位机）IP | `colorgroup.cpp` 第 13 行 |
| `10.0.0.34` | 本机 IP | `colorgroup.cpp` 第 14 行、`send.sh` 第 4 行 |
| `239.255.76.67` | LCM 组播地址 | `lcmutil.cpp` 第 17 行 |

### 3.2 端口号汇总

| 端口 | 协议 | 用途 | 位置 |
|------|------|------|------|
| **8000** | UDP | 接收上位机控制指令 | `udputil.cpp` 第 11 行 |
| **7667** | UDP Multicast | LCM 机器人控制消息 | `lcmutil.cpp` 第 17 行 |
| **30014** | UDP | 颜色阈值接收端口（绑定本机 10.0.0.34） | `colorgroup.cpp` 第 90 行 |
| **30015** | UDP | 发送原图到上位机 | `colorgroup.cpp` 第 223 行 |
| **30016** | UDP | 发送二值化图像到上位机 | `colorgroup.cpp` 第 228 行 |
| **30017** | UDP | 发送颜色阈值到上位机 | `colorgroup.cpp` 第 279 行 |

### 3.3 网络通信拓扑

```
┌─────────────┐     UDP:8000      ┌─────────────────┐    LCM:7667     ┌──────────┐
│   上位机     │ ────────────────> │   track1.6 主控  │ ──────────────> │ 机器狗   │
│  10.0.0.30  │                   │   10.0.0.34      │                 │ 下位机   │
│             │ <──────────────── │                  │                 │          │
│             │    UDP:30015/16/17 │                 │                 │          │
└─────────────┘                   └─────────────────┘                 └──────────┘
```

---

## 4. 支持的运行模式和参数

### 4.1 命令行参数

```bash
# 无参数（默认循迹模式）
./track

# 指定住户区方向
./track left     # 住户区在左侧（左转）
./track right    # 住户区在右侧（右转）
```

### 4.2 运行模式

1. **track（循迹模式）** — 默认模式，沿赛道白线行驶
2. **residence（住户区模式）** — 检测到住户区标记后，执行转向进入/离开
3. **limitHeight（限高模式）** — 降低站立高度通过限高区域
4. **stop（停止/交付模式）** — 停止并执行倾身交付动作
5. **upstair（上楼梯模式）** — 保留枚举，当前代码中未激活
6. **obstruct（障碍模式）** — 保留枚举，当前代码中直接切换回循迹

### 4.3 UDP 远程控制指令

上位机通过 UDP 向端口 8000 发送单字节指令：
- `0` (0x30) → limitHeight（已注释，不生效）
- `1` (0x31) → residence
- `2` (0x32) → stop
- `3` (0x33) → stop

---

## 5. 编译和运行步骤

### 5.1 依赖环境

- **Qt 5.10.0**（包含 QtWidgets、QtGui、QtNetwork、QtCore 模块）
- **OpenCV 5**（自定义编译路径 `/usr/local/lib/`）
  - 核心库：core、imgproc、highgui、videoio、imgcodecs、calib、features2d、flann、ml、objdetect、photo、stitching、wechat_qrcode
- **LCM**（Lightweight Communications and Marshalling）库
- **GCC/G++**（支持 C++11）

### 5.2 编译步骤

```bash
cd track1.6/track

# 方式一：使用 qmake
/home/user/Qt/5.10.0/gcc_64/bin/qmake track.pro
make

# 方式二：直接使用已有 Makefile
make
```

### 5.3 运行

```bash
# 需要 colorGroup.txt 文件在当前工作目录
./track          # 默认循迹模式
./track left     # 住户区在左侧
./track right    # 住户区在右侧
```

### 5.4 部署到目标机

```bash
# 使用 send.sh 脚本
cd track1.6/
./send.sh
# 会执行：linuxdeployqt 打包 → 删除 colorGroup.txt → SCP 到 10.0.0.34
```

### 5.5 前置条件

- 需要 `colorGroup.txt` 文件（颜色阈值配置），格式为：
  ```
  yellow min0 min1 min2 max0 max1 max2
  blue min0 min1 min2 max0 max1 max2
  violet min0 min1 min2 max0 max1 max2
  green min0 min1 min2 max0 max1 max2
  brown min0 min1 min2 max0 max1 max2
  red min0 min1 min2 max0 max1 max2
  white min0 min1 min2 max0 max1 max2
  ```
- LCM 需要在同一组播网络内才能通信
- 摄像头需要连接（自动检测 video0~3）

---

## 6. 版本更新历史（来自 更新日志.txt）

| 日期 | 内容 |
|------|------|
| 2021/08/01 | 最初版本，完成初步测试 |
| 2021/08/25 | 增加 `colorGroup.txt` 生成，track 程序实例化时自动读取 |
| 2021/09/09 | 摄像头自动识别功能，无需手动输入参数；修复摄像头中断问题（支持热插拔） |
| 2021/10/08 | 二维码识别库从 OpenCV 自带替换为微信开源的 wechat_qrcode |
| 2022/03/30 | 注释掉二维码部分，增加上位机读取视频功能 |

---

## 7. 与 track1.1 的区别

在当前代码库 (`/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/`) 中 **未找到 track1.1 目录**。`YoboGo-control/` 目录下仅包含 `track1.6/` 版本。

根据代码中的注释和更新日志，track1.6 相对于早期版本（推测的 track1.0~1.5）的演进特征：

### 7.1 已注释但保留的旧功能（可追溯的差异）

1. **二维码识别功能**（已注释）：
   - 使用了 `opencv2/wechat_qrcode.hpp`（微信二维码库），但二维码相关逻辑已被全部注释
   - 早期版本应有二维码识别功能

2. **多阶段住户区交互**（已注释）：
   - 旧版有 4 阶段住户区交互流程（residenceTransientProcess 1→4）
   - 新版简化为 2.5s 定时转向后直接切回循迹

3. **限高区多阶段控制**（已注释）：
   - 旧版有阶梯式降低高度的精确控制
   - 新版简化为 3 阶段（降低 → 恢复 → 切回）

4. **import/upslope 模式**（已注释）：
   - 早期版本可能有"导入"和"上坡"模式
   - 当前版本仅保留枚举定义但不使用

5. **图像处理变化**：
   - 旧版使用 HSV 颜色空间的 `inRange`（`colorgroup.whiteMin/whiteMax`）
   - 1.6 版改用灰度图直接阈值二值化 `inRange(grayImage, 130, 255, frame)`
   - 这表明 1.6 版专注于更简单的赛道识别方案

6. **LCM 频道变更**：
   - 旧频道名 `"robotctrl"` 被注释，改为 `"voice_lcm"`
   - 说明下位机固件可能有配套更新

### 7.2 架构特征总结

track1.6 是一个面向 **2023 年山东省高校机器人大赛** 的四足机器人赛道循迹控制程序，采用：
- **Qt** 框架提供多线程、网络通信、UI 基础
- **OpenCV 5** 进行图像采集和处理
- **LCM** 进行低延迟的机器人运动控制通信
- **UDP Socket** 与上位机进行指令交互和图像回传
- 大量旧功能以注释形式保留，说明这是一个迭代开发中的竞赛项目

---

## 8. 关键数据流总结

```
摄像头 (0~3) → VideoCapture → srcImage (640x480)
    ↓
resize(400x300) → 中值滤波(5x5) → 灰度化 → 二值化(130~255)
    ↓
getAverage(): 边缘提取 → 中线计算 → 平均偏差 average
    ↓
PID 控制: v_des[1] = 0.001 * (average - goalAverage)  [横移]
          v_des[2] = 0.006 * (goalAverage - average)  [转向]
    ↓
lcmUtil->send() → LCM publish("voice_lcm") → 239.255.76.67:7667 → 机器狗
    ↓ (每帧间隔 20ms ≈ 50fps)
主循环继续
```

**附带数据流**:
```
上位机(10.0.0.30) → UDP:8000 → UdpUtil.receivedFlag → 模式切换
上位机(10.0.0.30) → UDP:30014 → colorGroup 颜色阈值设置
colorGroup → UDP:30015 → 上位机(原图)
colorGroup → UDP:30016 → 上位机(二值化图)
colorGroup → UDP:30017 → 上位机(颜色阈值)
```
