# 网络调试经验 — 原生 Ubuntu 直连机器狗

记录 2026-09 原生 Ubuntu（HP OMEN 笔记本）通过网线连接机器狗 YoboGo-10S 的完整调试过程：连接流程、遇到的问题与解决、r8168 驱动、`robot` 网络配置、VMware 虚拟机与原生 Ubuntu 的差异分析、推荐硬件方案。

> 关联文档：
> - `docs/features/robot-connection-guide.md` — 标准连接操作与脚本工具
> - `docs/features/vm-connection-guide.md` — 官方 PDF 的 VMware 连接流程（逐字分析）
> - `docs/changes/2026-09-22.md` — 当日修改日志

---

## 目录

1. [机器狗网络连接完整流程](#1-机器狗网络连接完整流程)
2. [遇到的问题和解决方案](#2-遇到的问题和解决方案)
3. [r8168 驱动安装](#3-r8168-驱动安装)
4. [robot 网络配置](#4-robot-网络配置)
5. [VMware 虚拟机 vs 原生 Ubuntu 差异分析](#5-vmware-虚拟机-vs-原生-ubuntu-差异分析)
6. [推荐硬件方案](#6-推荐硬件方案)
7. [排障速查表](#7-排障速查表)

---

## 1. 机器狗网络连接完整流程

### 1.1 网络参数

| 项目 | 值 |
|------|-----|
| 机器狗（UP Board） | `10.0.0.34`，SSH `user@10.0.0.34`，密码 `123456` |
| 本机（上位机） | `10.0.0.30/24`，无默认网关（连接名 `robot`） |
| 子网 | `10.0.0.0/24`，直连不需要网关 |
| 上位机程序 bind 约束 | socketServer 硬编码 `nativeIp "10.0.0.30"`，本机 IP 不是 .30 则收不到图像 |

### 1.2 连接顺序

```
1. 插网线（电脑 RJ45 ↔ 机器狗外壳 RJ45）
2. 启动机器狗：先开电机开关，再开总开关（拔掉充电线！充电时开机会电机异响）
3. 等待 30-60s 系统启动
4. 本机激活 robot 有线连接（NetworkManager 自动/手动激活）
5. ping 10.0.0.34        ← 通了才算连上
6. ssh user@10.0.0.34    ← 密码 123456，终端保持开启
7. scp -r 代码/ user@10.0.0.34:/home/user/   ← 传程序
```

### 1.3 关于 WiFi 的策略（本项目）

官方 PDF 要求连接时关闭 WiFi，但本项目 **WiFi 保持开启**（供 AI/外网使用）：

- WiFi（如 `172.19.x.x`）与有线 `10.0.0.x` **不同子网**，路由表不冲突
- 关键是 `robot` 配置**不设默认网关、never-default**（见第 4 节），保证默认路由仍走 WiFi
- 若出现 `No route to host`，先检查 `robot` 连接是否激活、再考虑临时关 WiFi 做对照

---

## 2. 遇到的问题和解决方案

### 2.1 问题 1：外壳 RJ45 口物理链路不通（NO-CARRIER）

**现象**：

```bash
ip addr show eno1          # 无 inet 地址
ethtool eno1               # Speed: Unknown!  Link detected: no
dmesg | grep eno1          # NO-CARRIER
```

- 机器狗外壳只有一个 RJ45 口，**无法拆壳**检查内部接线
- 换网线无效；等待启动完成无效；强制 100Mbps / 10Mbps（`ethtool -s eno1 speed 100 duplex full autoneg off`）均 `Link detected: no`
- DHCP 激活失败：`No suitable device found ... device has no carrier`（NetworkManager 拒绝在无载波设备上激活）

**排查中排除的因素**：

| 假设 | 验证方式 | 结论 |
|------|----------|------|
| 网线坏了 | 换多根网线 | ❌ 排除 |
| 电脑网口坏了 | 回家接路由器，1000Mbps Full 正常 | ❌ 排除 |
| 电池问题 | 充电后重启 | ❌ 排除（充电线插着开机会电机异响，需拔掉） |
| IP/配置问题 | 多种 nmcli/ethtool 诊断 | ❌ 排除（物理层无载波，软件层无从下手） |
| r8169 驱动问题 | 换装 r8168-dkms | ❌ 未解决（见第 3 节） |
| 链路抖动 | 短暂 ping 通后又断 | 属于协商不稳定，非配置问题 |

**根因结论**：HP OMEN 的 Realtek RTL8111/8168 网卡与机器狗 UP Board 网口之间的**物理层 Auto-Negotiation 协商失败**（MDI/MDI-X 交叉或 PHY 速度/初始化兼容性），属于链路层以下问题，软件（Linux bridge、iptables、强制协商）无法修复。

**佐证**：r8168 驱动下 `ethtool` 的 `Supported link modes` 只剩 `10baseT/Half 10baseT/Full`（千兆/百兆消失），NIC 统计全零，PHY 初始化可能不完整。

### 2.2 问题 2：robot 配置抢走 WiFi 路由（断网）

**现象**：激活 `robot` 连接后 WiFi 断网。

**原因**：初版配置设置了默认网关 `10.0.0.1` 且允许默认路由。

**修复**（见第 4 节命令）：清空网关 + `never-default yes` + 提高 route-metric。

### 2.3 问题 3：充电时开机异常

充电线插着开机会持续响声和震动（电机异常）——**先拔充电线再开机**。顺序：先开电机开关，再开总开关。

---

## 3. r8168 驱动安装

**目的**：用 Realtek 官方 r8168 驱动替代内核自带的 r8169 驱动，排除驱动差异导致的协商问题。

```bash
sudo apt install -y r8168-dkms
# r8168-dkms 8.055.00-1build1
# DKMS 会为所有已安装内核编译模块
sudo reboot    # 重启后生效
```

**验证**：

```bash
dpkg -l r8168-dkms | tail -2
lsmod | grep r8168          # 应看到 r8168，而非 r8169
ethtool eno1                # 观察 Supported link modes / Link detected
```

**结果**：驱动切换成功，但与机器狗直连仍 `Link detected: no`——证明**不是单纯的驱动选择问题**，而是两块网卡 PHY 之间的协商兼容性。驱动已保留（比 r8169 更新，且接路由器正常）。

---

## 4. robot 网络配置

### 4.1 创建

```bash
sudo nmcli connection add type ethernet con-name "robot" ifname eno1 \
  ipv4.method manual ipv4.addresses "10.0.0.30/24" \
  ipv4.gateway "" ipv4.never-default yes ipv4.route-metric 600
```

### 4.2 修正（关键！防止抢 WiFi 路由）

```bash
sudo nmcli connection modify "robot" \
  ipv4.gateway "" \
  ipv4.never-default yes \
  ipv4.route-metric 600
```

| 参数 | 值 | 原因 |
|------|-----|------|
| `ipv4.addresses` | `10.0.0.30/24` | socketServer 硬编码 bind 10.0.0.30 |
| `ipv4.gateway` | 空 | 直连不需要网关；有网关会覆盖 WiFi 默认路由 |
| `ipv4.never-default` | `yes` | **绝不**成为默认路由出口 |
| `ipv4.route-metric` | `600` | 高于 WiFi 默认 metric，进一步降低优先级 |

### 4.3 激活 / 停用

```bash
sudo nmcli connection up "robot"
nmcli connection show --active
ip addr show eno1        # 应有 10.0.0.30/24
ip route                 # default 应仍指向 WiFi
```

### 4.4 自检清单

- [ ] `ip addr show eno1` 显示 `10.0.0.30/24`
- [ ] `ip route` 默认路由仍在 WiFi 接口上
- [ ] WiFi 外网正常（`curl -sI https://www.baidu.com`）
- [ ] `ping 10.0.0.34` 通（机器狗开机且链路正常时）
- [ ] `ssh user@10.0.0.34` 成功

---

## 5. VMware 虚拟机 vs 原生 Ubuntu 差异分析

### 5.1 现象

| 环境 | 与机器狗直连结果 |
|------|------------------|
| VMware 虚拟机 Ubuntu（桥接模式） | ✅ 能通（社区/官方文档验证的做法） |
| 原生 Ubuntu（同一台 HP OMEN） | ❌ `NO-CARRIER`，物理层不通 |

### 5.2 原因分析

```
原生 Ubuntu：
  Linux 驱动(r8168/r8169) ──直接控制──> RTL8111/8168 PHY <──协商──> UP Board 网口 PHY
                                        ↑ 协商失败 → NO-CARRIER

VMware 桥接模式：
  Ubuntu VM ──> vmnet 虚拟网卡 ──> Windows 网卡驱动 ──> RTL8111/8168 PHY <──协商──> UP Board 网口 PHY
                                   ↑ 物理由 Windows 驱动完成，协商参数/MDI-X 处理不同 → 协商成功
```

关键点：

1. **虚拟机看到的是虚拟网卡**，它与宿主机物理网卡之间走内存/内核桥，**不涉及物理层协商**
2. **物理层协商由 Windows Realtek 驱动完成**——Windows 驱动的 PHY 初始化、Auto-MDI/MDIX、降速回退策略与 Linux 驱动不同
3. VMware 的"虚拟交换机"**不解决**物理协商问题——它工作在链路层（L2），物理协商是它下面那一层
4. 因此"VMware 能通"本质上证明了：**网线、机器狗网口、电脑网口硬件都正常，是 Linux 驱动与 UP Board PHY 之间的协商兼容性问题**

### 5.3 三种可能的协商失败机制

1. **MDI/MDI-X 交叉问题**：两端未自动交叉（UP Board 可能不支持 Auto-MDI/MDIX）；路由器/交换机会自动处理，直连则看两端能力
2. **速度协商不兼容**：UP Board PHY 可能只有 10/100M，Linux 驱动协商策略与 Windows 不同
3. **PHY 寄存器初始化差异**：同一块 RTL8111/8168，r8168/r8169 与 Windows 驱动初始化参数不同（佐证：r8168 下 ethtool 只剩 10baseT 模式）

### 5.4 结论与对策

- 软件无法修复物理层（Auto-Negotiation 不受 OS 软件控制），只能**绕过**
- 短期绕过：继续用 VMware 桥接（已验证可行），或本机 Linux bridge 组合强制协商（效果不确定）
- 长期方案：见第 6 节硬件方案

---

## 6. 推荐硬件方案

按推荐度排序（价格为参考价）：

| 优先级 | 方案 | 约价 | 原理 / 说明 |
|--------|------|------|-------------|
| ⭐1 | **5 口小交换机**（~20 元） | 20 元 | 电脑和机器狗都接交换机；交换机负责物理层协商（Auto-MDI/MDIX + 速度回退），最接近 VMware 的"中转"效果。**首推** |
| 2 | **USB 转以太网适配器** | 15 元 | 换一颗完全不同的 PHY 芯片（RTL8153/AX88179 等），绕开 RTL8111/8168 与 UP Board 的兼容性；需改 `robot` 配置的 `ifname` |
| 3 | **交叉网线** | 10 元 | 验证 MDI/MDI-X 假说；若通则确认是交叉问题（正规路由/交换机环境下意义有限） |
| 4 | 软件强制协商（Linux bridge + ethtool） | 0 元 | **已验证不能修复物理层**，仅理论尝试项 |
| 备选 | 手机 USB 网络共享 / 串口直连（UART 排针） | 0 元 | 完全绕过以太网；串口需拆壳或找调试口 |

**使用交换机后的流程**：电脑 ↔ 交换机 ↔ 机器狗，交换机上电，其余步骤（`robot` 配置、ping/ssh/scp）不变。

---

## 7. 排障速查表

| 症状 | 检查 / 处理 |
|------|-------------|
| `eno1` NO-CARRIER | 换网线 → 接路由器验证电脑口 → 换交换机/USB 网卡（物理层问题，软件无效） |
| `Link detected: no` + 强制降速无效 | 物理协商失败，见第 5、6 节 |
| 短暂通又断（链路抖动） | 检查电池电量（耗尽时 UP Board 电压不稳）、网线接触、驱动协商稳定性 |
| SSH `No route to host` | 确认 `robot` 已激活、本机是 10.0.0.30、网线插好；必要时临时关 WiFi 对照 |
| 激活 robot 后 WiFi 断 | 检查 `ipv4.gateway` 是否为空、`never-default yes` 是否生效（第 4.2 节） |
| DHCP 激活报 `no carrier` | 物理链路不通，NetworkManager 拒绝激活属正常；改用静态 IP 的 `robot` 配置并先解决链路 |
| socketServer 收不到图像 | 本机 IP 必须是 `10.0.0.30`（源码硬编码 bind）；检查 30015/30016/30017 |
| 开机电机异响 | 拔充电线，先电机开关再总开关 |

---

*经验来源：2026-09 实机调试（详见 `docs/changes/2026-09-22.md`）；连接命令与官方 PDF 流程见 `docs/features/vm-connection-guide.md`。*
