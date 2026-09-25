# VMware 虚拟机 + Ubuntu 连接机器狗完整指南（源自官方调试说明文档）

> **资料来源**：`2026robocup中型组比赛资料10/中国机器人大赛调试说明文档.pdf`（18 页，WPS 文字制作，作者：贾璐，创建日期 2024-09-24；文内标题：**中型机器狗比赛调试说明**）
>
> 同目录下 `中国机器人大赛调试说明文档(1).pdf` 与前者 **完全相同**（MD5 均为 `f1fbbaf25b85aab58e766d2bdbe0f527`），可视为同一份文档。
>
> 本报告对 PDF 中**所有涉及网络连接与 VM 虚拟机配置的内容逐字引用**（含截图内文字），并结合 `socketServer` 源码给出分析。**引用原文用引用块/代码块标出，不作概括改写。**

---

## 目录

1. [核心结论速览](#1-核心结论速览)
2. [第一部分：开发环境搭建（VMware 虚拟机配置）— 逐字原文](#2-第一部分开发环境搭建vmware-虚拟机配置--逐字原文)
3. [第二部分：与机器狗建立连接 — 逐字原文](#3-第二部分与机器狗建立连接--逐字原文)
4. [第四部分：程序运行（网络相关）— 逐字原文](#4-第四部分程序运行网络相关--逐字原文)
5. [附录：可能遇到的问题（网络相关）— 逐字原文](#5-附录可能遇到的问题网络相关--逐字原文)
6. [PDF 截图中的关键网络信息（文字提取不到的部分）](#6-pdf-截图中的关键网络信息文字提取不到的部分)
7. [针对六个重点问题的回答](#7-针对六个重点问题的回答)
8. [socketServer 源码中的网络配置](#8-socketserver-源码中的网络配置)
9. [综合：推荐的完整连接流程](#9-综合推荐的完整连接流程)
10. [目录检查结果](#10-目录检查结果)

---

## 1. 核心结论速览

| 问题 | 答案（依据原文） |
|------|------|
| 虚拟机网络模式 | **桥接模式（Bridged，自动）**。原文：「虚拟机应当设置为桥接模式」；第 3 页 VMware 设置截图：「网络适配器　桥接模式 (自动)」。**不是 NAT，不是 Host-Only** |
| Windows 宿主机要不要配 IP | **文档完全未提及**在 Windows 端配置 IP。只要求：「关闭电脑 Wifi」 |
| 虚拟机内 Ubuntu 要不要配 IP | 文档没有写"手动填 IP"的步骤，但明确要求：「**Ubuntu 连接方式选择 robot**」——即在 Ubuntu 右上角网络菜单里选中名为 **`robot`** 的预置有线连接配置文件（镜像内已建好，里面预设了 10.0.0.x 网段 IP） |
| 连接顺序 | 关 WiFi → Ubuntu 选 `robot` 连接 → 网线连电脑与机器狗 → 按机器狗启动按钮 → 等启动 → `ping 10.0.0.34` → `ssh user@10.0.0.34` → `scp` 传代码 |
| 机器狗 IP | `10.0.0.34`（登录用户 `user`，密码 `123456`） |
| 本机（虚拟机）IP | `10.0.0.30`（证据：ssh 截图 `Last login ... from 10.0.0.30`；socketServer 源码 `#define nativeIp "10.0.0.30"`） |
| 网口/网线/指示灯 | 多处强调网线必须始终接好、可重新插拔；**全文没有任何"指示灯/网口灯"的描述** |
| VMware 适配器设置步骤 | 用 VMware Workstation 15 打开镜像；弹"已移动或复制"对话框时选「**我已复制该虚拟机(P)**」；硬件列表确认「网络适配器 = 桥接模式 (自动)」 |

**⚠️ 连不上机器狗时最可能的三个原因（按原文排查顺序）**：
1. 虚拟机**不是桥接模式**（必须桥接）；
2. **WiFi 没关**（Windows 端和虚拟机都要求关闭 WiFi）；
3. Ubuntu 网络菜单里**没有选中 `robot` 连接**（本机 IP 不在 10.0.0.x 网段，ping/ssh/socketServer 全部失败）。

---

## 2. 第一部分：开发环境搭建（VMware 虚拟机配置）— 逐字原文

PDF 第 2 页（目录标注"一、开发环境搭建 ......... 2"），章节标题及全文如下：

> **一、　开发环境搭建（镜像与之前一样，可直接使用）**
>
> （1）安装 VMware 系统包
>
> 下载安装 VMware Workstation 15 系统包，参考链接 https://cloud.tencent.com/developer/article/1436461，许可证秘钥可以在网上搜一个即可。
>
> （2）解压 Ubuntu 镜像
>
> 打开下载好并解压的开发系统包（建议使用 7zip 解压，其他解压软件可能会造成文件缺失）。解压后的文件夹内应包含以下文件：

（截图：解压后文件列表，含 `Ubuntu 64位.nvram`、`Ubuntu 64位.vmdk`、`Ubuntu 64位.vmsd`、`Ubuntu 64位.vmx`、`Ubuntu 64位.vmxdf`、`vmware.log`、`vmware-0.log`、`vmware-1.log`、`vmware-2.log` 等，类型列为"VMware 虚拟…"）

> （3）打开镜像文件
>
> 使用 VMware Workstation 15 打开 ubuntu 镜像文件
>
> 若出现如下界面，直接点击回车:

**【第 3 页截图 1 — VMware 打开菜单】** 红色箭头 ① 指向菜单栏「VMware Workstation」，箭头 ② 指向菜单「文件(F) → 打开(O)... Ctrl+O」。

**【第 3 页截图 2 — Windows 文件选择对话框】** 路径栏显示 `… YoBoGo(ubuntu) ▸ ubuntu`，文件列表中选中 **`Ubuntu 64位.vmx`**（类型：VMware 虚拟机配置），红色箭头 ① 指向该 .vmx 文件，箭头 ② 指向「打开(O)」按钮。

**【第 3 页截图 3 — 虚拟机库界面】** 显示「Ubuntu 64 位」条目，左侧：▶ 开启此虚拟机、**编辑虚拟机设置**（有红色箭头指向此处）。

**【第 3 页截图 4 — "已移动或复制"对话框】**（这是网络功能相关的关键一步）对话框标题 `Ubuntu 64 位 - VMware Workstation`，内容逐字为：

> 此虚拟机可能已被移动或复制。
>
> 为了配置特定的管理和网络功能，VMware Workstation 需要知道是否已移动或复制了此虚拟机。
>
> 如果您不知道，请回答"我已复制该虚拟机(P)"。
>
> 按钮：[我已移动该虚拟机(M)]　**[我已复制该虚拟机(P)]**　[取消]
>
> （红色箭头指向「我已复制该虚拟机(P)」按钮）

**【第 3 页截图 5 — 虚拟机硬件设置总览】（网络适配器模式的直接证据）** 逐字内容：

```
▶ 开启此虚拟机
  编辑虚拟机设置
设备
  内存                 2 GB
  处理器               2
  硬盘 (SCSI)          40 GB
  CD/DVD (SATA)        自动检测
  网络适配器            桥接模式 (自动)      ← 关键！Bridged (Automatic)
  USB 控制器           存在
  声卡                 自动检测
  打印机               存在
  显示器               自动检测
描述
  在此处键入对该虚拟机的描述。
```

> 打开后界面显示如下，输入密码 123456：

（截图：Ubuntu 16.04 LTS 登录界面，用户名 `user`，红色箭头标注「密码123456」）

> 进入桌面显示

（截图：Ubuntu Desktop 桌面，红色箭头分别标注「文件管理器」和侧边栏的开发环境图标）

**GNU GRUB 启动菜单截图（第 4 页顶部）**：

```
GNU GRUB  version 2.02"beta2-36ubuntu3.20
*Ubuntu
 Advanced options for Ubuntu
 Memory test (memtest86+x64.efi)
 Memory test (memtest86+x64.efi, serial console 115200)
```

---

## 3. 第二部分：与机器狗建立连接 — 逐字原文

**这是全篇最核心的网络连接章节**，PDF 第 4–6 页（目录标注"二、与机器狗建立连接 ......... 4"）。全文逐字如下：

> **二、　与机器狗建立连接**
>
> （1）**关闭电脑 Wifi，Ubuntu 连接方式选择 robot。**使用网线连接电脑与机器狗，按下机器狗启动按钮，等待机器狗启动
>
> 待机器狗启动后，ctrl+alt+t 打开终端，输入 ping 10.0.0.34 指令，显示如下代表连接成功。（10.0.34 是机器狗的 IP 地址）
>
> Ctrl+C 退出连接
>
> 若没有响应，检测一下网线是否接好，重新插拔网线，WiFi 应当关闭，**虚拟机应当设置为桥接模式**。
>
> （2）新打开一个终端（ctrl+alt+t），使用 ssh 指令登录到机器狗的终端，输入 ssh user@10.0.0.34 然后回车，输入登录密码：123456
>
> 进入 robot 终端，画面显示：
>
> （如果显示 ssh: connect to host 10.0.0.34 port 22: No route to host 可能是 WiFi 没有正常连接，建议重新插拔网线）
>
> （3）输入 ssh 指令的端口保持开启，打开新的终端（ctrl+alt+t）
>
> 通过 scp 指令将软件发送至机器狗（前提：两设备处在同一局域网，能够使用 ping 指令与机器狗通信）
>
> scp -r track1.1/ user@10.0.0.34:/home/user/ (若无法 发送，在指令最前边 加上 sudo 指令，输入密码：123456 )
>
> 等待完成即可

注：原文「（10.0.34 是机器狗的 IP 地址）」中 `10.0.34` 为原文笔误，正确地址是 `10.0.0.34`（全文其他位置及截图均为 `10.0.0.34`）。

### 3.1 本章节截图中的关键信息（pdftotext 提取不到）

**【第 5 页截图 1 — Ubuntu 网络托盘菜单】（"Ubuntu 连接方式选择 robot"的具体操作）** 逐字内容：

```
（点击 Ubuntu 右上角网络图标后弹出的菜单）
Ethernet Networks
  Ethernet connection 1
  Disconnect
  standard
  robot            ← 红框选中！要点击这一项
  VPN Connections  >
✓ Enable Networking
  Connection Information
  Edit Connections...
```

即：**在虚拟机内 Ubuntu 桌面右上角点开有线网络菜单 → 在 Ethernet Networks 列表里点击 `robot` 这个连接配置文件**。菜单里同时存在 `Ethernet connection 1`、`standard`、`robot` 三个预置配置，必须选 **`robot`**。

**【第 5 页截图 2 — ping 成功画面】** 逐字内容：

```
user@ubuntu:~$ ping 10.0.0.34
PING 10.0.0.34 (10.0.0.34) 56(84) bytes of data.
64 bytes from 10.0.0.34: icmp_seq=1 ttl=64 time=3.58 ms
64 bytes from 10.0.0.34: icmp_seq=2 ttl=64 time=0.772 ms
64 bytes from 10.0.0.34: icmp_seq=3 ttl=64 time=0.343 ms
64 bytes from 10.0.0.34: icmp_seq=4 ttl=64 time=0.446 ms
64 bytes from 10.0.0.34: icmp_seq=5 ttl=64 time=0.376 ms
64 bytes from 10.0.0.34: icmp_seq=6 ttl=64 time=0.327 ms
64 bytes from 10.0.0.34: icmp_seq=7 ttl=64 time=1.09 ms
64 bytes from 10.0.0.34: icmp_seq=8 ttl=64 time=0.473 ms
64 bytes from 10.0.0.34: icmp_seq=9 ttl=64 time=0.574 ms
```

**【第 6 页截图 1 — ssh 登录成功画面】** 逐字内容：

```
user@ubuntu:~$ ssh user@10.0.0.34
user@10.0.0.34's password:
Welcome to Ubuntu 16.04.6 LTS (GNU/Linux 4.4.86-rt99 x86_64)
 * Documentation:  https://help.ubuntu.com
 * Management:     https://landscape.canonical.com
 * Support:        https://ubuntu.com/advantage

96 packages can be updated.
78 updates are security updates.

Last login: Thu Jul 20 02:07:33 2023 from 10.0.0.32
user@Robot:~$
```

要点：
- 机器狗系统：**Ubuntu 16.04.6 LTS，内核 4.4.86-rt99 x86_64**；
- 机器狗主机名 **`Robot`**，登录后提示符 `user@Robot:~$`；
- `Last login: ... from 10.0.0.32` 说明**当时发起 ssh 的客户端 IP 是 `10.0.0.32`**（另一张截图显示 `from 10.0.0.30`，见下文）——客户端必须在 `10.0.0.x` 网段。

**【第 6 页截图 2 — scp 传输画面】** 逐字内容：

```
user@ubuntu:~$ scp -r track1.1 user@10.0.0.34:/home/user
user@10.0.0.34's password:
form.ui        100% 4648   4.5KB/s   00:00
...（依次传输 ui、cpp、h、pro 等文件，全部 100%）
colorGroup.txt 100% ...
```

---

## 4. 第四部分：程序运行（网络相关）— 逐字原文

PDF 目录标注"四、程序运行 ......... 10"。网络相关部分逐字如下：

> **四、　程序运行**
>
> 1. 使用网线连接电脑与机器狗（后续需要连接机器狗的操作，需一直保持网线连接至机器狗）
>
> 2. 快捷键 ctrl+alt+t 快速开启终端，使用 ssh 指令登录到机器狗的终端（如果显示 ssh: connect to host 10.0.0.34 port 22: No route to host 可能是 WiFi 没有正常连接）
>
> 3.在 Ubuntu 系统内打开终端（ctrl+alt+t），并输入 ssh user@10.0.0.34 然后回车
>
> 输入登录密码：123456
>
> 3.打开机器狗系统内的循迹程序，进入循迹程序的可执行文件所在的目录
>
> cd track1.1/build
>
> 参数 track 为 track 程序的输入参数，根据参数的不同可以执行不同的功能，携带 track 参数执行循迹功能。
>
> （输入指令运行后可以拔掉网线，如果不拔网线，终端会一直打印机器狗的模式状态直至退出）
>
> 命令指令对应任务：sudo ./track stop showImage 调颜色阈值
> 　　　　　　　　sudo ./track track 循迹
> 　　　　　　　　sudo ./track track showImage 蓝色限高杆、橙色停止
> 　　　　　　　　sudo ./track brown showImage 住户识别
> 　　　　　　　　sudo ./track brown green 绿色分岔路（全程代码 1）
> 　　　　　　　　sudo ./track violet red 红色分岔路（全程代码 2）
>
> 注意：住户送快递时，机器狗动作方向不同，如果修改代码，输入的指令也要做出相应的修改。
>
> 4.终止程序运行
>
> 在当前终端界面输入 ctrl+c。（如果拔掉网线后终止命令没反应，要重连网线后才会执行程序终止命令，若想提前终止，可以利用遥控器使机器狗停住）

**第五部分（遥控器）网络相关原文**：

> **五、　遥控器配合操作**
>
> 在颜色校准完成后，使用遥控器控制机器狗走到红线区域，运行循迹程序（成功运行后可选择拔掉网线，也可以不拔），将遥控器 c 键拨到中间即可开始循迹。

**第三部分（调节颜色阈值）中的网络相关原文**：

> 在没有进行 ssh 连接的终端中打开 socketServer 文件夹

> 2. 在 ssh 后远程连接的终端，打开机器狗系统内的循迹程序，进入循迹程序的可执行文件所在的目录，如果没有该目录是因为，没有机器狗系统内没有该程序，需要先将程序发到机器狗系统内，参考文档第二部分（网线始终与机器狗连接）

### 本章节 ssh 截图补充（第 11 页两张 ssh 成功画面）

```
user@ubuntu:~$ ssh user@10.0.0.34
...
Last login: Fri Apr  1 22:05:01 2022 from 10.0.0.30
user@Robot:~$
```

```
user@ubuntu:~$ ssh user@10.0.0.34
...
Last login: Thu Jun 27 17:22:31 2024 from 10.0.0.30
user@Robot:~$
```

**`from 10.0.0.30` 是虚拟机端 IP 的直接证据**——与 socketServer 源码 `nativeIp "10.0.0.30"` 完全吻合。

---

## 5. 附录：可能遇到的问题（网络相关）— 逐字原文

PDF 最后几页「可能遇到的问题」，网络相关条目逐字如下：

> **可能遇到的问题**
>
> **1. ping 不到机器狗**
>
> 答：检查是否和机器狗连接到同一局域网，检查电脑端是否关掉 wifi，检查网线连接是否与机器狗保持连接
>
> **3. 机器狗内部缺少 qt**
>
> 答：用 scp 把虚拟机内对应版本的 qt 发送到机器狗内（发送时注意对应主机和工控机地址的位置）
>
> **4. 机器狗连着网线可以循迹，但是拔掉网线不可以循迹。**
>
> 答：在 track 之前，先执行以下两个命令。
>
> sudo ifconfig lo multicast
>
> sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
>
> nohup ./track &
>
> **5. 错误：./track: /usr/lib/x86_64-linux-gnu/libQt5Network.so.5: version `Qt_5' not found (required by ./track) …**
>
> 原因：qt 版本问题
>
> 解决：
>
> 在虚拟机根目录下会有 Qt 的包
>
> 通过 scp 指令传输到机器狗的 upboard，**首先要保证两者网线连接**
>
> 打开一个终端输入 sudo scp -r Qt/ user@10.0.0.34:/home/user
>
> 等待传输完成后
>
> 在机器狗的 upboard 根目录下打开一个终端
>
> 输入 export LD_LIBRARY_PATH=/home/user/Qt/5.10.0/gcc_64/lib:$LD_LIBARY_PATH
>
> 输入 source ~/.bashrc
>
> **6. 拔掉网线后循迹程序死掉问题**
>
> 解决方法：使用指令：nohup sudo ./track & 来运行循迹程序
>
> **8、机器狗出现问题，电机不响，不能正常遥控，重新进行运动控制的代码下载**
>
> （如果机器狗能用遥控器正常遥控，则无需进行这一步）
>
> 连通机器狗后，在狗的系统里输入 top 指令，查看 mit_ctrl 的进程号：
>
> 使用指令杀死进程：sudo kill 1845(注意进程号都不一样，根据实际情况进行)
>
> 在虚拟机里使用 scp 指令下载代码到机器狗：
>
> scp -r robot-software user@10.0.0.34:/home/user/ (若无法发送，在指令最前边加上 sudo 指令，输入密码：123456 )
>
> 等待完成。
>
> 在狗的终端，输入指令 cd robot-software/build
>
> 最后输入./run_mc.sh ./mit_ctrl
>
> 打印 ESTOP 后，重启机器狗。
>
> **9.注意：所有代码都应放在home目录下！**

**全文 grep 确认**：`桥接` 仅出现 1 次（第 44–45 行「虚拟机应当设置为桥接模式」）；`NAT`、`Host-Only`、`网关`、`子网掩码`、`vmnet`、`指示灯`、`适配器`（文字部分）**均未出现**；唯一的 `ifconfig` 是给**机器狗端**配置组播的 `sudo ifconfig lo multicast`（不是配 IP）。

---

## 6. PDF 截图中的关键网络信息（文字提取不到的部分）

以下内容只存在于 PDF 截图里，`pdftotext` 无法提取，已通过渲染页面逐页目视确认：

| 位置 | 截图内容 | 意义 |
|------|----------|------|
| 第 3 页 | VMware 硬件列表：**网络适配器　桥接模式 (自动)** | **虚拟机必须用桥接模式**的直接证据 |
| 第 3 页 | "此虚拟机可能已被移动或复制"对话框，箭头指向**「我已复制该虚拟机(P)」** | 首次打开镜像时的选择，影响虚拟机网络标识（MAC 等） |
| 第 5 页 | Ubuntu 网络菜单：Ethernet Networks → **红框选中 `robot`** | "Ubuntu 连接方式选择 robot"的具体点击位置；同菜单还有 `Ethernet connection 1`、`standard`，**不能选错** |
| 第 5 页 | `ping 10.0.0.34` 全部通，`ttl=64 time<4ms` | 直连网线的正常延迟水平 |
| 第 6、11 页 | `ssh user@10.0.0.34` 成功，`Last login ... from 10.0.0.30 / 10.0.0.32` | 机器狗端是 `user@Robot`；**客户端 VM 的 IP 曾是 10.0.0.30 / 10.0.0.32** |
| 第 4 页 | 机器狗系统 `Ubuntu 16.04.6 LTS (GNU/Linux 4.4.86-rt99 x86_64)` | 狗上系统版本 |
| 第 3 页 | 打开的是 `Ubuntu 64位.vmx`，文件夹名 `YoBoGo(ubuntu)` | 对应优宝特 YoboGo 机器狗官方镜像 |

---

## 7. 针对六个重点问题的回答

### 7.1 虚拟机网络模式（桥接 / NAT / Host-Only）

**桥接模式（Bridged，自动）**，双重证据：

1. 原文（第 2 部分排障句）：
   > 若没有响应，检测一下网线是否接好，重新插拔网线，WiFi 应当关闭，**虚拟机应当设置为桥接模式**。
2. 第 3 页 VMware 设置截图：
   > 网络适配器　　**桥接模式 (自动)**

**全文没有任何 NAT 或 Host-Only 的字样**。若当前虚拟机是 NAT，机器狗（物理网线对端）根本不在 NAT 虚拟网段内，必然 ping 不通——这很可能就是一直连不上的原因。

设置路径（VMware Workstation 15）：虚拟机 → 设置 → 网络适配器 → 选「桥接模式(B)：直接连接物理网络」（截图中为"桥接模式(自动)"）。注意桥接要桥接到**插网线的那块物理网卡**（编辑 → 虚拟网络编辑器中确认，文档未展开写，但桥接错网卡同样不通）。

### 7.2 Windows 宿主机是否需要配置 IP

**文档从未要求在 Windows 端配置 IP。** 全部相关要求只有两条：

> （1）**关闭电脑 Wifi**，Ubuntu 连接方式选择 robot。

> 若没有响应，检测一下网线是否接好，重新插拔网线，**WiFi 应当关闭**，虚拟机应当设置为桥接模式。

以及排障条目：

> 答：检查是否和机器狗连接到同一局域网，**检查电脑端是否关掉 wifi**，检查网线连接是否与机器狗保持连接

也就是说 Windows 端只做一件事：**关掉 WiFi**（防止默认路由/网络优先级被 WiFi 抢走）。因为虚拟机是桥接模式，Ubuntu 直接以自己的 `10.0.0.x` IP 挂在物理网段上，**不需要** Windows 做任何 IP、桥接、ICS 共享配置。

### 7.3 虚拟机内 Ubuntu 是否需要配置 IP

**需要保证 Ubuntu 使用 `robot` 这个连接配置文件；但文档没有让你手动敲 IP**，因为镜像里已经预置了该配置文件（含静态 IP）。原文：

> （1）关闭电脑 Wifi，**Ubuntu 连接方式选择 robot**。

对应截图：Ubuntu 右上角网络菜单 → Ethernet Networks → 点击 **`robot`**（红框标注）。

推断的 IP 依据：
- socketServer 源码 `#define nativeIp "10.0.0.30"`，并且 `udpSocket->bind(QHostAddress(nativeIp), 30015/30016/30017)` —— **bind 失败则上位机收不到任何图像**，所以 Ubuntu 必须持有 `10.0.0.30`；
- ssh 截图 `Last login ... from 10.0.0.30`（及一次 `10.0.0.32`）；
- 机器狗是 `10.0.0.34`，同网段 `10.0.0.0/24`（子网掩码通常 255.255.255.0，文档未写；无网关要求——直连不需要网关）。

如果镜像里 `robot` 连接丢失/被改过，需要重建：`Edit Connections...` → 有线 → 新建/修改 `robot` → IPv4 设置 → Manual → 地址 `10.0.0.30`、掩码 `255.255.255.0`、网关留空。（此步为依据源码与截图的**推断补全**，文档本身没写。）

### 7.4 连接顺序（先什么后什么）

按原文操作顺序整理：

```
① 安装 VMware Workstation 15，用它打开 Ubuntu 镜像（打开 Ubuntu 64位.vmx）
② 弹出"已移动或复制"对话框 → 点「我已复制该虚拟机(P)」
③ 确认虚拟机设置：网络适配器 = 桥接模式 (自动)
④ 启动虚拟机，登录 Ubuntu（用户 user，密码 123456）
⑤ 关闭电脑 WiFi（Windows 端 + Ubuntu 内都要关）
⑥ Ubuntu 网络菜单选择连接方式 = robot
⑦ 用网线连接电脑与机器狗
⑧ 按下机器狗启动按钮，等待机器狗启动
⑨ ctrl+alt+t 打开终端 → ping 10.0.0.34（通 = 连接成功；Ctrl+C 退出）
⑩ 新开终端 → ssh user@10.0.0.34 → 密码 123456（此终端保持开着）
⑪ 再开终端 → scp -r track1.1/ user@10.0.0.34:/home/user/ 传程序
⑫（调阈值时）再开一个未做 ssh 的终端 → 运行 socketServer 上位机
```

失败时的顺序（原文排障）：

```
ping 不通 → 查网线（重新插拔）→ 查 WiFi 是否关闭 → 查虚拟机是否桥接模式
ssh 报 No route to host → "可能是 WiFi 没有正常连接，建议重新插拔网线"
scp 不通 → "前提：两设备处在同一局域网，能够使用 ping 指令与机器狗通信"
```

**先 ping、后 ssh、再 scp**——这是文档规定的严格顺序；scp 前提是 ping 通。

### 7.5 网口、网线、指示灯的描述

**网线**（全部原文）：

> 使用网线连接电脑与机器狗，按下机器狗启动按钮，等待机器狗启动

> 若没有响应，检测一下网线是否接好，重新插拔网线，WiFi 应当关闭，虚拟机应当设置为桥接模式。

> （如果显示 ssh: connect to host 10.0.0.34 port 22: No route to host 可能是 WiFi 没有正常连接，建议重新插拔网线）

> （3）…通过 scp 指令将软件发送至机器狗（前提：两设备处在同一局域网，能够使用 ping 指令与机器狗通信）

> 1. 使用网线连接电脑与机器狗（后续需要连接机器狗的操作，需一直保持网线连接至机器狗）

> 参考文档第二部分（网线始终与机器狗连接）

> （输入指令运行后可以拔掉网线，如果不拔网线，终端会一直打印机器狗的模式状态直至退出）

> 在当前终端界面输入 ctrl+c。（如果拔掉网线后终止命令没反应，要重连网线后才会执行程序终止命令，若想提前终止，可以利用遥控器使机器狗停住）

> 4. 机器狗连着网线可以循迹，但是拔掉网线不可以循迹。

> 通过 scp 指令传输到机器狗的 upboard，首先要保证两者网线连接

> 6. 拔掉网线后循迹程序死掉问题　解决方法：使用指令：nohup sudo ./track & 来运行循迹程序

**指示灯**：**全文 18 页没有任何关于指示灯、网口灯、LED 状态的描述**。排障手段只有"重新插拔网线"和"看 ping/ssh 输出"。

**网口**：未指定用电脑的哪个网口（笔记本就用自带 RJ45；台式机插主板网口）。桥接模式下虚拟机借用的就是这一个物理网口。

### 7.6 VMware 网络适配器的设置步骤

文档明确给出的步骤/状态：

1. **安装** VMware Workstation 15（第 2 页）；
2. **打开镜像**：文件 → 打开 → 选 `Ubuntu 64位.vmx`（第 3 页截图）；
3. **"此虚拟机可能已被移动或复制"对话框 → 点「我已复制该虚拟机(P)」**（第 3 页截图，原文配文"若出现如下界面，直接点击回车:"——以截图箭头为准指向"我已复制"）；
4. **硬件确认：网络适配器 = 桥接模式 (自动)**（第 3 页截图）；
5. **排障时复述：「虚拟机应当设置为桥接模式」**（第 2 部分原文）。

文档**没有**出现以下内容（已全文检索确认）：NAT 设置、Host-Only、VMnet0/VMnet8、虚拟网络编辑器对话框、桥接绑定哪块物理网卡、Windows IP 配置、Ubuntu 手动填 IP/网关/子网掩码、DHCP。

---

## 8. socketServer 源码中的网络配置

文件：`2026robocup中型组比赛资料10/socketServer/`

### 8.1 `widget.cpp`（全部 IP 相关）

```cpp
#include <QNetworkInterface>
#define nativeIp "10.0.0.30"     // 本机（Ubuntu 虚拟机）IP —— 第 4 行
#define goalIp "10.0.0.34"       // 目标（机器狗）IP   —— 第 5 行
```

绑定与收发（`initSocket()`，第 93–103 行）：

```cpp
void Widget::initSocket(){
    udpSocket=new QUdpSocket;
    udpSocket2=new QUdpSocket;
    udpSocket3=new QUdpSocket;
    udpSocket->bind(QHostAddress(nativeIp), 30015);   // 原始图像
    udpSocket2->bind(QHostAddress(nativeIp), 30016);  // 二值化图像
    udpSocket3->bind(QHostAddress(nativeIp), 30017);  // 阈值反馈
    ...
}
```

向机器狗发送（滑块/按钮回调，多处相同模式）：

```cpp
udpSocket->writeDatagram(colorThreadhold, QHostAddress(goalIp), 8000);
// 保存按钮、确定按钮同样发往 goalIp:8000
```

**含义**：
- 上位机（socketServer）**必须**运行在 IP 为 `10.0.0.30` 的机器上——`bind()` 指定了地址，IP 不是 `10.0.0.30` 时绑定失败，收不到机器狗推来的视频；
- 机器狗端往 `10.0.0.30` 的 UDP `30015/30016/30017` 推图像/阈值，监听 `UDP 8000` 接收指令；
- 端口汇总：`30015/30016/30017`（狗→上位机），`8000`（上位机→狗）；
- 另外循迹程序用 **LCM**（组播）通信，故拔网线循迹需要 `sudo ifconfig lo multicast` + `sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo`（在狗端执行）。

### 8.2 `widget.h`

仅声明 `QUdpSocket *udpSocket/udpSocket2/udpSocket3;` 等，**无额外网络配置**（IP 全部在 `widget.cpp` 的宏里）。

### 8.3 `socketServer.pro`

```
QT += core gui network
```

依赖 `network` 模块；OpenCV 头/库路径为 `/usr/local/include/opencv5/`、`/usr/local/lib/`。**无 IP 相关配置**。

### 8.4 `main.cpp` / `widget.ui`

无任何网络配置（`main.cpp` 仅启动 `Widget`；`widget.ui` 为阈值滑块界面）。

---

## 9. 综合：推荐的完整连接流程

把 PDF 原文与源码证据合并后的**可执行清单**（打 ✓ 项均为原文要求）：

### A. VMware 层（一次性）

- [ ] 安装 VMware Workstation 15 ✓
- [ ] 7zip 解压镜像包，打开 `Ubuntu 64位.vmx` ✓
- [ ] "已移动或复制"对话框 → **我已复制该虚拟机(P)** ✓
- [ ] 虚拟机设置 → 网络适配器 → **桥接模式（自动）** ✓（截图证据）
- [ ] 若有多网卡：虚拟网络编辑器里把桥接绑定到**插狗网线的物理网卡**（文档未写，常识补全）

### B. Windows 层

- [ ] **关闭 Windows WiFi**（原文两处强调"电脑端关掉 wifi"）✓
- [ ] 无需配置任何 IP（文档无此要求）✓

### C. Ubuntu 虚拟机内

- [ ] 登录（user / 123456）✓
- [ ] 关闭 Ubuntu 内 WiFi ✓
- [ ] 右上角网络菜单 → Ethernet Networks → **选 `robot`** ✓（截图红框）
- [ ] （自检）`ip addr show` 应看到 `10.0.0.30/24`（源码 + ssh 截图推断；若没有，检查 robot 配置文件）

### D. 连接顺序

1. 网线插入 电脑 ↔ 机器狗 ✓
2. 按下机器狗启动按钮，等待启动 ✓
3. `ping 10.0.0.34` 通为止（不通：重插网线 / 查 WiFi / 查桥接）✓
4. `ssh user@10.0.0.34`（密码 123456），终端保持开启 ✓
5. 新终端 `scp -r 代码/ user@10.0.0.34:/home/user/` ✓
6. 调阈值时：另开终端运行 socketServer（要求本机 = 10.0.0.30）

### E. 排障速查（原文）

| 症状 | 原文处理 |
|------|----------|
| ping 不通 | 检查同一局域网、电脑端关掉 wifi、网线保持连接；重插网线；**虚拟机设为桥接模式** |
| ssh `No route to host` | 可能是 WiFi 没有正常连接，建议重新插拔网线 |
| scp 不通 | 前提：两设备处在同一局域网，能 ping 通 |
| 拔网线不能循迹 | 狗端：`sudo ifconfig lo multicast`、`sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo`、`nohup ./track &` |
| 拔网线后进程死 | `nohup sudo ./track &` |
| 终止命令无效（已拔网线） | 重连网线后再 ctrl+c，或用遥控器停狗 |

---

## 10. 目录检查结果

| 路径 | 状态 |
|------|------|
| `2026robocup中型组比赛资料10/中国机器人大赛调试说明文档.pdf` | 18 页，已全文提取 + 逐页截图目视核对 |
| `2026robocup中型组比赛资料10/中国机器人大赛调试说明文档(1).pdf` | 与上者 MD5 相同，内容完全一致 |
| `2026robocup中型组比赛资料10/socketServer/` | `main.cpp`、`widget.cpp`、`widget.h`、`widget.ui`、`socketServer.pro` 全部读取；IP 见第 8 节 |
| `2026robocup中型组比赛资料10/比赛代码/` | **空目录**（已确认，无任何文件） |
| `2026robocup中型组比赛资料10/运动控制代码/` | **空目录**（已确认，无任何文件） |

---

*报告生成：基于 pdftotext 全文提取 + 300dpi 页面渲染逐页目视核对（重点核对第 3、5、6、11 页截图），grep 全文网络关键词（桥接/NAT/IP/网关/子网/ifconfig/vmnet/网卡/适配器/WiFi/网线/局域网/ssh/ping/scp/10.0.0/robot/连接方式/网络）。*
