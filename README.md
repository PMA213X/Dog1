# Dog1

四足机器狗（YoboGo-10S / Mini Cheetah）项目：实机控制、比赛资料、MIT Cheetah 控制源码与 Webots 仿真。

## 一、目录说明

| 路径 | 内容 |
|---|---|
| `2026robocup中型组比赛资料10/` | 2026 比赛资料：调试说明 PDF 两份（`中国机器人大赛调试说明文档.pdf` 及其副本，文内标题《中型机器狗比赛调试说明》）、`socketServer/`（Qt 颜色阈值上位机，UDP 下发阈值）、`比赛代码/` 与 `运动控制代码/`（空目录） |
| `YoboGo-control/` | YoboGo-10S 实机控制：`robot-software/`（MIT Cheetah HAL / mit_ctrl 运动控制）、`track1.6/`（视觉循迹 + 任务状态机，Qt + LCM）、`YoboGo-10S使用说明书(开源).docx` |
| `Cheetah-Software/` | MIT Cheetah-Software 完整源码（MPC + WBC），`user/MIT_Controller` 对应 mit_ctrl |
| `quadruped_ctrl/` | 另一套四足控制代码（ROS 包 + `standalone_simulation.py` 独立仿真） |
| `webots-sim/` | Webots R2025a 仿真：`worlds/`（`mini_cheetah.wbt` 步态试验台、`msl_match.wbt` MSL 比赛场地）、`protos/`（MslField / MslGoal / MslBall / MslEnvironment）、`controllers/mini_cheetah_controller/`（PD 站立 + trot）、`urdf/`（参考模型） |
| `docs/` | 项目文档：`features/`（规则、代码分析、教材笔记等）、`architecture/`、`api/`、`changes/`、`compose/spec/` |
| `scripts/` | 机器狗连接 / 部署 bash 脚本（`connect_check.sh`、`scp_to_robot.sh` 等） |
| `四足仿生机器人基本原理及开发教程（电子版课件）/` | 配套教材 PPT 课件（第 1–13 章） |
| `extract_pptx.py` | PPTX 文本提取脚本 |

## 二、仿真运行方法

### 依赖

Webots R2025a（apt 安装，路径 `/usr/local/webots`）：

```bash
webots --version
# 或：/usr/local/webots/webots --version
```

### 编译与运行

```bash
# 1) 编译控制器（可选，Webots 首次打开也会自动编译）
cd webots-sim/controllers/mini_cheetah_controller
export WEBOTS_HOME=/usr/local/webots
make

# 2a) 打开步态试验台（空白场地，PD 站立 + trot）
/usr/local/webots/webots webots-sim/worlds/mini_cheetah.wbt

# 2b) 打开 MSL 比赛场地（22×14 m 足球场、球门、球、挡板、旗杆）
/usr/local/webots/webots webots-sim/worlds/msl_match.wbt
```

### 按键表

先点击 3D 视图获得焦点，再按：

| 按键 | 功能 |
|---|---|
| `S` | 站立 |
| `T` | trot |
| `R` | 复位 |

### MSL 场景元素

| 项目 | 规格 |
|---|---|
| 场地 | 22 × 14 m |
| 标线宽 | 12.5 cm |
| 球门 | 2.44 × 2.0 m |
| 球 | FIFA 5 号（⌀0.22 m / 0.43 kg） |
| 安全边界 | 24 × 16 m |
| 旗杆 | 6 根 |

来源：`docs/features/robocup-midsize-rules.md`。

更多细节见 `docs/features/webots-sim-guide.md`（安装、控制器说明、常见问题）。

### 连接实机

见 `docs/features/robot-connection-guide.md`（机器狗 `10.0.0.34`，上位机 `10.0.0.30`）。
