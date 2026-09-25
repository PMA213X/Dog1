---
feature: manual-parkour
status: designed
updated: 2026-09-25
branch: master
---

# 手动遥控跑酷与障碍训练世界

## Report

## [S1] Problem

现有 Webots 控制器（`mini_cheetah_controller`、`ball_detector`）只支持固定 trot 慢跑与站立切换（键盘 S/T/R），步态速度是预设常量，**无法手动实时控制跑动、转向与跳跃**；世界文件也只有平地步态台（`mini_cheetah.wbt`）与 MSL 球场（`msl_match.wbt`），**缺少带台阶/箱台/斜坡/窄道/土坑的障碍训练场地**。

需要补齐三块能力：

- (a) **实时手动遥控**：键盘 / 手柄实时改写速度指令；
- (b) **预设跳跃**：一键触发的四阶段跳跃动作；
- (c) **障碍跑酷世界**：按 YoboGo 尺度改造开源布局的训练场地。

## [S2] Design

### S2.1 输入映射

**键盘**（Webots Keyboard API，`wb_keyboard_enable(100)` 采样）：

| 键 | 语义 | 输出分量 |
|---|---|---|
| W / S | 前进 / 后退 | vx ± |
| A / D | 左 / 右平移 | vy ± |
| Q / E | 左转 / 右转 | wz ± |
| Space | 跳 | 触发 S2.3 预设跳跃状态机 |
| Shift | 快跑档 | 步频×1.6、步幅×1.3（见 S2.2） |
| R | 复位 | 速度清零，回到站立位姿 |
| Esc | 停 | 速度清零；跳跃进行中 = 打断当前跳跃 |

**手柄**（Linux joystick API，设备固定 `/dev/input/js0`）：

| 输入 | 语义 | 输出分量 |
|---|---|---|
| 左摇杆 X / Y | 平移 | vx / vy |
| 右摇杆 X | 转向 | wz |
| A 钮 | 跳 | 触发 S2.3 预设跳跃状态机 |
| B 钮 | 复位 | 速度清零，回到站立位姿 |

契约细节：

- 键盘键值 → 目标速度做**斜率限制**逼近，松键后线性回零；同一分量的 W/S、A/D、Q/E 互斥，冲突时取后采样键。
- 手柄摇杆带**死区 ±0.1**，轴值归一化后线性映射到速度上限（S2.2）；钮号按 Linux js0 惯例 A=0 / B=1，以常量表集中定义便于不同手柄改键。
- 手柄只做 Linux `js0`（见 [S3]）；设备缺失时不阻塞键盘路径，仅打印一次提示。
- 键盘与手柄可同时在线：各分量取**非零优先，同分量键盘优先**。

### S2.2 速度指令 → 步态

控制器向 trot 步态输出速度指令：

```text
v_des = [vx, vy, wz]    // vx, vy 单位 m/s；wz 单位 rad/s
```

| 档位 | 触发 | 步态参数 |
|---|---|---|
| 正常 trot | 默认 | 基准步频 / 步幅 |
| 快跑 | Shift（或手柄满偏，可选） | 步频 **×1.6**、步幅 **×1.3** |

速度上限（源：YoboGo yaml `des_dp_max` / `des_dtheta_max`，`YoboGo-control/robot-software/config/mc-mit-ctrl-user-parameters.yaml`）：

| 分量 | 上限 | 来源 |
|---|---|---|
| vx | **1.0** m/s | `des_dp_max[0]` = 1.0（yaml:92） |
| vy | **0.5** m/s | `des_dp_max[1]` = 0.5（yaml:92） |
| wz | **2.0** rad/s | `des_dtheta_max`（yaml:93） |

指令流：`v_des` 先按上表**限幅**，再经斜率限制，最后驱动足端轨迹相位；Esc / R / 复位路径一律把 `v_des` 清零。

### S2.3 预设跳跃（4 阶段时间轴）

Space（或手柄 A 钮）触发后，按固定时间轴执行：

| 阶段 | 时长 | 动作 |
|---|---|---|
| 1 下蹲 | **0.15 s** | 站立高 `stand_height` **0.26 → 0.15** m（质心下沉蓄力） |
| 2 猛蹬 | **0.08 s** | 四腿伸直，关节力矩拉至上限（蹬地） |
| 3 空中收腿 | **0.25 s** | 收腿，准备着地 |
| 4 着地缓冲 | **0.20 s** | 软 PD 缓冲，回到站立 / 原步态 |

- 全程总时长 **0.68 s**；**任意阶段可被 Esc 打断**（取消剩余阶段，进入安全站立并清零 `v_des`）。
- 触发条件：非跳跃态才接受触发；跳跃中再次按 Space 忽略。
- 站立高基准 0.26 m 取自 yaml `des_p` z（yaml:87）。

### S2.4 障碍世界 `webots-sim/worlds/parkour.wbt`

从 GitHub 开源 Webots 项目**借布局**（候选：`bthwthw/Quadruped-Robot`、`cyberbotics/webots` samples 的 `obstacles.wbt`、`robotbenchmark/obstacle_avoidance`），改造为 YoboGo 尺度的跑酷路线。障碍物规格：

| 障碍 | 规格 |
|---|---|
| 台阶 | 高 **0.08–0.15** m（逐级递增） |
| 箱台 | **0.3 × 0.3 × 0.2** m |
| 斜坡 | **15°** 与 **25°** 各一 |
| 窄道 | 宽 **0.4** m |
| 土坑 | 深 **0.3–0.5** m，跨距可跳（配合 S2.3） |

资源与装配约束：

- **必须引用仓库内自建 PROTO 或 Webots 官方 EXTERNPROTO，不依赖网络**下载（与 `msl_match.wbt` 的 `EXTERNPROTO "../protos/..."` 约定一致）。
- 机器人使用 `webots-sim/tools/gen_yobogo_robot.py` 生成的**同款几何**（yobogo_10s，12 DOF，`endPoint==anchor` 约定），站立高对齐 `des_p` z=0.26，出生在跑酷路线起点。
- `WorldInfo.basicTimeStep` 取 4 ms（250 Hz，与既有世界一致）；路线沿单一方向前进，便于重复训练。

### S2.5 控制器

新增 `webots-sim/controllers/manual_control/`（**C++，Webots C API，中文注释**）：

- 实现方式：拷贝 `ball_detector` 的 **PD 站立 + trot 底座**（`KP=[3,3,3]`、`KD=[1,0.2,0.2]`），叠加 S2.1 快捷键/手柄输入、S2.2 速度指令链、S2.3 跳跃状态机。
- 模式状态机：`STAND / TROT / JUMP(1..4) / STOP / RESET`，Esc 全局停、R 复位、Space 进跳跃。
- 绑定：`parkour.wbt` 中 Robot 的 `controller` 字段指向 `manual_control`。
- **不改动** `ball_detector` 与 `mini_cheetah_controller`（保持原样可独立运行）。

## [S3] Out of Scope

- RL / 训练算法、强化学习框架（跑酷仅做手动训练场地，不做策略学习）。
- 视觉 / 球检测（已有 `ball_detector`，不合并）。
- 实机手柄驱动（只做 Linux `js0`，不做 Windows/Xbox/蓝牙协议适配）。
- 多机器人（单机器人单跑酷路线）。

## Tasks

- [ ] T1: `webots-sim/worlds/parkour.wbt` 障碍世界 — 台阶/箱台/斜坡/窄道/土坑 + gen_yobogo 同款机器人 — acceptance: Webots R2025a 可打开，含台阶/箱台/斜坡/窄道/土坑，机器人能站立 (covers: S2.4)
- [ ] T2: `manual_control` 键盘遥控跑 — WASD/QE 实时改 `v_des`，Shift 快跑 — acceptance: WASD/QE 实时改速度，Shift 快跑 (covers: S2.1, S2.2, S2.5)
- [ ] T3: 预设跳跃 — 空格触发 4 阶段时间轴，Esc 可打断 — acceptance: 空格触发 4 阶段，离地 >5cm 并落地站稳 (covers: S2.3)
- [ ] T4: 手柄 `/dev/input/js0` — 左右摇杆控方向，A 钮跳、B 钮复位 — acceptance: 左右摇杆能控方向，A钮跳 (covers: S2.1)
- [ ] T5: 文档 — `docs/features/webots-sim-guide.md` 增补 parkour 章节 + 键盘/手柄操作说明 — acceptance: guide 增补 parkour 章节 + 操作说明 (covers: S2.5)
