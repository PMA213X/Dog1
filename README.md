# Dog1

四足机器狗（YoboGo-10S / Mini Cheetah）项目：实机控制、比赛资料、MIT Cheetah 控制源码、Webots 仿真、手动遥控跑酷与 PPO 强化学习行走训练。

## 一、目录说明

| 路径 | 内容 |
|---|---|
| `2026robocup中型组比赛资料10/` | 2026 比赛资料：调试说明 PDF 两份（`中国机器人大赛调试说明文档.pdf` 及其副本，文内标题《中型机器狗比赛调试说明》）、`socketServer/`（Qt 颜色阈值上位机，UDP 下发阈值）、`比赛代码/` 与 `运动控制代码/`（空目录） |
| `YoboGo-control/` | YoboGo-10S 实机控制：`robot-software/`（MIT Cheetah HAL / mit_ctrl 运动控制）、`track1.6/`（视觉循迹 + 任务状态机，Qt + LCM）、`YoboGo-10S使用说明书(开源).docx` |
| `Cheetah-Software/` | MIT Cheetah-Software 完整源码（MPC + WBC），`user/MIT_Controller` 对应 mit_ctrl |
| `quadruped_ctrl/` | 另一套四足控制代码（ROS 包 + `standalone_simulation.py` 独立仿真） |
| `webots-sim/` | Webots R2025a 仿真：`worlds/`（4 个世界文件，见第三章）、`controllers/`（4 个控制器，见第三章）、`protos/`（MslField / MslGoal / MslBall / MslEnvironment / parkour）、`rl/`（PPO 训练套件，见第四章）、`tools/`（`gen_yobogo_robot.py` 机器人单一真源、`build_parkour_world.py`）、`urdf/`（参考模型） |
| `docs/` | 项目文档：`features/`（规则、代码分析、教材笔记、**`rl-training-guide.md` 训练完全指南**等）、`architecture/`、`api/`、`changes/`、`compose/spec/` |
| `checkpoints/` | PPO 训练 checkpoint（`ppo_walk_final` / `p3_turn_final` / `p4_stairs_final` 等） |
| `runs/` | TensorBoard 训练日志 |
| `logs/` | 训练日志、巡检记录、`reports/` 训练报告 |
| `videos/` | 回放视频（含第三人称视角） |
| `scripts/` | 机器狗连接 / 部署 bash 脚本（`connect_check.sh`、`scp_to_robot.sh`、`run_p4.sh` 等） |
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

---

## 三、Webots 仿真操作详解

### 3.1 世界文件（4 个）

| 世界文件 | 用途 | 说明 |
|---|---|---|
| `webots-sim/worlds/mini_cheetah.wbt` | **步态试验台** | 空白平地，PD 站立 + trot 快速验证步态 |
| `webots-sim/worlds/msl_match.wbt` | **MSL 球场** | 22×14 m 足球场、球门、球、挡板、旗杆（4 个 Msl PROTO） |
| `webots-sim/worlds/parkour.wbt` | **跑酷障碍** | 台阶/斜坡/窄道/土坑/箱台障碍路线（详见第五章） |
| `webots-sim/worlds/parkour_dev.wbt` | **训练用平地** | RL P1–P3 训练场地（平地；RL 运行时自动改写为 `.parkour_dev_rl.wbt` 副本） |

```bash
# 启动任一世界
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  /usr/local/webots/webots --mode=realtime webots-sim/worlds/parkour.wbt
```

> 注意：`--mode=realtime` 中 `realtime` **无连字符**；`--mode=real-time` 会报 `invalid option`。

### 3.2 控制器（4 个）

| 控制器 | 语言 | 功能 |
|---|---|---|
| `webots-sim/controllers/mini_cheetah_controller/` | C++ | **PD 站立 + trot**：S/T/R 切换站立/步态/复位，力矩限幅 15 N·m |
| `webots-sim/controllers/ball_detector/` | C++ | **球检测**：PD+trot 底座 + 橙色球像素阈值检测（`r>150 && 40<g<160 && b<90 ...`），PPM 存 `/tmp/ball_detect/` |
| `webots-sim/controllers/manual_control/` | C++ | **手动遥控**：WASD/QE 实时速度指令 + Shift 快跑 + 空格预设跳跃（详见第五章） |
| `webots-sim/controllers/rl_agent/rl_agent.py` | Python | **RL 策略**：Webots Supervisor，经 TCP 桥与 `webots-sim/rl/walk_env.py` 通信，执行 `q_des = q_stand + a·scale` |

编译 C++ 控制器：

```bash
export WEBOTS_HOME=/usr/local/webots
cd webots-sim/controllers/mini_cheetah_controller && make
cd ../ball_detector && make
cd ../manual_control && make
```

### 3.3 键盘操作

**通用（`mini_cheetah_controller` / `ball_detector`）** — 先点击 3D 视图获得焦点：

| 按键 | 功能 |
|---|---|
| `S` | 站立 |
| `T` | trot（重置步态相位） |
| `R` | 复位到站立 |

**手动遥控（`manual_control`，parkour 世界）**：

| 按键 | 功能 |
|---|---|
| `W` / `S` | 前进 / 后退（vx ±） |
| `A` / `D` | 左 / 右平移（vy ±） |
| `Q` / `E` | 左转 / 右转（wz ±） |
| `空格` | 跳（4 阶段预设跳跃：下蹲 0.15s → 蹬地 0.08s → 空中 0.25s → 着地 0.20s） |
| `Shift` | 快跑档（步频 ×1.6、步幅 ×1.3） |
| `R` | 复位（速度清零，回站立） |
| `Esc` | 停（速度清零；跳跃中 = 打断跳跃） |

> 速度上限：vx 1.0 m/s、vy 0.5 m/s、wz 2.0 rad/s（源自 YoboGo yaml `des_dp_max`）。
> 键盘无反应时先点 3D 视图；`wb_keyboard_get_key()` 无键返回 `-1`，判 Shift 前必须先判 `key < 0`。

### 3.4 视角切换（5 个 Viewpoint）

`msl_match.wbt` 与 `parkour.wbt` 各内置 5 个 Viewpoint，GUI 菜单 `视图 → 视角` 切换：

| # | `msl_match.wbt` | `parkour.wbt` |
|---|---|---|
| 1 | 第三人称·全景（整块球场） | 第三人称·全景（整条障碍路线） |
| 2 | 第三人称·跟随机器人（`follow ON`） | 第三人称·跟随机器人（`follow ON`） |
| 3 | 球门区近景 | 起点近景（出发姿态 + 第一段台阶） |
| 4 | 俯视全景（战术视角） | 终点俯视（金字塔与终点区） |
| 5 | 贴地低角度（等效机器人相机） | 侧面低角度（台阶 / 斜坡剖面） |

鼠标操作：左键旋转 / 滚轮缩放 / 右键平移。

### 3.5 NVIDIA 启动命令

```bash
# 独显渲染启动（推荐，NVIDIA Prime 双显卡笔记本）
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  /usr/local/webots/webots webots-sim/worlds/msl_match.wbt

# 后台启动并保持实时模式
nohup __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  /usr/local/webots/webots --mode=realtime webots-sim/worlds/parkour.wbt \
  > logs/webots.log 2>&1 &
```

### 3.6 常见坑

| 坑 | 解决 |
|---|---|
| **`ffmpeg x11grab` 截屏全黑**（NVIDIA 渲染下） | 改用 ImageMagick：`import -window <wid> out.png`，wid 取自 `xwininfo -root -tree \| grep webots-bin` |
| **`timeout` 误报崩溃** | `timeout 25 webots ...` 结束后打印"进程崩溃"常是 SIGTERM 误报；`INFO: ... Terminating.` 之后的警告可忽略 |
| **`--no-rendering` 禁用 Camera** | 该旗标会禁用 Camera 传感器，需要相机图像时**不能加** |
| 键盘无反应 | 先点击 3D 视图获得焦点（焦点在编辑器/Console 时键盘进不了控制器） |
| `Motor not found: xxx_motor` | 控制器未编译或与世界不匹配；`export WEBOTS_HOME=/usr/local/webots && make` |
| 多开 Webots 抢端口 | RL TCP 桥必须错开 `RL_BRIDGE_PORT`（11451 / 11452 / 11453） |
| `--batch` 静默退出 | Wayland 下 `--batch --mode=pause` 组合会退出；用 `--mode=pause` 不加 `--batch` |
| 相机预览黑块 | 3D 视口品红角标黑块是相机预览浮窗；`视图 → Optional Rendering → Camera` 可关 |

---

## 四、RL 训练（重点）

> **完整训练指南（原理、观测/奖励逐维解释、超参数、曲线解读、问题排查）：
> [`docs/features/rl-training-guide.md`](docs/features/rl-training-guide.md)**
> 本章是速查版。

### 4.1 四阶段课程

| 阶段 | 目标 | 步数 | 奖励 `ep_rew_mean` | 模型 |
|---|---|---|---|---|
| **P1 站立** | 不摔倒、保持直立 | 200k | 643 → 1,220（含 P2） | `ppo_walk_final.zip` |
| **P2 行走** | 稳定向前移动 | （含于 P1 的 200k） | 净前进 +220 | `ppo_walk_final.zip` |
| **P3 转向** | 对准目标航向 | 300k | 779 → 1,870 | `p3_turn_final.zip` |
| **P4 台阶** | 爬台阶 + 转向 | 200k | → **2,100** | `p4_stairs_final.zip` |

**最终成绩：奖励 643 → 2,100，200k + 300k + 200k 步，`ep_len_mean` 全程满额零摔倒。**

### 4.2 环境

Python **3.11** venv **`/tmp/rl_venv`**（勿放 NTFS 盘）：

| 组件 | 版本 |
|---|---|
| Python | 3.11.16 |
| PyTorch | 2.14.0+cpu |
| Stable-Baselines3 | 2.9.0 |
| Gymnasium | 1.3.0 |
| 观测维度 | 42（walk）/ 45（turn）/ 51（stairs） |
| 动作维度 | 12（关节角，范围 [-1, 1]） |
| 算法 | PPO，`MlpPolicy` 256×256，`n_steps=2048`，`batch=256`，`lr=3e-4` |

```bash
python3.11 -m venv /tmp/rl_venv
/tmp/rl_venv/bin/pip install torch --index-url https://download.pytorch.org/whl/cpu
/tmp/rl_venv/bin/pip install stable-baselines3 gymnasium tensorboard imageio imageio-ffmpeg
```

### 4.3 训练命令（完整可复制）

```bash
cd /run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1

# ---- P1 站立（200k 步，42 维）----
RL_BRIDGE_PORT=11451 PYTHONUNBUFFERED=1 setsid nohup /tmp/rl_venv/bin/python \
  webots-sim/rl/train_ppo.py \
  --total-steps 200000 --tag phase1_stand --ckpt-prefix phase1_stand \
  --checkpoint-interval 10000 --eval-interval 20000 --eval-episodes 10 \
  < /dev/null > logs/train_p1.log 2>&1 &

# ---- P2 行走（200k 步，42 维，从 P1 热启动）----
RL_BRIDGE_PORT=11451 PYTHONUNBUFFERED=1 setsid nohup /tmp/rl_venv/bin/python \
  webots-sim/rl/train_ppo.py \
  --total-steps 200000 --tag phase2_walk --ckpt-prefix ppo_walk \
  --resume checkpoints/phase1_stand_final.zip \
  --checkpoint-interval 20000 --eval-interval 20000 --eval-episodes 3 \
  < /dev/null > logs/train_p2.log 2>&1 &

# ---- P3 转向（300k 步，45 维，从 P2 热启动）----
RL_BRIDGE_PORT=11452 PYTHONUNBUFFERED=1 setsid nohup /tmp/rl_venv/bin/python \
  webots-sim/rl/train_ppo.py \
  --total-steps 300000 --tag phase3_turn --ckpt-prefix p3_turn \
  --resume checkpoints/ppo_walk_final.zip \
  --checkpoint-interval 20000 --eval-interval 20000 --eval-episodes 3 \
  < /dev/null > logs/train_p3.log 2>&1 &

# ---- P4 台阶（200k 步，51 维，从 P3 热启动；启动器会等 P3 结束）----
setsid nohup bash scripts/run_p4.sh < /dev/null > logs/p4_waiter.log 2>&1 &
```

> `--tag` 自动选择环境类型与奖励权重（`phase*_stand/walk→walk(42)`、
> `*_turn→turn(45)`、`*_stairs→stairs(51)`）。
> 后台长任务必须 `setsid nohup ... < /dev/null &`。
> 训练活性三查：日志字节增长率 / `ps -o pcpu` / checkpoint mtime
> （`pgrep` 活着 ≠ 在训练）。

### 4.4 模型文件

`checkpoints/` 下最终模型（过程 checkpoint 每 1～2 万步一个）：

| 文件 | 阶段 | 累计步数 |
|---|---|---|
| `checkpoints/ppo_walk_final.zip` | P1+P2 站立行走 | 200,704 |
| `checkpoints/p3_turn_final.zip` | P3 转向 | 301,056 |
| `checkpoints/p4_stairs_final.zip` | P4 台阶 | 200,960 |

### 4.5 监控

```bash
# 状态快照（生成 logs/STATUS.md：阶段/步数/步速/ETA/资源/错误）
python3 webots-sim/rl/monitor.py

# TensorBoard 曲线（http://localhost:6006）
tensorboard --logdir runs/
# 关注 rollout/ep_rew_mean（升）、rollout/ep_len_mean（满 1000）、train/explained_variance
```

### 4.6 回放

```bash
# 回放并录视频（第一人称 / 环境相机）
python3 webots-sim/rl/play.py --model checkpoints/ppo_walk_final.zip --video out.mp4

# 第三人称视频（后上方跟随相机；转向/台阶模型加 --turn）
python3 webots-sim/rl/play_third_person.py \
  --model checkpoints/p3_turn_final.zip --turn \
  --video videos/walk_p3_third_person.mp4

# 其它常用参数
python3 webots-sim/rl/play.py --model <zip> --episodes 3 --render --max-steps 500
```

---

## 五、手动遥控跑酷

### 5.1 打开障碍世界

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  /usr/local/webots/webots webots-sim/worlds/parkour.wbt
```

控制器为 `manual_control`（键盘遥控）。开发调试可用平地世界
`webots-sim/worlds/parkour_dev.wbt`。

### 5.2 障碍清单（`parkour.wbt`）

沿路线前进方向依次布置（布局参数借自开源 DDTRobot `stairs.wbt`，按 YoboGo 尺度重摆）：

| 障碍 | 规格 | 技巧 |
|---|---|---|
| **台阶** | 高 0.08–0.15 m（逐级递增） | 正对台阶慢速上，保持俯仰 |
| **斜坡** | 15° 与 25° 各一 | 上坡保持速度，下坡侧低速 |
| **窄道** | 宽 0.4 m | 对中直行，避免横移 |
| **土坑** | 深 0.3–0.5 m，跨距可跳 | 空格跳跃配合 Shift 助跑 |
| **箱台** | 0.3 × 0.3 × 0.2 m（含金字塔组） | 上台阶后站稳再下 |

### 5.3 按键操作

| 按键 | 功能 |
|---|---|
| `W` / `S` | 前进 / 后退 |
| `A` / `D` | 左 / 右平移 |
| `Q` / `E` | 左转 / 右转 |
| `空格` | **跳**（预设 4 阶段：下蹲 0.15s → 蹬地 0.08s → 空中收腿 0.25s → 着地缓冲 0.20s） |
| `Shift` | **快跑**（步频 ×1.6、步幅 ×1.3） |
| `R` | 复位（速度清零，回站立） |
| `Esc` | 停（速度清零；跳跃进行中 = 打断跳跃） |

手柄（可选，`/dev/input/js0`）：左摇杆平移、右摇杆 X 转向、A 跳、B 复位。

详见 `docs/compose/spec/manual-parkour.md`（输入映射、速度链、跳跃时间轴、障碍规格）。

---

## 附：文档索引

| 文档 | 内容 |
|---|---|
| `docs/features/rl-training-guide.md` | **RL 训练完全指南**（原理、观测/奖励逐维、四阶段命令、问题排查、曲线解读） |
| `docs/features/webots-sim-guide.md` | Webots 安装、控制器说明、按键、MSL 场地、常见问题 |
| `docs/compose/spec/manual-parkour.md` | 手动遥控跑酷设计与规格 |
| `docs/compose/spec/msl-match-field.md` | MSL 球场场景规格 |
| `docs/features/robocup-midsize-rules.md` | RoboCup 中型组规则速查 |
| `docs/features/robot-connection-guide.md` | 实机连接（10.0.0.34 / 10.0.0.30） |
