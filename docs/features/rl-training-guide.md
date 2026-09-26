# RL 训练完全指南（PPO × Webots 四足行走）

> 适用仓库：`Dog1`（YoboGo-10S / Mini Cheetah）
> 训练代码：`webots-sim/rl/`
> 最终成绩（2026-09-26 实测）：奖励 **643 → 2,100**，四阶段共 **200k + 300k + 200k** 步，`ep_len_mean` 全程满额 **零摔倒**。

---

## 目录

1. [训练原理：为什么用 RL / PPO](#1-训练原理为什么用-rl--ppo)
2. [系统架构](#2-系统架构)
3. [观测 / 动作 / 奖励设计](#3-观测--动作--奖励设计)
4. [环境搭建（Python venv）](#4-环境搭建python-venv)
5. [四阶段完整训练命令](#5-四阶段完整训练命令)
6. [超参数](#6-超参数)
7. [监控与回放](#7-监控与回放)
8. [奖励曲线解读](#8-奖励曲线解读)
9. [遇到的问题与解决](#9-遇到的问题与解决)
10. [进阶：继续训练 / 热启动 / 调奖励](#10-进阶继续训练--热启动--调奖励)

---

## 1. 训练原理：为什么用 RL / PPO

### 1.1 为什么选强化学习

四足行走的传统做法是**手工步态**（trot 轨迹 + PD），本仓库的
`webots-sim/controllers/mini_cheetah_controller` 就是这种：
步态相位、抬腿高度、关节期望角全是预设常量。它的优点是可预测，
缺点是：

- 换地形（台阶、斜坡、窄道）就要重调参数；
- 复杂目标（边走边转向对准球门、爬台阶）靠手工规则组合非常困难；
- 摔倒恢复、落地缓冲这类"模糊"行为难以用规则写全。

**强化学习**让策略自己从"试错"中学会：给定观测（关节角、姿态、速度……），
输出 12 个关节期望角，由奖励函数告诉它"站得稳、走得快、朝目标、别摔"。
策略最终学会的往往比手工规则更鲁棒。

### 1.2 PPO 简介

PPO（Proximal Policy Optimization，近端策略优化）是目前最常用的 on-policy 策略梯度算法：

- **策略网络** `π(a|s)`：观测 → 动作分布（本项目 `MlpPolicy`，256×256 两层 MLP）；
- **价值网络** `V(s)`：估计当前状态的长期回报，用于降低方差；
- **裁剪目标**：更新时限制新旧策略的概率比在 `1±clip_range`（默认 0.2）内，
  防止一步更新过大把策略"走崩"；
- **GAE**（`gae_lambda=0.95`）估计优势函数，`gamma=0.99` 折扣未来奖励。

为什么用 PPO 而不是 SAC/TD3：PPO 是 on-policy、实现成熟、超参数不敏感，
配合 Stable-Baselines3（SB3）几行代码就能跑，且对"仿真步进慢"的环境
（Webots 物理在 CPU 上）更稳定——不需要大量 replay buffer。

### 1.3 训练流程（一个循环）

```
Webots 物理步进 ──► rl_agent.py（Supervisor）──TCP 桥──► walk_env.py（Gym）
                                                              │
         PPO 收集 n_steps=2048 步 ◄────────────────────────────┘
                │
                ▼
         策略更新 10 epochs ──► 存 checkpoint ──► 继续收集
```

每 2048 步做一次策略更新；每 10,000～20,000 步落盘一个 checkpoint。

---

## 2. 系统架构

| 组件 | 路径 | 职责 |
|---|---|---|
| 训练入口 | `webots-sim/rl/train_ppo.py` | PPO 训练、断点续训、首层扩展热启动 |
| Gym 环境（P1/P2） | `webots-sim/rl/walk_env.py` | 42 维观测 / 12 维动作，奖励计算 |
| Gym 环境（P3 转向） | `webots-sim/rl/walk_env_turn.py` | 45 维观测，朝向目标奖励 |
| Gym 环境（P4 台阶） | `webots-sim/rl/walk_env_stairs.py` | 51 维观测，爬台阶奖励 + 地形采样 |
| 集中配置 | `webots-sim/rl/config.py` | 超参数 / 奖励权重 / 观测维度 |
| Webots 侧 | `webots-sim/controllers/rl_agent/rl_agent.py` | Supervisor：读状态、执行 `q_des = q_stand + a·scale` |
| 回放 | `webots-sim/rl/play.py`、`play_third_person.py` | 演示 / 录视频（第一 / 第三人称） |
| 监控 | `webots-sim/rl/monitor.py` | 生成 `logs/STATUS.md` 状态快照 |
| 世界文件 | `webots-sim/worlds/parkour_dev.wbt`（平地）、`.parkour_rl.wbt`（台阶） | 训练场地（由 walk_env 自动改写生成 `.{stem}_rl.wbt`） |

**通信链路**：`walk_env.py` 与 `rl_agent.py` 之间走 **本地 TCP 桥**
（JSON 行协议，默认端口 `RL_BRIDGE_PORT`，多实例必须错开：
P1=11451、P3=11452、P4=11453）。

---

## 3. 观测 / 动作 / 奖励设计

### 3.1 观测向量（42 / 45 / 51 维）

**基础 42 维**（`walk_env.py:_state_to_obs`）：

| 段 | 维数 | 含义 |
|---|---|---|
| `q` | 12 | 12 个关节角（顺序 fr → fl → hr → hl，每腿 `abd → hip → kn`） |
| `dq` | 12 | 12 个关节角速度 |
| `rpy` | 3 | 机身 roll / pitch / yaw |
| `v_body` | 3 | 机体系线速度 vx / vy / vz（世界系速度只用 yaw 旋转过来） |
| `prev_action` | 12 | 上一拍动作（让策略感知动作变化率，利于平滑） |

**P3 转向 45 维** = 42 维 + 尾部 3 维：

| 维 | 含义 |
|---|---|
| `yaw_err` | 当前航向与目标航向偏差（wrap 到 [-π, π]） |
| `wz` | 偏航角速度 |
| `vx` | 前向速度（与目标共用奖励项） |

**P4 台阶 51 维** = 45 维 + **6 维前下方地形采样**：
机器人前方地面 6 个采样点的高度（简化阶梯函数近似，非真实高度场；
台阶高度每集在 {0.05, 0.08, 0.12} m 中随机）。

> 维度常量集中在 `config.py`：`OBS_DIM=42`、`OBS_DIM_TURN=45`、`OBS_DIM_STAIRS=51`。
> 改观测必须同步改这里，`train_ppo.py` 启动时会自检维度。

### 3.2 动作向量（12 维）

- `action ∈ [-1, 1]^12`，一一对应 12 个关节；
- 映射：**`q_des = q_stand + a · scale`**
- 缩放 `scale` 每腿相同：**abd 0.3 rad，hip 0.5 rad，knee 0.5 rad**
  （`walk_env.py:ACTION_SCALE_LEG`，由 `rl_agent.py` 执行）；
- 本机型模型零位即站立姿态（`q_stand = 0`，实测站立高 z=0.26 m 稳定）。

### 3.3 奖励函数

总公式（`config.py` 注释 + 各 `walk_env*.py:_compute_reward`）：

```
r = alive_bonus + forward_vel*vx
    + roll_penalty*|roll| + pitch_penalty*|pitch|
    + action_sq*||a||²
    + heading_cos*cos(yaw_err) + yaw_rate_penalty*|wz| + heading_bonus*[|yaw_err|<0.2]   # P3
    + climb_reward*max(Δz,0) + on_step_reward*[站上台阶] + drop_penalty*[从台阶掉下]     # P4
    摔倒时另加 fall_penalty（并提前终止）
```

**分阶段权重**（`config.py:PHASE_REWARD_WEIGHTS`，由 `--tag` 自动切换）：

| 阶段 | 权重要点 | 意图 |
|---|---|---|
| P1 `stand` | `+1 alive −2|roll| −2|pitch| −10 fallen` | 只求不倒 |
| P2 `walk` | P1 + `+3·vx − 0.05·‖a‖²` | 前进 |
| P3 `turn` | P2 + `+2·cos(yaw_err) − 0.5·|wz| + 1·[|yaw_err|<0.2]` | 对准目标 |
| P4 `stairs` | P3 + `+2·max(Δz,0) + 1·[站上台阶] − 3·[从台阶掉下]` | 爬台阶 |
| （对照）`step` | P2 + 爬台阶项（无转向塑形） | 纯爬台阶对照 |

单集固定 **1000 步**（`MAX_EPISODE_STEPS`），控制频率 **50 Hz**（`SIM_DT=0.02`），
即一集 20 秒。

---

## 4. 环境搭建（Python venv）

> **必须用 Python 3.10/3.11**。系统 Python 3.14 装不了 CUDA 版 torch 轮子，
> 且 NTFS 分区上的 venv 会踩内核 NTFS3 bug，故 venv 建在 **`/tmp/rl_venv`**（tmpfs）。

```bash
# 1) 建 venv（/tmp 下，重启后需重建）
python3.11 -m venv /tmp/rl_venv

# 2) 装依赖（CPU 版 torch 即可；SB3 官方建议 MLP 策略跑 CPU）
/tmp/rl_venv/bin/pip install --upgrade pip
/tmp/rl_venv/bin/pip install torch --index-url https://download.pytorch.org/whl/cpu
/tmp/rl_venv/bin/pip install stable-baselines3 gymnasium tensorboard

# 3) 录视频（二选一）
/tmp/rl_venv/bin/pip install imageio imageio-ffmpeg
# 或
/tmp/rl_venv/bin/pip install opencv-python

# 4) 自检
/tmp/rl_venv/bin/python -c "import torch, stable_baselines3, gymnasium; print(torch.__version__)"
```

**本项目实测版本**：Python 3.11.16，torch 2.14.0+cpu，stable-baselines3 2.9.0，
gymnasium 1.3.0。

---

## 5. 四阶段完整训练命令

> 所有命令在**仓库根目录**执行。后台长任务必须
> `setsid nohup ... < /dev/null &`，否则会被会话超时连带杀掉。
> 多阶段并行时务必错开 `RL_BRIDGE_PORT`。

### P0 冒烟（可选，先跑通链路）

```bash
RL_BRIDGE_PORT=11451 PYTHONUNBUFFERED=1 setsid nohup /tmp/rl_venv/bin/python \
  webots-sim/rl/train_ppo.py \
  --total-steps 5000 --tag phase0_smoke --ckpt-prefix phase0_smoke \
  --checkpoint-interval 5000 --eval-interval 0 \
  < /dev/null > logs/phase0_smoke.log 2>&1 &
```

### P1 站立（200k 步，42 维，stand 权重）

```bash
RL_BRIDGE_PORT=11451 PYTHONUNBUFFERED=1 setsid nohup /tmp/rl_venv/bin/python \
  webots-sim/rl/train_ppo.py \
  --total-steps 200000 --tag phase1_stand --ckpt-prefix phase1_stand \
  --checkpoint-interval 10000 --eval-interval 20000 --eval-episodes 10 \
  < /dev/null > logs/train_p1.log 2>&1 &
```

产出：`checkpoints/phase1_stand_*.zip`、`phase1_stand_final.zip`
（`ep_len_mean` 全程 1000，零摔倒）。

### P2 行走（200k 步，42 维，walk 权重）

```bash
RL_BRIDGE_PORT=11451 PYTHONUNBUFFERED=1 setsid nohup /tmp/rl_venv/bin/python \
  webots-sim/rl/train_ppo.py \
  --total-steps 200000 --tag phase2_walk --ckpt-prefix ppo_walk \
  --resume checkpoints/phase1_stand_final.zip \
  --checkpoint-interval 20000 --eval-interval 20000 --eval-episodes 3 \
  < /dev/null > logs/train_full.log 2>&1 &
```

产出：`checkpoints/ppo_walk_final.zip`（累计 200,704 步，`ep_rew_mean` 643 → 1,220）。

### P3 转向（300k 步，45 维，turn 权重）

```bash
RL_BRIDGE_PORT=11452 PYTHONUNBUFFERED=1 setsid nohup /tmp/rl_venv/bin/python \
  webots-sim/rl/train_ppo.py \
  --total-steps 300000 --tag phase3_turn --ckpt-prefix p3_turn \
  --resume checkpoints/ppo_walk_final.zip \
  --checkpoint-interval 20000 --eval-interval 20000 --eval-episodes 3 \
  < /dev/null > logs/train_p3.log 2>&1 &
```

产出：`checkpoints/p3_turn_final.zip`（累计 301,056 步，264 分钟，18.9 steps/s，
`ep_rew_mean` 779 → **1,870**，`explained_variance` 0.97）。

### P4 台阶（200k 步，51 维，stairs 权重）

直接用启动器（会等 P3 结束后自动热启动）：

```bash
setsid nohup bash scripts/run_p4.sh < /dev/null > logs/p4_waiter.log 2>&1 &
```

或手动执行等价命令：

```bash
RL_BRIDGE_PORT=11453 PYTHONUNBUFFERED=1 setsid nohup /tmp/rl_venv/bin/python \
  webots-sim/rl/train_ppo.py \
  --total-steps 200000 --tag phase4_stairs --ckpt-prefix p4_stairs \
  --resume checkpoints/p3_turn_final.zip \
  --checkpoint-interval 20000 --eval-interval 20000 --eval-episodes 3 \
  < /dev/null > logs/train_p4.log 2>&1 &
```

产出：`checkpoints/p4_stairs_final.zip`（累计 200,960 步，`ep_rew_mean` → **2.1e+03 ≈ 2,100**）。

> `--tag` 会自动选择环境类型与奖励权重：
> `phase*_stand/walk → walk(42)`、`phase*_turn → turn(45)`、`phase*_stairs → stairs(51)`。
> 也可显式 `--env walk|turn|stairs` 覆盖。

### 阶段成绩汇总

| 阶段 | 步数 | 奖励 `ep_rew_mean` | 存活 | 最终模型 |
|---|---|---|---|---|
| P1 站立 | 200k | 643 → 1,220（含 P2） | 1000 满额，零摔倒 | `ppo_walk_final.zip` |
| P2 行走 | （含于上行 200k） | 净前进 +220 | 同上 | `ppo_walk_final.zip` |
| P3 转向 | 300k | 779 → 1,870 | 满额 | `p3_turn_final.zip` |
| P4 台阶 | 200k | → **2,100** | 满额 | `p4_stairs_final.zip` |
| **合计** | **200k+300k+200k** | **643 → 2,100** | **零摔倒** | `checkpoints/` |

---

## 6. 超参数

集中在 `webots-sim/rl/config.py`（`PPO_PARAMS` / `NET_ARCH`）：

| 超参数 | 值 | 含义 |
|---|---|---|
| `policy` | `MlpPolicy` | 多层感知机策略 |
| `net_arch` | `[256, 256]` | 策略/价值两层隐层 |
| `n_steps` | 2048 | 每次更新收集的环境步数 |
| `batch_size` | 256 | mini-batch 大小 |
| `n_epochs` | 10 | 同批数据优化轮数 |
| `gamma` | 0.99 | 折扣因子 |
| `gae_lambda` | 0.95 | GAE λ |
| `learning_rate` | 3e-4 | 学习率 |
| `ent_coef` | 0.0 | 熵正则（不鼓励探索噪声） |
| `clip_range` | 0.2 | PPO 裁剪范围 |
| `MAX_EPISODE_STEPS` | 1000 | 单集最大步数 |
| `SIM_DT` | 0.02 s | 控制频率 50 Hz |
| `CHECKPOINT_INTERVAL` | 10,000（可 `--checkpoint-interval` 覆盖） | 落盘间隔 |
| `EVAL_INTERVAL` | 20,000 | 评估间隔（0 关闭） |

`train_ppo.py` 常用参数：

| 参数 | 说明 |
|---|---|
| `--total-steps` | **累计**目标步数（`--resume` 时自动补齐差额） |
| `--resume <zip>` | 断点续训 / 热启动 |
| `--tag` | 阶段标签，决定奖励权重与环境类型 |
| `--ckpt-prefix` | checkpoint 文件名前缀 |
| `--checkpoint-interval` / `--eval-interval` / `--eval-episodes` | 落盘 / 评估节奏 |
| `--device` | `cuda` / `cpu` |
| `--env` | 显式指定 `walk` / `turn` / `stairs` |
| `--seed` / `--dry-run` | 随机种子 / 只解析参数 |

---

## 7. 监控与回放

### 7.1 状态快照

```bash
/tmp/rl_venv/bin/python webots-sim/rl/monitor.py
# 生成 logs/STATUS.md：阶段、步数、步速、ETA、checkpoint、GPU/CPU/内存、错误统计
```

### 7.2 TensorBoard 曲线

```bash
/tmp/rl_venv/bin/tensorboard --logdir runs/
# 浏览器打开 http://localhost:6006
```

关注：`rollout/ep_rew_mean`（奖励应升）、`rollout/ep_len_mean`（存活应满）、
`train/explained_variance`（>0.9 说明价值网络学得好）、`train/value_loss`。

### 7.3 回放 / 录视频

```bash
# 基本回放（打印每集奖励 / 存活步数 / 前进距离）
/tmp/rl_venv/bin/python webots-sim/rl/play.py --model checkpoints/ppo_walk_final.zip

# 录制视频（环境 rgb_array）
/tmp/rl_venv/bin/python webots-sim/rl/play.py \
  --model checkpoints/ppo_walk_final.zip --video out.mp4

# 第三人称视角视频（后上方跟随相机；P3/P4 可加 --turn）
/tmp/rl_venv/bin/python webots-sim/rl/play_third_person.py \
  --model checkpoints/p3_turn_final.zip --turn \
  --video videos/walk_p3_third_person.mp4
```

`play.py` 其它参数：`--episodes N`、`--render`（开 GUI）、`--max-steps N`、
`--stochastic`（随机动作）、`--device cpu|cuda`、`--dry-run`。

---

## 8. 奖励曲线解读

以 P1/P2（`logs/train_full.log`，97 次迭代 × 2048 步）为例：

| 区间 | 奖励 | 解读 |
|---|---|---|
| 0 – 22k 步 | 643 → 664 | 缓慢起步（从续训点微调） |
| 22k – 100k 步 | 664 → 921 | 稳定线性爬升（约 +3.3 / 千步），步态成形 |
| 100k – 155k 步 | 921 → 1,110 | 爬升加速，行走能力巩固 |
| 155k – 200k 步 | 1,110 → 1,220 | 增速放缓、趋于饱和 |

**怎么读：**

1. **基线 1,000 分 ≈ 纯存活**：`alive_bonus=1 × 1000 步`。奖励 ≈1000 时
   机器人只是"站住不动"；超出部分（P2 的 ~220）来自 `+3·vx` 减去姿态/动作惩罚。
2. **`ep_len_mean` 一直是 1000** 是最好的信号——说明零摔倒，P1 目标已达成。
3. **平台期**（斜率明显变小）就可以停，不必死磕步数；还想提升就调奖励（见第 10 节）。
4. P3 曲线从 779 起步（obs 从 42→45 扩展，新输入通道置零，短期掉分正常），
   后升到 1,870；`explained_variance` 0.92–0.97 说明价值网络贴合。
5. P4 因地形采样变慢（约 10 fps），奖励含爬台阶塑形，最终 ~2,100。
   其中 `+2·max(Δz,0)` 量级很小（~1e-3/步），主要靠 `+1·[站上台阶]` 驱动。

---

## 9. 遇到的问题与解决

### 9.1 磁盘配额爆（`Errno 122`）

- **现象**：pip / 训练写文件报 `OSError: [Errno 122] 磁盘配额超出`，
  但 `df -h /` 还有几十 G。
- **原因**：`/tmp`（tmpfs）被旧的球检测帧（`/tmp/ball_detect`、`*.rgb`）占满。
- **解决**：

```bash
du -sh /tmp/* 2>/dev/null | sort -h | tail
rm -rf /tmp/ball_detect /tmp/*.rgb
```

### 9.2 Python 3.14 装不了 CUDA torch（pip 索引 / 断流）

- **现象**：`pip install torch --index-url .../cu121` 失败：3.14 无 cu121 轮子；
  换 cu130 又遇下载断流 + 上面的"假配额"。
- **解决**：**放弃 CUDA**，建 Python **3.11** venv 于 `/tmp/rl_venv`，
  装 **CPU 版 torch**。SB3 官方本来就不建议 MLP 策略上 GPU
  （日志里的 `UserWarning: PPO on GPU ... MlpPolicy` 属正常提示）。
- **附**：NTFS 分区上的 venv 会触发内核 NTFS3 bug（`kernel BUG at fs/iomap/...`），
  进程卡 D 状态不可杀，**venv 千万别放 NTFS 盘**。

### 9.3 TCP 桥死锁（训练"卡死"）

- **现象**：P4 训练进程 `S sleeping`、CPU 0%、`logs/train_p4.log` 20 秒零增长、
  `n_updates` 停更，但 Webots 进程正常、TCP 端口仍在监听。
- **判定**：**不能只看 `pgrep` 活着就认为在训练**。活性三要素：
  ① 日志字节数增长率 ② `ps -o pcpu` ③ checkpoint mtime。
- **解决**：杀掉卡住的训练进程，从最近 checkpoint 断点续训：

```bash
kill -9 <训练PID>
RL_BRIDGE_PORT=11453 PYTHONUNBUFFERED=1 setsid nohup /tmp/rl_venv/bin/python \
  webots-sim/rl/train_ppo.py \
  --total-steps 200000 --tag phase4_stairs --ckpt-prefix p4_stairs \
  --resume checkpoints/p4_stairs_160000_steps.zip \
  --checkpoint-interval 20000 --eval-interval 20000 --eval-episodes 3 \
  < /dev/null > logs/train_p4.log 2>&1 &
```

- **相关坑**：多 Webots 实例必须错开 `RL_BRIDGE_PORT`（11451/11452/11453）；
  评估回调曾因抢端口报 `[Errno 98] Address already in use`（跳过一次评估即可）。

### 9.4 NVIDIA / 录屏 / 渲染

| 问题 | 解决 |
|---|---|
| `ffmpeg x11grab` 抓 Webots 画面**全黑**（NVIDIA 渲染） | 用 ImageMagick：`import -window <wid> out.png`，wid 取 `xwininfo -root -tree \| grep webots-bin` |
| GPU 利用率 0% | **正常**：SB3 的 MLP 策略官方建议跑 CPU；Webots 物理也在 CPU。GPU 只占 ~150 MiB 显存 |
| 启动不用独显 | `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia webots ...` |
| `--no-rendering` 后相机黑屏 | 该旗标会**禁用 Camera 传感器**，录相机画面时不能加 |
| `timeout` 包裹后报"进程崩溃" | 多为 SIGTERM 误报；`INFO: ... Terminating.` 之后的崩溃警告可忽略 |
| 键盘没反应 | 先点击 3D 视图获得焦点；`wb_keyboard_get_key()` 无键返回 `-1`，判 Shift 前必须先判 `key < 0` |

---

## 10. 进阶：继续训练 / 热启动 / 调奖励

### 10.1 断点续训（同一维度）

```bash
# --total-steps 是累计目标：从 20 万续到 40 万
/tmp/rl_venv/bin/python webots-sim/rl/train_ppo.py \
  --total-steps 400000 --tag phase2_walk --resume checkpoints/ppo_walk_final.zip
```

### 10.2 obs 维度扩展热启动（42 → 45 → 51）

`train_ppo.load_resume_model` 发现维度不一致时，会做**首层扩展**：
旧网络首层权重前 N 列原样拷贝，新增输入通道置零。这样 42→45→51
续训不丢已有能力（实测 `ep_len_mean` 始终满额）。

```bash
# 例：42 维行走模型 → 45 维转向
/tmp/rl_venv/bin/python webots-sim/rl/train_ppo.py \
  --total-steps 300000 --tag phase3_turn --resume checkpoints/ppo_walk_final.zip
# 日志出现：【训练】obs 维度不一致，执行首层扩展热启动 ...
```

> 维度变化大时也可以**从零重训**，但热启动通常省一半时间。

### 10.3 调奖励

改 `config.py` 的 `PHASE_REWARD_WEIGHTS`（或 `REWARD_WEIGHTS`），
`walk_env*.py` 直接 import 同一套权重，改完即生效：

| 症状 | 调法 |
|---|---|
| 走不远 / 躺平刷存活 | 提高 `forward_vel`，降低 `alive_bonus` |
| 动作抖动 | `action_sq` 更负（如 `-0.1`） |
| 老摔 | 提高 `orientation`/`fall_penalty` 绝对值 |
| 转向不灵 | 提高 `heading_cos`、`heading_bonus` |
| 不敢上台阶 | 提高 `climb_reward`、`on_step_reward`，降低 `drop_penalty` 绝对值 |

改观测拼接逻辑时，同步改 `config.py` 的 `OBS_DIM*`，否则启动自检直接报错。

### 10.4 改网络 / 超参数

- 网络宽度：`config.py` 的 `NET_ARCH = [256, 256]` → 可改 `[512, 512, 256]`；
- 学习率、`n_steps` 等：改 `PPO_PARAMS`；
- 改完无需动 `train_ppo.py`。

---

## 附：常用速查

```bash
# 训练活性三查
tail -f logs/train_p4.log
ls -lt checkpoints/ | head
ps -o pid,pcpu,etime,cmd -p $(pgrep -f train_ppo.py)

# 全部最终模型
ls checkpoints/ppo_walk_final.zip checkpoints/p3_turn_final.zip checkpoints/p4_stairs_final.zip

# 文档索引
#   webots 仿真操作    → README.md 第三章 / docs/features/webots-sim-guide.md
#   手动遥控跑酷       → README.md 第五章 / docs/compose/spec/manual-parkour.md
```
