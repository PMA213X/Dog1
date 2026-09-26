# Phase 3 转向训练报告

- **启动时间**：2026-09-26 08:39
- **PID**：1718834（写入 `logs/train_p3.pid`）
- **日志**：`logs/train_p3.log`
- **TensorBoard**：`runs/ppo_walk_phase3_turn_20260926-083934/PPO_0/`

## 1. 回放视频（P2 行走）

| 项 | 值 |
|---|---|
| 路径 | `videos/walk_p2.mp4` |
| 大小 | **1,358,703 bytes（≈1.29 MB）** |
| 规格 | 640×480 @ 30 fps，750 帧（3 集 × 250 步） |
| 后端 | OpenCV `mp4v`（imageio 未安装，play.py 自动回退） |
| 回放结果 | 平均奖励 341.3，平均存活 250/250 步，平均前进 1.173 m |

启动命令（`play.py` 原生支持 `--video`，无需补参数）：

```bash
RL_BRIDGE_PORT=11452 PYTHONUNBUFFERED=1 nohup /tmp/rl_venv/bin/python \
  webots-sim/rl/play.py --model checkpoints/ppo_walk_final.zip \
  --episodes 3 --max-steps 250 --video videos/walk_p2.mp4 \
  > logs/play_video.log 2>&1 &
```

> 说明：加 `--max-steps 250` 是为了控制内存（640×480 帧约 0.9 MB/帧，3×1000 步会把 8 GB 内存挤爆）。
> 用 `RL_BRIDGE_PORT=11452` 避让并行的 P1 训练（占用默认端口 11451）。
> `walk_env.py` 的 rgb_array 模式已去掉 `--no-rendering`（否则 Camera 传感器不出图）。

## 2. P3 启动命令

```bash
cd /run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1
RL_BRIDGE_PORT=11452 PYTHONUNBUFFERED=1 nohup /tmp/rl_venv/bin/python \
  webots-sim/rl/train_ppo.py \
  --total-steps 300000 --tag phase3_turn \
  --resume checkpoints/ppo_walk_final.zip \
  --checkpoint-interval 20000 --ckpt-prefix p3_turn \
  --eval-interval 20000 --eval-episodes 3 \
  > logs/train_p3.log 2>&1 &
echo $! > logs/train_p3.pid
```

- 环境：`webots-sim/rl/walk_env_turn.py` 的 `QuadrupedTurnEnv`（45 维观测）
- 热启动：42 → 45 维自动首层扩展（前 42 列复制，新增 3 列置零），实测拷贝 11 个同形状张量 + 2 处首层扩展
- checkpoint：每 20k 步 → `checkpoints/p3_turn_<step>.zip`，最终 `checkpoints/p3_turn_final.zip`

## 3. P3 观测与奖励

观测 45 维 = 原 42 维 + `[yaw_err, wz, vx]`：

| 新增维度 | 含义 |
|---|---|
| yaw_err | 当前航向 − 目标航向，wrap 到 [-π, π] |
| wz | 偏航角速度（rl_agent `getVelocity()[3:6]`，缺失时 yaw 差分） |
| vx | 机体系前进速度 |

奖励（`config.set_phase("turn")`）：

```
r = 1.0*alive + 3.0*max(vx,0) - 2|roll| - 2|pitch| - 0.05‖a‖² - 10*fallen
    + 2.0 * cos(yaw_err)          # 朝向目标
    - 0.5 * |wz|                  # 转向平滑
    + 1.0 * (|yaw_err| < 0.2)     # 朝向正确奖励
```

目标航向：每 episode 随机采样 (-π, π]（`heading_mode="random"`，可切 `left`/`right`）。

## 4. 当前奖励曲线（启动后约 5 分钟）

| total_timesteps | ep_rew_mean | ep_len_mean | fps |
|---|---|---|---|
| 2,048 | 1,780 | 1,000 | 49 |
| 4,096 | 754 | 1,000 | 48 |
| 6,144 | 471 | 1,000 | 48 |
| 8,192 | 779 | 1,000 | 48 |
| 10,240 | 716 | 1,000 | — |
| 12,288 | 873 | 1,000 | — |

- **存活稳定**：ep_len_mean 始终 1000（满长），热启动保住了行走能力
- 奖励从首批次 1780 回落到 500–900 区间：随机目标航向下 `cos(yaw_err)` 期望为 0，
  且策略正在探索「转向对准」动作，属正常塑形期波动；后续随对准率上升应逐步走高
- 预计 300k 步约 **100 分钟**（~48 steps/s）

## 5. P4 台阶（待 P3 达标后切换）

计划命令（100k 步，世界 `parkour.wbt`）：

```bash
RL_BRIDGE_PORT=11453 PYTHONUNBUFFERED=1 nohup /tmp/rl_venv/bin/python \
  webots-sim/rl/train_ppo.py \
  --total-steps 100000 --tag phase4_step \
  --resume checkpoints/p3_turn_final.zip \
  --checkpoint-interval 20000 --ckpt-prefix p4_step \
  > logs/train_p4.log 2>&1 &
```

- `config.PHASE_REWARD_WEIGHTS["step"]` 已含 `climb_reward=2.0`（`+2*Δz`）与 `on_step_reward=1.0`
- obs 地形高度采样可先不加（任务允许），后续需要时在 `walk_env_turn.py` 上再扩展
- 世界切换：`QuadrupedWalkEnv(world="webots-sim/worlds/parkour.wbt")`（`--world` 待补或改 `DEFAULT_WORLD`）

## 6. 已知问题

- 【评估】评估环境会与训练环境抢同一桥接端口（`Address already in use`），
  `EvaluateCallback` 捕获后自动跳过，**不影响训练**；指标以 rollout 统计为准
- 与并行的 P1 训练共存：P1 占 11451，P3 用 11452；两个 Webots 实例可并行
- `/tmp/rl_venv` 以 `--system-site-packages` 创建（原 venv 不存在；系统 Python 已有 torch 2.14+cu130 / sb3 2.9.0）
