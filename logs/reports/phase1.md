# Phase 1 站立训练报告

- **启动时间**：2026-09-26 08:21
- **命令**：`nohup python3 webots-sim/rl/train_ppo.py --total-steps 200000 --tag phase1_stand > logs/train.log 2>&1 &`
- **PID**：1700252（写入 `logs/train.pid`）
- **看门狗**：`webots-sim/rl/watchdog_p1.sh`（PID 1715857，`logs/watchdog.pid`），每 5 分钟巡检，挂了从最近 checkpoint 续训，P1 完成后自动启动 P2

## 奖励权重（P1 站立）

`config.py` 的 `PHASE_REWARD_WEIGHTS["stand"]` 已符合 P1 目标（中文注释齐全，`--tag phase1_stand` 经 `set_phase()` 自动生效）：

| 项 | 权重 |
|---|---|
| alive_bonus 存活 | +1.0 / 步 |
| roll_penalty \|roll\| | -2.0 |
| pitch_penalty \|pitch\| | -2.0 |
| fall_penalty 摔倒 | -10.0 |
| forward_vel / action_sq | 0（P1 不奖前进、不罚动作） |

## 进度快照（2026-09-26 08:36）

| 指标 | 数值 |
|---|---|
| 训练步数 | 30,720 / 200,000（约 15%） |
| 步速 | ~35 steps/s（预计全程约 95 分钟） |
| ep_len_mean | **1000**（episode 满长，未摔倒） |
| ep_rew_mean | ~917–918 |
| 存活率估计 | **≈100%（999.2 / 1000）**，已达 >80% 目标 |
| checkpoint | `checkpoints/phase1_stand_10000_steps.zip` 等，每 10k 一存 |
| TensorBoard | `runs/ppo_walk_phase1_stand_20260926-082149/PPO_1/` |

## 达标判定

**P1 目标「存活率 >80%」已满足**（满长存活 ≈100%）。训练继续跑满 200k 步以巩固站立策略；
看门狗会在结束后自动以 `--tag phase2_walk --total-steps 400000` 启动 P2（奖励含 3*vx）。

## 已知问题

- 【评估】创建评估环境失败：`[Errno 98] Address already in use` —— 仿真端口被训练环境独占，
  EvaluateCallback 捕获后自动跳过后续评估，**不影响训练**。存活/奖励指标以 rollout 统计为准。
- SB3 警告 MLP 策略在 GPU 上利用率低：预期现象，训练正常（~35 fps）。
