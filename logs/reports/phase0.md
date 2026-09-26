# Phase 0 冒烟训练报告

- **时间**：2026-09-26 08:18 ~ 08:21
- **命令**：`python3 webots-sim/rl/train_ppo.py --total-steps 5000 --tag phase0_smoke`
- **目的**：验证训练管线端到端可跑（依赖 → 环境 → PPO → checkpoint → TensorBoard）

## 依赖情况

| 包 | 版本 | 备注 |
|---|---|---|
| torch | 2.14.0+cu130 | CUDA 可用（RTX 4060 Laptop, 8GB） |
| stable-baselines3 | 2.9.0 | |
| gymnasium | 1.3.0 | |
| tensorboard | 2.21.0 | |

安装方式：`pip install --user --break-system-packages`（cu130 索引装 torch，PyPI 装其余）。
磁盘充足（根分区 96G 可用），未触发备选 venv 方案。

## 结果

- **状态**：✅ 成功，无报错
- **实跑步数**：6144 步（PPO n_steps=2048，3 次迭代后达到 ≥5000 目标）
- **耗时**：约 2.9 分钟（~29–35 steps/s）
- **episode 指标**：ep_len_mean = 1000（满长不摔倒），ep_rew_mean ≈ 918
- **checkpoint**：`checkpoints/phase0_smoke_final.zip` ✅
- **TensorBoard**：`runs/ppo_walk_phase0_smoke_20260926-081810/PPO_1/events.out.tfevents.*` ✅ 有内容

## 修复记录

- `--tag phase0_smoke` 原先不在 `config.set_phase` 别名表中导致 ValueError。
  已在 `webots-sim/rl/config.py` 增加别名：`phase0 / phase0_smoke / smoke → stand`（权重与 P1 站立一致）。

## 备注

- SB3 提示 MLP 策略在 GPU 上利用率低（非 CNN），属预期警告，训练正常。
- 经过 P0，管线各环节（环境 reset/step、PPO learn、Monitor、CheckpointCallback、TB 写入）全部验证通过。
