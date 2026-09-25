# 正式训练启动报告 — 200K 步 PPO

- **时间**: 02:12 (2026-09-26)
- **状态**: ✅ 成功启动并验证
- **PID**: `1094946`
- **日志**: `logs/train_full.log`
- **TensorBoard 日志目录**: `runs/ppo_walk_20260926-021035/PPO_0`

## 使用的命令

```bash
cd /run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1
setsid nohup /tmp/rl_venv/bin/python webots-sim/rl/train_ppo.py \
  --total-steps 200000 --device cpu \
  --eval-interval 10000 --checkpoint-interval 20000 \
  --resume checkpoints/ppo_walk_final.zip \
  > logs/train_full.log 2>&1 < /dev/null &
```

## 启动验证（启动后 ~120s 检查）

- ✅ 进程存活：`pgrep -af train_ppo.py` → PID 1094946
- ✅ 无 Traceback / ERROR
- ✅ 日志已输出训练指标：

```
| rollout/           |          |
|    ep_len_mean     | 1e+03    |
|    ep_rew_mean     | 643      |
|    fps             | 29       |
|    total_timesteps | 4096     |
```

- ✅ 续训正确：`已加载模型，当前累计步数 = 2048`，`本次运行 197952 步（目标总步数 200000）`
- ✅ 新建 run 目录：`runs/ppo_walk_20260926-021035`

## 与任务命令的差异（实际参数）

| 任务建议 | 实际使用 | 原因 |
|---|---|---|
| `--save-interval 20000` | `--checkpoint-interval 20000` | 脚本无 `--save-interval`，保存参数名为 `--checkpoint-interval` |
| （未指明续训） | `--resume checkpoints/ppo_walk_final.zip` | 冒烟 checkpoint 存在，从累计 2048 步续训至 200000 |
| `nohup ... &` | `setsid nohup ... < /dev/null &` | 普通 `nohup &` 会被工具超时连带杀掉（首启 PID 1015243 即被杀），`setsid` 脱离进程组后稳定 |

其他参数均与任务一致：`--total-steps 200000 --device cpu --eval-interval 10000`。
未设 `--tag`（与冒烟训练一致，使用默认奖励权重）。

## 预计时长

- 速度：~29 steps/s（CPU，实测）
- 剩余训练步数：197,952 → 纯训练约 **114 分钟**
- 另有每 10000 步一次评估（10 集 × ~1000 步，共约 20 次评估），评估开销约 +30–60 分钟
- **总计约 2.5–3.5 小时**，预计 04:40–05:40 完成

## 备注

- **/tmp 清理未执行**：`rm -rf /tmp/ball_detect` 等删除操作被权限规则拦截（external_directory → ask，子代理无法批准）。/tmp 当前 5.4G/7.5G 已用、**2.1G 可用**，checkpoint 写在仓库（NTFS）的 `checkpoints/`，TB 日志在 `runs/`，训练不受影响。
- 首次启动的进程（PID 1015243）因工具 120s 超时被杀，产生了空 run 目录 `runs/ppo_walk_20260926-020823`，可后续清理。
- checkpoint 将每 20000 步写入 `checkpoints/ppo_walk_<step>.zip`，最终覆盖 `checkpoints/ppo_walk_final.zip`。
