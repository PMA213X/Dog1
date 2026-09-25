# 训练监控面板 — 状态快照

> 生成时间：2026-09-26 01:25:39　|　工具：`webots-sim/rl/monitor.py`　|　时间窗口：14 小时

## 一、总体状态

| 项 | 值 |
|---|---|
| 当前阶段 | **— 等待启动**（来源：未发现阶段标注） |
| 训练进程 | ⏸ 未运行（`pgrep train_ppo` 无匹配） |
| 启动至今 | —— **等待启动** |
| 14h 窗口剩余 | 14 小时 0 分（尚未起算） |
| 训练步数 | 0（尚无步数记录） |
| 步速 | ——（样本不足，无法估算） |
| ETA | ——（无推进速率） |
| 状态结论 | 🟡 **等待启动**——训练未开始，先跑冒烟再放大步数 |

## 二、训练进度

| 项 | 值 |
|---|---|
| 最新 checkpoint | ——（`checkpoints/*.zip` 为空） |
| checkpoint 总数 | 0 |
| TensorBoard 事件 | ——（`runs/*/events*` 为空） |
| 事件文件总数 | 0 |
| 近 100 集奖励均值 | —— 无数据 |
| 存活步数均值 | —— 无数据 |
| 指标来源 | 无数据 |

## 三、系统资源

| 资源 | 状态 |
|---|---|
| GPU | NVIDIA GeForce RTX 4060 Laptop GPU｜占用 0%｜显存 26/8188 MiB｜51°C |
| CPU | 使用率 9%｜负载(1/5/15) 1.49 / 1.51 / 1.63 |
| 内存 | 已用 9.2/14.8 GiB（62%）｜可用 5.6 GiB｜Swap 10.4/20.0 GiB |
| 磁盘 | 74% 已用（1.2T/1.7T，剩 450G）｜挂载 /run/media/pma213x/0C6A8CC86A8CAFCE |

## 四、错误与告警

| 关键词 | 出现次数 |
|---|---|
| ERROR | ✅ 0 |
| NaN | ✅ 0 |
| OOM | ❗ 1 |
| **合计** | **1** |

> 扫描范围：`logs/` 下全部文本文件（不含 `STATUS.md` 自身）。

## 五、提示

- 训练尚未开始。就绪后启动：`python3 webots-sim/rl/train_ppo.py --total-steps 2048 --device cpu --eval-interval 0`（冒烟 P0）。
- 依赖未装齐时先确认：`python3 -c 'import torch, stable_baselines3'`，以及 `webots-sim/rl/walk_env.py` 是否存在。
- ⚠️ 日志中发现 1 处 ERROR/NaN/OOM，建议先排查再继续。
- 刷新本面板：`python3 webots-sim/rl/monitor.py`；持续监控：`python3 webots-sim/rl/monitor.py --watch`。

---

*本文件由 `monitor.py` 自动生成于 2026-09-26 01:25:39，请勿手工编辑。*
