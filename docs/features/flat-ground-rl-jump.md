# 平直地面移动与跳跃 RL

> **已废弃：2026-10-03 起本目录不得作为新训练来源。**
> 唯一新方案见 `docs/features/official-mini-cheetah-flat-ground-rl.md`，
> 架构约束见 `docs/architecture/webots-sim-deprecated.md`。

> 状态：恢复方案已通过完整验证并启动正式训练；当前处于 F1A 固定随机化恢复段，
> 一次有界健康确认已全部通过。尚未完成 F1～F3 正式验收，不宣称模型训练完成。

## 范围

- 仅覆盖 Webots R2025a 中完全平直、无障碍的 YoboGo-10S 地面。
- 训练采用 Gymnasium、Stable-Baselines3 PPO 与 JSON TCP 锁步桥。
- 训练只采样结构化命令域；键盘和 Linux 手柄仅用于最终控制接口。

## 契约

- 观测：57 维。
- 动作：12 维，范围 `[-1, 1]`。
- 控制：50 Hz，Webots 物理步长 4 ms，decimation 为 5。
- 跳跃：二进制锁存命令，不增加动作维度。
- 新 checkpoint 前缀：`yobogo_flat_jump_v1`。

## 训练阶段

| 阶段 | 累计目标 | 随机化 | 当前状态 |
| --- | ---: | --- | --- |
| F0 冒烟 | 5,000 | full | 已有 checkpoint，启动时复用 |
| F1 稳定 | 350,000 | F1A fixed → F1B curriculum | 正在执行 F1A |
| F2 移动 | 650,000 | full | 待执行 |
| F3 跳跃整合 | 1,200,000 | full | 待执行 |

F1 内部拆分为累计 260,000 步的 F1A 固定恢复段和累计 350,000 步的
F1B 课程恢复段；四阶段累计预算保持 1,200,000 步，验收阈值不降低。

## 恢复训练证据

- 完整单元/契约/静态测试：`88/88`，退出码 `0`。
- 四阶段 CUDA dry-run：退出码 `0`，确认 `57/12`、
  `CUDA_VISIBLE_DEVICES=0`、`--device cuda` 和阶段累计目标。
- Webots headless：退出码 `0`，50 步 `obs_dim=57/action_dim=12`，无摔倒。
- F1 短程恢复：从 `250000` 步 checkpoint 续训至 `250256` 步，
  临时输出重载为 `device=cuda`、57/12 维且参数全为有限值，退出码 `0`。
- TensorBoard：`http://127.0.0.1:6006/`，启动后 HTTP `200`，
  后台 PID `535469`。
- 正式顺序训练后台 PID `535810`，启动时复用 F0 `5000` 步 checkpoint，
  并保留原 F1 `250000` 步 checkpoint 后进入 F1A。
- 有界健康确认全部通过：阶段/训练/Webots 进程存活、GPU0、
  `CUDA_VISIBLE_DEVICES=0`、命令含 `--device cuda`、
  `total_timesteps=250512 -> 250768`、非空 event、checkpoint 重载、
  无 `NaN/Inf/OOM`、单一 TCP 端口 `11451`、HTTP `200`。
- Webots 正式训练使用本地 `DISPLAY=:0` GUI 和 `--mode=fast`，
  未使用 `--no-rendering` 或 `--minimize`；健康采样为
  `12.8 steps/s`，未触发健康失败。

## 日志与状态

- 测试：`/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/logs/flat_jump_unit_tests_20261003_final_after_stage_log_fix.log`
- CUDA dry-run：`/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/logs/flat_jump_cuda_dry_run_20261003_final_after_stage_log_fix.log`
- Headless：`/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/logs/flat_jump_webots_headless_20261003_final_after_stage_log_fix.log`
- F1 短程恢复：`/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/logs/flat_jump_f1_short_resume_20261003.log`
- 健康确认：`/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/logs/yobogo_flat_jump_v1/health_confirmation_formal_gui_attempt3_20261003.log`
- 健康证据：`/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/logs/yobogo_flat_jump_v1/health_confirmation_formal_gui_attempt3_20261003.json`
- 启动器：`/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/logs/yobogo_flat_jump_v1/launcher_formal_gui_attempt2_20261003.log`
- F1A 训练：`/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/logs/yobogo_flat_jump_v1/F1A_train.log`
- TensorBoard：`/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/logs/yobogo_flat_jump_v1/tensorboard.log`
- checkpoint 启动清单：`/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/logs/yobogo_flat_jump_v1/checkpoint_inventory_before_formal_20261003.log`
- 执行日志：`/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/logs/yobogo_flat_jump_v1/stages.log`
- 状态：`/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/logs/yobogo_flat_jump_v1/stage_status.json`

实时画面已在本机 `DISPLAY=:0` 桌面的 Webots 窗口运行。查看执行器输出可运行
`screen -r yobogo_flat_jump_formal`；不要另起第二个 Webots 或 TCP 端口。

## 能力边界

- 仅覆盖平直无障碍 Webots 地面；尚未完成 F1B、F2、F3 阶段或最终键盘/手柄验收。
- 当前只确认正式训练已健康启动；不宣称训练完成、任一恢复验收通过、F2/F3
  验收通过或实机可用。
