# Webots 训练目录废弃与 R3 隔离声明

> 状态：`webots-sim/` 已废弃；R3 初始化/能力/height reward 与端口/PGID
> 修复完成，`74/74` 单测与四机 dry-run 通过。正式 P0 已训练至
> `10,000` 步，首次 Gate 技术故障，修复后真实复跑尚未完成；
> 尚无 R3 Gate 通过结论。

## 决策

`webots-sim/` 保留作历史目录，但不得作为任何新训练的代码、world、
controller、契约、环境、奖励、训练入口、评估入口、测试、checkpoint、
日志或 TensorBoard 数据来源。

唯一新训练基础目录为：

```text
/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/official-mini-cheetah/
```

## 隔离约束

- 新模型前缀固定为 `official_mini_cheetah_flat_jump_v1`。
- R2 历史命名空间为 `official_mini_cheetah_flat_jump_v1_r2`。
- R3 独立命名空间固定为 `official_mini_cheetah_flat_jump_v1_r3`。
- checkpoint、runs、logs 必须位于 `official-mini-cheetah/` 下。
- TCP 端口固定为 `11452` 至 `11455`；R2 单 world 内四台机器人各占一路。
- R3 继续使用唯一 Webots 实例端口 `1234`，不启动第二个独立 Webots。
- TensorBoard 地址固定为 `http://127.0.0.1:6007/`。
- 新代码不得导入废弃目录内容，也不得复用其 checkpoint 或日志。
- 旧 checkpoint 前缀必须在参数解析和文件加载前被拒绝。
- 经用户一次性批准，仅允许把屈膝测试的 4 个 DAE 视觉资源复制到
  `official-mini-cheetah/assets/meshes/`；复制后运行时必须只引用 official
  本地路径，不得继续引用历史目录。
- DAE 复制只恢复外观，不改变 Robot 物理、碰撞或验收配置。
- 旧 F2 checkpoint 只保留存档；R2 禁止将其作为 resume 来源。
- R3 从 0 开始，禁止加载旧 `450000` 或 `2000000` checkpoint；
  `official_mini_cheetah_flat_jump_v1_r2_2000000_steps.zip` 及其余 R2
  checkpoint 只作历史归档，不作为 R3 起点。
- R3 的 runs/checkpoints/logs 必须与 R2 分目录，禁止覆盖或混写 event。
- R3 阶段累计预算为
  `10k/400k/1.5M/4M/6.5M/9.5M/14.5M/18M/20M`；
  阶段到预算未通过即停止，不自动延长。

## 迁移状态

- 旧训练链路已停止；以下 R2 结果均标记为历史，不代表 R3。
- 4 个 DAE 已一次性复制到 `official-mini-cheetah/assets/meshes/`。
- 新 world 的 13 个视觉 URL 全部指向 official 本地资源；
  运行时静态检查确认不含历史目录 URL。
- 新 world 与屈膝模板的物理字段一致性测试通过。
- R3 controller 致命初始化按稳定错误码分支输出，并保留 detail、异常
  类型和 traceback；连接错误单独报告为 `webots_connection_failed`。
- torque/contact 指标能力与致命初始化分离：能力缺失只降级指标，接触使用
  fail closed，扭矩使用 `capability_missing_pd_fallback`，RSI 屈膝恢复
  不受能力检查阻断。
- `height` reward 已修正为“目标高度为 0、偏离目标为负”的符号。
- 首次 P0 Gate 因训练 PGID 残留和 `1234` 端口冲突发生技术故障；
  成功退出后回收训练 PGID、Gate 前检查 `1234/11452-11455`、Webots 与
  controller 共用显式 Supervisor 端口并 fail-fast 的修复已完成。
- R3 单测 `74/74` 全部 `OK`、退出码 `0`；四机 dry-run 通过、退出码 `0`。
- 正式 P0 四机训练已达到 `10,000` 步并生成 checkpoint；修复后真实
  P0 Gate 复跑尚未完成。
- 历史 `failed_*`、P0 gate 失败与 R2 失败目录均保留作归档，不删除且
  不代表当前状态。
- 历史 R2 单 world 四机器人曾由独立 `setsid` 会话运行，PID 依次为
  `212183/212185/212404`；这些 PID 和 `182 s` 健康确认仅用于追溯。
- 健康判据按 PPO rollout 级 event 增长验收，不以 event 文件存在或非空
  作为通过条件；本次 `EVENT_GROWTH_COUNT=5`。
- 历史 R2 曾从 `450,000` 热启动并产生 `2000000` checkpoint；这些文件
  已归档，不作为 R3 resume、热启动或验收来源。
- 此前启动失败和 event false positive 只作为归档，不代表当前状态。
- 历史阶段预算 `5,000 / 450,000 / 750,000 / 1,300,000 / 20,000,000`
  仅保留作旧链路记录，不适用于 R3。
- 历史四机 10k 冒烟和本轮正式 P0 训练记录仅证明链路曾运行；
  修复后 P0 Gate 复跑、P1 Gate 及后续证据仍待回填，完成前不得宣称
  R3 Gate 已通过。
- 最终证据记录于
  `docs/features/official-mini-cheetah-flat-ground-rl.md` 与
  `docs/changes/2026-10-03.md`、`docs/changes/2026-10-04.md`。
