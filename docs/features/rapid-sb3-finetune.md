# Rapid 自定义 SB3 微调

## 当前状态

本功能已完成代码、500k 训练和一次 final Gate 评估；该 Gate 未通过，当前
最小改进已针对失败指标调整训练奖励、命令保持和更新稳定性。它把固定提交
Rapid Mini Cheetah 权重接入现有 SB3 PPO，只验收平地低速前进和停止；横移、
转向、跳跃、域随机化和实机参数不属于本阶段。

## 训练接口

```bash
python3 -m rl.train \
  --phase P2 \
  --init-from rapid \
  --num-envs 4 \
  --dry-run
```

- `--init-from rapid` 仅允许 P2，目标固定为累计 `500,000` transition，
  随机化强制为 `fixed`；
- `--pretrained-dir` 默认指向
  `official-mini-cheetah/external_models/rapid_locomotion_f5143ef/`，
  加载时仍执行固定提交、MIT LICENSE 与全部 SHA256 校验；
- 不传 `--init-from rapid` 时，原 57 维 MlpPolicy 和 P0～P7 路径保持不变；
- 本功能不修改、覆盖或删除历史 checkpoint，也不自动启动训练。

## Dict 观测与逐环境历史

`RapidFinetuneVecEnv` 包装底层 `SharedWorldVecEnv(4)`：

| 键 | 维度 | 含义 |
| --- | ---: | --- |
| `current` | 42 | Rapid 当前帧，包含重力、缩放命令、源顺序关节状态和上一源动作 |
| `history` | 630 | 15×42 历史，不足 15 帧用首帧补齐 |

四个 worker 各自维护独立历史。普通 step 使用本步源动作生成下一观测；
同步 terminal 使用 `info["terminal_observation"]` 的原始 57 维终止状态，
只读生成终止 Dict 后立即清空该 worker，再为自动 reset 的新 episode 重新
用首帧补齐。禁止跨 episode 复用历史或上一动作。

## 自定义 Policy 与初始化

`RapidActorCriticPolicy` 从 `ac_weights_last.pt` 严格载入：

- adaptation：`630→256→32→18`，ELU；
- actor：`60→512→256→128→12`，ELU；
- critic：`60→512→256→128→1`，输入同样使用 adaptation latent；
- 12 维 `log_std = log(源 std)`；
- critic-only 阶段冻结 actor 和 adaptation，只更新 critic；
- 离线测试中，相同 `current/history` 下确定性动作与
  `RapidPolicyAdapter` 的源动作最大绝对误差为 `0`，要求门槛 `<=1e-5`。

SB3 checkpoint 通过 `RapidPPO` save/load 往返；optimizer 的 actor、critic、
adaptation 是三个独立参数组。

## 动作接口与安全边界

- 策略动作空间保持 Rapid 源语义 `Box(-100,100)^12`；
- `RapidActionMapper` 是 play、训练与 Gate 共用映射门面：
  `0.125/0.30`、`0.25/0.50`、`0.25/0.50`；
- 映射后裁剪 `[-1,1]`，随后才交现有 `0.08/frame` 归一化动作限速和
  controller 目标限速；
- 历史中的 previous action 始终保存映射前源动作；
- 源动作从等效边界 `(2.4,2.0,2.0)` 的 95% 开始叠加
  `source_action_saturation_penalty()` 预惩罚，边界处线性到达满额；该
  预惩罚不放宽执行层边界；
- Rapid VecEnv 创建后把四个 `MiniCheetahFlatJumpEnv.finetune_mode` 设为
  `true`，环境将其原样传给 `RewardInputs`；旧 SB3 路径不包装环境，保持
  `false`。若底层缺少该接口则 fail closed；
- 饱和惩罚以 `reward_parts.source_action_saturation` 为 canonical 分项：
  env 已提供非零同号惩罚时不重复叠加，否则 VecEnv 只补一次。

## 500k 微调课程

| 累计步数 | 冻结策略 | 命令 |
| ---: | --- | --- |
| `0–20k` | 只训练 critic；actor、log_std、adaptation 冻结 | 75% `vx=0.10–0.18`，25% 零命令 |
| `20–100k` | actor 解冻由旧 `20k` 推迟到 `100k`，本段保持 critic-only；adaptation 冻结 | 同上 |
| `100–300k` | actor+critic；adaptation 冻结 | 同上 |
| `300–500k` | 仅当 200k～300k 前进进度已平台才以 `1e-6` 解冻 adaptation；否则继续冻结 | 同上 |

命令保持周期由旧 `250` transition 改为 `1000` transition，对应一个完整
1000 步 Gate episode；每个 episode 内保持同一命令，同步 reset 后再抽样。
本阶段全程只采样 75% `vx=0.10–0.18` 和 25% 零命令；`100k` 只解冻 actor，
不切换到 `vx=0.18–0.25`、`vy` 或 `wz`，也不加入命令噪声、延迟、质量或
外力随机化。adaptation 平台判定只统计 `vx>0` 的前进样本。

PPO 参数：

| 参数 | 值 |
| --- | ---: |
| `n_steps` | `512 / env` |
| `batch_size` | `256` |
| `n_epochs` | `5` |
| actor LR | `1e-5` |
| critic LR | `1e-4` |
| adaptation LR（启用时） | `1e-6` |
| `ent_coef` | `0.003` |
| `target_kl` | `0.01` |

## final Gate 依据与最小改进

`rapid_finetune_eval_20261006_223000/final_gate.json` 的 5+5 个 1000 步
episode 结果为 `passed=false`、`technical_failure=false`。主要失败值与门槛：

| 指标 | final Gate 实测 | 门槛 | 对应改进 |
| --- | ---: | ---: | --- |
| stop 真实四足接触比例 | `0.00882` | `>=0.85` | 仅有效四足 `+2`、缺足 `-1`，低姿接触不计数 |
| stop 平面速度 | `0.16534 m/s` | `<=0.12` | `command_tracking` 权重改为 `-8` |
| forward `mean_vx` | `0.08751 m/s` | `>=0.18` | 同上，并保持完整 episode 命令 |
| forward 速度误差 | `0.23443 m/s` | `<=0.20` | 同上 |
| forward 平均接触足数 | `2.39056` | `>=2.5` | 低于 2.5 足的连续缺口惩罚 `-2/足` |
| `height_p05` | `0.23623 m` | `>=0.25` | `height` 权重改为 `-300` |
| 足滑均值 | `0.05394 m/s` | `<=0.02` | 每足聚合、`0.015 m/s` 起罚、权重 `-40` |
| 最大漂移 | `2.86806 m` | `<=0.35` | 命令 250→1000 steps，并加强跟踪 |

同时，500k 训练末期 `action_saturation_rate` 最高约 `0.271`，因此源动作
饱和惩罚改为从 95% 边界开始预惩罚，避免边界处梯度仍为零。移动接触分项
不再按接触足数直接给正分；有效支撑要求精确 toe 接触且 `height>=0.25`、
`|roll|/|pitch|<=0.25`。shank、机身等非 toe 接触统一由
`non_foot_collision=-60` 大额惩罚。有效支撑过滤和非足安全惩罚是公共安全
修正，legacy checkpoint 的速度、姿态、步态、课程和执行限幅仍不改变。
训练 world 的 1 m 彩色网格是纯视觉增强，未改变碰撞、摩擦或物理地面。

## 遥测与 Gate

除 SB3 原生 PPO 指标外，TensorBoard 新增：

- `rapid/action_saturation_rate`
- `rapid/source_action_abs_mean`
- `rapid/speed_progress_mean`
- `rapid/stage`
- `rapid/adaptation_platform`
- `rapid/{actor,critic,adaptation}_lr`

Rapid stop/forward Gate、50k 固定基线比较和奖励分支见
`docs/features/rapid-finetune-reward-gate.md`。

## 测试

- `tests/rl/test_rapid_finetune.py` 覆盖 CLI 500k/fixed、冻结输出一致性、
  actor 解冻阈值、`target_kl`、全程 forward/zero 命令、1000 步保持、独立 LR、
  PPO save/load、动作映射、95% 饱和预惩罚、terminal 原始观测、历史 reset
  和统一映射门面；
- `tests/rl/test_contract_reward.py` 覆盖 `-8/-300/-40` 权重、有效支撑、
  stop `2/-1` 接触、moving 只罚不足、逐足滑移聚合、非足 `-60` 和饱和分项
  只叠加一次；
- `tests/rl/test_world_and_cli.py` 覆盖彩色网格纯视觉和物理不变；
- 全量 `tests/rl` 预期 `146/146 OK`；
- `py_compile`、Rapid 与 legacy `rl.train --dry-run` 均退出码 `0`。

## 遗留风险

- 真实四机 `zero/forward 1000` Webots 冒烟已执行并通过；
- 源 critic 的奖励尺度与当前 P2 不完全一致；
- adaptation 解冻可能破坏稳定步态，必须依据 300k 平台判定并保留回退点；
- 动作目标限速已通过 200 步短测，但完整 500k 仍需持续监控目标滞后、
  动作饱和、NaN、跌倒率和 `approx_kl`。
- final Gate 仍需使用改进后的课程和奖励重新训练、重新评估；本轮文档不把
  未通过的 final Gate 误写为验收通过。
