# Rapid 微调奖励与 P2 Gate

## 适用范围

本功能只服务 `phase=P2` 的 Rapid 冻结模型微调，开关为
`finetune_mode=true`。有效 toe 支撑过滤和非足接触 `-60` 是公共安全修正；
其余 P0～P7 速度、姿态、步态、课程和 P1/P2 Gate 语义不变。微调只做低速
前进和停止，不改变跳跃、横移、转向、随机化或执行层安全边界。

## 奖励分支

### 零命令

零命令继续使用 Rapid 微调专用的 `true_four_foot_contact` 和
`support_gap` 权重，但只统计有效 toe：

- 有效支撑要求精确 toe 接触，同时 `height>=0.25 m` 且
  `|roll|/|pitch|<=0.25 rad`；当前没有逐 toe 法向力，该高度/姿态组合是
  由现有状态可完整计算的等效支撑判据；
- 四足同时满足有效支撑才得到 `+2.0`，低姿拖行、侧躺蹭地和非 toe 接触
  不得正分；
- 有效缺足数量按 `support_gap=-1.0` 施加负奖励；
- `gait` 不改变零命令的安全站立语义；
- 姿态、高度、足滑、action rate、joint jitter 和摔倒项全部保留；
- shank、机身及其他非 toe 接触统一记为 `non_foot_contact`，按
  `non_foot_collision=-60` 大额惩罚；不放宽摔倒终止或执行层安全。

### 非零命令

当 `finetune_mode=true`、`phase=P2` 且命令模长大于 `1e-6` 时：

- `finetune_moving_contact` 不再按任意接触足数给正奖励；两足和四足在该
  分项均为 `0`，有效支撑仍由高度/姿态过滤；
- `n<2` 时，`finetune_moving_contact = -2.0*(4-n)`；一足为 `-6`，零足为
  `-8`，避免不足两足的“贴地挪动”被其他项奖励；
- 对 `n<2.5` 增加 `finetune_moving_contact_guard`：
  `-2.0*max(0,2.5-n)`，让两足状态明显劣于三足和四足；该项连续变化，
  不引入接触阈值跳变；
- 移动分支的四足奖励、`support_gap` 和 `gait` 权重为 `0`，不再奖励贴地
  拖行；
- `command_tracking` 权重改为 `-8`，直接惩罚 stop 速度残余和 forward
  速度误差及横向漂移；
- `height` 权重改为 `-300`，保护 `height_p05` 不再低于 Gate 下界；
- `foot_slip` 对每个有效 toe 各取 3 维接触点速度的水平模长，再在所有
  有效足上取均值，禁止最后一足覆盖整组；
- Rapid 起罚线收紧为 `0.015 m/s`、权重改为 `-40`；final 实测
  `0.05394 m/s` 是 Gate `0.02 m/s` 的 2.7 倍，旧 `-20` 负反馈不足；
- 计划位移进度、姿态、默认屈膝、action rate、joint jitter、扭矩、非足
  碰撞和摔倒惩罚继续参与总奖励，非足分项提升到 `-60`。

### 源动作饱和

`rl.reward.source_action_saturation_penalty(source_action)` 接收未裁剪的
Rapid 12 维源动作，按源尺度等效边界 `(2.4, 2.0, 2.0)` 重复四腿计算。
为在裁剪前提前压低饱和，从 95% 边界开始线性爬升：

```text
ramp = max(0, abs(source_action) / limit - 0.95) / 0.05
penalty = -0.5 * mean(ramp)
```

该分项在 95% 边界处仍为 `0`，到 100% 边界达到线性满额，并继续惩罚越界
值；它不改变当前执行动作的 `[-1,1]` 裁剪、相邻动作 `0.08/frame` 限速、
双层关节目标限速或急停。自定义 VecEnv 在 `env.step()` 后调用该纯函数并把
结果加到 reward；`RewardInputs` 也支持可选 `source_action`，但同一
transition 只允许通过 canonical 分项叠加一次。

`RewardInputs` 的兼容字段为：

```python
finetune_mode: bool = False
source_action: Sequence[float] | None = None
```

## rapid-finetune Gate

使用以下命令进入新分支：

```bash
python3 -m rl.evaluate \
  --phase P2 \
  --gate-mode rapid-finetune \
  --gate \
  ...
```

`legacy` 是默认模式；`rapid-finetune` 只允许 P2，且必须同时有 `stop` 和
`forward` 两个 case。门槛保持不变，其他高度、姿态、漂移、足滑、动作差、
关节抖动和有限值门槛复用 `PHYSICAL_THRESHOLDS`：

| case | 指标 | 门槛 |
| --- | --- | ---: |
| stop | 真实四足接触比例 | `>= 0.85` |
| stop | 平面速度绝对均值 | `<= 0.12 m/s` |
| forward | 平均接触足数 | `>= 2.5` |
| forward | 零接触比例 | `<= 0.01` |
| forward | 前向速度 | `>= 0.18 m/s` |
| forward | 前向速度误差均值 | `<= 0.20 m/s` |
| 全部 | 跌倒率 | `<= 0.05` |

移动 case 只记录四足同时接触比例，不用它单独否决 Gate；平均接触足数和
零接触比例才是移动支撑门槛。stop 的四足比例和速度仍严格检查。

### final Gate 数值依据

2026-10-06 的 500k final Gate 使用 5 个 stop 与 5 个 forward、每个 1000
步，结果为 `passed=false`、`technical_failure=false`。关键实测值为：

| 指标 | 实测值 | 门槛 | 判定 |
| --- | ---: | ---: | --- |
| stop 真实四足接触比例 | `0.00882` | `>=0.85` | 失败 |
| stop 平面速度绝对均值 | `0.16534 m/s` | `<=0.12` | 失败 |
| forward `mean_vx` | `0.08751 m/s` | `>=0.18` | 失败 |
| forward 速度误差均值 | `0.23443 m/s` | `<=0.20` | 失败 |
| forward 平均接触足数 | `2.39056` | `>=2.5` | 失败 |
| forward 零接触比例 | `0` | `<=0.01` | 通过 |
| `height_p05` | `0.23623 m` | `>=0.25` | 失败 |
| 足滑均值 | `0.05394 m/s` | `<=0.02` | 失败 |
| 最大漂移 | `2.86806 m` | `<=0.35` | 失败 |
| 跌倒率 | `0` | `<=0.05` | 通过 |

这些数值直接对应 stop 有效四足 `+2/-1`、`command_tracking=-8`、
`height=-300`、逐足聚合且 `0.015 m/s/-40` 的足滑、moving 2.5 足缺口和
95% 饱和预惩罚。`height_p05=0.23623` 同时使低姿接触不计入有效支撑，
`0.05394 m/s` 足滑则按 2.7 倍越界形成约 `-1.36` 的单样本负反馈。
命令全程保持 75% `vx=0.10–0.18`、25% zero，并由 250 steps 延长到 1000
steps；actor 解冻由 20k 推迟到 100k，PPO 保持 `target_kl=0.01`。这些更新
不改变 Gate 阈值或执行层行为。

## Rapid checkpoint 评估动作链

`rl.evaluate` 只在 `--gate-mode rapid-finetune` 下接受 Rapid Dict
checkpoint：

1. 加载并校验 `current=42`、`history=630`、`Box(-100,100)^12`；
2. 用 57 维状态和逐集历史生成 `current/history` 观测；
3. 模型输出 source action 后，只调用 core 的
   `RapidActionMapper.map_source_to_current()`；
4. mapper 输出先裁剪到 `[-1,1]`，再进入原有 `0.08/frame` 限速和
   `env.step()`；
5. legacy checkpoint 不构造 Rapid context，继续执行原来的
   `_predict_action → sanitize_action → env.step` 路径。

Rapid episode 每集 reset 独立历史和 previous source action；baseline
checkpoint 使用同一加载、观测和动作门面，因此比较不会混用 source/current
动作语义。执行层、controller 目标限速和急停不由评估代码修改。

## case 遥测

`summarize_episode()` 和 `case_telemetry()` 额外输出：

- `position_x/y/z`、`mean_vx/vy/vz`；
- `raw_displacement_m`（case 中也暴露为 `actual_displacement_m`）、
  `planned_displacement_m`、`planned_residual_m`；
- `average_contact_feet`、`zero_contact_ratio`、
  `true_four_contact_ratio`。

报告同时保留原 `drift_m`、速度误差、姿态、高度、足滑和抖动指标，便于区分
“没有接触”与“接触但走偏”。

## 50k 固定基线

`baseline_evaluation_due(step, "rapid-finetune")` 在 `50_000` 的整数倍返回
`True`，legacy 模式始终为 `False`。评估 CLI 的
`--baseline-checkpoint` 会在同一 stop/forward case 上执行冻结模型，报告
输出 `baseline_report` 和 `baseline_comparison`，包括：

- `forward_speed_delta` 与 `forward_speed_improvement_ratio`；
- `forward_average_contact_delta`；
- `stop_true_four_contact_delta`；
- `improved`。

基线只做对比，不自动改变 Gate 结果，也不删除任何历史 checkpoint。

## 验收与限制

- 零命令稳定、低速前进和停止回稳是本阶段验收目标；
- 跳跃仍由现有项目训练，Rapid 微调分支不产生跳跃动作；
- 限速阶梯、急停、非有限值保护和执行动作范围不由本 Gate 放宽；
- 有效支撑只影响训练奖励统计，不改 Gate 的原始 toe 遥测定义；
- 固定 zero/forward 基线、JSONL 和 TensorBoard 需要同时观察接触、位移、
  速度、动作饱和、目标限速滞后和 PPO entropy/approx KL/clip fraction；
- 旧 P0/P1/P2 Gate 未传 `--gate-mode rapid-finetune` 时保持原语义。
- 彩色网格地面只是 Webots 可视化辅助，不是观测、奖励、接触或摩擦输入；
  `boundingObject`、`WorldInfo.contactProperties` 和物理参数保持不变。
