# Rapid Locomotion Isaac Lab 外部策略接入

## 状态

代码接入、静态/单元验证、零速与低速 headless smoke、最终 5 秒测试视频
均已通过。接口审计结论与最终验收见下文。

## 接口

- 输入 42 维：`projected_gravity(3) + command(3) + q-default(12) +
  dq×0.05(12) + previous Rapid raw action(12)`；
- 历史 15×42=630，reset 清零，仅每 10 个 500 Hz 周期追加一次；
- adaptation `630→18`，`concat(42,18)=60`，body `60→12`；
- 仅输出 actor mean；
- Rapid 默认角按 FR/FL/RR/RL 排列，动作尺度 HAA `0.125`、
  hip/knee `0.25`；
- 命令先按 YoboGo 物理范围 `vx/vy∈[-0.6,0.6]`、`yaw∈[-1,1]`
  裁剪，再乘 `[2,2,0.25]`，最后按 Rapid 官方域防御性裁剪；
- 后腿 URDF 的 `axis=-1` 与 `rpy=(0,pi,0)` 成对等效，适配层对
  RR/RL HAA **不再额外取反**；
- `a_Y=2(D_Rapid-D_Yobo)+2S_R*a_Rapid`；history 保存网络 raw
  `a_Rapid`，执行支路再做 `±0.25` OOD 门限；
- 本次回放实例使用官方参数 `Kp=20`、`Kd=0.5`、
  `effort=18/18/26 N·m`；位置仍按项目 URDF 裁剪。

## 测试

- `py_compile=0`；
- 12 项单元测试 `OK`，覆盖 48→42、命令顺序、动作公式、raw history、
  reset 首次全零历史、10 周期调度、真实 JIT dummy；
- `--help=0`；
- 零速 100 步：通过，`base_height_min=0.241727 m`、
  roll `7.859°`、pitch `1.236°`；
- 低速 200 步：通过，`base_height_min=0.250331 m`、
  roll `20.919°`、pitch `1.750°`；
- 最终录制：300 控制周期/30 次推理/0 reset，路径速度
  `0.117293 m/s`，机身最低 `0.252241 m`、roll `20.263°`、
  pitch `9.198°`，安全门槛与前腿运动 Gate 均通过。

## 最终视频与根因判断

- 路径：`/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/mini-cheetah-isaaclab/outputs/rapid-locomotion/rapid_walk_test.mp4`
- `5.000000 s`、`960×540`、`20 FPS`、`100` 帧、`h264/mp4`、
  `2338809 bytes`
- `ffprobe`、`ffmpeg` 解码、逐帧计数均退出码 `0`

## 前腿诊断

新增 `play_rapid.py` 逐腿统计并写入
`rapid_walk_test.summary.leg_stats.json`。最终前腿 Gate：

- `front_target_range_min=0.062500 rad`，要求 `>=0.04`；
- `front_actual_range_min=0.161632 rad`，要求 `>=0.05`；
- FR/FL 实际 P2P（HAA/hip/knee）：
  `0.427/0.303/0.162 rad`、`0.429/0.209/0.323 rad`；
- 前腿 raw 样本 `167/180`（`92.8%`）被 `±0.25` OOD 门限裁剪，
  力矩裁剪 3 次；后腿 raw 样本 `174/180`（`96.7%`）被裁剪，
  力矩裁剪 51 次。

动作矩阵已核对为 FR/FL/RR/RL × HAA/hip/knee，首帧偏移来自 Rapid
默认角而非索引错误。相机改为前侧视角 `eye=(-2.8,2.8,1.35)`，降低
机身对 FR/FL 的遮挡。剩余前后腿观感差异主要来自冻结 Rapid 策略在
YoboGo 动力学域外的 OOD 放大和安全钳位，不是权重加载或 shape 问题；
该风险由 `±0.25` OOD 门限、URDF 位置限制及 `18/18/26 N·m` 力矩安全
裁剪兜底。

## 五方向场景录制（2026-10-08）

`training/scripts/run_rapid_scenarios.py` 现在顺序录制
`forward/backward/left/right/yaw`。场景结构固定为：

- 200 个控制周期零命令站立；站立阶段只保持 YoboGo 几何初态；
- 运动开始时清零 Rapid 适配器，然后执行 600 个控制周期固定命令，
  即 60 次 50 Hz 推理，满足“至少 300 个固定命令周期”；
- 每段输出独立 5 秒 MP4、summary 和逐腿 `leg_stats`；
- 汇总始终写 `scenarios_summary.json`，即使单段 Gate 失败也不提前
  中止后续录制；
- yaw 场景不再错误地强制 `0.10 m` 净位移，转角仍必须达到 `10°`
  且方向正确；平移净位移仍完整记录。

### 验收结果

| 场景 | 结果 | 运动净位移 | yaw 变化 | 关键证据 |
| --- | --- | ---: | ---: | --- |
| forward | 失败 | `0.460198 m` | `50.240°` | 机体前向投影 `-0.454776 m`，roll `61.410°` |
| backward | 通过 | `0.144161 m` | `26.618°` | 无 Gate 错误 |
| left | 失败 | `0.485051 m` | `-161.788°` | 方向错误、跌倒、reset 1 次 |
| right | 失败 | `0.346483 m` | `-165.201°` | 方向错误、跌倒、reset 1 次 |
| yaw | 通过 | `0.111664 m` | `74.784°` | 无 Gate 错误 |

五段站立阶段全部通过。五个视频均为 `5.000000 s`、`960×540`、
`20 FPS`、`100` 帧、`h264/mp4`；媒体三项校验全部退出码 `0`。
汇总入口返回 `2`，因为 `all_scenarios_pass=false`；这是策略在
YoboGo 动力学域外的真实失败，不是视频缺失或测量脚本异常。

方向证据同时记录：

- 起止世界位置与 yaw；
- 净位移、逐控制周期机体前向/侧向投影；
- 平均机体 `vx/vy` 与 yaw rate；
- 姿态、终止/reset、逐腿 target/actual/raw/applied 统计。

`jump_supported=false`，证据位于
`outputs/rapid-locomotion/scenarios/jump_support.json`；没有生成跳跃
视频。

最终回归：`py_compile=0`、12 项单测 `OK`、JIT dummy
`(2,18)/(2,12)`、`--help=0`、零速 100 步 `0`、低速 200 步 `0`。
