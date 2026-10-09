# Rapid Locomotion Isaac Lab 适配器 API

## `RapidLocomotionModel`

路径：`mini-cheetah-isaaclab/training/external_policy/rapid_locomotion/model.py`

- `dummy_forward(num_envs) -> (raw_action, latent)`；
- `__call__(current_observation42, history630) -> (raw_action12, latent18)`；
- 权重目录必须包含 `adaptation_module_latest.jit` 与
  `body_latest.jit`，缺任一文件直接报错；
- 输入必须为 `float32` 且有限，输出非有限直接报错。

## `RapidLocomotionAdapter`

路径：`mini-cheetah-isaaclab/training/external_policy/rapid_locomotion/adapter.py`

- `reset(env_ids=None)`：清零历史、Rapid 上一动作、保持动作和调度；
- `step(yobogo_observation48, reset_env_ids=None) -> AdapterStep`：
  每 10 个控制周期推理一次，其他周期保持上一关节目标；
- `AdapterStep.yobogo_action` 为可直接交给现有 `JointPositionAction`
  的 12 维动作；
- `AdapterStep.rapid_raw_action`/`policy_raw_action` 保持网络原始
  Rapid raw action，history 也记录 raw；
- `AdapterStep.applied_rapid_action` 是执行前 `±0.25` OOD 门限结果；
- `AdapterStep.position_clipped`/`torque_clipped` 分别报告 URDF 与
  `Kp=20,Kd=0.5,effort=18/18/26` 安全裁剪；
- `AdapterStep.flattened_history` 只在推理周期非 `None`。

`play_rapid.py` 还输出 `leg_motion_stats`，按 FR/FL/RR/RL 和
HAA/hip/knee 记录：

- `target`/`actual` 的 `min`、`max`、`rms`、`range`、`motion_rms`；
- `raw_action`/`applied_action` 的 `min`、`max`、`rms`、`range`；
- 每关节 `position_clip_count`、`torque_clip_count`；
- 前腿 Gate `_gate.front_target_range_min` 与
  `_gate.front_actual_range_min`，默认阈值分别为 `0.04 rad`、`0.05 rad`。

异常约定：shape、NaN/Inf、权重缺失、非 `float32`、非 10 周期均直接抛错，
不得静默回退到旧 RSL-RL loader。命令映射固定为“物理裁剪 →
`[2,2,0.25]` → Rapid 域防御裁剪”；后腿 HAA 不做额外符号反转。

## 场景指标与汇总 API

`play_rapid.py` 的场景 summary 在原有字段上新增：

- `scenario_settle_metrics`：站立阶段起止世界位置/yaw、净位移、
  yaw 变化、机身高度与 roll/pitch 极值、终止/reset 和
  `gate_errors/pass`；
- `scenario_metrics.mean_body_velocity_m_s`：运动阶段平均机体
  `[vx,vy]`；
- `scenario_metrics.mean_body_yaw_rate_rad_s`：运动阶段平均机体
  yaw rate；
- `scenario_metrics.body_forward_projection_m` /
  `body_lateral_projection_m`：按每步机体 yaw 投影累计的净位移；
- `scenario_metrics.displacement_ok`：平移场景要求净位移
  `>=0.10 m`；yaw 场景不受该平移下限约束，但 yaw Gate 仍要求
  `|yaw_change|>=10°` 且符号与命令一致。

方向 Gate 继续要求主投影达到 `0.10 m`、占总位移比例 `>=0.60`
且符号正确；本轮没有通过降低阈值掩盖失败。

`training/scripts/run_rapid_scenarios.py` 暴露以下运行行为：

- 固定场景命令：
  `forward=[0.6,0,0]`、`backward=[-0.6,0,0]`、
  `left=[0,0.6,0]`、`right=[0,-0.6,0]`、`yaw=[0,0,0.5]`；
- 每段 `settle=200`、`motion=600`、`video_steps=800`、
  `fps=20`、`sample_every=8`；
- 单段 Gate 失败仍返回并写入 summary，继续录制后续场景；
- `scenarios_summary.json` 包含 `scenarios`、`hard_errors`、
  `jump_support` 和 `all_scenarios_pass`；
- 全部媒体与 Gate 通过时返回 `0`；存在 Gate/硬错误时返回 `2`，
  但仍保留全部可用证据。
