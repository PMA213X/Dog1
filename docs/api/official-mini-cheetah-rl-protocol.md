# 官方 Mini Cheetah RL 协议

> 状态：接触 device tag、固定步数两阶段 RSI、TCP reset 超时对齐、
> headless 正式训练、同步死锁和 array-like 遥测修复已完成；`88/88` 单测与三项 dry-run
> 均退出码 `0`。旧 reward 的 P1 400,000 步 checkpoint 仅作历史证据，
> 本轮 reward/Gate 对齐修复后从 `0` 重新训练。
> TensorBoard 地址为 `http://127.0.0.1:6007/`。

R3 从 0 开始，阶段累计预算为
`10k / 400k / 1.5M / 4M / 6.5M / 9.5M / 14.5M / 18M / 20M`，
依次对应 P0～P7 和缓冲；步数均为四台机器人合计 transition。
R3 不加载旧 `450000` 或 `2000000` checkpoint，旧 R2 checkpoint
仅作历史归档。Gate 可在预算内提前通过；到预算未通过即记为
`failed` 并停止，不自动延长或降低门槛。

旧 R2 的 `182 s` 健康确认、40 项单测和 F1 gate 只代表历史链路状态，
不代表修正后的 R3 奖励、Gate 或训练结果。

## 观测

- 总维度：57。
- 动作：12，范围 `[-1, 1]`。
- 关节顺序：`fr/fl/hr/hl × abd/hip/kn`。

观测切片：

| 维度 | 字段 |
| ---: | --- |
| 0–12 | 关节角 |
| 12–24 | 关节角速度 |
| 24–27 | roll/pitch/yaw |
| 27–30 | 机体系速度 |
| 30–42 | 上一动作 |
| 42–45 | 机体系角速度 |
| 45–48 | `vx/vy/wz` |
| 48–49 | 跳跃请求锁存 |
| 49–50 | 跳跃阶段时间 |
| 50–51 | 机身高度 |
| 51–55 | 4 足接触 |
| 55–56 | 地形高度 |
| 56–57 | 平地标志 |

## 并行拓扑与 TCP

- 唯一训练 world：`flat_move_jump_rl.wbt`。
- Robot 名：`mini_cheetah_0` 至 `mini_cheetah_3`。
- Webots 实例：`127.0.0.1:1234`。
- TCP 地址：`127.0.0.1:11452` 至 `127.0.0.1:11455`，每台一路。
- 训练由 `SharedWorldVecEnv` 批量收发并同步 reset；评估使用单独的
  单机器人 `flat_move_jump_rl_eval.wbt`。
- 编码：UTF-8 JSON Lines。
- 消息：`hello`、`reset`、`act`、`get_rgb`、`exit`。
- 训练主进程退出后必须继续回收完整训练 PGID；Webots/controller 未全部退出
  或目标端口未释放时，不得进入 Gate。
- 训练 TCP reset/step 响应超时统一为 `180 s`，与 controller 锁步 socket
  一致；超时异常必须携带 worker、elapsed 和 reset 上下文。
- 正式 `run_stages` 训练命令使用 headless Webots，不传 `--webots-gui`；
  GUI 仅用于独立人工观察入口。
- 控制周期：`20 ms`，5 个 `4 ms` 物理步。
- reset 字段：`command`、`randomization`。
- act 字段：`action`、`command`、`jump_request`。
- state 字段：`q`、`dq`、`rpy`、`v_body`、`omega_body`、`contacts`、
  `height`、`position`、`jump_phase`、`jump_success`、`jump_landing`、`done`。

## 输入接口

- 键盘：`W/S`、`A/D`、`Q/E`、`Space`、`R`、`Esc`。
- 手柄：轴 `vy/vx/wz`、A 跳跃、B 复位。
- 训练模式由 TCP 驱动；play 模式使用本地输入。

## R3 阶段与命令契约

| 阶段 | 累计 transition | 命令/动作 |
| --- | ---: | --- |
| P0 | 0 → 10k | 固定 reset、全零命令，工程与奖励接口冒烟 |
| P1 | 10k → 400k | 全零命令、10 s（500 步）episode；280k～320k 接触奖励连续混合、300k～330k 静态课程单独淡出、340k 起小扰动 |
| P2 | 400k → 1.5M | `vx[-.10,.25]`、`vy[-.10,.10]`、`wz[-.30,.30]`，每 5 s 重采样；16 槽覆盖循环为 50% Gate forward、25% zero，并含左右横移/偏航；训练段与 Gate 间隔 50k |
| P3 | 1.5M → 4M | `vx[-.30,.60]`、`vy[-.30,.30]`、`wz[-1,1]`，每 2～5 s 重采样，约 20% 零命令 |
| P4 | 4M → 6.5M | 沿用 P3 命令，按固定顺序逐级开 DR |
| P5 | 6.5M → 9.5M | `jump_request` 上升沿、全零命令、原地跳 |
| P6 | 9.5M → 14.5M | `vx=.10→.20→.30`，再加轻转向/横移/命令切换 |
| P7 | 14.5M → 18M | InputAdapter 遥控命令和跳跃边沿 |
| 缓冲 | 18M → 20M | 不自动启用；无新批准不延长已结束阶段 |

P4 DR 顺序固定为 reset 姿态/位置 → 观测噪声 → 延迟 0～2 周期 →
摩擦 ±10% → 质量 ±5% → 电机强度 ±5% → 小幅外推。每级重跑
P1/P3 Gate，任一级 randomized `fall_rate <= 0.05` 才能继续。

## R3 奖励接口约束

- 速度跟踪必须使用负的 `||v-cmd||²`；P1 零命令不能给速度正奖励。
- `height` 必须使用负的平方高度误差：目标高度处为 `0`，高于或低于目标
  均为负值，不得出现偏离目标反而增加奖励的符号错误。
- delayed reward 和 action-rate 必须使用实际执行动作。
- `true_four_foot_contact` 只有四足同时有效接触才给正奖励，部分足接触
  由 `support_gap` 负奖励补齐，不把部分接触冒充四足。
- P1 的 `true_four_foot_contact` 权重在 280k～320k 从 `12` 连续混合到
  `3`；`support_gap` 同窗口从 `-1.0` 连续混合到 `-0.35`，P1 全程保留
  有效四足正奖励和缺足负惩罚。
- P1 `static_contact_course` 按当前四足接触比例给稠密正奖励；300k 起在
  30k 窗口内单独线性淡出，不与主奖励在同一边界硬跳。
- P2 `true_four_foot_contact` 权重为 `0.8`、`support_gap` 为 `-0.15`、
  `gait` 为 `-0.05`、`distance` 为 `0.50`；部分接触不能靠 gait 正奖励
  抵消四足接触目标。
- P2 新增仅在该阶段生效的 `command_speed_progress` 和
  `command_displacement_progress`，各权重 `1.5`：前者按平面速度/命令模长
  归一，后者按单周期机体系位移/`||平面命令||×0.02s` 归一。P1 和 P3～P7
  不启用这两项。
- 执行层 `ACTION_RATE_LIMIT=0.08`，训练和评估共用同一限幅，对齐
  `action_delta_rms <= 0.10` Gate。
- `ACTION_TARGET_RATE_LIMIT=0.03 rad/s`，仅用于策略动作映射后的关节目标；
  `apply_pd_action()` 在 `motor.setPosition()` 前按 `CONTROL_DT_SECONDS`
  将每周期目标增量限制为 `0.0006 rad`，10 s 最多变化 `0.30 rad`。
  选择依据为 `0.01` 探针 contact `1.0` 但 10 s 仅 `0.10 rad`、无法解释
  forward≈0，而 `0.75` 探针 contact 仅 `0.333`；故取比 `0.75` 低 25 倍的
  候选下界。RSI/reset 使用直接 `set_targets()`，不经过该限速。
- P1 episode 固定为 500 步；足滑只惩罚超过 `0.02 m/s` 的接触期平均滑移。
- P0/P1 加强 `foot_slip`、`action_rate` 和相邻关节速度差
  `joint_jitter` 惩罚，直接对应 P1 Gate 失败项。
- P1 关闭 `feet_air_time`、步态、距离和跳跃奖励。
- 必须分别暴露 `alive`、`height`、`roll/pitch`、零线速度/偏航、
  水平角速度、真实四足接触、默认姿态、action rate、关节速度、
  归一化扭矩、接触期足滑、非足部碰撞和摔倒分项。
- TensorBoard 必须记录 `rollout/ep_rew_mean`、
  `rollout/true_four_contact_ratio` 和 `termination_reason/*`。
- PPO loss/entropy 使用 SB3 原生 `train/loss`、
  `train/policy_gradient_loss`、`train/value_loss`、
  `train/entropy_loss`、`train/approx_kl` 和 `train/clip_fraction`，
  不重复实现一套不一致的 callback 指标。
- 接触遥测接受 `numpy.ndarray`、`list`、`tuple` 等 array-like 数据，
  仅 `foot_contact_source=node_id` 的四值样本进入比例统计。
- P2 起才逐步加入低速跟踪和轻微步态；移动奖励不得压过安全和平滑。

## Controller 初始化与能力降级

- 致命初始化分支输出稳定错误码：`missing_webots_controller`、
  `initialization_exception`、`timestep_mismatch`、`self_node_missing`、
  `robot_name_missing`、`worker_id_invalid`、`synchronization_failed`、
  `motor_device_missing`、`sensor_device_missing`、
  `device_count_mismatch`；TCP 连接错误使用 `webots_connection_failed`。
- 致命分支保留 detail、异常类型和 traceback；能力错误使用
  `RL_CAPABILITY_ERROR` 独立报告。
- torque feedback 缺失或采样周期错误时使用
  `capability_missing_pd_fallback`，不得伪造 torque feedback source。
- contact tracking、shank 解析或接触读取失败时 fail closed：接触与足滑
  清零，reward 前不接受原始假接触，source 为
  `capability_missing_fail_closed`；未映射 node 的接触也不猜测归属。
- shank 解析必须把整数 device tag 传给 `Supervisor.getFromDevice()`，
  按 `fr/fl/hr/hl_shank_link` 精确名称回溯并校验四个唯一 `node_id`。
- capability 错误只影响相应指标，不阻断初始化和 RSI 屈膝恢复。

RSI 首态协议：

- 第一阶段固定出生平移 `0.45 m`、水平姿态和零速度后收敛屈膝目标，
  最多 `500 × 4 ms`；该阶段只验证初始化，接地姿态由第二阶段独立验收；
- 第二阶段移回权威高度 `0.2713 m`，最多 `1000 × 4 ms`；
- 两阶段都使用固定 step 数，四台同步 controller 同时完成 reset，避免
  一台提前进入 `receive()` 导致其余 controller 等待仿真步死锁；
- 首态必须满足高度 `0.2713±0.03 m`、`|roll/pitch|<=0.08 rad`、
  关节目标误差 `<=0.05 rad`；
- 超限输出 `RL_RESET_TECHNICAL_FAILURE`，禁止返回约 `1.543 m` 的无效状态。
- 失败时同时输出 `RL_RSI_STAGE_DIAGNOSTIC`，包含原始 orientation、rpy、
  height、关节目标与反馈。

## R3 评估与 Gate

- Gate 启动前必须确认 `1234`、`11452-11455` 均无 `LISTEN`；真实 `LISTEN`
  冲突按 `0.25 s` 轮询、最长 `60 s` 有界等待并安全重试，等待旧
  Webots/controller 自然退出，不执行破坏性 kill。`TIME-WAIT`、
  `CLOSE-WAIT` 等非监听残留不阻断 bind。超时必须输出 timeout、attempts、
  last error、listening/active 端口诊断，并停止、不启动 `rl.evaluate`。
- 评估环境启动前同样检查 Supervisor 端口与 bridge 端口；Webots 和外部
  controller 必须使用同一个显式 Supervisor 端口 `1234`。
- Webots 若回退到其他端口，必须在 controller 启动前检测
  `Using port ... instead` 并以
  `webots_supervisor_port_mismatch` 失败，禁止 controller 连接旧 world。
- 每累计 10,000 transition 评估一次；每 20,000 transition 保存 best。
- `run_stages` 默认 10,000 transition 重启训练并触发正式 Gate；P2 专用
  50,000 transition，保证多个完整 PPO rollout。P0/P1/P4 的 10,000 特殊
  调度、P4 DR 逐级语义和各阶段总预算不变。
- 正式训练种子使用
  `BASE_TRAINING_SEED + 阶段索引×1,000,000 + 本地 phase-step offset`，
  每段命令随机数值序列不再重复。
- `EvalCallback` 在 `n_envs=4` 时使用 `eval_freq=2500`、
  `n_eval_episodes=20`、`deterministic=True`。
- 正式 Gate 每批 20 集：10 deterministic + 10 stochastic，稳定 0.5 s
  后观察 10～20 s，关键 Gate 连续 3 批通过。
- P0/P1 评估使用契约 `PHASE_RANDOMIZATION_MODES` 中的 `fixed`，不得硬编码
  `full`；P0 是四机握手、57/12、finite、奖励接口和短链路的工程 Gate，
  P1 才检查高度、四足接触、漂移、速度和抖动等物理指标。
- `steps<2`、reset 高度/姿态/关节越界、非有限值或异常接触 source 标记
  为 `technical_failure`，不进入任何物理均值；`run_stages` 将其识别为
  技术故障并停止，不当成有效 Gate 样本重试。
- P1 `true_four_contact_ratio_min=0.85`：最新 deterministic/stochastic
  为 `0.8611`/`0.8962`，该门槛仍要求绝大多数时间四足同时支撑；这是
  唯一经用户批准调整的 P1 项，其他物理阈值不变。
- 评估 episode 同时记录 `raw_displacement_m`、`planned_displacement_m`、
  `planned_residual_m` 和 `drift_component`；stop 的 `drift_m` 是原始位移，
  移动命令的 `drift_m` 是计划位移残差。mode 报告增加 `case_values`，
  stop/forward 各自给出位移、速度、命令误差与真实四足接触遥测。
- P1 高度、姿态、漂移、速度、接触、足滑、动作差和关节速度等物理阈值
  见 `docs/features/official-mini-cheetah-flat-ground-rl.md`。
- P2/P3/P4/P5/P6 的命令跟踪、摔倒率、DR 回归和跳跃率必须使用实际
  物理指标；训练 reward 只作趋势。
- 到阶段预算未通过即 `failed`，不自动延长或降低门槛。

## R3 Play

- 链路：`InputAdapter → 57 维观测 → deterministic model.predict → 12 维动作`。
- 零命令或输入超时自动回站立。
- 急停独立于模型、输入适配器和命令通道。
- 最终验收覆盖 30 s 站立、前后/横移/转向、原地跳、移动跳、超时、
  急停和连续 10 min 无崩溃。

## 安全

- 非有限动作、越界、输入超时和设备异常均回安全姿态。
- world 电机能力 `20 N·m`；屈膝复位和 RL 位置目标路径使用设备
  `20 N·m` 能力，不显式调用非零 `setTorque()`。
- 若进入显式力矩分支，`15 N·m` 仍为代码上限；当前安全异常分支只写零力矩。

历史 57/12 协议、40 项单测、TCP mock 和四机健康检查只代表旧 R2 链路。
本轮 R3 `80/80` 单测和三项 dry-run 已通过；真实 Webots 冒烟、正式训练
和 Gate 尚未执行。历史 P0 `10,000` 步在接触全零、无效首态和错误评估
随机化下生成，只作取证；P0 必须从零重训，不得宣称 R3 Gate 已通过。
