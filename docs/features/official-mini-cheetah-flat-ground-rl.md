# 官方 Mini Cheetah 平地遥控移动与跳跃 RL（R3）

> 状态：接触 device tag、两阶段固定步数 RSI、TCP reset 超时对齐、
> headless 正式训练、同步死锁和 array-like 遥测修复已完成；`88/88` 单测及三项 dry-run
> 均退出码 `0`。P1 旧 reward 的 400,000 步 checkpoint 仅作历史证据，
> 不得续训；本轮已修正 reward/Gate 对齐并计划从 `0` 重新训练。
> TensorBoard 地址为 `http://127.0.0.1:6007/`。
> 更新时间：2026-10-05。

## 1. 目标与边界

最终目标是通过键盘/手柄在 Webots 平直无障碍地面上完成按命令站立、前进/
后退、横移、转向，以及原地跳和移动跳；本轮只训练仿真策略，不包含实机导出。

- 唯一基础目录：`official-mini-cheetah/`。
- 新运行命名空间：`official_mini_cheetah_flat_jump_v1_r3`。
- 观测 57 维、动作 12 维、50 Hz；一个 RL 周期推进 5 个 4 ms 物理步。
- 单训练 world 内固定四台机器人，出生点 `(-4,-4,0.45)`、
  `(4,-4,0.45)`、`(-4,4,0.45)`、`(4,4,0.45)`。
- 唯一 Webots 实例端口 `1234`；TCP 固定 `11452/11453/11454/11455`。
- 四路动作批量发送后统一接收；任一台结束时四台统一 reset。
- 训练使用 `SharedWorldVecEnv(4)` 和单个 RTX 4060 `cuda:0` PPO learner。
- Gate 使用单机器人 `flat_move_jump_rl_eval.wbt`，不改变四机训练拓扑。
- `docs/` 只在仓库根目录保留一份，不在其他目录复制。

R3 从 0 开始，**不加载旧 `450000` 或 `2000000` checkpoint**。旧
`official_mini_cheetah_flat_jump_v1_r2_2000000_steps.zip` 及其余 R2
checkpoint 仅作历史归档，不是 R3 resume、热启动或验收来源；禁止覆盖旧文件。

## 2. 为什么重做 R3

旧 F1 虽然通过了过松的 gate，但奖励与验收存在以下漏洞：

1. **速度符号错误**：站立阶段把接近零命令时的速度奖励写成了随速度增大的
   正项，而不是负的 `||v-cmd||²`，可能鼓励机器人乱动。
2. **伪四足接触**：用“至少 3 足”之类聚合条件冒充四足支撑，不能证明四只脚
   都稳定着地。
3. **平滑惩罚过弱**：`action rate`、关节速度和扭矩约束不足，策略可高频抖动。
4. **delayed action 奖励错位**：奖励使用排队前动作，而不是实际执行的延迟动作，
   导致训练信号与物理执行不一致。
5. **Gate 过松**：旧 gate 主要看 `fall_rate` 和 episode 长度，不能识别抖动、
   漂移、姿态峰值、足滑、延迟动作或随机策略退化。

R3 同时修正奖励、终止条件、阶段课程、随机化和物理指标验收；旧 R2 结果只能
作为历史背景，不能作为 R3 的成绩或起点。

## 3. P0～P7 累计预算与阶段任务

所有步数都是 `SharedWorldVecEnv(4)` 下四台机器人共同产生的累计 transition，
不是每台机器人各自的步数。最终预算 `20,000,000`；18M～20M 为缓冲，
不新增能力阶段。

| 阶段 | 累计范围 | 增量 | 训练目的 | 命令/初始化 | DR/奖励开关 |
| --- | ---: | ---: | --- | --- | --- |
| P0 | 0 → 10k | 10k | 工程、奖励接口、四机链路与有限值冒烟 | 固定 reset，命令全零 | `fixed`；只验证接口，不把冒烟奖励当成绩 |
| P1 | 10k → 400k | 390k | 纯站立与安全姿态 | 命令全零；10 s（500 步）episode；280k～320k 连续混合接触激励，静态课程在 300k～330k 单独淡出，340k 起加入小范围 reset 扰动 | 无移动/步态/跳跃塑形；见第 6 节 |
| P2 | 400k → 1.5M | 1.1M | 低速命令跟踪 | `vx[-0.10,0.25]`、`vy[-0.10,0.10]`、`wz[-0.30,0.30]`；每 5 s 重采样；16 槽覆盖循环含 50% Gate forward、25% zero，并覆盖左右横移/左右偏航 | 逐步加入低速跟踪；正式训练段与 Gate 间隔改为 50k，安全/平滑优先 |
| P3 | 1.5M → 4M | 2.5M | 完整低中速命令 | `vx[-0.30,0.60]`、`vy[-0.30,0.30]`、`wz[-1.00,1.00]`；每 2～5 s 重采样，约 20% 零命令 | 完整移动跟踪，仍受姿态、平滑和安全项约束 |
| P4 | 4M → 6.5M | 2.5M | 逐步 domain randomization 鲁棒化 | 沿用 P3 命令 | 按第 5 节顺序逐级开 DR；每级复跑 P1/P3 Gate |
| P5 | 6.5M → 9.5M | 3.0M | 原地跳 | `jump_request` 上升沿，命令全零 | 按准备/起跳/腾空/落地/落地后分段奖励 |
| P6 | 9.5M → 14.5M | 5.0M | 移动跳 | `vx=.10→.20→.30`，再加轻转向、横移和命令切换 | 起跳、落地和继续跟踪联合验收 |
| P7 | 14.5M → 18M | 3.5M | 键盘/手柄遥控整合 | InputAdapter 生成命令和跳跃边沿；零命令与超时回站立 | 不再扩张能力范围，重点验证安全与长期稳定 |
| 缓冲 | 18M → 20M | 2.0M | 仅在 P7 验收需要时保留 | 同 P7 | 不自动使用；未获新批准不得延长已结束阶段 |

阶段推进规则：Gate 在预算用完前通过可立即进入下一阶段；到达预算仍未通过
即记为 `failed` 并停止，不自动延长预算、不降低门槛、不自动从旧 checkpoint
恢复。

## 4. P1 站立目标、RSI 与终止条件

P1 的目标不是“只要不摔”，而是可测量的平地站立：

- RSI 机身高度以 `contract.REFERENCE_HEIGHT=0.2713 m` 为唯一权威值；
- roll/pitch 约 `0`，机体系速度约 `0`，偏航角速度约 `0`；
- 四只脚均轻触地面，禁止用“3/4 接触”替代真实四足接触；
- 10 s（500 步）episode，前 0.5 s 稳定期不计入姿态/漂移统计；
- `command=[0,0,0]` 全程为零。

终止条件至少包含：

- 机身触地；
- 任一关节越界；
- `|roll|` 或 `|pitch| > 15°`；
- 机身高度低于 RSI 目标的 60%；
- 连续 1 s 无有效支撑（按每足独立接触计算）；
- 非有限观测/动作、桥接超时或协议错误。

P1 明确关闭 `feet_air_time`、步态、距离和跳跃奖励。奖励必须让“保持目标
高度、姿态、真实四足接触、低速、平滑”优于“乱动”。

## 5. P4 Domain Randomization 顺序

DR 必须逐级启用，不能一次全开。顺序固定为：

1. reset 姿态与位置扰动；
2. 观测噪声；
3. 动作延迟 0～2 个控制周期；
4. 地面摩擦 ±10%；
5. 机器人质量 ±5%；
6. 电机强度 ±5%；
7. 最后才加入小幅外推/边界扰动。

每增加一级都要重新运行对应 P1、P3 Gate；任一级 randomized `fall_rate >
0.05` 就停止在该级，不继续叠加下一级。共享 world 时每次 reset 只采样一次
全局摩擦，其余 episode 级扰动按四台机器人独立随机化，避免四路完全同分布
复制。

## 6. R3 奖励分项

R3 必须按下列分项记录和调试，不能只看总 reward：

| 分项 | 目的/约束 |
| --- | --- |
| `alive` | 只提供基础生存信号，不能单独鼓励拖延或乱动 |
| `height` | 奖励保持 RSI/阶段目标高度 |
| `roll_pitch` | 惩罚姿态偏离和机身触地风险 |
| `zero_velocity_yaw` | P1 使用负的 `||v-cmd||²`；零命令必须惩罚任何非零速度和偏航 |
| `horizontal_angular_velocity` | 抑制无命令的快速水平旋转 |
| `true_four_foot_contact` | 只有四只脚同时有效接触才给正奖励；部分足接触不冒充四足；P2 权重由 `0.2` 提高到 `0.8` |
| `support_gap` | 对缺失的接触足数施加负奖励；P1 权重从 280k 的 `-1.0` 在 40k 窗口内连续混合到 320k 的 `-0.35`，不再在 300k 一步硬切；P2 权重为 `-0.15` |
| `static_contact_course` | P1 静态四足课程按当前接触足比例给正奖励；300k 起在 30k 窗口内单独线性淡出，避免与主奖励同时跳变；静态课程结束后仍保留 `true_four_foot_contact` 与 `support_gap` |
| `command_speed_progress` | 仅 P2 启用，按计划平面速度与命令模长归一到 `[0,1]`，权重 `1.5`；达到 `0.18 m/s` 已明显优于原地站立 |
| `command_displacement_progress` | 仅 P2 启用，按单周期机体系位移与 `||平面命令||×0.02s` 归一到 `[-1,1]`，权重 `1.5`；同时修正到位移目标 |
| `default_pose` | 轻微拉回默认屈膝站立姿态，避免极端关节构型 |
| `action_rate` | 强约束相邻动作差，修复旧平滑弱的问题 |
| `joint_speed` | 惩罚高速关节抖动 |
| `normalized_torque` | 用设备能力归一化扭矩，限制能耗和冲击 |
| `contact_foot_slip` | 只在足部接触时惩罚足端滑移 |
| `non_foot_collision` | 机身或非足部位碰撞重罚 |
| `fall` | 摔倒/机身触地重罚并终止 |

关键一致性要求：

- 所有速度跟踪项均为 `负的 ||v-cmd||²` 形式，不能再出现“速度越大奖励越高”
  的符号漏洞；
- `height` 使用“负权重 × 平方高度误差”：等于 RSI/阶段目标高度时为 `0`，
  高于或低于目标均为负惩罚，修正此前符号可能反转的问题；
- delayed action 的 reward 必须使用**实际执行动作**，不能用延迟队列之前的
  原始动作；
- `action_rate` 必须用实际执行动作的相邻差，并在 P0/P1 加强惩罚以对齐 jitter Gate；
- 执行层动作变化率限幅为 `0.08`，让实际 `action_delta_rms` 不超过 Gate 的 `0.10`；
- 策略动作映射为关节位置目标后，在 `setPosition` 前执行目标变化率限速：
  `ACTION_TARGET_RATE_LIMIT=0.03 rad/s`，50 Hz 下每周期目标增量不超过
  `0.0006 rad`，10 s episode 最多允许关节目标变化 `0.30 rad`；该限速只作用于
  策略动作路径，reset/RSI 的直接目标写入保持原样。
- 限速依据：历史隔离探针 `0.01 rad/s` 为 `360/360 finite`、contact `1.0`，
  但 10 s 只允许 `0.10 rad`，无法解释 P2 forward 持续接近 `0`；`0.75 rad/s`
  探针 contact 均值仅 `0.333`。因此选择候选下界 `0.03 rad/s`，比已出现
  接触丢失的 `0.75` 低 25 倍；后续训练必须继续监测 contact 不得低于 `0.95`
  的隔离验证目标，P1 Gate 的 `0.85` 阈值不降低。
- 动作链根因验证：原实现中单关节动作即可导致膝/abad/hip 接触丢失；
  增加目标限速后的 12 关节×30 步隔离探针为 `360/360 finite`、接触丢失
  `0` 步、每个关节首动作期四足接触比例均为 `1.0`。
- `joint_jitter` 使用相邻控制周期关节速度差，P0/P1 单独惩罚高频关节抖动；
- `foot_slip` 在 P0/P1 对超过 `0.02 m/s` 的接触期平均滑移施加惩罚，直接对应 Gate 阈值；
- P2 的 `distance` 权重提高到 `0.50`；`gait` 由正奖励改为 `-0.05`，
  部分接触不得抵消四足接触奖励；P3 及以后仍保持原 `0.25`/`0.05` 语义；
- P2～P7 的移动/跳跃奖励不得压过姿态、接触、平滑和摔倒安全项；
- P1 不得启用距离、`feet_air_time`、步态节奏或跳跃分项；
- P1 的四足奖励在 280k～320k 从 `12` 连续混合到 `3`，缺足惩罚同步从
  `-1.0` 连续混合到 `-0.35`，全程保持有效四足接触激励；
- 静态课程从 300k 起另用 30k 单独淡出，与主奖励错开硬跳；reset 扰动延后到
  340k 才启用，错开奖励分布突变；
- P5 跳跃按准备、起跳、腾空、落地、落地后五段分别计分，事件奖励不得逐帧重复。

训练曲线中的 `rollout/ep_rew_mean` 只作趋势参考，不能代替物理指标验收；
TensorBoard 还必须记录 `rollout/true_four_contact_ratio` 和
`termination_reason/*` 计数；接触通道接受 `numpy.ndarray`、`list`、
`tuple` 等 array-like 数据。
PPO loss/entropy 无须额外自定义：SB3 `PPO.train()` 已直接记录
`train/loss`、`train/policy_gradient_loss`、`train/value_loss`、
`train/entropy_loss`、`train/approx_kl` 和 `train/clip_fraction`。

## 6.1 初始化失败、能力降级与 RSI

controller 对致命初始化问题返回稳定错误码，并保留 detail、异常类型和
traceback；以下分支不得退化为笼统的“初始化失败”：

- `missing_webots_controller`：无法导入 Webots `Supervisor`；
- `initialization_exception`：`initialize()` 抛出未预分类异常；
- `timestep_mismatch`：`basicTimeStep` 不等于 `4 ms`；
- `self_node_missing`：无法取得 Robot Supervisor 节点；
- `robot_name_missing`：Robot 缺少 `name` 字段；
- `worker_id_invalid`：worker ID 不在 `0..3`；
- `synchronization_failed`：`synchronization` 缺失或不为 `TRUE`；
- `motor_device_missing`：12 路指定 motor 任一缺失；
- `sensor_device_missing`：12 路指定 position sensor 任一缺失；
- `device_count_mismatch`：motor/sensor 最终数量不是 `12/12`；
- `webots_connection_failed`：训练 TCP 连接、超时或 socket 异常。

指标能力与致命初始化分开处理：

- torque feedback 接口缺失或采样周期不匹配时记录
  `torque_sampling_failed`，`torque_capability=false`，扭矩指标使用明确的
  `capability_missing_pd_fallback`；
- 接触点跟踪、四个具名 shank 解析或接触点读取失败时记录对应
  capability error，并将 `contact_capability=false`；
- shank 解析只把整数 device tag 传给当前 Webots Python 的
  `Supervisor.getFromDevice()`；按 `fr/fl/hr/hl_shank_link` 四个精确名称
  回溯，并要求四个 `node_id` 唯一，任一缺失即 fail closed；
- contact capability 缺失时四足接触和足滑均 fail closed：接触清零、
  source 记为 `capability_missing_fail_closed`，在计算 reward 前清掉原始
  假接触，不能用零值冒充真实四足支撑；
- 无法映射到预缓存 shank node 的接触同样 fail closed，不做象限猜测；
- capability 错误只降级相应物理指标，不阻断 controller 初始化，
  也不阻断 RSI 屈膝目标设置、保持和恢复。

冷启动 RSI 采用两阶段有界稳定：

- 第一阶段显式固定出生平移 `0.45 m`、水平姿态和零速度，持续施加屈膝
  目标，最多等待 `500 × 4 ms`，直到 12 路关节目标误差 `<=0.05 rad`
  且姿态合格；该阶段只完成初始化，不冒充接地后的物理姿态；
- 第二阶段把已收敛模型移回 `0.2713 m`，最多再等待 `1000 × 4 ms`；
  首个状态必须满足高度 `0.2713±0.03 m`、`|roll/pitch|<=0.08 rad` 和
  关节目标误差 `<=0.05 rad`；
- 两个阶段均执行固定数量的 Webots step，四台同步 controller 同时完成
  reset，避免一台提前进入 TCP receive 导致其余三台等待仿真步而死锁。
- 任一验收超时或越界均输出
  `RL_RESET_TECHNICAL_FAILURE`，禁止把约 `1.543 m` 的冷启动状态返回
  给环境，也不通过放宽高度、姿态或 Gate 阈值来掩盖。
- 失败时输出 `RL_RSI_STAGE_DIAGNOSTIC`，保留原始 orientation、rpy、
  height、关节目标和反馈，供根因取证。

## 7. Gate 设计

### 7.1 通用评估口径

- `run_stages` 默认每 10,000 transition 分段并触发正式 Gate；P2 专用
  `50,000` 分段，确保包含多个完整 PPO rollout（`2048×4=8192`），避免
  每段只更新一次并丢弃约 1,808 条尾部 buffer；
- P0、P1 以及 P4 的 DR 逐级语义仍使用 10,000 分段；P2 分段种子加入
  阶段索引和本地累计步，跨段命令数值序列不重复；
- 每 20,000 transition 保存一个 best checkpoint；
- SB3 `EvalCallback` 在 `n_envs=4` 时按总 transition 计数：
  每 10,000 transition 对应 `eval_freq=2500`，`n_eval_episodes=20`，
  `deterministic=True`；
- 正式 Gate 使用 20 集：10 集 deterministic + 10 集 stochastic；
- 每集先稳定 0.5 s，再观察 10～20 s；
- 关键 Gate 必须连续 3 批通过；
- stochastic 与 deterministic 都必须满足摔倒率要求，不能只报最优随机种子；
- 报告必须保存命令误差分位数、姿态峰值、接触、足滑和动作平滑指标，
  不能只保存平均 reward。
- 训练 TCP reset/step 响应使用与 controller 一致的 `180 s` 有界超时；
  超时必须报告 worker、elapsed 和阶段上下文，不能静默跳过 reset。
- 正式 `run_stages` 使用 headless Webots（不传 `--webots-gui`），避免
  GUI 渲染拖慢两阶段 RSI 并触发 reset 等待超时；GUI 只用于人工观察的
  独立入口。
- reset 高度/姿态/关节误差越界、`steps<2`、非有限值或接触 source
  异常的 episode 标记为 `technical_failure`；这些 episode 不进入任何
  物理均值，且 Gate 只按技术故障停止，不伪装成有效物理失败样本。
- P0/P1 评估严格使用契约中的 `fixed` 随机化；P0 只执行工程 Gate，
  P1 才执行站立物理 Gate。

### 7.2 P1 站立 Gate

连续 3 批、每批 20 集全部满足：

- deterministic：摔倒率 `0`；
- 完整完成率 `>=95%`；
- 机身高度 `>=95%` 时间位于目标高度 `±0.03 m`；
- roll/pitch `>=95%` 时间 `<=0.08 rad`，峰值 `<=0.15 rad`；
- 10 s 水平漂移 `<=0.05 m`；
- 平均线速度 `<=0.02 m/s`；
- `wz RMS <=0.10 rad/s`；
- 真实四足接触 `>=85%`；这是本次唯一经用户批准放宽的 P1 项，理由为最新
  deterministic/stochastic 分别达到 `0.8611`/`0.8962`，`85%` 仍要求
  绝大多数时间四足同时支撑；
- 接触期足滑 `<=0.02 m/s`；
- `Δaction P95 <=0.03/控制周期`；
- 关节速度 RMS `<=0.50 rad/s`。

### 7.3 P2～P7 Gate

| 阶段 | Gate |
| --- | --- |
| P0 | 四机握手、57/12、finite、阶段命令、奖励分项与短链路正常；只判工程通过 |
| P2 | 零命令平面速度 `<=0.12 m/s`；forward `vx=.25` 实际 `mean_vx>=0.18`、命令误差 `<=0.20 m/s`；真实四足接触 `>=0.85`；计划位移残差 `<=0.35 m`；`fall<=5%`；其他 P1 物理项不回退 |
| P3 | stop `<=0.05 m/s`；`vx=.40` 达到 `>=.30`；`vy=.20` 达到 `>=.15`；`wz=.60` 达到 `>=.50`；误差 P95 `<=0.15 m/s`；`fall<=5%`；停止后 2 s 回稳 |
| P4 | 每级 DR 均复跑 P1/P3 Gate；全部 randomized 集合 `fall<=5%` |
| P5 | 原地跳 takeoff `>=90%`、landing `>=90%`、落地 2 s 稳定 `>=90%`、`fall<=5%`，连续 3 批 |
| P6 | `.10/.20/.30 m/s` 各档及轻转向/横移/命令切换下，起跳和落地均 `>=90%`；落地后 2 s 继续跟踪；误差 P95 `<=0.20 m/s`；`fall<=5%` |
| P7 | 最终站立 30 s；前后移动、横移、转向；原地跳和移动跳；输入超时回站立；独立急停有效；连续 10 min 无崩溃 |

P0 与 P1 不得混用：P0 只检查四机握手、57/12、有限值、奖励接口和短链路；
高度、四足接触、漂移、速度和抖动物理阈值只属于 P1～P6。除用户明确批准的
真实四足接触由 `95%` 调整为 `85%` 外，其余 P1 阈值、三批验收和失败即停
规则均未放宽。

评估报告将 `raw_displacement_m`、`planned_displacement_m` 和
`planned_residual_m` 分开记录；stop 的 `drift_m` 明确为原始位移，移动
命令的 `drift_m` 明确为计划位移残差，并由 `drift_component` 标记来源。
每个 mode 另输出 `case_values`，至少覆盖 stop/forward 的位移、速度、
命令误差和真实四足接触。

### 7.4 阶段切换进程组与端口一致性

- 训练主进程退出不等于运行时已回收；`run_training()` 必须继续回收训练
  PGID，并等待其中的 Webots、controller 全部退出；
- 每次 Gate 启动前对 `1234` 与 `11452-11455` 执行有界安全等待：每
  `0.25 s` 重查一次，最多 `60 s`，等待旧 Webots/controller 自然退出；
  `TIME-WAIT`、`CLOSE-WAIT` 等非监听残留不阻断 bind，只有真实 `LISTEN`
  参与重试。超时错误必须包含 timeout、attempts、last error、实际
  listening/active 端口和“不执行破坏性 kill”的诊断，随后不得启动
  `rl.evaluate`；
- 当前失败状态下的 P0 checkpoint 不作为 resume；旧 P1 400,000 步
  checkpoint 也仅作历史证据。`rl.run_stages` 从零启动 P0，历史
  checkpoint 和失败日志全部保留。
- 评估 world 与 controller 使用同一个显式 Supervisor 端口 `1234`；
- Webots 启动后、controller 启动前必须校验目标端口已监听，并检测
  `Using port ... instead` 回退；端口不一致立即失败，禁止 controller
  连到旧训练 world；
- 该修复只保证阶段切换的生命周期和 fail-fast，不代表 P0 Gate 已通过。

## 8. Play 遥控链路

最终 play 数据流固定为：

```text
InputAdapter（键盘/手柄）
    → 57 维观测
    → deterministic model.predict
    → 12 维关节动作
```

- 零命令或输入超时自动回站立；
- 急停必须独立于策略和命令适配器，任何模式下都可触发；
- 急停、超时和异常都不能依赖模型“学会停车”；
- play 只加载 checkpoint，不写训练 checkpoint；
- 每次启动必须打印模型绝对路径、device 和输入设备状态；
- 最终验收覆盖：站立 30 s、前后移动、横移、转向、原地跳、移动跳、
  输入超时、急停、连续 10 min 无崩溃。

## 9. TensorBoard 与证据

- TensorBoard：`http://127.0.0.1:6007/`。
- R3 runs、checkpoint 和日志必须位于 `official-mini-cheetah/` 下并使用
  `_r3` 命名空间，不与 R2 数据混写。
- 训练过程至少记录 reward 分项、命令误差、姿态、接触、足滑、动作平滑、
  episode 长度和摔倒率。
- Gate 报告和 10k 冒烟证据必须落到独立日志；测试结果只按实际退出码回填。

## 10. 测试状态与待回填项

本轮已回填：

- [x] R3 unittest：`87/87` 全部 `OK`，退出码 `0`，覆盖初始化错误分支、
  capability fail-closed/PD fallback、RSI 不受能力检查阻断和 height reward
  符号，以及训练 PGID 回收、Gate 端口 fail-fast、`TIME-WAIT` 放行和评估
  端口一致性；
- [x] 四机 dry-run：通过，退出码 `0`；
- [x] 正式 P0 四机 Webots 训练：从 `0` 达到 `10,000` 步并生成正式 checkpoint；
- [x] P0 Gate：连续 3 批工程 Gate `report.passed=true`，P0 已进入 P1；
- [x] P1 四机健康与步数持续增长曾超过 180 秒；旧 reward P1 Gate 后
  已停止，等待新 reward 从零重训；

以下项目仍待后续证据回填：

- [ ] P0～P7 真实运行证据与阶段状态机完整回归；
- [ ] P1 deterministic + stochastic 20 集 Gate；
- [ ] TensorBoard event 随 rollout 持续增长与 HTTP 健康检查；
- [ ] play 输入、超时站立、独立急停和连续 10 min 稳定性。

历史失败 checkpoint 和日志仅作取证；当前 P0 `10,000` 步、三批工程
Gate 通过和 P1 健康增长是本轮新证据。P1 物理 Gate 仍待完成，不能把
P0 工程 Gate 通过写成 P1 站立能力验收。
历史 `failed_*` 日志目录只作归档，不删除，也不代表当前测试状态。

## 11. 联网参考资料

R3 的分阶段、DR、跳跃和评估方法参考以下外部资料；具体数值仍以本项目
验收表为准：

- MIT Rapid Locomotion：https://www.roboticsproceedings.org/rss18/p022.pdf
- MIT Learning Quadrupedal Locomotion over Challenging Terrain / Jump：
  https://proceedings.mlr.press/v164/margolis22a.html
- ANYmal learning：https://arxiv.org/abs/1901.08652
- legged_gym：https://github.com/leggedrobotics/legged_gym
- Stable-Baselines3 callbacks：
  https://stable-baselines3.readthedocs.io/en/master/guide/callbacks.html
- Stable-Baselines3 evaluation：
  https://stable-baselines3.readthedocs.io/en/master/common/evaluation.html
- Webots Supervisor：https://cyberbotics.com/doc/reference/supervisor

## 12. 历史 R2 证据（仅供追溯）

R2 曾验证单 world 四机、TCP 四路握手、CUDA learner 和 TensorBoard 链路；
其 `2026-10-04 14:54:38` 的 182 s 健康确认、`40` 项单测和旧
`450000/2000000` checkpoint 均属于**历史状态**。这些证据只能证明旧链路
曾运行，不能证明修正后的 R3 奖励、Gate 或行为正确。

历史文档入口：

- `docs/changes/2026-10-04.md`
- `docs/api/official-mini-cheetah-rl-protocol.md`
- `docs/architecture/webots-sim-deprecated.md`
