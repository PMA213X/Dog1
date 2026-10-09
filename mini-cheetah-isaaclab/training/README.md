# Isaac Lab 训练目录与分阶段计划

以后所有训练都在本目录内进行：`envs/` 放 manager 环境，`agents/`
放 RSL-RL/PPO 配置，`scripts/` 放导入、训练、评估、播放入口，
`checkpoints/`、`logs/`、`runs/` 分别保存模型、日志、实验快照与训练
报告。训练脚本不得写 `../outputs/` 或项目根目录；非训练阶段的视频与
部署包由父任务另行分配所有权，本目录不擅自生成。

数据只按 [`../research/data-manifest.md`](../research/data-manifest.md)
的最终来源规则使用：`YoboGo-control/` 为实机控制/接口第一权威，MIT
官方网络数据补其缺失的运动学和物理参考，新 CAD 目录只补外形/碰撞，
4 个 DAE 只补外观。`official-mini-cheetah/` 和旧 Webots 的非 DAE 参数
全部禁用。本目录不复制外部 URDF、USD、mesh 或策略。四腿固定映射为
`leg[0]=FR、leg[1]=FL、leg[2]=RR、leg[3]=RL`，最终整机质量口径为
`9 kg`，项目关节力矩为 `17/17/26 N·m`。

## 0. 依赖与目录

- 首版固定 Isaac Lab `v2.0.2`、Isaac Sim `4.5.0.0`、Python `3.10.21`；
- 训练器优先随 Isaac Lab v2.0 环境锁定的 RSL-RL PPO，禁止混装最新版
  后继续旧 checkpoint；
- 先通过 `list_envs` 和单环境 reset，版本升级必须单独立变更记录。

证据：[Isaac Lab v2.0.0](https://github.com/isaac-sim/IsaacLab/releases/tag/v2.0.0)、
[evelyd README](https://github.com/evelyd/MiniCheetah_IsaacLabExtension/blob/main/README.md)。

当前安装锁为 Isaac Lab `2.0.2`、Isaac Sim `4.5.0.0`、RSL-RL `2.3.3`、
Python `3.10.21`、PyTorch `2.5.1+cu121`。每次训练仍须把实际
`Isaac Lab/Isaac Sim/Python/RSL-RL/CUDA` 版本写入
`runs/<task>/<stage>/<seed>/<run-id>/environment-lock.txt`；版本不一致、
DAE 无法导入 USD、GPU/CUDA 不兼容或单环境 reset 失败即停止。禁止在
资产静态验收与单环境检查通过前生成正式 checkpoint。

## 1. 资产构建与环境导入

### 1.1 从三类可信输入合成 Isaac 资产

1. **CAD 外形/碰撞**：只读导入
   `../model/mit-mini-cheetah-1.snapshot.2/steadywin_v3.STEP` 和
   `Smart Engines.x_t`，按 `INVENTORY.md` 先确认单位、原点和四腿方向；
   将机身、髋、大腿、小腿、足端拆成碰撞候选，不在 CAD 内计算动力学。
2. **MIT 网络物理参数**：按 `data-manifest.md` 固定 ORCAgym 提交
   `5aaff694ae5f1c31e08040d59787f2cca4c5cfe0`，登记连杆原点、轴、
   逐 link 质量/惯量、足端和执行器参考；不复制其 URDF/mesh。
3. **YoboGo 控制参数**：关节数组顺序、侧别符号/零位、`target_jpos`、
   `Kp/Kd`、`500 Hz`、LCM/TCP 和安全限幅全部来自 `YoboGo-control/`。
4. **DAE 外观**：只挂 4 个 `../model/visual_dae/*.dae`；挂载变换由
   CAD/人工对照生成，不读取 official-mini 或 Webots 的 Pose/物理字段。
5. 在 `../model/<asset-version>/` 生成项目自己的 URDF/USD 和
   `asset-build-report.md`，记录每个字段来源、单位、坐标和哈希；随后用
   官方[项目生成器](https://isaac-sim.github.io/IsaacLab/main/source/overview/own-project/template.html)
   接入 `envs/`，首版任务名 `YoboGo-Velocity-Flat-v0`。

**构建前必须由用户确认**：

- CAD 长度单位、机体系、图片近/远侧是否对应机体系 `+Y/-Y`；
  四腿槽位到物理腿的映射已固定，禁止再凭截图镜像；
- `9 kg` 总质量到 `base/hip/thigh/shank/foot` 的显式逐 link 分配
  和对应惯量；
- 关节位置/速度机械限位；项目力矩已选定 `17/17/26 N·m`；
- 足端接触形状、摩擦初值、DAE 挂载变换；
- RL 动作是否直接使用 YoboGo PD，以及动作尺度/归一化。

**验收**：`list_envs` 发现任务；单环境 reset/step 1000 步全部有限；
资产均在本工作区；外部模型文件复制数为 0。

**停止**：找不到 DAE/CAD、关节顺序不等于 12 路实机顺序、坐标/单位
不确定、引用了 official-mini/Webots 非 DAE 参数、出现 NaN/Inf 或产物
写到工作区外。

## 2. 静态模型验收

- 4 个 DAE SHA256 与源一致；
- 12 关节名称、轴、零位、正负号和顺序满足
  `0=FR、1=FL、2=RR、3=RL`，且无前后/左右镜像；
- 初始目标等于 YoboGo `target_jpos`，实机限位/限速优先；
- 最终整机质量口径必须为用户选定的 `9 kg`；网络 MIT simple
  合计 `8.292 kg` 与说明书约 `10.5 kg` 只作来源证据，禁止直接混用；
- 逐 link 分配必须显式记录且合计严格为 `9 kg`；未生成分配报告时，
  动力学验收保持 `FAIL/PENDING`；
- 惯量矩阵对称正定，重心、足端、碰撞和 DAE 包围盒合理；
- `500 Hz` 实机周期与物理步长/decimation 一致；
- 项目关节力矩固定为 YoboGo 源码值 `17/17/26 N·m`；静态报告必须同时
  保留原码 `TODO CHECK WITH BEN` 和说明书 `±18` 的证据记录，网络
  ORCA `18/18/28` 只作执行器能力参考，不得覆盖项目值。

**停止**：任一项失败即停止，不启动训练，不自行修改 YoboGo 实机值。

## 2.1 阶段输入、输出与产物路径

所有任务使用以下固定目录；路径均相对 `mini-cheetah-isaaclab/`，
`<run-id>` 由 UTC 时间和短随机后缀组成：

| 产物 | 工作区相对路径 |
| --- | --- |
| 环境锁与导入报告 | `training/runs/environment/<timestamp>/`、`training/runs/import/<timestamp>/` |
| 静态模型验收 | `training/runs/static-validation/<timestamp>/` |
| checkpoint | `training/checkpoints/<task>/<stage>/<seed>/<run-id>/` |
| TensorBoard/实验快照 | `training/runs/<task>/<stage>/<seed>/<run-id>/` |
| 控制台、训练、评估日志 | `training/logs/<task>/<stage>/<seed>/<timestamp>/` |
| 导入后的 URDF/USD | `model/<asset-version>/` |
| 训练/检查报告与运行清单 | `training/runs/<task>/<stage>/<seed>/<run-id>/` |
| 视频、sim2sim/部署包 | 本轮不生成，路径由父任务另行确认 |

每次运行必须同时输出：`config.yaml`、`data-manifest.sha256`、
`environment-lock.txt`、`gate.json`、`metrics.json`；checkpoint 目录还必须
写 `checkpoint-manifest.json`。缺任一文件视为产物不完整。

| 阶段 | 输入 | 输出 | 验收标准 | 停止条件 |
| --- | --- | --- | --- | --- |
| 0 环境锁定 | 已安装包、GPU/CUDA | `training/runs/environment/` | 版本与锁定值一致，`list_envs` 和 1000 步 reset/step 通过 | 版本漂移、GPU/CUDA/Kit 不兼容、NaN/Inf |
| 1 资产构建/导入 | CAD 几何、MIT 物理参数、YoboGo 控制事实、4 个 DAE | URDF/USD 位于 `model/`，报告位于 `training/runs/import/` | 字段来源可追溯，12 关节、单位、坐标、碰撞和 DAE 包围盒通过 | 引用 official-mini/Webots 非 DAE 参数、外部文件复制数不为 0 |
| 2 静态验收 | 导入资产、`data-manifest.md` | `training/runs/static-validation/` | 质量口径全部登记，惯量正定，默认姿态/限位/扭矩/500 Hz 均通过 | 质量自动缩放、未知数据源、P0 物理字段 `pending` |
| 3 平地 PPO | 锁定环境、静态 PASS、固定 seed | checkpoints/logs/runs 按上表 | 零命令站立接触 `>=0.95`；`vx=0.15` 速度误差 `<=0.10 m/s`、偏航误差 `<=0.20 rad/s`、跌倒率 `<=5%`；动作饱和 `<=10%` | 连续两个固定 seed Gate 失败、NaN/Inf、奖励符号错误、产物越界 |
| 4 Rough/DR | 平地 Gate PASS、逐级随机化配置 | 对应阶段的 checkpoints/logs/runs | 每级 fall rate、速度误差、接触均达标后才进入下一级 | 任一级跌倒率 `>5%`，或使用未登记随机化字段 |
| 5 Sim2sim | 固定 checkpoint、观测/动作/周期契约 | 本轮不执行；后续报告须先获父任务分配 | 关节目标逐步 RMS `<=0.05 rad`，无越界/NaN，站立不倒，命令方向一致 | 任一指标失败，回退平地或 rough |
| 6 部署准备 | Sim2sim PASS、YoboGo 接口、安全契约 | 本轮不执行；后续报告须先获父任务分配 | 急停、阻尼、限幅、超时、版本校验和人工审核齐全 | 硬件桥/安全契约缺失、未人工审核、未获 Git 审核 |

## 3. 平地速度 PPO

顺序：`num_envs=4` 的 1000 步 reset 冒烟 → 零命令站立 →
`vx=0.15 m/s` 单轴短测 → 固定 seed 小预算 PPO → 扩大预算。

**验收**：

- 观测/动作/奖励无 NaN/Inf，终止原因可计数；
- 站立四足有效接触比例 `>=0.95`，机身不触地；
- `vx=0.15` 平均速度误差 `<=0.10 m/s`、偏航误差
  `<=0.20 rad/s`、跌倒率 `<=5%`；
- 安全限幅有效，checkpoint/TensorBoard 只写本目录。

**停止**：连续两个固定 seed Gate 失败、奖励符号错误、接触通道不可信、
动作饱和 `>10%` 或存在未登记数据来源。

## 4. Rough 与域随机化

只有平地 Gate 连续通过才进入。按 reset 扰动 → 观测噪声 → 动作延迟 →
摩擦 → 质量/惯量 → 电机强度逐级开启；每级单独记录 fall rate、速度误差
和接触。任一级跌倒率 `>5%` 就停在当前级，不叠加下一级。

## 5. Sim2sim

固定 checkpoint、观测顺序、周期和动作归一化，在独立 sim2sim 桥重放
同一轨迹，只迁移策略输出和接口，不迁移或读取旧 Webots 模型参数。

**验收**：关节目标逐步 RMS `<=0.05 rad`，无越界/NaN，站立不倒，同
命令方向一致，并记录延迟和动作裁剪比例。失败则回平地或 rough。

## 6. 部署准备

参考 `MiniCheetah_rl_deploy` 的状态机和安全门，但保持 YoboGo 接口。
必须具备急停、阻尼、关节/扭矩/速度限幅、超时和版本校验。

**停止**：硬件桥或安全契约未完成时禁止实机 RL；人工审核和 Git 审核
未完成时不执行实机命令。

## 7. 已实现的训练代码与串行验证顺序

### 7.1 文件职责

| 文件 | 职责 |
| --- | --- |
| `envs/robot_cfg.py` | 项目 USD 路径、12 关节契约、逐关节 PD/力矩与 `ArticulationCfg` |
| `envs/velocity_flat_env_cfg.py` | 官方 manager-based 平地速度骨架与 YoboGo 500 Hz 配置 |
| `envs/__init__.py` | 注册 `YoboGo-Velocity-Flat-v0` |
| `agents/rsl_rl_ppo_cfg.py` | RSL-RL 2.3.3 PPO，默认最多 100 iteration |
| `scripts/static_check.py` | 不启动 Isaac 的语法与契约检查 |
| `scripts/list_envs.py` | 任务注册、入口、周期和 USD 状态检查 |
| `scripts/check_env.py` | 单环境 reset/step、站立、随机动作检查 |
| `scripts/train_short.py` | 100 iteration 内短 PPO 入口与产物隔离 |

固定契约为：

- `leg[0]=FR、leg[1]=FL、leg[2]=RR、leg[3]=RL`；
- 12 关节顺序固定为
  `leg0_abad_joint,leg0_hip_joint,leg0_knee_joint,...,leg3_knee_joint`；
- `physics_dt=0.0004 s`、`decimation=5`，控制周期严格为 `0.002 s=500 Hz`；
- `Kp=3`；abad/hip/knee 的 `Kd=1/0.2/0.2`；
- abad/hip/knee 力矩限制 `17/17/26 N·m`；
- 实机 `TARGET_JOINT_POS` 逐腿为 `[-0.6,-1.0,2.7]` 或
  `[0.6,-1.0,2.7]`，abad 符号按 FR/FL/RR/RL 为 `- + - +`；
- URDF/Isaac `SIM_INITIAL_JOINT_POS` 固定为
  `[0,-0.785398163,1.865468294]` × 4，root 出生高度为 `0.26 m`；
- 唯一机器人引用为
  `../model/yobogo_mini_cheetah_v1/yobogo_mini_cheetah.usd`；
- 动作项和关节观测均设置 `preserve_order=True`，reset 精确回到
  `SIM_INITIAL_JOINT_POS`（不是 `target_jpos`），并关闭随机
  `add_base_mass`。

### 7.2 串行命令

```bash
PROJECT=/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/mini-cheetah-isaaclab
ISAACLAB=/home/pma213x/IsaacLab

# 1. 无需 Isaac 的静态检查
python3 "$PROJECT/training/scripts/static_check.py"

# 2. USD 生成后检查注册、配置和入口
"$ISAACLAB/isaaclab.sh" -p "$PROJECT/training/scripts/list_envs.py" --headless

# 3. 单环境 reset/step/站立/随机动作
"$ISAACLAB/isaaclab.sh" -p "$PROJECT/training/scripts/check_env.py" \
  --mode all --headless --seed 0

# 4. 资产与单环境 Gate 均通过后才允许短 PPO
"$ISAACLAB/isaaclab.sh" -p "$PROJECT/training/scripts/train_short.py" \
  --headless --num_envs 4 --seed 0 --max_iterations 100
```

短训练产物按以下路径写入，任何其他目录都视为越界：

- checkpoint：`checkpoints/YoboGo-Velocity-Flat-v0/ppo-short/<seed>/<run-id>/`；
- TensorBoard/配置快照：`runs/YoboGo-Velocity-Flat-v0/ppo-short/<seed>/<run-id>/`；
- 控制台日志：`logs/YoboGo-Velocity-Flat-v0/ppo-short/<seed>/<run-id>/train.log`；
- 单环境报告：`runs/env-check/<run-id>/{gate,metrics}.json`。

`train_short.py` 会屏蔽 RSL-RL 的 Git 状态收集，因此训练过程不执行
Git 读写命令；训练完成只写 `status=training_completed`，
站立接触、速度误差、跌倒率与动作饱和度仍保持 `gate=pending`，不得把
短训练 checkpoint 标为可用策略。

### 7.3 当前验证状态

- 静态检查：`STATIC_CHECK_OK files=11`，退出码 `0`；
- URDF/USD：生成与转换由资产线负责，配置按约定路径留待验证；
- Isaac 导入、任务注册、单环境 reset/step、站立/随机动作和 PPO：
  按约定由父任务串行执行，本轮未提前运行；
- 任一导入或运行失败均停止，不根据失败现象自行猜测修复。

### 7.4 资产导入与单环境 Gate 实际结果

- `py_compile`、URDF `generate`、URDF 默认 `validate`、`validate --strict`、
  训练侧静态检查和 `list_envs.py` 均退出码 `0`；
- URDF→USD 首次转换在 `TERM=dumb` 下失败，改用
  `TERM=xterm-256color` 后退出码 `0`；
- 导入期间已修复膝关节初态的 `float32` 类型问题，并把
  `base_external_force_torque` 接口修正到 `base_link`；
- `check_env.py --mode all` 的“零命令站立四足有效接触比例 `>=0.95`”
  门失败，产物
  `runs/env-check/2026-10-08_0200/gate.json` 与
  `runs/env-check/2026-10-08_0210/gate.json` 均明确记录
  `status=fail`、`AssertionError`；shell 包装命令虽因收集结果返回 `0`，
  但不得据此把主测试逻辑的异常/失败记为 `PASS`；
- 最终 FK+collision 诊断位于
  `runs/env-check/2026-10-08_0130/geometry-diagnosis.log`：在
  `root=0.30 m` 时足端最低世界 `z≈0.24606 m`，按目标姿态实现足地接触
  需要 `root≈0.053938 m`，与约 `0.26 m` 的预期站高矛盾；
- 诊断运行中接触力曾达到 `146 N`/`174 N`，说明 PhysX 接触传感器和
  碰撞查询可用，失败不是“传感器完全无数据”；
- 当前证据更倾向 YoboGo `target_jpos` 直接映射 MIT URDF 时轴方向、
  关节零偏或初态组合不自洽，尚不能判定是接触阈值问题。

### 7.5 资产拆分修复复测

2026-10-07 按只读诊断证据完成最小修复：保持 ORCAgym axis/RPY 不变，
拆分实机 `TARGET_JOINT_POS` 与 `SIM_INITIAL_JOINT_POS`，仿真初态改为
`[HAA=0,HFE=-0.785398163,KFE=1.865468294]` × 4，root 出生高度为
`0.26 m`，并将 4 个 foot visual origin 与 collision 对齐为
`(0,0,0.024)`。该 offset 是 foot link 局部坐标，必须随 foot 姿态旋转后
再求 world z；把它误当成世界竖直偏移会得到错误的 `-0.247310396 m`。
未修改 9 kg、17/17/26 N·m、0.002 s、PD 或受保护源目录。

复测结果：

- `py_compile`、`generate_urdf.py`、`validate_urdf.py` 默认与
  `--strict`、`static_check.py`、`list_envs.py`：退出码均为 `0`；
- `TERM=xterm-256color` URDF→USD：退出码 `0`；
- `check_env.py --mode all --seed 0`：退出码 `0`，`gate.json`
  `status=pass`，`metrics.json` `check_count=1657`；
- reset 实测 `root_z=0.2599999905 m`、
  `foot_collision_min=-0.2600000799 m`、
  `base_collision_min=-0.0400000066 m`；
- 站立四足有效接触比例门通过，reset/step、随机动作与根高检查均通过；
- 证据目录：`runs/env-check/2026-10-07_225234_repair/`。

### 7.6 正式 PPO 后台运行与训练规划

资产与单环境 Gate 通过后，正式 RSL-RL PPO 已由用户级 systemd 服务放入
真正后台，生命周期不依赖当前 Codex 会话：

- systemd unit：`yobogo-ppo-20261007233458`；
- Main PID：`2064669`；
- 后台归属：`PPID=1574`（常驻 `systemd --user` 管理器）、
  `SID=PGID=2064669`、`TPGID=-1`、`TTY=?`，训练主进程已成为独立会话
  leader；Python 子进程 `2064690` 与主进程同属该服务 cgroup/会话，
  不再绑定 Codex 或子代理终端，父会话退出不会发送 `SIGHUP`；
- 控制目录：`logs/formal-ppo_2026-10-07_233458_systemd/`；
- 控制日志：`logs/formal-ppo_2026-10-07_233458_systemd/train.log`；
- 状态与证据文件：同目录 `pid.txt`、`status.txt`、
  `systemd-active-state.txt`、`process-evidence.txt`、`scan-report.txt`；
- 本次 run：
  `logs/rsl_rl/yobogo_velocity_flat/2026-10-07_23-35-03_repair-v1-systemd_2026-10-07_233458/`；
- resume 来源：
  `2026-10-07_23-08-19_repair-v1-resume_2026-10-07_230815/model_1800.pt`；
- 最终核验快照（2026-10-07 23:40:06 +0800）：旧线索 PID `2032357`
  已不存在；unit 为 `active/running`，当前迭代 `2152/4800`，最新
  `model_2150.pt`，正式 checkpoint 共 `46` 个，TensorBoard event 已生成；
- GPU 进程为 Python PID `2064690`，显存 `2359 MiB`，未发现第二个正式
  PPO 训练进程，避免恢复启动造成双训练；
- `formal-ppo_*_systemd/train.log` 一次性异常扫描中 OOM、CUDA error、
  Traceback、Killed、NaN、Inf 均为 `0`；之后不持续轮询。

RSL-RL 从 `1800` 续训时，`--max_iterations 3000` 表示追加 3000 个
learning iteration，因此最终目标是总 `4800` iteration。每个 iteration
包含 `4 env × 24 steps/env = 96` 个环境步；每 50 iteration 保存一次
checkpoint。

训练规划如下（同一条连续 PPO 进程的监控/验收里程碑，不改变已加载参数）：

| 步骤名称 | 总 iteration / 环境步数 | 用途 | 23:40:06 进度 | checkpoint 频率 |
| --- | --- | --- | --- | ---: |
| 预热与续训基线 | `0–1800` / `0–172800` | PPO 预热、接触/姿态稳定，形成 `model_1800.pt` | 已完成 | 每 `50` iteration |
| systemd 续训稳定 | `1801–2400` / `172801–230400` | 检查有限值、四足接触与 checkpoint 连续性 | 进行中，`2152/4800`，最新 `model_2150.pt` | 每 `50` iteration |
| 速度跟踪重点 | `2401–3600` / `230401–345600` | 观察线速度/偏航跟踪、奖励和动作平滑度 | 未到达 | 每 `50` iteration |
| 收敛与 Gate 准备 | `3601–4800` / `345601–460800` | 最终收敛与 checkpoint 保留；完成后独立执行速度误差、跌倒率和动作饱和度 Gate | 未到达 | 每 `50` iteration |

TensorBoard 已由用户级 systemd 持久化，`6006` 被既有服务占用，因此只
绑定 localhost 的 `6007`：

```bash
systemctl --user status yobogo-tensorboard-20261007233806.service
```

- unit：`yobogo-tensorboard-20261007233806`；
- PID：`2069531`，`PPID=1574`、独立 `SID=2069531`、`TTY=?`；
- 绑定：`127.0.0.1:6007`，已通过 HTTP `205748` 字节响应核验；
- 控制目录：`runs/tensorboard-formal-ppo/`；
- event 目录：
  `logs/rsl_rl/yobogo_velocity_flat/`；
- 本地链接：[http://127.0.0.1:6007/](http://127.0.0.1:6007/)。

**当前状态**：后台 PPO `RUNNING`；训练、独立速度/跌倒率 Gate 与最终
checkpoint 验收尚未完成。
