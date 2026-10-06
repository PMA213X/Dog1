# 无摄像头遥控运动与跳跃（`yobogo_loco_jump_v1`）

> 适用范围：Webots 仿真内的本体感觉策略、键盘/手柄遥控触发和训练前安全检查。
> 当前版本：`yobogo_loco_jump_v1`。
> 更新时间：2026-10-02。

本文记录 55 维观测、12 维动作、50 Hz 控制、命令条件化跳跃、五阶段预算、
checkpoint 隔离、三层监控批准门，以及“无摄像头遥控越障”的能力边界。
本文不表示已经完成正式训练，也不表示模型已部署到 YoboGo-10S 实机。

---

## 1. 当前状态

| 项目 | 状态 | 说明 |
|---|---|---|
| 契约、环境、奖励、训练入口 | **前置已放行，训练进行中** | 55/12/50 Hz、五阶段累计预算和新前缀已落到源码 |
| 本地键盘/手柄输入 | **已完成（代码）** | W/S/A/D/Q/E、Shift、Space/R 与 `/dev/input/js0` 映射已实现 |
| 三层监控与批准门 | **已获授权，运行中** | 默认仍为 dry-run/只读；本轮已显式放行并启动，当前 watchdog 持续巡检 |
| 单元测试与 mock 短链路 | **通过（最终前置 GO）** | 24 文件编译、36 单测、监控 9 项、契约/dry-run、三段 mock 录像为 exit 0 |
| 真实 Webots + TCP / CUDA 前置冒烟 | **GO** | 2,000 步整体 exit 0，`obs=55/action=12` checkpoint 可重新加载 |
| 实体手柄 | **未验证** | `/dev/input/js0` 缺失 |
| Webots 三世界 batch | **已通过** | `msl_match/parkour/parkour_dev` 均 exit 0、0 ERROR、无设备缺失 |
| `parkour_dev` 内建 50 步 | **已通过** | `obs_dim=55/action_dim=12`；不替代真实 TCP 锁步 |
| S0～S4 正式训练 | **进行中** | 2026-10-01 15:19:46 启动，当前为 S0；尚无正式 checkpoint |
| 五集确定性评估、正式第三人称录像 | **训练后待完成** | mock 结果只能证明脚本链路，不能作为模型验收 |
| UP Board 导出与实机运行 | **本轮明确不执行** | 不生成 ONNX/TorchScript，不复制到 `10.0.0.34`，不改 `robot-software` |
| heartbeat automation `yobogo-30` | **ACTIVE** | 只读巡检；不启动、不恢复训练，也不代表阶段完成 |

### 1.1 正式训练启动快照

| 项目 | 值 |
|---|---|
| 启动时间 | 2026-10-01 15:19:46 +08:00 |
| 启动命令 | `bash webots-sim/rl/start_loco_jump_training.sh --start-training` |
| 设备 / 前缀 | CUDA / `yobogo_loco_jump_v1` |
| 五阶段累计目标 | 5,000 / 200,000 / 500,000 / 800,000 / 1,200,000 |
| 训练 / monitor+watchdog / TensorBoard PID | `389037` / `389032` / `388923` |
| 训练日志 | `logs/yobogo_loco_jump_v1/train.log` |
| 监控日志 / 状态 | `logs/yobogo_loco_jump_v1/monitor.log` / `status.json` |
| TensorBoard | `http://127.0.0.1:6006/`，数据在 `runs/yobogo_loco_jump_v1/` |
| checkpoint 目录 | `checkpoints/yobogo_loco_jump_v1/` |
| 首个状态 | S0 **2,048 / 5,000，约 19 steps/s** |
| watchdog / automation | watchdog 正常；`yobogo-30 = ACTIVE` |

当前只是**训练进行中**：S1～S4 尚未完成，尚无可验收正式 checkpoint、
五集最终评估和移动/转向/跳跃正式视频。

状态更新原则：

- “代码已完成”只表示接口、校验和安全门已实现；
- “测试已完成”只表示对应命令实际退出码为 0；
- 真实 Webots + TCP 前置联调已通过，但这只构成启动前置；
- 只有五阶段完成、训练日志和正式 checkpoint 同时具备，才可写“训练完成”；
- 本轮不得把 mock、dry-run 或历史 P1–P4 成绩改写为新模型成绩。

相关文档：

- [RL 训练完全指南](./rl-training-guide.md)
- [Webots 与 RL 的本地 TCP 桥协议](../api/rl-webots-tcp-bridge.md)
- [2026-10-01 修改日志](../changes/2026-10-01.md)

---

## 2. 系统边界

```text
Webots 键盘 / Linux js0 手柄
             │  50 Hz 本机输入
             ▼
  rl_agent.py（play 模式）
             │
             ├── 55 维本体感觉 + 命令 + 跳跃锁存 ──► PPO 确定性策略
             │
             └── 12 维动作 ──► q_des = q_stand + a × [0.3, 0.5, 0.5] × 4 腿
```

训练模式改为：

```text
loco_jump_env.py ◄──JSON 行 TCP 锁步──► rl_agent.py
       │                                      │
       ├── 命令采样 / 跳跃请求                ├── Webots 20 ms 物理推进
       ├── 55 维观测与分阶段奖励              └── 12 电机位置目标
       └── PPO 训练
```

关键边界：

1. 策略输入只有契约内的 55 维本体感觉与状态，没有 RGB、深度、激光、
   距离传感器或障碍语义；
2. 相机只用于按需录像/调试，`get_rgb` 不进入 55 维观测；
3. TCP 桥默认只监听 `127.0.0.1`，不是面向 UP Board 或公网的遥控协议；
4. play 模式的键盘/手柄读取发生在 Webots 控制器所在主机，不是新的网络遥控服务；
5. 当前没有实机状态估计替换方案，Supervisor 速度、位姿和解析地形均属于仿真特权信息。

---

## 3. 55 维观测与 12 维动作

### 3.1 观测切片

前 42 维与旧 `walk_env.py` 保持一致，后 13 维为本任务新增，顺序不可修改：

| 切片 | 维数 | 字段 | 单位/含义 |
|---|---:|---|---|
| `[0:12]` | 12 | `q` | 12 关节角，rad；腿序 `fr → fl → hr → hl`，每腿 `abd → hip → kn` |
| `[12:24]` | 12 | `dq` | 12 关节角速度，rad/s |
| `[24:27]` | 3 | `rpy` | roll / pitch / yaw，rad |
| `[27:30]` | 3 | `v_body` | 机体系线速度 vx / vy / vz，m/s |
| `[30:42]` | 12 | `prev_action` | 上一拍动作，`[-1, 1]` |
| `[42:45]` | 3 | `omega_body` | 机体系角速度，rad/s |
| `[45:48]` | 3 | `cmd_vx_vy_wz` | 当前命令 `[vx, vy, wz]` |
| `[48:49]` | 1 | `jump_request` | 跳跃锁存，0/1 |
| `[49:50]` | 1 | `jump_phase_time` | 本次锁存经过时间，s，裁剪到 `[0, 1]` |
| `[50:51]` | 1 | `body_height` | 机身高度，m |
| `[51:55]` | 4 | `foot_contact` | 四足接触，0/1，顺序 `fr, fl, hr, hl` |

唯一权威定义位于 `webots-sim/rl/loco_jump_contract.py`；环境和控制器必须在
导入期与运行期做维数、字段顺序和有限值校验。

### 3.2 动作

- 动作维度：`12`；
- 动作范围：每维 `[-1, 1]`；
- 关节顺序：`fr → fl → hr → hl`，每腿 `abd → hip → kn`；
- 映射：`q_des = q_stand + a × [0.3, 0.5, 0.5]`，按 4 腿重复；
- 控制周期：20 ms，即 **50 Hz**；
- Webots 基本步长为 4 ms，每个 RL 周期推进 5 个物理子步。

---

## 4. 命令与跳跃锁存

### 4.1 命令

命令字段固定为 `[vx, vy, wz]`，逐项限幅：

| 字段 | 范围 | 启用阶段 |
|---|---:|---|
| `vx` | `[-0.3, 0.6] m/s` | S2、S3、S4 |
| `vy` | `[-0.3, 0.3] m/s` | S2、S3、S4 |
| `wz` | `[-1.0, 1.0] rad/s` | S2、S3、S4 |

环境默认每 250 步重新采样一次命令；S0/S1 强制为全 0。目标偏航按
`wz × 0.02 s` 积分，命令误差为实测 `[vx, vy, wz]` 与目标命令的 L2 范数。

### 4.2 跳跃

只有 S3/S4 允许请求跳跃。默认在第 100 步首次自动请求，之后每 250 步尝试一次。

锁存规则：

1. Space 或手柄 A 的上升沿把 `jump_request` 从 0 置 1；
2. 锁存期间重复请求被忽略；
3. 必须先观察到四足全部离地，再观察到任一足恢复接触，才可因落地清除锁存，
   避免 trot 单足轮换在触发当帧被误判为落地；
4. 若未检测到离地后恢复，锁存最多持续 1.0 s 后强制清除；
5. 机身高度相对 reset 基线增益达到 `0.04 m`，本 episode 记为跳跃成功；
6. S4 中，跳跃成功后恢复接触才记为越障成功；
7. 跳跃和越障奖励只在事件发生当步计入，不能连续刷取。

---

## 5. 五阶段累计预算

下表是**全进程累计目标步数**，不是每阶段各自重复训练到该值。

| 阶段 | CLI tag | 累计目标 | 相对上一阶段增量 | 任务 |
|---|---|---:|---:|---|
| S0 | `phase0` | 5,000 | 5,000 | 链路冒烟，不计入正式训练预算 |
| S1 | `phase1` | 200,000 | 195,000 | 站立与稳定 |
| S2 | `phase2` | 500,000 | 300,000 | 命令跟踪 |
| S3 | `phase3` | 800,000 | 300,000 | 命令条件化跳跃 |
| S4 | `phase4` | 1,200,000 | 400,000 | 移动中跳跃与固定课程越障 |

正式训练入口把 S3/S4 的累计目标限制在 `[800000, 1200000]`。阶段完成后，
监控器校验新前缀 checkpoint，再把同一个累计目标和 `--resume` 交给下一阶段。

---

## 6. Checkpoint 隔离

- 唯一前缀：`yobogo_loco_jump_v1`；
- 步数间隔：每 50,000 步；
- 时间条件：每 1,800 秒，或达到步数条件时保存，二者任一满足；
- 前缀校验按完整边界匹配，`yobogo_loco_jump_v1beta`、
  `yobogo_loco_jump_v10`、`old_...` 均会被拒绝；
- `train_loco_jump.py`、`eval_loco_jump.py`、`play_third_person.py` 和
  `rl_agent.py` play 模式共用同一校验；
- 监控恢复前还要检查 ZIP 结构与 CRC，不能只看文件存在。

play 模式的 `RL_AGENT_CHECKPOINT` 接受三类路径：

- 绝对路径；
- 控制器相对路径；
- 仓库根相对路径，例如 `checkpoints/yobogo_loco_jump_v1/yobogo_loco_jump_v1_final.zip`。

相对路径统一解析为绝对路径。启动日志必须输出解析后的 checkpoint，
便于确认 Webots 实际加载的文件：

```text
【rl_agent】play 模式：checkpoint={绝对路径} ...
```

固定启动命令：

```bash
env -u RL_BRIDGE_PORT RL_AGENT_MODE=play \
  RL_AGENT_CHECKPOINT=checkpoints/yobogo_loco_jump_v1/yobogo_loco_jump_v1_final.zip \
  RL_AGENT_DEVICE=cpu RL_AGENT_EPISODES=1 RL_AGENT_MAX_STEPS=1000 \
  __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  /usr/local/webots/webots --mode=realtime \
  webots-sim/worlds/parkour_dev.wbt
```

### 6.1 play 启动安全日志与参数

play 模式只加载已有 checkpoint，不写 checkpoint、不启动训练。启动期间使用
**1 秒稳定窗口**（50 Hz 下 50 个控制周期），并要求在判定时间内观察到
**连续 10 个控制周期四足均接触**。对应诊断包括：

```text
【play-safety】稳定窗口 step={n}/50 contacts=...
【play-safety】raw=... safe=... jump=... contacts=... height=...
```

未满足连续接触条件时在 100 个控制周期后停止 play：

```text
【play-safety】稳定接触超时：100 个控制周期内未满足连续 10 周期四足接触，停止 play
```

每个控制周期的动作安全处理为：

- `raw` 动作先裁剪到 `[-0.5, 0.5]`；
- `safe` 动作相对上一安全动作的每周期变化率限制为 `0.05`；
- 机身高度超过 `0.600 m` 视为异常离地并停止 play：

```text
【play-safety】异常离地：height=... > 0.600m，停止 play
```

这些保护只用于降低 play 启动和运行风险，不代表 checkpoint 的动作质量、
训练目标或正式评估已经通过。

本轮 checkpoint 路径与 play 安全保护**仅适用于 `RL_AGENT_MODE=play`**。
训练模式的 55/12/50 Hz 契约、TCP JSON 行锁步协议和训练入口不变；
`eval_loco_jump.py` 的正式五集评估流程也不变。

2026-09-29 及以前的 P1–P4 checkpoint 属于旧模型几何和旧观测契约，本轮一律
不得热启动、正式回放或作为验收依据。

---

## 7. 三层监控与批准门

正式训练采用三层显式批准链，任一默认入口都不启动训练：

| 层 | 入口 | 默认行为 | 放行条件 |
|---|---|---|---|
| 1：启动预检 | `start_loco_jump_training.sh` | 默认或 `--dry-run` 只打印 S0～S4 命令 | 用户明确授权后执行 `--start-training`，才写 `train_command.json`、启动 TensorBoard 和 watchdog |
| 2：命令清单门 | `loco_jump_watchdog.sh` | `status/once/dry-run` 只读 | 只有显式 `start`，且 `train_command.json` 存在，才启动 monitor |
| 3：监控动作门 | `loco_jump_monitor.py` | 无模式、`--dry-run`、`--once` 均只读 | 只有显式 `--run` 才允许启动阶段、归档、恢复或推进 |

`--once` 使用 `allow_actions=false`，只写 `status.json`，绝不启动、归档或恢复。
运行期还包含：

- PID 与 `/proc/<pid>/cmdline` 匹配，防止 PID 复用误操作；
- 日志 NaN/Inf、OOM、TCP 错误识别；
- 600 秒启动宽限和 600 秒进度停滞判定；
- 内存、项目磁盘、`/tmp` 连续越界硬停止；
- 1,800 秒窗口内最多 3 次自动恢复；
- 无有效 checkpoint、恢复超限或资源持续越界时停止并等待人工处理。

本轮已获批准并执行 `start_loco_jump_training.sh --start-training`。
当前批准状态是“训练进行中”，不是“训练完成”。

---

## 8. 键盘与手柄映射

### 8.1 键盘

Webots 键盘短按保持 0.16 s，Space 和 R 严格按上升沿触发。

| 输入 | 输出 |
|---|---|
| `W` | `vx = +0.6 m/s` |
| `S` | `vx = -0.3 m/s` |
| `A` | `vy = +0.3 m/s` |
| `D` | `vy = -0.3 m/s` |
| `Q` | `wz = +1.0 rad/s` |
| `E` | `wz = -1.0 rad/s` |
| `Shift` | `sprint=true`（当前 55 维契约没有 sprint 通道） |
| `Space` | 跳跃请求上升沿 |
| `R` | episode 复位上升沿 |

同一轴 W/S、A/D、Q/E 冲突时取最近采样键；不同轴可组合。

### 8.2 Linux 手柄

固定打开 `/dev/input/js0`，死区为 `0.12`；设备缺失时只降级为键盘，不阻塞。

| 输入 | 输出 |
|---|---|
| 轴 1 | `vx = -axis1 × 0.6` |
| 轴 0 | `vy = axis0 × 0.3` |
| 轴 2 | `wz = axis2 × 1.0` |
| A 钮（button 0） | 跳跃请求上升沿 |
| B 钮（button 1） | episode 复位上升沿 |

键盘与手柄合并时，同一速度分量的键盘非零值优先；Space/R 边沿为“或”合并。
实体手柄尚未在本轮实测。

---

## 9. 无摄像头遥控越障边界

### 9.1 可以验证

- 本体感觉平衡和 12 关节动作平滑性；
- 键盘/手柄给定 `[vx, vy, wz]` 下的命令跟踪；
- 由人或固定课程显式触发的原地跳、移动中跳；
- 固定 Webots 地形内的反应式越障、落地恢复；
- 策略在没有图像输入时对命令和 `jump_request` 的响应。

### 9.2 不能宣称

- 不能自主识别障碍类型、距离、高度、沟宽或可通行路线；
- 不能在碰撞前获得障碍语义，也不能做视觉自主导航；
- `foot_contact` 只能反馈是否接触，不能替代前向测距；
- 解析地形、Supervisor 位姿和真值速度不能直接迁移到实机；
- S4 成功不等于比赛自主越障能力，也不等于实机 sim-to-real 成功；
- RoboCup 正式比赛要求自主运行，本文的人工触发只用于实验室仿真验证。

因此，本轮能力名称应写成“**无摄像头、人工/课程触发的反应式运动与越障**”，
不得简写成“自主避障”或“自主视觉越障”。

---

## 10. 本轮不导出 UP Board

明确不在本轮范围内：

- 不导出 ONNX、TensorRT、TorchScript 或纯 actor 文件；
- 不执行 `scp`、部署脚本或任何 `10.0.0.34` 写入；
- 不修改 `YoboGo-control/robot-software`、SPIne 或 STM32 控制链；
- 不把 SB3 `.zip` 直接复制到 UP Board；
- 不进行实机 50 Hz 推理、急停、状态估计或安全回退验收。

UP Board 后续部署必须单独立项，至少先完成训练、真实 Webots 评估、
actor 导出数值一致性、UP Board p99 延迟、状态估计替换、命令超时和
站立/被动回退测试，并再次获得用户授权。

---

## 11. 测试与训练命令

### 11.1 当前允许的只读/短链路检查

```bash
cd /run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1

# 契约、环境、奖励、训练入口单测
PYTHONPYCACHEPREFIX=/tmp/codex_dog1_tests_pycache \
python3 -m unittest discover -s webots-sim/rl/tests -p 'test_*.py' -v

# 监控批准门与 mock 恢复演练
python3 webots-sim/rl/test_loco_jump_monitor.py

# 五集内存 mock 评估；不启动 Webots、不读 checkpoint
python3 webots-sim/rl/eval_loco_jump.py \
  --self-test --episodes 5 \
  --output /tmp/yobogo_loco_jump_v1_mock_eval.json

# mock 第三人称视频短链路
bash webots-sim/rl/test_loco_jump_video.sh

# 默认 dry-run，绝不启动正式训练
bash webots-sim/rl/start_loco_jump_training.sh --dry-run
```

### 11.2 已完成的真实 Webots 冒烟与正式启动

真实 Webots + TCP 的 2,000 步 CUDA 前置冒烟已整体退出码 0。
正式训练启动命令为：

```bash
bash webots-sim/rl/start_loco_jump_training.sh --start-training
```

真实冒烟必须确认：

1. Webots 启动且 `rl_agent` 连接 55/12 契约；
2. 每次 `act` 推进 20 ms，无 TCP 超时；
3. `state` 必需字段、数组长度、有限值全部通过；
4. 日志无 NaN/Inf、OOM、Address already in use；
5. 生成新前缀 checkpoint 且 ZIP/CRC 校验通过；
6. 进程正常退出，无孤立 Webots。

### 11.3 正式训练运行状态

```bash
# 已于 2026-10-01 15:19:46 执行，当前不要重复启动
bash webots-sim/rl/start_loco_jump_training.sh --dry-run
```

监控状态：

```bash
bash webots-sim/rl/loco_jump_watchdog.sh status
bash webots-sim/rl/loco_jump_watchdog.sh once
```

### 11.4 训练完成后的正式评估

当前没有正式 checkpoint，以下命令只作为后续模板：

```bash
python3 webots-sim/rl/eval_loco_jump.py \
  --model checkpoints/yobogo_loco_jump_v1/yobogo_loco_jump_v1_final.zip \
  --env-id loco_jump_env:make_env \
  --env-arg tag=S4_mobile_terrain \
  --episodes 5 --seed 20261001 --max-steps 1000 \
  --device cpu \
  --output logs/yobogo_loco_jump_v1_eval.json
```

第三人称移动、转向和跳跃录像模板见
[`play_third_person.py`](../../webots-sim/rl/play_third_person.py)
文件头部。mock 录像只验证 mp4 写入、ffprobe 和 OpenCV 解码，不构成策略验收。

---

## 12. 遗留待办

1. 只读监控进行中的 S0～S4，不重复启动；
3. 训练后完成 5 集确定性评估和真实第三人称录像；
4. 实体 `/dev/input/js0` 映射实测；
5. 若要自主识别障碍，另立非视觉测距或视觉感知任务；
6. 若要部署 UP Board，另立导出、状态估计和安全回退任务。
