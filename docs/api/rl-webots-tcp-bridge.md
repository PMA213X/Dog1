# RL Webots TCP 桥协议（`yobogo_loco_jump_v1`）

> 状态：接口代码与真实 Webots TCP 前置冒烟已通过；正式训练进行中。
> 更新时间：2026-10-01。
> 权威源码：`webots-sim/rl/loco_jump_contract.py`、
> `webots-sim/rl/loco_jump_env.py`、
> `webots-sim/controllers/rl_agent/rl_agent.py`。

本协议只描述**开发机本机**的 Gym 环境与 Webots Supervisor 控制器之间的
JSON 行 TCP 桥。它不是 `YoboGo-control` 的 TCP 1986 协议，不是 UDP 运动指令，
也不是 UP Board 远程遥控协议。

正式训练启动快照：

- 启动时间：**2026-10-01 15:19:46 +08:00**；
- 命令：`bash webots-sim/rl/start_loco_jump_training.sh --start-training`；
- CUDA，新前缀 `yobogo_loco_jump_v1`；
- 累计目标：**5,000 / 200,000 / 500,000 / 800,000 / 1,200,000**；
- PID：训练 `389037`、monitor/watchdog `389032`、TensorBoard `388923`；
- 日志：`logs/yobogo_loco_jump_v1/train.log`、`monitor.log`、`status.json`；
- TensorBoard：`http://127.0.0.1:6006/`，数据目录
  `runs/yobogo_loco_jump_v1/`；
- checkpoint 目录：`checkpoints/yobogo_loco_jump_v1/`；
- 首状态：S0 **2,048 / 5,000，约 19 steps/s**；
- watchdog 正常，heartbeat automation `yobogo-30 = ACTIVE`。

这仍是**训练进行中**：尚无正式 checkpoint、最终五集评估和正式视频，
不得将 TCP 前置冒烟或启动状态写成训练完成。

---

## 1. 连接与锁步

| 项目 | 约定 |
|---|---|
| 服务端 | `loco_jump_env.py` |
| 客户端 | `rl_agent.py` |
| 默认主机 | `127.0.0.1` |
| 默认端口 | `11451`，可由 `RL_BRIDGE_PORT` 或 `bridge_port` 覆盖 |
| 编码 | UTF-8 JSON，每行一个对象，以 `\n` 结尾 |
| 控制周期 | 20 ms（50 Hz） |
| Webots 物理步长 | 4 ms；一个控制周期推进 5 个子步 |

连接顺序：

1. 环境生成运行期世界、监听端口并启动 Webots；
2. 控制器连接环境，发送 `{"type":"hello","timestep":4}`；
3. 环境发送 `reset`，控制器复位后返回一个 `state`；
4. 此后每个 `act` 必须返回一个 `state`，形成严格锁步；
5. `get_rgb` 不推进仿真，仅在录像/调试模式请求一帧。

---

## 2. 环境到控制器

### 2.1 `reset`

```json
{"type":"reset","command":[0.0,0.0,0.0],"jump_request":0}
```

| 字段 | 类型 | 说明 |
|---|---|---|
| `type` | string | 固定为 `reset` |
| `command` | number[3] | `[vx, vy, wz]`，按契约限幅 |
| `jump_request` | 0/1 | reset 时固定清零 |

控制器当前复位 12 关节、动作、命令和自身跳跃状态后返回 `state`。

### 2.2 `act`

```json
{
  "type":"act",
  "a":[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],
  "q_des":[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],
  "command":[0.4,0.0,0.0],
  "jump_request":0
}
```

| 字段 | 类型 | 约束与实际语义 |
|---|---|---|
| `type` | string | 固定为 `act` |
| `a` | number[12] | 必填；裁剪到 `[-1,1]`，控制器实际执行的策略动作 |
| `q_des` | number[12] | 环境按 `q_stand + a×scale` 生成；控制器当前重新由 `a` 计算，不单独应用该字段 |
| `command` | number[3] | `[vx,vy,wz]`；训练期由环境维护命令采样、误差和观测 |
| `jump_request` | 0/1 | 训练期由环境维护自动请求、锁存和奖励事件 |

说明：TCP 训练模式下，`command` 和 `jump_request` 作为环境侧调度状态进入
55 维观测；控制器只对 `a` 做关节位置控制。人工键盘/手柄输入只属于
`RL_AGENT_MODE=play` 的本机回放模式，不通过训练 TCP 桥传入。

### 2.3 `get_rgb`

```json
{"type":"get_rgb","camera":"third"}
```

`camera` 可取：

- `front`（默认）：前视 `front_camera`；
- `third`：第三人称跟随相机 `third_camera`。

返回：

```json
{"type":"rgb","w":640,"h":480,"data":"<base64 RGB>"}
```

RGB 帧不进入策略观测；禁用渲染或相机不存在时返回 `w=0,h=0,data=""`。

### 2.4 `exit`

```json
{"type":"exit"}
```

控制器关闭 TCP 连接并退出。

---

## 3. 控制器到环境的 `state`

必需字段：

| 字段 | 类型 | 长度/值域 | 含义 |
|---|---|---|---|
| `type` | string | `state` | 消息类型 |
| `q` | number[12] | rad | 关节角，`fr/fl/hr/hl × abad/hip/kn` |
| `dq` | number[12] | rad/s | 关节角速度 |
| `rpy` | number[3] | rad | roll / pitch / yaw |
| `omega` | number[3] | rad/s | 机体系角速度 |
| `v_body` | number[3] | m/s | 机体系线速度 |
| `contacts` | integer[4] | 0/1 | 足端接触，顺序 `fr,fl,hr,hl` |
| `height` | number | m | 世界系机身高度 |
| `jump_phase` | number | `[0,1]` | 当前锁存经过秒数，裁剪到 `[0,1]` |
| `done` | boolean | true/false | `|roll|>0.8`、`|pitch|>0.8` 或 `height<0.12` |

环境对所有数组做长度和 NaN/Inf 校验；任何缺失、错误长度、非有限数都会使
当前 step 失败，不允许静默降级。

控制器还保留以下旧字段，但它们**不参与 55 维观测**：

| 兼容字段 | 语义 |
|---|---|
| `v` | 世界系线速度 |
| `w` | 世界系角速度 |
| `x`, `y`, `z` | 世界系机体位置 |
| `body_height` | 与 `height` 同值 |
| `jump_phase_time` | 与 `jump_phase` 同值 |

---

## 4. 由 `state` 组装的 55 维观测

| 顺序 | 字段 | 维数 |
|---:|---|---:|
| 1 | `q` | 12 |
| 2 | `dq` | 12 |
| 3 | `rpy` | 3 |
| 4 | `v_body` | 3 |
| 5 | `prev_action` | 12 |
| 6 | `omega` | 3 |
| 7 | 当前 `command` | 3 |
| 8 | 当前跳跃锁存 | 1 |
| 9 | `jump_phase` | 1 |
| 10 | `height` | 1 |
| 11 | `contacts` | 4 |
| **合计** |  | **55** |

其中 `prev_action`、`command` 和跳跃锁存由环境维护；TCP `state` 只提供
本体状态。策略动作始终是 12 维。

---

## 5. 命令、跳跃与 checkpoint 边界

命令限幅：

- `vx ∈ [-0.3, 0.6] m/s`
- `vy ∈ [-0.3, 0.3] m/s`
- `wz ∈ [-1.0, 1.0] rad/s`

跳跃锁存：

- 仅 S3/S4 可请求；
- Space/手柄 A 为上升沿；
- 全离地后恢复接触，或经过 1.0 s，清除锁存；
- 成功阈值为相对 reset 高度增益 `0.04 m`。

所有正式模型、评估和回放只接受文件名前缀
`yobogo_loco_jump_v1`（允许随后为 `_`、`-`、`.`）。旧
`ppo_walk`、`phase1`、`p3_turn`、`p4_stairs` 等 checkpoint 必须拒绝。

---

## 6. 兼容性与版本演进

1. 新增必需字段属于破坏性变化，必须同步修改
   `loco_jump_contract.py`、环境、控制器和本文件；
2. 仅新增可选兼容字段可以保持旧消费者可读，但不得改变必需字段语义；
3. TCP 字段变化后必须重新执行真实 Webots 冒烟，mock 测试不能替代；
4. 前 42 维顺序、12 动作顺序和 50 Hz 周期改变时，旧 checkpoint 即失效；
5. 本文与源码不一致时，以 `loco_jump_contract.py` 的导入期自检为最终判定。

相关文档：

- [无摄像头遥控运动与跳跃](../features/remote-rl-locomotion.md)
- [RL 训练完全指南](../features/rl-training-guide.md)
- [机器人通信协议](./robot-communication.md)
