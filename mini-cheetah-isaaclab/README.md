# Mini Cheetah × Isaac Lab 训练工作区

本目录是 2026-10-07 新建的独立调研与训练工作区，用于把 YoboGo-10S
四足机器人迁移到 Isaac Lab。后续所有训练脚本、checkpoint、日志、
TensorBoard event 和运行产物只允许写入本目录的 `training/` 或
`outputs/`，不得散落到项目根目录的 `runs/`、`logs/`、`checkpoints/`。

## 目录职责

- `research/`：本轮联网调研、来源评价、数据来源矩阵和迁移步骤。
- `model/`：训练用模型资产；权威外观来源是已确认的 Mini Cheetah DAE。
- `training/`：环境、智能体、训练脚本、checkpoint、日志和 run。
- `outputs/`：评估视频、报告、导出策略等最终产物。

## 三层数据边界

1. 外观权威：已确认正确的官方：
   `official-mini-cheetah/assets/meshes/*.dae`。本工作区已通过复制建立
   `model/visual_dae/`，原文件保持不动。
2. 实机控制与接口权威：`YoboGo-control/`，提供控制周期、接口、安全
   限制和默认控制参数。
3. 缺失物理数据：可从网络上的 MIT 官方 Mini Cheetah 数据选取运动学、
   惯量、碰撞和执行器参考；逐项记录 URL、许可证、冲突和最终采用规则。

所有数值集中登记在
[`research/data-manifest.md`](research/data-manifest.md)。不复制外部
URDF、USD、mesh 或策略；Isaac Lab 示例机器人仍只作代码参考。

详细调研见 [`research/2026-10-07-minicheetah-isaaclab.md`](research/2026-10-07-minicheetah-isaaclab.md)。

## YoboGo Isaac Lab 键盘回放

项目提供独立回放入口
[`training/scripts/play_keyboard.py`](training/scripts/play_keyboard.py)。
入口复用现有 `training.envs` 注册、`YoboGoVelocityFlatEnvCfg` 和
`YOBOGO_MINI_CHEETAH_CFG`，不重新实现机器人模型，也不修改训练配置。
默认任务为 `YoboGo-Velocity-Flat-v0`，默认 checkpoint 为最终
`model_4799.pt`，默认 1 个环境并启动 GUI。

以下命令均从 `mini-cheetah-isaaclab/` 根目录执行：

```bash
source /home/pma213x/.venvs/minicheetah-isaaclab/bin/activate
export PYTHONUNBUFFERED=1
export TERM=xterm-256color
```

### 测试与接入命令

| 测试/用途 | 命令 | 实际退出码 | 证据日志 |
| --- | --- | ---: | --- |
| Python 语法 | `PYTHONPYCACHEPREFIX="$(mktemp -d /tmp/yobogo-play-pycache-record.XXXXXX)" /home/pma213x/.venvs/minicheetah-isaaclab/bin/python -m py_compile training/scripts/play_keyboard.py` | `0` | `training/logs/play-keyboard/2026-10-08/record-pycompile.log` |
| 训练代码静态检查 | `/home/pma213x/.venvs/minicheetah-isaaclab/bin/python training/scripts/static_check.py` | `0`（`STATIC_CHECK_OK files=12`） | `training/logs/play-keyboard/2026-10-08/record-static-check.log` |
| CLI 帮助 | `/home/pma213x/IsaacLab/isaaclab.sh -p training/scripts/play_keyboard.py --help` | `0` | `training/logs/play-keyboard/2026-10-08/record-help.log` |
| headless 主接入冒烟 | `/home/pma213x/IsaacLab/isaaclab.sh -p training/scripts/play_keyboard.py --headless --smoke-steps 5 --no-keyboard` | `0` | `training/logs/play-keyboard/2026-10-08/gui-fix-headless-smoke.log` |
| GUI 首次有界启动（历史） | `/home/pma213x/IsaacLab/isaaclab.sh -p training/scripts/play_keyboard.py --smoke-steps 1` | `137`（RTX PSO 编译等待期间被系统 `Killed`，未完成） | `training/logs/play-keyboard/2026-10-08/gui-smoke.log` |
| GUI 修复后有界诊断 | `/home/pma213x/IsaacLab/isaaclab.sh -p training/scripts/play_keyboard.py --smoke-steps 1` | `0`（完成 checkpoint、`Se2Keyboard` 和 1 步回放） | `training/logs/play-keyboard/2026-10-08/gui-diagnostic.log` |
| headless 行走录制 | `/home/pma213x/IsaacLab/isaaclab.sh -p training/scripts/play_keyboard.py --headless --record-video --walk-steps 400 --walk-speed 0.3 --video-fps 40 --output outputs/yobogo-walk/yobogo_walk_final.mp4` | `0` | `training/logs/play-keyboard/2026-10-08/record-headless.log` |
| 视频可播放性与帧检查 | `ffprobe -count_frames ... && ffmpeg -i outputs/yobogo-walk/yobogo_walk_final.mp4 -f null -` | `0` | `training/logs/play-keyboard/2026-10-08/record-media.log` |

headless 日志已确认：

- `checkpoint_loaded=true`；
- `completed_steps=5`；
- `final_command=[0.15000000596046448, 0.0, 0.0]`；
- 无第二个训练进程启动。

GUI 成功日志已确认：

- `Simulation App Startup Complete` 出现在 `240.317 s`；
- `checkpoint_loaded=true`、`keyboard_setup_done`、
  `event_loop_start`、`first_step_done` 均已出现；
- `completed_steps=1`，退出码 `0`，总耗时约 `293 s`；
- 阻塞位于 `app ready` 之后的 RtPso 异步 GPU 着色器管线编译，
  不在 checkpoint 加载、`Se2Keyboard` 初始化或脚本事件循环中。

### GUI 与 headless 方法

| 方法 | 命令 | 行为 |
| --- | --- | --- |
| GUI 键盘回放 | `/home/pma213x/IsaacLab/isaaclab.sh -p training/scripts/play_keyboard.py` | 等待 RTX 初始化后加载 checkpoint、创建官方 `Se2Keyboard`；关闭窗口或按 `Esc` 退出 |
| GUI 有界冒烟 | `/home/pma213x/IsaacLab/isaaclab.sh -p training/scripts/play_keyboard.py --smoke-steps 1` | 启用 GUI 和 `Se2Keyboard`，仅执行 1 步；修复后诊断退出码 `0` |
| headless 固定速度冒烟 | `/home/pma213x/IsaacLab/isaaclab.sh -p training/scripts/play_keyboard.py --headless --smoke-steps 5 --no-keyboard` | 不创建键盘，使用固定 `[0.15, 0, 0] m/s`，执行 5 步后退出 |
| headless 自动行走录制 | `/home/pma213x/IsaacLab/isaaclab.sh -p training/scripts/play_keyboard.py --headless --record-video --walk-steps 400 --walk-speed 0.3 --video-fps 40 --output outputs/yobogo-walk/yobogo_walk_final.mp4` | 固定 `0.3 m/s` 前进，不依赖键盘，生成 10 秒 MP4 |
| 覆盖参数 | 追加 `--task TASK --checkpoint PATH --num_envs N` | 任务默认 `YoboGo-Velocity-Flat-v0`，环境默认 `1` |

GUI 人工验证步骤：

```bash
cd /run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/mini-cheetah-isaaclab
/home/pma213x/IsaacLab/isaaclab.sh -p training/scripts/play_keyboard.py
```

1. 启动 GUI 键盘回放命令。窗口可能在前 `5–6` 分钟显示“无响应”，
   此阶段是首次扩展同步和 RtPso GPU 管线编译，不要强制结束；
2. 等到终端出现 `Simulation App Startup Complete`、
   `[STAGE] config_start` 和 `checkpoint_loaded=true`，再确认
   `keyboard_setup_done`；
3. 按官方键位并观察仿真中的速度目标箭头与机器人运动方向；
4. 按 `L` 确认命令清零，按 `Esc` 确认回放退出；
5. GUI 自动启动与 1 步回放已通过，真实人工按键运动方向仍待验收。

若超过 `6` 分钟仍未出现 `[STAGE] config_start`，保留终端与
`training/logs/play-keyboard/2026-10-08/` 日志后停止并报告，不要重复启动。

### 自动行走录制

录制模式不依赖键盘，固定输入前进速度，并强制启用 Isaac
`RecordVideo`/viewport RGB 渲染：

```bash
cd /run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/mini-cheetah-isaaclab
source /home/pma213x/.venvs/minicheetah-isaaclab/bin/activate
export PYTHONUNBUFFERED=1
export TERM=xterm-256color
/home/pma213x/IsaacLab/isaaclab.sh -p training/scripts/play_keyboard.py \
  --headless --record-video \
  --walk-steps 400 --walk-speed 0.3 --video-fps 40 \
  --output outputs/yobogo-walk/yobogo_walk_final.mp4
```

录制媒体信息：

| 项目 | 实测值 |
| --- | --- |
| 视频 | [`outputs/yobogo-walk/yobogo_walk_final.mp4`](outputs/yobogo-walk/yobogo_walk_final.mp4) |
| 绝对路径 | `/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/mini-cheetah-isaaclab/outputs/yobogo-walk/yobogo_walk_final.mp4` |
| 时长 | `10.000000 s` |
| 分辨率 | `960 × 540` |
| 帧率 | `40 FPS` |
| 帧数 | `400`（`ffprobe -count_frames` 实测） |
| 编码/容器 | `h264` / `mp4` |
| 文件大小 | `533678 bytes` |
| 机器人 XY 位移 | `0.241785 m` |
| 第一帧预览 | [`outputs/yobogo-walk/yobogo_walk_final_first_frame.png`](outputs/yobogo-walk/yobogo_walk_final_first_frame.png) |
| 末帧预览 | [`outputs/yobogo-walk/yobogo_walk_final_last_frame.png`](outputs/yobogo-walk/yobogo_walk_final_last_frame.png) |
| 录制日志 | `training/logs/play-keyboard/2026-10-08/record-headless.log` |
| 媒体校验日志 | `training/logs/play-keyboard/2026-10-08/record-media.log` |

`ffprobe`、逐帧解码和首末帧抽取均退出码 `0`。首轮实现因在
`env.reset()` 前调用 `env.render()` 触发 Gymnasium `ResetNeeded`，未生成
视频；该尝试保留在
`training/logs/play-keyboard/2026-10-08/record-headless-reset-needed-attempt.log`，
修复 reset 顺序后一次录制成功。

### checkpoint 对比与逐步遥测诊断

新增独立入口
[`training/scripts/diagnose_checkpoint.py`](training/scripts/diagnose_checkpoint.py)，
用于在完全相同的相机、seed 和步数条件下比较 `model_1800.pt` 与
`model_4799.pt`。入口支持精确 `--walk-speed 0`，每步写入
`checkpoint`、速度命令、机身高度/roll/pitch、四足接触力、12 路关节
位置/速度、原始动作和处理后关节目标；视频与 JSONL 遥测必须同时输出。

```bash
cd /run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/mini-cheetah-isaaclab
source /home/pma213x/.venvs/minicheetah-isaaclab/bin/activate
export PYTHONUNBUFFERED=1
export TERM=xterm-256color
/home/pma213x/IsaacLab/isaaclab.sh -p training/scripts/diagnose_checkpoint.py \
  --headless --record-video \
  --checkpoint training/logs/rsl_rl/yobogo_velocity_flat/2026-10-07_23-35-03_repair-v1-systemd_2026-10-07_233458/model_1800.pt \
  --walk-steps 200 --walk-speed 0 --video-fps 40 \
  --output outputs/yobogo-walk/diag-2026-10-08/compare_1800/near_zero.mp4 \
  --telemetry-output outputs/yobogo-walk/diag-2026-10-08/compare_1800/near_zero.jsonl
```

本轮首次 `model_1800` 近零速录制在第一个遥测步失败：RSL-RL 包装层
实际返回 `obs/reward/dones/extras`，初版把 `extras` 字典当作截断标志，
触发 `TypeError`。失败证据保存在
`outputs/yobogo-walk/diag-2026-10-08/compare_1800/near_zero.log`；
对应 JSONL 为 `0 bytes`，临时 MP4 为 `5246 bytes`。按“录制失败即停止”
约定，本轮未继续运行其余三段，因此尚未形成 A/B/C checkpoint 对比结论。
源码已按官方包装层返回契约修正，但本轮没有再次启动录制。

### 官方键位

不提供 WASD 映射；以下键位直接来自 Isaac Lab `Se2Keyboard`。

| 控制项 | 正向键 | 负向键/操作 | 默认灵敏度 | 环境命令范围 |
| --- | --- | --- | ---: | --- |
| 前后 `vx` | `Numpad8` / `ArrowUp` | `Numpad2` / `ArrowDown` | `±0.8 m/s` | `[-1.0, 1.0] m/s` |
| 侧移 `vy` | `Numpad4` / `ArrowLeft` | `Numpad6` / `ArrowRight` | `±0.4 m/s` | `[-1.0, 1.0] m/s` |
| 偏航 `yaw` | `Numpad7` / `Z` | `Numpad9` / `X` | `±1.0 rad/s` | `[-1.0, 1.0] rad/s` |
| 清零 | - | `L` | 全部归零 | 全部归零 |
| 退出 | - | `Esc` | 停止交互回放 | - |

### checkpoint 与 TensorBoard

| 项目 | 路径或链接 |
| --- | --- |
| 任务 | `YoboGo-Velocity-Flat-v0` |
| 默认 checkpoint | `training/logs/rsl_rl/yobogo_velocity_flat/2026-10-07_23-35-03_repair-v1-systemd_2026-10-07_233458/model_4799.pt` |
| checkpoint 绝对路径 | `/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/mini-cheetah-isaaclab/training/logs/rsl_rl/yobogo_velocity_flat/2026-10-07_23-35-03_repair-v1-systemd_2026-10-07_233458/model_4799.pt` |
| TensorBoard | [http://127.0.0.1:6007/](http://127.0.0.1:6007/) |
| 回放测试日志 | `training/logs/play-keyboard/2026-10-08/` |

### 注意事项

| 事项 | 结论 |
| --- | --- |
| 策略部署性 | `model_4799.pt` 尚未通过独立速度、跌倒率和动作饱和度 Gate，不保证可部署 |
| 随机命令 | 回放入口关闭 `heading_command`、站立抽样和 10 秒随机重采样，并在官方命令计算后覆盖 `base_velocity` |
| 训练配置 | 随机命令关闭只作用于本次回放，不修改既有训练配置或 checkpoint |
| 速度单位 | `vx/vy` 为 `m/s`，`yaw` 为 `rad/s`；写入前按环境 `[-1,1]` 范围裁剪 |
| 自动测试边界 | headless 接入和 GUI 启动/1 步回放均通过；真实人工键位验证仍待完成 |
| GUI 启动等待 | 首次扩展同步约 `55 s`，RtPso 等待日志从 `19.753 s` 持续到 `234.993 s`（约 `215 s`）；前 `5–6` 分钟显示无响应属于启动期 |
| 资源边界 | 不启动第二个训练，不修改 `YoboGo-control/`、`official-mini-cheetah/` 或 `Cheetah-Software/` |

## Rapid Locomotion 外部策略接入（2026-10-08）

新增独立入口 `training/scripts/play_rapid.py`，直接加载官方
`adaptation_module_latest.jit` 与 `body_latest.jit`，不使用既有 RSL-RL
loader。适配器位于 `training/external_policy/rapid_locomotion/`：

- 48 维 YoboGo 观测转换为 42 维 Rapid 观测；
- 15×42=630 历史，reset 清零，500 Hz 控制每 10 周期推理一次；
- `630→18` adaptation、拼接当前 42 维后 `60→12`，只取 actor mean；
- 命令先按 YoboGo 物理范围 `vx/vy∈[-0.6,0.6]`、`yaw∈[-1,1]`
  裁剪，再乘 `[2,2,0.25]`，最后按 Rapid 官方域防御性裁剪；
- 还原 `joint_pos_rel` 绝对角后减 Rapid 默认角；后腿 XML `axis=-1`
  与 `rpy=(0,pi,0)` 成对等效，不再额外反转 RR/RL HAA；
- previous action/history 保存网络原始 Rapid raw action；执行支路额外
  使用 `±0.25` OOD 门限，不删除位置/力矩安全裁剪；
- 本次回放实例覆盖为官方 Rapid 参数：`Kp=20`、`Kd=0.5`、
  `effort=18/18/26 N·m`，不冒充 YoboGo 实机 `17/17/26 N·m` 配置。

测试命令与结论：

| 测试 | 结果 |
| --- | --- |
| `py_compile`（适配器、模型、入口、测试） | 退出码 `0` |
| 12 项单元测试（48→42、命令顺序、公式、历史 raw、10 周期、JIT dummy） | 退出码 `0` |
| `play_rapid.py --help` | 退出码 `0` |
| 零速 100 步 headless | 通过；100 步/10 次推理，机身最低 `0.241727 m` |
| 低速 `0.15 m/s` 200 步 headless | 通过；机身最低 `0.250331 m`，roll `20.919°`、pitch `1.750°` |
| 最终视频录制 | 通过；300 步、30 次推理、0 次 reset/终止，前腿 Gate 通过 |

最终视频：

- 路径：`/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/mini-cheetah-isaaclab/outputs/rapid-locomotion/rapid_walk_test.mp4`
- 时长 `5.000000 s`、`960×540`、`20 FPS`、`100` 帧、
  `h264/mp4`、`2338809 bytes`
- 稳定性：机身最低 `0.252241 m`、最大 roll `20.263°`、
  最大 pitch `9.198°`、世界系路径速度 `0.117293 m/s`
- `ffprobe`、逐帧计数、`ffmpeg` 解码均退出码 `0`
- 逐腿证据：`rapid_walk_test.summary.leg_stats.json`

前腿诊断结论：

- 相机改为前方侧视 `eye=(-2.8,2.8,1.35)`，避免后视角遮挡 FR/FL；
- 前腿目标 P2P 最小值 `0.0625 rad`，实际关节 P2P 最小值
  `0.161632 rad`，前腿 Gate（target `>=0.04 rad`、actual `>=0.05 rad`）通过；
- FR/FL 实际 P2P（HAA/hip/knee）分别约
  `0.427/0.303/0.162 rad`、`0.429/0.209/0.323 rad`；
- 前腿 raw 动作样本被 `±0.25` OOD 门限裁剪 `167/180`（`92.8%`），
  力矩裁剪 3 次；后腿裁剪 `174/180`（`96.7%`），力矩裁剪 51 次。

接口索引/尺度/offset 已核对为 FR/FL/RR/RL × HAA/hip/knee；前腿并非
完全不动，剩余观感差异主要来自策略 OOD 放大、安全钳位和后视角/机体遮挡。

接口问题（符号、命令顺序、动作公式、历史初始化、50 Hz 调度）已修复；
网络原始动作仍会出现 OOD 放大，由 `±0.25` 执行门限、URDF 位置限位和
官方 Rapid 回放 PD/力矩配置共同兜底。证据日志见
`training/logs/rapid-locomotion/2026-10-08/`（本轮 `gait-*` 日志）。

### 五方向场景录制与 Gate（2026-10-08）

新增 `training/scripts/run_rapid_scenarios.py`，顺序录制前进、后退、
左移、右移和原地转向。每段均为 200 周期零命令站立 + 600 周期固定
Rapid 命令，运动阶段包含 60 次 50 Hz 推理；站立阶段保持 YoboGo
几何初态，并在运动开始时清零适配器历史，避免把站立闭环噪声带入
Rapid 历史。场景 Gate 失败仍继续录制其余场景，最后统一写入
`outputs/rapid-locomotion/scenarios/scenarios_summary.json`。

| 场景 | Rapid 命令 | Gate | 净位移 | yaw 变化 | 主要失败 |
| --- | --- | --- | ---: | ---: | --- |
| forward | `[0.6,0,0]` | 失败 | `0.460198 m` | `50.240°` | 机体前向投影 `-0.454776 m`；roll `61.410°` |
| backward | `[-0.6,0,0]` | 通过 | `0.144161 m` | `26.618°` | 无 |
| left | `[0,0.6,0]` | 失败 | `0.485051 m` | `-161.788°` | 侧向投影方向错误；跌倒并 reset 1 次 |
| right | `[0,-0.6,0]` | 失败 | `0.346483 m` | `-165.201°` | 侧向投影方向错误；跌倒并 reset 1 次 |
| yaw | `[0,0,0.5]` | 通过 | `0.111664 m` | `74.784°` | 无 |

五段站立阶段全部通过；五段视频均为 `5.000000 s`、`960×540`、
`20 FPS`、`100` 帧、`h264/mp4`。`ffprobe`、`ffmpeg -f null` 和
`ffprobe -count_frames` 对五段视频均退出码 `0`。逐段起止世界位置、
起止 yaw、机体前向/侧向投影、平均机体速度、姿态、终止/reset、逐腿
统计和媒体元数据见各 `*.summary.json`。

官方配置审计结论为 `jump_supported=false`，证据写入
`outputs/rapid-locomotion/scenarios/jump_support.json`；未生成伪造
跳跃视频。场景汇总退出码为 `2`，表示视频证据完整但
`all_scenarios_pass=false`，不得把本轮结论改写为五方向全部通过。

最终回归证据位于 `training/logs/rapid-locomotion/2026-10-08/final/`：

- `py_compile`、12 项单元测试、独立 JIT dummy 和 `--help` 均退出码 `0`；
- 零速 100 步：10/10 次推理、0 reset、机身最低 `0.244055 m`；
- 低速 0.15 m/s 200 步：20/20 次推理、0 reset、机身最低
  `0.241806 m`、最大 roll `36.561°`、pitch `6.943°`；
- 五段媒体校验见 `final/media-validation.json`。
