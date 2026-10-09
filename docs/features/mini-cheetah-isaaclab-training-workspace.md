# Mini Cheetah / Isaac Lab 训练工作区

> 状态：来源裁决、CAD 只读盘点、4 个权威外观 DAE 和训练规划已归档；
> 四腿数组映射、最终整机质量口径 `9 kg` 和项目力矩
> `17/17/26 N·m` 已固化；Isaac Sim `4.5.0.0`、Isaac Lab `v2.0.2` 和
> RSL-RL 已安装，官方空环境 headless 冒烟通过；项目 URDF/USD 已生成，
> 修复后 `check_env --mode all` 的 reset、几何、站立接触与随机动作 Gate
> 均为 `PASS`；正式 PPO 已通过用户级 systemd 放入后台，验证快照为
> `4799/4800`，最终 checkpoint `model_4799.pt` 已生成；独立策略 Gate
> 仍未验收，不保证可部署。
> 更新时间：2026-10-08。

## 1. 目标与边界

为 YoboGo-10S 迁移到 Isaac Lab 建立独立工作区，所有后续训练和产物
集中保存，避免继续散落到项目根目录的 `runs/`、`logs/`、
`checkpoints/`。

唯一工作区为：

`mini-cheetah-isaaclab/`

目录职责：

- `research/`：Mini Cheetah、Isaac Lab 和开源训练项目调研；
- `model/`：项目模型资产与 4 个权威外观 DAE；
- `training/`：环境、智能体、脚本、checkpoint、日志和 run；
- `outputs/`：评估报告、视频和策略导出包。

## 2. 数据来源规则

1. 实机控制与接口第一权威：`YoboGo-control/`。
2. YoboGo 缺失的运动学、逐 link 质量/惯量、足端和执行器参考：固定到
   网络 MIT ORCAgym 提交并登记 URL/许可证。
3. 唯一可信外形/碰撞几何：`mini-cheetah-isaaclab/model/mit-mini-cheetah-1.snapshot.2/`。
4. 唯一可信外观：4 个官方 DAE；工作区使用复制副本，原文件不动。
5. `official-mini-cheetah/` 和旧 Webots 除这 4 个 DAE 外全部禁用；
   第三方和 Isaac Lab 示例机器人数据只用于方法研究。

最终采用值：

- 四腿数组：`leg[0]=FR/RF`、`leg[1]=FL/LF`、`leg[2]=RR/RH`、
  `leg[3]=RL/LH`；机体系为 `+X` 前、`+Y` 左、`+Z` 上；
- 最终整机质量口径：`9 kg`；
- 项目关节力矩：YoboGo 源码 `17/17/26 N·m`，同时保留原码
  `TODO CHECK WITH BEN` 的证据标注。

## 3. 已归档外观资产

| 文件 | 源/副本 SHA256 |
| --- | --- |
| `mini_abad.dae` | `6a34448654e342b6b71e71a1ca263e327fba8f4f60fd3e91aab0f8afed03683c` |
| `mini_body.dae` | `ba877c099da9a76f973f4c955260e8fc95224ea45b12fbbc1eca0ab2dfd55343` |
| `mini_lower_link.dae` | `e18ac026e8febee7e1d933235ff1a147268fa9009c8a46d2cc92012c53c50c91` |
| `mini_upper_link.dae` | `88a55f7c1de47945b0f51e990ace895e5fb3e46b51b0e5fade6d683249a8958e` |

副本路径为 `mini-cheetah-isaaclab/model/visual_dae/`。DAE 不含碰撞、
惯量、关节和执行器事实，不能单独构成可训练模型。

## 4. CAD 外形资产

CAD 目录只含 `steadywin_v3.STEP` 与 `Smart Engines.x_t` 两个交换文件，
盘点结果见
[`INVENTORY.md`](../../mini-cheetah-isaaclab/model/mit-mini-cheetah-1.snapshot.2/INVENTORY.md)。
STEP 确认单位为毫米，含 128 个产品和 936 个装配实例，但没有材料、质量、
惯量、关节或工程图；Parasolid 只有密度线索且单位待导入确认。二者可作为
外形/碰撞拆分输入，不能作为动力学或控制参数来源。

## 5. 调研结论

- 未找到经核验、结果可复现且同时满足 Mini Cheetah + Isaac Lab +
  完整训练资产的 MIT 官方开源包。
- 首选骨架是 Isaac Lab 官方 manager-based locomotion 环境和 RSL-RL。
- [evelyd/MiniCheetah_IsaacLabExtension](https://github.com/evelyd/MiniCheetah_IsaacLabExtension)
  是最直接的社区线索，但 README 主要是 Isaac Lab 扩展模板，示例任务
  仍使用 Anymal，不能视为已验证 Mini Cheetah 训练成品。
- `legged_gym`、RSL-RL、盲四足 locomotion 和 Go2 项目分别用于参考
  奖励/DR/课程、训练器、teacher-student 和目录/验收结构。

完整来源矩阵、项目排名与后续步骤见
`mini-cheetah-isaaclab/research/2026-10-07-minicheetah-isaaclab.md`。

## 6. 训练约束

- 正式训练入口、配置和 checkpoint 必须位于
  `mini-cheetah-isaaclab/training/`；
- 报告、视频和导出策略位于 `mini-cheetah-isaaclab/outputs/`；
- 模型静态验收、最小环境冒烟未通过前，不启动正式训练；
- CAD 坐标/单位、`9 kg` 逐 link 动力学分配和关节位置/速度机械限位等
  P0 未确认前，不启动正式 PPO；力矩项目值和 `9 kg` 总质量口径已确认；
- 四腿数组映射已固定，资产静态测试必须验证无前后/左右镜像；
- 根 `docs/` 只保留一份，不在此工作区复制。

## 7. 环境安装

- Python 虚拟环境位于 `/home/pma213x/.venvs/minicheetah-isaaclab`，
  Python 版本为 `3.10.21`；
- pip/uv/临时缓存固定在 `/home/pma213x/.cache/`，日志固定在
  `mini-cheetah-isaaclab/outputs/environment-install/`；
- Isaac Sim `4.5.0.0` 后台安装结果为 `success/0`，EULA 已接受；
- Isaac Lab 仓库固定为 `v2.0.2`，SHA 为
  `b5fa0eb031a2413c182eeb54fa3a9295e8fd867c`；
- Isaac Lab 官方要求 `torch==2.5.1`，已从不兼容的
  `2.14.1+cu130` 调整为 `2.5.1+cu121`，CUDA 检查通过；
- RSL-RL `rsl-rl-lib 2.3.3` 已通过官方
  `isaaclab.sh --install rsl_rl` 安装；
- 官方 `list_envs.py` 列出 96 个环境，退出码 `0`；
- 官方空环境 headless 测试在 `cuda:0`/`cpu` 上各创建 1 个环境并各 step
  2 次，结果 `OK`、退出码 `0`，最终日志无 `[Error]`。

操作说明见
[`ENVIRONMENT.md`](../../mini-cheetah-isaaclab/ENVIRONMENT.md)。

## 8. 项目资产导入与单环境 Gate 实际结果

本轮已完成项目资产的基础校验和导入：`py_compile`、URDF `generate`、
URDF 默认 `validate`、`validate --strict`、训练侧静态检查和
`list_envs.py` 均退出码 `0`。URDF→USD 首次转换在 `TERM=dumb` 下失败，
改用 `TERM=xterm-256color` 后退出码 `0`；导入过程中已修复膝关节初态的
`float32` 类型问题，并把 `base_external_force_torque` 接口修正到
`base_link`。

基础命令通过不等于环境 Gate 通过。`check_env.py --mode all` 的零命令
站立四足有效接触比例门失败，`gate.json` 明确记录
`status=fail` 与
`AssertionError: 站立四足有效接触比例 >= 0.95`。shell 包装命令虽可能
返回 `0`，但主测试逻辑已经明确异常/失败，不得按 `PASS` 汇报。

最终 FK+collision 诊断证据为
[`geometry-diagnosis.log`](../../mini-cheetah-isaaclab/training/runs/env-check/2026-10-08_0130/geometry-diagnosis.log)：

- `root=0.30 m` 时，足端最低世界 `z≈0.24606 m`；
- 按当前 `target_jpos` 实现足地接触需要 `root≈0.053938 m`；
- 该高度与约 `0.26 m` 的预期站高矛盾；
- 诊断运行中接触力曾达到 `146 N`/`174 N`，证明 PhysX 接触传感器和
  碰撞查询可用。

历史证据指向 YoboGo `target_jpos` 直接映射 MIT URDF 时不自洽，而不是
“接触传感器无数据”。

## 9. 初态拆分修复与复测

2026-10-07 按已有 FK+collision 证据完成修复，保持 ORCAgym 固定提交的
axis/RPY 不变：

- 实机 `TARGET_JOINT_POS` 继续保留
  `FR/RR=[-0.6,-1,2.7]`、`FL/RL=[0.6,-1,2.7]`，仅作为控制侧语义；
- URDF/Isaac `SIM_INITIAL_JOINT_POS` 改为
  `[0,-0.785398163,1.865468294]` × 4；
- `ArticulationCfg` root 出生高度改为 `0.26 m`，reset 事件缩放系数固定
  为 `1.0`，因此精确回到 SIM 初态；零动作 PD offset 同样使用 SIM；
- 4 个 foot visual origin 与 collision 对齐为 `(0,0,0.024) m`；
- 上述 origin 是 foot link 局部坐标，FK 必须先随 foot 姿态旋转再求
  world z；当前姿态下碰撞中心 z 为 `-0.23979999998 m`、球底为
  `-0.25999999998 m`，不能把 `+0.024` 当成世界竖直偏移；
- 9 kg、17/17/26 N·m、`0.0004 s × 5 = 0.002 s`、PD
  `Kp=3/Kd=1,0.2,0.2` 未改变。

静态与动态证据：

| 测试 | 退出码/状态 |
| --- | ---: |
| `py_compile` | `0` |
| `generate_urdf.py` | `0` |
| `validate_urdf.py` | `0`（39 项通过） |
| `validate_urdf.py --strict` | `0`（39 项通过） |
| `static_check.py` | `0`（`STATIC_CHECK_OK files=11`） |
| `TERM=xterm-256color` URDF→USD | `0` |
| `list_envs.py` | `0` |
| `check_env.py --mode all` | `0`，`gate.json status=pass` |

动态 reset 实测：

- `root_z=0.2599999905 m`；
- `foot_collision_min=-0.2600000799 m`；
- `base_collision_min=-0.0400000066 m`；
- 站立四足有效接触比例 `>=0.95`；
- reset/step、500 次零动作站立、200 次随机动作均无 NaN/Inf，
  `metrics.json check_count=1657`。

证据目录：
`mini-cheetah-isaaclab/training/runs/env-check/2026-10-07_225234_repair/`。

## 10. 正式 PPO 后台运行

- systemd unit：`yobogo-ppo-20261007233458`；
- Main PID：`2064669`；
- `PPID=1574` 为常驻 `systemd --user` 管理器，`SID=PGID=2064669`、
  无控制终端；Python 子进程为 `2064690`，GPU 占用 `2359 MiB`；
  旧线索 PID `2032357` 已不存在，未发现第二个正式 PPO 进程；
- 日志：
  `mini-cheetah-isaaclab/training/logs/formal-ppo_2026-10-07_233458_systemd/train.log`；
- 状态与证据：
  `pid.txt`、`status.txt`、`systemd-active-state.txt`、
  `process-evidence.txt`、`scan-report.txt`；
- run：
  `mini-cheetah-isaaclab/training/logs/rsl_rl/yobogo_velocity_flat/2026-10-07_23-35-03_repair-v1-systemd_2026-10-07_233458/`；
- 从 `model_1800.pt` resume，追加 3000 iteration，因此最终目标为总
  `4800` iteration；每 iteration 为 `4×24=96` 环境步，每 50 iteration
  保存 checkpoint；
- 最终核验快照（2026-10-07 23:40:06 +0800）：`2152/4800`、
  `model_2150.pt`、正式 checkpoint 共 46 个、TensorBoard event 存在；
  OOM/CUDA error/Traceback/Killed/NaN/Inf 扫描均为 `0`。

训练步骤与当前进度：

| 步骤名称 | 总 iteration / 环境步数 | 用途 | 23:40:06 进度 | checkpoint 频率 |
| --- | --- | --- | --- | ---: |
| 预热与续训基线 | `0–1800` / `0–172800` | PPO 预热、接触/姿态稳定，形成 `model_1800.pt` | 已完成 | 每 `50` iteration |
| systemd 续训稳定 | `1801–2400` / `172801–230400` | 检查有限值、接触与 checkpoint 连续性 | 进行中，`2152/4800` | 每 `50` iteration |
| 速度跟踪重点 | `2401–3600` / `230401–345600` | 线速度/偏航跟踪、奖励和动作平滑度 | 未到达 | 每 `50` iteration |
| 收敛与 Gate 准备 | `3601–4800` / `345601–460800` | 最终收敛与 checkpoint 保留；完成后独立执行速度误差、跌倒率和动作饱和度 Gate | 未到达 | 每 `50` iteration |

TensorBoard 已由用户级 systemd 持久化；因 `6006` 已被占用，只绑定
localhost 的 `6007`：

```bash
systemctl --user status yobogo-tensorboard-20261007233806.service
```

- unit：`yobogo-tensorboard-20261007233806`；
- PID：`2069531`，`PPID=1574`、独立 `SID=2069531`、无控制终端；
- 绑定：`127.0.0.1:6007`，HTTP 响应核验通过；
- 状态文件：`mini-cheetah-isaaclab/training/runs/tensorboard-formal-ppo/`；
- event 目录：
  `mini-cheetah-isaaclab/training/logs/rsl_rl/yobogo_velocity_flat/`；
- 本地链接：[http://127.0.0.1:6007/](http://127.0.0.1:6007/)。

**当前状态**：正式 PPO 已到达 `4799/4800` 并生成最终 checkpoint
`model_4799.pt`，训练进程已退出；独立速度、跌倒率和动作饱和度 Gate
仍待验收，禁止表述为策略通过或可部署。

## 11. 官方键盘速度回放入口

2026-10-08 新增
[`mini-cheetah-isaaclab/training/scripts/play_keyboard.py`](../../mini-cheetah-isaaclab/training/scripts/play_keyboard.py)。
该入口只负责加载既有 RSL-RL checkpoint 和执行回放，不修改训练逻辑、
环境配置或 checkpoint。

### 11.1 接入方法

入口复用以下既有实现：

- `_bootstrap.py` 的项目路径引导；
- `training.envs` 对 `YoboGo-Velocity-Flat-v0` 的注册；
- `YoboGoVelocityFlatEnvCfg`；
- `YOBOGO_MINI_CHEETAH_CFG` 与项目 USD；
- `RslRlOnPolicyRunnerCfg` 和 `OnPolicyRunner.load()`。

默认 checkpoint：

`mini-cheetah-isaaclab/training/logs/rsl_rl/yobogo_velocity_flat/2026-10-07_23-35-03_repair-v1-systemd_2026-10-07_233458/model_4799.pt`

默认命令为：

```bash
source /home/pma213x/.venvs/minicheetah-isaaclab/bin/activate
export PYTHONUNBUFFERED=1
export TERM=xterm-256color
cd /run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/mini-cheetah-isaaclab
/home/pma213x/IsaacLab/isaaclab.sh -p training/scripts/play_keyboard.py
```

headless 固定速度冒烟命令：

```bash
/home/pma213x/IsaacLab/isaaclab.sh -p training/scripts/play_keyboard.py \
  --headless --smoke-steps 5 --no-keyboard
```

CLI 支持 `--task`、`--checkpoint`、`--num_envs`、`--headless`、
`--smoke-steps`、`--no-keyboard`、`--record-video`、`--walk-steps`、
`--walk-speed`、`--video-fps` 和 `--output`；默认任务为
`YoboGo-Velocity-Flat-v0`、最终 checkpoint、`1` 个环境、GUI、启用键盘，
录制默认 `400` 步、`0.3 m/s`、`40 FPS`。

自动行走录制命令：

```bash
/home/pma213x/IsaacLab/isaaclab.sh -p training/scripts/play_keyboard.py \
  --headless --record-video \
  --walk-steps 400 --walk-speed 0.3 --video-fps 40 \
  --output outputs/yobogo-walk/yobogo_walk_final.mp4
```

录制模式强制 `num_envs=1`、`enable_cameras=True`，复用官方
`gym.wrappers.RecordVideo` 与 viewport RGB 渲染；先 `env.reset()` 再预热
三次渲染，使用固定世界相机和 `960×540` 分辨率。成功视频：
`mini-cheetah-isaaclab/outputs/yobogo-walk/yobogo_walk_final.mp4`。

GUI 冷启动可能因扩展 registry 同步和 RtPso GPU 管线编译而显示
“无响应”约 `5–6` 分钟。终端出现
`Simulation App Startup Complete`、`[STAGE] config_start` 和
`checkpoint_loaded=true` 后，才进入可交互回放阶段。

### 11.2 官方键位与命令接管

| 控制项 | 正键 | 负键/操作 | 默认灵敏度 |
| --- | --- | --- | ---: |
| `vx` | `Numpad8` / `ArrowUp` | `Numpad2` / `ArrowDown` | `0.8 m/s` |
| `vy` | `Numpad4` / `ArrowLeft` | `Numpad6` / `ArrowRight` | `0.4 m/s` |
| `yaw` | `Numpad7` / `Z` | `Numpad9` / `X` | `1.0 rad/s` |
| 清零/退出 | - | `L` / `Esc` | 全零 / 停止 |

回放实例在创建环境配置时关闭 `heading_command`、
`rel_heading_envs`、`rel_standing_envs`，把 10 秒重采样改为
`1e9 s`，并在每次官方 `UniformVelocityCommand.compute()` 返回后覆盖
`vel_command_b=[vx,vy,yaw]`，再计算本次观测。写入前按环境范围
`[-1,1]` 裁剪，因此随机命令不会覆盖键盘值。

上述关闭只存在于回放入口，不修改 `velocity_flat_env_cfg.py` 中的
训练配置。

### 11.3 实际测试

| 测试 | 退出码 | 结果/日志 |
| --- | ---: | --- |
| 修复后 `py_compile` | `0` | `mini-cheetah-isaaclab/training/logs/play-keyboard/2026-10-08/gui-fix-pycompile.log` |
| `static_check.py` | `0` | `STATIC_CHECK_OK files=12` |
| 修复后 `play_keyboard.py --help` | `0` | `mini-cheetah-isaaclab/training/logs/play-keyboard/2026-10-08/gui-fix-help.log` |
| 修复后 `--headless --smoke-steps 5 --no-keyboard` | `0` | `checkpoint_loaded=true`、`completed_steps=5`、`final_command=[0.15,0,0]` |
| GUI 首次历史 `--smoke-steps 1` | `137` | RtPso 编译等待期间被系统 `Killed`，未进入 checkpoint 回放 |
| GUI 修复后 `--smoke-steps 1` | `0` | `240.317 s` 启动完成，随后 checkpoint、键盘和 1 步均成功 |
| 录制前 `py_compile`/静态/帮助 | `0` | `record-pycompile.log`、`record-static-check.log`、`record-help.log` |
| 首轮录制实现 | 日志含 `ResetNeeded` | 在 reset 前 render，未生成视频；日志 `record-headless-reset-needed-attempt.log` |
| 修复后 headless 行走录制 | `0` | `completed_steps=400`、`xy_displacement=0.241785 m`、`video_encoding_done` |
| `ffprobe`/逐帧解码/首末帧 | `0` | `record-media.log`、`record-ffprobe.json`、`record-frame-count.txt` |

录制媒体实测：`10.000000 s`、`960×540`、`40 FPS`、`400` 帧、
`h264/mp4`、`533678 bytes`；首帧和末帧均可见机器人，预览为
`outputs/yobogo-walk/yobogo_walk_final_first_frame.png` 与
`outputs/yobogo-walk/yobogo_walk_final_last_frame.png`。录制日志为
`mini-cheetah-isaaclab/training/logs/play-keyboard/2026-10-08/record-headless.log`。

GUI 无响应根因是 `app ready` 之后的
`Waiting for RtPso async group async compilation`，不是环境创建、
checkpoint 加载、`Se2Keyboard` 初始化或脚本事件循环。修复后诊断日志：
`mini-cheetah-isaaclab/training/logs/play-keyboard/2026-10-08/gui-diagnostic.log`。
日志末尾存在官方 `Se2Keyboard.__del__` 的非致命清理异常，发生在回放
完成之后，不影响退出码 `0`；真实人工按键运动方向仍待验收。

完整测试说明、GUI 人工验证步骤、checkpoint 绝对路径和 TensorBoard
[http://127.0.0.1:6007/](http://127.0.0.1:6007/) 见
[`mini-cheetah-isaaclab/README.md`](../../mini-cheetah-isaaclab/README.md)。
