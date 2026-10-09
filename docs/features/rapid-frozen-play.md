# Rapid Mini Cheetah 冻结模型单机人工 Play

> 适用范围：`official-mini-cheetah` 中 Rapid Locomotion 固定提交模型的
> 冻结推理接入。只做单机器人 Webots 人工自测，不微调、不训练、不改变
> P1/P2 Gate。

## 1. 启动

从 `official-mini-cheetah/` 执行：

```bash
python3 -m rl.play \
  --model-type rapid \
  --pretrained-dir external_models/rapid_locomotion_f5143ef \
  --input keyboard-joystick \
  --world eval \
  --device cuda \
  --max-steps 1000
```

- `--world eval` 使用 `worlds/flat_move_jump_rl_eval.wbt` 单机器人；
- `--max-steps 0` 表示不自动停止，人工测试建议先使用有限步数；
- `--model-type sb3` 与原 `--checkpoint` 路径继续可用；
- `--dry-run` 不加载模型、不启动 Webots。

启动后先确认输出：

```text
PLAY_MODEL_READY model_type=rapid ...
PLAY_INPUT_READY source=keyboard-joystick
PLAY_LOG_READY path=.../logs/official_mini_cheetah_flat_jump_v1_r3/play/rapid_play_*.jsonl
RL_CONTROLLER_PLAY_INPUT_READY keyboard=True joystick=True
```

键盘和手柄至少必须有一种可用，否则 controller 以
`RL_CONTROLLER_ERROR play_input_initialization_failed` 清晰失败。

## 2. 链路与安全

```text
Webots Keyboard/Joystick
    → controller 原始 keys/axes/buttons
    → InputAdapter
    → 57 维观测
    → RapidPolicyAdapter（42 维、15 帧、630→18、60→12）
    → 当前 12 维动作
    → action 限幅
    → 关节目标映射与 controller 目标限速
```

- 零命令、输入超时和输入缺失始终安全站立，不调用模型；
- 只有非零 `[vx,vy,wz]` 才启用 Rapid 推理；
- WASD 控制移动，QE 控制转向；
- Space 请求跳跃时记录 `unsupported/rejected`，不执行跳跃；
- Esc 锁存急停；R 只在急停后显式恢复为安全站立；
- 模型错误和非有限输出进入锁存急停；
- 执行层继续使用当前 `DEFAULT_CROUCH`、动作限幅、控制器目标限速和
  `TARGET_POSITION_LIMITS`，不放宽安全边界。

## 3. JSONL 自测日志

日志位于：

```text
official-mini-cheetah/logs/official_mini_cheetah_flat_jump_v1_r3/play/
```

文件名含时间戳，冲突时追加序号，不覆盖历史。每帧至少记录：

- `step`、`command`、`raw_source_action`；
- `mapped_yobo_action`、`applied_action`；
- `sanitization_delta`、`execution_delta`；
- `target_rate_limit_delta`、`host_target_rate_limit_delta`、
  `desired_executed_target_delta`；
- `target_lag_mean/rms/max` 与 `target_rate_limit`；
- `mapped_action_saturation_rate`、`applied_action_saturation_rate`；
- `position`、`v_body`、`displacement_world`、`displacement_body`、
  `displacement_step_norm` 和 `episode_distance_world`；
- `contacts`、`safe_stand`、`emergency_stop`、`reason`；
- `jump_requested`、`jump_rejected`、`jump_supported`；
- `observation_42`、`latent`、历史帧数量和 warmup 状态。

controller 提供的 `execution_telemetry`（play 兼容字段 `play_control`）
用于计算真实目标限速差值；尚未建立 play TCP 响应时回退到 host
目标限速差值。目标限速仅允许 `0.25/0.5/1.0 rad/s`，默认 `1.0`；
通过 `--action-target-rate-limit` 或
`RL_ACTION_TARGET_RATE_LIMIT` 选择，旧 `0.03` 明确拒绝。

## 4. 人工验收

1. 零命令保持 5 秒，机器人维持屈膝安全站立；
2. W/A/S/D/Q/E 分别产生预期方向的非零命令并由模型接管；
3. 手柄轴和按钮均能产生命令；
4. Space 日志为 `jump_rejected=true`、`jump_supported=false`；
5. Esc 后连续帧 `emergency_stop=true`，R 后恢复
   `safe_stand=true` 且急停清除；
6. `--max-steps 1000` 恰好在 1000 帧或更早安全终止；
7. JSONL 无 NaN/Inf，能比较源动作、当前动作、实际动作和限速差值。

在这些项目通过前，不启动微调、不删除历史 checkpoint。

## 5. 窗口关闭与日志判读

`--max-steps 1000` 是有限冒烟，不是持续遥控模式。当前 play 启动的
Webots 使用 fast 模式，完成 1,000 个 50 Hz 控制周期后会主动关闭窗口；
本机复现约 `13.643 s` 完成，进程退出码为 `0`，因此窗口快速消失不代表
Python、controller 或 TCP 崩溃。

- 持续人工测试使用 `--max-steps 0`；
- 有限测试结束时必须看到 `PLAY_SESSION_DONE {"steps":1000,...}`；
- 结束后检查启动时打印的 `PLAY_LOG_READY path=...`，文件应包含
  1,000 行 JSONL；
- 若没有 `PLAY_SESSION_DONE`，再检查终端 traceback、
  `webots_eval_world.log` 和 `webots_0.log`；
- controller 和 Webots 日志没有 traceback、`RL_CONTROLLER_ERROR` 或
  非有限状态时，不应把正常窗口关闭描述为异常退出。
