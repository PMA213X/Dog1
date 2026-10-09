# Rapid Mini Cheetah 冻结策略适配

## 当前状态

**冻结模型核心适配已完成，尚未接入 `rl/play.py`，未启动 Webots play、
微调或训练。**

本阶段只提供 `rl/rapid_policy.py::RapidPolicyAdapter`，把当前 57 维遥控
观测转换成 Rapid Mini Cheetah 模型需要的 42 维当前帧与 15 帧、630 维
历史，再通过 `630→18` adaptation 和 `60→12` body 输出当前 12 维动作。

## 来源与完整性

- 来源仓库：`https://github.com/SellCXHarsha/rapid-locomotion-rl`
- 固定提交：
  `f5143ef940e934849c00284e34caf164d6ce7b6e`
- 本地目录：
  `official-mini-cheetah/external_models/rapid_locomotion_f5143ef/`
- 资产：`ac_weights_last.pt`、`body_latest.jit`、
  `adaptation_module_latest.jit`、`parameters.pkl`、
  `mini_cheetah.urdf`、`LICENSE`、`manifest.json`
- 许可证：MIT
- 加载前校验仓库、提交、许可证、必备文件和 manifest 中全部 SHA256；
  任一不匹配直接拒绝加载。

## 42 维观测

字段顺序与源环境一致：

| 字段 | 维度 | 转换 |
|---|---:|---|
| `projected_gravity` | 3 | 从当前 `rpy` 计算机体系重力 |
| `commands` | 3 | `[vx,vy,wz] × [2,2,0.25]` |
| `q-default` | 12 | 当前关节映射到源顺序后减源默认角 |
| `dq` | 12 | 映射到源顺序后乘 `0.05` |
| 上一动作 | 12 | 使用转换前的 Rapid 源动作 |

42 维结果按源配置裁剪到 `[-100,100]`。当前独有的速度、跳跃、接触、
高度和地形维度不输入该冻结模型。

## 15 帧历史与网络

- 当前帧追加到历史末尾，不足 15 帧时用首帧补齐；
- 15×42 展平为 630 维；
- `adaptation_module_latest.jit: 630→18`；
- 拼接当前 42 维与 latent 得到 60 维；
- `body_latest.jit: 60→12`；
- 模型输出或 latent 非有限时拒绝执行，并且不推进历史状态。

## 关节映射与动作尺度

源 URDF、`parameters.pkl`、当前 `contract` 与两个 world 的 motor 轴在加载时
三方硬校验。当前映射为同序同号：

| 源关节 | 当前关节 | 符号 | 源尺度 | 当前尺度 |
|---|---|---:|---:|---:|
| `FR/FL/RR/RL_hip_joint` | `fr/fl/hr/hl_abd` | `+1` | `0.125` | `0.30` |
| `FR/FL/RR/RL_thigh_joint` | `fr/fl/hr/hl_hip` | `+1` | `0.25` | `0.50` |
| `FR/FL/RR/RL_calf_joint` | `fr/fl/hr/hl_kn` | `+1` | `0.25` | `0.50` |

按角位移等效转换：

```text
current_action = source_action × source_scale / current_scale
```

输出裁剪到 `[-1,1]`，但不在本层执行 `0.08/frame` 相邻限速；该限速、
`DEFAULT_CROUCH` 关节目标、双层目标限速和急停仍由现有 play/执行层负责。
源模型默认角只用于观测零点，不替换当前 `DEFAULT_CROUCH` 安全基准。
`RapidActionMapper` 已作为 play、Rapid 微调训练和 Gate 的统一映射门面；
SB3 自定义微调接入见 `docs/features/rapid-sb3-finetune.md`。

## 能力边界与 info

- 支持站立附近平衡、低速/高速移动、转向和平地及盲态地形鲁棒运动；
- 不支持跳跃；`jump_request=True` 只记录
  `jump_request_rejected=true` 和 `unsupported_requests=["jump"]`，
  不产生跳跃动作；
- `predict()` 返回 `(action12, info)`；
- `info` 至少包含 `raw_source_action`、`mapped_yobo_action`、`latent`、
  `observation_42`、`history_630`、`history_warmup`、
  `jump_request_supported` 和 `source_commit`。

## 测试

- `tests/rl/test_rapid_policy.py` 覆盖 manifest/SHA256、真实资产形状、
  42 维字段与缩放、15 帧历史与 630 维拼接、关节置换/符号可逆、动作比例
  转换、输出 12 维与裁剪、非有限保护和 jump 不支持；
- 端口诊断 mock 只按两个精确 `ss` 命令识别诊断调用，避免把诊断误算成
  Gate 启动；
- `tests/rl` 全量：`107/107 OK`；
- `py_compile`、P2 `rl.train`、`rl.run_stages`、
  `rl.evaluate --gate` dry-run 均退出 `0`。

## 后续放行条件

play 集成必须由后续任务单独完成并通过以下人工自测：

1. Webots 单机器人正常启动，零命令保持安全站立；
2. 键盘和手柄均能产生命令，非零命令时冻结模型接管；
3. Space 记录跳跃不支持且不执行跳跃；
4. Esc 锁存急停、R 恢复安全站立；
5. JSONL 能看到原始源动作、映射动作、实际执行动作及限幅/限速差值。

在这些条件通过前，不启动微调，不改变 P1/P2 Gate，不删除历史 checkpoint。
