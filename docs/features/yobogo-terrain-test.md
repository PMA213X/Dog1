# YoboGo 综合地形测试世界（`yobogo_terrain_test`）

> 适用范围：`yobogo_loco_jump_v1` 已有 checkpoint 的 Webots play 地形测试。
> 世界文件：[`webots-sim/worlds/yobogo_terrain_test.wbt`](../../webots-sim/worlds/yobogo_terrain_test.wbt)。
> 更新时间：2026-10-02。

本文记录独立综合地形测试世界的用途、隔离边界、路线尺寸、必须保留的运行契约、
启动方式、日志判读、静态测试预期和能力边界。本文不表示模型已完成地形验收，
也不表示训练、正式评估或实机验收通过。

---

## 1. 用途与隔离边界

`yobogo_terrain_test` 用于在不改动训练源世界的情况下，观察已有 checkpoint
在短距离、多类低矮地形上的启动、接触、跨越和停止表现。它是**测试/诊断世界**，
不是新的训练世界，也不是正式评估协议。

与现有世界的关系如下：

| 项目 | 边界 |
|---|---|
| `parkour_dev.wbt` | 继续作为 RL 训练与既有 play 的源世界，不被新世界覆盖或改写 |
| `parkour.wbt` | 继续作为手动遥控跑酷世界，不作为本测试的 play 世界 |
| `yobogo_terrain_test.wbt` | 只新增独立测试路线；通过显式传入世界路径启动 |
| RL 训练 | 默认世界、运行期世界副本规则、阶段预算、checkpoint 前缀和训练入口均不变 |
| play | 只把固定启动命令最后一行的 world 路径替换为新世界，其余环境变量保持不变 |
| 模型 | 不重新训练，不改观测、动作、50 Hz 契约、动作缩放或 checkpoint |

新世界只使用仓库内的 YoboGo Robot 段与 Webots 内置基础节点组合地形，不引入
第三方完整 `.wbt`。波士顿动力 **Spot 官方世界仅作为障碍组织和尺度的参考**；
本轮不下载、不复制其完整 world，也不带入其 PROTO、网格、控制器或资产。
这样可避免第三方 world 常见的许可证归属不清、远程资源、旧版 PROTO、绝对路径和
控制器不兼容风险。后续若确需引用外部资产，必须先单独核实逐文件许可证并经用户批准。

---

## 2. 路线设计

机器人从原点沿 `+X` 方向测试。路线坐标是约值，用于说明障碍相对出生点的位置；
静态测试以源 world 中的实际尺寸为准。

| 路线位置 | 地形 | 验证目的 |
|---|---|---|
| `0～2 m` | 平整启动区 | 稳定窗口后站立、直行和首段接触 |
| 约 `1.5 m` | 高 `0.10 m` 的矮障碍 | 低矮障碍跨越与足端接触变化 |
| 约 `3 m` | 三段递增台阶，高 `0.08 / 0.16 / 0.24 m` | 逐级上下台阶时的俯仰、抬腿和恢复 |
| 约 `5 m` | 坡道，最高点不超过 `0.20 m` | 连续坡面通过，不把坡道做成高台 |
| 约 `7 m` | 粗糙垫，表面起伏 `0.04～0.08 m` | 随机小高差下的支撑与扰动恢复 |
| 路线末端 | 独立停止区 | 避免机器人直接撞出地面，并保留观察/复位空间 |

启动区 `0～2 m` 必须保持平整且无碰撞物，因为 play 每次 reset 都把机器人恢复到
固定出生点；出生区存在障碍会把模型问题与出生碰撞混在一起。各段之间应留出
可观察的过渡区，粗糙垫只改变局部高度，不引入会卡死足端的窄缝。

---

## 3. 必须保留的运行契约

新世界必须从已验证的 `parkour_dev` YoboGo Robot 段继承以下内容。任何一项缺失，
即使 `.wbt` 能打开，也不能认为与 play/训练契约等价。

| 项目 | 必须值 | 原因 |
|---|---|---|
| 控制器 | `controller "rl_agent"` | play 由 `rl_agent.py` 读取 checkpoint 并执行动作 |
| Supervisor | `supervisor TRUE` | reset、停止和异常离地保护需要 Supervisor 权限 |
| 物理步长 | `basicTimeStep 4` | 每 5 个物理子步组成一个 20 ms 控制周期，保持 50 Hz |
| 出生点 | `translation 0 0 0.26`、`rotation 0 0 1 0` | 与控制器硬编码 reset 位姿一致并朝 `+X` |
| 足端接触材料 | `yobogo_foot/default` | 保持高摩擦、零回弹和软接触顺应参数 |
| 机身接触材料 | `body/default` | 保持机身摩擦、零回弹和软接触参数 |
| 机器人与设备 | YoboGo 原 Robot 段、12 电机、12 位置传感器、4 个足端触觉传感器及惯性单元 | 保持模型、观测与接触输入一致 |

足端 `ContactProperties` 的关键值应与源世界一致：

| 材料对 | 摩擦 / 回弹 | 接触顺应 |
|---|---|---|
| `yobogo_foot` / `default` | `coulombFriction [1.2]`、`bounce 0`、`bounceVelocity 0`、`forceDependentSlip [0.03]` | `softERP 0.45`、`softCFM 0.0005`、`maxContactJoints 20` |
| `body` / `default` | `coulombFriction [0.8]`、`bounce 0`、`bounceVelocity 0` | `softERP 0.2`、`softCFM 0.001` |

地形可以变化，Robot 段、控制器、Supervisor、出生点、时间步长和接触参数不得
为了适配障碍而顺手修改。地面和障碍必须有 `boundingObject`，否则视觉上可见但
不会产生期望的物理接触。

---

## 4. play 启动命令

固定命令来自 [README](../../README.md) 与
[无摄像头遥控运动文档](./remote-rl-locomotion.md)，与原命令相比**只替换最后一行
的 world 路径**：

```bash
env -u RL_BRIDGE_PORT RL_AGENT_MODE=play \
  RL_AGENT_CHECKPOINT=checkpoints/yobogo_loco_jump_v1/yobogo_loco_jump_v1_final.zip \
  RL_AGENT_DEVICE=cpu RL_AGENT_EPISODES=1 RL_AGENT_MAX_STEPS=1000 \
  __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
  /usr/local/webots/webots --mode=realtime \
  webots-sim/worlds/yobogo_terrain_test.wbt
```

`env -u RL_BRIDGE_PORT`、checkpoint、CPU、episode 数、最大步数、NVIDIA 渲染参数、
Webots 可执行文件和实时模式均不得随 world 路径一并改写。命令必须从仓库根目录执行，
并在启动日志确认 checkpoint 解析为绝对路径、稳定窗口放行以及首 10 周期四足接触。

---

## 5. 日志判读

一次 play 结束先按以下三类定位，不能只凭窗口是否关闭判断“成功”：

| 结束特征 | 含义 | 后续处理 |
|---|---|---|
| `【play-safety】异常离地：height=... > 0.600m，停止 play` | 安全护栏主动停止 | 记录停止前后高度、接触和动作，不表述为正常完成 |
| `【rl_agent】episode 1 结束：... done=True` | episode 正常结束 | 结合最后位姿与轨迹判断是否摔倒、是否到达停止区 |
| `Traceback` | Python/控制器异常 | 按故障处理，保存完整 stdout/stderr，不归类为模型正常结束 |

三类之外还要检查启动前的 checkpoint 绝对路径、稳定窗口和四足接触日志。
若稳定窗口已经放行，之后在平整启动区或地形上摔倒，应记为**模型质量问题或
地形能力不足**，不能归因于启动保护未生效。若在 50 周期稳定窗口内异常离地，
则先按安全护栏事件调查，不把它直接解释为模型完成或失败。

推荐将启动 stdout/stderr 保存到 `logs/` 下的独立 play 日志，避免只依赖 Webots
Console 的滚动窗口。三类结束的日志证据应同时保留截图/视频或最后位姿说明。

---

## 6. 静态测试与预期结果

配套静态测试 `webots-sim/rl/tests/test_yobogo_terrain_world.py` 只检查源文件
结构，不启动 Webots。预期覆盖：

| 测试项 | 预期结果 |
|---|---|
| 世界文件 | `yobogo_terrain_test.wbt` 存在、首行为 R2025a VRML 头、括号结构完整 |
| 地形路线 | `floor`、`hurdle`、`stair_08/stair_16/stair_24`、`ramp`、`rough_pad_*` 和 `stop_zone` 可定位；平整启动区、`0.10 m` 矮障碍、`0.08/0.16/0.24 m` 台阶、不超过 `0.20 m` 的坡道、`0.04～0.08 m` 粗糙垫和末端停止区尺寸正确 |
| 控制器与 Supervisor | 恰有一个 YoboGo Robot，绑定 `rl_agent` 且 `supervisor TRUE` |
| 物理与出生点 | `basicTimeStep 4`；出生点与 `rotation 0 0 1 0` 不变 |
| 接触参数 | `yobogo_foot/default` 与 `body/default` 的摩擦、回弹和顺应字段保持源值 |
| 设备 | 12 `RotationalMotor`、12 `PositionSensor`、4 个 `*_foot_touch` `TouchSensor` 及 1 个 `InertialUnit` 齐全 |
| 视角与资源 | 恰有 1 个 `DEF VP_FOLLOW Viewpoint`；无 `EXTERNPROTO`、HTTP URL、作者绝对路径或下载资源 |
| 资源自包含 | 不依赖下载的第三方完整 world，不覆盖 `parkour_dev.wbt` |

测试通过只证明文档所述源 world 静态结构存在且一致；它不证明 Webots GUI 加载为
0 ERROR，也不证明 checkpoint 能走完整路线。

后续运行验收的预期结果为：

1. play 启动日志显示 checkpoint 绝对路径、稳定窗口通过和连续 10 周期四足接触；
2. Webots Console 不出现 `Motor not found`、设备缺失、远程资源或 Python `Traceback`；
3. 机器人从平整区起步，能分别对矮障碍、台阶、坡道和粗糙垫给出可观察的跨越响应；
4. 到达末端停止区后按 episode 正常结束，或在记录为模型质量/能力不足的摔倒中结束；
5. 运行后 `parkour_dev.wbt`、训练默认世界和训练入口均无变化。

第 1～5 项均需实际运行日志或录像佐证，不能用静态测试结果替代。

---

## 7. 明确不改变的范围

- 本轮不重新训练，不启动新的训练阶段，也不生成新的正式 checkpoint；
- `parkour_dev.wbt` 与训练入口不改，RL 训练默认世界及其运行期副本规则不改；
- 55 维观测、12 维动作、50 Hz 契约、动作缩放、TCP 协议和安全保护参数不改；
- 不把静态测试通过、稳定窗口放行或一次播放未触发护栏表述为模型验收；
- 稳定窗口后摔倒记录为模型质量/地形能力问题，不以测试世界缺陷掩盖；
- 不下载或复制第三方完整 `.wbt`，不引入其许可证和依赖风险。

---

相关文档：

- [README](../../README.md)
- [Webots 与 RL 的本地 TCP 桥协议](../api/rl-webots-tcp-bridge.md)
- [无摄像头遥控运动与跳跃](./remote-rl-locomotion.md)
- [RL 训练完全指南](./rl-training-guide.md)
- [2026-10-02 修改日志](../changes/2026-10-02.md)
