# 官方 Mini Cheetah 平地主动控制

## 能力边界

本功能在 `official-mini-cheetah/` 独立目录内提供两类平地主动控制：手动键盘/手柄遥控，以及无需人工输入、按固定时间脚本执行的简化步态。主动 world 从被动验收 Robot 派生，但只替换 controller，不改变官方 Mini Cheetah 的质量、惯量、碰撞、网格、关节轴、设备名或零位几何。

本轮能力仅限 Webots R2025a 平地仿真。手动遥控是安全限幅的站立/简化速度驱动 trot，不是最优步态；固定脚本是确定性动作回放/步态链路验证，不是轨迹规划器、强化学习策略或实机控制。被动平地验收 PASS 不能外推为步态、运动性能或实机能力通过。

### 2026-10-03 姿态修订

主动站立目标已改为参考图推导的 `DEFAULT_CROUCH`，目标顺序为
`fr/fl/hr/hl × abd/hip/kn`：右腿 abad `-0.15`、左腿 `+0.15`，hip
`-0.80`、knee `1.60`。固定时间线的 `SLIGHTLY_EXTENDED` 为 hip
`-0.55`、knee `1.15`，仍保持明显弯膝。12 路目标有显式限位和
`0.75 rad/s` 变化率限制。

姿态相关主动 world 的出生高度为 `0.45 m`，12 个 `springConstant` 设为
`0`，以便位置控制完成屈膝目标；质量、COM、惯量、碰撞、DAE 和被动
`official_flat_ground_test.wbt` 均保持不变。详细时间线、测试命令和
Webots PASS 证据见 `official-mini-cheetah-posture-timeline.md`。

## World 与模型不变性

- 手动 world：`official-mini-cheetah/worlds/flat_ground_teleop.wbt`，主 Robot controller 为 `flat_ground_teleop`；
- 固定脚本 world：`official-mini-cheetah/worlds/script_walk_test.wbt`，主 Robot controller 为 `script_walk`；
- 被动 world `official_flat_ground_test.wbt` 及其 `acceptance_supervisor`、验收 artifacts 保持不变；
- `flat_ground_teleop.wbt` 和 `script_walk_test.wbt` 不含 `supervisor`、
  `acceptance_supervisor` 或被动验收节点；新增 `posture_timeline_test.wbt`
  的主 Robot 使用只读 `supervisor TRUE`，但不绑定验收 Supervisor；
- 主动/被动 Robot 均包含 12 `endPoint`、12 `RotationalMotor`、12 `PositionSensor`，关节轴为 4 个 X 轴和 8 个 Y 轴；
- 除 `controller`/`controllerArgs` 以及主动姿态测试所需的出生高度和
  `springConstant` 字段外，主动 Robot 与被动 Robot 结构一致。

## 手动遥控

控制器入口为 `official-mini-cheetah/controllers/flat_ground_teleop/flat_ground_teleop.py`。输入、状态机和 PD 逻辑拆分为纯模块：`teleop_types.py`、`teleop_input.py`、`teleop_state.py`、`joint_safety.py`、`mini_cheetah_pd.py`。这些模块可在没有 Webots `controller` 包的普通 Python 环境导入。

### 控制映射

| 输入 | 语义 |
|---|---|
| `Space` | 站立/行走模式切换，优先于同帧 `A` 的移动语义 |
| `A` | 站立时上升沿进入行走且该帧不横移；下一帧持续 `A` 向左横移；行走时只左移 |
| `W/S` | 前进 `+vx` / 后退 `-vx` |
| `A/D` | 左移 `+vy` / 右移 `-vy` |
| `Q/E` | 左转 `+wz` / 右转 `-wz` |
| `R` | 回站立并清空模式、输入和步态状态 |
| `Esc` | 强制站立 |
| 手柄轴 0/1/2 | `vy/vx/wz`，死区 `0.12`，Webots 有符号轴值归一化 |
| 手柄 A/B 钮 | 模式切换/复位 |

同轴冲突时最近采样键优先；键盘非零分量优先于手柄，边沿按逻辑或合并。速度限制为 `vx∈[-0.3,0.6]`、`vy∈[-0.3,0.3]`、`wz∈[-1,1]`。

### 安全规则

默认站立目标为 `DEFAULT_CROUCH`，站立态忽略运动输入。`Space`、`A`、`R`、
`Esc` 使用明确边沿/优先级；`R`/`Esc`、输入超时 `0.35 s`、手柄断连、
NaN/Inf、设备缺失、反馈越界或非有限值均回站立。12 路 PD 使用
`KP=[3,3,3]`、`KD=[1,0.2,0.2]`，控制器力矩限幅 `15 N·m`；world 中
`maxTorque=20` 仅代表设备能力。姿态时间线测试使用独立位置控制入口。

## 固定脚本步态

固定脚本使用独立 `official-mini-cheetah/controllers/script_walk/` 控制器和 `script_walk_test.wbt`。已确认的纯接口契约为：

- `gait_planner.ScriptWalkPlanner.sample(time) -> PlannerFrame`；
- `gait_planner.leg_ik`、`foot_target_at`、`touchdown_foot_x`；
- `joint_control.SafePDController.command(...) -> PDCommand`；
- `script_walk.ScriptWalkController`、`script_walk.main()`；
- `MOTOR_NAMES`、`SENSOR_NAMES`、`DEVICE_ORDER`、`SCRIPT_SEGMENTS`。

脚本按固定时间分段生成足端目标、逆解关节目标并经安全 PD 输出 12 路力矩。最终静态测试和 Webots 运行证据由独立 `test_script_walk.py` 与 `script_walk_run` 验收后回填；在证据完成前，本文件不宣称脚本实现或运行 PASS。

## 运行命令

```bash
# 手动 Webots 遥控
DISPLAY=:0 /usr/local/webots/webots \
  --mode=realtime --stdout --stderr \
  official-mini-cheetah/worlds/flat_ground_teleop.wbt

# 固定脚本测试 world
DISPLAY=:0 /usr/local/webots/webots \
  --mode=realtime --stdout --stderr \
  official-mini-cheetah/worlds/script_walk_test.wbt
```

手动运行需点击 Webots 3D 视图使键盘获得焦点。固定脚本运行命令和证据路径待独立运行代理完成后补充。

## 测试命令与当前结果

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B \
  official-mini-cheetah/tests/test_flat_ground_teleop.py -v
```

当前结果：`Ran 13 tests`、`OK`、退出码 `0`。覆盖主动/被动 world 隔离、模型除 controller 外一致、12 设备与 4X/8Y 轴、无 Webots import、PD/限幅/NaN、默认站立、Space/A 冲突、W/S/A/D/Q/E、R/Esc、手柄死区/断连/NaN/Inf/输入超时。

固定脚本测试命令、运行日志和退出码待独立验收后更新：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B \
  official-mini-cheetah/tests/test_script_walk.py -v
```

## 已知限制

- 未连接实体 Webots Joystick，实体手柄按钮/轴映射未实测；
- 手动 trot 是简化速度驱动控制，未做复杂地形、跳跃、鲁棒性或性能验收；
- 固定脚本的最终静态测试、Webots 运行和性能/稳定性结论待独立证据；
- 被动 PASS 仅证明官方模型平地被动落稳，不表示主动控制、步态或实机能力通过。
