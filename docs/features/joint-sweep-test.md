# 单关节顺序扫描测试

> 用途：验证 Webots 中机器人关节设备映射、顺序控制和停止链路。
> **本测试不加载 RL/checkpoint 模型，不建立 TCP 连接，也不作为模型、步态、
> 跑酷能力或实机验收。**

## 1. 测试组成

| 项目 | 路径 |
|---|---|
| 世界 | `webots-sim/worlds/joint_sweep_test.wbt` |
| 控制器 | `webots-sim/controllers/joint_sweep_test/joint_sweep_test.py` |
| 控制器单测 | `webots-sim/controllers/joint_sweep_test/test_joint_sweep_test.py` |
| 世界静态测试 | `webots-sim/rl/tests/test_joint_sweep_world.py` |

世界使用与本项目一致的 YoboGo 关节和传感器命名；控制器只读取本地 Webots
设备并发送关节目标，不导入训练环境，不读取 checkpoint，也不创建网络连接。
world 显式配置唯一的 `default/default` 接触对：
`coulombFriction 0.8`、`softERP 0.2`、`softCFM 0.001`、`bounce 0`、
`bounceVelocity 0`；机器人出生高度为 **0.15 m**，避免非足端默认材料回弹
和仰躺初始穿模。Robot 的 `selfCollision` 明确为 **FALSE**：该开关会禁用
机器人部件之间的内部自碰撞检测，本测试世界因此**不验证内部自碰撞**。

## 2. 测试姿态与执行路径

1. 机器人以 `translation 0 0 0.15`、`rotation 1 0 0 1.5708` 的
   **仰躺姿态**进入测试，避免把站立平衡能力混入单关节检查，同时保证初始
   碰撞几何不穿透 `z=0` 地面上表面。
2. 控制器按固定顺序**一次只扫描一个关节**；当前关节未结束时，不驱动下一关节。
3. 当前关节按 `0 → min → 0 → max → 0` 执行，`min` 和 `max` 两个拐点
   实际到位后各停留 **0.5 s**；中间零位不额外停留，未参与扫描的关节保持安全目标。
4. 每段关节运动指令按不超过 **0.20 rad/s** 推进，不使用跳变目标。
5. 位置跟踪误差超过 `0.05 rad` 并连续持续 **10 s** 时触发停止。停止后
   全部电机保持当前实际位置，不继续执行剩余关节；整个序列没有固定总时长，
   正常耗时取决于各关节的 `min`/`max` 绝对值和端点停留时间。
6. 12 个关节全部完成后，所有目标回到零位；实际位置全部进入 `±0.05 rad`
   后才打印“关节扫掠完成，已全部回零”。

启动期低空弹射的**主根因不是重力，而是 Robot 的 `selfCollision TRUE`**。
该 world 未覆盖 `WorldInfo.gravity`，仍使用 Webots 正常向下重力。只读 A/B
实验保持出生位姿、控制器和接触参数不变：

- A：`selfCollision TRUE`，机器人触地后出现低空弹射并触发位置跟踪超时；
- B：仅改为 `selfCollision FALSE`，机器人正常落地，且未出现扫描超时。

因此本测试世界将 `selfCollision` 固定为 `FALSE`。静态测试同时断言该值，避免
回归后再次启用内部自碰撞。此前补充的 `default/default` 零回弹接触对和
`0.15 m` 出生高度仍作为物理稳定性防线保留，但不再被表述为主根因；禁用
`selfCollision` 后，本测试只验证外部碰撞、设备映射和驱动链路，不覆盖部件
间自碰撞约束。

该顺序和路径以
`webots-sim/controllers/joint_sweep_test/joint_sweep_test.py`
中的关节列表为唯一实现来源，文档不另行维护第二份关节编号表。

## 3. 停止方式

先点击 Webots 3D 视图获得焦点，然后可使用：

| 按键 | 行为 |
|---|---|
| `Esc` | 立即停止扫描，所有电机改为保持当前实际位置，未执行关节不再继续 |
| `R` | 与 `Esc` 相同，立即停止扫描并保持当前实际位置 |

`Esc` 和 `R` 均优先于正常推进；人工停止或跟踪误差持续 10 s 后，控制器不会
自行恢复序列，也不会打印正常完成信息。如需重新扫描，应重新启动控制器或
重新打开世界。

## 4. 运行与验收命令

从仓库根目录执行。

### 4.1 GUI 启动

```bash
/usr/local/webots/webots --mode=realtime --stdout --stderr webots-sim/worlds/joint_sweep_test.wbt
```

### 4.2 自动测试

控制器单元测试：

```bash
python3 -m unittest discover -s webots-sim/controllers/joint_sweep_test -p 'test_*.py'
```

world 静态测试（同时覆盖出生高度、`default/default` 完整接触参数和
`webots-sim/rl/tests/` 中已有静态检查）：

```bash
python3 -m unittest discover -s webots-sim/rl/tests -p 'test_*.py'
```

批量加载验收：

```bash
DISPLAY=:0 timeout 300s /usr/local/webots/webots --batch --mode=fast --stdout --stderr webots-sim/worlds/joint_sweep_test.wbt
```

批量验收应同时满足：world 静态检查确认唯一的 `default/default` 接触对为
`coulombFriction 0.8`、`softERP 0.2`、`softCFM 0.001`、`bounce 0`、
`bounceVelocity 0`，出生参数为 `translation 0 0 0.15`、
`rotation 1 0 0 1.5708`，并明确 `selfCollision FALSE`；命令在 300 s
外层超时内自行结束且退出码为 `0`，
日志无 `ERROR`、`Motor ... not found`、`Sensor ... not found`、
`位置误差超时` 或 `Traceback`，并能观察到顺序扫描正常启动、正常完成时出现
“关节扫掠完成，已全部回零”。GUI 操作验收还需确认 `Esc` 和 `R` 均能中止
当前序列且不打印该完成信息。修复后的静态测试首次执行曾暴露解析器问题，
修复解析器后 7 项静态测试、13 项控制器单测和 50 项 RL/world 回归全部通过；
这些结果只证明静态契约与控制器逻辑，不能替代上述动态批量验收。

## 5. 隔离边界

- **不加载模型**：不读取 `checkpoints/` 下的任何 `.zip`，不运行
  Stable-Baselines3 推理，也不执行训练或回放评估。
- **不使用 TCP**：不设置训练桥端口，不进入 `walk_env.py` 的 TCP 锁步协议。
- **不修改训练入口**：不改 `webots-sim/rl/` 下的训练脚本、观测、动作、
  奖励或 checkpoint。
- **不修改 `parkour_dev`**：该测试使用独立 world 和控制器，不改变
  `webots-sim/worlds/parkour_dev.wbt` 的训练用途、出生点或运行契约。
- 本测试只证明设备顺序扫描和停止控制按设计工作，**不证明模型质量、
  步态稳定性、跑酷能力或实机安全**。
- 本测试为规避已由 A/B 定位的低空弹射而禁用 Robot 内部自碰撞，**不证明
  机器人部件之间满足自碰撞约束**；如需验证该能力，应另建 `selfCollision
  TRUE` 的隔离用例，不能混入当前扫描世界。

## 6. 相关文档

- 项目总览与世界清单：[`README.md`](../../README.md)
- 当日修改记录：[`docs/changes/2026-10-02.md`](../changes/2026-10-02.md)
