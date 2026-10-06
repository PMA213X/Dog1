# 官方 Mini Cheetah 平地被动验收

## 当前结论

**当前验收通过（仅限本文件定义的平地被动验收）。**

官方 Mini Cheetah 的生成器自检、9 项静态检查和 Webots R2025a 批量运行均为
PASS。Webots 以 `--batch --mode=fast` 在约 12 秒墙钟时间内完成 10 仿真秒
观测并以退出码 `0` 退出；`acceptance_summary.json` 的 `status` 为 `PASS`，
`duration_observed_s` 为 `10.0`，`sample_count` 为 `2501`，
`failure_reasons` 为空。渲染截图已人工检查，机器狗使用官方 DAE 外观、
四足落地姿态自然且没有被相机裁切。

该结果只证明官方模型在指定平地、无电机指令条件下的被动落稳判据，不表示
步态能力、运动性能或实机能力通过。

## 已实施范围

本轮新增独立目录 `official-mini-cheetah/`，不修改源模型或既有 YoboGo
world、controller 和测试：

| 路径 | 内容 |
|---|---|
| `official-mini-cheetah/worlds/official_flat_ground_test.wbt` | 官方 Mini Cheetah 静态平地被动验收 world，出生高度为 `0.40 m` |
| `official-mini-cheetah/tools/gen_official_mini_cheetah.py` | 从官方 Robot 文本提取、生成并交叉自检的生成器 |
| `official-mini-cheetah/controllers/acceptance_supervisor/acceptance_supervisor.py` | 独立、只读的 10 秒验收 Supervisor |
| `official-mini-cheetah/tests/test_official_flat_ground.py` | world、生成器和 Supervisor 的 9 项静态测试 |
| `official-mini-cheetah/artifacts/webots_batch.log` | Webots R2025a 批量运行的当前成功日志 |
| `official-mini-cheetah/artifacts/acceptance_summary.json` | 10 秒运行的机器可读 PASS 摘要 |
| `official-mini-cheetah/artifacts/acceptance_samples.jsonl` | 10 秒运行的逐采样记录 |
| `official-mini-cheetah/artifacts/official_model_visual_validation.png` | 带渲染的官方模型视觉验收截图 |
| `official-mini-cheetah/artifacts/runtime_failure.json` | 首次兼容迁移前的历史 FAIL 摘要，仅保留作历史证据 |

当前成功运行已生成采样与摘要；`runtime_failure.json` 不再描述最新运行状态，
最新状态以 `acceptance_summary.json` 和 `webots_batch.log` 为准。

## 官方模型来源

- Robot 文本主来源为仓库现有的
  `webots-sim/worlds/mini_cheetah.wbt`，其中 `Robot` 节点包含
  `translation 0 0 0.38` 和 `controller "mini_cheetah_controller"`；
- 生成器只提取该 `Robot` 节点，不修改参考 world；
- C++ 动力学真源为
  `Cheetah-Software/common/include/Dynamics/MiniCheetah.h`，用于核对本体
  尺寸、质量、连杆长度和惯量；
- URDF 真源为 `webots-sim/urdf/mini_cheetah.urdf`，用于核对 12 个主动
  关节、关节轴、安装点、质量和惯量；
- URDF 引用的官方资源为 `webots-sim/urdf/meshes/mini_body.dae`、
  `mini_abad.dae`、`mini_upper_link.dae` 和 `mini_lower_link.dae`；
  生成器验证这些文件存在、单位为米且 `Z_UP`，再按 URDF 的逐 link
  `origin/rpy` 放置 `Mesh`；
- 生成结果必须保持 12 个 `HingeJoint`、12 个 `RotationalMotor`、12 个
  `PositionSensor`、4 个 abad 轴和 8 个 hip/knee 轴，并禁止混入
  `yobogo`。

为兼容 Webots R2025a，生成器把旧 `child Solid` 转为 `endPoint Solid`，按
`anchor + child translation` 保留零位几何；把 Solid 的直接 `HingeJoint`
字段移入其 `children` 列表，并显式写入 `position 0`。同时清理
`InertialUnit.coordinateSystem`、`HingeJointParameters.stopSpringDamper` 和
`Physics.frictionMaterial`，把摩擦参数放入 `WorldInfo.contactProperties`，
把 3x3 `inertiaMatrix` 转成 R2025a 的两行格式；用官方 DAE `Mesh` 替换
简化 Box/Sphere 外观，按 URDF 的 RPY/平移放入 `Pose`，用官方 diffuse
`0.8 0.8 0.8`，并用 `Group` 承载小腿箱体与 toe 球体的多个碰撞子节点。
大网格设置 `castShadows FALSE` 以消除 R2025a 三角形阴影警告。

生成器对 world、C++ 和 URDF 三份来源执行交叉自检。生成器报告的
`cpp_body_mass` 与 `urdf_body_mass` 均为 `3.3`，`official_robot_joints`、
`official_robot_motors`、`official_robot_sensors` 和 `urdf_joints`
均为 `12`。

## 出生高度与控制器策略

- 生成器将源 Robot 的根高度从 `0.38 m` 调整为验收出生高度
  **`0.40 m`**，旋转固定为 `rotation 0 0 1 0`；
- 主 Robot 必须且只能有一个 `controller "<none>"`，并带
  `controllerArgs []`；
- 主 Robot 的原 `mini_cheetah_controller` 被禁用，是因为本验收只检查官方
  模型在平地上的**被动落稳**，不允许步态 PD、站立控制或任何电机驱动；
- 只读记录职责放在独立的 `DEF ACCEPTANCE_SUPERVISOR Robot` 节点并设置
  `supervisor TRUE`，其 `acceptance_supervisor` 不获取电机接口、不设置
  力矩/速度/位置；
- 该静态结构必须同时保持 `selfCollision TRUE`，避免静态测试把源模型
  当成 YoboGo 模型处理。

## 测试命令与结果

以下命令均从仓库根目录执行。`PYTHONDONTWRITEBYTECODE=1` 和 `-B` 用于保证
复核过程不在只读所有权目录中生成 `__pycache__`。

### 1. 静态测试

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B \
  official-mini-cheetah/tests/test_official_flat_ground.py -v
```

结果：**PASS**，`Ran 9 tests`、`OK`，退出码 **`0`**。

9 项分别覆盖：

1. 生成器默认文本与 world 中 Robot 完全一致；
2. 出生高度、旋转、控制器禁用和模型隔离；
3. 12 关节、电机、传感器、轴与显式零位；
4. 唯一静态平地和场景节点；
5. 四足 toe 接触零回弹；
6. 官方尺寸、质量和惯量常量；
7. anchor、endPoint 与 URDF 零位世界坐标误差小于 1 mm；
8. Supervisor 源码只读且不控制电机；
9. Supervisor 节点绑定与固定 artifact 文件名。

仓库内 `discover -p 'test_*.py'` 还会命中范围外的
`tests/test_manual_viewer.py`，该测试要求不存在的 `index.html`；本任务明确
禁止制作网页，因此未创建该文件，也不能把此范围外失败混入本验收结论。

### 2. 模型生成器

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B \
  official-mini-cheetah/tools/gen_official_mini_cheetah.py --report-json
```

结果：**PASS**，退出码 **`0`**；标准错误报告包含
`"status": "PASS"`、`endpoint_count: 12`、
`max_anchor_endpoint_error: 0.0`、`max_zero_world_error: 0.0`。

### 3. Webots 批量运行

```bash
DISPLAY=:0 timeout 90s /usr/local/webots/webots \
  --batch --stdout --stderr --mode=fast --no-rendering \
  official-mini-cheetah/worlds/official_flat_ground_test.wbt
```

结果：**PASS**，退出码 **`0`**，未触发 `timeout 90s`。当前日志只有
`acceptance_supervisor: Starting controller` 与 `Terminating` 两条 INFO，
没有 ERROR、WARNING、未知字段或丢失关节消息。

摘要关键值：

- `duration_observed_s = 10.0`、`sample_count = 2501`；
- `joint_count = 12`、`nan_anomaly_samples = 0`；
- `contact_samples = 258`、`robot_controller_commands = 0`；
- `max_geometry_penetration = 0.00124288 m`、
  `max_persistent_penetration_seconds = 0.0`；
- `final_linear_speed ≈ 1.15e-09 m/s`、
  `final_angular_speed ≈ 1.45e-07 rad/s`。

### 4. 带渲染截图

```bash
DISPLAY=:0 /usr/local/webots/webots --mode=realtime --stdout --stderr \
  official-mini-cheetah/worlds/official_flat_ground_test.wbt
```

在窗口出现后使用 ImageMagick `import -window <webots-window-id>` 截图，输出：

`official-mini-cheetah/artifacts/official_model_visual_validation.png`

截图人工核对结果：官方 `mini_body/abad/upper/lower` 网格可加载，机器狗完整
可见，四足与地面接触姿态自然，未出现 Box 占位外观或网格缺失。

## 证据路径

| 证据 | 路径 |
|---|---|
| Webots 当前成功日志 | `official-mini-cheetah/artifacts/webots_batch.log` |
| 10 秒 PASS 结构化摘要 | `official-mini-cheetah/artifacts/acceptance_summary.json` |
| 10 秒逐采样记录 | `official-mini-cheetah/artifacts/acceptance_samples.jsonl` |
| 官方模型视觉截图 | `official-mini-cheetah/artifacts/official_model_visual_validation.png` |
| 首次失败历史摘要 | `official-mini-cheetah/artifacts/runtime_failure.json` |
| 9 项静态测试源码 | `official-mini-cheetah/tests/test_official_flat_ground.py` |
| 生成器与三源交叉自检 | `official-mini-cheetah/tools/gen_official_mini_cheetah.py` |
| 被验收 world | `official-mini-cheetah/worlds/official_flat_ground_test.wbt` |
| 只读 Supervisor | `official-mini-cheetah/controllers/acceptance_supervisor/acceptance_supervisor.py` |

结构化摘要中的绝对证据日志路径为：

`/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/official-mini-cheetah/artifacts/webots_batch.log`

## 验收门槛与后续

当前状态为 **PASS / 平地被动验收与官方视觉验收通过**。放行条件已全部满足：
12 个关节在 R2025a 运行时存在，官方 DAE 资源可加载，独立 Supervisor 启动，
10 秒采样与摘要生成，被动落稳判据满足，且截图人工确认视觉一致。

本文件不覆盖步态控制、主动站立、复杂地形、运动性能或实机能力；这些项目
必须另行设计验收，不能由本轮 PASS 外推。
