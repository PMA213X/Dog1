---
feature: yobogo-model-camera
status: delivered
updated: 2026-10-01
branch: master
---

# YoboGo-10S 真实模型与相机 Webots 化

## Report

**What was built（2026-09-25 初版）** — `webots-sim/worlds/msl_match.wbt` 中的 MiniCheetah 盒模型（8.85 kg / 无相机）已替换为 **YoboGo-10S 实机规格模型**（`yobogo_10s`）：按 **Scheme A 包络缩放**重建 12 DOF 运动学链（说明书 **10.5 kg / 485×275×300 mm**），站立高与出生 z 对齐 yaml `des_p` **0.26**，机身惯量直接取 **RPC_inertia** [0.07, 0.26, 0.242]，电机 `maxTorque` 对齐 CAN 协议 ±18 N·m。几何由 `webots-sim/tools/gen_yobogo_robot.py` 作为**单一真源**生成（常量带来源标注 + 内置自检），world 机器人段通过受控替换流程更新；`mini_cheetah.wbt` 旧盒模型保留不动。初版腿段（大腿 **0.14** / 小腿 **0.12** m）为包络缩放估算，不编造实测值。

**2026-09-30 修订 / 2026-10-01 验收** — 上述 `0.14/0.12 m` 方案暴露出“腿段和恰好等于站高 → 模型零位直腿”的几何问题。本轮依据同款机器人课件改为**等效三关节屈膝模型**：保留每腿 `abd/hip/kn` 三驱动关节，采用工程暂定 `0.15/0.15 m` 与约 `120°` 膝部内角，补全显式限位、足端碰撞和四连杆外观；四连杆只作外观，不增加闭链。生成器、三个源 world、静态检查和 Webots 加载验收均已完成，详见 S4.5；精确杆长、真实限位和数值零位仍待 CAD/实机确认。

相机方面新增前置 `front_camera`（**640×480**），`fieldOfView` 与俯仰 pitch 为**一眼可见可改**的显式参数；当前为 **1.05 rad / −0.44 rad（约 −25°）**，生成器注释仍保留初始推荐 1.05 rad / −35° 与调参范围。质量合计 3.92 + 4×1.644 = **10.496** kg（≈ A 级 10.5）。溯源分级见 S2.1；当前模型参数另按说明书、课件、现有代码、图片比例、工程估算和待 CAD 确认分级。

**Verification（2026-10-01 新屈膝模型）** — 生成器自检 **PASS**：12 关节 `endPoint==anchor`、总质量 10.496 kg、`0.15/0.15 m`、膝内角 120°、足端球最低点约 z=0、12 组硬限位与 12 组软限位。静态检查 **PASS**；`msl_match.wbt`、`parkour.wbt`、`parkour_dev.wbt` 批处理加载均退出码 0、0 ERROR、0 Motor/Sensor not found；`parkour_dev` 另完成 50 步 `obs_dim=42/action_dim=12` 冒烟。2026-09-25 的旧直腿验证仅保留为历史记录。

**Journey log**
1. 腿段长度/质量在仓库内**缺失**（无 CAD / 实测），按用户决策采用 **Scheme A 包络缩放**工程估计（大腿 0.14 / 小腿 0.12 m），并在 S2.2 标注「估算」——C 级缺失项禁止编造真实值。
2. Webots R2025a Camera 视场角字段是 **`fieldOfView`**（**不是** `fov`）；光轴 = 相机局部 **+X**，俯仰 = 绕 **+Y** 右手旋转**正角 = 低头**（pitch −35° 写作 `rotation 0 1 0 0.61`）。
3. `endPoint.translation` **必须等于** joint `anchor`（R2025a 中相对父坐标系而非 anchor 系）；按「连杆原点在关节处」约定生成，12 关节 endPoint==anchor。
4. abad 盒初版超出 275 mm 包络，将偏置/盒宽**缩窄至 0.065 m** 后落回包络内。
5. 单腿质量算术初记 1.645 有误：0.64+0.75+0.254 = **1.644**，总和 4×1.644+3.92 = **10.496**（非精确 10.5，容差内对齐）。

## [S1] Problem

当前 Webots 机器人（`webots-sim/worlds/msl_match.wbt` 内联 mini_cheetah）是 MiniCheetah 盒模型：总质量约 **8.85 kg**（机身 3.3 + 4×(0.214+0.634+0.54)，见 `msl_match.wbt:221,240,259,799`），包络约 **0.38×0.22 m**（机身 Box `0.38 0.1 0.06`，`msl_match.wbt:92`；髋位 x=±0.19、腿侧展至 y≈±0.111），且 **无任何相机**。实机为 **YoboGo-10S**：485×275×300 mm、10.5 kg、12 DOF（`YoboGo-control/YoboGo-10S使用说明书(开源).docx` §1.3 基本参数；摘录 `docs/features/yobogo-manual.md:29-31`）。物理保真度与视觉能力均不足，无法做贴近实机的仿真验证。

用户决策（2026-09-25）：
- 几何方案 = **Scheme A（按实机包络缩放几何）**，不追求 CAD 级腿段真实值（仓库无）；
- 相机安装 = **按代码推断**（track1.6 视觉链路反推）；
- FOV 与 pitch = **必须是易调参数**（注释标明推荐值与范围）；
- 性能参考 = **YoboGo-control 官方代码**（`robot-software/config/mc-mit-ctrl-user-parameters.yaml` 等）。

## [S2] Design

### S2.1 参数溯源表（Provenance）

所有数值按下表溯源。可信度分级：**A** = 官方说明书原文；**B** = 官方开源代码/配置；**C** = 仓库缺失、**不得编造**（仅允许标注为工程估计）。

| 参数 | 数值 | 来源 | 可信度 |
|---|---|---|---|
| 尺寸 | 485×275×300 mm | 说明书 §1.3 基本参数（`yobogo-manual.md:29`） | A |
| 重量 | 10.5 kg | 说明书 §1.3（`yobogo-manual.md:30`） | A |
| 自由度 | 12（每腿 3） | 说明书 §1.3（`yobogo-manual.md:31`） | A |
| 力矩协议范围 | ±18 N·m | 说明书 §六 电机配置与使用，MC-MIT CAN 位域 `torque -18NM ~ 18NM` | A |
| 步高 | 0~0.2 m | 说明书 遥控器 Vrb 旋钮（`yobogo-manual.md:249`） | A |
| 站立高度 des_p z | 0.26 m | `YoboGo-control/robot-software/config/mc-mit-ctrl-user-parameters.yaml:87` | B |
| 抬腿高度 Swing_traj_height | 0.07 m | 同上 `:50` | B |
| 机身惯量 RPC_inertia | [0.07, 0.26, 0.242] | 同上 `:77` | B |
| 机身质量 RPC_mass | 9 kg | 同上 `:76` | B |
| 关节 Kp | [3, 3, 3] | 同上 `:5`（Kp_joint） | B |
| 关节 Kd | [1, 0.2, 0.2] | 同上 `:4`（Kd_joint） | B |
| 步态周期 gait_period_time | 0.5 s | 同上 `:97` | B |
| 期望速度上限 des_dp_max | [1.0, 0.5, 0] | 同上 `:92` | B |
| 相机分辨率 | 640×480 | `YoboGo-control/track1.6/track/main.cpp:24`（`Size(640,480)`） | B |
| 视觉仅用上半图 | rows 0–149 | 同上 `:168`（`i < frame.rows/2.0`，作用于 :203 缩放后的 400×300 图） | B |
| 目标中线 goalAverage | 200 | 同上 `:201`（同义默认 `average=200` 见 `:138`） | B |
| 控制站立高 stand_height | 0.3 默认 | 同上 `:31` | B |
| 腿段长度/质量、机身单独质量、减速比、电机型号 | **缺失** | 仓库无 CAD/实测数据 | **C — 禁止编造** |

C 级缺失项对应几何一律按 **Scheme A 包络缩放工程估计**，并在 S2.2 标注「估算」。

### S2.2 几何（Scheme A 包络缩放 — 估算）

> **2026-09-30 注**：本节保留为 2026-09-25 初版历史。`thigh=0.14 m`、`shank=0.12 m` 的直腿零位方案已由 S4 已验收的 `0.15/0.15 m + 约120°屈膝` 结果取代；不得同时把两套数值当作当前参数。

按 485×275×300 mm 包络 + 10.5 kg 总重反推分配（**估算**，无 C 级真实腿段数据）。质量/惯量取 B 级官方值（RPC_inertia / RPC_mass 直接沿用）：

| 部件 | 数值 | 来源 |
|---|---|---|
| 机身 Box | 0.40 × 0.13 × 0.10 m | 估算（包络内分配） |
| 机身 mass | 3.92 kg | 估算；4×1.644+3.92 = **10.496**（≈ A 级 10.5 kg） |
| 机身 inertia | [0.07, 0.26, 0.242] | **B 级直接取用** `RPC_inertia`（yaml:77） |
| 髋挂载点 | (±0.18, ±0.052, 0) | 估算 |
| abad 偏置 | ±0.065 m | 估算 |
| 大腿 thigh | 0.14 m | 估算 |
| 小腿 shank | 0.12 m | 估算 |
| 足端 toe | r = 0.02 m | 估算 |
| 单腿质量 | 1.644 kg（abad 0.64 / thigh 0.75 / shank 0.254） | 估算；4×1.644+3.92 = **10.496**（≈ A 级 10.5） |
| 电机 maxTorque | 18.0 N·m | **A 级** 协议值 ±18 N·m（说明书 §六）；现值 20.0（`msl_match.wbt:105` 等） |
| 出生位 translation | `8 0 0.26` | z=0.26 对齐 B 级 `des_p`（yaml:87）；xy 沿用 MSL 开球点 |
| 出生位 rotation | `0 0 1 π` | 沿用 `msl_match.wbt`（面向 -X 朝场地中心，见 msl-match-field 经验） |

验收包络：≈ **485×275×300 mm**、总质量 **10.5 kg**、12 关节 `endPoint` 锚点一致。

### S2.3 相机（安装为代码推断）

| 项 | 值 | 来源 |
|---|---|---|
| 分辨率 | 640×480 | **B 级** `main.cpp:24` |
| fov | **1.05 rad ≈ 60°** | 推断（UVC 摄像头常见 55–70°）；**易调** |
| pitch | 初始推荐 **-35°（≈ -0.61 rad）**；当前 **-0.44 rad（≈ -25°）** | 初始按视觉上半图推断；当前为看到 3 m 外球体而抬高，**易调** |
| translation | `0.20 0 0.055`（前上方） | 推断：机身长 485 mm / 站立高 0.30（`main.cpp:31`）→ 前上位置 |

**易调要求**：`fov` 与 `pitch` 必须写成一眼可见可改的显式参数（生成脚本顶部常量 / 世界文件内带注释字段），注释标明：

- `fov` 推荐 1.05 rad，范围 **0.96–1.22 rad**（≈55°–70°）；
- `pitch` 推荐 -0.61 rad（-35°），范围 **-0.52 ~ -0.70 rad**（≈ -30° ~ -40°）。

> **当前值限定**：生成器与三个源 world 的实际 pitch 为 `-0.44 rad`（约 -25°），低于上述初始建议范围；`fieldOfView` 当前仍为 1.05 rad。初始范围是调参建议，不是本轮模型验收条件。

其余（translation / 分辨率）同为显式参数，但优先级低于 fov/pitch。

### S2.4 性能对齐（对照官方代码）

| 项 | 官方值 | 现 Webots 值 | 处置 |
|---|---|---|---|
| 关节 PD | Kp=[3,3,3], Kd=[1,0.2,0.2]（yaml:5,4） | 控制器 `KP[3]={3,3,3}`, `KD[3]={1,0.2,0.2}`（`mini_cheetah_controller.cpp:40-41`） | **已一致，不动** |
| 电机力矩上限 | 协议 ±18 N·m（说明书 §六） | 当前 `maxTorque 18.0` | **已对齐** |
| 站立高度 | des_p z = 0.26（yaml:87） | 当前出生 z = 0.26 | **已对齐** |

控制器侧 `MAX_TORQUE = 15.0`（`mini_cheetah_controller.cpp:42`）为软件限幅，本特性不改控制器逻辑。

### S2.5 验收与测试边界

- 生成脚本产出的机器人段可被 Webots R2025a 加载，**0 ERROR**、12 电机/12 传感器全部就绪。
- 12 关节 `endPoint` 与 anchor 对齐（沿用 msl-match-field「连杆原点在关节处」约定，避免 endPoint 平移语义坑）。
- 总质量 10.5 kg、包络 ≈ 485×275×300 mm 可由脚本断言。
- 相机节点 640×480，`fov`/`pitch` 参数一眼可见可改，改动后加载无错误。
- 控制器仍可绑定运行（`make` 通过，可站立）。
- 不引入控制器逻辑/协议变更。

## [S3] Out of Scope

- 控制器逻辑 / 通信协议（LCM / SPIne）接入 Webots。
- 视觉算法（track1.6 循迹）搬进 Webots。
- 实机 CAD 级腿段真实值（仓库无数据，仅 Scheme A 估算）。
- 多相机 / 深度相机 / T265 等其它传感器。

## [S4] 2026-09-30 等效三关节模型重构（已交付）

### S4.1 目标与不变边界

- 修复旧模型 `0.14 + 0.12 = 0.26 m` 导致的零位直腿、视觉腿长失真和足端由小腿盒边触地；
- 保留 12 个 `HingeJoint`、12 个 `RotationalMotor`、12 个 `PositionSensor`，设备名 `{fr,fl,hr,hl}_{abd,hip,kn}_motor/_sensor`、腿序和关节序不变；
- 保持 `X=前、Y=左、Z=上`，课件坐标先转换到 Webots 坐标；
- 不修改 `rl_agent.py`、`walk_env*.py`、PPO 配置、TCP `state/act` 或 C++ 控制器；
- `OBS_DIM=42/45/51`、`ACTION_DIM=12` 不变；
- 不重建 `mini_cheetah.wbt`，不手改 `.parkour*_rl.wbt`、`.parkour_view*.wbt`；
- 旧 checkpoint 标记为旧模型资产，不承诺继续有效。

### S4.2 参数与来源分级

| 参数 | 本轮值 / 结论 | 来源分级 |
|---|---|---|
| 整机 | 485×275×300 mm、约 10.5 kg、12 DOF | **说明书原文** |
| 拓扑、膝结构、足端、定性零位 | 横滚+两俯仰、平行四边形膝、柔性足弧面、`Touch Sensor`、摆到限位的零位描述 | **课件原文** |
| 站立高度 0.26 m | 仅作操作参考 | **现有代码** |
| `thigh/shank` | **0.15 / 0.15 m** | **工程暂定** |
| 膝部内角 | 约 **120°** | **工程暂定** |
| 髋安装点 | `(±0.18, ±0.052)` m | **现有代码 + 工程估算** |
| 限位 | `abd ±0.6`、`hip ±1.0`、`kn ±1.0 rad` | **工程暂定** |
| 屈膝方向、四连杆比例、足底形状 | 只作定性核对 | **图片比例测量** |
| 精确腿长、四连杆杆长、真实机械限位、数值零位偏置 | 未确认 | **待实机/CAD确认** |

课件未提供三关节数值机械限位、四连杆杆长或数值零位，因此上述 `0.15/0.15 m`、约 `120°`、限位均必须明确标为工程暂定值；不得移植 Mini Cheetah 的 `0.062/0.209/0.18 m` 或减速比。

### S4.3 零位、限位、碰撞与外观

- 通过固定 frame/endpoint 偏置，使模型关节零位对应自然屈膝站姿；模型零位、实机编码器零位、自然站立角分开记录。
- 12 个关节全部显式写 `minPosition/maxPosition` 和 `minStop/maxStop`，暂定 `abd ±0.6 rad`、`hip ±1.0 rad`、`kn ±1.0 rad`，`maxVelocity` 保持 10 rad/s。
- 足端球/弧面已纳入 `boundingObject`，四条腿具有独立接触材质、摩擦和阻尼；已新增 4 个 `*_foot_touch`，但不接入现有 RL 观测。
- 四连杆/平行四边形外观跟随三个驱动关节运动；不增加闭链或被动关节。

### S4.4 世界文件与兼容性

- `gen_yobogo_robot.py` 是唯一模型真源；
- 重新生成 `msl_match.wbt`、`parkour.wbt`、`parkour_dev.wbt`；
- 核对 `tools/build_parkour_world.py` 对默认出生位姿字符串的依赖；
- 三个源 world 的核心机器人段一致；允许 controller、出生位姿不同，`parkour_dev` 另保留既有 `supervisor TRUE`；
- 生成器带文件参数时会整体覆盖输出文件，不能把源 world 直接当输出参数；仓库当前缺少可复用的非破坏性 Robot 段替换命令，后续重生成前需先补工具或沿用已验证流程；
- 几何、零位、碰撞和惯量改变后，即使接口维度相同，旧 checkpoint 也不能作为新模型回放、热启动或正式验收依据。

### S4.5 验收边界

**已完成：**

- 生成器静态断言 **PASS**：12/12/12 计数、`endPoint==anchor`、12 组软/硬限位、质量/惯量/质心/包络、120° 屈膝零位和足端球最低点 z≈0；
- 三个源 world 核心机器人段一致性 **PASS**，`mini_cheetah.wbt` 未修改；
- 三个 world Webots 批处理加载 **PASS**：退出码 0、0 ERROR、0 Motor/Sensor not found，仅既有重复 `Viewpoint` 警告；
- 足端 4 个球碰撞体、4 个 `*_foot_touch` 和四连杆 8 组纯外观节点 **PASS**；
- `parkour_dev` 50 步冒烟 **PASS**：`obs_dim=42`、`action_dim=12`；
- RL 和控制器源码未改，设备名、动作和观测维度不变；45/51 为源码常量静态确认。

**明确未验收/后续项：**

- `abd ±0.6 rad`、`hip/kn ±1.0 rad`、`0.15/0.15 m` 和四连杆比例仍是工程暂定值，待 CAD/实机确认；
- `mini_cheetah_controller`、`manual_control` 尚未同步新几何，不能作为自然步态验收；
- 旧 P1–P4 checkpoint 仅为历史资产，需使用新前缀重新训练；
- 45/51 维未做旧策略运行态回放。

## Tasks

- [x] T1: 编写 `tools/gen_yobogo_robot.py` 生成机器人段并替换世界内联段 — 12 关节 `endPoint==anchor`、总质量 10.5、包络约 485×275×300 — acceptance: 脚本断言通过，Webots 加载 0 ERROR (covers: S2.2)
- [x] T2: 相机节点 640×480 + 可调 `fov`/`pitch`（注释含推荐值与范围） — acceptance: 参数一眼可见可改，Webots 加载无错误 (covers: S2.3)
- [x] T3: 冒烟验证 0 ERROR + 控制器绑定 — acceptance: 12 电机/传感器就绪，`make` 通过可站立 (covers: S2.4, S2.5)
- [x] T4: 文档同步 `docs/features/webots-sim-guide.md` 增补参数来源表 — acceptance: 来源表与 S2.1 一致 (covers: S2.1)
- [x] T5: 按 S4 重构生成器与三个源 world — acceptance: 0.15/0.15 m、约120°屈膝、显式限位、足端碰撞、四连杆外观 (covers: S4.1–S4.4)
- [x] T6: 完成静态、world 一致性与 Webots 加载测试 — acceptance: 计数/锚点/限位/动力学通过，三世界 0 ERROR (covers: S4.5)
- [x] T7: 回填验收状态并将规格改为 `delivered` — acceptance: 代码实际值与文档一致，无两套冲突参数 (covers: S4.5)
