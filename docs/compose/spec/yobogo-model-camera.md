---
feature: yobogo-model-camera
status: delivered
updated: 2026-09-25
branch: master
---

# YoboGo-10S 真实模型与相机 Webots 化

## Report

**What was built** — `webots-sim/worlds/msl_match.wbt` 中的 MiniCheetah 盒模型（8.85 kg / 无相机）已替换为 **YoboGo-10S 实机规格模型**（`yobogo_10s`）：按 **Scheme A 包络缩放**重建 12 DOF 运动学链（说明书 **10.5 kg / 485×275×300 mm**），站立高与出生 z 对齐 yaml `des_p` **0.26**，机身惯量直接取 **RPC_inertia** [0.07, 0.26, 0.242]，电机 `maxTorque` 对齐 CAN 协议 ±18 N·m。几何由 `webots-sim/tools/gen_yobogo_robot.py` 作为**单一真源**生成（常量带 `[说明书]/[yaml]/[估算]` 来源标注 + 内置自检），world 内机器人段可一键重生成覆盖；`mini_cheetah.wbt` 旧盒模型保留不动。腿段（大腿 **0.14** / 小腿 **0.12** m）等 C 级缺失项按包络缩放**估算**，不编造实测值。

相机方面新增前置 `front_camera`（**640×480**），`fieldOfView` 与俯仰 pitch 为**一眼可见可改**的显式参数（推荐 1.05 rad / −35°，注释标明范围 0.96–1.22 rad / −30°~−40°），改动生成脚本顶部 `CAMERA_*` 常量或 world 字段后重新生成即可。质量合计 3.92 + 4×1.644 = **10.496** kg（≈ A 级 10.5）。溯源分级见 S2.1：A = 说明书原文，B = 官方 yaml / 代码（`des_p`、`RPC_inertia`、Kp/Kd、`main.cpp` 视觉参数），C = 包络缩放估算（腿段 0.14/0.12 等）。

**Verification** — `python3 webots-sim/tools/gen_yobogo_robot.py` 内置断言 **PASS**（总质量 10.496、12 关节 `endPoint==anchor`、`fieldOfView` 字段名正确）；`webots --batch --minimize --stderr --stdout webots-sim/worlds/msl_match.wbt` 冒烟加载 **0 ERROR、0 Motor/Sensor not found**，12 电机 + 12 传感器全部就绪，控制器启动后各关节角稳定在 **≈0**（站立不抖动）。质量合计 3.92 + 4×1.644 = **10.496** kg（`<0.02` 容差对齐 10.5）；包络 ≈ 485×275×300 mm 由脚本几何常量约束。

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
| pitch | **-35°（≈ -0.61 rad）** | 推断：视觉仅用上半图 rows 0–149（`main.cpp:168`）→ 画面顶行必须看到地面，故下俯；**易调** |
| translation | `0.20 0 0.055`（前上方） | 推断：机身长 485 mm / 站立高 0.30（`main.cpp:31`）→ 前上位置 |

**易调要求**：`fov` 与 `pitch` 必须写成一眼可见可改的显式参数（生成脚本顶部常量 / 世界文件内带注释字段），注释标明：

- `fov` 推荐 1.05 rad，范围 **0.96–1.22 rad**（≈55°–70°）；
- `pitch` 推荐 -0.61 rad（-35°），范围 **-0.52 ~ -0.70 rad**（≈ -30° ~ -40°）。

其余（translation / 分辨率）同为显式参数，但优先级低于 fov/pitch。

### S2.4 性能对齐（对照官方代码）

| 项 | 官方值 | 现 Webots 值 | 处置 |
|---|---|---|---|
| 关节 PD | Kp=[3,3,3], Kd=[1,0.2,0.2]（yaml:5,4） | 控制器 `KP[3]={3,3,3}`, `KD[3]={1,0.2,0.2}`（`mini_cheetah_controller.cpp:40-41`） | **已一致，不动** |
| 电机力矩上限 | 协议 ±18 N·m（说明书 §六） | `maxTorque 20.0`（`msl_match.wbt` 各 Motor 节点） | **20 → 18** |
| 站立高度 | des_p z = 0.26（yaml:87） | 出生 z = 0.38（`msl_match.wbt:70`） | **0.38 → 0.26** |

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

## Tasks

- [x] T1: 编写 `tools/gen_yobogo_robot.py` 生成机器人段并替换世界内联段 — 12 关节 `endPoint==anchor`、总质量 10.5、包络约 485×275×300 — acceptance: 脚本断言通过，Webots 加载 0 ERROR (covers: S2.2)
- [x] T2: 相机节点 640×480 + 可调 `fov`/`pitch`（注释含推荐值与范围） — acceptance: 参数一眼可见可改，Webots 加载无错误 (covers: S2.3)
- [x] T3: 冒烟验证 0 ERROR + 控制器绑定 — acceptance: 12 电机/传感器就绪，`make` 通过可站立 (covers: S2.4, S2.5)
- [x] T4: 文档同步 `docs/features/webots-sim-guide.md` 增补参数来源表 — acceptance: 来源表与 S2.1 一致 (covers: S2.1)
