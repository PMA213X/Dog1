---
feature: msl-match-field
status: delivered
updated: 2026-09-25
branch: master
commits: 9a75b2b..worktree
---

# MSL 比赛场地 Webots 场景

## Report

**What was built** — 新增 `webots-sim/worlds/msl_match.wbt`（815 行）：按 MSL Rulebook 2025 v26.0（`docs/features/robocup-midsize-rules.md`）构建的 22 m × 14 m 正式比赛场地场景。含绿色地毯与全套白色标线（边线/球门线/中线/中圈 r=2 m/中心点/球门区 0.75 m×3.94 m/罚球区 2.25 m×6.94 m/罚球点 3.6 m/角弧 r=0.75 m）、双方球门（内距 2.44 m × 横梁下沿 2.0 m × 深 0.5 m + 球网）、FIFA 5 号球（⌀0.22 m / 0.43 kg / 橙色）、黑色安全挡板 24×16 m 围合（高 0.10 m，距线 ≥1 m）、6 根旗杆、双方向光。几何封装为 4 个 PROTO（MslField 1443 行 / MslGoal 611 行 / MslBall 44 行 / MslEnvironment 263 行），world 以 `EXTERNPROTO "../protos/..."` 引用。复用既有 mini_cheetah 机器人与 `mini_cheetah_controller`（PD 站立 + trot，键盘 S/T/R），出生位 `8 0 0.38`、绕 Z 轴 180° 面向场地中心。

**Review** — 独立评审初判 FAIL（机器人 endPoint 平移语义 + visual/bbox 错位 2 个 critical），按「连杆原点在关节处」约定重生成机器人段后复审 **PASS**；标线/球门/球/边界几何由两位评审脚本断言全数达标。

**Verification** — `make` 控制器编译通过；`webots --batch --minimize --stderr --stdout webots-sim/worlds/msl_match.wbt` 加载 **0 ERROR、0 Motor/Sensor not found**，12 电机 + 12 传感器全部就绪，机器人稳定站立（IMU 显示 roll≈-180° 系 yaw=180° 时欧拉角奇异点，非真实翻转）。PROTO 几何由实现子代理脚本断言（标线数量/尺寸链 2.44/2.0/0.5/0.43 等）。

**Journey log**
1. 资料夹 PDF 是「中型机器狗循迹任务赛」调试说明，与 `docs/features/robocup-midsize-rules.md` 的 MSL 足球规则是两套场景；用户明确选择 MSL 足球场 + 仅 world 范围。
2. 原 `mini_cheetah.wbt` 机器人段用的是 Gazebo 风格语法（`child Solid`/`stopSpringDamper`/`frictionMaterial`/裸 PBRAppearance），R2025a 会静默丢弃整棵关节树导致电机找不到；新 world 已改为 `endPoint` + `Shape` 包裹 + 2 行 `inertiaMatrix` + 腿部 boundingObject，旧 world 保持不动。
3. `rotation 0 1 0 π` 会把机器人/球门上下颠倒（Z 轴翻成 -Z）；面向 -X 必须用 `rotation 0 0 1 π`。
4. Webots 的 `DirectionalLight` 不能放在 Group/Pose 的 children 里，只能挂在 world 顶层。
5. `timeout` 杀 Webots 进程会被误报为「控制器崩溃」；判定崩溃要看退出前是否有真实 segfault/ERROR。
6. 独立评审抓出 2 个 critical：Webots R2025a 的 `endPoint Solid.translation` 相对**父坐标系**（非 joint anchor），照抄 baseline 的 `child` 偏移会让四条腿堆在机体质心；大腿/小腿 visual 与 boundingObject 必须用同一 Transform 偏移。已按「连杆原点在关节处」约定重生成机器人段并复核：腿挂载 x=±0.19、visual/bbox 一致。
7. `InertialUnit.coordinateSystem` 在 R2025a 中不存在（原 world 同样报 ERROR），不能通过加回该字段恢复 NUE；IMU 打印的 roll≈−180° 是 yaw=180° 时欧拉角奇异 + 坐标系默认值差异，PD/trot 不用 IMU 作反馈，不影响站立。

## [S1] Problem

`webots-sim/worlds/mini_cheetah.wbt` 目前只是「单台四足 + 10×10 m 方格地板」的步态试验台，缺少任何 MSL（RoboCup Middle Size League）比赛元素：场地尺寸/地毯、标线、球门、球、安全边界、旗杆、灯光。需要按 MSL Rulebook 2025 v26.0（2026 年沿用，见 `docs/features/robocup-midsize-rules.md`）把 Webots 世界改造成合规比赛场景，用于赛前场地可视化与算法验证。

用户决策（2026-09-25）：
- 场景目标 = **MSL 足球场**（不是循迹任务赛道）；
- 范围 = **仅 world 场景**（控制器/视觉/通信不动）；
- 场地尺寸 = **22 m × 14 m 正式赛场**；
- 工作区 = **直接在 master 上改**（用户显式选择，覆盖 compose-next 默认 worktree）。

## [S2] Design

### S2.1 坐标系与总体布局

沿用 Webots/机器人惯例：**X=前，Y=左，Z=上**。场地中心在原点。

| 量 | 值 |
|---|---|
| 场地长（沿 X） | 22 m，x ∈ [-11, +11] |
| 场地宽（沿 Y） | 14 m，y ∈ [-7, +7] |
| 地面 | z = 0 |
| 球门位置 | x = ±11（球门线），开口朝向场地内侧 |
| 中心 | 原点 (0,0) |

新建世界文件 `webots-sim/worlds/msl_match.wbt`（不删除 `mini_cheetah.wbt` 步态试验台）。机器人沿用现有内联 `mini_cheetah` 模型与 `mini_cheetah_controller` 绑定，出生位姿置于开球点附近（如 translation `8 0 0.38`，rotation 朝 -X）。

### S2.2 场地与标线（MslField.proto）

- **地毯**：22 × 14 m 绿色平面（建议 Box 22×14×0.02，顶面 z=0，绿色非反光材质，roughness 高、metalness 0）。
- **标线**：白色，宽 **0.125 m**，自**外沿**测量；线用薄 Box/Plane 贴在地毯上（顶面 z=0.001 避免 z-fighting）。
- 标线集合（全部以规则为准）：
  - 边线（touchline）：y = ±7（外沿），沿 X 全长 22 m
  - 球门线（goal line）：x = ±11（外沿），沿 Y 全长 14 m
  - 中线（halfway line）：x = 0，沿 Y 全长 14 m
  - 中心点（center spot）：⌀ 0.15 m 圆盘于原点
  - 中圈（center circle）：**r = 2 m** 圆环（可用 IndexedLineSet/圆环薄带近似，或 32~64 段薄 Box 拼弧）
  - 球门区（goal area）：纵深 **0.75 m**，宽 = 门宽 2.44 + 两侧各 0.75 = **3.94 m**
  - 罚球区（penalty area）：纵深 **2.25 m**，宽 = 2.44 + 两侧各 2.25 = **6.94 m**
  - 罚球点（penalty mark）：⌀ 0.15 m，距球门线中点 **3.6 m**
  - 角球弧（corner arc）：**r = 0.75 m** 四分之一圆，四角各一

### S2.3 球门（MslGoal.proto）

| 项目 | 规格 |
|---|---|
| 立柱内距 | **2.44 m** |
| 横梁下沿高度 | **2.0 m** |
| 立柱/横梁截面 | ≤ 0.12 m（取 0.12 m 或 0.10 m 方柱） |
| 球门深度 | ≥ 0.5 m（取 0.5 m） |
| 颜色 | 白色 PBRAppearance |
| 球网 | 延伸至安全边界，底部覆盖 30–40 cm；用细杆网格或半透明面片近似 |

PROTO 以球门中心在 +X 球门线为基准建模，实例化两次：`(11 0 0)` 与 `(-11 0 0)`（后者 rotation `0 0 1 π`）。

### S2.4 球（MslBall.proto）

| 项目 | 规格 |
|---|---|
| 类型 | FIFA 5 号 |
| 周长 | 68–70 cm → 直径 0.2165–0.2228 m，取 **⌀ 0.22 m**（radius 0.11） |
| 质量 | **0.410–0.450 kg**，取 0.43 kg |
| 颜色 | **避免黑/白/绿主色** — 用橙红主色 + 白色瓣（或纯橙） |
| 物理 | Sphere + PBRAppearance；密度按质量/体积设置 |

球初始放在中心点上（translation `0 0 0.11`）。

### S2.5 安全边界、旗杆、灯光（MslEnvironment.proto 或并入 world）

- **安全边界**：黑色挡板，高 **0.08–0.15 m**（取 0.10 m），距边线/球门线 **≥ 1 m**。即围合矩形 24 × 16 m（x ∈ [-12,12], y ∈ [-8,8]）四边各一条挡板，厚度 0.05 m。
- **旗杆**：场地四角 + 半场线与边线交点（共 6 根），高度约 1.5 m，杆 + 小旗面。
- **灯光**：至少 1 个 DirectionalLight（或 2–4 个 SpotLight）确保场地可辨；Background 保持简洁。

### S2.6 世界文件装配（msl_match.wbt）

节点顺序建议：WorldInfo（basicTimeStep 4，与控制器 250 Hz 一致）→ Viewpoint → Background → DirectionalLight → MslField → MslGoal×2 → MslBall → 挡板/旗杆 → 既有 Robot（mini_cheetah）。

### S2.7 验收与测试边界

- 世界文件可被 Webots R2025a 打开，无 PROTO 语法错误（无 `WARNING: ... unknown field` / `error`）。
- 控制器仍能编译并绑定运行（`make` 通过；按 S/T/R 可站立/trot）。
- 视觉检查：场地/标线/球门/球/挡板/旗杆齐全且尺寸正确（可从 Viewpoint 俯视核对）。
- 不引入控制器代码变更。

## [S3] Out of Scope

- 控制器视觉、颜色识别、循迹状态机、LCM/UDP 通信（键盘 S/T/R 与 PD/trot 保持原样）。
- 多机器人、对手、裁判盒/RefBox、比赛状态机。
- 机器人外形按 MSL 52×52×40–80 cm 校形（现投影约 0.38×0.22 m，宽度偏小，**仅记录不改**）。
- 本地 18×12 m 场地变体。
- URDF/网格加载（沿用盒模型）。

## Tasks

- [x] T1: 编写 `webots-sim/protos/MslField.proto` — 绿毯 22×14 + 全套白线（边线/球门线/中线/中圈/中心点/球门区/罚球区/罚球点/角弧）— acceptance: PROTO 可被 world 实例化，几何尺寸符合 S2.2 表 (covers: S2.2)
- [x] T2: 编写 `webots-sim/protos/MslGoal.proto` — 2.44×2.0 m 白门 + 球网 + 0.5 m 深 — acceptance: 单 PROTO 两次实例化后双门对称，尺寸符合 S2.3 (covers: S2.3)
- [x] T3: 编写 `webots-sim/protos/MslBall.proto` — ⌀0.22 m / 0.43 kg / 非黑白绿配色 — acceptance: 球落地稳定，质量尺寸符合 S2.4 (covers: S2.4)
- [x] T4: 安全挡板 + 6 旗杆 + 灯光 — acceptance: 挡板 24×16 围合、旗杆 6 根、场地照明可见 (covers: S2.5)
- [x] T5: 组装 `webots-sim/worlds/msl_match.wbt`，复用现有 mini_cheetah 机器人与控制器 — acceptance: Webots 打开无错误，控制器 `make` 通过并可站立/trot，俯视图元素齐全 (covers: S2.1, S2.6, S2.7)
- [x] T6: 更新文档 — `docs/compose/spec/msl-match-field.md` Report + `docs/features/webots-sim-guide.md` 增补 MSL 场景章节 — acceptance: 文档描述与最终 world 一致 (covers: S2.7)
