# Mini Cheetah / YoboGo 最终确认数据清单

> 状态：2026-10-07 最终来源裁决；四腿数组映射、最终整机质量口径
> `9 kg` 和项目关节力矩 `17/17/26 N·m` 已固化；URDF 资产已完成静态
> 合成与惯量正定验收；CAD 分件/挂载和机械限位仍为 P0，见第 6、8 节。

## 1. 来源优先级与禁用边界

1. `../../YoboGo-control/`：实机控制、接口、控制周期、安全和本地冲突的
   第一权威。
2. 网络上的 MIT 官方 Mini Cheetah 数据：YoboGo 未提供时，作为连杆长度、
   运动学、逐 link 质量/惯量、足端和执行器参考的第一来源。
3. `../model/mit-mini-cheetah-1.snapshot.2/`：唯一可信的 CAD 外形/碰撞
   几何来源，详见 [`INVENTORY.md`](../model/mit-mini-cheetah-1.snapshot.2/INVENTORY.md)。
4. `../model/visual_dae/` 下 4 个 DAE：唯一可信外观来源，仅用于 visual。
5. `../../official-mini-cheetah/` 与旧 Webots 除上述 4 个 DAE 外，全部
   禁用；其 URDF、world、质量、惯量、限位、碰撞、默认姿态和训练配置
   不得进入本清单或资产。

冲突时 YoboGo 覆盖网络；网络只填补 YoboGo 缺口。CAD 只能补几何，DAE
只能补外观。禁止从任一来源做未记录的比例缩放。

## 2. 最终采用数据

| 数据项 | 最终采用值 | 来源 | 使用规则 |
| --- | --- | --- | --- |
| 关节数与关节顺序 | 4 腿 x 3，向量内每腿 `abad/hip/knee` | YoboGo `HardwareBridge.cpp`、`rt_spi.cpp` | 实机语义权威 |
| 四腿编号映射 | `0=FR/RF`、`1=FL/LF`、`2=RR/RH`、`3=RL/LH` | YoboGo `RobotRunner.cpp`、`rt_spi.cpp`；MIT 官方 `getting_started.md`、`Quadruped.h` | 资产测试必须逐腿核对；禁止前后或左右镜像 |
| 侧别符号/零位 | `abad {-1,-1,1,1}`、`hip {-1,1,-1,1}`、`knee {±0.6429}`；`hip_offset` 与 `knee_offset` 见源码 | YoboGo `rt_spi.cpp` | 是实机编码器换算，不是 CAD 关节轴 |
| 初始目标 | `[-0.6,-1.0,2.7, 0.6,-1.0,2.7, -0.6,-1.0,2.7, 0.6,-1.0,2.7] rad` | YoboGo `initial_jpos_ctrl.yaml` | 权威；正式资产仍需确认安全限位 |
| 关节 PD | `Kp=[3,3,3]`、`Kd=[1,0.2,0.2]` | YoboGo `mc-mit-ctrl-user-parameters.yaml` | 实机默认权威；训练动作空间不能假定等价 |
| 控制周期 | `0.002 s = 500 Hz` | YoboGo `mini-cheetah-defaults.yaml`、`HardwareBridge.cpp` | 作为 decimation/物理步长约束 |
| LCM | `leg_control_command/data`、`state_estimator`、`spi_*`、`interface*`、`main_cheetah_visualization`、`hw_vectornav` | YoboGo `RobotRunner.cpp`、`HardwareBridge.cpp`、`macroConfig.h` | 部署接口权威 |
| TCP | 服务端口 `1986`，状态发送 `50 ms = 20 Hz` | YoboGo `rt_socket.h/cpp` | 接口权威；不是训练控制周期 |
| 关节力矩 | **项目最终值 `17/17/26 N·m`**；源码原注释为 `TODO CHECK WITH BEN`；说明书另有 `±18 N·m` | 用户选定；YoboGo `rt_spi.cpp`、`YoboGo-10S使用说明书(开源).docx` | 保留源码 TODO 作为证据，不改写源码；网络值不得覆盖项目值 |
| 最终整机质量口径 | **`9 kg`**；`RPC_inertia=[0.07,0.26,0.242]` | 用户选定；YoboGo `mc-mit-ctrl-user-parameters.yaml` | 项目总质量验收值为 `9 kg`；逐 link 分配仍须显式记录 |
| 连杆长度/运动学 | 髋安装 `x=±0.14775, y=±0.049 m`；大腿/小腿关节偏移 `0.2085/0.22 m` 量级；关节轴与四腿符号 | MIT ORCAgym URDF | 网络权威候选；与 CAD 对齐后才进入资产 |
| 逐 link 质量/惯量 | `base 3.3`；每腿 `hip 0.54`、`thigh 0.634`、`shank 0.064`、`foot 0.01 kg`；对应 URDF 惯量张量（foot 派生见第 8.2 节） | MIT ORCAgym `mini_cheetah_simple.urdf` | 网络权威候选；不与 `RPC_mass=9` 混算 |
| 足端/碰撞参考 | 足球半径 `0.0202 m`；机身盒、大小腿圆柱候选 | MIT ORCAgym URDF | 仅作碰撞设计参考，最终外形碰撞以 CAD 为准 |
| 执行器参考 | `HAA/HFE: effort 18, velocity 41`；`KFE: effort 28, velocity 26.8`；rotor armature/damping 见 rotor URDF | MIT ORCAgym URDF/生成脚本 | 参考能力，不覆盖 YoboGo 安全限幅 |
| DAE 外观 | `mini_abad/body/lower_link/upper_link.dae` 4 件 | `official-mini-cheetah` 源的已确认副本 | 仅外观；禁止携带其挂载/物理参数 |

### 2.1 MIT 网络权威数值（固定提交）

运动学单位为米/弧度；`RF/LF/RH/LH` 是 MIT 源文件命名，已按第 2.2 节
证据固定到 YoboGo 槽位：

| 项目 | `leg[0]` RF/FR | `leg[1]` LF/FL | `leg[2]` RH/RR | `leg[3]` LH/RL |
| --- | --- | --- | --- | --- |
| HAA 相对 base 原点 | `(0.14775,-0.049,0)` | `(0.14775,0.049,0)` | `(-0.14775,-0.049,0)` | `(-0.14775,0.049,0)` |
| HFE 相对 hip 原点 | `(0.055,-0.019,0)` | `(0.055,0.019,0)` | `(0.055,-0.019,0)` | `(0.055,0.019,0)` |
| KFE 相对 thigh 原点 | `(0,-0.049,-0.2085)` | `(0,0.049,-0.2085)` | `(0,-0.049,-0.2085)` | `(0,0.049,-0.2085)` |
| 足端相对 shank | `(0,0,-0.22)` | `(0,0,-0.22)` | `(0,0,-0.22)` | `(0,0,-0.22)` |

关节轴为 HAA 前腿 `+X`、后腿 `-X`（后腿父坐标绕 Y 旋转 `pi`），
HFE/KFE 局部轴均为 `(0,-1,0)`。逐 link 物理数据：

| link | 质量 kg | 质心（源 link 坐标） | 惯量 `ixx/ixy/ixz/iyy/iyz/izz` |
| --- | ---: | --- | --- |
| base | 3.3 | `(0,0,0)` | `0.011253/0/0/0.362030/0/0.042673` |
| hip | 0.54 | `(0.055,±0.036,0)` | `0.000381/0.000058/0.00000045/0.000560/0.00000095/0.000444` |
| thigh | 0.634 | `(0,±0.016,-0.02)` | `0.001983/0.000245/0.000013/0.002103/0.0000015/0.000408` |
| shank | 0.064 | `(0,0,-0.061)` | `0.000245/0/0/0.000248/0/0.000006` |
| foot | 0.01 | `(0,0,0)` | 源文件为全零，不能直接作为 PhysX 惯量；资产按第 8.2 节派生 |

simple URDF 逐 link 合计 `8.292 kg`。它与 YoboGo `RPC_mass=9 kg`
是两个口径，不自动合并。碰撞参考为 base 盒
`0.30 x 0.20 x 0.10 m`、thigh 圆柱 `L=0.17,r=0.015 m`、
shank 圆柱 `L=0.10,r=0.010 m`、足球 `r=0.0202 m`；hip 碰撞在源文件
中被注释。最终碰撞仍以 CAD 拆分为准。

执行器参考：HAA/HFE `effort=18 N·m, velocity=41 rad/s`，KFE
`effort=28 N·m, velocity=26.8 rad/s`；rotor URDF 另给
`armature=0.002268/0.002268/0.005484 kg·m²`、`damping=0.01 N·m·s/rad`，
生成脚本记录 rotor `0.055 kg`、`63e-6 kg·m²`、减速比 `6/6/9.33`。
这些是 MIT 能力参考，不覆盖用户选定的 YoboGo 项目限幅
`17/17/26 N·m`。

### 2.2 四腿物理命名、观察坐标系与图片推导

YoboGo 的 `RobotRunner.cpp` 用 `buildMiniCheetah<float>()` 构造
`_quadruped`，再把同一个 `_quadruped` 交给 `LegController`；其
`rt_spi.cpp` 的左右侧换算符号为
`hip {-1,+1,-1,+1}`。YoboGo 仓库片段没有单独画出前后腿图，因此前后
信息按来源优先级用 MIT 官方网络接口文档补齐：

- [getting_started.md 固定提交](https://github.com/mit-biomimetics/Cheetah-Software/blob/c71c5a138d3e418cc833e94e25357ceea8955daa/documentation/getting_started.md#L77-L84)
  明确机体系 `+X` 向前、`+Y` 向左、`+Z` 向上，俯视腿序为
  `FRONT / 1 0 / 3 2 / BACK`，右侧一列是 `0/2`；
- [Quadruped.h 固定提交](https://github.com/mit-biomimetics/Cheetah-Software/blob/c71c5a138d3e418cc833e94e25357ceea8955daa/common/include/Dynamics/Quadruped.h#L80-L100)
  给出 `sideSigns={-1,+1,-1,+1}`，并规定 `leg 0/1` 的 `+X`、
  `leg 2/3` 的 `-X`，`leg 1/3` 的 `+Y`。

由此得到唯一数组映射：

| 槽位 | 机体系髋关节象限 | 物理命名 |
| --- | --- | --- |
| `leg[0]` | `x>0, y<0` | 右前 `FR`（也记 `RF`） |
| `leg[1]` | `x>0, y>0` | 左前 `FL`（也记 `LF`） |
| `leg[2]` | `x<0, y<0` | 右后 `RR`（也记 `RH`） |
| `leg[3]` | `x<0, y>0` | 左后 `RL`（也记 `LH`） |

用户图片的观察结论：

1. 图中机身长轴从屏幕左上端延伸到右下端，红色按钮位于机身长侧面并
   明显靠近屏幕左端；用户确认“有按钮的地方是后侧”，所以**屏幕左端
   = 后端，屏幕右端 = 前端**。这确定了前后方向，不产生前后镜像。
2. 图片没有 CAD 坐标三轴标记，单凭透视无法唯一判断“屏幕近侧面”是
   机体系 `+Y` 还是 `-Y`。因此不得把图片的近/远侧四象限写成最终
   槽位。两个候选分别为：
   - 近侧面是右侧 `-Y`：左近 `leg[2]`、左远 `leg[3]`、右近
     `leg[0]`、右远 `leg[1]`；
   - 近侧面是左侧 `+Y`：左近 `leg[3]`、左远 `leg[2]`、右近
     `leg[1]`、右远 `leg[0]`。
3. 该图片歧义**不阻塞数组到物理腿的最终映射**；解除 CAD 截图象限
   标注的最小动作是在 CAD 查看器打开坐标三轴并确认 `+Y`，或用
   单腿小位移点动核对一个槽位。正式资产仍以上表四腿映射为准。

## 3. YoboGo 本地事实与冲突

- `target_jpos` 的 `knee=2.7 rad` 接近网络参考上界，但 YoboGo 没有完整
  机械限位；不能据此证明安全。
- 力矩源码 `17/17/26` 带 `TODO CHECK WITH BEN`，且与说明书 `±18`
  存在证据差异；用户已明确选择 `17/17/26 N·m` 作为**本项目值**。
  该选择不改写 YoboGo 源码，网络 `18/18/28` 仍不得覆盖项目值。
- 用户已选择最终整机质量口径 `9 kg`。它是项目总质量验收值，不是
  逐 link 数值；网络 `8.292 kg` 和说明书约 `10.5 kg` 只保留为来源
  证据，逐 link 分配必须显式记录且合计为 `9 kg`。
- 四腿数组已固定为 `0=FR、1=FL、2=RR、3=RL`；资产仍必须输出
  “数组槽位 -> URDF joint -> 实机侧别”的静态测试，防止装配时镜像。
- TCP `1986` 的 `50 ms` 是状态发送周期；LCM/SPI/控制循环的 `2 ms`
  才是 500 Hz 控制周期，二者不得混用。

## 4. 网络 MIT 官方来源与许可证

主要数值来源固定到 ORCAgym 主分支提交
[`5aaff694ae5f1c31e08040d59787f2cca4c5cfe0`](https://github.com/mit-biomimetics/ORCAgym/tree/5aaff694ae5f1c31e08040d59787f2cca4c5cfe0)：

- [mini_cheetah_simple.urdf](https://github.com/mit-biomimetics/ORCAgym/blob/5aaff694ae5f1c31e08040d59787f2cca4c5cfe0/resources/robots/mini_cheetah/urdf/mini_cheetah_simple.urdf)
  提供运动学、逐 link 质量/惯量、碰撞和足端；
- [mini_cheetah_rotor.urdf](https://github.com/mit-biomimetics/ORCAgym/blob/5aaff694ae5f1c31e08040d59787f2cca4c5cfe0/resources/robots/mini_cheetah/urdf/mini_cheetah_rotor.urdf)
  提供 rotor armature/damping；
- [generate_urdfs_mc.sh](https://github.com/mit-biomimetics/ORCAgym/blob/5aaff694ae5f1c31e08040d59787f2cca4c5cfe0/resources/robots/mini_cheetah/urdf/generate_urdfs_mc.sh)
  记录 rotor mass、转动惯量、减速比和阻尼推导；
- [evaluateURDFMiniCheetah.m](https://github.com/mit-biomimetics/ORCAgym/blob/5aaff694ae5f1c31e08040d59787f2cca4c5cfe0/resources/robots/mini_cheetah/urdf/evaluateURDFMiniCheetah.m)
  给出默认验证姿态 `[0,-0.785398,1.596976]` 量级；
- [LICENSE](https://github.com/mit-biomimetics/ORCAgym/blob/5aaff694ae5f1c31e08040d59787f2cca4c5cfe0/LICENSE)
  为 BSD 3-Clause 文本（版权头为 ETH Zurich/NVIDIA）。

MIT 官方 [Cheetah-Software](https://github.com/mit-biomimetics/Cheetah-Software)
仅作控制接口交叉核对；其许可证为 MIT。本工作区不复制外部 URDF、USD、
mesh 或策略文件，只登记本清单中的数值和 URL。

## 5. CAD 可用范围

CAD 目录含 1 个 STEP AP203（毫米、128 产品、936 装配实例、60 实体
B-rep）和 1 个 Parasolid 文本文件（单位待导入确认、含密度属性）。
STEP 没有质量/惯量/关节/工程图；Parasolid 没有可直接使用的逐 link
质量、惯量或机器人语义坐标系。两文件都没有执行脚本。

因此 CAD 能提取并拆分**可信外形/碰撞几何**，但不能提供动力学或控制
事实。Parasolid 的密度不能冒充质量；其单位和 CAD 原点必须在 CAD 内核
导入报告中确认。

## 6. 最终缺口与 P0 阻塞

| 优先级 | 缺口 | 解除条件 |
| --- | --- | --- |
| P0 | CAD 装配到 `base/hip/thigh/shank/foot`、四腿和 DAE 挂载的坐标/单位映射 | 人工 CAD 分件、轴/原点对照并生成导入报告 |
| P0 | 实机关节位置/速度机械限位 | 用户/实测确认；网络值不能自动替代 |
| P0 | 碰撞简化、足端接触、DAE visual 挂载 | CAD 几何 + 用户确认后静态验收 |
| P1 | 执行器延迟、关节干摩擦、足端材料/摩擦、IMU 位姿与噪声、TCP/LCM 端到端延迟、关节反馈滤波 | 按 [`2026-10-07-p1-parameters.md`](./2026-10-07-p1-parameters.md) 的待实测清单完成后才可进入 DR |
| P1 | Isaac Lab 资产导入、PhysX、环境锁和 1000 步冒烟 | 通过 `training/README.md` 阶段门 |

在 P0 全部关闭前，只能做数据整理和静态导入，不得启动正式 PPO 或实机
部署。

已关闭：四腿数组映射、最终整机质量口径 `9 kg`、项目关节力矩
`17/17/26 N·m`、`9 kg` 逐 link 统一缩放质量分配、4 个 foot 的 PhysX
正定派生惯量。图片近/远侧的 CAD 象限标注是装配核对项，不改变已固定
的数组映射。

## 7. P1 调研结果索引

完整证据、真实 URL、原文数值、单位和可信等级见
[`2026-10-07-p1-parameters.md`](./2026-10-07-p1-parameters.md)。本轮仅允许
更新该文件和本清单，不改根 `docs/`、`training/README.md` 或环境安装文件。

可直接登记的网络/本地值：

| 数据 | 值 | 类型 |
| --- | --- | --- |
| MIT 软件 softstop | abad `±1.5 rad`、hip `±5.0 rad`、knee 无 | 软限位参考，不是机械 stop |
| 足端碰撞候选 | ORCAgym `0.0202 m`；Rapid Locomotion `0.0175 m` | 冲突候选，CAD 待核 |
| 本地仿真地面 | ground `mu=0.5`、restitution `0`；mesh/box `mu=0.7` | 仿真参数 |
| 速度参考 | HAA/HFE `41 rad/s`、KFE `26.8 rad/s` | 网络 URDF 能力参考 |
| MIT 关节模型摩擦 | dry friction `0.2 N·m`、damping `0.01 N·m·s/rad` | MIT 模型参考 |
| 本地 IMU 仿真噪声 | accel `0.005`、gyro `0.005`、quat `0.003` | 本地仿真参数，单位待确认 |
| 通信周期 | control/SPI/LCM `2 ms`；TCP 状态发送 `50 ms` | 周期，不是端到端延迟 |

明确未找到，不得填入猜测值：

- hip/abad/knee 机械硬限位与机械 stop 角度；
- 足端材料牌号、本构参数与实测足地摩擦系数；
- 执行器闭环延迟；
- YoboGo 实机关节速度硬饱和；
- YoboGo 实机 IMU 安装位姿；
- TCP/LCM 端到端延迟与抖动；
- 关节编码器反馈滤波参数。

## 8. URDF 资产派生说明与静态验收

### 8.1 产物与固定契约

资产目录为 `../model/yobogo_mini_cheetah_v1/`：

- `yobogo_mini_cheetah.urdf`：正式静态 URDF；
- `generate_urdf.py`：按本清单固定数值生成 URDF；
- `validate_urdf.py`：结构、动力学、控制元数据、网格和来源边界验收。

计划 USD 路径为
`../model/yobogo_mini_cheetah_v1/yobogo_mini_cheetah.usd`；本轮资产子任务
只生成和验收 URDF，不伪造 USD。

命名固定为 `base_link` 和 `legN_hip_link / legN_thigh_link /
legN_shank_link / legN_foot_link`。12 个驱动关节按数组顺序为
`leg0_abad_joint, leg0_hip_joint, leg0_knee_joint, ... leg3_knee_joint`，
槽位映射继续使用 `leg0=FR、leg1=FL、leg2=RR、leg3=RL`；每腿另有
`legN_shank_to_foot_joint` fixed joint。因此资产为 `17 links`、
`16 joints = 12 revolute + 4 fixed`。

### 8.2 质量与惯量缩放

动力学沿用第 2.1 节 ORCAgym 固定提交的原始值，未从旧 URDF 复制。
MIT simple URDF 合计 `8.292 kg`，项目验收值为 `9 kg`，所以所有质量和
惯量分量统一乘：

```text
scale = 9 / 8.292 = 1.085383502170767
```

| link 类型 | 原始质量 kg | URDF 质量 kg | 数量 |
| --- | ---: | ---: | ---: |
| base | 3.300 | 3.581765557164 | 1 |
| hip | 0.540 | 0.586107091172 | 4 |
| thigh | 0.634 | 0.688133140376 | 4 |
| shank | 0.064 | 0.069464544139 | 4 |
| foot | 0.010 | 0.010853835022 | 4 |
| **合计** | **8.292** | **9.000000000000** | **17** |

非足端惯量按同一比例逐分量缩放，静态检查通过。4 个 foot 的源惯量
张量均为全零，不能直接作为 PhysX 惯量；资产使用可信的缩放后 foot
质量与 MIT 保守碰撞球半径，按实心球公式派生：

```text
m = 0.01 * 9 / 8.292 = 0.0108538350217 kg
r = 0.0202 m
I = 2/5 * m * r^2 = 1.7715195369e-06 kg·m^2
ixx = iyy = izz = I
ixy = ixz = iyz = 0
```

这是为满足 PhysX 正定性、由可信质量和 MIT 碰撞半径推导的近似，
**不是 MIT 原始惯量，也不是猜测的 `epsilon`**。生成器、URDF
metadata 和验收器都明确记录该公式、输入值、结果值及来源边界；4 个
foot 的惯量特征值均严格大于 0。

### 8.3 visual、碰撞、限位和控制元数据

URDF 的 `13` 个 mesh 实例只引用 `../visual_dae/` 下的 4 个 DAE，SHA256
与工作副本一致。body 原点对齐；abad 按 `+Y` 外伸、upper 按 `-X` 长轴、
lower 按 `+Z` 长轴做轴对齐旋转。该挂载仍是 DAE 几何与运动学的静态
候选，不是 CAD 分件结果，待 USD/截图核对。

碰撞主动启用 ORCAgym 保守候选：base box
`0.30 x 0.20 x 0.10 m`、hip cylinder `L=0.025,r=0.05 m`、thigh
cylinder `L=0.17,r=0.015 m`、shank cylinder `L=0.10,r=0.010 m`、
foot sphere `r=0.0202 m`。hip 尺寸取自源文件注释掉的候选块；CAD 尚未
分件，这些几何不能当作实机 CAD 事实。

`<limit>` 写入 abad `±1.5 rad`、hip `±5.0 rad` 和 knee
`[-2.7,2.7] rad`。abad/hip 明确标为 MIT 软件 softstop 参考；knee 只是
本地初始/安全候选；机械 hard stop 仍未知。effort 固定为项目值
`17/17/26 N·m`，velocity `41/41/26.8 rad/s` 只保留为 ORCAgym 能力参考。

URDF 没有稳定的 PD/周期/初始目标字段，因此生成器把这些事实写成结构化
XML 注释并由验收器检查。2026-10-07 起明确拆分两类姿态：

- 实机 `target_jpos`：`FR/RR=[-0.6,-1.0,2.7]`、
  `FL/RL=[0.6,-1.0,2.7] rad`，仅表示 YoboGo 控制侧逻辑坐标，
  下游 `rt_spi` 还执行逐腿符号/零偏换算；
- URDF/Isaac `SIM_INITIAL_JOINT_POS`：每腿
  `[0,-0.785398163,1.865468294] rad`，root 出生高度 `0.26 m`；
  这是保持 ORCAgym axis/rpy 不变的几何站姿。

`Kp=[3,3,3]`、`Kd=[1,0.2,0.2]`、周期 `0.002 s = 500 Hz` 不变。
4 个 foot visual 与 collision origin 均为 `(0,0,0.024) m`。
该 origin 位于 foot link 局部坐标系，计算 world 碰撞球心时须先乘 foot
姿态旋转；当前 SIM 下其 world z 贡献约为 `+0.01131 m`，所以
`foot_link_z=-0.251110396 m` 对应球底 `-0.260000000 m`，而不是把
`+0.024` 直接当世界竖直偏移得到的 `-0.247310396 m`。
消费方不得把 URDF `<limit>` 的 effort 或 velocity 当成控制周期/PD，
也不得把实机 `target_jpos` 当成 URDF 几何初态。

### 8.4 检查命令与退出码

| 命令 | 退出码 | 结果 |
| --- | ---: | --- |
| `python3 -m py_compile generate_urdf.py validate_urdf.py` | `0` | 脚本语法通过 |
| `python3 generate_urdf.py` | `0` | URDF 可重复生成 |
| `python3 validate_urdf.py` | `0` | `34` 项通过、`0` 项警告、`0` 项失败 |
| `python3 validate_urdf.py --strict-inertia` | `0` | `34` 项通过、`0` 项警告、`0` 项失败；4 个 foot 严格正定 |

2026-10-07 拆分修复后的复测命令与退出码：

| 命令 | 退出码 | 结果 |
| --- | ---: | --- |
| `python3 -m py_compile generate_urdf.py validate_urdf.py ...` | `0` | 6 个修复相关 Python 文件语法通过 |
| `python3 generate_urdf.py` | `0` | target/SIM/root/foot origin metadata 重新生成 |
| `python3 validate_urdf.py` | `0` | `39` 项通过、`0` 项警告、`0` 项失败 |
| `python3 validate_urdf.py --strict` | `0` | 同样 `39` 项通过；兼容 `--strict-inertia` |
| `python3 training/scripts/static_check.py` | `0` | `STATIC_CHECK_OK files=11` |

默认与严格验收新增：实机 target/SIM 拆分、root `0.26 m`、
foot visual/collision 对齐、独立 URDF FK 的
`foot_collision_min≈-0.260000 m` 和
`base_collision_min=-0.040000 m`。

默认验收覆盖 XML、命名/顺序、拓扑、12 驱动关节、逐 link 质量和惯量、
`9 kg` 合计、foot 派生元数据与 4 个惯量特征值严格大于 0、四腿运动学、
限位/力矩/速度、初始姿态、PD/500 Hz、4 个网格路径与 SHA256、碰撞、
腿映射、来源边界、机械限位边界和 CAD 边界。`--strict-inertia`
明确执行同一严格正定门禁。

### 8.5 来源边界与剩余状态

URDF 动力学/运动学/碰撞只登记 ORCAgym 固定提交
`5aaff694ae5f1c31e08040d59787f2cca4c5cfe0` 的数值；控制事实来自
YoboGo；softstop 只标 MIT 软件参考；visual 只引用 4 个本地 DAE。
资产内没有旧 Webots/`official-mini-cheetah` 的 URDF、USD、world、
质量、惯量、限位、碰撞、默认姿态或训练配置。

本轮关闭了“`9 kg` 总质量到逐 link 质量的显式分配”和“URDF 静态合成”。
本轮同时以有记录的实心球近似关闭了 foot 惯量正定性。

2026-10-07 拆分修复后已完成 URDF→USD、任务注册与单环境 Gate：

- `TERM=xterm-256color` 转换退出码 `0`；
- `list_envs.py` 退出码 `0`；
- `check_env.py --mode all` 退出码 `0`，
  `gate.json status=pass`、`metrics.json check_count=1657`；
- 实测 reset `root_z=0.2599999905 m`、
  `foot_collision_min=-0.2600000799 m`、
  `base_collision_min=-0.0400000066 m`，站立四足有效接触比例门通过；
- 证据：`../training/runs/env-check/2026-10-07_225234_repair/`。

正式 PPO 已进入用户级 systemd 后台：unit
`yobogo-ppo-20261007233458`、PID `2064669`，从 `model_1800.pt` 续训，
后台验证快照 `1847/4800`，累计 40 个 checkpoint，TensorBoard event
已生成，日志 OOM/CUDA error/Traceback/Killed/NaN/Inf 扫描数为 `0`。
后台运行和训练规划详见 `../training/README.md` 第 7.6 节。

仍未关闭：CAD 分件及 DAE/碰撞坐标确认、实机机械限位、正式 PPO 完成、
独立速度/跌倒率/动作饱和度验收。
