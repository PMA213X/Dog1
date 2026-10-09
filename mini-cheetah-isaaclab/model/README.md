# 模型资产

## 当前产物

- URDF：`yobogo_mini_cheetah_v1/yobogo_mini_cheetah.urdf`
- 可复现生成器：`yobogo_mini_cheetah_v1/generate_urdf.py`
- 静态验收器：`yobogo_mini_cheetah_v1/validate_urdf.py`
- 计划 USD：`yobogo_mini_cheetah_v1/yobogo_mini_cheetah.usd`

本轮只生成并验收 URDF；USD 由训练环境子任务生成，不在本目录脚本中伪造。

URDF 契约固定为：

- `17` 个 link：`base_link`，以及每腿
  `legN_hip_link / legN_thigh_link / legN_shank_link / legN_foot_link`。
- `16` 个 joint：`12` 个 revolute 驱动关节和 `4` 个
  `shank_to_foot` fixed joint。
- 驱动关节按数组顺序：
  `leg0_abad_joint, leg0_hip_joint, leg0_knee_joint, ... leg3_knee_joint`。
- 腿槽位固定为 `leg0=FR、leg1=FL、leg2=RR、leg3=RL`，每腿顺序为
  `abad → hip → knee`。

## 外观权威来源

权威外观且已确认正确的文件只有以下 4 个 DAE：

| 文件 | SHA256 |
| --- | --- |
| `mini_abad.dae` | `6a34448654e342b6b71e71a1ca263e327fba8f4f60fd3e91aab0f8afed03683c` |
| `mini_body.dae` | `ba877c099da9a76f973f4c955260e8fc95224ea45b12fbbc1eca0ab2dfd55343` |
| `mini_lower_link.dae` | `e18ac026e8febee7e1d933235ff1a147268fa9009c8a46d2cc92012c53c50c91` |
| `mini_upper_link.dae` | `88a55f7c1de47945b0f51e990ace895e5fb3e46b51b0e5fade6d683249a8958e` |

权威原始路径为 `../../official-mini-cheetah/assets/meshes/*.dae`。
`visual_dae/` 是普通复制得到的工作副本，原始文件未移动、未改名、未覆盖。
URDF 的 `13` 个 mesh 实例只引用这 4 个本地副本，不读取原始目录。

DAE 包围盒长轴与 URDF 链路方向的静态对齐关系为：

- `mini_body.dae`：原点对齐 `base_link`；
- `mini_abad.dae`：`+Y` 为外伸方向，右侧 hip 绕 `X` 翻转；
- `mini_upper_link.dae`：`-X` 为长轴，绕 `Y` 旋转 `-90°` 后沿大腿
  `-Z`；
- `mini_lower_link.dae`：`+Z` 为长轴，绕 `Y` 旋转 `180°` 后沿小腿
  `-Z`。

这些是按 DAE 几何和 ORCAgym 运动学给出的静态挂载候选，不是 CAD 分件
结论；仍需在 USD/截图中做视觉核对。

## 数据来源与派生口径

运动学、逐 link 质量/惯量和碰撞候选来自 MIT ORCAgym 固定提交
`5aaff694ae5f1c31e08040d59787f2cca4c5cfe0`。控制周期、初始姿态和 PD
来自本项目 `YoboGo-control`。软件 softstop 参考来自 MIT
`Cheetah-Software/SpineBoard`。没有复制外部 URDF、USD、mesh 或策略。

ORCAgym simple URDF 的逐 link 合计为 `8.292 kg`。项目最终整机质量口径
为 `9 kg`，因此每个质量和惯量分量统一乘：

```text
scale = 9 / 8.292 = 1.085383502170767
```

| link 类型 | MIT 原始质量 kg | URDF 质量 kg | 实例数 |
| --- | ---: | ---: | ---: |
| base | 3.300 | 3.581765557164 | 1 |
| hip | 0.540 | 0.586107091172 | 4 |
| thigh | 0.634 | 0.688133140376 | 4 |
| shank | 0.064 | 0.069464544139 | 4 |
| foot | 0.010 | 0.010853835022 | 4 |
| **合计** | **8.292** | **9.000000000000** | **17** |

所有非足端惯量张量也使用同一比例缩放。ORCAgym 的 4 个 `foot` 惯量
张量源值全零，不能直接用于 PhysX；本资产使用可信的缩放后 foot 质量与
MIT 保守碰撞球半径，按实心球公式派生：

```text
m = 0.01 * 9 / 8.292 = 0.0108538350217 kg
r = 0.0202 m
I = 2/5 * m * r^2 = 1.7715195369e-06 kg·m^2
ixx = iyy = izz = I
ixy = ixz = iyz = 0
```

这是为满足 PhysX 正定性、由可信质量和 MIT 碰撞半径推导的近似，**不是
MIT 原始惯量，也不是猜测的 `epsilon`**。4 个 foot 的惯量特征值均严格
大于 0，派生公式、输入值和性质已写入 URDF metadata 并由验收器检查。

碰撞采用 MIT 保守候选：

- base：`0.30 × 0.20 × 0.10 m` box；
- hip：`length=0.025, radius=0.05 m` cylinder。该尺寸来自 ORCAgym
  源文件中被注释的候选块，本资产为完整包围主动启用；
- thigh：`length=0.17, radius=0.015 m` cylinder；
- shank：`length=0.10, radius=0.010 m` cylinder；
- foot：`radius=0.0202 m` sphere。

CAD 尚未分件，以上几何只能作为训练/静态导入候选，不能写成实机 CAD
碰撞事实。

## 控制与限位元数据

URDF 标准没有 PD、控制周期和“初始目标位姿”的稳定表达字段，因此这些
值以结构化 XML 注释写入，验收器会逐项检查：

- 初始目标：`FR/RR=[-0.6,-1.0,2.7]`，
  `FL/RL=[0.6,-1.0,2.7] rad`；
- PD：`Kp=[3,3,3]`、`Kd=[1,0.2,0.2]`；
- 控制周期：`0.002 s = 500 Hz`；
- 项目力矩限幅：`17/17/26 N·m`。

`<limit>` 中的 `abad ±1.5 rad`、`hip ±5.0 rad` 是 MIT 软件 softstop
参考；`knee [-2.7,2.7] rad` 只作为本地初始/安全候选。三者都不是已确认
的机械 hard stop，机械硬限位仍未知。`velocity=41/41/26.8 rad/s` 是
ORCAgym 能力参考，也不是 YoboGo 实机速度硬饱和。

## 静态验收

在项目根目录执行：

```bash
python3 -m py_compile \
  model/yobogo_mini_cheetah_v1/generate_urdf.py \
  model/yobogo_mini_cheetah_v1/validate_urdf.py
python3 model/yobogo_mini_cheetah_v1/generate_urdf.py
python3 model/yobogo_mini_cheetah_v1/validate_urdf.py
python3 model/yobogo_mini_cheetah_v1/validate_urdf.py --strict-inertia
```

本轮结果：

| 命令 | 退出码 | 结果 |
| --- | ---: | --- |
| `py_compile` | `0` | 两个脚本语法通过 |
| `generate_urdf.py` | `0` | URDF 可重复生成 |
| `validate_urdf.py` | `0` | `34` 项通过、`0` 项警告、`0` 项失败 |
| `validate_urdf.py --strict-inertia` | `0` | `34` 项通过、`0` 项警告、`0` 项失败；4 个 foot 严格正定 |

默认模式覆盖 XML、命名/顺序、拓扑、12 驱动关节、逐 link 质量和惯量、
`9 kg` 合计、foot 派生元数据与 4 个惯量特征值严格大于 0、运动学、
限位/力矩/速度、初始姿态、PD/500 Hz、4 个网格路径和 SHA256、碰撞、
腿映射、来源边界、机械限位边界和 CAD 边界。`--strict-inertia`
明确执行同一严格正定门禁，本轮两种模式均退出码 `0`。

## 数据边界

DAE 只负责外观；运动学、质量和碰撞来自固定 ORCAgym 提交；控制事实来自
YoboGo；softstop 明确标成 MIT 软件参考。不得把 ORCAgym 力矩
`18/18/28`、执行器参考能力、CAD 密度或旧 Webots/`official-mini-cheetah`
的 URDF 参数写入本资产。

更完整的来源、冲突和 P0 状态见
[`../research/data-manifest.md`](../research/data-manifest.md)。
