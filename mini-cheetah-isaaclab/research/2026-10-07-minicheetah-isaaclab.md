# Mini Cheetah / Isaac Lab 开源实例调研（2026-10-07）

## 1. 调研结论与硬边界

当前没有找到一个同时满足以下条件的“拿来即训”开源项目：

1. 明确使用 MIT Mini Cheetah；
2. 基于 Isaac Lab；
3. 提供完整、可复现的训练结果；
4. 模型数据又符合本项目的来源限制。

最实际的迁移路径是：以 Isaac Lab 官方 manager-based 四足 locomotion
环境和 RSL-RL 训练入口为骨架，参考 Mini Cheetah 扩展项目的资产接入
方式；机器人数据按三层来源获取：实机控制接口来自
`YoboGo-control/`，外观只使用已确认的 4 个官方 DAE，本地缺失物理
参考可来自 MIT 官方网络数据并登记冲突。

### 三层数据边界

- 本地 `official-mini-cheetah/assets/meshes/*.dae` 是外观权威，已复制
  到 `../model/visual_dae/`，原文件不动。
- `YoboGo-control/` 是实机控制、接口、控制周期和安全限制权威。
- YoboGo 缺少的运动学、逐连杆惯量、碰撞和执行器标称能力，允许从
  网络上的 MIT 官方 Mini Cheetah 数据选取；必须在
  [`data-manifest.md`](data-manifest.md) 记录数值、URL、许可证、选用
  理由、冲突和最终采用规则。
- 其他第三方和 Isaac Lab 示例机器人仍只可参考代码，不得直接覆盖
  本项目模型。
- 所有训练、checkpoint、日志和 run 只能位于 `../training/`；
  最终报告和导出产物位于 `../outputs/`。

## 2. 本地 official-mini-cheetah 资产清单与缺口

本轮只把以下 4 个文件认定为允许使用且已复制的官方外观资产：

| 文件 | 字节数 | SHA256 |
| --- | ---: | --- |
| `mini_abad.dae` | 356344 | `6a34448654e342b6b71e71a1ca263e327fba8f4f60fd3e91aab0f8afed03683c` |
| `mini_body.dae` | 1326634 | `ba877c099da9a76f973f4c955260e8fc95224ea45b12fbbc1eca0ab2dfd55343` |
| `mini_lower_link.dae` | 1236637 | `e18ac026e8febee7e1d933235ff1a147268fa9009c8a46d2cc92012c53c50c91` |
| `mini_upper_link.dae` | 1626723 | `88a55f7c1de47945b0f51e990ace895e5fb3e46b51b0e5fade6d683249a8958e` |

源路径：`../../official-mini-cheetah/assets/meshes/`

副本路径：[`../model/visual_dae/`](../model/visual_dae/)

源文件和副本逐文件 SHA256 一致。DAE 只覆盖外观，明确缺少：

- 连杆树、关节名称、顺序、轴和零位；
- 质量、质心和惯量；
- 碰撞几何和足端接触模型；
- 关节限位、速度/扭矩能力与安全限制；
- 执行器延迟、控制频率和动作到目标的映射；
- 默认站立姿态、机体系和传感器定义；
- 地面摩擦、域随机化范围和验收阈值。

以上缺口优先从 `YoboGo-control/` 提取；本地确实缺失的物理参考可按
[`data-manifest.md`](data-manifest.md) 从 MIT 官方网络数据选取。不得
读取本机 `Cheetah-Software/`，也不复制外部 URDF/USD/策略文件。

## 3. MIT 官方 Mini Cheetah 数据检索结果

### 3.1 可核验的官方/论文入口

- MIT Bi Robotics 的 Cheetah 软件仓库：
  [mit-biomimetics/Cheetah-Software](https://github.com/mit-biomimetics/Cheetah-Software)
- Mini Cheetah 论文入口：
  [IEEE Xplore: A Highly Modularized Quadrupedal Robot for Legged Locomotion](https://ieeexplore.ieee.org/document/8745607)
- Isaac Lab 官方仓库：
  [isaac-sim/IsaacLab](https://github.com/isaac-sim/IsaacLab)

论文和官方软件可用于核对 Mini Cheetah 的基本公开事实，例如 12 个
驱动关节、每腿 3 自由度等；但它们不是本项目模型数据源。尤其本机的
`Cheetah-Software/` 被明确禁止读取，本轮没有读取。

### 3.2 对“MIT 官方完整 Isaac Lab 模型包”的判断

本轮没有核验到 MIT 官方发布的、带 USD/Isaac Lab AssetCfg、训练环境
和可复现 checkpoint 的完整 Mini Cheetah 包。MIT 官方材料更偏控制软件、
论文与硬件方法；Isaac Lab 资产和训练主要由 NVIDIA、ETH 以及社区维护。
因此不能把任一第三方 `mini_cheetah_urdf` 当作 MIT 官方权威模型。

### 3.3 网络 Mini Cheetah 数据如何处理

网络结果按三层来源规则处理：

- 可用于查找 Mini Cheetah 关节/网格命名历史的第三方仓库：
  [Derek-TH-Wang/mini_cheetah_urdf](https://github.com/Derek-TH-Wang/mini_cheetah_urdf)
- 唯一直接命中 Mini Cheetah + Isaac Lab 的社区扩展：
  [evelyd/MiniCheetah_IsaacLabExtension](https://github.com/evelyd/MiniCheetah_IsaacLabExtension)

MIT 官方 ORCAgym/Mini Cheetah URDF 可作为缺失物理参数的**数值来源**，
但只把核验后的数值写入集中清单，不复制文件；其他第三方仍只作研究线索。

## 4. Isaac Lab / 训练参考项目排名

| 排名 | 项目 | 为什么值得参考 | 限制与结论 |
| ---: | --- | --- | --- |
| 1 | [isaac-sim/IsaacLab](https://github.com/isaac-sim/IsaacLab) 与 [强化学习官方文档](https://isaac-sim.github.io/IsaacLab/main/source/overview/reinforcement-learning/rl_existing_scripts.html) | 官方 manager-based 环境、观测/动作/奖励管理器、RSL-RL/SKRL/RL-Games 入口和四足示例最完整，版本兼容信息明确 | **首选骨架**；示例机器人数据不得进入本项目 |
| 2 | [evelyd/MiniCheetah_IsaacLabExtension](https://github.com/evelyd/MiniCheetah_IsaacLabExtension) | 唯一明确同时出现 Mini Cheetah 与 Isaac Lab 的社区仓库，README 面向 Isaac Lab 2.0 扩展模板 | README 主要是 Extension Template，示例任务仍使用 Anymal；未证明已完整训练 Mini Cheetah，**只参考工程组织** |
| 3 | [leggedrobotics/legged_gym](https://github.com/leggedrobotics/legged_gym) 与 [rsl_rl](https://github.com/leggedrobotics/rsl_rl) | 四足 sim-to-real 的成熟 reward、DR、地形课程、PPO 和部署结构；Isaac Lab 官方也采用 RSL-RL | `legged_gym` 基于旧 Isaac Gym，README 已提示迁移；**移植算法思想，不直接运行旧框架** |
| 4 | [IshaanM05/blind-quadruped-locomotion](https://github.com/IshaanM05/blind-quadruped-locomotion) | 把 manager env、平坦速度跟踪、地形课程、域随机化、teacher-student 蒸馏拆成连续阶段，文档较细 | README 明示 Phase 0–1 仍在进行，尚未给出成熟 Go2 成果；**适合按阶段设计，不能直接当成品** |
| 5 | [Nersisiian/go2-isaac-lab-suite](https://github.com/Nersisiian/go2-isaac-lab-suite) | 有 train/play/evaluate、配置、checkpoint 的完整目录形态，覆盖行走、跳跃等任务 | 机器人是 Go2，README 的完成度声明需要复现验证；**只借鉴目录、命令和验收结构** |

### 推荐组合

1. 用 Isaac Lab 官方 locomotion manager 环境作为基线；
2. 用 RSL-RL 作为 PPO/蒸馏训练器；
3. 读 `MiniCheetah_IsaacLabExtension` 了解社区如何组织机器人扩展；
4. 读 `blind-quadruped-locomotion` 的阶段路线，但按本项目重新验收；
5. 所有机器人参数在本工作区内从 `YoboGo-control/` 生成清单并校验。

## 5. 数据来源矩阵

| 数据类别 | 唯一/允许来源 | 是否可来自网络或其他目录 | 落盘位置 |
| --- | --- | --- | --- |
| 4 个外观 DAE | `official-mini-cheetah/assets/meshes/*.dae` | 允许，且仅此 4 个 | `../model/visual_dae/` |
| 连杆与关节拓扑 | YoboGo 接口顺序 + MIT 缺失坐标 | 可，仅 MIT 官方并登记清单 | `../model/` 生成的项目资产 |
| 惯量、质心、碰撞 | YoboGo 总量优先 + MIT 分布 | 可，仅 MIT 官方并记录换算 | `../model/` |
| 关节顺序、轴、零位、默认姿态 | YoboGo 顺序/零位 + MIT 缺失限位 | 有限允许 | `../model/` 与 `../training/envs/` |
| 执行器、控制周期、动作映射 | YoboGo 安全值 + MIT 标称参考 | 仅 MIT 标称值可补缺 | `../training/envs/` 配置 |
| 奖励、观测、课程、DR 初值 | 项目设计文档 + YoboGo 事实 | 网络只可参考方法，不能复制机器人数值 | `../training/envs/` |
| PPO/RSL-RL 训练器代码 | Isaac Lab / RSL-RL 官方开源 | 可 | `../training/agents/`、`scripts/` |
| checkpoint、日志、TensorBoard run | 本地训练产生 | 不适用 | `../training/checkpoints|logs|runs/` |
| 评估报告、视频、导出策略 | 本地评估产生 | 不适用 | `../outputs/` |

## 6. 后续步骤

1. **建立三层数据清单**：从 `YoboGo-control/` 提取实机接口、零位、
   控制周期和安全值；MIT 官方网络数据只补缺失物理值；每项记录数值、
   URL、许可证、冲突与最终规则。
2. **装配项目模型**：在 `../model/` 生成本项目自己的 URDF/USD 或
   Isaac Lab 资产配置，外观引用 `visual_dae/`；只使用数据清单批准的
   YoboGo/MIT 数值，不复制外部文件。
3. **模型静态验收**：检查 12 关节、四腿命名、足端、单位、坐标系、
   零位、重力、惯量正定、碰撞和 DAE 路径；不通过则停止。
4. **建立平坦速度环境**：在 `../training/envs/` 按 Isaac Lab manager
   拆分观测、动作、奖励、命令和事件；先只做站立与低速前进。
5. **最小冒烟**：少量环境、短步数验证 reset、有限值、接触、关节限位、
   checkpoint 路径和 TensorBoard 路径；不把冒烟当成绩。
6. **PPO 基线**：在 `../training/agents/` 固定 seed 和配置，在
   `../training/scripts/` 提供 train/play/evaluate；所有产物只写本目录。
7. **逐步增强**：通过静态验收后再加命令覆盖、域随机化、地形课程、
   teacher-student 蒸馏；每一级单独验收，不一次全开。
8. **文档同步**：每次代码修改更新根 `docs/changes/`，能力变化更新
   根 `docs/features/`；不得在工作区再复制一份 `docs/`。

## 7. 本轮未做事项

- 未读取 `Cheetah-Software/`；
- 未修改 `YoboGo-control/` 或 `official-mini-cheetah/` 原文件；
- 未安装 Isaac Lab，未启动训练；
- 未执行任何 `git add/commit/push/merge/rebase`。
