# Mini Cheetah 训练参考项目核验（2026-10-07）

## 1. 使用边界

本文件只记录证据 URL、许可证、版本和迁移方法。**不下载、不复制**
任何外部模型、URDF、USD、mesh 或策略权重。机器人数据按三层来源
执行，详见 [`data-manifest.md`](data-manifest.md)：

1. 本地 `official-mini-cheetah/assets/meshes/*.dae`：外观权威；
2. `YoboGo-control/`：实机控制、接口、控制周期和安全权威；
3. 网络上的 MIT 官方 Mini Cheetah 数据：仅补齐本地缺失的运动学、
   惯量、碰撞和执行器参考，并记录 URL、许可证与冲突裁决。

## 2. 推荐排名

| 排名 | 项目 | 版本/环境 | 许可证证据 | 可迁移度 | 证据 URL |
| ---: | --- | --- | --- | --- | --- |
| 1 | Isaac Lab 官方 manager velocity | 本项目**计划**锁定 `Isaac Lab v2.0.x` + `Isaac Sim 4.5.x`；尚未在本机安装验证 | 仓库 GitHub 识别 BSD-3-Clause，README 同时展示 BSD-3-Clause/Apache-2.0 | **最高**：观测/动作/奖励/命令/事件 manager 与 RSL-RL 入口直接作为骨架 | [环境列表](https://isaac-sim.github.io/IsaacLab/main/source/overview/environments.html)、[v2.0.0](https://github.com/isaac-sim/IsaacLab/releases/tag/v2.0.0)、[RL 脚本](https://isaac-sim.github.io/IsaacLab/main/source/overview/reinforcement-learning/rl_existing_scripts.html) |
| 2 | 官方项目生成器 | Isaac Lab 已把旧扩展模板迁入本体生成器；旧 `IsaacLabExtensionTemplate` 已标记不再维护 | 旧模板 Apache-2.0；生成器随 Isaac Lab 主仓库许可证 | **高**：生成本项目扩展结构，随后替换为 YoboGo 数据 | [How to create your own project](https://isaac-sim.github.io/IsaacLab/main/source/overview/own-project/template.html)、[旧模板存档](https://github.com/isaac-sim/IsaacLabExtensionTemplate) |
| 3 | `evelyd/MiniCheetah_IsaacLabExtension` | README 明示 Isaac Sim 4.5.0、Isaac Lab 2.0.0 | MIT（根 `LICENCE`/README badge） | **中高**：工程结构、依赖和脚本可参考；不是完成训练证明 | [仓库](https://github.com/evelyd/MiniCheetah_IsaacLabExtension)、[README](https://github.com/evelyd/MiniCheetah_IsaacLabExtension/blob/main/README.md) |
| 4 | MIT `ORCAgym` / `pkGym` | 旧 Isaac Gym Preview 3/4、Python 3.8；ORCA README 明确建议新项目用 pkGym | 根 LICENSE 为 BSD 3-Clause 文本，但 GitHub API 返回 `NOASSERTION` | **中**：Mini Cheetah 配置、奖励和课程可读；框架需移植 | [ORCAgym](https://github.com/mit-biomimetics/ORCAgym)、[pkGym](https://github.com/mit-biomimetics/pkGym) |
| 5 | `rapid-locomotion-rl` | Isaac Gym Preview 3、PyTorch 1.10/CUDA 11.3，论文 RSS 2022 | 顶层 MIT；内含 `legged_gym/rsl_rl` 各自许可证 | **中**：teacher-student、快速运动课程与部署观察值得参考 | [README](https://github.com/Improbable-AI/rapid-locomotion-rl/blob/main/README.md)、[LICENSE](https://github.com/Improbable-AI/rapid-locomotion-rl/blob/main/LICENSE) |
| 6 | `xu-yang16/rl_wbc` | Isaac Gym 训练框架；`train.py`/`eval.py` | MIT | **中低**：RL+WBC 分层、`mini_cheetah` 评估接口可参考，非 Isaac Lab | [README](https://github.com/xu-yang16/rl_wbc/blob/master/README.md)、[LICENSE](https://github.com/xu-yang16/rl_wbc/blob/master/LICENSE) |
| 7 | `MiniCheetah_rl_deploy` | MuJoCo sim2sim，实机构建明确被安全门阻止 | BSD-3-Clause | **部署阶段中高，训练阶段低**：只用于策略接口、状态机和安全门参考 | [README](https://github.com/yun-tianmin9/MiniCheetah_rl_deploy/blob/main/README.md)、[LICENSE](https://github.com/yun-tianmin9/MiniCheetah_rl_deploy/blob/main/LICENSE) |

## 3. 重点项目细节

### 3.1 evelyd / MiniCheetah_IsaacLabExtension

- Isaac Sim：`4.5.0`；Isaac Lab：`2.0.0`。
- 安装对象：`python -m pip install -e source/mini_cheetah`。
- README 验证任务仍是模板 Anymal：
  `Template-Isaac-Velocity-Rough-Anymal-D-v0`。
- `scripts/rsl_rl/train.py` 通过 `--task` 训练；`scripts/rsl_rl/play.py`
  通过同一 `--task` 回放。
- 证据：
  [train.py](https://github.com/evelyd/MiniCheetah_IsaacLabExtension/blob/main/scripts/rsl_rl/train.py)、
  [play.py](https://github.com/evelyd/MiniCheetah_IsaacLabExtension/blob/main/scripts/rsl_rl/play.py)。
- **风险**：名称虽为 Mini Cheetah，README 和验证命令仍主要是扩展模板/
  Anymal；必须审计注册任务、资产来源和 reward，不能把仓库名当作已完成
  Mini Cheetah 训练。

### 3.2 MIT ORCAgym / pkGym

- ORCA README 的训练任务为 `mini_cheetah_osc`，命令：
  `python gym/scripts/train.py --task=mini_cheetah_osc`；
  回放为 `python play_ORC.py --task=mini_cheetah_osc --ORC_toggle=...`。
- ORCA README 明示 `pkGym` 是新项目推荐版本。
- 两者都基于旧 Isaac Gym，不是 Isaac Lab；迁移到 manager velocity 时
  只移植观测/动作/奖励/PD/课程语义，不移植仿真调用。
- 证据：[ORCA README](https://github.com/mit-biomimetics/ORCAgym/blob/main/README.md)、
  [pkGym README](https://github.com/mit-biomimetics/pkGym/blob/main/README.md)。

### 3.3 rapid-locomotion-rl

- 明确支持 MIT Mini Cheetah 与 Go1。
- `scripts/train.py`、`play.py`、`test.py` 分别训练、评估、冒烟。
- 有 Grid Adaptive Curriculum 和 teacher-student；默认约 4000 环境、
  README 说明约需 12 GB 显存。
- 风险：依赖旧 Isaac Gym、PyTorch 1.10/CUDA 11.3，不能直接作为
  Isaac Lab v2.0 运行代码。

### 3.4 rl_wbc

- `python train.py` 训练，`python eval.py` 评估。
- `eval.py --name=[go1,go2,a1,mini_cheetah]`，
  `--use_real_robot=[0,1,2]` 对应 Isaac Gym、MuJoCo、实机。
- 可迁移点是高层 RL 与低层 WBC 分层及动作安全门；不把其机器人数值
  覆盖本项目数据清单。

### 3.5 MiniCheetah_rl_deploy

- 用途是 MuJoCo sim2sim 和未来 sim2real，不是训练器。
- README 明确实机控制在 Cheetah-Software 硬件桥和安全契约完成前被阻止；
  仿真构建有更宽的关节位置容差，硬件构建保持严格门限。
- 只迁移接口、状态机、日志和停止条件设计。

## 4. Isaac Lab v2.0 选择依据

官方环境页给出的 manager velocity 示例包括：

- `Isaac-Velocity-Flat-Anymal-D-v0`
- `Isaac-Velocity-Rough-Anymal-D-v0`
- `Isaac-Velocity-Flat-Unitree-Go2-v0`
- `Isaac-Velocity-Flat-Unitree-A1-v0`

其中平坦任务支持 RSL-RL PPO。推荐使用平坦 manager velocity 作为第一
阶段，而不是先复制 rough/跳跃任务。项目生成器采用官方文档当前入口，
旧扩展模板仅作 evelyd 仓库的版本对照。

## 5. 版本与项目风险

- `Isaac Lab v2.0.x / Isaac Sim 4.5.x / Python 3.10` 是本项目首版
  **计划锁定值**，当前工作区没有安装锁、环境清单或运行日志，不能写成
  “已兼容”或“已验证”。
- 首次启动必须核验实际 Isaac Lab、Isaac Sim/Kit、Python、RSL-RL、
  CUDA、GPU 驱动和 USD importer 版本；任何一项漂移都必须重新跑
  `list_envs`、资产导入和单环境 1000 步冒烟。
- DAE 导入 USD、`2 ms/500 Hz` 控制周期、decimation、PhysX 接触和
  RSL-RL checkpoint 格式均是本项目风险，不由参考仓库版本证明。
- 旧 Isaac Gym 项目不能直接运行在 Isaac Lab；evelyd 仓库名也不证明
  已完成 Mini Cheetah 训练。
- 没有 YoboGo 逐 link 质量/惯量、碰撞、机械限位和已确认力矩前，
  `YoboGo-Velocity-Flat-v0` 只能停留在静态验收，不得开始正式 PPO。

## 6. 结论

可直接迁移的是“工程骨架、manager 分层、RSL-RL 入口和验收方法”；
Mini Cheetah 训练代码必须逐项审计。没有一个仓库可以替代三层数据清单。
本文件没有下载或复制任何外部模型、URDF、USD 或策略。
