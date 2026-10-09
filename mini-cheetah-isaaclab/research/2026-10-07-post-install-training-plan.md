# Isaac Sim 4.5 / Isaac Lab 2.0 安装后训练路线图

> 调研日期：2026-10-07。本文只给路线、命令方向、验收与停止条件，
> 不复制任何外部 URDF、USD、mesh、策略或机器人数据。
>
> 机器人数据仍以 [`data-manifest.md`](data-manifest.md) 为唯一裁决入口：
> `YoboGo-control/` 为实机第一权威，网络 MIT 官方数据只补 YoboGo 缺口，
> CAD 只补几何，4 个 DAE 只补外观；`official-mini-cheetah/`、旧 Webots
> 中除这 4 个 DAE 外的数据全部禁用。

## 1. 当前状态快照

- venv：`/home/pma213x/.venvs/minicheetah-isaaclab`，Python `3.10.21`；
- 目标：Isaac Sim `4.5.0.0`、Isaac Lab `2.0.x`、RSL-RL、PyTorch；
- 2026-10-07 16:06 左右检查后台安装：`state=running`、PID `1529211`，
  尚不能安装 Isaac Lab 或运行训练；
- 日志正在解析 PyTorch/CUDA 依赖，日志已出现 `torch>=2.5.1` 的解析结果，
  但最终实际版本尚未固定；在 `isaacsim` 与 Isaac Lab 均能导入前，不把
  PyTorch/CUDA 标为兼容；
- P1 已完成的实机事实：整机质量口径 `9 kg`、关节力矩
  `17/17/26 N·m`、`leg[0..3]=FR/FL/RR/RL`、控制周期 `500 Hz`；
- P0/P1 未完成项仍阻断正式训练：CAD 分件与坐标、逐 link `9 kg` 分配、
  机械限位、足端接触、DAE 挂载、执行器延迟等，详见
  [`2026-10-07-p1-parameters.md`](2026-10-07-p1-parameters.md)。

## 2. 调研结论

### 2.1 官方已验证的步骤

以下命令和流程已经在 Isaac Lab v2.0.0 官方文档或官方源码中核对，
但**尚未在本机执行验证**：

1. Isaac Sim pip 安装后运行 `isaacsim`；Isaac Lab 从源码安装，并用
   `./isaaclab.sh -i rsl_rl` 安装配套 RSL-RL；
2. 空场景验证：
   `./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py`；
3. 任务列表验证：
   `./isaaclab.sh -p scripts/environments/list_envs.py`；
4. 官方 manager-based 平地速度任务示例可通过
   `scripts/reinforcement_learning/rsl_rl/train.py` 训练、`play.py` 回放；
5. URDF 可用官方 `scripts/tools/convert_urdf.py` 转为 USD，资产配置可用
   `UsdFileCfg`（推荐）或 `UrdfFileCfg` 创建 `ArticulationCfg`；
6. `play.py` 回放时会自动从 checkpoint 生成 `policy.pt` 与 `policy.onnx`；
7. RSL-RL 恢复训练使用 `--resume --load_run ... --checkpoint ...`。

关键官方证据：

- 标题：**Installation using Isaac Sim pip — Isaac Lab Documentation**
  <https://isaac-sim.github.io/IsaacLab/v2.0.0/source/setup/installation/pip_installation.html>
- 标题：**Available Environments — Isaac Lab Documentation**
  <https://isaac-sim.github.io/IsaacLab/v2.0.0/source/overview/environments.html>
- 标题：**Creating a Manager-Based RL Environment — Isaac Lab Documentation**
  <https://isaac-sim.github.io/IsaacLab/v2.0.0/source/tutorials/03_envs/create_manager_rl_env.html>
- 标题：**Registering an Environment — Isaac Lab Documentation**
  <https://isaac-sim.github.io/IsaacLab/v2.0.0/source/tutorials/03_envs/register_rl_env_gym.html>
- 标题：**Importing a New Asset — Isaac Lab Documentation**
  <https://isaac-sim.github.io/IsaacLab/v2.0.0/source/how-to/import_new_asset.html>
- 标题：**Writing an Asset Configuration — Isaac Lab Documentation**
  <https://isaac-sim.github.io/IsaacLab/v2.0.0/source/how-to/write_articulation_cfg.html>
- 标题：**Debugging and Training Guide — Isaac Lab Documentation**
  <https://isaac-sim.github.io/IsaacLab/v2.0.0/source/overview/reinforcement-learning/training_guide.html>
- 标题：**leggedrobotics/rsl_rl**
  <https://github.com/leggedrobotics/rsl_rl>
- 标题：**Adding your own learning library — Isaac Lab Documentation**
  <https://isaac-sim.github.io/IsaacLab/main/source/how-to/add_own_library.html>
- 标题：**Create new project or task — Isaac Lab Documentation**
  <https://isaac-sim.github.io/IsaacLab/main/source/overview/own-project/template.html>

### 2.2 本项目仍待验证的步骤

- 本机 Isaac Sim/Kit、Python、PyTorch、CUDA、GPU 驱动是否互相兼容；
- Isaac Lab `v2.0.x` 的精确检出版本与 RSL-RL 实际发行包版本；
- DAE 是否能正常导入 USD，CAD 单位/轴向/原点是否正确；
- `YoboGo-Velocity-Flat-v0` 是否注册、四腿数组是否无镜像；
- `physics_dt × decimation = 0.002 s` 是否严格实现 500 Hz；
- `9 kg` 是否按逐 link 分配并保持总和严格为 `9 kg`；
- 关节力矩是否确实按 `17/17/26 N·m` 截断；
- 单环境 1000 步、PPO 冒烟、恢复训练、导出和 sim2sim。

任何“官方已验证”都不等于“本项目已通过”。

## 3. 分阶段路线图

命令中的 `<ISAACLAB>` 表示 Isaac Lab 源码根目录，`<PROJECT>` 表示
`mini-cheetah-isaaclab/`。所有输出必须落在本工作区既定目录，禁止写到
Isaac Lab 仓库或项目根目录外。

| 阶段 | 输入 | 命令方向 | 输出 | 验收 | 失败停止条件 |
| --- | --- | --- | --- | --- | --- |
| 0 等待安装结束 | 后台 status/log | 读取 status；仅当 `state=success` 且 `exit_code=0` 才继续 | 环境安装成功记录 | Isaac Sim 安装进程结束且退出码为 0 | `failed`、进程异常、日志出现无法恢复的 resolver/GPU 错误 |
| 1 Isaac Sim/GPU 验证 | venv、NVIDIA 驱动 | 激活 venv；核对 `python`、`pip show`、`isaacsim --help`、`torch.cuda.is_available()` | `training/runs/environment/<timestamp>/environment-lock.txt` | Isaac Sim 启动、Python 3.10、GPU/CUDA 可用、实际版本与目标一致 | Kit 无法启动、CUDA 不可用、版本漂移或同一 venv 再次出现 pip 锁冲突 |
| 2 Isaac Lab + RSL-RL | Isaac Lab `v2.0.x` 源码 | `<ISAACLAB>/isaaclab.sh -i rsl_rl`；不要单独执行 `pip index rsl-rl` | 可编辑安装的 Isaac Lab 与配套 RSL-RL | `import isaaclab`、`import rsl_rl` 成功；版本写入环境锁 | RSL-RL 与 Isaac Lab ABI/依赖不兼容、PyTorch 被意外重装 |
| 3 官方最小冒烟 | Isaac Lab 自带脚本 | `create_empty.py` → `list_envs.py` → 官方 A1 平地 `random_agent.py --num_envs 4` | console、environment-lock、最小运行报告 | 退出码 0；官方任务可见；日志无 NaN/Inf；GPU 正常 | 空场景崩溃、任务列表为空、官方任务 reset/step 失败 |
| 4 资产构建/导入 | CAD、4 个 DAE、YoboGo、MIT 网络数据 | 生成项目 URDF/USD；`scripts/tools/convert_urdf.py` 转 USD；写 `ArticulationCfg` | `model/<asset-version>/`、导入报告 | 12 关节顺序 `FR/FL/RR/RL`、无镜像；质量总和 `9 kg`；力矩 `17/17/26`；DAE 只作 visual | 引用禁用来源、坐标/单位不明、外部模型复制、动力学字段无来源、P0 未关闭 |
| 5 项目环境注册 | 管理器环境、RSL-RL 配置 | 实现 `ManagerBasedRLEnvCfg`、`gym.register(id="YoboGo-Velocity-Flat-v0")`；`list_envs.py` 查找 | `training/envs/`、`training/agents/`、任务注册 | 任务可发现；1000 步 reset/step 全为有限值；控制周期为 0.002 s | 任务未注册、接触/动作/奖励通道异常、NaN/Inf、周期错误 |
| 6 零/随机动作与站立冒烟 | 锁定资产和环境 | `zero_agent.py` 与 `random_agent.py --num_envs 4`；随后做短时站立测试 | 视频/日志、`gate.json` | 站立不爆、四足接触可信、动作不越界；此阶段不产出正式 checkpoint | 爆炸、穿透、力矩超限、接触信号不可信 |
| 7 PPO 短训练 | `num_envs=4`、固定 seed、短 `max_iterations` | `scripts/rsl_rl/train.py --task YoboGo-Velocity-Flat-v0 --num_envs 4 --headless --seed 0 --max_iterations N` | 临时 checkpoint、TensorBoard、日志 | 多个有限 reward/loss；恢复与 play 能加载；所有产物路径正确 | reward 符号错误、NaN/Inf、动作饱和 `>10%`、产物越界 |
| 8 平地正式 PPO | 静态验收 PASS、固定 seed | 先零命令站立，再 `vx=0.15 m/s` 短测，最后扩大预算 | checkpoints/logs/runs/outputs | 站立接触 `>=0.95`；速度误差 `<=0.10 m/s`；偏航误差 `<=0.20 rad/s`；跌倒率 `<=5%` | 连续两个固定 seed Gate 失败、跌倒率超标、奖励/终止逻辑错误 |
| 9 play/export/resume | 合格 checkpoint | `play.py` 自动生成 `policy.pt`/`policy.onnx`；`train.py --resume --load_run ... --checkpoint ...` | `exported/policy.pt`、`policy.onnx`、恢复日志 | checkpoint 可恢复；导出输入输出维度与观测/动作一致；ONNX/TorchScript CPU 推理通过 | 无法加载、导出维度不一致、策略输出越界或与 play 不一致 |
| 10 Sim2sim | 固定 checkpoint、同一观测/动作/500 Hz 契约 | 独立 sim2sim 桥重放，不迁移旧 Webots 物理参数 | `outputs/.../sim2sim/`、指标和视频 | 关节目标 RMS `<=0.05 rad`；无越界/NaN；站立不倒；命令方向一致 | 任一指标失败，回退平地或 rough，不得进入实机 |
| 11 部署准备 | Sim2sim PASS、YoboGo 接口 | 只迁策略与接口；核对急停、阻尼、限幅、超时、版本校验 | `outputs/.../deployment-review/` | 人工审核、安全门与 Git 审核齐全 | 硬件桥/安全契约缺失、未审核、P1 延迟等未知项未按风险接受 |

## 4. 具体命令模板

### 4.1 环境验证

```bash
source /home/pma213x/.venvs/minicheetah-isaaclab/bin/activate

python --version
pip show isaacsim isaacsim-core torch rsl-rl-lib || true
isaacsim --help

python - <<'PY'
import torch
print("torch:", torch.__version__)
print("cuda:", torch.version.cuda)
print("available:", torch.cuda.is_available())
if torch.cuda.is_available():
    print("device:", torch.cuda.get_device_name(0))
PY

cd <ISAACLAB>
./isaaclab.sh --help
./isaaclab.sh -i rsl_rl
./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py
./isaaclab.sh -p scripts/environments/list_envs.py
./isaaclab.sh -p scripts/environments/random_agent.py \
  --task Isaac-Velocity-Flat-Unitree-A1-v0 --num_envs 4
```

RSL-RL 的正确安装方向是随 Isaac Lab 安装器完成，再核对实际包与
`import rsl_rl`；不要用 `pip index rsl-rl` 的失败结果推断包不存在。

### 4.2 资产转换与项目任务

```bash
cd <ISAACLAB>
./isaaclab.sh -p scripts/tools/convert_urdf.py \
  <PROJECT>/model/<asset-version>/<name>.urdf \
  <PROJECT>/model/<asset-version>/<name>.usd \
  --headless

cd <PROJECT>
python scripts/environments/list_envs.py
python scripts/environments/random_agent.py \
  --task YoboGo-Velocity-Flat-v0 --num_envs 4 --headless
python scripts/environments/zero_agent.py \
  --task YoboGo-Velocity-Flat-v0 --num_envs 4 --headless
```

项目生成器路径可能把脚本放在 `scripts/rsl_rl/`，而 Isaac Lab 核心源码
使用 `scripts/reinforcement_learning/rsl_rl/`；实施时以实际生成结构和
`--help` 为准，不凭猜测改路径。

### 4.3 训练、恢复、回放与导出

```bash
cd <PROJECT>
python scripts/rsl_rl/train.py \
  --task YoboGo-Velocity-Flat-v0 \
  --num_envs 4 --headless --seed 0 --max_iterations 5

python scripts/rsl_rl/train.py \
  --task YoboGo-Velocity-Flat-v0 \
  --num_envs 4 --headless --seed 0 \
  --resume --load_run <run-folder> --checkpoint <model.pt>

python scripts/rsl_rl/play.py \
  --task YoboGo-Velocity-Flat-v0 \
  --num_envs 4 --headless \
  --load_run <run-folder> --checkpoint <model.pt>
```

官方 `play.py` 会在 checkpoint 同级的 `exported/` 目录写：

- `policy.pt`：TorchScript；
- `policy.onnx`：ONNX。

短训练只用于验证链路，不把其 checkpoint 标为可用策略。

## 5. 本项目关键验收口径

1. **来源门**：每个资产字段能回指 `data-manifest.md`；禁用来源引用数为
   0，外部模型文件复制数为 0。
2. **装配门**：`leg[0]=FR、leg[1]=FL、leg[2]=RR、leg[3]=RL`，12 关节
   顺序、轴、零位和正负号逐关节核对，无前后/左右镜像。
3. **动力学门**：逐 link 质量显式登记且合计严格 `9 kg`；惯量对称正定；
   关节力矩限制为 `17/17/26 N·m`，保留原码 `TODO CHECK WITH BEN` 证据。
4. **周期门**：`physics_dt × decimation = 0.002 s`，即 500 Hz；TCP 50 ms
   状态周期不得替代控制周期。
5. **冒烟门**：官方空场景、官方任务和项目任务均退出码 0；项目单环境
   1000 步 reset/step 无 NaN/Inf。
6. **训练门**：先站立，后 `vx=0.15 m/s`；满足平地 Gate 才扩大预算。
7. **导出门**：恢复、play、ONNX/TorchScript 推理维度和动作范围一致。
8. **迁移门**：sim2sim 关节目标 RMS `<=0.05 rad` 后才讨论实机；P1 延迟、
   摩擦、限位等未知项必须逐项实测或由用户明确接受风险。

## 6. P1 未关闭时如何使用未知项

- 机械硬限位未知：先只在已验证软件安全区内做冒烟，禁止用 MIT softstop
  冒充 YoboGo 机械 stop；
- 执行器延迟、摩擦、足地摩擦未知：平地 PPO 可以在固定仿真参数下进行，
  但不得声称 sim2real 已验证；Rough/DR 开始前至少完成对应台架实测或
  把不确定范围写入随机化；
- IMU 位姿/噪声、TCP/LCM 延迟未知：不影响纯关节观测的首次站立冒烟，
  但影响含本体/IMU延迟的部署候选；
- `9 kg` 逐 link 分配、CAD 坐标和碰撞未知：直接阻断资产静态 PASS，
  进而阻断正式 PPO。

## 7. SearXNG 查询记录

本轮使用 `mcp__searxng_search__search_web`，未调用 Tavily：

1. `Isaac Lab 2.0 installation Isaac Sim 4.5 verify list_envs example commands`
2. `Isaac Lab 2.0 manager based velocity locomotion train play export rsl_rl official`
3. `Isaac Lab import URDF USD custom robot ArticulationCfg official documentation`
4. `Isaac Lab 2.0 RSL-RL install rsl_rl official repository dependencies`
5. `site:isaac-sim.github.io/IsaacLab v2.0 RSL RL train.py play.py export policy ONNX TorchScript`
6. `Isaac Lab RSL RL resume checkpoint train.py --resume command official`
7. `Isaac Lab sim2sim export ONNX TorchScript MuJoCo quadruped official documentation`
8. `Isaac Lab 2.0 manager based velocity configuration env.yaml agent.yaml training command`

查询 5 返回 0 条；随后检查本地 SearXNG：首页 HTTP 200，JSON API
`/search?q=Isaac%20Lab&format=json` HTTP 200，且其余查询正常返回，
因此没有把该空结果当作服务故障，也没有据此编造结论。

## 8. 结论

安装完成后的最短可执行顺序是：

`Isaac Sim/GPU 验证 → Isaac Lab v2.0.x + RSL-RL → 空场景/list_envs/官方
任务冒烟 → 项目资产与静态 PASS → 注册 YoboGo 任务 → 1000 步冒烟 →
短 PPO → 站立/速度 Gate → play/export/resume → sim2sim → 人工部署审核`。

在环境锁定和资产 P0 完成前，不得启动正式 PPO；在 sim2sim 与部署安全门
完成前，不得下发实机策略。
