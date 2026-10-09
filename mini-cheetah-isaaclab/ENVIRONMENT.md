# Mini Cheetah Isaac Lab 环境说明

> 更新时间：2026-10-07。
> 本文件只描述 Python 环境、安装状态和后续锁定步骤，不包含任何凭据。

## 1. 环境位置与边界

- Python 虚拟环境：
  `/home/pma213x/.venvs/minicheetah-isaaclab`
- Python 版本：`3.10.21`
- 安装日志：
  `outputs/environment-install/`
- pip、uv、临时文件和下载缓存均位于 `/home/pma213x/.cache/`，
  不写入 NTFS 项目盘；
- 不使用 Docker，不修改系统级 Python 包；
- 训练运行时仍必须把实际版本锁到
  `training/runs/environment/<timestamp>/environment-lock.txt`。

## 2. 实际安装版本

- Python：`3.10.21`
- Isaac Sim：`4.5.0.0`
- Isaac Lab 仓库：`v2.0.2`
- Isaac Lab SHA：
  `b5fa0eb031a2413c182eeb54fa3a9295e8fd867c`
- Isaac Lab 扩展包：`isaaclab 0.34.9`、`isaaclab-tasks 0.10.24`、
  `isaaclab-rl 0.1.0`
- PyTorch：`2.5.1+cu121`，`torch.version.cuda=12.1`
- RSL-RL：`rsl-rl-lib 2.3.3`
- Gymnasium：`1.2.3`

Isaac Sim 首次安装得到的 `torch 2.14.1+cu130` 与 Isaac Lab 2.0.2
明确固定的 `torch==2.5.1` 不兼容，因此按官方文档改为
`torch==2.5.1 --index-url https://download.pytorch.org/whl/cu121`。
降级后 `torch.cuda.is_available()` 和 CUDA 张量计算均通过。

Isaac Lab 的 `isaaclab_rl` 包顶层会无条件导入 RSL-RL、RL-Games、SB3、
skrl 四个包装器，因此除 `rsl-rl` 外还按该包官方 `all` extra 安装：

- `gym 0.23.1`
- `rl-games 1.6.1`
- `stable-baselines3 2.8.0`
- `skrl 2.1.0`

完整包清单见
`outputs/environment-install/environment-lock.txt`。

## 3. EULA 与后台安装结果

NVIDIA Omniverse EULA 已按官方提示接受，`import isaacsim` 成功，
日志为 `outputs/environment-install/eula-import.log`。

Isaac Sim 后台安装结果以
`outputs/environment-install/isaacsim-install.status` 为准：

- `state=success`
- 开始时间：`2026-10-07T15:21:56+08:00`
- 结束时间：`2026-10-07T16:10:41+08:00`
- `exit_code=0`

固定文件：

- 状态：`isaacsim-install.status`
- PID：`isaacsim-install.pid`
- 独立日志：`isaacsim-install.log`
- 启动脚本：`run-isaacsim-install.sh`
- 命令记录：`commands.log`

状态允许值：

- `running`：安装仍在进行，不对同一 venv 执行其他 pip；
- `success`：Isaac Sim 退出码为 `0`，才可继续 Isaac Lab/RSL-RL；
- `failed`：立即停止，保留日志和退出码，不自动更换大版本。

## 4. 已完成的官方冒烟

- EULA 后 `import isaacsim`：退出码 `0`；
- PyTorch CUDA 检查：`CUDA_AVAILABLE=True`，
  `RTX 4060 Laptop GPU`，CUDA 张量结果正确，退出码 `0`；
- 官方 `list_envs.py`：列出 96 个注册环境，退出码 `0`；
- 官方 `test_manager_based_env.py`：
  `Ran 1 test ... OK`，分别在 `cuda:0` 和 `cpu` 上创建 1 个空环境并
  执行 2 次 step，退出码 `0`，最终日志无 `[Error]`；
- 所有命令和日志位于 `outputs/environment-install/`。

这些结果证明基础环境可 reset/step、CUDA 正常、日志可写，但尚未运行
正式训练，也未生成正式 checkpoint。训练前仍须按项目训练计划生成
`training/runs/environment/<timestamp>/environment-lock.txt` 并完成
YoboGo 资产静态验收。
