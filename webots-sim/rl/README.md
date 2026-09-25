# 四足机器狗 PPO 行走训练（Stable-Baselines3）

用 PPO（Stable-Baselines3）在 Webots 仿真中训练四足机器狗行走。
本目录是**训练侧**；Gym 环境接口由同目录下的 `walk_env.py` 提供。

```
webots-sim/rl/
├── config.py       # 超参数 / 奖励权重 / 观测动作维度（集中管理，优先改这里）
├── walk_env.py     # Gym 环境 QuadrupedWalkEnv（obs 42 维，action 12 维）
├── train_ppo.py    # 训练入口
├── play.py         # 加载模型回放 / 演示 / 录视频
└── README.md       # 本说明
```

---

## 1. 环境准备

### 1.1 Python 依赖

需要 Python 3.9+（推荐 3.10 / 3.11，需与 PyTorch 官方 wheel 匹配）：

```bash
pip install stable-baselines3 gymnasium torch tensorboard
# 录视频（二选一即可）
pip install imageio imageio-ffmpeg
# 或
pip install opencv-python
```

> 建议在虚拟环境中安装，例如：
> ```bash
> python3 -m venv .venv && source .venv/bin/activate
> pip install stable-baselines3 gymnasium torch tensorboard imageio imageio-ffmpeg
> ```

### 1.2 环境接口约定

`train_ppo.py` / `play.py` 直接使用 `walk_env.py`：

```python
from walk_env import QuadrupedWalkEnv
# obs    : float32, 42 维
# action : float32, 12 维，范围 [-1, 1]
# reset() -> (obs, info)
# step(a) -> (obs, reward, terminated, truncated, info)
```

若 `walk_env.py` 尚未就绪，仍可先用 `--dry-run` 验证命令行参数：

```bash
python train_ppo.py --dry-run
python play.py --dry-run
```

---

## 2. 怎么训练

在 `webots-sim/rl/` 目录下执行：

```bash
# 推荐：先 dry-run 确认参数
python train_ppo.py --dry-run

# 正式训练 20 万步
python train_ppo.py --total-steps 200000
```

常用参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--total-steps` | `200000` | 训练总环境步数 |
| `--device` | `cuda` | 计算设备（无 GPU 用 `--device cpu`） |
| `--logdir` | `runs/ppo_walk_<时间戳>` | TensorBoard 日志目录 |
| `--ckptdir` | `checkpoints` | checkpoint 保存目录 |
| `--checkpoint-interval` | `10000` | 每 N 步存一次 `checkpoints/ppo_walk_<step>.zip` |
| `--eval-interval` | `20000` | 每 N 步评估一次（0 关闭） |
| `--eval-episodes` | `10` | 每次评估跑几集（取平均） |
| `--resume <zip>` | 无 | 从已有模型继续训练 |
| `--seed` | 无 | 随机种子 |
| `--dry-run` | 关 | 只解析参数，不启动训练 |

示例：

```bash
# 指定设备与日志目录
python train_ppo.py --total-steps 200000 --device cuda --logdir runs/my_run

# 从第 10 万步的 checkpoint 继续，再训到 20 万步
python train_ppo.py --total-steps 200000 --resume checkpoints/ppo_walk_100000.zip

# CPU 小规模冒烟测试
python train_ppo.py --total-steps 2048 --device cpu --eval-interval 0
```

训练过程中控制台会周期性打印：

```
【训练】步数=20000 奖励=...
【评估】步数=20000 平均奖励=xx.xxx 平均存活步数=xx.x (共 10 集)
```

产物：

- TensorBoard 日志：`runs/ppo_walk_<时间戳>/`
- checkpoint：`checkpoints/ppo_walk_10000.zip`、`ppo_walk_20000.zip` …
- 最终模型：`checkpoints/ppo_walk_final.zip`

---

## 3. 怎么看训练曲线

```bash
tensorboard --logdir runs
```

浏览器打开 <http://localhost:6006>，关注：

- `rollout/ep_remean` — episode 平均奖励（应稳步上升）
- `rollout/ep_len_mean` — 存活步数（应越来越长）
- `train/approx_kl`、`train/entropy_loss` — 策略更新是否稳定
- `train/value_loss` — 价值函数是否收敛

---

## 4. 怎么回放

```bash
# 基本回放（5 集，打印每集奖励 / 存活步数 / 前进距离）
python play.py --model checkpoints/ppo_walk_final.zip

# 打开渲染窗口看 3 集
python play.py --model checkpoints/ppo_walk_final.zip --episodes 3 --render

# 录制视频
python play.py --model checkpoints/ppo_walk_final.zip --video walk_demo.mp4

# 限制单集最多 500 步
python play.py --model checkpoints/ppo_walk_final.zip --max-steps 500
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--model` | 必填 | 模型 `.zip` 路径 |
| `--episodes` | `5` | 回放集数 |
| `--render` | 关 | 打开仿真渲染窗口 |
| `--video` | 无 | 录制 `rgb_array` 到 mp4 |
| `--device` | `cuda` | 计算设备 |
| `--stochastic` | 关 | 随机采样动作（默认确定性动作） |
| `--max-steps` | 无 | 单集步数上限 |

输出示例：

```
【回放】第 1/5 集：奖励=123.456  存活步数=870  前进距离=3.210 m
【回放】汇总：平均奖励=...  平均存活步数=...  平均前进距离=... m  （共 5 集）
```

---

## 5. 超参数（见 `config.py`）

| 超参数 | 值 | 含义 |
|--------|-----|------|
| `n_steps` | 2048 | 每次策略更新收集的环境步数 |
| `batch_size` | 256 | mini-batch 大小 |
| `n_epochs` | 10 | 每批数据优化轮数 |
| `gamma` | 0.99 | 折扣因子 |
| `gae_lambda` | 0.95 | GAE λ |
| `learning_rate` | 3e-4 | 学习率 |
| `ent_coef` | 0.0 | 熵正则系数 |
| `clip_range` | 0.2 | PPO 裁剪范围 |
| `policy` | `MlpPolicy` | 策略网络类型 |
| `net_arch` | `[256, 256]` | 隐层结构 |

网络与训练过程常量（`OBS_DIM=42`、`ACTION_DIM=12`、`CHECKPOINT_INTERVAL`、`EVAL_INTERVAL` 等）也都在 `config.py`。

---

## 6. 预计训练耗时（RTX 4060）

**瓶颈在 Webots 物理仿真步进，不在 GPU。** PPO 的 GPU 前向/反向相对仿真时间通常可忽略。

| 场景 | 环境步速（约） | 20 万步耗时（约） |
|------|----------------|-------------------|
| Webots 无 GUI + 简单地形 | 50–150 steps/s | **25 分钟 ~ 1.2 小时** |
| Webots 开 GUI / 含视觉渲染 | 15–50 steps/s | 1 ~ 4 小时 |
| 复杂地形 + 每步相机观测 | 5–20 steps/s | 3 ~ 11 小时 |

另外每 2 万步跑 10 集评估会额外占用时间（评估期间训练暂停），checkpoint 写盘开销可忽略。

实用建议：

1. **训练时不要开渲染窗口**（`train_ppo.py` 默认 headless）。
2. 用 `tensorboard` 看曲线判断是否该停：奖励进入平台期即可停，不必死磕 20 万步。
3. 先 `--total-steps 2048 --eval-interval 0` 跑通链路，再放大步数。
4. 若 GPU 显存空闲，可把 `--device` 保持 `cuda`；Webots 进程本身不占 GPU。

---

## 7. 后续如何改奖励 / 观测

### 7.1 改奖励

奖励在 `walk_env.py` 内计算。推荐流程：

1. 在 `config.py` 的 `REWARD_WEIGHTS` 里改权重（前进、姿态、平滑、摔倒惩罚等）。
2. 在 `walk_env.py` 中 `from config import REWARD_WEIGHTS` 并使用同一套权重
   （若 `walk_env.py` 已自带权重，请两边同步，避免训练与文档不一致）。
3. 权重含义见 `config.py` 注释：

   | 权重键 | 作用 |
   |--------|------|
   | `forward_vel` | 前进速度奖励（主目标） |
   | `orientation` | 机身俯仰/横滚稳定 |
   | `height` | 机身高度保持 |
   | `alive_bonus` | 每步存活小奖励 |
   | `action_rate` | 动作变化率惩罚（鼓励平滑） |
   | `torque` | 能耗惩罚 |
   | `fall_penalty` | 摔倒提前终止惩罚 |

4. 常见调参方向：
   - **走不远 / 躺平刷存活** → 提高 `forward_vel`，降低 `alive_bonus`。
   - **动作抖动** → 把 `action_rate` 负得更多（如 `-0.05`）。
   - **老摔** → 提高 `orientation` / `fall_penalty` 的绝对值。

### 7.2 改观测

1. 在 `walk_env.py` 中改观测拼接逻辑。
2. 同步修改 `config.py` 的 `OBS_DIM`（`train_ppo.py` 启动时会做维度自检，不一致会直接报错）。
3. 若维度变化较大，建议重新训练而不是 `--resume`（旧 checkpoint 的网络输入维度不匹配）。

### 7.3 改网络 / 超参数

- 网络宽度：`config.py` 里 `NET_ARCH = [256, 256]`，可改为 `[512, 512, 256]` 等。
- 学习率、`n_steps` 等：直接改 `PPO_PARAMS`。
- 改完无需动 `train_ppo.py`。

---

## 8. 常见问题

**Q: `--dry-run` 报 walk_env 未就绪？**
A: 正常，说明 `walk_env.py` 还没写好或有 import 错误。`--dry-run` 只验证参数解析；正式训练前必须保证 `walk_env.py` 可导入。

**Q: `cuda` 不可用？**
A: 用 `--device cpu`。或安装对应 CUDA 版本的 PyTorch：`pip install torch --index-url https://download.pytorch.org/whl/cu121`。

**Q: 评估时 Webots 打不开第二个实例？**
A: 用 `--eval-interval 0` 关闭定期评估；或修改 `walk_env.py` 支持单实例复用。

**Q: 想接着上次训练？**
A: `python train_ppo.py --total-steps 400000 --resume checkpoints/ppo_walk_200000.zip`，
`--total-steps` 是**累计**目标步数，脚本会自动算出还需再跑多少步。

**Q: 视频录出来是空的 / 写失败？**
A: 确认 `walk_env.py` 的 `render()` 在 `render_mode="rgb_array"` 时返回 `H×W×3` 的 uint8 数组；
并安装 `imageio imageio-ffmpeg` 或 `opencv-python`。
