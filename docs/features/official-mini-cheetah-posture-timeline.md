# 官方 Mini Cheetah 默认姿态时间线

## 能力边界

本功能提供独立的平地姿态测试 world/controller：

- world：`official-mini-cheetah/worlds/posture_timeline_test.wbt`
- controller：`official-mini-cheetah/controllers/posture_timeline/`
- 测试 world 不绑定 `acceptance_supervisor`，不写被动验收 artifacts；
  主 Robot 的 `supervisor TRUE` 仅用于读取自身位姿、接触点和几何状态。
- 只覆盖 Webots R2025a 平地固定时间线，不是实机控制、步态规划或被动验收替代品。

## 姿态目标与推导

默认姿态来自用户参考图的屈膝、髋部外撑站姿；左右腿严格镜像。目标顺序为
`fr/fl/hr/hl × abd/hip/kn`：

| 姿态 | abad（右/左） | hip | knee |
|---|---:|---:|---:|
| `DEFAULT_CROUCH` | `-0.15 / +0.15` | `-0.80` | `1.60` |
| `SLIGHTLY_EXTENDED` | `-0.15 / +0.15` | `-0.55` | `1.15` |

推导依据为官方 `MiniCheetah.h` 的 `0.062/0.209/0.195 m` 腿长、官方 abad/hip
轴方向，以及 `SpineBoard.h` 的 abad `±1.5 rad`、hip `±5.0 rad` 软停。
官方 knee 没有软停，因此 `-2.7..2.7 rad` 只作为姿态软件包络，不是机械限位。
默认膝内角约 `88.33°`，伸展态约 `114.11°`，两者均明显弯曲。

12 路目标全部经过显式限位和 `0.75 rad/s` 相邻帧变化率限制。主动普通打开
`flat_ground_teleop.wbt` 后默认目标为 `DEFAULT_CROUCH`；姿态测试控制器使用
Webots 位置控制跟踪该限速目标。

## 固定时间线

| 时间 | 阶段 |
|---|---|
| 0–10 s | 保持 `DEFAULT_CROUCH` |
| 10–12 s | smoothstep 平滑过渡到 `SLIGHTLY_EXTENDED` |
| 12–14 s | 保持 `SLIGHTLY_EXTENDED` |
| 14–16 s | smoothstep 平滑返回 `DEFAULT_CROUCH` |

`0/10/12/14/16 s` 边界目标连续，边界一阶导为零；前 3 秒仅作为出生间隙和
四足落地沉降，不计入默认阶段高度稳定跨度。

## 出生高度依据

测试 world 和普通主动 world 的 Robot 出生高度从 `0.40 m` 调整为 `0.45 m`。
在零关节几何下，`0.45 m` 出生高度对应约 `0.061 m` 初始间隙；默认姿态几何
对应约 `0.178 m` 目标间隙。该调整只给四足自然下落留出间隙，不改变碰撞体、
质量、COM、惯量、DAE 或被动 world。

## 测试与运行

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B \
  official-mini-cheetah/tests/test_posture_timeline.py -v

PYTHONDONTWRITEBYTECODE=1 python3 -B \
  official-mini-cheetah/tests/test_flat_ground_teleop.py -v

PYTHONDONTWRITEBYTECODE=1 python3 -B \
  official-mini-cheetah/tests/test_posture_timeline.py WorldStaticTests -v
```

三次测试均退出码 `0`。Webots 批处理命令：

```bash
DISPLAY=:0 timeout 60s /usr/local/webots/webots \
  --batch --stdout --stderr --mode=fast --no-rendering \
  official-mini-cheetah/worlds/posture_timeline_test.wbt
```

最终退出码 `0`，日志末尾出现 `POSTURE_RESULT {"status": "PASS", ...}`。

## 运行命令

```bash
DISPLAY=:0 /usr/local/webots/webots \
  --mode=realtime --stdout --stderr \
  official-mini-cheetah/worlds/posture_timeline_test.wbt
```

## 证据

- 批处理日志：`logs/posture_timeline_20261003/webots_batch_accepted.log`
- 结果日志：`logs/posture_timeline_20261003/posture_result.log`
- 结果摘要：16 仿真秒、33 个有限样本、32 个接触样本、最大穿地
  `0.000309 m`、默认阶段高度跨度 `2.32e-14 m`、伸展阶段高度跨度
  `0.000593 m`、最大倾角 `0.001715 rad`、最终目标误差 `0.005965 rad`、
  无 `ERROR/WARNING/NaN/Inf`。

## 已知限制

- 仅验证平地、固定时间线和四足投影/接触状态；
- 未验证复杂地形、步态鲁棒性、实体手柄或实机执行；
- 主动 world 的 `springConstant=0` 是姿态控制所需的主动测试配置，被动
  `official_flat_ground_test.wbt` 仍保持官方 `springConstant=100` 和
  `controller "<none>"`。
