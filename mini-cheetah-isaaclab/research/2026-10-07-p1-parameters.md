# Mini Cheetah / YoboGo P1 参数证据表

> 调研日期：2026-10-07  
> 来源优先级：`YoboGo-control` > MIT 官方/高可信网络资料 > 用户可信 CAD 外形。  
> `official-mini-cheetah` 与旧 Webots 除 4 个官方 DAE 外未参与本表；CAD 仅可用于
> 几何核对，不能替代动力学、控制或接触实测。

## 1. 结论总表

| 参数 | 可确认内容 | 决策 | 可信等级 | data-manifest |
| --- | --- | --- | --- | --- |
| 机械硬限位 | 未找到可靠的 hip/abad/knee 机械 stop 角度 | **未找到**；不得把软件 softstop、URDF limit 或线缆最大角度写成机械硬限位 | A：本地与 MIT 官方源均无 | 仅登记“待实测/待 CAD” |
| 软件 softstop | MIT `SpineBoard`：abad `[-1.5, 1.5] rad`、hip `[-5.0, 5.0] rad`；knee 当前无 softstop；越界 PD 为 `Kp=100, Kd=0.4` | 仅可作为 MIT 仿真/板级软限位参考，不覆盖 YoboGo | A：MIT 官方仓库 | 可写入“网络 softstop 参考”，不可写硬限位 |
| 足端碰撞半径 | ORCAgym 足球 `r=0.0202 m`；Rapid Locomotion URDF 为 `r=0.0175 m` | 沿用现有 manifest 的 `0.0202 m` 网络碰撞候选，并记录 `0.0175 m` 冲突；最终以 CAD 实测为准 | A：MIT 两个高可信仓库 | 可写候选，标记冲突/CAD 待核 |
| 足端材料/物理摩擦 | 未找到官方材料牌号、弹性模量或实测接触摩擦系数 | **未找到**；`mu` 只能标成仿真地面参数 | A/B：网络搜索无结果 | 只能列为待实测 |
| 仿真地面摩擦 | YoboGo `ground-plane mu=0.5, restitution=0.0`；mesh/box `mu=0.7` | 可写“本地仿真配置”，不能冒充实机足地摩擦 | A：本地 | 可写仿真参数 |
| 执行器延迟 | 未找到可靠的执行器闭环延迟数值；仅确认控制/SPI 周期 `0.002 s` | **未找到**；周期不等于延迟 | A：本地 | 待台架阶跃/频响实测 |
| 速度饱和 | ORCAgym：HAA/HFE `41 rad/s`、KFE `26.8 rad/s`；YoboGo 电机指令范围 `-30..30 rad/s` | 只能作为网络/电机指令参考，不是 YoboGo 关节机械速度硬限位 | A：MIT ORCAgym；B：本地说明书 | 可写参考值并标“非实机硬限位” |
| 干摩擦/阻尼 | MIT 模型：dry friction `0.2 N·m`、damping `0.01 N·m·s/rad` | 可写 MIT 模型参考，不可当作 YoboGo 实测值 | A：MIT Cheetah-Software | 可写参考，实测仍缺 |
| IMU 安装位姿 | YoboGo 未给安装位姿；Rapid Locomotion URDF 为固定关节 `rpy=0, xyz=0` | **实机位姿未找到**；只能记录网络模型假设 | B：高可信 MIT RL 仓库 | 待 CAD/实机测量 |
| IMU 噪声 | YoboGo 仿真：accel `0.005`、gyro `0.005`、quat `0.003`；状态估计器 process noise `0.02` | 可写“仿真噪声参数”，物理单位/随机过程仍需确认 | A：本地 | 可写仿真参数，标单位待确认 |
| TCP/LCM 延迟 | TCP 状态发送周期 `50 ms`；LCM/SPI/控制周期 `2 ms`；未找到实测端到端延迟 | **端到端延迟未找到**；不得用周期代替延迟 | A：本地 | 周期可写，延迟待实测 |
| 关节反馈滤波 | MIT `LegController::updateData` 直接复制 `q/qd`；未找到关节反馈滤波参数 | **未找到**；不得把 `RPC_filter` 当关节滤波 | A：MIT/本地 | 待实测或代码路径确认 |

## 2. 机械限位：softstop 与 hard stop 分离

### 2.1 已确认的软件 softstop

标题：**Cheetah-Software — SpineBoard.h**  
URL：<https://github.com/mit-biomimetics/Cheetah-Software/blob/master/common/include/SimUtilities/SpineBoard.h#L70-L76>  
原文：

```cpp
const float q_limit_p[3] = {1.5f, 5.0f, 0.f};
const float q_limit_n[3] = {-1.5f, -5.0f, 0.f};
const float kp_softstop = 100.f;
const float kd_softstop = 0.4f;
```

标题：**Cheetah-Software — SpineBoard.cpp**  
URL：<https://github.com/mit-biomimetics/Cheetah-Software/blob/master/common/src/SimUtilities/SpineBoard.cpp#L83-L122>  
原文明确写 `Check abad softstop`、`Check hip softstop`、`No knee softstop right now`。

可信等级：**A（MIT 官方源码）**。单位由关节状态与 MIT 控制约定确定为 rad。
结论：这是软限位 PD，不是机械 stop。

标题：**Question about softstop · Issue #70**  
URL：<https://github.com/mit-biomimetics/Cheetah-Software/issues/70>  
原文：`hip soft limitation is [-5.0, 5.0], as the same as the hip limitation in actual robot`。  
可信等级：**A-（官方仓库 issue，但不是机械图纸）**。它只说明软件限制，仍不能证明机械 stop。

### 2.2 未确认项

- YoboGo `rt_spi.cpp` 只有力矩限幅 `17/17/26 N·m`，没有 `q_min/q_max` 或机械 stop。
- ORCAgym URDF 的 `lower/upper` 是仿真 DOF limit，不是机械 stop。
- 搜索到 ICRA 论文镜像提到 hip 受线缆限制约 `±270°`，但没有从 MIT 可验证原文/图纸确认，
  且“线缆允许范围”也不等于机械 stop，因此不采纳。
- **最终状态：hip/abad/knee 机械硬限位与 stop 角度均未找到，必须实测或取得官方工程图。**

## 3. 足端接触

### 3.1 可用几何

标题：**ORCAgym — mini_cheetah_rotor.urdf**  
URL：<https://github.com/mit-biomimetics/ORCAgym/blob/5aaff694ae5f1c31e08040d59787f2cca4c5cfe0/resources/robots/mini_cheetah/urdf/mini_cheetah_rotor.urdf>  
原文：

```xml
<origin xyz="0 0.0 0.024"/>
<geometry><sphere radius="0.0202"/></geometry>
<mass value="0.01"/>
```

可信等级：**A（MIT Biomimetic Robotics Lab ORCAgym，BSD-3-Clause）**。

冲突参考：

- **Rapid Locomotion via Reinforcement Learning**
  <https://github.com/Improbable-AI/rapid-locomotion-rl/blob/main/resources/robots/mini_cheetah/urdf/mini_cheetah_simple.urdf>
  为 `sphere radius="0.0175"`，可信等级 A-，但与 ORCAgym 不同。
- 因此 `0.0202 m` 继续作为 manifest 候选，`0.0175 m` 记录为冲突；CAD 球径核对后再冻结。

### 3.2 材料与摩擦未找到

标题：**SearXNG 查询：MIT Mini Cheetah foot material rubber friction coefficient official**  
结果：`0 results`。  
结论：没有找到官方足端材料牌号、超弹性/橡胶参数或实测摩擦系数。

本地仅能确认仿真地面：

- `YoboGo-control/robot-software/config/default-terrain.yaml`：ground-plane
  `mu=0.5, restitution=0.0`；mesh/box `mu=0.7`。
- 这些是地面碰撞配置，不是足端材料本构，也不是实机足地摩擦。

## 4. 执行器延迟、速度与摩擦

### 4.1 延迟：未找到

本地已确认：

- `controller_dt=0.002 s`（500 Hz）；
- SPI 任务周期 `0.002 s`（500 Hz）；
- `RobotRunner::run()` 注释为定时 `2 ms`。

但上述都是**采样/控制周期**，不是“命令到力矩响应”的延迟。
YoboGo 与 MIT Cheetah-Software 均未给出 actuator delay 数值，
Rapid Locomotion 仓库也未找到 `delay/latency` 参数。
**执行器延迟必须通过阶跃或正弦频响实测，当前只能标记待实测。**

### 4.2 速度参考

标题：**ORCAgym — mini_cheetah_rotor.urdf**  
URL：<https://github.com/mit-biomimetics/ORCAgym/blob/5aaff694ae5f1c31e08040d59787f2cca4c5cfe0/resources/robots/mini_cheetah/urdf/mini_cheetah_rotor.urdf>

- HAA：`velocity="41"` rad/s；
- HFE：`velocity="41"` rad/s；
- KFE：`velocity="26.8"` rad/s。

本地说明书第 6 节电机命令字段给出 `-30 rad/s ~ 30 rad/s`。
这是电机指令表示范围，不是关节机械速度 stop。两者都不写成 YoboGo 硬速度限位。

### 4.3 干摩擦/阻尼：MIT 模型参考

标题：**Cheetah-Software — MiniCheetah.h**  
URL：<https://github.com/mit-biomimetics/Cheetah-Software/blob/master/common/include/Dynamics/MiniCheetah.h#L41-L46>  
原文：`_jointDamping=.01`、`_jointDryFriction=.2`。

标题：**Cheetah-Software — ActuatorModel.h**  
URL：<https://github.com/mit-biomimetics/Cheetah-Software/blob/master/common/include/Dynamics/ActuatorModel.h#L22-L30>  
原文单位：

- damping：`Nm/(rad/sec)`；
- dryFriction：`Nm`。

可信等级：**A（MIT 官方模型）**。  
结论：`0.01 N·m·s/rad` 与 `0.2 N·m` 可作为 MIT 仿真参考，但 YoboGo 实机关节干摩擦仍需台架实测。

## 5. IMU

### 5.1 安装位姿

标题：**Rapid Locomotion — mini_cheetah.urdf**  
URL：<https://github.com/Improbable-AI/rapid-locomotion-rl/blob/main/resources/robots/mini_cheetah/urdf/mini_cheetah.urdf>

```xml
<joint name="imu_joint" type="fixed">
  <origin rpy="0 0 0" xyz="0 0 0"/>
</joint>
```

可信等级：**A-（MIT Improbable AI Lab 项目模型）**。  
它只证明该网络模型把 IMU 放在 trunk 原点；YoboGo 实机安装位姿未找到，不能直接采用。

### 5.2 仿真噪声

本地 `YoboGo-control/robot-software/config/simulator-defaults.yaml`：

```yaml
vectornav_imu_accelerometer_noise: 0.005
vectornav_imu_gyro_noise         : 0.005
vectornav_imu_quat_noise         : 0.003
```

本地 `mini-cheetah-defaults.yaml`：

```yaml
imu_process_noise_position : 0.02
imu_process_noise_velocity : 0.02
```

可信等级：**A（YoboGo 本地配置）**。  
这些是状态估计/仿真参数，文件未明确说明完整物理单位和噪声分布，
因此只登记原值，不能声称是 IMU datasheet 噪声。

YoboGo `rt_vectornav.cpp` 只读取滤波配置；`accelWindowSize=4` 的写入示例被注释，
所以**没有确认生效的 IMU 软件滤波参数**。

## 6. TCP / LCM 延迟与关节反馈滤波

### 6.1 周期不是延迟

- TCP：`PeriodicFunction(..., 0.05, ...)` 与 `usleep(50000)`，即每 `50 ms` 发送一次状态。
- LCM/SPI/控制：`2 ms` 周期，并发布 `leg_control_command/data`、`state_estimator`、`spi_*`。
- LCM 官方文档只宣称 high-bandwidth/low latency，没有给出本机器人端到端数值：
  <http://lcm-proj.github.io/lcm/>。

**TCP/LCM 端到端延迟、抖动、丢包率均未找到，必须在 `10.0.0.30 <-> 10.0.0.34`
链路上用时间戳实测。**

### 6.2 关节反馈滤波：未找到

标题：**Cheetah-Software — LegController.cpp**  
URL：<https://github.com/mit-biomimetics/Cheetah-Software/blob/master/common/src/Controllers/LegController.cpp#L109-L175>  
原文中 `q`、`qd` 直接从板卡数据赋给 `datas[leg]`，没有低通/中值/卡尔曼滤波。

YoboGo `mc-mit-ctrl-user-parameters.yaml` 的
`RPC_filter=[0.5, 0.1, 0]` 是 RPC 预测滤波参数，不是关节编码器反馈滤波。

**关节位置/速度反馈滤波参数未找到；训练资产默认不添加额外滤波，
实机部署前通过阶跃响应实测决定。**

## 7. 搜索记录

使用 `mcp__searxng_search__search_web` 的主要查询：

1. `MIT Mini Cheetah robot joint range of motion hip knee`
2. `Mini Cheetah URDF joint limits lower upper`
3. `MIT Mini Cheetah soft stop joint limit mechanical stop official`
4. `MIT Cheetah Software MiniCheetah joint dry friction 0.2 damping 0.01`
5. `Mini Cheetah foot ball radius friction material URDF official MIT`
6. `MIT Mini Cheetah foot material rubber friction coefficient official`
7. `Mini Cheetah actuator delay domain randomization rapid locomotion MIT`
8. `MIT Cheetah Software Mini Cheetah IMU mounting position noise official`
9. `LCM lightweight communications marshalling measured latency UDP MIT Cheetah`
10. `MIT Cheetah Software LegController joint velocity filter feedback`
11. `mit-biomimetics ORCAgym mini_cheetah_rotor URDF velocity 41 26.8`

未把 `official-mini-cheetah`/旧 Webots 数据混入本表。

## 8. 待实测清单

1. hip/abad/knee 机械硬 stop 与安全工作区；
2. 足端材料、球径 CAD 实测、足地静/动摩擦与恢复系数；
3. 执行器阶跃延迟、闭环带宽、速度饱和；
4. 关节干摩擦、黏性阻尼、迟滞；
5. 实机 IMU 安装 `xyz/rpy`、噪声分布与生效滤波；
6. TCP/LCM 端到端延迟、抖动和丢包；
7. 关节反馈滤波是否存在及其截止频率。
