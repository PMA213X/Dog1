# CAD 外形文件只读盘点

> 盘点日期：2026-10-07
> 使用边界：本目录只作为**外形/碰撞几何候选来源**；质量、惯量、关节、
> 限位和执行器参数不得从这里进入训练资产。

## 1. 文件清单

| 文件 | 格式 | 字节数 | 行数 | SHA256 |
| --- | --- | ---: | ---: | --- |
| `Smart Engines.x_t` | Parasolid 文本交换文件 | 14374607 | 175589 | `dc326f9545a9e3725deebbee8dadc0117437607c321e4f3223d3f53be80724cb` |
| `steadywin_v3.STEP` | ISO-10303-21 STEP AP203 | 37759291 | 433971 | `d91467b9ae9a3fc73f14aa7e0869685dac8425b572b5d91514317bbb0bad0135` |

目录内只有上述 2 个几何交换文件；没有原生 CAD 装配/零件文件、工程图、
PDF 图纸、脚本或网格文件。

## 2. STEP 盘点

- 头信息：`STEP AP203`，导出器 `SwSTEP 2.0 / SolidWorks 2015`，
  时间 `2019-11-19T06:51:19`，根产品名 `steadywin_v3`。
- 单位：长度明确为毫米 `SI_UNIT(.MILLI., .METRE.)`，平面角为弧度，
  立体角为球面度。
- 产品/装配：128 个 `PRODUCT`，其中 123 个通用零件名
  `Smart Engines.sldasm-Part-1..123`，另有 `steadywin_v3`、
  `3_BODY`、`5887_1`、`GEAR_1_BODY`、`GEAR_2_BODY`；
  936 个装配实例变换。
- B-rep 统计：60 个 `MANIFOLD_SOLID_BREP`、67 个
  `MANIFOLD_SURFACE_SHAPE_REPRESENTATION`、60 个闭合壳、
  81 个开放壳、7188 个高级面。
- 未发现工程图、网格化表示、材料、密度、质量、惯量、运动副或关节实体。
- 装配坐标变换存在，但没有 YoboGo 的 `base/hip/thigh/shank/foot`、
  四腿名称、关节轴或零位语义，不能直接自动映射为 URDF/USD。

## 3. Parasolid 盘点

- 头信息：`FORMAT=text`、`APPL=unigraphics`、`GUISE=transmit`，
  内部原文件名 `C:\Users\Administrator\Desktop\GG\GG.x_t`，
  日期 `2-sep-2019`。
- 可读头没有声明长度单位，必须由 CAD 内核导入后确认；不得猜测为
  米或毫米后直接写入 Isaac。
- 文本中存在 127 处 `Kg/Cu M` 密度属性，以及 `3_BODY`、
  `GEAR_1_BODY`、`GEAR_2_BODY` 等 body 标识；未找到显式
  `MASS`、`INERTIA`、`JOINT`、装配语义或机器人坐标系定义。
- 密度字符串不是可用的逐 link 质量/惯量，不能用于动力学验收。

## 4. 可提取性与缺口

| 项目 | 结论 |
| --- | --- |
| 几何提取 | STEP 是可由 CAD 内核读取的 B-rep；Parasolid 需兼容内核。本次只读盘点，未执行转换或生成脚本 |
| 外形/碰撞 | 可作为后续拆分机身、髋、大腿、小腿和足端几何的唯一 CAD 输入 |
| 单位 | STEP 已确认 mm；Parasolid 待导入确认 |
| 坐标系 | 有装配变换，但没有机器人语义坐标系，必须人工对齐 YoboGo 腿序与方向 |
| 材料/质量/惯量 | 不可用；STEP 无相关实体，Parasolid 只有密度线索 |
| 关节/限位/执行器 | 不可用；文件中没有运动副和控制语义 |
| 工程图 | 无 |

因此，CAD 可以补齐**可信外形和碰撞几何**，但仍需从
`YoboGo-control/` 获取控制事实，从 MIT 官方网络 URDF 获取运动学和
逐 link 物理参考，再由用户确认最终合成方案。
