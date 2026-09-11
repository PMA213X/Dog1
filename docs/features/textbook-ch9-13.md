# 四足仿生机器人基本原理及开发教程 —— 第9-13章内容提取报告

> **教材信息**：《四足仿生机器人基本原理及开发教程》
> **作者**：李彬（智能人机交互创新团队）
> **提取范围**：第9章～第13章（共5个PPTX文件，约65MB）
> **提取日期**：2026年9月

---

# 第9章 四足机器人结构设计与安装流程

## 9.1 四足机器人结构设计

### 9.1.1 整体结构概述

机器人物理平台经过2次更新迭代：
- **第一款**：膝关节自由度和电机之间为皮带连接，电机输出力矩较大时容易造成皮带打滑，需要重新调整电机零位。
- **第二款**（本书所用）：膝关节处采用**连杆连接方式**，其他机构和硬件设计与第一款一致，解决了电机打滑、零位调整问题。

整体结构主要由以下部分组成：
1. 尾部壳体
2. 头部壳体及电机外包固定单元
3. 躯干腔体
4. 四条腿（每条腿3个动力单元）

每条腿有3个动力单元，利用安装的3个电机分别控制：
- **大腿侧摆**（侧摆电机）
- **大腿俯仰**（大腿电机）
- **小腿的伸缩**（小腿电机）

### 9.1.2 整体结构示意图（图9.1）

标注部件：
1. 机器人小腿
2. 可拆卸足端
3. 脚底缓冲
4. 防撞保护盖
5. 底部相机
6. 面部相机
7. 显示装置
8. 电机保护
9. 尾部保护盖
10. 大腿电机
11. 小腿电机
12. 小腿关节销钉
13. 小腿连杆销钉
14. 面部端盖

### 9.1.3 剖视图与关节动力图

- **图9.2**：整机侧面剖视图
- **图9.3**：整机横向剖视图
- **图9.4**：关节动力图

### 9.1.4 腿部连杆机构

机器人腿部采用**连杆机构**，其特点：
- 元素之间的接触是面接触，磨损相应较少
- 构成这些运动副的元素加工比较简单
- 且易得到较高的制造精度
- 能起到增力或扩大行程的作用

**设计要点**：采用**平行四边形机构**来传递动力，保证对边的边长相等。

**运动学关系**：
- 小腿转速与电机转速关系：小腿转速是电机转速的 **1/减速比** 倍
- 扭矩关系：减速器输出扭矩 = 输入扭矩/减速比
- 小腿某点的扭矩是电机的 **减速比** 倍

**机械限位**：在小腿伸缩到极限时，需要一个机械限位，防止四边形连杆机构在超过极限位置时卡死。解决办法是增加限位单元，在机器人外侧壳体上铸造一块凸起挡块。

### 9.1.5 足底结构设计（图9.8）

为了降低机器人运动时的噪声，实现足-地接触时的机械缓冲，开发了机器人足底结构：
- 足底骨架采用**中空架构**（材料可以为塑胶或者金属）
- 保证足底具有尽量平坦且面积大的弧面
- 足够大的面接触可以保证机器人稳定站立
- 足底缓冲半球采用由**橡胶、硅胶**等材料制成的柔性单元

### 9.1.6 动力单元（图9.9）

单个关节引擎由**电机、减速机和控制器**组成。

工作原理：
- 外转子及内部定子三相无刷线圈组成基本动力单元
- 在驱动电路板的控制下完成对关节的位置、速度、力矩混合控制
- 增大扭矩后的力通过法兰盘输出
- 输出单元采用**薄壁交叉滚子轴承**完成，保护内部器件不受冲击

### 9.1.7 分电板设计（图9.10）

为了批量化生产：
- 将前后腿电机固定在一块底板上
- 设置通信线
- 两个侧摆电机同时贯穿PCB板的安装梁电机背部的螺丝孔
- 将三者组合起来形成一个**前部模组**
- 大幅减少机身内线束，方便装配和维修

---

## 9.2 四足机器人安装流程

### 9.2.1 电机的装配（9个步骤）

#### (1) 装配转子

**零配件**：钢圈、转子支架、磁铁、磁钢胶（乐泰 AA326）

**步骤**：
1. 将酒精喷洒在钢圈内表面，使用厨房用纸将内表面油污擦干净
2. 将转子支架嵌入钢圈内并压紧
3. 在转子支架垭口处涂少量磁钢胶
4. 将分离好的磁铁插入垭口（注意两个相邻磁铁之间侧面相斥）
5. 将装配好的转子磁铁上多余的胶水擦除
6. 将转子置于**60℃加热保温箱中保温30分钟**（或在常温下静置4小时）

**磁铁安装注意事项**：
1. 检查转子支架垭口与磁铁尺寸是否匹配
2. 转子磁铁安装前需要提前划线，不同批次划线时需要先检查磁铁的方向，且不同批次划线时不得使用同一颜色油漆笔
3. 涂胶过程中避免胶水粘在备用磁铁上
4. 磁铁要安装到位（竖直装入，安装到底，与钢圈贴合紧密）
5. 安装完成后手工检查磁铁是否有突起
6. 检查无误后，将安装好的转子放置在无铁屑等杂质的干净位置上

**太阳轮压装**：使用工装治具和压力机将太阳轮压装在转子支架上，压装之前需要在太阳轮底部和转子支架的太阳轮孔内涂抹适量**固持胶（乐泰 648）**。

#### (2) 装配定子线圈

**所需材料**：电动螺丝刀、螺丝、AB胶、铜线圈、转子、轴承、电机外壳

**步骤**：
1. 试装定子线圈是否到位：将合格的定子线圈放入定子支架，观察线圈是否在定子支架上晃动
2. 检查合格后，取出定子线圈并在其内表面涂上**AB胶水（ERGO 9900）**
3. 将线圈粘在定子支架上
4. 使用万用表检查定子线圈的绝缘性

#### (3) 装配定/转子

1. 在深沟球轴承表面涂抹少量润滑脂
2. 先将轴承装入定子支架上（检查轴承安装后是否运转顺畅）
3. 将已装好的转子扣置在已装好的定子上
4. 使用夹具治具和台钻将转子压进转子轴承内
5. 压到位后检查转子是否运转良好且无卡顿现象

#### (4) 装配齿圈

1. 清理齿圈外表面
2. 在齿圈外表面与电机端盖齿圈孔处涂抹**固持胶（乐泰648）**
3. 使用工装治具和台钻将齿圈压进电机端盖
4. 压装完成后，在齿圈和电机端盖上用油漆笔画一条线（便于后期检查松动）

#### (5) 装配减速器

**材料**：小轴承、交叉滚子轴承、行星齿轮轴、不锈钢垫片、滚针轴承

**步骤**：
1. 将小轴承外圈涂上润滑脂，使用工装将小轴承压进行星架下的轴承孔中
2. 检查交叉滚子轴承是否嵌入良好
3. 在交叉滚子轴承内表面涂抹润滑脂
4. 依次安装行星齿轮轴、不锈钢垫片、滚针轴承于行星架上
5. 拧紧固定螺钉（螺钉上需要涂抹螺纹胶）

#### (6) 装配电机端盖

1. 将减速器部分装在电机端盖上（要缓慢插入）
2. 在电机端盖螺纹上打上**高强度螺纹胶（乐泰277）**
3. 用三爪卡盘拧紧轴承、压紧盖
4. 在行星架上涂抹**磁钢胶（乐泰AA326）**
5. 使用工具判断霍尔磁铁方向后，将其粘在行星架上
6. 使用工具给减速器注入**粘度大的润滑脂**

#### (7) 装配电机机械

1. 将中心轴插入太阳轮孔中
2. 将定/转子部分和减速器部分装配到一起
3. 在螺钉处涂抹**低强度螺纹胶（乐泰 426）**，并拧紧螺钉
4. 用手转动电机输出盘，检查电机是否运转顺畅无噪声
5. 使用磁铁胶在转子支架另一端粘贴编码器圆形磁铁

#### (8) 装配电机驱动板

1. 再次检测电机线圈是否绝缘
2. 在驱动板下的螺丝孔处垫**尼龙垫片**
3. 用螺钉紧固驱动板
4. 按照接线定义的要求焊接驱动板引线
5. 注意：大腿电机的引线短，小腿电机的引线长，侧摆电机的驱动板和腿部电机的驱动板不同

#### (9) 电机初始化设置

**电机电源接口**：供电电压为**12～24V**

**电机调试口引脚定义**：
| 引脚 | 功能 |
|------|------|
| GND | 接地端 |
| TXD (PA2) | 指令发送端 |
| RXD (PA3) | 指令接收端 |
| SWCLK | 时钟端 |
| SWDIO | 数据端 |
| RST | 复位端 |

**电机配置所需软件环境**：
- 操作系统：Windows 10
- 编译器：Keil 5.27及以上版本
- 运行环境：Keil.STM32F4xx_DFP.2.14.0.pack

**推荐使用**：ST-LINK串口下载设备（USB转TTL）

**电机初始化配置过程**：

1. **确认电机版本**：通过计算机连接串口到电机，打开串口调试软件（推荐使用串口调试助手），将波特率设置为**921600**，输入/输出设置为ASCII码

2. **矫正编码器**：将关节摆动到工作空间的中间值外，在串口调试软件窗口输入指令 `c` 并发送，此时电机会自动运行矫正。矫正完成后断电重启。

3. **取霍尔最小值**：输入指令 `r` 并发送，此时界面会打印霍尔信息。D0、D1、D2、D3、D4、D5分别代表6个霍尔元件。持续转动关节，直到数值变化不大后，再输入指令 `u` 并发送，然后断电重启。

4. **取霍尔中间值**：输入指令 `t` 并发送。转动关节，使得任意两个相邻的霍尔元件的值相等，再输入指令 `u` 并发送，然后断电重启。

5. **零位初始化**：

**方法1**（串口指令）：
- 将腿部摆动到零位位姿：髋关节横向水平，大腿纵向水平，小腿朝上摆动到关节限位处（后腿则小腿反向摆到关节限位处）
- 在串口调试软件上输入指令 `z` 并发送
- 该命令需多次执行，直到返回值变化不大为止
- 重新上电并输入命令 `p`，观看输出的角度信息是否正确

**方法2**（电机调零板）：
- 调零板使用USB转接线供电，通过CAN口连接线实现与电机CAN口通信
- 电机上电，按下调零板复位按钮RESET
- 选择电机调零选项（SET_MOTOR_ZERO 2）
- 转动调零板黑色旋钮，找到要调零的电机ID
- 按下KEY1键，实现所选电机的零位调整

6. **设置关节通信序号**：
- 关节通过CAN口进行通信，一个腿使用一个CAN口
- 序号划分规则：髋关节=1、大腿=2、小腿=3
- 在调试软件菜单界面输入命令 `s`，进入配置界面
- 输入命令 `i` 并发送（代表CAN ID）
- 输入序号（如 `01`）并发送

### 9.2.2 腿部机构的装配

#### 1. 装配和胶粘足底

**所需材料**：AB胶、壁球、剪刀、尼龙脚支架、0.1g电子秤、一次性纸杯、皱纹胶带

**步骤**：
1. 将壁球按图示剪开一个缺口备用
2. 准备好电子秤，打开AB胶的盖子，取一次性纸杯放在电子秤上后去皮清零
3. 倒入一定量的A溶液得到读数a，再去皮清零
4. 倒入B溶液得到读数b，**注意b的数值必须是a的2倍且a加b的和是6的整数倍**
5. 将混合好的溶液从电子秤上取下使用玻璃棒快速搅拌
6. 取一个剪开的壁球放在电子秤上并去皮清零
7. 将搅拌好的胶（6克）从壁球的剪口处倒入
8. 取下壁球，在剪口处插入脚支架
9. 用皱纹纸将壁球和脚支架绕成十字状封好
10. 按压出两个黄豆大小的9900胶，搅拌均匀后放在脚支架的凹槽内
11. 将铝件插入脚支架的凹槽内
12. 放置**2个小时**后即可使用

#### 2. 装配大带轮

**材料**：大带轮、大挡片、深沟球轴承（尺寸：17mm×23mm×4mm）、轴套、花型圆头M3×6螺钉、乐泰277胶

**步骤**：
1. 在大带轮的一侧孔内涂抹乐泰277胶
2. 将大挡片穿过大带轮的凸起处对好螺纹孔
3. 将花型圆头M3×6的螺钉拧入螺纹孔紧定
4. 将轴套插入大带轮的中心孔
5. 随后放入深沟球轴承，并使用压力机压装到位

#### 3. 装配大带轮/小腿

**材料**：小腿、装配好的大带轮、皮带、乐泰426胶、沉头花型螺钉M3×8

将加强皮带放入小腿插口里，然后将大齿轮插入小腿对准螺纹孔，涂抹乐泰426胶后拧入螺钉。

#### 4. 装配惰轮

**材料**：H形架、惰轮轴、垫片、惰轮轴承（滚针轴承4×8×8）、卡簧C4、卡簧钳

#### 5. 装配大腿/小腿

分为**皮带腿**和**连杆腿**两部分：

**皮带腿安装材料**：大腿连接件、大腿内壳、大腿外壳、装带轮的小腿、惰轮组件、惰轮轴套、花型盘头螺钉M3×16、盘头螺钉M3×22、花型沉头螺钉M4×6、乐泰胶426

**连杆腿装配**：
- 使用工装和手动压力机把**滚针轴承**压入连杆孔
- 将**深沟球轴承**压入轴承座
- 中间连杆和小腿连杆按顺序装配
- 连杆大腿：通过压力机把销钉轴穿过大腿外侧钣金和安装好的小腿

**减速器与腿的装配**：
- 遥杆装配：将遥杆上的定位盲孔与减速器的定位销配合安装
- 减速器上的六个螺纹孔要涂抹**乐泰 277胶水**
- 放置沉头锥形M4×8螺钉并依次拧紧

### 9.2.3 关节模组的装配

#### 侧摆关节模组
- 将侧摆电机装上CAN线后安装在支撑板上
- 将电机连接架固定在侧摆电机输出盘上（螺钉需涂抹**乐泰277**）
- 装上塑料过线板

#### 大腿电机关节模组
- 将驱动板保护盖安装在大腿电机输出盘后部
- 在螺钉处涂抹螺纹胶

#### 小腿电机模组
- 将下齿轮盖固定在小腿电机输出盘上（涂抹**乐泰 277**）
- 将销钉压入小齿轮
- 将小齿轮固定在下齿轮盖上，把上齿轮盖装在小齿轮上
- 最后将小腿电机和大腿电机固定在一起
- **注意**：此处螺钉应该严格按照规定的大小使用，否则可能造成电路短路

### 9.2.4 分电板装配

- 安装分电板及布置前/后部分的走线
- 需要遵循**短线从上部走、长线从下部走**的原则
- 连线过程中需要认真对应相应的接口，避免接错位置
- 安装分电板时，避免压到或损坏接线

### 9.2.5 躯干的装配

**装配材料**：电池、UP board板、电源板、铜柱等

**步骤**：
1. 确定电池的安装位置（按UP board板的位置确定正反方向）
2. 在电池安装位置处先贴一层绝缘胶带
3. 再贴绝缘双面胶以固定电池
4. 铜柱挂垫片，拧在电池安装框上
5. 电源板固定在螺柱上
6. 插装4P线、2P线

### 9.2.6 腿部整体安装

1. 把皮带穿过大腿连接件内部，将皮带装在小齿轮上
2. 拉紧皮带，紧固大腿连接件和小腿电机的紧固螺钉
3. 把膝关节轴、膝关节轴承、膝关节轴套装在大腿内壳上
4. 固定大腿内壳与大腿连接件，拧紧固定螺钉
5. 安装惰轮部件，并同时安装大腿外壳

---

### 习题

- 9.1 简要叙述四足机器人四连杆腿部机构的构成原理。
- 9.2 掌握四足机器人电机初始化设置的方法。
- 9.3 了解四足机器人各个部件安装过程中的注意事项。

---

# 第10章 四足机器人硬件和电路结构介绍

## 本章概述

本章主要介绍四足机器人的硬件、电路结构框架及电路原理图，包含6部分：
1. 10.1 执行器
2. 10.2 动力源
3. 10.3 UP board
4. 10.4 遥控器
5. 10.5 SPIne信号转接板
6. 10.6 接线转接板

---

## 10.1 执行器

### 执行器电路主要模块

| 模块 | 功能 |
|------|------|
| 驱动算法处理电路 | 驱动电路核心，采用**STM32F446**作为主控制器 |
| 位置检测电路 | 完成电机转子的绝对位置获取 |
| 电流检测电路 | 检测电机三相绕组的电流 |
| 功率桥电路 | 给电机定子绕组提供需要的电压和电流 |

### STM32F446主控制器参数

| 参数 | 规格 |
|------|------|
| 主频 | **180MHz** |
| 内部硬件浮点运算单元 | 支持 |
| FOC运算需求 | 可满足高达**40kHz** |
| ADC | 至少2～3路 |
| SPI | 2路（读取编码器数值 + 驱动芯片配置） |
| PWM | 3路互补输出 |
| 通信接口 | CAN接口（系统设置）+ 串口（调试） |

### FOC工作原理

1. 采集两相电流
2. 经过**Clarke变换**后得到两轴正交电流量
3. 经过**旋转变换（Park变换）**后得到正交的电流量 id、iq
4. 将 id 与 iq 分别送进**PI调节器**，得到对应的输出 ud 和 uq
5. 通过传感器得到电机转过的角度
6. 进行**逆Park变换**，得到二轴电压量 uα 和 uβ
7. 对 uα 和 uβ 进行**Clarke逆变换**，得到实际需要的三相电压输出值

### FOC调试步骤（从内到外）

1. 人为给 uα、uβ 赋值，确保电机能运行，确认SVPWM没问题
2. 调试采样电流和编码器的角度输出
3. 调试电流环的PID参数
4. 调试速度环参数
5. 调试位置环参数

### MOSFET选择

三相电动机控制器的核心是三相逆变器。由于电机电感非常低（约**30mH**），使用了较高的开关频率和**40kHz**的控制环路频率。

### 编码器选择

采用**绝对式磁编码器**：
- **型号**：AS5047P
- 分辨率：**14位**绝对角位置
- 最高转速：**28krpm**
- 测量范围：完整的**360°**角
- 接口：标准**4线SPI串行接口**
- 优点：体积小、安装方便、抗干扰性强、价格低廉、集成度高

### 关节选择方案

采用**准直驱方案**：
- 盘式电机（惯量轻、极数多）+ 行星减速机
- 盘式电机价格低、工艺成熟
- 行星减速机综合成本只有谐波减速机的**十分之一**
- 将电机、减速机和控制器做在一个腔体内
- 每个电机单元可以做到电源和信号线并联形式的链式连接

---

## 10.2 动力源

### 电池系统

- 电池类型：**18650锂电池**
- 连续放电能力：**20A**
- 两节并联可产生**40A**的放电能力
- 瞬间电流可达**100A**

### 电源管理板功能

1. 开机启动浪涌电流限制
2. 整机供电高低压转换与电路系统隔离
3. 电流检测与电流保护
4. 电池电压检测
5. 系统开关机
6. 异常报警及对外通信

### 电源管理板CPU

采用**STM32F103**处理器，完成电流采样、保护阈值设置、CAN通信处理等功能。

### 控制隔离

- 电机供电与系统供电隔离
- 低压系统和高压系统之间的数据隔离
- 保证微处理器x86系统和低压5V系统的安全
- 保证核心CPU在工作过程中不会因为电流冲击而复位和跑飞

### 保护电路

- 机器人站立姿态时，系统电流约**2A**
- 行走时系统平均电流约**5A**
- 最大峰值约**15A**
- 峰值电流保护模式设置为**15A**

### 延时上电保护

- 12个电机控制器并联在一起具有很大的输入电容
- 输入电压为24V时，输入电容的总储能约为**1.5J**
- 设计了**预充电电路**

### 通信接口

电源板预留**CAN通信接口**，用于系统扩展、保护阈值设置。

---

## 10.3 UP board

### 基本规格

| 参数 | 规格 |
|------|------|
| 处理器 | Intel Atom X5-Z8350（4核） |
| 主频 | 1.44GHz（最高1.92GHz） |
| 内存 | 4GB DDR3L |
| 存储 | eMMC（16/32/64GB） |
| 功耗 | 平均约**7W**，峰值约**5W** |
| 架构 | x86 |
| 操作系统 | Linux（带preempt-rt补丁） |

### 通信特性

- **SPI接口**：数据传输波特率可达**20～50Mbps**（由FPGA实现）
- 通过**485总线和CAN总线**传输方式克服分布电容和噪声干扰
- **LCM**（Lightweight Communications and Marshalling）：局域网轻量级通信与数据封送库
- **Eigen**：矩阵运算库

### 扩展接口

- HDMI显示接口
- RJ45通信网口
- USB数据接口
- 双排DIP40排针（SPI接口、电源接口、UART接口等）
- 40-pin Raspberry Pin兼容排针

---

## 10.4 遥控器

### 按钮功能对照表

| 按钮 | 功能 |
|------|------|
| 方向油门操作杆（上/下） | 调整俯仰角 |
| 方向油门操作杆（左/右） | 控制转向 |
| 副翼/升降操作杆（上/下） | 控制前进/后退 |
| 副翼/升降操作杆（左/右） | 控制左/右横移 |
| SwA二段开关（上） | 进入Locomotion（动）模式 |
| SwA二段开关（下） | BalanceStand（平衡站立）模式 |
| SwC在上，SwD在上 | **Trot步态** |
| SwC在上，SwD在下 | **Flyingtrot步态** |
| SwC在中间，SwD在上 | **Slowtrot步态** |
| SwC在中间，SwD在下 | **Bound步态** |
| SwC在下，SwD在上 | **Pace步态** |
| SwC在下，SwD在下 | **Gallop步态** |
| SwE三段开关（上） | 进入运动模式选择 |
| SwE三段开关（中） | RecoveryStand（恢复站立）模式 |
| SwE三段开关（下） | Passive（被动）模式 |
| SwG三段开关（上） | **前空翻** |
| SwG三段开关（中） | 无功能 |
| SwG三段开关（下） | **后空翻** |
| VrA旋钮开关 | 控制机器人站高（范围：**0.18～0.35m**） |
| VrB旋钮开关 | 控制步高（范围：**0～0.1m**） |

### 启动顺序

1. 将遥控器各拨钮拨到初始位置
2. 将四足机器人摆动到初始位置
3. 打开电机开关和总开关
4. 等待1～2分钟，机器人自动上电并进入Passive模式
5. 向上拨动SwE至中间位置 → 站立姿态
6. 向上拨动SwE至最上方 → 平衡站立模式
7. 向上拨动SwA → 运动模式

---

## 10.5 SPIne信号转接板

### 总体设计

- 使用了**四路CAN总线**，每条腿对应一路
- 每条腿上的三个电机通信CAN总线并在一起
- 需要**两个STM32单片机**作为数据通信及调试接口
- 调试接口经过USB接口引出

### 通信架构

- 使用**STM32F446**芯片
- 通过芯片自带的CAN总线发送给执行器
- CAN总线带宽：**1Mbps**
- 1路CAN总线可以在**1kHz**带宽下控制3个执行器
- **串口1（PA2、PA3）**读取IMU数据
- **串口4（PA0、PA1）**发送给UP board
- x86内核以**1000Hz或者500Hz**的频率与MCU通信

### SPI数据包结构（spi_command_t）

```c
struct spi_command_t {
    float q_des_abad[2];     // 一条腿三个关节的位置信息
    float q_des_hip[2];
    float q_des_knee[2];
    float qd_des_abad[2];    // 速度信息
    float qd_des_hip[2];
    float qd_des_knee[2];
    float kp_abad[2];        // KP增益信息
    float kp_hip[2];
    float kp_knee[2];
    float kd_abad[2];        // KD增益信息
    float kd_hip[2];
    float kd_knee[2];
    float tau_abad_ff[2];    // 前馈扭矩增益信息
    float tau_hip_ff[2];
    float tau_knee_ff[2];
    int32_t flags[2];        // 附加标志
    int32_t checksum;        // 累加校验和
};
```

### SPI中断服务程序

```c
void spi_isr(void) {
    GPIOC->ODR |= (1 << 8);
    GPIOC->ODR &= ~(1 << 8);
    int bytecount = 0;
    SPI1->DR = tx_buff[0];
    while(cs == 0) {
        if(SPI1->SR&0x1) {
            rx_buff[bytecount] = SPI1->DR;  // 接收来自 UP board的数据命令
            bytecount++;
            if(bytecount<TX_LEN) {
                SPI1->DR = tx_buff[bytecount];
                // 把准备上报的关节数据上传给UP board进行数据交换
            }
        }
    }
    uint32_t calc_checksum = xor_checksum((uint32_t*)rx_buff,32);
    for(int i = 0; i < CMD_LEN; i++) {
        ((uint16_t*)(&spi_command))[i] = rx_buff[i];
    }
    if(calc_checksum != spi_command.checksum){
        spi_data.flags[1] = 0xdead;
    }
    control();
    PackAll();      // 把UP board发过来的信息封装到每条腿上
    WriteAll();     // 把信息经过CAN发送给每个关节
}
```

### 主程序

```c
int main() {
    pc.baud(921600);
    pc.attach(&serial_isr);
    estop.mode(PullUp);
    can1.frequency(1000000);
    can1.filter(CAN_ID<<21, 0xFFE00004, CANStandard, 0);
    can2.frequency(1000000);
    can2.filter(CAN_ID<<21, 0xFFE00004, CANStandard, 0);
    memset(&tx_buff, 0, TX_LEN * sizeof(uint16_t));
    memset(&spi_data, 0, sizeof(spi_data_t));
    memset(&spi_command, 0, sizeof(spi_command_t));
    // ... 初始化CAN消息ID和长度 ...
    // 主循环中持续读取CAN1、CAN2的反馈数据
    while(1) {
        counter++;
        can2.read(rxMsg2);
        unpack_reply(rxMsg2, &l2_state);
        can1.read(rxMsg1);
        unpack_reply(rxMsg1, &l1_state);
        wait_us(10);
    }
}
```

### control()函数

```c
void control() {
    spi_data.q_abad[0] = l1_state.a.p;
    spi_data.q_hip[0] = l1_state.h.p;
    spi_data.q_knee[0] = l1_state.k.p;
    spi_data.qd_abad[0] = l1_state.a.v;
    spi_data.qd_hip[0] = l1_state.h.v;
    spi_data.qd_knee[0] = l1_state.k.v;
    // ... 类似设置l2_state ...
    l1_control.a.p_des = spi_command.q_des_abad[0];
    l1_control.a.v_des = spi_command.qd_des_abad[0];
    l1_control.a.kp = spi_command.kp_abad[0];
    l1_control.a.kd = spi_command.kd_abad[0];
    l1_control.a.t_ff = spi_command.tau_abad_ff[0];
    // ... 类似设置其他关节 ...
    spi_data.flags[0] |= softstop_joint(l1_state.a, &l1_control.a, A_LIM_P, A_LIM_N);
    // ... 其他关节限位检查 ...
    spi_data.checksum = xor_checksum((uint32_t*)&spi_data, 14);
    for(int i = 0; i < DATA_LEN; i++){
        tx_buff[i] = ((uint16_t*)(&spi_data))[i];
    }
}
```

**注意**：来自关节电机的速度、位置信息被打包了，而扭矩信息被丢弃。原因是这套方案是**准直驱方案**，即x86发送多大的扭矩（电流指令），关节电机就立刻输出多大的扭矩。指定扭矩=反馈扭矩。

---

## 10.6 接线转接板

- 安装在机器人头部和尾部
- 只需将前后腿执行器的接头按顺序分别插接在接线转接板相应接口上即可
- 每条腿仅用一条线缆（将电源线和信号线并在一起）

**单个关节电机对外接口**：内部有两条**24V电源线**和两条**CAN信号线**。

---

### 习题

- 10.1 理解四足机器人的硬件系统框图和各部分之间的关系。
- 10.2 简要叙述FOC控制原理。
- 10.3 简要叙述电源管理板各部分的功能。

---

# 第11章 基于模型预测控制的四足机器人全身运动控制方法

## 11.1 MPC基础知识介绍

### MPC定义

模型预测控制（Model Predictive Control, MPC）是一种能够优化目标和约束的控制方法。MPC的核心思想是预测系统的未来输出值，其将所得到的最优控制序列的第1个优化解作用于系统，并依次滚动向前进行。

### MPC五大特点

1. 对模型要求低，建模方便，不需要深入了解过程内部机理
2. 滚动优化策略具有较好的动态控制效果
3. 使用简单实用的模型校正方法，具有较强的鲁棒性
4. 不增加理论难度，可以推广应用于有约束、纯滞后、多输入/多输出、非线性等工业生产过程中
5. 是一种易于计算及实现的优化控制算法

### MPC实现的三个步骤

#### 1. 预测模型
模型预测控制的模型称为预测模型。强调的是模型的功能而不是模型的结构，只要模型可利用过去已知数据信息预测系统未来的输出行为，就可以作为预测模型。

#### 2. 滚动优化
MPC求解所需指标的最优解，将其作为控制的输入。优化过程不是采用一成不变的全局最优化目标，而是采用**滚动式的有限时域优化策略**。

#### 3. 反馈校正
采用反馈校正来弥补模型不确定性缺陷。反馈校正后的滚动优化可有效地克服系统中的不确定性，提高系统的控制精度和鲁棒性。

---

## 11.2 MPC控制策略在四足机器人中的应用

### 相关研究

- **2018年 Carlo**：提出基于MPC确定力矩控制下四足机器人地面反力的方法，将机器人动力学简化为凸优化问题，利用**二次规划（QP）**寻找最优足底力
- **Neunert**：提出基于接触点的刚体系统全身非线性MPC方法，接触位置、序列和时间不需要预先指定
- **2019年 Guo**：采用MPC方法控制四足机器人的足端轨迹跟踪，利用牛顿-欧拉公式进行动力学建模

---

## 11.3 四足机器人整体运动控制框架

### 11.3.1 一般四足机器人整体运动控制框架

给定期望的平移速度和航向变化速率，高层控制器规划平滑、可控的机器人质心参考轨迹，然后映射到躯干和腿部控制器中。

关键变量：
- 期望身体姿态
- 布尔状态变量
- 足端输出力
- 足端位置
- 足端关节跟踪运动力矩
- 机器人实时状态估计
- 机器人足端触地估计

### 11.3.2 基于MPC的四足机器人运动控制框架

控制流程：
1. 遥控器获取用户输入（步态类型、运动速度和方向指令）
2. MPC控制器中的运动学/动力学模型计算足端接触力和位置信息
3. 状态估计器得到机器人身体姿态信息（速度、加速度）
4. WBC（全身控制）计算关节力矩、位置和速度指令
5. 传递给每个关节驱动器
6. 循环完成

---

## 11.4 MPC控制框架

### 11.4.1 MPC控制框架

利用**离散有限预测时域控制器**来实现机器人足端期望接触力的计算。

包含五部分：
1. 支撑腿和摆动腿控制
2. 触地探测算法
3. 地面作用力控制
4. 四足机器人状态估计器设计
5. 四足机器人期望质心估计

### 支撑腿和摆动腿控制

- 雅可比矩阵
- MPC计算出的最优足底力作为前馈力
- 支撑相微分调节矩阵
- 腿的速度和参考速度
- 对角正定比例和导数增益矩阵

### 触地探测算法

利用多种信息通过信息融合方式进行腿是否触地的判定：
- 垂直方向速度急剧减小
- 足底与地面的接触力增加
- 判定条件包括：足底速度变化、压力变化、压力阈值、小腿速度、接触时间等

### 机器人打滑过程分析

判定足底是否打滑的条件：
- 水平分量超过地面最大静摩擦力
- 足底压力的变化
- 支撑阶段腿的速度变化阈值
- 滑动时间和滑动超时值

### 地面作用力控制

关节力矩表示为足端雅可比矩阵与力向量的乘积。

### 状态估计器设计

采用**两阶段传感器融合算法**：
- **第一阶段**：利用IMU陀螺仪和加速度计读数进行滤波融合
- **第二阶段**：利用各条腿的运动学测量结果，基于方向估计进行机器人腿的位置和速度估计

### 期望质心估计

定义一种**虚拟支撑多边形**的计算方法，使用非线性加权策略：
- 支撑状态自适应权重因子
- 摆动状态自适应权重因子
- 使用**高斯误差函数**

权重因子设计：
- 越接近支撑相中间段的腿越适合作为支撑腿
- 越接近摆动相中间段的腿越不适合作为下一时刻的落足腿

---

## 11.4.2 机器人躯干简化动力学建模

### 刚体动力学方程

不考虑腿部动力学，只对躯干建立动力学模型。

**牛顿方程**（世界坐标系中）：
- 惯性张量
- 角速度
- 腿在地面的接触点相对于躯干中心的位置

### 状态方程

状态变量包括：
- 世界坐标系下机器人质心的位置
- 躯干的姿态角
- 躯干质心速度
- 世界坐标系下躯干角速度

### 离散化

使用**零阶保持器**进行采样，以固定时间间隔进行离散化。

---

## 11.4.3 MPC实现

### MPC控制器设计

基于被控系统的模型，结合系统的姿态角、位置等状态量和控制输入量，构建状态空间。MPC控制器预测出系统稳定运行的**最优足底力**。

### MPC控制器求解

采用**精确离散法**将状态方程进行离散化。

### 标准MPC问题

寻找控制输入序列，保证系统沿着参考轨迹运行，在控制效果和跟踪精度之间进行折中。

### 摩擦锥约束

每条腿的6个不等式约束，防止机器人打滑。

### QP求解

将MPC问题转换成QP（二次规划）问题，使用**qpOASES开源库**求解。

**qpOASES**是一个可二次开发的二次规划求解器，能够使用在线有效集策略处理和解决凸二次规划问题。

---

## 11.4.4 基于MPC的全身运动控制（WBC）

### WBC控制器设计

基于**零空间映射**的全身运动控制（Whole Body Control, WBC）方法，实现把低优先级任务映射到高优先级任务的零空间中。

四足机器人运动控制任务按优先级：
1. 躯干的位置任务
2. 躯干的姿态任务
3. 支撑腿的任务
4. 摆动腿的任务

### 动力学方程

```
Mq̈ + C(q,q̇) + G = S^T τ + J^T F
```

- M：惯性矩阵
- C：科式力和离心力
- G：重力项
- S：选择矩阵
- τ：关节扭矩
- F：支撑力
- J：支撑腿的雅可比矩阵

### 零空间映射

动态连续雅可比矩阵的逆和零空间映射矩阵的计算，实现**NSP（Null Space Pursuit）**下的优先级任务规划。

### 基于QP的任务求解

将支撑力和指令关节加速度进行微调，使它们满足动力学方程中躯干上的加速度约束。

约束条件：
- 躯干上的加速度等式约束
- 支撑腿对应的摩擦锥约束

---

## 11.5 基于MPC方法的运动规划

### 11.5.1 步态规划

MPC求解时间是几十毫秒，使用**稀疏化**等方式可提高求解速度。将具体的时间序列变成以MPC更新时间为单位的腿部状态设计。

### 常见步态类型

| 步态 | 类型 | 说明 |
|------|------|------|
| **trot** | 对称步态 | 对角线腿同相 |
| **flying trot** | 对称步态 | 对角线腿同相（更快） |
| **bound** | 对称步态 | 前后腿同相 |
| **pronk** | 非对称步态 | 四腿同相 |
| **gallop** | 非对称步态 | 非对称相位 |

### 11.5.2 足端轨迹规划

使用**Raibert启发式规则**计算足端位置信息：
- 机器人身体位置
- 髋关节相对于机体坐标系的位置
- 落地角度和离地角度保持一致

### 11.5.3 基于MPC方法的步态切换

步态切换时仅需调整相位切换点和每条腿相位偏移点。

### 11.5.4 斜坡地形的姿态调整

#### 简化的斜坡地形自适应方法

利用四足机器人每条腿的当前位置估计地形坡度，调整身体姿态。

#### 稳定域可调的斜坡地形自适应方法

基于**ZMP（Zero Moment Point）稳定条件**，通过对足端位置与躯干姿态的两步调整，实现机器人质心对斜坡的适应。

**坐标系定义**：
- 机体坐标系：原点位于机器人机体质心处
- 世界坐标系：Z轴垂直水平面向上
- 前进坐标系：原点与机体坐标系重合
- 斜面坐标系：原点为机器人质心在斜面的投影

**足端位置调整**：依据躯干姿态信息实现足端位置的坐标映射

**躯干姿态调整**：提出"**虚拟斜坡**"概念，即四足机器人由平坦路面运动到斜坡时前后两支撑足端所形成的二维平面。

---

## 11.6 基于Webots软件平台的MPC方法仿真验证

### 仿真目标指标

- 快速运动：**10km/h**
- 大负重：自重的**40%**
- 多步态运动能力

### Webots软件安装

- 安装最新Linux版本的Webots软件
- 推荐使用**GTX 1050以上显卡**
- 配置WEBOTS_HOME环境变量

### 机器人模型搭建

- 通过圆柱体、立方体等基本形状搭建机器人模型
- 赋予每个关节、连杆与物理平台相同的**尺寸、质量、惯量**等属性
- 每个关节包含**HingeJoint**（旋转关节）
- 在device中添加**RotationMotor**和**PositionSensor**
- 小腿末端放置**Touch Sensor**（足端力传感器）
- 添加**Inertial Unit**（姿态传感器）和**Gyro**（角速度传感器）

### 模型优化

将SolidWorks中的设计图导入Webots：
- 更改每个连杆零件的原点和方向
- 方向统一：向前为x，向右为z，向上为y
- 保存为**VRML 97**格式

### 控制器创建与编译

支持两种方式：
1. **基于makefile**：指定源文件和头文件路径
2. **基于cmake**：更好的维护性

### 多步态运动仿真结果

- **trot步态**：最大稳定速度**2.5m/s**（仿真中最大可达2.8m/s）
- 前向运动目标速度从0加速到2.5m/s
- 侧向运动目标速度由0加速到1m/s
- 自转速度从0加速到2.5rad/s

### 斜坡地形测试

- 测试斜坡：**22°**
- 机器人在斜坡地形上方运动时能够跟随目标速度**0.5m/s**
- 姿态调整量约为**15°**

---

## 11.7 基于四足机器人物理平台的MPC方法实现验证

### 实验机器人参数

- 每条腿3个电机：1个横滚自由度 + 2个俯仰自由度
- 控制核心：UP board
- 姿态检测：IMU
- 驱动装置：STM32F446电机驱动板
- 执行单元：无刷直流电机
- 电源：锂电池

### 实验参数

- 步高：**3cm**
- 步频：**4Hz**
- 步态周期：**0.25s**

### 实验环境

- 水平路面：长1.5m、宽0.5m
- 斜坡路面：长1.5m、宽0.5m，坡度**14°**
- 材质：硬PVC塑料板

### 实验结果

- 水平路面俯仰角变化范围：**-0.57°～-1.43°**
- 爬坡过程中俯仰角自适应调整
- 调整完成后躯干俯仰角保持在斜坡角度值附近波动

---

### 习题

- 11.1 简要叙述MPC的步骤和用于机器人控制的优势。
- 11.2 理解并掌握基于MPC的四足机器人运动控制框架。
- 11.3 掌握基于Webots软件平台的四足机器人建模方法。

---

# 第12章 四足机器人运动控制程序框架介绍

## 12.1 机器人建模程序

### 12.1.1 机器人模型参数设置

```cpp
Quadruped<T> buildMiniCheetah() {
    Quadruped<T> cheetah;
    cheetah._robotType = RobotType::MINI_CHEETAH;
    cheetah._bodyMass = 3.3;           // 躯干质量 3.3kg
    cheetah._bodyLength = 0.19 * 2;     // 躯干长度
    cheetah._bodyWidth = 0.049 * 2;     // 躯干宽度
    cheetah._bodyHeight = 0.05 * 2;     // 躯干高度
    cheetah._abadGearRatio = 6;         // 侧摆减速比
    cheetah._hipGearRatio = 6;          // 髋关节减速比
    cheetah._kneeGearRatio = 9.33;      // 膝关节减速比
    cheetah._abadLinkLength = 0.062;    // 侧摆连杆长度
}
```

**惯量参数设置**：

```cpp
Mat3<T> abadRotationalInertia;
abadRotationalInertia << 381, 58, 0.45, 58, 560, 0.95, 0.45, 0.95, 444;
abadRotationalInertia = abadRotationalInertia * 1e-6;
Vec3<T> abadCOM(0, 0.036, 0);
SpatialInertia<T> abadInertia(0.54, abadCOM, abadRotationalInertia);
```

**关节位置参数**：

```cpp
cheetah._abadRotorLocation = Vec3<T>(0.125, 0.049, 0);
cheetah._abadLocation = Vec3<T>(cheetah._bodyLength, cheetah._bodyWidth, 0) * 0.5;
cheetah._hipLocation = Vec3<T>(0, cheetah._abadLinkLength, 0);
cheetah._hipRotorLocation = Vec3<T>(0, 0.04, 0);
cheetah._kneeLocation = Vec3<T>(0, 0, -cheetah._hipLinkLength);
cheetah._kneeRotorLocation = Vec3<T>(0, 0, 0);
```

**关节传动比偏置和关节零位**（数组元素顺序：右前腿、左前腿、右后腿、左后腿）：

```cpp
const float abad_side_sign[4] = {-1.f, -1.f, 1.f, 1.f};
const float hip_side_sign[4] = {-1.f, 1.f, -1.f, 1.f};
const float knee_side_sign[4] = {-.6429f, .6429f, -.6429f, .6429f};
const float abad_offset[4] = {0.f, 0.f, 0.f, 0.f};
const float hip_offset[4] = {M_PI / 2.f, -M_PI / 2.f, -M_PI / 2.f, M_PI / 2.f};
const float knee_offset[4] = {K_KNEE_OFFSET_POS, -K_KNEE_OFFSET_POS,
                               -K_KNEE_OFFSET_POS, K_KNEE_OFFSET_POS};
```

### 12.1.2 质心动力学模型

```cpp
void ct_ss_mats(Matrix<fpt,3,3> I_world, fpt m, Matrix<fpt,3,4> r_feet,
                Matrix<fpt,3,3> R_yaw, Matrix<fpt,13,13>& A, Matrix<fpt,13,12>& B) {
    A.setZero();
    A(3,9) = 1.f;
    A(4,10) = 1.f;
    A(5,11) = 1.f;
    A(11,12) = 1.f;
    A.block(0,6,3,3) = R_yaw.transpose();
    B.setZero();
    Matrix<fpt,3,3> I_inv = I_world.inverse();
    for(s16 b = 0; b < 4; b++){
        B.block(6,b*3,3,3) = cross_mat(I_inv, r_feet.col(b));
        B.block(9,b*3,3,3) = Matrix<fpt,3,3>::Identity() / m;
    }
}
```

### 12.1.3 浮动基座动力学模型

与固定基相比，浮动基机器人的基座速度和角速度均受机器人本体运动影响。

本书四足机器人采用**浮动基动力学模型**，主要用在WBC控制器和仿真中。

根据建立的浮动基动力学可计算：
- **M**：惯性矩阵
- **C**：科氏力和离心力矩阵
- **G**：重力矩阵

该浮基座动力学依据Roy Featherstone的《Rigid Body Dynamic Algorithms》建立。

**浮动基座模型构建代码**：

```cpp
template <typename T>
bool Quadruped<T>::buildModel(FloatingBaseModel<T>& model) {
    Vec3<T> bodyDims(_bodyLength, _bodyWidth, _bodyHeight);
    model.addBase(_bodyInertia);
    model.addGroundContactBoxPoints(5, bodyDims);
    for (int legID = 0; legID < 4; legID++) {
        bodyID++;
        Mat6<T> xtreeAbad = createSXform(I3, withLegSigns<T>(_abadLocation, legID));
        Mat6<T> xtreeAbadRotor = createSXform(I3, withLegSigns<T>(_abadRotorLocation, legID));
        if (sideSign < 0) {
            model.addBody(_abadInertia.flipAlongAxis(CoordinateAxis::Y),
                         _abadRotorInertia.flipAlongAxis(CoordinateAxis::Y),
                         _abadGearRatio, baseID, JointType::Revolute,
                         CoordinateAxis::X, xtreeAbad, xtreeAbadRotor);
        } else {
            model.addBody(_abadInertia, _abadRotorInertia, _abadGearRatio, baseID,
                         JointType::Revolute, CoordinateAxis::X, xtreeAbad, xtreeAbadRotor);
        }
    }
}
```

---

## 12.2 机器人信息数据输入

### 12.2.1 遥控器指令的输入

```cpp
template <typename T>
bool Quadruped<T>::updateGamepadCommand(GamepadCommand &gamepadCommand) {
    if (_qGamepad) {
        gamepadCommand.leftBumper = _qGamepad->buttonL1();
        gamepadCommand.rightBumper = _qGamepad->buttonR1();
        // ... 按钮映射 ...
        gamepadCommand.leftStickAnalog = Vec2<float>(_qGamepad->axisLeftX(), -_qGamepad->axisLeftY());
        gamepadCommand.rightStickAnalog = Vec2<float>(_qGamepad->axisRightX(), -_qGamepad->axisRightY());
    } else {
        gamepadCommand.zero();
    }
}
```

**状态轨迹生成**：

```cpp
data.stateDes(6) = deadband(gamepadCommand->leftStickAnalog[1], minVelX, maxVelX);
data.stateDes(7) = deadband(gamepadCommand->leftStickAnalog[0], minVelY, maxVelY);
data.stateDes(8) = 0.0;
data.stateDes(0) = stateEstimate->position(0) + dt * data.stateDes(6);
data.stateDes(1) = stateEstimate->position(1) + dt * data.stateDes(7);
data.stateDes(2) = 0.45;
data.stateDes(11) = deadband(gamepadCommand->rightStickAnalog[0], minTurnRate, maxTurnRate);
data.stateDes(4) = deadband(gamepadCommand->rightStickAnalog[1], minPitch, maxPitch);
data.stateDes(5) = stateEstimate->rpy(2) + dt * data.stateDes(11);
```

### 12.2.2 本体感受信息输入

**IMU数据读取**（通过VectorNav串行通信）：

```cpp
void vectornav_handler(void* userData, VnUartPacket* packet, size_t running_index) {
    vec4f quat; vec3f omega; vec3f a;
    // 检查数据类型为BINARY
    // 提取四元数、角速度、加速度
    quat = VnUartPacket_extractVec4f(packet);
    omega = VnUartPacket_extractVec3f(packet);
    a = VnUartPacket_extractVec3f(packet);
    // 发布到LCM
    vectornav_lcm->publish("hw_vectornav", &vectornav_lcm_data);
}
```

**电机关节数据读取**（通过SPI与SPIne通信）：

```cpp
for (int spi_board = 0; spi_board < 2; spi_board++) {
    spi_to_spine(command, &g_spine_cmd, spi_board * 2);
    // ... SPI数据交换 ...
    int rv = ioctl(spi_board == 0 ? spi_1_fd : spi_2_fd,
                   SPI_IOC_MESSAGE(1), &spi_message);
    // ... 解析反馈数据 ...
    spine_to_spi(data, &g_spine_data, spi_board * 2);
}
```

---

## 12.3 机器人运动轨迹规划

### 12.3.1 步态序列规划

```cpp
trotting(horizonLength, Vec4<int>(0,5,5,0), Vec4<int>(5,5,5,5), "Trotting"),
bounding(horizonLength, Vec4<int>(5,5,0,0), Vec4<int>(5,5,5,5), "Bounding"),
pronking(horizonLength, Vec4<int>(0,0,0,0), Vec4<int>(4,4,4,4), "Pronking"),
galloping(horizonLength, Vec4<int>(0,2,7,9), Vec4<int>(6,6,6,6), "Galloping"),
standing(horizonLength, Vec4<int>(0,0,0,0), Vec4<int>(10,10,10,10), "Standing"),
trotRunning(horizonLength, Vec4<int>(0,5,5,0), Vec4<int>(3,3,3,3), "Trot Running"),
walking(horizonLength, Vec4<int>(0,3,5,8), Vec4<int>(5,5,5,5), "Walking"),
walking2(horizonLength, Vec4<int>(0,5,5,0), Vec4<int>(7,7,7,7), "Walking2"),
pacing(horizonLength, Vec4<int>(5,0,5,0), Vec4<int>(5,5,5,5), "Pacing")
```

### 12.3.2 摆动腿轨迹规划

使用**三次贝塞尔曲线**规划：

```cpp
template <typename T>
void FootSwingTrajectory<T>::computeSwingTrajectoryBezier(T phase, T swingTime) {
    _p = Interpolate::cubicBezier<Vec3<T>>(_p0, _pf, phase);
    _v = Interpolate::cubicBezierFirstDerivative<Vec3<T>>(_p0, _pf, phase) / swingTime;
    _a = Interpolate::cubicBezierSecondDerivative<Vec3<T>>(_p0, _pf, phase) / (swingTime * swingTime);
    T zp, zv, za;
    if(phase < T(0.5)) {
        zp = Interpolate::cubicBezier<T>(_p0[2], _p0[2] + _height, phase * 2);
        zv = Interpolate::cubicBezierFirstDerivative<T>(_p0[2], _p0[2] + _height, phase * 2) * 2 / swingTime;
        za = Interpolate::cubicBezierSecondDerivative<T>(_p0[2], _p0[2] + _height, phase * 2) * 4 / (swingTime * swingTime);
    } else {
        zp = Interpolate::cubicBezier<T>(_p0[2] + _height, _pf[2], phase * 2 - 1);
        zv = Interpolate::cubicBezierFirstDerivative<T>(_p0[2] + _height, _pf[2], phase * 2 - 1) * 2 / swingTime;
        za = Interpolate::cubicBezierSecondDerivative<T>(_p0[2] + _height, _pf[2], phase * 2 - 1) * 4 / (swingTime * swingTime);
    }
    _p[2] = zp; _v[2] = zv; _a[2] = za;
}
```

**落足点规划**（Raibert启发式）：

```cpp
float pfx_rel = seResult.vWorld[0] * .5 * gait->_stance * dtMPC +
    .03f*(seResult.vWorld[0]-v_des_world[0]) +
    (0.5f*seResult.position[2]/9.81f) * (seResult.vWorld[1]*stateCommand->data.stateDes[2]);
float pfy_rel = seResult.vWorld[1] * .5 * gait->_stance * dtMPC +
    .03f*(seResult.vWorld[1]-v_des_world[1]) +
    (0.5f*seResult.position[2]/9.81f) * (-seResult.vWorld[0]*stateCommand->data.stateDes[2]);
pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
Pf[0] += pfx_rel;
Pf[1] += pfy_rel;
Pf[2] = -0.01;
```

### 12.3.3 机器人状态轨迹规划

```cpp
float trajInitial[12] = {(float)rpy_comp[0], (float)rpy_comp[1],
    (float)stateCommand->data.stateDes[5], xStart, yStart, (float)0.26,
    0, 0, (float)stateCommand->data.stateDes[11],
    v_des_world[0], v_des_world[1], 0};
```

---

## 12.4 机器人控制器设计

### 12.4.1 MPC控制器

**构建连续状态空间**：

```cpp
void ct_ss_mats(Matrix<fpt,3,3> I_world, fpt m, Matrix<fpt,3,4> r_feet,
                Matrix<fpt,3,3> R_yaw, Matrix<fpt,13,13>& A, Matrix<fpt,13,12>& B) {
    A.setZero();
    A(3,9) = 1.f; A(4,10) = 1.f; A(5,11) = 1.f; A(11,12) = 1.f;
    A.block(0,6,3,3) = R_yaw.transpose();
    B.setZero();
    Matrix<fpt,3,3> I_inv = I_world.inverse();
    for(s16 b = 0; b < 4; b++) {
        B.block(6,b*3,3,3) = cross_mat(I_inv, r_feet.col(b));
        B.block(9,b*3,3,3) = Matrix<fpt,3,3>::Identity() / m;
    }
}
```

**离散化处理**：

```cpp
ABc.setZero();
ABc.block(0,0,13,13) = Ac;
ABc.block(0,13,13,12) = Bc;
ABc = dt*ABc;
expmm = ABc.exp();
Adt = expmm.block(0,0,13,13);
Bdt = expmm.block(0,13,13,12);
```

**构建优化目标函数**：

```cpp
qH = 2*(B_qp.transpose()*S*B_qp + update->alpha*eye_12h);
qg = 2*B_qp.transpose()*S*(A_qp*x_0 - X_d);
matrix_to_real(H_qpoases, qH, setup->horizon*12, setup->horizon*12);
matrix_to_real(g_qpoases, qg, setup->horizon*12, 1);
matrix_to_real(A_qpoases, fmat, setup->horizon*20, setup->horizon*12);
matrix_to_real(ub_qpoases, U_b, setup->horizon*20, 1);
```

**摩擦锥约束设置**：

```cpp
fpt mu = 1.f/setup->mu;
Matrix<fpt,5,3> f_block;
f_block << mu, 0,  1.f,
          -mu, 0,  1.f,
           0,  mu, 1.f,
           0, -mu, 1.f,
           0,   0, 1.f;
for(s16 i = 0; i < setup->horizon*4; i++) {
    fmat.block(i*5,i*3,5,3) = f_block;
}
```

**QP求解**：

```cpp
qpOASES::QProblem problem_red(new_vars, new_cons);
qpOASES::Options op;
op.setToMPC();
op.printLevel = qpOASES::PL_NONE;
problem_red.setOptions(op);
int rval = problem_red.init(H_red, g_red, A_red, NULL, NULL, lb_red, ub_red, nWSR);
int rval2 = problem_red.getPrimalSolution(q_red);
```

### 12.4.2 WBC控制器

**模型更新**：

```cpp
template<typename T>
void WBC_Ctrl<T>::_UpdateModel(const StateEstimate<T> & state_est,
                                const LegControllerData<T> * leg_data) {
    _state.bodyOrientation = state_est.orientation;
    _state.bodyPosition = state_est.position;
    for(size_t i(0); i<3; ++i) {
        _state.bodyVelocity[i] = state_est.omegaBody[i];
        _state.bodyVelocity[i+3] = state_est.vBody[i];
        for(size_t leg(0); leg<4; ++leg) {
            _state.q[3*leg + i] = leg_data[leg].q[i];
            _state.qd[3*leg + i] = leg_data[leg].qd[i];
            _full_config[3*leg + i + 6] = _state.q[3*leg + i];
        }
    }
    _model.setState(_state);
    _model.contactJacobians();
    _model.massMatrix();
    _model.generalizedGravityForce();
    _model.generalizedCoriolisForce();
    _A = _model.getMassMatrix();
    _grav = _model.getGravityForce();
    _coriolis = _model.getCoriolisForce();
    _Ainv = _A.inverse();
}
```

**零空间矩阵构建与层级任务求解**：

```cpp
DMat<T> Nc(num_qdot_, num_qdot_);
Nc.setIdentity();
if(contact_list.size() > 0) {
    DMat<T> Jc, Jc_i;
    contact_list[0]->getContactJacobian(Jc);
    // ... 构建接触雅可比矩阵 ...
    _BuildProjectionMatrix(Jc, Nc);
}
// ... 构建所有任务对应的零空间矩阵 ...
for (size_t i(1); i < task_list.size(); ++i) {
    task = task_list[i];
    task->getTaskJacobian(Jt);
    JtPre = Jt * N_pre;
    _PseudoInverse(JtPre, JtPre_pinv);
    delta_q = prev_delta_q + JtPre_pinv * (task->getPosError() - Jt * prev_delta_q);
    qdot = prev_qdot + JtPre_pinv * (task->getDesVel() - Jt * prev_qdot);
}
```

**QP求解**：

```cpp
T f = solve_quadprog(G, g0, CE, ce0, CI, ci0, z);
for (size_t i(0); i < _dim_floating; ++i) qddot_pre[i] += z[i];
_GetSolution(qddot_pre, cmd);
```

**指令更新**：

```cpp
template<typename T>
void WBC_Ctrl<T>::_UpdateLegCMD(ControlFSMData<T> & data) {
    LegControllerCommand<T> * cmd = data._legController->commands;
    for (size_t leg(0); leg < cheetah::num_leg; ++leg) {
        cmd[leg].zero();
        for (size_t jidx(0); jidx < cheetah::num_leg_joint; ++jidx) {
            cmd[leg].tauFeedForward[jidx] = _tau_ff[cheetah::num_leg_joint * leg + jidx];
            cmd[leg].qDes[jidx] = _des_jpos[cheetah::num_leg_joint * leg + jidx];
            cmd[leg].qdDes[jidx] = _des_jvel[cheetah::num_leg_joint * leg + jidx];
            cmd[leg].kpJoint(jidx, jidx) = _Kp_joint[jidx];
            cmd[leg].kdJoint(jidx, jidx) = _Kd_joint[jidx];
        }
    }
}
```

### 12.4.3 腿部控制器

**腿部雅可比矩阵和位置计算**：

```cpp
void computeLegJacobianAndPosition(Quadruped<T>& quad, Vec3<T>& q, Mat3<T>* J,
                                    Vec3<T>* p, int leg) {
    T l1 = quad._abadLinkLength;
    T l2 = quad._hipLinkLength;
    T l3 = quad._kneeLinkLength;
    T sideSign = quad.getSideSign(leg);
    T s1 = std::sin(q(0)); T s2 = std::sin(q(1)); T s3 = std::sin(q(2));
    T c1 = std::cos(q(0)); T c2 = std::cos(q(1)); T c3 = std::cos(q(2));
    T c23 = c2 * c3 - s2 * s3;
    T s23 = s2 * c3 + c2 * s3;
    if (J) {
        J->operator()(0, 0) = 0;
        J->operator()(0, 1) = l3 * c23 + l2 * c2;
        J->operator()(0, 2) = l3 * c23;
        J->operator()(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - l1 * sideSign * s1;
        J->operator()(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
        J->operator()(1, 2) = -l3 * s1 * s23;
        J->operator()(2, 0) = l3 * s1 * c23 + l2 * c2 * s1 + l1 * sideSign * c1;
        J->operator()(2, 1) = l3 * c1 * s23 + l2 * c1 * s2;
        J->operator()(2, 2) = l3 * c1 * s23;
    }
    if (p) {
        p->operator()(0) = l3 * s23 + l2 * s2;
        p->operator()(1) = l1 * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
        p->operator()(2) = l1 * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;
    }
}
```

**关节扭矩计算**：

```cpp
legTorque += datas[leg].J.transpose() * footForce;
spiCommand->tau_abad_ff[leg] = legTorque(0);
spiCommand->tau_hip_ff[leg] = legTorque(1);
spiCommand->tau_knee_ff[leg] = legTorque(2);
// ... 设置PD增益和期望位置速度 ...
```

**FOC底层控制代码**（dq0变换和SVPWM）：

```cpp
void dq0(float theta, float a, float b, float c, float *d, float *q) {
    float cf = FastCos(theta); float sf = FastSin(theta);
    *d = 0.6666667f*(cf*a + (0.86602540378f*sf-.5f*cf)*b
        + (-0.86602540378f*sf-.5f*cf)*c);
    *q = 0.6666667f*(-sf*a - (-0.86602540378f*cf-.5f*sf)*b
        - (0.86602540378f*cf-.5f*sf)*c);
}

void svm(float v_bus, float u, float v, float w,
         float *dtc_u, float *dtc_v, float *dtc_w) {
    float v_offset = (fminf3(u,v,w) + fmaxf3(u,v,w))*0.5f;
    *dtc_u = fminf(fmaxf(((u-v_offset)/v_bus + .5f), DTC_MIN), DTC_MAX);
    *dtc_v = fminf(fmaxf(((v-v_offset)/v_bus + .5f), DTC_MIN), DTC_MAX);
    *dtc_w = fminf(fmaxf(((w-v_offset)/v_bus + .5f), DTC_MIN), DTC_MAX);
}
```

---

## 12.5 状态估计器

### 姿态估计器

```cpp
template <typename T>
void CheaterOrientationEstimator<T>::run() {
    this->_stateEstimatorData.result->orientation =
        this->_stateEstimatorData.cheaterState->orientation.template cast<T>();
    this->_stateEstimatorData.result->rBody =
        ori::quaternionToRotationMatrix(this->_stateEstimatorData.result->orientation);
    this->_stateEstimatorData.result->omegaBody =
        this->_stateEstimatorData.cheaterState->omegaBody.template cast<T>();
    this->_stateEstimatorData.result->omegaWorld =
        this->_stateEstimatorData.result->rBody.transpose() * this->_stateEstimatorData.result->omegaBody;
    this->_stateEstimatorData.result->rpy =
        ori::quatToRPY(this->_stateEstimatorData.result->orientation);
}
```

### 线性卡尔曼位置速度估计器

```cpp
template <typename T>
void LinearKFPositionVelocityEstimator<T>::setup() {
    T dt = this->_stateEstimatorData.parameters->controller_dt;
    _xhat.setZero(); _ps.setZero(); _vs.setZero(); _A.setZero();
    _A.block(0, 0, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();
    _A.block(0, 3, 3, 3) = dt * Eigen::Matrix<T, 3, 3>::Identity();
    _A.block(3, 3, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();
    _A.block(6, 6, 12, 12) = Eigen::Matrix<T, 12, 12>::Identity();
    _B.setZero();
    _B.block(3, 0, 3, 3) = dt * Eigen::Matrix<T, 3, 3>::Identity();
    // ... 构建观测矩阵C ...
    _P.setIdentity(); _P = T(100) * _P;
    _Q0.setIdentity();
    _Q0.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<T, 3, 3>::Identity();
    _Q0.block(3, 3, 3, 3) = (dt * 9.8f / 20.f) * Eigen::Matrix<T, 3, 3>::Identity();
    _Q0.block(6, 6, 12, 12) = dt * Eigen::Matrix<T, 12, 12>::Identity();
    _R0.setIdentity();
}
// 输出：
this->_stateEstimatorData.result->position = _xhat.block(0, 0, 3, 1);
this->_stateEstimatorData.result->vWorld = _xhat.block(3, 0, 3, 1);
this->_stateEstimatorData.result->vBody =
    this->_stateEstimatorData.result->rBody * this->_stateEstimatorData.result->vWorld;
```

---

### 习题

- 12.1 理解MPC控制程序的实现过程。
- 12.2 理解WBC控制程序的实现过程。
- 12.3 理解四足机器人步态规划程序的实现过程。
- 12.4 固定基动力学模型和浮动基动力学模型的区别和联系是什么？

---

# 第13章 四足机器人实验仿真与验证

## 本章包含的11个实验

1. Ubuntu系统环境配置与软件安装实验
2. 四足机器人上位机代码编译与运行仿真实验
3. 实体机器人运行实验
4. trot步态设计及验证实验
5. bound步态设计及验证实验
6. pace步态设计及验证实验
7. walk步态设计及验证实验
8. 后空翻步态设计与运行实验
9. 四足机器人斜坡自适应调整实验
10. 循迹测试实验
11. 障碍物识别跟踪实验

---

## 实验1：Ubuntu系统环境配置与软件安装

### 实验目的
1. 了解Ubuntu系统的基本操作指令及用法
2. 掌握Ubuntu环境配置的步骤

### 机器人系统总体结构

系统分为4部分：
1. **上位机**（PC端）：提供图形控制界面，负责参数调整、发送控制指令及实时和离线仿真
2. **通信模块**：基于**LCM**通信实现双方数据交互
3. **控制器**（机器人躯干上）：负责控制算法和状态估计器的实现
4. **执行器**（每个关节上）：执行运动控制指令、伺服控制

### 软件介绍

**Qt框架**：
- 由Qt Company开发的跨平台C++图形用户界面应用程序开发框架
- 支持Windows、Linux等操作系统
- 包含250个以上的C++类
- 使用**signals/slots**机制替代callback

**LCM（Lightweight Communications and Marshalling）**：
- 用于消息传递和数据编组的库和工具
- 提供发布/订阅消息传递模型
- 支持低延迟进程间通信
- 使用**UDP组播**的高效广播机制

### 实验步骤

#### 1. 安装依赖

```bash
sudo apt install mesa-common-dev freeglut3-dev coinor-libipopt-dev \
    libblas-dev liblapack-dev gfortran liblapack-dev coinor-libipopt-dev \
    cmake gcc build-essential libglib2.0-dev
```

#### 2. 安装openjdk

```bash
sudo apt-get update
sudo apt-get install openjdk-8-jdk
```

#### 3. 安装LCM（1.3.1）

```bash
./configure    # 确保Java Support为Enabled
make
sudo make install
sudo ldconfig
```

#### 4. 安装Eigen

```bash
# 必须从Eigen官网下载安装包，不可直接用Ubuntu快捷方式
mkdir build
cd build
cmake ..
make install
```

#### 5. 安装Qt5.10

```bash
chmod a+x qt-opensource-linux-x64-5.10.0.run
./qt-opensource-linux-x64-5.10.0.run
```

**注意**：安装完Qt后需修改sim/CMakeLists.txt中的Qt安装路径。

#### 6. 安装git

```bash
sudo apt install git
```

---

## 实验2：四足机器人上位机代码编译与运行仿真

### 控制界面说明

- **左侧面板**：更改模拟器设置（默认载入Simulator-defaults.yaml）
- **中间面板**：更改机器人设置（control_mode参数）
- **右侧面板**：用户参数

**control_mode参数**：
- 0 → 初始状态
- 6 → JOINT_PD（关节PD控制）
- 3 → STAND_UP（站立）
- 4 → LOCOMOTION（运动）

**cmpc_gait参数**（步态选择）：

| 值 | 步态 |
|----|------|
| 1 | bound |
| 2 | pronk |
| 3 | gallop |
| 4 | stand |
| 5 | trot run |
| 6 | walk1 |
| 7 | walk2 |
| 8 | pace |

### 仿真操作

- 按**T键**使仿真尽可能快地进行
- 按**空格键**打开免费相机模式
- 相机模式下使用**W、A、S、D、R、F**键移动相机
- 鼠标左键拖动调整方向

### 编译步骤

```bash
cd wbc_v4
mkdir mc-build
cd mc-build
cmake -DMINI_CHEETAH_BUILD=TRUE ..
make -j     # 或 make -j4 / make -j8
```

### 运行仿真

```bash
# 在mc-build中运行
./user/MIT_Controller/mit_ctrl
```

### 控制机器人运动

在控制界面中：
1. 将use_rc值改为1
2. 将control_mode改为6（JOINT_PD）→ 电机上电
3. 将control_mode改为3（STAND_UP）→ 机器人站立
4. 将control_mode改为4（LOCOMOTION）→ 机器人以trot步态运动

---

## 实验3：实体机器人运行实验

### IP地址设置

**四足机器人IP地址信息**：
| 参数 | 值 |
|------|-----|
| IPv4 | 10.0.0.34 |
| 子网掩码 | 255.255.255.0 |
| 默认网关 | 10.0.0.1 |

**PC端IP设置**：10.0.0.n（n≠1且n≠34）

### 运行步骤

1. 打开机器人电源开关与电机开关
2. 编译代码：`cmake -DMINI_CHEETAH_BUILD=TRUE ..; make -j4或j8`
3. 连接UP board：`ssh user@10.0.0.34`
4. 发送代码到UP board：`../scripts/send_to_mini_cheetah.sh ./user/MIT_Controller/mit_ctrl`
5. 运行机器人：`cd ./robot-software/build; ./run_mc.sh ./mit_ctrl`
6. 使用遥控器控制

---

## 实验4：trot步态设计及验证

### 步态原理

- trot步态（对角小跑步态）**占空比为0.5**
- 每条腿抬起时间与在地面时间相等
- 处于一条对角线上的两条腿动作一致
- 与另一条对角线上的两条腿相位相反

### 实验步骤

1. 在ConvexMPCLocomotion.cpp中植入trot步态代码
2. 编译：`make -j4或j8`
3. 下载程序到机器人
4. 运行并控制

---

## 实验5：bound步态设计及验证

### 步态原理

- 两条前腿和两条后腿分别同时运动
- 主要利用后腿的蹬地动作完成跳跃
- **占空比为0.5**
- 前进时躯体会有明显的俯仰运动

### 实验步骤

1. 在ConvexMPCLocomotion.cpp中植入bound步态代码
2. 编译并下载
3. 将gait_type值改为**1**

---

## 实验6：pace步态设计及验证

### 步态原理

- **占空比为0.5**
- 同侧的两条腿同时迈步、相位相同

### 实验步骤

1. 植入pace步态代码
2. 编译并下载
3. 将gait_type值改为**8**

---

## 实验7：walk步态设计及验证

### 步态原理

- **占空比β≥0.75**
- 任意时刻至少有三条腿接触地面
- 是一种**静步态**
- 四条腿分别在不同时刻迈步

### 实验步骤

1. 植入walk步态代码
2. 编译并下载
3. 将gait_type值改为**6**

---

## 实验8：后空翻步态设计与运行

### 动作解析

1. **蹲下（预备）阶段**：完成初始化配置，电机产生向后与向上的加速度
2. **腾空阶段**：在惯性作用下完成腾空翻转
3. **落地缓冲阶段**：电机加速度保持为零，确保足端稳定着地

### 关键代码

- 后空翻执行函数在**FSM_State_BackFlip.cpp**文件的70～85行
- 蹲下阶段与落地缓冲阶段在**BackFlipCtrl.cpp**文件中调用

### 遥控器操作

1. 按E键到最上方 → 站立状态
2. 向下拨动G键 → 完成后空翻
3. **注意**：后空翻结束后先将E键拨到中间位置，再将G键拨到中间位置

---

## 实验9：四足机器人斜坡自适应调整实验

### 实验原理

当机器人在斜坡上行走时：
- 质心投影点向斜面负梯度方向偏移
- 落足点处于工作空间的上部或下部
- 需要调整躯干姿态以避免足尖处于极限位置

### 实验步骤

1. 将足端位置坐标映射植入ConvexMPCLocomotion.cpp
2. 将姿态调整算法植入代码
3. 编译：`make -j8`
4. 下载并运行
5. 使用MATLAB分析俯仰角数据

---

## 实验10：循迹测试实验

### 核心技术

1. **图像二值化**：削弱蓝色和绿色通道，加强红色通道，转为黑白图像
2. **腐蚀膨胀处理**：消除干扰点，突出目标红线
3. **轮廓提取**：使用`findCours()`函数，通过最大面积法寻找最大轮廓
4. **最小包围矩形**：使用`minAreaRect()`函数获取

### OpenCV函数

- `findCours()` - 寻找外轮廓
- `minAreaRect()` - 最小包围矩形

### 实验步骤

1. 使用Filezilla将循迹代码导入UP board
2. 编译：`qmake` → `make`
3. 查看摄像头端口号：`ls dev/video*`
4. 运行：`./track 0`

---

## 实验11：障碍物识别跟踪实验

### ROS（Robot Operating System）

- 通用机器人软件开发平台
- 是一个元操作系统（中间件）
- 核心：通信框架（松耦合分布式架构）
- 通信方式：**话题（Topic）** 和 **服务（Service）**

### YOLO目标检测

- YOLO（You Only Look Once）
- 能够实现实时检测
- 将一整张图片作为输入，直接在输出层回归出边界框的位置及类别
- 本书使用**YOLOv3**

### OpenCV模块

| 模块 | 功能 |
|------|------|
| Core | 核心功能，Mat类、XML读写类 |
| ImgProc | 图像处理（滤波、变换、直方图） |
| HighGUI | 高级图形界面，显示图片、窗口操作 |

### 硬件模块

#### Intel RealSense D435

| 参数 | 规格 |
|------|------|
| 类型 | RGBD深度相机 |
| 深度技术 | 立体成像 |
| 深度分辨率 | 最高**1280×720** |
| 深度帧率 | 最高**90fps** |
| 测距范围 | 0.28m～10m |
| RGB分辨率 | 1920×1080 |
| RGB帧率 | 30fps |
| 视场角 | 87°×58°（深度），69°×42°（RGB） |

#### NVIDIA Jetson TX2

| 参数 | 规格 |
|------|------|
| GPU | 256个CUDA核心（Pascal架构） |
| CPU | 双核Denver2 + 四核ARM Cortex-A57 |
| 内存 | 8GB 128-bit LPDDR4 |
| 显存带宽 | 59.7 GB/s |
| 功耗 | 7.5W～15W |
| 接口 | Wi-Fi、蓝牙、USB、HDMI |

### 实验流程

1. **ROS安装**（ROS Kinetic + Ubuntu 16.04）
2. **安装realsense开发包**
3. **启动摄像头节点**：`roslaunch realsense2_camera rs_camera.launch`
4. **启动检测节点**：`rosrun realsense position_publisher.py`
5. **启动深度图检测**：`rosrun realsense depth_publisher`
6. **启动运动控制**：`./run_mc.sh ./mit_ctrl`
7. **启动跟踪**：`rosrun realsense human_tracker`

### ROS安装步骤

```bash
# 1. 添加ROS镜像
sudo sh -c 'echo "deb http://packages.ros.org/ros/Ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# 2. 设置密钥
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# 3. 更新
sudo apt-get update

# 4. 安装ROS Kinetic完整版
sudo apt-get install ros-kinetic-desktop-full

# 5. 初始化rosdep
sudo rosdep init
rosdep update

# 6. 配置环境变量
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 提到的机器人型号/名称

| 名称 | 说明 |
|------|------|
| **MIT Mini Cheetah** | MIT仿生机器人实验室的四足机器人，重量约9kg（20磅），全球首个完成后空翻的四足机器人 |
| **MIT Cheetah 3** | MIT第三代猎豹四足机器人 |
| **Mini Cheetah** | 本书基于的开源控制框架（Cheetah-Software） |

## 提到的仿真工具和方法

| 工具 | 用途 |
|------|------|
| **Webots** | 四足机器人仿真平台，用于MPC方法验证 |
| **MATLAB** | 数据分析、俯仰角曲线绘制 |
| **Gazebo** | ROS生态系统中的仿真工具 |
| **SolidWorks** | 机械设计，模型导入Webots |
| **Visual Studio** | 上位机控制代码编写 |
| **Qt** | 控制界面开发 |
| **Eigen** | 矩阵运算库 |
| **qpOASES** | 开源二次规划求解器 |
| **LCM** | 轻量级通信与数据封送库 |

## 提到的硬件信息汇总

| 硬件 | 型号/规格 |
|------|----------|
| 主控制器 | **UP Board**（Intel Atom X5-Z8350, 4核, 1.44GHz, 4GB RAM） |
| 电机驱动板 | **STM32F446**（主频180MHz, 内置FPU） |
| 电源管理CPU | **STM32F103** |
| 磁编码器 | **AS5047P**（14位, 28krpm, SPI接口） |
| 深度相机 | **Intel RealSense D435**（RGBD, 1280×720, 90fps） |
| AI计算模块 | **NVIDIA Jetson TX2**（256 CUDA核心, Pascal架构） |
| IMU | VectorNav系列 |
| 电池 | **18650锂电池**（20A连续放电, 100A瞬间） |
| 电机驱动芯片 | **DRV8323** |
| 电机类型 | 无刷直流电机（盘式电机 + 行星减速机） |
| 通信总线 | **CAN总线**（1Mbps）、**SPI**（20-50Mbps）、**485总线** |
| 遥控器 | SBUS协议遥控器 |
| 编程环境 | Keil 5.27+, Ubuntu 16.04, Qt5.10 |
