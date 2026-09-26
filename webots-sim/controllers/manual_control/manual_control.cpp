// manual_control.cpp
// Webots 控制器：机器狗键盘/手柄手动遥控（跑、转、跳）
//
// 功能：
//   A. 键盘遥控：W/S 前后，A/D 平移，Q/E 转向，Shift 快跑，Space 跳，R 复位
//   B. 速度 → 步态：速度映射到 trot 步幅/步频，wz 左右腿差动，vy 转 abd 偏置
//   C. 预设跳跃状态机：下蹲→蹬地→空中→着地（一次触发不可叠加，Esc 可取消）
//   D. Linux 手柄（/dev/input/js0，无外部依赖）：摇杆平移/转向，A 跳，B 复位
//   E. 中文注释 + 每 500ms 状态打印
//
// 行动力基础复用自：webots-sim/controllers/ball_detector/ball_detector.cpp
// （PD 站立 + 开环 trot），关节名 / PD 参数按 YoboGo-10S 实机配置。
//
// 关键常量（YoboGo-10S）：
//   电机名 {fr,fl,hr,hl}_{abd,hip,kn}_motor/_sensor
//   MAX_TORQUE 18，KP 3.0，KD 1.0/0.2/0.2
//   站立高度 0.26（yaml des_p[2]），腿长 thigh 0.14 + shank 0.12

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/gyro.h>
#include <webots/accelerometer.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>

#include <cstdio>
#include <cmath>
#include <cstring>
#include <cstdlib>

// ---- Linux 手柄 API（无第三方依赖）----
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>

// ============================================================
// 常量 —— 几何 / 控制（YoboGo-10S）
// ============================================================
#define NUM_LEGS        4
#define NUM_JOINTS      3   // 每条腿：abd, hip, kn
#define TOTAL_JOINTS    (NUM_LEGS * NUM_JOINTS)

// 控制周期：Webots basicTimeStep = 4ms → 约 250Hz
static const double DT = 0.004;

// 腿编号
enum LegID { FR = 0, FL = 1, HR = 2, HL = 3 };

// 每条腿内的关节编号
enum JointID { ABD = 0, HIP = 1, KN = 2 };

// ============================================================
// PD 增益 / 力矩上限（YoboGo-10S 实机）
// ============================================================
static const double KP[3] = {3.0, 3.0, 3.0};
static const double KD[3] = {1.0, 0.2, 0.2};
static const double MAX_TORQUE = 18.0;

// 跳跃着地缓冲用软 PD（KP 降为 1.0）
static const double KP_LAND = 1.0;

// 站立关节角 [abd, hip, knee]（弧度）
// 角度 0 时腿竖直向下（本 Webots 模型约定）
static const double STANDING_DES[3] = {0.0, 0.0, 0.0};

// 站立高度（yaml des_p[2]），跳跃时显示值会变化
static const double STAND_HEIGHT = 0.26;

// trot 步态基础参数
static const double TROT_PERIOD_BASE = 0.5;    // 基础步态周期（秒）
static const double TROT_SWING_HEIGHT = 0.06;  // 摆动抬腿高度（米）
// trot 相位偏移：FR+HL 同相，FL+HR 同相（对角小跑）
static const double TROT_PHASE[NUM_LEGS] = {0.0, 0.5, 0.5, 0.0};

// 腿部运动学（YoboGo-10S，**不要**用 MiniCheetah 的 0.209/0.18）
static const double L_ABD = 0.065;   // abd 偏置（髋 y 向）
static const double L_HIP = 0.14;    // 大腿长度 [说明书/合同]
static const double L_KNEE = 0.12;   // 小腿长度 [说明书/合同]

// 髋部相对机体中心的位置 [FR, FL, HR, HL]（YoboGo-10S 髋安装点）
static const double HIP_POS[NUM_LEGS][3] = {
    { 0.18, -0.052, 0.0},  // FR
    { 0.18,  0.052, 0.0},  // FL
    {-0.18, -0.052, 0.0},  // HR
    {-0.18,  0.052, 0.0},  // HL
};

// ============================================================
// 常量 —— 手动遥控
// ============================================================
// 键盘速度目标
static const double VX_FORWARD  =  0.6;  // W 前进 (m/s)
static const double VX_BACKWARD = -0.3;  // S 后退 (m/s)
static const double VY_LEFT     =  0.3;  // A 左移 (m/s)
static const double VY_RIGHT    = -0.3;  // D 右移 (m/s)
static const double WZ_LEFT     =  1.0;  // Q 左转 (rad/s)
static const double WZ_RIGHT    = -1.0;  // E 右转 (rad/s)

// 快跑倍率（按住 Shift）
static const double SPRINT_FREQ_SCALE  = 1.6;  // 步频 ×1.6
static const double SPRINT_STRIDE_SCALE = 1.3; // 步幅 ×1.3
static const double SPRINT_SPEED_SCALE  = 1.3; // 速度上限 ×1.3

// 速度滑回 0 的一阶低通时间常数（秒）
static const double VEL_TAU = 0.3;

// 转向/平移映射到关节的系数
static const double YAW_HIP_BIAS = 0.1;   // 左右腿 hip 差动：±wz*0.1
static const double VY_ABD_BIAS = 0.15;   // q_des[ABD] += vy*0.15

// 速度阈值：低于此值视为静止（站立）
static const double VEL_EPS = 0.01;

// 状态打印周期：125 步 × 4ms = 500ms
static const int REPORT_PERIOD = 125;

// ============================================================
// 常量 —— 跳跃状态机
// ============================================================
enum JumpPhase {
    JUMP_NONE = 0,
    JUMP_CROUCH,  // 下蹲 0.15s
    JUMP_PUSH,    // 蹬地 0.08s
    JUMP_AIR,     // 空中 0.25s
    JUMP_LAND     // 着地 0.20s
};

static const double JUMP_CROUCH_TIME = 0.15;
static const double JUMP_PUSH_TIME   = 0.08;
static const double JUMP_AIR_TIME    = 0.25;
static const double JUMP_LAND_TIME   = 0.20;

// 下蹲深度：把所有关节目标往下压 0.06 rad
static const double CROUCH_DEPTH = 0.06;
// 下蹲时站立高度显示 0.26 → 0.15
static const double CROUCH_HEIGHT = 0.15;

// 蹬地：hip/knee 猛伸目标（开环跳跃，无落地姿态控制）
static const double PUSH_HIP = 0.45;
static const double PUSH_KNEE = 0.35;
// 空中收腿：knee -0.4
static const double AIR_KNEE = -0.4;

// ============================================================
// 设备句柄
// ============================================================
static WbDeviceTag motors[NUM_LEGS][NUM_JOINTS];
static WbDeviceTag sensors[NUM_LEGS][NUM_JOINTS];
static WbDeviceTag gyro, accelerometer, inertial_unit;

// 传感器数据
static double joint_pos[NUM_LEGS][NUM_JOINTS];
static double joint_vel[NUM_LEGS][NUM_JOINTS];
static double imu_orientation[4]; // 四元数 (w, x, y, z)
static double imu_angular_vel[3]; // rad/s
static double imu_accel[3];       // m/s^2

// ============================================================
// 控制器状态
// ============================================================
static double sim_time = 0.0;
static long step_count = 0;

// 速度指令（经低通后实际使用）
static double cmd_vx = 0.0;
static double cmd_vy = 0.0;
static double cmd_wz = 0.0;
static int sprint_mode = 0;   // 0=正常 1=快跑（Shift 按住）

// 跳跃状态机
static int jump_phase = JUMP_NONE;
static double jump_phase_time = 0.0;   // 当前阶段内已持续时间

// yaw 目标（wz 累加）
static double yaw_target = 0.0;

// 当前站立高度显示值
static double stand_height_display = STAND_HEIGHT;

// ============================================================
// 手柄状态（Linux joystick API）
// ============================================================
static int js_fd = -1;
static int js_warned = 0;   // 只提示一次
static double js_vx = 0.0, js_vy = 0.0, js_wz = 0.0;
static int js_jump_req = 0;   // 手柄 A 钮边沿
static int js_reset_req = 0;  // 手柄 B 钮边沿
static double js_axis[8] = {0};
static int js_button[12] = {0};

// ============================================================
// 电机 / 传感器名称模板（YoboGo-10S，不能改）
// ============================================================
static const char* MOTOR_NAMES[NUM_LEGS][NUM_JOINTS] = {
    {"fr_abd_motor", "fr_hip_motor", "fr_kn_motor"},
    {"fl_abd_motor", "fl_hip_motor", "fl_kn_motor"},
    {"hr_abd_motor", "hr_hip_motor", "hr_kn_motor"},
    {"hl_abd_motor", "hl_hip_motor", "hl_kn_motor"},
};

static const char* SENSOR_NAMES[NUM_LEGS][NUM_JOINTS] = {
    {"fr_abd_sensor", "fr_hip_sensor", "fr_kn_sensor"},
    {"fl_abd_sensor", "fl_hip_sensor", "fl_kn_sensor"},
    {"hr_abd_sensor", "hr_hip_sensor", "hr_kn_sensor"},
    {"hl_abd_sensor", "hl_hip_sensor", "hl_kn_sensor"},
};

// ============================================================
// 辅助函数 —— 设备初始化 / 传感读取 / PD
// ============================================================

static void initialize_devices() {
    // 初始化电机与编码器
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int j = 0; j < NUM_JOINTS; j++) {
            motors[leg][j] = wb_robot_get_device(MOTOR_NAMES[leg][j]);
            sensors[leg][j] = wb_robot_get_device(SENSOR_NAMES[leg][j]);
            if (motors[leg][j] == 0)
                printf("WARNING: 电机未找到: %s\n", MOTOR_NAMES[leg][j]);
            if (sensors[leg][j] == 0)
                printf("WARNING: 传感器未找到: %s\n", SENSOR_NAMES[leg][j]);
            wb_position_sensor_enable(sensors[leg][j], 1); // 1ms 采样
        }
    }

    // 初始化 IMU
    gyro = wb_robot_get_device("gyro");
    accelerometer = wb_robot_get_device("accelerometer");
    inertial_unit = wb_robot_get_device("inertial unit");
    wb_gyro_enable(gyro, 1);
    wb_accelerometer_enable(accelerometer, 1);
    wb_inertial_unit_enable(inertial_unit, 1);

    // 键盘：100ms 采样（与 ball_detector 一致）
    wb_keyboard_enable(100);

    memset(joint_pos, 0, sizeof(joint_pos));
    memset(joint_vel, 0, sizeof(joint_vel));
}

static void read_sensors() {
    // 读关节位置，差分求速度
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int j = 0; j < NUM_JOINTS; j++) {
            double new_pos = wb_position_sensor_get_value(sensors[leg][j]);
            if (!isnan(new_pos)) {
                joint_vel[leg][j] = (new_pos - joint_pos[leg][j]) / DT;
                joint_pos[leg][j] = new_pos;
            }
        }
    }

    // 读 IMU
    const double* gyro_val = wb_gyro_get_values(gyro);
    const double* accel_val = wb_accelerometer_get_values(accelerometer);
    const double* orient_val = wb_inertial_unit_get_quaternion(inertial_unit);

    if (gyro_val) {
        imu_angular_vel[0] = gyro_val[0];
        imu_angular_vel[1] = gyro_val[1];
        imu_angular_vel[2] = gyro_val[2];
    }
    if (accel_val) {
        imu_accel[0] = accel_val[0];
        imu_accel[1] = accel_val[1];
        imu_accel[2] = accel_val[2];
    }
    if (orient_val) {
        imu_orientation[0] = orient_val[0]; // w
        imu_orientation[1] = orient_val[1]; // x
        imu_orientation[2] = orient_val[2]; // y
        imu_orientation[3] = orient_val[3]; // z
    }
}

// 单关节 PD 力矩（可指定 kp/kd，用于着地软 PD）
static double joint_pd_control_gain(double q_des, double q, double qd_des, double qd,
                                    int joint_idx, double kp_scale) {
    double error = q_des - q;
    double vel_error = qd_des - qd;
    double torque = KP[joint_idx] * kp_scale * error + KD[joint_idx] * vel_error;

    // 限幅
    if (torque > MAX_TORQUE) torque = MAX_TORQUE;
    if (torque < -MAX_TORQUE) torque = -MAX_TORQUE;
    return torque;
}

// 默认 PD（kp_scale = 1.0）
static double joint_pd_control(double q_des, double q, double qd_des, double qd,
                               int joint_idx) {
    return joint_pd_control_gain(q_des, q, qd_des, qd, joint_idx, 1.0);
}

// ============================================================
// 手柄初始化 / 读取（Linux joystick API）
// ============================================================
static void init_joystick() {
    js_fd = open("/dev/input/js0", O_RDONLY | O_NONBLOCK);
    if (js_fd < 0) {
        // 无手柄则跳过、不报错（只打印一次提示）
        if (!js_warned) {
            printf("【手柄】/dev/input/js0 不存在，跳过手柄输入（仅用键盘）\n");
            js_warned = 1;
        }
        return;
    }
    printf("【手柄】已打开 /dev/input/js0\n");
}

// 读取所有待处理的手柄事件（非阻塞）
static void read_joystick() {
    if (js_fd < 0) return;

    struct js_event e;
    while (read(js_fd, &e, sizeof(e)) == (ssize_t)sizeof(e)) {
        int type = e.type & ~JS_EVENT_INIT; // 去掉初始化标志
        if (type == JS_EVENT_AXIS && e.number < 8) {
            // 轴值范围 -32767..32767
            double v = (double)e.value / 32767.0;
            if (v > 1.0) v = 1.0;
            if (v < -1.0) v = -1.0;
            js_axis[e.number] = v;

            // 轴 0=左摇杆X(平移Y) 1=左摇杆Y(前进X) 2=右摇杆X(转向) 3=右摇杆Y
            // 游戏手柄 Y 轴上推为负 → 前进取负号
            const double deadzone = 0.12;
            double a0 = (fabs(js_axis[0]) > deadzone) ? js_axis[0] : 0.0;
            double a1 = (fabs(js_axis[1]) > deadzone) ? js_axis[1] : 0.0;
            double a2 = (fabs(js_axis[2]) > deadzone) ? js_axis[2] : 0.0;

            js_vy = a0 * 0.3;          // 左右平移
            js_vx = -a1 * 0.6;         // 前进（上推为正）
            js_wz = -a2 * 1.0;         // 转向（右推为负 wz=右转）
        } else if (type == JS_EVENT_BUTTON && e.number < 12) {
            int prev = js_button[e.number];
            js_button[e.number] = e.value;
            // 钮 0=A(跳) 1=B(复位)，按下沿触发
            if (e.value && !prev) {
                if (e.number == 0) js_jump_req = 1;
                if (e.number == 1) js_reset_req = 1;
            }
        }
    }
}

// ============================================================
// 键盘读取 —— 解析为一次控制指令
// ============================================================
// Webots 键码：普通 ASCII；Shift 修饰位可能在 0x10000 或 0x100000
// 任务要求用 key & 0x10000 判断；这里两种都兼容。
static int key_has_shift(int key) {
    return ((key & 0x10000) != 0) || ((key & 0x100000) != 0);
}

// 大小写都兼容：把字母键规范成大写
static int norm_alpha(int base) {
    if (base >= 'a' && base <= 'z') return base - 'a' + 'A';
    return base;
}

// 一次键盘解析结果
struct KeyCmd {
    int moving;       // 有方向键按下
    int want_jump;    // 请求跳跃
    int want_reset;   // 请求复位
    int want_escape;  // Esc 按下
    int sprint;       // Shift 按住（快跑档）
    double vx, vy, wz;
};

// 只调用一次 wb_keyboard_get_key()，解析出全部指令
static KeyCmd parse_keyboard(int key) {
    KeyCmd c;
    memset(&c, 0, sizeof(c));
    c.vx = c.vy = c.wz = 0.0;

    // 无键：wb_keyboard_get_key() 返回 -1（或 0）
    // 注意：必须先判 key<0，否则 -1 的位模式会误判成 Shift 修饰
    if (key < 0 || key == 0) return c;

    int has_shift = key_has_shift(key);
    int base = key & 0xFFFF;

    // Shift 修饰位本身可能占满低位：单独 Shift 按住也算快跑档
    if (has_shift) c.sprint = 1;

    // 只剩 Shift 修饰位（低位为 0）：仅快跑档
    if (base == 0) return c;

    base = norm_alpha(base);
    double speed_scale = has_shift ? SPRINT_SPEED_SCALE : 1.0;

    switch (base) {
        case 'W': // 前进
            c.vx = VX_FORWARD * speed_scale;
            c.moving = 1;
            break;
        case 'S': // 后退
            c.vx = VX_BACKWARD * speed_scale;
            c.moving = 1;
            break;
        case 'A': // 左移
            c.vy = VY_LEFT * speed_scale;
            c.moving = 1;
            break;
        case 'D': // 右移
            c.vy = VY_RIGHT * speed_scale;
            c.moving = 1;
            break;
        case 'Q': // 左转
            c.wz = WZ_LEFT;
            c.moving = 1;
            break;
        case 'E': // 右转
            c.wz = WZ_RIGHT;
            c.moving = 1;
            break;
        case ' ': // Space 跳跃
            c.want_jump = 1;
            break;
        case 'R': // 复位
            c.want_reset = 1;
            break;
        case 27:  // Esc
            c.want_escape = 1;
            break;
        default:
            break;
    }
    return c;
}

// ============================================================
// 速度指令更新 —— 有键直接跟随；无键一阶低通滑向 0（τ=0.3s）
// ============================================================
static void update_velocity(const KeyCmd& k) {
    if (k.moving) {
        // 有方向键：直接跟随本次目标
        cmd_vx = k.vx;
        cmd_vy = k.vy;
        cmd_wz = k.wz;
    } else {
        // 无键 / Esc：一阶低通滑向 0
        double alpha = 1.0 - exp(-DT / VEL_TAU);
        cmd_vx += (0.0 - cmd_vx) * alpha;
        cmd_vy += (0.0 - cmd_vy) * alpha;
        cmd_wz += (0.0 - cmd_wz) * alpha;
    }
    sprint_mode = k.sprint ? 1 : 0;

    // 速度上限（快跑 ×1.3）
    double vmax = 1.0 * (sprint_mode ? SPRINT_SPEED_SCALE : 1.0);
    if (cmd_vx >  0.6 * vmax) cmd_vx =  0.6 * vmax;
    if (cmd_vx < -0.3 * vmax) cmd_vx = -0.3 * vmax;
    if (cmd_vy >  0.3 * vmax) cmd_vy =  0.3 * vmax;
    if (cmd_vy < -0.3 * vmax) cmd_vy = -0.3 * vmax;
    if (cmd_wz >  WZ_LEFT)  cmd_wz =  WZ_LEFT;
    if (cmd_wz <  WZ_RIGHT) cmd_wz =  WZ_RIGHT;

    // wz 累加到 yaw 目标
    yaw_target += cmd_wz * DT;

    // 融合手柄（手柄有输入时覆盖键盘速度）
    if (js_fd >= 0 && (fabs(js_vx) > VEL_EPS || fabs(js_vy) > VEL_EPS || fabs(js_wz) > VEL_EPS)) {
        cmd_vx = js_vx;
        cmd_vy = js_vy;
        cmd_wz = js_wz;
    }
}

// ============================================================
// 跳跃状态机
// ============================================================
static const char* jump_phase_name(int p) {
    switch (p) {
        case JUMP_NONE:   return "NONE";
        case JUMP_CROUCH: return "CROUCH(下蹲)";
        case JUMP_PUSH:   return "PUSH(蹬地)";
        case JUMP_AIR:    return "AIR(空中)";
        case JUMP_LAND:   return "LAND(着地)";
        default:          return "?";
    }
}

static void jump_start() {
    if (jump_phase != JUMP_NONE) return; // 一次触发不可叠加
    jump_phase = JUMP_CROUCH;
    jump_phase_time = 0.0;
    printf("【跳跃】阶段=CROUCH(下蹲) 开始\n");
}

static void jump_cancel() {
    if (jump_phase == JUMP_NONE) return;
    printf("【跳跃】阶段=%s 被 Esc 取消，回站立\n", jump_phase_name(jump_phase));
    jump_phase = JUMP_NONE;
    jump_phase_time = 0.0;
    stand_height_display = STAND_HEIGHT;
}

// 每步推进跳跃状态机；Esc / R 可打断
static void update_jump_state(int want_jump, int want_reset, int esc_pressed) {
    // 复位键随时生效
    if (want_reset) {
        jump_cancel();
        cmd_vx = cmd_vy = cmd_wz = 0.0;
        yaw_target = 0.0;
        return;
    }

    // 跳跃中按 Esc 取消回站立
    if (esc_pressed && jump_phase != JUMP_NONE) {
        jump_cancel();
        return;
    }

    // 触发跳跃（仅 NONE 态）
    if (want_jump && jump_phase == JUMP_NONE) {
        jump_start();
    }

    if (jump_phase == JUMP_NONE) {
        stand_height_display = STAND_HEIGHT;
        return;
    }

    jump_phase_time += DT;

    switch (jump_phase) {
        case JUMP_CROUCH:
            stand_height_display = STAND_HEIGHT +
                (CROUCH_HEIGHT - STAND_HEIGHT) * (jump_phase_time / JUMP_CROUCH_TIME);
            if (jump_phase_time >= JUMP_CROUCH_TIME) {
                jump_phase = JUMP_PUSH;
                jump_phase_time = 0.0;
                printf("【跳跃】阶段=PUSH(蹬地)\n");
            }
            break;
        case JUMP_PUSH:
            stand_height_display = CROUCH_HEIGHT;
            if (jump_phase_time >= JUMP_PUSH_TIME) {
                jump_phase = JUMP_AIR;
                jump_phase_time = 0.0;
                printf("【跳跃】阶段=AIR(空中)\n");
            }
            break;
        case JUMP_AIR:
            stand_height_display = CROUCH_HEIGHT + 0.10; // 空中高度略升（显示用）
            if (jump_phase_time >= JUMP_AIR_TIME) {
                jump_phase = JUMP_LAND;
                jump_phase_time = 0.0;
                printf("【跳跃】阶段=LAND(着地)\n");
            }
            break;
        case JUMP_LAND:
            stand_height_display = STAND_HEIGHT;
            if (jump_phase_time >= JUMP_LAND_TIME) {
                jump_phase = JUMP_NONE;
                jump_phase_time = 0.0;
                stand_height_display = STAND_HEIGHT;
                printf("【跳跃】阶段=NONE，回到站立\n");
            }
            break;
        default:
            break;
    }
}

// ============================================================
// 行动力模式
// ============================================================

// 站立控制 —— 保持中立位姿
static void control_standing() {
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int j = 0; j < NUM_JOINTS; j++) {
            double torque = joint_pd_control(
                STANDING_DES[j], joint_pos[leg][j],
                0.0, joint_vel[leg][j], j);
            wb_motor_set_torque(motors[leg][j], torque);
        }
    }
}

// trot 步态控制（含速度映射 / 差动转向 / vy→abd）
static void control_trot() {
    // ---- 速度 → 步态参数 ----
    double speed_mag = sqrt(cmd_vx * cmd_vx + cmd_vy * cmd_vy);

    // 步幅：hip_sweep = 0.25 + 0.15*|vx|；快跑再 ×1.3
    double hip_sweep = (0.25 + 0.15 * fabs(cmd_vx)) *
                       (sprint_mode ? SPRINT_STRIDE_SCALE : 1.0);

    // 步态周期：TROT_PERIOD = 0.5 / (1.0 + 0.6*speed_factor)
    // speed_factor 按实际速度比例（0~1），速度越快周期越短
    double speed_factor = speed_mag / 0.6;
    if (speed_factor > 1.0) speed_factor = 1.0;
    double trot_period = TROT_PERIOD_BASE / (1.0 + 0.6 * speed_factor);

    // 快跑：步频 ×1.6（周期再缩短为 1/1.6）
    if (sprint_mode) trot_period /= SPRINT_FREQ_SCALE;

    double phase = fmod(sim_time / trot_period, 1.0);

    for (int leg = 0; leg < NUM_LEGS; leg++) {
        double leg_phase = fmod(phase + TROT_PHASE[leg], 1.0);

        double q_des[NUM_JOINTS] = {0.0, 0.0, 0.0};
        double qd_des[NUM_JOINTS] = {0.0, 0.0, 0.0};

        if (leg_phase < 0.5) {
            // 支撑相：腿着地向后蹬
            double stance_progress = leg_phase / 0.5; // 0 -> 1
            double hip_angle = hip_sweep * (1.0 - 2.0 * stance_progress);
            q_des[HIP] = hip_angle;
            qd_des[HIP] = -hip_sweep * 2.0 / 0.5;
            q_des[KN] = 0.0; // 膝伸直
        } else {
            // 摆动相：抬起并回摆
            double swing_progress = (leg_phase - 0.5) / 0.5; // 0 -> 1
            double swing_hip = hip_sweep * (1.0 - 2.0 * swing_progress);
            q_des[HIP] = swing_hip;
            qd_des[HIP] = 0.0;

            // 钟形抬膝（L_KNEE 用 YoboGo 的 0.12）
            double lift = TROT_SWING_HEIGHT;
            double swing_knee = -lift / L_KNEE * sin(swing_progress * M_PI);
            q_des[KN] = swing_knee;
        }

        // ---- 转向 wz：左右腿差动（左侧 +wz*0.1，右侧 -wz*0.1）----
        int is_left = (leg == FL || leg == HL);
        double hip_bias = is_left ? (cmd_wz * YAW_HIP_BIAS) : (-cmd_wz * YAW_HIP_BIAS);
        q_des[HIP] += hip_bias;

        // ---- 平移 vy：累加到 abd 关节偏置 q_des[ABD] += vy*0.15 ----
        q_des[ABD] += cmd_vy * VY_ABD_BIAS;

        // PD 控制
        for (int j = 0; j < NUM_JOINTS; j++) {
            double torque = joint_pd_control(
                q_des[j], joint_pos[leg][j],
                qd_des[j], joint_vel[leg][j], j);
            wb_motor_set_torque(motors[leg][j], torque);
        }
    }
}

// 跳跃控制 —— 按状态机阶段生成关节目标
static void control_jump() {
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        double q_des[NUM_JOINTS] = {STANDING_DES[0], STANDING_DES[1], STANDING_DES[2]};
        double qd_des[NUM_JOINTS] = {0.0, 0.0, 0.0};
        double kp_scale = 1.0;

        switch (jump_phase) {
            case JUMP_CROUCH: {
                // 下蹲：stand_height 0.26→0.15，关节目标下压 0.06
                double t = jump_phase_time / JUMP_CROUCH_TIME;
                if (t > 1.0) t = 1.0;
                double drop = CROUCH_DEPTH * t;
                q_des[HIP] = STANDING_DES[HIP] - drop;
                q_des[KN]  = STANDING_DES[KN]  - drop;
                q_des[ABD] = STANDING_DES[ABD];
                break;
            }
            case JUMP_PUSH: {
                // 蹬地：四腿 hip/knee 猛伸
                // kp_scale=13 → 初始力矩 3*13*0.45≈17.5Nm（贴近 MAX_TORQUE=18），
                // 力矩随伸直自然回落，避免 bang-bang 硬冲击
                q_des[HIP] = PUSH_HIP;
                q_des[KN]  = PUSH_KNEE;
                q_des[ABD] = 0.0;
                kp_scale = 13.0;
                break;
            }
            case JUMP_AIR: {
                // 空中：收腿（knee -0.4）
                q_des[HIP] = 0.1;          // 髋略收
                q_des[KN]  = AIR_KNEE;     // knee -0.4
                q_des[ABD] = 0.0;
                break;
            }
            case JUMP_LAND: {
                // 着地：软 PD（KP 降为 1.0）缓冲 0.2s
                q_des[HIP] = STANDING_DES[HIP];
                q_des[KN]  = STANDING_DES[KN];
                q_des[ABD] = STANDING_DES[ABD];
                kp_scale = KP_LAND / KP[HIP]; // 使等效 KP=1.0
                break;
            }
            default:
                break;
        }

        for (int j = 0; j < NUM_JOINTS; j++) {
            double torque = joint_pd_control_gain(
                q_des[j], joint_pos[leg][j],
                qd_des[j], joint_vel[leg][j], j, kp_scale);
            wb_motor_set_torque(motors[leg][j], torque);
        }
    }
}

// ============================================================
// 状态打印（每 500ms）
// ============================================================
static void print_status() {
    const char* speed_name = sprint_mode ? "快跑" : "正常";
    const char* mode_name;
    if (jump_phase != JUMP_NONE) {
        mode_name = "跳";
    } else if (fabs(cmd_vx) > VEL_EPS || fabs(cmd_vy) > VEL_EPS || fabs(cmd_wz) > VEL_EPS) {
        mode_name = "走";
    } else {
        mode_name = "站";
    }

    printf("【手动】vx=%.2f vy=%.2f wz=%.2f 速度档=%s 模式=%s 站立高度=%.3f\n",
           cmd_vx, cmd_vy, cmd_wz, speed_name, mode_name, stand_height_display);

    // 附加关节反馈，便于确认关节是否响应
    printf("【关节】FR_hip=%.3f FR_kn=%.3f FL_hip=%.3f FL_kn=%.3f "
           "HR_hip=%.3f HR_kn=%.3f HL_hip=%.3f HL_kn=%.3f\n",
           joint_pos[FR][HIP], joint_pos[FR][KN],
           joint_pos[FL][HIP], joint_pos[FL][KN],
           joint_pos[HR][HIP], joint_pos[HR][KN],
           joint_pos[HL][HIP], joint_pos[HL][KN]);
}

static void print_help() {
    printf("=== Manual Control Controller（手动遥控）===\n");
    printf("目标机型：YoboGo-10S（MAX_TORQUE=18, 站立高度=0.26, 腿长 0.14+0.12）\n");
    printf("──────────── 按键表 ────────────\n");
    printf("  W / S        前进 vx=+0.6 / 后退 vx=-0.3 m/s\n");
    printf("  A / D        左移 vy=+0.3 / 右移 vy=-0.3 m/s\n");
    printf("  Q / E        左转 wz=+1.0 / 右转 wz=-1.0 rad/s\n");
    printf("  Shift(按住)  快跑：步频×1.6 步幅×1.3 速度上限×1.3\n");
    printf("  Space        跳跃（下蹲→蹬地→空中→着地，不可叠加）\n");
    printf("  R            复位（回站立，清速度）\n");
    printf("  Esc / 无键   速度滑回 0（一阶低通 τ=0.3s）\n");
    printf("──────────── 手柄 ────────────\n");
    printf("  左摇杆 X/Y   平移 vy / 前进 vx\n");
    printf("  右摇杆 X     转向 wz\n");
    printf("  A 钮         跳跃    B 钮：复位\n");
    printf("────────────────────────────────\n");
    printf("每 500ms 打印状态：【手动】vx=.. vy=.. wz=..\n");
    printf("==========================================\n");
}

// ============================================================
// 自动测试钩子（仅当环境变量 MC_AUTOTEST=1 时启用）
// 用于无键盘的冒烟测试：按时间轴合成 W / Space / Q / R 指令
// ============================================================
static int autotest_enabled = 0;

static void init_autotest() {
    const char* s = getenv("MC_AUTOTEST");
    autotest_enabled = (s && s[0] == '1');
    if (autotest_enabled)
        printf("【自测】MC_AUTOTEST=1，启用合成按键（W→跳→Q→R 时间轴）\n");
}

static KeyCmd autotest_key(double t) {
    KeyCmd c;
    memset(&c, 0, sizeof(c));
    if (t >= 1.0 && t < 3.0) {
        c.vx = VX_FORWARD;   // 前进 trot
        c.moving = 1;
    } else if (t >= 3.5 && t < 3.5 + DT * 2) {
        c.want_jump = 1;     // 触发一次跳跃
    } else if (t >= 5.5 && t < 7.5) {
        c.wz = WZ_LEFT;      // 左转
        c.moving = 1;
    } else if (t >= 8.0 && t < 8.0 + DT * 2) {
        c.want_reset = 1;    // 复位
    }
    return c;
}

// ============================================================
// 主循环
// ============================================================
int main() {
    wb_robot_init();

    print_help();
    initialize_devices();
    init_joystick();
    init_autotest();

    // 等待传感器稳定
    wb_robot_step(100);

    while (wb_robot_step(4) != -1) { // 4ms 时间步
        sim_time += DT;
        step_count++;

        // 读全部传感器
        read_sensors();

        // ---- 键盘：每步只读一次，解析为完整指令 ----
        // 自测模式下用合成按键覆盖，便于无键盘冒烟验证
        KeyCmd key = autotest_enabled ? autotest_key(sim_time)
                                      : parse_keyboard(wb_keyboard_get_key());

        // ---- 手柄事件 + 跳/复位边沿 ----
        read_joystick();
        int want_jump = key.want_jump;
        int want_reset = key.want_reset;
        if (js_jump_req) {
            want_jump = 1;
            js_jump_req = 0;
        }
        if (js_reset_req) {
            want_reset = 1;
            js_reset_req = 0;
        }

        // 更新速度（无键低通归零 / 有键跟随 / 手柄融合）
        update_velocity(key);

        // 跳跃状态机推进（Esc 可取消）
        update_jump_state(want_jump, want_reset, key.want_escape);

        // ---- 行动力 ----
        if (jump_phase != JUMP_NONE) {
            control_jump();
        } else if (fabs(cmd_vx) < VEL_EPS && fabs(cmd_vy) < VEL_EPS &&
                   fabs(cmd_wz) < VEL_EPS) {
            control_standing();
        } else {
            control_trot();
        }

        // ---- 每 500ms 打印状态 ----
        if ((step_count % REPORT_PERIOD) == 0) {
            print_status();
        }
    }

    if (js_fd >= 0) close(js_fd);
    wb_robot_cleanup();
    return 0;
}
