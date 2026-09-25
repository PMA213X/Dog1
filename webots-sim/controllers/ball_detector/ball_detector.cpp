// ball_detector.cpp
// Webots 控制器：四足行走 + 橙色足球检测（视觉管线里程碑 1）
// 功能：
//   A. PD 站立 + 小步慢跑 trot（复用 mini_cheetah_controller 的步态逻辑）
//   B. 每 5 步（约 20 Hz）用纯 C 像素循环检测橙色球（无 OpenCV 依赖）
//   C. 每 500 ms 打印检测结果，并把标注帧写成 PPM 到 /tmp/ball_detect/
//   D. 默认进入 trot，键盘 S/T/R 仍可切换站立 / 慢跑 / 复位
//
// 行动力部分来源：webots-sim/controllers/mini_cheetah_controller/mini_cheetah_controller.cpp
// 视觉部分：world 球 baseColor 0.9 0.35 0.1 → 典型 sRGB ≈ (230, 90, 25)

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/gyro.h>
#include <webots/accelerometer.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/camera.h>

#include <cstdio>
#include <cmath>
#include <cstring>
#include <cstdlib>

// ============================================================
// 常量 —— 行动力（与 mini_cheetah_controller 一致）
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
// PD 增益（来自教材 / MIT 控制器参数）
// 关节 PD：kp_joint = [3, 3, 3], kd_joint = [1, 0.2, 0.2]
// ============================================================
static const double KP[3] = {3.0, 3.0, 3.0};
static const double KD[3] = {1.0, 0.2, 0.2};
static const double MAX_TORQUE = 15.0;

// 站立关节角 [abd, hip, knee]（弧度）
// 角度 0 时腿竖直向下（本 Webots 模型约定）
static const double STANDING_DES[3] = {0.0, 0.0, 0.0};

// trot 步态参数
static const double TROT_PERIOD = 0.5;        // 一个步态周期（秒）
static const double TROT_STANCE_TIME = 0.25;  // 支撑相时长
static const double TROT_SWING_HEIGHT = 0.06; // 摆动抬腿高度（米，加大便于观察）
// trot 相位偏移：FR+HL 同相，FL+HR 同相（对角小跑）
static const double TROT_PHASE[NUM_LEGS] = {0.0, 0.5, 0.5, 0.0};

// 腿部正运动学（简化）
// abd_link_length = 0.062, hip_link_length = 0.209, knee_link = 0.18
static const double L_ABD = 0.062;
static const double L_HIP = 0.209;
static const double L_KNEE = 0.18;

// 髋部相对机体中心的位置 [FR, FL, HR, HL]
static const double HIP_POS[NUM_LEGS][3] = {
    { 0.19, -0.111, 0.0},  // FR: x=0.19, y=-(0.049+0.062)
    { 0.19,  0.111, 0.0},  // FL
    {-0.19, -0.111, 0.0},  // HR
    {-0.19,  0.111, 0.0},  // HL
};

// ============================================================
// 常量 —— 视觉（里程碑 1：阈值 + 针孔成像估距）
// ============================================================
#define OUT_DIR         "/tmp/ball_detect"
#define MAX_IMG_W       640
#define MAX_IMG_H       480

static const double REAL_BALL_R = 0.11;  // 球真实半径（米），FIFA 5 号球
static const double CAM_FOV    = 1.05;   // 水平视场角（弧度），与 world fieldOfView 一致
static const int    DETECT_PERIOD = 5;   // 每 5 步检测一次（4ms*5=20ms，约 20Hz）
static const int    REPORT_PERIOD = 125; // 每 125 步 = 500ms 打印/存图一次

// ============================================================
// 设备句柄
// ============================================================
static WbDeviceTag motors[NUM_LEGS][NUM_JOINTS];
static WbDeviceTag sensors[NUM_LEGS][NUM_JOINTS];
static WbDeviceTag gyro, accelerometer, inertial_unit;
static WbDeviceTag cam = 0;

// 传感器数据
static double joint_pos[NUM_LEGS][NUM_JOINTS];
static double joint_vel[NUM_LEGS][NUM_JOINTS];
static double prev_joint_pos[NUM_LEGS][NUM_JOINTS];
static double imu_orientation[4]; // 四元数 (w, x, y, z)
static double imu_angular_vel[3]; // rad/s
static double imu_accel[3];       // m/s^2

// 控制器状态
static double sim_time = 0.0;
static int control_mode = 1; // 0=站立, 1=trot（默认慢跑，便于观察视野变化）
static long step_count = 0;  // 已执行的 wb_robot_step 次数

// 检测结果（每次检测后更新，打印/存图时读取）
static int    det_found = 0;   // 本周期是否找到球
static double det_cx = 0.0;    // 质心 x（像素）
static double det_cy = 0.0;    // 质心 y（像素）
static long   det_n = 0;       // 匹配像素数
static int    det_x0 = 0, det_y0 = 0, det_x1 = 0, det_y1 = 0; // 外接框
static double det_dist = 0.0;  // 距离估计（米）
static double det_bearing = 0.0; // 方位角（弧度，正=右侧）

// 标注帧缓冲（RGB，最多 640*480*3）
static unsigned char frame_buf[MAX_IMG_W * MAX_IMG_H * 3];
static long frame_index = 0; // 已保存的帧序号

// ============================================================
// 电机 / 传感器名称模板
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
// 辅助函数 —— 设备初始化 / 传感读取 / 行动力
// ============================================================

static void initialize_devices() {
    // 初始化电机与编码器
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int j = 0; j < NUM_JOINTS; j++) {
            motors[leg][j] = wb_robot_get_device(MOTOR_NAMES[leg][j]);
            sensors[leg][j] = wb_robot_get_device(SENSOR_NAMES[leg][j]);
            if (motors[leg][j] == 0)
                printf("WARNING: Motor not found: %s\n", MOTOR_NAMES[leg][j]);
            if (sensors[leg][j] == 0)
                printf("WARNING: Sensor not found: %s\n", SENSOR_NAMES[leg][j]);
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

    // 键盘
    wb_keyboard_enable(100); // 100ms 采样

    // 前置相机（橙球检测）
    cam = wb_robot_get_device("front_camera");
    if (cam == 0) {
        printf("WARNING: front_camera 未找到，视觉检测将不可用\n");
    } else {
        wb_camera_enable(cam, 100); // 100ms 采样周期
    }

    // 上一拍位置清零
    memset(prev_joint_pos, 0, sizeof(prev_joint_pos));
    memset(joint_pos, 0, sizeof(joint_pos));
    memset(joint_vel, 0, sizeof(joint_vel));
}

static void read_sensors() {
    // 读关节位置
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int j = 0; j < NUM_JOINTS; j++) {
            double new_pos = wb_position_sensor_get_value(sensors[leg][j]);
            if (!isnan(new_pos)) {
                joint_vel[leg][j] = (new_pos - joint_pos[leg][j]) / DT;
                prev_joint_pos[leg][j] = joint_pos[leg][j];
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

// 单关节 PD 力矩
static double joint_pd_control(double q_des, double q, double qd_des, double qd,
                                int joint_idx) {
    double error = q_des - q;
    double vel_error = qd_des - qd;
    double torque = KP[joint_idx] * error + KD[joint_idx] * vel_error;

    // 限幅
    if (torque > MAX_TORQUE) torque = MAX_TORQUE;
    if (torque < -MAX_TORQUE) torque = -MAX_TORQUE;
    return torque;
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

// trot 步态控制
// 对角腿同相：FR+HL, FL+HR
// 相位 0.0 = 支撑相开始，0.5 = 进入摆动相
static void control_trot() {
    double phase = fmod(sim_time / TROT_PERIOD, 1.0);

    for (int leg = 0; leg < NUM_LEGS; leg++) {
        // 该腿的相位
        double leg_phase = fmod(phase + TROT_PHASE[leg], 1.0);

        double q_des[NUM_JOINTS] = {0.0, 0.0, 0.0};
        double qd_des[NUM_JOINTS] = {0.0, 0.0, 0.0};

        if (leg_phase < 0.5) {
            // 支撑相：腿着地向后蹬
            // 髋关节支撑相内向前（正向）扫过
            double stance_progress = leg_phase / 0.5; // 0 -> 1
            double hip_angle = 0.35 * (1.0 - 2.0 * stance_progress); // 扫过 +/-0.35 rad（加大步幅便于观察）
            q_des[HIP] = hip_angle;
            qd_des[HIP] = -0.35 * 2.0 / 0.5; // 扫过速度
            q_des[KN] = 0.0; // 膝伸直
        } else {
            // 摆动相：抬起并回摆
            double swing_progress = (leg_phase - 0.5) / 0.5; // 0 -> 1
            // 钟形摆动轨迹
            double swing_hip = 0.35 * (1.0 - 2.0 * swing_progress);
            q_des[HIP] = swing_hip;
            qd_des[HIP] = 0.0;

            // 钟形抬膝
            double lift = TROT_SWING_HEIGHT;
            double swing_knee = -lift / L_KNEE * sin(swing_progress * M_PI);
            q_des[KN] = swing_knee;
        }

        // 外展：保持 0（简单 trot 无侧向运动）
        q_des[ABD] = 0.0;

        // PD 控制
        for (int j = 0; j < NUM_JOINTS; j++) {
            double torque = joint_pd_control(
                q_des[j], joint_pos[leg][j],
                qd_des[j], joint_vel[leg][j], j);
            wb_motor_set_torque(motors[leg][j], torque);
        }
    }
}

// ============================================================
// 辅助函数 —— 纯 C 绘图 / PPM 输出
// ============================================================

// 写单个 RGB 像素（越界自动忽略）
static void set_pixel(unsigned char* img, int w, int h, int x, int y,
                      unsigned char r, unsigned char g, unsigned char b) {
    if (x < 0 || y < 0 || x >= w || y >= h) return;
    unsigned char* p = img + 3 * (y * w + x);
    p[0] = r; p[1] = g; p[2] = b;
}

// 水平线段
static void draw_hline(unsigned char* img, int w, int h, int x0, int x1, int y,
                       unsigned char r, unsigned char g, unsigned char b) {
    if (x0 > x1) { int t = x0; x0 = x1; x1 = t; }
    for (int x = x0; x <= x1; x++)
        set_pixel(img, w, h, x, y, r, g, b);
}

// 垂直线段
static void draw_vline(unsigned char* img, int w, int h, int x, int y0, int y1,
                       unsigned char r, unsigned char g, unsigned char b) {
    if (y0 > y1) { int t = y0; y0 = y1; y1 = t; }
    for (int y = y0; y <= y1; y++)
        set_pixel(img, w, h, x, y, r, g, b);
}

// 十字（半宽 arm）
static void draw_cross(unsigned char* img, int w, int h, int cx, int cy, int arm,
                       unsigned char r, unsigned char g, unsigned char b) {
    draw_hline(img, w, h, cx - arm, cx + arm, cy, r, g, b);
    draw_vline(img, w, h, cx, cy - arm, cy + arm, r, g, b);
}

// 矩形框（描边）
static void draw_rect(unsigned char* img, int w, int h, int x0, int y0, int x1, int y1,
                      unsigned char r, unsigned char g, unsigned char b) {
    draw_hline(img, w, h, x0, x1, y0, r, g, b);
    draw_hline(img, w, h, x0, x1, y1, r, g, b);
    draw_vline(img, w, h, x0, y0, y1, r, g, b);
    draw_vline(img, w, h, x1, y0, y1, r, g, b);
}

// 把 RGB 帧写成 PPM (P6) 文件
static int save_ppm(const char* path, const unsigned char* rgb, int w, int h) {
    FILE* f = fopen(path, "wb");
    if (!f) {
        printf("WARNING: 无法写入 %s\n", path);
        return -1;
    }
    fprintf(f, "P6\n%d %d\n255\n", w, h);
    fwrite(rgb, 1, (size_t)w * h * 3, f);
    fclose(f);
    return 0;
}

// ============================================================
// 橙色球检测（纯 C 像素循环，无 OpenCV）
// 图像格式：Webots 相机缓冲为 BGRA、4 字节/像素
//   （见 camera.h: wb_camera_image_get_red/green/blue 宏）
// 阈值规则（球 baseColor 0.9 0.35 0.1 → sRGB ≈ 230,90,25）：
//   r > 150 && g > 40 && g < 160 && b < 90 && r > g + 40 && g > b
// ============================================================
static void detect_ball(const unsigned char* im, int w, int h) {
    long n = 0;
    long sum_x = 0, sum_y = 0;
    int x0 = w, y0 = h, x1 = -1, y1 = -1;

    // 整幅扫描（640x480 全图；如需加速可只扫上 2/3）
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int r = wb_camera_image_get_red(im, w, x, y);
            int g = wb_camera_image_get_green(im, w, x, y);
            int b = wb_camera_image_get_blue(im, w, x, y);
            if (r > 150 && g > 40 && g < 160 && b < 90 && r > g + 40 && g > b) {
                n++;
                sum_x += x;
                sum_y += y;
                if (x < x0) x0 = x;
                if (x > x1) x1 = x;
                if (y < y0) y0 = y;
                if (y > y1) y1 = y;
            }
        }
    }

    det_n = n;
    if (n <= 0) {
        det_found = 0;
        det_cx = det_cy = 0.0;
        det_dist = det_bearing = 0.0;
        det_x0 = det_y0 = det_x1 = det_y1 = 0;
        return;
    }

    det_found = 1;
    det_cx = (double)sum_x / (double)n;
    det_cy = (double)sum_y / (double)n;
    det_x0 = x0; det_y0 = y0; det_x1 = x1; det_y1 = y1;

    // 半径（像素）← 等效圆面积
    double radius_px = sqrt((double)n / M_PI);
    if (radius_px < 1.0) radius_px = 1.0;

    // 距离估计：dist = (REAL_BALL_R * IMAGE_W) / (radius_px * 2 * tan(fov/2))
    double fov_half = CAM_FOV * 0.5;
    det_dist = (REAL_BALL_R * (double)w) / (radius_px * 2.0 * tan(fov_half));

    // 方位角：bearing = atan( (cx - w/2) / (w/2) / tan(fov/2) )，正 = 右侧
    det_bearing = atan((det_cx - 0.5 * (double)w) / (0.5 * (double)w) / tan(fov_half));
}

// 把当前相机帧（BGRA）转成 RGB 复制进标注缓冲
static void copy_frame(unsigned char* dst, const unsigned char* src, int w, int h) {
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            unsigned char* d = dst + 3 * (y * w + x);
            d[0] = wb_camera_image_get_red(src, w, x, y);
            d[1] = wb_camera_image_get_green(src, w, x, y);
            d[2] = wb_camera_image_get_blue(src, w, x, y);
        }
    }
}

// 在帧上画叠加层：中心绿十字、球外接框红框、质心黄十字、左上角图例色条
static void draw_overlay(unsigned char* img, int w, int h) {
    // 图像中心十字（绿色）
    int cx0 = w / 2, cy0 = h / 2;
    draw_cross(img, w, h, cx0, cy0, 20, 0, 255, 0);

    if (det_found) {
        // 球外接框（红色）
        draw_rect(img, w, h, det_x0, det_y0, det_x1, det_y1, 255, 0, 0);
        // 质心十字（黄色）
        draw_cross(img, w, h, (int)(det_cx + 0.5), (int)(det_cy + 0.5), 12, 255, 255, 0);
    }

    // 左上角图例色条（1 像素行）：红=检测到球，灰=未检测到
    {
        unsigned char lr = det_found ? 255 : 128;
        unsigned char lg = det_found ? 0   : 128;
        unsigned char lb = det_found ? 0   : 128;
        for (int x = 0; x < 40; x++)
            set_pixel(img, w, h, x, 0, lr, lg, lb);
    }
}

// 保存一帧标注图：frame_%05d.ppm + 软链 frame_latest.ppm
static void save_annotated_frame(const unsigned char* im, int w, int h) {
    copy_frame(frame_buf, im, w, h);
    draw_overlay(frame_buf, w, h);

    char path[256];
    snprintf(path, sizeof(path), OUT_DIR "/frame_%05ld.ppm", frame_index);
    if (save_ppm(path, frame_buf, w, h) != 0)
        return;

    // 更新 latest 软链，便于外部工具固定读取最新帧
    char cmd[320];
    snprintf(cmd, sizeof(cmd), "ln -sfn %s " OUT_DIR "/frame_latest.ppm", path);
    if (system(cmd) != 0)
        printf("WARNING: 更新 frame_latest.ppm 软链失败\n");

    frame_index++;
}

// 打印检测结果（每 500ms 一次）
static void report_detection() {
    if (det_found) {
        printf("球检测: cx=%.1f cy=%.1f 像素数=%ld 距离≈%.2fm 方位≈%.1f°\n",
               det_cx, det_cy, det_n, det_dist, det_bearing * 180.0 / M_PI);
    } else {
        printf("球检测: 未找到\n");
    }
}

// ============================================================
// 主循环
// ============================================================
int main() {
    wb_robot_init();

    printf("=== Ball Detector Controller ===\n");
    printf("功能：PD 站立/trot + 橙色球检测（纯 C 像素循环）\n");
    printf("控制：\n");
    printf("  'S' : 站立\n");
    printf("  'T' : trot 慢跑（默认）\n");
    printf("  'R' : 复位到站立\n");
    printf("输出：每 500ms 打印球检测，并写 PPM 到 %s/\n", OUT_DIR);
    printf("================================\n");

    initialize_devices();

    // 创建输出目录
    if (system("mkdir -p " OUT_DIR) != 0)
        printf("WARNING: 创建 %s 失败\n", OUT_DIR);

    // 等待传感器稳定
    wb_robot_step(100);

    control_mode = 1; // 默认 trot：相机视野随前进变化，便于观察球的表观运动

    while (wb_robot_step(4) != -1) { // 4ms 时间步
        sim_time += DT;
        step_count++;

        // 读全部传感器
        read_sensors();

        // 键盘
        int key = wb_keyboard_get_key();
        switch (key) {
            case 'S':
                control_mode = 0;
                printf("模式: STANDING\n");
                break;
            case 'T':
                control_mode = 1;
                sim_time = 0.0; // 复位步态相位
                printf("模式: TROT\n");
                break;
            case 'R':
                control_mode = 0;
                sim_time = 0.0;
                printf("模式: RESET to STANDING\n");
                break;
            default:
                break;
        }

        // 执行行动力
        switch (control_mode) {
            case 0:
                control_standing();
                break;
            case 1:
                control_trot();
                break;
            default:
                control_standing();
                break;
        }

        // ---- 视觉：每 5 步检测一次（step 已在 while 条件里完成，图像已就绪）----
        if (cam != 0 && (step_count % DETECT_PERIOD) == 0) {
            const unsigned char* im = wb_camera_get_image(cam);
            if (im) {
                int w = wb_camera_get_width(cam);
                int h = wb_camera_get_height(cam);
                if (w > 0 && h > 0 && w <= MAX_IMG_W && h <= MAX_IMG_H)
                    detect_ball(im, w, h);
            }
        }

        // ---- 每 500ms：打印 + 存标注帧 ----
        if ((step_count % REPORT_PERIOD) == 0) {
            report_detection();
            if (cam != 0) {
                const unsigned char* im = wb_camera_get_image(cam);
                if (im) {
                    int w = wb_camera_get_width(cam);
                    int h = wb_camera_get_height(cam);
                    if (w > 0 && h > 0 && w <= MAX_IMG_W && h <= MAX_IMG_H)
                        save_annotated_frame(im, w, h);
                }
            }
        }
    }

    wb_robot_cleanup();
    return 0;
}
