// mini_cheetah_controller.cpp
// Webots controller for Mini Cheetah quadruped robot
// Features: PD standing controller, simple trot gait, IMU reading
//
// Reference: textbook-ch9-13.md (MPC/WBC framework), robot-software-analysis.md (joint PD gains)

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

// ============================================================
// Constants
// ============================================================
#define NUM_LEGS        4
#define NUM_JOINTS      3   // per leg: abd, hip, knee
#define TOTAL_JOINTS    (NUM_LEGS * NUM_JOINTS)

// Control frequency: Webots basicTimeStep = 4ms -> ~250Hz
static const double DT = 0.004;

// Leg indices
enum LegID { FR = 0, FL = 1, HR = 2, HL = 3 };

// Joint indices within each leg
enum JointID { ABD = 0, HIP = 1, KN = 2 };

// ============================================================
// PD Gains (from textbook / MIT controller parameters)
// Joint PD: kp_joint = [3, 3, 3], kd_joint = [1, 0.2, 0.2]
// Standing pose gains
// ============================================================
static const double KP[3] = {3.0, 3.0, 3.0};
static const double KD[3] = {1.0, 0.2, 0.2};
static const double MAX_TORQUE = 15.0;

// Standing joint angles [abd, hip, knee] in radians
// At angle 0, legs point straight down (in our Webots model)
static const double STANDING_DES[3] = {0.0, 0.0, 0.0};

// Trot gait parameters
static const double TROT_PERIOD = 0.5;       // seconds per cycle
static const double TROT_STANCE_TIME = 0.25; // stance phase duration
static const double TROT_SWING_HEIGHT = 0.04; // swing height in meters
// Phase offsets for trot: FR+HL in phase, FL+HR in phase (diagonal)
static const double TROT_PHASE[NUM_LEGS] = {0.0, 0.5, 0.5, 0.0};

// Leg forward kinematics (simplified)
// abd_link_length = 0.062, hip_link_length = 0.209, knee_link = 0.18
static const double L_ABD = 0.062;
static const double L_HIP = 0.209;
static const double L_KNEE = 0.18;

// Hip positions relative to body center [FR, FL, HR, HL]
static const double HIP_POS[NUM_LEGS][3] = {
    { 0.19, -0.111, 0.0},  // FR: x=0.19, y=-(0.049+0.062)
    { 0.19,  0.111, 0.0},  // FL
    {-0.19, -0.111, 0.0},  // HR
    {-0.19,  0.111, 0.0},  // HL
};

// ============================================================
// Device handles
// ============================================================
static WbDeviceTag motors[NUM_LEGS][NUM_JOINTS];
static WbDeviceTag sensors[NUM_LEGS][NUM_JOINTS];
static WbDeviceTag gyro, accelerometer, inertial_unit;

// Sensor data
static double joint_pos[NUM_LEGS][NUM_JOINTS];
static double joint_vel[NUM_LEGS][NUM_JOINTS];
static double prev_joint_pos[NUM_LEGS][NUM_JOINTS];
static double imu_orientation[4]; // quaternion (w, x, y, z)
static double imu_angular_vel[3]; // rad/s
static double imu_accel[3];       // m/s^2

// Controller state
static double sim_time = 0.0;
static int control_mode = 0; // 0=standing, 1=trot

// ============================================================
// Motor name templates
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
// Helper functions
// ============================================================

static void initialize_devices() {
    // Initialize motors and sensors
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int j = 0; j < NUM_JOINTS; j++) {
            motors[leg][j] = wb_robot_get_device(MOTOR_NAMES[leg][j]);
            sensors[leg][j] = wb_robot_get_device(SENSOR_NAMES[leg][j]);
            if (motors[leg][j] == 0)
                printf("WARNING: Motor not found: %s\n", MOTOR_NAMES[leg][j]);
            if (sensors[leg][j] == 0)
                printf("WARNING: Sensor not found: %s\n", SENSOR_NAMES[leg][j]);
            wb_position_sensor_enable(sensors[leg][j], 1); // 1ms sampling period
        }
    }

    // Initialize IMU sensors
    gyro = wb_robot_get_device("gyro");
    accelerometer = wb_robot_get_device("accelerometer");
    inertial_unit = wb_robot_get_device("inertial unit");
    wb_gyro_enable(gyro, 1);
    wb_accelerometer_enable(accelerometer, 1);
    wb_inertial_unit_enable(inertial_unit, 1);

    // Enable keyboard
    wb_keyboard_enable(100); // 100ms sampling

    // Initialize previous positions
    memset(prev_joint_pos, 0, sizeof(prev_joint_pos));
    memset(joint_pos, 0, sizeof(joint_pos));
    memset(joint_vel, 0, sizeof(joint_vel));
}

static void read_sensors() {
    // Read joint positions
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

    // Read IMU
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

// Convert quaternion to roll, pitch, yaw (NUE frame)
static void quat_to_rpy(const double q[4], double* roll, double* pitch, double* yaw) {
    double w = q[0], x = q[1], y = q[2], z = q[3];
    *roll  = atan2(2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y));
    *pitch = asin(2.0*(w*y - z*x));
    *yaw   = atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z));
}

// PD control for a single joint
static double joint_pd_control(double q_des, double q, double qd_des, double qd,
                                int joint_idx) {
    double error = q_des - q;
    double vel_error = qd_des - qd;
    double torque = KP[joint_idx] * error + KD[joint_idx] * vel_error;

    // Clamp torque
    if (torque > MAX_TORQUE) torque = MAX_TORQUE;
    if (torque < -MAX_TORQUE) torque = -MAX_TORQUE;
    return torque;
}

// ============================================================
// Control modes
// ============================================================

// Standing controller - maintain neutral pose
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

// Trot gait controller
// Diagonal legs move together: FR+HL, FL+HR
// Phase 0.0 = start of stance, 0.5 = transition to swing
static void control_trot() {
    double phase = fmod(sim_time / TROT_PERIOD, 1.0);

    for (int leg = 0; leg < NUM_LEGS; leg++) {
        // Leg-specific phase
        double leg_phase = fmod(phase + TROT_PHASE[leg], 1.0);

        double q_des[NUM_JOINTS] = {0.0, 0.0, 0.0};
        double qd_des[NUM_JOINTS] = {0.0, 0.0, 0.0};

        if (leg_phase < 0.5) {
            // Stance phase: leg on ground, push backward
            // Hip joint moves forward (positive) during stance
            double stance_progress = leg_phase / 0.5; // 0 -> 1
            double hip_angle = 0.15 * (1.0 - 2.0 * stance_progress); // sweep +/-0.15 rad
            q_des[HIP] = hip_angle;
            qd_des[HIP] = -0.15 * 2.0 / 0.5; // velocity of sweep
            q_des[KN] = 0.0; // knee straight
        } else {
            // Swing phase: lift and return
            double swing_progress = (leg_phase - 0.5) / 0.5; // 0 -> 1
            // Bell-shaped swing trajectory
            double swing_hip = 0.15 * (1.0 - 2.0 * swing_progress);
            q_des[HIP] = swing_hip;
            qd_des[HIP] = 0.0;

            // Bell-shaped knee lift
            double lift = TROT_SWING_HEIGHT;
            double swing_knee = -lift / L_KNEE * sin(swing_progress * M_PI);
            q_des[KN] = swing_knee;
        }

        // Abduction: keep at 0 (no lateral motion for simple trot)
        q_des[ABD] = 0.0;

        // Apply PD control
        for (int j = 0; j < NUM_JOINTS; j++) {
            double torque = joint_pd_control(
                q_des[j], joint_pos[leg][j],
                qd_des[j], joint_vel[leg][j], j);
            wb_motor_set_torque(motors[leg][j], torque);
        }
    }
}

// ============================================================
// Main
// ============================================================
int main() {
    wb_robot_init();

    printf("=== Mini Cheetah Controller ===\n");
    printf("Control modes:\n");
    printf("  'S' : Standing (default)\n");
    printf("  'T' : Trot gait\n");
    printf("  'R' : Reset to standing\n");
    printf("===============================\n");

    initialize_devices();

    // Wait for sensors to stabilize
    wb_robot_step(100);

    control_mode = 0; // Start in standing mode

    while (wb_robot_step(4) != -1) { // 4ms time step
        sim_time += DT;

        // Read all sensors
        read_sensors();

        // Check keyboard input
        int key = wb_keyboard_get_key();
        switch (key) {
            case 'S':
                control_mode = 0;
                printf("Mode: STANDING\n");
                break;
            case 'T':
                control_mode = 1;
                sim_time = 0.0; // Reset gait phase
                printf("Mode: TROT\n");
                break;
            case 'R':
                control_mode = 0;
                sim_time = 0.0;
                printf("Mode: RESET to STANDING\n");
                break;
            default:
                break;
        }

        // Execute control
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

        // Print debug info every 500ms (125 steps at 4ms)
        if (((int)(sim_time * 1000)) % 500 < 4) {
            double roll, pitch, yaw;
            quat_to_rpy(imu_orientation, &roll, &pitch, &yaw);
            printf("[t=%.2f] IMU: roll=%.2f pitch=%.2f yaw=%.2f | "
                   "FR_hip=%.3f FR_kn=%.3f\n",
                   sim_time, roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI,
                   joint_pos[FR][HIP], joint_pos[FR][KN]);
        }
    }

    wb_robot_cleanup();
    return 0;
}
