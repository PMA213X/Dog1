/**
 * joystick.h — 手柄/遥控器输入模块（Linux Joystick API，纯 POSIX）
 *
 * 功能概述：
 *   非阻塞读取 /dev/input/js0，将常见 Xbox / AT9S USB 手柄的摇杆与按键
 *   归一化为 JoystickState。不依赖 Webots API 或任何第三方库。
 *
 * 特性：
 *   - 非阻塞读取，适合每控制步调用一次 joystick_update()
 *   - 设备拔掉后自动重连（每 5 秒尝试一次 open）
 *   - 摇杆死区 |v| < 0.12 归零
 *   - 轴/按钮映射做成常量，可在 joystick.c 顶部按实际手柄调整
 *
 * 轴映射（可在 joystick.c 中修改）：
 *   0 = 左摇杆 X, 1 = 左摇杆 Y, 2 = 右摇杆 X, 3 = 右摇杆 Y
 *   （常见 Xbox / AT9S USB 映射）
 * 按钮映射（可在 joystick.c 中修改）：
 *   0 = A(跳), 1 = B(复位), 2 = X, 3 = Y, 4 = LB, 5 = RB
 */

// 最小用法示例：
// ------------------------------------------------------------
//   JoystickState js;
//   if (joystick_init() == 0) {
//       joystick_update(&js);
//       float vx = -js.ly * 1.0f;  // 左摇杆前后
//       float wz = js.rx * 2.0f;   // 右摇杆转向
//       if (js.a) { /* 跳 */ }
//   }
// ------------------------------------------------------------

#ifndef JOYSTICK_H
#define JOYSTICK_H

#ifdef __cplusplus
extern "C" {
#endif

/** 手柄状态：摇杆值 [-1,1]，按钮按下=1 */
typedef struct {
    float lx, ly;   /* 左摇杆 x,y  [-1,1] */
    float rx, ry;   /* 右摇杆 x,y  [-1,1] */
    int a, b, x, y; /* 按钮：A(跳) B(复位) X Y，按下=1 */
    int lb, rb;     /* 肩键 LB/RB，按下=1 */
    int connected;  /* 0=未连接 / 1=已连接 */
} JoystickState;

/**
 * 打开手柄设备并做非阻塞初始化。
 * 返回 0=成功；-1=当前无设备（不报错，仅提示，可继续调用 update
 * 等待自动重连）。
 */
int joystick_init(void);

/**
 * 非阻塞刷新状态。每控制步调用一次。
 * 传入 st 不能为 NULL。设备断开时 st->connected=0，摇杆值归零、
 * 按钮保持最后状态或归零（实现里统一归零）。
 * 返回 0=成功（含未连接场景）；-1=参数错误。
 */
int joystick_update(JoystickState* st);

/** 关闭设备，释放资源。可重复调用。 */
void joystick_close(void);

#ifdef __cplusplus
}
#endif

#endif /* JOYSTICK_H */
