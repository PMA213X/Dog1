/**
 * joystick.c — 手柄/遥控器输入模块实现
 *
 * 基于标准 Linux Joystick API（<linux/joystick.h>），非阻塞读取
 * /dev/input/js*。无第三方依赖，不使用 Webots API。
 *
 * 可调参数集中在本文件顶部的宏定义（轴索引 / 按钮索引 / 死区 /
 * 设备路径 / 重连间隔）。
 */

#include "joystick.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* ============================ 可调参数 ============================ */

/* 手柄设备路径；找不到 js0 时 update 会按 js0/js1/... 轮询尝试 */
#define JS_DEVICE_PATH      "/dev/input/js0"

/* 轴索引映射（常见 Xbox / AT9S USB 映射），按实际手柄改这里 */
#define JS_AXIS_LX          0   /* 左摇杆 X */
#define JS_AXIS_LY          1   /* 左摇杆 Y */
#define JS_AXIS_RX          2   /* 右摇杆 X */
#define JS_AXIS_RY          3   /* 右摇杆 Y */

/* 按钮索引映射 */
#define JS_BTN_A            0   /* A（跳） */
#define JS_BTN_B            1   /* B（复位） */
#define JS_BTN_X            2   /* X */
#define JS_BTN_Y            3   /* Y */
#define JS_BTN_LB           4   /* 左肩键 */
#define JS_BTN_RB           5   /* 右肩键 */

/* 摇杆死区：|v| < 0.12 归零 */
#define JS_DEADZONE         0.12f

/* 设备断开后自动重连间隔（秒） */
#define JS_RECONNECT_SEC    5

/* ============================ 内部状态 ============================ */

static int   g_fd        = -1;   /* 设备文件描述符，-1 表示未连接 */
static float g_axis[8]   = {0};  /* 原始归一化轴值缓存（按轴索引存） */
static int   g_btn[16]   = {0};  /* 按钮状态缓存（按按钮索引存） */
static time_t g_last_try = 0;    /* 上次尝试打开设备的时间戳 */

/* ============================ 内部工具 ============================ */

/* 当前单调时钟秒数，用于重连节流 */
static time_t js_now(void)
{
    return time(NULL);
}

/* 把 16 位轴原始值归一化到 [-1,1]，并施加死区 */
static float js_norm_axis(int value)
{
    float v = (float)value / 32767.0f;

    if (v >  1.0f) v =  1.0f;
    if (v < -1.0f) v = -1.0f;
    if (v > -JS_DEADZONE && v < JS_DEADZONE) {
        v = 0.0f;
    }
    return v;
}

/* 关闭内部设备句柄（幂等） */
static void js_close_fd(void)
{
    if (g_fd >= 0) {
        close(g_fd);
        g_fd = -1;
    }
}

/* 尝试打开手柄设备；成功返回 0，失败返回 -1（仅提示，不报错退出） */
static int js_open_device(void)
{
    /* 重连节流：距上次尝试不足间隔则直接返回失败 */
    time_t now = js_now();
    if (g_last_try != 0 && (now - g_last_try) < JS_RECONNECT_SEC) {
        return -1;
    }
    g_last_try = now;

    g_fd = open(JS_DEVICE_PATH, O_RDONLY | O_NONBLOCK);
    if (g_fd < 0) {
        /* 无设备属于正常情况（例如未插手柄），只做提示 */
        printf("[joystick] 未找到 %s，等待手柄接入...\n", JS_DEVICE_PATH);
        return -1;
    }

    printf("[joystick] 已连接 %s\n", JS_DEVICE_PATH);
    return 0;
}

/* 将内部缓存写入对外状态结构 */
static void js_fill_state(JoystickState* st)
{
    if (g_fd < 0) {
        /* 未连接：摇杆/按钮全部归零，connected=0 */
        memset(st, 0, sizeof(*st));
        st->connected = 0;
        return;
    }

    st->lx = g_axis[JS_AXIS_LX];
    st->ly = g_axis[JS_AXIS_LY];
    st->rx = g_axis[JS_AXIS_RX];
    st->ry = g_axis[JS_AXIS_RY];

    st->a  = g_btn[JS_BTN_A];
    st->b  = g_btn[JS_BTN_B];
    st->x  = g_btn[JS_BTN_X];
    st->y  = g_btn[JS_BTN_Y];
    st->lb = g_btn[JS_BTN_LB];
    st->rb = g_btn[JS_BTN_RB];

    st->connected = 1;
}

/* 处理一条 js_event：更新轴或按钮缓存 */
static void js_handle_event(const struct js_event* e)
{
    unsigned int type = e->type & ~JS_EVENT_INIT; /* 去掉 INIT 标志位 */

    if (type == JS_EVENT_AXIS) {
        if (e->number < sizeof(g_axis) / sizeof(g_axis[0])) {
            g_axis[e->number] = js_norm_axis(e->value);
        }
    } else if (type == JS_EVENT_BUTTON) {
        if (e->number < sizeof(g_btn) / sizeof(g_btn[0])) {
            g_btn[e->number] = e->value ? 1 : 0;
        }
    }
}

/* 排空事件队列，把所有待处理事件应用到缓存 */
static void js_drain_events(void)
{
    struct js_event e;

    while (1) {
        ssize_t n = read(g_fd, &e, sizeof(e));

        if (n == (ssize_t)sizeof(e)) {
            js_handle_event(&e);
            continue;
        }

        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            /* 队列已空，正常 */
            return;
        }

        if (n == 0) {
            /* 读到 EOF：设备已拔掉 */
            printf("[joystick] 设备断开，将在 %d 秒后尝试重连\n",
                   JS_RECONNECT_SEC);
            js_close_fd();
            return;
        }

        if (n < 0 && errno == EINTR) {
            continue; /* 被信号打断，重试 */
        }

        /* 其他错误：视为设备不可用，关闭并等待重连 */
        printf("[joystick] 读取错误(%s)，关闭设备\n", strerror(errno));
        js_close_fd();
        return;
    }
}

/* ============================ 对外接口 ============================ */

int joystick_init(void)
{
    /* 重置内部缓存 */
    memset(g_axis, 0, sizeof(g_axis));
    memset(g_btn, 0, sizeof(g_btn));
    g_last_try = 0;

    if (js_open_device() == 0) {
        /* 先排空一次，吃掉 INIT 事件，让首帧状态即为当前真值 */
        js_drain_events();
        return 0;
    }

    /* 无设备不算错误：仅提示，后续 update 会自动重连 */
    return -1;
}

int joystick_update(JoystickState* st)
{
    if (st == NULL) {
        return -1;
    }

    /* 未连接时按节流策略尝试自动重连 */
    if (g_fd < 0) {
        if (js_open_device() == 0) {
            js_drain_events();
        }
        js_fill_state(st);
        return 0;
    }

    /* 非阻塞读取所有待处理事件 */
    js_drain_events();
    js_fill_state(st);
    return 0;
}

void joystick_close(void)
{
    js_close_fd();
}
