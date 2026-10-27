/* SPDX-License-Identifier: MIT */
/*
 * evdev_touch.c - 读取 /dev/input/eventX 触摸屏并注册为 LVGL 指针输入
 *
 * 预编译的 liblvgl.so 未编入任何输入驱动 (LV_USE_EVDEV/LV_USE_LINUX_LIBINPUT 均为 0),
 * 因此在应用侧实现一个轻量 evdev 触摸驱动:
 *   - 自动扫描 /dev/input/event*, 按设备能力 (EV_ABS + ABS_MT_POSITION_X 或
 *     ABS_X+BTN_TOUCH) 识别触摸屏, 无需配置路径
 *   - 同时支持协议 A (ABS_X/ABS_Y+BTN_TOUCH) 和协议 B (ABS_MT_SLOT/
 *     ABS_MT_TRACKING_ID/ABS_MT_POSITION_*)
 *   - 触摸原始坐标按设备分辨率缩放到屏幕坐标
 */

#include "evdev_touch.h"
#include "common.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/input.h>
#include <sys/ioctl.h>

#include "lvgl.h"

#define EVDEV_MAX_SLOTS 8

#define BIT_IS_SET(bits, bit) \
    ((bits)[(bit) / (8 * sizeof(unsigned long))] & (1UL << ((bit) % (8 * sizeof(unsigned long)))))

typedef struct {
    int fd;
    int screen_w, screen_h;
    int x_max, y_max;          /* 触摸屏原始分辨率 (最大值) */
    int use_mt;                /* 1 = 协议 B 多点触摸 */
    int cur_slot;
    int slot_tracking[EVDEV_MAX_SLOTS];
    int slot_x[EVDEV_MAX_SLOTS];
    int slot_y[EVDEV_MAX_SLOTS];
    int x, y;                  /* 上报给 LVGL 的坐标 (屏幕坐标系) */
    int pressed;
} csg_evdev_touch_ctx_t;

static csg_evdev_touch_ctx_t s_csg_evdev_ctx = { .fd = -1 };

/* 把触摸原始坐标换算到屏幕坐标系 */
static void csg_evdev_touch_update_point(void)
{
    int sx = s_csg_evdev_ctx.slot_x[0];
    int sy = s_csg_evdev_ctx.slot_y[0];

    if (s_csg_evdev_ctx.x_max > 0) sx = (int)((int64_t)sx * s_csg_evdev_ctx.screen_w / (s_csg_evdev_ctx.x_max + 1));
    if (s_csg_evdev_ctx.y_max > 0) sy = (int)((int64_t)sy * s_csg_evdev_ctx.screen_h / (s_csg_evdev_ctx.y_max + 1));

    if (sx < 0) sx = 0;
    if (sy < 0) sy = 0;
    if (sx >= s_csg_evdev_ctx.screen_w) sx = s_csg_evdev_ctx.screen_w - 1;
    if (sy >= s_csg_evdev_ctx.screen_h) sy = s_csg_evdev_ctx.screen_h - 1;

    s_csg_evdev_ctx.x = sx;
    s_csg_evdev_ctx.y = sy;
}

/* 非阻塞读取并处理所有待处理输入事件 (由 LVGL 线程调用) */
static void csg_evdev_touch_drain(void)
{
    struct input_event ev;

    while (read(s_csg_evdev_ctx.fd, &ev, sizeof(ev)) == (ssize_t)sizeof(ev)) {

        if (ev.type == EV_ABS) {
            if (ev.code == ABS_MT_SLOT) {
                if (ev.value >= 0 && ev.value < EVDEV_MAX_SLOTS)
                    s_csg_evdev_ctx.cur_slot = ev.value;
            }
            else if (ev.code == ABS_MT_TRACKING_ID) {
                s_csg_evdev_ctx.slot_tracking[s_csg_evdev_ctx.cur_slot] = ev.value;
            }
            else if (ev.code == ABS_MT_POSITION_X) {
                s_csg_evdev_ctx.slot_x[s_csg_evdev_ctx.cur_slot] = ev.value;
            }
            else if (ev.code == ABS_MT_POSITION_Y) {
                s_csg_evdev_ctx.slot_y[s_csg_evdev_ctx.cur_slot] = ev.value;
            }
            else if (ev.code == ABS_X) {
                s_csg_evdev_ctx.slot_x[0] = ev.value;   /* 协议 A 单点 */
            }
            else if (ev.code == ABS_Y) {
                s_csg_evdev_ctx.slot_y[0] = ev.value;
            }
        }
        else if (ev.type == EV_KEY && ev.code == BTN_TOUCH) {
            s_csg_evdev_ctx.pressed = (ev.value == 1);
        }
        else if (ev.type == EV_SYN) {
            /* 协议 B: 用触点跟踪 ID 判断按压状态 */
            if (s_csg_evdev_ctx.use_mt) {
                s_csg_evdev_ctx.pressed = 0;
                for (int i = 0; i < EVDEV_MAX_SLOTS; i++) {
                    if (s_csg_evdev_ctx.slot_tracking[i] >= 0) {
                        s_csg_evdev_ctx.pressed = 1;
                        if (i != 0) {   /* 把活动触点挪到 0 号槽 (UI 只需单点) */
                            s_csg_evdev_ctx.slot_x[0] = s_csg_evdev_ctx.slot_x[i];
                            s_csg_evdev_ctx.slot_y[0] = s_csg_evdev_ctx.slot_y[i];
                        }
                        break;
                    }
                }
            }
            csg_evdev_touch_update_point();
        }
    }
}

/* 判断设备是否为触摸屏 (EV_ABS + 多点绝对坐标, 或 单点绝对坐标+BTN_TOUCH) */
static int csg_evdev_touch_is_touchscreen(int fd)
{
    unsigned long evbits[EV_MAX / (8 * sizeof(unsigned long)) + 1] = {0};
    unsigned long absbits[ABS_MAX / (8 * sizeof(unsigned long)) + 1] = {0};
    unsigned long keybits[KEY_MAX / (8 * sizeof(unsigned long)) + 1] = {0};

    if (ioctl(fd, EVIOCGBIT(0, sizeof(evbits)), evbits) < 0) return 0;
    if (!BIT_IS_SET(evbits, EV_ABS)) return 0;
    if (ioctl(fd, EVIOCGBIT(EV_ABS, sizeof(absbits)), absbits) < 0) return 0;
    if (ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(keybits)), keybits) < 0) return 0;

    if (BIT_IS_SET(absbits, ABS_MT_POSITION_X)) return 1;
    if (BIT_IS_SET(absbits, ABS_X) && BIT_IS_SET(keybits, BTN_TOUCH)) return 1;
    return 0;
}

/* LVGL 输入读取回调 */
static void csg_evdev_touch_read_cb(lv_indev_t * indev, lv_indev_data_t * data)
{
    (void)indev;
    if (s_csg_evdev_ctx.fd >= 0) csg_evdev_touch_drain();

    data->point.x = s_csg_evdev_ctx.x;
    data->point.y = s_csg_evdev_ctx.y;
    data->state = s_csg_evdev_ctx.pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

int csg_evdev_touch_init(int screen_w, int screen_h)
{
    char path[32];
    int fd = -1;

    if (s_csg_evdev_ctx.fd >= 0) return 0;

    /* 自动扫描并选择第一个触摸屏设备 */
    for (int i = 0; i < 32; i++) {
        snprintf(path, sizeof(path), "/dev/input/event%d", i);
        fd = open(path, O_RDONLY | O_NONBLOCK);
        if (fd < 0) continue;

        if (csg_evdev_touch_is_touchscreen(fd)) {
            char name[256] = "";
            ioctl(fd, EVIOCGNAME(sizeof(name)), name);
            CSG_LOGI("EVDEV", "Touch device: %s (%s)", path, name);
            break;
        }
        close(fd);
        fd = -1;
    }

    if (fd < 0) {
        CSG_LOGW("EVDEV", "No touchscreen found, UI buttons will not respond (use CLI args to start preview)");
        return -1;
    }

    s_csg_evdev_ctx.fd = fd;
    s_csg_evdev_ctx.screen_w = screen_w;
    s_csg_evdev_ctx.screen_h = screen_h;
    for (int i = 0; i < EVDEV_MAX_SLOTS; i++) s_csg_evdev_ctx.slot_tracking[i] = -1;

    /* 读取触摸分辨率 */
    struct input_absinfo abs;
    if (ioctl(fd, EVIOCGABS(ABS_MT_POSITION_X), &abs) == 0 && abs.maximum > 0) {
        s_csg_evdev_ctx.x_max = abs.maximum;
        s_csg_evdev_ctx.use_mt = 1;
    }
    else if (ioctl(fd, EVIOCGABS(ABS_X), &abs) == 0 && abs.maximum > 0) {
        s_csg_evdev_ctx.x_max = abs.maximum;
    }
    if (ioctl(fd, EVIOCGABS(ABS_MT_POSITION_Y), &abs) == 0 && abs.maximum > 0) {
        s_csg_evdev_ctx.y_max = abs.maximum;
    }
    else if (ioctl(fd, EVIOCGABS(ABS_Y), &abs) == 0 && abs.maximum > 0) {
        s_csg_evdev_ctx.y_max = abs.maximum;
    }

    CSG_LOGI("EVDEV", "Touch range %dx%d -> screen %dx%d",
         s_csg_evdev_ctx.x_max + 1, s_csg_evdev_ctx.y_max + 1, screen_w, screen_h);

    /* 注册 LVGL 指针输入设备 */
    lv_indev_t * indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, csg_evdev_touch_read_cb);
    lv_indev_set_user_data(indev, &s_csg_evdev_ctx);

    return 0;
}

void csg_evdev_touch_deinit(void)
{
    if (s_csg_evdev_ctx.fd >= 0) {
        close(s_csg_evdev_ctx.fd);
        s_csg_evdev_ctx.fd = -1;
    }
}

/* ==================== 鼠标支持 (EV_REL + BTN_LEFT) ====================
 * 相对坐标鼠标: 累加 REL_X/REL_Y 得到光标位置, BTN_LEFT 表示按下.
 * 以独立的 LVGL 指针输入设备注册, 与触屏驱动共存.              */
typedef struct {
    int fd;
    int screen_w, screen_h;
    int cursor_x, cursor_y;
    int pressed;
} csg_evdev_mouse_ctx_t;

static csg_evdev_mouse_ctx_t s_csg_evdev_mouse = { .fd = -1 };

/* 判断设备是否为鼠标: EV_REL + REL_X + REL_Y + BTN_LEFT */
static int csg_evdev_mouse_is_mouse(int fd)
{
    unsigned long evbits[EV_MAX / (8 * sizeof(unsigned long)) + 1] = {0};
    unsigned long relbits[REL_MAX / (8 * sizeof(unsigned long)) + 1] = {0};
    unsigned long keybits[KEY_MAX / (8 * sizeof(unsigned long)) + 1] = {0};

    if (ioctl(fd, EVIOCGBIT(0, sizeof(evbits)), evbits) < 0) return 0;
    if (!BIT_IS_SET(evbits, EV_REL)) return 0;
    if (ioctl(fd, EVIOCGBIT(EV_REL, sizeof(relbits)), relbits) < 0) return 0;
    if (ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(keybits)), keybits) < 0) return 0;
    if (!BIT_IS_SET(relbits, REL_X) || !BIT_IS_SET(relbits, REL_Y)) return 0;
    if (!BIT_IS_SET(keybits, BTN_LEFT)) return 0;
    return 1;
}

/* 非阻塞读取并累加相对位移, 光标限制在屏幕内 */
static void csg_evdev_mouse_drain(void)
{
    struct input_event ev;
    while (read(s_csg_evdev_mouse.fd, &ev, sizeof(ev)) == (ssize_t)sizeof(ev)) {
        if (ev.type == EV_REL) {
            if (ev.code == REL_X)       s_csg_evdev_mouse.cursor_x += ev.value;
            else if (ev.code == REL_Y)  s_csg_evdev_mouse.cursor_y += ev.value;
        }
        else if (ev.type == EV_KEY && ev.code == BTN_LEFT) {
            s_csg_evdev_mouse.pressed = (ev.value == 1);
        }
    }
    if (s_csg_evdev_mouse.cursor_x < 0) s_csg_evdev_mouse.cursor_x = 0;
    if (s_csg_evdev_mouse.cursor_y < 0) s_csg_evdev_mouse.cursor_y = 0;
    if (s_csg_evdev_mouse.cursor_x >= s_csg_evdev_mouse.screen_w)
        s_csg_evdev_mouse.cursor_x = s_csg_evdev_mouse.screen_w - 1;
    if (s_csg_evdev_mouse.cursor_y >= s_csg_evdev_mouse.screen_h)
        s_csg_evdev_mouse.cursor_y = s_csg_evdev_mouse.screen_h - 1;
}

/* LVGL 鼠标读取回调 */
static void csg_evdev_mouse_read_cb(lv_indev_t * indev, lv_indev_data_t * data)
{
    (void)indev;
    if (s_csg_evdev_mouse.fd >= 0) csg_evdev_mouse_drain();
    data->point.x = s_csg_evdev_mouse.cursor_x;
    data->point.y = s_csg_evdev_mouse.cursor_y;
    data->state = s_csg_evdev_mouse.pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

int csg_evdev_mouse_init(int screen_w, int screen_h)
{
    char path[32];
    int fd = -1;

    if (s_csg_evdev_mouse.fd >= 0) return 0;

    /* 自动扫描并选择第一个鼠标设备 */
    for (int i = 0; i < 32; i++) {
        snprintf(path, sizeof(path), "/dev/input/event%d", i);
        fd = open(path, O_RDONLY | O_NONBLOCK);
        if (fd < 0) continue;
        if (csg_evdev_mouse_is_mouse(fd)) {
            char name[256] = "";
            ioctl(fd, EVIOCGNAME(sizeof(name)), name);
            CSG_LOGI("EVDEV", "Mouse device: %s (%s)", path, name);
            break;
        }
        close(fd);
        fd = -1;
    }

    if (fd < 0) {
        CSG_LOGW("EVDEV", "No mouse found, mouse input disabled");
        return -1;
    }

    s_csg_evdev_mouse.fd = fd;
    s_csg_evdev_mouse.screen_w = screen_w;
    s_csg_evdev_mouse.screen_h = screen_h;
    /* 光标初始位于屏幕中心 */
    s_csg_evdev_mouse.cursor_x = screen_w / 2;
    s_csg_evdev_mouse.cursor_y = screen_h / 2;

    /* 注册 LVGL 指针输入设备 */
    lv_indev_t * indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, csg_evdev_mouse_read_cb);
    lv_indev_set_user_data(indev, &s_csg_evdev_mouse);

    /* 显示一个可见光标: 用小方块 obj (不依赖字体符号, 保证可见) */
    lv_obj_t * cursor = lv_obj_create(lv_screen_active());
    lv_obj_set_size(cursor, 12, 12);
    lv_obj_set_style_bg_color(cursor, lv_color_white(), LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(cursor, LV_OPA_COVER, LV_STATE_DEFAULT);
    lv_indev_set_cursor(indev, cursor);

    return 0;
}

void csg_evdev_mouse_deinit(void)
{
    if (s_csg_evdev_mouse.fd >= 0) {
        close(s_csg_evdev_mouse.fd);
        s_csg_evdev_mouse.fd = -1;
    }
}