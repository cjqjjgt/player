/* SPDX-License-Identifier: MIT */
/*
 * evdev_touch.h - evdev 触摸屏输入
 */

#ifndef __EVDEV_TOUCH_H__
#define __EVDEV_TOUCH_H__

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化: 自动扫描 /dev/input/event*, 注册 LVGL 指针输入设备
 * screen_w/screen_h: 屏幕分辨率, 用于坐标缩放 */
int csg_evdev_touch_init(int screen_w, int screen_h);

/* 释放输入设备 */
void csg_evdev_touch_deinit(void);

/* 绑定式鼠标支持: 扫描 EV_REL+BTN_LEFT 的鼠标设备, 注册为独立
 * LVGL 指针输入 (与触屏共存). 无鼠标时返回 -1, 不影响触屏. */
int csg_evdev_mouse_init(int screen_w, int screen_h);

/* 释放鼠标输入设备 */
void csg_evdev_mouse_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* __EVDEV_TOUCH_H__ */