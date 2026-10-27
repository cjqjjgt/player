/* SPDX-License-Identifier: MIT */
/*
 * lvgl_display.h - LVGL 视频显示模块
 *
 * 基于 LVGL 实现视频帧显示到 framebuffer
 * 参考: /home/dell/lvgl-master
 */

#ifndef __LVGL_DISPLAY_H__
#define __LVGL_DISPLAY_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化 LVGL 显示 */
int csg_lvgl_display_init(int screen_w, int screen_h);

/* 更新视频帧显示 */
void csg_lvgl_display_update_video(void *rgb_data, int width, int height);

/* ==================== 多路分屏显示 (NVR 分屏墙) ====================
 * ★ 多路 RTSP 预览/录制场景使用: 每个通道一个独立 LVGL 图像窗口,
 *   每通道双缓冲, 解码线程调 update_video_chn 只做 memcpy(加锁),
 *   真正上屏在 LVGL tick 线程统一 present. 与单路 API 完全隔离,
 *   单路播放器 (chn=0) 行为不受影响. */

/* 最大分屏通道数 (对应 1/2/4/6/9/16/18/32 布局) */
#define CSG_LVGL_MAX_VIDEO_CHN 32

/* 设置分屏网格: chn_cnt=1 全屏(单路, 保持原行为); chn_cnt>1 按 NVR 布局
 * (1/2/4/6/9/16/18/32) 自动选网格, 每个窗口铺满各自矩形.
 * 返回实际通道数 (<=chn_cnt, 超最大支持按最大支持回退). */
int csg_lvgl_display_set_grid(int chn_cnt);

/* 按通道更新视频帧: chn∈[0, grid_chn_cnt). 与单路 API 语义一致,
 * 只拷贝数据到该通道双缓冲, 不直接操作 LVGL 对象. */
void csg_lvgl_display_update_video_chn(int chn, void *rgb_data,
                                       int width, int height);

/* 获取某通道分屏窗口的像素尺寸 (RGA 缩放目标尺寸, 不含间距).
 * 单路时返回屏幕内保持宽高比的显示尺寸(与 update_video 逻辑一致). */
void csg_lvgl_display_get_chn_size(int chn, int *w, int *h);

/* 清空全部通道画面 (停止多路时调用, 避免残留最后一帧) */
void csg_lvgl_display_reset_channels(void);

/* 当前分屏通道数 */
int csg_lvgl_display_get_grid_chn_cnt(void);

/* 暂停/恢复视频显示: paused=1 时丢弃在途帧, 画面保持暂停前最后一帧 */
void csg_lvgl_display_set_paused(int paused);

/* 逐帧步进: 暂停状态下放行下一帧上屏 */
void csg_lvgl_display_step_frame(void);

/* 重置帧计数器 (用于新播放开始时) */
void csg_lvgl_display_reset_counter(void);

/* 获取视频显示区域 */
void csg_lvgl_display_get_video_area(int *x, int *y, int *w, int *h);

/* 获取屏幕分辨率 (供上层计算视频显示缩放目标尺寸) */
void csg_lvgl_display_get_screen_size(int *w, int *h);

/* LVGL 主循环 tick */
void csg_lvgl_display_tick(void);

/* 释放 LVGL 显示 */
void csg_lvgl_display_deinit(void);

/* 获取 LVGL 是否已初始化 */
int csg_lvgl_display_is_ready(void);

/* ==================== 全局 LVGL 互斥锁 ====================
 * LVGL 非线程安全 (LV_USE_OS=LV_OS_NONE), 所有跨线程的 LVGL API 调用
 * 必须持有此锁. tick 线程在 lv_timer_handler() 前后加锁, 其他线程在
 * 调用 LVGL API 前加锁. 锁为递归锁, 同一线程可重入 (LVGL 定时器/事件
 * 回调内再调用公开 API 不死锁). */
void csg_lvgl_lock(void);
void csg_lvgl_unlock(void);

#ifdef __cplusplus
}
#endif

#endif /* __LVGL_DISPLAY_H__ */