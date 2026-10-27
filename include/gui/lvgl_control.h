/* SPDX-License-Identifier: MIT */
/*
 * lvgl_control.h - LVGL 控制面板模块
 *
 * 本地播放器控制界面:
 *   - 进度条 + 时间显示
 *   - 播放/暂停/快进/快退/逐帧
 *   - 倍速控制
 *   - 本地视频文件浏览器
 *   - 半透明控制栏自动隐藏
 */

#ifndef __LVGL_CONTROL_H__
#define __LVGL_CONTROL_H__

#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 控制命令 */
typedef enum {
    CSG_CTRL_CMD_STOP_PREVIEW  = 2,
    CSG_CTRL_CMD_PAUSE         = 3,
    CSG_CTRL_CMD_RESUME        = 4,
    CSG_CTRL_CMD_QUIT          = 5,
    CSG_CTRL_CMD_SEEK_POSITION = 8,
    CSG_CTRL_CMD_STEP_FORWARD  = 10,
    CSG_CTRL_CMD_STEP_BACKWARD = 11,
    CSG_CTRL_CMD_TOGGLE_PAUSE  = 12,
    CSG_CTRL_CMD_NEXT_FRAME    = 13,
    CSG_CTRL_CMD_SHOW_CONTROLS = 14,
    CSG_CTRL_CMD_PLAY_LOCAL    = 17,
    CSG_CTRL_CMD_STOP_LOCAL    = 18,
    CSG_CTRL_CMD_LOCAL_BROWSE  = 19,
    CSG_CTRL_CMD_LOCAL_SELECT  = 20,
    CSG_CTRL_CMD_LOCAL_PAUSE   = 21,
    CSG_CTRL_CMD_LOCAL_RESUME  = 22,
    CSG_CTRL_CMD_LOCAL_STOP    = 23,
    CSG_CTRL_CMD_SPEED_SET     = 24,
    /* RTSP 拉流 (live555) */
    CSG_CTRL_CMD_RTSP_PLAY     = 25,
    CSG_CTRL_CMD_RTSP_STOP     = 26,
    /* 录制 (MP4v2 直接 remux) */
    CSG_CTRL_CMD_RECORD_START  = 27,
    CSG_CTRL_CMD_RECORD_STOP   = 28,
    /* 多路拉流 (NVR 分屏: 每路独立 RTSP/MPP/录制) */
    CSG_CTRL_CMD_MULTI_PLAY    = 29,
    CSG_CTRL_CMD_MULTI_STOP    = 30,
} _csg_ctrl_cmd_e;

typedef _csg_ctrl_cmd_e csg_ctrl_cmd_t;

/* Seek 参数 */
typedef struct {
    double   position;
    int64_t  time;
    int      fast_seek;
} _csg_ctrl_seek_param_s;

typedef _csg_ctrl_seek_param_s csg_ctrl_seek_param_t;

/* RTSP 播放参数 (CSG_CTRL_CMD_RTSP_PLAY):
 * preview_url 预览子码流 (喂 MPP 显示); record_url 录制主码流 (自动录制),
 * 可为空字符串, 为空时回退用 preview_url. */
typedef struct {
    char preview_url[512];
    char record_url[512];
} _csg_ctrl_rtsp_play_s;

typedef _csg_ctrl_rtsp_play_s csg_ctrl_rtsp_play_t;

/* 多路播放参数 (CSG_CTRL_CMD_MULTI_PLAY).
 * 双码流分离:
 *   每路 (preview[i], record[i]).
 *   count==1 (单路): 用 preview[0] 主码流既显示又录制 (record[0] 可空).
 *   count>1  (多路): 每路子码流(preview)解码显示 + 主码流(record)独立录制.
 */
#define CSG_CTRL_MULTI_MAX_URL 32
typedef struct {
    char preview[CSG_CTRL_MULTI_MAX_URL][512];  /* 预览 URL */
    char record [CSG_CTRL_MULTI_MAX_URL][512];  /* 录制主码流 URL (可空) */
    int  count;                                 /* 路数 */
} _csg_ctrl_multi_play_s;

typedef _csg_ctrl_multi_play_s csg_ctrl_multi_play_t;

/* 控制回调 */
typedef void (*csg_ctrl_callback)(csg_ctrl_cmd_t cmd, void *data, void *user_data);

/* 初始化控制面板 */
int csg_lvgl_control_init(csg_ctrl_callback cb, void *user_data);

/* 更新播放状态 */
void csg_lvgl_control_set_state(csg_player_state_t state);

/* 更新统计信息 */
void csg_lvgl_control_update_stats(csg_player_stats_t *stats);

/* 更新播放进度 */
void csg_lvgl_control_update_progress(int64_t current_ts, int64_t length, int live);
void csg_lvgl_control_update_progress_forced(int64_t current_ts, int64_t length,
                                          int live, int force_clear_seeking);

/* 暂停时冻结进度条 */
void csg_lvgl_control_freeze_progress(int64_t ts);

/* 重置进度条插值状态 (新视频/切换文件/停止时调用) */
void csg_lvgl_control_reset_progress(void);

/* seek 瞬时跳转 */
void csg_lvgl_control_seek_instant(double position, int64_t time);

/* 切换播放/暂停按钮图标 */
void csg_lvgl_control_set_paused_ui(int paused);

/* 设置倍速因子 */
void csg_lvgl_control_set_speed(float speed);

/* 更新直播状态 */
void csg_lvgl_control_set_live(int live);

/* 显示控制栏 */
void csg_lvgl_control_show_bar(void);

/* 显示提示消息 */
void csg_lvgl_control_show_message(const char *msg);

/* 释放控制面板 */
void csg_lvgl_control_deinit(void);

/* ==================== 本地视频浏览器 ==================== */

/* 显示/隐藏本地视频浏览器面板 */
void csg_lvgl_control_show_local_browser(int show);

/* 更新本地视频文件列表 */
void csg_lvgl_control_update_local_list(void *files, int count);

/* 设置本地浏览器中选中的文件路径 */
void csg_lvgl_control_set_local_file_path(const char *path);

/* 本地播放结束 */
void csg_lvgl_control_local_eof(void);

/* ==================== RTSP 拉流 / 录制 ==================== */

/* 显示/隐藏 RTSP URL 连接面板 */
void csg_lvgl_control_show_connect_panel(int show);

/* 设置录制状态 (REC/STOP 按钮文字与颜色切换, 由 main 线程调用) */
void csg_lvgl_control_set_recording(int recording);

#ifdef __cplusplus
}
#endif

#endif /* __LVGL_CONTROL_H__ */