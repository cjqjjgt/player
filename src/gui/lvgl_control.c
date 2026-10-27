/* SPDX-License-Identifier: MIT */
/*
 * lvgl_control.c - LVGL 控制面板实现
 *
 * 功能:
 *   - URL 输入框 + 播放/停止按钮 (运行时直接输入 rtsp:// 或 udp:// 地址拉流)
 *   - 播放控制栏 (参考 VLC vlc_player_timer_point 设计):
 *     - 进度条: 记录 (流时间戳, 系统时刻) 采样点, LVGL 线程按系统时钟插值推算当前进度
 *     - 暂停: 以 pause_date 冻结插值, 画面保持暂停前最后一帧
 *     - seek: seeking 期间进度条显示目标值, 新帧到达后恢复插值
 *     - 控制栏: 半透明底部面板, 3 秒无操作自动隐藏, 触摸唤醒
 */

#include "lvgl_control.h"
#include "local_playback.h"
#include "lvgl_display.h"   /* csg_lvgl_lock/csg_lvgl_unlock: 全局 LVGL 递归锁 */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <time.h>

#include "lvgl.h"

/* 控制栏自动隐藏超时 (ms) */
#define CONTROL_BAR_HIDE_TIMEOUT  3000
/* 进度条刷新周期 (ms) */
#define PROGRESS_REFRESH_PERIOD   100
/* 进度条范围 0~1000 (0.1% 分辨率) */
#define PROGRESS_SLIDER_MAX       1000

/* RTSP 拉流默认地址 (可在连接面板中修改) */
#define DEFAULT_RTSP_URL "rtsp://admin:123456@192.168.1.100/ch01/sub/"

/* 系统时间 (微秒, 单调时钟) — 对应 VLC 的 vlc_tick_now() */
static int64_t csg_lvgl_ctrl_now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

/* ==================== 进度插值状态 ====================
 * 生产者: main 线程 (流回调里采样) / 解码回调线程 (仅写采样点)
 * 消费者: LVGL tick 线程 (csg_lvgl_ctrl_progress_timer_cb 插值显示)
 * 通过 s_csg_lvgl_ctrl_progress.mutex 保护 */
typedef struct {
    pthread_mutex_t mutex;
    int64_t ts;              /* 流时间戳 (微秒) */
    int64_t system_date;     /* 采样 ts 时的系统时刻 (微秒) */
    int64_t length;          /* 总时长 (微秒, CSG_PLAYER_TICK_INVALID=未知/直播) */
    double  position;        /* 位置 0.0~1.0 */
    float   rate;            /* 播放速率 (1.0=正常) */
    float   speed;           /* 倍速因子 (0.5/0.75/1.0/1.5/2.0) */
    int     live;            /* 1=直播流 */
    int     paused;          /* 1=已暂停 (插值冻结) */
    int64_t pause_date;      /* 暂停时的系统时刻 (微秒) */
    int     seeking;         /* 1=seek进行中 (显示目标值) */
    double  seek_pos;        /* seek 目标位置 */
    int64_t seek_ts;         /* seek 目标时间 (微秒) */
} csg_lvgl_ctrl_progress_t;

static csg_lvgl_ctrl_progress_t s_csg_lvgl_ctrl_progress;

typedef struct {
    csg_ctrl_callback   callback;
    void           *user_data;

    /* 主屏幕控件 */
    lv_obj_t       *title_label;
    lv_obj_t       *status_label;
    lv_obj_t       *fps_label;

    /* 播放状态 */
    csg_player_state_t  state;

    /* ==================== 播放控制栏 ==================== */
    lv_obj_t       *control_bar;       /* 半透明底部控制栏容器 */
    lv_obj_t       *play_pause_btn;
    lv_obj_t       *play_pause_label;
    lv_obj_t       *bwd_btn;           /* 快退 -10s */
    lv_obj_t       *bwd_label;
    lv_obj_t       *fwd_btn;           /* 快进 +10s */
    lv_obj_t       *fwd_label;
    lv_obj_t       *progress_slider;   /* 进度条/seek */
    lv_obj_t       *time_label;        /* "01:23 / 05:00" 或 "LIVE 01:23" */
    lv_timer_t     *auto_hide_timer;   /* 自动隐藏定时器 */
    lv_timer_t     *progress_timer;    /* 进度刷新定时器 */
    int             overlay_visible;
    int             user_interacting;  /* 拖动进度条中 (禁止自动隐藏) */
    int             live;              /* 1=直播流 (禁用seek) */

    /* 统计缓存: 解码线程写, LVGL 线程渲染 */
    csg_player_stats_t  ctrl_stats;

    /* ==================== 本地视频浏览器 ==================== */
    lv_obj_t       *local_browser;       /* 本地视频浏览器面板 */
    lv_obj_t       *local_list;          /* 视频文件列表 */
    lv_obj_t       *local_path_label;    /* 当前播放文件路径 */
    lv_obj_t       *local_scan_btn;      /* 扫描按钮 */
    lv_obj_t       *local_play_btn;      /* 本地播放按钮 */
    lv_obj_t       *local_stop_btn;      /* 本地停止按钮 */
    lv_obj_t       *local_entry_btn;    /* 入口按钮: "Local Video" */
    lv_obj_t       *local_entry_label;
    int             local_browser_visible;
    char            local_selected_path[512]; /* 选中的文件路径 */

    /* 倍速控制 (按钮 + 弹出菜单) */
    lv_obj_t       *speed_btn;          /* 倍速按钮 */
    lv_obj_t       *speed_label;        /* 当前倍速文字 */
    lv_obj_t       *speed_popup;        /* 弹出选择面板 */
    lv_obj_t       *speed_popup_btns[6]; /* 6个选项按钮: 1x/2x/4x/6x/8x/16x */

    /* ==================== RTSP 拉流连接面板 ==================== */
    lv_obj_t       *rtsp_entry_btn;     /* 入口按钮: "RTSP" */
    lv_obj_t       *rtsp_entry_label;
    lv_obj_t       *connect_panel;      /* URL 输入面板 */
    lv_obj_t       *url_ta;             /* 预览子码流 URL 输入框 */
    lv_obj_t       *record_ta;          /* 录制主码流 URL 输入框 (可选) */
    lv_obj_t       *url_play_btn;       /* 播放按钮 */
    lv_obj_t       *url_stop_btn;       /* 停止按钮 */
    lv_obj_t       *url_kb;             /* 软键盘 */
    int             connect_panel_visible;

    /* ==================== 录制按钮 ==================== */
    lv_obj_t       *record_btn;         /* REC/STOP 按钮 */
    lv_obj_t       *record_label;
    int             recording;          /* 1=正在录制 */
} csg_lvgl_ctrl_ctx_t;

static csg_lvgl_ctrl_ctx_t *s_csg_lvgl_ctrl_ctx = NULL;

static void csg_lvgl_ctrl_progress_lock(void)
{
    pthread_mutex_lock(&s_csg_lvgl_ctrl_progress.mutex);
}

static void csg_lvgl_ctrl_progress_unlock(void)
{
    pthread_mutex_unlock(&s_csg_lvgl_ctrl_progress.mutex);
}

/* ==================== 时间格式化 ==================== */
static void csg_lvgl_ctrl_format_time_us(int64_t us, char *buf, int buf_size)
{
    if (us < 0) {
        snprintf(buf, buf_size, "--:--");
        return;
    }
    int sec = (int)(us / 1000000);
    int min = sec / 60;
    int hr = min / 60;
    sec %= 60;
    min %= 60;
    if (hr > 0)
        snprintf(buf, buf_size, "%d:%02d:%02d", hr, min, sec);
    else
        snprintf(buf, buf_size, "%02d:%02d", min, sec);
}

/* ==================== 进度插值 (参考 VLC vlc_player_timer_point_Interpolate) ====================
 * 当前进度 = 采样点时间 + (现在 - 采样系统时刻) × 速率
 * 暂停时以 pause_date 冻结 (drift = 0), seek 期间返回目标值 */
static void csg_lvgl_ctrl_progress_interpolate(int64_t now, int64_t *out_ts, double *out_pos)
{
    csg_lvgl_ctrl_progress_lock();

    if (s_csg_lvgl_ctrl_progress.seeking) {
        if (out_ts)  *out_ts = s_csg_lvgl_ctrl_progress.seek_ts;
        if (out_pos) *out_pos = s_csg_lvgl_ctrl_progress.seek_pos;
        csg_lvgl_ctrl_progress_unlock();
        return;
    }

    if (s_csg_lvgl_ctrl_progress.paused)
        now = s_csg_lvgl_ctrl_progress.pause_date;   /* 暂停: 强制插值到暂停时刻 */

    int64_t drift = (now - s_csg_lvgl_ctrl_progress.system_date) * s_csg_lvgl_ctrl_progress.rate * s_csg_lvgl_ctrl_progress.speed;
    int64_t ts = s_csg_lvgl_ctrl_progress.ts + drift;
    if (out_ts) *out_ts = ts;

    if (s_csg_lvgl_ctrl_progress.live || s_csg_lvgl_ctrl_progress.length == CSG_PLAYER_TICK_INVALID ||
        s_csg_lvgl_ctrl_progress.length <= 0) {
        if (out_pos) *out_pos = -1.0;  /* 直播: 无位置概念 */
    } else {
        double pos = s_csg_lvgl_ctrl_progress.position +
                     (double)drift / (double)s_csg_lvgl_ctrl_progress.length;
        if (pos < 0.0) pos = 0.0;
        if (pos > 1.0) pos = 1.0;
        if (out_pos) *out_pos = pos;
    }
    csg_lvgl_ctrl_progress_unlock();
}

/* 更新时间标签; override_pos >= 0 时用该位置 (拖动中预览), 否则用插值结果 */
static void csg_lvgl_ctrl_update_time_label(csg_lvgl_ctrl_ctx_t *ctx, double override_pos)
{
    if (!ctx || !ctx->time_label) return;

    int64_t ts;
    if (override_pos >= 0.0) {
        csg_lvgl_ctrl_progress_lock();
        int64_t length = s_csg_lvgl_ctrl_progress.length;
        csg_lvgl_ctrl_progress_unlock();
        ts = (length > 0 && length != CSG_PLAYER_TICK_INVALID)
                 ? (int64_t)(override_pos * length) : 0;
    } else {
        csg_lvgl_ctrl_progress_interpolate(csg_lvgl_ctrl_now_us(), &ts, NULL);
    }

    char cur[16], tot[16];
    csg_lvgl_ctrl_format_time_us(ts, cur, sizeof(cur));

    char text[64];
    if (ctx->live) {
        snprintf(text, sizeof(text), "LIVE %s", cur);
    } else {
        csg_lvgl_ctrl_progress_lock();
        int64_t length = s_csg_lvgl_ctrl_progress.length;
        csg_lvgl_ctrl_progress_unlock();
        csg_lvgl_ctrl_format_time_us(length, tot, sizeof(tot));
        snprintf(text, sizeof(text), "%s / %s", cur, tot);
    }
    lv_label_set_text(ctx->time_label, text);
}

/* ==================== 控制栏显示/隐藏 ==================== */

void csg_lvgl_control_show_bar(void)
{
    if (!s_csg_lvgl_ctrl_ctx || !s_csg_lvgl_ctrl_ctx->control_bar) return;

    csg_lvgl_lock();
    if (!s_csg_lvgl_ctrl_ctx->overlay_visible) {
        s_csg_lvgl_ctrl_ctx->overlay_visible = 1;
        lv_obj_set_hidden(s_csg_lvgl_ctrl_ctx->control_bar, false);
    }
    if (s_csg_lvgl_ctrl_ctx->auto_hide_timer) {
        lv_timer_resume(s_csg_lvgl_ctrl_ctx->auto_hide_timer);
        lv_timer_reset(s_csg_lvgl_ctrl_ctx->auto_hide_timer);
    }
    csg_lvgl_unlock();
}

static void csg_lvgl_ctrl_hide_bar(void)
{
    if (!s_csg_lvgl_ctrl_ctx || !s_csg_lvgl_ctrl_ctx->control_bar) return;
    if (s_csg_lvgl_ctrl_ctx->user_interacting) return;   /* 拖动中不隐藏 */

    s_csg_lvgl_ctrl_ctx->overlay_visible = 0;
    lv_obj_set_hidden(s_csg_lvgl_ctrl_ctx->control_bar, true);
    if (s_csg_lvgl_ctrl_ctx->auto_hide_timer)
        lv_timer_pause(s_csg_lvgl_ctrl_ctx->auto_hide_timer);
}

static void csg_lvgl_ctrl_auto_hide_timer_cb(lv_timer_t *timer)
{
    csg_lvgl_ctrl_ctx_t *ctx = (csg_lvgl_ctrl_ctx_t *)lv_timer_get_user_data(timer);
    if (!ctx) return;
    if (ctx->user_interacting) {
        lv_timer_reset(timer);   /* 拖动中不隐藏 */
        return;
    }
    csg_lvgl_ctrl_hide_bar();
}

/* 任意触摸屏幕 → 唤醒控制栏 */
static void csg_lvgl_ctrl_screen_press_cb(lv_event_t *e)
{
    (void)e;
    csg_lvgl_control_show_bar();
}

/* ==================== 进度条刷新 (LVGL 线程, 100ms) ==================== */
static void csg_lvgl_ctrl_progress_timer_cb(lv_timer_t *timer)
{
    csg_lvgl_ctrl_ctx_t *ctx = (csg_lvgl_ctrl_ctx_t *)lv_timer_get_user_data(timer);
    if (!ctx) return;

    /* 直播状态 UI 同步: 在 LVGL 线程内从 progress.live 同步 ctx->live
     * (update_progress 可能来自解码/拉流线程, 不再跨线程写 ctx->live) */
    csg_lvgl_ctrl_progress_lock();
    ctx->live = s_csg_lvgl_ctrl_progress.live;
    csg_lvgl_ctrl_progress_unlock();
    if (ctx->live)
        lv_obj_add_state(ctx->progress_slider, LV_STATE_DISABLED);
    else
        lv_obj_clear_state(ctx->progress_slider, LV_STATE_DISABLED);
    if (ctx->bwd_btn) lv_obj_set_hidden(ctx->bwd_btn, ctx->live);
    if (ctx->fwd_btn) lv_obj_set_hidden(ctx->fwd_btn, ctx->live);

    int64_t now = csg_lvgl_ctrl_now_us();
    int64_t ts;
    double pos;
    csg_lvgl_ctrl_progress_interpolate(now, &ts, &pos);

    /* 进度条 (拖动中不回写, 避免与用户手指抢位置) */
    if (pos >= 0.0 && !ctx->user_interacting) {
        lv_slider_set_value(ctx->progress_slider,
                            (int32_t)(pos * PROGRESS_SLIDER_MAX + 0.5),
                            LV_ANIM_OFF);
    }
    csg_lvgl_ctrl_update_time_label(ctx, -1.0);

    /* FPS 标签渲染 (右上角) */
    char text[64];
    snprintf(text, sizeof(text), "FPS: %.1f", ctx->ctrl_stats.fps);
    lv_label_set_text(ctx->fps_label, text);
}

/* ==================== 控制栏事件回调 ==================== */

static void csg_lvgl_ctrl_play_pause_btn_cb(lv_event_t *e)
{
    csg_lvgl_ctrl_ctx_t *ctx = lv_event_get_user_data(e);
    if (ctx->callback) {
        ctx->callback(CSG_CTRL_CMD_TOGGLE_PAUSE, NULL, ctx->user_data);
    }
}

static void csg_lvgl_ctrl_bwd_btn_cb(lv_event_t *e)
{
    csg_lvgl_ctrl_ctx_t *ctx = lv_event_get_user_data(e);
    if (ctx->callback) {
        ctx->callback(CSG_CTRL_CMD_STEP_BACKWARD, NULL, ctx->user_data);
    }
}

static void csg_lvgl_ctrl_fwd_btn_cb(lv_event_t *e)
{
    csg_lvgl_ctrl_ctx_t *ctx = lv_event_get_user_data(e);
    if (ctx->callback) {
        ctx->callback(CSG_CTRL_CMD_STEP_FORWARD, NULL, ctx->user_data);
    }
}

/* 倍速按钮: 点击弹出/收起选择面板 */
static const float s_csg_lvgl_ctrl_speed_values[] = { 1.0f, 2.0f, 4.0f, 6.0f, 8.0f, 16.0f };
static const char *s_csg_lvgl_ctrl_speed_labels[] = { "1x", "2x", "4x", "6x", "8x", "16x" };
#define SPEED_OPTION_COUNT 6

static void csg_lvgl_ctrl_speed_popup_btn_cb(lv_event_t *e)
{
    csg_lvgl_ctrl_ctx_t *ctx = lv_event_get_user_data(e);
    lv_obj_t *btn = lv_event_get_target(e);
    if (!ctx || !btn) return;

    /* 从按钮的 user_data 中取出倍速值 (存储时乘以 1000 转为 int) */
    int val = (int)(intptr_t)lv_obj_get_user_data(btn);
    float speed = (float)val / 1000.0f;
    if (speed < 0.5f) return;

    /* 用 speed 值匹配标签文字 (比取子控件 text 更可靠) */
    for (int i = 0; i < SPEED_OPTION_COUNT; i++) {
        float diff = s_csg_lvgl_ctrl_speed_values[i] - speed;
        if (diff < 0.0f) diff = -diff;
        if (diff < 0.05f) {
            lv_label_set_text(ctx->speed_label, s_csg_lvgl_ctrl_speed_labels[i]);
            break;
        }
    }

    /* 收起弹出面板 */
    lv_obj_set_hidden(ctx->speed_popup, true);

    /* 通知播放器 */
    if (ctx->callback) {
        ctx->callback(CSG_CTRL_CMD_SPEED_SET, &speed, ctx->user_data);
    }
}

/* 点击屏幕任意位置, 如果弹出面板打开且点击在外部, 则收起 */
static void csg_lvgl_ctrl_speed_popup_outside_cb(lv_event_t *e)
{
    csg_lvgl_ctrl_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->speed_popup) return;
    if (lv_obj_is_hidden(ctx->speed_popup)) return;

    lv_indev_t *indev = lv_indev_get_act();
    if (!indev) return;
    lv_point_t point;
    lv_indev_get_point(indev, &point);
    if (point.x < 0 || point.y < 0) return;

    /* 点击在弹出面板内部 → 不收起 */
    lv_area_t popup_area;
    lv_obj_get_coords(ctx->speed_popup, &popup_area);
    if (point.x >= popup_area.x1 && point.x <= popup_area.x2 &&
        point.y >= popup_area.y1 && point.y <= popup_area.y2) return;

    /* 点击在倍速按钮上 → 不收起 (让按钮回调处理切换) */
    lv_area_t btn_area;
    lv_obj_get_coords(ctx->speed_btn, &btn_area);
    if (point.x >= btn_area.x1 && point.x <= btn_area.x2 &&
        point.y >= btn_area.y1 && point.y <= btn_area.y2) return;

    /* 其他位置 → 收起 */
    lv_obj_set_hidden(ctx->speed_popup, true);
}

static void csg_lvgl_ctrl_speed_btn_cb(lv_event_t *e)
{
    csg_lvgl_ctrl_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->speed_popup) return;

    /* 切换弹出面板显示/隐藏 */
    bool hidden = lv_obj_is_hidden(ctx->speed_popup);
    if (hidden) {
        /* 弹出时定位到按钮上方, 防止超出屏幕顶部 */
        lv_obj_update_layout(ctx->speed_btn);
        lv_area_t btn_area;
        lv_obj_get_coords(ctx->speed_btn, &btn_area);
        int popup_h = 38 * SPEED_OPTION_COUNT + 8;
        int popup_y = btn_area.y1 - popup_h - 4;
        if (popup_y < 0) popup_y = 0;
        lv_obj_set_pos(ctx->speed_popup, btn_area.x1 - 4, popup_y);
        /* 确保弹出面板在最顶层 */
        lv_obj_move_foreground(ctx->speed_popup);
    }
    lv_obj_set_hidden(ctx->speed_popup, !hidden);
}

static void csg_lvgl_ctrl_progress_slider_cb(lv_event_t *e)
{
    csg_lvgl_ctrl_ctx_t *ctx = lv_event_get_user_data(e);
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_PRESSING) {
        /* 拖动中: 禁止自动隐藏 + 本地预览时间 */
        ctx->user_interacting = 1;
        csg_lvgl_control_show_bar();
        double pos = (double)lv_slider_get_value(ctx->progress_slider) /
                     PROGRESS_SLIDER_MAX;
        csg_lvgl_ctrl_update_time_label(ctx, pos);
    } else if (code == LV_EVENT_RELEASED) {
        ctx->user_interacting = 0;
        csg_lvgl_control_show_bar();
        if (lv_obj_has_state(ctx->progress_slider, LV_STATE_DISABLED))
            return;   /* 直播流: 忽略 seek */
        double pos = (double)lv_slider_get_value(ctx->progress_slider) /
                     PROGRESS_SLIDER_MAX;
        if (ctx->callback) {
            ctx->callback(CSG_CTRL_CMD_SEEK_POSITION, &pos, ctx->user_data);
        }
    }
}

/* ==================== 本地视频浏览器 ==================== */

static void csg_lvgl_ctrl_local_scan_btn_cb(lv_event_t *e)
{
    csg_lvgl_ctrl_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->callback) return;
    ctx->callback(CSG_CTRL_CMD_LOCAL_BROWSE, NULL, ctx->user_data);
}

static void csg_lvgl_ctrl_local_play_btn_cb(lv_event_t *e)
{
    csg_lvgl_ctrl_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->callback) return;

    if (ctx->local_selected_path[0] == '\0') {
        CSG_LOGW("LVGL_CTRL", "No file selected for local playback");
        csg_lvgl_control_show_message("Select a file first!");
        return;
    }
    char *path_copy = strdup(ctx->local_selected_path);
    if (path_copy) {
        ctx->callback(CSG_CTRL_CMD_LOCAL_SELECT, path_copy, ctx->user_data);
    }
}

static void csg_lvgl_ctrl_local_stop_btn_cb(lv_event_t *e)
{
    csg_lvgl_ctrl_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->callback) return;
    ctx->callback(CSG_CTRL_CMD_LOCAL_STOP, NULL, ctx->user_data);
}

static void csg_lvgl_ctrl_local_file_item_cb(lv_event_t *e)
{
    char *path = (char *)lv_event_get_user_data(e);
    if (!s_csg_lvgl_ctrl_ctx) return;

    if (path) {
        strncpy(s_csg_lvgl_ctrl_ctx->local_selected_path, path, sizeof(s_csg_lvgl_ctrl_ctx->local_selected_path) - 1);
        if (s_csg_lvgl_ctrl_ctx->local_path_label) {
            const char *name = strrchr(path, '/');
            name = name ? name + 1 : path;
            lv_label_set_text(s_csg_lvgl_ctrl_ctx->local_path_label, name);
        }
        CSG_LOGI("LVGL_CTRL", "Selected local file: %s", path);
    }
}

static void csg_lvgl_ctrl_local_file_item_delete_cb(lv_event_t *e)
{
    char *path = (char *)lv_event_get_user_data(e);
    if (path) {
        free(path);
    }
}

static void csg_lvgl_ctrl_local_entry_btn_cb(lv_event_t *e)
{
    csg_lvgl_ctrl_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) return;

    ctx->local_browser_visible = !ctx->local_browser_visible;
    lv_obj_set_hidden(ctx->local_browser, !ctx->local_browser_visible);
    if (ctx->local_browser_visible && ctx->callback) {
        ctx->callback(CSG_CTRL_CMD_LOCAL_BROWSE, NULL, ctx->user_data);
    }
}

/* 点击面板外部 → 收起面板 */
static void csg_lvgl_ctrl_local_browser_outside_cb(lv_event_t *e)
{
    csg_lvgl_ctrl_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->local_browser) return;
    if (!ctx->local_browser_visible) return;

    lv_indev_t *indev = lv_indev_get_act();
    if (!indev) return;
    lv_point_t point;
    lv_indev_get_point(indev, &point);
    if (point.x < 0 || point.y < 0) return;

    lv_area_t panel_area;
    lv_obj_get_coords(ctx->local_browser, &panel_area);
    bool inside = (point.x >= panel_area.x1 && point.x <= panel_area.x2 &&
                   point.y >= panel_area.y1 && point.y <= panel_area.y2);
    if (!inside) {
        ctx->local_browser_visible = 0;
        lv_obj_set_hidden(ctx->local_browser, true);
    }
}

/* ==================== RTSP 拉流 / 录制 ==================== */

/* 录制按钮: 未录制 → 发送 RECORD_START(主码流URL); 录制中 → 发送 RECORD_STOP
 * 主码流 URL 留空时回退用预览子码流 URL (单码流录制, 仍可用). */
static void csg_lvgl_ctrl_record_btn_cb(lv_event_t *e)
{
    csg_lvgl_ctrl_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->callback) return;

    if (!ctx->recording) {
        const char *rec = (ctx->record_ta) ? lv_textarea_get_text(ctx->record_ta) : "";
        const char *src = (rec && rec[0]) ? rec
                        : (ctx->url_ta ? lv_textarea_get_text(ctx->url_ta) : "");
        if (!src || src[0] == '\0') {
            csg_lvgl_control_show_message("Input RTSP URL!");
            return;
        }
        char *url = strdup(src);
        if (url) {
            CSG_LOGI("LVGL_CTRL", "Record main-stream: %s", url);
            ctx->callback(CSG_CTRL_CMD_RECORD_START, url, ctx->user_data);
        }
    } else {
        ctx->callback(CSG_CTRL_CMD_RECORD_STOP, NULL, ctx->user_data);
    }
}

/* 从 URL 输入框提取地址并发起拉流 (每路一行, 自动码流切换):
 *   1 行      → 单路, 主码流显示+录制
 *   N 行      → 多路, 子码流预览 + 主码流录制 (程序自动切) */
static void csg_lvgl_ctrl_url_play(csg_lvgl_ctrl_ctx_t *ctx)
{
    if (!ctx || !ctx->url_ta || !ctx->callback) return;

    const char *url = lv_textarea_get_text(ctx->url_ta);
    if (!url || url[0] == '\0') {
        csg_lvgl_control_show_message("Input RTSP URL!");
        return;
    }

    /* 逐行拆分: 忽略空行/首尾空白, 最多 CSG_CTRL_MULTI_MAX_URL 行 */
    char urls[CSG_CTRL_MULTI_MAX_URL][512];
    int count = 0;
    const char *p = url;
    while (*p && count < CSG_CTRL_MULTI_MAX_URL) {
        while (*p == '\n' || *p == '\r' || *p == ' ' || *p == '\t') p++;
        if (!*p) break;
        const char *e = p;
        while (*e && *e != '\n' && *e != '\r') e++;
        int len = (int)(e - p);
        while (len > 0 && (p[len - 1] == ' ' || p[len - 1] == '\t')) len--;
        if (len > 0 && len < 512) {
            memcpy(urls[count], p, (size_t)len);
            urls[count][len] = '\0';
            count++;
        }
        p = e;
    }
    if (count == 0) {
        csg_lvgl_control_show_message("Input RTSP URL!");
        return;
    }

    /* 收起键盘与面板 */
    if (ctx->url_kb) {
        lv_keyboard_set_textarea(ctx->url_kb, NULL);
        lv_obj_set_hidden(ctx->url_kb, true);
    }
    ctx->connect_panel_visible = 0;
    if (ctx->connect_panel) lv_obj_set_hidden(ctx->connect_panel, true);

    /* 每路一行预览URL, 录制主码流由 nvr_multi 自动切换生成 */
    csg_ctrl_multi_play_t *mp = calloc(1, sizeof(*mp));
    if (!mp) return;
    mp->count = count;
    for (int i = 0; i < count; i++)
        snprintf(mp->preview[i], sizeof(mp->preview[i]), "%.511s", urls[i]);

    if (count <= 1) {
        CSG_LOGI("LVGL_CTRL", "RTSP single play (auto main): %s", mp->preview[0]);
    } else {
        CSG_LOGI("LVGL_CTRL", "Multi play (sub preview + main record): %d chn", count);
    }
    ctx->callback(CSG_CTRL_CMD_MULTI_PLAY, mp, ctx->user_data);
}

static void csg_lvgl_ctrl_url_play_btn_cb(lv_event_t *e)
{
    csg_lvgl_ctrl_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) return;
    csg_lvgl_ctrl_url_play(ctx);
}

static void csg_lvgl_ctrl_url_stop_btn_cb(lv_event_t *e)
{
    csg_lvgl_ctrl_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->callback) return;
    ctx->callback(CSG_CTRL_CMD_RTSP_STOP, NULL, ctx->user_data);
}

/* RTSP 入口按钮: 切换连接面板显隐 */
static void csg_lvgl_ctrl_rtsp_entry_btn_cb(lv_event_t *e)
{
    csg_lvgl_ctrl_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->connect_panel) return;

    ctx->connect_panel_visible = !ctx->connect_panel_visible;
    lv_obj_set_hidden(ctx->connect_panel, !ctx->connect_panel_visible);
}

/* 点击面板外部 → 收起连接面板 */
static void csg_lvgl_ctrl_connect_panel_outside_cb(lv_event_t *e)
{
    csg_lvgl_ctrl_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->connect_panel) return;
    if (!ctx->connect_panel_visible) return;

    lv_indev_t *indev = lv_indev_get_act();
    if (!indev) return;
    lv_point_t point;
    lv_indev_get_point(indev, &point);
    if (point.x < 0 || point.y < 0) return;

    lv_area_t panel_area;
    lv_obj_get_coords(ctx->connect_panel, &panel_area);
    bool inside = (point.x >= panel_area.x1 && point.x <= panel_area.x2 &&
                   point.y >= panel_area.y1 && point.y <= panel_area.y2);
    if (!inside) {
        ctx->connect_panel_visible = 0;
        lv_obj_set_hidden(ctx->connect_panel, true);
    }
}

/* ==================== URL 软键盘 ==================== */

static void csg_lvgl_ctrl_url_kb_cancel_cb(lv_event_t *e)
{
    csg_lvgl_ctrl_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx || !ctx->url_kb) return;
    lv_keyboard_set_textarea(ctx->url_kb, NULL);
    lv_obj_set_hidden(ctx->url_kb, true);
}

static void csg_lvgl_ctrl_url_ta_ready_cb(lv_event_t *e)
{
    csg_lvgl_ctrl_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) return;
    csg_lvgl_ctrl_url_play(ctx);
}

static void csg_lvgl_ctrl_url_ta_focus_cb(lv_event_t *e)
{
    csg_lvgl_ctrl_ctx_t *ctx = lv_event_get_user_data(e);
    if (!ctx) return;
    if (!ctx->url_ta) return;

    if (!ctx->url_kb) {
        ctx->url_kb = lv_keyboard_create(lv_screen_active());
        lv_obj_set_size(ctx->url_kb, LV_HOR_RES, LV_PCT(45));
        lv_obj_align(ctx->url_kb, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_add_event_cb(ctx->url_kb, csg_lvgl_ctrl_url_kb_cancel_cb,
                            LV_EVENT_CANCEL, ctx);
        lv_obj_add_event_cb(ctx->url_kb, csg_lvgl_ctrl_url_ta_ready_cb,
                            LV_EVENT_READY, ctx);
    }
    lv_keyboard_set_textarea(ctx->url_kb, ctx->url_ta);
    lv_obj_set_hidden(ctx->url_kb, false);
}

/* 创建 RTSP URL 连接面板 (右侧, 屏幕高度 40%) */
static void csg_lvgl_ctrl_create_connect_panel(csg_lvgl_ctrl_ctx_t *ctx)
{
    lv_obj_t *scr = lv_screen_active();

    ctx->connect_panel = lv_obj_create(scr);
    lv_obj_set_size(ctx->connect_panel, 680, LV_PCT(40));
    lv_obj_align(ctx->connect_panel, LV_ALIGN_RIGHT_MID, -10, 0);
    lv_obj_set_style_bg_color(ctx->connect_panel, lv_color_hex(0x1E1E2E), 0);
    lv_obj_set_style_bg_opa(ctx->connect_panel, LV_OPA_90, 0);
    lv_obj_set_style_radius(ctx->connect_panel, 8, 0);
    lv_obj_set_style_border_width(ctx->connect_panel, 1, 0);
    lv_obj_set_style_border_color(ctx->connect_panel, lv_color_hex(0x89B4FA), 0);
    lv_obj_set_style_pad_all(ctx->connect_panel, 10, 0);
    lv_obj_set_layout(ctx->connect_panel, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(ctx->connect_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(ctx->connect_panel, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_hidden(ctx->connect_panel, true);

    /* 标题 */
    lv_obj_t *title = lv_label_create(ctx->connect_panel);
    lv_label_set_text(title, "RTSP Stream");
    lv_obj_set_style_text_color(title, lv_color_hex(0x89B4FA), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);

    /* URL 输入框: 多行 (每行一个 URL, 多行=多路分屏; 单行=单路播放) */
    ctx->url_ta = lv_textarea_create(ctx->connect_panel);
    lv_obj_set_size(ctx->url_ta, LV_PCT(100), 88);
    lv_textarea_set_one_line(ctx->url_ta, false);
    lv_textarea_set_text(ctx->url_ta, DEFAULT_RTSP_URL);
    lv_textarea_set_placeholder_text(ctx->url_ta,
        "rtsp://user:pass@ip/ch01/sub/\n"
        "(one URL per line; multi-line = multi-channel preview+record)");
    lv_obj_set_style_bg_color(ctx->url_ta, lv_color_hex(0x313244), 0);
    lv_obj_set_style_text_color(ctx->url_ta, lv_color_hex(0xCDD6F4), 0);
    lv_obj_add_event_cb(ctx->url_ta, csg_lvgl_ctrl_url_ta_focus_cb,
                        LV_EVENT_FOCUSED, ctx);

    /* 录制主码流 URL 输入框 (留空则回退用预览子码流 URL; 多路模式下无效,
     * 每路拉什么录什么, 该框仅在单路模式生效) */
    ctx->record_ta = lv_textarea_create(ctx->connect_panel);
    lv_obj_set_size(ctx->record_ta, LV_PCT(100), 40);
    lv_textarea_set_one_line(ctx->record_ta, true);
    lv_textarea_set_text(ctx->record_ta, "");
    lv_textarea_set_placeholder_text(ctx->record_ta, "Main-stream URL (record, optional)");
    lv_obj_set_style_bg_color(ctx->record_ta, lv_color_hex(0x313244), 0);
    lv_obj_set_style_text_color(ctx->record_ta, lv_color_hex(0xCDD6F4), 0);
    lv_obj_add_event_cb(ctx->record_ta, csg_lvgl_ctrl_url_ta_focus_cb,
                        LV_EVENT_FOCUSED, ctx);

    /* 按钮行 */
    lv_obj_t *row = lv_obj_create(ctx->connect_panel);
    lv_obj_remove_style_all(row);
    lv_obj_set_size(row, LV_PCT(100), 40);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(row, 8, 0);

    ctx->url_play_btn = lv_button_create(row);
    lv_obj_set_size(ctx->url_play_btn, 120, 34);
    lv_obj_set_style_bg_color(ctx->url_play_btn, lv_color_hex(0xA6E3A1), 0);
    lv_obj_set_style_radius(ctx->url_play_btn, 6, 0);
    lv_obj_add_event_cb(ctx->url_play_btn, csg_lvgl_ctrl_url_play_btn_cb,
                        LV_EVENT_CLICKED, ctx);
    lv_obj_t *play_lbl = lv_label_create(ctx->url_play_btn);
    lv_label_set_text(play_lbl, "Play");
    lv_obj_set_style_text_font(play_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(play_lbl);

    ctx->url_stop_btn = lv_button_create(row);
    lv_obj_set_size(ctx->url_stop_btn, 120, 34);
    lv_obj_set_style_bg_color(ctx->url_stop_btn, lv_color_hex(0xF38BA8), 0);
    lv_obj_set_style_radius(ctx->url_stop_btn, 6, 0);
    lv_obj_add_event_cb(ctx->url_stop_btn, csg_lvgl_ctrl_url_stop_btn_cb,
                        LV_EVENT_CLICKED, ctx);
    lv_obj_t *stop_lbl = lv_label_create(ctx->url_stop_btn);
    lv_label_set_text(stop_lbl, "Stop");
    lv_obj_set_style_text_font(stop_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(stop_lbl);

    /* 点击面板外部收起 */
    lv_obj_add_event_cb(scr, csg_lvgl_ctrl_connect_panel_outside_cb,
                        LV_EVENT_CLICKED, ctx);
}


static void csg_lvgl_ctrl_create_local_browser(csg_lvgl_ctrl_ctx_t *ctx)
{
    lv_obj_t *scr = lv_screen_active();

    /* 本地视频浏览器面板 (左侧, 垂直排列, 填满屏幕高度 90%) */
    ctx->local_browser = lv_obj_create(scr);
    lv_obj_set_size(ctx->local_browser, 500, LV_PCT(90));
    lv_obj_align(ctx->local_browser, LV_ALIGN_LEFT_MID, 10, 0);
    lv_obj_set_style_bg_color(ctx->local_browser, lv_color_hex(0x1E1E2E), 0);
    lv_obj_set_style_bg_opa(ctx->local_browser, LV_OPA_90, 0);
    lv_obj_set_style_radius(ctx->local_browser, 8, 0);
    lv_obj_set_style_border_width(ctx->local_browser, 1, 0);
    lv_obj_set_style_border_color(ctx->local_browser, lv_color_hex(0xA6E3A1), 0);
    lv_obj_set_style_pad_all(ctx->local_browser, 8, 0);
    lv_obj_set_layout(ctx->local_browser, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(ctx->local_browser, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(ctx->local_browser, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_hidden(ctx->local_browser, true);

    /* 标题行 (固定高度) */
    lv_obj_t *row1 = lv_obj_create(ctx->local_browser);
    lv_obj_remove_style_all(row1);
    lv_obj_set_size(row1, LV_PCT(100), 34);
    lv_obj_set_flex_flow(row1, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row1, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(row1, 4, 0);

    lv_obj_t *title = lv_label_create(row1);
    lv_label_set_text(title, "Local Video");
    lv_obj_set_style_text_color(title, lv_color_hex(0xA6E3A1), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);

    ctx->local_path_label = lv_label_create(row1);
    lv_obj_set_flex_grow(ctx->local_path_label, 1);
    lv_label_set_text(ctx->local_path_label, "No file selected");
    lv_obj_set_style_text_color(ctx->local_path_label, lv_color_hex(0xCDD6F4), 0);
    lv_obj_set_style_text_font(ctx->local_path_label, &lv_font_montserrat_12, 0);

    ctx->local_scan_btn = lv_button_create(row1);
    lv_obj_set_size(ctx->local_scan_btn, 60, 28);
    lv_obj_set_style_bg_color(ctx->local_scan_btn, lv_color_hex(0x45475A), 0);
    lv_obj_set_style_radius(ctx->local_scan_btn, 6, 0);
    lv_obj_add_event_cb(ctx->local_scan_btn, csg_lvgl_ctrl_local_scan_btn_cb, LV_EVENT_CLICKED, ctx);
    lv_obj_t *scan_lbl = lv_label_create(ctx->local_scan_btn);
    lv_label_set_text(scan_lbl, "Scan");
    lv_obj_set_style_text_font(scan_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(scan_lbl);

    /* 文件列表区域 (弹性填充剩余高度, 内部可滚动, 面板本身不滚动) */
    lv_obj_t *row2 = lv_obj_create(ctx->local_browser);
    lv_obj_remove_style_all(row2);
    lv_obj_set_size(row2, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_grow(row2, 1);  /* 弹性填充: 吃掉标题行和按钮行之外的所有剩余高度 */
    lv_obj_set_flex_flow(row2, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_column(row2, 2, 0);

    ctx->local_list = lv_list_create(row2);
    lv_obj_set_size(ctx->local_list, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(ctx->local_list, lv_color_hex(0x313244), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ctx->local_list, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(ctx->local_list, 6, LV_PART_MAIN);

    /* 按钮行: Play / Stop (固定高度, 始终可见, 不需滚动面板) */
    lv_obj_t *row3 = lv_obj_create(ctx->local_browser);
    lv_obj_remove_style_all(row3);
    lv_obj_set_size(row3, LV_PCT(100), 40);
    lv_obj_set_flex_flow(row3, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row3, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(row3, 8, 0);

    ctx->local_play_btn = lv_button_create(row3);
    lv_obj_set_size(ctx->local_play_btn, 120, 34);
    lv_obj_set_style_bg_color(ctx->local_play_btn, lv_color_hex(0xA6E3A1), 0);
    lv_obj_set_style_radius(ctx->local_play_btn, 6, 0);
    lv_obj_add_event_cb(ctx->local_play_btn, csg_lvgl_ctrl_local_play_btn_cb, LV_EVENT_CLICKED, ctx);
    lv_obj_t *play_lbl = lv_label_create(ctx->local_play_btn);
    lv_label_set_text(play_lbl, "Play Local");
    lv_obj_set_style_text_font(play_lbl, &lv_font_montserrat_14, 0);
    lv_obj_center(play_lbl);

    ctx->local_stop_btn = lv_button_create(row3);
    lv_obj_set_size(ctx->local_stop_btn, 120, 34);
    lv_obj_set_style_bg_color(ctx->local_stop_btn, lv_color_hex(0xF38BA8), 0);
    lv_obj_set_style_radius(ctx->local_stop_btn, 6, 0);
    lv_obj_add_event_cb(ctx->local_stop_btn, csg_lvgl_ctrl_local_stop_btn_cb, LV_EVENT_CLICKED, ctx);
    lv_obj_t *stop_lbl = lv_label_create(ctx->local_stop_btn);
    lv_label_set_text(stop_lbl, "Stop Local");
    lv_obj_set_style_text_font(stop_lbl, &lv_font_montserrat_14, 0);
    lv_obj_center(stop_lbl);

    /* 点击面板外部 → 自动收起 */
    lv_obj_add_event_cb(scr, csg_lvgl_ctrl_local_browser_outside_cb, LV_EVENT_CLICKED, ctx);

    ctx->local_browser_visible = 0;
}

/* ==================== 控制栏创建 ==================== */

static void csg_lvgl_ctrl_create_control_bar(csg_lvgl_ctrl_ctx_t *ctx)
{
    lv_obj_t *scr = lv_screen_active();

    /* 半透明底部控制栏 */
    ctx->control_bar = lv_obj_create(scr);
    lv_obj_set_size(ctx->control_bar, LV_HOR_RES, 44);
    lv_obj_align(ctx->control_bar, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(ctx->control_bar, lv_color_hex(0x1E1E2E), 0);
    lv_obj_set_style_bg_opa(ctx->control_bar, LV_OPA_80, 0);
    lv_obj_set_style_radius(ctx->control_bar, 0, 0);
    lv_obj_set_style_border_width(ctx->control_bar, 1, 0);
    lv_obj_set_style_border_side(ctx->control_bar, LV_BORDER_SIDE_TOP, 0);
    lv_obj_set_style_border_color(ctx->control_bar, lv_color_hex(0x45475A), 0);
    lv_obj_set_style_pad_all(ctx->control_bar, 4, 0);
    lv_obj_set_hidden(ctx->control_bar, true);

    /* 第 1 行: 快退 / 播放暂停 / 快进 / 进度条 / 时间 */
    lv_obj_t *row1 = lv_obj_create(ctx->control_bar);
    lv_obj_remove_style_all(row1);
    lv_obj_set_size(row1, LV_PCT(100), 34);
    lv_obj_set_pos(row1, 0, 0);
    lv_obj_set_flex_flow(row1, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row1, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_left(row1, 4, 0);
    lv_obj_set_style_pad_right(row1, 4, 0);
    lv_obj_set_style_pad_column(row1, 4, 0);

    /* 快退 */
    ctx->bwd_btn = lv_button_create(row1);
    lv_obj_set_size(ctx->bwd_btn, 52, 30);
    lv_obj_set_style_bg_color(ctx->bwd_btn, lv_color_hex(0x313244), 0);
    lv_obj_set_style_radius(ctx->bwd_btn, 6, 0);
    lv_obj_add_event_cb(ctx->bwd_btn, csg_lvgl_ctrl_bwd_btn_cb, LV_EVENT_CLICKED, ctx);
    ctx->bwd_label = lv_label_create(ctx->bwd_btn);
    lv_label_set_text(ctx->bwd_label, "-30s");
    lv_obj_set_style_text_font(ctx->bwd_label, &lv_font_montserrat_12, 0);
    lv_obj_center(ctx->bwd_label);

    /* 播放/暂停 */
    ctx->play_pause_btn = lv_button_create(row1);
    lv_obj_set_size(ctx->play_pause_btn, 60, 30);
    lv_obj_set_style_bg_color(ctx->play_pause_btn, lv_color_hex(0x89B4FA), 0);
    lv_obj_set_style_radius(ctx->play_pause_btn, 6, 0);
    lv_obj_add_event_cb(ctx->play_pause_btn, csg_lvgl_ctrl_play_pause_btn_cb, LV_EVENT_CLICKED, ctx);
    ctx->play_pause_label = lv_label_create(ctx->play_pause_btn);
    lv_label_set_text(ctx->play_pause_label, "Pause");
    lv_obj_set_style_text_font(ctx->play_pause_label, &lv_font_montserrat_12, 0);
    lv_obj_center(ctx->play_pause_label);

    /* 快进 */
    ctx->fwd_btn = lv_button_create(row1);
    lv_obj_set_size(ctx->fwd_btn, 52, 30);
    lv_obj_set_style_bg_color(ctx->fwd_btn, lv_color_hex(0x313244), 0);
    lv_obj_set_style_radius(ctx->fwd_btn, 6, 0);
    lv_obj_add_event_cb(ctx->fwd_btn, csg_lvgl_ctrl_fwd_btn_cb, LV_EVENT_CLICKED, ctx);
    ctx->fwd_label = lv_label_create(ctx->fwd_btn);
    lv_label_set_text(ctx->fwd_label, "+30s");
    lv_obj_set_style_text_font(ctx->fwd_label, &lv_font_montserrat_12, 0);
    lv_obj_center(ctx->fwd_label);

    /* 倍速按钮 + 弹出选择面板 */
    ctx->speed_btn = lv_button_create(row1);
    lv_obj_set_size(ctx->speed_btn, 52, 30);
    lv_obj_set_style_bg_color(ctx->speed_btn, lv_color_hex(0x313244), 0);
    lv_obj_set_style_radius(ctx->speed_btn, 6, 0);
    lv_obj_set_style_border_width(ctx->speed_btn, 1, 0);
    lv_obj_set_style_border_color(ctx->speed_btn, lv_color_hex(0x74C7EC), 0);
    lv_obj_add_event_cb(ctx->speed_btn, csg_lvgl_ctrl_speed_btn_cb, LV_EVENT_CLICKED, ctx);
    ctx->speed_label = lv_label_create(ctx->speed_btn);
    lv_label_set_text(ctx->speed_label, "1x");
    lv_obj_set_style_text_color(ctx->speed_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(ctx->speed_label, &lv_font_montserrat_12, 0);
    lv_obj_center(ctx->speed_label);

    /* 录制按钮 (REC/STOP 切换) */
    ctx->record_btn = lv_button_create(row1);
    lv_obj_set_size(ctx->record_btn, 52, 30);
    lv_obj_set_style_bg_color(ctx->record_btn, lv_color_hex(0xF38BA8), 0);
    lv_obj_set_style_radius(ctx->record_btn, 6, 0);
    lv_obj_add_event_cb(ctx->record_btn, csg_lvgl_ctrl_record_btn_cb, LV_EVENT_CLICKED, ctx);
    ctx->record_label = lv_label_create(ctx->record_btn);
    lv_label_set_text(ctx->record_label, "REC");
    lv_obj_set_style_text_color(ctx->record_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(ctx->record_label, &lv_font_montserrat_12, 0);
    lv_obj_center(ctx->record_label);

    /* 弹出选择面板 (紧贴按钮上方, 垂直排列 6 个选项, 可滚动)
     * 12px 右侧留白给滚动条, 滚动条在框外围显示 */
    ctx->speed_popup = lv_obj_create(scr);
    lv_obj_set_size(ctx->speed_popup, 72, 38 * SPEED_OPTION_COUNT + 8);
    lv_obj_set_style_bg_color(ctx->speed_popup, lv_color_hex(0x313244), 0);
    lv_obj_set_style_bg_opa(ctx->speed_popup, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(ctx->speed_popup, 6, 0);
    lv_obj_set_style_border_width(ctx->speed_popup, 1, 0);
    lv_obj_set_style_border_color(ctx->speed_popup, lv_color_hex(0x74C7EC), 0);
    lv_obj_set_style_pad_top(ctx->speed_popup, 4, 0);
    lv_obj_set_style_pad_bottom(ctx->speed_popup, 4, 0);
    lv_obj_set_style_pad_left(ctx->speed_popup, 4, 0);
    lv_obj_set_style_pad_right(ctx->speed_popup, 12, 0);  /* 滚动条外侧空间 */
    lv_obj_set_style_pad_row(ctx->speed_popup, 2, 0);      /* 按钮间距 */
    lv_obj_set_layout(ctx->speed_popup, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(ctx->speed_popup, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(ctx->speed_popup, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    /* 允许垂直滚动, 滚动条显示在右侧外围 */
    lv_obj_set_scrollbar_mode(ctx->speed_popup, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_scroll_dir(ctx->speed_popup, LV_DIR_VER);
    lv_obj_set_hidden(ctx->speed_popup, true);
    /* 确保弹出面板在最顶层 */
    lv_obj_move_foreground(ctx->speed_popup);

    for (int i = 0; i < SPEED_OPTION_COUNT; i++) {
        lv_obj_t *btn = lv_button_create(ctx->speed_popup);
        lv_obj_set_size(btn, 56, 32);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x45475A), 0);
        lv_obj_set_style_radius(btn, 4, 0);
        lv_obj_set_style_border_width(btn, 0, 0);
        /* 存储倍速值到 user_data */
        lv_obj_set_user_data(btn, (void *)(intptr_t)(int)(s_csg_lvgl_ctrl_speed_values[i] * 1000));
        lv_obj_add_event_cb(btn, csg_lvgl_ctrl_speed_popup_btn_cb, LV_EVENT_CLICKED, ctx);
        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, s_csg_lvgl_ctrl_speed_labels[i]);
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, 0);
        lv_obj_center(lbl);
        ctx->speed_popup_btns[i] = btn;
    }

    /* 初始状态: 1x 高亮 */
    lv_obj_set_style_bg_color(ctx->speed_popup_btns[0], lv_color_hex(0x74C7EC), 0);

    /* 点击屏幕外部收起弹出面板 */
    lv_obj_add_event_cb(scr, csg_lvgl_ctrl_speed_popup_outside_cb, LV_EVENT_CLICKED, ctx);

    /* 进度条 */
    ctx->progress_slider = lv_slider_create(row1);
    lv_slider_set_range(ctx->progress_slider, 0, PROGRESS_SLIDER_MAX);
    lv_slider_set_value(ctx->progress_slider, 0, LV_ANIM_OFF);
    lv_obj_set_size(ctx->progress_slider, LV_PCT(100), 16);
    lv_obj_set_flex_grow(ctx->progress_slider, 1);
    lv_obj_set_style_bg_color(ctx->progress_slider, lv_color_hex(0x45475A),
                              LV_PART_MAIN);
    lv_obj_set_style_bg_color(ctx->progress_slider, lv_color_hex(0x89B4FA),
                              LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(ctx->progress_slider, lv_color_hex(0xCDD6F4),
                              LV_PART_KNOB);
    lv_obj_add_event_cb(ctx->progress_slider, csg_lvgl_ctrl_progress_slider_cb,
                        LV_EVENT_PRESSING, ctx);
    lv_obj_add_event_cb(ctx->progress_slider, csg_lvgl_ctrl_progress_slider_cb,
                        LV_EVENT_RELEASED, ctx);

    /* 时间标签 */
    ctx->time_label = lv_label_create(row1);
    lv_label_set_text(ctx->time_label, "00:00 / 00:00");
    lv_obj_set_style_text_color(ctx->time_label, lv_color_hex(0xCDD6F4), 0);
    lv_obj_set_style_text_font(ctx->time_label, &lv_font_montserrat_12, 0);
    lv_obj_set_style_pad_left(ctx->time_label, 4, 0);

    /* 自动隐藏 + 进度刷新定时器 */
    ctx->auto_hide_timer = lv_timer_create(csg_lvgl_ctrl_auto_hide_timer_cb,
                                           CONTROL_BAR_HIDE_TIMEOUT, ctx);
    lv_timer_pause(ctx->auto_hide_timer);
    ctx->progress_timer = lv_timer_create(csg_lvgl_ctrl_progress_timer_cb,
                                          PROGRESS_REFRESH_PERIOD, ctx);
}

/* ==================== 初始化 ==================== */

int csg_lvgl_control_init(csg_ctrl_callback cb, void *user_data)
{
    if (s_csg_lvgl_ctrl_ctx) return 0;

    csg_lvgl_lock();
    csg_lvgl_ctrl_ctx_t *ctx = calloc(1, sizeof(csg_lvgl_ctrl_ctx_t));
    if (!ctx) {
        csg_lvgl_unlock();
        return -1;
    }

    ctx->callback = cb;
    ctx->user_data = user_data;
    ctx->state = CSG_PLAYER_STATE_IDLE;
    ctx->live = 0;
    s_csg_lvgl_ctrl_ctx = ctx;

    pthread_mutex_init(&s_csg_lvgl_ctrl_progress.mutex, NULL);
    s_csg_lvgl_ctrl_progress.ts = 0;
    s_csg_lvgl_ctrl_progress.system_date = csg_lvgl_ctrl_now_us();
    s_csg_lvgl_ctrl_progress.length = CSG_PLAYER_TICK_INVALID;
    s_csg_lvgl_ctrl_progress.position = 0.0;
    s_csg_lvgl_ctrl_progress.rate = 1.0f;
    s_csg_lvgl_ctrl_progress.speed = 1.0f;
    s_csg_lvgl_ctrl_progress.live = 0;
    s_csg_lvgl_ctrl_progress.paused = 0;
    s_csg_lvgl_ctrl_progress.pause_date = 0;
    s_csg_lvgl_ctrl_progress.seeking = 0;
    s_csg_lvgl_ctrl_progress.seek_pos = 0.0;
    s_csg_lvgl_ctrl_progress.seek_ts = 0;

    lv_obj_t *scr = lv_screen_active();

    /* 简洁顶部信息栏 */
    lv_obj_t *top_bar = lv_obj_create(scr);
    lv_obj_remove_style_all(top_bar);
    lv_obj_set_size(top_bar, LV_HOR_RES, 36);
    lv_obj_align(top_bar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(top_bar, lv_color_hex(0x1E1E2E), 0);
    lv_obj_set_style_bg_opa(top_bar, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_left(top_bar, 8, 0);
    lv_obj_set_style_pad_right(top_bar, 8, 0);
    lv_obj_set_flex_flow(top_bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(top_bar, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    /* 标题 */
    ctx->title_label = lv_label_create(top_bar);
    lv_label_set_text(ctx->title_label, "RK3572 Player");
    lv_obj_set_style_text_color(ctx->title_label, lv_color_hex(0xCDD6F4), 0);
    lv_obj_set_style_text_font(ctx->title_label, &lv_font_montserrat_14, 0);

    /* 状态标签 */
    ctx->status_label = lv_label_create(top_bar);
    lv_label_set_text(ctx->status_label, "IDLE");
    lv_obj_set_style_text_color(ctx->status_label, lv_color_hex(0xA6E3A1), 0);
    lv_obj_set_style_text_font(ctx->status_label, &lv_font_montserrat_12, 0);

    /* 本地视频入口按钮 */
    ctx->local_entry_btn = lv_button_create(top_bar);
    lv_obj_set_size(ctx->local_entry_btn, 110, 28);
    lv_obj_set_style_bg_color(ctx->local_entry_btn, lv_color_hex(0xA6E3A1), 0);
    lv_obj_set_style_radius(ctx->local_entry_btn, 6, 0);
    lv_obj_add_event_cb(ctx->local_entry_btn, csg_lvgl_ctrl_local_entry_btn_cb, LV_EVENT_CLICKED, ctx);
    ctx->local_entry_label = lv_label_create(ctx->local_entry_btn);
    lv_label_set_text(ctx->local_entry_label, "Local Video");
    lv_obj_set_style_text_font(ctx->local_entry_label, &lv_font_montserrat_12, 0);
    lv_obj_center(ctx->local_entry_label);

    /* RTSP 拉流入口按钮 */
    ctx->rtsp_entry_btn = lv_button_create(top_bar);
    lv_obj_set_size(ctx->rtsp_entry_btn, 90, 28);
    lv_obj_set_style_bg_color(ctx->rtsp_entry_btn, lv_color_hex(0x89B4FA), 0);
    lv_obj_set_style_radius(ctx->rtsp_entry_btn, 6, 0);
    lv_obj_add_event_cb(ctx->rtsp_entry_btn, csg_lvgl_ctrl_rtsp_entry_btn_cb, LV_EVENT_CLICKED, ctx);
    ctx->rtsp_entry_label = lv_label_create(ctx->rtsp_entry_btn);
    lv_label_set_text(ctx->rtsp_entry_label, "RTSP");
    lv_obj_set_style_text_font(ctx->rtsp_entry_label, &lv_font_montserrat_12, 0);
    lv_obj_center(ctx->rtsp_entry_label);

    /* FPS 标签 (右上角, 独立于顶部栏) */
    ctx->fps_label = lv_label_create(scr);
    lv_label_set_text(ctx->fps_label, "FPS: 0.0");
    lv_obj_set_style_text_color(ctx->fps_label, lv_color_hex(0xA6E3A1), 0);
    lv_obj_set_style_text_font(ctx->fps_label, &lv_font_montserrat_14, 0);
    lv_obj_align(ctx->fps_label, LV_ALIGN_TOP_RIGHT, -15, 40);

    /* 播放控制栏 */
    csg_lvgl_ctrl_create_control_bar(ctx);

    /* 本地视频浏览器 */
    csg_lvgl_ctrl_create_local_browser(ctx);

    /* RTSP 连接面板 */
    csg_lvgl_ctrl_create_connect_panel(ctx);

    /* 任意触摸屏幕 → 唤醒底部控制栏 */
    lv_obj_add_event_cb(scr, csg_lvgl_ctrl_screen_press_cb, LV_EVENT_PRESSED, ctx);

    CSG_LOGI("LVGL_CTRL", "Initialized");
    csg_lvgl_unlock();
    return 0;
}

void csg_lvgl_control_set_state(csg_player_state_t state)
{
    if (!s_csg_lvgl_ctrl_ctx) return;
    csg_lvgl_lock();
    s_csg_lvgl_ctrl_ctx->state = state;

    const char *state_str[] = {
        "IDLE", "SCANNING", "CONNECTED", "PLAYING", "PAUSED", "STOPPED", "COMPLETED"
    };
    char text[64];
    snprintf(text, sizeof(text), "%s", state_str[state]);
    lv_label_set_text(s_csg_lvgl_ctrl_ctx->status_label, text);

    switch (state) {
    case CSG_PLAYER_STATE_PLAYING:
        lv_obj_set_style_text_color(s_csg_lvgl_ctrl_ctx->status_label, lv_color_hex(0xA6E3A1), 0);
        lv_label_set_text(s_csg_lvgl_ctrl_ctx->play_pause_label, "Pause");
        csg_lvgl_control_show_bar();
        break;
    case CSG_PLAYER_STATE_PAUSED:
        lv_obj_set_style_text_color(s_csg_lvgl_ctrl_ctx->status_label, lv_color_hex(0xF9E2AF), 0);
        lv_label_set_text(s_csg_lvgl_ctrl_ctx->play_pause_label, "Play");
        csg_lvgl_control_show_bar();
        break;
    case CSG_PLAYER_STATE_IDLE:
    case CSG_PLAYER_STATE_STOPPED:
        lv_obj_set_style_text_color(s_csg_lvgl_ctrl_ctx->status_label, lv_color_hex(0xF38BA8), 0);
        csg_lvgl_control_show_bar();
        break;
    case CSG_PLAYER_STATE_COMPLETED:
        lv_obj_set_style_text_color(s_csg_lvgl_ctrl_ctx->status_label, lv_color_hex(0x89B4FA), 0);
        lv_label_set_text(s_csg_lvgl_ctrl_ctx->play_pause_label, "Play");
        csg_lvgl_control_show_bar();
        break;
    case CSG_PLAYER_STATE_SCANNING:
        lv_obj_set_style_text_color(s_csg_lvgl_ctrl_ctx->status_label, lv_color_hex(0xF9E2AF), 0);
        break;
    default:
        break;
    }
    csg_lvgl_unlock();
}

void csg_lvgl_control_set_paused_ui(int paused)
{
    if (!s_csg_lvgl_ctrl_ctx || !s_csg_lvgl_ctrl_ctx->play_pause_label) return;
    csg_lvgl_lock();
    lv_label_set_text(s_csg_lvgl_ctrl_ctx->play_pause_label, paused ? "Play" : "Pause");

    csg_lvgl_ctrl_progress_lock();
    s_csg_lvgl_ctrl_progress.paused = paused ? 1 : 0;
    int64_t now = csg_lvgl_ctrl_now_us();
    if (paused) {
        s_csg_lvgl_ctrl_progress.pause_date = now;
        s_csg_lvgl_ctrl_progress.system_date = now;
    } else {
        /* ★ 恢复时重设 system_date: 避免 interpolate 用旧 system_date
         * 算出暂停期间的 drift, 导致进度条先跳到前面再退回来. */
        s_csg_lvgl_ctrl_progress.system_date = now;
    }
    csg_lvgl_ctrl_progress_unlock();
    csg_lvgl_unlock();
}

/* 暂停时冻结进度条到指定 PTS, 避免 drain 过程中残留帧覆盖进度 */
void csg_lvgl_control_freeze_progress(int64_t ts)
{
    if (!s_csg_lvgl_ctrl_ctx) return;
    csg_lvgl_lock();

    csg_lvgl_ctrl_progress_lock();
    s_csg_lvgl_ctrl_progress.ts = ts;
    s_csg_lvgl_ctrl_progress.position = (s_csg_lvgl_ctrl_progress.length > 0 &&
                           s_csg_lvgl_ctrl_progress.length != CSG_PLAYER_TICK_INVALID &&
                           ts >= 0)
                              ? (double)ts / (double)s_csg_lvgl_ctrl_progress.length
                              : 0.0;
    s_csg_lvgl_ctrl_progress.paused = 1;
    int64_t now = csg_lvgl_ctrl_now_us();
    s_csg_lvgl_ctrl_progress.pause_date = now;
    s_csg_lvgl_ctrl_progress.system_date = now;
    csg_lvgl_ctrl_progress_unlock();

    csg_lvgl_control_show_bar();
    csg_lvgl_ctrl_update_time_label(s_csg_lvgl_ctrl_ctx, -1.0);
    csg_lvgl_unlock();
}

/* 重置进度条插值状态 (新视频/切换文件/停止时调用, 主线程).
 * 解决: 播放结束后进度条继续读秒、切换文件进度条不归零的问题. */
void csg_lvgl_control_reset_progress(void)
{
    if (!s_csg_lvgl_ctrl_ctx) return;
    csg_lvgl_lock();

    csg_lvgl_ctrl_progress_lock();
    s_csg_lvgl_ctrl_progress.ts = 0;
    s_csg_lvgl_ctrl_progress.length = CSG_PLAYER_TICK_INVALID;
    s_csg_lvgl_ctrl_progress.position = 0.0;
    s_csg_lvgl_ctrl_progress.rate = 1.0f;
    s_csg_lvgl_ctrl_progress.speed = 1.0f;
    s_csg_lvgl_ctrl_progress.live = 0;
    s_csg_lvgl_ctrl_progress.paused = 0;
    s_csg_lvgl_ctrl_progress.pause_date = 0;
    s_csg_lvgl_ctrl_progress.seeking = 0;
    s_csg_lvgl_ctrl_progress.seek_pos = 0.0;
    s_csg_lvgl_ctrl_progress.seek_ts = 0;
    s_csg_lvgl_ctrl_progress.system_date = csg_lvgl_ctrl_now_us();
    csg_lvgl_ctrl_progress_unlock();

    s_csg_lvgl_ctrl_ctx->live = 0;
    if (s_csg_lvgl_ctrl_ctx->progress_slider)
        lv_slider_set_value(s_csg_lvgl_ctrl_ctx->progress_slider, 0, LV_ANIM_OFF);
    if (s_csg_lvgl_ctrl_ctx->time_label)
        lv_label_set_text(s_csg_lvgl_ctrl_ctx->time_label, "00:00 / 00:00");
    csg_lvgl_unlock();
}

void csg_lvgl_control_set_speed(float speed)
{
    csg_lvgl_lock();
    csg_lvgl_ctrl_progress_lock();
    float old_speed = s_csg_lvgl_ctrl_progress.speed;
    s_csg_lvgl_ctrl_progress.speed = speed;

    /* ★ 修复: 保持当前插值时间不变, 用浮点除法避免 (int64_t)speed 截断导致进度条跳动 */
    if (old_speed > 0.0f && (old_speed < speed - 0.05f || old_speed > speed + 0.05f)) {
        int64_t now = csg_lvgl_ctrl_now_us();
        double drift_before = (double)(now - s_csg_lvgl_ctrl_progress.system_date) * (double)old_speed;
        int64_t new_system_date = now - (int64_t)(drift_before / (double)speed);
        s_csg_lvgl_ctrl_progress.system_date = new_system_date;
    }
    csg_lvgl_ctrl_progress_unlock();

    /* 同步更新倍速按钮文字 + 高亮当前选中项 */
    if (s_csg_lvgl_ctrl_ctx && s_csg_lvgl_ctrl_ctx->speed_label) {
        for (int i = 0; i < SPEED_OPTION_COUNT; i++) {
            float diff = s_csg_lvgl_ctrl_speed_values[i] - speed;
            if (diff < 0.0f) diff = -diff;
            if (diff < 0.05f) {
                lv_label_set_text(s_csg_lvgl_ctrl_ctx->speed_label, s_csg_lvgl_ctrl_speed_labels[i]);

                /* 高亮当前选中项, 其他项恢复默认 */
                for (int j = 0; j < SPEED_OPTION_COUNT; j++) {
                    if (s_csg_lvgl_ctrl_ctx->speed_popup_btns[j]) {
                        lv_obj_set_style_bg_color(s_csg_lvgl_ctrl_ctx->speed_popup_btns[j],
                            (j == i) ? lv_color_hex(0x74C7EC) : lv_color_hex(0x45475A), 0);
                    }
                }
                break;
            }
        }
    }
    csg_lvgl_unlock();
}

void csg_lvgl_control_set_live(int live)
{
    if (!s_csg_lvgl_ctrl_ctx) return;
    csg_lvgl_lock();
    s_csg_lvgl_ctrl_ctx->live = live;
    csg_lvgl_control_show_bar();
    csg_lvgl_ctrl_update_time_label(s_csg_lvgl_ctrl_ctx, -1.0);
    csg_lvgl_unlock();
}

/* RTSP 连接面板显隐 (main 线程可调用) */
void csg_lvgl_control_show_connect_panel(int show)
{
    if (!s_csg_lvgl_ctrl_ctx || !s_csg_lvgl_ctrl_ctx->connect_panel) return;
    csg_lvgl_lock();
    s_csg_lvgl_ctrl_ctx->connect_panel_visible = show;
    lv_obj_set_hidden(s_csg_lvgl_ctrl_ctx->connect_panel, !show);
    csg_lvgl_unlock();
}

/* 录制状态 (REC/STOP 按钮文字与颜色切换, main 线程调用) */
void csg_lvgl_control_set_recording(int recording)
{
    if (!s_csg_lvgl_ctrl_ctx || !s_csg_lvgl_ctrl_ctx->record_btn) return;
    csg_lvgl_lock();
    s_csg_lvgl_ctrl_ctx->recording = recording;
    if (recording) {
        lv_obj_set_style_bg_color(s_csg_lvgl_ctrl_ctx->record_btn, lv_color_hex(0xF38BA8), 0);
        lv_label_set_text(s_csg_lvgl_ctrl_ctx->record_label, "STOP");
    } else {
        lv_obj_set_style_bg_color(s_csg_lvgl_ctrl_ctx->record_btn, lv_color_hex(0x313244), 0);
        lv_label_set_text(s_csg_lvgl_ctrl_ctx->record_label, "REC");
    }
    csg_lvgl_unlock();
}

void csg_lvgl_control_update_stats(csg_player_stats_t *stats)
{
    if (!s_csg_lvgl_ctrl_ctx || !stats) return;
    /* 解码线程写统计缓存, LVGL 线程 (csg_lvgl_ctrl_progress_timer_cb) 渲染 */
    s_csg_lvgl_ctrl_ctx->ctrl_stats = *stats;
}

/* 注册进度采样点 (流时间戳, 当前系统时刻)。
 * 可由解码/拉流线程调用, 内部加锁; 不直接操作 LVGL 对象。 */
void csg_lvgl_control_update_progress(int64_t current_ts, int64_t length, int live)
{
    if (!s_csg_lvgl_ctrl_ctx) return;

    csg_lvgl_ctrl_progress_lock();
    int was_seeking = s_csg_lvgl_ctrl_progress.seeking;
    s_csg_lvgl_ctrl_progress.ts = current_ts;
    s_csg_lvgl_ctrl_progress.length = length;
    s_csg_lvgl_ctrl_progress.live = live;
    s_csg_lvgl_ctrl_progress.system_date = csg_lvgl_ctrl_now_us();
    s_csg_lvgl_ctrl_progress.position = (length > 0 && length != CSG_PLAYER_TICK_INVALID &&
                           current_ts >= 0)
                              ? (double)current_ts / (double)length : 0.0;
    /* ★ v3.1: 防御层 — 旧帧不应清除 seeking.
     * 主线程在 post_resume_sync 中已过滤掉绝大部分旧帧, 但极端竞态下
     * 旧帧仍可能先于 Seek 后新帧到达此函数. 若旧帧 PTS 与 seek 目标 PTS
     * 差距 >10 秒, 说明是前一个播放位置的残留帧, 不清除 seeking. */
    if (was_seeking) {
        int64_t seek_ts = s_csg_lvgl_ctrl_progress.seek_ts;
        int64_t diff = (current_ts > seek_ts)
                     ? (current_ts - seek_ts)
                     : (seek_ts - current_ts);
        if (diff <= 10000000) {  /* ≤10 秒 → 正常 Seek 后帧, 清除 seeking */
            s_csg_lvgl_ctrl_progress.seeking = 0;
        }
        /* else: 旧帧, 保留 seeking, 进度条继续显示 seek 目标位置 */
    }
    csg_lvgl_ctrl_progress_unlock();
    /* ctx->live 由 progress_timer_cb 在 LVGL 线程从 progress.live 同步, 这里不跨线程写 */
}

/* 强制更新进度 (用于 Seek 后第一帧): 强制清除 seeking 状态
 * 当 force_clear_seeking=1 时, 立即清除 seeking 状态, 用实际帧 PTS 更新进度 */
void csg_lvgl_control_update_progress_forced(int64_t current_ts, int64_t length,
                                          int live, int force_clear_seeking)
{
    if (!s_csg_lvgl_ctrl_ctx) return;

    csg_lvgl_ctrl_progress_lock();
    s_csg_lvgl_ctrl_progress.ts = current_ts;
    s_csg_lvgl_ctrl_progress.length = length;
    s_csg_lvgl_ctrl_progress.live = live;
    s_csg_lvgl_ctrl_progress.system_date = csg_lvgl_ctrl_now_us();
    s_csg_lvgl_ctrl_progress.position = (length > 0 && length != CSG_PLAYER_TICK_INVALID &&
                           current_ts >= 0)
                              ? (double)current_ts / (double)length : 0.0;
    if (force_clear_seeking)
        s_csg_lvgl_ctrl_progress.seeking = 0;
    csg_lvgl_ctrl_progress_unlock();
}

/* seek 瞬时反馈: 进度条立即显示目标位置 (由 LVGL 线程调用) */
void csg_lvgl_control_seek_instant(double position, int64_t time)
{
    if (!s_csg_lvgl_ctrl_ctx) return;
    csg_lvgl_lock();

    csg_lvgl_ctrl_progress_lock();
    s_csg_lvgl_ctrl_progress.seeking = 1;
    s_csg_lvgl_ctrl_progress.seek_pos = position;
    s_csg_lvgl_ctrl_progress.seek_ts = time;
    csg_lvgl_ctrl_progress_unlock();

    if (!s_csg_lvgl_ctrl_ctx->user_interacting) {
        lv_slider_set_value(s_csg_lvgl_ctrl_ctx->progress_slider,
                            (int32_t)(position * PROGRESS_SLIDER_MAX + 0.5),
                            LV_ANIM_OFF);
    }
    csg_lvgl_ctrl_update_time_label(s_csg_lvgl_ctrl_ctx, position);
    csg_lvgl_control_show_bar();
    csg_lvgl_unlock();
}

void csg_lvgl_control_show_message(const char *msg)
{
    if (!s_csg_lvgl_ctrl_ctx || !msg) return;
    CSG_LOGI("LVGL_CTRL", "%s", msg);
}

/* ==================== 本地视频浏览器公共 API ==================== */

void csg_lvgl_control_show_local_browser(int show)
{
    if (!s_csg_lvgl_ctrl_ctx || !s_csg_lvgl_ctrl_ctx->local_browser) return;
    csg_lvgl_lock();
    s_csg_lvgl_ctrl_ctx->local_browser_visible = show;
    lv_obj_set_hidden(s_csg_lvgl_ctrl_ctx->local_browser, !show);
    csg_lvgl_unlock();
}

void csg_lvgl_control_update_local_list(void *files, int count)
{
    if (!s_csg_lvgl_ctrl_ctx || !s_csg_lvgl_ctrl_ctx->local_list) return;

    csg_local_file_info_t *file_info = (csg_local_file_info_t *)files;
    if (!file_info || count <= 0) {
        CSG_LOGI("LVGL_CTRL", "No files to display");
        return;
    }

    csg_lvgl_lock();
    /* LV_EVENT_DELETE 会触发 csg_lvgl_ctrl_local_file_item_delete_cb 来释放 path_copy */
    lv_obj_clean(s_csg_lvgl_ctrl_ctx->local_list);

    for (int i = 0; i < count; i++) {
        char btn_text[512];
        if (file_info[i].duration_us > 0) {
            int dur_sec = (int)(file_info[i].duration_us / 1000000);
            int dur_min = dur_sec / 60;
            dur_sec %= 60;
            snprintf(btn_text, sizeof(btn_text), "%s  [%dx%d  %d:%02d]",
                     file_info[i].name, file_info[i].width, file_info[i].height,
                     dur_min, dur_sec);
        } else {
            snprintf(btn_text, sizeof(btn_text), "%s  [%dx%d]",
                     file_info[i].name, file_info[i].width, file_info[i].height);
        }

        lv_obj_t *btn = lv_list_add_button(s_csg_lvgl_ctrl_ctx->local_list, NULL, btn_text);
        /* 按钮默认即可点击，无需额外设置 flag */

        char *path_copy = strdup(file_info[i].path);
        lv_obj_add_event_cb(btn, csg_lvgl_ctrl_local_file_item_cb, LV_EVENT_CLICKED, path_copy);
        lv_obj_add_event_cb(btn, csg_lvgl_ctrl_local_file_item_delete_cb, LV_EVENT_DELETE, path_copy);
    }
    csg_lvgl_unlock();
    CSG_LOGI("LVGL_CTRL", "Updated local file list: %d files", count);
}

void csg_lvgl_control_set_local_file_path(const char *path)
{
    if (!s_csg_lvgl_ctrl_ctx || !path) return;
    csg_lvgl_lock();
    strncpy(s_csg_lvgl_ctrl_ctx->local_selected_path, path, sizeof(s_csg_lvgl_ctrl_ctx->local_selected_path) - 1);
    if (s_csg_lvgl_ctrl_ctx->local_path_label) {
        const char *name = strrchr(path, '/');
        name = name ? name + 1 : path;
        lv_label_set_text(s_csg_lvgl_ctrl_ctx->local_path_label, name);
    }
    csg_lvgl_unlock();
}

const char *lvgl_control_get_local_file_path(void)
{
    if (!s_csg_lvgl_ctrl_ctx) return NULL;
    return s_csg_lvgl_ctrl_ctx->local_selected_path;
}

void csg_lvgl_control_local_eof(void)
{
    if (!s_csg_lvgl_ctrl_ctx) return;
    CSG_LOGI("LVGL_CTRL", "Local video playback finished (EOF)");
}

void csg_lvgl_control_deinit(void)
{
    if (!s_csg_lvgl_ctrl_ctx) return;
    csg_lvgl_lock();
    if (s_csg_lvgl_ctrl_ctx->progress_timer) lv_timer_delete(s_csg_lvgl_ctrl_ctx->progress_timer);
    if (s_csg_lvgl_ctrl_ctx->auto_hide_timer) lv_timer_delete(s_csg_lvgl_ctrl_ctx->auto_hide_timer);
    if (s_csg_lvgl_ctrl_ctx->local_browser) lv_obj_del(s_csg_lvgl_ctrl_ctx->local_browser);
    pthread_mutex_destroy(&s_csg_lvgl_ctrl_progress.mutex);
    free(s_csg_lvgl_ctrl_ctx);
    s_csg_lvgl_ctrl_ctx = NULL;
    csg_lvgl_unlock();
    CSG_LOGI("LVGL_CTRL", "Deinitialized");
}