/* SPDX-License-Identifier: MIT */
/*
 * lvgl_display.c - LVGL DRM 显示实现
 *
 * 基于 LVGL 9.x DRM 驱动实现 /dev/dri/card0 渲染
 * 参考: /home/dell/lvgl-master/src/drivers/display/drm
 */

#include "lvgl_display.h"
#include "evdev_touch.h"
#include "common.h"
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>

#include "lvgl.h"

/* 每通道显示窗口: 独立的 LVGL 图像控件 + 双缓冲.
 * 双缓冲: back_idx=解码线程写, pending_idx=待LVGL消费.
 * 解码线程写完交换 back_idx, LVGL 消费 pending_idx. 两者永不冲突,
 * 彻底消除撕裂/鬼影. */
typedef struct {
    lv_obj_t       *img;            /* 图像控件 */
    lv_image_dsc_t  dsc;
    pthread_mutex_t buf_mutex;      /* 本窗口双缓冲锁 */
    uint8_t         *buf[2];
    int              buf_size[2];
    int              back_idx;      /* 解码线程当前写的 buffer */
    int              pending_idx;   /* 待 LVGL 消费的 buffer (-1=无新帧) */
    int              video_w;       /* RGA 输出实际宽 (此尺寸上屏) */
    int              video_h;
    int              dirty;         /* 有待消费新帧 */
    uint32_t         frames;
    /* 分屏网格矩形 (屏幕像素坐标, 多路时有效) */
    int              rx, ry, rw, rh;
    int              hidden;        /* 网格未用到的窗口隐藏 */
} csg_lvgl_win_t;

typedef struct {
    int              screen_w;
    int              screen_h;

    lv_display_t    *display;
    lv_obj_t        *main_scr;
    lv_obj_t        *video_scr;

    /* 多路分屏窗口: chn=0 即原单路播放窗口, 语义完全兼容 */
    csg_lvgl_win_t   win[CSG_LVGL_MAX_VIDEO_CHN];
    int              grid_chn_cnt;  /* 当前分屏通道数 (1=单路全屏) */
    int              grid_cols;
    int              grid_rows;

    /* 暂停/逐帧步进 (仅作用于 chn=0 单路播放器) */
    int              video_paused;
    int              video_step_pending;

    /* 运行状态 */
    int              running;
    int              ready;
    pthread_t        tick_thread;
} csg_lvgl_disp_ctx_t;

static csg_lvgl_disp_ctx_t *s_csg_lvgl_disp_ctx = NULL;

/* ==================== 全局 LVGL 递归互斥锁 ====================
 * LVGL 9.x 非线程安全 (LV_USE_OS=LV_OS_NONE, 无内置锁).
 * 所有跨线程的 LVGL API 调用 (main/解码/拉流线程 → control API,
 * tick 线程 → lv_timer_handler) 必须互斥, 否则触发
 * "lv_inv_area: Asserted ... rendering_in_progress" 断言后 while(1) 死循环,
 * 导致 main 线程卡死 (RTSP 自动录制不启动) 及 UI 卡顿/花屏.
 * 用递归锁: LVGL 定时器/事件回调 (在 lv_timer_handler 内) 会再调用
 * 公开 API, 同一线程重入不死锁. */
static pthread_mutex_t s_lvgl_mutex;

void csg_lvgl_lock(void)
{
    pthread_mutex_lock(&s_lvgl_mutex);
}

void csg_lvgl_unlock(void)
{
    pthread_mutex_unlock(&s_lvgl_mutex);
}

/* 在 LVGL 线程内把某个窗口的最新视频帧交给图像控件.
 * 使用双缓冲: 从 pending_idx 取帧, 解码线程用 back_idx, 两者永不冲突.
 * 渲染期间 buffer 内容稳定, 彻底消除撕裂/鬼影. */
static void csg_lvgl_disp_win_present(csg_lvgl_win_t *w)
{
    if (!w || !w->img) return;

    int frame_idx = -1;
    int vw = 0, vh = 0;

    pthread_mutex_lock(&w->buf_mutex);
    if (w->dirty && w->pending_idx >= 0) {
        frame_idx = w->pending_idx;
        w->dirty = 0;
        w->pending_idx = -1;
        vw = w->video_w;
        vh = w->video_h;
    }
    pthread_mutex_unlock(&w->buf_mutex);

    if (frame_idx < 0 || !w->buf[frame_idx] || vw <= 0 || vh <= 0) return;

    w->frames++;
    if (w->frames == 1 || w->frames % 300 == 0) {
        CSG_LOGI("LVGL_DISP", "present win frame #%u %dx%d",
                 w->frames, vw, vh);
    }

    /* 更新描述符指向 front buffer (此时 LVGL 独占, 解码线程在用 back_idx) */
    w->dsc.header.w = vw;
    w->dsc.header.h = vh;
    w->dsc.header.stride = vw * 4;
    w->dsc.data_size = vw * vh * 4;
    w->dsc.data = w->buf[frame_idx];

    /* 单路 (chn=0): 保持宽高比, 居中显示, 黑边由屏幕背景填充.
     * 多路分屏: 窗口已定位到网格矩形, RGA 已拉伸到矩形尺寸, 100% 显示. */
    lv_image_set_src(w->img, &w->dsc);
    if (w->rw > 0 && w->rh > 0) {
        /* ★★ 多路分屏: 必须先把对齐改为左上角! 控件创建时是 ALIGN_CENTER,
         * 若不改, set_pos(rx,ry) 会被当成"相对屏幕中心"的偏移, 所有分屏窗口
         * 重叠挤到中心偏右下 (左上格 rx=c*cw 为正时整体越界, 一半在屏外). */
        if (w->rx >= 0 && w->ry >= 0) {
            lv_obj_set_align(w->img, LV_ALIGN_TOP_LEFT);
            lv_obj_set_pos(w->img, w->rx, w->ry);
        }
        lv_obj_set_size(w->img, w->rw, w->rh);
        lv_image_set_scale(w->img, 256);   /* 100%: 禁用 LVGL 软件缩放 */
    } else {
        /* 全屏居中路径 (原单路行为) */
        lv_obj_set_align(w->img, LV_ALIGN_CENTER);
        lv_obj_set_size(w->img, vw, vh);
        lv_obj_center(w->img);
        lv_image_set_scale(w->img, 256);
    }
    lv_obj_set_hidden(w->img, false);
    lv_obj_invalidate(w->img);
}

static void csg_lvgl_disp_video_present(csg_lvgl_disp_ctx_t *ctx)
{
    if (!ctx) return;

    /* 暂停/逐帧步进 (仅单路 chn=0) */
    int paused = ctx->video_paused;
    int step = ctx->video_step_pending;
    ctx->video_step_pending = 0;

    for (int chn = 0; chn < ctx->grid_chn_cnt; chn++) {
        csg_lvgl_win_t *w = &ctx->win[chn];
        if (w->hidden) continue;
        if (chn == 0 && paused && !step) continue;  /* 单路暂停保持最后一帧 */
        csg_lvgl_disp_win_present(w);
    }
}

static void *csg_lvgl_disp_tick_thread(void *arg)
{
    (void)arg;
    while (s_csg_lvgl_disp_ctx && s_csg_lvgl_disp_ctx->running) {
        usleep(5000);
        lv_tick_inc(5);
        /* 整个渲染过程持全局 LVGL 锁: 与 main/解码线程的 control API
         * 互斥, 杜绝 rendering_in_progress 断言导致的死循环卡死. */
        csg_lvgl_lock();
        csg_lvgl_disp_video_present(s_csg_lvgl_disp_ctx);   /* 视频帧上屏必须在 LVGL 线程做 */
        lv_timer_handler();
        csg_lvgl_unlock();
    }
    return NULL;
}

int csg_lvgl_display_init(int screen_w, int screen_h)
{
    if (s_csg_lvgl_disp_ctx) {
        CSG_LOGW("LVGL_DISP", "Already initialized");
        return 0;
    }

    csg_lvgl_disp_ctx_t *ctx = calloc(1, sizeof(csg_lvgl_disp_ctx_t));
    if (!ctx) return -1;

    s_csg_lvgl_disp_ctx = ctx;
    ctx->screen_w = screen_w;
    ctx->screen_h = screen_h;
    ctx->grid_chn_cnt = 1;   /* 默认单路 */
    for (int i = 0; i < CSG_LVGL_MAX_VIDEO_CHN; i++) {
        ctx->win[i].pending_idx = -1;
        ctx->win[i].back_idx = 0;
        ctx->win[i].rx = -1;  /* 默认全屏居中路径 */
        pthread_mutex_init(&ctx->win[i].buf_mutex, NULL);
    }

    /* 初始化全局 LVGL 递归互斥锁 (递归: LVGL 回调内可重入) */
    pthread_mutexattr_t lvgl_mtx_attr;
    pthread_mutexattr_init(&lvgl_mtx_attr);
    pthread_mutexattr_settype(&lvgl_mtx_attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&s_lvgl_mutex, &lvgl_mtx_attr);
    pthread_mutexattr_destroy(&lvgl_mtx_attr);

    /* 初始化 LVGL */
    lv_init();

    /* 创建 DRM 显示 */
    ctx->display = lv_linux_drm_create();
    if (!ctx->display) {
        CSG_LOGE("LVGL_DISP", "lv_linux_drm_create failed");
        for (int i = 0; i < CSG_LVGL_MAX_VIDEO_CHN; i++)
            pthread_mutex_destroy(&ctx->win[i].buf_mutex);
        pthread_mutex_destroy(&s_lvgl_mutex);
        free(ctx);
        s_csg_lvgl_disp_ctx = NULL;
        return -1;
    }

    /* 设置 DRM 设备路径 */
    lv_result_t ret = lv_linux_drm_set_file(ctx->display, "/dev/dri/card0", -1);
    if (ret != LV_RESULT_OK) {
        CSG_LOGE("LVGL_DISP", "lv_linux_drm_set_file failed");
        lv_display_delete(ctx->display);
        for (int i = 0; i < CSG_LVGL_MAX_VIDEO_CHN; i++)
            pthread_mutex_destroy(&ctx->win[i].buf_mutex);
        pthread_mutex_destroy(&s_lvgl_mutex);
        free(ctx);
        s_csg_lvgl_disp_ctx = NULL;
        return -1;
    }

    ctx->screen_w = lv_display_get_horizontal_resolution(ctx->display);
    ctx->screen_h = lv_display_get_vertical_resolution(ctx->display);

    CSG_LOGI("LVGL_DISP", "DRM display: %dx%d", ctx->screen_w, ctx->screen_h);

    lv_display_set_user_data(ctx->display, ctx);

    /* 注意: 绝不能在这里调用 lv_display_set_buffers() 覆盖显示 buffer!
     * lv_linux_drm_set_file() 已把两个 DRM dumb buffer 注册为显示 buffer
     * (LV_DISPLAY_RENDER_MODE_DIRECT), 驱动在 LV_EVENT_REFR_START 时按
     * 指针匹配当前活跃 buffer。换成自己的内存后匹配永远失败,
     * drm_flush 断言 act_buf != NULL (lv_linux_drm.c:1072) 导致卡死。
     * LVGL 会直接渲染进 DRM buffer, 这里什么都不用做。
     */

    /* 创建默认主题 */
    lv_theme_t *theme = lv_theme_default_init(ctx->display,
                                              lv_color_hex(0x1E1E2E),
                                              lv_color_hex(0xCDD6F4),
                                              LV_THEME_DEFAULT_DARK,
                                              &lv_font_montserrat_14);
    lv_display_set_theme(ctx->display, theme);

    /* 创建两个屏幕: 主屏幕(控制)和视频屏幕 */
    ctx->main_scr = lv_obj_create(NULL);
    ctx->video_scr = lv_obj_create(NULL);

    /* ★ 禁用主屏幕滚动: 视频画面应铺满屏幕, 不允许拖动/滚动条出现 */
    lv_obj_set_scrollable(ctx->main_scr, false);
    lv_obj_set_style_pad_all(ctx->main_scr, 0, 0);

    /* 设置屏幕背景 */
    lv_obj_set_style_bg_color(ctx->main_scr, lv_color_hex(0x1E1E2E), 0);
    lv_obj_set_style_bg_color(ctx->video_scr, lv_color_hex(0x000000), 0);

    lv_screen_load(ctx->main_scr);

    /* 创建视频图像控件: 先创建、垫底, 控制面板创建在其上层;
     * 未出第一帧前隐藏, 避免渲染空数据。
     * 注意: 这里不调用 lv_image_set_src (描述符还是空的, 会告警),
     * 第一帧到达时由 csg_lvgl_disp_win_present() 设置。
     * 32 个窗口一次创建 (覆盖 32 路分屏), 未使用的隐藏. */
    for (int i = 0; i < CSG_LVGL_MAX_VIDEO_CHN; i++) {
        csg_lvgl_win_t *w = &ctx->win[i];
        w->img = lv_image_create(ctx->main_scr);
        w->dsc.header.cf = LV_COLOR_FORMAT_XRGB8888;
        /* ★ 固定尺寸为屏幕大小 + 清零 padding, 防止图像内容撑大控件导致滚动 */
        lv_obj_set_size(w->img, ctx->screen_w, ctx->screen_h);
        lv_obj_set_style_pad_all(w->img, 0, 0);
        lv_obj_set_align(w->img, LV_ALIGN_CENTER);
        lv_obj_set_hidden(w->img, true);
        /* ★ 图像控件内容裁剪模式: 超出部分裁剪而非撑大控件 */
        lv_obj_set_style_clip_corner(w->img, true, 0);
        w->hidden = 1;
    }

    /* 初始化触摸输入 (自动扫描 /dev/input/event*) */
    if (csg_evdev_touch_init(ctx->screen_w, ctx->screen_h) != 0) {
        CSG_LOGW("LVGL_DISP", "Touch init failed");
    }

    /* 初始化鼠标输入 (独立指针设备, 与触屏共存, 无鼠标则跳过) */
    if (csg_evdev_mouse_init(ctx->screen_w, ctx->screen_h) != 0) {
        CSG_LOGW("LVGL_DISP", "Mouse init failed");
    }

    /* 启动 tick 线程 */
    ctx->running = 1;
    pthread_create(&ctx->tick_thread, NULL, csg_lvgl_disp_tick_thread, NULL);

    ctx->ready = 1;
    CSG_LOGI("LVGL_DISP", "Initialized successfully");
    return 0;
}

void csg_lvgl_display_set_paused(int paused)
{
    if (!s_csg_lvgl_disp_ctx || !s_csg_lvgl_disp_ctx->ready) return;

    csg_lvgl_disp_ctx_t *ctx = s_csg_lvgl_disp_ctx;
    pthread_mutex_lock(&ctx->win[0].buf_mutex);
    ctx->video_paused = paused;
    pthread_mutex_unlock(&ctx->win[0].buf_mutex);
}

/* 逐帧步进: 暂停状态下放行下一帧上屏 (由解码线程在送入步进帧前调用) */
void csg_lvgl_display_step_frame(void)
{
    if (!s_csg_lvgl_disp_ctx || !s_csg_lvgl_disp_ctx->ready) return;

    csg_lvgl_disp_ctx_t *ctx = s_csg_lvgl_disp_ctx;
    pthread_mutex_lock(&ctx->win[0].buf_mutex);
    ctx->video_step_pending = 1;
    pthread_mutex_unlock(&ctx->win[0].buf_mutex);
}

/* 重置帧计数器 (用于新播放开始时) */
void csg_lvgl_display_reset_counter(void)
{
    if (!s_csg_lvgl_disp_ctx) return;
    s_csg_lvgl_disp_ctx->win[0].frames = 0;
}

/* ==================== 多路分屏网格 ====================
 * NVR 标准布局: 1/2/4/6/9/16/18/32 路 (cols x rows) */
static const int s_csg_grid_supported[] = { 1, 2, 4, 6, 9, 16, 18, 32 };
static const int s_csg_grid_cols[]       = { 1, 2, 2, 3, 3,  4,  6,  8 };
static const int s_csg_grid_rows[]       = { 1, 1, 2, 2, 3,  4,  3,  4 };
#define CSG_GRID_COUNT (int)(sizeof(s_csg_grid_supported) / sizeof(s_csg_grid_supported[0]))

int csg_lvgl_display_set_grid(int chn_cnt)
{
    if (!s_csg_lvgl_disp_ctx || !s_csg_lvgl_disp_ctx->ready) return 0;
    if (chn_cnt < 1) chn_cnt = 1;
    if (chn_cnt > CSG_LVGL_MAX_VIDEO_CHN) chn_cnt = CSG_LVGL_MAX_VIDEO_CHN;

    csg_lvgl_disp_ctx_t *ctx = s_csg_lvgl_disp_ctx;

    /* 找到第一个 >= chn_cnt 的布局 */
    int cols = 1, rows = 1;
    for (int i = 0; i < CSG_GRID_COUNT; i++) {
        if (s_csg_grid_supported[i] >= chn_cnt) {
            cols = s_csg_grid_cols[i];
            rows = s_csg_grid_rows[i];
            break;
        }
    }
    if (chn_cnt == 1) { cols = 1; rows = 1; }

    ctx->grid_chn_cnt = chn_cnt;
    ctx->grid_cols = cols;
    ctx->grid_rows = rows;

    CSG_LOGI("LVGL_DISP", "grid: %d chn -> %dx%d", chn_cnt, cols, rows);

    /* 布局窗口矩形 (2px 分屏间距, 黑缝) */
    int gap = 2;
    int cw = ctx->screen_w / cols;
    int ch = ctx->screen_h / rows;
    if (cw < 2) cw = 2;
    if (ch < 2) ch = 2;

    csg_lvgl_lock();
    /* 单路: 全屏居中路径 (rx=-1 触发 present 的居中逻辑) */
    if (chn_cnt == 1) {
        ctx->win[0].rx = -1;
        ctx->win[0].ry = -1;
        ctx->win[0].rw = 0;
        ctx->win[0].rh = 0;
        ctx->win[0].hidden = 0;
        lv_obj_set_hidden(ctx->win[0].img, true);
        for (int i = 1; i < CSG_LVGL_MAX_VIDEO_CHN; i++) {
            ctx->win[i].hidden = 1;
            lv_obj_set_hidden(ctx->win[i].img, true);
        }
        csg_lvgl_unlock();
        return chn_cnt;
    }

    /* 多路: 分屏矩形, 未使用格子隐藏 */
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            int idx = r * cols + c;
            if (idx >= CSG_LVGL_MAX_VIDEO_CHN) continue;
            csg_lvgl_win_t *w = &ctx->win[idx];
            if (idx < chn_cnt) {
                w->rx = c * cw + gap / 2;
                w->ry = r * ch + gap / 2;
                w->rw = cw - gap;
                w->rh = ch - gap;
                w->hidden = 0;
                lv_obj_set_pos(w->img, w->rx, w->ry);
                lv_obj_set_size(w->img, w->rw, w->rh);
                lv_obj_set_hidden(w->img, true);  /* 出帧后由 present 显示 */
            } else {
                w->hidden = 1;
                lv_obj_set_hidden(w->img, true);
            }
        }
    }
    csg_lvgl_unlock();
    return chn_cnt;
}

void csg_lvgl_display_update_video_chn(int chn, void *rgb_data,
                                       int width, int height)
{
    if (!s_csg_lvgl_disp_ctx || !s_csg_lvgl_disp_ctx->ready) return;
    if (chn < 0 || chn >= CSG_LVGL_MAX_VIDEO_CHN) return;
    if (!rgb_data || width <= 0 || height <= 0) return;

    csg_lvgl_disp_ctx_t *ctx = s_csg_lvgl_disp_ctx;
    csg_lvgl_win_t *w = &ctx->win[chn];
    int needed_size = width * height * 4;

    pthread_mutex_lock(&w->buf_mutex);

    int idx = w->back_idx;
    if (w->buf_size[idx] < needed_size) {
        free(w->buf[idx]);
        w->buf[idx] = NULL;
    }
    if (!w->buf[idx]) {
        w->buf[idx] = malloc(needed_size);
        w->buf_size[idx] = needed_size;
        if (!w->buf[idx]) {
            pthread_mutex_unlock(&w->buf_mutex);
            return;
        }
    }

    memcpy(w->buf[idx], rgb_data, (size_t)needed_size);
    w->pending_idx = idx;
    w->back_idx = 1 - idx;
    w->video_w = width;
    w->video_h = height;
    w->dirty = 1;

    /* ★ 新帧到达即解除隐藏: 有帧的通道必然是"活跃应显示"的.
     * 否则本地播放路径 (LOCAL_SELECT 不调 set_grid) 时 win[0].hidden
     * 仍为初始化时的 1, tick 线程 present 用 if(hidden) continue 跳过,
     * 导致屏幕全黑只有声音. 多路未用格子无帧写入, 始终保持 hidden. */
    w->hidden = 0;

    pthread_mutex_unlock(&w->buf_mutex);
}

void csg_lvgl_display_update_video(void *rgb_data, int width, int height)
{
    /* 单路 API 兼容: 映射到 chn=0 */
    csg_lvgl_display_update_video_chn(0, rgb_data, width, height);
}

void csg_lvgl_display_get_chn_size(int chn, int *w, int *h)
{
    if (w) *w = 0;
    if (h) *h = 0;
    if (!s_csg_lvgl_disp_ctx) return;

    csg_lvgl_disp_ctx_t *ctx = s_csg_lvgl_disp_ctx;
    if (chn == 0 && ctx->grid_chn_cnt <= 1) {
        /* 单路: 全屏尺寸, 调用方自行保持宽高比 */
        if (w) *w = ctx->screen_w;
        if (h) *h = ctx->screen_h;
        return;
    }
    if (chn < 0 || chn >= ctx->grid_chn_cnt) return;
    if (w) *w = ctx->win[chn].rw;
    if (h) *h = ctx->win[chn].rh;
}

void csg_lvgl_display_reset_channels(void)
{
    if (!s_csg_lvgl_disp_ctx) return;

    csg_lvgl_disp_ctx_t *ctx = s_csg_lvgl_disp_ctx;
    csg_lvgl_lock();
    for (int i = 0; i < CSG_LVGL_MAX_VIDEO_CHN; i++) {
        csg_lvgl_win_t *w = &ctx->win[i];
        pthread_mutex_lock(&w->buf_mutex);
        w->dirty = 0;
        w->pending_idx = -1;
        pthread_mutex_unlock(&w->buf_mutex);
        if (w->img) lv_obj_set_hidden(w->img, true);
    }
    csg_lvgl_unlock();
}

int csg_lvgl_display_get_grid_chn_cnt(void)
{
    return s_csg_lvgl_disp_ctx ? s_csg_lvgl_disp_ctx->grid_chn_cnt : 0;
}

void csg_lvgl_display_get_video_area(int *x, int *y, int *w, int *h)
{
    if (s_csg_lvgl_disp_ctx) {
        *x = 0;
        *y = 0;
        *w = s_csg_lvgl_disp_ctx->screen_w;
        *h = s_csg_lvgl_disp_ctx->screen_h;
    }
}

void csg_lvgl_display_get_screen_size(int *w, int *h)
{
    if (w) *w = s_csg_lvgl_disp_ctx ? s_csg_lvgl_disp_ctx->screen_w : 1920;
    if (h) *h = s_csg_lvgl_disp_ctx ? s_csg_lvgl_disp_ctx->screen_h : 1080;
}

void csg_lvgl_display_tick(void)
{
    /* tick 由独立线程处理 */
}

void csg_lvgl_display_deinit(void)
{
    if (!s_csg_lvgl_disp_ctx) return;

    csg_lvgl_disp_ctx_t *ctx = s_csg_lvgl_disp_ctx;
    ctx->running = 0;
    ctx->ready = 0;

    pthread_join(ctx->tick_thread, NULL);

    csg_evdev_touch_deinit();
    csg_evdev_mouse_deinit();
    for (int i = 0; i < CSG_LVGL_MAX_VIDEO_CHN; i++) {
        if (ctx->win[i].buf[0]) free(ctx->win[i].buf[0]);
        if (ctx->win[i].buf[1]) free(ctx->win[i].buf[1]);
        pthread_mutex_destroy(&ctx->win[i].buf_mutex);
    }

    if (ctx->display) {
        lv_display_set_default(NULL);
        lv_display_delete(ctx->display);
    }

    lv_deinit();
    pthread_mutex_destroy(&s_lvgl_mutex);
    free(ctx);
    s_csg_lvgl_disp_ctx = NULL;
    CSG_LOGI("LVGL_DISP", "Deinitialized");
}

int csg_lvgl_display_is_ready(void)
{
    return s_csg_lvgl_disp_ctx && s_csg_lvgl_disp_ctx->ready;
}