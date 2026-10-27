/* SPDX-License-Identifier: MIT */
/*
 * av_sync.c - 音视频同步实现
 *
 * 参考:
 *   VLC clock.c  - 主时钟线性函数: system = ts * coeff / rate + offset
 *   XBMC VideoPlayerAudio.cpp - SYNC_DISCON/SYNC_RESAMPLE
 *   MPV adjust_sync / calc_best_speed - 渐进式 delay 调整
 *
 * 核心算法:
 *   音频时钟 = 基准 PTS + (已播放采样数 / 采样率) * 速度因子
 *   video_judge = 比较 (视频PTS - 音频时钟) 与阈值
 */

#include "av_sync.h"
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

#define LOG_TAG "AV_SYNC"
#include "common.h"

/* 默认同步阈值: 30ms
 * ★ 调大原因: 音频时钟 base_pts=音频首帧PTS 而视频首帧PTS=0,
 * 容器内音视频起点天然偏差 + ALSA 缓冲延迟导致音频时钟系统性超前,
 * 10ms 阈值过严会让正常播放的视频帧全部误判 DROP → 卡顿.
 * 30ms 配合 av_offset 首帧补偿后, 容忍解码/渲染正常抖动. */
#define DEFAULT_SYNC_THRESHOLD_US  30000

/* 最大连续丢帧数 */
#define DEFAULT_MAX_DROP           5

/* ★ rkadk_player.c 借鉴: DROP 降级为"异常兜底"阈值 (100ms, 随倍速放大)
 * 旧阈值 (30ms) 对音频时钟瞬时抖动过于敏感, 正常播放偏差被误判 DROP → 卡顿.
 * 借鉴 rkadk_player.c SendVideoData 的"永不丢帧"思想:
 *   常规 ±100ms 内偏差一律走 WAIT (延后显示), 不丢帧不卡顿;
 *   仅当 frame_timer 严重落后 now (解码卡死 / Seek 残留旧帧) 才 DROP. */
#define DROP_FALLBACK_US           100000

/* ★ 起播偏移锁定帧数: 前 N 帧用 EMA 平滑 av_offset 后再永久锁定.
 *
 * 历史踩坑 (v3.0): 20 帧锁定窗口过大, 启动阶段视频解码器快速输出
 * (远超实时), 视频 PTS 推进速度 >> 音频时钟推进速度, 导致前 20 帧
 * EMA 收敛到异常大的偏移值 (实测 1~2 秒), 锁定后稳态漂移变小,
 * 但 av_offset 不变 → 过度补偿 → 大量帧被误判 DROP/WAIT → 卡顿.
 *
 * 修复 (v4.0): 缩短到 5 帧, 增大新样本权重 (α=0.5),
 * 快速收敛到稳态值. 锁定后若持续检测到漂移 > 200ms,
 * 触发重新校准 (单点快照), 防止过度补偿累积. */
#define AV_OFFSET_LOCK_FRAMES       5

/* av_offset 重新校准阈值: 锁定后若连续 3 次漂移 > 此值, 触发重锁 */
#define AV_OFFSET_RECAL_THRESHOLD_US  200000
#define AV_OFFSET_RECAL_COUNT         3

/* ★ v5.5 av_offset 最大可锁定幅度: 音画时间轴起点差异只可能是容器起点
 * (几十~一两百 ms) + 音频发声前不可避免的硬件缓冲. 若 diff 超过此值,
 * 说明音频主时钟或视频节奏已偏离 (如解码超前 backlog), 此时把大 diff 锁成
 * av_offset 只会让"错误"成为常量 (历史 v3.0 锁过 1~2s). 超限则不清偿,
 * 视频帧直接向主时钟 PTS→墙上映射对齐 (不引入偏移). */
#define AV_OFFSET_MAX_US              400000

/* ★ v5.6 音画时间轴平移补偿总开关.
 *
 * 决策背景 (v5.5 后验证): 音频主时钟 base_pts 已锚定到"真正写入 ALSA 的
 * 首帧音频 PTS"(out_pts), 主时钟即音频媒体时间轴; 视频帧 PTS 同为媒体
 * 时间轴, 两者天然对齐. 此时 av_offset 的唯一作用是吸收起播阶段"视频解码
 * 超前"的瞬态 (MPP 快速出帧而音频未追平, 实测首锁 ~+100ms):
 *   - 该超前本来就会被 judge 的 WAIT 渐进收敛 (每次最多等 2×frame_duration,
 *     画面消费一帧 71ms 媒体 / 等待期间音频推进 >71ms → 队列领先逐步耗尽),
 *     是纯瞬态, 不该被锁成常量;
 *   - 一旦锁成常量, 每次都在 audible==pts-offset 时才显示 → 画面恒定落后
 *     声音 offset → 唇音误差.
 * 参考 rk3572_player_cp (同步已验证良好) 也无任何 av_offset 平移, 纯
 * PTS→音频时钟映射 + 有界 WAIT.
 *
 * 置 0 = 完全禁用 av_offset (EMA/锁定/重校准全跳过, eff_pts=pts);
 * 置 1 = 恢复 v5.5 逻辑 (仅当确实存在容器起点差且主时钟非媒体轴时再开). */
#define AV_OFFSET_COMPENSATION        0

/* ★ MPV 风格渐进式 delay 上限: 单次等待量不超过 frame_duration 的倍数.
 * MPV adjust_sync 用: max_change = frame_time * factor (factor=0.1~0.4).
 * 这里用更激进的 factor=2 以快速收敛 (嵌入式场景容忍度更高). */
#define MAX_SLEEP_FRAME_DURATION_MULT  2

/* 音频时钟插值: 保留最近一次 ALSA 播放位置采样,
 * 通过系统时钟差值 + 已播放采样数推算当前音频时钟 */
typedef struct {
    int64_t  pts;            /* 写入缓冲区时的帧 PTS (微秒) */
    int64_t  system_time;    /* 采样时的系统时间 (微秒) */
    int64_t  played_samples; /* 当时的已播放采样数 */
    int      rate;           /* 采样率 */
    int      valid;          /* 1=有效 */
} csg_av_sync_clock_sample_t;

struct csg_av_sync_ctx {
    csg_av_clock_master_t master;      /* 主时钟类型 */
    int64_t   threshold;           /* 同步阈值 (微秒) */
    int       max_drop;            /* 最大连续丢帧数 */

    /* 音频时钟 */
    csg_av_sync_clock_sample_t audio_sample; /* 最近一次采样 */
    int64_t  audio_clock;          /* 当前音频时钟 (微秒) */

    /* 视频时钟 */
    int64_t  video_clock;          /* 最近显示的视频帧 PTS */
    int       video_clock_valid;

    /* 状态 */
    int       paused;              /* 1=暂停 */
    int64_t   pause_system_time;   /* 暂停时的系统时间 */
    int64_t   pause_audio_clock;   /* 暂停时的音频时钟 */
    float     speed;               /* 播放速度 (0.5/0.75/1.0/1.5/2.0) */

    /* 连续丢帧计数 */
    int       consecutive_drops;

    /* EMA 平滑滤波: 指数移动平均, 过滤 PTS 单帧抖动 */
    double    diff_ema;           /* 平滑后的 diff */
    double    ema_alpha;          /* 平滑系数 (默认 0.3) */
    int       diff_ema_valid;     /* EMA 是否已初始化 */

    /* ★ frame_timer 节奏控制器: 确保视频帧按音频时钟节奏等间隔显示
     *   核心思路: 每帧的显示时刻 = 上一帧显示时刻 + frame_duration + diff修正
     *   这样即使解码器超产帧, 显示节奏也被音频时钟牢牢锁住. */
    int64_t   frame_timer;        /* 预期下一帧显示时刻 (系统单调时间, 微秒) */
    int64_t   frame_duration;     /* 默认帧时长 (微秒, = 1000000/fps) */
    int       frame_timer_valid;  /* frame_timer 是否已初始化 */
    int64_t   last_displayed_pts; /* 上一帧 PTS (用于计算实际帧间隔) */

    /* ★ 音频时钟 LPF 平滑: 防止 ALSA 硬件延迟查询的微抖动
     *   对 csg_av_sync_set_audio_clock 传入的时钟值做一阶低通滤波,
     *   α=0.9 兼顾快速跟踪 + 轻量平滑, 滞后 < 2ms. */
    int64_t   audio_clock_ema;       /* 平滑后的音频时钟 */
    int       audio_clock_ema_valid; /* EMA 是否已初始化 */

    /* ★ 音视频时间轴偏移补偿 (av_offset)
     * 问题: 音频时钟 base_pts=音频首帧PTS (如 128000us), 视频首帧PTS=0,
     * 容器内音视频起点天然不一致 + ALSA 缓冲延迟 → 音频时钟系统性超前,
     * 10ms 阈值下正常视频帧全部误判 DROP → 卡顿.
     *
     * 解决: 首个真实视频帧 judge 时记录
     *   av_offset = 视频PTS - 音频时钟
     * 之后所有视频帧用 effective_pts = PTS - av_offset 与音频时钟比较,
     * 首帧 diff=0 立即显示, 后续帧按"相对 PTS 差"精确对齐音频时间轴. */
    int64_t   av_offset;       /* 视频时间轴相对音频时钟的平移量 (微秒) */
    int       av_offset_valid; /* 是否已建立 */
    int       av_offset_ema_init; /* EMA 补偿是否已建立 (锁定前也用) */
    int       av_offset_cnt;      /* 已累计帧数, 达到 AV_OFFSET_LOCK_FRAMES 后锁定 */

    /* ★ av_offset 重新校准 (v4.0): 锁定后若持续检测到漂移超过阈值,
     * 触发重新校准, 防止锁定时的异常值在稳态下过度补偿. */
    int       av_offset_recal_cnt; /* 连续漂移超限帧数 */
    int       av_offset_need_recal; /* 需要重新校准 */

    /* ★ 音频时钟稳定性检查: 前 0.5 秒不计算 av_offset,
     * 防止启动阶段视频解码器快速输出导致的虚假大偏移.
     * v5.0: 从 2 秒缩短到 0.5 秒, 配合未稳定期 frame_duration 节流,
     * 避免视频帧跑飞导致 av_offset 锁定异常大值 → 开头卡顿. */
    int64_t   audio_clock_first_valid_us; /* 音频时钟首次有效时系统时间 */
    int       audio_clock_stable; /* 音频时钟已稳定运行超过 0.5 秒 */

    /* ★ rkadk_player.c 借鉴: 帧间隔节流下限
     * last_display_wall: 上次实际显示帧的墙上时刻 (CLOCK_MONOTONIC us).
     * 帧间隔下限 = last_display_wall + frame_duration (1/fps),
     * 保证相邻显示帧间隔 ≥ 理论帧间隔, 防止解码超产时
     * 多帧瞬间冲出 (爆帧抖动) — 等价于 rkadk 的 frameTime 节流. */
    int64_t   last_display_wall;

    /* ★ rkadk_player.c 借鉴: 等待量防递增 (costtime 思想)
     * 连续 WAIT 时本次等待量不超过上次, 防止音频时钟暂时停摆
     * (如 ALSA 欠载恢复) 时视频无限等待导致卡死. */
    int64_t   last_wait_us;
    int       last_wait_valid;
};

/* 系统时间 (微秒, 单调时钟) */
static int64_t csg_av_sync_sys_now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

csg_av_sync_ctx_t *csg_av_sync_create(void)
{
    csg_av_sync_ctx_t *ctx = calloc(1, sizeof(csg_av_sync_ctx_t));
    if (!ctx) return NULL;

    ctx->master = CSG_AV_CLOCK_MASTER_AUDIO;
    ctx->threshold = DEFAULT_SYNC_THRESHOLD_US;
    ctx->max_drop = DEFAULT_MAX_DROP;
    ctx->speed = 1.0f;
    ctx->audio_sample.valid = 0;
    ctx->video_clock_valid = 0;
    ctx->ema_alpha = 0.85;       /* EMA 平滑系数: 0.85 快速响应, 减少滞后 */
    ctx->diff_ema_valid = 0;

    CSG_LOGI("AV_SYNC", "Created (threshold=%lldus, max_drop=%d)",
         (long long)ctx->threshold, ctx->max_drop);
    return ctx;
}

void csg_av_sync_destroy(csg_av_sync_ctx_t *ctx)
{
    if (!ctx) return;
    CSG_LOGI("AV_SYNC", "Destroyed");
    free(ctx);
}

void csg_av_sync_set_master_clock(csg_av_sync_ctx_t *ctx, csg_av_clock_master_t master)
{
    if (!ctx) return;
    ctx->master = master;
    CSG_LOGI("AV_SYNC", "Master clock: %s",
         master == CSG_AV_CLOCK_MASTER_AUDIO  ? "AUDIO"  :
         master == CSG_AV_CLOCK_MASTER_SYSTEM ? "SYSTEM" :
         master == CSG_AV_CLOCK_MASTER_VIDEO  ? "VIDEO"  : "UNKNOWN");
}

void csg_av_sync_set_threshold(csg_av_sync_ctx_t *ctx, int64_t threshold_us)
{
    if (!ctx) return;
    ctx->threshold = threshold_us;
}

void csg_av_sync_set_max_drop(csg_av_sync_ctx_t *ctx, int max_drop)
{
    if (!ctx) return;
    ctx->max_drop = max_drop;
}

void csg_av_sync_update_audio_clock(csg_av_sync_ctx_t *ctx, int64_t pts,
                                int samples, int rate)
{
    (void)samples;  /* 由 csg_av_sync_update_audio_played 使用 */
    if (!ctx || !ctx->paused) {
        /* 正常更新 */
        if (ctx) {
            ctx->audio_sample.pts = pts;
            ctx->audio_sample.system_time = csg_av_sync_sys_now_us();
            ctx->audio_sample.played_samples = 0;  /* 由 audio_render 更新 */
            ctx->audio_sample.rate = rate;
            ctx->audio_sample.valid = 1;
            ctx->audio_clock = pts;
        }
    }
}

/* 由 audio_render 调用, 更新已播放采样数 */
void csg_av_sync_update_audio_played(csg_av_sync_ctx_t *ctx, int64_t played_samples)
{
    if (!ctx) return;
    if (ctx->audio_sample.valid && ctx->audio_sample.rate > 0) {
        ctx->audio_sample.played_samples = played_samples;
        /* 音频时钟 = 基准PTS + (已播放采样数 / 采样率) * 1000000 * speed */
        int64_t elapsed = (int64_t)((double)played_samples
                                    / ctx->audio_sample.rate * 1000000.0
                                    * ctx->speed);
        ctx->audio_clock = ctx->audio_sample.pts + elapsed;
    }
}

/* 直接设置音频时钟 (微秒)
 *
 * ★ 不使用 LPF 平滑!
 * 音频时钟已通过墙上时钟插值 (audio_render_get_clock) 实现精确计算,
 * 公式: clock = base_pts + (now - base_wall_us) * speed
 * 该值基于 CLOCK_MONOTONIC, 单调递增且无抖动, 不需要 EMA 滤波.
 *
 * 旧方案使用 EMA (α=0.8) 导致:
 *   - 稳态滞后 16.5ms (15fps 下)
 *   - 瞬态滞后 60ms+ (初始收敛阶段)
 *   - 滞后直接累加到 drift, 导致 drift 始终 +60~102ms 无法收敛
 *
 * 新方案: 直接使用原始值, 零滞后, 同步精度 < 1ms. */
void csg_av_sync_set_audio_clock(csg_av_sync_ctx_t *ctx, int64_t clock_us)
{
    if (!ctx) return;

    /* 记录音频时钟首次有效时的系统时间, 用于判断音频时钟是否已稳定.
     * ★ v5.5 死锁修复: 原条件 !audio_sample.valid — 但解码侧
     * csg_av_sync_update_audio_clock 在渲染时钟>0 之前就已置 valid=1,
     * 导致此处首次收到真实渲染时钟时 first_valid_us 永不记录,
     * audio_clock_stable 永远为 0 → av_offset EMA/锁定 整段不可达,
     * 视频在"未稳定"分支按 frame_duration 自由跑飞, 音画恒差 ~1s.
     * 修复: 只要首次出现正渲染时钟即记录, 与 valid 标志解耦. */
    if (ctx->audio_clock_first_valid_us == 0 && clock_us > 0) {
        ctx->audio_clock_first_valid_us = csg_av_sync_sys_now_us();
    }

    /* 直接使用原始时钟值, 不做 EMA 平滑 */
    ctx->audio_clock = clock_us;
    ctx->audio_sample.valid = 1;

    /* 音频时钟稳定检查: 时钟已运行超过 0.5 秒 → 视为稳定
     * v5.0: 从 2 秒缩短到 0.5 秒, 配合未稳定期 frame_duration 节流,
     * 让 av_offset 在视频帧还没跑飞前就锁定, 避免开头卡顿. */
    if (!ctx->audio_clock_stable && ctx->audio_clock_first_valid_us > 0) {
        if (clock_us > 0 && (csg_av_sync_sys_now_us() - ctx->audio_clock_first_valid_us) > 500000) {
            ctx->audio_clock_stable = 1;
        }
    }
}

void csg_av_sync_update_video_clock(csg_av_sync_ctx_t *ctx, int64_t pts)
{
    if (!ctx) return;
    ctx->video_clock = pts;
    ctx->video_clock_valid = 1;
}

int64_t csg_av_sync_get_master_clock(csg_av_sync_ctx_t *ctx)
{
    if (!ctx) return 0;

    if (ctx->paused) {
        return ctx->pause_audio_clock;
    }

    switch (ctx->master) {
    case CSG_AV_CLOCK_MASTER_AUDIO:
        if (ctx->audio_sample.valid) {
            return ctx->audio_clock;
        }
        /* 音频时钟无效时回退到系统时钟 */
        return csg_av_sync_sys_now_us();
    case CSG_AV_CLOCK_MASTER_SYSTEM:
        return csg_av_sync_sys_now_us();
    case CSG_AV_CLOCK_MASTER_VIDEO:
        return ctx->video_clock_valid ? ctx->video_clock : 0;
    default:
        return 0;
    }
}

int64_t csg_av_sync_get_audio_clock(csg_av_sync_ctx_t *ctx)
{
    if (!ctx) return 0;
    if (ctx->paused) return ctx->pause_audio_clock;
    return ctx->audio_sample.valid ? ctx->audio_clock : 0;
}

csg_av_sync_action_t csg_av_sync_video_judge(csg_av_sync_ctx_t *ctx, int64_t pts,
                                     int64_t *wait_us)
{
    if (!ctx) return CSG_AV_SYNC_ACTION_DISPLAY;
    if (wait_us) *wait_us = 0;

    /* 无音频时钟或使用视频时钟时, 直接显示 */
    if (ctx->master == CSG_AV_CLOCK_MASTER_VIDEO || !ctx->audio_sample.valid) {
        return CSG_AV_SYNC_ACTION_DISPLAY;
    }

    /* ================================================================
     * ★ PTS → 墙上时钟 直接映射算法 (v2.0)
     *
     * 核心原理 (数学推导):
     *   音频时钟: audio_clock = base_pts + (now - base_wall_us) * speed
     *   帧的理想显示时刻: base_wall_us + (pts - base_pts) / speed
     *                     = now + (pts - audio_clock) / speed
     *
     * 为什么旧算法 (frame_timer 累积) 必然失败:
     *   frame_timer += frame_delay 按 PTS 间隔累积,
     *   但音频时钟按墙上时钟 (CLOCK_MONOTONIC) 线性推进,
     *   两者起点不同步 → 累积后永远差一个固定偏移量.
     *
     *   旧 correction = diff/2 公式:
     *     sleep = frame_timer - now + diff/2
     *     但 audio_clock 在 sleep 期间也推进了 sleep 微秒,
     *     实际 drift 变化 = (pts_new - pts_old) - sleep
     *                     = frame_delay - (frame_delay + diff/2)
     *                     = -diff/2  ← 过冲! 正向差变负向差, 振荡发散.
     *
     * 新算法 (PTS 直接映射):
     *   ① 每个帧的显示时间 = 音频时钟追上该帧 PTS 的时刻
     *   ② 不累积, 不修正, 每帧独立计算, 零累积误差
     *   ③ 自动处理 B 帧乱序 / VFR / 倍速 / Seek, 无需特判
     * ================================================================ */
    /* ★ 无条件使用 PTS→墙上时钟映射, 不依赖 frame_duration */
    {
        int64_t now = csg_av_sync_sys_now_us();
        int64_t master_clock = ctx->audio_clock;

        /* ★ v5.0: 音频时钟完全无效 (master_clock<=0) 时的帧节流:
         * 用 frame_duration 按视频自身 PTS 节奏节流, 防止首帧瞬间冲出.
         *
         * ★ v5.5: 删除 "|| !audio_clock_stable" 的自由跑飞分支!
         * 原逻辑: 稳定前视频按 frame_duration 自由显示, 不向音频时钟
         * 对齐 → 渲染时钟首次出现时视频 PTS 已领先 audible ~1s, 该领先
         * 是恒定量, 后面无论如何补偿都表现为"音画整体错位 ~1s".
         * 修复: 只要有音频时钟 (解码侧或渲染侧 >0) 就直接走下方
         * PTS→墙上映射 (av_offset 未锁定时 eff=pts), 让视频帧等待
         * 音频时钟追平, 音画从首帧起就对齐. */
        if (master_clock <= 0) {
            if (ctx->frame_duration > 0 && ctx->last_display_wall > 0) {
                int64_t fd = ctx->frame_duration;
                if (ctx->speed > 1.0f)
                    fd = (int64_t)((double)ctx->frame_duration / ctx->speed);
                int64_t min_display = ctx->last_display_wall + fd;
                if (now < min_display) {
                    if (wait_us) *wait_us = min_display - now;
                    return CSG_AV_SYNC_ACTION_WAIT;
                }
            }
            ctx->last_display_wall = now;
            return CSG_AV_SYNC_ACTION_DISPLAY;
        }

        /* ---- 0. ★ 音视频时间轴偏移补偿 (av_offset) ----
         * 首个真实视频帧 judge 时记录 av_offset = 视频PTS - 音频时钟,
         * 将视频时间轴平移到音频时钟时间轴, 消除容器内音视频起点偏差
         * (音频首帧PTS=128000 vs 视频首帧PTS=0) + ALSA 缓冲延迟,
         * 否则音频时钟系统性超前 → 正常视频帧全部误判 DROP → 卡顿.
         *
         * 仅低倍速 (speed<4) 生效: 高倍速走 main.c 的 last_displayed_pts
         * 锚定策略, 主时钟即视频原始 PTS, 引入平移会破坏帧间隔公式.
         *
         * ★ v4.0 改进:
         *   1. 音频时钟稳定 (>0.5s) 后才开始 EMA 计算, 避免启动阶段
         *      视频解码器快速输出导致虚假大偏移被锁定
         *   2. EMA α=0.5 (旧 0.2), 更快收敛到稳态值
         *   3. 锁定后若持续检测到漂移 > 200ms, 触发重新校准 */
        if (AV_OFFSET_COMPENSATION && ctx->speed < 4.0f) {
            int64_t diff = (int64_t)pts - master_clock;

            /* 锁定后重新校准检查: 用有效漂移 (diff-av_offset) 判断.
             * ★ v5.4 致命修复: 旧代码用原始 diff 判断,
             * 但 av_offset 锁定后原始 diff 仍是 ~1.3s (环形缓冲延迟),
             * 始终 > 200ms 阈值 → 每 3 帧触发重锁 → 死循环.
             * 修复: 用补偿后有效漂移 (diff - av_offset) 判断. */
            if (ctx->av_offset_valid) {
                int64_t eff_drift = diff - ctx->av_offset;
                int64_t check = eff_drift < 0 ? -eff_drift : eff_drift;
                if (check > AV_OFFSET_RECAL_THRESHOLD_US) {
                    ctx->av_offset_recal_cnt++;
                    if (ctx->av_offset_recal_cnt >= AV_OFFSET_RECAL_COUNT) {
                        CSG_LOGI("AV_SYNC", "AV offset recal: "
                             "eff_drift=%+lldus (raw=%+lldus) locked=%+lldus, resetting",
                             (long long)eff_drift, (long long)diff,
                             (long long)ctx->av_offset);
                        ctx->av_offset = diff;
                        ctx->av_offset_valid = 0;
                        ctx->av_offset_ema_init = 0;
                        ctx->av_offset_cnt = 0;
                        ctx->av_offset_recal_cnt = 0;
                        ctx->av_offset_need_recal = 0;
                    }
                } else {
                    ctx->av_offset_recal_cnt = 0;
                }
            }

            /* 未锁定: 等待音频时钟稳定 (>0.5s) 后才开始 EMA 计算 */
            if (!ctx->av_offset_valid) {
                if (ctx->audio_clock_stable) {
                    int64_t abs_diff = diff < 0 ? -diff : diff;
                    if (abs_diff > AV_OFFSET_MAX_US) {
                        /* 偏差超限: 不锁偏移, 重置计数, 后续帧直接以原始
                         * PTS 向主时钟映射对齐 (av_offset 保持无效). */
                        ctx->av_offset_cnt = 0;
                        if (!ctx->av_offset_ema_init) {
                            CSG_LOGI("AV_SYNC", "AV offset skipped: diff=%+lldus "
                                 "> max %dus (keep 0, pure PTS mapping)",
                                 (long long)diff, AV_OFFSET_MAX_US);
                        }
                        ctx->av_offset_ema_init = 0;
                    } else {
                        if (!ctx->av_offset_ema_init) {
                            /* 首样本直接使用, 不做 EMA 平滑 */
                            ctx->av_offset = diff;
                            ctx->av_offset_ema_init = 1;
                        } else {
                            /* EMA α=0.5: 新样本权重 50%, 旧值权重 50%,
                             * 快速收敛到稳态值 (5 帧内即可稳定) */
                            ctx->av_offset = (int64_t)(0.5 * (double)diff
                                                     + 0.5 * (double)ctx->av_offset);
                        }
                        ctx->av_offset_cnt++;
                        if (ctx->av_offset_cnt >= AV_OFFSET_LOCK_FRAMES) {
                            ctx->av_offset_valid = 1;
                            ctx->av_offset_recal_cnt = 0;
                            CSG_LOGI("AV_SYNC", "AV offset locked: %+lldus "
                                 "(EMA α=0.5 over %d frames, "
                                 "video_pts=%lld, audio=%lld, stable=%d)",
                                 (long long)ctx->av_offset, ctx->av_offset_cnt,
                                 (long long)pts, (long long)master_clock,
                                 ctx->audio_clock_stable);
                        }
                    }
                }
            }
        }

        /* 有效 PTS: EMA 已建立 (锁定前即补偿, 避免起播等待/爆帧)
         * 且低倍速时平移, 否则用原始 PTS */
        int64_t eff_pts = ((ctx->av_offset_ema_init || ctx->av_offset_valid)
                           && ctx->speed < 4.0f)
                          ? (int64_t)pts - ctx->av_offset : (int64_t)pts;

        /* ---- 1. ★ PTS → 墙上时钟映射 ----
         * frame_timer = now + (eff_pts - audio_clock) / speed
         *
         * 数学等价: base_wall_us + (eff_pts - base_pts) / speed
         * 但用 now + diff/speed 可以避免存储 base_pts/base_wall_us,
         * 且自动适配 Seek/倍速切换后的新锚点.
         *
         * 倍速: speed=2.0 时, (eff_pts-audio_clock)/2.0 = 帧以 2 倍速显示,
         *       frame_timer 推进速度为 PTS 的一半, 匹配 2x 播放. */
        int64_t frame_timer;
        if (ctx->speed > 0.0f && ctx->speed != 1.0f) {
            frame_timer = now + (int64_t)((double)(eff_pts - master_clock) / ctx->speed);
        } else {
            frame_timer = now + (eff_pts - master_clock);
        }

        /* ---- 1.5 ★ rkadk 借鉴: 帧间隔节流下限 ----
         * 即使按 PTS 映射本帧应显示, 也强制相邻显示帧间隔 ≥ frame_duration.
         * 等价于 rkadk_player.c SendVideoData 的 frameTime=1000000/fps 节流:
         * 防止解码超产 (B 帧提前到 / 多帧同时出) 时瞬间冲出 → 视觉抖动.
         *
         * 仅低倍速生效: ≥4x 走 main.c 的 last_displayed_pts 锚定策略,
         * 帧间隔由关键帧间隔/speed 决定 (远大于 frame_duration), 无需下限. */
        if (ctx->speed < 4.0f && ctx->last_display_wall > 0
            && ctx->frame_duration > 0) {
            /* ★ v5.5: 帧间隔下限按倍速缩放 (2x → frame_duration/2),
             * 否则 2x 下每帧强制隔 frame_duration (71ms) 显示, 视频帧
             * 以 ~1x 速度堆积, 解码队列暴涨 → 抖动/卡顿. */
            int64_t fd = ctx->frame_duration;
            if (ctx->speed > 1.0f)
                fd = (int64_t)((double)ctx->frame_duration / ctx->speed);
            int64_t min_display = ctx->last_display_wall + fd;
            if (now < min_display) {
                if (wait_us) *wait_us = min_display - now;
                return CSG_AV_SYNC_ACTION_WAIT;
            }
        }

        /* ---- 2. ★ rkadk 借鉴: DROP 降级为异常兜底 (100ms 级) ----
         *
         * 旧逻辑 (v2.0): frame_timer < now - threshold (30ms) → DROP
         *   问题: 音频时钟瞬时抖动 / ALSA 缓冲波动可达 ±30ms,
         *   正常播放帧被误判 DROP → 跳帧卡顿.
         *
         * 新逻辑 (v3.0): frame_timer < now - DROP_FALLBACK_US (100ms×speed) → DROP
         *   借鉴 rkadk"永不丢帧"思想, 常规偏差一律走 WAIT (延后显示);
         *   仅解码器严重落后 (解码卡死) 或 Seek 残留旧帧才 DROP 兜底. */
        {
            int64_t drop_fb = (int64_t)DROP_FALLBACK_US;
            if (ctx->speed > 1.0f)
                drop_fb = (int64_t)((double)DROP_FALLBACK_US * ctx->speed);

            if (frame_timer < now - drop_fb) {
                if (ctx->consecutive_drops < ctx->max_drop) {
                    ctx->consecutive_drops++;
                    ctx->last_displayed_pts = pts;
                    /* DROP 也推进节奏基准, 防止恢复后帧间隔下限误判 */
                    ctx->last_display_wall = now;
                    return CSG_AV_SYNC_ACTION_DROP;
                }
                /* 连续丢帧达上限, 强制显示避免卡死 */
                ctx->consecutive_drops = 0;
                ctx->last_displayed_pts = pts;
                ctx->last_display_wall = now;
                return CSG_AV_SYNC_ACTION_DISPLAY;
            }
        }

        ctx->consecutive_drops = 0;

        /* ---- 3. ★ MPV 风格渐进式 delay 调整 (v5.3) ----
         *
         * 旧算法: sleep_us = frame_timer - now (一次性等待全部 drift).
         *   问题: 当 drift 较大时 (如 av_offset 锁定前 200~500ms),
         *   一次性等待全部 drift → 等待期间音频时钟持续推进
         *   → over-shoot (视频从超前变落后) → 下一帧 DROP
         *   → 振荡发散, 收敛极慢.
         *
         * MPV adjust_sync 策略:
         *   change = av_delay * 0.1  (每次只修正 10% 偏差)
         *   max_change = frame_time * factor  (限制单次修正量)
         *   渐进式调整, 避免一次性过度修正导致的振荡.
         *
         * 实现 (v5.3):
         *   ① 计算 sleep_us = frame_timer - now (全量等待)
         *   ② 取 min(sleep_us, frame_duration * 2) 作为渐进上限
         *      (等价于 MPV 的 max_change = frame_time * 0.4)
         *   ③ 保留 200ms 绝对上限防异常值
         *   ④ 死区 3ms: 避免频繁 usleep 开销
         *
         * 效果: 300ms 初始偏差 → 每次修正 ≤ 2×frame_duration
         *   (30fps → 66ms/次, 5 次收敛到 ±30ms 内),
         *   不再出现一次性 over-shoot → 振荡收敛. */
        int64_t sleep_us = frame_timer - now;
        int64_t max_sleep;
        if (ctx->frame_duration > 0) {
            max_sleep = ctx->frame_duration * MAX_SLEEP_FRAME_DURATION_MULT;
            if (max_sleep < 40000) max_sleep = 40000;   /* 下限 40ms: 25fps 场景 */
            if (max_sleep > 200000) max_sleep = 200000;  /* 上限 200ms: 防 B 帧间隔异常 */
        } else {
            max_sleep = 200000;
        }
        if (ctx->speed > 2.0f) {
            max_sleep = 600000;  /* 高倍速 600ms 绝对上限, 覆盖 4x 大 GOP */
        }
        if (sleep_us > max_sleep) sleep_us = max_sleep;
        if (sleep_us > 0 && sleep_us < 3000) sleep_us = 0;

        /* ---- 4. 更新状态, 返回决策 ----
         * ★ B 帧 PTS 乱序保护: 当 PTS 倒退时 (B 帧 decode 顺序),
         *   不更新 last_displayed_pts, 保持单调递增用于日志统计. */
        if (pts > ctx->last_displayed_pts || ctx->last_displayed_pts == 0) {
            ctx->last_displayed_pts = pts;
        }

        /* ---- 4.5 ★ v5.6: 删除 "等待量防递增" 钳制 ----
         * 旧逻辑: 连续 WAIT 时本次等待量 ≤ 上次, 防音频时钟停摆时 wait
         * 无限递增把视频"等死". 但实测致命缺陷: 起播音频暖机期间视频按
         * frame_duration 自由显示, 建立 ~0.6s 的领先; 一旦音频时钟加入,
         * 需要更大的 wait 把视频拖回音频, 而该钳制把 wait 钉死在收敛瞬间
         * 的小值 (实测 19.7ms) → 视频永远无法减速追赶 → 画面恒定超前声音
         * 一个解码队列深度 (几百 ms), 永不收敛.
         *
         * PTS→墙上映射下 wait = eff_pts - audio_clock, 1x 睡眠期间音频
         * 时钟以 1:1 推进, 睡满 wait 恰好追平, 无过冲, 因此每次应允许睡到
         * 所需全量; max_sleep (2×frame_duration) 已给出上界防止异常值把
         * 视频拖死, 音频时钟真正停摆时 wait 会被 max_sleep 封顶 → 视频最多
         * 慢放, 不会卡死. 参考 rk3572_player_cp 亦无此钳制. */
        if (sleep_us > 0) {
            ctx->last_wait_us = sleep_us;
            ctx->last_wait_valid = 1;
            if (wait_us) *wait_us = sleep_us;
            return CSG_AV_SYNC_ACTION_WAIT;
        }

        /* 实际显示: 记录显示时刻作为帧间隔节流下限基准 */
        ctx->last_display_wall = now;
        ctx->last_wait_valid = 0;
        return CSG_AV_SYNC_ACTION_DISPLAY;
    }

}

void csg_av_sync_pause(csg_av_sync_ctx_t *ctx)
{
    if (!ctx || ctx->paused) return;
    ctx->paused = 1;
    ctx->pause_system_time = csg_av_sync_sys_now_us();
    ctx->pause_audio_clock = ctx->audio_clock;
    CSG_LOGI("AV_SYNC", "Paused (audio_clock=%lldus)",
         (long long)ctx->pause_audio_clock);
}

void csg_av_sync_resume(csg_av_sync_ctx_t *ctx)
{
    if (!ctx || !ctx->paused) return;
    ctx->paused = 0;
    CSG_LOGI("AV_SYNC", "Resumed");
}

void csg_av_sync_seek_reset(csg_av_sync_ctx_t *ctx)
{
    if (!ctx) return;
    ctx->audio_sample.valid = 0;
    ctx->audio_clock = 0;
    ctx->audio_clock_ema = 0;
    ctx->audio_clock_ema_valid = 0;
    ctx->video_clock_valid = 0;
    ctx->consecutive_drops = 0;
    ctx->paused = 0;
    ctx->diff_ema = 0.0;
    ctx->diff_ema_valid = 0;
    /* ★ Seek 后必须重置 frame_timer, 否则旧帧的节奏记忆
     * 会导致新位置第一帧等待异常长的时间. */
    ctx->frame_timer_valid = 0;
    ctx->frame_timer = 0;
    ctx->last_displayed_pts = 0;
    ctx->last_display_wall = 0;  /* 帧间隔下限基准复位 */
    ctx->last_wait_valid = 0;
    /* ★ Seek 后必须重置 av_offset 补偿, 否则旧位置与音频时钟的偏移
     * 关系不适用于新位置, 导致 av_offset 过度补偿 → eff_pts 严重偏离
     * 音频时钟 → 帧全部误判 WAIT/DROP. */
    ctx->av_offset_valid = 0;
    ctx->av_offset = 0;
    ctx->av_offset_ema_init = 0;
    ctx->av_offset_cnt = 0;
    CSG_LOGI("AV_SYNC", "Seek reset (frame_timer + av_offset cleared)");
}

void csg_av_sync_set_frame_duration_by_fps(csg_av_sync_ctx_t *ctx, float fps)
{
    if (!ctx || fps <= 0.0f) return;
    ctx->frame_duration = (int64_t)(1000000.0f / fps);
    ctx->frame_timer_valid = 0;  /* 新视频, 重新开始计时 */
    ctx->frame_timer = 0;
    ctx->last_displayed_pts = 0;
    ctx->last_display_wall = 0;  /* 帧间隔下限基准复位 */
    ctx->last_wait_valid = 0;
    CSG_LOGI("AV_SYNC", "Frame duration: %lldus (%.2f fps) → frame_timer ENABLED",
         (long long)ctx->frame_duration, (double)fps);
}

void csg_av_sync_reset_frame_timer(csg_av_sync_ctx_t *ctx)
{
    if (!ctx) return;
    ctx->frame_timer_valid = 0;
    ctx->frame_timer = 0;
    ctx->last_displayed_pts = 0;
    ctx->last_display_wall = 0;
    ctx->last_wait_valid = 0;
    /* ★ 重置 av_offset 重新校准状态, Seek 后需要重新计算偏移 */
    ctx->av_offset = 0;
    ctx->av_offset_valid = 0;
    ctx->av_offset_ema_init = 0;
    ctx->av_offset_cnt = 0;
    ctx->av_offset_recal_cnt = 0;
    ctx->av_offset_need_recal = 0;
    ctx->audio_clock_stable = 0;
    ctx->audio_clock_first_valid_us = 0;
    CSG_LOGI("AV_SYNC", "Frame timer reset");
}

void csg_av_sync_set_speed(csg_av_sync_ctx_t *ctx, float speed)
{
    if (!ctx) return;
    ctx->speed = speed;

    /* ★ 倍速切换时重置 diff_ema: 速度变化后 PTS 与音频时钟的
     * 关系发生根本改变, 旧 EMA 值会误导同步决策.
     * 例如 1x→2x 时旧 EMA=+14ms, 但 2x 下 drift 可能是 +50ms,
     * 残留的 +14ms EMA 会导致前几帧错误地减少等待. */
    ctx->diff_ema_valid = 0;
    ctx->diff_ema = 0.0;

    /* 倍速切换时重置 frame_timer: 速度变化后 frame_delay 已变,
     * 旧 frame_timer 的累积速率不再适用, 重置后首帧重新对齐. */
    ctx->frame_timer_valid = 0;
    ctx->frame_timer = 0;
    ctx->last_displayed_pts = 0;
    ctx->last_display_wall = 0;  /* 节奏基准随速度重置 */
    ctx->last_wait_valid = 0;

    /* ★ DROP 兜底阈值在 judge 内按 speed 动态缩放 (DROP_FALLBACK_US×speed),
     * 此处不再设置 threshold (已不参与判定, 保留字段仅为兼容 set_threshold API). */
    CSG_LOGI("AV_SYNC", "Speed: %.2fx (diff_ema + frame_timer reset)", (double)speed);
}

float csg_av_sync_get_speed(csg_av_sync_ctx_t *ctx)
{
    return ctx ? ctx->speed : 1.0f;
}

void csg_av_sync_stop(csg_av_sync_ctx_t *ctx)
{
    if (!ctx) return;
    ctx->audio_sample.valid = 0;
    ctx->audio_clock = 0;
    ctx->audio_clock_ema = 0;
    ctx->audio_clock_ema_valid = 0;
    ctx->video_clock_valid = 0;
    ctx->consecutive_drops = 0;
    ctx->paused = 0;
    ctx->diff_ema = 0.0;
    ctx->diff_ema_valid = 0;
    ctx->frame_timer_valid = 0;
    ctx->frame_timer = 0;
    ctx->last_displayed_pts = 0;
    ctx->last_display_wall = 0;
    ctx->last_wait_valid = 0;
    ctx->av_offset_valid = 0;
    ctx->av_offset = 0;
    ctx->av_offset_ema_init = 0;
    ctx->av_offset_cnt = 0;
    CSG_LOGI("AV_SYNC", "Stopped");
}

int csg_av_sync_has_audio_clock(csg_av_sync_ctx_t *ctx)
{
    return ctx ? ctx->audio_sample.valid : 0;
}

int csg_av_sync_has_video_clock(csg_av_sync_ctx_t *ctx)
{
    return ctx ? ctx->video_clock_valid : 0;
}