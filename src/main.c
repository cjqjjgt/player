/* SPDX-License-Identifier: MIT */
/*
 * main.c - RK3572 本地视频播放器主入口
 *
 * 数据流:
 *   MP4 文件 → mp4v2 解封装 → MPP 硬件解码 (NV12)
 *   → RGA 硬件色彩转换 (NV12 → RGB888) → LVGL 显示
 *   音频: AAC → faad2 软解 → ALSA 播放
 *
 * 功能:
 *   - 本地视频播放 (MP4/MKV/AVI)
 *   - 音视频同步 (AV_SYNC)
 *   - 播放控制 (暂停/恢复/seek/倍速/逐帧步进)
 *   - LVGL 图形界面 (进度条/控制栏/文件浏览器)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <errno.h>
#include <time.h>

#include "common.h"
#include "mpp_decode.h"
#include "rga_convert.h"
#include "im2d.h"
#include "lvgl_display.h"
#include "lvgl_control.h"
#include "local_playback.h"
#include "audio_decode.h"
#include "audio_render.h"
#include "av_sync.h"
#include "rtsp_live555.h"
#include "mp4_record.h"
#include "nvr_multi.h"
#include "demux_buffer.h"
#include "lvgl.h"

#define LOCAL_VIDEO_DIR  "/userdata/records"

/* 后台预扫描缓存: 启动时后台线程扫描所有视频文件信息,
 * 用户点击"本地视频"时直接使用缓存, 无需等待扫描. */
#define MAX_CACHED_FILES        256
static csg_local_file_info_t    s_csg_cached_files[MAX_CACHED_FILES];
static int                      s_csg_cached_file_count = 0;
static volatile int             s_csg_scan_complete = 0;
static pthread_t                s_csg_scan_thread;
static pthread_mutex_t          s_csg_scan_mutex = PTHREAD_MUTEX_INITIALIZER;

/* ==================== 全局状态 ==================== */
/* 单路主码流录制上下文: 一套独立的 live555 主码流句柄 + MP4v2 录制器.
 * auto_rec 与 manual_rec 各自独立, 互不干扰. */
typedef struct {
    void           *rtsp_handle;   /* live555 主码流句柄 (仅喂本录制器) */
    void           *record_handle; /* MP4v2 录制句柄 */
    int             recording;     /* 1=本路正在录制 */
    int             start_pending; /* 主码流连接后待建录制器标志 */
    char            record_path[512];
} csg_rec_ctx_t;

typedef struct {
    int             running;
    void           *mpp_handle;
    int             video_width;
    int             video_height;
    int             video_fps;
    int             codec_type;
    csg_player_state_t  state;
    volatile int    cmd_in_progress;  /* ★ v5.4 重入保护: 防止 LVGL 线程中
                                         csg_player_on_control_cmd 被重入调用
                                         (如 lv_label_set_text 触发的布局事件
                                         再次触发回调) 导致崩溃 */
    csg_player_stats_t  stats;
    pthread_mutex_t mutex;

    void           *rgb_buf;
    int             rgb_buf_size;

    csg_player_time_t   time_state;
    int             live;
    int             step_frame_pending;

    int             local_handle;
    int             is_local_play;

    /* ==================== RTSP 拉流 (live555) + 双录制 (MP4v2) ==================== */
    void           *rtsp_handle;      /* live555 预览子码流句柄 (喂 MPP 显示) */
    int             rtsp_codec;       /* 预览拉流编码类型 (SDP 确定) */
    int             rtsp_width;
    int             rtsp_height;
    int             rtsp_fps;
    pthread_mutex_t record_mutex;     /* 保护两路录制上下文 */

    csg_rec_ctx_t   auto_rec;         /* ① 拉流即自动录制 (主码流, RTSP_PLAY 时启动) */
    csg_rec_ctx_t   manual_rec;       /* ② 点击 REC 手动录制 (主码流) */

    int             state_gen;
    int             mpp_gen;

    int64_t         last_displayed_pts;
    int             post_resume_sync;
    int64_t         resume_min_pts;
    int             resume_keyframe_seen;
    int64_t         last_keyframe_pts;
    int64_t         last_fed_kf_pts;   /* 高倍速下实际送入 MPP 的上一个关键帧 PTS (关键帧稀疏化) */
    int             mpp_needs_keyframe;
    int             mpp_kf_wait_decode;
    int64_t         seek_target_pts;
    int64_t         audio_seek_pts;     /* Seek 后音频 PTS 钳制基准 */
    unsigned int    seek_drop_count;
    int64_t         expected_kf_pts;
    volatile int    seek_fast_feed_done;

    /* Seek 后音频就绪等待 */
    int             waiting_audio_ready;   /* 1=等待音频时钟推进后解冻 */
    int             audio_wait_frame_count; /* 等待帧计数(超时保护) */
    int64_t         audio_sync_base_pts;   /* Seek 后音频时钟基准 PTS */

    csg_av_sync_ctx_t     *av_sync_handle;
    csg_audio_decode_ctx_t *audio_dec_handle;
    csg_audio_render_ctx_t *audio_render_handle;
    int              has_audio;

    /* 本地播放音频轨参数 (setup_audio_pipeline 时从 local_playback 获取):
     * PCM 轨 (本项目录制格式) 不走 FAAD2, 分发线程直读渲染;
     * AAC 轨走 FAAD2, 输出 PCM 由 on_audio_pcm 统一重采样到 48kHz。 */
    int              local_audio_codec;   /* CSG_AUDIO_CODEC_AAC / _PCM */
    int              local_audio_rate;
    int              local_audio_ch;

    /* RTSP 音频渲染器懒创建 (首帧 G.711 到达时, 采样率/声道确定后才创建):
     * 回调运行在 live555 音频 sink 线程, 用独立互斥锁保护首帧创建,
     * 避免与 RTSP 停止/切换时的销毁竞争. */
    pthread_mutex_t  rtsp_audio_mutex;
    int              rtsp_audio_tried;  /* 1=已尝试创建 (失败则不每帧重试) */

    int              audio_frame_count;
    int              audio_pcm_count;

    /* 关键帧 PTS 环形缓冲区: 用于高倍速输出侧防御.
     * 不依赖 mpp_decode 的 is_keyframe 标志 (该标志因 pending_is_keyframe
     * 全局变量被覆盖而不可靠), 改用 PTS 匹配验证输出帧是否为已知关键帧. */
#define MAIN_KF_RING_SIZE 16

    /* 高倍速关键帧过滤阈值: speed ≥ 4x 时视频只解码/显示 I 关键帧, 丢弃 P/B 帧;
     * speed < 4x (含 2x) 时全部帧正常解码播放.
     * 对应 GUI 倍速档位 {1,2,4,6,8,16}x (见 lvgl_control.c
     * s_csg_lvgl_ctrl_speed_values). 改档位时需同步更新此处. */
#define MAIN_KEYFRAME_ONLY_MIN_SPEED 4.0f

    /* 高倍速关键帧显示目标帧率 (~2fps): 让 4x/6x/8x/16x 统一呈现"明显跳帧"
     * 的顿挫观感. min_gap = speed / 2 (秒), 例: 16x→8s, 8x→4s, 4x→2s.
     * 这样 16x 不再"顺滑快进", 而是和 8x 一样每 ~500ms 顿一下显示一帧. */
#define MAIN_KF_DISPLAY_FPS 2.0f
    int64_t kf_pts_ring[MAIN_KF_RING_SIZE];
    int kf_pts_ring_head;   /* 写入位置 */
    int kf_pts_ring_count;  /* 有效条目数 */

    /* ==================== 解封装→解码 缓冲层 (动态数组双端队列) ====================
     * 解封装线程 (生产者) push 深拷贝帧, 视频/音频分发线程 (消费者) pop 后送
     * MPP/FAAD2 解码. 缓冲层解耦生产与消费, 解封装线程不再被 MPP BUFFER_FULL
     * /ALSA 阻塞, 队列超限自动背压, 防止 OOM. 参考 GStreamer queue element. */
    void           *demux_buf;         /* csg_demux_buffer_t* */
    pthread_t       vdispatch_thread;  /* 视频分发线程 */
    volatile int    vdispatch_run;     /* 1=运行 */
    pthread_t       adispatch_thread;  /* 音频分发线程 */
    volatile int    adispatch_run;     /* 1=运行 */

    /* ==================== 动态帧率计算 ====================
     * 当 video_fps 为 0 (容器未提供 FPS) 时, 从 PTS 间隔推算实际帧率,
     * 用于驱动 frame_timer 节奏控制器, 防止解码超产时帧间隔节流失效. */
    int64_t         fps_prev_pts;      /* 上一帧 PTS, 用于计算帧间隔 */
    int             fps_calc_count;    /* 已计算帧数 */
    int64_t         fps_interval_sum;  /* 帧间隔累加 (用于平均) */
    int             fps_dynamic_set;   /* 1=已动态设置 frame_duration */

    /* ★ v5.9: 常开音频渲染器 — 程序启动时创建一次, 退出时销毁.
     * 避免每次切换文件都经历 snd_pcm_close/open 导致的
     * NAU8822 codec power_off/power_on 循环, 该循环会破坏
     * DAPM 内部 Jack Detect 状态, 导致第二次播放无声. */
    csg_audio_render_ctx_t *audio_render_persistent;
} csg_player_app_ctx_t;

static csg_player_app_ctx_t s_csg_player_app = {0};

static int s_csg_state_gen_counter = 0;

static inline int csg_player_next_state_gen(void) { return ++s_csg_state_gen_counter; }
static inline int csg_player_current_state_gen(void) { return s_csg_state_gen_counter; }

static struct timespec s_csg_player_stats_time;

/* ==================== 后台预扫描线程 ====================
 * 启动时立即扫描 LOCAL_VIDEO_DIR 下所有视频文件 (含子目录),
 * 获取分辨率、时长等信息缓存到 s_csg_cached_files.
 * 主线程在 CSG_CTRL_CMD_LOCAL_BROWSE 时直接使用缓存,
 * 避免用户第一次点击时等待扫描. */
static void *csg_player_scan_thread(void *arg)
{
    (void)arg;
    CSG_LOGI("MAIN", "Background scan started: %s", LOCAL_VIDEO_DIR);

    int count = csg_local_playback_scan_dir_recursive(
        LOCAL_VIDEO_DIR, s_csg_cached_files, MAX_CACHED_FILES);

    pthread_mutex_lock(&s_csg_scan_mutex);
    s_csg_cached_file_count = count;
    s_csg_scan_complete = 1;
    pthread_mutex_unlock(&s_csg_scan_mutex);

    CSG_LOGI("MAIN", "Background scan complete: %d video files found", count);
    return NULL;
}

static int64_t csg_player_now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

/* ==================== 音频统一重采样 (线性插值) + 上混双声道 ====================
 * ★ 历史踩坑: NAU8822 在低采样率 (8k/16k) 下时钟/滤波表现差, 且单声道
 *   只驱动一个声道。所有音频 (RTSP G.711 / 本地 AAC / 本地 PCM) 统一
 *   线性插值重采样到 48kHz (NAU8822 最佳工作频率) + 单声道上混双声道。
 * ★ 增益按源区分: RTSP G.711 弱信号 (±100 本底) 需 ×8;
 *   本地 AAC/PCM 已是满幅度, 增益必须为 1 (否则削波爆音)。 */
#define RTSP_AUDIO_OUT_RATE  48000
#define RTSP_AUDIO_OUT_CH    2
#define RTSP_AUDIO_GAIN      8      /* ★ v5.16: 与参考项目 (2.0sy) 对齐, 恢复为 8.
                                     * G.711 弱信号峰值±100, 需×8才能听到人声. */
#define LOCAL_PLAY_GAIN      8      /* ★ v5.16: 与参考项目 (2.0sy) 对齐, 从 5→8.
                                     * 录制源为 G.711 弱信号 (峰值±100),
                                     * 即使 AAC 编码后解码输出幅度仍很小,
                                     * 需 ×8 增益才能听到人声. 参考项目
                                     * 80% 音量 ×8 增益可正常播放人声. */

/* 线性插值重采样 + 上混双声道 + 增益, 返回输出样本数 (每通道). */
static int csg_player_audio_resample(const int16_t *in, int in_samples,
                                     int in_rate, int in_channels,
                                     int16_t *out, int out_cap, int gain)
{
    int out_rate = RTSP_AUDIO_OUT_RATE;
    int oc = RTSP_AUDIO_OUT_CH;

    if (in_samples <= 0 || in_channels <= 0 || in_rate <= 0) return 0;

    /* 无需重采样且已是双声道: 仅增益 */
    if (in_rate == out_rate && in_channels == oc) {
        int n = 0;
        for (int i = 0; i + 1 < in_samples * oc && n + 2 <= out_cap; i += 2) {
            int l = in[i] * gain;
            int r = in[i + 1] * gain;
            if (l > 32767) l = 32767; else if (l < -32768) l = -32768;
            if (r > 32767) r = 32767; else if (r < -32768) r = -32768;
            out[n++] = (int16_t)l;
            out[n++] = (int16_t)r;
        }
        return n / oc;
    }

    /* 通用: 输出样本 o 对应输入位置 o*in_rate/out_rate, 线性插值.
     * 单声道输入时 s0/s1 取第 1 声道, 输出复制到双声道.
     * 多声道 (>2) 输入同样取第 1 声道 (NVR 场景音频基本单声道). */
    int64_t step = ((int64_t)in_rate << 16) / out_rate;
    int max_out = (int)(((int64_t)in_samples * out_rate) / in_rate);
    int n = 0;
    for (int o = 0; o < max_out && n + 2 <= out_cap; o++) {
        int64_t in_pos = (int64_t)o * step;
        int i0 = (int)(in_pos >> 16);
        int frac = (int)(in_pos & 0xFFFF);
        int i1 = (i0 + 1 < in_samples) ? i0 + 1 : i0;
        int s0 = in[i0 * in_channels];          /* 取第 1 声道 */
        int s1 = in[i1 * in_channels];
        int v = s0 + (int)(((int64_t)(s1 - s0) * frac) >> 16);
        v = v * gain;
        if (v > 32767) v = 32767; else if (v < -32768) v = -32768;
        out[n++] = (int16_t)v;
        out[n++] = (int16_t)v;                  /* 上混右声道 */
    }
    return n / oc;
}

/* ==================== 音频回调链 ====================
 * FAAD2 (AAC) 解码输出 / PCM 轨直读 的统一出口:
 * 原生采样率/声道 → 重采样 48kHz + 上混双声道 (增益=1) → ALSA 渲染器。
 * ★ 渲染器固定 48kHz/2ch (NAU8822 最佳工作频率), 修复本地 AAC 低采样率
 *   (如 16kHz) 直接按原生率打开导致的"本地播放无声"问题。 */
static void csg_player_on_audio_pcm(void *pcm_data, int samples,
                         int channels, int rate,
                         int64_t pts, void *user_data)
{
    csg_player_app_ctx_t *ctx = (csg_player_app_ctx_t *)user_data;
    if (!ctx->running) return;
    if (!ctx->audio_render_handle) return;
    if (!pcm_data || samples <= 0 || channels <= 0 || rate <= 0) return;

    /* Seek 后音频 PTS 钳制:
     * av_seek_frame 可能使音频流产生与视频关键帧严重不一致的 PTS
     * (如视频 85s 但音频 PTS 108s)。若直接用此 PTS 设置音频时钟,
     * 会导致 AV 同步主时钟跳变, 视频永远落后音频 -> 同步失效。
     * 修复: 用 seek 目标 PTS 作为首帧音频时钟基准, 确保音视频
     * 时钟起点一致。后续帧的 PTS 偏移由码流自身保证连续性。 */
    if (ctx->audio_seek_pts > 0) {
        int64_t diff = pts - ctx->audio_seek_pts;
        if (diff > 1000000 || diff < -1000000) {
            CSG_LOGI("MAIN", "Audio PTS clamp: %lld -> %lld (diff=%+lldus)",
                 (long long)pts, (long long)ctx->audio_seek_pts,
                 (long long)diff);
            pts = ctx->audio_seek_pts;
        }
        ctx->audio_seek_pts = 0;
    }

    ctx->audio_pcm_count++;

    /* 统一重采样 48kHz/2ch 后写入渲染器 (渲染器固定 48k/2ch 打开) */
    {
        /* 300ms 容量: FAAD2 单帧 1024 样本, PCM chunk 最大 160 样本,
         * 300ms@48k 远超单帧需求, 无需堆分配 */
        static int16_t s_local_pcm_out[RTSP_AUDIO_OUT_RATE * 3 / 10
                                       * RTSP_AUDIO_OUT_CH];
        /* ★ 本地播放信号源为 AAC 解码输出 (满幅度 int16_t, ±32767),
         * 增益必须为 1, 否则 ×8 会导致硬削波 → 刺耳爆音.
         * RTSP G.711 路径使用 RTSP_AUDIO_GAIN=8 (弱信号 ±100 专用). */
        int gain = LOCAL_PLAY_GAIN;
        int out_samples = csg_player_audio_resample(
            (const int16_t *)pcm_data, samples, rate, channels,
            s_local_pcm_out,
            (int)(sizeof(s_local_pcm_out) / sizeof(s_local_pcm_out[0])), gain);
        if (out_samples > 0) {
            csg_audio_render_write(ctx->audio_render_handle,
                                   s_local_pcm_out, out_samples, pts);
        }
    }

    /* AV 同步: 音频时钟用原始 (未重采样) 采样数/采样率更新 */
    if (ctx->av_sync_handle && pts != CSG_PLAYER_TICK_INVALID) {
        csg_av_sync_update_audio_clock(ctx->av_sync_handle, pts, samples, rate);
    }

    if (ctx->audio_pcm_count % 100 == 0) {
        CSG_LOGI("MAIN", "Audio PCM #%d: %d samples %dHz/%dch, pts=%lld",
             ctx->audio_pcm_count, samples, rate, channels, (long long)pts);
    }
}

/* ==================== G.711 → PCM16 (ITU-T G.711 标准查表公式) ==================== */

/* μ-law → 16-bit linear PCM */
static int16_t csg_ulaw2linear(uint8_t u_val)
{
    u_val = ~u_val;
    int t = ((u_val & 0x0F) << 3) + 0x84;
    t <<= (u_val & 0x70) >> 4;
    return (u_val & 0x80) ? (int16_t)(0x84 - t) : (int16_t)(t - 0x84);
}

/* A-law → 16-bit linear PCM */
static int16_t csg_alaw2linear(uint8_t a_val)
{
    a_val ^= 0x55;
    int t = (a_val & 0x0F) << 4;
    int seg = (a_val & 0x70) >> 4;
    switch (seg) {
    case 0: t += 8; break;
    case 1: t += 0x108; break;
    default: t += 0x108; t <<= seg - 1; break;
    }
    return (a_val & 0x80) ? (int16_t)t : (int16_t)(-t);
}

/* ==================== RTSP 音频回调 (live555 音频 sink 线程) ====================
 * G.711 (PCMU/PCMA) 原始字节直接透传 (rtsp_live555.cpp 不解码):
 *   ① 录制: G711 原始字节 → mp4_record_push_audio → libmov 封装 G711a/u 轨;
 *   ② 预览: G711 → PCM16 → 重采样 48kHz/双声道 + 增益 → ALSA 渲染器.
 * AAC (mpeg4-generic) raw access unit 透传:
 *   ① 录制: AAC raw bytes → mp4_record_push_audio → libmov 封装 AAC 轨;
 *   ② 预览: AAC → FAAD2 解码 → PCM16 → 重采样 48kHz/双声道 + 增益 → ALSA.
 * ★ 采样率/声道数在 SDP 之后才确定, 故首帧懒创建 ALSA 渲染器 (参照 MPP 懒创建). */

/* RTSP AAC 经 FAAD2 解码后 PCM 回调 (与本地播放 csg_player_on_audio_pcm 同构) */
static void csg_player_on_rtsp_aac_pcm(void *pcm_data, int samples,
                                       int channels, int rate,
                                       int64_t pts, void *user_data)
{
    csg_player_app_ctx_t *ctx = (csg_player_app_ctx_t *)user_data;
    if (!ctx->running || !ctx->live) return;
    if (!pcm_data || samples <= 0 || channels <= 0 || rate <= 0) return;

    ctx->audio_pcm_count++;

    /* 统一重采样 48kHz/2ch 后写入渲染器 */
    static int16_t s_rtsp_pcm_out[RTSP_AUDIO_OUT_RATE * 3 / 10 * RTSP_AUDIO_OUT_CH];
    int out_samples = csg_player_audio_resample(
        (const int16_t *)pcm_data, samples, rate, channels,
        s_rtsp_pcm_out,
        (int)(sizeof(s_rtsp_pcm_out) / sizeof(s_rtsp_pcm_out[0])),
        RTSP_AUDIO_GAIN);
    if (out_samples > 0 && ctx->audio_render_handle) {
        csg_audio_render_write(ctx->audio_render_handle,
                               s_rtsp_pcm_out, out_samples, pts);
    }

    /* AV 同步: 音频时钟用原始 (未重采样) 采样数/采样率更新 */
    if (ctx->av_sync_handle && pts != CSG_PLAYER_TICK_INVALID) {
        csg_av_sync_update_audio_clock(ctx->av_sync_handle, pts, samples, rate);
    }

    if (ctx->audio_pcm_count % 100 == 0) {
        CSG_LOGI("MAIN", "RTSP AAC PCM #%d: %d samples %dHz/%dch, pts=%lld",
             ctx->audio_pcm_count, samples, rate, channels, (long long)pts);
    }
}

/* RTSP 音频原始帧回调 (live555 AudioDummySink 线程):
 *   G.711 (PCMU/PCMA) → ①录制直写 ②G711→PCM16→重采样→ALSA
 *   AAC (mpeg4-generic) → ①录制直写 ②FAAD2解码→PCM16→重采样→ALSA
 *   MP3/MP1A/AC3/Opus → ①录制透传原始字节 ②预览暂不支持 (标记)
 *   LPCM (L16/L8) → ①录制透传(G711u兼容) ②PCM16直写→ALSA
 * ★ 创建只发生一次; 失败置 rtsp_audio_tried, 不再每帧重试打日志. */
static void csg_player_on_rtsp_audio_pcm(const uint8_t *data, int len,
                                         int codec_id, int sample_rate,
                                         int channels, uint64_t pts,
                                         void *user_data)
{
    csg_player_app_ctx_t *ctx = (csg_player_app_ctx_t *)user_data;
    if (!ctx->running || !ctx->live) return;
    if (!data || len <= 0 || sample_rate <= 0 || channels <= 0) return;

    /* ===== 录制: 所有音频编码类型均透传原始字节 (libmov 封装) ===== */
    pthread_mutex_lock(&ctx->record_mutex);
    csg_rec_ctx_t *recs[2] = { &ctx->auto_rec, &ctx->manual_rec };
    for (int i = 0; i < 2; i++) {
        if (recs[i]->recording && recs[i]->record_handle) {
            /* AAC 专属: 首帧到达时设置 ASC (录制线程创建 MP4 writer 前必需) */
            if (codec_id == CSG_AUDIO_CODEC_AAC) {
                const uint8_t *asc = NULL;
                int asc_size = 0;
                if (ctx->rtsp_handle) {
                    csg_rtsp_live555_get_audio_asc(ctx->rtsp_handle, &asc, &asc_size);
                }
                if (asc && asc_size > 0) {
                    mp4_record_set_audio_asc(recs[i]->record_handle, asc, asc_size);
                }
            }
            mp4_record_config_audio(recs[i]->record_handle, codec_id,
                                    sample_rate, channels);
            mp4_record_push_audio(recs[i]->record_handle, data, len, pts);
        }
    }
    pthread_mutex_unlock(&ctx->record_mutex);

    /* ===== 预览: 按编码类型分发 ===== */

    /* ★ AAC 路径: raw access unit → FAAD2 解码 → csg_player_on_rtsp_aac_pcm 回调 */
    if (codec_id == CSG_AUDIO_CODEC_AAC) {

        /* ② 预览: 懒创建 FAAD2 解码器 (与本地播放复用 audio_dec_handle) */
        if (!ctx->audio_dec_handle) {
            const uint8_t *asc = NULL;
            int asc_size = 0;
            if (ctx->rtsp_handle) {
                csg_rtsp_live555_get_audio_asc(ctx->rtsp_handle, &asc, &asc_size);
            }
            if (asc && asc_size > 0) {
                ctx->audio_dec_handle = csg_audio_decode_create(
                    CSG_AUDIO_CODEC_AAC, asc, asc_size,
                    sample_rate, channels, 0,
                    csg_player_on_rtsp_aac_pcm, ctx);
                if (ctx->audio_dec_handle) {
                    CSG_LOGI("MAIN", "RTSP AAC decoder created: %dHz/%dch ASC=%d bytes",
                         sample_rate, channels, asc_size);
                } else {
                    CSG_LOGE("MAIN", "RTSP AAC decoder create failed");
                }
            } else {
                /* 首帧 AAC 到达时 ASC 尚未就绪 (DESCRIBE 异步完成),
                 * 记录日志后跳过, 下一帧重试创建. */
                if (ctx->audio_frame_count == 0) {
                    CSG_LOGW("MAIN", "RTSP AAC ASC not ready, retry next frame");
                }
                return;
            }
        }
        if (ctx->audio_dec_handle) {
            /* ③ 懒创建 ALSA 渲染器 (与 G.711 路径同构) */
            if (!ctx->audio_render_handle && !ctx->rtsp_audio_tried) {
                pthread_mutex_lock(&ctx->rtsp_audio_mutex);
                if (!ctx->audio_render_handle && !ctx->rtsp_audio_tried) {
                    /* ★ v5.9: 优先复用常开音频渲染器 */
                    if (ctx->audio_render_persistent) {
                        ctx->audio_render_handle = ctx->audio_render_persistent;
                        csg_audio_render_flush(ctx->audio_render_handle, 0);
                        CSG_LOGI("MAIN", "RTSP audio device: reuse persistent");
                    } else {
                        static const char *s_rtsp_audio_devices[] = {
                            "hw:1,0", "hw:0,0", "default", NULL
                        };
                        for (int di = 0; s_rtsp_audio_devices[di]; di++) {
                            ctx->audio_render_handle = csg_audio_render_create(
                                RTSP_AUDIO_OUT_RATE, RTSP_AUDIO_OUT_CH, 16,
                                s_rtsp_audio_devices[di]);
                            if (ctx->audio_render_handle) {
                                CSG_LOGI("MAIN", "RTSP audio device: %s (probe OK)",
                                         s_rtsp_audio_devices[di]);
                                break;
                            }
                        }
                    }
                    if (ctx->audio_render_handle) {
                        csg_audio_render_start(ctx->audio_render_handle);
                        ctx->has_audio = 1;
                        /* ★ v5.6: RTSP 拉流路径缺失 AV 同步上下文创建,
                         *   导致 AV-SYNC 模块完全跳过, 视频帧自由跑飞,
                         *   音画不同步. 此处与本地播放路径对齐, 创建
                         *   音频主时钟同步上下文. */
                        if (!ctx->av_sync_handle) {
                            ctx->av_sync_handle = csg_av_sync_create();
                            if (ctx->av_sync_handle) {
                                csg_av_sync_set_master_clock(
                                    ctx->av_sync_handle,
                                    CSG_AV_CLOCK_MASTER_AUDIO);
                                CSG_LOGI("MAIN", "RTSP AV sync created (audio master)");
                            }
                        }
                    }
                    ctx->rtsp_audio_tried = 1;
                }
                pthread_mutex_unlock(&ctx->rtsp_audio_mutex);
            }
            /* ④ 送 AAC 帧到 FAAD2 解码, 输出 PCM 由 csg_player_on_rtsp_aac_pcm 处理 */
            csg_audio_decode_send_packet(ctx->audio_dec_handle, data, len, (int64_t)pts);
            ctx->audio_frame_count++;
            /* ★ v5.6: 首帧诊断, 确认音频流已到达 */
            if (ctx->audio_frame_count == 1) {
                CSG_LOGI("MAIN", "RTSP first audio frame: codec=%d %dHz/%dch len=%d pts=%llu",
                     codec_id, sample_rate, channels, len, (unsigned long long)pts);
            }
        }
        return;   /* AAC 路径到此结束, 不落入 G.711 分支 */
    }

    /* ① 录制: G711 原始字节直接透传 (不解码), libmov 封装 G711a/u 轨.
     * config_audio 幂等, 首帧到达时配置编码类型/采样率/声道即可.
     * ★ 注意: 录制部分已提升到函数顶部统一处理, 此处仅保留预览逻辑. */

    /* ② 预览: 懒创建 ALSA 渲染器 (固定 48kHz/双声道, 由本回调重采样适配) */
    if (!ctx->audio_render_handle && !ctx->rtsp_audio_tried) {
        pthread_mutex_lock(&ctx->rtsp_audio_mutex);
        if (!ctx->audio_render_handle && !ctx->rtsp_audio_tried) {
            /* ★ v5.9: 优先复用常开音频渲染器 */
            if (ctx->audio_render_persistent) {
                ctx->audio_render_handle = ctx->audio_render_persistent;
                csg_audio_render_flush(ctx->audio_render_handle, 0);
                CSG_LOGI("MAIN", "RTSP audio device: reuse persistent");
            } else {
                /* 设备探测: 与本地播放一致, NAU8822 耳机/喇叭优先. */
                static const char *s_rtsp_audio_devices[] = {
                    "hw:1,0",   /* RK3572 NAU8822 编解码器 (耳机/喇叭) — 优先 */
                    "hw:0,0",   /* HDMI/SAI4 数字音频 (兜底) */
                    "default",  /* ALSA 默认路由 (最后兜底) */
                    NULL
                };
                for (int di = 0; s_rtsp_audio_devices[di]; di++) {
                    ctx->audio_render_handle = csg_audio_render_create(
                        RTSP_AUDIO_OUT_RATE, RTSP_AUDIO_OUT_CH, 16,
                        s_rtsp_audio_devices[di]);
                    if (ctx->audio_render_handle) {
                        CSG_LOGI("MAIN", "RTSP audio device: %s (probe OK)",
                                 s_rtsp_audio_devices[di]);
                        break;
                    }
                    CSG_LOGW("MAIN", "RTSP audio device: %s (probe failed, try next)",
                             s_rtsp_audio_devices[di]);
                }
            }
            if (ctx->audio_render_handle) {
                csg_audio_render_start(ctx->audio_render_handle);
                ctx->has_audio = 1;
                CSG_LOGI("MAIN", "RTSP audio renderer ready: %dHz/%dch "
                     "(src %dHz/%dch, resampled+stereo+gain=%d)",
                     RTSP_AUDIO_OUT_RATE, RTSP_AUDIO_OUT_CH,
                     sample_rate, channels, RTSP_AUDIO_GAIN);
                /* ★ v5.6: RTSP G.711 路径同步创建 AV 同步上下文 */
                if (!ctx->av_sync_handle) {
                    ctx->av_sync_handle = csg_av_sync_create();
                    if (ctx->av_sync_handle) {
                        csg_av_sync_set_master_clock(
                            ctx->av_sync_handle,
                            CSG_AV_CLOCK_MASTER_AUDIO);
                        CSG_LOGI("MAIN", "RTSP AV sync created (audio master)");
                    }
                }
            } else {
                CSG_LOGE("MAIN", "RTSP audio renderer create failed (%dHz/%dch)",
                     sample_rate, channels);
            }
            ctx->rtsp_audio_tried = 1;
        }
        pthread_mutex_unlock(&ctx->rtsp_audio_mutex);
    }

    /* ③ 预览: G711 → PCM16 (每样本 1 字节), 再重采样/上混/增益 → ALSA */
    if (ctx->audio_render_handle) {
        int samples = len;   /* G711 每样本 1 字节 */
        if (samples > 4096) samples = 4096;
        static int16_t s_g711_pcm[4096];
        for (int i = 0; i < samples; i++) {
            s_g711_pcm[i] = (codec_id == CSG_AUDIO_CODEC_G711A)
                ? csg_alaw2linear(data[i]) : csg_ulaw2linear(data[i]);
        }
        static int16_t s_rtsp_pcm_out[RTSP_AUDIO_OUT_RATE / 20 * RTSP_AUDIO_OUT_CH]; /* 50ms 容量 */
        int out_samples = csg_player_audio_resample(
            s_g711_pcm, samples, sample_rate, channels,
            s_rtsp_pcm_out,
            (int)(sizeof(s_rtsp_pcm_out) / sizeof(s_rtsp_pcm_out[0])),
            RTSP_AUDIO_GAIN);
        if (out_samples > 0) {
            csg_audio_render_write(ctx->audio_render_handle, s_rtsp_pcm_out,
                                   out_samples, (int64_t)pts);
            ctx->audio_pcm_count++;
        }
    }

    /* ★ LPCM (L16/L8): ①录制已统一处理 ②预览: PCM16 直写 ALSA (无需 G.711 解码) */
    if (codec_id == CSG_AUDIO_CODEC_LPCM) {
        /* 懒创建 ALSA 渲染器 (复用 G711 路径的渲染器) */
        if (!ctx->audio_render_handle && !ctx->rtsp_audio_tried) {
            pthread_mutex_lock(&ctx->rtsp_audio_mutex);
            if (!ctx->audio_render_handle && !ctx->rtsp_audio_tried) {
                /* ★ v5.9: 优先复用常开音频渲染器 */
                if (ctx->audio_render_persistent) {
                    ctx->audio_render_handle = ctx->audio_render_persistent;
                    csg_audio_render_flush(ctx->audio_render_handle, 0);
                    CSG_LOGI("MAIN", "RTSP LPCM audio device: reuse persistent");
                } else {
                    static const char *s_rtsp_audio_devices[] = {
                        "hw:1,0", "hw:0,0", "default", NULL
                    };
                    for (int di = 0; s_rtsp_audio_devices[di]; di++) {
                        ctx->audio_render_handle = csg_audio_render_create(
                            RTSP_AUDIO_OUT_RATE, RTSP_AUDIO_OUT_CH, 16,
                            s_rtsp_audio_devices[di]);
                        if (ctx->audio_render_handle) {
                            CSG_LOGI("MAIN", "RTSP LPCM audio device: %s (probe OK)",
                                     s_rtsp_audio_devices[di]);
                            break;
                        }
                    }
                }
                if (ctx->audio_render_handle) {
                    csg_audio_render_start(ctx->audio_render_handle);
                    ctx->has_audio = 1;
                } else {
                    CSG_LOGE("MAIN", "RTSP LPCM audio renderer create failed");
                }
                ctx->rtsp_audio_tried = 1;
            }
            pthread_mutex_unlock(&ctx->rtsp_audio_mutex);
        }
        /* L16: 每样本 2 字节, 直接当作 PCM16 数组处理 */
        if (ctx->audio_render_handle) {
            int samples = len / 2;  /* 16-bit → 样本数 */
            if (samples > 4096) samples = 4096;
            static int16_t s_lpcm_pcm_out[RTSP_AUDIO_OUT_RATE / 20 * RTSP_AUDIO_OUT_CH];
            int out_samples = csg_player_audio_resample(
                (const int16_t *)data, samples, sample_rate, channels,
                s_lpcm_pcm_out,
                (int)(sizeof(s_lpcm_pcm_out) / sizeof(s_lpcm_pcm_out[0])),
                RTSP_AUDIO_GAIN);
            if (out_samples > 0) {
                csg_audio_render_write(ctx->audio_render_handle,
                                       s_lpcm_pcm_out, out_samples, (int64_t)pts);
                ctx->audio_pcm_count++;
            }
        }
        ctx->audio_frame_count++;
        return;
    }

    /* ★ MP3/MP1A/AC3/Opus: ①录制已统一处理 ②预览暂不支持解码, 仅打日志 */
    if (codec_id == CSG_AUDIO_CODEC_MP3 || codec_id == CSG_AUDIO_CODEC_MP1A ||
        codec_id == CSG_AUDIO_CODEC_AC3 || codec_id == CSG_AUDIO_CODEC_OPUS) {
        ctx->audio_frame_count++;
        if (ctx->audio_frame_count % 100 == 0) {
            const char *name = (codec_id == CSG_AUDIO_CODEC_MP3)  ? "MP3" :
                               (codec_id == CSG_AUDIO_CODEC_MP1A) ? "MP1A" :
                               (codec_id == CSG_AUDIO_CODEC_AC3)  ? "AC3" : "Opus";
            CSG_LOGI("MAIN", "RTSP %s audio #%d: %d bytes %dHz/%dch (recording only, no preview)",
                 name, ctx->audio_frame_count, len, sample_rate, channels);
        }
        return;
    }
}

static void csg_player_on_audio_frame(const uint8_t *data, int len,
                           int64_t pts, int codec_id,
                           int sample_rate, int channels,
                           uint64_t channel_layout,
                           const uint8_t *extradata,
                           int extradata_size,
                           void *user_data)
{
    csg_player_app_ctx_t *ctx = (csg_player_app_ctx_t *)user_data;
    (void)codec_id;
    (void)sample_rate;
    (void)channels;
    (void)channel_layout;
    (void)extradata;
    (void)extradata_size;
    if (!ctx->running) return;
    if (!ctx->demux_buf) return;   /* 缓冲层未就绪: 丢弃 */

    /* 深拷贝入队: 解封装线程立即返回, FAAD2 解码由音频分发线程消费 */
    csg_demux_buffer_push_audio(ctx->demux_buf, data, len, pts, &ctx->running);
}

/* ==================== 音频分发线程 (消费者: 队列 → FAAD2 / PCM 直读) ==================== */
static void *csg_player_audio_dispatch_thread(void *arg)
{
    csg_player_app_ctx_t *ctx = (csg_player_app_ctx_t *)arg;
    csg_demux_audio_frame_t f;

    while (ctx->adispatch_run) {
        int r = csg_demux_buffer_pop_audio(ctx->demux_buf, &f);
        if (r == -1) break;                 /* 缓冲层已 shutdown */
        if (r == 0) continue;

        if (!ctx->running) {
            csg_demux_buffer_free_frame(f.data);
            continue;
        }

        /* ★ PCM 音频轨 (本项目 RTSP 录制格式 sowt): 不经 FAAD2,
         * 直读 PCM 后统一走 on_audio_pcm (内部重采样 48kHz 渲染) */
        if (ctx->local_audio_codec == CSG_AUDIO_CODEC_PCM) {
            int ch = (ctx->local_audio_ch > 0) ? ctx->local_audio_ch : 1;
            int samples = f.len / (2 * ch);
            if (samples > 0) {
                ctx->audio_frame_count++;
                csg_player_on_audio_pcm(f.data, samples, ch,
                                        ctx->local_audio_rate, f.pts, ctx);
                if (ctx->audio_frame_count % 200 == 0) {
                    CSG_LOGI("MAIN", "PCM audio packet #%d: %d samples, pts=%lld",
                         ctx->audio_frame_count, samples, (long long)f.pts);
                }
            }
            csg_demux_buffer_free_frame(f.data);
            continue;
        }

        /* ★ G711 (µ-law/A-law) 音频轨 (本项目 RTSP 录制格式, libmov G711a/u 轨):
         * 不经 FAAD2, 逐字节解码 → PCM16 → 统一走 on_audio_pcm.
         * 与 RTSP 预览路径 (csg_player_on_rtsp_audio_pcm) 完全同构:
         * on_audio_pcm 内部重采样 48kHz/2ch + ×8 增益 (G711 弱信号 ±200 才可闻). */
        if (ctx->local_audio_codec == CSG_AUDIO_CODEC_G711A ||
            ctx->local_audio_codec == CSG_AUDIO_CODEC_G711U) {
            int ch = (ctx->local_audio_ch > 0) ? ctx->local_audio_ch : 1;
            int samples = f.len / ch;   /* G711 每样本每声道 1 字节 */
            if (samples > 0) {
                static int16_t s_g711_pcm[8192];
                int max_s = (int)(sizeof(s_g711_pcm) / sizeof(s_g711_pcm[0])) / ch;
                if (samples > max_s) samples = max_s;
                const uint8_t *p = (const uint8_t *)f.data;
                int is_alaw = (ctx->local_audio_codec == CSG_AUDIO_CODEC_G711A);
                for (int i = 0; i < samples * ch; i++) {
                    s_g711_pcm[i] = is_alaw ? csg_alaw2linear(p[i])
                                            : csg_ulaw2linear(p[i]);
                }
                ctx->audio_frame_count++;
                csg_player_on_audio_pcm(s_g711_pcm, samples, ch,
                                        ctx->local_audio_rate, f.pts, ctx);
                if (ctx->audio_frame_count % 200 == 0) {
                    CSG_LOGI("MAIN", "G711 audio packet #%d: %d samples, pts=%lld",
                         ctx->audio_frame_count, samples, (long long)f.pts);
                }
            }
            csg_demux_buffer_free_frame(f.data);
            continue;
        }

        /* ★ LPCM (L16, 录制时用 G711u 兼容封装): PCM16 直读 → 统一走 on_audio_pcm */
        if (ctx->local_audio_codec == CSG_AUDIO_CODEC_LPCM) {
            int ch = (ctx->local_audio_ch > 0) ? ctx->local_audio_ch : 1;
            int samples = f.len / (2 * ch);  /* 16-bit PCM, 每样本 2 字节 */
            if (samples > 0) {
                ctx->audio_frame_count++;
                csg_player_on_audio_pcm((const int16_t *)f.data, samples, ch,
                                        ctx->local_audio_rate, f.pts, ctx);
                if (ctx->audio_frame_count % 200 == 0) {
                    CSG_LOGI("MAIN", "LPCM audio packet #%d: %d samples, pts=%lld",
                         ctx->audio_frame_count, samples, (long long)f.pts);
                }
            }
            csg_demux_buffer_free_frame(f.data);
            continue;
        }

        /* ★ MP3/MP1A/AC3/Opus: 本地播放暂不支持解码, 跳过 (已录制, 可用第三方播放器) */
        if (ctx->local_audio_codec == CSG_AUDIO_CODEC_MP3 ||
            ctx->local_audio_codec == CSG_AUDIO_CODEC_MP1A ||
            ctx->local_audio_codec == CSG_AUDIO_CODEC_AC3 ||
            ctx->local_audio_codec == CSG_AUDIO_CODEC_OPUS) {
            ctx->audio_frame_count++;
            if (ctx->audio_frame_count % 200 == 0) {
                const char *name = (ctx->local_audio_codec == CSG_AUDIO_CODEC_MP3)  ? "MP3" :
                                   (ctx->local_audio_codec == CSG_AUDIO_CODEC_MP1A) ? "MP1A" :
                                   (ctx->local_audio_codec == CSG_AUDIO_CODEC_AC3)  ? "AC3" : "Opus";
                CSG_LOGI("MAIN", "Local %s audio #%d: %d bytes (no decoder, skip)",
                     name, ctx->audio_frame_count, f.len);
            }
            csg_demux_buffer_free_frame(f.data);
            continue;
        }

        /* AAC: 队列 → FAAD2 解码 → on_audio_pcm 回调 */
        if (!ctx->audio_dec_handle) {
            csg_demux_buffer_free_frame(f.data);
            continue;
        }

        ctx->audio_frame_count++;
        csg_audio_decode_send_packet(ctx->audio_dec_handle, f.data, f.len, f.pts);

        if (ctx->audio_frame_count % 100 == 0) {
            CSG_LOGI("MAIN", "Audio packet #%d: %d bytes, pts=%lld",
                 ctx->audio_frame_count, f.len, (long long)f.pts);
        }
        csg_demux_buffer_free_frame(f.data);
    }
    return NULL;
}

/* ==================== 缓冲层 启停 (播放器核心层) ==================== */

/* 前向声明: 视频分发线程定义在其后, 供 start_demux_buffer 使用 */
static void *csg_player_video_dispatch_thread(void *arg);

/* 启动解封装→解码缓冲层: 创建双端队列 + 拉起视频/音频分发线程.
 * 在设置解封装回调之前调用, 保证首帧入队时消费者已就绪. */
static void csg_player_start_demux_buffer(csg_player_app_ctx_t *ctx)
{
    if (ctx->demux_buf) return;   /* 已存在: 幂等 */

    csg_demux_buffer_cfg_t cfg = {0};
    csg_demux_buffer_t *dbuf = NULL;
    if (csg_demux_buffer_create(&cfg, &dbuf) != 0) {
        CSG_LOGE("MAIN", "Failed to create demux buffer");
        ctx->demux_buf = NULL;
        return;
    }
    ctx->demux_buf = dbuf;

    ctx->vdispatch_run = 1;
    ctx->adispatch_run = 1;
    if (pthread_create(&ctx->vdispatch_thread, NULL,
                       csg_player_video_dispatch_thread, ctx) != 0) {
        ctx->vdispatch_run = 0;
        CSG_LOGE("MAIN", "Failed to create video dispatch thread");
    }
    if (pthread_create(&ctx->adispatch_thread, NULL,
                       csg_player_audio_dispatch_thread, ctx) != 0) {
        ctx->adispatch_run = 0;
        CSG_LOGE("MAIN", "Failed to create audio dispatch thread");
    }
    CSG_LOGI("MAIN", "Demux buffer + dispatch threads started");
}

/* 停止缓冲层: 清空队列(代次++唤醒push) → shutdown(唤醒pop) → join 分发线程 →
 * 销毁. 必须在 MPP/audio decoder 销毁之前调用, 否则分发线程可能访问已释放句柄. */
static void csg_player_stop_demux_buffer(csg_player_app_ctx_t *ctx)
{
    if (!ctx->demux_buf) return;

    csg_demux_buffer_flush(ctx->demux_buf);
    ctx->vdispatch_run = 0;
    ctx->adispatch_run = 0;
    csg_demux_buffer_shutdown(ctx->demux_buf);

    if (ctx->vdispatch_thread) {
        pthread_join(ctx->vdispatch_thread, NULL);
        ctx->vdispatch_thread = 0;
    }
    if (ctx->adispatch_thread) {
        pthread_join(ctx->adispatch_thread, NULL);
        ctx->adispatch_thread = 0;
    }

    csg_demux_buffer_destroy(ctx->demux_buf);
    ctx->demux_buf = NULL;
    CSG_LOGI("MAIN", "Demux buffer + dispatch threads stopped");
}

static void csg_player_on_mpp_frame(void *yuv_data, int yuv_len,
                         int width, int height, int hor_stride, int ver_stride,
                         uint64_t pts, int is_keyframe, void *user_data);

static void csg_player_on_local_frame(void *data, int len, int is_keyframe,
                           uint64_t pts, int codec_type, void *user_data);

static void csg_player_local_event_cb(csg_local_event_t event, void *user_data);
static void csg_player_local_eof_ui_async(void *data);
static void csg_player_local_error_cleanup_async(void *data);

/* ==================== 音频管线 ==================== */
static int csg_player_setup_audio_pipeline(csg_player_app_ctx_t *ctx, int local_handle)
{
    int codec_id = 0, sample_rate = 0, channels = 0;
    uint64_t channel_layout = 0;
    const uint8_t *extradata = NULL;
    int extradata_size = 0;

    /* 复位上一文件的音频轨参数 (防切文件后残留误判 PCM/AAC) */
    ctx->local_audio_codec = CSG_AUDIO_CODEC_NONE;
    ctx->local_audio_rate = 0;
    ctx->local_audio_ch = 0;

    if (csg_local_playback_get_audio_info(local_handle, &codec_id,
                                      &sample_rate, &channels,
                                      &channel_layout,
                                      &extradata, &extradata_size) != 0) {
        CSG_LOGI("MAIN", "No audio stream in file");
        ctx->has_audio = 0;
        return 0;
    }

    CSG_LOGI("MAIN", "Audio stream: codec=%d, %dHz/%dch, extradata=%d bytes",
         codec_id, sample_rate, channels, extradata_size);

    /* 创建音视频同步上下文 */
    ctx->av_sync_handle = csg_av_sync_create();
    if (!ctx->av_sync_handle) {
        CSG_LOGE("MAIN", "Failed to create AV sync");
        return -1;
    }
    csg_av_sync_set_master_clock(ctx->av_sync_handle, CSG_AV_CLOCK_MASTER_AUDIO);

    /* 记录音频轨参数 (音频分发线程据此区分 PCM 直读 / AAC→FAAD2) */
    ctx->local_audio_codec = codec_id;
    ctx->local_audio_rate = sample_rate;
    ctx->local_audio_ch = channels;

    /* AAC 轨: 创建 FAAD2 解码器.
     * PCM / G711 (µ-law/A-law) 轨无需解码: PCM 直读渲染,
     * G711 在音频分发线程逐字节解码后渲染 (见 csg_player_audio_dispatch_thread). */
    if (codec_id == CSG_AUDIO_CODEC_AAC) {
        ctx->audio_dec_handle = csg_audio_decode_create(codec_id,
                                                    extradata, extradata_size,
                                                    sample_rate, channels,
                                                    channel_layout,
                                                    csg_player_on_audio_pcm, ctx);
        if (!ctx->audio_dec_handle) {
            CSG_LOGE("MAIN", "Failed to create audio decoder");
            csg_av_sync_destroy(ctx->av_sync_handle);
            ctx->av_sync_handle = NULL;
            return -1;
        }
    } else if (codec_id != CSG_AUDIO_CODEC_PCM &&
               codec_id != CSG_AUDIO_CODEC_G711A &&
               codec_id != CSG_AUDIO_CODEC_G711U) {
        CSG_LOGE("MAIN", "Unsupported audio codec=%d", codec_id);
        csg_av_sync_destroy(ctx->av_sync_handle);
        ctx->av_sync_handle = NULL;
        return -1;
    }

    /* ★ v5.9: 优先复用常开音频渲染器, 避免 snd_pcm_close/open 循环
     * 导致 NAU8822 codec power_off 破坏 DAPM 内部 Jack Detect 状态.
     * 若 persistent 渲染器不存在 (首次启动或 fallback), 则走原有创建流程. */
    if (ctx->audio_render_persistent) {
        ctx->audio_render_handle = ctx->audio_render_persistent;
        CSG_LOGI("MAIN", "Audio device: reuse persistent (probe OK)");

        /* flush 重置 PCM + 重新初始化 mixer + 清空 ring buffer + 重置时钟 */
        csg_audio_render_flush(ctx->audio_render_handle, 0);
    } else {
        /* 创建音频渲染器 (ALSA) — ★ 固定 48kHz/双声道 (NAU8822 最佳工作频率):
         * 所有音频 (AAC 原生率 / PCM 8k) 由回调层统一重采样适配,
         * 修复按 AAC 原生低采样率 (如 16kHz) 打开导致的"本地播放无声"。
         * ★ 设备自动探测: RK3572 + NAU8822 声卡号不固定,
         *   依次尝试 hw:1,0 → hw:0,0 → default, 第一个成功即使用.
         *   hw:1,0 优先: RK3572 上 NAU8822 编解码器通常挂在 card1,
         *   card0 是 HDMI/SAI4 数字接口 (无模拟输出). */
        static const char *s_audio_devices[] = {
            "hw:1,0",   /* RK3572 NAU8822 编解码器 (耳机/喇叭) — 优先 */
            "hw:0,0",   /* HDMI/SAI4 数字音频 (无模拟输出, 兜底) */
            "default",  /* ALSA 默认路由 (最后兜底) */
            NULL
        };

        ctx->audio_render_handle = NULL;
        for (int di = 0; s_audio_devices[di]; di++) {
            ctx->audio_render_handle = csg_audio_render_create(
                RTSP_AUDIO_OUT_RATE, RTSP_AUDIO_OUT_CH, 16, s_audio_devices[di]);
            if (ctx->audio_render_handle) {
                CSG_LOGI("MAIN", "Audio device: %s (probe OK)", s_audio_devices[di]);
                break;
            }
            CSG_LOGW("MAIN", "Audio device: %s (probe failed, try next)", s_audio_devices[di]);
        }
    }

    if (!ctx->audio_render_handle) {
        CSG_LOGE("MAIN", "Failed to create audio renderer (all devices failed)");
        goto fail;
    }

    /* 启动音频渲染 */
    if (csg_audio_render_start(ctx->audio_render_handle) != 0) {
        CSG_LOGE("MAIN", "Failed to start audio renderer");
        goto fail;
    }

    ctx->has_audio = 1;
    CSG_LOGI("MAIN", "Audio pipeline ready: %s %dHz/%dch -> render 48kHz/2ch",
             (codec_id == CSG_AUDIO_CODEC_PCM) ? "PCM" :
             (codec_id == CSG_AUDIO_CODEC_AAC) ? "AAC" : "G711",
             sample_rate, channels);
    return 0;

fail:
    if (ctx->audio_render_handle) {
        csg_audio_render_destroy(ctx->audio_render_handle);
        ctx->audio_render_handle = NULL;
    }
    if (ctx->audio_dec_handle) {
        csg_audio_decode_destroy(ctx->audio_dec_handle);
        ctx->audio_dec_handle = NULL;
    }
    csg_av_sync_destroy(ctx->av_sync_handle);
    ctx->av_sync_handle = NULL;
    return -1;
}

static void csg_player_teardown_audio_pipeline(csg_player_app_ctx_t *ctx)
{
    if (ctx->audio_render_handle) {
        csg_audio_render_stop(ctx->audio_render_handle);
        /* ★ v5.9: 常开渲染器只 stop 不 destroy, 保持 PCM 设备打开.
         * 避免 snd_pcm_close/open 循环导致 NAU8822 codec power_off
         * 破坏 DAPM 内部 Jack Detect 状态. */
        if (ctx->audio_render_handle != ctx->audio_render_persistent) {
            csg_audio_render_destroy(ctx->audio_render_handle);
        } else {
            CSG_LOGI("MAIN", "Audio renderer: keep persistent, stop only");
        }
        ctx->audio_render_handle = NULL;
    }
    if (ctx->audio_dec_handle) {
        csg_audio_decode_destroy(ctx->audio_dec_handle);
        ctx->audio_dec_handle = NULL;
    }
    if (ctx->av_sync_handle) {
        csg_av_sync_destroy(ctx->av_sync_handle);
        ctx->av_sync_handle = NULL;
    }
    ctx->has_audio = 0;
    /* 音频轨参数一并复位 (防切到无音轨文件后残留) */
    ctx->local_audio_codec = CSG_AUDIO_CODEC_NONE;
    ctx->local_audio_rate = 0;
    ctx->local_audio_ch = 0;
}

/* ==================== 解封装 → 缓冲队列 (生产者) ====================
 * 解封装线程 (local_playback) 每读到一帧编码数据即深拷贝入队后立即返回,
 * 不再被 MPP 同步 send_frame 阻塞. 由视频分发线程从队列取帧送硬件解码. */
static void csg_player_rec_frame_common(csg_player_app_ctx_t *ctx,
                                        csg_rec_ctx_t *rec, const char *tag,
                                        void *data, int len, int is_keyframe,
                                        uint64_t pts);
static void csg_player_on_local_frame(void *data, int len, int is_keyframe,
                           uint64_t pts, int codec_type, void *user_data)
{
    csg_player_app_ctx_t *ctx = (csg_player_app_ctx_t *)user_data;
    if (!ctx->running) return;
    if (!ctx->demux_buf) return;   /* 缓冲层未就绪: 丢弃 */

    /* RTSP 录制: 复用预览帧 (预览与录制同源, 不建第二个 live555 实例,
     * 根治双实例事件循环并发导致的堆损坏 free(): invalid pointer) */
    if (ctx->live && ctx->rtsp_handle) {
        if ((ctx->stats.recv_packets % 30) == 0)
            CSG_LOGI("MAIN", "RECV cb live=1 rtsp=%p auto(s=%d r=%d h=%p) manual(s=%d r=%d h=%p)",
                     ctx->rtsp_handle,
                     ctx->auto_rec.start_pending, ctx->auto_rec.recording, ctx->auto_rec.record_handle,
                     ctx->manual_rec.start_pending, ctx->manual_rec.recording, ctx->manual_rec.record_handle);
        csg_player_rec_frame_common(ctx, &ctx->auto_rec, "AUTO",
                                    data, len, is_keyframe, pts);
        csg_player_rec_frame_common(ctx, &ctx->manual_rec, "MANUAL",
                                    data, len, is_keyframe, pts);
    }

    int pr = csg_demux_buffer_push_video(ctx->demux_buf, data, len, is_keyframe,
                                         (int64_t)pts, codec_type, &ctx->running);
    /* ★ DIAG: 定位缓冲层断点 — 解封装回调是否入队成功 */
    if ((ctx->stats.recv_packets % 30) == 0) {
        CSG_LOGI("MAIN", "DIAG cb: len=%d kf=%d pts=%llu push=%d",
             len, is_keyframe, (unsigned long long)pts, pr);
    }
}

/* ==================== 视频分发线程: 缓冲队列 → MPP 解码 ====================
 * 原 csg_player_on_local_frame 主体迁入本函数 (在视频分发线程内执行).
 * 高倍速关键帧稀疏化/过滤、关键帧 PTS 记录等逻辑保持单线程语义,
 * 与 MPP 输出回调线程、UI 线程的共享状态访问关系与原架构一致. */
static void csg_player_dispatch_video_frame(csg_player_app_ctx_t *ctx,
                                            void *data, int len, int is_keyframe,
                                            uint64_t pts, int codec_type)
{
    (void)codec_type;
    if (!ctx->running) return;

    if (ctx->state != CSG_PLAYER_STATE_PLAYING && ctx->state != CSG_PLAYER_STATE_PAUSED) {
        return;
    }

    /* ★ live 流首帧懒创建 MPP 解码器: RTSP 场景下编码/分辨率由 SDP 异步
     * 确定, PLAY 命令执行后才开始收帧, 此时 csg_rtsp_live555_get_raw_info
     * 已可返回有效参数. 本地播放路径 MPP 已在 PLAY_LOCAL 中创建 (live=0). */
    if (ctx->live && !ctx->mpp_handle && ctx->rtsp_handle) {
        int codec, w, h, sl, pl, vl;
        const uint8_t *sps, *pps, *vps;
        if (csg_rtsp_live555_get_raw_info(ctx->rtsp_handle, &codec, &w, &h,
                                          &sps, &sl, &pps, &pl, &vps, &vl) == 0
            && w > 0 && h > 0) {
            ctx->rtsp_codec = codec;
            ctx->rtsp_width = w;
            ctx->rtsp_height = h;
            ctx->mpp_handle = csg_mpp_decode_create((csg_player_codec_t)codec, w, h);
            if (ctx->mpp_handle) {
                /* 注入 SDP sprop 参数集 (Annex-B), 保证首帧可解 */
                uint8_t edata[512];
                int ed = 0;
                const uint8_t st[4] = {0, 0, 0, 1};
                if (codec == CSG_CODEC_H265 && vl > 0) {
                    memcpy(edata + ed, st, 4); ed += 4;
                    memcpy(edata + ed, vps, (size_t)vl); ed += vl;
                }
                if (sl > 0) {
                    memcpy(edata + ed, st, 4); ed += 4;
                    memcpy(edata + ed, sps, (size_t)sl); ed += sl;
                }
                if (pl > 0) {
                    memcpy(edata + ed, st, 4); ed += 4;
                    memcpy(edata + ed, pps, (size_t)pl); ed += pl;
                }
                if (ed > 0)
                    csg_mpp_decode_set_extra_data(ctx->mpp_handle, edata, ed);
                csg_mpp_decode_set_callback(ctx->mpp_handle, csg_player_on_mpp_frame, ctx);
                ctx->state_gen = csg_player_next_state_gen();
                ctx->mpp_gen = ctx->state_gen;
                CSG_LOGI("MAIN", "RTSP MPP lazy-created: %s %dx%d",
                     codec == CSG_CODEC_H264 ? "H264" : "H265", w, h);
            }
        }
    }

    /* 录制已拆到独立主码流 (csg_player_on_record_frame): 预览子码流只喂 MPP,
     * 主码流才进 MP4v2 录制器. 双码流互不干扰, 高倍速预览不影响录制完整性. */

    ctx->stats.recv_packets++;
    ctx->stats.recv_bytes += len;

    /* MPP reset 后等待关键帧才能恢复解码 */
    if (ctx->mpp_needs_keyframe && !is_keyframe) {
        ctx->stats.drop_frames++;
        return;
    }

    if (ctx->mpp_handle) {
        float spd = ctx->time_state.speed;
        int high_speed = (spd >= MAIN_KEYFRAME_ONLY_MIN_SPEED);

        if (is_keyframe) {
            /* ★ 高倍速关键帧稀疏化: 统一高倍速显示节奏 (~2fps),
             * 让 16x 也像 8x 一样"明显顿挫"地跳帧.
             *
             * 背景: 8x 因音频时钟超前, AV-SYNC 会丢弃约一半关键帧,
             * 画面每 ~500ms 顿一下 (2fps), 观感上"明显在跳帧";
             * 而 16x 音频跟不上, 所有关键帧都显示, 变成 ~8fps 的
             * "顺滑快进", 用户误以为没有跳帧. 实际上两者都在只播关键帧.
             *
             * 这里按目标显示帧率 (~2fps) 限制送入 MPP 的关键帧间隔:
             *   min_gap = speed / MAIN_KF_DISPLAY_FPS (秒) → us
             *   例: 16x → 8s, 8x → 4s, 4x → 2s, 6x → 3s
             * 过密的关键帧直接丢弃 (I 帧独立可解, 丢弃不影响后续解码),
             * 让媒体时间轴按 speed 倍速推进, 同时大幅降低 MPP 解码负载
             * (对 2K 视频 16x 至关重要, 之前全帧送入 MPP 吞吐不足只有 ~3x).
             * 全 I 帧 / 极短 GOP 视频同理被限到 ~2fps.
             * MPP reset 后等待关键帧 (mpp_needs_keyframe=1) 期间不稀疏化,
             * 保证首个关键帧一定送入解码器. */
            if (high_speed && !ctx->mpp_needs_keyframe && ctx->last_fed_kf_pts > 0) {
                int64_t gap = (int64_t)pts - ctx->last_fed_kf_pts;
                int64_t min_gap = (int64_t)((double)spd * 1000000.0 / MAIN_KF_DISPLAY_FPS);
                if (min_gap < 40000) min_gap = 40000;   /* 下限 40ms */
                if (gap > 0 && gap < min_gap) {
                    ctx->stats.drop_frames++;
                    if ((ctx->stats.drop_frames % 100) == 0) {
                        CSG_LOGI("MAIN", "Speed-filter: drop dense keyframe (speed=%.1fx, "
                             "gap=%lldus, total_drop=%u)", spd, (long long)gap,
                             ctx->stats.drop_frames);
                    }
                    return;
                }
            }
            ctx->last_fed_kf_pts = (int64_t)pts;
            ctx->last_keyframe_pts = (int64_t)pts;

            /* 记录关键帧 PTS 到环形缓冲区, 用于输出侧 PTS 验证.
             * 这是高倍速防御层的核心数据源: 输出帧的 PTS 若无法匹配
             * 任何已知关键帧 PTS, 则判定为管道残留 P 帧, 直接丢弃. */
            ctx->kf_pts_ring[ctx->kf_pts_ring_head] = (int64_t)pts;
            ctx->kf_pts_ring_head = (ctx->kf_pts_ring_head + 1) % MAIN_KF_RING_SIZE;
            if (ctx->kf_pts_ring_count < MAIN_KF_RING_SIZE) {
                ctx->kf_pts_ring_count++;
            }

            if (ctx->mpp_needs_keyframe) {
                ctx->mpp_needs_keyframe = 0;
                ctx->mpp_kf_wait_decode = 1;
                ctx->expected_kf_pts = (int64_t)pts;
            }
        }

        /* 高倍速帧过滤: speed ≥ 4x 时只保留 I 关键帧, 丢弃 P/B 帧.
         * 2x: 全部帧正常解码播放, 音频正常输出.
         * 4x/6x/8x/16x: 只解码 I 关键帧 (必要时再稀疏化), av_sync 通过
         *   frame_timer 缩放自动控制显示节奏, 实现时间轴快速推进.
         * 切回低倍速后 P/B 帧恢复正常解码, 不需额外处理. */
        if (high_speed && !is_keyframe) {
            ctx->stats.drop_frames++;
            if ((ctx->stats.drop_frames % 100) == 0) {
                CSG_LOGI("MAIN", "Speed-filter: drop non-keyframe (speed=%.1fx, "
                     "total_drop=%u)", spd, ctx->stats.drop_frames);
            }
            return;
        }

        csg_mpp_decode_send_frame(ctx->mpp_handle, data, len, is_keyframe, pts);
    }
}

/* ==================== 视频分发线程 (消费者: 队列 → MPP) ==================== */
static void *csg_player_video_dispatch_thread(void *arg)
{
    csg_player_app_ctx_t *ctx = (csg_player_app_ctx_t *)arg;
    csg_demux_video_frame_t f;

    while (ctx->vdispatch_run) {
        int r = csg_demux_buffer_pop_video(ctx->demux_buf, &f);
        if (r == -1) break;                 /* 缓冲层已 shutdown */
        if (r == 0) continue;               /* 暂时为空, 重试 */

        /* ★ DIAG: 定位缓冲层断点 — 分发线程是否取到帧 */
        if ((ctx->stats.recv_packets % 30) == 0) {
            CSG_LOGI("MAIN", "DIAG dispatch: len=%d kf=%d pts=%llu state=%d",
                 f.len, f.is_keyframe, (unsigned long long)f.pts, ctx->state);
        }

        /* 停止/非播放态: 丢弃残留帧 */
        if (!ctx->running ||
            (ctx->state != CSG_PLAYER_STATE_PLAYING &&
             ctx->state != CSG_PLAYER_STATE_PAUSED)) {
            csg_demux_buffer_free_frame(f.data);
            continue;
        }

        csg_player_dispatch_video_frame(ctx, f.data, f.len, f.is_keyframe,
                                        f.pts, f.codec_type);
        csg_demux_buffer_free_frame(f.data);
    }
    return NULL;
}

/* ==================== MPP 解码输出回调 ==================== */
static void csg_player_on_mpp_frame(void *yuv_data, int yuv_len,
                         int width, int height, int hor_stride, int ver_stride,
                         uint64_t pts, int is_keyframe, void *user_data)
{
    csg_player_app_ctx_t *ctx = (csg_player_app_ctx_t *)user_data;
    (void)yuv_len;
    if (!ctx->running) return;

    /* 防竞争: 若回调到达时 MPP 已被切换到新的 (generation 不匹配), 丢弃 */
    if (ctx->mpp_gen != ctx->state_gen) return;

    /* 暂停: 不显示新帧, 画面保持暂停前最后一帧; 逐帧步进放行一帧 */
    if (ctx->state == CSG_PLAYER_STATE_PAUSED) {
        if (!ctx->step_frame_pending) return;
        ctx->step_frame_pending = 0;
        csg_lvgl_display_step_frame();
    } else if (ctx->state != CSG_PLAYER_STATE_PLAYING) {
        return;
    }

    /* Seek 后直接显示第一个解码帧 (不再等待 catchup 到目标 PTS).
     * 精确匹配: mpp_kf_wait_decode=1 → 关键帧已送入, 首个输出帧即视为 DPB 重建完成 */
    int was_post_resume_sync = ctx->post_resume_sync;
    if (ctx->post_resume_sync) {
        if (ctx->resume_min_pts > 0) {
            /* 暂停恢复路径: 丢弃 pre-pause 残留帧 */
            if ((int64_t)pts <= ctx->resume_min_pts) {
                CSG_LOGI("MAIN", "sync-drop: pts=%llu <= min=%lld, discard",
                     (unsigned long long)pts, (long long)ctx->resume_min_pts);
                return;
            }
            CSG_LOGI("MAIN", "resume-sync: first valid frame pts=%llu (min=%lld)",
                 (unsigned long long)pts, (long long)ctx->resume_min_pts);
        } else {
            /* Seek 路径: mpp_kf_wait_decode=0 → 关键帧尚未送入 MPP, 残留帧丢弃 */
            if (!ctx->mpp_kf_wait_decode) {
                CSG_LOGI("MAIN", "seek-discard: residual frame pts=%llu (kf not yet fed), drop",
                     (unsigned long long)pts);
                return;
            }
            /* ★ mpp_kf_wait_decode=1 → 关键帧已送入 MPP.
             * 注意: 不能依赖 is_keyframe 判断! mpp_decode.c 中 pending_is_keyframe
             * 是全局变量, 会被后续 send_frame() 覆盖, 导致关键帧被误报为 non-keyframe.
             * 日志证据: "seek-drop: non-keyframe #1 pts=27966000" — 这恰是对齐关键帧 PTS.
             *
             * 修复: 用 PTS 范围判断新旧帧, 而非 is_keyframe 标志.
             * 关键帧 PTS (audio_sync_base_pts) 是 Seek 对齐位置, 新帧 PTS 应在附近.
             * 旧帧 PTS 差距 >10 秒 (如 103s → 28s 的回退 Seek), 直接丢弃. */
            if (ctx->audio_sync_base_pts > 0) {
                int64_t kf_pts = ctx->audio_sync_base_pts;
                int64_t diff = (int64_t)pts - kf_pts;
                if (diff < 0) diff = -diff;
                if (diff > 10000000) {  /* >10 秒 → 旧帧 */
                    ctx->seek_drop_count++;
                    if (ctx->seek_drop_count <= 3) {
                        CSG_LOGI("MAIN", "seek-drop: stale frame pts=%llu (kf_pts=%lld, diff=%lldus)",
                             (unsigned long long)pts, (long long)kf_pts, (long long)diff);
                    }
                    if (ctx->seek_drop_count <= 50) {
                        return;
                    }
                    CSG_LOGW("MAIN", "seek: %d stale frames dropped, forcing through",
                         ctx->seek_drop_count);
                }
                /* PTS 在关键帧 10 秒内 → 新帧, 放行 */
            }
        }
        ctx->post_resume_sync = 0;
        ctx->resume_keyframe_seen = 0;
        ctx->seek_target_pts = 0;
        ctx->resume_min_pts = 0;
        ctx->seek_drop_count = 0;
        CSG_LOGI("MAIN", "sync: showing frame pts=%llu", (unsigned long long)pts);

        if (ctx->mpp_kf_wait_decode) {
            ctx->mpp_kf_wait_decode = 0;
            ctx->mpp_needs_keyframe = 0;
            ctx->expected_kf_pts = 0;

            /* Seek 后有音频: 延迟解冻, 等待音频时钟真正开始推进后再显示.
             * 否则视频解码远快于音频, 音频时钟卡在 base_pts 不动,
             * 导致大量视频帧被 DROP -> 画面卡顿 + 音视频不同步. */
            if (ctx->has_audio) {
                ctx->waiting_audio_ready = 1;
                ctx->audio_wait_frame_count = 0;
                CSG_LOGI("MAIN", "Keyframe decoded OK, waiting for audio clock "
                     "(base=%lld)...", (long long)ctx->audio_sync_base_pts);
            } else {
                csg_lvgl_display_set_paused(0);
                ctx->seek_fast_feed_done = 1;
                CSG_LOGI("MAIN", "Keyframe decoded OK, DPB rebuilt, display unfrozen");
            }
        }
    } else if (ctx->mpp_kf_wait_decode) {
        /* 非 post_resume_sync 但仍在等待关键帧解码 (如首帧启动场景) */
        ctx->mpp_kf_wait_decode = 0;
        ctx->mpp_needs_keyframe = 0;
        ctx->expected_kf_pts = 0;
        CSG_LOGI("MAIN", "Keyframe decoded OK (startup), DPB rebuilt");
    }

    ctx->stats.dec_frames++;

    /* ★ 动态帧率计算: 当容器未提供 FPS (video_fps=0) 时,
     * 从前 10 帧的 PTS 间隔推算实际帧率, 用于驱动 frame_timer 节流.
     * 否则 frame_duration=0 → 帧间隔节流失效 → 解码超产帧覆盖显示缓冲 → 卡顿. */
    if (!ctx->fps_dynamic_set && ctx->av_sync_handle
        && ctx->video_fps <= 0 && ctx->fps_calc_count < 10) {
        if (ctx->fps_calc_count > 0 && (int64_t)pts > ctx->fps_prev_pts) {
            int64_t interval = (int64_t)pts - ctx->fps_prev_pts;
            ctx->fps_interval_sum += interval;
            ctx->fps_calc_count++;
            /* 至少 3 帧后输出平均帧间隔, 设置 frame_duration */
            if (ctx->fps_calc_count >= 3) {
                int64_t avg_interval = ctx->fps_interval_sum
                                       / (ctx->fps_calc_count - 1);
                if (avg_interval > 1000 && avg_interval < 500000) {
                    float dyn_fps = 1000000.0f / (float)avg_interval;
                    csg_av_sync_set_frame_duration_by_fps(
                        ctx->av_sync_handle, dyn_fps);
                    CSG_LOGI("MAIN", "Dynamic FPS: %.1ffps (avg_interval=%lldus "
                         "over %d frames)",
                         (double)dyn_fps, (long long)avg_interval,
                         ctx->fps_calc_count);
                    ctx->fps_dynamic_set = 1;
                }
            }
        } else {
            ctx->fps_calc_count = 1;
        }
        ctx->fps_prev_pts = (int64_t)pts;
    }

    /* Seek 后音频就绪等待: 关键帧已解码但音频时钟尚未推进,
     * 丢弃中间帧等待音频管线产出首帧 PCM 数据.
     * 超时 15 帧后强制解冻, 防止音频卡死导致画面永久冻结.
     *
     * 解冻时若视频已大幅超前音频 (等待期间视频解码器持续输出),
     * 将音频时钟快进到当前视频 PTS, 避免解冻后立即大量 DROP. */
    if (ctx->waiting_audio_ready && ctx->audio_render_handle) {
        int64_t audio_clock = csg_audio_render_get_clock(ctx->audio_render_handle);
        ctx->audio_wait_frame_count++;

        if (audio_clock > ctx->audio_sync_base_pts ||
            ctx->audio_wait_frame_count > 15) {
            ctx->waiting_audio_ready = 0;

            int64_t current_pts = (int64_t)pts;

            /* 等待期间视频解码器持续输出, 视频 PTS 可能已远超音频时钟.
             * 若 drift 超过 200ms, 将音频时钟快进到当前视频 PTS,
             * 避免解冻后 AV-SYNC 立即触发大量 DROP 导致画面跳跃.
             *
             * ★ 双向修正: 高倍速 (≥4x) 下 Seek 后音频时钟以 speed 倍率推进,
             *   视频解码管道重建耗时 0.3~0.5s, 期间音频时钟已超前视频 PTS 数秒.
             *   原逻辑只处理 video > audio 方向, 现增加 audio > video 方向,
             *   将音频时钟回退到当前视频 PTS, 消除启动延迟. */
            if (audio_clock > 0) {
                int64_t drift = current_pts - audio_clock;
                if (drift > 200000 || drift < -200000) {
                    CSG_LOGI("MAIN", "Audio clock skip: %lld -> %lld "
                         "(drift was %+lldus, waited %d frames)",
                         (long long)audio_clock, (long long)current_pts,
                         (long long)drift,
                         ctx->audio_wait_frame_count);
                    csg_audio_render_adjust_clock(ctx->audio_render_handle, current_pts);
                    audio_clock = current_pts;
                }
            }

            csg_lvgl_display_set_paused(0);
            ctx->seek_fast_feed_done = 1;
            CSG_LOGI("MAIN", "Audio ready: clock=%lld > base=%lld "
                 "(waited %d frames)",
                 (long long)audio_clock,
                 (long long)ctx->audio_sync_base_pts,
                 ctx->audio_wait_frame_count);
        } else {
            /* 音频未就绪, 丢弃帧等待 */
            ctx->stats.drop_frames++;
            return;
        }
    }

    /* ★ 高倍速防御层: speed ≥ 4x 时只应输出 I 关键帧.
     * 不依赖 mpp_decode 的 is_keyframe 标志 (该标志因 pending_is_keyframe
     * 全局变量被覆盖而不可靠), 改用 PTS 环形缓冲区验证:
     *   1. csg_player_on_local_frame 将送入 MPP 的关键帧 PTS 记录到环形缓冲区
     *   2. 输出帧的 PTS 若无法匹配任何已知关键帧 PTS (500ms 容差),
     *      则判定为管道残留非关键帧, 直接丢弃
     *
     * 主防线在 csg_player_on_local_frame (不入 MPP) 和
     * csg_player_set_speed (低→高倍速切换时 reset MPP),
     * 此处为最后一道防御, 防止任何管道残留帧导致花屏. */
    {
        float spd = ctx->time_state.speed;
        if (spd >= MAIN_KEYFRAME_ONLY_MIN_SPEED) {
            int is_verified_kf = 0;
            for (int i = 0; i < ctx->kf_pts_ring_count; i++) {
                int64_t diff = (int64_t)pts - ctx->kf_pts_ring[i];
                if (diff < 0) diff = -diff;
                if (diff < 500000) {  /* 500ms 容差: MPP PTS 90kHz 往返转换误差 <1μs */
                    is_verified_kf = 1;
                    break;
                }
            }
            if (!is_verified_kf) {
                ctx->stats.drop_frames++;
                if (ctx->stats.drop_frames % 50 == 0) {
                    CSG_LOGI("MAIN", "Speed-guard: drop non-keyframe pts=%llu "
                         "(speed=%.1fx, total_drop=%u, ring_cnt=%d)",
                         (unsigned long long)pts, spd, ctx->stats.drop_frames,
                         ctx->kf_pts_ring_count);
                }
                return;
            }
        }
    }

    /* ================================================================
     * 音视频同步 (v3.0 — 高倍速视频 PTS 锚定策略)
     *
     * 问题 (v2.0):
     *   音频时钟公式 clock = base_pts + delta * speed 中, delta 是 wall clock
     *   时间差. 在 ≥4x 倍速下, 视频只解码 I 帧, I 帧之间的 wall clock 间隔
     *   仅 10~50ms (跳过了 P/B 帧解码), 即使乘以 speed=8 也只推进 80~400ms,
     *   而视频 PTS 推进了 2~6 秒 (GOP 间隔). wall clock 时间差远小于 PTS 时间差,
     *   导致 drift 持续累积到约 5.9s 后达到平衡 (max_sleep=600ms 限制).
     *
     * 解决 (v3.0):
     *   1x/2x: 保持墙上时钟插值, 音频时钟 = base_pts + delta * speed.
     *          (全部帧解码, wall clock 帧间隔与 PTS 帧间隔匹配)
     *   ≥4x:  使用视频 PTS 锚定策略:
     *         ① 用 last_displayed_pts 作为音频时钟基准 (替代墙上时钟插值)
     *         ② frame_timer = now + (pts - last_displayed_pts) / speed
     *            帧间隔由 PTS 差/speed 决定, 与 wall clock 解耦
     *         ③ 显示帧后将 audio_clock 锚定到当前帧 PTS,
     *            使下一帧的 frame_timer 正确反映帧间隔
     *         ④ 例: 8x, GOP=2s, 关键帧稀疏化 min_gap=4s
     *            帧1 pts=10s → 显示, adjust_clock(10s)
     *            帧2 pts=14s → audio_clock=10s, frame_timer=now+4/8=now+0.5s
     *            → 等 0.5s 显示, adjust_clock(14s) → 帧间隔 0.5s, 帧率 2fps ✓
     * ================================================================ */
    if (ctx->has_audio && ctx->av_sync_handle && ctx->audio_render_handle) {
        float spd = ctx->time_state.speed;
        int high_speed = (spd >= MAIN_KEYFRAME_ONLY_MIN_SPEED);
        int64_t audio_clock;

        if (high_speed) {
            /* ≥4x 倍速: 视频 PTS 锚定策略.
             * 用 last_displayed_pts 作为音频时钟, 使 frame_timer 计算
             * 与 wall clock 解耦, 仅由 PTS 差和 speed 决定帧间隔. */
            if (ctx->last_displayed_pts > 0) {
                audio_clock = ctx->last_displayed_pts;
            } else {
                /* 首帧: 回退到墙上时钟插值 */
                audio_clock = csg_audio_render_get_clock(ctx->audio_render_handle);
            }
        } else {
            /* 1x/2x: 正常使用音频渲染器的墙上时钟插值 */
            audio_clock = csg_audio_render_get_clock(ctx->audio_render_handle);
        }

        if (audio_clock > 0) {
            csg_av_sync_set_audio_clock(ctx->av_sync_handle, audio_clock);
        }

        int64_t wait_us = 0;
        csg_av_sync_action_t action = csg_av_sync_video_judge(ctx->av_sync_handle,
                                                       (int64_t)pts, &wait_us);

        int64_t drift = (int64_t)pts - audio_clock;

        /* ★ 限频打印: 原来每帧都打 AV-SYNC action 日志, 串口日志风暴
         * 阻塞视频分发线程 (每帧 usleep+printf), 拖慢显示导致卡顿/花屏.
         * 改为每 100 帧统计打印一次, 保留 WAIT/DROP 的可观测性. */
        if (action != CSG_AV_SYNC_ACTION_DISPLAY &&
            (ctx->stats.dec_frames % 100) == 0) {
            CSG_LOGI("MAIN", "AV-SYNC: action=%s pts=%lld audio=%lld drift=%+lldus",
                 action == CSG_AV_SYNC_ACTION_DROP   ? "DROP"   :
                 action == CSG_AV_SYNC_ACTION_WAIT   ? "WAIT"   :
                 action == CSG_AV_SYNC_ACTION_REPEAT ? "REPEAT" : "?",
                 (long long)pts, (long long)audio_clock, (long long)drift);
        }

        if (ctx->stats.dec_frames % 50 == 0) {
            CSG_LOGI("MAIN", "AV-SYNC: pts=%lld audio=%lld drift=%+lldus",
                 (long long)pts, (long long)audio_clock, (long long)drift);
        }

        switch (action) {
        case CSG_AV_SYNC_ACTION_DROP:
            /* 关键帧绝对不能丢: 丢 I 帧会导致后续所有 P 帧无法解码, 直接花屏.
             * 改为 DISPLAY 强制显示, 宁可短暂不同步也不能丢关键帧.
             *
             * ★ 高倍速: DROP blocked 时不再 adjust_clock.
             *    v3.0 策略在帧显示后统一做 PTS 锚定, 此处干预会破坏
             *    last_displayed_pts 锚定链, 导致 frame_timer 计算错误. */
            if (is_keyframe) {
                CSG_LOGI("MAIN", "AV-SYNC: DROP blocked for keyframe pts=%lld "
                     "(drift=%+lldus)", (long long)pts, (long long)drift);
                break;  /* 走 DISPLAY 路径 */
            }
            ctx->stats.drop_frames++;
            usleep(50000);
            return;
        case CSG_AV_SYNC_ACTION_WAIT:
            /* ★ 高倍速动态上限: AV-SYNC 已按 speed 缩放 wait_us,
             *   此处只做绝对上限保护 (防止异常值卡死).
             *   v3.0 视频 PTS 锚定下 wait_us = (pts - last_pts) / speed,
             *   4x GOP=2s → wait=500ms, 16x GOP=2s → wait=125ms.
             *   上限 600ms 覆盖极端场景 (4x + GOP=2.4s). */
            {
                int64_t max_wait = 600000;
                if (wait_us > max_wait) wait_us = max_wait;
            }
            if (wait_us > 3000) {
                usleep((unsigned int)wait_us);

                /* ★ 等待后重新校验 drift: 防止过冲
                 * 等待期间音频时钟持续推进, 若已追平或超过视频 PTS,
                 * 记录过冲日志. */
                int64_t audio_clock_after = csg_audio_render_get_clock(
                    ctx->audio_render_handle);
                if (audio_clock_after > 0) {
                    int64_t drift_after = (int64_t)pts - audio_clock_after;
                    /* ★ 限频: 原来每帧打 over-shoot, 日志风暴, 改每 100 帧 */
                    if (drift_after < 0 && (ctx->stats.dec_frames % 100) == 0) {
                        CSG_LOGI("MAIN", "AV-SYNC: over-shoot wait=%lldus "
                             "drift_after=%+lldus",
                             (long long)wait_us, (long long)drift_after);
                    }
                }
            }
            break;
        case CSG_AV_SYNC_ACTION_REPEAT:
            return;
        case CSG_AV_SYNC_ACTION_DISPLAY:
        default:
            break;
        }

        /* ★ 2x 倍速: AV-SYNC 之后钳制, 仅当 audio_clock 超前 > 100ms 时触发.
         * 2x 全部帧解码, 帧间隔小, 正常同步精度下 drift 在 50~70ms,
         * 若 MPP 解码吞吐量不足会导致音频时钟逐步超前, 积累到 100ms 时钳制. */
        if (spd >= 1.99f && spd <= 2.01f) {
            int64_t cur_audio_clock = csg_audio_render_get_clock(
                ctx->audio_render_handle);
            if (cur_audio_clock > (int64_t)pts + 100000) {
                csg_audio_render_adjust_clock(ctx->audio_render_handle,
                                              (int64_t)pts);
            }
        }

        /* ★ ≥4x 倍速: 显示帧后将音频时钟锚定到当前帧 PTS.
         * 这是 v3.0 视频 PTS 锚定策略的核心: 每帧显示后更新锚点,
         * 使下一帧的 frame_timer = now + (next_pts - pts) / speed
         * 正确反映帧间隔, 与 wall clock 完全解耦.
         * 注意: 必须在 AV-SYNC 判断之后执行, 否则当前帧的 frame_timer
         * 计算会使用自己的 PTS (frame_timer = now + 0 = now → 立即显示). */
        if (high_speed && ctx->audio_render_handle) {
            csg_audio_render_adjust_clock(ctx->audio_render_handle, (int64_t)pts);
        }

        /* ★ v5.6 显示残差观测: 走到这里说明本帧即将上屏.
         * 立即读一次 audible 主时钟, resid = pts - audible 即为"上屏瞬间
         * 画面相对可听音频的偏差" (正 = 画面超前声音). 限频打印防风暴. */
        /* ★ v5.6 显示残差观测 (限频): 走到这里说明本帧即将上屏.
         * resid = pts - audible 为上屏瞬间画面相对可听音频的偏差
         * (正 = 画面超前声音), 期望稳态在 ±30ms 内小幅波动. */
        if (ctx->stats.dec_frames % 50 == 0 && ctx->audio_render_handle) {
            int64_t audible_now = csg_audio_render_get_clock(
                ctx->audio_render_handle);
            if (audible_now > 0) {
                CSG_LOGI("MAIN", "AV-SYNC show: pts=%lld audible=%lld "
                     "resid=%+lldus wait=%lld",
                     (long long)pts, (long long)audible_now,
                     (long long)((int64_t)pts - audible_now),
                     (long long)wait_us);
            }
        }
    } else if (ctx->stats.dec_frames % 50 == 0) {
        CSG_LOGI("MAIN", "AV-SYNC: no audio (pts=%lld)", (long long)pts);
    }

    /* 进度采样 */
    if (pts > 0) {
        if (was_post_resume_sync) {
            int64_t progress_pts;
            if (ctx->resume_min_pts > 0) {
                progress_pts = (int64_t)pts;
            } else {
                progress_pts = (ctx->last_keyframe_pts > 0)
                                   ? ctx->last_keyframe_pts : (int64_t)pts;
            }
            ctx->time_state.ts = progress_pts;
            csg_lvgl_control_update_progress_forced(progress_pts,
                                                ctx->time_state.length,
                                                ctx->time_state.live,
                                                1);
        } else {
            ctx->time_state.ts = (int64_t)pts;
            csg_lvgl_control_update_progress(ctx->time_state.ts,
                                         ctx->time_state.length,
                                         ctx->time_state.live);
        }
    }

    if (ctx->stats.dec_frames <= 10 || ctx->stats.dec_frames % 100 == 0) {
        CSG_LOGI("MAIN", "MPP out frame #%u %dx%d pts=%llu",
             ctx->stats.dec_frames, width, height, (unsigned long long)pts);
    }

    /* ===== 持久 RGB 缓冲 + RGA 硬件缩放转换 (自适应屏幕) =====
     * 视频分辨率与屏幕不匹配时, 用 RGA imresize 硬件缩放 NV12→RGB,
     * 避免把原始大图交给 LVGL 软件缩放 (lv_image_set_scale) 导致
     * CPU 高负载 → 鼠标卡顿, 以及原尺寸超出屏幕 → 花屏. */
    int scr_w = 1920, scr_h = 1080;
    csg_lvgl_display_get_screen_size(&scr_w, &scr_h);

    /* 保持宽高比缩放到屏幕内, 居中黑边由 LVGL present 处理 */
    double scale = (double)scr_w / (double)width;
    if ((double)scr_h / (double)height < scale)
        scale = (double)scr_h / (double)height;
    int disp_w = (int)(width * scale);
    int disp_h = (int)(height * scale);
    if (disp_w < 2) disp_w = 2;
    if (disp_h < 2) disp_h = 2;
    disp_w &= ~1;   /* RGA 目标偶数对齐 */
    disp_h &= ~1;

    int rgb_size = disp_w * disp_h * 4;
    if (ctx->rgb_buf_size < rgb_size) {
        free(ctx->rgb_buf);
        ctx->rgb_buf = malloc(rgb_size);
        ctx->rgb_buf_size = ctx->rgb_buf ? rgb_size : 0;
    }
    if (!ctx->rgb_buf) {
        CSG_LOGE("MAIN", "RGB alloc failed: req=%d", rgb_size);
        ctx->stats.drop_frames++;
        return;
    }

    int ret = csg_rga_convert_nv12_to_rgb_scaled(yuv_data, width, height,
                                      hor_stride, ver_stride,
                                      ctx->rgb_buf, disp_w, disp_h,
                                      RK_FORMAT_BGRA_8888);
    if (ret == 0) {
        if (csg_lvgl_display_is_ready()) {
            csg_lvgl_display_update_video(ctx->rgb_buf, disp_w, disp_h);
            ctx->stats.disp_frames++;
            if (pts > 0 && ctx->state == CSG_PLAYER_STATE_PLAYING) {
                ctx->last_displayed_pts = (int64_t)pts;
            }
            if (ctx->time_state.seeking) {
                ctx->time_state.seeking = 0;
            }
        }
    } else {
        ctx->stats.drop_frames++;
    }

    /* 更新统计 */
    struct timespec ts_now;
    clock_gettime(CLOCK_MONOTONIC, &ts_now);
    double elapsed = (ts_now.tv_sec - s_csg_player_stats_time.tv_sec) +
                     (ts_now.tv_nsec - s_csg_player_stats_time.tv_nsec) / 1e9;
    if (elapsed >= 1.0) {
        uint32_t frames = ctx->stats.disp_frames;
        uint64_t bytes = ctx->stats.recv_bytes;
        ctx->stats.fps = (float)(frames / elapsed);
        uint64_t raw_bitrate = (bytes * 8) / (uint64_t)elapsed;
        ctx->stats.bitrate = (uint32_t)(raw_bitrate / 1000);
        ctx->stats.disp_frames = 0;
        ctx->stats.recv_bytes = 0;
        clock_gettime(CLOCK_MONOTONIC, &s_csg_player_stats_time);
        csg_lvgl_control_update_stats(&ctx->stats);
    }
}

/* ==================== 本地播放事件回调 ==================== */
static void csg_player_local_event_cb(csg_local_event_t event, void *user_data)
{
    csg_player_app_ctx_t *ctx = (csg_player_app_ctx_t *)user_data;
    if (!ctx) return;

    switch (event) {
    case CSG_LOCAL_EVENT_EOF:
        CSG_LOGI("MAIN", "Local video EOF");
        if (ctx->is_local_play) {
            ctx->state = CSG_PLAYER_STATE_COMPLETED;
            ctx->time_state.paused = 1;
            ctx->time_state.pause_date = csg_player_now_us();
            lv_async_call(csg_player_local_eof_ui_async, ctx);
            CSG_LOGI("MAIN", "Entered COMPLETED state (resources preserved for seek)");
        }
        break;
    case CSG_LOCAL_EVENT_SEEK_DONE:
        CSG_LOGI("MAIN", "Local video seek done");
        if (ctx->is_local_play && ctx->local_handle >= 0 && ctx->audio_render_handle) {
            int64_t kf_pts = csg_local_playback_get_last_seek_kf_pts(ctx->local_handle);
            if (kf_pts > 0) {
                /* 修正音频时钟: 以视频关键帧实际 PTS 为基准,
                 * 消除 target 与关键帧 PTS 的天然偏差,
                 * 使音视频从同一位置起步. */
                csg_audio_render_adjust_clock(ctx->audio_render_handle, kf_pts);
                ctx->audio_seek_pts = kf_pts;
                ctx->audio_sync_base_pts = kf_pts;
                CSG_LOGI("MAIN", "Audio clock synced to keyframe PTS=%lldus", (long long)kf_pts);
            }
        }
        break;
    case CSG_LOCAL_EVENT_ERROR:
        CSG_LOGE("MAIN", "Local video playback error");
        if (ctx->is_local_play) {
            ctx->is_local_play = 0;
            ctx->state = CSG_PLAYER_STATE_IDLE;
            if (ctx->local_handle >= 0) {
                csg_local_playback_force_stop_thread(ctx->local_handle);
            }
            lv_async_call(csg_player_local_error_cleanup_async, ctx);
        }
        break;
    default:
        break;
    }
}

static void csg_player_local_eof_ui_async(void *data)
{
    csg_player_app_ctx_t *ctx = (csg_player_app_ctx_t *)data;
    if (!ctx) return;

    csg_lvgl_display_set_paused(1);
    csg_lvgl_control_set_paused_ui(1);
    csg_lvgl_control_set_state(CSG_PLAYER_STATE_COMPLETED);
    csg_lvgl_control_set_live(0);

    /* EOF 时进度条直接冻结到视频总时长(100%),
     * 避免倍速下 last_displayed_pts 与 length 不一致导致
     * 进度条不显示在末尾的问题. */
    int64_t eof_pts = ctx->time_state.length;
    if (eof_pts <= 0 || eof_pts == CSG_PLAYER_TICK_INVALID) {
        eof_pts = ctx->last_displayed_pts > 0
                      ? ctx->last_displayed_pts
                      : ctx->time_state.ts;
    }
    csg_lvgl_control_freeze_progress(eof_pts);

    csg_lvgl_control_local_eof();
    CSG_LOGI("MAIN", "EOF UI async applied (COMPLETED state)");
}

static void csg_player_local_error_cleanup_async(void *data)
{
    csg_player_app_ctx_t *ctx = (csg_player_app_ctx_t *)data;
    if (!ctx) return;

    CSG_LOGI("MAIN", "Async cleanup: playback error");
    csg_lvgl_control_show_message("Playback error");

    /* ★ 崩溃修复: 先停分发线程再销毁音频管线 (防 use-after-free) */
    csg_player_stop_demux_buffer(ctx);

    if (ctx->local_handle >= 0) {
        csg_local_playback_cleanup_sync(ctx->local_handle);
        ctx->local_handle = -1;
    }
    csg_player_teardown_audio_pipeline(ctx);
    if (ctx->mpp_handle) {
        csg_mpp_decode_stop(ctx->mpp_handle);
        csg_mpp_decode_destroy(ctx->mpp_handle);
        ctx->mpp_handle = NULL;
    }
    memset(&ctx->time_state, 0, sizeof(ctx->time_state));
    ctx->time_state.rate = 1.0f;
    ctx->time_state.speed = 1.0f;
    ctx->time_state.length = CSG_PLAYER_TICK_INVALID;
    ctx->time_state.system_date = csg_player_now_us();
    ctx->time_state.pause_date = CSG_PLAYER_TICK_INVALID;
    ctx->step_frame_pending = 0;
    ctx->live = 0;
    ctx->is_local_play = 0;
    csg_lvgl_display_set_paused(0);
    csg_lvgl_control_set_live(0);
    csg_lvgl_control_set_paused_ui(0);
    ctx->state = CSG_PLAYER_STATE_IDLE;
    csg_lvgl_control_set_state(CSG_PLAYER_STATE_IDLE);
    memset(&ctx->stats, 0, sizeof(ctx->stats));
}

/* ==================== 播放器控制 ==================== */
static void csg_player_pause(csg_player_app_ctx_t *ctx)
{
    if (ctx->state != CSG_PLAYER_STATE_PLAYING) return;

    int64_t displayed_pts = ctx->last_displayed_pts;
    int64_t decoded_pts = ctx->time_state.ts;
    int64_t frozen_pts = (displayed_pts > 0) ? displayed_pts : decoded_pts;
    CSG_LOGI("MAIN", "Pause: decoded_pts=%lld, displayed_pts=%lld, frozen_pts=%lld",
         (long long)decoded_pts, (long long)displayed_pts, (long long)frozen_pts);

    ctx->last_displayed_pts = frozen_pts;

    int64_t now = csg_player_now_us();
    ctx->time_state.paused = 1;
    ctx->time_state.pause_date = now;
    ctx->time_state.system_date = now;
    ctx->time_state.ts = frozen_pts;

    csg_lvgl_display_set_paused(1);

    ctx->state = CSG_PLAYER_STATE_PAUSED;
    csg_lvgl_control_set_paused_ui(1);
    csg_lvgl_control_set_state(CSG_PLAYER_STATE_PAUSED);

    if (ctx->is_local_play && ctx->local_handle >= 0) {
        csg_local_playback_pause(ctx->local_handle);
    }

    if (ctx->audio_render_handle) {
        csg_audio_render_pause(ctx->audio_render_handle);
    }

    ctx->resume_min_pts = frozen_pts;
    csg_lvgl_control_freeze_progress(frozen_pts);

    CSG_LOGI("MAIN", "Paused (DPB intact, no drain, resume_min=%lld)",
         (long long)frozen_pts);
}

static void csg_player_resume(csg_player_app_ctx_t *ctx)
{
    if (ctx->state != CSG_PLAYER_STATE_PAUSED && ctx->state != CSG_PLAYER_STATE_COMPLETED) return;

    int64_t resume_from_pos = -1;
    if (ctx->state == CSG_PLAYER_STATE_COMPLETED) {
        resume_from_pos = 0;
        CSG_LOGI("MAIN", "Resume from COMPLETED: restart from beginning");
    }

    ctx->time_state.paused = 0;
    ctx->time_state.pause_date = CSG_PLAYER_TICK_INVALID;
    ctx->time_state.system_date = csg_player_now_us();
    ctx->time_state.ts = ctx->last_displayed_pts;

    ctx->state = CSG_PLAYER_STATE_PLAYING;

    if (ctx->is_local_play && ctx->local_handle >= 0) {
        if (resume_from_pos == 0) {
            if (ctx->mpp_handle) {
                csg_mpp_decode_reset(ctx->mpp_handle);
                ctx->mpp_needs_keyframe = 1;
                ctx->mpp_kf_wait_decode = 0;
            }
            ctx->post_resume_sync = 0;
            ctx->resume_keyframe_seen = 0;
            csg_local_playback_seek(ctx->local_handle, 0.0);

            if (ctx->audio_dec_handle) {
                csg_audio_decode_reset(ctx->audio_dec_handle);
            }
            if (ctx->audio_render_handle) {
                csg_audio_render_flush(ctx->audio_render_handle, 0);
                csg_audio_render_resume(ctx->audio_render_handle);
            }
        } else {
            ctx->post_resume_sync = 1;
            ctx->seek_target_pts = 0;
            ctx->resume_keyframe_seen = 0;
            csg_local_playback_resume(ctx->local_handle, 0);
            CSG_LOGI("MAIN", "Resume: post-resume-sync (filter pre-pause frames, min_pts=%lld)",
                 (long long)ctx->resume_min_pts);

            if (ctx->audio_render_handle) {
                csg_audio_render_resume(ctx->audio_render_handle);
            }
        }
    }

    csg_lvgl_display_set_paused(0);
    csg_lvgl_control_set_paused_ui(0);
    csg_lvgl_control_set_state(CSG_PLAYER_STATE_PLAYING);
    CSG_LOGI("MAIN", "Resumed (state=%d, last_disp=%lld)",
         ctx->state, (long long)ctx->last_displayed_pts);
}

static void csg_player_step_frame(csg_player_app_ctx_t *ctx)
{
    if (ctx->state != CSG_PLAYER_STATE_PAUSED) return;
    if (!ctx->mpp_handle) return;

    ctx->step_frame_pending = 1;
    CSG_LOGI("MAIN", "Step frame");
}

static void csg_player_seek(csg_player_app_ctx_t *ctx, double position)
{
    if (ctx->time_state.live) {
        CSG_LOGI("MAIN", "Seek ignored (live stream)");
        return;
    }
    if (position < 0.0) position = 0.0;
    if (position > 1.0) position = 1.0;

    int64_t target = (ctx->time_state.length > 0)
                         ? (int64_t)(position * ctx->time_state.length) : 0;

    CSG_LOGI("MAIN", "Seek to %.1f%% (t=%lldus)", position * 100.0,
         (long long)target);

    csg_lvgl_display_set_paused(1);
    ctx->time_state.seeking = 1;
    ctx->time_state.seek_pos = position;
    ctx->time_state.seek_ts = target;
    csg_lvgl_control_seek_instant(position, target);

    /* ★ v3.1: post_resume_sync 和 audio_sync_base_pts 必须在 MPP reset 之前设置.
     * 竞态窗口: MPP reset 期间, 回调线程可能仍在处理旧帧 (Reset 前已解码完成,
     * 但回调尚未执行). 若 post_resume_sync=0, 旧帧被放行, 调用
     * csg_lvgl_control_update_progress 清除 seeking 标志并更新进度为旧位置,
     * 导致进度条"弹回"旧位置再跳回新位置.
     *
     * 提前设置后, 旧帧进入 post_resume_sync 块, 被 PTS 范围检查 (>10秒) 丢弃,
     * 不会污染进度条状态. */
    ctx->post_resume_sync = 1;
    ctx->seek_target_pts = target;
    ctx->resume_keyframe_seen = 0;
    ctx->resume_min_pts = 0;
    ctx->seek_drop_count = 0;
    ctx->seek_fast_feed_done = 0;
    ctx->waiting_audio_ready = 0;
    ctx->audio_wait_frame_count = 0;
    ctx->audio_seek_pts = target;
    ctx->audio_sync_base_pts = target;

    ctx->mpp_needs_keyframe = 1;
    ctx->mpp_kf_wait_decode = 0;
    ctx->expected_kf_pts = 0;
    ctx->last_keyframe_pts = 0;
    ctx->last_fed_kf_pts = 0;
    ctx->kf_pts_ring_head = 0;
    ctx->kf_pts_ring_count = 0;
    /* ★ Seek: 清空解封装→解码缓冲队列, 丢弃旧位置残留帧 (代次++ 让解封装
     * 线程的 push 放弃旧数据, 回主循环响应 seek 定位). */
    if (ctx->demux_buf) {
        csg_demux_buffer_flush(ctx->demux_buf);
    }
    if (ctx->mpp_handle) {
        csg_mpp_decode_reset(ctx->mpp_handle);

        if (ctx->is_local_play && ctx->local_handle >= 0) {
            const uint8_t *edata = NULL;
            int esize = 0;
            if (csg_local_playback_get_extradata(ctx->local_handle, &edata, &esize) == 0) {
                csg_mpp_decode_set_extra_data(ctx->mpp_handle, edata, esize);
            } else {
                CSG_LOGW("MAIN", "Seek: no extradata to inject (raw stream, rely on in-band PS)");
            }
        }
    }

    if (ctx->audio_dec_handle) {
        csg_audio_decode_reset(ctx->audio_dec_handle);
    }
    if (ctx->audio_render_handle) {
        /* Seek 时音频时钟归零, 等待 SEEK_DONE 事件中用关键帧 PTS 修正.
         * 不在此处预设 target, 因为 target 与视频关键帧 PTS 天然存在偏差,
         * 导致音频时钟超前视频, 永远无法同步. */
        csg_audio_render_flush(ctx->audio_render_handle, 0);
        csg_audio_render_resume(ctx->audio_render_handle);
    }

    /* audio_seek_pts 在 SEEK_DONE 事件中设置, 以关键帧 PTS 为基准.
     * 此处先设为 target 作为兜底, 防止极限情况下 SEEK_DONE 未触发. */
    ctx->audio_seek_pts = target;
    ctx->audio_sync_base_pts = target;

    /* ★ Seek 后重置 frame_timer: 清除旧帧的节奏记忆,
     * 否则新位置第一帧可能等待异常长的时间. */
    if (ctx->av_sync_handle) {
        csg_av_sync_reset_frame_timer(ctx->av_sync_handle);
    }

    if (ctx->state == CSG_PLAYER_STATE_COMPLETED) {
        ctx->state = CSG_PLAYER_STATE_PLAYING;
        ctx->time_state.paused = 0;
        ctx->time_state.pause_date = CSG_PLAYER_TICK_INVALID;
        ctx->time_state.system_date = csg_player_now_us();
        ctx->time_state.ts = target;
        csg_lvgl_control_set_paused_ui(0);
        csg_lvgl_control_set_state(CSG_PLAYER_STATE_PLAYING);
        CSG_LOGI("MAIN", "Seek from COMPLETED: resumed PLAYING state");
    }

    if (ctx->is_local_play && ctx->local_handle >= 0) {
        csg_local_playback_seek(ctx->local_handle, position);
    }
}

static void csg_player_step(csg_player_app_ctx_t *ctx, int64_t delta_us)
{
    if (ctx->time_state.live) return;

    int64_t cur = ctx->time_state.ts;
    int64_t length = ctx->time_state.length;
    if (length <= 0 || length == CSG_PLAYER_TICK_INVALID) return;

    int64_t target = cur + delta_us;
    if (target < 0) target = 0;
    if (target > length) target = length;

    double pos = (double)target / (double)length;
    csg_player_seek(ctx, pos);
}

static void csg_player_set_speed(csg_player_app_ctx_t *ctx, float speed)
{
    if (speed < 0.5f || speed > 16.0f) return;
    float old_speed = ctx->time_state.speed;
    ctx->time_state.speed = speed;

    int64_t now = csg_player_now_us();
    int64_t drift_before = (now - ctx->time_state.system_date) * (int64_t)old_speed;
    int64_t new_system_date = now - drift_before / (int64_t)speed;
    ctx->time_state.system_date = new_system_date;

    csg_lvgl_control_set_speed(speed);

    if (ctx->av_sync_handle) {
        csg_av_sync_set_speed(ctx->av_sync_handle, speed);
    }

    if (ctx->is_local_play && ctx->local_handle >= 0) {
        csg_local_playback_set_speed(ctx->local_handle, speed);
    }

    /* ★ 同步更新音频渲染器的倍速因子, 使音频时钟以倍速推进.
     * 缺失此调用会导致 2x 下音频时钟仍以 1x 推进,
     * drift 线性增长到数秒. */
    if (ctx->audio_render_handle) {
        /* ★ 跨越 4x 边界时, 先校准时钟再 flush 环形缓冲区:
         *   ① 先用视频 PTS 调整时钟锚点 (adjust_clock).
         *   ② flush 清空旧 PCM 数据, 传入 correct_pts 保留时钟锚点.
         *   ③ set_speed 用当前时钟值重新校准锚点.
         *
         * 顺序必须是 adjust→flush→set_speed:
         *   flush 会重置 base_pts, 但 new_base_pts>0 时会保留;
         *   set_speed 用 current_clock 覆盖 base_pts, 必须最后调用.
         *
         * 若不校准直接 flush(0), base_pts=0 → set_speed 时
         * current_clock=0 → 锚点不更新 → audio_clock 从 0 开始以 4x 推进
         * → 系统 uptime×4 秒的巨大值 → 全部帧 WAIT/DROP 循环. */
        int cross_to_high = (old_speed < MAIN_KEYFRAME_ONLY_MIN_SPEED
                             && speed >= MAIN_KEYFRAME_ONLY_MIN_SPEED);
        int cross_to_low = (old_speed >= MAIN_KEYFRAME_ONLY_MIN_SPEED
                            && speed < MAIN_KEYFRAME_ONLY_MIN_SPEED);
        if (cross_to_high || cross_to_low) {
            int64_t correct_pts = ctx->last_displayed_pts > 0
                                  ? ctx->last_displayed_pts : 0;
            if (correct_pts > 0) {
                csg_audio_render_adjust_clock(ctx->audio_render_handle, correct_pts);
            }
            csg_audio_render_flush(ctx->audio_render_handle, correct_pts);
            CSG_LOGI("MAIN", "Speed cross %dx→%dx: flush audio ring (base_pts=%lld)",
                 old_speed >= MAIN_KEYFRAME_ONLY_MIN_SPEED ? 4 : (int)old_speed,
                 speed >= MAIN_KEYFRAME_ONLY_MIN_SPEED ? 4 : (int)speed,
                 (long long)correct_pts);
        }

        csg_audio_render_set_speed(ctx->audio_render_handle, speed);

        /* ★ 从高倍速切回低倍速时, 音频时钟在高速下已严重超前视频 PTS,
         * csg_audio_render_set_speed 保留旧 current_clock 作为新 base_pts,
         * 导致切回 1x 后 audio_clock 比视频 PTS 大 48~60 秒 → 全部帧 DROP.
         *
         * 修复: 用当前视频帧 PTS 重置音频时钟, 消除累积差距.
         * 注意: adjust_clock 必须在 set_speed 之后调用, 否则会被覆盖. */
        if (old_speed > speed + 0.5f && ctx->last_displayed_pts > 0) {
            csg_audio_render_adjust_clock(ctx->audio_render_handle,
                                          ctx->last_displayed_pts);
            CSG_LOGI("MAIN", "Speed down: audio clock clamped to video PTS=%lld",
                 (long long)ctx->last_displayed_pts);
        }
    }

    /* ★ 从低倍速 (<4x) 切换到高倍速 (≥4x) 时, MPP 管道中残留的 P 帧
     * 缺少参考帧会导致花屏. 必须 reset 解码器清空 DPB,
     * 后续只送 I 帧, 确保画面干净无花屏.
     * 反向切换 (高→低) 不需要 reset, 因为 P/B 帧恢复送入后
     * 从下一个 I 帧开始有完整参考链. */
    if (old_speed < MAIN_KEYFRAME_ONLY_MIN_SPEED && speed >= MAIN_KEYFRAME_ONLY_MIN_SPEED
        && ctx->mpp_handle) {
        csg_mpp_decode_reset(ctx->mpp_handle);
        ctx->mpp_needs_keyframe = 1;
        ctx->mpp_kf_wait_decode = 0;
        /* 清除关键帧 PTS 环形缓冲区, 避免旧帧 PTS 干扰新倍速下的验证 */
        ctx->kf_pts_ring_head = 0;
        ctx->kf_pts_ring_count = 0;
        ctx->last_fed_kf_pts = 0;
        CSG_LOGI("MAIN", "Speed up to %.1fx: MPP reset to flush residual P-frames, "
             "waiting for I-frame", speed);
    }

    CSG_LOGI("MAIN", "Playback speed: %.2fx -> %.2fx", old_speed, speed);
}

/* ==================== RTSP 拉流 / 录制 辅助 ==================== */

/* 生成录制文件名: /userdata/records/YYYYMMDD_HHMMSS<suffix>.mp4
 * suffix 用于区分两路录制: 拉流自动录制 "_auto", 手动录制 "_manual".
 * 历史踩坑点: 板端 /userdata 可能不存在该子目录, 必须先 mkdir -p,
 * 否则 MP4v2 打开输出文件失败导致录制静默无效. */
static void csg_player_make_record_path_into(char *path, int path_size,
                                             const char *suffix)
{
    if (mkdir(LOCAL_VIDEO_DIR, 0755) != 0 && errno != EEXIST) {
        CSG_LOGW("MAIN", "mkdir %s failed: %s", LOCAL_VIDEO_DIR, strerror(errno));
    }

    time_t t = time(NULL);
    struct tm tm;
    localtime_r(&t, &tm);
    snprintf(path, path_size,
             "%s/%04d%02d%02d_%02d%02d%02d%s.mp4",
             LOCAL_VIDEO_DIR,
             tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
             tm.tm_hour, tm.tm_min, tm.tm_sec,
             suffix ? suffix : "");
}

/* 停止单路录制 (幂等): 先停 MP4v2 录制器 (等队列排空, 落盘 moov),
 * 再断开该路录制主码流. 两路录制各自独立, 只影响传入的 rec.
 * manual_rec 停止时同步刷新 REC 按钮状态. */
static void csg_player_rec_stop(csg_player_app_ctx_t *ctx, csg_rec_ctx_t *rec,
                                const char *tag)
{
    void *rec_h;

    pthread_mutex_lock(&ctx->record_mutex);
    rec->start_pending = 0;
    rec->recording = 0;
    rec_h = rec->record_handle;
    rec->record_handle = NULL;
    pthread_mutex_unlock(&ctx->record_mutex);

    if (rec_h) {
        mp4_record_stop(rec_h);
        mp4_record_destroy(rec_h);
        CSG_LOGI("MAIN", "[%s] Recording stopped: %s", tag, rec->record_path);
    }
    if (rec == &ctx->manual_rec)
        csg_lvgl_control_set_recording(0);
}

/* 停止 RTSP 拉流 (幂等): 停两路录制 + 停预览子码流 */
static void csg_player_stop_rtsp(csg_player_app_ctx_t *ctx)
{
    csg_player_rec_stop(ctx, &ctx->auto_rec, "AUTO");
    csg_player_rec_stop(ctx, &ctx->manual_rec, "MANUAL");

    if (ctx->rtsp_handle) {
        /* ★ v5.6: 停止前诊断音频帧接收情况.
         * 若 SDP 声明了音频轨但始终未收到音频帧, 可能是摄像头
         * 在子码流上不发送音频 RTP 数据, 需提示用户切主码流. */
        int acodec = 0, arate = 0, ach = 0, sdp_done = 0;
        csg_rtsp_live555_get_audio_info(ctx->rtsp_handle, &acodec, &arate,
                                        &ach, &sdp_done);
        if (sdp_done && acodec != CSG_AUDIO_CODEC_NONE
            && ctx->audio_frame_count == 0) {
            CSG_LOGW("MAIN", "RTSP audio: SDP declared audio codec=%d %dHz/%dch, "
                 "but 0 audio frames received! "
                 "Camera may not send audio on sub-stream, "
                 "try main stream (/ch01/main/) for recording with audio.",
                 acodec, arate, ach);
        } else if (sdp_done && acodec != CSG_AUDIO_CODEC_NONE) {
            CSG_LOGI("MAIN", "RTSP audio: received %d frames, codec=%d %dHz/%dch",
                 ctx->audio_frame_count, acodec, arate, ach);
        }

        csg_rtsp_live555_stop(ctx->rtsp_handle);
        ctx->rtsp_handle = NULL;
        CSG_LOGI("MAIN", "RTSP stopped");
    }

    /* ★ 崩溃修复: 必须先停 demux 分发线程再销毁音频管线。
     * 历史踩坑: 音频分发线程可能正从队列取帧调用 FAAD2/渲染器,
     * teardown 直接销毁句柄 → use-after-free 崩溃 (切文件必现)。
     * stop_demux_buffer 幂等, 后续 start_demux_buffer 会重建。 */
    csg_player_stop_demux_buffer(ctx);

    /* ★ RTSP 音频渲染器: live555 事件循环线程已 join (音频回调不再运行),
     * 此时销毁 ALSA 渲染器无竞态. 重置懒创建标志, 下次拉流可重建. */
    csg_player_teardown_audio_pipeline(ctx);
    ctx->rtsp_audio_tried = 0;
}

/* 录制主码流帧公共处理: 运行在对应 live555 解码线程.
 * 录制器由主线程 rec_connect 同步创建 (SDP 就绪后), 这里只做推帧,
 * 绝不在 live555 线程做 MP4v2 初始化 (历史踩坑: 首帧懒建触发
 * mp4v2 堆损坏 free(): invalid pointer → 拉流即崩). */
static void csg_player_rec_frame_common(csg_player_app_ctx_t *ctx,
                                        csg_rec_ctx_t *rec, const char *tag,
                                        void *data, int len, int is_keyframe,
                                        uint64_t pts)
{
    (void)tag;
    if (!ctx->running) return;

    pthread_mutex_lock(&ctx->record_mutex);
    if (rec->recording && rec->record_handle) {
        mp4_record_push_frame(rec->record_handle, data, len, is_keyframe, pts);
    }
    pthread_mutex_unlock(&ctx->record_mutex);
}

/* 自动录制 (拉流即录) 主码流回调 */
static void csg_player_on_auto_record_frame(void *data, int len, int is_keyframe,
                                            uint64_t pts, int codec_type,
                                            void *user_data)
{
    csg_player_app_ctx_t *ctx = (csg_player_app_ctx_t *)user_data;
    (void)codec_type;
    csg_player_rec_frame_common(ctx, &ctx->auto_rec, "AUTO",
                                data, len, is_keyframe, pts);
}

/* 手动录制 (点击 REC) 主码流回调 */
static void csg_player_on_manual_record_frame(void *data, int len, int is_keyframe,
                                              uint64_t pts, int codec_type,
                                              void *user_data)
{
    csg_player_app_ctx_t *ctx = (csg_player_app_ctx_t *)user_data;
    (void)codec_type;
    csg_player_rec_frame_common(ctx, &ctx->manual_rec, "MANUAL",
                                data, len, is_keyframe, pts);
}

/* 启动单路录制: 复用预览帧 (预览与录制同源 URL 时, 不再创建第二个 live555
 * 实例 — 双实例事件循环并发会触发 live555 库级堆损坏 free(): invalid pointer).
 * 参数集/分辨率由预览句柄 SDP 确定, 就绪后由 rec_frame_common 懒建录制器. */
static void csg_player_rec_connect(csg_player_app_ctx_t *ctx, csg_rec_ctx_t *rec,
                                   const char *tag,
                                   csg_rtsp_frame_callback cb,
                                   const char *record_url)
{
    (void)cb;
    if (rec->recording) {
        CSG_LOGW("MAIN", "[%s] Already recording", tag);
        return;
    }
    if (rec->record_handle) {
        CSG_LOGW("MAIN", "[%s] Recording already attached", tag);
        return;
    }
    if (!record_url || record_url[0] == '\0') {
        CSG_LOGW("MAIN", "[%s] Cannot record: no record URL", tag);
        return;
    }
    if (!ctx->rtsp_handle) {
        CSG_LOGW("MAIN", "[%s] Cannot record: no preview stream", tag);
        return;
    }

    csg_player_make_record_path_into(rec->record_path,
                                     sizeof(rec->record_path),
                                     (rec == &ctx->manual_rec) ? "_manual" : "_auto");

    /* 参考 rk3572_player-拉流-播放-录制 项目: 录制器由主线程同步创建,
     * 不依赖首帧懒建 (避免在 live555 解码线程里做 MP4 初始化).
     * SDP 参数集在 PLAY 成功后已就绪, 直接组装 extradata 创建录制器. */
    pthread_mutex_lock(&ctx->record_mutex);

    int codec = CSG_CODEC_H264, w = 0, h = 0, sl = 0, pl = 0, vl = 0;
    const uint8_t *sps = NULL, *pps = NULL, *vps = NULL;
    int gri = csg_rtsp_live555_get_raw_info(ctx->rtsp_handle, &codec, &w, &h,
                                            &sps, &sl, &pps, &pl, &vps, &vl);
    CSG_LOGI("MAIN", "[%s] get_raw_info=%d codec=%d %dx%d sl=%d pl=%d vl=%d",
             tag, gri, codec, w, h, sl, pl, vl);

    /* SDP 未就绪 (DESCRIBE/SETUP/PLAY 尚未完成, get_raw_info 返回 -1):
     * 主线程轮询等待 SDP 就绪后再同步创建录制器.
     * ★ 绝不依赖首帧懒建 — 首帧回调运行在 live555 解码线程, 在其中做
     *   MP4v2 初始化是历史崩溃根因 (堆损坏 free(): invalid pointer).
     * ★ 历史踩坑: 仅等 w/h>0 会在 SETUP 完成前 (sprop 尚未解析) 就创建
     *   录制器, 导致 extradata 无 SPS/PPS, 录制线程又因帧间 SPS/PPS 分布
     *   不稳定 (时有时无) 提取不全 → h26x_param_sets_ready 永远为 0,
     *   录制文件 avcC/hvcC 空, 无法解码播放. 必须等参数集就绪再创建. */
    int params_ready = 0;
    if (!(gri == 0 && w > 0 && h > 0)) {
        int waited_ms = 0;
        while (waited_ms < 2000) {
            usleep(50 * 1000);
            waited_ms += 50;
            gri = csg_rtsp_live555_get_raw_info(ctx->rtsp_handle, &codec, &w, &h,
                                                &sps, &sl, &pps, &pl, &vps, &vl);
            if (gri == 0 && w > 0 && h > 0) {
                /* 参数集就绪: H264 需 SPS+PPS; H265 需 VPS+SPS+PPS */
                params_ready = (codec == CSG_CODEC_H265) ?
                    (vl > 0 && sl > 0 && pl > 0) : (sl > 0 && pl > 0);
                if (params_ready) break;
            }
        }
        CSG_LOGI("MAIN", "[%s] SDP wait done after %dms (gri=%d %dx%d sps=%d pps=%d vps=%d)",
                 tag, waited_ms, gri, w, h, sl, pl, vl);
    } else {
        /* 首次 get_raw_info 已就绪, 检查参数集是否同步就绪 */
        params_ready = (codec == CSG_CODEC_H265) ?
            (vl > 0 && sl > 0 && pl > 0) : (sl > 0 && pl > 0);
        if (!params_ready) {
            int waited_ms = 0;
            while (waited_ms < 2000) {
                usleep(50 * 1000);
                waited_ms += 50;
                gri = csg_rtsp_live555_get_raw_info(ctx->rtsp_handle, &codec, &w, &h,
                                                    &sps, &sl, &pps, &pl, &vps, &vl);
                params_ready = (codec == CSG_CODEC_H265) ?
                    (vl > 0 && sl > 0 && pl > 0) : (sl > 0 && pl > 0);
                if (params_ready) break;
            }
        }
    }

    if (gri == 0 && w > 0 && h > 0) {
        int fps = (ctx->rtsp_fps > 0) ? ctx->rtsp_fps : 25;
        uint8_t extradata[512];
        int ed_len = 0;
        const uint8_t start[4] = {0, 0, 0, 1};
        if (codec == CSG_CODEC_H265 && vl > 0) {
            memcpy(extradata + ed_len, start, 4); ed_len += 4;
            memcpy(extradata + ed_len, vps, (size_t)vl); ed_len += vl;
        }
        if (sl > 0) {
            memcpy(extradata + ed_len, start, 4); ed_len += 4;
            memcpy(extradata + ed_len, sps, (size_t)sl); ed_len += sl;
        }
        if (pl > 0) {
            memcpy(extradata + ed_len, start, 4); ed_len += 4;
            memcpy(extradata + ed_len, pps, (size_t)pl); ed_len += pl;
        }

        /* 音频轨参数: SDP 声明的编码/采样率/声道在 audio subsession SETUP 时
         * 即已确定 (与视频参数集几乎同时). 轮询至音频信息明确 (有音频) 或
         * SDP 遍历完成 (sdp_done, 确认无音频), 从而在 mp4_record_start 之前
         * 就把音频轨参数 + ASC 配置好 —— moov 建轨不再依赖首音频帧是否在
         * 录制线程的 3s 窗口内到达 (历史: 部分摄像头 AAC 首帧偏晚 → 文件
         * 无音频轨; 无音频流则 0 阻塞直接仅视频). */
        int acodec = CSG_AUDIO_CODEC_NONE, arate = 0, ach = 0, sdp_done = 0;
        csg_rtsp_live555_get_audio_info(ctx->rtsp_handle, &acodec, &arate,
                                        &ach, &sdp_done);
        int audio_waited_ms = 0;
        while (acodec == CSG_AUDIO_CODEC_NONE && !sdp_done && audio_waited_ms < 1500) {
            usleep(50 * 1000);
            audio_waited_ms += 50;
            csg_rtsp_live555_get_audio_info(ctx->rtsp_handle, &acodec, &arate,
                                            &ach, &sdp_done);
        }
        if (acodec != CSG_AUDIO_CODEC_NONE) {
            CSG_LOGI("MAIN", "[%s] SDP audio: codec=%d %dHz/%dch (waited %dms)",
                     tag, acodec, arate, ach, audio_waited_ms);
        } else {
            CSG_LOGI("MAIN", "[%s] SDP audio: none (sdp_done=%d, waited %dms)",
                     tag, sdp_done, audio_waited_ms);
        }

        void *rec_h = mp4_record_create(rec->record_path, codec, w, h, fps,
                                        ed_len > 0 ? extradata : NULL, ed_len);
        /* 同步带上 SDP 音频配置 (须在 mp4_record_start 前, 录制线程据此建音频轨) */
        if (rec_h && acodec != CSG_AUDIO_CODEC_NONE) {
            if (acodec == CSG_AUDIO_CODEC_AAC) {
                const uint8_t *asc = NULL;
                int asc_size = 0;
                if (csg_rtsp_live555_get_audio_asc(ctx->rtsp_handle,
                                                   &asc, &asc_size) == 0 &&
                    asc && asc_size > 0) {
                    mp4_record_set_audio_asc(rec_h, asc, asc_size);
                }
            }
            if (arate <= 0) arate = 8000;
            if (ach <= 0) ach = 1;
            mp4_record_config_audio(rec_h, acodec, arate, ach);
        }
        if (rec_h && mp4_record_start(rec_h) == 0) {
            rec->record_handle = rec_h;
            rec->recording = 1;
            rec->start_pending = 0;
            CSG_LOGI("MAIN", "[%s] Recording started (sync): %s (%dx%d@%d, codec=%d)",
                     tag, rec->record_path, w, h, fps, codec);
        } else {
            if (rec_h) mp4_record_destroy(rec_h);
            rec->start_pending = 0;
            CSG_LOGW("MAIN", "[%s] Sync create failed, recording skipped", tag);
        }
    } else {
        /* SDP 始终未就绪 (摄像头无响应/拉流失败): 放弃自动录制, 不设
         * start_pending (不再走首帧懒建, 防止在 live555 线程初始化 MP4). */
        rec->start_pending = 0;
        CSG_LOGW("MAIN", "[%s] SDP not ready (gri=%d %dx%d), auto-record skipped",
                 tag, gri, w, h);
    }

    pthread_mutex_unlock(&ctx->record_mutex);

    CSG_LOGI("MAIN", "[%s] Recording (shared preview stream): %s -> %s",
             tag, record_url, rec->record_path);
    CSG_LOGI("MAIN", "[%s] rec_connect done (start_pending=%d recording=%d)",
             tag, rec->start_pending, rec->recording);
}

/* 手动录制入口 (点击 REC): 要求预览已启动, 独立主码流句柄, 与自动录制互不干扰 */
static void csg_player_start_record(csg_player_app_ctx_t *ctx, const char *record_url)
{
    if (!ctx->rtsp_handle) {
        CSG_LOGW("MAIN", "Cannot record: no preview stream");
        csg_lvgl_control_show_message("Start preview first!");
        return;
    }
    if (!record_url || record_url[0] == '\0') {
        CSG_LOGW("MAIN", "Cannot record: no record URL");
        csg_lvgl_control_show_message("Input main-stream URL!");
        return;
    }
    csg_lvgl_control_show_message("Connecting main stream...");
    csg_player_rec_connect(ctx, &ctx->manual_rec, "MANUAL",
                           csg_player_on_manual_record_frame, record_url);
}

/* ==================== 控制命令回调 ==================== */
static void csg_player_on_control_cmd(csg_ctrl_cmd_t cmd, void *data, void *user_data)
{
    csg_player_app_ctx_t *ctx = (csg_player_app_ctx_t *)user_data;

    /* ★ v5.4 重入保护: LVGL 线程中 lv_label_set_text 等操作可能触发
     * 布局事件, 导致 csg_player_on_control_cmd 被重入调用.
     * 重入时 ctx 状态可能处于初始化/销毁中间态, 直接操作会崩溃.
     * 修复: 用 cmd_in_progress 标志阻止重入, 外层调用完成后
     * 内层调用看到的 ctx 状态始终一致. */
    if (ctx->cmd_in_progress) {
        CSG_LOGW("MAIN", "Re-entrant control cmd %d ignored (cmd_in_progress)", cmd);
        return;
    }
    ctx->cmd_in_progress = 1;

    switch (cmd) {
    case CSG_CTRL_CMD_STOP_PREVIEW:
        CSG_LOGI("MAIN", "Stopping playback...");
        /* 先停两路录制 + 拉流 (幂等, 必须先于 MPP 销毁) */
        csg_player_stop_rtsp(ctx);
        if (ctx->local_handle >= 0) {
            csg_local_playback_stop(ctx->local_handle);
            ctx->local_handle = -1;
        }
        /* 停止解封装→解码缓冲层 (须先于 audio/MPP 销毁, 防分发线程访问已释放句柄) */
        csg_player_stop_demux_buffer(ctx);
        if (ctx->mpp_handle) {
            csg_mpp_decode_stop(ctx->mpp_handle);
            csg_mpp_decode_destroy(ctx->mpp_handle);
            ctx->mpp_handle = NULL;
        }
        csg_player_teardown_audio_pipeline(ctx);
        memset(&ctx->time_state, 0, sizeof(ctx->time_state));
        ctx->time_state.rate = 1.0f;
        ctx->time_state.speed = 1.0f;
        ctx->time_state.length = CSG_PLAYER_TICK_INVALID;
        ctx->time_state.system_date = csg_player_now_us();
        ctx->time_state.pause_date = CSG_PLAYER_TICK_INVALID;
        /* ★ 停止播放时重置进度条 */
        csg_lvgl_control_reset_progress();
        ctx->step_frame_pending = 0;
        ctx->live = 0;
        ctx->is_local_play = 0;
        csg_lvgl_display_set_paused(0);
        csg_lvgl_control_set_live(0);
        csg_lvgl_control_set_paused_ui(0);
        ctx->state = CSG_PLAYER_STATE_IDLE;
        csg_lvgl_control_set_state(CSG_PLAYER_STATE_IDLE);
        memset(&ctx->stats, 0, sizeof(ctx->stats));
        ctx->post_resume_sync = 0;
        ctx->resume_keyframe_seen = 0;
        ctx->resume_min_pts = 0;
        ctx->last_keyframe_pts = 0;
        break;

    case CSG_CTRL_CMD_PAUSE:
        csg_player_pause(ctx);
        break;

    case CSG_CTRL_CMD_RESUME:
        csg_player_resume(ctx);
        break;

    case CSG_CTRL_CMD_TOGGLE_PAUSE:
        if (ctx->state == CSG_PLAYER_STATE_PLAYING) {
            csg_player_pause(ctx);
        } else if (ctx->state == CSG_PLAYER_STATE_PAUSED || ctx->state == CSG_PLAYER_STATE_COMPLETED) {
            csg_player_resume(ctx);
        }
        break;

    case CSG_CTRL_CMD_SEEK_POSITION: {
        double *pos = (double *)data;
        if (pos) csg_player_seek(ctx, *pos);
        break;
    }

    case CSG_CTRL_CMD_STEP_FORWARD:
            csg_player_step(ctx, 30 * 1000000LL);
            break;
        case CSG_CTRL_CMD_STEP_BACKWARD:
            csg_player_step(ctx, -30 * 1000000LL);
            break;

    case CSG_CTRL_CMD_NEXT_FRAME:
        csg_player_step_frame(ctx);
        break;

    case CSG_CTRL_CMD_SHOW_CONTROLS:
        csg_lvgl_control_show_bar();
        break;

    case CSG_CTRL_CMD_PLAY_LOCAL: {
        const char *file_path = (const char *)data;
        if (!file_path || file_path[0] == '\0') {
            CSG_LOGW("MAIN", "Empty file path, ignore");
            break;
        }
        CSG_LOGI("MAIN", "Playing local video: %s (old state=%d)", file_path, ctx->state);

        ctx->state = CSG_PLAYER_STATE_IDLE;
        csg_lvgl_control_set_state(CSG_PLAYER_STATE_IDLE);

        /* ★ 崩溃修复: 先停分发线程再销毁音频管线 (防 use-after-free) */
        csg_player_stop_demux_buffer(ctx);

        if (ctx->local_handle >= 0) {
            csg_local_playback_cleanup_sync(ctx->local_handle);
            ctx->local_handle = -1;
        }
        csg_player_teardown_audio_pipeline(ctx);
        if (ctx->mpp_handle) {
            csg_mpp_decode_stop(ctx->mpp_handle);
            csg_mpp_decode_destroy(ctx->mpp_handle);
            ctx->mpp_handle = NULL;
        }
        memset(&ctx->time_state, 0, sizeof(ctx->time_state));
        ctx->time_state.rate = 1.0f;
        ctx->time_state.speed = 1.0f;
        ctx->time_state.length = CSG_PLAYER_TICK_INVALID;
        ctx->time_state.system_date = csg_player_now_us();
        ctx->time_state.pause_date = CSG_PLAYER_TICK_INVALID;

        /* ★ 新视频播放时重置进度条为 0 (切换文件进度条归零) */
        csg_lvgl_control_reset_progress();
        /* ★ 新视频播放时重置 UI 倍速显示为 1x */
        csg_lvgl_control_set_speed(1.0f);
        ctx->step_frame_pending = 0;
        ctx->live = 0;
        ctx->post_resume_sync = 0;
        ctx->resume_keyframe_seen = 0;
        ctx->resume_min_pts = 0;
        ctx->last_keyframe_pts = 0;
        ctx->last_displayed_pts = 0;
        ctx->mpp_needs_keyframe = 0;
        ctx->mpp_kf_wait_decode = 0;

        ctx->is_local_play = 1;
        csg_lvgl_display_set_paused(0);
        csg_lvgl_control_set_live(0);
        csg_lvgl_control_set_paused_ui(0);
        memset(&ctx->stats, 0, sizeof(ctx->stats));
        /* 重置动态帧率计算: 新视频可能 FPS 不同 */
        ctx->fps_prev_pts = 0;
        ctx->fps_calc_count = 0;
        ctx->fps_interval_sum = 0;
        ctx->fps_dynamic_set = 0;        
        clock_gettime(CLOCK_MONOTONIC, &s_csg_player_stats_time);

        ctx->local_handle = csg_local_playback_create(file_path);
        if (ctx->local_handle < 0) {
            CSG_LOGE("MAIN", "Failed to create local playback: %s", file_path);
            ctx->state = CSG_PLAYER_STATE_IDLE;
            csg_lvgl_control_set_state(CSG_PLAYER_STATE_IDLE);
            ctx->is_local_play = 0;
            break;
        }

        csg_local_playback_set_event_callback(ctx->local_handle, csg_player_local_event_cb, ctx);
        csg_local_playback_set_seek_feed_done_ptr(ctx->local_handle,
                                              &ctx->seek_fast_feed_done);

        csg_local_playback_get_video_info(ctx->local_handle,
                                      &ctx->video_width, &ctx->video_height,
                                      &ctx->video_fps, &ctx->codec_type);
        /* ★ 启用 frame_timer 节奏控制器: 确保视频帧按音频时钟节奏等间隔显示 */
        if (ctx->video_fps > 0 && ctx->av_sync_handle) {
            csg_av_sync_set_frame_duration_by_fps(ctx->av_sync_handle,
                                              (float)ctx->video_fps);
        }
        int64_t duration = csg_local_playback_get_duration(ctx->local_handle);
        if (duration > 0) {
            ctx->time_state.length = duration;
            ctx->live = 0;
            CSG_LOGI("MAIN", "Local video: %dx%d@%dfps, duration=%.1f sec",
                 ctx->video_width, ctx->video_height, ctx->video_fps,
                 (double)duration / 1000000.0);
        } else {
            ctx->time_state.length = CSG_PLAYER_TICK_INVALID;
            ctx->live = 1;
            CSG_LOGI("MAIN", "Local video: %dx%d@%dfps, unknown duration",
                 ctx->video_width, ctx->video_height, ctx->video_fps);
        }

        ctx->mpp_handle = csg_mpp_decode_create(ctx->codec_type,
                                            ctx->video_width,
                                            ctx->video_height);
        if (!ctx->mpp_handle) {
            CSG_LOGE("MAIN", "Failed to create MPP decoder for local video");
            csg_local_playback_stop(ctx->local_handle);
            csg_local_playback_cleanup_sync(ctx->local_handle);
            ctx->local_handle = -1;
            ctx->state = CSG_PLAYER_STATE_IDLE;
            csg_lvgl_control_set_state(CSG_PLAYER_STATE_IDLE);
            ctx->is_local_play = 0;
            break;
        }

        ctx->state_gen = csg_player_next_state_gen();
        ctx->mpp_gen = ctx->state_gen;

        ctx->state = CSG_PLAYER_STATE_PLAYING;
        csg_lvgl_control_set_state(CSG_PLAYER_STATE_PLAYING);

        /* ★ 解封装→解码 缓冲层: 在设置解封装回调前拉起视频/音频分发线程,
         * 保证首帧入队时消费者已就绪 (解耦解封装与 MPP/FAAD2 同步阻塞). */
        csg_player_start_demux_buffer(ctx);

        csg_mpp_decode_set_callback(ctx->mpp_handle, csg_player_on_mpp_frame, ctx);
        csg_local_playback_set_callback(ctx->local_handle, csg_player_on_local_frame, ctx);
        csg_local_playback_set_audio_callback(ctx->local_handle, csg_player_on_audio_frame, ctx);

        csg_player_setup_audio_pipeline(ctx, ctx->local_handle);

        /* ★ 新视频播放时强制重置所有层的倍速为 1x.
         * csg_lvgl_control_set_speed(1.0f) 只更新 UI 进度追踪,
         * csg_player_set_speed 才同时更新 audio_render/av_sync/local_playback 三层. */
        csg_player_set_speed(ctx, 1.0f);

        /* ★ 本地播放显式注入参数集 (hvcC/avcC, MPP 内部转 Annex-B):
         * 与 seek 路径保持一致, 首个关键帧前 MPP 即建立 SPS/PPS 表,
         * 避免 H.265 只靠码流内嵌参数集导致 "pps: invalid pps id 0". */
        if (ctx->local_handle >= 0) {
            const uint8_t *edata = NULL;
            int esize = 0;
            if (csg_local_playback_get_extradata(ctx->local_handle, &edata, &esize) == 0
                && esize > 0) {
                csg_mpp_decode_set_extra_data(ctx->mpp_handle, edata, esize);
            }
        }

        csg_local_playback_start(ctx->local_handle);
        break;
    }

    case CSG_CTRL_CMD_STOP_LOCAL: {
        if (!ctx->is_local_play || ctx->local_handle < 0) {
            CSG_LOGW("MAIN", "Not playing local video");
            break;
        }
        CSG_LOGI("MAIN", "Stopping local playback...");
        csg_local_playback_stop(ctx->local_handle);
        ctx->local_handle = -1;

        /* 停止解封装→解码缓冲层 (须先于 audio/MPP 销毁) */
        csg_player_stop_demux_buffer(ctx);

        csg_player_teardown_audio_pipeline(ctx);

        if (ctx->mpp_handle) {
            csg_mpp_decode_stop(ctx->mpp_handle);
            csg_mpp_decode_destroy(ctx->mpp_handle);
            ctx->mpp_handle = NULL;
        }
        memset(&ctx->time_state, 0, sizeof(ctx->time_state));
        ctx->time_state.rate = 1.0f;
        ctx->time_state.speed = 1.0f;
        ctx->time_state.length = CSG_PLAYER_TICK_INVALID;
        ctx->time_state.system_date = csg_player_now_us();
        ctx->time_state.pause_date = CSG_PLAYER_TICK_INVALID;
        ctx->step_frame_pending = 0;
        ctx->live = 0;
        ctx->is_local_play = 0;
        ctx->post_resume_sync = 0;
        ctx->resume_keyframe_seen = 0;
        ctx->resume_min_pts = 0;
        ctx->last_keyframe_pts = 0;
        ctx->last_displayed_pts = 0;
        ctx->mpp_needs_keyframe = 0;
        ctx->mpp_kf_wait_decode = 0;
        csg_lvgl_control_set_live(0);
        csg_lvgl_control_set_paused_ui(0);
        ctx->state = CSG_PLAYER_STATE_IDLE;
        csg_lvgl_control_set_state(CSG_PLAYER_STATE_IDLE);
        memset(&ctx->stats, 0, sizeof(ctx->stats));
        break;
    }

    case CSG_CTRL_CMD_LOCAL_BROWSE: {
        /* ★ 用户点击"Scan"按钮: 强制重新扫描目录.
         *   旧方案只返回启动时缓存, 播放过程中新增的视频文件无法被检测.
         *   新方案每次点击 Scan 都重新遍历目录, 确保文件列表实时更新.
         *
         *   扫描同步执行 (LVGL 线程内), 25 个文件耗时 <50ms, 不会阻塞 UI.
         *   扫描过程中显示 "Scanning..." 提示. */
        csg_lvgl_control_show_message("Scanning...");

        int count = csg_local_playback_scan_dir_recursive(
            LOCAL_VIDEO_DIR, s_csg_cached_files, MAX_CACHED_FILES);

        pthread_mutex_lock(&s_csg_scan_mutex);
        s_csg_cached_file_count = count;
        s_csg_scan_complete = 1;
        pthread_mutex_unlock(&s_csg_scan_mutex);

        CSG_LOGI("MAIN", "Manual re-scan: %d video files found", count);

        if (count > 0) {
            csg_lvgl_control_update_local_list(s_csg_cached_files, count);
            csg_lvgl_control_show_local_browser(1);
            csg_lvgl_control_show_message(NULL);  /* 清除提示 */
        } else {
            CSG_LOGW("MAIN", "No video files found in %s", LOCAL_VIDEO_DIR);
            csg_lvgl_control_show_message("No video files found");
        }
        break;
    }

    case CSG_CTRL_CMD_LOCAL_SELECT: {
        const char *file_path = (const char *)data;
        if (!file_path || file_path[0] == '\0') {
            CSG_LOGW("MAIN", "Empty file path for local select");
            break;
        }
        CSG_LOGI("MAIN", "Selected local file: %s (old state=%d)", file_path, ctx->state);

        /* ★ v5.4 步骤级日志: 每个关键步骤前打印, 精确定位崩溃位置 */
        CSG_LOGI("MAIN", "LOCAL_SELECT step1: set_local_file_path");
        csg_lvgl_control_set_local_file_path(file_path);

        ctx->state = CSG_PLAYER_STATE_IDLE;
        CSG_LOGI("MAIN", "LOCAL_SELECT step2: set_state IDLE");
        csg_lvgl_control_set_state(CSG_PLAYER_STATE_IDLE);

        /* ★ 崩溃修复: 先停分发线程再销毁音频管线。
         * 旧逻辑只在 LOCAL_STOP 停, 切文件 (SELECT) 不停 → 音频分发线程
         * 访问已销毁的 FAAD2/ALSA 句柄 → use-after-free 崩溃。 */
        CSG_LOGI("MAIN", "LOCAL_SELECT step3: stop_demux_buffer");
        csg_player_stop_demux_buffer(ctx);

        CSG_LOGI("MAIN", "LOCAL_SELECT step4: cleanup old local_handle=%d", ctx->local_handle);
        if (ctx->local_handle >= 0) {
            csg_local_playback_cleanup_sync(ctx->local_handle);
            ctx->local_handle = -1;
        }
        CSG_LOGI("MAIN", "LOCAL_SELECT step5: teardown_audio_pipeline");
        csg_player_teardown_audio_pipeline(ctx);
        if (ctx->mpp_handle) {
            CSG_LOGI("MAIN", "LOCAL_SELECT step5b: destroy old MPP");
            csg_mpp_decode_stop(ctx->mpp_handle);
            csg_mpp_decode_destroy(ctx->mpp_handle);
            ctx->mpp_handle = NULL;
        }
        memset(&ctx->time_state, 0, sizeof(ctx->time_state));
        ctx->time_state.rate = 1.0f;
        ctx->time_state.speed = 1.0f;
        ctx->time_state.length = CSG_PLAYER_TICK_INVALID;
        ctx->time_state.system_date = csg_player_now_us();
        ctx->time_state.pause_date = CSG_PLAYER_TICK_INVALID;
        ctx->step_frame_pending = 0;
        ctx->live = 0;
        ctx->post_resume_sync = 0;
        ctx->resume_keyframe_seen = 0;
        ctx->resume_min_pts = 0;
        ctx->last_keyframe_pts = 0;
        ctx->last_displayed_pts = 0;
        ctx->mpp_needs_keyframe = 0;
        ctx->mpp_kf_wait_decode = 0;

        ctx->is_local_play = 1;
        memset(&ctx->stats, 0, sizeof(ctx->stats));
        csg_lvgl_display_set_paused(0);
        csg_lvgl_control_set_live(0);
        csg_lvgl_control_set_paused_ui(0);
        /* 重置动态帧率计算: 新视频可能 FPS 不同 */
        ctx->fps_prev_pts = 0;
        ctx->fps_calc_count = 0;
        ctx->fps_interval_sum = 0;
        ctx->fps_dynamic_set = 0;
        clock_gettime(CLOCK_MONOTONIC, &s_csg_player_stats_time);

        CSG_LOGI("MAIN", "LOCAL_SELECT step6: create local_playback for %s", file_path);
        ctx->local_handle = csg_local_playback_create(file_path);
        if (ctx->local_handle < 0) {
            CSG_LOGE("MAIN", "Failed to create local playback: %s", file_path);
            ctx->state = CSG_PLAYER_STATE_IDLE;
            csg_lvgl_control_set_state(CSG_PLAYER_STATE_IDLE);
            ctx->is_local_play = 0;
            break;
        }

        CSG_LOGI("MAIN", "LOCAL_SELECT step7: set callbacks + get info");
        csg_local_playback_set_event_callback(ctx->local_handle, csg_player_local_event_cb, ctx);
        csg_local_playback_set_seek_feed_done_ptr(ctx->local_handle,
                                              &ctx->seek_fast_feed_done);

        csg_local_playback_get_video_info(ctx->local_handle,
                                      &ctx->video_width, &ctx->video_height,
                                      &ctx->video_fps, &ctx->codec_type);
        /* ★ 启用 frame_timer 节奏控制器: 确保视频帧按音频时钟节奏等间隔显示 */
        if (ctx->video_fps > 0 && ctx->av_sync_handle) {
            csg_av_sync_set_frame_duration_by_fps(ctx->av_sync_handle,
                                              (float)ctx->video_fps);
        }
        int64_t duration = csg_local_playback_get_duration(ctx->local_handle);
        if (duration > 0) {
            ctx->time_state.length = duration;
            ctx->live = 0;
        } else {
            ctx->time_state.length = CSG_PLAYER_TICK_INVALID;
            ctx->live = 1;
        }

        ctx->mpp_handle = csg_mpp_decode_create(ctx->codec_type,
                                            ctx->video_width,
                                            ctx->video_height);
        if (!ctx->mpp_handle) {
            CSG_LOGE("MAIN", "Failed to create MPP decoder for local video");
            csg_local_playback_stop(ctx->local_handle);
            csg_local_playback_cleanup_sync(ctx->local_handle);
            ctx->local_handle = -1;
            ctx->state = CSG_PLAYER_STATE_IDLE;
            csg_lvgl_control_set_state(CSG_PLAYER_STATE_IDLE);
            ctx->is_local_play = 0;
            break;
        }
        CSG_LOGI("MAIN", "LOCAL_SELECT step8: MPP decoder created");

        ctx->state_gen = csg_player_next_state_gen();
        ctx->mpp_gen = ctx->state_gen;

        ctx->state = CSG_PLAYER_STATE_PLAYING;
        csg_lvgl_control_set_state(CSG_PLAYER_STATE_PLAYING);

        /* ★ 解封装→解码 缓冲层: 在设置解封装回调前拉起视频/音频分发线程,
         * 保证首帧入队时消费者已就绪 (解耦解封装与 MPP/FAAD2 同步阻塞). */
        CSG_LOGI("MAIN", "LOCAL_SELECT step9: start_demux_buffer");
        csg_player_start_demux_buffer(ctx);

        CSG_LOGI("MAIN", "LOCAL_SELECT step10: set callbacks");
        csg_mpp_decode_set_callback(ctx->mpp_handle, csg_player_on_mpp_frame, ctx);
        csg_local_playback_set_callback(ctx->local_handle, csg_player_on_local_frame, ctx);
        csg_local_playback_set_audio_callback(ctx->local_handle, csg_player_on_audio_frame, ctx);

        CSG_LOGI("MAIN", "LOCAL_SELECT step11: setup_audio_pipeline");
        csg_player_setup_audio_pipeline(ctx, ctx->local_handle);

        /* ★ 新视频播放时强制重置所有层的倍速为 1x (同上) */
        CSG_LOGI("MAIN", "LOCAL_SELECT step12: set_speed + start");
        csg_player_set_speed(ctx, 1.0f);

        csg_local_playback_start(ctx->local_handle);
        csg_lvgl_control_show_local_browser(0);
        CSG_LOGI("MAIN", "LOCAL_SELECT done: playing %s", file_path);
        break;
    }

    case CSG_CTRL_CMD_LOCAL_PAUSE: {
        if (ctx->is_local_play && ctx->local_handle >= 0) {
            if (ctx->state == CSG_PLAYER_STATE_PLAYING) {
                csg_player_pause(ctx);
            }
            CSG_LOGI("MAIN", "Local video paused");
        }
        break;
    }

    case CSG_CTRL_CMD_LOCAL_RESUME: {
        if (ctx->is_local_play && ctx->local_handle >= 0) {
            if (ctx->state == CSG_PLAYER_STATE_PAUSED) {
                csg_player_resume(ctx);
            }
            CSG_LOGI("MAIN", "Local video resumed");
        }
        break;
    }

    case CSG_CTRL_CMD_LOCAL_STOP: {
        if (!ctx->is_local_play || ctx->local_handle < 0) {
            CSG_LOGW("MAIN", "Not playing local video");
            break;
        }
        CSG_LOGI("MAIN", "Stopping local playback (UI)...");
        csg_local_playback_stop(ctx->local_handle);
        ctx->local_handle = -1;

        /* 停止解封装→解码缓冲层 (须先于 audio/MPP 销毁) */
        csg_player_stop_demux_buffer(ctx);

        csg_player_teardown_audio_pipeline(ctx);

        if (ctx->mpp_handle) {
            csg_mpp_decode_stop(ctx->mpp_handle);
            csg_mpp_decode_destroy(ctx->mpp_handle);
            ctx->mpp_handle = NULL;
        }
        memset(&ctx->time_state, 0, sizeof(ctx->time_state));
        ctx->time_state.rate = 1.0f;
        ctx->time_state.speed = 1.0f;
        ctx->time_state.length = CSG_PLAYER_TICK_INVALID;
        ctx->time_state.system_date = csg_player_now_us();
        ctx->time_state.pause_date = CSG_PLAYER_TICK_INVALID;
        ctx->step_frame_pending = 0;
        ctx->live = 0;
        ctx->is_local_play = 0;
        csg_lvgl_display_set_paused(0);
        csg_lvgl_control_set_live(0);
        csg_lvgl_control_set_paused_ui(0);
        ctx->state = CSG_PLAYER_STATE_IDLE;
        csg_lvgl_control_set_state(CSG_PLAYER_STATE_IDLE);
        memset(&ctx->stats, 0, sizeof(ctx->stats));
        break;
    }

    case CSG_CTRL_CMD_QUIT:
        csg_player_stop_rtsp(ctx);   /* 内含停两路录制 + 预览 */
        ctx->running = 0;
        csg_player_stop_demux_buffer(ctx);   /* 停分发线程, 避免退出时线程被强杀 */
        break;

    case CSG_CTRL_CMD_SPEED_SET: {
        float *spd = (float *)data;
        if (spd && *spd >= 0.5f && *spd <= 16.0f) {
            csg_player_set_speed(ctx, *spd);
        }
        break;
    }

    case CSG_CTRL_CMD_RTSP_PLAY: {
        /* data = csg_ctrl_rtsp_play_t*: 预览子码流 URL + 录制主码流 URL(自动录制) */
        csg_ctrl_rtsp_play_t *pp = (csg_ctrl_rtsp_play_t *)data;
        if (!pp || pp->preview_url[0] == '\0') break;
        const char *url = pp->preview_url;
        /* 自动录制主码流 URL: 未填则回退预览 URL */
        const char *rec_url = (pp->record_url[0] != '\0') ? pp->record_url : pp->preview_url;
        CSG_LOGI("MAIN", "RTSP play: %s (auto-record: %s)", url, rec_url);

        /* 先清理旧的本地播放/两路录制/拉流 */
        csg_player_stop_rtsp(ctx);
        if (ctx->local_handle >= 0) {
            csg_local_playback_cleanup_sync(ctx->local_handle);
            ctx->local_handle = -1;
        }
        csg_player_teardown_audio_pipeline(ctx);
        /* 停止上一个播放残留的缓冲层 (须先于 MPP 销毁) */
        csg_player_stop_demux_buffer(ctx);
        if (ctx->mpp_handle) {
            csg_mpp_decode_stop(ctx->mpp_handle);
            csg_mpp_decode_destroy(ctx->mpp_handle);
            ctx->mpp_handle = NULL;
        }
        memset(&ctx->time_state, 0, sizeof(ctx->time_state));
        ctx->time_state.rate = 1.0f;
        ctx->time_state.speed = 1.0f;
        ctx->time_state.length = CSG_PLAYER_TICK_INVALID;
        ctx->time_state.system_date = csg_player_now_us();
        ctx->time_state.pause_date = CSG_PLAYER_TICK_INVALID;
        /* ★ 新拉流开始时重置进度条为 0 (直播不显示位置, 但清掉上次残留) */
        csg_lvgl_control_reset_progress();
        ctx->step_frame_pending = 0;
        ctx->post_resume_sync = 0;
        ctx->resume_keyframe_seen = 0;
        ctx->resume_min_pts = 0;
        ctx->last_keyframe_pts = 0;
        ctx->last_displayed_pts = 0;
        ctx->mpp_needs_keyframe = 0;
        ctx->mpp_kf_wait_decode = 0;
        ctx->kf_pts_ring_head = 0;
        ctx->kf_pts_ring_count = 0;
        ctx->last_fed_kf_pts = 0;

        ctx->is_local_play = 0;
        ctx->live = 1;
        csg_lvgl_display_set_paused(0);
        csg_lvgl_control_set_live(1);
        csg_lvgl_control_set_paused_ui(0);
        memset(&ctx->stats, 0, sizeof(ctx->stats));
        clock_gettime(CLOCK_MONOTONIC, &s_csg_player_stats_time);

        /* live555 拉流 (编码/分辨率由 SDP 异步确定, MPP 首帧懒创建) */
        ctx->rtsp_handle = csg_rtsp_live555_create(url, CSG_CODEC_H264);
        if (!ctx->rtsp_handle) {
            CSG_LOGE("MAIN", "Failed to create RTSP pull");
            csg_lvgl_control_show_message("RTSP create failed!");
            free(pp);
            ctx->live = 0;
            break;
        }
        /* ★ RTSP 首帧到达前拉起缓冲层 + 分发线程 (MPP 首帧懒创建在分发线程) */
        csg_player_start_demux_buffer(ctx);
        csg_rtsp_live555_set_callback(ctx->rtsp_handle, csg_player_on_local_frame, ctx);
        /* ★ RTSP 音频: 注册 G.711→PCM16 回调 (须在 start 前, live555 遍历
         * subsession 时据此决定是否拉取 PCMU/PCMA 音频轨) */
        csg_rtsp_live555_set_audio_callback(ctx->rtsp_handle,
                                            csg_player_on_rtsp_audio_pcm, ctx);
        if (csg_rtsp_live555_start(ctx->rtsp_handle) != 0) {
            CSG_LOGE("MAIN", "Failed to start RTSP pull");
            csg_rtsp_live555_stop(ctx->rtsp_handle);
            ctx->rtsp_handle = NULL;
            csg_lvgl_control_show_message("RTSP start failed!");
            free(pp);
            ctx->live = 0;
            break;
        }

        ctx->state = CSG_PLAYER_STATE_PLAYING;
        csg_lvgl_control_set_state(CSG_PLAYER_STATE_PLAYING);
        CSG_LOGI("MAIN", "RTSP started: %s", url);

        /* ① 拉流即自动录制: 独立主码流句柄, 与手动录制互不干扰 */
        csg_lvgl_control_show_message("Auto recording main stream...");
        csg_player_rec_connect(ctx, &ctx->auto_rec, "AUTO",
                               csg_player_on_auto_record_frame, rec_url);

        free(pp);
        break;
    }

    case CSG_CTRL_CMD_RTSP_STOP:
        /* ★ UI Stop 按钮: 多路模式正在运行时优先停多路 (幂等) */
        if (nvr_multi_get_active() > 0) {
            nvr_multi_stop();
            csg_lvgl_control_show_message("Multi preview stopped");
            break;
        }
        if (!ctx->rtsp_handle) {
            CSG_LOGW("MAIN", "Not pulling RTSP");
            break;
        }
        CSG_LOGI("MAIN", "Stopping RTSP...");
        /* 停两路录制 + 预览子码流 */
        csg_player_stop_rtsp(ctx);
        /* 停止解封装→解码缓冲层 (须先于 MPP 销毁) */
        csg_player_stop_demux_buffer(ctx);
        if (ctx->mpp_handle) {
            csg_mpp_decode_stop(ctx->mpp_handle);
            csg_mpp_decode_destroy(ctx->mpp_handle);
            ctx->mpp_handle = NULL;
        }
        memset(&ctx->time_state, 0, sizeof(ctx->time_state));
        ctx->time_state.rate = 1.0f;
        ctx->time_state.speed = 1.0f;
        ctx->time_state.length = CSG_PLAYER_TICK_INVALID;
        ctx->time_state.system_date = csg_player_now_us();
        ctx->time_state.pause_date = CSG_PLAYER_TICK_INVALID;
        /* ★ 停止拉流时重置进度条 */
        csg_lvgl_control_reset_progress();
        ctx->step_frame_pending = 0;
        ctx->live = 0;
        ctx->is_local_play = 0;
        csg_lvgl_display_set_paused(0);
        csg_lvgl_control_set_live(0);
        csg_lvgl_control_set_paused_ui(0);
        ctx->state = CSG_PLAYER_STATE_IDLE;
        csg_lvgl_control_set_state(CSG_PLAYER_STATE_IDLE);
        memset(&ctx->stats, 0, sizeof(ctx->stats));
        break;

    case CSG_CTRL_CMD_RECORD_START: {
        /* data = 录制主码流 URL (strdup, 需释放). 预览未启动则忽略 */
        const char *rec_url = (const char *)data;
        if (!ctx->rtsp_handle) {
            CSG_LOGW("MAIN", "Cannot record: no preview stream");
            csg_lvgl_control_show_message("Start preview first!");
            free((void *)rec_url);
            break;
        }
        csg_player_start_record(ctx, rec_url);
        free((void *)rec_url);
        break;
    }

    case CSG_CTRL_CMD_RECORD_STOP:
        /* 只停手动录制, 不影响自动录制 (互不干扰) */
        csg_player_rec_stop(ctx, &ctx->manual_rec, "MANUAL");
        break;

    /* ==================== 多路分屏拉流/录制 (NVR_MULTI) ==================== */
    case CSG_CTRL_CMD_MULTI_PLAY: {
        /* data = csg_ctrl_multi_play_t* (heap, 需释放).
         * 双码流: count==1 单路主码流显示+录制; count>1 每路子码流预览+主码流录制 */
        csg_ctrl_multi_play_t *mp = (csg_ctrl_multi_play_t *)data;
        if (!mp || mp->count < 1) {
            if (mp) free(mp);
            break;
        }
        /* 进入多路模式前先停掉单路播放 (RTSP/本地), 避免资源叠加 */
        if (ctx->rtsp_handle || ctx->local_handle >= 0) {
            CSG_LOGI("MAIN", "Multi play: stopping current playback first");
            if (ctx->rtsp_handle) {
                csg_player_stop_rtsp(ctx);
                csg_player_stop_demux_buffer(ctx);
                if (ctx->mpp_handle) {
                    csg_mpp_decode_stop(ctx->mpp_handle);
                    csg_mpp_decode_destroy(ctx->mpp_handle);
                    ctx->mpp_handle = NULL;
                }
            }
            if (ctx->local_handle >= 0) {
                csg_local_playback_stop(ctx->local_handle);
                ctx->local_handle = -1;
                ctx->is_local_play = 0;
            }
            ctx->state = CSG_PLAYER_STATE_IDLE;
            csg_lvgl_control_set_state(CSG_PLAYER_STATE_IDLE);
            csg_lvgl_control_set_live(0);
        }

        nvr_multi_url_t mu[NVR_MULTI_MAX_CHN];
        int cnt = mp->count > NVR_MULTI_MAX_CHN ? NVR_MULTI_MAX_CHN : mp->count;
        for (int i = 0; i < cnt; i++) {
            snprintf(mu[i].preview, sizeof(mu[i].preview), "%.511s",
                     mp->preview[i]);
            snprintf(mu[i].record, sizeof(mu[i].record), "%.511s",
                     mp->record[i]);
        }

        int started = nvr_multi_start(mu, cnt);
        csg_lvgl_control_show_message(
            started > 0 ? "Multi preview started!" : "Multi preview failed!");
        CSG_LOGI("MAIN", "Multi play: %d/%d channels started", started, cnt);
        free(mp);
        break;
    }

    case CSG_CTRL_CMD_MULTI_STOP:
        nvr_multi_stop();
        csg_lvgl_control_show_message("Multi preview stopped");
        break;

    default:
        break;
    }

    ctx->cmd_in_progress = 0;
}

/* ==================== 信号处理 ==================== */
static void csg_player_signal_handler(int sig)
{
    CSG_LOGI("MAIN", "Signal %d received, exiting...", sig);
    s_csg_player_app.running = 0;
}

/* ==================== 主函数 ==================== */
int main(int argc, char *argv[])
{
    CSG_LOGI("MAIN", "=== RK3572 本地视频播放器 ===");
    CSG_LOGI("MAIN", "Features: Local Playback + MPP Decode + RGA Convert + LVGL Display");

    signal(SIGINT, csg_player_signal_handler);
    signal(SIGTERM, csg_player_signal_handler);

    s_csg_player_app.running = 1;
    s_csg_player_app.state = CSG_PLAYER_STATE_IDLE;
    s_csg_player_app.local_handle = -1;
    s_csg_player_app.is_local_play = 0;
    s_csg_player_app.has_audio = 0;
    s_csg_player_app.av_sync_handle = NULL;
    s_csg_player_app.audio_dec_handle = NULL;
    s_csg_player_app.audio_render_handle = NULL;
    pthread_mutex_init(&s_csg_player_app.mutex, NULL);
    pthread_mutex_init(&s_csg_player_app.record_mutex, NULL);
    pthread_mutex_init(&s_csg_player_app.rtsp_audio_mutex, NULL);
    s_csg_player_app.rtsp_audio_tried = 0;
    memset(&s_csg_player_app.stats, 0, sizeof(s_csg_player_app.stats));
    memset(&s_csg_player_app.time_state, 0, sizeof(s_csg_player_app.time_state));
    s_csg_player_app.time_state.rate = 1.0f;
    s_csg_player_app.time_state.speed = 1.0f;
    s_csg_player_app.time_state.length = CSG_PLAYER_TICK_INVALID;
    s_csg_player_app.time_state.system_date = csg_player_now_us();
    s_csg_player_app.time_state.pause_date = CSG_PLAYER_TICK_INVALID;

    if (csg_rga_convert_init() != 0) {
        CSG_LOGE("MAIN", "RGA init failed");
        return -1;
    }

    if (csg_lvgl_display_init(0, 0) != 0) {
        CSG_LOGE("MAIN", "LVGL display init failed");
        csg_rga_convert_deinit();
        return -1;
    }

    if (csg_lvgl_control_init(csg_player_on_control_cmd, &s_csg_player_app) != 0) {
        CSG_LOGE("MAIN", "LVGL control init failed");
        csg_lvgl_display_deinit();
        csg_rga_convert_deinit();
        return -1;
    }

    /* 启动后台预扫描线程: 程序启动时立即扫描视频目录 (含子目录),
     * 获取所有视频文件的分辨率、时长信息. 用户首次点击"本地视频"
     * 时直接使用缓存, 无需等待扫描. */
    if (pthread_create(&s_csg_scan_thread, NULL, csg_player_scan_thread, NULL) != 0) {
        CSG_LOGW("MAIN", "Failed to start background scan thread");
    }

    if (argc > 1) {
        if (strcmp(argv[1], "--multi") == 0 && argc > 2) {
            /* ★ CLI 多路分屏 (自动码流切换):
             *   rk3572_player --multi url1 url2 url3 ...
             *   每路只需给一个 URL (主/子码流均可), 程序自动:
             *     单路 → 主码流显示+录制; 多路 → 子码流预览 + 主码流录制.
             *   最多 NVR_MULTI_MAX_CHN 路, 超过截断. */
            int cnt = argc - 2;
            if (cnt > NVR_MULTI_MAX_CHN) cnt = NVR_MULTI_MAX_CHN;
            nvr_multi_url_t mu[NVR_MULTI_MAX_CHN];
            memset(mu, 0, sizeof(mu));
            for (int i = 0; i < cnt; i++)
                snprintf(mu[i].preview, sizeof(mu[i].preview), "%s", argv[2 + i]);
            CSG_LOGI("MAIN", "Auto-starting multi preview: %d channels", cnt);
            int started = nvr_multi_start(mu, cnt);
            CSG_LOGI("MAIN", "Multi preview: %d/%d channels started", started, cnt);
        } else if (strncmp(argv[1], "rtsp://", 7) == 0) {
            /* ★ CLI 直接拉 RTSP 并自动录制主码流 (无触摸屏 UI 时测试用):
             * preview_url 与 record_url 都用该主码流 URL, 拉流即预览+录制.
             * ★ 必须 heap 分配 rp: CSG_CTRL_CMD_RTSP_PLAY 处理器末尾会
             *   free(pp), 传栈上变量会把栈地址交给 free → Segfault
             *   (历史踩坑: free(): invalid pointer / Segmentation fault). */
            CSG_LOGI("MAIN", "Auto-starting RTSP: %s (auto-record main stream)", argv[1]);
            csg_ctrl_rtsp_play_t *rp = calloc(1, sizeof(csg_ctrl_rtsp_play_t));
            if (rp) {
                strncpy(rp->preview_url, argv[1], sizeof(rp->preview_url) - 1);
                strncpy(rp->record_url, argv[1], sizeof(rp->record_url) - 1);
                /* rp 所有权交给命令处理器, 由内部 free(pp) 释放 */
                csg_player_on_control_cmd(CSG_CTRL_CMD_RTSP_PLAY, rp, &s_csg_player_app);
            }
        } else {
            CSG_LOGI("MAIN", "Auto-starting local video: %s", argv[1]);
            csg_player_on_control_cmd(CSG_CTRL_CMD_PLAY_LOCAL, argv[1], &s_csg_player_app);
        }
    }

    CSG_LOGI("MAIN", "Entering main loop...");

    /* ★ v5.9: 创建常开音频渲染器, 程序生命周期内只 open/close 一次.
     * 避免每次切换文件都经历 snd_pcm_close/open 循环导致的
     * NAU8822 codec power_off/power_on, 该循环会破坏 DAPM 内部
     * Jack Detect 状态, 导致第二次播放无声.
     *
     * 设备探测: hw:1,0 (NAU8822) → hw:0,0 (HDMI) → default */
    {
        static const char *s_persist_devices[] = {
            "hw:1,0", "hw:0,0", "default", NULL
        };
        for (int di = 0; s_persist_devices[di]; di++) {
            s_csg_player_app.audio_render_persistent = csg_audio_render_create(
                RTSP_AUDIO_OUT_RATE, RTSP_AUDIO_OUT_CH, 16,
                s_persist_devices[di]);
            if (s_csg_player_app.audio_render_persistent) {
                CSG_LOGI("MAIN", "Persistent audio device: %s (probe OK, "
                     "will stay open until exit)", s_persist_devices[di]);
                break;
            }
            CSG_LOGW("MAIN", "Persistent audio device: %s (probe failed)", s_persist_devices[di]);
        }
        if (!s_csg_player_app.audio_render_persistent) {
            CSG_LOGW("MAIN", "No persistent audio device available, "
                 "will create per-playback (may cause 2nd-play silent)");
        }
    }
    while (s_csg_player_app.running) {
        usleep(10000);
    }

    CSG_LOGI("MAIN", "Cleaning up...");

    /* ★ RTSP 收尾: 停两路录制(finalize MP4 写 moov) + 停拉流.
     * 必须放在 teardown_audio 之前, 否则 Ctrl+C 退出时录制文件
     * 因未 finalize 而打不开 (moov 缺失/seek 越界). */
    csg_player_stop_rtsp(&s_csg_player_app);

    /* ★ 多路收尾: 停全部通道 (finalize 各路 MP4 + 停 RTSP + 销毁解码器) */
    nvr_multi_stop();

    if (s_csg_player_app.local_handle >= 0) {
        csg_local_playback_stop(s_csg_player_app.local_handle);
    }
    csg_player_teardown_audio_pipeline(&s_csg_player_app);

    /* ★ v5.9: 销毁常开音频渲染器 — 在 teardown 之后,
     * 确保当前播放的音频管线已停止, PCM 设备可安全关闭. */
    if (s_csg_player_app.audio_render_persistent) {
        csg_audio_render_destroy(s_csg_player_app.audio_render_persistent);
        s_csg_player_app.audio_render_persistent = NULL;
        CSG_LOGI("MAIN", "Persistent audio device destroyed");
    }
    if (s_csg_player_app.mpp_handle) {
        csg_mpp_decode_destroy(s_csg_player_app.mpp_handle);
    }
    if (s_csg_player_app.rgb_buf) {
        free(s_csg_player_app.rgb_buf);
        s_csg_player_app.rgb_buf = NULL;
    }

    csg_lvgl_control_deinit();
    csg_lvgl_display_deinit();
    csg_rga_convert_deinit();
    pthread_mutex_destroy(&s_csg_player_app.mutex);
    pthread_mutex_destroy(&s_csg_player_app.rtsp_audio_mutex);
    pthread_mutex_destroy(&s_csg_scan_mutex);

    CSG_LOGI("MAIN", "=== Exit ===");
    return 0;
}