/* SPDX-License-Identifier: MIT */
/*
 * demux_buffer.c - 解封装 → 解码 缓冲层实现
 *
 * 底层用 csg_vecdeque (动态数组双端队列) 分别存放视频/音频帧.
 * 生产者 (解封装线程) push 深拷贝帧, 消费者 (分发线程) pop 送硬件解码.
 * 水位背压 + 代次 (generation) 机制见头文件注释.
 */

#include "demux_buffer.h"
#include "common/vecdeque.h"
#include "common.h"

#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <time.h>

/* 默认水位 (嵌入式内存安全: 视频 32MB / 音频 4MB 上限) */
#define DEMUX_DEF_VIDEO_MAX_FRAMES  64
#define DEMUX_DEF_VIDEO_MAX_BYTES   (32u * 1024 * 1024)
#define DEMUX_DEF_AUDIO_MAX_FRAMES  256
#define DEMUX_DEF_AUDIO_MAX_BYTES   (4u * 1024 * 1024)
#define DEMUX_DEF_VIDEO_INIT_CAP    16
#define DEMUX_DEF_AUDIO_INIT_CAP    32

/* 背压/停止等待轮询周期 (ms): 保证及时响应 seek/stop */
#define DEMUX_WAIT_TIMEOUT_MS       10

struct csg_demux_buffer_s {
    csg_vecdeque_t video_q;      /* 视频帧队列 (存 csg_demux_video_frame_t 副本) */
    csg_vecdeque_t audio_q;      /* 音频帧队列 (存 csg_demux_audio_frame_t 副本) */

    pthread_mutex_t lock;
    pthread_cond_t  cond;        /* push/pop 互通知 */

    size_t  video_bytes;         /* 当前视频队列总字节数 */
    size_t  audio_bytes;         /* 当前音频队列总字节数 */

    size_t  video_max_frames;
    size_t  video_max_bytes;
    size_t  audio_max_frames;
    size_t  audio_max_bytes;

    unsigned int generation;     /* flush 代次: push 检测到变化即放弃旧数据 */
    int          stopped;        /* 1=已 shutdown, pop 返回 -1 */
};

static void csg_demux_abs_timeout_ms(struct timespec *ts, int ms)
{
    clock_gettime(CLOCK_MONOTONIC, ts);
    ts->tv_sec += ms / 1000;
    ts->tv_nsec += (long)(ms % 1000) * 1000000L;
    if (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec += 1;
        ts->tv_nsec -= 1000000000L;
    }
}

int csg_demux_buffer_create(const csg_demux_buffer_cfg_t *cfg, csg_demux_buffer_t **out)
{
    if (!out) return -1;

    csg_demux_buffer_t *b = (csg_demux_buffer_t *)calloc(1, sizeof(*b));
    if (!b) return -1;

    size_t vmf = DEMUX_DEF_VIDEO_MAX_FRAMES;
    size_t vmb = DEMUX_DEF_VIDEO_MAX_BYTES;
    size_t amf = DEMUX_DEF_AUDIO_MAX_FRAMES;
    size_t amb = DEMUX_DEF_AUDIO_MAX_BYTES;
    size_t vic = DEMUX_DEF_VIDEO_INIT_CAP;
    size_t aic = DEMUX_DEF_AUDIO_INIT_CAP;
    if (cfg) {
        if (cfg->video_max_frames) vmf = cfg->video_max_frames;
        if (cfg->video_max_bytes)  vmb = cfg->video_max_bytes;
        if (cfg->audio_max_frames) amf = cfg->audio_max_frames;
        if (cfg->audio_max_bytes)  amb = cfg->audio_max_bytes;
        if (cfg->video_init_cap)   vic = cfg->video_init_cap;
        if (cfg->audio_init_cap)   aic = cfg->audio_init_cap;
    }
    b->video_max_frames = vmf;
    b->video_max_bytes  = vmb;
    b->audio_max_frames = amf;
    b->audio_max_bytes  = amb;

    if (csg_vecdeque_init(&b->video_q, sizeof(csg_demux_video_frame_t), vic) != 0 ||
        csg_vecdeque_init(&b->audio_q, sizeof(csg_demux_audio_frame_t), aic) != 0) {
        csg_vecdeque_destroy(&b->video_q);
        csg_vecdeque_destroy(&b->audio_q);
        free(b);
        return -1;
    }

    pthread_mutex_init(&b->lock, NULL);
    pthread_cond_init(&b->cond, NULL);
    b->generation = 0;
    b->stopped = 0;

    *out = b;
    CSG_LOGI("DEMUX_BUF", "created (video: %zu frames/%zu MB, audio: %zu frames/%zu KB)",
         vmf, vmb / (1024 * 1024), amf, amb / 1024);
    return 0;
}

void csg_demux_buffer_destroy(csg_demux_buffer_t *b)
{
    if (!b) return;

    csg_demux_buffer_flush(b);   /* 释放残留帧 */

    csg_vecdeque_destroy(&b->video_q);
    csg_vecdeque_destroy(&b->audio_q);
    pthread_cond_destroy(&b->cond);
    pthread_mutex_destroy(&b->lock);
    free(b);
}

/* 释放队列中所有帧的动态数据 (须持锁调用) */
static void csg_demux_buffer_free_queued(csg_demux_buffer_t *b)
{
    csg_demux_video_frame_t vf;
    while (csg_vecdeque_pop_head(&b->video_q, &vf) == 0) {
        if (vf.data) free(vf.data);
    }
    csg_demux_audio_frame_t af;
    while (csg_vecdeque_pop_head(&b->audio_q, &af) == 0) {
        if (af.data) free(af.data);
    }
    b->video_bytes = 0;
    b->audio_bytes = 0;
}

/* ==================== push (生产者) ==================== */

static int csg_demux_buffer_push_common(csg_demux_buffer_t *b,
                                        int is_video, const uint8_t *data, int len,
                                        const volatile int *abort_flag,
                                        csg_demux_video_frame_t *vf,
                                        csg_demux_audio_frame_t *af)
{
    if (!b || !data || len <= 0) return -1;

    unsigned int gen = b->generation;   /* 记录进入时代次 */

    for (;;) {
        /* 停止检查: abort_flag 指向 ctx->running (1=运行中, 0=已停止).
         * ★ 历史踩坑: 原判断 if(abort_flag && *abort_flag) 把 running=1
         *   (正常播放) 误判为"需要停止", 导致 push 永远返回 -1,
         *   所有帧入队失败 → 分发线程取不到帧 → MPP 无输入 → 黑屏,
         *   且 AUDIO_DEC/AUDIO_RENDER 全程 0 帧. 必须取反. */
        if (abort_flag && !*abort_flag) return -1;

        pthread_mutex_lock(&b->lock);

        if (b->generation != gen) {
            /* 期间被 flush (seek/stop): 放弃本帧, 让生产者回主循环 */
            pthread_mutex_unlock(&b->lock);
            return -1;
        }
        if (b->stopped) {
            pthread_mutex_unlock(&b->lock);
            return -1;
        }

        size_t max_frames = is_video ? b->video_max_frames : b->audio_max_frames;
        size_t cur_frames = is_video ? csg_vecdeque_length(&b->video_q)
                                     : csg_vecdeque_length(&b->audio_q);
        size_t max_bytes  = is_video ? b->video_max_bytes : b->audio_max_bytes;
        size_t cur_bytes  = is_video ? b->video_bytes     : b->audio_bytes;

        if (cur_frames < max_frames && cur_bytes + (size_t)len <= max_bytes) {
            /* 深拷贝入队 */
            uint8_t *copy = (uint8_t *)malloc((size_t)len);
            if (!copy) {
                pthread_mutex_unlock(&b->lock);
                return -1;
            }
            memcpy(copy, data, (size_t)len);

            if (is_video) {
                vf->data = copy;
                csg_vecdeque_push_tail(&b->video_q, vf);
                b->video_bytes += (size_t)len;
            } else {
                af->data = copy;
                csg_vecdeque_push_tail(&b->audio_q, af);
                b->audio_bytes += (size_t)len;
            }
            pthread_cond_broadcast(&b->cond);   /* 唤醒消费者 */
            pthread_mutex_unlock(&b->lock);
            return 0;
        }

        /* 队列满: 背压等待 (带超时, 便于及时检查 abort/代次) */
        struct timespec ts;
        csg_demux_abs_timeout_ms(&ts, DEMUX_WAIT_TIMEOUT_MS);
        pthread_cond_timedwait(&b->cond, &b->lock, &ts);
        pthread_mutex_unlock(&b->lock);
    }
}

int csg_demux_buffer_push_video(csg_demux_buffer_t *b, const uint8_t *data, int len,
                                int is_keyframe, uint64_t pts, int codec_type,
                                const volatile int *abort_flag)
{
    csg_demux_video_frame_t vf;
    vf.data = NULL;
    vf.len = len;
    vf.is_keyframe = is_keyframe;
    vf.pts = pts;
    vf.codec_type = codec_type;
    return csg_demux_buffer_push_common(b, 1, data, len, abort_flag, &vf, NULL);
}

int csg_demux_buffer_push_audio(csg_demux_buffer_t *b, const uint8_t *data, int len,
                                int64_t pts, const volatile int *abort_flag)
{
    csg_demux_audio_frame_t af;
    af.data = NULL;
    af.len = len;
    af.pts = pts;
    return csg_demux_buffer_push_common(b, 0, data, len, abort_flag, NULL, &af);
}

/* ==================== pop (消费者) ==================== */

int csg_demux_buffer_pop_video(csg_demux_buffer_t *b, csg_demux_video_frame_t *out)
{
    if (!b || !out) return -1;

    for (;;) {
        pthread_mutex_lock(&b->lock);

        if (b->stopped) {
            pthread_mutex_unlock(&b->lock);
            return -1;
        }
        if (csg_vecdeque_length(&b->video_q) > 0) {
            csg_vecdeque_pop_head(&b->video_q, out);
            b->video_bytes -= (size_t)out->len;
            pthread_cond_broadcast(&b->cond);   /* 唤醒背压等待的 push */
            pthread_mutex_unlock(&b->lock);
            return 1;
        }

        struct timespec ts;
        csg_demux_abs_timeout_ms(&ts, DEMUX_WAIT_TIMEOUT_MS);
        pthread_cond_timedwait(&b->cond, &b->lock, &ts);
        pthread_mutex_unlock(&b->lock);
        /* 空队列且未停止 → 返回 0 让调用方重试/退出判断 */
        if (b->stopped) return -1;
        return 0;
    }
}

int csg_demux_buffer_pop_audio(csg_demux_buffer_t *b, csg_demux_audio_frame_t *out)
{
    if (!b || !out) return -1;

    for (;;) {
        pthread_mutex_lock(&b->lock);

        if (b->stopped) {
            pthread_mutex_unlock(&b->lock);
            return -1;
        }
        if (csg_vecdeque_length(&b->audio_q) > 0) {
            csg_vecdeque_pop_head(&b->audio_q, out);
            b->audio_bytes -= (size_t)out->len;
            pthread_cond_broadcast(&b->cond);
            pthread_mutex_unlock(&b->lock);
            return 1;
        }

        struct timespec ts;
        csg_demux_abs_timeout_ms(&ts, DEMUX_WAIT_TIMEOUT_MS);
        pthread_cond_timedwait(&b->cond, &b->lock, &ts);
        pthread_mutex_unlock(&b->lock);
        if (b->stopped) return -1;
        return 0;
    }
}

void csg_demux_buffer_free_frame(void *data)
{
    if (data) free(data);
}

void csg_demux_buffer_flush(csg_demux_buffer_t *b)
{
    if (!b) return;

    pthread_mutex_lock(&b->lock);
    csg_demux_buffer_free_queued(b);
    b->generation++;           /* 代次变化: 唤醒的 push 检测到后放弃旧帧 */
    pthread_cond_broadcast(&b->cond);
    pthread_mutex_unlock(&b->lock);
}

void csg_demux_buffer_shutdown(csg_demux_buffer_t *b)
{
    if (!b) return;

    pthread_mutex_lock(&b->lock);
    b->stopped = 1;
    pthread_cond_broadcast(&b->cond);
    pthread_mutex_unlock(&b->lock);
}

size_t csg_demux_buffer_video_count(csg_demux_buffer_t *b)
{
    if (!b) return 0;
    pthread_mutex_lock(&b->lock);
    size_t n = csg_vecdeque_length(&b->video_q);
    pthread_mutex_unlock(&b->lock);
    return n;
}

size_t csg_demux_buffer_audio_count(csg_demux_buffer_t *b)
{
    if (!b) return 0;
    pthread_mutex_lock(&b->lock);
    size_t n = csg_vecdeque_length(&b->audio_q);
    pthread_mutex_unlock(&b->lock);
    return n;
}

size_t csg_demux_buffer_video_bytes(csg_demux_buffer_t *b)
{
    if (!b) return 0;
    pthread_mutex_lock(&b->lock);
    size_t n = b->video_bytes;
    pthread_mutex_unlock(&b->lock);
    return n;
}

size_t csg_demux_buffer_audio_bytes(csg_demux_buffer_t *b)
{
    if (!b) return 0;
    pthread_mutex_lock(&b->lock);
    size_t n = b->audio_bytes;
    pthread_mutex_unlock(&b->lock);
    return n;
}
