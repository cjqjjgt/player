/* SPDX-License-Identifier: MIT */
/*
 * mp4_record.c - 基于 libmov fmp4_writer 的录制模块 (消息队列 + 录制线程)
 *
 * 直接 remux: 拉流回调收到的 Annex-B H264/H265 编码帧 + G.711 原始字节
 * 由录制线程转换为 length-prefix 格式后调用 libmov 写入 fMP4。
 * 全程不经过 FFmpeg, 不经过软编码。
 *
 * 参考: /home/dell/rk3572_nvr/src/preview/record_fmp4_writer.cpp
 *       /home/dell/rk3572_nvr/third_party/ireader_media_server/libmov
 *
 * 历史踩坑点:
 * 1. libmov fmp4_writer 期望 length-prefix (4字节大端长度) 样本,
 *    不能直接写 Annex-B (start code), 否则播放器无法解析 NAL 边界.
 * 2. 必须先 fmp4_writer_add_video 注入 avcC/hvcC (由 libflv 从 Annex-B
 *    参数集构建), 再写第一帧, 否则 moov 里缺少 avcC/hvcC, 播放器黑屏.
 * 3. ★ fmp4 的 moov 在 fmp4_writer_init_segment 时一次性落盘, 之后不能再
 *    新增 track. 因此录制线程必须先等到首 IDR (视频参数集 + PTS 轴) 和
 *    音频配置 (G711 编码类型), 再创建 writer + add track + init_segment.
 * 4. 录制线程必须与拉流/解码线程分离 (消息队列解耦), 否则慢速磁盘
 *    写入会阻塞 RTP 接收, 导致解码卡顿/丢帧.
 * 5. 队列满时丢弃最旧帧而非阻塞生产者, 保证录制不拖垮拉流实时性.
 */

/* _GNU_SOURCE 由 Makefile 全局定义 (-D_GNU_SOURCE), 无需重复定义 */

#include "mp4_record.h"
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <pthread.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <sys/stat.h>

#include "fmp4-writer.h"
#include "mov-format.h"
#include "mpeg4-avc.h"
#include "mpeg4-hevc.h"

/* 录制信息队列项 (内部复用接口类型) */
typedef struct {
    uint8_t  *data;
    int       len;
    int       is_keyframe;
    uint64_t  pts;
    int       valid;
    int       is_audio;        /* 1=音频 G711 帧 (len=字节数) */
} MP4_RECORD_QUEUE_ITEM_S;

typedef struct {
    fmp4_writer_t      *writer;
    FILE               *fp;
    int                 video_track;
    int                 audio_track;
    int                 codec_type;
    int                 width;
    int                 height;
    int                 fps;
    char                file_path[512];
    int                 header_written;
    int                 first_pts_set;
    int64_t             start_pts_us;
    int64_t             last_pts_us;
    uint32_t            frame_idx;

    /* 视频参数集 (Annex-B 格式, 含 start code; 供构建 avcC/hvcC) */
    uint8_t             annexb[2048];
    int                 annexb_len;
    int                 video_config_ready;  /* avcC/hvcC 已可构建 */

    /* 音频轨 (G711a/u 直写 / AAC raw access unit 直写) */
    int                 audio_codec;         /* CSG_AUDIO_CODEC_G711A/U/AAC, 0=未配置 */
    int                 audio_sample_rate;
    int                 audio_channels;
    int64_t             last_audio_rel_us;   /* 上一音频帧相对 PTS (单调保护) */
    /* AAC 专属: AudioSpecificConfig (录制时写入 esds box) */
    uint8_t             audio_asc[64];
    int                 audio_asc_size;

    /* 消息队列 (环形缓冲区) */
    MP4_RECORD_QUEUE_ITEM_S queue[MP4_RECORD_QUEUE_MAX];
    int                 queue_write;
    int                 queue_read;
    int                 queue_count;
    pthread_mutex_t     queue_mutex;
    pthread_cond_t      queue_cond;

    /* 录制线程 */
    pthread_t           record_thread;
    int                 thread_running;
    int                 thread_exit_ok;

    MP4_RECORD_INFO_S   info;
} MP4_RECORD_CTX;

/* ==================== libmov mov_buffer 回调 (FILE*) ==================== */

static int record_io_write(void *param, const void *data, uint64_t bytes)
{
    MP4_RECORD_CTX *ctx = (MP4_RECORD_CTX *)param;
    if (!ctx || !ctx->fp || !data) return -EINVAL;
    if (bytes > 0 && fwrite(data, 1, (size_t)bytes, ctx->fp) != bytes)
        return -EIO;
    return 0;
}

static int record_io_seek(void *param, int64_t offset)
{
    MP4_RECORD_CTX *ctx = (MP4_RECORD_CTX *)param;
    if (!ctx || !ctx->fp) return -EIO;
    if (offset < 0)
        return fseeko(ctx->fp, offset, SEEK_END) == 0 ? 0 : -EIO;
    return fseeko(ctx->fp, offset, SEEK_SET) == 0 ? 0 : -EIO;
}

static int64_t record_io_tell(void *param)
{
    MP4_RECORD_CTX *ctx = (MP4_RECORD_CTX *)param;
    if (!ctx || !ctx->fp) return -1;
    return (int64_t)ftello(ctx->fp);
}

static struct mov_buffer_t s_record_buffer = {
    NULL,               /* read (写模式不需要) */
    record_io_write,
    record_io_seek,
    record_io_tell
};

/* ==================== Annex-B 工具 ==================== */

/* 找到 Annex-B start code (00 00 01 或 00 00 00 01), 返回 NAL 数据起始偏移 */
static int find_start_code(const uint8_t *buf, int size, int *code_len)
{
    for (int i = 0; i + 3 < size; i++) {
        if (buf[i] == 0 && buf[i+1] == 0 && buf[i+2] == 1) {
            *code_len = 3;
            return i + 3;
        }
        if (i + 4 < size && buf[i] == 0 && buf[i+1] == 0 &&
            buf[i+2] == 0 && buf[i+3] == 1) {
            *code_len = 4;
            return i + 4;
        }
    }
    return -1;
}

/*
 * 把一整帧 Annex-B (可能含多个 start code) 转换为 MP4 的 length-prefix 样本.
 * 跳过参数集/SEI NAL (已在 avcC/hvcC 中, 样本里重复写会导致部分播放器异常).
 * 返回新 malloc 缓冲区 (调用方 free), 大小写回 *out_len. 失败返回 NULL.
 */
static uint8_t *annexb_to_length_prefix(const uint8_t *data, int len,
                                        int codec_type, int *out_len)
{
    /* 最坏情况: 每个字节都可能变成 4 字节长度前缀 + 数据, 直接按 2x 预留 */
    uint8_t *out = malloc((size_t)len * 2 + 64);
    if (!out) return NULL;

    int o = 0;
    int off = 0;
    int code_len = 0;
    while (off < len) {
        /* off 始终指向某个 start code(或缓冲起点), find_start_code 立刻
         * 返回其载荷起点; 若是孤立的垃圾字节则会被当作帧头前的噪声跳过. */
        int start = find_start_code(data + off, len - off, &code_len);
        if (start < 0) break;
        int nal_start = off + start;

        /* 从当前 NAL 载荷起找下一个 start code, 界定本 NAL 结束位置 */
        int next = -1;
        for (int i = nal_start; i + 3 < len; i++) {
            if (data[i] == 0 && data[i+1] == 0 && data[i+2] == 1) { next = i; break; }
            if (i + 4 < len && data[i] == 0 && data[i+1] == 0 &&
                data[i+2] == 0 && data[i+3] == 1) { next = i; break; }
        }
        int nal_end = (next >= 0) ? next : len;
        int nal_len = nal_end - nal_start;
        if (nal_len > 0) {
            uint8_t nal_type = (codec_type == CSG_CODEC_H265)
                ? ((data[nal_start] >> 1) & 0x3F) : (data[nal_start] & 0x1F);
            /* H264: SPS(7)/PPS(8)/SEI(9); H265: VPS(32)/SPS(33)/PPS(34)/SEI(35) */
            int skip = (codec_type == CSG_CODEC_H265)
                ? (nal_type == 32 || nal_type == 33 || nal_type == 34 || nal_type == 35)
                : (nal_type == 7 || nal_type == 8 || nal_type == 9);
            if (!skip) {
                uint32_t n = (uint32_t)nal_len;
                out[o++] = (uint8_t)(n >> 24);
                out[o++] = (uint8_t)(n >> 16);
                out[o++] = (uint8_t)(n >> 8);
                out[o++] = (uint8_t)n;
                memcpy(out + o, data + nal_start, (size_t)nal_len);
                o += nal_len;
            }
        }
        /* 关键: 前进到下一个 start code 的 *起点*(next), 而不是其载荷
         * (next+ncode). 否则下一次 find_start_code 会从载荷中间开始扫描,
         * 把下一段 NAL 当噪声吞掉, 且缓冲尾部(最后一段 NAL)永远丢失 ——
         * 典型症状: 关键帧(VPS/SPS/PPS+IDR)只剩参数集, IDR 切片被丢掉. */
        off = (next >= 0) ? next : len;
    }

    *out_len = o;
    return out;
}

/* 从 Annex-B 参数集/首帧构建 avcC/hvcC (libflv). 返回配置字节数, <=0 失败 */
static int build_video_config(const uint8_t *annexb, int annexb_len,
                              int codec_type, uint8_t *config, size_t cap)
{
    if (codec_type == CSG_CODEC_H264) {
        struct mpeg4_avc_t avc;
        memset(&avc, 0, sizeof(avc));
        if (mpeg4_avc_from_nalu(annexb, (size_t)annexb_len, &avc) <= 0)
            return -1;
        if (avc.nb_sps < 1 || avc.nb_pps < 1)
            return -1;
        return mpeg4_avc_decoder_configuration_record_save(&avc, config, cap);
    } else {
        struct mpeg4_hevc_t hevc;
        memset(&hevc, 0, sizeof(hevc));
        if (mpeg4_hevc_from_nalu(annexb, (size_t)annexb_len, &hevc) <= 0)
            return -1;
        if (hevc.numOfArrays < 3)   /* VPS/SPS/PPS 三者齐全 */
            return -1;
        return mpeg4_hevc_decoder_configuration_record_save(&hevc, config, cap);
    }
}

/* ==================== fMP4 文件初始化 ==================== */

/*
 * 在录制线程中创建 fMP4 文件 + 视频/音频 track + init_segment.
 * 必须在视频参数集与音频配置都就绪后调用 (moov 一次性落盘, 之后不能加 track).
 * 返回 0 成功; 失败返回 -1 (文件已清理).
 */
static int record_init_mp4(MP4_RECORD_CTX *ctx)
{
    ctx->fp = fopen(ctx->file_path, "wb");
    if (!ctx->fp) {
        CSG_LOGE("RECORD", "fopen failed: %s", ctx->file_path);
        return -1;
    }

    ctx->writer = fmp4_writer_create(&s_record_buffer, ctx, MOV_FLAG_SEGMENT);
    if (!ctx->writer) {
        CSG_LOGE("RECORD", "fmp4_writer_create failed");
        fclose(ctx->fp);
        ctx->fp = NULL;
        return -1;
    }

    /* 视频轨: 构建 avcC/hvcC */
    uint8_t config[8192];
    int config_size = build_video_config(ctx->annexb, ctx->annexb_len,
                                         ctx->codec_type, config, sizeof(config));
    if (config_size <= 0) {
        CSG_LOGE("RECORD", "build_video_config failed (annexb_len=%d)", ctx->annexb_len);
        goto fail;
    }
    ctx->video_track = fmp4_writer_add_video(ctx->writer,
        (ctx->codec_type == CSG_CODEC_H264) ? MOV_OBJECT_H264 : MOV_OBJECT_H265,
        ctx->width, ctx->height, config, (size_t)config_size);
    if (ctx->video_track < 0) {
        CSG_LOGE("RECORD", "fmp4_writer_add_video failed");
        goto fail;
    }

    /* 音频轨 (G711a/u / AAC / MP3 / MP1A / AC3 / Opus / LPCM 直写)
     * ★ 扩展: 覆盖所有 libmov 支持的音频对象类型.
     * LPCM(L16/L8) 无原生 MOV_OBJECT, 用 G711u 兼容封装 (回放时做 μ-law 解码). */
    if (ctx->audio_codec == CSG_AUDIO_CODEC_G711A ||
        ctx->audio_codec == CSG_AUDIO_CODEC_G711U) {
        uint8_t object = (ctx->audio_codec == CSG_AUDIO_CODEC_G711A)
            ? MOV_OBJECT_G711a : MOV_OBJECT_G711u;
        ctx->audio_track = fmp4_writer_add_audio(ctx->writer, object,
            (ctx->audio_channels > 0) ? ctx->audio_channels : 1, 16,
            (ctx->audio_sample_rate > 0) ? ctx->audio_sample_rate : 8000,
            NULL, 0);
        if (ctx->audio_track < 0) {
            CSG_LOGE("RECORD", "fmp4_writer_add_audio failed");
            goto fail;
        }
        CSG_LOGI("RECORD", "Audio track added: G711 %s %dHz/%dch",
            (ctx->audio_codec == CSG_AUDIO_CODEC_G711A) ? "alaw" : "ulaw",
            ctx->audio_sample_rate, ctx->audio_channels);
    } else if (ctx->audio_codec == CSG_AUDIO_CODEC_AAC) {
        ctx->audio_track = fmp4_writer_add_audio(ctx->writer, MOV_OBJECT_AAC,
            (ctx->audio_channels > 0) ? ctx->audio_channels : 1, 16,
            (ctx->audio_sample_rate > 0) ? ctx->audio_sample_rate : 8000,
            (ctx->audio_asc_size > 0) ? ctx->audio_asc : NULL,
            (size_t)ctx->audio_asc_size);
        if (ctx->audio_track < 0) {
            CSG_LOGE("RECORD", "fmp4_writer_add_audio (AAC) failed");
            goto fail;
        }
        CSG_LOGI("RECORD", "Audio track added: AAC %dHz/%dch ASC=%d bytes",
            ctx->audio_sample_rate, ctx->audio_channels, ctx->audio_asc_size);
    } else if (ctx->audio_codec == CSG_AUDIO_CODEC_MP3) {
        ctx->audio_track = fmp4_writer_add_audio(ctx->writer, MOV_OBJECT_MP3,
            (ctx->audio_channels > 0) ? ctx->audio_channels : 1, 16,
            (ctx->audio_sample_rate > 0) ? ctx->audio_sample_rate : 44100,
            NULL, 0);
        if (ctx->audio_track < 0) {
            CSG_LOGE("RECORD", "fmp4_writer_add_audio (MP3) failed");
            goto fail;
        }
        CSG_LOGI("RECORD", "Audio track added: MP3 %dHz/%dch",
            ctx->audio_sample_rate, ctx->audio_channels);
    } else if (ctx->audio_codec == CSG_AUDIO_CODEC_MP1A) {
        ctx->audio_track = fmp4_writer_add_audio(ctx->writer, MOV_OBJECT_MP1A,
            (ctx->audio_channels > 0) ? ctx->audio_channels : 1, 16,
            (ctx->audio_sample_rate > 0) ? ctx->audio_sample_rate : 44100,
            NULL, 0);
        if (ctx->audio_track < 0) {
            CSG_LOGE("RECORD", "fmp4_writer_add_audio (MP1A) failed");
            goto fail;
        }
        CSG_LOGI("RECORD", "Audio track added: MP1A %dHz/%dch",
            ctx->audio_sample_rate, ctx->audio_channels);
    } else if (ctx->audio_codec == CSG_AUDIO_CODEC_AC3) {
        ctx->audio_track = fmp4_writer_add_audio(ctx->writer, MOV_OBJECT_AC3,
            (ctx->audio_channels > 0) ? ctx->audio_channels : 1, 16,
            (ctx->audio_sample_rate > 0) ? ctx->audio_sample_rate : 48000,
            NULL, 0);
        if (ctx->audio_track < 0) {
            CSG_LOGE("RECORD", "fmp4_writer_add_audio (AC3) failed");
            goto fail;
        }
        CSG_LOGI("RECORD", "Audio track added: AC3 %dHz/%dch",
            ctx->audio_sample_rate, ctx->audio_channels);
    } else if (ctx->audio_codec == CSG_AUDIO_CODEC_OPUS) {
        ctx->audio_track = fmp4_writer_add_audio(ctx->writer, MOV_OBJECT_OPUS,
            (ctx->audio_channels > 0) ? ctx->audio_channels : 1, 16,
            (ctx->audio_sample_rate > 0) ? ctx->audio_sample_rate : 48000,
            NULL, 0);
        if (ctx->audio_track < 0) {
            CSG_LOGE("RECORD", "fmp4_writer_add_audio (OPUS) failed");
            goto fail;
        }
        CSG_LOGI("RECORD", "Audio track added: OPUS %dHz/%dch",
            ctx->audio_sample_rate, ctx->audio_channels);
    } else if (ctx->audio_codec == CSG_AUDIO_CODEC_LPCM) {
        /* LPCM (L16/L8): libmov 无原生 PCM 对象, 用 G711u 兼容封装.
         * 原始字节直写, 回放时识别为 G711u 做 μ-law→PCM16 解码还原.
         * ★ 注意: L16 是 16-bit PCM, 但写入 G711u 轨时 libmov 按 8-bit 处理;
         *   实际数据是原始 16-bit PCM 字节, 需要在回放时做特殊处理. */
        ctx->audio_track = fmp4_writer_add_audio(ctx->writer, MOV_OBJECT_G711u,
            (ctx->audio_channels > 0) ? ctx->audio_channels : 1, 16,
            (ctx->audio_sample_rate > 0) ? ctx->audio_sample_rate : 8000,
            NULL, 0);
        if (ctx->audio_track < 0) {
            CSG_LOGE("RECORD", "fmp4_writer_add_audio (LPCM→G711u) failed");
            goto fail;
        }
        CSG_LOGI("RECORD", "Audio track added: LPCM %dHz/%dch (wrapped as G711u)",
            ctx->audio_sample_rate, ctx->audio_channels);
    }

    /* → [ftyp][moov] */
    if (fmp4_writer_init_segment(ctx->writer) != 0) {
        CSG_LOGE("RECORD", "fmp4_writer_init_segment failed");
        goto fail;
    }
    fflush(ctx->fp);
    ctx->header_written = 1;
    CSG_LOGI("RECORD", "fMP4 init OK: %s, vtrack=%d atrack=%d, %dx%d@%d, codec=%d",
        ctx->file_path, ctx->video_track, ctx->audio_track,
        ctx->width, ctx->height, ctx->fps, ctx->codec_type);
    return 0;

fail:
    if (ctx->writer) {
        fmp4_writer_destroy(ctx->writer);
        ctx->writer = NULL;
    }
    if (ctx->fp) {
        fclose(ctx->fp);
        ctx->fp = NULL;
    }
    remove(ctx->file_path);
    return -1;
}

/* ==================== 帧写入 ==================== */

static void record_write_video(MP4_RECORD_CTX *ctx, MP4_RECORD_QUEUE_ITEM_S *item)
{
    if (ctx->video_track < 0 || !ctx->writer) return;

    /* 相对 PTS (微秒, 以首 IDR 为 0 点) */
    int64_t rel_pts = (int64_t)item->pts - ctx->start_pts_us;
    if (rel_pts < 0) rel_pts = 0;

    /* PTS 单调性保护: 不得倒退 */
    if (ctx->last_pts_us >= 0 && rel_pts <= ctx->last_pts_us) {
        int64_t frame_dur = (ctx->fps > 0) ? (1000000LL / ctx->fps) : 40000LL;
        rel_pts = ctx->last_pts_us + frame_dur;
    }
    ctx->last_pts_us = rel_pts;

    /* libmov 时间戳单位 = ms */
    int64_t ts_ms = rel_pts / 1000;

    int lp_len = 0;
    uint8_t *lp = annexb_to_length_prefix(item->data, item->len,
                                          ctx->codec_type, &lp_len);
    if (lp && lp_len > 0) {
        int flags = item->is_keyframe ? MOV_AV_FLAG_KEYFREAME : 0;
        if (fmp4_writer_write(ctx->writer, ctx->video_track, lp,
                              (size_t)lp_len, ts_ms, ts_ms, flags) == 0) {
            ctx->frame_idx++;
            ctx->info.frame_count++;
            if (item->is_keyframe) ctx->info.keyframe_count++;
            ctx->info.duration_us = rel_pts;
        } else {
            CSG_LOGE("RECORD", "fmp4_writer_write video failed (len=%d, ts=%lld)",
                lp_len, (long long)ts_ms);
            ctx->info.state = MP4_RECORD_STATE_ERROR;
        }
    } else {
        CSG_LOGW("RECORD", "annexb_to_length_prefix failed (len=%d)", item->len);
    }
    free(lp);
}

static void record_write_audio(MP4_RECORD_CTX *ctx, MP4_RECORD_QUEUE_ITEM_S *item)
{
    if (ctx->audio_track < 0 || !ctx->writer) return;
    if (ctx->audio_codec == CSG_AUDIO_CODEC_NONE) return;
    if (!ctx->first_pts_set) return;   /* 尚未等到视频首 IDR: 丢弃前置音频 */

    /* 音频 PTS 归一化: 以视频首 IDR 为公共时间轴基准 */
    int64_t a_rel = (int64_t)item->pts - ctx->start_pts_us;
    if (a_rel < 0) return;
    /* 单调保护: PTS 倒退直接丢弃 (音频断流容忍) */
    if (ctx->last_audio_rel_us >= 0 && a_rel <= ctx->last_audio_rel_us)
        return;
    ctx->last_audio_rel_us = a_rel;

    int64_t ts_ms = a_rel / 1000;
    if (fmp4_writer_write(ctx->writer, ctx->audio_track, item->data,
                          (size_t)item->len, ts_ms, ts_ms, 0) == 0) {
        ctx->info.audio_frame_count++;
    } else {
        CSG_LOGE("RECORD", "fmp4_writer_write audio failed (len=%d, ts=%lld)",
            item->len, (long long)ts_ms);
    }
}

/* ==================== 队列工具 ==================== */

/* 从队列取一项 (阻塞). 返回 0 成功; -1 线程已停止且队列空 */
static int record_pop_item(MP4_RECORD_CTX *ctx, MP4_RECORD_QUEUE_ITEM_S *out)
{
    pthread_mutex_lock(&ctx->queue_mutex);

    while (ctx->queue_count == 0 && ctx->thread_running) {
        pthread_cond_wait(&ctx->queue_cond, &ctx->queue_mutex);
    }
    if (!ctx->thread_running && ctx->queue_count == 0) {
        pthread_mutex_unlock(&ctx->queue_mutex);
        return -1;
    }

    int idx = ctx->queue_read;
    *out = ctx->queue[idx];
    ctx->queue[idx].valid = 0;
    ctx->queue[idx].data = NULL;   /* 所有权转移 */
    ctx->queue_read = (ctx->queue_read + 1) % MP4_RECORD_QUEUE_MAX;
    ctx->queue_count--;

    pthread_mutex_unlock(&ctx->queue_mutex);
    return 0;
}

/* 短暂等待音频配置 (G711 编码类型在首音频帧到达前未知) */
static void record_wait_audio_config(MP4_RECORD_CTX *ctx, int timeout_ms)
{
    struct timespec deadline;
    clock_gettime(CLOCK_REALTIME, &deadline);
    deadline.tv_nsec += (long)timeout_ms * 1000000L;
    if (deadline.tv_nsec >= 1000000000L) {
        deadline.tv_sec += deadline.tv_nsec / 1000000000L;
        deadline.tv_nsec %= 1000000000L;
    }

    pthread_mutex_lock(&ctx->queue_mutex);
    while (ctx->audio_codec == CSG_AUDIO_CODEC_NONE && ctx->thread_running) {
        int rc = pthread_cond_timedwait(&ctx->queue_cond, &ctx->queue_mutex,
                                        &deadline);
        if (rc == ETIMEDOUT) break;
    }
    pthread_mutex_unlock(&ctx->queue_mutex);
}

/* ==================== 录制线程 ==================== */

static void *record_thread(void *arg)
{
    MP4_RECORD_CTX *ctx = (MP4_RECORD_CTX *)arg;
    CSG_LOGI("RECORD", "Record thread started");

    /* Phase 1: 等待首 IDR (建立 PTS 轴 + 视频参数集兜底) */
    MP4_RECORD_QUEUE_ITEM_S first_idr;
    memset(&first_idr, 0, sizeof(first_idr));
    int got_idr = 0;
    while (1) {
        MP4_RECORD_QUEUE_ITEM_S item;
        if (record_pop_item(ctx, &item) != 0) break;
        if (!item.valid || !item.data || item.len <= 0) { free(item.data); continue; }
        if (item.is_audio) { free(item.data); continue; }   /* IDR 前音频丢弃 */
        if (item.is_keyframe) {
            first_idr = item;
            got_idr = 1;
            break;
        }
        free(item.data);   /* IDR 前非关键帧丢弃 */
    }

    if (!got_idr) {
        /* 停止时仍未等到首 IDR: 无可录内容, 不建文件 */
        CSG_LOGW("RECORD", "No IDR before stop, recording aborted");
        ctx->info.state = MP4_RECORD_STATE_IDLE;
        ctx->thread_running = 0;
        ctx->thread_exit_ok = 1;
        return NULL;
    }

    ctx->start_pts_us = (int64_t)first_idr.pts;
    ctx->first_pts_set = 1;

    /* 视频参数集兜底: extradata 参数集不完整时 (如 SDP 无 sprop-sps),
     * 从首 IDR 帧内 Annex-B 提取 VPS/SPS/PPS 构建 avcC/hvcC. */
    if (!ctx->video_config_ready) {
        uint8_t config[8192];
        int cs = build_video_config(first_idr.data, first_idr.len,
                                    ctx->codec_type, config, sizeof(config));
        if (cs > 0) {
            int n = first_idr.len;
            if (n > (int)sizeof(ctx->annexb)) n = (int)sizeof(ctx->annexb);
            memcpy(ctx->annexb, first_idr.data, (size_t)n);
            ctx->annexb_len = n;
            ctx->video_config_ready = 1;
            CSG_LOGI("RECORD", "Video param sets built from first IDR (len=%d)", n);
        } else {
            CSG_LOGE("RECORD", "No complete param sets in first IDR, abort");
            free(first_idr.data);
            ctx->info.state = MP4_RECORD_STATE_ERROR;
            ctx->thread_running = 0;
            ctx->thread_exit_ok = 1;
            return NULL;
        }
    }

    /* Phase 2: 等待音频配置 (无音频流则超时后仅录视频).
     * ★ v6: 音频轨参数已由 rec_connect 在 mp4_record_start 前从 SDP 同步配置
     *   (codec/采样率/声道/ASC), 此处 codec 通常已就绪, 等待立即返回.
     *   仅当某个入口确实依赖首音频帧配置时才需等待 —— 超时从 3s 缩短到 300ms:
     *   3s 阻塞期间队列无人消费, 40 帧环形缓冲被写满后逐帧丢弃, 录制会丢失
     *   开头约 3s 视频 (见运行日志 "Queue full, dropped oldest frame" 风暴). */
    record_wait_audio_config(ctx, 300);
    if (ctx->audio_codec == CSG_AUDIO_CODEC_NONE) {
        CSG_LOGI("RECORD", "No audio config, recording video-only");
    }

    /* Phase 3: 创建 writer + track + init_segment */
    if (record_init_mp4(ctx) != 0) {
        free(first_idr.data);
        ctx->info.state = MP4_RECORD_STATE_ERROR;
        ctx->thread_running = 0;
        ctx->thread_exit_ok = 1;
        return NULL;
    }

    /* Phase 4: 写首 IDR */
    record_write_video(ctx, &first_idr);
    free(first_idr.data);

    /* Phase 5: 主循环 */
    while (1) {
        MP4_RECORD_QUEUE_ITEM_S item;
        if (record_pop_item(ctx, &item) != 0) break;
        if (!item.valid || !item.data || item.len <= 0) { free(item.data); continue; }

        if (item.is_audio) {
            record_write_audio(ctx, &item);
        } else {
            record_write_video(ctx, &item);
        }
        free(item.data);
    }

    /* 关闭: 写最后 fragment + mfra, 再关文件 */
    if (ctx->writer) {
        fmp4_writer_destroy(ctx->writer);
        ctx->writer = NULL;
    }
    if (ctx->fp) {
        fflush(ctx->fp);
        fdatasync(fileno(ctx->fp));
        fclose(ctx->fp);
        ctx->fp = NULL;
    }
    ctx->header_written = 0;

    struct stat st;
    if (stat(ctx->file_path, &st) == 0) {
        ctx->info.file_size = st.st_size;
    }

    ctx->thread_exit_ok = 1;
    CSG_LOGI("RECORD", "Record thread exit: frames=%u keyframes=%u audio=%u "
        "duration=%.1f sec size=%lld",
        ctx->info.frame_count, ctx->info.keyframe_count, ctx->info.audio_frame_count,
        (double)ctx->info.duration_us / 1000000.0,
        (long long)ctx->info.file_size);
    return NULL;
}

/* ==================== 创建 / 销毁 ==================== */

void *mp4_record_create(const char *output_path, int codec_type,
                        int width, int height, int fps,
                        const uint8_t *extradata_annexb, int extradata_len)
{
    if (!output_path) return NULL;

    MP4_RECORD_CTX *ctx = calloc(1, sizeof(MP4_RECORD_CTX));
    if (!ctx) return NULL;

    strncpy(ctx->file_path, output_path, sizeof(ctx->file_path) - 1);
    ctx->codec_type = codec_type;
    ctx->width = width;
    ctx->height = height;
    ctx->fps = fps;
    ctx->start_pts_us = -1;
    ctx->last_pts_us = -1;
    ctx->last_audio_rel_us = -1;
    ctx->video_track = -1;
    ctx->audio_track = -1;
    pthread_mutex_init(&ctx->queue_mutex, NULL);
    pthread_cond_init(&ctx->queue_cond, NULL);

    /* 保存 Annex-B 参数集 (供构建 avcC/hvcC); 预检完整性 */
    if (extradata_annexb && extradata_len > 0) {
        int n = extradata_len;
        if (n > (int)sizeof(ctx->annexb)) n = (int)sizeof(ctx->annexb);
        memcpy(ctx->annexb, extradata_annexb, (size_t)n);
        ctx->annexb_len = n;
        uint8_t config[64];
        if (build_video_config(ctx->annexb, ctx->annexb_len, codec_type,
                               config, sizeof(config)) > 0) {
            ctx->video_config_ready = 1;
        }
    }

    /* 确保目录存在 */
    char dir_path[512];
    strncpy(dir_path, output_path, sizeof(dir_path) - 1);
    char *last_slash = strrchr(dir_path, '/');
    if (last_slash) {
        *last_slash = '\0';
        mkdir(dir_path, 0755);
    }

    CSG_LOGI("RECORD", "Created: path=%s, codec=%d, %dx%d@%d, config_ready=%d",
        output_path, codec_type, width, height, fps, ctx->video_config_ready);
    return ctx;
}

void mp4_record_destroy(void *handle)
{
    if (!handle) return;
    MP4_RECORD_CTX *ctx = (MP4_RECORD_CTX *)handle;

    mp4_record_stop(ctx);

    /* 清理队列中残留的未处理帧 */
    for (int i = 0; i < MP4_RECORD_QUEUE_MAX; i++) {
        if (ctx->queue[i].data) {
            free(ctx->queue[i].data);
            ctx->queue[i].data = NULL;
        }
    }

    pthread_mutex_destroy(&ctx->queue_mutex);
    pthread_cond_destroy(&ctx->queue_cond);
    free(ctx);
    CSG_LOGI("RECORD", "Destroyed");
}

/* ==================== 开始录制 ==================== */

int mp4_record_start(void *handle)
{
    if (!handle) return -1;
    MP4_RECORD_CTX *ctx = (MP4_RECORD_CTX *)handle;

    pthread_mutex_lock(&ctx->queue_mutex);

    if (ctx->thread_running) {
        pthread_mutex_unlock(&ctx->queue_mutex);
        return -1;
    }

    ctx->header_written = 0;
    ctx->first_pts_set = 0;
    ctx->frame_idx = 0;
    ctx->last_pts_us = -1;
    ctx->start_pts_us = -1;
    /* 音频: 时间戳复位, 编码配置保留 (config_audio 可先/后于 start) */
    ctx->last_audio_rel_us = -1;
    ctx->queue_write = 0;
    ctx->queue_read = 0;
    ctx->queue_count = 0;
    memset(ctx->queue, 0, sizeof(ctx->queue));
    ctx->info.state = MP4_RECORD_STATE_RECORDING;
    ctx->info.frame_count = 0;
    ctx->info.keyframe_count = 0;
    ctx->info.audio_frame_count = 0;
    ctx->info.duration_us = 0;
    ctx->info.file_size = 0;

    ctx->thread_running = 1;
    ctx->thread_exit_ok = 0;
    pthread_create(&ctx->record_thread, NULL, record_thread, ctx);

    pthread_mutex_unlock(&ctx->queue_mutex);

    CSG_LOGI("RECORD", "Started: %s (libmov fmp4, queue=%d)",
        ctx->file_path, MP4_RECORD_QUEUE_MAX);
    return 0;
}

/* ==================== 推送帧到消息队列 ==================== */

int mp4_record_push_frame(void *handle, void *data, int len,
                          int is_keyframe, uint64_t pts_us)
{
    if (!handle || !data || len <= 0) return -1;
    MP4_RECORD_CTX *ctx = (MP4_RECORD_CTX *)handle;

    pthread_mutex_lock(&ctx->queue_mutex);

    if (!ctx->thread_running || ctx->info.state != MP4_RECORD_STATE_RECORDING) {
        pthread_mutex_unlock(&ctx->queue_mutex);
        return -1;
    }

    /* 环形队列满时丢弃最旧帧 (录制不阻塞拉流) */
    if (ctx->queue_count >= MP4_RECORD_QUEUE_MAX) {
        int old_idx = ctx->queue_read;
        if (ctx->queue[old_idx].data) {
            free(ctx->queue[old_idx].data);
        }
        ctx->queue[old_idx].valid = 0;
        ctx->queue_read = (ctx->queue_read + 1) % MP4_RECORD_QUEUE_MAX;
        ctx->queue_count--;
        CSG_LOGW("RECORD", "Queue full, dropped oldest frame");
    }

    /* 写入新帧 (拷贝, 避免调用方缓冲生命周期问题) */
    int idx = ctx->queue_write;
    ctx->queue[idx].data = malloc((size_t)len);
    if (!ctx->queue[idx].data) {
        pthread_mutex_unlock(&ctx->queue_mutex);
        return -1;
    }
    memcpy(ctx->queue[idx].data, data, (size_t)len);
    ctx->queue[idx].len = len;
    ctx->queue[idx].is_keyframe = is_keyframe;
    ctx->queue[idx].pts = pts_us;
    ctx->queue[idx].valid = 1;
    ctx->queue[idx].is_audio = 0;        /* 槽位复用: 必须显式清音频标志 */

    ctx->queue_write = (ctx->queue_write + 1) % MP4_RECORD_QUEUE_MAX;
    ctx->queue_count++;

    pthread_cond_signal(&ctx->queue_cond);
    pthread_mutex_unlock(&ctx->queue_mutex);

    return 0;
}

/* ==================== 停止录制 ==================== */

int mp4_record_stop(void *handle)
{
    if (!handle) return -1;
    MP4_RECORD_CTX *ctx = (MP4_RECORD_CTX *)handle;

    pthread_mutex_lock(&ctx->queue_mutex);

    if (!ctx->thread_running) {
        pthread_mutex_unlock(&ctx->queue_mutex);
        return 0;
    }

    ctx->thread_running = 0;
    ctx->info.state = MP4_RECORD_STATE_STOPPING;
    pthread_cond_signal(&ctx->queue_cond);
    pthread_mutex_unlock(&ctx->queue_mutex);

    /* 等待录制线程退出 (最多等 5 秒) */
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += 5;

    int join_ret = pthread_timedjoin_np(ctx->record_thread, NULL, &ts);
    if (join_ret != 0) {
        /* 强制取消并等待: 避免与 destroy 的 free 竞争 */
        CSG_LOGW("RECORD", "Record thread join timeout, force cancel");
        pthread_cancel(ctx->record_thread);
        pthread_join(ctx->record_thread, NULL);
        if (ctx->writer) {
            fmp4_writer_destroy(ctx->writer);
            ctx->writer = NULL;
        }
        if (ctx->fp) {
            fclose(ctx->fp);
            ctx->fp = NULL;
        }
        ctx->header_written = 0;
    }

    ctx->info.state = MP4_RECORD_STATE_IDLE;

    CSG_LOGI("RECORD", "Stopped: %s, frames=%u keyframes=%u audio=%u "
        "duration=%.1f sec, size=%.1f MB",
        ctx->file_path, ctx->info.frame_count, ctx->info.keyframe_count,
        ctx->info.audio_frame_count,
        (double)ctx->info.duration_us / 1000000.0,
        (double)ctx->info.file_size / (1024.0 * 1024.0));
    return 0;
}

/* ==================== 查询信息 ==================== */

int mp4_record_get_info(void *handle, MP4_RECORD_INFO_S *info)
{
    if (!handle || !info) return -1;
    MP4_RECORD_CTX *ctx = (MP4_RECORD_CTX *)handle;

    pthread_mutex_lock(&ctx->queue_mutex);
    memcpy(info, &ctx->info, sizeof(MP4_RECORD_INFO_S));
    pthread_mutex_unlock(&ctx->queue_mutex);

    return 0;
}

/* ==================== 音频配置 / 推送 ==================== */

int mp4_record_config_audio(void *handle, int audio_codec,
                            int sample_rate, int channels)
{
    if (!handle || sample_rate <= 0 || channels <= 0) return -1;
    /* ★ 扩展: 允许所有 libmov 支持的音频编码类型 (与 common.h 同步) */
    if (audio_codec != CSG_AUDIO_CODEC_G711A &&
        audio_codec != CSG_AUDIO_CODEC_G711U &&
        audio_codec != CSG_AUDIO_CODEC_AAC &&
        audio_codec != CSG_AUDIO_CODEC_MP3 &&
        audio_codec != CSG_AUDIO_CODEC_MP1A &&
        audio_codec != CSG_AUDIO_CODEC_AC3 &&
        audio_codec != CSG_AUDIO_CODEC_OPUS &&
        audio_codec != CSG_AUDIO_CODEC_LPCM) return -1;
    MP4_RECORD_CTX *ctx = (MP4_RECORD_CTX *)handle;

    pthread_mutex_lock(&ctx->queue_mutex);
    /* 幂等: 已配置且参数一致直接返回; 参数不一致以首次为准 */
    if (ctx->audio_codec != CSG_AUDIO_CODEC_NONE) {
        int same = (ctx->audio_codec == audio_codec)
                && (ctx->audio_sample_rate == sample_rate)
                && (ctx->audio_channels == channels);
        pthread_mutex_unlock(&ctx->queue_mutex);
        return same ? 0 : -1;
    }
    ctx->audio_codec = audio_codec;
    ctx->audio_sample_rate = sample_rate;
    ctx->audio_channels = channels;
    pthread_cond_signal(&ctx->queue_cond);   /* 唤醒 record_init 的音频等待 */
    pthread_mutex_unlock(&ctx->queue_mutex);

    CSG_LOGI("RECORD", "Audio configured: %s %dHz/%dch",
        (audio_codec == CSG_AUDIO_CODEC_G711A) ? "G711 alaw" :
        (audio_codec == CSG_AUDIO_CODEC_G711U) ? "G711 ulaw" :
        (audio_codec == CSG_AUDIO_CODEC_AAC)   ? "AAC" :
        (audio_codec == CSG_AUDIO_CODEC_MP3)   ? "MP3" :
        (audio_codec == CSG_AUDIO_CODEC_MP1A)  ? "MP1A" :
        (audio_codec == CSG_AUDIO_CODEC_AC3)   ? "AC3" :
        (audio_codec == CSG_AUDIO_CODEC_OPUS)  ? "Opus" :
        (audio_codec == CSG_AUDIO_CODEC_LPCM)  ? "LPCM" : "?",
        sample_rate, channels);
    return 0;
}

int mp4_record_set_audio_asc(void *handle, const uint8_t *asc, int asc_size)
{
    if (!handle || !asc || asc_size <= 0) return -1;
    if (asc_size > (int)sizeof(((MP4_RECORD_CTX *)handle)->audio_asc)) {
        CSG_LOGE("RECORD", "ASC too large: %d > %d",
             asc_size, (int)sizeof(((MP4_RECORD_CTX *)handle)->audio_asc));
        return -1;
    }
    MP4_RECORD_CTX *ctx = (MP4_RECORD_CTX *)handle;

    pthread_mutex_lock(&ctx->queue_mutex);
    if (ctx->audio_asc_size > 0) {
        /* 幂等: 已设置则直接返回 */
        pthread_mutex_unlock(&ctx->queue_mutex);
        return 0;
    }
    memcpy(ctx->audio_asc, asc, (size_t)asc_size);
    ctx->audio_asc_size = asc_size;
    pthread_mutex_unlock(&ctx->queue_mutex);

    CSG_LOGI("RECORD", "Audio ASC set: %d bytes", asc_size);
    return 0;
}

int mp4_record_push_audio(void *handle, const uint8_t *data, int len,
                          uint64_t pts_us)
{
    if (!handle || !data || len <= 0) return -1;
    MP4_RECORD_CTX *ctx = (MP4_RECORD_CTX *)handle;

    pthread_mutex_lock(&ctx->queue_mutex);

    if (!ctx->thread_running || ctx->info.state != MP4_RECORD_STATE_RECORDING) {
        pthread_mutex_unlock(&ctx->queue_mutex);
        return -1;
    }
    /* 队列满时丢最旧帧 (与视频一致, 录制不阻塞拉流/音频回调) */
    if (ctx->queue_count >= MP4_RECORD_QUEUE_MAX) {
        int old_idx = ctx->queue_read;
        if (ctx->queue[old_idx].data) {
            free(ctx->queue[old_idx].data);
        }
        ctx->queue[old_idx].valid = 0;
        ctx->queue_read = (ctx->queue_read + 1) % MP4_RECORD_QUEUE_MAX;
        ctx->queue_count--;
        CSG_LOGW("RECORD", "Queue full, dropped oldest frame (audio)");
    }

    int idx = ctx->queue_write;
    ctx->queue[idx].data = malloc((size_t)len);
    if (!ctx->queue[idx].data) {
        pthread_mutex_unlock(&ctx->queue_mutex);
        return -1;
    }
    memcpy(ctx->queue[idx].data, data, (size_t)len);
    ctx->queue[idx].len = len;
    ctx->queue[idx].is_keyframe = 0;
    ctx->queue[idx].pts = pts_us;
    ctx->queue[idx].valid = 1;
    ctx->queue[idx].is_audio = 1;

    ctx->queue_write = (ctx->queue_write + 1) % MP4_RECORD_QUEUE_MAX;
    ctx->queue_count++;

    pthread_cond_signal(&ctx->queue_cond);
    pthread_mutex_unlock(&ctx->queue_mutex);

    return 0;
}