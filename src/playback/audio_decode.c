/* SPDX-License-Identifier: MIT */
/*
 * audio_decode.c - 音频解码实现 (FAAD2 / libfaad)
 *
 * ★ 替换说明:
 *   AAC 解码采用 faad2, 按硬件优先 / 零软转原则,
 *   本项目本地回放音轨为 AAC, 选用 FAAD2 软解。
 *   FAAD2 无重采样器 → 本模块直接输出 AAC 原生采样率/声道数,
 *   ALSA 渲染层按实际参数打开设备, 避免 CPU 重采样带来的音质损失。
 *
 * 核心流程:
 *   MP4 AAC sample (RAW data, 无 ADTS 头) + ASC (AudioSpecificConfig)
 *   → NeAACDecInit2(ASC) 初始化
 *   → NeAACDecDecode(sample) → S16LE 交错 PCM → 回调
 *
 * 注意 (历史踩坑点):
 *   1. NVR 的 MP4 音轨 esds box 必须含 ASC, 否则 NeAACDecInit2 失败。
 *      create() 依赖该 extradata。
 *   2. FAAD2 一包通常解一帧(1024样本/声道), 但为稳妥按 bytesconsumed
 *      循环解码, 兼容"一 sample 多 AU"的异常码流。
 *   3. 输出声道可能为 mono(1ch) 或 stereo(2ch), 不强制混音, ALSA 适配。
 */

#include "audio_decode.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* FAAD2 编解码接口 */
#include <neaacdec.h>

#define LOG_TAG "AUDIO_DEC"
#include "common.h"

struct csg_audio_decode_ctx {
    NeAACDecHandle handle;        /* FAAD2 解码句柄 */
    int            sample_rate;   /* AAC 原生采样率 (Hz) */
    int            channels;      /* AAC 原生声道数 */
    int            initialized;
    int            frame_count;   /* 统计 */
    csg_audio_decode_callback callback;
    void          *user_data;
};

csg_audio_decode_ctx_t *csg_audio_decode_create(int codec_id,
                                      const uint8_t *extradata,
                                      int extradata_size,
                                      int sample_rate,
                                      int channels,
                                      uint64_t channel_layout,
                                      csg_audio_decode_callback callback,
                                      void *user_data)
{
    (void)channel_layout;

    if (!callback) {
        CSG_LOGE("AUDIO_DEC", "Callback is NULL");
        return NULL;
    }

    if (codec_id != CSG_AUDIO_CODEC_AAC) {
        CSG_LOGE("AUDIO_DEC", "Unsupported audio codec_id=%d (only AAC)", codec_id);
        return NULL;
    }

    csg_audio_decode_ctx_t *ctx = calloc(1, sizeof(csg_audio_decode_ctx_t));
    if (!ctx) return NULL;
    ctx->callback = callback;
    ctx->user_data = user_data;

    /* 打开 FAAD2 解码器 */
    ctx->handle = NeAACDecOpen();
    if (!ctx->handle) {
        CSG_LOGE("AUDIO_DEC", "NeAACDecOpen failed");
        free(ctx);
        return NULL;
    }

    /* 配置: 输出 16 位 PCM; 保留下混(>2ch 降为 2ch); 默认 LC */
    NeAACDecConfigurationPtr conf = NeAACDecGetCurrentConfiguration(ctx->handle);
    if (conf) {
        conf->defObjectType = LC;
        conf->outputFormat  = FAAD_FMT_16BIT;
        conf->downMatrix    = 1;   /* 5.1 → 2.0, 避免 ALSA 多声道处理 */
        NeAACDecSetConfiguration(ctx->handle, conf);
    }

    unsigned long sr = (sample_rate > 0) ? (unsigned long)sample_rate : 48000;
    unsigned char ch = (channels > 0)   ? (unsigned char)channels : 2;

    /* 用 ASC (DecoderSpecificInfo) 初始化
     * ★ 必须先用 MP4v2 从 esds 取回 ASC (见 local_playback 的
     *   csg_local_playback_get_audio_info), 缺失则解码无法启动. */
    if (extradata && extradata_size > 0) {
        char r = NeAACDecInit2(ctx->handle, (unsigned char *)extradata,
                               (unsigned long)extradata_size, &sr, &ch);
        if (r != 0) {
            CSG_LOGE("AUDIO_DEC", "NeAACDecInit2 failed (bad ASC, size=%d)",
                 extradata_size);
            NeAACDecClose(ctx->handle);
            free(ctx);
            return NULL;
        }
    } else {
        CSG_LOGE("AUDIO_DEC", "No ASC extradata, cannot init FAAD2");
        NeAACDecClose(ctx->handle);
        free(ctx);
        return NULL;
    }

    ctx->sample_rate = (int)sr;
    ctx->channels    = (int)ch;
    ctx->initialized = 1;

    CSG_LOGI("AUDIO_DEC", "Created (FAAD2): AAC native %dHz/%dch",
         ctx->sample_rate, ctx->channels);

    return ctx;
}

void csg_audio_decode_destroy(csg_audio_decode_ctx_t *ctx)
{
    if (!ctx) return;
    if (ctx->handle) NeAACDecClose(ctx->handle);
    CSG_LOGI("AUDIO_DEC", "Destroyed (decoded %d frames)", ctx->frame_count);
    free(ctx);
}

int csg_audio_decode_send_packet(csg_audio_decode_ctx_t *ctx, const uint8_t *data,
                             int len, int64_t pts)
{
    if (!ctx || !ctx->initialized || !data || len <= 0) return -1;

    const uint8_t *in = data;
    int off = 0;
    int64_t frame_pts = pts;

    while (off < len) {
        NeAACDecFrameInfo info;
        memset(&info, 0, sizeof(info));

        void *pcm = NeAACDecDecode(ctx->handle, &info,
                                   (unsigned char *)in + off,
                                   (unsigned long)(len - off));

        if (info.error > 0) {
            CSG_LOGW("AUDIO_DEC", "FAAD2 decode error=%d (%s)",
                 info.error,
                 info.error ? (NeAACDecGetErrorMessage(info.error) ?
                 NeAACDecGetErrorMessage(info.error) : "") : "");
            /* 错误帧跳过, 不递增 offset 会死循环, 必须保证前进 */
            if (info.bytesconsumed > 0) off += (int)info.bytesconsumed;
            else ++off;
            frame_pts += 1024; /* 近似推进 */
            continue;
        }

        if (info.bytesconsumed <= 0) break;
        off += (int)info.bytesconsumed;

        if (pcm && info.samples > 0 && info.channels > 0) {
            /* FAAD2 info.samples 为每声道采样数(通常 1024), 与
             * audio_render_write 的"每通道采样数"语义一致. */
            ctx->frame_count++;
            ctx->callback(pcm, (int)info.samples, (int)info.channels,
                          (int)info.samplerate, frame_pts, ctx->user_data);
        }

        /* 帧时间步进: ~1024 样本/声道 */
        if (info.samplerate > 0) {
            frame_pts += (int64_t)((double)info.samples * 1000000.0
                                   / info.samplerate);
        }
    }

    return 0;
}

int csg_audio_decode_flush(csg_audio_decode_ctx_t *ctx)
{
    /* FAAD2 为按帧独立解码, 无内部缓冲, 无需冲刷 */
    (void)ctx;
    return 0;
}

void csg_audio_decode_reset(csg_audio_decode_ctx_t *ctx)
{
    if (!ctx) return;
    /* Seek 后复位丢包同步 (frame=-1 不完全复位, 用 0 全复位) */
    NeAACDecPostSeekReset(ctx->handle, 0);
    CSG_LOGI("AUDIO_DEC", "Reset (PostSeekReset)");
}

int csg_audio_decode_get_target_rate(csg_audio_decode_ctx_t *ctx)
{
    return ctx ? ctx->sample_rate : CSG_AUDIO_DECODE_TARGET_RATE;
}

int csg_audio_decode_get_target_channels(csg_audio_decode_ctx_t *ctx)
{
    return ctx ? ctx->channels : CSG_AUDIO_DECODE_TARGET_CHANNELS;
}