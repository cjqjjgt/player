/* SPDX-License-Identifier: MIT */
/*
 * audio_decode.h - 音频解码模块
 *
 * 使用 FAAD2 (libfaad) 软解码 AAC (AAC-LC / HE-AAC etc).
 * FAAD2 不携带重采样器, 因此输出为 AAC 原生采样率/声道数,
 * ALSA 渲染层按实际参数适配 (见 csg_audio_decode_get_target_rate).
 *
 * 依赖第三方库:
 *   头文件: thirdlib/faad2/neaacdec.h
 *   链接库: lib/libfaad.a (已由 build/cmake-aarch64-toolchain.cmake 交叉编译)
 */

#ifndef __AUDIO_DECODE_H__
#define __AUDIO_DECODE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 目标 PCM 格式 (FAAD2 FAAD_FMT_16BIT → S16LE 交错) */
#define CSG_AUDIO_DECODE_TARGET_RATE      48000   /* 默认回退, 实际以 AAC 原生为准 */
#define CSG_AUDIO_DECODE_TARGET_CHANNELS  2       /* 默认回退, 实际以 AAC 原生为准 */
#define CSG_AUDIO_DECODE_TARGET_FORMAT    0x10    /* 0x10: S16LE */

/* 音频解码回调: 输出 PCM 帧, 采样率为 AAC 原生采样率(可能 44.1k/48k etc)
 *   pcm_data:  PCM 采样数据 (S16LE 交错)
 *   samples:   采样数 (每通道)
 *   channels:  声道数
 *   rate:      采样率 (Hz)
 *   pts:       该帧的 PTS (微秒)
 *   user_data: 用户数据
 */
typedef void (*csg_audio_decode_callback)(void *pcm_data, int samples,
                                      int channels, int rate,
                                      int64_t pts, void *user_data);

/* 音频解码上下文 */
typedef struct csg_audio_decode_ctx csg_audio_decode_ctx_t;

/* 创建音频解码器
 *   codec_id:     编码 ID (CSG_AUDIO_CODEC_AAC, 见 common.h)
 *   extradata:    编码器额外数据 (AAC 的 AudioSpecificConfig / ASC)
 *   extradata_size: 额外数据大小
 *   sample_rate:  原始采样率 (仅作 init 回退值)
 *   channels:     原始声道数
 *   channel_layout: 未使用 (FAAD2 无此概念)
 *   callback:     PCM 帧回调
 *   user_data:    用户数据
 *   return:       句柄, NULL 表示失败 */
csg_audio_decode_ctx_t *csg_audio_decode_create(int codec_id,
                                      const uint8_t *extradata,
                                      int extradata_size,
                                      int sample_rate,
                                      int channels,
                                      uint64_t channel_layout,
                                      csg_audio_decode_callback callback,
                                      void *user_data);

/* 销毁音频解码器 */
void csg_audio_decode_destroy(csg_audio_decode_ctx_t *ctx);

/* 送入编码数据包解码
 *   data:       编码数据 (一个完整 AAC access unit)
 *   len:        数据长度
 *   pts:        该包的 PTS (微秒)
 *   return:     0=成功, <0=失败 */
int csg_audio_decode_send_packet(csg_audio_decode_ctx_t *ctx, const uint8_t *data,
                             int len, int64_t pts);

/* 冲刷解码器 (FAAD2 为按帧解码, 无内部缓冲, 属接口兼容保留) */
int csg_audio_decode_flush(csg_audio_decode_ctx_t *ctx);

/* 重置解码器 (Seek 后调用) */
void csg_audio_decode_reset(csg_audio_decode_ctx_t *ctx);

/* 获取实际输出采样率 (AAC 原生采样率) */
int csg_audio_decode_get_target_rate(csg_audio_decode_ctx_t *ctx);

/* 获取实际输出声道数 (AAC 原生声道数) */
int csg_audio_decode_get_target_channels(csg_audio_decode_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* __AUDIO_DECODE_H__ */