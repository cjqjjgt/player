/* SPDX-License-Identifier: MIT */
/*
 * audio_render.h - 音频渲染模块 (ALSA PCM 播放)
 *
 * 参考:
 *   VLC audio_output/dec.c  - vlc_aout_stream 设计
 *   XBMC AudioSinkAE.cpp      - ActiveAE 音频引擎
 *   NVR  board_hal_audio.c   - 嵌入式 ALSA PCM 播放
 *
 * 核心设计:
 *   环形缓冲区 (Ring Buffer) + 播放线程
 *   解码线程 (生产者) → 环形缓冲 → ALSA 播放线程 (消费者)
 *   从 ALSA 获取已播放采样数, 用于计算音频时钟
 */

#ifndef __AUDIO_RENDER_H__
#define __AUDIO_RENDER_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 环形缓冲区大小 (毫秒, 默认 200ms) */
#define CSG_AUDIO_RENDER_BUFFER_MS    200

/* 音频渲染上下文 */
typedef struct csg_audio_render_ctx csg_audio_render_ctx_t;

/* 音频渲染回调: 获取音频时钟 (由 av_sync 使用)
 *   pts:       当前音频时钟 (微秒)
 *   user_data: 用户数据
 */
typedef void (*csg_audio_render_clock_cb)(int64_t pts, void *user_data);

/* 创建音频渲染器
 *   sample_rate:   采样率 (Hz, 如 48000)
 *   channels:      声道数 (1 或 2)
 *   bits_per_sample: 每样本位数 (16)
 *   device:        ALSA 设备名 (如 "default" 或 "hw:0,0", NULL=default)
 *   return:        句柄, NULL 表示失败 */
csg_audio_render_ctx_t *csg_audio_render_create(int sample_rate, int channels,
                                      int bits_per_sample,
                                      const char *device);

/* 销毁音频渲染器 */
void csg_audio_render_destroy(csg_audio_render_ctx_t *ctx);

/* 启动播放线程 */
int csg_audio_render_start(csg_audio_render_ctx_t *ctx);

/* 停止播放线程 */
void csg_audio_render_stop(csg_audio_render_ctx_t *ctx);

/* 写入 PCM 数据到环形缓冲区 (生产者, 由解码线程调用)
 *   data:     PCM 数据
 *   samples:  采样数 (每通道)
 *   pts:      该帧的 PTS (微秒)
 *   return:   0=成功, 1=缓冲区满丢弃, <0=失败 */
int csg_audio_render_write(csg_audio_render_ctx_t *ctx, const void *data,
                       int samples, int64_t pts);

/* 获取音频时钟 (微秒, 基于 ALSA 已播放采样数)
 *   返回当前正在播放的音频位置的 PTS */
int64_t csg_audio_render_get_clock(csg_audio_render_ctx_t *ctx);

/* 获取环形缓冲区中等待播放的延迟 (微秒) */
int64_t csg_audio_render_get_delay(csg_audio_render_ctx_t *ctx);

/* 暂停: 停止 ALSA 播放, 清空环形缓冲 */
void csg_audio_render_pause(csg_audio_render_ctx_t *ctx);

/* 恢复: 重新启动 ALSA 播放 */
void csg_audio_render_resume(csg_audio_render_ctx_t *ctx);

/* 冲刷: 清空环形缓冲区和 ALSA 缓冲 (Seek 后调用).
 * new_base_pts=0 时等待首帧自动初始化时钟;
 * new_base_pts>0 时直接设为目标 PTS. */
void csg_audio_render_flush(csg_audio_render_ctx_t *ctx, int64_t new_base_pts);

/* 调整时钟基准: 不破坏环形缓冲, 仅修改 base_pts.
 * 用于 Seek 完成后用实际关键帧 PTS 修正时钟, 使音视频同步. */
void csg_audio_render_adjust_clock(csg_audio_render_ctx_t *ctx, int64_t new_base_pts);

/* 设置倍速: 音频时钟推进速度因子 (1.0=正常, 2.0=2倍速) */
void csg_audio_render_set_speed(csg_audio_render_ctx_t *ctx, float speed);

/* 设置音量 (0~100) */
void csg_audio_render_set_volume(csg_audio_render_ctx_t *ctx, int volume);

/* 静音/取消静音 */
void csg_audio_render_set_mute(csg_audio_render_ctx_t *ctx, int mute);

/* 获取音量 */
int csg_audio_render_get_volume(csg_audio_render_ctx_t *ctx);

/* 获取静音状态 */
int csg_audio_render_get_mute(csg_audio_render_ctx_t *ctx);

/* 检查是否正在播放 */
int csg_audio_render_is_playing(csg_audio_render_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* __AUDIO_RENDER_H__ */