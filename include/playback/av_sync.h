/* SPDX-License-Identifier: MIT */
/*
 * av_sync.h - 音视频同步模块
 *
 * 参考 VLC clock.c (vlc_clock_main_t) + XBMC VideoPlayerAudio.cpp (CDVDClock)
 * 核心设计:
 *   1. 音频为主时钟 (audio master clock)
 *   2. 视频帧 PTS 与音频时钟比对, 决定同步策略
 *   3. 支持暂停/Seek/倍速
 */

#ifndef __AV_SYNC_H__
#define __AV_SYNC_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 同步策略 */
typedef enum {
    CSG_AV_SYNC_ACTION_DISPLAY = 0,
    CSG_AV_SYNC_ACTION_DROP    = 1,
    CSG_AV_SYNC_ACTION_REPEAT  = 2,
    CSG_AV_SYNC_ACTION_WAIT    = 3,
} _csg_av_sync_action_e;

typedef _csg_av_sync_action_e csg_av_sync_action_t;

/* 时钟类型 */
typedef enum {
    CSG_AV_CLOCK_MASTER_AUDIO  = 0,
    CSG_AV_CLOCK_MASTER_SYSTEM = 1,
    CSG_AV_CLOCK_MASTER_VIDEO  = 2,
} _csg_av_clock_master_e;

typedef _csg_av_clock_master_e csg_av_clock_master_t;

/* 音视频同步上下文 */
typedef struct csg_av_sync_ctx csg_av_sync_ctx_t;

/* 创建 AV 同步上下文 */
csg_av_sync_ctx_t *csg_av_sync_create(void);

/* 销毁 */
void csg_av_sync_destroy(csg_av_sync_ctx_t *ctx);

/* 设置主时钟类型 (默认音频时钟) */
void csg_av_sync_set_master_clock(csg_av_sync_ctx_t *ctx, csg_av_clock_master_t master);

/* 设置同步阈值 (微秒, 默认 20000us = 20ms)
 * 视频与音频时钟差值在此范围内视为同步, 直接显示 */
void csg_av_sync_set_threshold(csg_av_sync_ctx_t *ctx, int64_t threshold_us);

/* 设置最大连续丢帧数 (默认 5) */
void csg_av_sync_set_max_drop(csg_av_sync_ctx_t *ctx, int max_drop);

/* 更新音频时钟 (由音频渲染线程调用)
 *   pts:      当前写入音频缓冲的帧 PTS (微秒)
 *   samples:  该帧的采样数
 *   rate:     采样率 (Hz)
 *   played:   ALSA 已播放的累计采样数 */
void csg_av_sync_update_audio_clock(csg_av_sync_ctx_t *ctx, int64_t pts,
                                int samples, int rate);

/* 由 audio_render 调用, 更新已播放采样数 */
void csg_av_sync_update_audio_played(csg_av_sync_ctx_t *ctx, int64_t played_samples);

/* 直接设置音频时钟 (微秒, 用于从 audio_render_get_clock 同步) */
void csg_av_sync_set_audio_clock(csg_av_sync_ctx_t *ctx, int64_t clock_us);

/* 更新视频时钟 (由视频解码线程调用, 可选)
 *   pts: 当前显示的视频帧 PTS (微秒) */
void csg_av_sync_update_video_clock(csg_av_sync_ctx_t *ctx, int64_t pts);

/* 获取主时钟时间 (微秒) */
int64_t csg_av_sync_get_master_clock(csg_av_sync_ctx_t *ctx);

/* 获取音频时钟 (微秒) */
int64_t csg_av_sync_get_audio_clock(csg_av_sync_ctx_t *ctx);

/* 视频帧同步判定: 根据当前视频 PTS 与主时钟的差异,
 * 返回同步策略和等待时间.
 *   pts:       视频帧 PTS (微秒)
 *   wait_us:   输出参数, 需要等待的微秒数 (AV_SYNC_WAIT 时有效)
 *   return:    同步策略 */
csg_av_sync_action_t csg_av_sync_video_judge(csg_av_sync_ctx_t *ctx, int64_t pts,
                                     int64_t *wait_us);

/* 暂停: 冻结时钟 */
void csg_av_sync_pause(csg_av_sync_ctx_t *ctx);

/* 恢复: 解冻时钟 */
void csg_av_sync_resume(csg_av_sync_ctx_t *ctx);

/* Seek 后重置时钟 (清除音频时钟历史, 准备接收新 PTS) */
void csg_av_sync_seek_reset(csg_av_sync_ctx_t *ctx);

/* 设置帧时长 (微秒) → 启用 frame_timer 节奏控制
 *   fps: 视频帧率 (如 30.0)
 *   调用后 csg_av_sync_video_judge 内部启用 frame_timer 节奏控制器,
 *   确保视频帧按音频时钟节奏等间隔显示, 防止解码超产导致 drift 累积.
 *   不调用则回退到旧版逐帧独立判断模式. */
void csg_av_sync_set_frame_duration_by_fps(csg_av_sync_ctx_t *ctx, float fps);

/* 重置 frame_timer (Seek 后调用, 清空上一帧的节奏记忆) */
void csg_av_sync_reset_frame_timer(csg_av_sync_ctx_t *ctx);

/* 设置播放速度 (0.5/0.75/1.0/1.5/2.0) */
void csg_av_sync_set_speed(csg_av_sync_ctx_t *ctx, float speed);

/* 获取播放速度 */
float csg_av_sync_get_speed(csg_av_sync_ctx_t *ctx);

/* 停止: 重置所有状态 */
void csg_av_sync_stop(csg_av_sync_ctx_t *ctx);

/* 检查是否有音频时钟 (首次音频帧到达后为 1) */
int csg_av_sync_has_audio_clock(csg_av_sync_ctx_t *ctx);

/* 检查是否有视频时钟 */
int csg_av_sync_has_video_clock(csg_av_sync_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* __AV_SYNC_H__ */