/* SPDX-License-Identifier: MIT */
/*
 * common.h - 公共类型定义和宏
 *
 * 飞凌 OK3572 LVGL 视频播放器
 */

#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 日志宏 ==================== */
#ifndef LOG_TAG
#define LOG_TAG      "RK3572_LVGL"
#endif

#define CSG_LOG_ERROR    0
#define CSG_LOG_WARN     1
#define CSG_LOG_INFO     2
#define CSG_LOG_DEBUG    3

#ifndef LOG_LEVEL
#define LOG_LEVEL    CSG_LOG_DEBUG
#endif

#define CSG_LOG(level, tag, fmt, ...) \
    do { \
        if (level <= LOG_LEVEL) { \
            const char *lvl_str[] = {"E", "W", "I", "D"}; \
            printf("[%s][%s] " fmt "\n", lvl_str[level], tag, ##__VA_ARGS__); \
        } \
    } while (0)

#define CSG_LOGE(tag, fmt, ...) CSG_LOG(CSG_LOG_ERROR, tag, fmt, ##__VA_ARGS__)
#define CSG_LOGW(tag, fmt, ...) CSG_LOG(CSG_LOG_WARN,  tag, fmt, ##__VA_ARGS__)
#define CSG_LOGI(tag, fmt, ...) CSG_LOG(CSG_LOG_INFO,  tag, fmt, ##__VA_ARGS__)
#define CSG_LOGD(tag, fmt, ...) CSG_LOG(CSG_LOG_DEBUG, tag, fmt, ##__VA_ARGS__)

/* ==================== 视频相关 ==================== */
#define CSG_MAX_FRAME_WIDTH      4096
#define CSG_MAX_FRAME_HEIGHT     2160

/* 编解码类型 */
typedef enum {
    CSG_CODEC_NONE  = 0,
    CSG_CODEC_H264  = 1,
    CSG_CODEC_H265  = 2,
} _csg_player_codec_e;

/* 音频编码类型 (自用常量)
 * ★ 与 libmov mov-format.h 的 MOV_OBJECT_* 对应, 录制时直接映射.
 * 新增类型必须同步更新 mp4_record_config_audio / record_init_mp4 的允许列表. */
#define CSG_AUDIO_CODEC_NONE   0
#define CSG_AUDIO_CODEC_AAC    1   /* AAC (MPEG4-GENERIC / mpeg4-generic) */
#define CSG_AUDIO_CODEC_PCM    2   /* MP4 PCM16 音频轨 (sowt/twos, 本项目录制格式) */
#define CSG_AUDIO_CODEC_G711A  3   /* G.711 A-law  (libmov 封装为 alaw) */
#define CSG_AUDIO_CODEC_G711U  4   /* G.711 mu-law (libmov 封装为 ulaw) */
#define CSG_AUDIO_CODEC_MP3    5   /* MPEG Audio Layer 3 (libmov MOV_OBJECT_MP3) */
#define CSG_AUDIO_CODEC_MP1A   6   /* MPEG-1 Audio Layer 1/2 (libmov MOV_OBJECT_MP1A) */
#define CSG_AUDIO_CODEC_AC3    7   /* Dolby Digital AC-3 (libmov MOV_OBJECT_AC3) */
#define CSG_AUDIO_CODEC_OPUS   8   /* Opus (libmov MOV_OBJECT_OPUS) */
#define CSG_AUDIO_CODEC_LPCM   9   /* Linear PCM 16-bit (L16, libmov 无原生对象, 转为 G711u 兼容录制) */

typedef _csg_player_codec_e csg_player_codec_t;

/* 无效时间戳 */
#define CSG_PLAYER_TICK_INVALID  ((int64_t)INT64_MIN)

/* 播放状态 */
typedef enum {
    CSG_PLAYER_STATE_IDLE      = 0,
    CSG_PLAYER_STATE_SCANNING  = 1,
    CSG_PLAYER_STATE_CONNECTED = 2,
    CSG_PLAYER_STATE_PLAYING   = 3,
    CSG_PLAYER_STATE_PAUSED    = 4,
    CSG_PLAYER_STATE_STOPPED   = 5,
    CSG_PLAYER_STATE_COMPLETED = 6,
} _csg_player_state_e;

typedef _csg_player_state_e csg_player_state_t;

/* 视频帧信息 */
typedef struct {
    void     *data;
    int       len;
    int       width;
    int       height;
    int       format;
    uint64_t  pts;
    int       is_keyframe;
} _csg_player_video_frame_s;

typedef _csg_player_video_frame_s csg_player_video_frame_t;

/* 播放器统计 */
typedef struct {
    uint32_t  recv_packets;
    uint32_t  recv_bytes;
    uint32_t  dec_frames;
    uint32_t  disp_frames;
    uint32_t  drop_frames;
    float     fps;
    uint32_t  bitrate;
    uint64_t  start_time;
} _csg_player_stats_s;

typedef _csg_player_stats_s csg_player_stats_t;

/* 播放器时间状态 */
typedef struct {
    int64_t   ts;
    int64_t   system_date;
    int64_t   length;
    double    position;
    float     rate;
    float     speed;
    int       live;
    int       paused;
    int64_t   pause_date;
    int       seeking;
    double    seek_pos;
    int64_t   seek_ts;
} _csg_player_time_s;

typedef _csg_player_time_s csg_player_time_t;

#ifdef __cplusplus
}
#endif

#endif /* __COMMON_H__ */