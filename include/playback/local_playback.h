/* SPDX-License-Identifier: MIT */
/*
 * local_playback.h - 本地视频播放模块
 *
 * 使用 mp4v2 解封装本地 MP4 视频文件，复用MPP解码管线
 * v2.0: 异步停止/清理模型 + 4K 分辨率检测 + 暂停/恢复 PTS 同步
 */

#ifndef __LOCAL_PLAYBACK_H__
#define __LOCAL_PLAYBACK_H__

#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CSG_LOCAL_EVENT_EOF       = 0,
    CSG_LOCAL_EVENT_SEEK_DONE = 1,
    CSG_LOCAL_EVENT_ERROR     = 2,
} _csg_local_event_e;

typedef _csg_local_event_e csg_local_event_t;

typedef void (*csg_local_frame_callback)(void *data, int len, int is_keyframe,
                                     uint64_t pts, int codec_type,
                                     void *user_data);

/* 音频帧回调: 输出编码音频数据包
 *   data:       编码音频数据
 *   len:        数据长度
 *   pts:        该帧的 PTS (微秒)
 *   codec_id:  音频编码 ID (CSG_AUDIO_CODEC_AAC)
 *   sample_rate: 原始采样率
 *   channels:   原始声道数
 *   channel_layout: 原始声道布局
 *   extradata:  编码器额外数据 (如 AAC AudioSpecificConfig)
 *   extradata_size: 额外数据大小
 *   user_data:  用户数据
 */
typedef void (*csg_local_audio_frame_callback)(const uint8_t *data, int len,
                                           int64_t pts, int codec_id,
                                           int sample_rate, int channels,
                                           uint64_t channel_layout,
                                           const uint8_t *extradata,
                                           int extradata_size,
                                           void *user_data);

typedef void (*csg_local_event_callback)(csg_local_event_t event, void *user_data);

typedef struct {
    char     path[512];
    char     name[256];
    int64_t  duration_us;
    int64_t  file_size;
    int      width;
    int      height;
} _csg_local_file_info_s;

typedef _csg_local_file_info_s csg_local_file_info_t;

int csg_local_playback_is_supported(const char *file_path);

int csg_local_playback_scan_dir(const char *dir_path,
                            csg_local_file_info_t *files, int max_count);

/* 递归扫描目录 (含子目录), 返回找到的视频文件数 */
int csg_local_playback_scan_dir_recursive(const char *dir_path,
                            csg_local_file_info_t *files, int max_count);

int csg_local_playback_create(const char *file_path);

void csg_local_playback_set_callback(int handle, csg_local_frame_callback cb,
                                 void *user_data);

void csg_local_playback_set_event_callback(int handle, csg_local_event_callback cb,
                                       void *user_data);

/* 设置音频帧回调 (解码线程调用, 每次读到音频包时触发) */
void csg_local_playback_set_audio_callback(int handle, csg_local_audio_frame_callback cb,
                                       void *user_data);

/* 获取音频流信息 (在 create 后调用)
 *   codec_id:     音频编码 ID
 *   sample_rate:   采样率
 *   channels:      声道数
 *   channel_layout: 声道布局
 *   extradata:     编码器额外数据 (指针属于上下文, 只读)
 *   extradata_size: 额外数据大小
 *   return: 0=有音频流, -1=无音频流 */
int csg_local_playback_get_audio_info(int handle, int *codec_id,
                                  int *sample_rate, int *channels,
                                  uint64_t *channel_layout,
                                  const uint8_t **extradata,
                                  int *extradata_size);

int csg_local_playback_start(int handle);

int csg_local_playback_pause(int handle);

/* 恢复播放: target_pts 为暂停时的 PTS (微秒),
 * 用于精确 seek 回暂停位置继续; 传 0 则退化为 seek 到最近关键帧 */
int csg_local_playback_resume(int handle, int64_t target_pts);

/* 非阻塞停止: 仅设 running=0 并 detach 线程, 不阻塞 UI 线程 */
int csg_local_playback_stop(int handle);

/* 同步清理 (blocking): 等待线程退出并释放资源. 用于确认线程已完成的场景 */
int csg_local_playback_cleanup_sync(int handle);

int csg_local_playback_get_video_info(int handle, int *width, int *height,
                                  int *fps, int *codec_type);

int64_t csg_local_playback_get_duration(int handle);

/* 获取最后一次 Seek 的实际关键帧 PTS (微秒).
 * 用于修正音频时钟: 用户拖动位置与最近关键帧天然存在偏差,
 * 音视频应以关键帧 PTS 为共同起点, 否则视频永远落后. */
int64_t csg_local_playback_get_last_seek_kf_pts(int handle);

int64_t csg_local_playback_get_current_position(int handle);

/* 获取视频流的参数集 extra data (MP4 hvcC / Annex-B).
 * seek 后 MPP reset 需先注入参数集, 该接口从视频流 codecpar 取回.
 * 返回 0 成功, data/size 有效 (指针属于本地播放上下文, 调用方只读). */
int csg_local_playback_get_extradata(int handle, const uint8_t **data, int *size);

int csg_local_playback_seek(int handle, double position);

void csg_local_playback_get_stats(int handle, csg_player_stats_t *stats);

int csg_local_playback_force_stop_thread(int handle);

int csg_local_playback_is_active(int handle);

int csg_local_playback_get_downscale(int handle);

/* 设置倍速因子 (0.5/0.75/1.0/1.5/2.0) */
int csg_local_playback_set_speed(int handle, float speed);

/* 注册 Seek 快速喂帧停止标志指针: 指向 APP_CTX.seek_fast_feed_done.
 * on_mpp_frame 显示第一帧后置 1, 播放线程检测后停止快速喂帧. */
void csg_local_playback_set_seek_feed_done_ptr(int handle, volatile int *ptr);

#ifdef __cplusplus
}
#endif

#endif /* __LOCAL_PLAYBACK_H__ */