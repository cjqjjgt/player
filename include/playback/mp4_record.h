/* SPDX-License-Identifier: MIT */
/*
 * mp4_record.h - 基于 libmov fmp4_writer 的录制模块 (消息队列 + 录制线程)
 *
 * 直接 remux: 拉流回调收到的 Annex-B H264/H265 编码帧 + G.711 原始字节
 * 由录制线程转换为 length-prefix 格式后调用 libmov 写入 fMP4。
 * 全程不经过 FFmpeg, 不经过软编码。
 *
 * 参考: /home/dell/rk3572_nvr/src/preview/record_fmp4_writer.cpp
 *       /home/dell/rk3572_nvr/third_party/ireader_media_server/libmov
 *
 * 架构:
 *   拉流回调 → mp4_record_push_frame / mp4_record_push_audio → 环形消息队列
 *                                                           → record_thread → libmov 写文件
 */

#ifndef __MP4_RECORD_H__
#define __MP4_RECORD_H__

#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 消息队列最大帧数 (内存上限 = MAX * 单帧最大尺寸, 防止 OOM) */
#define MP4_RECORD_QUEUE_MAX  40

/* 录制状态 */
typedef enum {
    MP4_RECORD_STATE_IDLE      = 0,
    MP4_RECORD_STATE_RECORDING = 1,
    MP4_RECORD_STATE_STOPPING  = 2,
    MP4_RECORD_STATE_ERROR     = 3,
} MP4_RECORD_STATE_E;

/* 录制信息 */
typedef struct {
    char          file_path[512];
    int64_t       duration_us;
    int64_t       file_size;
    uint32_t      frame_count;
    uint32_t      keyframe_count;
    uint32_t      audio_frame_count;   /* 音频帧数 (G711 chunk 数) */
    MP4_RECORD_STATE_E state;
} MP4_RECORD_INFO_S;

/*
 * 创建录制句柄
 * output_path:   输出 MP4 文件路径
 * codec_type:    CSG_CODEC_H264 或 CSG_CODEC_H265
 * width/height:  视频分辨率
 * fps:           帧率
 * extradata_annexb: 参数集 (Annex-B 格式: 00 00 00 01 VPS ... 00 00 00 01 SPS ...
 *                  00 00 00 01 PPS), 内部经 libflv 构建 avcC/hvcC 写入 MP4 track.
 *                  参数集不完整时延迟到首 IDR 帧提取 (见 record_thread 兜底).
 * extradata_len: extradata 长度
 */
void *mp4_record_create(const char *output_path, int codec_type,
                        int width, int height, int fps,
                        const uint8_t *extradata_annexb, int extradata_len);

/* 开始录制: 启动录制线程 (writer 在首 IDR + 音频配置就绪后创建) */
int mp4_record_start(void *handle);

/* 推送一帧编码数据到消息队列 (拉流回调线程调用, 线程安全)
 * data:         Annex-B 编码包
 * len:          包长度
 * is_keyframe:  是否关键帧 (IDR)
 * pts_us:       该帧 PTS (微秒)
 */
int mp4_record_push_frame(void *handle, void *data, int len,
                          int is_keyframe, uint64_t pts_us);

/* 停止录制: 通知录制线程退出, 等待完成, 写 mfra 并关闭 */
int mp4_record_stop(void *handle);

/* ==================== 音频录制 (G.711 直写 / AAC raw access unit 直写) ====================
 * ★ 拉流录制必须同时封装音频+视频: RTSP 音频原始字节直接透传,
 *   libmov 封装为 G711a/u 轨 (alaw/ulaw) 或 AAC 轨 (含 ASC esds).
 * audio_codec:   CSG_AUDIO_CODEC_G711A / CSG_AUDIO_CODEC_G711U / CSG_AUDIO_CODEC_AAC
 * sample_rate/channels: 音频源参数
 * 幂等: 首帧音频到达时调用一次即可, 重复调用直接返回。 */
int mp4_record_config_audio(void *handle, int audio_codec,
                            int sample_rate, int channels);

/* 设置 AAC AudioSpecificConfig (ASC, 录制 AAC 轨时必需, 写入 esds box).
 * 须在 mp4_record_config_audio 之前调用 (或紧随其后, 在录制线程创建
 * MP4 writer 之前), 幂等. 仅对 AAC 编码有效; 无 AAC 音轨时无需调用. */
int mp4_record_set_audio_asc(void *handle, const uint8_t *asc, int asc_size);

/* 推送一帧音频原始字节到消息队列 (拉流音频回调线程调用, 线程安全)
 * G.711: data = 原始字节 (A-law/mu-law, 无 ADTS 头), len = 字节数
 * AAC:   data = raw access unit (无 ADTS 头), len = 字节数
 * pts_us: 该帧首样本 PTS (微秒, 与视频共用同一时间轴) */
int mp4_record_push_audio(void *handle, const uint8_t *data, int len,
                          uint64_t pts_us);

/* 获取录制信息 (线程安全) */
int mp4_record_get_info(void *handle, MP4_RECORD_INFO_S *info);

/* 销毁录制句柄 (自动 stop) */
void mp4_record_destroy(void *handle);

#ifdef __cplusplus
}
#endif

#endif /* __MP4_RECORD_H__ */