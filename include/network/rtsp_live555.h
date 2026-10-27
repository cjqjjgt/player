/* SPDX-License-Identifier: MIT */
/*
 * rtsp_live555.h - 基于 live555 的 RTSP 拉流模块
 *
 * 用 live555 替代 ffmpeg 的 RTSP demuxer (参考项目已验证可用).
 * live555 事件循环在独立线程运行, 解封装后的 H264/H265 Annex-B 帧
 * 通过回调送出. 帧内包含 VCL/SPS/PPS(首帧注入) 完整访问单元,
 * 可直接喂 MPP 解码, 也可直接喂 MP4v2 录制封装.
 */

#ifndef RTSP_LIVE555_H
#define RTSP_LIVE555_H

#include "common.h"   /* CSG_CODEC_H264 / CSG_CODEC_H265 */

#ifdef __cplusplus
extern "C" {
#endif

/* 帧数据回调 (H264/H265 Annex-B 编码流, 与 local_playback 回调同签名) */
typedef void (*csg_rtsp_frame_callback)(void *data, int len, int is_keyframe,
                                        uint64_t pts, int codec_type,
                                        void *user_data);

/* 音频原始字节回调 (PCMU/PCMA G.711 不解码, AAC raw access unit 不解码, 直接透传)
 *   data:        G711 原始字节 / AAC raw access unit (无 ADTS 头)
 *   len:         字节数
 *   codec_id:    CSG_AUDIO_CODEC_G711A / CSG_AUDIO_CODEC_G711U / CSG_AUDIO_CODEC_AAC
 *   sample_rate: 采样率 (Hz, 如 8000)
 *   channels:    声道数
 *   pts:         时间戳 (微秒, 相对首帧)
 *   user_data:   用户数据 */
typedef void (*csg_rtsp_audio_callback)(const uint8_t *data, int len,
                                        int codec_id, int sample_rate,
                                        int channels, uint64_t pts,
                                        void *user_data);

/* 创建 RTSP 拉流句柄. url 可含认证: rtsp://user:pass@host/... */
void *csg_rtsp_live555_create(const char *url, int codec_type_hint);

/* 设置帧回调 */
void csg_rtsp_live555_set_callback(void *handle, csg_rtsp_frame_callback cb,
                                   void *user_data);

/* 设置音频 PCM 回调 (未设置则不拉音频) */
void csg_rtsp_live555_set_audio_callback(void *handle, csg_rtsp_audio_callback cb,
                                         void *user_data);

/* 开始拉流 (内部起事件循环线程) */
int csg_rtsp_live555_start(void *handle);

/* 停止拉流并回收句柄 */
void csg_rtsp_live555_stop(void *handle);

/* 获取源流原始参数 (供录制使用, 返回内部数据指针, 调用方不允许 free) */
int csg_rtsp_live555_get_raw_info(void *handle, int *codec, int *width, int *height,
                                  const uint8_t **sps, int *sps_len,
                                  const uint8_t **pps, int *pps_len,
                                  const uint8_t **vps, int *vps_len);

/* 获取音频 ASC (AudioSpecificConfig, AAC 解码器初始化必需).
 * 仅在 DESCRIBE 完成后有效, 无 AAC 音频轨时 asc_size 返回 0.
 * 返回内部数据指针, 调用方不允许 free, 仅在 handle 存活期间有效. */
int csg_rtsp_live555_get_audio_asc(void *handle, const uint8_t **asc, int *asc_size);

/* 获取音频源流信息 (SDP 声明), 供录制在首帧到达前同步建轨.
 *   codec:      CSG_AUDIO_CODEC_* (0=CSG_AUDIO_CODEC_NONE)
 *   sample_rate/channels: SDP 时钟率/声道 (无音频时为 0)
 *   sdp_done:   1 = 所有 subsession 已完成 SETUP(已发 PLAY). 此时若 codec==NONE,
 *               可判定该流确实无音频 (不必再等首音频帧). */
int csg_rtsp_live555_get_audio_info(void *handle, int *codec, int *sample_rate,
                                    int *channels, int *sdp_done);

#ifdef __cplusplus
}
#endif

#endif /* RTSP_LIVE555_H */