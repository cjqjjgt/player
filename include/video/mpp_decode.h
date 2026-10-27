/* SPDX-License-Identifier: MIT */
/*
 * mpp_decode.h - MPP 硬件解码模块
 *
 * 基于 RK MPP 实现 H.264/H.265 硬件解码
 * 参考: /home/dell/mpp-develop/test/mpi_dec_test.c
 */

#ifndef __MPP_DECODE_H__
#define __MPP_DECODE_H__

#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 解码帧回调 - 输出 NV12 YUV 数据 */
typedef void (*csg_mpp_decode_callback)(void *yuv_data, int yuv_len,
                                    int width, int height, int hor_stride,
                                    int ver_stride, uint64_t pts,
                                    int is_keyframe, void *user_data);

/* 创建解码器 */
void *csg_mpp_decode_create(csg_player_codec_t codec_type, int width, int height);

/* 设置帧回调 */
void csg_mpp_decode_set_callback(void *handle, csg_mpp_decode_callback cb, void *user_data);

/* 发送一帧数据到解码器 */
int csg_mpp_decode_send_frame(void *handle, void *data, int len,
                          int is_keyframe, uint64_t pts);

/* 发送参数集 (VPS/SPS/PPS) 额外数据到解码器.
 * Reset 清空 MPP 解码器状态后, 必须先注入参数集再送入首个关键帧,
 * 否则关键帧因缺少 SPS/PPS 而无法解析. data 为 hvcC (MP4) 或 Annex-B
 * 格式均可, MPP 内部 h265d_extradata 能自动识别. */
int csg_mpp_decode_set_extra_data(void *handle, const void *data, int len);

/* 获取解码器宽高 */
int csg_mpp_decode_get_width(void *handle);
int csg_mpp_decode_get_height(void *handle);

/* 清空解码器内部缓冲 (seek 后重新同步用) */
int csg_mpp_decode_reset(void *handle);

/* 暂停时主动排空管线残留帧 (保留 DPB 参考帧链, 恢复后零跳帧) */
int csg_mpp_decode_drain(void *handle);

/* 停止解码器, flush 待处理帧 (destroy 前调用) */
int csg_mpp_decode_stop(void *handle);

/* 销毁解码器 */
void csg_mpp_decode_destroy(void *handle);

#ifdef __cplusplus
}
#endif

#endif /* __MPP_DECODE_H__ */