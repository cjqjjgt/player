/* SPDX-License-Identifier: MIT */
/*
 * mpp_decode.c - MPP 硬件解码实现
 *
 * 基于 RK MPP (librockchip_mpp) 实现 H.264/H.265 硬件解码
 * 输出 NV12 YUV 数据
 * 参考: /home/dell/mpp-develop/test/mpi_dec_test.c
 */

#include "mpp_decode.h"
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>

#include "rk_mpi.h"
#include "mpp_frame.h"
#include "mpp_packet.h"
#include "mpp_buffer.h"
#include "mpp_log.h"

/* MPP_PACKET_FLAG_EXTRA_DATA 定义在内部头 mpp_packet_impl.h (公开头
 * mpp_packet.h 未导出), 这里使用其稳定 ABI 值, 用于注入参数集. */
#ifndef MPP_PACKET_FLAG_EXTRA_DATA
#define MPP_PACKET_FLAG_EXTRA_DATA (0x00000002)
#endif

typedef struct {
    MppCtx          ctx;
    MppApi         *mpi;
    MppPacket       packet;
    MppBufferGroup  frm_grp;
    int             info_changed;
    int             width;
    int             height;
    csg_player_codec_t    codec_type;
    uint32_t        frame_count;
    uint32_t        keyframes;
    uint32_t        put_failures;
    uint32_t        drop_frames;   /* errinfo/discard 丢弃帧数 (seek 重建 DPB 期间应只有少量) */
    int             pending_is_keyframe; /* 当前送入的帧是否为关键帧 */

    /* 关键帧跟踪环形队列: 解决 pending_is_keyframe 被后续 send_frame 覆盖后,
     * 输出帧的 is_keyframe 错位问题 (P 帧被误报为 I 帧, 导致高倍速花屏). */
#define MPP_KF_RING_SIZE 64
    struct {
        int is_keyframe;
        int64_t pts;
    } kf_ring[MPP_KF_RING_SIZE];
    int kf_ring_head;   /* 写入位置 */
    int kf_ring_tail;   /* 读取位置 */
    int kf_ring_count;  /* 队列中元素数量 */

    csg_mpp_decode_callback callback;
    void           *user_data;

    /* MPP mpi 互斥握手: mpi 接口非线程安全, 并发 reset vs put_packet/get_frame
     * 是未定义行为. send_frame 在播放线程执行, reset 在 UI 线程执行.
     * 锁只在函数入口/出口短暂持有, 不跨越回调/MPP 调用, 无死锁风险. */
    pthread_mutex_t lock;
    pthread_cond_t  cond;
    int             send_active;   /* 1=播放线程在 send_frame 内 */
    int             reset_active;  /* 1=UI 线程在 reset 内 (阻止新 send_frame) */
} csg_mpp_decode_ctx_t;

static void csg_mpp_decode_sleep_ms(int ms)
{
    usleep(ms * 1000);
}

/* 进入/退出 MPP 互斥临界区.
 * as_reset=1: reset 语义 — 阻止新 send_frame 进入, 并等待在途 send_frame 退出.
 * as_reset=0: send_frame 语义 — 等待 reset 完成, 然后标记自己在临界区内.
 * 锁只在入口/出口持有, 临界区内的 mpi 调用和回调都不持锁. */
static void csg_mpp_decode_lock_enter(csg_mpp_decode_ctx_t *ctx, int as_reset)
{
    pthread_mutex_lock(&ctx->lock);
    if (as_reset) {
        ctx->reset_active = 1;
        while (ctx->send_active)
            pthread_cond_wait(&ctx->cond, &ctx->lock);
    } else {
        while (ctx->reset_active)
            pthread_cond_wait(&ctx->cond, &ctx->lock);
        ctx->send_active = 1;
    }
    pthread_mutex_unlock(&ctx->lock);
}

static void csg_mpp_decode_lock_exit(csg_mpp_decode_ctx_t *ctx, int as_reset)
{
    pthread_mutex_lock(&ctx->lock);
    if (as_reset)
        ctx->reset_active = 0;
    else
        ctx->send_active = 0;
    pthread_cond_broadcast(&ctx->cond);
    pthread_mutex_unlock(&ctx->lock);
}

void *csg_mpp_decode_create(csg_player_codec_t codec_type, int width, int height)
{
    csg_mpp_decode_ctx_t *ctx = calloc(1, sizeof(csg_mpp_decode_ctx_t));
    if (!ctx) return NULL;

    MPP_RET ret;
    MppCodingType type;

    if (codec_type == CSG_CODEC_H265) {
        type = MPP_VIDEO_CodingHEVC;
    } else {
        type = MPP_VIDEO_CodingAVC;
    }

    ctx->codec_type = codec_type;
    ctx->width = width;
    ctx->height = height;
    ctx->info_changed = 0;

    pthread_mutex_init(&ctx->lock, NULL);
    pthread_cond_init(&ctx->cond, NULL);

    /* 创建 MPP 上下文 */
    ret = mpp_create(&ctx->ctx, &ctx->mpi);
    if (ret != MPP_OK) {
        CSG_LOGE("MPP_DEC", "mpp_create failed: %d", ret);
        free(ctx);
        return NULL;
    }

    /* 初始化解码器 */
    ret = mpp_init(ctx->ctx, MPP_CTX_DEC, type);
    if (ret != MPP_OK) {
        CSG_LOGE("MPP_DEC", "mpp_init failed: %d", ret);
        mpp_destroy(ctx->ctx);
        free(ctx);
        return NULL;
    }

    /* 配置解码器:
     * 1. MPP_DEC_SET_ENABLE_FAST_PLAY: 让 IDR/CRA 关键帧立即输出 (跳过重排),
     *    解决 Seek 后第一帧延迟; 之后帧恢复正常显示顺序重排输出
     * 注意: MPP_DEC_SET_IMMEDIATE_OUT (fast_out) 故意不设置 — 它按解码顺序输出,
     *    对 B 帧流是错误顺序 (画面先跳到 I 帧又退回 B 帧, 即"进度条前跳后退");
     *    MPP_DEC_SET_DISABLE_DPB_CHECK 对 H.265 是 no-op (仅 H.264 生效), 无效.
     *
     * 注意: 不再设置 MPP_DEC_SET_DISABLE_ERROR!
     * 实测 DISABLE_ERROR=1 后, seek 重建 DPB 期间 miss_ref 帧不再被打 errinfo,
     * 直接带着空占位引用槽位提交 rkvdec2 硬件 → 每个 seek 都出现
     * "mpp_rkvdec2 task timeout → resetting → irq_status 0xf0000004" + 花屏.
     * 回退默认 errinfo 标记: 缺失参考的帧由应用层 (get_frame 的 errinfo/discard
     * 丢弃) 过滤, 不进硬件, 消除超时花屏.
     * 级联问题 (单个 miss_ref → 之后全 GOP 被丢弃 → 2s 跳) 由 MPP 源码
     * h265d_flow.c 去掉 has_err 级联解决, 见 csg_mpp_decode_reset. */
    RK_S32 fast_play = 1;
    ret = ctx->mpi->control(ctx->ctx, MPP_DEC_SET_ENABLE_FAST_PLAY, &fast_play);
    if (ret == MPP_OK) {
        CSG_LOGI("MPP_DEC", "Set ENABLE_FAST_PLAY enabled (IDR output immediately)");
    } else {
        CSG_LOGW("MPP_DEC", "Set ENABLE_FAST_PLAY failed: %d", ret);
    }

    /* 创建 packet */
    ret = mpp_packet_init(&ctx->packet, NULL, 0);
    if (ret != MPP_OK) {
        CSG_LOGE("MPP_DEC", "mpp_packet_init failed: %d", ret);
        mpp_destroy(ctx->ctx);
        free(ctx);
        return NULL;
    }

    CSG_LOGI("MPP_DEC", "Created: codec=%s, %dx%d",
         codec_type == CSG_CODEC_H265 ? "H265" : "H264", width, height);
    return ctx;
}

void csg_mpp_decode_set_callback(void *handle, csg_mpp_decode_callback cb, void *user_data)
{
    if (!handle) return;
    csg_mpp_decode_ctx_t *ctx = (csg_mpp_decode_ctx_t *)handle;
    ctx->callback = cb;
    ctx->user_data = user_data;
}

int csg_mpp_decode_send_frame(void *handle, void *data, int len,
                          int is_keyframe, uint64_t pts)
{
    if (!handle || !data || len <= 0) return -1;

    csg_mpp_decode_ctx_t *ctx = (csg_mpp_decode_ctx_t *)handle;
    MPP_RET ret;
    int pkt_done = 0;

    /* ★ DIAG: 定位缓冲层断点 — MPP 是否收到帧 */
    if (ctx->frame_count % 30 == 0) {
        CSG_LOGI("MPP_DEC", "DIAG send_frame: len=%d kf=%d pts=%llu",
             len, is_keyframe, (unsigned long long)pts);
    }

    if (is_keyframe) {
        ctx->keyframes++;
        if (ctx->keyframes <= 3) {
            CSG_LOGI("MPP_DEC", "keyframe #%u len=%d pts=%llu",
                 ctx->keyframes, len, (unsigned long long)pts);
        }
    }
    ctx->pending_is_keyframe = is_keyframe;

    /* 推入关键帧跟踪环形队列: 解决 pending_is_keyframe 被后续 send_frame
     * 覆盖后, 输出帧的 is_keyframe 错位问题 (P 帧被误报为 I 帧, 导致高倍速花屏). */
    ctx->kf_ring[ctx->kf_ring_head].is_keyframe = is_keyframe;
    ctx->kf_ring[ctx->kf_ring_head].pts = (int64_t)pts;
    ctx->kf_ring_head = (ctx->kf_ring_head + 1) % MPP_KF_RING_SIZE;
    if (ctx->kf_ring_count == MPP_KF_RING_SIZE) {
        /* 队列满: 覆盖最旧条目 (极端情况, 正常不应发生) */
        ctx->kf_ring_tail = (ctx->kf_ring_tail + 1) % MPP_KF_RING_SIZE;
    } else {
        ctx->kf_ring_count++;
    }

    /* 设置 packet 数据 */
    mpp_packet_set_data(ctx->packet, data);
    mpp_packet_set_size(ctx->packet, len);
    mpp_packet_set_pos(ctx->packet, data);
    mpp_packet_set_length(ctx->packet, len);

    /* 设置 PTS (MPP 内部使用 90kHz 时间基, 从微秒转换) */
    mpp_packet_set_pts(ctx->packet, (RK_S64)pts * 9 / 100);

    /* 与 reset 互斥: reset 在 UI 线程执行, 本函数在播放线程执行.
     * 锁只在入口持有, 不跨越下面的 mpi 调用/回调. */
    csg_mpp_decode_lock_enter(ctx, 0);

    /* 发送 packet 到解码器。
     * 循环结构参考官方 mpi_dec_test (mpi_dec_test.c dec_simple):
     *   - decode_put_packet 失败 (输入队列满) 时不原地重试,
     *     而是先消费输出帧 (get_frame), 再回到外层循环重试 put,
     *     输入输出交替推进, 避免 MPP 内部线程被卡死
     *   - decode_get_frame 带 MPP_ERR_TIMEOUT 重试 (有内部超时,
     *     不会永久阻塞; 官方测试即用此模式, 无需 poll) */
    do {
        RK_S32 times = 30;   /* get_frame TIMEOUT 重试 30 次 (30ms) */

        if (!pkt_done) {
            ret = ctx->mpi->decode_put_packet(ctx->ctx, ctx->packet);
            if (ret == MPP_OK) {
                pkt_done = 1;
            } else if (ret != MPP_ERR_BUFFER_FULL && ret != MPP_ERR_DISPLAY_FULL) {
                ctx->put_failures++;
                if (ctx->put_failures <= 3) {
                    CSG_LOGW("MPP_DEC", "put_packet failed: %d", ret);
                }
                break;   /* 非缓冲满错误, 放弃本包 */
            }
            /* BUFFER_FULL: 先消费输出, 外层循环再试 */
        }

        /* 获取所有可用的解码帧 */
        for (;;) {
            MppFrame frame = NULL;

            ret = ctx->mpi->decode_get_frame(ctx->ctx, &frame);
            if (ret == MPP_ERR_TIMEOUT) {
                if (times > 0) {
                    times--;
                    csg_mpp_decode_sleep_ms(1);
                    continue;
                }
                break;   /* 本包尚未解出帧, 留给下一次 send 轮询 */
            }
            if (ret != MPP_OK) {
                if (ret != MPP_ERR_TIMEOUT && ctx->put_failures <= 3) {
                    CSG_LOGW("MPP_DEC", "get_frame failed: %d", ret);
                }
                break;
            }
            if (frame == NULL) {
                /* 关键: MPP 无输出帧时返回 MPP_OK 且 frame=NULL (非 TIMEOUT)!
                 * 必须在此退出, 否则死循环卡死解码线程 (mpp.c mpp_get_frame
                 * 无帧时返回 MPP_OK + NULL, 参考官方 mpi_dec_test 的 break) */
                break;
            }

            if (frame) {
                if (mpp_frame_get_info_change(frame)) {
                    RK_U32 width = mpp_frame_get_width(frame);
                    RK_U32 height = mpp_frame_get_height(frame);
                    RK_U32 hor_stride = mpp_frame_get_hor_stride(frame);
                    RK_U32 ver_stride = mpp_frame_get_ver_stride(frame);
                    RK_U32 buf_size = mpp_frame_get_buf_size(frame);

                    CSG_LOGI("MPP_DEC", "Info change: %dx%d, stride=%dx%d, buf=%d",
                         width, height, hor_stride, ver_stride, buf_size);

                    ctx->width = width;
                    ctx->height = height;

                    /* 创建 buffer group */
                    if (ctx->frm_grp) {
                        mpp_buffer_group_put(ctx->frm_grp);
                    }

                    ret = mpp_buffer_group_get_internal(&ctx->frm_grp,
                                                        MPP_BUFFER_TYPE_ION);
                    if (ret) {
                        ret = mpp_buffer_group_get_internal(&ctx->frm_grp,
                                                            MPP_BUFFER_TYPE_DRM);
                    }
                    if (ret) {
                        CSG_LOGE("MPP_DEC", "Failed to get buffer group");
                        mpp_frame_deinit(&frame);
                        break;
                    }

                    mpp_buffer_group_limit_config(ctx->frm_grp, buf_size, 24);
                    ret = ctx->mpi->control(ctx->ctx, MPP_DEC_SET_EXT_BUF_GROUP,
                                           ctx->frm_grp);
                    if (ret) {
                        CSG_LOGE("MPP_DEC", "set buffer group failed: %d", ret);
                        mpp_frame_deinit(&frame);
                        break;
                    }

                    ctx->info_changed = 1;

                    ret = ctx->mpi->control(ctx->ctx, MPP_DEC_SET_INFO_CHANGE_READY, NULL);
                    if (ret) {
                        CSG_LOGE("MPP_DEC", "info change ready failed: %d", ret);
                    }
                } else {
                    RK_U32 err_info = mpp_frame_get_errinfo(frame);
                    RK_U32 discard = mpp_frame_get_discard(frame);

                    if (err_info || discard) {
                        /* seek 重建 DPB 期间 miss_ref 帧被 MPP 标记 errinfo,
                         * 在此丢弃, 不进 RGA/上屏 (也避免把脏帧当正常帧显示).
                         * 级联已由 h265d_flow.c 去掉 has_err 传播, 丢弃数量
                         * 应只有引用缺失前导帧的那几帧, 不应成百上千. */
                        ctx->drop_frames++;
                        if (ctx->drop_frames <= 5) {
                            CSG_LOGW("MPP_DEC", "drop errinfo=%u discard=%u (drop #%u)",
                                 err_info, discard, ctx->drop_frames);
                        }
                        mpp_frame_deinit(&frame);
                        continue;
                    }

                    if (ctx->info_changed && ctx->callback) {
                        MppBuffer buf = mpp_frame_get_buffer(frame);
                        void *ptr = mpp_buffer_get_ptr(buf);
                        int yuv_len = mpp_frame_get_buf_size(frame);
                        int hor_stride = mpp_frame_get_hor_stride(frame);
                        int ver_stride = mpp_frame_get_ver_stride(frame);

                        /* 使用帧的实际显示 PTS (而非包的 PTS)
                         * 对于 H.265 B 帧编码, 包的 PTS 是解码顺序的,
                         * 而 mpp_frame_get_pts 返回的是显示顺序的 PTS.
                         * MPP PTS 单位通常是 90kHz, 转换为微秒: pts_us = pts_90k * 1000 / 90
                         *
                         * ★ 关键修复: 原来用 frame_pts_90k > 0 判断, 导致 I 帧 PTS=0
                         *   被错误 fallback 到当前发送包的 PTS (如 170000us),
                         *   使首帧 drift 初始就是 147ms, 同步永远无法收敛.
                         *   改为 >= 0: PTS=0 是合法值, 只有负值 (-1) 表示 MPP 未设置 PTS. */
                        int64_t frame_pts_90k = mpp_frame_get_pts(frame);
                        uint64_t frame_pts;
                        if (frame_pts_90k >= 0) {
                            frame_pts = (uint64_t)(frame_pts_90k * 1000 / 90);
                        } else {
                            frame_pts = pts;  /* fallback: MPP 未设置 PTS, 使用包的 PTS */
                        }

                        /* 从环形队列弹出正确的关键帧状态.
                         * 解决 pending_is_keyframe 被后续 send_frame 覆盖后,
                         * 输出帧的 is_keyframe 错位问题 (P 帧被误报为 I 帧). */
                        int out_is_keyframe = ctx->pending_is_keyframe;  /* Fallback */
                        if (ctx->kf_ring_count > 0) {
                            out_is_keyframe = ctx->kf_ring[ctx->kf_ring_tail].is_keyframe;
                            ctx->kf_ring_tail = (ctx->kf_ring_tail + 1) % MPP_KF_RING_SIZE;
                            ctx->kf_ring_count--;
                        }

                        ctx->callback(ptr, yuv_len,
                                     ctx->width, ctx->height,
                                     hor_stride, ver_stride,
                                     frame_pts,
                                     out_is_keyframe,
                                     ctx->user_data);
                        ctx->frame_count++;
                        if (ctx->frame_count == 1 || ctx->frame_count % 5 == 0) {
                            CSG_LOGI("MPP_DEC", "output frame #%u %dx%d pts=%llu (raw=%lld)",
                                 ctx->frame_count, ctx->width, ctx->height,
                                 (unsigned long long)frame_pts, (long long)frame_pts_90k);
                        }
                    }
                }
                mpp_frame_deinit(&frame);
            }
        }

        if (pkt_done)
            break;
        csg_mpp_decode_sleep_ms(1);   /* 给 MPP 内部线程消费输入的时间 (官方注释: 1080p 解码约 2ms) */
    } while (1);

    csg_mpp_decode_lock_exit(ctx, 0);
    return 0;
}

/* 把 MP4 的 avcC 参数集额外数据转换成 Annex-B 起始码格式 (H.264).
 *
 * avcC 格式 (AVCDecoderConfigurationRecord):
 *   1B configurationVersion (=1)
 *   1B AVCProfileIndication
 *   1B profile_compatibility
 *   1B AVCLevelIndication
 *   1B reserved(6b) + lengthSizeMinusOne(2b)
 *   1B reserved(3b) + numOfSequenceParameterSets(5b)
 *   每个 SPS: 2B len + NAL data
 *   1B numOfPictureParameterSets
 *   每个 PPS: 2B len + NAL data
 *
 * 返回 malloc 的新缓冲 (调用方 free), 失败返回 -1. */
static int csg_mpp_decode_avcc_to_annexb(const uint8_t *in, int in_len, uint8_t **out, int *out_len)
{
    static const uint8_t sc[4] = { 0, 0, 0, 1 };
    int pos = 0, num_sps = 0, num_pps = 0;
    int buf_cap = 0, buf_used = 0;
    uint8_t *buf = NULL;
    int i;

    *out = NULL;
    *out_len = 0;

    if (!in || in_len < 7 || in[0] != 1) return -1;

    /* 跳过前 5 字节: configVersion, profile, compatibility, level, lengthSize */
    pos = 5;

    num_sps = in[pos] & 0x1F;
    pos++;

    for (i = 0; i < num_sps; i++) {
        if (pos + 2 > in_len) { free(buf); return -1; }
        int nal_len = (in[pos] << 8) | in[pos + 1];
        pos += 2;
        if (pos + nal_len > in_len) { free(buf); return -1; }

        if (buf_cap < buf_used + 4 + nal_len) {
            int new_cap = buf_used + 4 + nal_len + 128;
            uint8_t *nb = realloc(buf, new_cap);
            if (!nb) { free(buf); return -1; }
            buf = nb;
            buf_cap = new_cap;
        }
        memcpy(buf + buf_used, sc, 4);
        buf_used += 4;
        memcpy(buf + buf_used, in + pos, nal_len);
        buf_used += nal_len;
        pos += nal_len;
    }

    if (pos >= in_len) { free(buf); return -1; }
    num_pps = in[pos];
    pos++;

    for (i = 0; i < num_pps; i++) {
        if (pos + 2 > in_len) { free(buf); return -1; }
        int nal_len = (in[pos] << 8) | in[pos + 1];
        pos += 2;
        if (pos + nal_len > in_len) { free(buf); return -1; }

        if (buf_cap < buf_used + 4 + nal_len) {
            int new_cap = buf_used + 4 + nal_len + 128;
            uint8_t *nb = realloc(buf, new_cap);
            if (!nb) { free(buf); return -1; }
            buf = nb;
            buf_cap = new_cap;
        }
        memcpy(buf + buf_used, sc, 4);
        buf_used += 4;
        memcpy(buf + buf_used, in + pos, nal_len);
        buf_used += nal_len;
        pos += nal_len;
    }

    if (!buf_used) { free(buf); return -1; }
    *out = buf;
    *out_len = buf_used;
    return 0;
}

/* 把 MP4 的 hvcC 参数集额外数据转换成 Annex-B 起始码格式.
 *
 * 注意: 绝对不能直接把 hvcC 注入 MPP! h265d_extradata() 检测到 hvcC 后
 * 会把 parser 永久切成 NALFF (长度前缀) 模式 (is_nalff=1, reset 不清除),
 * 而应用送入解码器的是 Annex-B 码流 (hevc_mp4toannexb 之后), 于是
 * h265d_split_nal 把起始码/帧数据当 4 字节长度解析, 读到的是缓冲地址,
 * 报 "nal: unit size invalid extract_length ..." 并丢包.
 * 转成 Annex-B (is_nalff=0) 后走 h265d_extradata() 的 Annex-B 分支.
 *
 * 偏移与 MPP h265d_extradata 的 Annex-B 布局一致:
 * configVersion=1, numOfArrays 在偏移 22, 之后每个 array:
 *   1B (array_completeness + NAL_unit_type) + 2B numNalus + (2B len + data)*
 * 返回 malloc 的新缓冲 (调用方 free), 失败返回 -1. */
static int csg_mpp_decode_hvcc_to_annexb(const uint8_t *in, int in_len, uint8_t **out, int *out_len)
{
    static const uint8_t sc[4] = { 0, 0, 0, 1 };
    int pos = 0, num_arrays = 0;
    int buf_cap = 0, buf_used = 0;
    uint8_t *buf = NULL;
    int i, j;

    *out = NULL;
    *out_len = 0;

    if (!in || in_len < 23 || in[0] != 1) return -1;   /* configVersion 必须为 1 */

    num_arrays = in[22];
    pos = 23;

    for (i = 0; i < num_arrays; i++) {
        int cnt = 0;
        if (pos + 3 > in_len) { free(buf); return -1; }
        cnt = (in[pos + 1] << 8) | in[pos + 2];   /* in[pos] 是 NAL_unit_type, 跳过 */
        pos += 3;

        for (j = 0; j < cnt; j++) {
            int nal_len = 0;
            if (pos + 2 > in_len) { free(buf); return -1; }
            nal_len = (in[pos] << 8) | in[pos + 1];
            pos += 2;
            if (pos + nal_len > in_len) { free(buf); return -1; }

            if (buf_cap < buf_used + 4 + nal_len) {
                int new_cap = buf_used + 4 + nal_len + 128;
                uint8_t *nb = realloc(buf, new_cap);
                if (!nb) { free(buf); return -1; }
                buf = nb;
                buf_cap = new_cap;
            }
            memcpy(buf + buf_used, sc, 4);
            buf_used += 4;
            memcpy(buf + buf_used, in + pos, nal_len);
            buf_used += nal_len;
            pos += nal_len;
        }
    }

    if (!buf_used) { free(buf); return -1; }
    *out = buf;
    *out_len = buf_used;
    return 0;
}

/* Reset 后注入参数集 (VPS/SPS/PPS).
 * MPP reset 清空解码器状态, 必须在首个关键帧送入前把保存的 extra data
 * (MP4 hvcC) 注入, 否则关键帧缺少 SPS/PPS 无法解析. 注意 hvcC 会切 NALFF,
 * 这里必须转成 Annex-B 注入. 与 send_frame 共享 ctx->packet, 在 lock 下使用,
 * 用完清除 EXTRA_DATA 标志, 避免污染后续 send_frame. */
int csg_mpp_decode_set_extra_data(void *handle, const void *data, int len)
{
    if (!handle || !data || len <= 0) return -1;
    csg_mpp_decode_ctx_t *ctx = (csg_mpp_decode_ctx_t *)handle;

    const uint8_t *edata = (const uint8_t *)data;
    int edata_len = len;
    uint8_t *converted = NULL;
    int converted_len = 0;
    int is_annexb = 0;
    int is_avcc = 0;

    /* 格式检测: 以 00 00 00 / 00 00 01 起始码开头 = 已是 Annex-B */
    if (len >= 3 && edata[0] == 0 && edata[1] == 0 && edata[2] <= 1) {
        is_annexb = 1;
    } else {
        /* 尝试 avcC (H.264) 或 hvcC (H.265) 转换.
         * avcC: byte[4] 高 6bit 为 1 (reserved), byte[5] 高 3bit 为 1 */
        is_avcc = (len >= 6 && (edata[4] & 0xFC) == 0xFC
                   && (edata[5] & 0xE0) == 0xE0);
        if (is_avcc) {
            if (csg_mpp_decode_avcc_to_annexb(edata, len, &converted, &converted_len) != 0) {
                CSG_LOGW("MPP_DEC", "set_extra_data: avcC->Annex-B conversion failed, skip inject");
                return -1;
            }
        } else {
            if (csg_mpp_decode_hvcc_to_annexb(edata, len, &converted, &converted_len) != 0) {
                CSG_LOGW("MPP_DEC", "set_extra_data: hvcC->Annex-B conversion failed, skip inject");
                return -1;
            }
        }
        edata = converted;
        edata_len = converted_len;
    }

    csg_mpp_decode_lock_enter(ctx, 0);

    /* data 来自 AVStream codecpar 只读 extradata (或上面转换出的临时缓冲),
     * MPP put_packet 会深拷贝 (mpp_packet_copy_init), 指针只需在本调用期间有效 */
    mpp_packet_set_data(ctx->packet, (void *)edata);
    mpp_packet_set_size(ctx->packet, edata_len);
    mpp_packet_set_pos(ctx->packet, (void *)edata);
    mpp_packet_set_length(ctx->packet, edata_len);
    mpp_packet_set_flag(ctx->packet, MPP_PACKET_FLAG_EXTRA_DATA);

    MPP_RET ret = ctx->mpi->decode_put_packet(ctx->ctx, ctx->packet);
    if (ret != MPP_OK) {
        CSG_LOGW("MPP_DEC", "put extra_data failed: %d", ret);
    } else {
        CSG_LOGI("MPP_DEC", "extra_data injected (%s, %d bytes)",
             is_annexb ? "Annex-B" : (is_avcc ? "avcC->Annex-B" : "hvcC->Annex-B"), edata_len);
    }

    /* 清除标志: ctx->packet 会被 send_frame 复用 */
    mpp_packet_set_flag(ctx->packet, 0);

    csg_mpp_decode_lock_exit(ctx, 0);

    if (converted) free(converted);
    return 0;
}

/* 清空解码器内部缓冲 (参考帧/DPB), 用于 seek 后重新同步。
 * 与销毁重建相比开销小得多, 且保留 buffer group。
 * 关键: reset 后必须 drain DPB 中所有残留帧, 否则恢复时会吐旧帧跳帧 */
int csg_mpp_decode_reset(void *handle)
{
    if (!handle) return -1;
    csg_mpp_decode_ctx_t *ctx = (csg_mpp_decode_ctx_t *)handle;

    /* 与 send_frame 互斥: 阻止新 send_frame 进入, 等待在途 send_frame 退出.
     * 避免 reset 的 dpb_flush 释放槽位缓冲时, 播放线程还在 get_frame 使用该缓冲. */
    csg_mpp_decode_lock_enter(ctx, 1);

    MPP_RET ret = ctx->mpi->reset(ctx->ctx);
    if (ret != MPP_OK) {
        CSG_LOGE("MPP_DEC", "reset failed: %d", ret);
        csg_mpp_decode_lock_exit(ctx, 1);
        return -1;
    }

    /* reset 后立即设置关键配置:
     * 1. DISABLE_DPB_CHECK: 仅 H.264 有效, H.265 是 no-op; 保留以防切换编码格式
     * 2. ENABLE_FAST_PLAY: 让 IDR 帧立即输出
     * 注意: 不再设置 DISABLE_ERROR (同 create, 见上面的说明).
     * reset 清空 DPB 后, open-GOP 关键帧之后的帧会 miss_ref (引用被跳过的
     * 前导帧), 默认 errinfo 标记让应用层丢弃这些帧, 不进硬件, 消除 rkvdec2
     * timeout + 花屏. 为避免这些 miss_ref 级联丢到下一关键帧 (2s 跳), MPP
     * 源码 h265d_flow.c 已去掉 ref->err_status.has_err 级联传播, 只有真正
     * 缺失参考的帧被丢弃, 之后帧引用 DPB 中已解码帧正常输出. */
    RK_S32 disable_dpb = 1;
    ret = ctx->mpi->control(ctx->ctx, MPP_DEC_SET_DISABLE_DPB_CHECK, &disable_dpb);
    if (ret == MPP_OK) {
        CSG_LOGI("MPP_DEC", "Reset: DISABLE_DPB_CHECK enabled");
    } else {
        CSG_LOGW("MPP_DEC", "Reset: DISABLE_DPB_CHECK failed: %d", ret);
    }

    RK_S32 fast_play = 1;
    ret = ctx->mpi->control(ctx->ctx, MPP_DEC_SET_ENABLE_FAST_PLAY, &fast_play);
    if (ret == MPP_OK) {
        CSG_LOGI("MPP_DEC", "Reset: ENABLE_FAST_PLAY enabled");
    } else {
        CSG_LOGW("MPP_DEC", "Reset: ENABLE_FAST_PLAY failed: %d", ret);
    }

    /* reset 后必须 drain DPB: 循环取出所有残留帧并丢弃.
     * (此函数仅由 seek / 停止路径调用, 暂停恢复不 reset 也不 drain,
     *  DPB 完整保留实现无痛恢复) */
    csg_mpp_decode_sleep_ms(30);

    int drained = 0;
    for (int loop = 0; loop < 10; loop++) {
        int got_frame = 0;
        for (int i = 0; i < 30; i++) {
            MppFrame frame = NULL;
            ret = ctx->mpi->decode_get_frame(ctx->ctx, &frame);
            if (ret == MPP_OK && frame) {
                mpp_frame_deinit(&frame);
                got_frame = 1;
                drained++;
            } else if (ret == MPP_OK && frame == NULL) {
                break;
            } else if (ret == MPP_ERR_TIMEOUT) {
                csg_mpp_decode_sleep_ms(1);
                continue;
            } else {
                break;
            }
        }
        if (!got_frame) break;
        csg_mpp_decode_sleep_ms(5);
    }

    if (ctx->frm_grp) {
        ctx->info_changed = 1;
    } else {
        ctx->info_changed = 0;
    }
    CSG_LOGI("MPP_DEC", "Reset drained %d frames, info_changed=%d",
         drained, ctx->info_changed);

    /* 清除关键帧跟踪环形队列, 避免 Reset 后残留旧条目导致 is_keyframe 错位 */
    ctx->kf_ring_head = 0;
    ctx->kf_ring_tail = 0;
    ctx->kf_ring_count = 0;

    csg_mpp_decode_lock_exit(ctx, 1);
    return 0;
}

/* 暂停时主动排空管线中残留的已解码帧.
 * 关键设计:
 *   1. 不调用 mpp->reset() — 保留 DPB 参考帧链, 恢复后可立即解码 P 帧
 *   2. 循环 drain decode_get_frame 直到空, 丢弃所有帧 (不回调)
 *   3. 保留 buffer group 和 MPP 上下文, 恢复时无需重建
 * 与 reset 对比:
 *   - reset 清空 DPB + 管线, 恢复时必须等关键帧重建参考链 (耗时 50-200ms)
 *   - drain 只清空输出队列, DPB 完整保留, 恢复瞬时衔接 (耗时 <5ms) */
int csg_mpp_decode_drain(void *handle)
{
    if (!handle) return -1;
    csg_mpp_decode_ctx_t *ctx = (csg_mpp_decode_ctx_t *)handle;

    int drained = 0;
    for (int loop = 0; loop < 10; loop++) {
        int got_frame = 0;
        for (int i = 0; i < 30; i++) {
            MppFrame frame = NULL;
            MPP_RET ret = ctx->mpi->decode_get_frame(ctx->ctx, &frame);
            if (ret == MPP_OK && frame) {
                mpp_frame_deinit(&frame);
                got_frame = 1;
                drained++;
            } else if (ret == MPP_OK && frame == NULL) {
                break;
            } else if (ret == MPP_ERR_TIMEOUT) {
                csg_mpp_decode_sleep_ms(1);
                continue;
            } else {
                break;
            }
        }
        if (!got_frame) break;
        csg_mpp_decode_sleep_ms(5);
    }

    CSG_LOGI("MPP_DEC", "Drained %d frames from pipeline (DPB preserved)", drained);
    return 0;
}

int csg_mpp_decode_get_width(void *handle)
{
    if (!handle) return 0;
    return ((csg_mpp_decode_ctx_t *)handle)->width;
}

int csg_mpp_decode_get_height(void *handle)
{
    if (!handle) return 0;
    return ((csg_mpp_decode_ctx_t *)handle)->height;
}

int csg_mpp_decode_stop(void *handle)
{
    if (!handle) return -1;
    csg_mpp_decode_ctx_t *ctx = (csg_mpp_decode_ctx_t *)handle;

    csg_mpp_decode_lock_enter(ctx, 1);

    if (ctx->ctx) {
        /* 1. 先 flush 输入队列: 丢弃所有未完成的 packet (通过一次 reset 清空) */
        ctx->mpi->reset(ctx->ctx);

        /* 2. 等待 reset 生效, 然后 drain 所有残留帧 */
        csg_mpp_decode_sleep_ms(30);

        /* 3. 循环 drain: 取出所有可能的输出帧 (reset 后这些帧可能包含部分解码) */
        for (int loop = 0; loop < 10; loop++) {
            int got_frame = 0;
            for (int i = 0; i < 30; i++) {
                MppFrame frame = NULL;
                MPP_RET ret = ctx->mpi->decode_get_frame(ctx->ctx, &frame);
                if (ret == MPP_OK && frame) {
                    /* 丢弃残留帧: 不再回调 */
                    mpp_frame_deinit(frame);
                    got_frame = 1;
                } else if (ret == MPP_OK && frame == NULL) {
                    break;
                } else if (ret == MPP_ERR_TIMEOUT) {
                    csg_mpp_decode_sleep_ms(1);
                    continue;
                } else {
                    break;
                }
            }
            if (!got_frame) break;
            csg_mpp_decode_sleep_ms(10);
        }
    }

    csg_mpp_decode_lock_exit(ctx, 1);
    CSG_LOGI("MPP_DEC", "Stop done");
    return 0;
}

void csg_mpp_decode_destroy(void *handle)
{
    if (!handle) return;
    csg_mpp_decode_ctx_t *ctx = (csg_mpp_decode_ctx_t *)handle;

    csg_mpp_decode_lock_enter(ctx, 1);

    /* 1. 先 stop (flush + drain) */
    if (ctx->ctx) {
        ctx->mpi->reset(ctx->ctx);
        csg_mpp_decode_sleep_ms(30);
        for (int loop = 0; loop < 5; loop++) {
            int got_frame = 0;
            for (int i = 0; i < 20; i++) {
                MppFrame frame = NULL;
                MPP_RET ret = ctx->mpi->decode_get_frame(ctx->ctx, &frame);
                if (ret == MPP_OK && frame) {
                    mpp_frame_deinit(frame);
                    got_frame = 1;
                } else if (ret == MPP_OK && frame == NULL) {
                    break;
                } else if (ret == MPP_ERR_TIMEOUT) {
                    csg_mpp_decode_sleep_ms(1);
                    continue;
                } else {
                    break;
                }
            }
            if (!got_frame) break;
        }
    }

    csg_mpp_decode_lock_exit(ctx, 1);

    /* 2. 释放 packet */
    if (ctx->packet) {
        mpp_packet_deinit(&ctx->packet);
    }

    /* 3. 销毁 MPP 上下文 */
    if (ctx->ctx) {
        mpp_destroy(ctx->ctx);
    }

    /* 4. 释放 buffer group */
    if (ctx->frm_grp) {
        mpp_buffer_group_put(ctx->frm_grp);
        ctx->frm_grp = NULL;
    }

    pthread_mutex_destroy(&ctx->lock);
    pthread_cond_destroy(&ctx->cond);
    free(ctx);
    CSG_LOGI("MPP_DEC", "Destroyed");
}