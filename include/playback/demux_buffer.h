/* SPDX-License-Identifier: MIT */
/*
 * demux_buffer.h - 解封装 → 解码 缓冲层 (动态数组双端队列)
 *
 * 数据流: 解封装线程(生产者) → [视频队列 | 音频队列] → 分发线程(消费者) → 硬件解码
 * 对应 GStreamer pipeline: demux → queue → decoder 中的 queue element.
 *
 * 作用:
 *   ① 解耦生产者/消费者线程: 解封装线程不再被 MPP BUFFER_FULL / ALSA 阻塞,
 *      视频与音频各自独立分发线程, 互不干扰.
 *   ② 平滑 IO/网络抖动: 队列吸收读取突发, 解码端按自身节奏消费.
 *   ③ 水位背压: 队列超限时 push 阻塞 (cond 等待), 天然限制内存占用,
 *      杜绝嵌入式设备因无限缓冲导致 OOM 死机 (内存安全铁则).
 *   ④ 帧数据深拷贝: 解封装线程内部缓冲区会被复用, 入队前必须深拷贝,
 *      消费者取帧后用 csg_demux_buffer_free_frame() 释放.
 *
 * 线程安全: 内部 pthread_mutex + pthread_cond 保护, 队列元素存取均加锁.
 * 停止/Seek 语义:
 *   - push 期间若被 flush (代次 generation 变化, 即发生 seek/stop), 放弃本帧返回 -1,
 *     生产者 (解封装线程) 随即回到主循环检查 seek_pending/running.
 *   - shutdown 后 pop 立即返回 -1, 消费者线程据此退出.
 */

#ifndef __CSG_DEMUX_BUFFER_H__
#define __CSG_DEMUX_BUFFER_H__

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 视频缓冲帧 (队列内深拷贝存储) */
typedef struct {
    uint8_t *data;        /* 深拷贝的编码帧数据 (队列拥有, 消费者须释放) */
    int      len;         /* 数据长度 */
    int      is_keyframe; /* 1=关键帧 */
    uint64_t pts;         /* 时间戳 (微秒) */
    int      codec_type;  /* CSG_CODEC_H264 / CSG_CODEC_H265 */
} _csg_demux_video_frame_t;

typedef _csg_demux_video_frame_t csg_demux_video_frame_t;

/* 音频缓冲帧 (队列内深拷贝存储) */
typedef struct {
    uint8_t *data;        /* 深拷贝的编码音频帧 (AAC raw) */
    int      len;
    int64_t  pts;         /* 时间戳 (微秒) */
} _csg_demux_audio_frame_t;

typedef _csg_demux_audio_frame_t csg_demux_audio_frame_t;

/* 缓冲层配置 (0 = 使用默认值) */
typedef struct {
    size_t video_max_frames;  /* 视频队列最大帧数 (默认 64) */
    size_t video_max_bytes;   /* 视频队列最大字节数 (默认 32MB) */
    size_t audio_max_frames;  /* 音频队列最大帧数 (默认 256) */
    size_t audio_max_bytes;   /* 音频队列最大字节数 (默认 4MB) */
    size_t video_init_cap;    /* 视频队列初始容量 (默认 16) */
    size_t audio_init_cap;    /* 音频队列初始容量 (默认 32) */
} _csg_demux_buffer_cfg_t;

typedef _csg_demux_buffer_cfg_t csg_demux_buffer_cfg_t;

typedef struct csg_demux_buffer_s csg_demux_buffer_t;

/* 创建缓冲层. cfg 可为 NULL (全默认). 返回 0 成功, out 返回句柄. */
int  csg_demux_buffer_create(const csg_demux_buffer_cfg_t *cfg,
                             csg_demux_buffer_t **out);

/* 销毁缓冲层 (需先 csg_demux_buffer_shutdown 唤醒并停止消费者).
 * 内部释放所有残留帧数据. */
void csg_demux_buffer_destroy(csg_demux_buffer_t *b);

/* 视频帧入队 (深拷贝). abort_flag 非空且 *abort_flag!=0 时放弃 (生产者停止检查).
 * 队列满时阻塞等待消费者 (带超时轮询); 若期间被 flush (代次变化) 放弃返回 -1.
 * 返回 0 入队成功, -1 放弃 (停止/seek/内存不足). */
int  csg_demux_buffer_push_video(csg_demux_buffer_t *b, const uint8_t *data, int len,
                                 int is_keyframe, uint64_t pts, int codec_type,
                                 const volatile int *abort_flag);

/* 音频帧入队 (深拷贝). 语义同上. */
int  csg_demux_buffer_push_audio(csg_demux_buffer_t *b, const uint8_t *data, int len,
                                 int64_t pts, const volatile int *abort_flag);

/* 视频帧出队 (阻塞, 内部 10ms 超时轮询以便及时响应停止).
 * 返回 1=取到帧(out 有效, 用完须 csg_demux_buffer_free_frame), 0=暂时为空(重试),
 * -1=缓冲层已 shutdown. */
int  csg_demux_buffer_pop_video(csg_demux_buffer_t *b, csg_demux_video_frame_t *out);

/* 音频帧出队. 语义同上. */
int  csg_demux_buffer_pop_audio(csg_demux_buffer_t *b, csg_demux_audio_frame_t *out);

/* 释放由 pop 取出的帧数据 */
void csg_demux_buffer_free_frame(void *data);

/* 清空两队列并释放所有残留帧数据, 代次++ 唤醒所有阻塞的 push/pop.
 * 用于 Seek / 停止: 生产者 (解封装线程) 在 push 中检测到代次变化后放弃旧帧,
 * 回主循环响应 seek/stop. */
void csg_demux_buffer_flush(csg_demux_buffer_t *b);

/* 标记停止并唤醒所有等待者 (销毁前调用): pop 返回 -1, push 立即放弃. */
void csg_demux_buffer_shutdown(csg_demux_buffer_t *b);

/* 统计 (调试/水位监控) */
size_t csg_demux_buffer_video_count(csg_demux_buffer_t *b);
size_t csg_demux_buffer_audio_count(csg_demux_buffer_t *b);
size_t csg_demux_buffer_video_bytes(csg_demux_buffer_t *b);
size_t csg_demux_buffer_audio_bytes(csg_demux_buffer_t *b);

#ifdef __cplusplus
}
#endif

#endif /* __CSG_DEMUX_BUFFER_H__ */
