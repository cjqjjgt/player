/* SPDX-License-Identifier: MIT */
/*
 * nvr_multi.h - NVR 多路 RTSP 拉流/解码/录制模块
 *
 * 架构 (参考 nvr-main TPreviewManager / TRecordManage 的"每通道独立"思想):
 *   每通道 = 1 个 live555 RTSP 句柄 (拉流) + 1 个 MPP 解码器 (硬件解码)
 *          + 1 个 mp4_record 录制器 (编码流直写, 不软编码)
 *
 * 数据流:
 *   RTSP 帧回调 (live555 内部 decode_thread)
 *     ├─ → MPP 解码 (NV12) → RGA 缩放+转RGB → lvgl_display_update_video_chn
 *     └─ → mp4_record_push_frame (同源编码帧直写 MP4)
 *   RTSP 音频回调 → mp4_record_push_audio (G711/AAC 透传录制)
 *
 * ★ 每路只用一个 RTSP 句柄: 同一帧同时喂解码显示 + 录制。
 *   (单路播放器用双句柄"预览子码流+录制主码流", 多路下每路再开第二句柄
 *   会使 live555 实例数翻倍, 事件循环/堆压力过大, 且同一 URL 双实例
 *   存在历史堆损坏风险, 故多路统一"一路一句柄, 拉什么录什么"。
 *   需要录制独立主码流的场景请用单路播放器.)
 *
 * ★ 多路 MPP 解码实例: RK3572 MPP 按 ctx 隔离, 支持多实例并发。
 *   实测 4 路 1080p30 H264 可稳定并发, 超过请降低帧率/分辨率或减少路数,
 *   否则 VDEC 超载会触发 MPP errinfo 丢弃 (花屏).
 *
 * ★ 每路独立录制文件: /userdata/records/multi/chn<N>_YYYYMMDD_HHMMSS.mp4
 */

#ifndef __NVR_MULTI_H__
#define __NVR_MULTI_H__

#include "common.h"
#include "rtsp_live555.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 最大多路通道数 (与 lvgl_display 分屏上限一致) */
#define NVR_MULTI_MAX_CHN  32

/* 多路录制目录 (自动创建) */
#define NVR_MULTI_RECORD_DIR "/userdata/records/multi"

/* 每通道的 (预览URL, 录制主码流URL) 一对.
 * - 多路 (count>1): preview=子码流 (解码显示), record=主码流 (仅录制, 不解码)
 * - 单路 (count==1): 只填 preview (主码流), 同一句柄既显示又录制
 *   (record 留空即可, 程序回退复用 preview)
 */
typedef struct {
    char preview[512];   /* 预览 URL */
    char record[512];    /* 录制主码流 URL (单路可留空) */
} nvr_multi_url_t;

/* 每通道录制状态 */
typedef struct {
    int      active;            /* 1=本通道已启动拉流 */
    int      connected;         /* 1=预览已收到首帧 (拉流成功) */
    int      decoded;           /* 1=已创建 MPP 解码器并出帧 */
    int      recording;         /* 1=录制器已启动 */
    int      codec;             /* CSG_CODEC_H264/H265 */
    int      width, height;     /* 视频分辨率 */
    char     url[512];          /* 预览 URL */
    char     record_path[512];  /* 录制文件路径 */
    uint64_t video_frames;      /* 预览已送解码帧数 */
    uint64_t rec_frames;        /* 已录制帧数 */
} nvr_multi_chn_info_t;

/*
 * 启动多路拉流+解码+录制 (同步建好所有通道资源):
 *   urls:  每路 (预览URL, 录制主码流URL) 数组
 *   count: 路数, ∈[1, NVR_MULTI_MAX_CHN]
 *          count==1 (单路): 只用 urls[0].preview (主码流), 该路既显示又录制.
 *          count>1  (多路): 每路子码流解码显示 + 主码流独立录制.
 * 返回实际启动路数; 0 表示全部失败.
 * 内部按 count 自动设置 LVGL 分屏网格, 每路录制文件命名 chn<N>_时间戳.mp4.
 * 重复调用: 先自动 stop 再重新启动.
 */
int nvr_multi_start(const nvr_multi_url_t *urls, int count);

/* 停止全部多路拉流/解码/录制 (幂等): 逆序释放 RTSP→录制器→MPP→RGB缓冲,
 * 恢复单路显示. 在程序退出清理时调用. */
void nvr_multi_stop(void);

/* 当前活动通道数 (0=未运行) */
int nvr_multi_get_active(void);

/* 获取某通道状态 (线程安全); 通道无效或未运行返回 -1 */
int nvr_multi_get_info(int chn, nvr_multi_chn_info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* __NVR_MULTI_H__ */
