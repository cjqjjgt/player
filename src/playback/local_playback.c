/* SPDX-License-Identifier: MIT */
/*
 * local_playback.c - 本地视频播放模块实现 (MP4v2 版)
 *
 * ★ 替换说明:
 *   本地回放范围界定为仅 MP4/MOV/M4V,
 *   采用 mp4v2 解封装, 视频转 Annex-B 喂 MPP 硬解。
 *
 *   工作流程:
 *     MP4Read → MP4ReadSample(video) 得到 length-prefixed NALU sample
 *     → 自研 mp4toannexb 转换 (4字节长度 → 00 00 00 01 start code)
 *     → 关键帧前从 avcC/hvcC 注入 SPS/PPS/VPS 参数集
 *     → 复用 MPP 硬件解码。
 *   音频: MP4ReadSample(audio) → 原始 AAC sample + ASC → FAAD2 解码。
 *
 * 依赖第三方库:
 *   头文件: thirdlib/mp4v2/mp4v2/mp4v2.h
 *   链接库: lib/libmp4v2.so.2.1.3 (交叉编译, 见 build/cmake-aarch64-toolchain.cmake)
 *
 * 历史踩坑点:
 *   1. MP4 视频 sample 是 length-prefixed NALU, MPP 需要 Annex-B,
 *      必须自行转换并注入参数集, 否则 seek/倍速后黑屏。
 *   2. 视频 PTS = DTS(sampleTime) + renderingOffset, 有 B 帧时二者不等,
 *      丢 DTS 会导致显示顺序错乱, 必须用合成 PTS。
 *   3. MP4 音频 track 的 timescale 即采样率; 音频 extradata 即 ASC。
 */

#include "local_playback.h"
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <errno.h>
#include <dirent.h>
#include <sys/stat.h>
#include <ctype.h>
#include <time.h>

#include "mov-reader.h"
#include "mov-format.h"
#include "mov-buffer.h"

#define MAX_LOCAL_CTX_NUM 4

/* RK3572 RGA2_PRO/MPP 建议上限 */
#define RK_MAX_WIDTH    4096
#define RK_MAX_HEIGHT   2304

/* mov_reader_read 样本缓冲 (2K I 帧 + 余量) */
#define CSG_LOCAL_READ_BUF_SIZE (2 * 1024 * 1024)
#define CSG_LOCAL_AUDIO_BUF_SIZE (64 * 1024)

static const char *s_csg_local_playback_exts[] = {
    ".mp4", ".mov", ".m4v", NULL
};

/* ==================== AAC ASC (AudioSpecificConfig) 解析 ====================
 * 历史踩坑: MP4 音频 track 的 timescale 在标准上应等于采样率, 但部分来源/
 * 录制器会写成 90000 (视频 timescale) 等非法值. 若把 timescale 直接当采样率
 * (旧实现: audio_sample_rate = timescale), 会得到 90000Hz 的错误采样率,
 * 导致 ALSA/FAAD2/音频主时钟按错误采样率计算, 音视频同步错乱.
 * 正确做法: 采样率/声道必须从 ASC 解析 (ISO/IEC 14496-3 Table 1.15),
 * timescale 仅保留用于 MP4 sample PTS 换算. */

/* ASC 采样率索引表 */
static const int csg_aac_asc_freqs[13] = {
    96000, 88200, 64000, 48000, 44100, 32000, 24000,
    22050, 16000, 12000, 11025, 8000, 7350
};

/* 从 ASC 解析 AAC 采样率; 失败或索引非法时返回 fallback.
 * 布局: asc[0] 高5bit=audioObjectType, 低3bit=freq_idx 高3位;
 *       asc[1] 高1bit=freq_idx 低1位, bit6-3=channel_config.
 * 注意: 对 SBR/HE-AAC (aot=5/29) 此解析得到的是 core 采样率,
 *       最终以 FAAD2 NeAACDecInit2 输出为准 (audio_render 用其输出率). */
static int csg_aac_asc_sample_rate(const uint8_t *asc, int asc_size, int fallback)
{
    if (!asc || asc_size < 2) return fallback;
    int freq_idx = ((asc[0] & 0x07) << 1) | ((asc[1] >> 7) & 0x01);
    if (freq_idx == 15) {
        /* 显式 24bit 采样率: asc[1]低7bit + asc[2]全8bit + asc[3]高9bit */
        if (asc_size >= 4) {
            int f = ((asc[1] & 0x7F) << 17) | (asc[2] << 9) |
                    ((asc[3] >> 7) & 0x1FF);
            if (f > 0) return f;
        }
        return fallback;
    }
    if (freq_idx < 13) return csg_aac_asc_freqs[freq_idx];
    return fallback;
}

/* 从 ASC 解析 AAC 声道数; channel_config=0 (PCE) 或非法时返回 fallback. */
static int csg_aac_asc_channels(const uint8_t *asc, int asc_size, int fallback)
{
    if (!asc || asc_size < 2) return fallback;
    int ch = (asc[1] >> 3) & 0x0F;
    return (ch > 0 && ch <= 8) ? ch : fallback;
}


/* ==================== 系统时钟 (替代 av_gettime) ==================== */
static int64_t csg_local_playback_now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

/* ==================== MP4 → Annex-B 转换 ==================== */

/* 将一条 NALU payload 以 start code 形式追加到 out */
static int csg_mp4_append_nal_annexb(uint8_t *out, int cap, int *o,
                                     const uint8_t *nal, int nal_len)
{
    if (!out || !o || !nal || nal_len <= 0) return -1;
    if (*o + 4 + nal_len > cap) return -1;
    out[(*o)++] = 0x00;
    out[(*o)++] = 0x00;
    out[(*o)++] = 0x00;
    out[(*o)++] = 0x01;
    memcpy(out + *o, nal, nal_len);
    *o += nal_len;
    return 0;
}

/* 解析 avcC (AVCDecoderConfigurationRecord), 抽出全部 SPS/PPS 打 Annex-B.
 * 布局: [5]=numOfSPS, sps 自 [6] 起; 随后 numOfPPS + pps. */
static int csg_mp4_avcc_to_annexb(const uint8_t *e, int es,
                                  uint8_t *out, int cap)
{
    if (!e || es < 8) return 0;
    int o = 0;
    int i = 5;
    uint8_t nsp = e[i] & 0x1F;
    i++;
    for (int k = 0; k < nsp && i + 2 <= es; k++) {
        int nl = (e[i] << 8) | e[i + 1];
        i += 2;
        if (i + nl > es) break;
        if (csg_mp4_append_nal_annexb(out, cap, &o, e + i, nl) < 0) break;
        i += nl;
    }
    if (i < es) {
        uint8_t npp = e[i];
        i++;
        for (int k = 0; k < npp && i + 2 <= es; k++) {
            int nl = (e[i] << 8) | e[i + 1];
            i += 2;
            if (i + nl > es) break;
            if (csg_mp4_append_nal_annexb(out, cap, &o, e + i, nl) < 0) break;
            i += nl;
        }
    }
    return o;
}

/* 解析 hvcC (HEVCDecoderConfigurationRecord), 抽出 VPS/SPS/PPS 打 Annex-B.
 * 布局: [22]=numOfArrays, 各 array 自 [23] 起.
 *   array: [0]=类型 (低6位), [1:2]=numNalus, 随后每 NALU: [2字节长度]+data */
static int csg_mp4_hvcc_to_annexb(const uint8_t *e, int es,
                                  uint8_t *out, int cap)
{
    if (!e || es < 24) return 0;
    int o = 0;
    uint8_t numArrays = e[22];
    int i = 23;
    for (int a = 0; a < numArrays && i < es; a++) {
        uint8_t type = e[i] & 0x3F;
        (void)type;
        i++;
        if (i + 2 > es) break;
        int numNalus = (e[i] << 8) | e[i + 1];
        i += 2;
        for (int k = 0; k < numNalus && i + 2 <= es; k++) {
            int nl = (e[i] << 8) | e[i + 1];
            i += 2;
            if (i + nl > es) break;
            if (csg_mp4_append_nal_annexb(out, cap, &o, e + i, nl) < 0) break;
            i += nl;
        }
    }
    return o;
}

/* ==================== Annex-B 码流 VCL/RAP 检测 (复用原逻辑) ==================== */

static int csg_local_playback_count_vcl_nal_units(const uint8_t *buf, int len)
{
    int vcl = 0;
    int i = 0;
    while (i + 3 < len) {
        if (buf[i] == 0 && buf[i + 1] == 0) {
            int sc_len = 0;
            if (buf[i + 2] == 1) {
                sc_len = 3;
            } else if (i + 4 < len && buf[i + 2] == 0 && buf[i + 3] == 1) {
                sc_len = 4;
            }
            if (sc_len) {
                int nal_start = i + sc_len;
                if (nal_start + 2 <= len) {
                    uint8_t type = (buf[nal_start] >> 1) & 0x3F;
                    if (type <= 9 || (type >= 16 && type <= 21))
                        vcl++;
                }
                i = nal_start;
                continue;
            }
        }
        i++;
    }
    return vcl;
}

static int csg_bitpos_get_bit(const uint8_t *buf, int len, int bit_pos)
{
    if (bit_pos < 0) return -1;
    int byte_idx = bit_pos >> 3;
    if (byte_idx >= len) return -1;
    return (buf[byte_idx] >> (7 - (bit_pos & 7))) & 1;
}

static int csg_bitpos_read_ue(const uint8_t *buf, int len, int *bit_pos)
{
    int zeros = 0;
    while (1) {
        int bit = csg_bitpos_get_bit(buf, len, *bit_pos);
        (*bit_pos)++;
        if (bit < 0) return -1;
        if (bit == 1) break;
        zeros++;
        if (zeros > 31) return -1;
    }
    int value = (1 << zeros) - 1;
    for (int i = 0; i < zeros; i++) {
        int bit = csg_bitpos_get_bit(buf, len, *bit_pos);
        (*bit_pos)++;
        if (bit < 0) return -1;
        value |= bit << (zeros - 1 - i);
    }
    return value;
}

static int csg_local_playback_h264_is_i_slice(const uint8_t *nal, int nal_len)
{
    const uint8_t *p = nal + 1;
    if (nal_len <= 1) return 0;

    uint8_t clean[16];
    int ci = 0;
    int zeros = 0;
    while (p < nal + nal_len && ci < (int)sizeof(clean)) {
        uint8_t b = *p++;
        if (zeros >= 2 && b == 0x03) { zeros = 0; continue; }
        clean[ci++] = b;
        if (b == 0) zeros++; else zeros = 0;
    }
    if (ci < 2) return 0;

    int bit_pos = 0;
    int first_mb = csg_bitpos_read_ue(clean, ci, &bit_pos);
    if (first_mb < 0) return 0;
    int slice_type = csg_bitpos_read_ue(clean, ci, &bit_pos);
    if (slice_type < 0) return 0;
    return (slice_type == 2 || slice_type == 7) ? 1 : 0;
}

static int csg_local_playback_scan_is_rap(int codec_type, const uint8_t *buf, int len)
{
    if (!buf || len <= 4) return 0;
    int i = 0;
    while (i + 3 < len) {
        if (buf[i] == 0 && buf[i + 1] == 0) {
            int sc_len = 0;
            if (buf[i + 2] == 1) {
                sc_len = 3;
            } else if (i + 4 < len && buf[i + 2] == 0 && buf[i + 3] == 1) {
                sc_len = 4;
            }
            if (sc_len) {
                int nal_start = i + sc_len;
                if (nal_start < len) {
                    uint8_t nal0 = buf[nal_start];
                    if (codec_type == CSG_CODEC_H265) {
                        uint8_t type = (nal0 >> 1) & 0x3F;
                        if (type >= 16 && type <= 21) return 1;
                    } else {
                        uint8_t type = nal0 & 0x1F;
                        if (type == 5) return 1;
                        if (type == 1 && csg_local_playback_h264_is_i_slice(
                                buf + nal_start, len - nal_start)) {
                            return 1;
                        }
                    }
                }
                i = nal_start;
                continue;
            }
        }
        i++;
    }
    return 0;
}

/* ==================== libmov IO & 上下文 ==================== */

typedef struct {
    FILE *fp;
} csg_local_playback_mov_io_t;

static int csg_local_playback_mov_read(void *param, void *data, uint64_t bytes)
{
    csg_local_playback_mov_io_t *io = (csg_local_playback_mov_io_t *)param;
    return (io && io->fp && fread(data, 1, (size_t)bytes, io->fp) == bytes) ? 0 : -1;
}

static int csg_local_playback_mov_write(void *param, const void *data, uint64_t bytes)
{
    (void)param; (void)data; (void)bytes;
    return -1;   /* 只读 */
}

static int csg_local_playback_mov_seek(void *param, int64_t offset)
{
    csg_local_playback_mov_io_t *io = (csg_local_playback_mov_io_t *)param;
    if (!io || !io->fp) return -1;
    /* libmov 约定: offset>=0 从文件头 SEEK_SET; offset<0 从文件尾 SEEK_END */
    if (offset < 0)
        return fseeko(io->fp, offset, SEEK_END) == 0 ? 0 : -1;
    return fseeko(io->fp, offset, SEEK_SET) == 0 ? 0 : -1;
}

static int64_t csg_local_playback_mov_tell(void *param)
{
    csg_local_playback_mov_io_t *io = (csg_local_playback_mov_io_t *)param;
    return (io && io->fp) ? (int64_t)ftello(io->fp) : -1;
}

static const struct mov_buffer_t s_csg_local_playback_mov_buffer = {
    csg_local_playback_mov_read,
    csg_local_playback_mov_write,
    csg_local_playback_mov_seek,
    csg_local_playback_mov_tell
};

typedef struct {
    mov_reader_t  *mov;
    FILE          *fp;
    csg_local_playback_mov_io_t mov_io;
    uint32_t       mov_video_track;   /* (uint32_t)-1 表示无效 */
    uint32_t       mov_audio_track;
    int64_t        mov_duration_ms;   /* mov_reader_getduration (fMP4 下可靠) */

    int            codec_type;      /* CSG_CODEC_H264/H265 */
    int            video_width;
    int            video_height;
    int            video_fps;
    int            nal_length_size;   /* 视频 NALU 长度字段字节数 (avcC[4]&3+1 / hvcC[21]&3+1) */
    int64_t        duration_us;
    int64_t        current_position_us;
    int            running;
    int            paused;
    int            finished;
    int            thread_joined;
    pthread_t      thread;
    pthread_mutex_t mutex;
    csg_local_frame_callback callback;
    csg_local_event_callback event_cb;
    void          *user_data;
    void          *event_user_data;

    /* 音频流 */
    int              audio_codec_id;   /* CSG_AUDIO_CODEC_AAC / G711A / G711U */
    int              audio_sample_rate;
    int              audio_channels;
    uint8_t       *audio_extradata;  /* AAC: ASC; G711: NULL */
    int              audio_extradata_size;
    csg_local_audio_frame_callback audio_callback;
    void            *audio_user_data;
    csg_player_stats_t   stats;

    int              seek_pending;
    double           seek_position;
    int64_t          resume_target_pts;
    char             file_path[512];
    int              downscale;
    float            speed;
    volatile int     rate_reset_pending;

    /* 视频参数集 & 样本缓冲 (create 时一次性分配, 运行时不 malloc) */
    uint8_t         *video_extradata;  /* avcC/hvcC (onvideo 回调填充) */
    int              video_extradata_size;
    uint8_t         *read_buf;         /* mov_reader_read 缓冲 */
    uint32_t         read_buf_size;
    uint8_t         *audio_sample_buf; /* 音频样本暂存 (onread 回调复制) */
    uint32_t         audio_sample_buf_size;
    uint8_t         *annexb_buf;       /* 视频转 Annex-B + 注入参数集输出 */
    int              annexb_cap;

    /* onread 回调暂存的当前帧状态 */
    int              pending_is_video;
    int              pending_is_key;
    int64_t          pending_pts_us;
    int              pending_video_len;
    int              pending_audio_len;

    volatile int    *seek_fast_feed_done_ptr;
    int64_t          last_seek_kf_pts;

    int              rap_scan_active;
    int              rap_scan_count;
    int              rap_found_count;
} csg_local_playback_ctx_t;

static csg_local_playback_ctx_t *s_csg_local_playback_ctx_table[MAX_LOCAL_CTX_NUM] = {NULL};

int csg_local_playback_is_supported(const char *file_path)
{
    if (!file_path) return 0;
    const char *dot = strrchr(file_path, '.');
    if (!dot) return 0;
    char ext[16];
    int i;
    for (i = 0; i < (int)strlen(dot) && i < 15; i++) {
        ext[i] = tolower((unsigned char)dot[i]);
    }
    ext[i] = '\0';
    for (int j = 0; s_csg_local_playback_exts[j]; j++) {
        if (strcmp(ext, s_csg_local_playback_exts[j]) == 0) return 1;
    }
    return 0;
}

/* ==================== 文件信息探测 (libmov) ==================== */

typedef struct {
    int width;
    int height;
    int got_video;
} csg_local_playback_probe_t;

static void csg_local_playback_probe_on_video(void *param, uint32_t track, uint8_t object,
                                              int width, int height,
                                              const void *extra, size_t bytes)
{
    csg_local_playback_probe_t *p = (csg_local_playback_probe_t *)param;
    (void)track; (void)extra; (void)bytes;
    if (p && (object == MOV_OBJECT_H264 || object == MOV_OBJECT_H265)) {
        p->width = width;
        p->height = height;
        p->got_video = 1;
    }
}

static int csg_local_playback_probe_file_info(const char *full_path,
                                              int *width, int *height,
                                              int64_t *duration_us)
{
    if (!full_path || !width || !height || !duration_us) return -1;

    FILE *fp = fopen(full_path, "rb");
    if (!fp) return -1;

    csg_local_playback_mov_io_t io = { fp };
    mov_reader_t *mov = mov_reader_create(&s_csg_local_playback_mov_buffer, &io);
    if (!mov) {
        fclose(fp);
        return -1;
    }

    csg_local_playback_probe_t probe = { 0 };
    struct mov_reader_trackinfo_t ti;
    memset(&ti, 0, sizeof(ti));
    ti.onvideo = csg_local_playback_probe_on_video;
    mov_reader_getinfo(mov, &ti, &probe);

    int ret = -1;
    if (probe.got_video) {
        *width = probe.width;
        *height = probe.height;
        int64_t dur_ms = (int64_t)mov_reader_getduration(mov);
        *duration_us = (dur_ms > 0) ? dur_ms * 1000 : -1;
        /* ★ 只要视频轨存在即接受; 真实分辨率由 MPP 从 SPS 解出,
         *   RGA 缩放使用解码后的实际宽高, 不依赖容器值。 */
        ret = 0;
    }
    mov_reader_destroy(mov);
    fclose(fp);
    return ret;
}

int csg_local_playback_scan_dir(const char *dir_path,
                            csg_local_file_info_t *files, int max_count)
{
    if (!dir_path || !files || max_count <= 0) return 0;

    DIR *dir = opendir(dir_path);
    if (!dir) {
        CSG_LOGW("LOCAL_PLAY", "Cannot open dir: %s", dir_path);
        return 0;
    }

    int count = 0;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL && count < max_count) {
        if (!csg_local_playback_is_supported(entry->d_name)) continue;

        char full_path[512];
        snprintf(full_path, sizeof(full_path), "%s/%s", dir_path, entry->d_name);

        struct stat st;
        if (stat(full_path, &st) != 0 || !S_ISREG(st.st_mode)) continue;

        memset(&files[count], 0, sizeof(csg_local_file_info_t));
        snprintf(files[count].path, sizeof(files[count].path), "%s", full_path);
        snprintf(files[count].name, sizeof(files[count].name), "%s", entry->d_name);
        files[count].file_size = st.st_size;

        if (csg_local_playback_probe_file_info(full_path,
                                        &files[count].width,
                                        &files[count].height,
                                        &files[count].duration_us) == 0) {
            count++;
        }
    }
    closedir(dir);
    CSG_LOGI("LOCAL_PLAY", "Scanned %d files from %s", count, dir_path);
    return count;
}

/* 填充单个视频文件的信息 (分辨率、时长), 供递归扫描复用 */
static int csg_local_playback_fill_file_info(csg_local_file_info_t *info,
                                         const char *full_path)
{
    if (!info || !full_path) return -1;

    memset(info, 0, sizeof(csg_local_file_info_t));
    snprintf(info->path, sizeof(info->path), "%s", full_path);

    const char *name = strrchr(full_path, '/');
    name = name ? name + 1 : full_path;
    strncpy(info->name, name, sizeof(info->name) - 1);
    info->name[sizeof(info->name) - 1] = '\0';

    struct stat st;
    if (stat(full_path, &st) == 0) {
        info->file_size = st.st_size;
    }

    if (csg_local_playback_probe_file_info(full_path,
                                    &info->width, &info->height,
                                    &info->duration_us) != 0) {
        return -1;
    }
    /* ★ 宽高为 0 也放行 (muxer 未写 tkhd 宽高), 播放时由 SPS/解码器给出 */
    return 0;
}

static int csg_local_playback_scan_recursive(const char *dir_path,
                                         csg_local_file_info_t *files,
                                         int max_count, int *count)
{
    DIR *dir = opendir(dir_path);
    if (!dir) return -1;

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL && *count < max_count) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
            continue;

        char full_path[512];
        snprintf(full_path, sizeof(full_path), "%s/%s", dir_path, entry->d_name);

        struct stat st;
        if (stat(full_path, &st) != 0) continue;

        if (S_ISDIR(st.st_mode)) {
            csg_local_playback_scan_recursive(full_path, files, max_count, count);
        } else if (S_ISREG(st.st_mode)) {
            if (!csg_local_playback_is_supported(entry->d_name)) continue;
            if (csg_local_playback_fill_file_info(&files[*count], full_path) == 0) {
                (*count)++;
            } else {
                CSG_LOGW("LOCAL_PLAY", "Skipping %s: fill info failed", full_path);
            }
        }
    }
    closedir(dir);
    return 0;
}

int csg_local_playback_scan_dir_recursive(const char *dir_path,
                                      csg_local_file_info_t *files, int max_count)
{
    if (!dir_path || !files || max_count <= 0) return 0;

    int count = 0;
    csg_local_playback_scan_recursive(dir_path, files, max_count, &count);
    CSG_LOGI("LOCAL_PLAY", "Recursive scan: %d files from %s", count, dir_path);
    return count;
}

/* ==================== Seek (libmov mov_reader_seek) ==================== */

/* 跳转到目标时刻: mov_reader_seek 内部用 stss 索引对齐到 <= 目标的关键帧,
 * 输出落点时间 (ms). 默认模式 (非 FMP4_FAST) 下 create 已构建完整样本表,
 * stss 由 mov_index_build 从 sample flags 生成, 对 fMP4/普通 MP4 均可靠. */
static int csg_local_playback_seek_to_keyframe(csg_local_playback_ctx_t *ctx,
                                               int64_t want_ms, int64_t *got_ms)
{
    if (!ctx || !ctx->mov) return 0;
    if (want_ms < 0) want_ms = 0;
    int64_t ts = want_ms;
    if (mov_reader_seek(ctx->mov, &ts) != 0)
        return 0;
    if (got_ms) *got_ms = ts;
    return 1;
}

/* ==================== libmov 轨道信息 & 样本回调 ==================== */

static void csg_local_playback_on_video_info(void *param, uint32_t track, uint8_t object,
                                             int width, int height,
                                             const void *extra, size_t bytes)
{
    csg_local_playback_ctx_t *ctx = (csg_local_playback_ctx_t *)param;
    if (!ctx) return;
    if (object == MOV_OBJECT_H264 || object == MOV_OBJECT_H265) {
        ctx->mov_video_track = track;
        ctx->codec_type = (object == MOV_OBJECT_H264) ? CSG_CODEC_H264 : CSG_CODEC_H265;
        ctx->video_width = width;
        ctx->video_height = height;
        /* extra = avcC/hvcC, 直接缓存 (无需再合成) */
        if (extra && bytes > 0) {
            free(ctx->video_extradata);
            ctx->video_extradata = malloc(bytes);
            if (ctx->video_extradata) {
                memcpy(ctx->video_extradata, extra, bytes);
                ctx->video_extradata_size = (int)bytes;
            }
        }
    }
}

static void csg_local_playback_on_audio_info(void *param, uint32_t track, uint8_t object,
                                             int channels, int bits, int rate,
                                             const void *extra, size_t bytes)
{
    csg_local_playback_ctx_t *ctx = (csg_local_playback_ctx_t *)param;
    (void)bits;
    if (!ctx) return;
    if (object == MOV_OBJECT_AAC || object == MOV_OBJECT_G711a || object == MOV_OBJECT_G711u) {
        ctx->mov_audio_track = track;
        if (object == MOV_OBJECT_AAC) {
            ctx->audio_codec_id = CSG_AUDIO_CODEC_AAC;
            if (extra && bytes > 0) {
                free(ctx->audio_extradata);
                ctx->audio_extradata = malloc(bytes);
                if (ctx->audio_extradata) {
                    memcpy(ctx->audio_extradata, extra, bytes);
                    ctx->audio_extradata_size = (int)bytes;
                }
            }
            ctx->audio_sample_rate = csg_aac_asc_sample_rate(
                ctx->audio_extradata, ctx->audio_extradata_size,
                rate > 0 ? rate : 8000);
            ctx->audio_channels = csg_aac_asc_channels(
                ctx->audio_extradata, ctx->audio_extradata_size,
                channels > 0 ? channels : 1);
        } else {
            /* G.711 原始字节直写, 无 extra data */
            ctx->audio_codec_id = (object == MOV_OBJECT_G711a)
                ? CSG_AUDIO_CODEC_G711A : CSG_AUDIO_CODEC_G711U;
            ctx->audio_sample_rate = rate > 0 ? rate : 8000;
            ctx->audio_channels = channels > 0 ? channels : 1;
        }
    }
}

/* mov_reader_read 的样本回调: 同步调用, 必须快速返回.
 * 视频 → 转 Annex-B (关键帧注入参数集) 到 annexb_buf;
 * 音频 → 复制原始字节 (AAC raw / G711) 到 audio_sample_buf. */
static void csg_local_playback_on_sample(void *param, uint32_t track, const void *data,
                                         size_t bytes, int64_t pts, int64_t dts, int flags)
{
    csg_local_playback_ctx_t *ctx = (csg_local_playback_ctx_t *)param;
    (void)dts;
    if (!ctx) return;
    ctx->pending_is_video = 0;
    ctx->pending_is_key = 0;
    ctx->pending_pts_us = pts * 1000;
    ctx->pending_video_len = 0;
    ctx->pending_audio_len = 0;

    if (track == ctx->mov_video_track) {
        ctx->pending_is_video = 1;
        ctx->pending_is_key = (flags & MOV_AV_FLAG_KEYFREAME) ? 1 : 0;
        int o = 0;
        if (ctx->pending_is_key) {
            if (ctx->codec_type == CSG_CODEC_H265)
                o = csg_mp4_hvcc_to_annexb(ctx->video_extradata,
                                           ctx->video_extradata_size,
                                           ctx->annexb_buf, ctx->annexb_cap);
            else
                o = csg_mp4_avcc_to_annexb(ctx->video_extradata,
                                           ctx->video_extradata_size,
                                           ctx->annexb_buf, ctx->annexb_cap);
            if (o <= 0) {
                CSG_LOGW("LOCAL_PLAY", "keyframe param set empty (extradata=%d)",
                     ctx->video_extradata_size);
            }
        }
        int i = 0;
        int lsz = ctx->nal_length_size;
        if (lsz < 1 || lsz > 4) lsz = 4;
        const uint8_t *p = (const uint8_t *)data;
        while (i + lsz <= (int)bytes) {
            uint32_t n = 0;
            for (int b = 0; b < lsz; b++) n = (n << 8) | p[i++];
            if (n == 0 || i + (int)n > (int)bytes) break;
            if (csg_mp4_append_nal_annexb(ctx->annexb_buf, ctx->annexb_cap,
                                          &o, p + i, (int)n) < 0)
                break;
            i += (int)n;
        }
        ctx->pending_video_len = o;
    } else if (track == ctx->mov_audio_track) {
        if (bytes > ctx->audio_sample_buf_size)
            bytes = ctx->audio_sample_buf_size;
        memcpy(ctx->audio_sample_buf, data, bytes);
        ctx->pending_audio_len = (int)bytes;
    }
}

int csg_local_playback_is_supported_checked = 0;

static void *csg_local_playback_thread(void *arg)
{
    csg_local_playback_ctx_t *ctx = (csg_local_playback_ctx_t *)arg;

    CSG_LOGI("LOCAL_PLAY", "Thread started: %s", ctx->file_path);

    int64_t base_pts = 0;
    int64_t base_sys = 0;
    int rate_initialized = 0;
    int64_t pause_accum_us = 0;
    int64_t pause_start_us = 0;
    int was_paused = 0;
    int64_t resume_target_pts = 0;
    int64_t catchup_target_us = 0;
    int read_fail_cnt = 0;   /* 连续读帧失败计数, 防止坏轨死循环 */

resume_read_loop:
    while (ctx->running) {
        int paused, seek_pending;
        double seek_pos;
        int64_t thread_resume_target = 0;

        pthread_mutex_lock(&ctx->mutex);
        paused = ctx->paused;
        seek_pending = ctx->seek_pending;
        seek_pos = ctx->seek_position;
        thread_resume_target = ctx->resume_target_pts;
        pthread_mutex_unlock(&ctx->mutex);

        if (seek_pending) {
            pthread_mutex_lock(&ctx->mutex);
            ctx->seek_pending = 0;
            pthread_mutex_unlock(&ctx->mutex);

            int64_t target_us = (ctx->duration_us > 0)
                                ? (int64_t)(seek_pos * ctx->duration_us) : 0;
            int64_t want_ms = target_us / 1000;
            int64_t land_ms = want_ms;
            if (csg_local_playback_seek_to_keyframe(ctx, want_ms, &land_ms)) {
                int64_t kf_pts_us = land_ms * 1000;
                rate_initialized = 0;
                pause_accum_us = 0;
                base_pts = 0;
                base_sys = 0;
                was_paused = 0;
                /* 直接跳到关键帧立即显示 */
                catchup_target_us = kf_pts_us;
                ctx->last_seek_kf_pts = kf_pts_us;
                CSG_LOGI("LOCAL_PLAY", "seek to %.1f%% done (kf_pts=%lldus)",
                     seek_pos * 100.0, (long long)kf_pts_us);
                pthread_mutex_lock(&ctx->mutex);
                ctx->current_position_us = target_us;
                pthread_mutex_unlock(&ctx->mutex);
                if (ctx->event_cb) {
                    ctx->event_cb(CSG_LOCAL_EVENT_SEEK_DONE, ctx->event_user_data);
                }
            }
            continue;
        }

        if (paused) {
            if (!was_paused) {
                pause_start_us = csg_local_playback_now_us();
                was_paused = 1;
                resume_target_pts = 0;
                catchup_target_us = 0;
                CSG_LOGI("LOCAL_PLAY", "Pause engaged (sample pos preserved)");
            }
            usleep(5000);
            continue;
        } else {
            if (was_paused) {
                int64_t now = csg_local_playback_now_us();
                pause_accum_us += (now - pause_start_us);
                was_paused = 0;

                /* 零延迟恢复: 不 seek, 文件位置停留在暂停帧之后 */
                rate_initialized = 0;
                pause_accum_us = 0;
                base_pts = 0;
                base_sys = 0;
                catchup_target_us = 0;

                if (thread_resume_target > 0) {
                    int64_t want_ms = thread_resume_target / 1000;
                    int64_t land_ms = want_ms;
                    if (csg_local_playback_seek_to_keyframe(ctx, want_ms, &land_ms)) {
                        resume_target_pts = thread_resume_target;
                        ctx->last_seek_kf_pts = land_ms * 1000;
                    }
                    pthread_mutex_lock(&ctx->mutex);
                    ctx->resume_target_pts = 0;
                    pthread_mutex_unlock(&ctx->mutex);
                    CSG_LOGI("LOCAL_PLAY", "Resumed: seek to keyframe before pts=%lld",
                         (long long)thread_resume_target);
                } else {
                    resume_target_pts = 0;
                    CSG_LOGI("LOCAL_PLAY", "Resumed: instant (sample pos preserved)");
                }
            }
        }

        /* 读取下一帧: mov_reader 按时间顺序交错返回视频/音频,
         * on_sample 回调暂存一帧到 ctx->pending_* */
        ctx->pending_is_video = 0;
        ctx->pending_audio_len = 0;
        int ret = mov_reader_read(ctx->mov, ctx->read_buf, ctx->read_buf_size,
                                  csg_local_playback_on_sample, ctx);
        if (ret <= 0) {
            /* EOF 或错误 */
            CSG_LOGI("LOCAL_PLAY", "End of file (ret=%d)", ret);
            if (ctx->event_cb) {
                ctx->event_cb(CSG_LOCAL_EVENT_EOF, ctx->event_user_data);
            }
            ctx->finished = 1;
            while (ctx->running) {
                int was_seek = 0;
                int was_resume = 0;
                double new_seek_pos = 0;
                int64_t new_resume_pts = 0;

                pthread_mutex_lock(&ctx->mutex);
                if (ctx->seek_pending) {
                    was_seek = 1;
                    new_seek_pos = ctx->seek_position;
                    ctx->seek_pending = 0;
                    ctx->paused = 0;
                }
                if (ctx->resume_target_pts > 0) {
                    was_resume = 1;
                    new_resume_pts = ctx->resume_target_pts;
                    ctx->resume_target_pts = 0;
                    ctx->paused = 0;
                }
                pthread_mutex_unlock(&ctx->mutex);

                if (was_seek || was_resume) {
                    int64_t target = (ctx->duration_us > 0)
                        ? (int64_t)(was_seek ? new_seek_pos * ctx->duration_us
                                           : new_resume_pts)
                        : 0;
                    int64_t want_ms = target / 1000;
                    int64_t land_ms = want_ms;
                    if (csg_local_playback_seek_to_keyframe(ctx, want_ms, &land_ms)) {
                        ctx->last_seek_kf_pts = land_ms * 1000;
                        ctx->finished = 0;
                        rate_initialized = 0;
                        pause_accum_us = 0;
                        base_pts = 0;
                        base_sys = 0;
                        was_paused = 0;
                        catchup_target_us = target;
                        if (ctx->event_cb) {
                            ctx->event_cb(CSG_LOCAL_EVENT_SEEK_DONE, ctx->event_user_data);
                        }
                        goto resume_read_loop;
                    }
                }
                usleep(5000);
            }
            break;
        }

        if (ctx->pending_is_video) {
            uint8_t *send_data = ctx->annexb_buf;
            int send_len = ctx->pending_video_len;
            int is_kf = ctx->pending_is_key;
            int64_t pts = ctx->pending_pts_us;

            if (send_len <= 0) {
                read_fail_cnt++;
                if (read_fail_cnt >= 100) {   /* 连续失败约 1s, 判定为不可恢复 */
                    CSG_LOGE("LOCAL_PLAY", "read video sample failed %d times consecutively, abort playback",
                         read_fail_cnt);
                    if (ctx->event_cb)
                        ctx->event_cb(CSG_LOCAL_EVENT_ERROR, ctx->event_user_data);
                    ctx->finished = 1;
                    ctx->running = 0;
                    break;
                }
                CSG_LOGW("LOCAL_PLAY", "read video sample failed, retrying... (%d)", read_fail_cnt);
                usleep(10000);
                continue;
            }

            read_fail_cnt = 0;
            ctx->stats.recv_packets++;
            ctx->stats.recv_bytes += send_len;

            if (pts > 0) {
                pthread_mutex_lock(&ctx->mutex);
                ctx->current_position_us = pts;
                pthread_mutex_unlock(&ctx->mutex);
            }

            if (resume_target_pts > 0 && pts >= resume_target_pts) {
                CSG_LOGI("LOCAL_PLAY", "Reached resume target: pts=%lld >= target=%lld",
                     (long long)pts, (long long)resume_target_pts);
                resume_target_pts = 0;
            }

            if (ctx->rate_reset_pending) {
                ctx->rate_reset_pending = 0;
                rate_initialized = 0;
                pause_accum_us = 0;
                CSG_LOGI("LOCAL_PLAY", "Rate calibration reset for new speed: %.2fx", ctx->speed);
            }

            /* Seek 追赶: 关键帧→目标 按解码速度喂帧 */
            if (catchup_target_us > 0 && pts > 0) {
                if (pts >= catchup_target_us) {
                    CSG_LOGI("LOCAL_PLAY", "Catch-up done: pts=%lld >= target=%lld",
                         (long long)pts, (long long)catchup_target_us);
                    catchup_target_us = 0;
                    rate_initialized = 0;
                    pause_accum_us = 0;
                }
            }

            if (rate_initialized && pts > 0 && catchup_target_us == 0) {
                float spd = ctx->speed > 0.0f ? ctx->speed : 1.0f;
                int64_t pts_delta = pts - base_pts;
                int64_t target_sys = base_sys + (int64_t)((double)pts_delta / (double)spd)
                                     - pause_accum_us;
                int64_t now = csg_local_playback_now_us();
                int64_t diff = target_sys - now;

                if (diff > 0 && diff < 1000000) {
                    usleep(diff);
                } else if (diff <= 0) {
                    int64_t behind = -diff;
                    int64_t behind_thresh = 500000;
                    if (spd > 1.0f) {
                        behind_thresh = (int64_t)(500000.0 * spd);
                        if (behind_thresh > 3000000) behind_thresh = 3000000;
                    }
                    if (behind > behind_thresh) {
                        rate_initialized = 0;
                        pause_accum_us = 0;
                    }
                }
            }

            if (!rate_initialized && pts > 0) {
                base_pts = pts;
                base_sys = csg_local_playback_now_us();
                pause_accum_us = 0;
                rate_initialized = 1;
            }

            /* 比特流 RAP 检测: 覆盖容器 sync 标志 (容错) */
            if (ctx->rap_scan_active) {
                int real_rap = csg_local_playback_scan_is_rap(ctx->codec_type,
                                                              send_data, send_len);
                if (real_rap) ctx->rap_found_count++;
                ctx->rap_scan_count++;
                if (ctx->rap_scan_count >= 200 && ctx->rap_found_count == 0) {
                    CSG_LOGW("LOCAL_PLAY", "No RAP found after %d packets, "
                         "fallback to container sync flag", ctx->rap_scan_count);
                    ctx->rap_scan_active = 0;
                } else {
                    is_kf = real_rap;
                }
            }

            /* 倍速关键帧统计诊断 */
            if (is_kf) {
                int vcl = csg_local_playback_count_vcl_nal_units(send_data, send_len);
                if (vcl > 1) {
                    CSG_LOGW("LOCAL_PLAY", "KEYFRAME packet VCL=%d len=%d (multi-frame sample)",
                         vcl, send_len);
                }
            }

            if (ctx->callback) {
                ctx->callback(send_data, send_len, is_kf, (uint64_t)pts,
                              ctx->codec_type, ctx->user_data);
            }

            /* Seek 快速喂帧停止 */
            if (catchup_target_us > 0 && ctx->seek_fast_feed_done_ptr
                && *ctx->seek_fast_feed_done_ptr) {
                CSG_LOGI("LOCAL_PLAY", "Seek display done, stop fast-feed at pts=%lld",
                     (long long)pts);
                catchup_target_us = 0;
                rate_initialized = 0;
                pause_accum_us = 0;
                *ctx->seek_fast_feed_done_ptr = 0;
            }
        } else if (ctx->pending_audio_len > 0) {
            /* 音频: mov_reader 按时间顺序交错返回, 直接回调 (无需追赶).
             * G711 原始字节 / AAC raw 均原样透传, 上层负责解码/渲染. */
            if (ctx->audio_callback) {
                ctx->audio_callback(ctx->audio_sample_buf, ctx->pending_audio_len,
                                    ctx->pending_pts_us,
                                    ctx->audio_codec_id,
                                    ctx->audio_sample_rate,
                                    ctx->audio_channels,
                                    0,
                                    ctx->audio_extradata,
                                    ctx->audio_extradata_size,
                                    ctx->audio_user_data);
            }
        } else {
            /* 未知轨, 跳过 */
            continue;
        }
    }

    ctx->finished = 1;
    ctx->running = 0;
    CSG_LOGI("LOCAL_PLAY", "Thread stopped");
    return NULL;
}

/* ==================== join & cleanup ==================== */

static void csg_local_playback_join_and_cleanup(int slot)
{
    if (slot < 0 || slot >= MAX_LOCAL_CTX_NUM) return;
    csg_local_playback_ctx_t *ctx = s_csg_local_playback_ctx_table[slot];
    if (!ctx) return;

    if (ctx->running) ctx->running = 0;

    /* ★ v5.4 修复: 检查 thread_joined 标志。
     * csg_local_playback_stop 使用 pthread_detach 分离线程,
     * 对已 detach 的线程调用 pthread_join/pthread_tryjoin_np 返回 EINVAL,
     * 50 次循环浪费 500ms 且阻塞 LVGL 线程导致 UI 卡死。
     * 修复: 已 join/detach 的线程直接跳过等待, 避免阻塞。 */
    if (!ctx->thread_joined) {
        int joined = 0;
        for (int i = 0; i < 50; i++) {
            int ret = pthread_tryjoin_np(ctx->thread, NULL);
            if (ret == 0) { joined = 1; break; }
            if (ret == EINVAL) break;   /* 已 detach, 无法 join, 直接退出 */
            usleep(10000);
        }
        if (!joined) {
            /* 最后尝试 join, 对已 detach 的线程会返回 EINVAL, 忽略 */
            int ret = pthread_join(ctx->thread, NULL);
            if (ret != 0) {
                CSG_LOGW("LOCAL_PLAY", "join thread slot=%d failed: %d (may be detached)", slot, ret);
            }
        }
    }
    ctx->thread_joined = 1;

    if (ctx->video_extradata) { free(ctx->video_extradata); ctx->video_extradata = NULL; }
    if (ctx->audio_extradata) { free(ctx->audio_extradata); ctx->audio_extradata = NULL; }
    if (ctx->read_buf) { free(ctx->read_buf); ctx->read_buf = NULL; }
    if (ctx->audio_sample_buf) { free(ctx->audio_sample_buf); ctx->audio_sample_buf = NULL; }
    if (ctx->annexb_buf) { free(ctx->annexb_buf); ctx->annexb_buf = NULL; }
    if (ctx->mov) { mov_reader_destroy(ctx->mov); ctx->mov = NULL; }
    if (ctx->fp) { fclose(ctx->fp); ctx->fp = NULL; }
    pthread_mutex_destroy(&ctx->mutex);
    free(ctx);
    s_csg_local_playback_ctx_table[slot] = NULL;
    CSG_LOGI("LOCAL_PLAY", "Reclaimed ctx slot=%d (sync)", slot);
}

/* ==================== create ==================== */

int csg_local_playback_create(const char *file_path)
{
    int i, handle = -1;

    /* 先同步回收已停止/完成旧 ctx */
    for (i = 0; i < MAX_LOCAL_CTX_NUM; i++) {
        if (s_csg_local_playback_ctx_table[i]) {
            csg_local_playback_ctx_t *old = s_csg_local_playback_ctx_table[i];
            int is_stopped = !old->running || old->finished;
            if (is_stopped) {
                csg_local_playback_join_and_cleanup(i);
            }
        }
    }

    for (i = 0; i < MAX_LOCAL_CTX_NUM; i++) {
        if (s_csg_local_playback_ctx_table[i] == NULL) {
            handle = i;
            break;
        }
    }
    if (handle < 0) {
        CSG_LOGW("LOCAL_PLAY", "All slots occupied, force-reclaim slot 0");
        csg_local_playback_join_and_cleanup(0);
        for (i = 0; i < MAX_LOCAL_CTX_NUM; i++) {
            if (s_csg_local_playback_ctx_table[i] == NULL) { handle = i; break; }
        }
    }
    if (handle < 0) {
        CSG_LOGE("LOCAL_PLAY", "No free context slot");
        return -1;
    }

    csg_local_playback_ctx_t *ctx = calloc(1, sizeof(csg_local_playback_ctx_t));
    if (!ctx) return -1;

    strncpy(ctx->file_path, file_path, sizeof(ctx->file_path) - 1);
    ctx->mov_video_track = (uint32_t)-1;
    ctx->mov_audio_track = (uint32_t)-1;
    ctx->thread_joined = 0;
    ctx->speed = 1.0f;
    ctx->rap_scan_active = 1;
    ctx->nal_length_size = 4;
    pthread_mutex_init(&ctx->mutex, NULL);

    /* 打开文件 & libmov */
    ctx->fp = fopen(file_path, "rb");
    if (!ctx->fp) {
        CSG_LOGE("LOCAL_PLAY", "fopen failed: %s", file_path);
        goto fail;
    }
    ctx->mov_io.fp = ctx->fp;
    ctx->mov = mov_reader_create(&s_csg_local_playback_mov_buffer, &ctx->mov_io);
    if (!ctx->mov) {
        CSG_LOGE("LOCAL_PLAY", "mov_reader_create failed: %s", file_path);
        goto fail;
    }

    struct mov_reader_trackinfo_t ti;
    memset(&ti, 0, sizeof(ti));
    ti.onvideo = csg_local_playback_on_video_info;
    ti.onaudio = csg_local_playback_on_audio_info;
    mov_reader_getinfo(ctx->mov, &ti, ctx);

    if (ctx->mov_video_track == (uint32_t)-1) {
        CSG_LOGE("LOCAL_PLAY", "No video track in %s", file_path);
        goto fail;
    }

    /* 视频编码 & 分辨率 (onvideo 回调已填充 codec_type/width/height/extradata) */
    if (ctx->codec_type != CSG_CODEC_H264 && ctx->codec_type != CSG_CODEC_H265) {
        CSG_LOGE("LOCAL_PLAY", "Unsupported video codec (only H.264/H.265)");
        goto fail;
    }
    CSG_LOGI("LOCAL_PLAY", "Video track: %dx%d codec=%s",
         ctx->video_width, ctx->video_height,
         ctx->codec_type == CSG_CODEC_H265 ? "H265" : "H264");

    /* NALU 长度字段: h264: avcC[4]&3+1 ; h265: hvcC[21]&3+1 */
    if (ctx->video_extradata && ctx->video_extradata_size > 0) {
        if (ctx->codec_type == CSG_CODEC_H265) {
            ctx->nal_length_size = (ctx->video_extradata[21] & 0x03) + 1;
        } else {
            ctx->nal_length_size = (ctx->video_extradata[4] & 0x03) + 1;
        }
    }
    if (ctx->nal_length_size < 1 || ctx->nal_length_size > 4)
        ctx->nal_length_size = 4;

    /* 分辨率检查 */
    if (ctx->video_width > RK_MAX_WIDTH || ctx->video_height > RK_MAX_HEIGHT) {
        CSG_LOGE("LOCAL_PLAY", "Resolution %dx%d exceeds RK3572 max %dx%d",
             ctx->video_width, ctx->video_height, RK_MAX_WIDTH, RK_MAX_HEIGHT);
        goto fail;
    }
    if (ctx->video_width >= 3840 || ctx->video_height >= 2160) {
        ctx->downscale = 1;
        CSG_LOGW("LOCAL_PLAY", "4K video detected: %dx%d", ctx->video_width, ctx->video_height);
    }

    /* 音频轨道 (onaudio 回调已填充 codec_id/rate/channels/extradata) */
    if (ctx->mov_audio_track != (uint32_t)-1) {
        CSG_LOGI("LOCAL_PLAY", "Audio track: codec=%d %dHz/%dch asc=%d",
             ctx->audio_codec_id, ctx->audio_sample_rate, ctx->audio_channels,
             ctx->audio_extradata_size);
    }

    /* 时长 (mov_reader_getduration; 默认模式 create 已构建完整样本表, 对 fMP4 可靠) */
    ctx->mov_duration_ms = (int64_t)mov_reader_getduration(ctx->mov);
    if (ctx->mov_duration_ms > 0) {
        ctx->duration_us = ctx->mov_duration_ms * 1000;
        CSG_LOGI("LOCAL_PLAY", "Duration: %lld us (%.1f sec)",
             (long long)ctx->duration_us, (double)ctx->duration_us / 1000000.0);
    } else {
        ctx->duration_us = -1;
    }

    /* 预分配样本缓冲 */
    ctx->read_buf_size = CSG_LOCAL_READ_BUF_SIZE;
    ctx->read_buf = malloc(ctx->read_buf_size);
    if (!ctx->read_buf) goto fail;
    ctx->annexb_cap = CSG_LOCAL_READ_BUF_SIZE + 4096;
    ctx->annexb_buf = malloc(ctx->annexb_cap);
    if (!ctx->annexb_buf) goto fail;
    if (ctx->mov_audio_track != (uint32_t)-1) {
        ctx->audio_sample_buf_size = CSG_LOCAL_AUDIO_BUF_SIZE;
        ctx->audio_sample_buf = malloc(ctx->audio_sample_buf_size);
        if (!ctx->audio_sample_buf) goto fail;
    }

    s_csg_local_playback_ctx_table[handle] = ctx;
    CSG_LOGI("LOCAL_PLAY", "Created handle=%d, file=%s", handle, file_path);
    return handle;

fail:
    csg_local_playback_join_and_cleanup(handle);
    return -1;
}

void csg_local_playback_set_callback(int handle, csg_local_frame_callback cb,
                                     void *user_data)
{
    if (handle < 0 || handle >= MAX_LOCAL_CTX_NUM || !s_csg_local_playback_ctx_table[handle])
        return;
    csg_local_playback_ctx_t *ctx = s_csg_local_playback_ctx_table[handle];
    pthread_mutex_lock(&ctx->mutex);
    ctx->callback = cb;
    ctx->user_data = user_data;
    pthread_mutex_unlock(&ctx->mutex);
}

void csg_local_playback_set_event_callback(int handle, csg_local_event_callback cb,
                                           void *user_data)
{
    if (handle < 0 || handle >= MAX_LOCAL_CTX_NUM || !s_csg_local_playback_ctx_table[handle])
        return;
    csg_local_playback_ctx_t *ctx = s_csg_local_playback_ctx_table[handle];
    pthread_mutex_lock(&ctx->mutex);
    ctx->event_cb = cb;
    ctx->event_user_data = user_data;
    pthread_mutex_unlock(&ctx->mutex);
}

void csg_local_playback_set_audio_callback(int handle, csg_local_audio_frame_callback cb,
                                           void *user_data)
{
    if (handle < 0 || handle >= MAX_LOCAL_CTX_NUM || !s_csg_local_playback_ctx_table[handle])
        return;
    csg_local_playback_ctx_t *ctx = s_csg_local_playback_ctx_table[handle];
    pthread_mutex_lock(&ctx->mutex);
    ctx->audio_callback = cb;
    ctx->audio_user_data = user_data;
    pthread_mutex_unlock(&ctx->mutex);
}

int csg_local_playback_get_audio_info(int handle, int *codec_id,
                                  int *sample_rate, int *channels,
                                  uint64_t *channel_layout,
                                  const uint8_t **extradata,
                                  int *extradata_size)
{
    if (handle < 0 || handle >= MAX_LOCAL_CTX_NUM || !s_csg_local_playback_ctx_table[handle])
        return -1;
    csg_local_playback_ctx_t *ctx = s_csg_local_playback_ctx_table[handle];

    if (ctx->mov_audio_track == (uint32_t)-1 || ctx->audio_codec_id == CSG_AUDIO_CODEC_NONE)
        return -1;

    pthread_mutex_lock(&ctx->mutex);
    if (codec_id)       *codec_id       = ctx->audio_codec_id;
    if (sample_rate)    *sample_rate    = ctx->audio_sample_rate;
    if (channels)       *channels       = ctx->audio_channels;
    if (channel_layout) *channel_layout = 0;
    if (extradata)      *extradata      = ctx->audio_extradata;
    if (extradata_size) *extradata_size = ctx->audio_extradata_size;
    pthread_mutex_unlock(&ctx->mutex);
    return 0;
}

/* ==================== 控制 ==================== */

int csg_local_playback_start(int handle)
{
    if (handle < 0 || handle >= MAX_LOCAL_CTX_NUM || !s_csg_local_playback_ctx_table[handle])
        return -1;
    csg_local_playback_ctx_t *ctx = s_csg_local_playback_ctx_table[handle];

    ctx->running = 1;
    ctx->paused = 0;
    ctx->finished = 0;
    ctx->current_position_us = 0;
    memset(&ctx->stats, 0, sizeof(ctx->stats));
    ctx->stats.start_time = csg_local_playback_now_us();

    return pthread_create(&ctx->thread, NULL, csg_local_playback_thread, ctx);
}

int csg_local_playback_pause(int handle)
{
    if (handle < 0 || handle >= MAX_LOCAL_CTX_NUM || !s_csg_local_playback_ctx_table[handle])
        return -1;
    csg_local_playback_ctx_t *ctx = s_csg_local_playback_ctx_table[handle];

    pthread_mutex_lock(&ctx->mutex);
    ctx->paused = 1;
    ctx->resume_target_pts = 0;
    pthread_mutex_unlock(&ctx->mutex);
    CSG_LOGI("LOCAL_PLAY", "Paused");
    return 0;
}

int csg_local_playback_resume(int handle, int64_t target_pts)
{
    if (handle < 0 || handle >= MAX_LOCAL_CTX_NUM || !s_csg_local_playback_ctx_table[handle])
        return -1;
    csg_local_playback_ctx_t *ctx = s_csg_local_playback_ctx_table[handle];

    pthread_mutex_lock(&ctx->mutex);
    ctx->paused = 0;
    if (target_pts > 0) {
        ctx->resume_target_pts = target_pts;
    }
    pthread_mutex_unlock(&ctx->mutex);
    if (target_pts > 0) {
        CSG_LOGI("LOCAL_PLAY", "Resumed (target_pts=%lld)", (long long)target_pts);
    } else {
        CSG_LOGI("LOCAL_PLAY", "Resumed (instant, no seek)");
    }
    return 0;
}

int csg_local_playback_stop(int handle)
{
    if (handle < 0 || handle >= MAX_LOCAL_CTX_NUM || !s_csg_local_playback_ctx_table[handle])
        return -1;
    csg_local_playback_ctx_t *ctx = s_csg_local_playback_ctx_table[handle];

    /* ★ v5.4 修复: 不再使用 pthread_detach.
     * 旧代码 pthread_detach 后 join_and_cleanup 无法 join 线程,
     * 导致 50 次 busy-loop 浪费 500ms 阻塞 LVGL 线程,
     * 且 detach 后线程可能仍在运行, join_and_cleanup 释放 ctx
     * 时存在 use-after-free 竞态风险.
     * 修复: 只设置 running=0, 线程自然退出,
     * join_and_cleanup 统一 join 回收. */
    ctx->running = 0;
    /* 不设置 thread_joined, 让 join_and_cleanup 负责 join */
    CSG_LOGI("LOCAL_PLAY", "Marked stop handle=%d (async)", handle);
    return 0;
}

int csg_local_playback_cleanup_sync(int handle)
{
    if (handle < 0 || handle >= MAX_LOCAL_CTX_NUM || !s_csg_local_playback_ctx_table[handle])
        return -1;
    csg_local_playback_join_and_cleanup(handle);
    return 0;
}

int csg_local_playback_get_video_info(int handle, int *width, int *height,
                                  int *fps, int *codec_type)
{
    if (handle < 0 || handle >= MAX_LOCAL_CTX_NUM || !s_csg_local_playback_ctx_table[handle])
        return -1;
    csg_local_playback_ctx_t *ctx = s_csg_local_playback_ctx_table[handle];
    if (width)  *width  = ctx->video_width;
    if (height) *height = ctx->video_height;
    if (fps)    *fps    = ctx->video_fps;
    if (codec_type) *codec_type = ctx->codec_type;
    return 0;
}

int64_t csg_local_playback_get_duration(int handle)
{
    if (handle < 0 || handle >= MAX_LOCAL_CTX_NUM || !s_csg_local_playback_ctx_table[handle])
        return -1;
    return s_csg_local_playback_ctx_table[handle]->duration_us;
}

int64_t csg_local_playback_get_last_seek_kf_pts(int handle)
{
    if (handle < 0 || handle >= MAX_LOCAL_CTX_NUM || !s_csg_local_playback_ctx_table[handle])
        return -1;
    csg_local_playback_ctx_t *ctx = s_csg_local_playback_ctx_table[handle];
    int64_t pts;
    pthread_mutex_lock(&ctx->mutex);
    pts = ctx->last_seek_kf_pts;
    pthread_mutex_unlock(&ctx->mutex);
    return pts;
}

int csg_local_playback_get_extradata(int handle, const uint8_t **data, int *size)
{
    if (handle < 0 || handle >= MAX_LOCAL_CTX_NUM || !s_csg_local_playback_ctx_table[handle])
        return -1;
    csg_local_playback_ctx_t *ctx = s_csg_local_playback_ctx_table[handle];
    if (!data || !size) return -1;

    pthread_mutex_lock(&ctx->mutex);
    *data = ctx->video_extradata;
    *size = ctx->video_extradata_size;
    pthread_mutex_unlock(&ctx->mutex);
    return (*data && *size > 0) ? 0 : -1;
}

int64_t csg_local_playback_get_current_position(int handle)
{
    if (handle < 0 || handle >= MAX_LOCAL_CTX_NUM || !s_csg_local_playback_ctx_table[handle])
        return 0;
    csg_local_playback_ctx_t *ctx = s_csg_local_playback_ctx_table[handle];
    pthread_mutex_lock(&ctx->mutex);
    int64_t pos = ctx->current_position_us;
    pthread_mutex_unlock(&ctx->mutex);
    return pos;
}

int csg_local_playback_seek(int handle, double position)
{
    if (handle < 0 || handle >= MAX_LOCAL_CTX_NUM || !s_csg_local_playback_ctx_table[handle])
        return -1;
    csg_local_playback_ctx_t *ctx = s_csg_local_playback_ctx_table[handle];
    if (position < 0.0) position = 0.0;
    if (position > 1.0) position = 1.0;

    pthread_mutex_lock(&ctx->mutex);
    ctx->seek_position = position;
    ctx->seek_pending = 1;
    ctx->paused = 0;
    CSG_LOGI("LOCAL_PLAY", "Seek request: pos=%.3f, paused=%d, running=%d, finished=%d",
         position, ctx->paused, ctx->running, ctx->finished);
    pthread_mutex_unlock(&ctx->mutex);
    return 0;
}

int csg_local_playback_is_active(int handle)
{
    if (handle < 0 || handle >= MAX_LOCAL_CTX_NUM || !s_csg_local_playback_ctx_table[handle])
        return 0;
    csg_local_playback_ctx_t *ctx = s_csg_local_playback_ctx_table[handle];
    return (ctx->running == 1) ? 1 : 0;
}

int csg_local_playback_get_downscale(int handle)
{
    if (handle < 0 || handle >= MAX_LOCAL_CTX_NUM || !s_csg_local_playback_ctx_table[handle])
        return 0;
    return s_csg_local_playback_ctx_table[handle]->downscale;
}

void csg_local_playback_get_stats(int handle, csg_player_stats_t *stats)
{
    if (handle < 0 || handle >= MAX_LOCAL_CTX_NUM || !s_csg_local_playback_ctx_table[handle] || !stats)
        return;
    memcpy(stats, &s_csg_local_playback_ctx_table[handle]->stats, sizeof(csg_player_stats_t));
}

int csg_local_playback_force_stop_thread(int handle)
{
    if (handle < 0 || handle >= MAX_LOCAL_CTX_NUM || !s_csg_local_playback_ctx_table[handle])
        return -1;
    s_csg_local_playback_ctx_table[handle]->running = 0;
    return 0;
}

int csg_local_playback_set_speed(int handle, float speed)
{
    if (handle < 0 || handle >= MAX_LOCAL_CTX_NUM || !s_csg_local_playback_ctx_table[handle])
        return -1;
    if (speed < 0.5f || speed > 16.0f)
        return -1;

    csg_local_playback_ctx_t *ctx = s_csg_local_playback_ctx_table[handle];
    pthread_mutex_lock(&ctx->mutex);
    ctx->speed = speed;
    ctx->rate_reset_pending = 1;
    pthread_mutex_unlock(&ctx->mutex);

    CSG_LOGI("LOCAL_PLAY", "Speed changed: %.2fx", speed);
    return 0;
}

void csg_local_playback_set_seek_feed_done_ptr(int handle, volatile int *ptr)
{
    if (handle < 0 || handle >= MAX_LOCAL_CTX_NUM || !s_csg_local_playback_ctx_table[handle])
        return;
    s_csg_local_playback_ctx_table[handle]->seek_fast_feed_done_ptr = ptr;
    CSG_LOGI("LOCAL_PLAY", "seek_fast_feed_done_ptr registered");
}