/* SPDX-License-Identifier: MIT */
/*
 * nvr_multi.c - NVR 多路 RTSP 拉流/解码/录制实现 (双码流分离)
 *
 * 需求 (用户明确):
 *   - 单路 (count==1): 只拉主码流, 同一句柄既解码显示又录制.
 *   - 多路 (count>1): 每路拉两条 RTSP —— 子码流(预览, MPP解码显示) +
 *                    主码流(录制, 编码流直写 mp4, 不解码).
 *   - 录制只录主码流, 不录子码流.
 *
 * 每通道两条独立流水线:
 *   预览链 preview_rtsp (子码流/单路主码流):
 *      帧回调 → MPP 硬件解码 (NV12) → RGA → LVGL 分屏窗口
 *   录制链 record_rtsp (主码流, 仅多路存在):
 *      帧回调 → (SPS/PPS 封进 MP4) 编码帧直写 mp4_record, 不解码
 *   单路: 无 record_rtsp, 预览句柄兼作录制源 (rec_src = preview_rtsp)
 *
 * ★ 线程模型 (决定 stop 安全顺序):
 *   MPP 为同步解码, 回调在同一 RTSP 解码线程执行完才返回. stop 时
 *   先停所有 RTSP 句柄 (join 内部解码线程), 之后所有 MPP/RGA/LVGL
 *   与录制推帧必然已退出, 再销毁解码器/录制器无 use-after-free.
 *
 * ★ 多路显示: csg_lvgl_display_set_grid(count) 分屏, LVGL 内部每窗口双缓冲.
 *
 * ★ 录制: 每路 /userdata/records/multi/chn<N>_时间戳.mp4, 主码流编码流
 *   直接 remux, 不经软编码. 音频仅录制不播放 (NVR 预览标准行为).
 */

#include "nvr_multi.h"
#include "mpp_decode.h"
#include "rga_convert.h"
#include "lvgl_display.h"
#include "mp4_record.h"
#include "im2d.h"            /* RK_FORMAT_BGRA_8888 */

#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <time.h>
#include <sys/stat.h>
#include <errno.h>
#include <unistd.h>

#define NVR_MULTI_TAG "NVR_MULTI"

/* ==================== 每通道上下文 ==================== */
typedef struct {
    int  chn;
    int  single;             /* 1=单路: preview_rtsp 兼录制源 */
    /* 拉流 (双句柄) */
    void *preview_rtsp;      /* 预览句柄 (多路子码流 / 单路主码流) */
    void *record_rtsp;       /* 录制句柄 (主码流; 单路=NULL) */
    void *rec_src;           /* 录制源句柄 = single?preview_rtsp:record_rtsp */
    char  url_cache[512];    /* 预览 URL (状态查询) */
    /* 解码显示 */
    void *decoder;
    void *rgb_buf;
    int   rgb_size;
    int   disp_w, disp_h;
    int   vdec_ready;        /* 1=MPP 解码器已建 */
    /* 录制 */
    void *record;
    int   rec_started;
    int   rec_audio_cfg;
    int   rec_ready;         /* 1=录制器已建 */
    char  record_path[512];
    /* 参数集 */
    uint8_t *vdec_edata;  int vdec_edata_len;  /* 预览 MPP 用 (Annex-B) */
    uint8_t *rec_edata;   int rec_edata_len;   /* 录制 mp4 用   (Annex-B) */
    int   codec, width, height;
    uint64_t video_frames;
    uint64_t rec_frames;
    uint64_t audio_frames;
} nvr_multi_chn_t;

/* 多路管理器 (全局单例) */
typedef struct {
    nvr_multi_chn_t chn[NVR_MULTI_MAX_CHN];
    int  chn_cnt;
    volatile int running;
    pthread_mutex_t mutex;
} nvr_multi_ctx_t;

static nvr_multi_ctx_t s_nvr_multi_ctx;

/* ==================== 自动码流识别/切换 ====================
 * 用户只需提供一个 URL, 程序自动配对同一路画面的主/子码流.
 * 你的模板:
 *   主码流 rtsp://admin:123456@192.168.9.188/ch01/main/live
 *   子码流 rtsp://admin:123456@192.168.9.188/ch02/sub/live
 * → 主/子码流的 通道号不同: main→sub 时 ch+1, sub→main 时 ch-1.
 *   例: main ch01 → sub ch02;  sub ch02 → main ch01.
 *   多路时各通道各自 pair (如 ch01 main↔ch02 sub, ch03 main↔ch04 sub).
 * 若 URL 非 /ch<NN>/main|sub 结构, 回退纯 main↔sub 标记替换, 尽量不破坏. */

/* 纯 main↔sub 标记互换 (保底, URL 无 /ch<NN> 格式时) */
static void nvr_multi_swap_marker(const char *src, char *dst, size_t n)
{
    const char *p = strstr(src, "main");
    const char *q = strstr(src, "sub");
    if (!p && !q) { snprintf(dst, n, "%s", src); return; }
    if (p && q) { /* 取更靠前的标记 */
        if (q < p) p = NULL;
        else q = NULL;
    }
    const char *hit = p ? p : q;
    const char *repl = p ? "sub" : "main";
    int hit_len = p ? 4 : 3;
    size_t off = (size_t)(hit - src);
    snprintf(dst, n, "%.*s%s%s", (int)off, src, repl, hit + hit_len);
}

/* 按 /ch<NN>/main|sub 结构切到目标码流:
 *   want_main=1: main 保持, sub→main (通道 -1)
 *   want_main=0: main→sub (通道 +1), sub 保持
 * 识别不了则回退纯标记替换. */
static void nvr_multi_switch_chn(const char *src, int want_main,
                                 char *dst, size_t n)
{
    const char *ch = strstr(src, "/ch");
    if (!ch) { nvr_multi_swap_marker(src, dst, n); return; }

    const char *num = ch + 3;          /* 跳过 "/ch" */
    int N = 0;
    const char *q = num;
    while (*q >= '0' && *q <= '9') { N = N * 10 + (*q - '0'); q++; }
    if (q == num) { nvr_multi_swap_marker(src, dst, n); return; }

    int is_main = (strncmp(q, "/main", 5) == 0);
    int is_sub  = (strncmp(q, "/sub", 4) == 0);
    if (!is_main && !is_sub) { nvr_multi_swap_marker(src, dst, n); return; }

    int newN;
    const char *mk;
    if (is_main) {
        if (want_main) { newN = N;            mk = "main"; }
        else           { newN = N + 1;        mk = "sub"; }
    } else {
        if (want_main) { newN = N - 1; mk = "main"; if (newN < 1) newN = 1; }
        else           { newN = N;            mk = "sub"; }
    }
    int ml = (mk[0] == 'm') ? 5 : 4;
    snprintf(dst, n, "%.*s/ch%d/%s%s", (int)(ch - src), src,
             newN, mk, q + ml);
}

/* 确保结果是"标准子码流" (main→sub 通道+1) */
static void nvr_multi_url_to_sub(const char *src, char *dst, size_t n)
{
    if (!src) { if (n) dst[0] = 0; return; }
    nvr_multi_switch_chn(src, 0, dst, n);
}

/* 确保结果是"标准主码流" (sub→main 通道-1) */
static void nvr_multi_url_to_main(const char *src, char *dst, size_t n)
{
    if (!src) { if (n) dst[0] = 0; return; }
    nvr_multi_switch_chn(src, 1, dst, n);
}

/* ==================== 录制文件路径 ==================== */
static void nvr_multi_make_record_path(nvr_multi_chn_t *c)
{
    if (mkdir(NVR_MULTI_RECORD_DIR, 0755) != 0 && errno != EEXIST) {
        CSG_LOGW(NVR_MULTI_TAG, "mkdir %s failed: %s",
                 NVR_MULTI_RECORD_DIR, strerror(errno));
    }

    time_t t = time(NULL);
    struct tm tm;
    localtime_r(&t, &tm);
    snprintf(c->record_path, sizeof(c->record_path),
             "%s/chn%d_%04d%02d%02d_%02d%02d%02d.mp4",
             NVR_MULTI_RECORD_DIR, c->chn,
             tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
             tm.tm_hour, tm.tm_min, tm.tm_sec);
}

/* 从某 RTSP 句柄取 SDP, 组装 Annex-B 参数集 [VPS] SPS PPS.
 * 返回 malloc 缓冲 (由调用方 free). 失败返回 NULL. */
static uint8_t *nvr_multi_build_annexb(void *rtsp, int codec,
                                       int *out_len)
{
    if (!rtsp) return NULL;
    int w = 0, h = 0, sl = 0, pl = 0, vl = 0;
    const uint8_t *sps = NULL, *pps = NULL, *vps = NULL;
    *out_len = 0;
    if (csg_rtsp_live555_get_raw_info(rtsp, &codec, &w, &h,
                                      &sps, &sl, &pps, &pl, &vps, &vl) != 0
        || w <= 0 || h <= 0 || sl <= 0 || pl <= 0) {
        return NULL;
    }

    uint8_t edata[512];
    int ed = 0;
    const uint8_t st[4] = { 0, 0, 0, 1 };
    if (codec == CSG_CODEC_H265 && vl > 0 && ed + 4 + vl <= (int)sizeof(edata)) {
        memcpy(edata + ed, st, 4); ed += 4;
        memcpy(edata + ed, vps, (size_t)vl); ed += vl;
    }
    if (sl > 0 && ed + 4 + sl <= (int)sizeof(edata)) {
        memcpy(edata + ed, st, 4); ed += 4;
        memcpy(edata + ed, sps, (size_t)sl); ed += sl;
    }
    if (pl > 0 && ed + 4 + pl <= (int)sizeof(edata)) {
        memcpy(edata + ed, st, 4); ed += 4;
        memcpy(edata + ed, pps, (size_t)pl); ed += pl;
    }
    if (ed == 0) return NULL;
    uint8_t *buf = malloc((size_t)ed);
    if (!buf) return NULL;
    memcpy(buf, edata, (size_t)ed);
    *out_len = ed;
    return buf;
}

/* 从某句柄取视频宽高+编码 */
static int nvr_multi_get_vres(void *rtsp, int *codec, int *w, int *h)
{
    if (!rtsp) return -1;
    int sl = 0, pl = 0, vl = 0;
    const uint8_t *sps = NULL, *pps = NULL, *vps = NULL;
    int c = 0, vw = 0, vh = 0;
    if (csg_rtsp_live555_get_raw_info(rtsp, &c, &vw, &vh,
                                      &sps, &sl, &pps, &pl, &vps, &vl) != 0)
        return -1;
    if (vw <= 0 || vh <= 0) return -1;
    *codec = c; *w = vw; *h = vh;
    return 0;
}

/* ==================== MPP 解码输出回调 (预览链, RTSP 解码线程) ====================
 * NV12 → RGA 缩放/转RGB → 对应分屏窗口. RGA 由内部全局锁串行化. */
static void nvr_multi_on_yuv(void *yuv_data, int yuv_len, int width, int height,
                             int hor_stride, int ver_stride, uint64_t pts,
                             int is_keyframe, void *user_data)
{
    nvr_multi_chn_t *c = (nvr_multi_chn_t *)user_data;
    (void)yuv_len; (void)pts; (void)is_keyframe;

    if (!s_nvr_multi_ctx.running || !c) return;

    if (c->disp_w <= 0 || c->disp_h <= 0) {
        int dw = 0, dh = 0;
        csg_lvgl_display_get_chn_size(c->chn, &dw, &dh);
        c->disp_w = dw & ~1;
        c->disp_h = dh & ~1;
        if (c->disp_w < 2) c->disp_w = 2;
        if (c->disp_h < 2) c->disp_h = 2;
    }

    int size = c->disp_w * c->disp_h * 4;
    if (c->rgb_size < size) {
        void *nb = realloc(c->rgb_buf, (size_t)size);
        if (!nb) return;
        c->rgb_buf = nb;
        c->rgb_size = size;
    }
    if (!c->rgb_buf) return;

    if (csg_rga_convert_nv12_to_rgb_scaled(yuv_data, width, height,
                                           hor_stride, ver_stride,
                                           c->rgb_buf, c->disp_w, c->disp_h,
                                           RK_FORMAT_BGRA_8888) == 0) {
        csg_lvgl_display_update_video_chn(c->chn, c->rgb_buf,
                                          c->disp_w, c->disp_h);
    }
}

/* ==================== 录制链: 确保录制器已建 ====================
 * 从 rec_src (主码流) 的 SDP 创建 mp4_record (含 SPS/PPS 封装 + 音频轨). */
static void nvr_multi_rec_ensure(nvr_multi_chn_t *c)
{
    if (c->rec_ready || !c->rec_src || !s_nvr_multi_ctx.running) return;

    int codec = 0, w = 0, h = 0;
    if (nvr_multi_get_vres(c->rec_src, &codec, &w, &h) != 0) return;

    c->codec = codec;
    c->width = w;
    c->height = h;

    c->rec_edata = nvr_multi_build_annexb(c->rec_src, codec, &c->rec_edata_len);
    nvr_multi_make_record_path(c);
    c->record = mp4_record_create(c->record_path, codec, w, h, 25,
                                  c->rec_edata, c->rec_edata_len);
    c->rec_ready = 1;   /* 无论创建成败, 标记已尝试过, 避免每帧重试 */
    if (!c->record) {
        CSG_LOGE(NVR_MULTI_TAG, "chn%d MP4 create failed: %s", c->chn, c->record_path);
        return;
    }

    /* 音频轨 (主码流) */
    int acodec = 0, arate = 0, ach = 0, sdp_done = 0;
    csg_rtsp_live555_get_audio_info(c->rec_src, &acodec, &arate, &ach, &sdp_done);
    if (acodec != CSG_AUDIO_CODEC_NONE) {
        if (acodec == CSG_AUDIO_CODEC_AAC) {
            const uint8_t *asc = NULL;
            int asc_size = 0;
            csg_rtsp_live555_get_audio_asc(c->rec_src, &asc, &asc_size);
            if (asc_size > 0)
                mp4_record_set_audio_asc(c->record, asc, asc_size);
        }
        mp4_record_config_audio(c->record, acodec, arate, ach);
        c->rec_audio_cfg = 1;
    }
    if (mp4_record_start(c->record) == 0) {
        c->rec_started = 1;
        CSG_LOGI(NVR_MULTI_TAG, "chn%d recording: %s (%s %dx%d)",
                 c->chn, c->record_path,
                 codec == CSG_CODEC_H265 ? "H265" : "H264", w, h);
    } else {
        mp4_record_destroy(c->record);
        c->record = NULL;
        CSG_LOGE(NVR_MULTI_TAG, "chn%d MP4 start failed", c->chn);
    }
}

static void nvr_multi_rec_push(nvr_multi_chn_t *c, void *data, int len,
                               int is_keyframe, uint64_t pts)
{
    if (!c->record || !c->rec_started) return;
    if (mp4_record_push_frame(c->record, data, len, is_keyframe, pts) == 0)
        c->rec_frames++;
}

/* ==================== 录制链音频 (主码流) ==================== */
static void nvr_multi_rec_audio(nvr_multi_chn_t *c, const uint8_t *data, int len,
                                int codec_id, int sample_rate, int channels,
                                uint64_t pts)
{
    if (!c->record || !c->rec_started) return;
    c->audio_frames++;

    if (!c->rec_audio_cfg) {
        if (codec_id == CSG_AUDIO_CODEC_AAC) {
            const uint8_t *asc = NULL;
            int asc_size = 0;
            if (c->rec_src)
                csg_rtsp_live555_get_audio_asc(c->rec_src, &asc, &asc_size);
            if (asc_size > 0)
                mp4_record_set_audio_asc(c->record, asc, asc_size);
        }
        mp4_record_config_audio(c->record, codec_id, sample_rate, channels);
        c->rec_audio_cfg = 1;
    }
    mp4_record_push_audio(c->record, data, len, pts);
}

/* ==================== 预览链视频回调 (live555 解码线程) ====================
 * 子码流/单路主码流: ①建+喂 MPP 解码显示 ②(单路) 同源编码帧直写录制. */
static void nvr_multi_preview_frame(void *data, int len, int is_keyframe,
                                    uint64_t pts, int codec_type, void *user_data)
{
    nvr_multi_chn_t *c = (nvr_multi_chn_t *)user_data;
    (void)codec_type;

    if (!s_nvr_multi_ctx.running || !c || !data || len <= 0) return;
    c->video_frames++;

    /* 预览 MPP: 从预览句柄 SDP 懒建 */
    if (!c->vdec_ready && c->preview_rtsp) {
        int codec = 0, w = 0, h = 0;
        if (nvr_multi_get_vres(c->preview_rtsp, &codec, &w, &h) == 0) {
            c->vdec_edata = nvr_multi_build_annexb(c->preview_rtsp, codec,
                                                   &c->vdec_edata_len);
            c->decoder = csg_mpp_decode_create((csg_player_codec_t)codec, w, h);
            if (c->decoder) {
                if (c->vdec_edata && c->vdec_edata_len > 0)
                    csg_mpp_decode_set_extra_data(c->decoder,
                                                  c->vdec_edata, c->vdec_edata_len);
                csg_mpp_decode_set_callback(c->decoder, nvr_multi_on_yuv, c);
            } else {
                CSG_LOGE(NVR_MULTI_TAG, "chn%d MPP create failed", c->chn);
            }
        }
        c->vdec_ready = 1;
    }

    /* 送解码 (同步: 内部 get_frame, 输出回调在此线程执行) */
    if (c->decoder) {
        csg_mpp_decode_send_frame(c->decoder, data, len, is_keyframe, pts);
    }

    /* 单路: 预览句柄即录制源, 同帧直写录制; 多路录制由 record_rtsp 独立回调. */
    if (!c->record_rtsp) {
        nvr_multi_rec_ensure(c);
        nvr_multi_rec_push(c, data, len, is_keyframe, pts);
    }
}

/* ==================== 录制链视频回调 (主码流) ====================
 * 仅 remux 编码帧 → mp4, 不解码 (高码率主码流不占 MPP/CPU). */
static void nvr_multi_record_frame(void *data, int len, int is_keyframe,
                                   uint64_t pts, int codec_type, void *user_data)
{
    nvr_multi_chn_t *c = (nvr_multi_chn_t *)user_data;
    (void)codec_type;

    if (!s_nvr_multi_ctx.running || !c || !data || len <= 0) return;

    nvr_multi_rec_ensure(c);
    nvr_multi_rec_push(c, data, len, is_keyframe, pts);
}

/* ==================== 预览链音频 (单路录制用) ==================== */
static void nvr_multi_preview_audio(const uint8_t *data, int len, int codec_id,
                                    int sample_rate, int channels, uint64_t pts,
                                    void *user_data)
{
    nvr_multi_chn_t *c = (nvr_multi_chn_t *)user_data;
    if (!s_nvr_multi_ctx.running || !c) return;
    if (!c->record_rtsp) {   /* 单路: 预览句柄兼录制, 收音频 */
        nvr_multi_rec_ensure(c);
        nvr_multi_rec_audio(c, data, len, codec_id, sample_rate, channels, pts);
    }
}

/* ==================== 录制链音频 ==================== */
static void nvr_multi_record_audio(const uint8_t *data, int len, int codec_id,
                                   int sample_rate, int channels, uint64_t pts,
                                   void *user_data)
{
    nvr_multi_chn_t *c = (nvr_multi_chn_t *)user_data;
    if (!s_nvr_multi_ctx.running || !c) return;
    nvr_multi_rec_audio(c, data, len, codec_id, sample_rate, channels, pts);
}

/* 释放单通道资源 (须在 running=0 且 RTSP 线程全部 join 后调用) */
static void nvr_multi_free_chn(nvr_multi_chn_t *c)
{
    if (c->record) {
        mp4_record_stop(c->record);
        mp4_record_destroy(c->record);
        c->record = NULL;
    }
    if (c->decoder) {
        csg_mpp_decode_stop(c->decoder);
        csg_mpp_decode_destroy(c->decoder);
        c->decoder = NULL;
    }
    free(c->rgb_buf);           c->rgb_buf = NULL;
    free(c->vdec_edata);        c->vdec_edata = NULL;
    free(c->rec_edata);         c->rec_edata = NULL;
    c->rgb_size = c->vdec_edata_len = c->rec_edata_len = 0;
    c->rec_started = c->rec_ready = c->vdec_ready = 0;
}

/* ==================== 公开接口 ==================== */

int nvr_multi_start(const nvr_multi_url_t *urls, int count)
{
    if (!urls || count < 1) return 0;
    if (count > NVR_MULTI_MAX_CHN) count = NVR_MULTI_MAX_CHN;
    int single = (count == 1);

    pthread_mutex_lock(&s_nvr_multi_ctx.mutex);
    if (s_nvr_multi_ctx.running) {
        pthread_mutex_unlock(&s_nvr_multi_ctx.mutex);
        nvr_multi_stop();
        pthread_mutex_lock(&s_nvr_multi_ctx.mutex);
    }

    s_nvr_multi_ctx.running = 1;
    s_nvr_multi_ctx.chn_cnt = count;

    csg_lvgl_display_reset_channels();
    csg_lvgl_display_set_grid(count);

    int started = 0;
    for (int i = 0; i < count; i++) {
        nvr_multi_chn_t *c = &s_nvr_multi_ctx.chn[i];
        memset(c, 0, sizeof(*c));
        c->chn = i;
        c->single = single;

        const char *pin = urls[i].preview;
        const char *rin = urls[i].record;
        if (!pin || pin[0] == '\0') {
            CSG_LOGW(NVR_MULTI_TAG, "chn%d: empty preview URL, skipped", i);
            continue;
        }

        /* ★ 自动码流切换 (用户只需给一个 URL):
         *   - 单路: 预览用主码流 (播放+录制同一主码流)
         *   - 多路: 预览自动切到子码流(省资源), 录制自动切到主码流(高清) */
        char purl[512], rurl[512];
        if (single) {
            nvr_multi_url_to_main(pin, purl, sizeof(purl));
            snprintf(rurl, sizeof(rurl), "%.511s", purl);  /* 录制回退同主码流 */
        } else {
            nvr_multi_url_to_sub(pin, purl, sizeof(purl));
            if (rin && rin[0])
                snprintf(rurl, sizeof(rurl), "%.511s", rin);
            else
                nvr_multi_url_to_main(pin, rurl, sizeof(rurl));  /* 自动主码流录制 */
        }
        snprintf(c->url_cache, sizeof(c->url_cache), "%.511s", purl);

        /* 预览句柄 (多路子码流 / 单路主码流) */
        c->preview_rtsp = csg_rtsp_live555_create(purl, CSG_CODEC_H264);
        if (!c->preview_rtsp) {
            CSG_LOGE(NVR_MULTI_TAG, "chn%d: preview RTSP create failed: %s", i, purl);
            continue;
        }
        csg_rtsp_live555_set_callback(c->preview_rtsp, nvr_multi_preview_frame, c);
        csg_rtsp_live555_set_audio_callback(c->preview_rtsp, nvr_multi_preview_audio, c);

        /* 录制句柄 (主码流): 多路必须; 单路仅当用户显式给异于预览的录制URL,
         * 否则复用预览句柄 (rec_src=preview), 避免同一 URL 双实例 */
        int open_rec = !single;
        if (single && rin && rin[0] && strcmp(rin, purl) != 0) open_rec = 1;
        if (open_rec) {
            c->record_rtsp = csg_rtsp_live555_create(rurl, CSG_CODEC_H264);
            if (!c->record_rtsp) {
                CSG_LOGE(NVR_MULTI_TAG, "chn%d: record RTSP create failed: %s",
                         i, rurl);
                csg_rtsp_live555_stop(c->preview_rtsp);
                c->preview_rtsp = NULL;
                continue;
            }
            csg_rtsp_live555_set_callback(c->record_rtsp, nvr_multi_record_frame, c);
            csg_rtsp_live555_set_audio_callback(c->record_rtsp, nvr_multi_record_audio, c);
        }
        c->rec_src = c->record_rtsp ? c->record_rtsp : c->preview_rtsp;

        if (csg_rtsp_live555_start(c->preview_rtsp) != 0) {
            CSG_LOGE(NVR_MULTI_TAG, "chn%d: preview RTSP start failed: %s", i, purl);
            if (c->record_rtsp) csg_rtsp_live555_stop(c->record_rtsp);
            csg_rtsp_live555_stop(c->preview_rtsp);
            c->preview_rtsp = c->record_rtsp = NULL;
            continue;
        }
        if (c->record_rtsp && csg_rtsp_live555_start(c->record_rtsp) != 0) {
            CSG_LOGE(NVR_MULTI_TAG, "chn%d: record RTSP start failed: %s", i, rurl);
            csg_rtsp_live555_stop(c->record_rtsp);
            c->record_rtsp = NULL;
            c->rec_src = c->preview_rtsp;
        }

        started++;
        CSG_LOGI(NVR_MULTI_TAG, "chn%d started: prev=%s%s", i, purl,
                 c->record_rtsp ? " | rec=" : " (single: prev==record)");
        if (c->record_rtsp) CSG_LOGI(NVR_MULTI_TAG, "chn%d rec=%s", i, rurl);
    }

    pthread_mutex_unlock(&s_nvr_multi_ctx.mutex);

    if (started == 0) {
        nvr_multi_stop();
        return 0;
    }
    CSG_LOGI(NVR_MULTI_TAG, "multi started: %d/%d channels (%s)",
             started, count, single ? "single-main" : "sub-preview + main-record");
    return started;
}

void nvr_multi_stop(void)
{
    pthread_mutex_lock(&s_nvr_multi_ctx.mutex);

    if (!s_nvr_multi_ctx.running && s_nvr_multi_ctx.chn_cnt == 0) {
        pthread_mutex_unlock(&s_nvr_multi_ctx.mutex);
        return;
    }

    s_nvr_multi_ctx.running = 0;
    CSG_LOGI(NVR_MULTI_TAG, "stopping %d channels...", s_nvr_multi_ctx.chn_cnt);

    /* ① 停所有 RTSP 句柄 (join 内部解码线程):
     * 此后 MPP/RGA/LVGL 与录制推帧必然已退出 (MPP 同步解码). */
    for (int i = 0; i < NVR_MULTI_MAX_CHN; i++) {
        nvr_multi_chn_t *c = &s_nvr_multi_ctx.chn[i];
        if (c->record_rtsp) csg_rtsp_live555_stop(c->record_rtsp);
        if (c->preview_rtsp) csg_rtsp_live555_stop(c->preview_rtsp);
        c->record_rtsp = c->preview_rtsp = c->rec_src = NULL;
    }

    /* ② 释放: 录制器(落盘 moov) → MPP → RGB/参数集 */
    for (int i = 0; i < NVR_MULTI_MAX_CHN; i++) {
        nvr_multi_chn_t *c = &s_nvr_multi_ctx.chn[i];
        if (c->rec_started && c->record) {
            CSG_LOGI(NVR_MULTI_TAG, "chn%d recording finished: %s (%llu frames)",
                     c->chn, c->record_path, (unsigned long long)c->rec_frames);
        }
        nvr_multi_free_chn(c);
    }

    s_nvr_multi_ctx.chn_cnt = 0;

    /* ③ 恢复单路显示 */
    csg_lvgl_display_reset_channels();
    csg_lvgl_display_set_grid(1);

    pthread_mutex_unlock(&s_nvr_multi_ctx.mutex);
    CSG_LOGI(NVR_MULTI_TAG, "multi stopped");
}

int nvr_multi_get_active(void)
{
    if (!s_nvr_multi_ctx.running) return 0;
    int n = 0;
    for (int i = 0; i < NVR_MULTI_MAX_CHN; i++) {
        if (s_nvr_multi_ctx.chn[i].preview_rtsp) n++;
    }
    return n;
}

int nvr_multi_get_info(int chn, nvr_multi_chn_info_t *info)
{
    if (!info || chn < 0 || chn >= NVR_MULTI_MAX_CHN) return -1;
    if (!s_nvr_multi_ctx.running) return -1;

    pthread_mutex_lock(&s_nvr_multi_ctx.mutex);
    nvr_multi_chn_t *c = &s_nvr_multi_ctx.chn[chn];
    if (!c->preview_rtsp) {
        pthread_mutex_unlock(&s_nvr_multi_ctx.mutex);
        return -1;
    }
    memset(info, 0, sizeof(*info));
    info->active = 1;
    info->connected = (c->video_frames > 0);
    info->decoded = (c->decoder != NULL);
    info->recording = (c->rec_started && c->record != NULL);
    info->codec = c->codec;
    info->width = c->width;
    info->height = c->height;
    info->video_frames = c->video_frames;
    info->rec_frames = c->rec_frames;
    strncpy(info->url, c->url_cache, sizeof(info->url) - 1);
    strncpy(info->record_path, c->record_path, sizeof(info->record_path) - 1);
    pthread_mutex_unlock(&s_nvr_multi_ctx.mutex);
    return 0;
}