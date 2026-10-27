/* SPDX-License-Identifier: MIT */
/*
 * rtsp_live555.cpp - 基于 live555 的 RTSP 拉流模块
 *
 * 参考: /home/dell/rk3572_player-拉流-播放-录制/src/network/rtsp_live555.cpp
 *       /home/dell/vlc/modules/access/live555.cpp
 *       /home/dell/live555-mirror-main/testProgs/testRTSPClient.cpp
 *
 * 用 live555 替代 ffmpeg 3.4 的 RTSP demuxer。
 * live555 事件循环跑在独立线程, H264/H265 RTP 经 H26xVideoStreamFramer
 * 解封装成 Annex-B 完整访问单元, 通过回调喂给 MPP 解码器 / MP4v2 录制器。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <atomic>

#include "rtsp_live555.h"
#include "common.h"

#include "liveMedia.hh"
#include "BasicUsageEnvironment.hh"

#define RTSP_LIVE555_BUFFER_SIZE (2 * 1024 * 1024)

/* ==================== live555 多实例初始化互斥锁 ====================
 * ★ 历史踩坑: 播放器同时运行两个 live555 实例 (主拉流预览 + 独立录像流),
 *   两个实例几乎同时首次 createNew 时, live555 库的静态/全局对象
 *   (如 ourIPAddress、Socket 静态表) 首次初始化非线程安全, 并发触发
 *   "munmap_chunk(): invalid pointer" 堆损坏崩溃.
 * 修复: 全局互斥锁串行化 create/start, 保证同一时刻只有一个实例在做
 *       live555 初始化/建连; 各实例的 TaskScheduler/UsageEnvironment
 *       相互独立, 事件循环线程后续并发是安全的. */
static pthread_mutex_t s_rtsp_init_mutex = PTHREAD_MUTEX_INITIALIZER;

typedef struct CSG_RTSP_LIVE555_CTX CSG_RTSP_LIVE555_CTX;

/* ==================== C++ 内部类 ==================== */

class StreamClientState {
public:
    StreamClientState(CSG_RTSP_LIVE555_CTX *c);
    virtual ~StreamClientState();
    MediaSession *session;
    MediaSubsessionIterator *iter;
    MediaSubsession *subsession;
    CSG_RTSP_LIVE555_CTX *c;
};

class ourRTSPClient : public RTSPClient {
public:
    static ourRTSPClient *createNew(UsageEnvironment &env, char const *rtspURL,
                                    CSG_RTSP_LIVE555_CTX *c);
    StreamClientState scs;
protected:
    ourRTSPClient(UsageEnvironment &env, char const *rtspURL, CSG_RTSP_LIVE555_CTX *c);
    virtual ~ourRTSPClient();
};

class DummySink : public MediaSink {
public:
    static DummySink *createNew(UsageEnvironment &env, CSG_RTSP_LIVE555_CTX *c);
protected:
    DummySink(UsageEnvironment &env, CSG_RTSP_LIVE555_CTX *c);
    virtual ~DummySink();
    virtual Boolean continuePlaying();
    static void afterGettingFrame(void *clientData, unsigned frameSize,
                                  unsigned numTruncatedBytes,
                                  struct timeval presentationTime,
                                  unsigned durationInMicroseconds);
    void afterGettingFrame2(unsigned frameSize, unsigned numTruncatedBytes,
                            struct timeval presentationTime);
private:
    /* H265 RTP → Annex-B 组包 (绕开 live555 framer) */
    void appendNAL(const u_int8_t *nal, unsigned size);
    void deliverAU(int64_t pts);

    CSG_RTSP_LIVE555_CTX *c;
    u_int8_t *receiveBuffer;

    /* 组包缓冲: 累积当前 access unit (start code + NAL 序列) */
    u_int8_t *au_buf;
    unsigned  au_size;
    unsigned  au_capacity;
    int       have_vcl;        /* 当前 access unit 已含 VCL NAL (帧边界标志) */
    int64_t   au_ts;           /* 当前 access unit 的相对时间戳 (us, 从首帧=0 开始) */
    int       sniffed;         /* 首个 RTP payload 已打印 (诊断) */

    /* PTS 归一化: 记录首帧的壁钟时间戳, 后续帧以此为零点计算相对时间 */
    int64_t   m_first_presentation_us;  /* 首帧壁钟时间 (us), -1 = 未设置 */
};

/* ==================== 音频 sink (PCMU/PCMA G.711 原始字节透传) ==================== */

class AudioDummySink : public MediaSink {
public:
    static AudioDummySink *createNew(UsageEnvironment &env, CSG_RTSP_LIVE555_CTX *c);
protected:
    AudioDummySink(UsageEnvironment &env, CSG_RTSP_LIVE555_CTX *c);
    virtual ~AudioDummySink();
    virtual Boolean continuePlaying();
    static void afterGettingFrame(void *clientData, unsigned frameSize,
                                  unsigned numTruncatedBytes,
                                  struct timeval presentationTime,
                                  unsigned durationInMicroseconds);
    void afterGettingFrame2(unsigned frameSize, struct timeval presentationTime);
private:
    CSG_RTSP_LIVE555_CTX *c;
    u_int8_t *receiveBuffer;
    int64_t   m_first_audio_us;   /* 首帧壁钟时间 (us), -1=未设置 */
    /* ★ v5.6: 无音频超时检测.
     *   摄像头在子码流上可能不发送音频 RTP, 5 秒后仍未收到首帧
     *   则打印警告, 提示用户切换主码流. */
    TaskToken m_no_audio_task;
    static void noAudioTimeout(void *clientData);
};

/* ==================== C 结构体 ==================== */

/* 队列帧 */
typedef struct {
    u_int8_t *data;
    unsigned  size;
    int       key;
    uint64_t  pts;
} RTSP_QUEUED_FRAME;

#define RTSP_FRAME_QUEUE_MAX 16

struct CSG_RTSP_LIVE555_CTX {
    TaskScheduler *scheduler;
    UsageEnvironment *env;
    ourRTSPClient *client;
    Authenticator *auth;

    pthread_t thread;
    std::atomic<char> running;   /* 事件循环线程标志, 停止时置 0 */

    csg_rtsp_frame_callback cb;
    void *user_data;

    /* ==================== 音频 (PCMU/PCMA G.711 / AAC) ==================== */
    csg_rtsp_audio_callback audio_cb;   /* 未设置则不拉音频 */
    void *audio_user_data;
    int  audio_found;                   /* SDP 中发现音频 subsession */
    int  audio_setup_done;              /* 音频 subsession 已完成 SETUP+sink */
    int  sdp_done;                      /* 所有 subsession SETUP 完成, 已发 PLAY:
                                         * 此时 audio_codec==NONE 可判定流无音频 */
    int  audio_codec;                   /* CSG_AUDIO_CODEC_G711U / G711A / AAC */
    int  audio_sample_rate;             /* 采样率 (rtpmap 时钟率) */
    int  audio_channels;                /* 声道数 */
    unsigned long long audio_frame_count;
    /* AAC 专属: AudioSpecificConfig (base64 解码后, FAAD2 初始化必需) */
    u_int8_t audio_asc[64];
    int      audio_asc_size;

    int codec;               /* 实际编码 (DESCRIBE 后确定) */
    int video_w, video_h, video_fps;

    unsigned long long frame_count;   /* 已交付到回调的帧数 (诊断用) */

    /* SDP sprop 参数集 (base64 解码后), 用于自定义组包时注入首帧 */
    u_int8_t sprop_vps[128]; unsigned sprop_vps_size;
    u_int8_t sprop_sps[128]; unsigned sprop_sps_size;
    u_int8_t sprop_pps[128]; unsigned sprop_pps_size;

    /* ==================== 解码队列 ====================
     * 事件循环线程只负责收 RTP + 组包, 把帧压入队列快速返回;
     * MPP 解码等耗时操作由独立解码线程执行, 避免阻塞 RTP 接收 */
    pthread_mutex_t q_mutex;
    pthread_cond_t  q_cond;      /* 有新帧时唤醒解码线程 */
    int             q_stop;      /* 停止标志 */
    pthread_t       decode_thread;
    RTSP_QUEUED_FRAME q_frames[RTSP_FRAME_QUEUE_MAX];
    int q_head, q_tail, q_count;
};

/* ==================== 音频 sink 实现 ====================
 * ★ 必须放在 CSG_RTSP_LIVE555_CTX 结构体定义之后: 成员函数访问
 *   c->audio_cb 等字段, 前向声明下无法解引用不完整类型 (编译错误). */

AudioDummySink *AudioDummySink::createNew(UsageEnvironment &env, CSG_RTSP_LIVE555_CTX *c)
{
    return new AudioDummySink(env, c);
}

AudioDummySink::AudioDummySink(UsageEnvironment &env, CSG_RTSP_LIVE555_CTX *c)
    : MediaSink(env), c(c)
{
    receiveBuffer = new u_int8_t[4096];
    m_first_audio_us = -1;
    m_no_audio_task = NULL;
}

AudioDummySink::~AudioDummySink()
{
    /* 取消未触发的超时任务 */
    if (m_no_audio_task) {
        envir().taskScheduler().unscheduleDelayedTask(m_no_audio_task);
        m_no_audio_task = NULL;
    }
    delete[] receiveBuffer;
}

Boolean AudioDummySink::continuePlaying()
{
    if (fSource == NULL) return False;
    fSource->getNextFrame(receiveBuffer, 4096,
                          afterGettingFrame, this, onSourceClosure, this);
    /* ★ v5.6: 启动 5 秒超时检测.
     *   若摄像头在子码流上不发送音频 RTP, 5 秒后打印警告. */
    if (m_no_audio_task == NULL) {
        m_no_audio_task = envir().taskScheduler().scheduleDelayedTask(
            5000000, noAudioTimeout, this);
    }
    return True;
}

void AudioDummySink::afterGettingFrame(void *clientData, unsigned frameSize,
                                       unsigned numTruncatedBytes,
                                       struct timeval presentationTime,
                                       unsigned /*durationInMicroseconds*/)
{
    AudioDummySink *sink = (AudioDummySink *)clientData;
    sink->afterGettingFrame2(frameSize, presentationTime);
    (void)numTruncatedBytes;
}

/* G.711 每采样 1 字节 / AAC raw access unit, 原始字节直接透传 (不解码), 推给上层回调 */
void AudioDummySink::afterGettingFrame2(unsigned frameSize,
                                        struct timeval presentationTime)
{
    /* ★ v5.6: 收到首帧后取消超时任务 */
    if (m_no_audio_task) {
        envir().taskScheduler().unscheduleDelayedTask(m_no_audio_task);
        m_no_audio_task = NULL;
    }

    if (c && c->audio_cb && frameSize > 0) {
        int64_t wall_us = (int64_t)presentationTime.tv_sec * 1000000 +
                          (int64_t)presentationTime.tv_usec;
        if (m_first_audio_us < 0) m_first_audio_us = wall_us;
        int64_t pts = wall_us - m_first_audio_us;
        if (pts < 0) pts = 0;

        int codec_id = c->audio_codec;   /* G711A/G711U/AAC 由 DESCRIBE 阶段确定 */
        c->audio_frame_count++;
        if (c->audio_frame_count == 1) {
            CSG_LOGI("RTSP", "First audio frame: size=%u codec=%d (stream has audio)",
                 frameSize, codec_id);
        }
        c->audio_cb(receiveBuffer, (int)frameSize, codec_id,
                    c->audio_sample_rate, c->audio_channels,
                    (uint64_t)pts, c->audio_user_data);
    }
    continuePlaying();
}

/* ★ v5.6: 音频超时回调.
 *   AudioDummySink 创建 5 秒后仍未收到首帧音频数据,
 *   说明摄像头在子码流上不发送音频 RTP 包.
 *   打印警告提示用户切换主码流. */
void AudioDummySink::noAudioTimeout(void *clientData)
{
    AudioDummySink *sink = (AudioDummySink *)clientData;
    sink->m_no_audio_task = NULL;  /* 任务已触发, 清除令牌 */

    if (sink->c && sink->c->audio_found && sink->c->audio_frame_count == 0) {
        CSG_LOGW("RTSP", "Audio sink: no audio frames received after 5 seconds! "
             "The camera may not send audio on this sub-stream. "
             "Try using main stream (/ch01/main/) for recording with audio.");
    }
}

/* ==================== 工具函数 ==================== */

/* 从 url 拆出 user/pass 和去掉认证的干净 URL */
static void parse_url_auth(const char *url, char *clean, int clean_size,
                           char *user, int user_size, char *pass, int pass_size)
{
    user[0] = pass[0] = 0;
    if (strncmp(url, "rtsp://", 7) == 0) {
        const char *rest = url + 7;
        const char *at = strchr(rest, '@');
        const char *slash = strchr(rest, '/');
        if (at && (!slash || at < slash)) {
            const char *colon = (const char *)memchr(rest, ':', (size_t)(at - rest));
            size_t ulen = (colon ? (size_t)(colon - rest) : (size_t)(at - rest));
            if (ulen > 0 && ulen < (size_t)user_size) {
                memcpy(user, rest, ulen);
                user[ulen] = 0;
            }
            if (colon) {
                size_t plen = (size_t)(at - colon - 1);
                if (plen < (size_t)pass_size) {
                    memcpy(pass, colon + 1, plen);
                    pass[plen] = 0;
                }
            }
            snprintf(clean, clean_size, "rtsp://%s", at + 1);
            return;
        }
    }
    snprintf(clean, clean_size, "%s", url);
}

/* ==================== base64 解码 (SDP sprop 参数集) ==================== */

static int base64_decode_char(char c)
{
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
}

/* 解码 base64 字符串到 out (容量 out_size), 返回字节数 */
static int base64_decode(const char *in, unsigned char *out, int out_size)
{
    int o = 0, val = 0, bits = 0;
    for (const char *p = in; *p && o < out_size; ++p) {
        if (*p == '=') break;
        int d = base64_decode_char(*p);
        if (d < 0) continue;
        val = (val << 6) | d;
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            out[o++] = (unsigned char)((val >> bits) & 0xFF);
        }
    }
    return o;
}

/* 检测访问单元是否含 IDR (H264 type 5; H265 type 19/20) */
static int detect_keyframe(const uint8_t *buf, int len, int codec)
{
    int i;
    for (i = 0; i < len - 4; i++) {
        if (buf[i] == 0 && buf[i + 1] == 0 && buf[i + 2] == 0 && buf[i + 3] == 1) {
            int nal = buf[i + 4];
            if (codec == CSG_CODEC_H264) {
                if ((nal & 0x1F) == 5) return 1;   /* IDR */
            } else {
                int t = (nal >> 1) & 0x3F;
                if (t == 19 || t == 20) return 1;  /* IDR_W_RADL / IDR_N_LP */
            }
        }
    }
    return 0;
}

/* ==================== 回调 (live555 事件循环线程) ==================== */

static void continueAfterDESCRIBE(RTSPClient *rtspClient, int resultCode, char *resultString);
static void setupNextSubsession(RTSPClient *rtspClient);
static void continueAfterSETUP(RTSPClient *rtspClient, int resultCode, char *resultString);
static void continueAfterPLAY(RTSPClient *rtspClient, int resultCode, char *resultString);

static void continueAfterDESCRIBE(RTSPClient *rtspClient, int resultCode, char *resultString)
{
    UsageEnvironment &env = rtspClient->envir();
    ourRTSPClient *client = (ourRTSPClient *)rtspClient;
    CSG_RTSP_LIVE555_CTX *c = client->scs.c;
    StreamClientState &scs = client->scs;

    if (resultCode != 0) {
        CSG_LOGW("RTSP", "DESCRIBE failed: %d %s", resultCode,
             resultString ? resultString : "");
        delete[] resultString;
        c->running = 1;
        return;
    }

    char *const sdp = resultString;
    scs.session = MediaSession::createNew(env, sdp);
    delete[] sdp;
    if (scs.session == NULL || !scs.session->hasSubsessions()) {
        CSG_LOGW("RTSP", "Failed to create MediaSession from SDP");
        c->running = 1;
        return;
    }

    CSG_LOGI("RTSP", "Got SDP");
    scs.iter = new MediaSubsessionIterator(*scs.session);
    setupNextSubsession(rtspClient);
}

static void setupNextSubsession(RTSPClient *rtspClient)
{
    UsageEnvironment &env = rtspClient->envir();
    ourRTSPClient *client = (ourRTSPClient *)rtspClient;
    CSG_RTSP_LIVE555_CTX *c = client->scs.c;
    StreamClientState &scs = client->scs;

    scs.subsession = scs.iter->next();
    if (scs.subsession != NULL) {
        const char *codecName = scs.subsession->codecName();
        if (codecName == NULL) {
            setupNextSubsession(rtspClient);
            return;
        }

        /* ===== 视频 subsession (H264/H265) ===== */
        if (strcmp(codecName, "H264") == 0 || strcmp(codecName, "H265") == 0 ||
            strcmp(codecName, "HEVC") == 0) {
            if (!scs.subsession->initiate()) {
                CSG_LOGW("RTSP", "initiate failed for video %s: %s",
                     codecName, env.getResultMsg());
                setupNextSubsession(rtspClient);
                return;
            }
            if (strncmp(codecName, "H26", 3) == 0 && strncmp(codecName, "H265", 4) != 0)
                c->codec = CSG_CODEC_H264;
            else
                c->codec = CSG_CODEC_H265;
            c->video_w = scs.subsession->videoWidth();
            c->video_h = scs.subsession->videoHeight();
            c->video_fps = scs.subsession->videoFPS();
            CSG_LOGI("RTSP", "Found video: %s %dx%d@%d codec=%d",
                 codecName, c->video_w, c->video_h, c->video_fps, c->codec);
            rtspClient->sendSetupCommand(*scs.subsession, continueAfterSETUP, False, False);
            return;
        }

        /* ===== 音频 subsession (全类型识别, 仅当设置了 audio_cb) =====
         * ★ 历史踩坑: 只识别 PCMU/PCMA/MPEG4-GENERIC 三种, 导致使用 L16/
         *   MPA/OPUS 等音频编码的摄像头被静默跳过 → 录制文件 audio=0.
         * 修复: 识别所有 live555 可报告的音频 codec 名称, 映射到 libmov
         *   支持的 MOV_OBJECT_* 对象类型, 录制时透传原始字节. */
        if (c->audio_cb) {
            int audio_ok = 0;

            /* ---- G.711 (PCMU/PCMA) → libmov alaw/ulaw ---- */
            if (strcmp(codecName, "PCMU") == 0 || strcmp(codecName, "PCMA") == 0) {
                c->audio_codec = (strcmp(codecName, "PCMA") == 0)
                    ? CSG_AUDIO_CODEC_G711A : CSG_AUDIO_CODEC_G711U;
                audio_ok = 1;
            }
            /* ---- AAC (mpeg4-generic) → libmov AAC ---- */
            else if (strcmp(codecName, "MPEG4-GENERIC") == 0 ||
                     strcmp(codecName, "mpeg4-generic") == 0) {
                c->audio_codec = CSG_AUDIO_CODEC_AAC;
                /* 提取 ASC: fmtp config= 参数 (base64 编码的 AudioSpecificConfig) */
                const char *fm = scs.subsession->fmtp_config();
                if (fm && fm[0]) {
                    c->audio_asc_size = (int)base64_decode(fm, c->audio_asc,
                                                           sizeof(c->audio_asc));
                } else {
                    c->audio_asc_size = 0;
                    CSG_LOGW("RTSP", "AAC fmtp config missing (ASC), decoder may fail");
                }
                audio_ok = 1;
            }
            /* ---- MPEG Audio (MPA/MP3) → libmov MP3 ---- */
            else if (strcmp(codecName, "MPA") == 0 ||
                     strcmp(codecName, "MP3") == 0) {
                c->audio_codec = CSG_AUDIO_CODEC_MP3;
                audio_ok = 1;
            }
            /* ---- Opus → libmov OPUS ---- */
            else if (strcmp(codecName, "OPUS") == 0 ||
                     strcmp(codecName, "opus") == 0) {
                c->audio_codec = CSG_AUDIO_CODEC_OPUS;
                audio_ok = 1;
            }
            /* ---- AC-3 → libmov AC3 ---- */
            else if (strcmp(codecName, "AC3") == 0 ||
                     strcmp(codecName, "ac-3") == 0) {
                c->audio_codec = CSG_AUDIO_CODEC_AC3;
                audio_ok = 1;
            }
            /* ---- Linear PCM 16-bit (L16) → 录制时转 G711u 兼容封装
             *   (libmov 无原生 LPCM 对象, 用 ulaw 轨兼容存储,
             *    回放时识别为 G711u 做 μ-law→PCM16 解码即可还原) ---- */
            else if (strcmp(codecName, "L16") == 0 ||
                     strcmp(codecName, "L8") == 0) {
                c->audio_codec = CSG_AUDIO_CODEC_LPCM;
                audio_ok = 1;
            }

            if (audio_ok) {
                if (!scs.subsession->initiate()) {
                    CSG_LOGW("RTSP", "initiate failed for audio %s: %s",
                         codecName, env.getResultMsg());
                    setupNextSubsession(rtspClient);
                    return;
                }
                c->audio_found = 1;
                c->audio_sample_rate = scs.subsession->rtpTimestampFrequency();
                c->audio_channels = scs.subsession->numChannels();
                if (c->audio_channels <= 0) c->audio_channels = 1;
                if (c->audio_sample_rate <= 0) c->audio_sample_rate = 8000;
                CSG_LOGI("RTSP", "Found audio: %s %dHz/%dch codec=%d",
                     codecName, c->audio_sample_rate, c->audio_channels,
                     c->audio_codec);
                rtspClient->sendSetupCommand(*scs.subsession, continueAfterSETUP,
                                             False, False);
                return;
            }
        }

        /* 跳过其他 subsession (如 GSM/G.722/G.726 等不支持的编码) */
        CSG_LOGI("RTSP", "Skip unknown subsession: %s", codecName ? codecName : "?");
        setupNextSubsession(rtspClient);
        return;
    }

    /* ===== 所有 subsession 已 SETUP 完成, 发送 PLAY ===== */
    if (c->codec == CSG_CODEC_H264 || c->codec == CSG_CODEC_H265) {
        CSG_LOGI("RTSP", "All subsessions setup (video codec=%d audio=%d), PLAY",
             c->codec, c->audio_found);
        /* SDP 遍历完成标志: 供录制在创建时据此判定"确实无音频", 避免空等 */
        c->sdp_done = 1;
        rtspClient->sendPlayCommand(*scs.session, continueAfterPLAY);
    } else {
        CSG_LOGW("RTSP", "No H264/H265 video subsession found");
        c->running = 1;
    }
}

static void continueAfterSETUP(RTSPClient *rtspClient, int resultCode, char *resultString)
{
    UsageEnvironment &env = rtspClient->envir();
    ourRTSPClient *client = (ourRTSPClient *)rtspClient;
    CSG_RTSP_LIVE555_CTX *c = client->scs.c;
    StreamClientState &scs = client->scs;

    delete[] resultString;
    if (resultCode != 0) {
        CSG_LOGW("RTSP", "SETUP failed: %d", resultCode);
        c->running = 1;
        return;
    }
    CSG_LOGI("RTSP", "SETUP ok, ports %d-%d", scs.subsession->clientPortNum(),
         scs.subsession->clientPortNum() + 1);

    FramedSource *source = scs.subsession->readSource();
    if (source == NULL) {
        CSG_LOGW("RTSP", "No readSource");
        c->running = 1;
        return;
    }

    /* 区分视频/音频 subsession (视频走 DummySink 组包, 音频走 AudioDummySink) */
    const char *codec_name = scs.subsession->codecName();
    int is_video = codec_name &&
        (strcmp(codec_name, "H264") == 0 || strcmp(codec_name, "H265") == 0 ||
         strcmp(codec_name, "HEVC") == 0);
    /* 音频 subession 判断: 覆盖所有已识别的音频 codec 名称.
     * ★ 必须与 setupNextSubsession 中的音频 codec 列表保持同步,
     *   否则 audio_cb 已设置但 is_audio 为 false → 不创建 AudioDummySink
     *   → 音频数据丢失. */
    int is_audio = codec_name && c->audio_cb &&
        (strcmp(codec_name, "PCMU") == 0 ||
         strcmp(codec_name, "PCMA") == 0 ||
         strcmp(codec_name, "MPEG4-GENERIC") == 0 ||
         strcmp(codec_name, "mpeg4-generic") == 0 ||
         strcmp(codec_name, "MPA") == 0 ||
         strcmp(codec_name, "MP3") == 0 ||
         strcmp(codec_name, "OPUS") == 0 ||
         strcmp(codec_name, "opus") == 0 ||
         strcmp(codec_name, "AC3") == 0 ||
         strcmp(codec_name, "ac-3") == 0 ||
         strcmp(codec_name, "L16") == 0 ||
         strcmp(codec_name, "L8") == 0);

    if (is_video) {
    /* 提取 SDP sprop 参数集 (base64 解码), 供组包时注入首帧.
     * ★ 历史踩坑: H264 SDP 标准用 "sprop-parameter-sets=SPS,PPS" 逗号格式,
     *   live555 的 fmtp_spropsps()/fmtp_spropppps() 读的是独立 "sprop-sps"/
     *   "sprop-pps" 属性 (仅 H265 标准使用), 对 H264 返回空 → 录制器/解码器
     *   无参数集 → 录制文件不完整/无法解码.
     *   必须按 codec 区分: H264 用 fmtp_spropparametersets() 逗号拆分,
     *   H265 用 fmtp_spropvps()/fmtp_spropsps()/fmtp_spropppps() 独立属性. */
        int is_h264 = (c->codec == CSG_CODEC_H264) ||
                      (codec_name && strcmp(codec_name, "H264") == 0);

        if (is_h264) {
            /* H264: sprop-parameter-sets=SPS,PPS (逗号分隔, 通常 2 个) */
            const char *psets = scs.subsession->fmtp_spropparametersets();
            if (psets && psets[0]) {
                char buf[512];
                strncpy(buf, psets, sizeof(buf) - 1);
                buf[sizeof(buf) - 1] = 0;
                char *sps_b64 = buf;
                char *comma = strchr(buf, ',');
                char *pps_b64 = NULL;
                if (comma) {
                    *comma = 0;
                    pps_b64 = comma + 1;
                }
                c->sprop_sps_size = (unsigned)base64_decode(
                                        sps_b64 ? sps_b64 : "",
                                        c->sprop_sps, sizeof(c->sprop_sps));
                c->sprop_pps_size = (unsigned)base64_decode(
                                        pps_b64 ? pps_b64 : "",
                                        c->sprop_pps, sizeof(c->sprop_pps));
            }
        } else {
            /* H265: sprop-vps/sprop-sps/sprop-pps 独立属性 */
            const char *vps_b64 = scs.subsession->fmtp_spropvps();
            const char *sps_b64 = scs.subsession->fmtp_spropsps();
            const char *pps_b64 = scs.subsession->fmtp_sproppps();
            c->sprop_vps_size = (unsigned)base64_decode(vps_b64 ? vps_b64 : "",
                                                        c->sprop_vps, sizeof(c->sprop_vps));
            c->sprop_sps_size = (unsigned)base64_decode(sps_b64 ? sps_b64 : "",
                                                        c->sprop_sps, sizeof(c->sprop_sps));
            c->sprop_pps_size = (unsigned)base64_decode(pps_b64 ? pps_b64 : "",
                                                        c->sprop_pps, sizeof(c->sprop_pps));
        }
        CSG_LOGI("RTSP", "Sprop params: vps=%u sps=%u pps=%u bytes (codec=%s)",
             c->sprop_vps_size, c->sprop_sps_size, c->sprop_pps_size,
             codec_name ? codec_name : "?");

        /* ★ 历史踩坑: 部分摄像头 (如 192.168.9.103 子码流) 的 H264 RTP 载荷
         * 与 live555 H264VideoStreamFramer 不兼容, 报
         *   "MultiFramedRTPSource::doGetNextFrame1(): The total received frame
         *    size exceeds the client's buffer size (86). trailing data dropped"
         * → framer 组不出完整帧 → 解码/录制全程无数据, 事件循环空转 CPU 99%.
         * 因此 H264 与 H265 一律走自定义 RTP → Annex-B 组包, 直接喂 RTP source,
         * 绝不经过 live555 的 H26xVideoStreamFramer. */
        scs.subsession->sink = DummySink::createNew(env, c);
        if (scs.subsession->sink == NULL) {
            CSG_LOGW("RTSP", "Failed to create video sink");
            c->running = 1;
            return;
        }
        scs.subsession->sink->startPlaying(*source, NULL, scs.subsession);
        CSG_LOGI("RTSP", "Video sink started: %s", codec_name);
    } else if (is_audio) {
        /* 音频: AudioDummySink 直接把 RTP 载荷 (G.711 单字节采样) 解码为
         * PCM16, 经上层回调喂 ALSA 输出到耳机. */
        c->audio_setup_done = 1;
        scs.subsession->sink = AudioDummySink::createNew(env, c);
        if (scs.subsession->sink == NULL) {
            CSG_LOGW("RTSP", "Failed to create audio sink");
            c->running = 1;
            return;
        }
        scs.subsession->sink->startPlaying(*source, NULL, scs.subsession);
        CSG_LOGI("RTSP", "Audio sink started: %s %dHz %dch",
             codec_name, c->audio_sample_rate, c->audio_channels);
    } else {
        CSG_LOGW("RTSP", "Unhandled subsession after SETUP: %s",
             codec_name ? codec_name : "?");
    }

    /* ★ 继续 SETUP 下一个 subsession: 视频+音频都要 SETUP 完成后再 PLAY.
     * 历史踩坑: 此处直接 sendPlayCommand 会让音频 subsession 永远不被
     * SETUP/创建 sink → SDP 里即使有 PCMU/PCMA 也听不到声音.
     * 全部遍历完成后由 setupNextSubsession 发送 PLAY. */
    setupNextSubsession(rtspClient);
}

static void continueAfterPLAY(RTSPClient *rtspClient, int resultCode, char *resultString)
{
    delete[] resultString;
    ourRTSPClient *client = (ourRTSPClient *)rtspClient;
    CSG_RTSP_LIVE555_CTX *c = client->scs.c;

    if (resultCode != 0) {
        CSG_LOGW("RTSP", "PLAY failed: %d", resultCode);
        c->running = 1;
        return;
    }
    CSG_LOGI("RTSP", "Playing: %dx%d@%d", c->video_w, c->video_h, c->video_fps);
}

/* ==================== DummySink ==================== */

DummySink *DummySink::createNew(UsageEnvironment &env, CSG_RTSP_LIVE555_CTX *c)
{
    return new DummySink(env, c);
}

DummySink::DummySink(UsageEnvironment &env, CSG_RTSP_LIVE555_CTX *c)
    : MediaSink(env), c(c)
{
    receiveBuffer = new u_int8_t[RTSP_LIVE555_BUFFER_SIZE];
    au_capacity = RTSP_LIVE555_BUFFER_SIZE / 2;   /* 1MB access unit 缓冲 */
    au_buf = new u_int8_t[au_capacity];
    au_size = 0;
    have_vcl = 0;
    au_ts = 0;
    sniffed = 0;
    m_first_presentation_us = -1;
}

DummySink::~DummySink()
{
    delete[] au_buf;
    delete[] receiveBuffer;
}

Boolean DummySink::continuePlaying()
{
    if (fSource == NULL) return False;
    fSource->getNextFrame(receiveBuffer, RTSP_LIVE555_BUFFER_SIZE,
                          afterGettingFrame, this, onSourceClosure, this);
    return True;
}

void DummySink::afterGettingFrame(void *clientData, unsigned frameSize,
                                  unsigned numTruncatedBytes,
                                  struct timeval presentationTime,
                                  unsigned /*durationInMicroseconds*/)
{
    DummySink *sink = (DummySink *)clientData;
    sink->afterGettingFrame2(frameSize, numTruncatedBytes, presentationTime);
}

/* 追加一个完整 NAL 单元 (加 start code) 到组包缓冲 */
void DummySink::appendNAL(const u_int8_t *nal, unsigned size)
{
    if (au_size + 4 + size > au_capacity) return;
    au_buf[au_size++] = 0x00;
    au_buf[au_size++] = 0x00;
    au_buf[au_size++] = 0x00;
    au_buf[au_size++] = 0x01;
    memcpy(au_buf + au_size, nal, size);
    au_size += size;
}

/* 交付一个完整 access unit (首帧前注入 sprop 参数集) */
void DummySink::deliverAU(int64_t pts)
{
    if (au_size == 0) return;
    if (c == NULL || c->cb == NULL) { au_size = 0; have_vcl = 0; return; }

    /* ★ 首帧 或 每个 IDR 关键帧 前注入 VPS/SPS/PPS (按需, size>0 才注入,
     * 避免 H264 无 VPS 时产生孤立 start code 空 NAL).
     * 原因: 录制器可能在首帧到达前才启动 (错过首帧参数集), 且录制线程
     * 从帧内提取 SPS/PPS. 必须让每个 IDR 自包含参数集, 才能保证:
     *   1) 录制文件任意位置 seek 到关键帧可解码播放
     *   2) 录制线程即使错过首帧也能在某个 IDR 帧提取到 SPS/PPS
     *   (历史踩坑: 只在首帧注入 → 录制器报 param sets incomplete 死等) */
    int need_param = (c->frame_count == 0);
    if (!need_param)
        need_param = detect_keyframe(au_buf, au_size, c->codec);

    unsigned extra = 0;
    if (need_param) {
        extra = (c->sprop_vps_size ? c->sprop_vps_size + 4 : 0)
              + (c->sprop_sps_size ? c->sprop_sps_size + 4 : 0)
              + (c->sprop_pps_size ? c->sprop_pps_size + 4 : 0);
    }

    u_int8_t *buf = (u_int8_t *)malloc(extra + au_size);
    if (buf == NULL) { au_size = 0; have_vcl = 0; return; }
    unsigned off = 0;
    if (extra > 0) {
        const u_int8_t start[4] = {0x00, 0x00, 0x00, 0x01};
        if (c->sprop_vps_size) {
            memcpy(buf + off, start, 4); off += 4;
            memcpy(buf + off, c->sprop_vps, c->sprop_vps_size); off += c->sprop_vps_size;
        }
        if (c->sprop_sps_size) {
            memcpy(buf + off, start, 4); off += 4;
            memcpy(buf + off, c->sprop_sps, c->sprop_sps_size); off += c->sprop_sps_size;
        }
        if (c->sprop_pps_size) {
            memcpy(buf + off, start, 4); off += 4;
            memcpy(buf + off, c->sprop_pps, c->sprop_pps_size); off += c->sprop_pps_size;
        }
    }
    memcpy(buf + off, au_buf, au_size); off += au_size;

    int key = detect_keyframe(buf, off, c->codec);
    c->frame_count++;
    if (c->frame_count == 1 || c->frame_count % 50 == 0) {
        CSG_LOGI("RTSP", "frame #%llu size=%u key=%d",
             (unsigned long long)c->frame_count, off, key);
    }

    /* 压入解码队列 (环形, 满则丢最旧帧保持实时), 立即返回。
     * MPP 解码等耗时操作由解码线程执行, 绝不在事件循环线程做,
     * 否则阻塞 RTP 接收会导致只出首帧后事件循环卡死。 */
    pthread_mutex_lock(&c->q_mutex);
    if (c->q_count < RTSP_FRAME_QUEUE_MAX) {
        RTSP_QUEUED_FRAME *f = &c->q_frames[c->q_tail];
        f->data = buf;              /* 所有权转移给解码线程 */
        f->size = off;
        f->key = key;
        f->pts = (uint64_t)pts;
        c->q_tail = (c->q_tail + 1) % RTSP_FRAME_QUEUE_MAX;
        c->q_count++;
        pthread_cond_signal(&c->q_cond);
        buf = NULL;
    }
    pthread_mutex_unlock(&c->q_mutex);
    if (buf) free(buf);             /* 队列满时丢弃 */

    au_size = 0;
    have_vcl = 0;
}

/* 判断 NAL 类型是否属于 VCL (视频编码层), 每个 access unit 以 VCL 开头。
 * 用"新的 VCL NAL 出现"作为帧边界, 不依赖 RTP 时间戳 (部分摄像头时间戳不递增)。
 *  H264: NAL type 1..5 为 VCL (1-4 非 IDR, 5 = IDR)
 *  H265: NAL type ≤ 31 为 VCL */
static Boolean is_vcl_nal(unsigned nal_type, int codec)
{
    if (codec == CSG_CODEC_H264)
        return (nal_type >= 1 && nal_type <= 5);
    return nal_type <= 31;
}

void DummySink::afterGettingFrame2(unsigned frameSize, unsigned numTruncatedBytes,
                                   struct timeval presentationTime)
{
    CSG_RTSP_LIVE555_CTX *c = this->c;

    if (numTruncatedBytes > 0) {
        fprintf(stderr, "rtsp_live555: frame truncated by %u bytes\n",
                numTruncatedBytes);
    }

    if (c == NULL || c->cb == NULL) {
        continuePlaying();
        return;
    }

    /* 将 live555 的壁钟 presentationTime 转换为相对时间戳:
     * 首帧记录基准, 后续帧减去基准得到从 0 开始的单调递增 PTS (微秒). */
    int64_t wall_time_us = (int64_t)presentationTime.tv_sec * 1000000 +
                           (int64_t)presentationTime.tv_usec;
    if (m_first_presentation_us < 0) {
        m_first_presentation_us = wall_time_us;
    }
    int64_t rtp_time = wall_time_us - m_first_presentation_us;
    if (rtp_time < 0) rtp_time = 0;  /* 防 NTP 跳回 */

    /* ============ 自定义 RTP → Annex-B 组包 (H264 & H265) ============
     * 直接消费 RTP source (不经过 live555 framer), 统一在此组包:
     *   - 已是 Annex-B (start code 开头) 的载荷直接累积
     *   - 否则按 RFC 3984 (H264) / RFC 7798 (H265) 解析 单NAL/聚合/分片 */

    if (frameSize < 2) {
        continuePlaying();
        return;
    }

    /* 已是 Annex-B (start code 开头)? 直接累积 */
    if (frameSize >= 4 && receiveBuffer[0] == 0x00 &&
        receiveBuffer[1] == 0x00 && receiveBuffer[2] == 0x00 &&
        receiveBuffer[3] == 0x01) {
        if (au_size + frameSize <= au_capacity) {
            memcpy(au_buf + au_size, receiveBuffer, frameSize);
            au_size += frameSize;
        }
        have_vcl = 1;
        continuePlaying();
        return;
    }

    const u_int8_t *p = receiveBuffer;
    unsigned remaining = frameSize;

    if (c->codec == CSG_CODEC_H264) {
        /* ============ H264: RFC 3984 RTP 组包 (NAL 头 1 字节) ============ */
        unsigned nal_type = p[0] & 0x1F;

        if (nal_type == 24 || nal_type == 25) {         /* STAP-A / STAP-B */
            unsigned hdr_len = (nal_type == 24) ? 1 : 3;  /* STAP-B 含 2B DON */
            p += hdr_len; remaining -= hdr_len;
            while (remaining >= 2) {
                unsigned nlen = (p[0] << 8) | p[1];
                p += 2; remaining -= 2;
                if (nlen == 0 || nlen > remaining) break;
                unsigned inner_type = p[0] & 0x1F;
                if (is_vcl_nal(inner_type, c->codec) && have_vcl)
                    deliverAU(au_ts);
                if (is_vcl_nal(inner_type, c->codec) && !have_vcl)
                    au_ts = rtp_time;
                appendNAL(p, nlen);
                if (is_vcl_nal(inner_type, c->codec)) have_vcl = 1;
                p += nlen; remaining -= nlen;
            }
        } else if (nal_type == 28 || nal_type == 29) {  /* FU-A / FU-B */
            /* FU-A: [FU indicator][FU header][payload]; FU-B: 再后 2B DON */
            unsigned data_off = (nal_type == 28) ? 2 : 4;
            if (frameSize <= data_off) {
                continuePlaying();
                return;
            }
            unsigned char fu = receiveBuffer[data_off - 1];
            unsigned char s = (fu >> 7) & 1;
            unsigned char fu_type = fu & 0x1F;
            if (s) {
                /* 新 NAL 开始: 重建 1 字节 NAL 头 (保留 F/NRI, 替换 Type) */
                u_int8_t nal_hdr = (u_int8_t)((receiveBuffer[0] & 0xE0) | fu_type);
                if (is_vcl_nal(fu_type, c->codec) && have_vcl)
                    deliverAU(au_ts);
                if (is_vcl_nal(fu_type, c->codec) && !have_vcl)
                    au_ts = rtp_time;
                appendNAL(&nal_hdr, 1);
                if (is_vcl_nal(fu_type, c->codec)) have_vcl = 1;
            }
            if (au_size + (frameSize - data_off) <= au_capacity) {
                memcpy(au_buf + au_size, receiveBuffer + data_off,
                       frameSize - data_off);
                au_size += frameSize - data_off;
            }
        } else {                        /* 单 NAL 包 (1-23, 含 SPS/PPS/IDR/P) */
            if (is_vcl_nal(nal_type, c->codec) && have_vcl)
                deliverAU(au_ts);
            if (is_vcl_nal(nal_type, c->codec) && !have_vcl)
                au_ts = rtp_time;
            appendNAL(receiveBuffer, frameSize);
            if (is_vcl_nal(nal_type, c->codec)) have_vcl = 1;
        }
    } else {
        /* ============ H265: RFC 7798 RTP 组包 (PayloadHdr 2 字节) ============ */
        unsigned nal_type = (p[0] >> 1) & 0x3F;

        if (nal_type == 48) {           /* AP 聚合包 */
            p += 2; remaining -= 2;
            while (remaining >= 3) {
                unsigned nlen = (p[0] << 8) | p[1];
                p += 2; remaining -= 2;
                if (nlen == 0 || nlen > remaining) break;
                unsigned inner_type = (p[0] >> 1) & 0x3F;
                if (is_vcl_nal(inner_type, c->codec) && have_vcl)
                    deliverAU(au_ts);
                if (is_vcl_nal(inner_type, c->codec) && !have_vcl)
                    au_ts = rtp_time;
                appendNAL(p, nlen);
                if (is_vcl_nal(inner_type, c->codec)) have_vcl = 1;
                p += nlen; remaining -= nlen;
            }
        } else if (nal_type == 49) {    /* FU 分片包 */
            if (frameSize < 3) {
                continuePlaying();
                return;
            }
            unsigned char fu = receiveBuffer[2];
            unsigned char s = (fu >> 7) & 1;
            unsigned char fu_type = fu & 0x3F;
            if (s) {
                u_int8_t nal_hdr[2];
                nal_hdr[0] = (receiveBuffer[0] & 0x81) | (fu_type << 1);
                nal_hdr[1] = receiveBuffer[1];
                if (is_vcl_nal(fu_type, c->codec) && have_vcl)
                    deliverAU(au_ts);
                if (is_vcl_nal(fu_type, c->codec) && !have_vcl)
                    au_ts = rtp_time;
                appendNAL(nal_hdr, 2);
                if (is_vcl_nal(fu_type, c->codec)) have_vcl = 1;
            }
            if (au_size + (frameSize - 3) <= au_capacity) {
                memcpy(au_buf + au_size, receiveBuffer + 3, frameSize - 3);
                au_size += frameSize - 3;
            }
        } else {                        /* 单 NAL 包 */
            if (is_vcl_nal(nal_type, c->codec) && have_vcl)
                deliverAU(au_ts);
            if (is_vcl_nal(nal_type, c->codec) && !have_vcl)
                au_ts = rtp_time;
            appendNAL(receiveBuffer, frameSize);
            if (is_vcl_nal(nal_type, c->codec)) have_vcl = 1;
        }
    }

    continuePlaying();
}

/* ==================== StreamClientState / ourRTSPClient ==================== */

StreamClientState::StreamClientState(CSG_RTSP_LIVE555_CTX *c) : c(c)
{
    session = NULL;
    iter = NULL;
    subsession = NULL;
}

StreamClientState::~StreamClientState()
{
    delete iter;
    if (session != NULL) {
        Medium::close(session);
    }
}

ourRTSPClient *ourRTSPClient::createNew(UsageEnvironment &env, char const *rtspURL,
                                        CSG_RTSP_LIVE555_CTX *c)
{
    return new ourRTSPClient(env, rtspURL, c);
}

ourRTSPClient::ourRTSPClient(UsageEnvironment &env, char const *rtspURL, CSG_RTSP_LIVE555_CTX *c)
    : RTSPClient(env, rtspURL, 1, "RK3572 Player", 0, -1), scs(c)
{
}

ourRTSPClient::~ourRTSPClient()
{
}

/* ==================== 解码线程: 队列消费 + 回调 (MPP 解码等耗时操作) ==================== */

static void *decode_thread_fn(void *arg)
{
    CSG_RTSP_LIVE555_CTX *c = (CSG_RTSP_LIVE555_CTX *)arg;

    for (;;) {
        pthread_mutex_lock(&c->q_mutex);
        while (c->q_count == 0 && !c->q_stop)
            pthread_cond_wait(&c->q_cond, &c->q_mutex);
        if (c->q_stop && c->q_count == 0) {
            pthread_mutex_unlock(&c->q_mutex);
            break;
        }
        RTSP_QUEUED_FRAME f = c->q_frames[c->q_head];
        c->q_head = (c->q_head + 1) % RTSP_FRAME_QUEUE_MAX;
        c->q_count--;
        pthread_mutex_unlock(&c->q_mutex);

        /* MPP 解码 + RGA + LVGL (在独立线程, 不阻塞 RTP 接收) */
        if (c->cb) {
            c->cb(f.data, f.size, f.key, f.pts, c->codec, c->user_data);
        }
        free(f.data);
    }
    CSG_LOGI("RTSP", "Decode thread exited");
    return NULL;
}

/* ==================== C 接口 ==================== */

extern "C" void *csg_rtsp_live555_create(const char *url, int codec_type_hint)
{
    CSG_RTSP_LIVE555_CTX *c = (CSG_RTSP_LIVE555_CTX *)calloc(1, sizeof(CSG_RTSP_LIVE555_CTX));
    if (c == NULL) return NULL;
    c->codec = codec_type_hint;
    c->running = 0;

    /* 初始化解码队列 */
    pthread_mutex_init(&c->q_mutex, NULL);
    pthread_cond_init(&c->q_cond, NULL);
    c->q_stop = 0;
    c->q_head = c->q_tail = c->q_count = 0;

    char user[64], pass[64], clean[512];
    parse_url_auth(url, clean, sizeof(clean), user, sizeof(user), pass, sizeof(pass));
    CSG_LOGI("RTSP", "Connecting to %s (user=%s)", clean, user[0] ? user : "(none)");

    /* 串行化 live555 库初始化 (多实例并发首次初始化会堆损坏) */
    pthread_mutex_lock(&s_rtsp_init_mutex);

    c->scheduler = BasicTaskScheduler::createNew();
    c->env = BasicUsageEnvironment::createNew(*c->scheduler);
    c->auth = new Authenticator(user, pass);

    c->client = ourRTSPClient::createNew(*c->env, clean, c);
    if (c->client == NULL) {
        CSG_LOGW("RTSP", "RTSPClient::createNew failed");
        delete c->auth;
        c->env->reclaim();
        delete c->scheduler;
        pthread_mutex_unlock(&s_rtsp_init_mutex);
        pthread_mutex_destroy(&c->q_mutex);
        pthread_cond_destroy(&c->q_cond);
        free(c);
        return NULL;
    }

    pthread_mutex_unlock(&s_rtsp_init_mutex);
    return c;
}

extern "C" void csg_rtsp_live555_set_callback(void *handle, csg_rtsp_frame_callback cb,
                                              void *user_data)
{
    CSG_RTSP_LIVE555_CTX *c = (CSG_RTSP_LIVE555_CTX *)handle;
    if (c == NULL) return;
    c->cb = cb;
    c->user_data = user_data;
}

extern "C" void csg_rtsp_live555_set_audio_callback(void *handle,
                                                    csg_rtsp_audio_callback cb,
                                                    void *user_data)
{
    CSG_RTSP_LIVE555_CTX *c = (CSG_RTSP_LIVE555_CTX *)handle;
    if (c == NULL) return;
    c->audio_cb = cb;
    c->audio_user_data = user_data;
}

static void *event_loop_thread(void *arg)
{
    CSG_RTSP_LIVE555_CTX *c = (CSG_RTSP_LIVE555_CTX *)arg;

    /* 事件循环线程内触发 DESCRIBE, 然后进入事件循环 */
    c->client->sendDescribeCommand(continueAfterDESCRIBE, c->auth);
    c->env->taskScheduler().doEventLoop(&c->running);
    CSG_LOGI("RTSP", "Event loop exited");
    return NULL;
}

extern "C" int csg_rtsp_live555_start(void *handle)
{
    CSG_RTSP_LIVE555_CTX *c = (CSG_RTSP_LIVE555_CTX *)handle;
    if (c == NULL) return -1;

    c->running = 0;
    c->q_stop = 0;

    /* 串行化建连: 避免两个实例的事件循环线程同时首次 sendDescribeCommand
     * 触发 live555 内部静态资源初始化竞态 (见 s_rtsp_init_mutex 注释) */
    pthread_mutex_lock(&s_rtsp_init_mutex);

    /* 先启动解码线程 (消费队列), 再启动事件循环 (生产) */
    if (pthread_create(&c->decode_thread, NULL, decode_thread_fn, c) != 0) {
        CSG_LOGE("RTSP", "decode thread create failed");
        pthread_mutex_unlock(&s_rtsp_init_mutex);
        return -1;
    }
    if (pthread_create(&c->thread, NULL, event_loop_thread, c) != 0) {
        CSG_LOGE("RTSP", "event loop thread create failed");
        c->q_stop = 1;
        pthread_cond_broadcast(&c->q_cond);
        pthread_join(c->decode_thread, NULL);
        pthread_mutex_unlock(&s_rtsp_init_mutex);
        return -1;
    }

    pthread_mutex_unlock(&s_rtsp_init_mutex);
    return 0;
}

extern "C" void csg_rtsp_live555_stop(void *handle)
{
    CSG_RTSP_LIVE555_CTX *c = (CSG_RTSP_LIVE555_CTX *)handle;
    if (c == NULL) return;

    /* ① 停解码线程: 唤醒并等待其退出 (不再回调, MPP 才能安全销毁) */
    pthread_mutex_lock(&c->q_mutex);
    c->q_stop = 1;
    pthread_cond_broadcast(&c->q_cond);
    pthread_mutex_unlock(&c->q_mutex);
    pthread_join(c->decode_thread, NULL);

    /* ② 停事件循环线程 */
    c->running = 1;
    pthread_join(c->thread, NULL);

    /* ③ 清理队列残余帧 */
    while (c->q_count > 0) {
        free(c->q_frames[c->q_head].data);
        c->q_head = (c->q_head + 1) % RTSP_FRAME_QUEUE_MAX;
        c->q_count--;
    }

    /* ④ 释放 live555 资源 */
    if (c->client != NULL) {
        Medium::close(c->client);
        c->client = NULL;
    }
    if (c->auth != NULL) {
        delete c->auth;
        c->auth = NULL;
    }
    c->env->reclaim();
    delete c->scheduler;

    pthread_mutex_destroy(&c->q_mutex);
    pthread_cond_destroy(&c->q_cond);
    free(c);
    CSG_LOGI("RTSP", "Stopped");
}

extern "C" int csg_rtsp_live555_get_raw_info(void *handle, int *codec, int *width, int *height,
                                             const uint8_t **sps, int *sps_len,
                                             const uint8_t **pps, int *pps_len,
                                             const uint8_t **vps, int *vps_len)
{
    CSG_RTSP_LIVE555_CTX *c = (CSG_RTSP_LIVE555_CTX *)handle;
    if (!c || !codec || !width || !height || !sps || !sps_len ||
        !pps || !pps_len || !vps || !vps_len) return -1;

    if (c->codec != CSG_CODEC_H264 && c->codec != CSG_CODEC_H265) return -1;
    if (c->video_w <= 0 || c->video_h <= 0) return -1;

    *codec = c->codec;
    *width = c->video_w;
    *height = c->video_h;
    *sps = c->sprop_sps;
    *sps_len = (int)c->sprop_sps_size;
    *pps = c->sprop_pps;
    *pps_len = (int)c->sprop_pps_size;
    *vps = c->sprop_vps;
    *vps_len = (int)c->sprop_vps_size;

    return 0;
}

extern "C" int csg_rtsp_live555_get_audio_asc(void *handle, const uint8_t **asc, int *asc_size)
{
    CSG_RTSP_LIVE555_CTX *c = (CSG_RTSP_LIVE555_CTX *)handle;
    if (!c || !asc || !asc_size) return -1;
    if (c->audio_asc_size <= 0) {
        *asc = NULL;
        *asc_size = 0;
        return -1;
    }
    *asc = c->audio_asc;
    *asc_size = c->audio_asc_size;
    return 0;
}

extern "C" int csg_rtsp_live555_get_audio_info(void *handle, int *codec,
                                               int *sample_rate, int *channels,
                                               int *sdp_done)
{
    CSG_RTSP_LIVE555_CTX *c = (CSG_RTSP_LIVE555_CTX *)handle;
    if (!c || !codec || !sample_rate || !channels || !sdp_done) return -1;

    *codec = c->audio_codec;          /* CSG_AUDIO_CODEC_* / NONE(无音频) */
    *sample_rate = c->audio_sample_rate;
    *channels = c->audio_channels;
    *sdp_done = c->sdp_done;          /* 1 = 所有 subsession SETUP 完成(已发 PLAY) */
    return 0;
}