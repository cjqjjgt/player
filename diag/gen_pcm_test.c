/* 诊断工具: 复制 src.mp4 的视频轨, 添加 PCM16 sowt 音频轨
 * (弱信号 ±100 正弦, 模拟 RTSP G.711 直录电平), 输出 dst.mp4.
 * 用于验证本地播放 PCM 轨的增益修复 (弱信号 -> ×8 -> 可闻电平). */
#include <mp4v2/mp4v2.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

int main(int argc, char *argv[]) {
    if (argc < 3) { fprintf(stderr, "usage: %s src.mp4 dst.mp4\n", argv[0]); return 1; }

    MP4FileHandle src = MP4Read(argv[1]);
    if (src == MP4_INVALID_FILE_HANDLE) { fprintf(stderr, "open src failed\n"); return 1; }
    MP4FileHandle dst = MP4Create(argv[2], 0);
    if (dst == MP4_INVALID_FILE_HANDLE) { fprintf(stderr, "create dst failed\n"); return 1; }

    /* --- 复制视频轨 (变帧率: 每样本用真实 duration 写入) --- */
    MP4TrackId vt = MP4FindTrackId(src, 0, MP4_VIDEO_TRACK_TYPE, 0);
    if (!MP4_IS_VALID_TRACK_ID(vt)) { fprintf(stderr, "no video track\n"); return 1; }
    uint32_t vtscale = MP4GetTrackTimeScale(src, vt);
    uint16_t vw = MP4GetTrackVideoWidth(src, vt);
    uint16_t vh = MP4GetTrackVideoHeight(src, vt);

    /* 解析 avcC extradata 提取 profile/level/SPS/PPS (与 mp4_record.c 一致) */
    uint8_t *ext = NULL; uint32_t extsz = 0;
    uint8_t profile = 0, compat = 0, level = 0, nlsz = 3;
    uint8_t sps[512], pps[512];
    uint32_t sps_len = 0, pps_len = 0;
    int has_avcc = MP4GetTrackESConfiguration(src, vt, &ext, &extsz) && ext && extsz >= 7;
    if (has_avcc) {
        profile = ext[1];
        compat  = ext[2];
        level   = ext[3];
        nlsz    = ext[4] & 0x03;
        uint32_t o = 5;
        uint32_t nsps = ext[o] & 0x1F; o++;
        for (uint32_t i = 0; i < nsps && o + 2 <= extsz; i++) {
            uint32_t n = (ext[o] << 8) | ext[o + 1]; o += 2;
            if (n > 0 && o + n <= extsz && sps_len == 0) {
                memcpy(sps, ext + o, n); sps_len = n;
            }
            o += n;
        }
        if (o < extsz) {
            uint32_t npps = ext[o]; o++;
            for (uint32_t i = 0; i < npps && o + 2 <= extsz; i++) {
                uint32_t n = (ext[o] << 8) | ext[o + 1]; o += 2;
                if (n > 0 && o + n <= extsz && pps_len == 0) {
                    memcpy(pps, ext + o, n); pps_len = n;
                }
                o += n;
            }
        }
        printf("avcC parsed: profile=%u level=%u nlsz=%u sps=%u pps=%u\n",
               profile, level, nlsz, sps_len, pps_len);
    } else {
        printf("WARN: no avcC extradata, using defaults\n");
    }

    MP4TrackId vdst = MP4AddH264VideoTrack(dst, vtscale, MP4_INVALID_DURATION,
                                           vw, vh, profile, compat, level, nlsz);
    if (!MP4_IS_VALID_TRACK_ID(vdst)) { fprintf(stderr, "add video failed\n"); return 1; }
    if (sps_len > 0 && pps_len > 0) {
        MP4AddH264SequenceParameterSet(dst, vdst, sps, (uint16_t)sps_len);
        MP4AddH264PictureParameterSet(dst, vdst, pps, (uint16_t)pps_len);
    }

    uint32_t nsamp = MP4GetTrackNumberOfSamples(src, vt);
    for (uint32_t i = 1; i <= nsamp; i++) {
        uint8_t *buf = NULL; uint32_t sz = 0; MP4Duration dur = 0; uint8_t sync = 0;
        MP4ReadSample(src, vt, i, &buf, &sz, NULL, &dur, NULL, &sync);
        if (buf) {
            MP4WriteSample(dst, vdst, buf, sz, dur, 0, sync != 0);
            free(buf);
        }
    }
    if (ext) free(ext);

    /* --- 添加 PCM16 sowt 弱信号音频轨 8kHz/1ch (模拟 G.711 ±100 电平) --- */
    MP4TrackId adst = MP4AddPcm16AudioTrack(dst, 8000, 1);
    if (!MP4_IS_VALID_TRACK_ID(adst)) { fprintf(stderr, "add audio failed\n"); return 1; }
    const int rate = 8000;
    const int total = rate * ((argc >= 4) ? atoi(argv[3]) : 4);  /* 秒数, 默认 4 */
    const int chunk = 160;             /* 20ms, 与录制器一致 */
    for (int off = 0; off < total; off += chunk) {
        int n = (total - off < chunk) ? total - off : chunk;
        int16_t pcm[chunk];
        for (int i = 0; i < n; i++) {
            double t = (double)(off + i) / rate;
            pcm[i] = (int16_t)(100.0 * sin(2.0 * M_PI * 1000.0 * t));  /* ±100 1kHz */
        }
        MP4WriteSample(dst, adst, (const uint8_t *)pcm, (uint32_t)(n * 2), (MP4Duration)n, 0, true);
    }

    MP4Close(dst, 0);
    MP4Close(src, 0);
    printf("OK: video %u samples copied, audio 4s weak-PCM(+-100) written\n", (unsigned)nsamp);
    return 0;
}
