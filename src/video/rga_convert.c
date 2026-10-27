/* SPDX-License-Identifier: MIT */
/*
 * rga_convert.c - RGA 硬件色彩转换实现
 *
 * 基于 librga 实现 NV12 YUV → RGB888/RGB565 硬件加速转换
 * 参考: /home/dell/librga-main/samples/cvtcolor_demo
 *       /home/dell/librga-main/include/im2d.h
 */

#include "rga_convert.h"
#include "common.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

#include "im2d.h"
#include "RgaUtils.h"

/* RGA 全局状态 */
static int s_csg_rga_inited = 0;

/* ★ 多路并发调用互斥锁: NVR 多路场景下每路一个解码线程并发调用 RGA,
 * librga im2d 虽带内部锁, 但多线程同时 submit 大尺寸转换仍可能造成
 * RGA 任务队列拥塞/脏状态. 串行化后多路分屏帧率本身不高 (每路
 * 10~25fps), 单次转换 <2ms, 串行化开销可接受. */
static pthread_mutex_t s_csg_rga_mutex = PTHREAD_MUTEX_INITIALIZER;

/* im2d 成功返回可能是 IM_STATUS_SUCCESS(1) 或 IM_STATUS_NOERROR(2),
 * 只判 ==IM_STATUS_SUCCESS 会把成功的 NOERROR 误判为失败 */
static int csg_rga_convert_status_ok(IM_STATUS s)
{
    return (s == IM_STATUS_SUCCESS || s == IM_STATUS_NOERROR);
}

int csg_rga_convert_init(void)
{
    if (s_csg_rga_inited) return 0;

    CSG_LOGI("RGA", "RGA version: %s", querystring(RGA_VERSION));
    CSG_LOGI("RGA", "init success");

    s_csg_rga_inited = 1;
    return 0;
}

int csg_rga_convert_nv12_to_rgb(void *nv12_src, int src_w, int src_h,
                            int src_hor_stride, int src_ver_stride,
                            void *rgb_dst, int dst_w, int dst_h,
                            int dst_format)
{
    if (!s_csg_rga_inited) {
        CSG_LOGE("RGA", "Not initialized");
        return -1;
    }

    IM_STATUS ret;
    im_rect src_rect, dst_rect;

    memset(&src_rect, 0, sizeof(src_rect));
    memset(&dst_rect, 0, sizeof(dst_rect));
    src_rect.width = src_w;
    src_rect.height = src_h;
    dst_rect.width = dst_w;
    dst_rect.height = dst_h;

    pthread_mutex_lock(&s_csg_rga_mutex);

    /* RK3572 RGA2_PRO 带 MMU, 直接传虚拟地址即可 (参考 librga cvtcolor_demo)。
     * 不能用 importbuffer_virtualaddr(param): 它按 w*h*1.5 估算缓冲大小,
     * 不认 MPP 的 ver_stride (1920x1088), 导致 imcheck 报缓冲越界 */
    rga_buffer_t src_img = wrapbuffer_virtualaddr(nv12_src, src_w, src_h,
                                                  RK_FORMAT_YCbCr_420_SP,
                                                  src_hor_stride, src_ver_stride);
    rga_buffer_t dst_img = wrapbuffer_virtualaddr(rgb_dst, dst_w, dst_h,
                                                  dst_format);

    /* 检查参数 */
    ret = imcheck(src_img, dst_img, src_rect, dst_rect);
    if (!csg_rga_convert_status_ok(ret)) {
        CSG_LOGE("RGA", "imcheck failed: ret=%d (%s)", ret, imStrError(ret));
        pthread_mutex_unlock(&s_csg_rga_mutex);
        return -1;
    }

    /* 执行色彩转换: NV12 -> RGB */
    ret = imcvtcolor(src_img, dst_img, RK_FORMAT_YCbCr_420_SP, dst_format);
    if (!csg_rga_convert_status_ok(ret)) {
        CSG_LOGE("RGA", "imcvtcolor failed: ret=%d (%s)", ret, imStrError(ret));
        pthread_mutex_unlock(&s_csg_rga_mutex);
        return -1;
    }
    pthread_mutex_unlock(&s_csg_rga_mutex);
    return 0;
}

int csg_rga_convert_nv12_to_rgb_scaled(void *nv12_src, int src_w, int src_h,
                                    int src_hor_stride, int src_ver_stride,
                                    void *rgb_dst, int dst_w, int dst_h,
                                    int dst_format)
{
    if (!s_csg_rga_inited) {
        CSG_LOGE("RGA", "Not initialized");
        return -1;
    }

    IM_STATUS ret;
    im_rect src_rect, dst_rect;

    memset(&src_rect, 0, sizeof(src_rect));
    memset(&dst_rect, 0, sizeof(dst_rect));
    src_rect.width = src_w;
    src_rect.height = src_h;
    dst_rect.width = dst_w;
    dst_rect.height = dst_h;

    rga_buffer_t src_img = wrapbuffer_virtualaddr(nv12_src, src_w, src_h,
                                                  RK_FORMAT_YCbCr_420_SP,
                                                  src_hor_stride, src_ver_stride);
    rga_buffer_t dst_img = wrapbuffer_virtualaddr(rgb_dst, dst_w, dst_h,
                                                  dst_format);

    pthread_mutex_lock(&s_csg_rga_mutex);

    ret = imcheck(src_img, dst_img, src_rect, dst_rect);
    if (!csg_rga_convert_status_ok(ret)) {
        CSG_LOGE("RGA", "imcheck(scaled) failed: ret=%d (%s)", ret, imStrError(ret));
        pthread_mutex_unlock(&s_csg_rga_mutex);
        return -1;
    }

    /* RGA 硬件缩放 + NV12→RGB 一步完成.
     * ★ 历史踩坑: 原 imcvtcolor 仅做色彩转换不支持缩放, 且 main.c 只对
     *   4K(≥3840) 视频缩到 1080p, 2560x1440 直接按原尺寸 cvtcolor 输出,
     *   LVGL 用 lv_image_set_scale 软件缩放大图 (1440p 每帧 14.7MB 软件
     *   重采样), CPU 高负载 → 鼠标卡顿; 且原尺寸上 1080p 屏幕超出 → 花屏.
     *   必须用 imresize 让 RGA 硬件缩放. */
    ret = imresize(src_img, dst_img,
                   (double)dst_w / (double)src_w,
                   (double)dst_h / (double)src_h,
                   IM_SYNC, IM_INTERP_LINEAR);
    if (!csg_rga_convert_status_ok(ret)) {
        CSG_LOGE("RGA", "imresize failed: ret=%d (%s)", ret, imStrError(ret));
        pthread_mutex_unlock(&s_csg_rga_mutex);
        return -1;
    }
    pthread_mutex_unlock(&s_csg_rga_mutex);
    return 0;
}

void csg_rga_convert_deinit(void)
{
    s_csg_rga_inited = 0;
    CSG_LOGI("RGA", "Deinitialized");
}