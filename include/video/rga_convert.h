/* SPDX-License-Identifier: MIT */
/*
 * rga_convert.h - RGA 硬件色彩转换模块
 *
 * 基于 librga 实现 NV12 → RGB888/RGB565 硬件加速转换
 * 参考: /home/dell/librga-main/samples/cvtcolor_demo
 */

#ifndef __RGA_CONVERT_H__
#define __RGA_CONVERT_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化 RGA */
int csg_rga_convert_init(void);

/* NV12 转 RGB888 (硬件加速) */
int csg_rga_convert_nv12_to_rgb(void *nv12_src, int src_w, int src_h,
                            int src_hor_stride, int src_ver_stride,
                            void *rgb_dst, int dst_w, int dst_h,
                            int dst_format);

/* NV12 缩放 + 转 RGB888 (硬件加速, 一步完成).
 * 视频分辨率与屏幕不匹配时用 RGA 硬件缩放到 dst_w×dst_h,
 * 避免把原始大图交给 LVGL 软件缩放导致 CPU 高负载/鼠标卡顿. */
int csg_rga_convert_nv12_to_rgb_scaled(void *nv12_src, int src_w, int src_h,
                                       int src_hor_stride, int src_ver_stride,
                                       void *rgb_dst, int dst_w, int dst_h,
                                       int dst_format);

/* 释放 RGA */
void csg_rga_convert_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* __RGA_CONVERT_H__ */