/* SPDX-License-Identifier: MIT */
/*
 * onvif_discovery.h - ONVIF WS-Discovery 设备搜索模块 (纯 C 实现)
 *
 * 功能:
 *   1. 通过 WS-Discovery 组播 Probe 搜索局域网内 ONVIF 摄像头
 *   2. 解析 ProbeMatch 响应获取设备信息
 *   3. 异步搜索线程, 结果通过回调通知 UI 层
 *
 * 协议参考: OASIS WS-Discovery 1.1 / ONVIF Core Specification
 * 零第三方依赖: 纯 socket + 内置 XML 字符串解析
 */

#ifndef __ONVIF_DISCOVERY_H__
#define __ONVIF_DISCOVERY_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 搜索到的设备信息 ==================== */
#define ONVIF_DISCOVERY_MAX_DEVICES     64
#define ONVIF_DISCOVERY_IP_LEN          64
#define ONVIF_DISCOVERY_NAME_LEN        128
#define ONVIF_DISCOVERY_XADDR_LEN       512
#define ONVIF_DISCOVERY_SCOPES_LEN      1024
#define ONVIF_DISCOVERY_ENDPOINT_LEN    256
#define ONVIF_DISCOVERY_TYPES_LEN       256
#define ONVIF_DISCOVERY_HARDWARE_LEN    128
#define ONVIF_DISCOVERY_LOCATION_LEN    128

typedef struct {
    char    endpoint_ref[ONVIF_DISCOVERY_ENDPOINT_LEN];  /* EndpointReference Address */
    char    types[ONVIF_DISCOVERY_TYPES_LEN];            /* 设备类型 */
    char    scopes[ONVIF_DISCOVERY_SCOPES_LEN];          /* Scopes 原始字符串 */
    char    xaddrs[ONVIF_DISCOVERY_XADDR_LEN];           /* XAddrs 地址列表 */
    char    ip[ONVIF_DISCOVERY_IP_LEN];                  /* 解析后的 IPv4 地址 */
    int     port;                                        /* 端口 */
    char    name[ONVIF_DISCOVERY_NAME_LEN];             /* 设备名称 (来自 scopes) */
    char    hardware[ONVIF_DISCOVERY_HARDWARE_LEN];      /* 硬件型号 (来自 scopes) */
    char    location[ONVIF_DISCOVERY_LOCATION_LEN];      /* 位置 (来自 scopes) */
    int     metadata_version;                            /* 元数据版本 */
} onvif_device_t;

/* ==================== 搜索状态回调 ==================== */
typedef enum {
    ONVIF_DISCOVERY_EVENT_DEVICE_FOUND = 0,  /* 发现新设备, data 为 onvif_device_t* */
    ONVIF_DISCOVERY_EVENT_SEARCH_DONE = 1,   /* 搜索完成 */
    ONVIF_DISCOVERY_EVENT_SEARCH_ERROR = 2,  /* 搜索出错 */
} onvif_discovery_event_t;

/* 回调函数: 搜索线程中回调, 不要在回调中做耗时操作 */
typedef void (*onvif_discovery_callback)(onvif_discovery_event_t event,
                                         const void *data, void *user_data);

/* ==================== API ==================== */

/* 初始化模块 (创建互斥锁等) */
int onvif_discovery_init(void);

/* 释放模块 */
void onvif_discovery_exit(void);

/* 启动异步搜索
 * timeout_ms: 收集设备响应的超时时间 (毫秒), 建议 3000~5000
 * callback: 搜索事件回调 (可为 NULL)
 * user_data: 回调透传参数
 * 返回: 0 成功, -1 已在搜索中, -2 线程创建失败
 */
int onvif_discovery_start(int timeout_ms,
                          onvif_discovery_callback callback,
                          void *user_data);

/* 停止搜索 (异步, 置标志位后线程自行退出) */
void onvif_discovery_stop(void);

/* 查询搜索状态: 1=搜索中, 0=空闲 */
int onvif_discovery_is_searching(void);

/* 获取已搜索到的设备数量 */
int onvif_discovery_get_device_count(void);

/* 获取第 index 个设备信息 (拷贝到 out)
 * 返回: 0 成功, -1 索引越界 */
int onvif_discovery_get_device(int index, onvif_device_t *out);

/* 清空设备列表 */
void onvif_discovery_clear_devices(void);

/* 获取本机 IP (eth0), 用于过滤自身 */
int onvif_discovery_get_local_ip(char *ip, int ip_len);

#ifdef __cplusplus
}
#endif

#endif /* __ONVIF_DISCOVERY_H__ */