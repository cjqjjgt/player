/* SPDX-License-Identifier: MIT */
/*
 * onvif_discovery.c - ONVIF WS-Discovery 设备搜索实现 (纯 C)
 *
 * 协议参考: OASIS WS-Discovery 1.1 / ONVIF Core Specification
 * 搜索流程:
 *   1. 创建 UDP socket, 绑定本地端口
 *   2. 加入组播组 239.255.255.250:3702
 *   3. 发送 WS-Discovery Probe 报文
 *   4. 循环接收 ProbeMatch 响应 (超时 1 秒)
 *   5. 字符串解析 XML 提取设备信息
 *   6. 去重后存入设备列表
 *   7. 超时后退出线程
 */

#include "onvif_discovery.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <net/if.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

/* ==================== 常量 ==================== */
#define ONVIF_MCAST_ADDR    "239.255.255.250"
#define ONVIF_MCAST_PORT    3702
#define ONVIF_RECV_BUF_SIZE 8192
#define ONVIF_RECV_TIMEOUT_SEC  1
#define ONVIF_RECV_TIMEOUT_USEC 0

/* ==================== 全局状态 (模块级静态变量) ==================== */
static onvif_device_t       s_devices[ONVIF_DISCOVERY_MAX_DEVICES];
static int                  s_device_count = 0;
static pthread_mutex_t      s_mutex = PTHREAD_MUTEX_INITIALIZER;

static volatile int         s_searching = 0;
static volatile int         s_stop_requested = 0;
static pthread_t            s_search_thread = 0;
static int                  s_search_timeout_ms = 5000;

static onvif_discovery_callback  s_callback = NULL;
static void                *s_callback_user_data = NULL;

/* ==================== 工具函数 ==================== */

/* 生成 UUID 字符串 (基于随机数, 用于 MessageID) */
static void make_uuid(char out[64])
{
    unsigned char rnd[16];
    int fd;
    unsigned int seed;
    int i;

    seed = (unsigned int)time(NULL) ^ (unsigned int)getpid() ^
           (unsigned int)(uintptr_t)pthread_self();

    fd = open("/dev/urandom", O_RDONLY);
    if (fd >= 0) {
        if (read(fd, rnd, sizeof(rnd)) != (int)sizeof(rnd)) {
            for (i = 0; i < 16; i++) {
                seed = seed * 1103515245u + 12345u;
                rnd[i] = (unsigned char)(seed >> 24);
            }
        }
        close(fd);
    } else {
        for (i = 0; i < 16; i++) {
            seed = seed * 1103515245u + 12345u;
            rnd[i] = (unsigned char)(seed >> 24);
        }
    }

    rnd[6] = (unsigned char)((rnd[6] & 0x0F) | 0x40);
    rnd[8] = (unsigned char)((rnd[8] & 0x3F) | 0x80);
    snprintf(out, 64,
        "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
        rnd[0], rnd[1], rnd[2], rnd[3], rnd[4], rnd[5], rnd[6], rnd[7],
        rnd[8], rnd[9], rnd[10], rnd[11], rnd[12], rnd[13], rnd[14], rnd[15]);
}

/* 获取 eth0 的 IPv4 地址 */
int onvif_discovery_get_local_ip(char *ip, int ip_len)
{
    int fd;
    struct ifreq ifr;

    if (ip == NULL || ip_len <= 0) return -1;

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return -1;

    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, "eth0", IFNAMSIZ - 1);

    if (ioctl(fd, SIOCGIFADDR, &ifr) == 0) {
        struct sockaddr_in *sin = (struct sockaddr_in *)&ifr.ifr_addr;
        const char *addr = inet_ntoa(sin->sin_addr);
        snprintf(ip, ip_len, "%s", addr ? addr : "");
        close(fd);
        return 0;
    }
    close(fd);
    return -1;
}

/* 判断是否为点分 IPv4 地址 */
static int is_ipv4(const char *host)
{
    int dots = 0;
    int digits = 0;
    const char *p;

    if (host == NULL || *host == '\0') return 0;

    for (p = host; *p != '\0'; p++) {
        if (*p == '.') {
            if (digits == 0) return 0;
            dots++;
            digits = 0;
        } else if (*p >= '0' && *p <= '9') {
            digits++;
        } else {
            return 0;
        }
    }
    return (dots == 3 && digits > 0);
}

/* 从 Scopes 字符串中提取 onvif://www.onvif.org/<key>/xxx 的值 */
static void scope_value(const char *scopes, const char *key, char *out, int out_len)
{
    const char *p;
    char key_pattern[128];
    int key_len;

    if (out == NULL || out_len <= 0) return;
    out[0] = '\0';
    if (scopes == NULL || key == NULL) return;

    snprintf(key_pattern, sizeof(key_pattern), "onvif://www.onvif.org/%s/", key);
    key_len = (int)strlen(key_pattern);

    p = scopes;
    while ((p = strstr(p, key_pattern)) != NULL) {
        const char *start = p + key_len;
        const char *end = strchr(start, ' ');
        int len;

        if (end == NULL) {
            len = (int)strlen(start);
        } else {
            len = (int)(end - start);
        }

        if (len > 0 && len < out_len) {
            memcpy(out, start, len);
            out[len] = '\0';
            return;
        }
        p = start;
    }
}

/* ==================== 简单 XML 字符串解析 ====================
 * 嵌入式环境不依赖 expat/libxml2, 用字符串搜索解析 SOAP/XML。
 * 仅处理 WS-Discovery ProbeMatch 报文, 不做完整 XML 解析。 */

/* 从 XML 文本中提取名为 tag 的元素的文本内容
 * 例: <d:Address>urn:uuid:xxx</d:Address> -> "urn:uuid:xxx"
 * 返回: 找到的文本长度, 0 表示未找到 */
static int xml_get_tag_text(const char *xml, const char *tag,
                            char *out, int out_len)
{
    char open_tag[128];
    char close_tag[128];
    const char *start, *end;
    int len;

    if (xml == NULL || tag == NULL || out == NULL || out_len <= 0) return 0;
    out[0] = '\0';

    /* 构造开始标签: 匹配 <xxx:tag> 或 <tag> */
    snprintf(open_tag, sizeof(open_tag), ">%s", tag);
    start = strstr(xml, open_tag);
    if (start == NULL) {
        /* 尝试匹配 <tag> (无命名空间前缀) */
        snprintf(open_tag, sizeof(open_tag), "<%s>", tag);
        start = strstr(xml, open_tag);
        if (start != NULL) {
            start += strlen(open_tag);
        }
    } else {
        /* 跳过 >tag> 中的 ">tag>" */
        start = strchr(start + 1, '>');
        if (start != NULL) start++;
    }

    if (start == NULL) return 0;

    /* 构造结束标签: </xxx:tag> 或 </tag> */
    snprintf(close_tag, sizeof(close_tag), "</%s", tag);
    end = strstr(start, close_tag);
    if (end == NULL) {
        /* 尝试匹配 </tag> */
        snprintf(close_tag, sizeof(close_tag), "</%s>", tag);
        end = strstr(start, close_tag);
    }

    if (end == NULL) return 0;

    len = (int)(end - start);
    if (len <= 0 || len >= out_len) return 0;

    memcpy(out, start, len);
    out[len] = '\0';
    return len;
}

/* 从 XAddrs 字符串中解析出第一个可用的 IPv4 地址和端口
 * XAddrs 格式: "http://192.168.1.100:80/onvif/device_service http://..."
 * 优先使用 IPv4 地址, 如果没有则使用 src_ip (来自 recvfrom) */
static int parse_xaddrs(const char *xaddrs, const char *src_ip,
                        onvif_device_t *dev)
{
    const char *p;
    char url[512];
    const char *url_start, *url_end;
    int url_len;
    const char *host_start, *host_end;
    const char *port_start;
    int host_len, port_len;
    char host[ONVIF_DISCOVERY_IP_LEN];
    int found = 0;

    if (xaddrs == NULL || dev == NULL) return 0;

    p = xaddrs;
    while (*p != '\0') {
        /* 跳过空白 */
        while (*p == ' ' || *p == '\t' || *p == '\n') p++;
        if (*p == '\0') break;

        url_start = p;
        url_end = strchr(p, ' ');
        if (url_end == NULL) {
            url_end = p + strlen(p);
        }
        url_len = (int)(url_end - url_start);
        if (url_len >= (int)sizeof(url)) url_len = (int)sizeof(url) - 1;

        memcpy(url, url_start, url_len);
        url[url_len] = '\0';

        /* 解析 http://host:port/path */
        if (strncmp(url, "http://", 7) == 0) {
            host_start = url + 7;
        } else {
            p = url_end;
            if (*p == ' ') p++;
            continue;
        }

        /* 跳过可能的 user:pass@ */
        {
            const char *at = strchr(host_start, '@');
            if (at != NULL) host_start = at + 1;
        }

        host_end = strchr(host_start, ':');
        if (host_end == NULL) {
            host_end = strchr(host_start, '/');
        }
        if (host_end == NULL) {
            host_end = host_start + strlen(host_start);
        }

        host_len = (int)(host_end - host_start);
        if (host_len >= (int)sizeof(host)) host_len = (int)sizeof(host) - 1;
        memcpy(host, host_start, host_len);
        host[host_len] = '\0';

        if (is_ipv4(host)) {
            snprintf(dev->ip, sizeof(dev->ip), "%s", host);
            found = 1;
        }

        /* 解析端口 */
        port_start = strchr(host_start, ':');
        if (port_start != NULL && port_start < host_end) {
            port_start++;
            port_len = 0;
            while (port_start[port_len] >= '0' && port_start[port_len] <= '9') {
                port_len++;
            }
            if (port_len > 0 && port_len < 10) {
                char port_str[10];
                memcpy(port_str, port_start, port_len);
                port_str[port_len] = '\0';
                dev->port = atoi(port_str);
            }
        }
        if (dev->port == 0) dev->port = 80;

        if (found) break;

        p = url_end;
        if (*p == ' ') p++;
    }

    /* 如果 XAddrs 中没有 IPv4, 使用 recvfrom 的源 IP */
    if (!found && src_ip != NULL) {
        snprintf(dev->ip, sizeof(dev->ip), "%s", src_ip);
        found = 1;
    }

    return found;
}

/* 解析 ProbeMatch 报文, 提取设备信息 */
static int parse_probe_match(const char *xml, const char *src_ip,
                             onvif_device_t *dev)
{
    char buf[ONVIF_DISCOVERY_SCOPES_LEN];

    if (xml == NULL || dev == NULL) return 0;

    memset(dev, 0, sizeof(*dev));

    /* 提取 Address (EndpointReference) */
    xml_get_tag_text(xml, "Address", dev->endpoint_ref,
                     sizeof(dev->endpoint_ref));

    /* 提取 Types */
    xml_get_tag_text(xml, "Types", dev->types, sizeof(dev->types));

    /* 提取 Scopes */
    xml_get_tag_text(xml, "Scopes", dev->scopes, sizeof(dev->scopes));

    /* 提取 XAddrs */
    xml_get_tag_text(xml, "XAddrs", dev->xaddrs, sizeof(dev->xaddrs));

    /* 提取 MetadataVersion */
    memset(buf, 0, sizeof(buf));
    if (xml_get_tag_text(xml, "MetadataVersion", buf, sizeof(buf)) > 0) {
        dev->metadata_version = atoi(buf);
    }

    /* 解析 XAddrs 获取 IP 和端口 */
    if (!parse_xaddrs(dev->xaddrs, src_ip, dev)) {
        return 0;
    }

    /* 从 Scopes 提取名称、硬件、位置 */
    scope_value(dev->scopes, "name", dev->name, sizeof(dev->name));
    scope_value(dev->scopes, "hardware", dev->hardware, sizeof(dev->hardware));
    scope_value(dev->scopes, "location", dev->location, sizeof(dev->location));

    return 1;
}

/* 检查设备是否重复 (同 IP:port 或同 EndpointReference) */
static int is_device_duplicate(const onvif_device_t *dev, int count)
{
    int i;

    for (i = 0; i < count; i++) {
        /* 同 EndpointReference */
        if (dev->endpoint_ref[0] != '\0' &&
            s_devices[i].endpoint_ref[0] != '\0' &&
            strcmp(dev->endpoint_ref, s_devices[i].endpoint_ref) == 0) {
            return i;  /* 返回重复设备索引 */
        }
        /* 同 IP:port */
        if (strcmp(dev->ip, s_devices[i].ip) == 0 &&
            dev->port == s_devices[i].port) {
            return i;
        }
    }
    return -1;
}

/* ==================== 搜索线程 ==================== */
static void *search_thread(void *arg)
{
    (void)arg;
    int fd;
    struct sockaddr_in local_addr;
    struct sockaddr_in mcast_addr;
    struct ip_mreq mreq;
    char probe[2048];
    int probe_len;
    char uuid[64];
    char local_ip[ONVIF_DISCOVERY_IP_LEN];
    int opt = 1;
    int ttl = 2;
    long deadline_ms;
    long waited_ms;
    int max_iterations;
    int iterations;
    int ret;

    local_ip[0] = '\0';

    /* 创建 UDP socket */
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        pthread_mutex_lock(&s_mutex);
        s_searching = 0;
        s_stop_requested = 0;
        if (s_callback) {
            s_callback(ONVIF_DISCOVERY_EVENT_SEARCH_ERROR, NULL,
                       s_callback_user_data);
        }
        pthread_mutex_unlock(&s_mutex);
        return NULL;
    }

    /* 允许重用端口 */
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(opt));

    /* 绑定本地端口 (0 = 系统分配) */
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = 0;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(fd, (struct sockaddr *)&local_addr, sizeof(local_addr)) != 0) {
        close(fd);
        pthread_mutex_lock(&s_mutex);
        s_searching = 0;
        s_stop_requested = 0;
        if (s_callback) {
            s_callback(ONVIF_DISCOVERY_EVENT_SEARCH_ERROR, NULL,
                       s_callback_user_data);
        }
        pthread_mutex_unlock(&s_mutex);
        return NULL;
    }

    /* 获取本机 IP */
    onvif_discovery_get_local_ip(local_ip, sizeof(local_ip));

    /* 加入组播组 */
    memset(&mreq, 0, sizeof(mreq));
    mreq.imr_multiaddr.s_addr = inet_addr(ONVIF_MCAST_ADDR);
    mreq.imr_interface.s_addr = local_ip[0] ? inet_addr(local_ip)
                                            : htonl(INADDR_ANY);
    if (setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                   (char *)&mreq, sizeof(mreq)) != 0) {
        printf("[ONVIF] 加入组播组失败: %s\n", strerror(errno));
    }

    /* 指定组播出口网卡 */
    if (local_ip[0]) {
        struct in_addr if_addr;
        if (inet_pton(AF_INET, local_ip, &if_addr) == 1) {
            setsockopt(fd, IPPROTO_IP, IP_MULTICAST_IF,
                       &if_addr, sizeof(if_addr));
        }
    }
    setsockopt(fd, IPPROTO_IP, IP_MULTICAST_TTL, (char *)&ttl, sizeof(ttl));

    /* 设置接收超时 */
    {
        struct timeval tv;
        tv.tv_sec = ONVIF_RECV_TIMEOUT_SEC;
        tv.tv_usec = ONVIF_RECV_TIMEOUT_USEC;
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }

    /* 组播目标 */
    memset(&mcast_addr, 0, sizeof(mcast_addr));
    mcast_addr.sin_family = AF_INET;
    mcast_addr.sin_port = htons(ONVIF_MCAST_PORT);
    mcast_addr.sin_addr.s_addr = inet_addr(ONVIF_MCAST_ADDR);

    /* 构造 Probe 报文 */
    make_uuid(uuid);
    probe_len = snprintf(probe, sizeof(probe),
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
        "<e:Envelope xmlns:e=\"http://www.w3.org/2003/05/soap-envelope\""
        " xmlns:w=\"http://schemas.xmlsoap.org/ws/2004/08/addressing\""
        " xmlns:d=\"http://schemas.xmlsoap.org/ws/2005/04/discovery\""
        " xmlns:dn=\"http://www.onvif.org/ver10/network/wsdl\">"
        "<e:Header>"
        "<w:MessageID>urn:uuid:%s</w:MessageID>"
        "<w:To>urn:schemas-xmlsoap-org:ws:2005:04:discovery</w:To>"
        "<w:Action>"
        "http://schemas.xmlsoap.org/ws/2005/04/discovery/Probe"
        "</w:Action>"
        "</e:Header>"
        "<e:Body>"
        "<d:Probe>"
        "<d:Types>dn:NetworkVideoTransmitter</d:Types>"
        "</d:Probe>"
        "</e:Body>"
        "</e:Envelope>",
        uuid);

    /* 发送 Probe */
    ret = sendto(fd, probe, probe_len, 0,
                 (struct sockaddr *)&mcast_addr, sizeof(mcast_addr));
    if (ret < 0) {
        printf("[ONVIF] 发送 Probe 失败: %s\n", strerror(errno));
        close(fd);
        pthread_mutex_lock(&s_mutex);
        s_searching = 0;
        s_stop_requested = 0;
        if (s_callback) {
            s_callback(ONVIF_DISCOVERY_EVENT_SEARCH_ERROR, NULL,
                       s_callback_user_data);
        }
        pthread_mutex_unlock(&s_mutex);
        return NULL;
    }

    printf("[ONVIF] Probe 已发送, 等待设备响应 (超时 %dms)...\n",
           s_search_timeout_ms);

    /* 收集 ProbeMatch 响应 */
    deadline_ms = (long)s_search_timeout_ms;
    waited_ms = 0;
    max_iterations = (int)(deadline_ms / 250) + 16;
    iterations = 0;

    while (waited_ms < deadline_ms && iterations < max_iterations) {
        char buf[ONVIF_RECV_BUF_SIZE];
        int n;
        struct timespec t0, t1;
        struct sockaddr_in src_addr;
        socklen_t src_len = sizeof(src_addr);
        char src_ip[ONVIF_DISCOVERY_IP_LEN];
        int stop_flag;

        /* 检查停止标志 */
        pthread_mutex_lock(&s_mutex);
        stop_flag = s_stop_requested;
        pthread_mutex_unlock(&s_mutex);
        if (stop_flag) break;

        memset(&src_addr, 0, sizeof(src_addr));
        src_ip[0] = '\0';
        clock_gettime(CLOCK_MONOTONIC, &t0);
        n = recvfrom(fd, buf, sizeof(buf) - 1, 0,
                     (struct sockaddr *)&src_addr, &src_len);
        clock_gettime(CLOCK_MONOTONIC, &t1);

        if (n > 0 && src_addr.sin_family == AF_INET) {
            inet_ntop(AF_INET, &src_addr.sin_addr, src_ip, sizeof(src_ip));
        }

        waited_ms += (t1.tv_sec - t0.tv_sec) * 1000 +
                     (t1.tv_nsec - t0.tv_nsec) / 1000000;
        iterations++;

        if (n <= 0) continue;

        buf[n] = '\0';

        /* 只处理 ProbeMatch */
        if (strstr(buf, "ProbeMatch") == NULL) continue;

        /* 解析响应 */
        {
            onvif_device_t dev;
            int dup_index;

            if (!parse_probe_match(buf, src_ip, &dev)) continue;

            pthread_mutex_lock(&s_mutex);

            /* 过滤本机 */
            if (local_ip[0] && strcmp(dev.ip, local_ip) == 0) {
                pthread_mutex_unlock(&s_mutex);
                continue;
            }

            /* 去重 */
            dup_index = is_device_duplicate(&dev, s_device_count);
            if (dup_index >= 0) {
                /* 如果之前的是主机名, 现在有 IPv4, 更新 */
                if (!is_ipv4(s_devices[dup_index].ip) && is_ipv4(dev.ip)) {
                    s_devices[dup_index] = dev;
                }
                pthread_mutex_unlock(&s_mutex);
                continue;
            }

            /* 添加到列表 */
            if (s_device_count < ONVIF_DISCOVERY_MAX_DEVICES) {
                s_devices[s_device_count] = dev;
                s_device_count++;
                printf("[ONVIF] 发现设备[%d]: %s (%s:%d)\n",
                       s_device_count - 1, dev.name[0] ? dev.name : "未知",
                       dev.ip, dev.port);

                if (s_callback) {
                    s_callback(ONVIF_DISCOVERY_EVENT_DEVICE_FOUND, &dev,
                               s_callback_user_data);
                }
            }
            pthread_mutex_unlock(&s_mutex);
        }
    }

    /* 退出组播组 */
    setsockopt(fd, IPPROTO_IP, IP_DROP_MEMBERSHIP, (char *)&mreq, sizeof(mreq));
    close(fd);

    printf("[ONVIF] 搜索完成, 共发现 %d 台设备\n", s_device_count);

    pthread_mutex_lock(&s_mutex);
    s_searching = 0;
    s_stop_requested = 0;
    if (s_callback) {
        s_callback(ONVIF_DISCOVERY_EVENT_SEARCH_DONE, NULL,
                   s_callback_user_data);
    }
    pthread_mutex_unlock(&s_mutex);

    return NULL;
}

/* ==================== 公开 API ==================== */

int onvif_discovery_init(void)
{
    /* 互斥锁已在静态初始化中完成, 这里做额外初始化 */
    s_device_count = 0;
    s_searching = 0;
    s_stop_requested = 0;
    printf("[ONVIF] 设备发现模块已初始化\n");
    return 0;
}

void onvif_discovery_exit(void)
{
    onvif_discovery_stop();
    s_device_count = 0;
    printf("[ONVIF] 设备发现模块已释放\n");
}

int onvif_discovery_start(int timeout_ms,
                          onvif_discovery_callback callback,
                          void *user_data)
{
    pthread_mutex_lock(&s_mutex);

    if (s_searching) {
        pthread_mutex_unlock(&s_mutex);
        printf("[ONVIF] 搜索已在进行中, 忽略重复启动\n");
        return -1;
    }

    s_search_timeout_ms = (timeout_ms > 0) ? timeout_ms : 5000;
    s_device_count = 0;
    s_stop_requested = 0;
    s_searching = 1;
    s_callback = callback;
    s_callback_user_data = user_data;

    pthread_mutex_unlock(&s_mutex);

    if (pthread_create(&s_search_thread, NULL, search_thread, NULL) != 0) {
        pthread_mutex_lock(&s_mutex);
        s_searching = 0;
        s_stop_requested = 0;
        pthread_mutex_unlock(&s_mutex);
        printf("[ONVIF] 搜索线程创建失败\n");
        return -2;
    }

    /* 分离线程, 搜索线程自行退出, 不阻塞主线程 */
    pthread_detach(s_search_thread);
    printf("[ONVIF] 开始搜索设备 (超时 %dms)\n", s_search_timeout_ms);
    return 0;
}

void onvif_discovery_stop(void)
{
    pthread_mutex_lock(&s_mutex);
    s_stop_requested = 1;
    pthread_mutex_unlock(&s_mutex);
    printf("[ONVIF] 搜索停止请求已发出\n");
}

int onvif_discovery_is_searching(void)
{
    int ret;
    pthread_mutex_lock(&s_mutex);
    ret = s_searching ? 1 : 0;
    pthread_mutex_unlock(&s_mutex);
    return ret;
}

int onvif_discovery_get_device_count(void)
{
    int ret;
    pthread_mutex_lock(&s_mutex);
    ret = s_device_count;
    pthread_mutex_unlock(&s_mutex);
    return ret;
}

int onvif_discovery_get_device(int index, onvif_device_t *out)
{
    int ret = -1;
    if (out == NULL) return -1;

    pthread_mutex_lock(&s_mutex);
    if (index >= 0 && index < s_device_count) {
        *out = s_devices[index];
        ret = 0;
    }
    pthread_mutex_unlock(&s_mutex);
    return ret;
}

void onvif_discovery_clear_devices(void)
{
    pthread_mutex_lock(&s_mutex);
    s_device_count = 0;
    pthread_mutex_unlock(&s_mutex);
}