/* SPDX-License-Identifier: MIT */
/*
 * audio_render.c - 音频渲染实现 (ALSA PCM)
 *
 * 参考:
 *   VLC audio_output/dec.c  - stream_GetDelay() 音频时钟算法
 *   NVR  board_hal_audio.c  - 嵌入式 ALSA PCM 播放
 *
 * 核心设计:
 *   环形缓冲区 (FIFO):
 *     - 每个 slot 包含 PCM 数据 + PTS + 采样数
 *     - 读写索引用原子操作, 无锁设计
 *   播放线程:
 *     - 从环形缓冲取数据 → snd_pcm_writei
 *     - 更新已播放采样数 → 用于计算音频时钟
 *   音频时钟算法 (参考 VLC stream_GetDelay):
 *     audio_clock = 基准PTS + (已播放采样数 / 采样率) * 1000000
 */

#include "audio_render.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <errno.h>

#include <alsa/asoundlib.h>

#define LOG_TAG "AUDIO_RENDER"
#include "common.h"

/* 环形缓冲区 slot 数
 * ★ v5.5: 从 8 减到 6 (0.8s → 0.6s). 诊断确认: 录制/本地回放时
 * 解码线程在渲染线程首写前快速填满整环 (0.8s), 之后音频解码按
 * 实时速率推进 → 环内始终滞留 ~0.8s 未播数据 → 主时钟 (audible)
 * 系统性落后视频 ~1s, 且该落后恒为常量, av_offset 补偿后仍导致
 * 音画整体错位. 减小环深 + 降低 get_clock 扣除上限 (见下), 将
 * 解码超前量压缩到 <250ms, 首帧起播延迟可接受, 同步由视频帧
 * 向主时钟对齐实现 (不再允许视频自由跑飞). */
/* ★ v5.18: 环形缓冲 slot 数对齐参考项目 (2.0sy).
 * 参考项目 16 槽 × 100ms = 1.6s 缓冲, 经板端验证音频连续无爆音.
 * 旧值 6 槽缓冲太小, 解码超前时频繁满丢弃, 破坏音频连续性,
 * 导致"嘈杂声音太大/听不到人声"的听感. */
#define RING_BUFFER_SLOTS  16

/* ★ v5.18: 移除 MASTER_DELAY_CAP_US 和 DELAY_EMA_ALPHA.
 * 参考项目 (2.0sy) 的 get_clock 是纯墙上时钟插值, 不做 delay 扣除.
 * 旧代码的 delay 扣除 (含 EMA 平滑 + CAP 上限) 导致音频时钟系统性
 * 偏低, AV-SYNC 频繁 WAIT/DROP 振荡 → 音频渲染节奏紊乱 → 嘈杂听感.
 * 对齐参考项目: 不做 delay 扣除, 音频时钟直接反映"已写入位置",
 * 同步由 AV-SYNC 的 av_offset 补偿固定延迟. */

/* 每个 slot 最大采样数 (48kHz × 100ms = 4800) */
#define MAX_SAMPLES_PER_SLOT  4800

/* ALSA period 大小 (帧数) */
#define ALSA_PERIOD_SIZE    1024

/* ALSA buffer 大小 (帧数) */
#define ALSA_BUFFER_SIZE    4096

/* 写超时 (ms) */
#define ALSA_WRITE_TIMEOUT_MS  100

/* 环形缓冲 slot */
typedef struct {
    int16_t  data[MAX_SAMPLES_PER_SLOT * 2];  /* S16LE 交错, 最大双声道 */
    int      samples;      /* 采样数 (每通道) */
    int64_t  pts;          /* 该帧的 PTS (微秒) */
    int      valid;        /* 1=有效 */
} csg_audio_render_ring_slot_t;

struct csg_audio_render_ctx {
    /* ALSA */
    snd_pcm_t      *pcm;
    char            device[64];
    int             sample_rate;
    int             channels;
    int             bits_per_sample;
    int             frame_bytes;      /* 每帧字节数 = channels * bits/8 */

    /* 环形缓冲区 */
    csg_audio_render_ring_slot_t        slots[RING_BUFFER_SLOTS];
    volatile int    write_idx;        /* 生产者写入位置 */
    volatile int    read_idx;         /* 消费者读取位置 */
    pthread_mutex_t ring_mutex;

    /* 播放线程 */
    pthread_t       thread;
    volatile int    running;
    volatile int    playing;

    /* 音频时钟 (墙上时钟插值, 不再依赖 snd_pcm_delay) */
    int64_t         base_pts;         /* 当前播放段的基准 PTS */
    int64_t         base_wall_us;     /* base_pts 对应的系统墙上时钟 */
    int64_t         last_clock;       /* 上次返回的时钟值 (单调保证) */
    int64_t         played_samples;   /* 已播放的累计采样数 */
    int64_t         clock_ema;        /* 时钟 EMA 平滑值 (过滤锚点跳变) */
    int             clock_ema_valid;  /* EMA 是否已初始化 */
    unsigned int    clock_dbg_cnt;    /* CLOCK-DBG 限频计数 */
    /* ★ v5.18: 移除 delay_ema/delay_ema_valid, 对齐参考项目.
     * 参考项目 get_clock 不做 delay 扣除, 无需 EMA 平滑. */
    pthread_mutex_t clock_mutex;

    /* ★ flush 代际计数器: flush 时递增, 音频线程检测变化后丢弃旧帧.
     * 解决竞态: 音频线程在 flush 前读出了旧 PTS 帧, flush 后继续写完
     * ALSA 并用旧 PTS 覆盖时钟, 导致 Seek 后时钟跳到上一个播放位置. */
    volatile int    flush_gen;

    /* ★ Seek 待处理标志: flush 时置 1, 音频线程首帧更新锚点后置 0.
     * 解决: adjust_clock 将时钟设为视频 PTS (临时),
     * 音频线程首帧将时钟校准为音频 PTS (永久),
     * 消除视频 PTS 与音频 PTS 的容器偏移导致的唇音不同步. */
    volatile int    seek_pending;

    /* ★ 时钟初始化完成标志: 防止 PTS=0 时 base_pts<=0 条件重复触发.
     * 问题: 音频首帧 PTS=0 → base_pts=0 → 0<=0 仍为真 → 第二帧
     * 又初始化 → base_wall_us 被重置, 墙上时钟插值丢失时间差. */
    volatile int    clock_init_done;

    /* 倍速: 音频时钟推进速度因子 (默认 1.0) */
    float           speed;

    /* 音量 */
    int             volume;           /* 0~100 */
    int             muted;

    /* 统计 */
    int64_t         total_written;
    int64_t         total_played;
    int             underrun_count;
};

/* 前向声明: 混音器自动初始化 (定义在 create 之后) */
static void csg_audio_render_init_mixer(csg_audio_render_ctx_t *ctx);

/* ★ 不区分大小写子串匹配 (替代非 POSIX 的 strcasestr)
 * 在 haystack 中搜索 needle 的子串, 忽略大小写.
 * 返回首次匹配位置的指针, 或 NULL. */
static const char *csg_stristr(const char *haystack, const char *needle)
{
    if (!haystack || !needle || !*needle) return NULL;
    size_t nlen = strlen(needle);
    for (const char *p = haystack; *p; p++) {
        size_t i;
        for (i = 0; i < nlen; i++) {
            if (tolower((unsigned char)p[i]) != tolower((unsigned char)needle[i]))
                break;
        }
        if (i == nlen) return p;
    }
    return NULL;
}

/* ★ v5.12: I2C 直接控制 NAU8822 寄存器 (绕过 DAPM 框架).
 *
 * === 使用场景 ===
 * 当 DAPM 的 Jack Detect 机制持续覆盖 hp/spk switch 控件时,
 * mixer API 和 GPIO sysfs 均无法生效. 此时通过 I2C 总线直接
 * 读写 NAU8822 寄存器, 绕过内核 DAPM 框架.
 *
 * NAU8822 I2C 地址: 0x1A (7-bit), 即 8-bit 写地址 0x34.
 *
 * 关键寄存器:
 *   0x02: Power Management 2
 *         Bit 6: DACL_EN   (左 DAC 使能)
 *         Bit 5: DACR_EN   (右 DAC 使能)
 *         Bit 3: HPL_EN    (左耳机输出使能)
 *         Bit 2: HPR_EN    (右耳机输出使能)
 *         Bit 1: SPKL_EN   (左喇叭输出使能)
 *         Bit 0: SPKR_EN   (右喇叭输出使能)
 *         全部使能: 0x6F
 *   0x2F: Jack Detect Control
 *         Bit 0: JD_EN     (0=禁用JD, 1=使能JD)
 *         禁用 JD: 0x00, DAPM 不再因"未插入"关闭输出
 *
 * === 重要提示 ===
 * I2C 写入可能被 DAPM 在下一个 workqueue 周期覆盖.
 * 先禁用 JD_EN (0x2F=0x00) 可阻止 DAPM 基于 Jack 状态覆盖输出.
 * 若仍被覆盖, 需在设备树中禁用 JD 或修改驱动. */
static int nau8822_i2c_write_reg(int i2c_bus, uint8_t reg, uint8_t val)
{
    char dev_path[32];
    snprintf(dev_path, sizeof(dev_path), "/dev/i2c-%d", i2c_bus);

    int fd = open(dev_path, O_RDWR);
    if (fd < 0) {
        CSG_LOGW("AUDIO_RENDER", "I2C: open %s failed: %s", dev_path, strerror(errno));
        return -1;
    }

    /* NAU8822 7-bit 地址 0x1A */
    if (ioctl(fd, I2C_SLAVE, 0x1A) < 0) {
        CSG_LOGW("AUDIO_RENDER", "I2C: set slave addr 0x1A failed: %s", strerror(errno));
        close(fd);
        return -1;
    }

    uint8_t buf[2] = { reg, val };
    if (write(fd, buf, 2) != 2) {
        CSG_LOGW("AUDIO_RENDER", "I2C: write reg 0x%02X=0x%02X failed: %s",
             reg, val, strerror(errno));
        close(fd);
        return -1;
    }

    close(fd);
    return 0;
}

static int nau8822_i2c_read_reg(int i2c_bus, uint8_t reg, uint8_t *val)
{
    char dev_path[32];
    snprintf(dev_path, sizeof(dev_path), "/dev/i2c-%d", i2c_bus);

    int fd = open(dev_path, O_RDWR);
    if (fd < 0) {
        return -1;
    }

    if (ioctl(fd, I2C_SLAVE, 0x1A) < 0) {
        close(fd);
        return -1;
    }

    /* 先写寄存器地址, 再读数据 */
    uint8_t reg_buf[1] = { reg };
    if (write(fd, reg_buf, 1) != 1) {
        close(fd);
        return -1;
    }

    if (read(fd, val, 1) != 1) {
        close(fd);
        return -1;
    }

    close(fd);
    return 0;
}

/* ★ v5.9: 后置 GPIO 输出使能断言 (hp switch / spk switch) — 带重试+回读验证.
 *
 * === 根因分析 ===
 * NAU8822 的 hp switch (numid=80) 和 spk switch (numid=79) 是 DAPM 管理的
 * GPIO 输出控件. 它们的实际硬件状态由 DAPM 框架根据音频路径电源状态决定.
 * snd_ctl_elem_write 虽然返回成功, 但 DAPM 在下一个电源状态转换时可能覆盖.
 *
 * 常见触发条件:
 *   1. 耳机插入检测 (Jack Detect) 认为未插入 → DAPM 关闭 hp switch
 *   2. DAPM 上电事件 (PCM 首次 writei) 后异步复位 GPIO 为默认值
 *   3. 某 DAPM widget 未正确激活导致路径不完整
 *
 * === 策略 (v5.12 增强) ===
 *   A. 先尝试 snd_ctl_elem_write (常规 ALSA mixer 控制)
 *   B. 写入后立即回读验证是否真正生效
 *   C. 若 mixer 控制无效, 尝试 GPIO sysfs 直接控制硬件引脚
 *   E. ★ v5.12: 若以上均无效, 通过 I2C 直接控制 NAU8822 寄存器
 *      - 先禁用 Jack Detect (0x2F=0x00), 阻止 DAPM 基于 JD 状态覆盖
 *      - 再使能输出放大器 (0x02=0x6F), 强制开启 DAC+HP+SPK
 *      - 回读验证, 带重试机制对抗 DAPM workqueue 异步覆盖
 *   D. 输出详细诊断信息帮助定位根因
 *
 * 必须在 DMA prime write 之后调用, 确保音频信号
 * 能通过 GPIO 到达物理输出引脚 (耳机/喇叭). */
static void csg_audio_render_assert_output_gpio(csg_audio_render_ctx_t *ctx)
{
    if (!ctx || !ctx->device[0]) return;

    int card = 0;
    const char *p = strchr(ctx->device, ':');
    if (p) card = atoi(p + 1);

    char ctl_name[32];
    snprintf(ctl_name, sizeof(ctl_name), "hw:%d", card);

    /* ===== 阶段 A: 枚举控件, 找到 hp switch 和 spk switch ===== */
    snd_ctl_elem_id_t *hp_id = NULL, *spk_id = NULL;
    snd_ctl_elem_id_t *hp_id_alloc = NULL, *spk_id_alloc = NULL;
    unsigned int hp_numid = 0, spk_numid = 0;

    snd_ctl_t *ctl = NULL;
    int ret = snd_ctl_open(&ctl, ctl_name, 0);
    if (ret < 0) {
        CSG_LOGW("AUDIO_RENDER", "GPIO assert: snd_ctl_open(hw:%d) failed: %s",
             card, snd_strerror(ret));
        return;
    }

    snd_ctl_elem_list_t *elist;
    snd_ctl_elem_list_alloca(&elist);
    snd_ctl_elem_list_alloc_space(elist, 200);

    ret = snd_ctl_elem_list(ctl, elist);
    if (ret < 0) {
        CSG_LOGW("AUDIO_RENDER", "GPIO assert: snd_ctl_elem_list failed: %s",
             snd_strerror(ret));
        snd_ctl_close(ctl);
        return;
    }

    unsigned int count = snd_ctl_elem_list_get_count(elist);

    for (unsigned int i = 0; i < count; i++) {
        snd_ctl_elem_id_t *eid;
        snd_ctl_elem_id_alloca(&eid);
        snd_ctl_elem_list_get_id(elist, i, eid);

        const char *name = snd_ctl_elem_id_get_name(eid);
        if (!name) continue;

        snd_ctl_elem_info_t *info;
        snd_ctl_elem_info_alloca(&info);
        snd_ctl_elem_info_set_id(info, eid);
        if (snd_ctl_elem_info(ctl, info) < 0) continue;
        if (snd_ctl_elem_info_get_type(info) != SND_CTL_ELEM_TYPE_BOOLEAN)
            continue;

        if (csg_stristr(name, "hp switch") && !csg_stristr(name, "headphone")) {
            hp_id_alloc = malloc(snd_ctl_elem_id_sizeof());
            if (hp_id_alloc) {
                snd_ctl_elem_id_copy(hp_id_alloc, eid);
                hp_id = hp_id_alloc;
                hp_numid = snd_ctl_elem_info_get_numid(info);
            }
        } else if (csg_stristr(name, "spk switch") && !csg_stristr(name, "speaker")) {
            spk_id_alloc = malloc(snd_ctl_elem_id_sizeof());
            if (spk_id_alloc) {
                snd_ctl_elem_id_copy(spk_id_alloc, eid);
                spk_id = spk_id_alloc;
                spk_numid = snd_ctl_elem_info_get_numid(info);
            }
        }
    }

    if (!hp_id && !spk_id) {
        CSG_LOGW("AUDIO_RENDER", "GPIO assert: no hp/spk switch found "
             "on card %d, output may be silent!", card);
        snd_ctl_close(ctl);
        return;
    }

    /* ===== 阶段 B: snd_ctl 重试写入 + 回读验证 ===== */
    #define GPIO_RETRY_MAX   3
    #define GPIO_RETRY_DELAY 15000  /* 15ms, 给 DAPM workqueue 调度时间 */

    int hp_ok = 0, spk_ok = 0;
    int hp_write_count = 0, spk_write_count = 0;

    for (int round = 0; round < GPIO_RETRY_MAX; round++) {
        if (round > 0) usleep(GPIO_RETRY_DELAY);

        /* 处理 hp switch */
        if (hp_id && !hp_ok) {
            snd_ctl_elem_value_t *val;
            snd_ctl_elem_value_alloca(&val);
            snd_ctl_elem_value_set_id(val, hp_id);

            if (snd_ctl_elem_read(ctl, val) >= 0) {
                long cur = snd_ctl_elem_value_get_boolean(val, 0);
                if (cur == 1) {
                    hp_ok = 1;
                    CSG_LOGI("AUDIO_RENDER", "GPIO assert: hp switch "
                         "already ON (round %d)", round);
                } else {
                    snd_ctl_elem_value_set_boolean(val, 0, 1);
                    ret = snd_ctl_elem_write(ctl, val);
                    hp_write_count++;
                    if (ret >= 0) {
                        /* ★ v5.9: 写入后立即回读验证 */
                        snd_ctl_elem_value_t *verify_val;
                        snd_ctl_elem_value_alloca(&verify_val);
                        snd_ctl_elem_value_set_id(verify_val, hp_id);
                        if (snd_ctl_elem_read(ctl, verify_val) >= 0) {
                            long verified = snd_ctl_elem_value_get_boolean(verify_val, 0);
                            if (verified == 1) {
                                hp_ok = 1;
                                CSG_LOGI("AUDIO_RENDER", "GPIO assert: hp switch "
                                     "OFF→ON (round %d, verified)", round);
                            } else {
                                CSG_LOGW("AUDIO_RENDER", "GPIO assert: hp switch "
                                     "OFF→ON (round %d) but DAPM reverted to OFF "
                                     "immediately! (numid=%u)", round, hp_numid);
                            }
                        } else {
                            CSG_LOGW("AUDIO_RENDER", "GPIO assert: hp switch "
                                 "write OK but verify read failed (round %d)", round);
                        }
                    } else {
                        CSG_LOGW("AUDIO_RENDER", "GPIO assert: hp switch "
                             "write failed (round %d): %s", round,
                             snd_strerror(ret));
                    }
                }
            }
        }

        /* 处理 spk switch */
        if (spk_id && !spk_ok) {
            snd_ctl_elem_value_t *val;
            snd_ctl_elem_value_alloca(&val);
            snd_ctl_elem_value_set_id(val, spk_id);

            if (snd_ctl_elem_read(ctl, val) >= 0) {
                long cur = snd_ctl_elem_value_get_boolean(val, 0);
                if (cur == 1) {
                    spk_ok = 1;
                    CSG_LOGI("AUDIO_RENDER", "GPIO assert: spk switch "
                         "already ON (round %d)", round);
                } else {
                    snd_ctl_elem_value_set_boolean(val, 0, 1);
                    ret = snd_ctl_elem_write(ctl, val);
                    spk_write_count++;
                    if (ret >= 0) {
                        /* ★ v5.9: 写入后立即回读验证 */
                        snd_ctl_elem_value_t *verify_val;
                        snd_ctl_elem_value_alloca(&verify_val);
                        snd_ctl_elem_value_set_id(verify_val, spk_id);
                        if (snd_ctl_elem_read(ctl, verify_val) >= 0) {
                            long verified = snd_ctl_elem_value_get_boolean(verify_val, 0);
                            if (verified == 1) {
                                spk_ok = 1;
                                CSG_LOGI("AUDIO_RENDER", "GPIO assert: spk switch "
                                     "OFF→ON (round %d, verified)", round);
                            } else {
                                CSG_LOGW("AUDIO_RENDER", "GPIO assert: spk switch "
                                     "OFF→ON (round %d) but DAPM reverted to OFF "
                                     "immediately! (numid=%u)", round, spk_numid);
                            }
                        } else {
                            CSG_LOGW("AUDIO_RENDER", "GPIO assert: spk switch "
                                 "write OK but verify read failed (round %d)", round);
                        }
                    } else {
                        CSG_LOGW("AUDIO_RENDER", "GPIO assert: spk switch "
                             "write failed (round %d): %s", round,
                             snd_strerror(ret));
                    }
                }
            }
        }

        if ((!hp_id || hp_ok) && (!spk_id || spk_ok)) break;
    }

    /* ===== 阶段 C: GPIO sysfs 回退 (mixer 控制无效时) ===== */
    /* ★ v5.9: 若 snd_ctl 写入后 DAPM 立即回退, 说明 DAPM 策略不允许
     * 软件控制这些 GPIO. 尝试通过 GPIO sysfs 直接控制硬件引脚.
     *
     * 注意: NAU8822 的 GPIO 可能由 codec 内部管理, 不一定导出到
     * SoC 的 GPIO sysfs. 此回退仅在 GPIO 导出到 sysfs 时生效. */
    if ((hp_id && !hp_ok) || (spk_id && !spk_ok)) {
        CSG_LOGW("AUDIO_RENDER", "GPIO assert: mixer control ineffective "
             "(hp=%d/%d spk=%d/%d), trying GPIO sysfs fallback...",
             hp_write_count, hp_ok ? 1 : 0,
             spk_write_count, spk_ok ? 1 : 0);

        /* 枚举 /sys/class/gpio 寻找 NAU8822 相关的 GPIO */
        /* 常见命名: gpiochip 基址 + 偏移, 或在设备树中定义 */
        const char *gpio_paths[] = {
            "/sys/class/gpio/gpio256/value",  /* 常见 NAU8822 hp GPIO */
            "/sys/class/gpio/gpio257/value",  /* 常见 NAU8822 spk GPIO */
            NULL
        };

        for (int i = 0; gpio_paths[i]; i++) {
            FILE *f = fopen(gpio_paths[i], "r");
            if (f) {
                fclose(f);
                /* GPIO 已导出, 尝试写入 1 */
                f = fopen(gpio_paths[i], "w");
                if (f) {
                    fprintf(f, "1");
                    fclose(f);
                    CSG_LOGI("AUDIO_RENDER", "GPIO sysfs: wrote 1 to %s",
                         gpio_paths[i]);
                    if (csg_stristr(gpio_paths[i], "gpio256") && hp_id) hp_ok = 1;
                    if (csg_stristr(gpio_paths[i], "gpio257") && spk_id) spk_ok = 1;
                }
            }
        }

        if ((hp_id && !hp_ok) || (spk_id && !spk_ok)) {
            CSG_LOGW("AUDIO_RENDER", "GPIO sysfs: no exported GPIO found "
                 "for NAU8822. Run 'diag_gpio.sh' to identify GPIO pins.");
        }
    }

    /* ===== 阶段 E: I2C 寄存器直接控制 (终极回退, v5.12) =====
     *
     * 当 mixer 控制和 GPIO sysfs 均无效时, 说明 DAPM 的 Jack Detect
     * 机制在内核驱动层持续覆盖输出使能位. 此时通过 I2C 总线直接
     * 读写 NAU8822 寄存器, 绕过内核 DAPM 框架.
     *
     * 策略 (分两步):
     *   步骤1: 禁用 Jack Detect (寄存器 0x2F → 0x00)
     *          - 阻止 DAPM 基于"未插入"状态覆盖输出使能
     *   步骤2: 强制使能输出放大器 (寄存器 0x02 → 0x6F)
     *          - Bit[6:5] DACL_EN/DACR_EN: 左右 DAC 使能
     *          - Bit[3:2] HPL_EN/HPR_EN:   左右耳机输出使能
     *          - Bit[1:0] SPKL_EN/SPKR_EN: 左右喇叭输出使能
     *
     * 带重试: DAPM workqueue 可能在 I2C 写入后异步覆盖,
     * 最多重试 3 轮, 每轮间隔 15ms.
     *
     * I2C 总线探测: 优先尝试 i2c-1 (RK3572 上 NAU8822 通常挂在 i2c1),
     * 失败后尝试 i2c-0. */
    if ((hp_id && !hp_ok) || (spk_id && !spk_ok)) {
        CSG_LOGW("AUDIO_RENDER", "GPIO assert: trying NAU8822 I2C direct "
             "register control (bypass DAPM)...");

        int i2c_bus_list[] = { 1, 0, 2, 3, -1 };
        int i2c_bus = -1;

        /* 探测可用 I2C 总线: 哪个总线能读到 NAU8822 寄存器 0x00 (chip ID) */
        for (int i = 0; i2c_bus_list[i] >= 0; i++) {
            uint8_t chip_id = 0;
            if (nau8822_i2c_read_reg(i2c_bus_list[i], 0x00, &chip_id) == 0) {
                CSG_LOGI("AUDIO_RENDER", "I2C: found NAU8822 on i2c-%d "
                     "(chip_id=0x%02X)", i2c_bus_list[i], chip_id);
                i2c_bus = i2c_bus_list[i];
                break;
            }
        }

        if (i2c_bus < 0) {
            CSG_LOGW("AUDIO_RENDER", "I2C: NAU8822 not found on any I2C bus "
                 "(tried i2c-0/1/2/3). Check: i2cdetect -y 1");
        } else {
            /* 步骤1: 禁用 Jack Detect, 阻止 DAPM 基于 JD 状态覆盖输出 */
            uint8_t jd_val = 0;
            if (nau8822_i2c_read_reg(i2c_bus, 0x2F, &jd_val) == 0) {
                CSG_LOGI("AUDIO_RENDER", "I2C: reg 0x2F (JD_CTRL) = 0x%02X "
                     "(JD_EN=%d)", jd_val, jd_val & 0x01);
                if (jd_val & 0x01) {
                    /* JD_EN=1 → 禁用 */
                    if (nau8822_i2c_write_reg(i2c_bus, 0x2F, jd_val & ~0x01) == 0) {
                        CSG_LOGI("AUDIO_RENDER", "I2C: reg 0x2F → 0x%02X "
                             "(JD_EN disabled, DAPM won't override output)",
                             jd_val & ~0x01);
                        usleep(10000);  /* 等 DAPM 处理完 */
                    }
                } else {
                    CSG_LOGI("AUDIO_RENDER", "I2C: JD_EN already disabled");
                }
            }

            /* 步骤2: 强制使能输出放大器, 带重试对抗 DAPM 异步覆盖 */
            #define I2C_RETRY_MAX   3
            #define I2C_RETRY_DELAY 15000  /* 15ms */

            int i2c_hp_ok = 0, i2c_spk_ok = 0;

            for (int round = 0; round < I2C_RETRY_MAX; round++) {
                if (round > 0) usleep(I2C_RETRY_DELAY);

                /* 读取当前 0x02 寄存器值 */
                uint8_t pm2_val = 0;
                if (nau8822_i2c_read_reg(i2c_bus, 0x02, &pm2_val) != 0) {
                    CSG_LOGW("AUDIO_RENDER", "I2C: read reg 0x02 failed (round %d)", round);
                    continue;
                }

                /* 检查输出使能位 */
                int dac_on  = (pm2_val & 0x60) == 0x60;  /* bit[6:5] */
                int hp_on   = (pm2_val & 0x0C) == 0x0C;  /* bit[3:2] */
                int spk_on  = (pm2_val & 0x03) == 0x03;  /* bit[1:0] */

                if (dac_on && hp_on && spk_on) {
                    i2c_hp_ok = 1;
                    i2c_spk_ok = 1;
                    CSG_LOGI("AUDIO_RENDER", "I2C: reg 0x02=0x%02X "
                         "(DAC+HP+SPK all ON, round %d)", pm2_val, round);
                    break;
                }

                /* 有未使能的位 → 写入 0x6F 全部使能 */
                uint8_t new_val = pm2_val | 0x6F;  /* 保留其他位, 置位输出使能 */
                if (nau8822_i2c_write_reg(i2c_bus, 0x02, new_val) == 0) {
                    CSG_LOGI("AUDIO_RENDER", "I2C: reg 0x02 0x%02X→0x%02X "
                         "(DAC+HP+SPK forced ON, round %d)",
                         pm2_val, new_val, round);

                    /* 回读验证 */
                    uint8_t verify_val = 0;
                    if (nau8822_i2c_read_reg(i2c_bus, 0x02, &verify_val) == 0) {
                        int v_dac = (verify_val & 0x60) == 0x60;
                        int v_hp  = (verify_val & 0x0C) == 0x0C;
                        int v_spk = (verify_val & 0x03) == 0x03;

                        if (v_dac && v_hp && v_spk) {
                            i2c_hp_ok = 1;
                            i2c_spk_ok = 1;
                            CSG_LOGI("AUDIO_RENDER", "I2C: reg 0x02=0x%02X "
                                 "verified (round %d)", verify_val, round);
                            break;
                        } else {
                            CSG_LOGW("AUDIO_RENDER", "I2C: reg 0x02=0x%02X "
                                 "DAPM reverted! (DAC=%d HP=%d SPK=%d, round %d)",
                                 verify_val, v_dac, v_hp, v_spk, round);
                        }
                    }
                } else {
                    CSG_LOGW("AUDIO_RENDER", "I2C: write reg 0x02 failed (round %d)", round);
                }
            }

            if (i2c_hp_ok) {
                hp_ok = 1;
                CSG_LOGI("AUDIO_RENDER", "I2C: headphone output ENABLED via register 0x02");
            }
            if (i2c_spk_ok) {
                spk_ok = 1;
                CSG_LOGI("AUDIO_RENDER", "I2C: speaker output ENABLED via register 0x02");
            }

            #undef I2C_RETRY_MAX
            #undef I2C_RETRY_DELAY

            /* 若 I2C 也失败, 输出最终诊断 */
            if (!i2c_hp_ok || !i2c_spk_ok) {
                CSG_LOGW("AUDIO_RENDER", "I2C: register control also ineffective! "
                     "DAPM is aggressively overriding reg 0x02. "
                     "Root fix: modify kernel DTS to disable Jack Detect "
                     "or patch NAU8822 driver to ignore JD status.");
            }
        }
    }

    /* ===== 阶段 D: 最终状态判定 + 诊断建议 ===== */
    if (hp_id && !hp_ok) {
        CSG_LOGE("AUDIO_RENDER", "GPIO assert: hp switch STILL OFF "
             "after %d mixer writes + GPIO sysfs fallback! "
             "Audio output to headphone will be SILENT! "
             "(numid=%u, check: amixer -c %d cset numid=%u 1)",
             hp_write_count, hp_numid, card, hp_numid);
    }
    if (spk_id && !spk_ok) {
        CSG_LOGE("AUDIO_RENDER", "GPIO assert: spk switch STILL OFF "
             "after %d mixer writes + GPIO sysfs fallback! "
             "Audio output to speaker will be SILENT! "
             "(numid=%u, check: amixer -c %d cset numid=%u 1)",
             spk_write_count, spk_numid, card, spk_numid);
    }

    /* ★ v5.9: 输出诊断建议, 帮助定位 DAPM/Jack Detect 根因 */
    if ((hp_id && !hp_ok) || (spk_id && !spk_ok)) {
        CSG_LOGE("AUDIO_RENDER", "=== DIAGNOSTIC: GPIO output blocked ===");
        CSG_LOGE("AUDIO_RENDER", "  Possible causes:");
        CSG_LOGE("AUDIO_RENDER", "  1. Jack Detect: DAPM thinks no headphone "
             "plugged → run 'amixer -c %d controls | grep -i jack'", card);
        CSG_LOGE("AUDIO_RENDER", "  2. DAPM path incomplete: check "
             "'cat /sys/kernel/debug/asoc/*/dapm/*' if debugfs mounted");
        CSG_LOGE("AUDIO_RENDER", "  3. GPIO pin conflict: check "
             "'cat /sys/kernel/debug/gpio | grep -i nau'");
        CSG_LOGE("AUDIO_RENDER", "  4. Run 'diag_gpio.sh' on target for "
             "full diagnostics");
        CSG_LOGE("AUDIO_RENDER", "=== END DIAGNOSTIC ===");
    }

    #undef GPIO_RETRY_MAX
    #undef GPIO_RETRY_DELAY

    snd_ctl_close(ctl);
    free(hp_id_alloc);
    free(spk_id_alloc);
}

csg_audio_render_ctx_t *csg_audio_render_create(int sample_rate, int channels,
                                      int bits_per_sample, const char *device)
{
    csg_audio_render_ctx_t *ctx = calloc(1, sizeof(csg_audio_render_ctx_t));
    if (!ctx) return NULL;

    ctx->sample_rate = sample_rate;
    ctx->channels = channels;
    ctx->bits_per_sample = bits_per_sample;
    ctx->frame_bytes = channels * bits_per_sample / 8;
    ctx->volume = 100; /* ★ v5.17: 用户要求 100% 音量.
                         * 80% 在弱信号录制源下音量仍偏小. */
    ctx->muted = 0;
    ctx->speed = 1.0f;

    if (device && device[0]) {
        strncpy(ctx->device, device, sizeof(ctx->device) - 1);
    } else {
        strcpy(ctx->device, "default");
    }

    pthread_mutex_init(&ctx->ring_mutex, NULL);
    pthread_mutex_init(&ctx->clock_mutex, NULL);

    /* 打开 ALSA PCM 设备:
     * 硬件约束: 探测顺序固定为 hw:1,0 → hw:0,0 → default.
     * 优先尝试调用方指定设备, 失败后按固定顺序回退 (去重),
     * 并短促退避重试 — 板端常见的"第二次无声"根因正是上一实例
     * 未完全释放/子流残留, 直接 open(hw:1,0) 返回 EBUSY. */
    static const char *s_probe_order[] = { "hw:1,0", "hw:0,0", "default" };
    const char *cands[6];
    int nc = 0;
    if (ctx->device[0]) {
        cands[nc++] = ctx->device;
    }
    for (int i = 0; i < 3; i++) {
        int dup = 0;
        for (int j = 0; j < nc; j++)
            if (!strcmp(cands[j], s_probe_order[i])) { dup = 1; break; }
        if (!dup) cands[nc++] = s_probe_order[i];
    }

    int ret = -1;
    for (int ci = 0; ci < nc; ci++) {
        ret = snd_pcm_open(&ctx->pcm, cands[ci], SND_PCM_STREAM_PLAYBACK, 0);
        if (ret == 0) {
            /* 实际打开成功, 记录最终设备名 */
            strncpy(ctx->device, cands[ci], sizeof(ctx->device) - 1);
            break;
        }
        CSG_LOGW("AUDIO_RENDER", "snd_pcm_open(%s) failed: %s",
             cands[ci], snd_strerror(ret));
        usleep(50000);  /* 50ms 退避, 等上一实例释放设备 */
    }
    if (ret < 0) {
        CSG_LOGE("AUDIO_RENDER", "all candidate devices busy/failed "
             "(tried %d): %s", nc, snd_strerror(ret));
        goto fail;
    }

    /* 配置硬件参数 */
    snd_pcm_hw_params_t *hw_params;
    snd_pcm_hw_params_alloca(&hw_params);
    snd_pcm_hw_params_any(ctx->pcm, hw_params);

    snd_pcm_access_t access = SND_PCM_ACCESS_RW_INTERLEAVED;
    snd_pcm_format_t format = (bits_per_sample == 16)
                              ? SND_PCM_FORMAT_S16_LE
                              : SND_PCM_FORMAT_S16_LE;

    ret = snd_pcm_hw_params_set_access(ctx->pcm, hw_params, access);
    if (ret < 0) goto hw_fail;

    ret = snd_pcm_hw_params_set_format(ctx->pcm, hw_params, format);
    if (ret < 0) goto hw_fail;

    ret = snd_pcm_hw_params_set_channels(ctx->pcm, hw_params, channels);
    if (ret < 0) goto hw_fail;

    unsigned int rate = sample_rate;
    int req_rate = sample_rate;
    ret = snd_pcm_hw_params_set_rate_near(ctx->pcm, hw_params, &rate, 0);
    if (ret < 0) goto hw_fail;

    ret = snd_pcm_hw_params_set_period_size_near(ctx->pcm, hw_params,
                                                  &(snd_pcm_uframes_t){ALSA_PERIOD_SIZE}, 0);
    if (ret < 0) goto hw_fail;

    ret = snd_pcm_hw_params_set_buffer_size_near(ctx->pcm, hw_params,
                                                  &(snd_pcm_uframes_t){ALSA_BUFFER_SIZE});
    if (ret < 0) goto hw_fail;

    ret = snd_pcm_hw_params(ctx->pcm, hw_params);
    if (ret < 0) {
hw_fail:
        CSG_LOGE("AUDIO_RENDER", "HW params failed: %s", snd_strerror(ret));
        snd_pcm_close(ctx->pcm);
        ctx->pcm = NULL;
        goto fail;
    }

    /* 从硬件读回实际生效的采样率 (避免 set_rate_near 协商后不一致) */
    unsigned int actual_rate = 0;
    ret = snd_pcm_hw_params_get_rate(hw_params, &actual_rate, 0);
    if (ret < 0) {
        actual_rate = rate;  /* 降级: 用 set_rate_near 返回的值 */
    }
    ctx->sample_rate = (int)actual_rate;

    CSG_LOGI("AUDIO_RENDER", "Created: device=%s, req=%dHz → actual=%uHz, %dch/%dbit",
         ctx->device, req_rate, actual_rate, channels, bits_per_sample);
    if (actual_rate != (unsigned int)req_rate) {
        CSG_LOGW("AUDIO_RENDER", "Sample rate mismatch! req=%dHz, actual=%uHz, "
             "clock may drift %+.1f%%",
             req_rate, actual_rate,
             (double)((int)actual_rate - req_rate) / req_rate * 100.0);
    }

    /* 配置软件参数 */
    snd_pcm_sw_params_t *sw_params;
    snd_pcm_sw_params_alloca(&sw_params);
    snd_pcm_sw_params_current(ctx->pcm, sw_params);

    ret = snd_pcm_sw_params_set_start_threshold(ctx->pcm, sw_params,
                                                ALSA_BUFFER_SIZE / 2);
    if (ret < 0) {
        CSG_LOGW("AUDIO_RENDER", "sw_params start_threshold failed: %s",
             snd_strerror(ret));
    }

    ret = snd_pcm_sw_params(ctx->pcm, sw_params);
    if (ret < 0) {
        CSG_LOGW("AUDIO_RENDER", "sw_params failed: %s", snd_strerror(ret));
    }

    /* 自动初始化混音器, 使能耳机/喇叭输出路径 */
    csg_audio_render_init_mixer(ctx);

    /* ★ v5.14: 移除 GPIO assert 调用.
     *
     * 根因 (参考 rk3572_player_nvr_2.0sy 可正常播放):
     *   GPIO assert 内部通过 snd_ctl_elem_read 回读控件值,
     *   这会触发 DAPM 路径重算. DAPM 重算时 Jack Detect 检测到
     *   "未插入" → 覆盖 hp/spk switch 为 OFF.
     *   GPIO assert 发现 OFF → 写入 ON → 再次触发 DAPM 重算 →
     *   再次被覆盖 OFF → 3 轮重试全部失败 → 死循环.
     *
     *   参考项目完全不做 GPIO assert, mixer 初始化后直接信任
     *   ALSA 控件写入结果. 音频线程首次 writei 触发 DAPM 上电,
     *   DAPM 自然使能路径上全部 widget (含 hp/spk).
     *
     *   若 DAPM 上电后仍无声 (罕见), 需检查内核 DTS 的 Jack Detect
     *   配置, 而非在用户空间与 DAPM 对抗. */

    return ctx;

fail:
    pthread_mutex_destroy(&ctx->ring_mutex);
    pthread_mutex_destroy(&ctx->clock_mutex);
    free(ctx);
    return NULL;
}

/* ==================== 混音器自动初始化 ====================
 * RK3572 + NAU8822: 上电后默认所有输出路径静音/关闭,
 * 必须通过 amixer 使能才能听到声音. 此函数自动完成配置,
 * 避免每次手动运行 amixer 命令.
 *
 * ★ 两层处理:
 *   第1层: 使能输出开关 (Headphone/Speaker Switch, ZC, DAC 等)
 *   第2层: 设置输出音量 (Headphone/Speaker Playback Volume)
 *   旧方案只处理开关, 音量默认 0 导致无声.
 *
 * 支持的控件 (按实际 DTS 命名):
 *   Headphone / Headphone Switch → 耳机输出总开关
 *   hp switch                    → 耳机 GPIO 开关
 *   Speaker / Speaker Switch     → 喇叭输出总开关
 *   spk switch                   → 喇叭 GPIO 开关
 *   Headphone ZC                 → 耳机零交叉 (防爆音)
 *   Speaker ZC                   → 喇叭零交叉 (防爆音)
 *   DAC 128x Oversampling        → DAC 过采样 (提升音质)
 *   Headphone Playback Volume    → 耳机音量 (关键!)
 *   Speaker Playback Volume      → 喇叭音量 (关键!)
 *   Master Playback Volume       → 主音量 (兜底)
 *   PCM Playback Volume          → PCM 音量 (兜底)
 *
 * 策略: 遍历所有控件, 子串不区分大小写匹配.
 * 不存在的控件静默跳过, 兼容不同板卡配置. */
static void csg_audio_render_init_mixer(csg_audio_render_ctx_t *ctx)
{
    if (!ctx || !ctx->device[0]) return;

    /* 从设备名解析声卡号: "hw:1,0" → card=1, "hw:0,0" → card=0 */
    int card = 0;
    const char *p = strchr(ctx->device, ':');
    if (p) {
        card = atoi(p + 1);
    }
    /* "default" 设备无法解析, card 保持 0 (通常是正确的) */

    snd_ctl_t *ctl = NULL;
    char ctl_name[32];
    snprintf(ctl_name, sizeof(ctl_name), "hw:%d", card);

    /* ★ v5.13: 移除 snd_pcm_drop + snd_pcm_prepare!
     *
     * 根因分析 (参考 rk3572_player_nvr_2.0sy 可正常播放):
     *   snd_pcm_drop → DAPM 下电, snd_pcm_prepare → DAPM 上电准备.
     *   上电过程中 NAU8822 的 Jack Detect 检测到"未插入",
     *   立即覆盖 hp switch / spk switch 为 OFF.
     *   后续 mixer 写入 hp/spk ON 后, DAPM 在下一个 workqueue 周期
     *   又因 JD 状态覆盖回 OFF → 输出放大器始终关闭 → "嗯嗯"声.
     *
     * 参考项目 (2.0sy) 不做 drop/prepare, mixer 在 create 时设置一次,
     * 音频线程首次 writei 自然触发 DAPM 上电, 此时 hp/spk 已是 ON,
     * DAPM 不会覆盖 → 正常播放人声.
     *
     * 若前次非正常退出导致硬件/驱动缓存不一致, 由 csg_audio_render_start
     * 中的 snd_pcm_prepare 统一处理, 不在此处引入危险的 DAPM 重置. */

    int ret = snd_ctl_open(&ctl, ctl_name, 0);
    if (ret < 0) {
        CSG_LOGW("AUDIO_RENDER", "snd_ctl_open(%s) failed: %s, "
             "mixer auto-config skipped", ctl_name, snd_strerror(ret));
        return;
    }

    CSG_LOGI("AUDIO_RENDER", "Mixer init: card=%d (%s)", card, ctl_name);

    /* ---- 第1层: 输出使能开关 (不区分大小写子串匹配) ----
     * 覆盖 NAU8822 全部模拟输出路径:
     *   Headphone/Speaker Switch → 总开关
     *   Headphone/Speaker Playback Switch → 播放开关
     *   hp/spk switch → GPIO 开关
     *   Headphone/Speaker ZC → 零交叉防爆音
     *   DAC 128x Oversampling → 提升音质
     *   Output Mixer LDAC/RDAC → DAC 到输出混音器
     *   Clk Auto → SAI 时钟自动开关 (重要!)
     *   AUXOUT Playback → 辅助输出
     *   DAC Limiter → DAC 限幅器 */
    static const char *s_enable_keys[] = {
        "headphone switch",       /* 耳机总开关 (numid=81) */
        "headphone playback",     /* 耳机播放开关 (numid=65) */
        "hp switch",              /* 耳机 GPIO 开关 (numid=80) */
        "speaker switch",         /* 喇叭总开关 (numid=82) */
        "speaker playback",       /* 喇叭播放开关 (numid=68) */
        "spk switch",             /* 喇叭 GPIO 开关 (numid=79) */
        "headphone zc",           /* 耳机零交叉防爆音 (numid=64) */
        "speaker zc",             /* 喇叭零交叉防爆音 (numid=67) */
        "dac 128x oversampling",  /* DAC 过采样 (numid=74) */
        "dac oversampling",       /* 兜底 */
        "output mixer ldac",      /* 左声道DAC→输出混音器 (numid=87) */
        "output mixer rdac",      /* 右声道DAC→输出混音器 (numid=91) */
        "clk auto",               /* SAI时钟自动 (numid=34, 关键!) */
        "auxout playback",        /* 辅助输出播放 (numid=70) */
        "dac limiter switch",     /* DAC限幅器 (numid=47) */
        "dac playback",           /* DAC 播放使能 (兜底) */
        "playback switch",        /* 播放总开关 (兜底) */
        "pcm playback",           /* PCM 播放开关 (兜底) */
        "dac switch",             /* DAC 开关 (兜底) */
        "playback deemphasis",    /* 去加重 (兜底) */
        /* ★ v5.14: 移除 filter/Jack Detect 控件.
         *   参考项目 (2.0sy) 不操作这些控件, 写入它们会触发
         *   DAPM 路径重算, 导致 hp/spk switch 被 Jack Detect 覆盖.
         *   滤波功能由 NAU8822 硬件默认配置处理, 无需用户空间干预. */
        NULL
    };

    /* ★ 显式强制关闭的控件: 输入混音类开关 (Left/Right Input Mixer L2/R2、
     *   Output Mixer LINMIX/RINMIX、AUX 混音等) 会把 LINE_IN/MIC 等外部
     *   输入信号混入 DAC 输出, 导致播放器能听到"现场回音"而不是视频原声.
     *   历史踩坑: 宽松子串匹配 "input mixer"/"output mixer linmix" 误把
     *   输入混音当输出使能打开, 板端出现明显回声. 这里显式 OFF 兜底. */
    static const char *s_disable_keys[] = {
        "input mixer",            /* 输入混音器总开关 */
        "output mixer linmix",    /* 左输入→输出混音 */
        "output mixer rinmix",    /* 右输入→输出混音 */
        "aux1 output mixer",      /* AUX1 输出混音器 (含输入混音路径) */
        "aux2 output mixer",      /* AUX2 输出混音器 (含输入混音路径) */
        /* ★ v5.14: 精简为与参考项目 (2.0sy) 完全一致.
         *   移除交叉连接禁用 (left/right output mixer rdac/ldac),
         *   移除 HPF 禁用, 移除 linmix/rinmix 子开关.
         *   参考项目只禁用 5 个控件, 已足够消除输入混音回音. */
        NULL
    };

    /* ---- 第1.5层: ENUMERATED 枚举型开关 (非布尔型, 需特殊处理) ----
     * SAI1 Transmit Mono Switch: 使能 SAI 发送通道, 否则 PCM 数据
     * 无法从 CPU 传输到 NAU8822 编解码器. */
    static const char *s_enum_keys[] = {
        "transmit mono",           /* SAI1 Transmit Mono Switch (numid=7) */
        /* ★ v5.14: 精简为与参考项目 (2.0sy) 完全一致.
         *   移除 Jack Detect 和 filter 枚举控件,
         *   避免触发 DAPM 路径重算. */
        NULL
    };

    /* ---- 第2层: 输出音量控件 (设为最大值)
     * ★ v5.17: 用户要求 100% 音量. ---- */
    static const char *s_volume_keys[] = {
        "headphone volume",        /* 耳机音量 (numid=66, 最优先) */
        "speaker volume",          /* 喇叭音量 (numid=69) */
        "pcm volume",              /* PCM 数字音量 (numid=42) */
        "dac limiter volume",      /* DAC 限幅器音量 (numid=51) */
        "master playback volume",  /* 主音量 (兜底) */
        "master volume",           /* 简化命名 (兜底) */
        NULL
    };

    int switch_count = 0;
    int volume_count = 0;

    /* 遍历混音器所有控件 (NAU8822 约 100 个, 200 足够) */
    snd_ctl_elem_list_t *elist;
    snd_ctl_elem_list_alloca(&elist);
    snd_ctl_elem_list_alloc_space(elist, 200);

    ret = snd_ctl_elem_list(ctl, elist);
    if (ret < 0) {
        CSG_LOGW("AUDIO_RENDER", "snd_ctl_elem_list failed: %s", snd_strerror(ret));
        snd_ctl_close(ctl);
        return;
    }

    unsigned int count = snd_ctl_elem_list_get_count(elist);
    CSG_LOGI("AUDIO_RENDER", "Mixer: %u controls on card %d", count, card);

    for (unsigned int i = 0; i < count; i++) {
        snd_ctl_elem_id_t *eid;
        snd_ctl_elem_id_alloca(&eid);
        snd_ctl_elem_list_get_id(elist, i, eid);

        const char *name = snd_ctl_elem_id_get_name(eid);
        unsigned int idx = snd_ctl_elem_id_get_index(eid);

        /* 获取控件类型 */
        snd_ctl_elem_info_t *info;
        snd_ctl_elem_info_alloca(&info);
        snd_ctl_elem_info_set_id(info, eid);
        ret = snd_ctl_elem_info(ctl, info);
        if (ret < 0) continue;

        snd_ctl_elem_type_t etype = snd_ctl_elem_info_get_type(info);

        /* ---- 第1层: 布尔型开关 ---- */
        if (etype == SND_CTL_ELEM_TYPE_BOOLEAN) {
            /* ★ 先检查是否在禁用列表: 输入混音类开关显式强制 OFF,
             *   防止宽松子串匹配误把输入混音当输出使能打开导致回音. */
            int is_disable = 0;
            for (int k = 0; s_disable_keys[k]; k++) {
                if (csg_stristr(name, s_disable_keys[k])) {
                    is_disable = 1;
                    break;
                }
            }
            if (is_disable) {
                snd_ctl_elem_value_t *val;
                snd_ctl_elem_value_alloca(&val);
                snd_ctl_elem_value_set_id(val, eid);
                ret = snd_ctl_elem_read(ctl, val);
                if (ret < 0) continue;
                long cur = snd_ctl_elem_value_get_boolean(val, idx);
                if (cur != 0) {
                    snd_ctl_elem_value_set_boolean(val, idx, 0);
                    ret = snd_ctl_elem_write(ctl, val);
                    if (ret >= 0) {
                        CSG_LOGI("AUDIO_RENDER", "Mixer SW: %s[%u] ON→OFF "
                             "(input-mix disabled)", name, idx);
                        switch_count++;
                    }
                }
                continue;
            }

            /* 不区分大小写子串匹配 */
            int found = 0;
            for (int k = 0; s_enable_keys[k]; k++) {
                if (csg_stristr(name, s_enable_keys[k])) {
                    found = 1;
                    break;
                }
            }
            if (!found) continue;

            snd_ctl_elem_value_t *val;
            snd_ctl_elem_value_alloca(&val);
            snd_ctl_elem_value_set_id(val, eid);
            ret = snd_ctl_elem_read(ctl, val);
            if (ret < 0) continue;

            long cur = snd_ctl_elem_value_get_boolean(val, idx);
            /* ★ v5.15: 仅在 cur==0 时写入 ON, 不做 re-assert.
             * 参考 rk3572_player_nvr_2.0sy (可正常播放人声):
             *   每次 snd_ctl_elem_write 都会触发内核控件通知,
             *   可能引起 DAPM 路径重算. re-assert 写入已为 ON 的
             *   控件会导致 DAPM 在重算时因 Jack Detect "未插入"
             *   而覆盖 hp/spk switch → 输出放大器关闭 → "嗯嗯"声.
             *   参考项目只写 OFF→ON, 不做 ON→ON re-assert,
             *   其 mixer 日志中无任何 "re-assert" 信息. */
            if (cur == 0) {
                snd_ctl_elem_value_set_boolean(val, idx, 1);
                ret = snd_ctl_elem_write(ctl, val);
                if (ret >= 0) {
                    CSG_LOGI("AUDIO_RENDER", "Mixer SW: %s[%u] OFF→ON", name, idx);
                    switch_count++;
                } else {
                    CSG_LOGW("AUDIO_RENDER", "Mixer SW: %s[%u] write failed: %s",
                         name, idx, snd_strerror(ret));
                }
            }
            continue;
        }

        /* ---- 第1.5层: 枚举型开关 (SAI1 Transmit Mono 等) ----
         * ENUMERATED 类型不是布尔型, 需要用 snd_ctl_elem_value_get_enumerated.
         * 匹配到关键字后, 查找 "Enable" 或 "On" 选项并设置.
         *
         * ★ v5.9: Jack Detect 控件的目标值是 "Inserted" (插入),
         *   不是 "Enable"/"On". 扩大搜索范围, 覆盖所有常见枚举项. */
        if (etype == SND_CTL_ELEM_TYPE_ENUMERATED) {
            int found = 0;
            for (int k = 0; s_enum_keys[k]; k++) {
                if (csg_stristr(name, s_enum_keys[k])) {
                    found = 1;
                    break;
                }
            }
            if (!found) continue;

            unsigned int items = snd_ctl_elem_info_get_items(info);
            int enable_idx = -1;

            /* 遍历所有枚举项, 按优先级查找:
             *   1. "Inserted" (Jack Detect 插入状态)
             *   2. "Enable" / "On" (通用使能)
             *   3. "Sharp" / "Fast" (滤波器模式, 最佳频响平坦度)
             *   4. "Yes" / "True" (兜底) */
            for (unsigned int item = 0; item < items; item++) {
                snd_ctl_elem_info_set_item(info, item);
                ret = snd_ctl_elem_info(ctl, info);
                if (ret < 0) continue;
                const char *item_name = snd_ctl_elem_info_get_item_name(info);
                if (!item_name) continue;
                if (strcasecmp(item_name, "Inserted") == 0 ||
                    strcasecmp(item_name, "Enable") == 0 ||
                    strcasecmp(item_name, "On") == 0 ||
                    strcasecmp(item_name, "Sharp") == 0 ||
                    strcasecmp(item_name, "Fast") == 0) {
                    enable_idx = (int)item;
                    break;
                }
            }

            if (enable_idx < 0) {
                CSG_LOGI("AUDIO_RENDER", "Mixer ENUM: %s[%u] no matching option "
                     "(searched: Inserted/Enable/On/Sharp/Fast), skip",
                     name, idx);
                continue;
            }

            snd_ctl_elem_value_t *val;
            snd_ctl_elem_value_alloca(&val);
            snd_ctl_elem_value_set_id(val, eid);
            ret = snd_ctl_elem_read(ctl, val);
            if (ret < 0) continue;

            long cur = snd_ctl_elem_value_get_enumerated(val, idx);
            /* ★ v5.15: 仅在 cur != enable_idx 时写入, 不做 re-assert.
             * 与布尔开关同理 (见上方 v5.15 注释).
             * 参考项目 (2.0sy) 不会对已为目标值的控件强制写入,
             * 避免触发 DAPM 路径重算. */
            if (cur != enable_idx) {
                snd_ctl_elem_value_set_enumerated(val, idx, enable_idx);
                ret = snd_ctl_elem_write(ctl, val);

                /* 获取目标值的名称用于日志 */
                snd_ctl_elem_info_set_item(info, enable_idx);
                snd_ctl_elem_info(ctl, info);
                const char *target_name = snd_ctl_elem_info_get_item_name(info);
                if (!target_name) target_name = "?";

                if (ret >= 0) {
                    CSG_LOGI("AUDIO_RENDER", "Mixer ENUM: %s[%u] → %s",
                         name, idx, target_name);
                    switch_count++;
                } else {
                    CSG_LOGW("AUDIO_RENDER", "Mixer ENUM: %s[%u] write failed: %s",
                         name, idx, snd_strerror(ret));
                }
            }
            continue;
        }

        /* ---- 第2层: 整数型音量 ---- */
        if (etype == SND_CTL_ELEM_TYPE_INTEGER) {
            int found = 0;
            for (int k = 0; s_volume_keys[k]; k++) {
                if (csg_stristr(name, s_volume_keys[k])) {
                    found = 1;
                    break;
                }
            }
            if (!found) continue;

            long vol_min = snd_ctl_elem_info_get_min(info);
            long vol_max = snd_ctl_elem_info_get_max(info);
            long vol_range = vol_max - vol_min;
            long vol_target;

            /* ★ 范围 ≤ 2 的控件 (如 0-1 布尔型音量) 直接设为最大值,
             * 否则 100% 计算: 0 + 1*100/100 = 0 → 等价于静音.
             * 范围 > 2 的正常音量控件设为 100%.
             * ★ v5.17: 用户要求 100% 音量. */
            if (vol_range <= 2) {
                vol_target = vol_max;
            } else {
                vol_target = vol_min + vol_range * 100 / 100;  /* 100% */
            }

            snd_ctl_elem_value_t *val;
            snd_ctl_elem_value_alloca(&val);
            snd_ctl_elem_value_set_id(val, eid);
            ret = snd_ctl_elem_read(ctl, val);
            if (ret < 0) continue;

            /* 双声道都设置 */
            for (unsigned int ch = 0; ch <= idx; ch++) {
                long cur = snd_ctl_elem_value_get_integer(val, ch);
                if (cur != vol_target) {
                    snd_ctl_elem_value_set_integer(val, ch, vol_target);
                }
            }
            ret = snd_ctl_elem_write(ctl, val);
            if (ret >= 0) {
                CSG_LOGI("AUDIO_RENDER", "Mixer VOL: %s[%u] → %ld (range %ld~%ld, 100%%)",
                     name, idx, vol_target, vol_min, vol_max);
                volume_count++;
            } else {
                CSG_LOGW("AUDIO_RENDER", "Mixer VOL: %s[%u] write failed: %s",
                     name, idx, snd_strerror(ret));
            }
        }
    }

    snd_ctl_elem_list_free_space(elist);
    snd_ctl_close(ctl);

    if (switch_count == 0 && volume_count == 0) {
        CSG_LOGW("AUDIO_RENDER", "Mixer: NO controls matched! "
             "Check 'amixer -c %d controls' for actual names.", card);
    } else {
        CSG_LOGI("AUDIO_RENDER", "Mixer done: %d switches enabled, %d volumes set",
             switch_count, volume_count);
    }
}

void csg_audio_render_destroy(csg_audio_render_ctx_t *ctx)
{
    if (!ctx) return;

    csg_audio_render_stop(ctx);

    if (ctx->pcm) {
        /* ★★★ v5.9 关键修复: SAIP DMA 五步硬件复位序列 ★★★
         *
         * === 根因 ===
         * snd_pcm_drop 只是停止 DMA 传输, 但 SAIP DMA 控制器内部
         * 状态机 (FIFO 指针、水位线、burst 计数器) 并未完整复位.
         * 直接 snd_pcm_close 在 DMA 非空闲状态下关闭, 导致:
         *   1. 第二次 snd_pcm_open 虽然成功 (不报 EBUSY), 但 DMA
         *      控制器处于脏状态, writei 数据无法真正输出到 I2S 总线
         *   2. NAU8822 codec 收不到有效 I2S 时钟/数据, DAPM 自动
         *      关闭 hp switch / spk switch GPIO 输出
         *   3. 表现为: 第二次播放无声, 且后续 aplay 也无声,
         *      必须重启才能恢复
         *
         * === 修复 ===
         * snd_pcm_prepare() 是 SAIP DMA 控制器的完整硬件复位指令:
         *   1. 复位 DMA FIFO 读写指针
         *   2. 清除 underrun/overrun 状态
         *   3. 重新配置 burst 长度和水位线
         *   4. 将 DMA 状态机从 SETUP 拉到 PREPARED (READY)
         * 然后 snd_pcm_drop 回到 SETUP, snd_pcm_hw_free 释放
         * 硬件资源, 最后 snd_pcm_close 干净关闭.
         *
         * 这是瑞芯微 BSP 中 SAIP DMA 释放的标准五步序列,
         * 缺失 snd_pcm_prepare 是导致"第二次无声"的经典踩坑点. */

        /* 步骤1: snd_pcm_drop — 确保 DMA 已停止 (stop 中已调用, 此处兜底) */
        snd_pcm_drop(ctx->pcm);

        /* 步骤2: snd_pcm_prepare — ★核心★ 复位 SAIP DMA 控制器硬件状态机 */
        {
            int ret = snd_pcm_prepare(ctx->pcm);
            if (ret < 0) {
                CSG_LOGW("AUDIO_RENDER", "snd_pcm_prepare failed during destroy: %s "
                     "(DMA may not be fully reset, trying hw_free anyway)",
                     snd_strerror(ret));
            } else {
                CSG_LOGI("AUDIO_RENDER", "SAIP DMA prepared (hardware reset OK)");
            }
        }

        /* 步骤3: snd_pcm_drop — 从 PREPARED 回到 SETUP */
        snd_pcm_drop(ctx->pcm);

        /* 步骤4: snd_pcm_hw_free — 释放 DMA 通道、I2S 时钟等硬件资源 */
        {
            int ret = snd_pcm_hw_free(ctx->pcm);
            if (ret < 0) {
                CSG_LOGW("AUDIO_RENDER", "snd_pcm_hw_free failed: %s",
                     snd_strerror(ret));
            } else {
                CSG_LOGI("AUDIO_RENDER", "SAIP DMA hw_free OK");
            }
        }

        /* 步骤5: snd_pcm_close — 关闭 PCM 句柄 */
        snd_pcm_close(ctx->pcm);
        ctx->pcm = NULL;

        CSG_LOGI("AUDIO_RENDER", "SAIP DMA fully released (5-step reset sequence)");
    }

    pthread_mutex_destroy(&ctx->ring_mutex);
    pthread_mutex_destroy(&ctx->clock_mutex);

    CSG_LOGI("AUDIO_RENDER", "Destroyed (underruns=%d)", ctx->underrun_count);
    free(ctx);
}

static void *csg_audio_render_thread(void *arg)
{
    csg_audio_render_ctx_t *ctx = (csg_audio_render_ctx_t *)arg;

    CSG_LOGI("AUDIO_RENDER", "Thread started");

    /* 预缓冲: 等待环形缓冲积累至少 2 个 slot 再开始写 ALSA */
    while (ctx->running && ctx->playing) {
        pthread_mutex_lock(&ctx->ring_mutex);
        int avail = (ctx->write_idx - ctx->read_idx + RING_BUFFER_SLOTS)
                    % RING_BUFFER_SLOTS;
        pthread_mutex_unlock(&ctx->ring_mutex);
        if (avail >= 2) break;
        usleep(10000);
    }

    while (ctx->running) {
        int has_data = 0;
        int16_t *out_buf = NULL;
        int out_samples = 0;
        int64_t out_pts = 0;

        /* ★ 记录当前代际: flush 后会递增, 用于检测旧帧.
         * 音频线程在 flush 前读出的帧会在 flush 后检测到
         * 代际变化, 从而丢弃旧帧, 不写 ALSA, 不更新时钟. */
        int saved_gen = ctx->flush_gen;

        pthread_mutex_lock(&ctx->ring_mutex);

        /* ★ 跳过被生产者作废的 slot: 缓冲区满时丢弃最旧帧会把 read_idx
         * 推进并置 valid=0 (见 csg_audio_render_write). 若不跳过, 此处
         * read_idx != write_idx 恒真但 slot 无效, 消费者既不写 ALSA 也
         * 不进静音分支 (has_new 恒真), 死循环卡死 → 本地播放完全无声.
         * 跳过无效 slot 后要么命中下一个有效帧, 要么 read_idx 追上
         * write_idx 走正常空缓冲分支. */
        while (ctx->read_idx != ctx->write_idx &&
               !ctx->slots[ctx->read_idx].valid) {
            ctx->read_idx = (ctx->read_idx + 1) % RING_BUFFER_SLOTS;
        }

        if (ctx->read_idx != ctx->write_idx) {
            csg_audio_render_ring_slot_t *slot = &ctx->slots[ctx->read_idx];
            if (slot->valid) {
                out_buf = slot->data;
                out_samples = slot->samples;
                out_pts = slot->pts;
                slot->valid = 0;
                ctx->read_idx = (ctx->read_idx + 1) % RING_BUFFER_SLOTS;
                has_data = 1;
            }
        }
        pthread_mutex_unlock(&ctx->ring_mutex);

        if (has_data && out_buf && out_samples > 0) {
            /* ★ 代际检查: 若 flush 在读帧之后发生, 此帧属于旧位置,
             * 丢弃不写 ALSA, 防止旧音频数据污染输出. */
            if (ctx->flush_gen != saved_gen) {
                CSG_LOGI("AUDIO_RENDER", "Discard stale frame pts=%lld "
                     "(gen %d->%d)", (long long)out_pts,
                     saved_gen, ctx->flush_gen);
                continue;
            }

            /* 写前准备: 检测 ALSA 状态 */
            snd_pcm_state_t state = snd_pcm_state(ctx->pcm);
            if (state == SND_PCM_STATE_XRUN) {
                snd_pcm_prepare(ctx->pcm);
                ctx->underrun_count++;
                CSG_LOGW("AUDIO_RENDER", "XRUN recovered (#%d)", ctx->underrun_count);
            } else if (state == SND_PCM_STATE_SUSPENDED) {
                snd_pcm_resume(ctx->pcm);
            }

            /* 硬件音量调节 */
            if (ctx->muted) {
                memset(out_buf, 0, out_samples * ctx->frame_bytes);
            } else if (ctx->volume < 100) {
                float vol = ctx->volume / 100.0f;
                for (int i = 0; i < out_samples * ctx->channels; i++) {
                    out_buf[i] = (int16_t)(out_buf[i] * vol);
                }
            }

            /* ★ 倍速音频处理: speed > 1.0 时压缩采样实现快放效果.
             * ★ v5.1 抗混叠修复: 2x 使用 2-采样移动平均滤波, 避免简单丢弃
             *   导致的频谱混叠 (声音刺耳/嘈杂). 公式: out[i] = avg(in[i*2], in[i*2+1]).
             *   ≥4x 静音路径不经过此代码, 无需处理.
             *   原理: 每隔 step 个采样保留 1 个, 步长 = (int)speed.
             *   例: 2x → 每 2 个采样平均为 1 个 (抗混叠)
             *       4x → 每 4 个采样保留 1 个 (丢弃 3/4)
             *       16x → 每 16 个采样保留 1 个 (丢弃 15/16)
             *
             *   注意: 原位修改, 不分配新内存;
             *         步长向下取整, 非整数倍速 (如 1.5x) 也兼容.
             *
             *   时钟影响: 当前时钟是纯墙上插值 (base_pts + delta*speed),
             *   丢弃采样后音频以 speed 倍率播放, 时钟以 speed 倍率推进,
             *   两者天然匹配, 无需额外时钟调整. */
            if (ctx->speed > 1.0f && out_samples > 0) {
                int step = (int)(ctx->speed + 0.5f);  /* 四舍五入取整 */
                if (step < 2) step = 2;
                int ch = ctx->channels;
                int total_out = out_samples / step;
                if (total_out > 0) {
                    /* ★ v5.1: 2x 使用移动平均抗混叠滤波,
                     * 避免直接丢弃采样导致的频谱混叠 (声音刺耳). */
                    if (step == 2) {
                        for (int i = 0; i < total_out; i++) {
                            for (int c = 0; c < ch; c++) {
                                int32_t s1 = out_buf[i * 2 * ch + c];
                                int32_t s2 = out_buf[(i * 2 + 1) * ch + c];
                                out_buf[i * ch + c] = (int16_t)((s1 + s2) / 2);
                            }
                        }
                    } else {
                        for (int i = 0; i < total_out; i++) {
                            for (int c = 0; c < ch; c++) {
                                out_buf[i * ch + c] = out_buf[i * step * ch + c];
                            }
                        }
                    }
                    out_samples = total_out;
                } else {
                    /* 帧太短 (不足 step 个采样), 丢弃整个帧 */
                    continue;
                }
            }

            /* 写入 ALSA */
            snd_pcm_sframes_t frames = snd_pcm_writei(ctx->pcm, out_buf,
                                                       out_samples);
            if (frames < 0) {
                /* ★ v4.0: 根据错误码分类处理
                 * - EBADFD: PCM 在错误状态 (flush 改变了设备状态),
                 *   直接 prepare 恢复, 不退避, 不断开
                 * - EPIPE: 欠载 (underrun), recover 自动 prepare
                 * - 其他: 尝试 recover, 失败则 prepare 完整复位 */
                if (frames == -EBADFD) {
                    /* Seek/Flush 导致的设备状态变更, prepare 即可恢复 */
                    snd_pcm_prepare(ctx->pcm);
                    ctx->underrun_count++;
                    /* 回退 read_idx, 下次重试 */
                    pthread_mutex_lock(&ctx->ring_mutex);
                    ctx->read_idx = (ctx->read_idx - 1 + RING_BUFFER_SLOTS)
                                    % RING_BUFFER_SLOTS;
                    ctx->slots[ctx->read_idx].valid = 1;
                    pthread_mutex_unlock(&ctx->ring_mutex);
                    usleep(5000);  /* 短退避 5ms, 让 flush 的 silence 写入完成 */
                    continue;
                }
                /* 欠载或错误, 恢复 */
                frames = snd_pcm_recover(ctx->pcm, frames, 1);
                if (frames < 0) {
                    CSG_LOGW("AUDIO_RENDER", "snd_pcm_writei failed: %s",
                         snd_strerror(frames));
                    /* 恢复失败: 尝试 prepare 完整复位, 退避 10ms 防死循环 */
                    snd_pcm_prepare(ctx->pcm);
                    ctx->underrun_count++;
                    /* 回退 read_idx, 下次重试 */
                    pthread_mutex_lock(&ctx->ring_mutex);
                    ctx->read_idx = (ctx->read_idx - 1 + RING_BUFFER_SLOTS)
                                    % RING_BUFFER_SLOTS;
                    ctx->slots[ctx->read_idx].valid = 1;
                    pthread_mutex_unlock(&ctx->ring_mutex);
                    usleep(10000);
                    continue;
                }
            }

            if (frames > 0) {
                /* ★ 代际再检查: ALSA 写入期间可能发生 flush,
                 * 若代际已变, 跳过时钟更新, 防止旧 PTS 覆盖. */
                if (ctx->flush_gen != saved_gen) {
                    CSG_LOGI("AUDIO_RENDER", "Skip clock update for stale frame "
                         "pts=%lld (gen %d->%d)", (long long)out_pts,
                         saved_gen, ctx->flush_gen);
                    continue;
                }

                /* ============================================================
                 * ★ 时钟锚点管理
                 *
                 * 层次:
                 *   ① seek_pending: adjust_clock 已临时设为视频 PTS,
                 *      首帧音频到达时重新校准为音频 PTS, 消除唇音偏移.
                 *      (Seek 后音频 PTS 与视频 PTS 存在容器偏移, 如 2s)
                 *   ② !clock_init_done: 首次启动, 一次性初始化锚点
                 *      (用 clock_init_done 标志替代 base_pts<=0,
                 *       避免 PTS=0 时 0<=0 重复触发 → base_wall 丢失时间差)
                 *   ③ 其他: 锚点不变, 时钟由墙上时钟线性插值推进
                 * ============================================================ */
                pthread_mutex_lock(&ctx->clock_mutex);

                if (ctx->seek_pending) {
                    /* Seek 后首帧音频 → 校准锚点为音频 PTS */
                    struct timespec ts;
                    clock_gettime(CLOCK_MONOTONIC, &ts);
                    ctx->base_wall_us = ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
                    ctx->base_pts = out_pts;
                    ctx->seek_pending = 0;
                    ctx->clock_init_done = 1;
                    /* ★ v5.10: last_clock 设为 out_pts 而非 0, 防止
                     * 单调保证把时钟锁死在 0 (Seek 后 out_pts 可能很大,
                     * 扣除延迟后 clock ≈ out_pts - delay, 若 last_clock=0
                     * 则 clock < last_clock 触发单调保证 → 锁死在 0). */
                    ctx->last_clock = out_pts;
                    CSG_LOGI("AUDIO_RENDER", "Clock recalibrated to audio "
                         "PTS=%lld (seek_pending cleared)",
                         (long long)out_pts);
                } else if (!ctx->clock_init_done) {
                    /* 首次启动: 一次性初始化锚点.
                     *
                     * ★ v5.10 关键修复: base_pts 设为 0, base_wall_us
                     * 回退 out_pts 时间量, 使主时钟从 0 开始推进.
                     *
                     * 旧代码: base_pts = out_pts (如 120000us), 导致:
                     *   clock = 120000 + delta - delay
                     *   扣除 delay(~127500)后 clock ≈ 27665, 但 last_clock
                     *   被设为 120000, 单调保证把 clock 锁死在 120000
                     *   → 音频时钟始终 120ms 超前视频 → 音画不同步.
                     *
                     * 新代码: base_pts = 0, base_wall_us 回退:
                     *   clock = 0 + (now - (now - out_pts)) - delay
                     *         = out_pts - delay
                     *   若 delay ≈ out_pts, clock ≈ 0, 与视频 PTS=0 对齐.
                     *   last_clock 安全设为 0, 单调保证不会误触发. */
                    struct timespec ts;
                    clock_gettime(CLOCK_MONOTONIC, &ts);
                    int64_t now_us = ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
                    ctx->base_pts = 0;
                    ctx->base_wall_us = now_us - out_pts;  /* 回退首帧 PTS 时间量 */
                    ctx->last_clock = 0;
                    ctx->clock_init_done = 1;
                    CSG_LOGI("AUDIO_RENDER", "Clock init: first_audio_pts=%lld "
                         "base_pts=0 base_wall=%lld (aligned to video timeline)",
                         (long long)out_pts, (long long)ctx->base_wall_us);
                }

                ctx->played_samples += frames;
                ctx->total_played += frames;
                pthread_mutex_unlock(&ctx->clock_mutex);
            }
        } else {
            /* 环形缓冲为空 → 用 snd_pcm_wait 阻塞等待硬件就绪,
             * 若超时仍无数据, 写入静音帧防止 XRUN 连锁触发 */
            int poll_ret = snd_pcm_wait(ctx->pcm, 100);  /* 100ms 超时 */
            if (poll_ret < 0) {
                /* 设备可能处于错误状态 (如 suspend 后未恢复),
                 * 尝试 prepare 复位, 避免后续 silence 写入持续失败 */
                snd_pcm_prepare(ctx->pcm);
            }

            /* 再次检查环形缓冲 (可能在等待期间被填充) */
            pthread_mutex_lock(&ctx->ring_mutex);
            int has_new = (ctx->read_idx != ctx->write_idx);
            pthread_mutex_unlock(&ctx->ring_mutex);

            if (!has_new) {
                /* 真的无数据: 写入静音帧防止硬件欠载 */
                int16_t silence[ALSA_PERIOD_SIZE * 2];
                memset(silence, 0, sizeof(silence));
                snd_pcm_sframes_t sf = snd_pcm_writei(ctx->pcm, silence,
                                                       ALSA_PERIOD_SIZE);
                if (sf < 0) {
                    sf = snd_pcm_recover(ctx->pcm, sf, 1);
                    if (sf < 0) {
                        /* recover 失败: 设备可能处于致命错误状态,
                         * 尝试 prepare 完整复位; 无论成败都退避 10ms
                         * 防止死循环刷屏 (百万次/秒) */
                        snd_pcm_prepare(ctx->pcm);
                        ctx->underrun_count++;
                        CSG_LOGW("AUDIO_RENDER", "silence write failed, XRUN (#%d)",
                             ctx->underrun_count);
                        usleep(10000);  /* 10ms 退避 */
                    }
                }
            }
        }
    }

    CSG_LOGI("AUDIO_RENDER", "Thread stopped");
    return NULL;
}

int csg_audio_render_start(csg_audio_render_ctx_t *ctx)
{
    if (!ctx || !ctx->pcm) return -1;

    if (ctx->running) return 0;

    /* 准备 PCM */
    int ret = snd_pcm_prepare(ctx->pcm);
    if (ret < 0) {
        /* PCM 可能在 RUNNING 状态 (如 flush 中 writei 后未 drop),
         * snd_pcm_prepare 在 RUNNING 状态下返回 EBUSY. 先 drop 再 prepare. */
        if (ret == -EBUSY) {
            CSG_LOGW("AUDIO_RENDER", "PCM busy (RUNNING?), drop + retry prepare");
            snd_pcm_drop(ctx->pcm);
            ret = snd_pcm_prepare(ctx->pcm);
        }
        if (ret < 0) {
            CSG_LOGE("AUDIO_RENDER", "snd_pcm_prepare failed: %s", snd_strerror(ret));
            return -1;
        }
    }

    /* ★ v5.13: 移除 PCM prepare 后的 mixer 重初始化 + DMA prime write +
     * GPIO assert 三步操作.
     *
     * 根因分析 (参考 rk3572_player_nvr_2.0sy 可正常播放):
     *   DMA prime write (snd_pcm_writei 静音帧) 会触发 DAPM 上电,
     *   NAU8822 的 Jack Detect 在上电时检测到"未插入"后覆盖
     *   hp switch / spk switch 为 OFF. 后续 GPIO assert 3 轮重试
     *   全部被 DAPM 立即回退, 导致输出放大器始终关闭 → "嗯嗯"声.
     *
     * 参考项目做法:
     *   mixer 在 csg_audio_render_create 时初始化一次 (此时 PCM 刚打开
     *   但未 prepare, DAPM 未激活), 设置 hp/spk switch 为 ON.
     *   start 只做 prepare + 清缓冲 + 重置时钟 + 创建线程.
     *   音频线程首次 snd_pcm_writei 自然触发 DAPM 上电,
     *   此时 hp/spk 已经 ON, DAPM 不会覆盖 (仅在下电→上电的
     *   完整循环中才会重新评估 Jack Detect).
     *
     * 保留 persistent audio device 架构: create 时 mixer 初始化 +
     * GPIO assert 兜底, 后续 start/flush 不再重复 mixer 操作. */

    /* 清空环形缓冲 */
    pthread_mutex_lock(&ctx->ring_mutex);
    for (int i = 0; i < RING_BUFFER_SLOTS; i++) {
        ctx->slots[i].valid = 0;
    }
    ctx->write_idx = 0;
    ctx->read_idx = 0;
    pthread_mutex_unlock(&ctx->ring_mutex);

    /* 重置时钟 */
    pthread_mutex_lock(&ctx->clock_mutex);
    ctx->base_pts = 0;
    ctx->base_wall_us = 0;
    ctx->last_clock = 0;
    ctx->played_samples = 0;
    ctx->clock_ema = 0;
    ctx->clock_ema_valid = 0;
    pthread_mutex_unlock(&ctx->clock_mutex);

    ctx->running = 1;
    ctx->playing = 1;

    ret = pthread_create(&ctx->thread, NULL, csg_audio_render_thread, ctx);
    if (ret != 0) {
        CSG_LOGE("AUDIO_RENDER", "pthread_create failed: %d", ret);
        ctx->running = 0;
        ctx->playing = 0;
        return -1;
    }

    CSG_LOGI("AUDIO_RENDER", "Started");
    return 0;
}

void csg_audio_render_stop(csg_audio_render_ctx_t *ctx)
{
    if (!ctx) return;
    if (!ctx->running) return;

    ctx->running = 0;

    /* ★ 先 drop 停止设备, 让阻塞在 snd_pcm_wait / snd_pcm_writei 的写线程
     * 立即返回, 再 join. 若先 join, 写线程可能卡在 100ms wait 或长时间
     * 的设备写拨付而迟迟不退出 → 本实例无法及时释放子流 → 下一次
     * snd_pcm_open(hw:1,0) 拿到 EBUSY, 表现为"第二次运行无声". */
    if (ctx->pcm) {
        snd_pcm_drop(ctx->pcm);
    }
    pthread_join(ctx->thread, NULL);
    ctx->playing = 0;

    CSG_LOGI("AUDIO_RENDER", "Stopped");
}

int csg_audio_render_write(csg_audio_render_ctx_t *ctx, const void *data,
                       int samples, int64_t pts)
{
    if (!ctx || !ctx->running || !data || samples <= 0) return -1;

    int next_write = (ctx->write_idx + 1) % RING_BUFFER_SLOTS;

    pthread_mutex_lock(&ctx->ring_mutex);

    /* ★ v5.18: 对齐参考项目 (2.0sy), 不做大 chunk 拆分.
     * 参考项目直接截断到 MAX_SAMPLES_PER_SLOT, 单次写入一个 slot.
     * 旧代码的拆分逻辑 (while 循环) 导致多个 slot 共享相同 PTS,
     * 渲染线程消费时 played_samples 递增但锚点不更新, 时钟推进
     * 与音频实际播放脱节 → AV-SYNC 振荡 → 嘈杂听感. */
    if (next_write == ctx->read_idx) {
        /* 缓冲区满, 丢弃最旧的一帧 */
        ctx->read_idx = (ctx->read_idx + 1) % RING_BUFFER_SLOTS;
        ctx->slots[ctx->read_idx].valid = 0;
        pthread_mutex_unlock(&ctx->ring_mutex);
        return 1;
    }

    csg_audio_render_ring_slot_t *slot = &ctx->slots[ctx->write_idx];

    int copy_samples = samples;
    if (copy_samples > MAX_SAMPLES_PER_SLOT) {
        copy_samples = MAX_SAMPLES_PER_SLOT;
    }

    int copy_bytes = copy_samples * ctx->frame_bytes;
    memcpy(slot->data, data, copy_bytes);
    slot->samples = copy_samples;
    slot->pts = pts;
    slot->valid = 1;

    ctx->write_idx = next_write;
    ctx->total_written += copy_samples;

    pthread_mutex_unlock(&ctx->ring_mutex);

    return 0;
}

int64_t csg_audio_render_get_clock(csg_audio_render_ctx_t *ctx)
{
    if (!ctx) return 0;

    /* 暂停时冻结时钟: 暂停期间 base_wall_us 不再更新,
     * 若继续插值时钟会持续跑偏, 导致恢复播放时同步跳变. */
    if (!ctx->playing) {
        return ctx->last_clock;
    }

    pthread_mutex_lock(&ctx->clock_mutex);

    if (!ctx->clock_init_done || ctx->sample_rate <= 0) {
        pthread_mutex_unlock(&ctx->clock_mutex);
        return 0;
    }

    /* ★ 纯墙上时钟插值: 锚点永不更新, 时钟由 CLOCK_MONOTONIC 线性推进.
     *
     * 公式: clock = base_pts + (now - base_wall_us) * speed
     *
     * ★ v5.18: 对齐参考项目 (2.0sy), 不做 delay 扣除.
     * 参考项目 get_clock 是纯墙上时钟插值, 音频时钟直接反映"已写入位置".
     * 旧代码的 delay 扣除 (含 EMA 平滑 + CAP 上限) 导致音频时钟系统性
     * 偏低, AV-SYNC 频繁 WAIT/DROP 振荡 → 音频渲染节奏紊乱 → 嘈杂听感.
     * 同步由 AV-SYNC 的 av_offset 补偿固定延迟, 无需在时钟层扣除. */
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    int64_t now_us = ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;

    int64_t delta = now_us - ctx->base_wall_us;
    float spd = ctx->speed > 0.0f ? ctx->speed : 1.0f;
    int64_t clock = ctx->base_pts + (int64_t)((double)delta * spd);

    /* 调试: 仅当 delta 为负 (时钟倒退, 真正异常) 时打印, 且限频.
     * ★ 历史踩坑: 原条件 delta>10000000 在播放超 10 秒后每帧触发
     *   (正常播放 delta 就是墙钟流逝), 造成串口日志风暴, 阻塞调用线程
     *   导致 UI 卡顿/花屏, 必须移除. */
    if (delta < 0 && (ctx->clock_dbg_cnt++ % 200) == 0) {
        CSG_LOGI("AUDIO_RENDER", "CLOCK-DBG: base_pts=%lld base_wall=%lld "
             "now=%lld delta=%lld playing=%d",
             (long long)ctx->base_pts, (long long)ctx->base_wall_us,
             (long long)now_us, (long long)delta, ctx->playing);
    }

    /* 单调保证: 仅保留安全网, 锚点不更新后基本不会触发 */
    if (clock < ctx->last_clock) {
        clock = ctx->last_clock;
    } else {
        ctx->last_clock = clock;
    }

    pthread_mutex_unlock(&ctx->clock_mutex);
    return clock;
}

int64_t csg_audio_render_get_delay(csg_audio_render_ctx_t *ctx)
{
    if (!ctx) return 0;

    pthread_mutex_lock(&ctx->ring_mutex);

    /* 环形缓冲中等待播放的采样数 */
    int pending = 0;
    int idx = ctx->read_idx;
    while (idx != ctx->write_idx) {
        if (ctx->slots[idx].valid) {
            pending += ctx->slots[idx].samples;
        }
        idx = (idx + 1) % RING_BUFFER_SLOTS;
    }
    pthread_mutex_unlock(&ctx->ring_mutex);

    /* ALSA 硬件缓冲中未播放的帧数 */
    snd_pcm_sframes_t delay_frames = 0;
    if (ctx->pcm) {
        snd_pcm_delay(ctx->pcm, &delay_frames);
        if (delay_frames < 0) delay_frames = 0;
    }

    int64_t total_samples = pending + delay_frames;
    int64_t delay_us = (int64_t)((double)total_samples
                                  / ctx->sample_rate * 1000000.0);
    return delay_us;
}

void csg_audio_render_pause(csg_audio_render_ctx_t *ctx)
{
    if (!ctx) return;
    if (!ctx->playing) return;

    ctx->playing = 0;

    if (ctx->pcm) {
        snd_pcm_drop(ctx->pcm);
    }

    /* 清空环形缓冲 */
    pthread_mutex_lock(&ctx->ring_mutex);
    for (int i = 0; i < RING_BUFFER_SLOTS; i++) {
        ctx->slots[i].valid = 0;
    }
    ctx->write_idx = 0;
    ctx->read_idx = 0;
    pthread_mutex_unlock(&ctx->ring_mutex);

    CSG_LOGI("AUDIO_RENDER", "Paused");
}

void csg_audio_render_resume(csg_audio_render_ctx_t *ctx)
{
    if (!ctx) return;
    if (ctx->playing) return;

    ctx->playing = 1;

    if (ctx->pcm) {
        snd_pcm_prepare(ctx->pcm);
    }

    /* ★ v5.13: 移除 resume 后的 mixer 重初始化 + DMA prime write +
     * GPIO assert 三步操作.
     *
     * 与 csg_audio_render_start 相同根因:
     *   DMA prime write 触发 DAPM 上电, Jack Detect 覆盖 hp/spk switch.
     *   参考项目 (2.0sy) resume 只做 snd_pcm_prepare + playing=1,
     *   音频线程首次 writei 自然上电, 不触发 JD 覆盖. */

    /* ★ 恢复时重校准时钟锚点: 暂停期间 base_wall_us 未更新,
     * 若不重设, 恢复后 clock = base_pts + (now - base_wall_old) * speed
     * 会直接跳变暂停时长 (本日志中跳了 13 秒), 导致全部帧被 DROP.
     *
     * 修复: base_pts = 冻结的 last_clock, base_wall_us = 当前系统时间,
     * 恢复后 clock = last_clock + (now - now) = last_clock, 零跳变. */
    pthread_mutex_lock(&ctx->clock_mutex);
    if (ctx->last_clock > 0) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        ctx->base_pts = ctx->last_clock;
        ctx->base_wall_us = ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
        CSG_LOGI("AUDIO_RENDER", "Resume clock recal: base_pts=%lld, wall=%lld",
             (long long)ctx->base_pts, (long long)ctx->base_wall_us);
    }
    pthread_mutex_unlock(&ctx->clock_mutex);

    CSG_LOGI("AUDIO_RENDER", "Resumed");
}

void csg_audio_render_flush(csg_audio_render_ctx_t *ctx, int64_t new_base_pts)
{
    if (!ctx) return;

    if (ctx->pcm) {
        snd_pcm_drop(ctx->pcm);
        snd_pcm_prepare(ctx->pcm);

        /* ★ v5.13: 移除 flush 后的 mixer 重初始化 + 静音预填充 +
         * GPIO assert 三步操作.
         *
         * 根因: snd_pcm_writei(silence) 触发 DAPM 上电,
         *   Jack Detect 在上电时覆盖 hp/spk switch 为 OFF.
         *   参考项目 (2.0sy) flush 只做 drop+prepare+清缓冲+重置时钟,
         *   音频线程首次 writei 自然上电, 不触发 JD 覆盖.
         *
         * 注意: 不再做末尾的 snd_pcm_drop (旧代码为让 start 的 prepare
         * 不受 EBUSY 影响). 现在 start 不再做额外操作, 直接 prepare 即可. */
    }

    pthread_mutex_lock(&ctx->ring_mutex);
    for (int i = 0; i < RING_BUFFER_SLOTS; i++) {
        ctx->slots[i].valid = 0;
    }
    ctx->write_idx = 0;
    ctx->read_idx = 0;
    /* ★ flush 代际递增: 通知音频线程丢弃所有旧帧,
     * 防止旧 PTS 帧在 flush 后覆盖时钟. */
    ctx->flush_gen++;
    /* ★ Seek 待处理标志: 下一帧音频到达时校准时钟锚点为音频 PTS. */
    ctx->seek_pending = 1;
    ctx->clock_init_done = 0;  /* 复位, 允许音频线程重新初始化锚点 */
    pthread_mutex_unlock(&ctx->ring_mutex);

    /* 音频时钟复位:
     * new_base_pts > 0  → Seek 场景: 直接设为目标 PTS + 当前时间锚点
     * new_base_pts = 0  → 普通停止: 归零, 首帧到达时自动初始化 */
    pthread_mutex_lock(&ctx->clock_mutex);
    ctx->base_pts = new_base_pts;
    if (new_base_pts > 0) {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        ctx->base_wall_us = ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
        ctx->last_clock = new_base_pts;
        ctx->clock_ema = new_base_pts;
        ctx->clock_ema_valid = 1;
    } else {
        ctx->base_wall_us = 0;
        ctx->last_clock = 0;
        ctx->clock_ema = 0;
        ctx->clock_ema_valid = 0;
    }
    ctx->played_samples = 0;
    pthread_mutex_unlock(&ctx->clock_mutex);

    CSG_LOGI("AUDIO_RENDER", "Flushed (base_pts=%lld)", (long long)new_base_pts);
}

void csg_audio_render_adjust_clock(csg_audio_render_ctx_t *ctx, int64_t new_base_pts)
{
    if (!ctx || new_base_pts <= 0) return;

    pthread_mutex_lock(&ctx->clock_mutex);
    ctx->base_pts = new_base_pts;
    /* ★ base_wall_us 必须同步设为当前时间, 不能留 0!
     * 否则 clock = base_pts + (now - 0) 会把系统 uptime 全加进去,
     * 导致时钟值膨胀到 23933 秒, 引发后续内存异常崩溃. */
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    ctx->base_wall_us = ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
    ctx->last_clock = new_base_pts;
    ctx->clock_ema = new_base_pts;
    ctx->clock_ema_valid = 1;
    ctx->played_samples = 0;
    pthread_mutex_unlock(&ctx->clock_mutex);

    CSG_LOGI("AUDIO_RENDER", "Clock adjusted to %lldus (wall=%lldus)",
         (long long)new_base_pts, (long long)ctx->base_wall_us);
}

void csg_audio_render_set_speed(csg_audio_render_ctx_t *ctx, float speed)
{
    if (!ctx) return;
    if (speed < 0.25f) speed = 0.25f;
    if (speed > 16.0f) speed = 16.0f;

    pthread_mutex_lock(&ctx->clock_mutex);

    /* ★ 倍速切换时必须重新校准锚点, 否则时钟推进速率与实际音频播放速率不匹配.
     * 问题: 只改speed不改锚点, get_clock用新speed × 旧锚点时间差,
     *      但音频线程写入PCM速率仍是1x, 导致时钟严重落后.
     * 解决: 计算当前时钟, 以此为新的base_pts, 当前时间为新的base_wall_us,
     *      使时钟从此刻开始以新speed正确推进. */
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    int64_t now_us = ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;

    /* 用旧speed计算当前时钟 */
    float old_speed = ctx->speed > 0.0f ? ctx->speed : 1.0f;
    int64_t current_clock = 0;
    if (ctx->base_pts > 0 && ctx->base_wall_us > 0) {
        current_clock = ctx->base_pts
                      + (int64_t)((double)(now_us - ctx->base_wall_us) * old_speed);
    } else if (ctx->last_clock > 0) {
        current_clock = ctx->last_clock;
    }

    /* 更新speed */
    ctx->speed = speed;

    /* 重新设置锚点: 以当前时钟为基准, 从此刻开始以新speed推进 */
    if (current_clock > 0) {
        ctx->base_pts = current_clock;
        ctx->base_wall_us = now_us;
        CSG_LOGI("AUDIO_RENDER", "Speed anchor recalibrated: speed=%.2f, base_pts=%lld, wall=%lld",
             speed, (long long)ctx->base_pts, (long long)ctx->base_wall_us);
    }

    pthread_mutex_unlock(&ctx->clock_mutex);

    /* ★ 所有倍速均保持音频输出, 不静音.
     * 高倍速 (≥4x) 下音频采样被稀释 (step=speed), 音质下降但可辨识,
     * 用户可通过耳机听到快放效果 (类似"叽叽喳喳"的快进声音).
     * 同步策略: 高倍速下音频时钟锚定由 main.c 的 AV-SYNC 层管理,
     * 每显示一帧视频后将音频时钟同步到该帧 PTS,
     * 使下一帧的 frame_timer = (next_pts - current_pts) / speed 正确反映帧间隔. */
    ctx->muted = 0;
}

void csg_audio_render_set_volume(csg_audio_render_ctx_t *ctx, int volume)
{
    if (!ctx) return;
    if (volume < 0) volume = 0;
    if (volume > 100) volume = 100;
    ctx->volume = volume;
}

void csg_audio_render_set_mute(csg_audio_render_ctx_t *ctx, int mute)
{
    if (!ctx) return;
    ctx->muted = mute ? 1 : 0;
}

int csg_audio_render_get_volume(csg_audio_render_ctx_t *ctx)
{
    return ctx ? ctx->volume : 0;
}

int csg_audio_render_get_mute(csg_audio_render_ctx_t *ctx)
{
    return ctx ? ctx->muted : 0;
}

int csg_audio_render_is_playing(csg_audio_render_ctx_t *ctx)
{
    return ctx ? ctx->playing : 0;
}