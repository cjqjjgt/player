/*
 * mp4diag.c - MP4 box 结构诊断工具 (不依赖 mp4v2, 纯手写解析)
 * 用途: 定位 mp4v2 打开 MP4 时 "SetPosition: seek failed: errno 22" 根因.
 * 原理: mp4v2 在 MP4File::Open 解析 moov 时按 box size 递归 seek,
 *       若某个 box size 字段异常(超文件剩余范围) 即 seek 越界.
 * 本工具遍历所有 box, 打印越界位置, 直接暴露问题 box.
 * 编译: aarch64-buildroot-linux-gnu-gcc -O2 -o mp4diag mp4diag.c
 * 板上: ./mp4diag /userdata/records/xxx.mp4
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

static const unsigned char *g_buf;
static size_t g_size;

static uint32_t rd32(const unsigned char *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |  (uint32_t)p[3];
}
static uint64_t rd64(const unsigned char *p)
{
    return ((uint64_t)rd32(p) << 32) | (uint64_t)rd32(p + 4);
}

static int is_container(const unsigned char *t)
{
    static const char *c[] = {
        "moov","trak","mdia","minf","stbl","edts","dinf","udta",
        "mvex","moof","traf","mfra","meta","ilst","schi","sinf", NULL
    };
    for (int i = 0; c[i]; i++)
        if (memcmp(t, c[i], 4) == 0) return 1;
    return 0;
}

static void indent(int d) { for (int i = 0; i < d; i++) fputs("  ", stdout); }

static int walk_box(const unsigned char *p, size_t remain, int depth)
{
    if (remain < 8) { indent(depth); printf("TRUNCATED remain=%zu\n", remain); return -1; }

    uint32_t size32 = rd32(p);
    const unsigned char *type = p + 4;
    uint64_t size; size_t hdr = 8;
    if (size32 == 1) {
        if (remain < 16) { indent(depth); printf("TRUNCATED(64hdr) off=%lu\n", (unsigned long)(p - g_buf)); return -1; }
        size = rd64(p + 8); hdr = 16;
    } else if (size32 == 0) {
        size = remain;
    } else {
        size = size32;
    }
    unsigned long off = (unsigned long)(p - g_buf);

    if (size > remain) {
        indent(depth);
        printf("**** OUT-OF-RANGE **** box='%.4s' size=%llu off=%lu remain=%zu -> seek 越界(errno 22)!\n",
               type, (unsigned long long)size, off, remain);
        return -1;
    }
    if (size < hdr) {
        indent(depth);
        printf("**** BAD-SIZE **** box='%.4s' size=%llu<hdr=%zu off=%lu\n", type, (unsigned long long)size, hdr, off);
        return -1;
    }

    indent(depth);
    printf("box='%.4s' size=%llu off=%lu\n", type, (unsigned long long)size, off);

    const unsigned char *body = p + hdr;
    size_t body_len = size - hdr;

    if (memcmp(type, "ftyp", 4) == 0 && body_len >= 8) {
        indent(depth + 1); printf("major_brand='%.4s' minor=%u\n", body, rd32(body + 4));
    } else if (memcmp(type, "stsd", 4) == 0 && body_len >= 8) {
        unsigned n = rd32(body + 4);
        indent(depth + 1); printf("entry_count=%u\n", n);
        const unsigned char *e = body + 8; size_t elen = body_len - 8;
        for (unsigned i = 0; i < n && elen >= 8; i++) {
            uint32_t esz = rd32(e);
            indent(depth + 2);
            printf("sample_entry[%u]='%.4s' size=%u off=%lu\n", i, e + 4, esz, (unsigned long)(e - g_buf));
            if (esz < 8 || esz > elen) break;
            e += esz; elen -= esz;
        }
    } else if ((memcmp(type, "stco", 4) == 0 || memcmp(type, "co64", 4) == 0) && body_len >= 8) {
        unsigned n = rd32(body + 4);
        indent(depth + 1); printf("chunk_offset_count=%u\n", n);
        const unsigned char *e = body + 8; size_t elen = body_len - 8; unsigned bad = 0;
        for (unsigned i = 0; i < n; i++) {
            uint64_t offc;
            if (memcmp(type, "co64", 4) == 0) { if (elen < 8) break; offc = rd64(e); e += 8; elen -= 8; }
            else                               { if (elen < 4) break; offc = rd32(e); e += 4; elen -= 4; }
            if (offc >= g_size) {
                if (bad < 5) { indent(depth + 2); printf("chunk[%u] off=%llu >= size=%zu <-- 越界!\n", i, (unsigned long long)offc, g_size); }
                bad++;
            }
        }
        if (bad) { indent(depth + 2); printf("共 %u/%u chunk 越界\n", bad, n); }
    } else if ((memcmp(type, "stsz", 4) == 0 || memcmp(type, "stz2", 4) == 0) && body_len >= 12) {
        indent(depth + 1); printf("sample_count=%u\n", rd32(body + 8));
    } else if (memcmp(type, "mvhd", 4) == 0 && body_len >= 20) {
        unsigned ver = body[0];
        uint64_t ts, dur;
        if (ver == 1) { ts = rd32(body + 20); dur = rd64(body + 24); }
        else          { ts = rd32(body + 12); dur = rd32(body + 16); }
        indent(depth + 1);
        printf("mvhd ver=%u timescale=%llu duration=%llu (%.2fs)\n", ver,
               (unsigned long long)ts, (unsigned long long)dur, ts ? (double)dur / ts : 0.0);
    } else if (memcmp(type, "tkhd", 4) == 0 && body_len >= 84) {
        indent(depth + 1); printf("tkhd ver=%u track_id=%u\n", body[0], rd32(body + 12));
    }

    if (is_container(type) && body_len > 0) {
        const unsigned char *c = body; size_t cused = 0;
        while (cused + 8 <= body_len) {
            int rc = walk_box(c, body_len - cused, depth + 1);
            if (rc < 0) return -1;
            uint32_t cs = rd32(c); uint64_t csize = cs; size_t chdr = 8;
            if (cs == 1) { csize = rd64(c + 8); chdr = 16; }
            else if (cs == 0) { csize = body_len - cused; }
            if (csize < chdr || csize > body_len - cused) return -1;
            c += (size_t)csize; cused += (size_t)csize;
        }
        if (body_len - cused > 0) { indent(depth + 1); printf("(container tail %zu bytes)\n", body_len - cused); }
    }
    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: %s <file.mp4>\n", argv[0]); return 1; }
    int fd = open(argv[1], O_RDONLY);
    if (fd < 0) { perror("open"); return 1; }
    struct stat st;
    if (fstat(fd, &st) != 0) { perror("fstat"); return 1; }
    g_size = (size_t)st.st_size;
    g_buf = mmap(NULL, g_size, PROT_READ, MAP_SHARED, fd, 0);
    if (g_buf == MAP_FAILED) { perror("mmap"); return 1; }

    printf("== %s (%zu bytes) ==\n", argv[1], g_size);
    int rc = 0; size_t used = 0;
    while (used + 8 <= g_size) {
        rc = walk_box(g_buf + used, g_size - used, 0);
        if (rc < 0) { printf("\n>>> offset %zu 处结构错误(导致 seek 越界) <<<\n", used); break; }
        uint32_t sz = rd32(g_buf + used); uint64_t bsize = sz;
        if (sz == 1) bsize = rd64(g_buf + used + 8);
        else if (sz == 0) bsize = g_size - used;
        if (bsize < 8 || bsize > g_size - used) { printf(">>> 顶层越界 offset %zu <<<\n", used); rc = -1; break; }
        used += (size_t)bsize;
    }
    if (rc == 0 && used < g_size) printf(">>> 末尾残留 %zu 字节 <<<\n", g_size - used);
    else if (rc == 0) printf("\n>>> 全部 box 结构完好(无越界) <<<\n");

    munmap((void *)g_buf, g_size);
    close(fd);
    return 0;
}
