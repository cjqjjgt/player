/* SPDX-License-Identifier: MIT */
/*
 * vecdeque.c - 动态数组双端队列实现 (参考 GStreamer GstVecDeque)
 *
 * 扩容 (对应 gst_vec_deque_do_expand):
 *   旧队列环状分布: [head, size) + [0, tail), 有效元素为 length 个.
 *   本实现从 head 出发顺序数出 length 个元素逐个复制到新数组开头,
 *   任意时刻调用都正确 (不依赖"满时 head==tail"这一不变量), 逻辑最稳.
 *   head=0, tail=length 重排为线性布局, 后续取模寻址不受影响.
 */

#include "common/vecdeque.h"

#include <stdlib.h>
#include <string.h>

#define CSG_VECDEQUE_DEFAULT_CAPACITY  16

int csg_vecdeque_init(csg_vecdeque_t *q, size_t elt_size, size_t initial_capacity)
{
    if (!q || elt_size == 0) return -1;

    q->elt_size = elt_size;
    q->size = (initial_capacity > 0) ? initial_capacity : CSG_VECDEQUE_DEFAULT_CAPACITY;
    q->head = 0;
    q->tail = 0;
    q->length = 0;

    q->array = (uint8_t *)calloc(q->size, elt_size);
    if (!q->array) {
        q->size = 0;
        return -1;
    }
    return 0;
}

void csg_vecdeque_destroy(csg_vecdeque_t *q)
{
    if (!q) return;
    if (q->array) free(q->array);
    q->array = NULL;
    q->size = 0;
    q->head = 0;
    q->tail = 0;
    q->length = 0;
    q->elt_size = 0;
}

size_t csg_vecdeque_length(const csg_vecdeque_t *q)
{
    return q ? q->length : 0;
}

size_t csg_vecdeque_capacity(const csg_vecdeque_t *q)
{
    return q ? q->size : 0;
}

void csg_vecdeque_clear(csg_vecdeque_t *q)
{
    if (!q) return;
    q->head = 0;
    q->tail = 0;
    q->length = 0;
    /* 容量保留, 底层内存不释放 (后续 push 直接复用, 避免内存抖动) */
}

/* 扩容到 new_capacity (>= length). 从 head 出发数 length 个元素重排到新数组开头. */
int csg_vecdeque_resize(csg_vecdeque_t *q, size_t new_capacity)
{
    if (!q || new_capacity < q->length) return -1;

    uint8_t *newarr = (uint8_t *)malloc(new_capacity * q->elt_size);
    if (!newarr) return -1;

    const uint8_t *old = q->array;
    size_t pos = 0;
    for (size_t i = 0; i < q->length; i++) {
        size_t idx = (q->head + i) % q->size;
        memcpy(newarr + pos * q->elt_size, old + idx * q->elt_size, q->elt_size);
        pos++;
    }

    free(q->array);
    q->array = newarr;
    q->size = new_capacity;
    q->head = 0;
    q->tail = q->length;
    return 0;
}

/* 内部扩容: 翻倍策略 (摊销 O(1)) */
static int csg_vecdeque_do_expand(csg_vecdeque_t *q)
{
    size_t newsize = q->size * 2;
    if (newsize < q->size) newsize = q->size + 1;   /* 防溢出 */
    return csg_vecdeque_resize(q, newsize);
}

int csg_vecdeque_push_tail(csg_vecdeque_t *q, const void *elem)
{
    if (!q || !elem) return -1;

    /* 满了就扩容 (gst_vec_deque_push_tail_struct) */
    if (q->length >= q->size) {
        if (csg_vecdeque_do_expand(q) != 0) return -1;
    }

    memcpy(q->array + q->tail * q->elt_size, elem, q->elt_size);
    q->tail++;
    q->tail %= q->size;   /* 环形取模 (ARM64 上编译为 UDIV, 1 周期) */
    q->length++;
    return 0;
}

int csg_vecdeque_pop_head(csg_vecdeque_t *q, void *out)
{
    if (!q || !out) return -1;
    if (q->length == 0) return -1;

    memcpy(out, q->array + q->head * q->elt_size, q->elt_size);
    q->head++;
    q->head %= q->size;
    q->length--;
    return 0;
}

int csg_vecdeque_peek_head(const csg_vecdeque_t *q, void *out)
{
    if (!q || !out) return -1;
    if (q->length == 0) return -1;

    memcpy(out, q->array + q->head * q->elt_size, q->elt_size);
    return 0;
}
