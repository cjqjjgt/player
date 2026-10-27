/* SPDX-License-Identifier: MIT */
/*
 * vecdeque.h - 动态数组实现的双端队列 (参考 GStreamer GstVecDeque)
 *
 * 设计要点 (对应 GStreamer 播放器数据流流程图 §2 源码级解析):
 *   ① 底层是"动态数组 + 环形寻址"的混合体:
 *      - 连续内存块, 缓存友好, O(1) 随机访问
 *      - head/tail 取模实现环形寻址, O(1) 头尾插入/删除
 *      - 容量不足时翻倍扩容 (摊销 O(1)), 扩容后重排为线性布局
 *   ② 元素存储: 存"结构体副本" (struct_array 模式), 元素大小固定 elt_size
 *   ③ 本容器只负责存储与扩容, 不做内存水位/背压控制 ——
 *      那部分由上层 demux_buffer (解封装→解码缓冲层) 负责.
 *
 * 线程安全: 本容器非线程安全, 由调用方保证互斥 (demux_buffer 内部持锁).
 */

#ifndef __CSG_VECDEQUE_H__
#define __CSG_VECDEQUE_H__

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t *array;        /* 堆上连续内存块 (非链表) */
    size_t   size;         /* 总容量 (槽位数), 扩容时翻倍 */
    size_t   head;         /* 头部索引 (下一个要 pop 的位置), 环形取模 */
    size_t   tail;         /* 尾部索引 (下一个要 push 的位置), 环形取模 */
    size_t   length;       /* 当前元素个数 */
    size_t   elt_size;     /* 每个元素的大小 (字节) */
} _csg_vecdeque_t;

typedef _csg_vecdeque_t csg_vecdeque_t;

/* 初始化: elt_size 必须 >0, initial_capacity 为初始槽位数 (0 则用默认 16) */
int  csg_vecdeque_init(csg_vecdeque_t *q, size_t elt_size, size_t initial_capacity);

/* 释放底层内存 (元素本身若含动态指针, 由调用方先行清理) */
void csg_vecdeque_destroy(csg_vecdeque_t *q);

/* 当前元素个数 / 总容量 */
size_t csg_vecdeque_length(const csg_vecdeque_t *q);
size_t csg_vecdeque_capacity(const csg_vecdeque_t *q);

/* 尾部入队: 把 *elem (elt_size 字节) 复制到队尾. 满时自动翻倍扩容.
 * 返回 0 成功, -1 内存不足. */
int  csg_vecdeque_push_tail(csg_vecdeque_t *q, const void *elem);

/* 头部出队: 把队首元素复制到 *out (elt_size 字节).
 * 返回 0 成功, -1 队列为空. */
int  csg_vecdeque_pop_head(csg_vecdeque_t *q, void *out);

/* 查看队首 (不弹出). 返回 0 成功, -1 队列为空. */
int  csg_vecdeque_peek_head(const csg_vecdeque_t *q, void *out);

/* 清空队列 (保留容量, 不释放元素动态成员) */
void csg_vecdeque_clear(csg_vecdeque_t *q);

/* 手动扩容到 new_capacity (至少保留当前元素). 返回 0 成功, -1 失败. */
int  csg_vecdeque_resize(csg_vecdeque_t *q, size_t new_capacity);

#ifdef __cplusplus
}
#endif

#endif /* __CSG_VECDEQUE_H__ */
