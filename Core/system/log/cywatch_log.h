/******************************************************************************
 * Copyright (C) 2026 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_log.h
 *
 * @par dependencies
 * - stdint.h
 *
 * @author zw1194
 *
 * @brief 带互斥量的 printf: 把整条日志串行化, 解决跨任务打印乱码/丢字符。
 *
 * Processing flow:
 *
 * log_init()   : 建互斥量(第一个 printf 之前调一次)
 * log_printf() : 拿锁 → vprintf → 放锁
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 为什么锁的是整条日志而不是 fputc: 微库(MicroLib)的 printf 自身不可重入,
 *       只锁每字符的发送挡不住**格式化阶段**的并发。
 * @note 不能在中断里调: 互斥量不是 FromISR 安全的。本工程当前没有 ISR printf。
 *****************************************************************************/
#ifndef __CYWATCH_LOG_H__
#define __CYWATCH_LOG_H__

/***********************************Includes***********************************/
#include <stdint.h>
/***********************************Includes***********************************/

/**********************************Declaring***********************************/
/* 建日志互斥量(幂等). 在任何并发 printf 之前调一次.
   返回 0 成功 / -1 互斥量创建失败(内核堆不足) */
int8_t log_init(void);

/* 带锁的 printf. log_init() 之前调用会降级为直接 vprintf —— 那时还是单线程, 没人抢.
   返回写入的字符数(同 printf) */
int log_printf(const char *fmt, ...);
/**********************************Declaring***********************************/

#endif // __CYWATCH_LOG_H__
