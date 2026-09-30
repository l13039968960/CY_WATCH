/******************************************************************************
 * Copyright (C) 2026 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_log.c
 *
 * @par dependencies
 * - cywatch_log.h
 * - cmsis_os2.h
 * - stdio.h / stdarg.h
 *
 * @author zw1194
 *
 * @brief Implete the mutex-guarded printf: 见 cywatch_log.h 的 @note。
 *
 * Processing flow:
 *
 * 本文件只有一个静态互斥量。printf 最终走 uart.c 的 fputc, 每字符一次阻塞
 * HAL_UART_Transmit(115200 下 ~87us), 全程无闸门; 两个任务同时打, 第二个在
 * HAL 的 __HAL_LOCK 上拿到 HAL_BUSY 并被 fputc 的 (void) 静默丢掉 —— 表现为
 * 日志缺字/串行错位。这里用一把互斥量把整条日志包起来。
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *****************************************************************************/
#include "system/log/cywatch_log.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <stdarg.h>
#include "cmsis_os2.h"

/* NULL = 还没建(或建失败). 此时 log_printf 不拿锁直接打: log_init() 之前还是
   单线程启动阶段, 没有并发可言 */
static osMutexId_t s_log_mutex = NULL;

/* 日志互斥量 */
int8_t log_init(void)
{
    /* 幂等: 重复调用直接当成功, 不会建出第二把锁 */
    if (NULL != s_log_mutex)
    {
        return 0;
    }

    /* 默认属性: 非递归 + 优先级继承(FreeRTOS mutex 的默认语义) */
    s_log_mutex = osMutexNew(NULL);
    if (NULL == s_log_mutex)
    {
        return -1;
    }

    return 0;
}

/* 带锁的 printf */
int log_printf(const char *fmt, ...)
{
    va_list ap;
    int n;

    /* 阻塞排队, 等前一条日志打完. 最坏是 rtstats 一整屏(~700B, ~61ms) */
    if (NULL != s_log_mutex)
    {
        (void)osMutexAcquire(s_log_mutex, osWaitForever);
    }

    va_start(ap, fmt);
    n = vprintf(fmt, ap);
    va_end(ap);

    if (NULL != s_log_mutex)
    {
        (void)osMutexRelease(s_log_mutex);
    }

    return n;
}
/* USER CODE END 0 */
