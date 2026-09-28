/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file rtstats.h
 *
 * @par dependencies
 * - stdint.h
 *
 * @author zw1194
 *
 * @brief Provide the FreeRTOS run-time statistics timebase and printer task.
 *
 * Processing flow:
 *
 * rtstats_init()  : 内核在 vTaskStartScheduler() 里调一次, 启动时基(TIM5)
 * rtstats_start() : 在 osKernelStart() 之前调一次, 建 "rtstats" 任务
 * "rtstats" 任务体: 每 5 秒打印一次内核堆 + 任务表(栈水位) + CPU 占比
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
#ifndef __RTSTATS_H__
#define __RTSTATS_H__

/***********************************Includes***********************************/
#include <stdint.h>

/**********************************Declaring***********************************/
/* 启动运行统计时基(TIM5 自由计数 10kHz). 只有一个调用者: FreeRTOS 内核在
   vTaskStartScheduler() 里经 portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() 调用,
   见 FreeRTOSConfig.h */
void rtstats_init(void);

/* 读时基计数(10kHz, 1 个计数 = 100us), 供 portGET_RUN_TIME_COUNTER_VALUE 调用 */
uint32_t rtstats_get_counter(void);

/* 建运行统计打印任务("rtstats"). 在 osKernelInitialize() 之后、
   osKernelStart() 之前调用(任务体只打印, 不碰任何设备) */
int8_t rtstats_start(void);

/**********************************Declaring***********************************/

#endif // __RTSTATS_H__
