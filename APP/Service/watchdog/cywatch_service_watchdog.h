/******************************************************************************
 * Copyright (C) 2026 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_service_watchdog.h
 *
 * @par dependencies
 * - stdint.h
 *
 * @author zw1194
 *
 * @brief 看门狗服务: 建一个任务, 周期喂 IWDG(≈4.096s), 每 1s 一口.
 *
 * Processing flow:
 *
 * 1. main.c 在 osKernelStart() 之前调 service_watchdog_init() —— 只建任务;
 * 2. 任务体跑起来后先启动 IWDG, 再进 while(1) 喂狗, 之后无人再碰它.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 这是**纯周期喂狗**, 不带心跳汇聚: 它只证明"调度器还在跑、本任务还能被
 *       切到". 抓不到"某个服务单独卡死而系统其余部分照跑" —— 要抓那个得让各
 *       服务上报心跳, 是另一个设计.
 *
 * @warning ★IWDG 在任务体内启动, 不在 main 里★: 启动阶段(LCD 重试、fatfs 首次
 *          挂载/格式化可达几十秒)不能计时, 否则上电就被打进复位循环. 也就是
 *          说**上电到看门狗任务首次被调度之间是无保护窗口**.
 *****************************************************************************/
#ifndef __CYWATCH_SERVICE_WATCHDOG_H__
#define __CYWATCH_SERVICE_WATCHDOG_H__

/***********************************Includes***********************************/
#include <stdint.h>
/***********************************Includes***********************************/

/**********************************Declaring***********************************/
/* 启动看门狗服务. 需在 osKernelInitialize() 之后、osKernelStart() 之前调.
   只建任务, IWDG 本身在任务体内启动(见头文件 @warning)
   @return  0  成功
           -1  "watchdog" 任务创建失败(内核堆不足) */
int8_t service_watchdog_init(void);
/**********************************Declaring***********************************/

#endif // __CYWATCH_SERVICE_WATCHDOG_H__
