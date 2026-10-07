/******************************************************************************
 * Copyright (C) 2026 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_service_watchdog.c
 *
 * @par dependencies
 * - cywatch_service_watchdog.h
 * - cywatch_watchdog.h
 * - cmsis_os2.h
 * - system/log/cywatch_log.h
 *
 * @author zw1194
 *
 * @brief 看门狗服务的实现: 一个高优先级任务, 周期喂 IWDG.
 *
 * Processing flow:
 *
 * service_watchdog_run: cywatch_watchdog_init() → while(1){ feed; osDelay(1s) }
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 优先级取 osPriorityHigh(高于 fatfs/attitude/humiture 的 Normal, 也高于
 *       nordic_rx 的 AboveNormal): 喂狗任务只做两件事, 必须随时抢得到 CPU ——
 *       尤其 fatfs 整片格式化是几十秒的连续 SPI 阻塞, 优先级不够就轮不到它.
 *       @note 它不是最高档(Realtime): 没必要跟中断抢, 1s 的余量足够.
 *****************************************************************************/
#include "cywatch_service_watchdog.h"

#include "cywatch_watchdog.h"        /* cywatch_watchdog_init / _feed */
#include "cmsis_os2.h"
#include "system/log/cywatch_log.h"  /* log_printf() */

/***********************************Defines************************************/
/* 喂狗周期. IWDG 标称超时 4.096s, 但 LSI 容差大(真实 2.8~7.7s), 取 1/4 留余量:
   即使 LSI 偏到最短的 2.8s, 也还能喂上两次. 改大前先看 cywatch_watchdog.h */
#define SERVICE_WATCHDOG_FEED_TICK_MS   (1000u)
/***********************************Defines************************************/

/**********************************Declaring***********************************/
static const osThreadAttr_t g_service_watchdog_attr =
{
    .name       = "watchdog",
    .attr_bits  = 0,
    .cb_mem     = NULL,
    .cb_size    = 0,
    .stack_mem  = NULL,
    .stack_size = 1024,
    .priority   = osPriorityHigh,
};

static void service_watchdog_run(void *pvParameters);
/**********************************Declaring***********************************/

/******************************************************************************
 * @name    service_watchdog_run
 * @brief   看门狗任务: 启动 IWDG 后按 1s 周期喂狗
 * @param   pvParameters[in] 未使用
 *
 * @return  无(不返回)
 *
 * @note    先启动再进循环是刻意的 —— main.c 那边建任务时就启动的话, 启动阶段
 *          (LCD 重试、fatfs 挂载/首次格式化)会被计时(见头文件 @warning)
 *****************************************************************************/
static void service_watchdog_run(void *pvParameters)
{
    (void)pvParameters;

    if (0 != cywatch_watchdog_init())
    {
        /* 只报一句, 不特殊处理: HAL_IWDG_Init 是"先写 KR 启动、再等寄存器同步",
           超时说明 PR/RLR 还没落下去, 但看门狗很可能已经在跑了 —— 此时继续
           喂狗是唯一能做的事 */
        log_printf("WATCHDOG: IWDG init failed\r\n");
    }

    while (1)
    {
        cywatch_watchdog_feed();
        osDelay(SERVICE_WATCHDOG_FEED_TICK_MS);
    }
}

/******************************************************************************
 * @name    service_watchdog_init
 * @brief   启动看门狗服务(只建任务, IWDG 在任务体内启动)
 * @param   无
 *
 * @return  0 成功 / -1 任务创建失败(内核堆不足)
 *****************************************************************************/
int8_t service_watchdog_init(void)
{
    if (NULL == osThreadNew(service_watchdog_run, NULL, &g_service_watchdog_attr))
    {
        log_printf("WATCHDOG: osThreadNew failed\r\n");
        return -1;
    }

    return 0;
}
