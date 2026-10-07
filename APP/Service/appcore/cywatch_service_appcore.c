/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_service_appcore.c
 *
 * @author zw1194
 *
 * @brief app_core 服务: EasyAPP 事件总线唯一的消费者. 每 10ms 把队列里的事件
 *        取出来分发给各 APP 页面的处理函数, 处理函数跑在本任务上下文里.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 移植自 Dirver_Test/service/app_core/cywatch_service_appcore.c, 逻辑未改.
 *****************************************************************************/
#include "cywatch_service_appcore.h"
#include "easyapp_port.h"
#include "cmsis_os2.h"

static const osThreadAttr_t g_service_appcore_attr =
{
    .name       = "appcore",
    .attr_bits  = 0,
    .cb_mem     = NULL,
    .cb_size    = 0,
    .stack_mem  = NULL,
    .stack_size = 2048,
    .priority   = osPriorityNormal,
};

static void service_appcore_run(void *pvParameters)
{
    (void)pvParameters;

    while(1)
    {
        for(uint8_t i = 0; i < 5; i++)
            easyapp_core_run();
        osDelay(10);
    }
}

void service_appcore_init(void)
{
    if (NULL == osThreadNew(service_appcore_run, NULL, &g_service_appcore_attr))
    {
        /* 任务创建失败(堆不足等), 无事件分发可跑 */
        for (;;)
        {
        }
    }
}
