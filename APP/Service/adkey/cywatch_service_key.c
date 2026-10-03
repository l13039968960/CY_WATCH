/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_service_key.c
 *
 * @par dependencies
 * - cywatch_log.h
 * - cywatch_service_key.h
 * - cywatch_adapter_adkey.h
 * - cmsis_os2.h
 * - stdio.h
 *
 * @author zw1194
 *
 * @brief Implete the AD key service: 周期读键, 变化时打印。
 *
 * Processing flow:
 *
 * service_key_init() 建 "key" 任务; 任务体内 key_bsp_inst() 之后进入
 * while(1) { key_bsp_read_key(); osDelay(SERVICE_KEY_POLL_MS); }
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note key_bsp_inst() 内含 ADC 初始化与采样自检, 都要在任务上下文做, 所以设备
 *       构造放在任务体第一句而不是 service_key_init() 里。
 *****************************************************************************/
#include "cywatch_service_key.h"

#include "cywatch_adapter_adkey.h"
#include "cmsis_os2.h"
#include "system/log/cywatch_log.h" /* log_printf() */
#include <stdio.h>

/**********************************Variables***********************************/
/* 服务状态. uint8_t: Cortex-M4 上单字节读写天然原子, 不需要临界区 */
static uint8_t s_inited = 0U; /* service_key_init() 是否已经建过任务 */

/* 任务属性. 优先级压到 Low: 它只是个调试打印口, 不该跟 LVGL 抢 */
static const osThreadAttr_t g_key_attr =
{
    .name       = "key",
    .attr_bits  = 0U,
    .cb_mem     = NULL,
    .cb_size    = 0U,
    .stack_mem  = NULL,
    .stack_size = SERVICE_KEY_TASK_STACK,
    .priority   = osPriorityLow,
};
/**********************************Variables***********************************/

/******************************Static Functions********************************/
/******************************************************************************
 * @name    service_key_task
 * @brief   "key" 任务体: 构造设备 → 周期读键 → 键号变化时打印
 * @param   p_arg[in] 未使用
 *
 * @return  无(初始化失败时 osThreadExit)
 *
 * @note    只打印"变化": 每周期都打会把串口刷爆, 而按下与松开的跳变才是要看的东西。
 * @note    稳态下 last_key 初值取 0(无按键), 所以上电时若没按着键, 第一轮不打印。
 *****************************************************************************/
static void service_key_task(void *p_arg)
{
    uint16_t key      = 0U; /* 本轮键号 */
    uint16_t last_key = 0U; /* 上轮键号 */
    int8_t   rc;

    (void)p_arg;

    rc = key_bsp_inst();
    if (SERVICE_KEY_OK != rc)
    {
        log_printf("[KEY] key_bsp_inst 失败 rc=%d "
               "(-1 ADC实例 -4 ADC初始化 -5 采样自检; 查 PA2 接线与 adc_hal 的引脚cfg)\r\n",
               (int)rc);
        osThreadExit();
    }

    log_printf("[KEY] 初始化完成, 每 %u ms 轮询一次(串口 115200)\r\n",
           (unsigned)SERVICE_KEY_POLL_MS);

    while (1)
    {
        if (SERVICE_KEY_OK == key_bsp_read_key(&key))
        {
            if (key != last_key)
            {
                if (0U == key)
                {
                    log_printf("[KEY] 松开\r\n");
                }
                else
                {
                    log_printf("[KEY] 键号 %u\r\n", (unsigned)key);
                }

                last_key = key;
            }
        }

        osDelay(SERVICE_KEY_POLL_MS);
    }
}
/******************************Static Functions********************************/

/**********************************Functions***********************************/
int8_t service_key_init(void)
{
    /* 幂等: 重复调用直接当成功, 不会建出第二个任务 */
    if (0U != s_inited)
    {
        return SERVICE_KEY_OK;
    }

    if (NULL == osThreadNew(service_key_task, NULL, &g_key_attr))
    {
        return SERVICE_KEY_ERR_THREAD;
    }

    s_inited = 1U;
    return SERVICE_KEY_OK;
}
/**********************************Functions***********************************/
