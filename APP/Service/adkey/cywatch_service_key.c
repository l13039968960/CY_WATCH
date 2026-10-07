/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_service_key.c
 *
 * @par dependencies
 * - cywatch_service_key.h
 * - cywatch_adapter_adkey.h
 * - cmsis_os2.h
 *
 * @author zw1194
 *
 * @brief Implete the AD key service: 周期读键, 识别单击/双击/长按。
 *
 * Processing flow:
 *
 * service_key_init() 建 "key" 任务; 任务体内 key_bsp_inst() 之后进入
 * while(1) { key_bsp_read_key(); 手势状态机推进并发事件; osDelay(周期) }
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
#include "easyapp_port.h"

/**********************************Variables***********************************/
/* 任务属性. 优先级压到 Low: 它只做轮询, 不该跟 LVGL 抢 */
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

/* 状态机计时单位: 一次轮询 = SERVICE_KEY_POLL_MS, 门限按轮询次数数 */
#define KEY_LONG_TICKS (SERVICE_KEY_LONG_MS / SERVICE_KEY_POLL_MS)
#define KEY_DBL_TICKS  (SERVICE_KEY_DBL_MS / SERVICE_KEY_POLL_MS)

/* 按键手势状态机 */
typedef enum
{
    KEY_STATE_IDLE = 0, /* 空闲: 没有键按下 */
    KEY_STATE_PRESS,    /* 按下中: 计时判长按 */
    KEY_STATE_WAIT_DBL, /* 已松手: 等双击窗口(双击要求两次是同一个键) */
    KEY_STATE_WAIT_UP,  /* 手势已发: 等松手再回空闲 */
}Key_State_t;

/******************************************************************************
 * @name    key_event_send
 * @brief   发按键事件: 事件 ID 固定, 键值放 event_flags, 手势放 event_data
 * @param   key[in]    键号(1..3)
 * @param   action[in] 手势(service_key_action_t)
 *
 * @return  无
 *
 * @note    event_data 是 void*, 手势按值塞进去((void *)(uint32_t)action), 消费方
 *          用 (service_key_action_t)(uint32_t)event->event_data 取回。它不指向任何
 *          变量, 所以不会被下一个手势改写。
 *****************************************************************************/
static void key_event_send(uint16_t key, service_key_action_t action)
{
    (void)x_port_easyapp_event_send(EVT_SERVICE_KEY_PRESS, (uint32_t)key,
                                    (void *)(uint32_t)action);
}

/******************************************************************************
 * @name    service_key_task
 * @brief   "key" 任务体: 构造设备 → 周期读键 → 手势状态机发事件
 * @param   p_arg[in] 未使用
 *
 * @return  无(初始化失败时 osThreadExit)
 *
 * @note    必须周期调用 key_bsp_read_key: 驱动侧的去抖采样靠它推进。
 * @note    驱动给的是**电平**(按住不放就一直返回同一键号, 没有边沿), 所以单击/
 *          双击/长按只能靠"采样值 + 时间"推。
 * @note    长按只发一次; 长按/双击发过之后要等松手才回空闲, 否则松手那一刻会被
 *          当成单击补发一条。
 *****************************************************************************/
static void service_key_task(void *p_arg)
{
    uint16_t    key     = 0U; /* 本轮键号 */
    uint16_t    cur_key = 0U; /* 当前手势归属的键号 */
    uint16_t    cnt     = 0U; /* 当前状态内的轮询计数 */
    int8_t      rc;
    Key_State_t state   = KEY_STATE_IDLE;

    (void)p_arg;

    rc = key_bsp_inst();
    if (SERVICE_KEY_OK != rc)
    {
        osThreadExit();
    }

    while (1)
    {
        (void)key_bsp_read_key(&key); /* 周期采样维持驱动去抖 */

        switch (state)
        {
            case KEY_STATE_IDLE:
                if (0U != key)
                {
                    cur_key = key;
                    cnt     = 1U;
                    state   = KEY_STATE_PRESS;
                }
                break;

            case KEY_STATE_PRESS:
                /* 松手前键号变了(多键同按的残留)时不产生手势, 等松手 */
                if (0U == key)
                {
                    /* 松手: 还分不清单击还是双击, 进等待窗口 */
                    cnt   = 0U;
                    state = KEY_STATE_WAIT_DBL;
                }
                else if (key == cur_key)
                {
                    cnt++;
                    if (cnt >= KEY_LONG_TICKS)
                    {
                        key_event_send(cur_key, SERVICE_KEY_ACTION_LONG);
                        state = KEY_STATE_WAIT_UP;
                    }
                }
                break;

            case KEY_STATE_WAIT_DBL:
                if (0U != key)
                {
                    if (key == cur_key)
                    {
                        key_event_send(cur_key, SERVICE_KEY_ACTION_DOUBLE);
                        state = KEY_STATE_WAIT_UP;
                    }
                    else
                    {
                        /* 窗口内按下的是别的键: 前一下按单击定案, 这一下另起一次 */
                        key_event_send(cur_key, SERVICE_KEY_ACTION_CLICK);
                        cur_key = key;
                        cnt     = 1U;
                        state   = KEY_STATE_PRESS;
                    }
                }
                else if (++cnt >= KEY_DBL_TICKS)
                {
                    key_event_send(cur_key, SERVICE_KEY_ACTION_CLICK);
                    state = KEY_STATE_IDLE;
                }
                break;

            case KEY_STATE_WAIT_UP:
                if (0U == key)
                {
                    state = KEY_STATE_IDLE;
                }
                break;

            default:
                state = KEY_STATE_IDLE;
                break;
        }

        osDelay(SERVICE_KEY_POLL_MS);
    }
}
/******************************Static Functions********************************/

/**********************************Functions***********************************/
void service_key_init(void)
{
    if (NULL == osThreadNew(service_key_task, NULL, &g_key_attr))
    {
        /* 任务创建失败(堆不足等), 无按键服务可跑 */
        for (;;)
        {
        }
    }
}
/**********************************Functions***********************************/
