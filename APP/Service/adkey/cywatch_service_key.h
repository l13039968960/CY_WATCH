/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_service_key.h
 *
 * @par dependencies
 * - stdint.h
 *
 * @author zw1194
 *
 * @brief AD 按键服务: 建一个常驻 "key" 任务, 周期读键, 识别单击/双击/长按并发事件。
 *
 * Processing flow:
 *
 * service_key_init()  : 建 "key" 任务(不碰设备)
 * "key" 任务体        : key_bsp_inst() → 循环 { key_bsp_read_key(); 状态机推进;
 *                        osDelay(周期) }
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 本服务上报"哪个键、什么手势": 事件 ID 固定是 EVT_SERVICE_KEY_PRESS,
 *       键值(1/2/3)在 event_flags, 手势(service_key_action_t)按值塞在 event_data
 *       里 —— 消费方取回用 (service_key_action_t)(uint32_t)event->event_data。
 *       按键的业务含义(切页/返回/唤醒)由上层消费者决定。
 * @note 手势都是**单键**的: 双击要求两次按的是同一个键; 双击窗口内按下别的键时,
 *       前一下按单击定案, 这一下另起一次(不会被吞掉)。
 *****************************************************************************/
#ifndef __CYWATCH_SERVICE_KEY_H__
#define __CYWATCH_SERVICE_KEY_H__

/***********************************Includes***********************************/
#include <stdint.h>
/***********************************Includes***********************************/

/***********************************Defines************************************/
#define SERVICE_KEY_OK         (0)

/* 轮询周期(ms). 必须大于驱动的 ADKEY_DEBOUNCE_MS(20), 否则去抖复采会把整个周期
   占满, 按键响应反而变迟钝。周期越短按下/松手采得越准(双击越不容易漏), 代价是
   ADC(与电源电压检测共用)的采样次数成正比上涨 */
#define SERVICE_KEY_POLL_MS    (50U)

/* 手势门限(ms). 两者都必须是 SERVICE_KEY_POLL_MS 的整数倍 —— 状态机按轮询次数计时,
   除不尽会向下取整, 实际门限比标称值短 */
#define SERVICE_KEY_LONG_MS    (1000U) /* 按住这么久算长按(只发一次) */
#define SERVICE_KEY_DBL_MS     (300U)  /* 松手后这么久内没有第二次按下就算单击
                                          —— 它同时就是单击的响应延迟, 别为了好按调大 */

#define SERVICE_KEY_TASK_STACK (1024U) /* 与 rtstats 的周期打印任务同量级 */
/***********************************Defines************************************/

/**********************************Declaring***********************************/
/* 按键手势: 发 EVT_SERVICE_KEY_N_ACTION 时放在 event_flags 里
   @note 本枚举由**生产方**(本服务)定义, 消费方 include 本头即可, 不要各页再抄一份 */
typedef enum
{
    SERVICE_KEY_ACTION_CLICK  = 0, /* 单击 */
    SERVICE_KEY_ACTION_DOUBLE,     /* 双击 */
    SERVICE_KEY_ACTION_LONG,       /* 长按 */
}service_key_action_t;

/* 按键服务初始化: 创建 "key" 任务 */
void service_key_init(void);
/**********************************Declaring***********************************/

#endif // __CYWATCH_SERVICE_KEY_H__
