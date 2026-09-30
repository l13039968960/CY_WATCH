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
 * @brief AD 按键服务: 建一个常驻 "key" 任务, 周期读键, 键号变化时打印。
 *
 * Processing flow:
 *
 * service_key_init()  : 建 "key" 任务(不碰设备)
 * "key" 任务体        : key_bsp_inst() → 循环 { key_bsp_read_key(); osDelay(周期) }
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 本服务只做"轮询 + 打印", 不含任何按键业务动作 —— 键号怎么用(切页/返回/唤醒)
 *       由上层消费者决定, 这里只把变化打到串口, 便于接线核对与阈值标定。
 *****************************************************************************/
#ifndef __CYWATCH_SERVICE_KEY_H__
#define __CYWATCH_SERVICE_KEY_H__

/***********************************Includes***********************************/
#include <stdint.h>
/***********************************Includes***********************************/

/***********************************Defines************************************/
#define SERVICE_KEY_OK         (0)
#define SERVICE_KEY_ERR_THREAD (-1) /* 建 "key" 任务失败(内核堆不足) */

/* 轮询周期(ms). 必须大于驱动的 ADKEY_DEBOUNCE_MS(20), 否则去抖复采会把整个周期
   占满, 按键响应反而变迟钝 */
#define SERVICE_KEY_POLL_MS    (100U)

#define SERVICE_KEY_TASK_STACK (1024U) /* 与 rtstats 的周期打印任务同量级 */
/***********************************Defines************************************/

/**********************************Declaring***********************************/
/* 建 "key" 任务(幂等: 重复调用直接当成功).
   返回 SERVICE_KEY_OK / SERVICE_KEY_ERR_THREAD */
int8_t service_key_init(void);
/**********************************Declaring***********************************/

#endif // __CYWATCH_SERVICE_KEY_H__
