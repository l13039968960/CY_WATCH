/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_service_humiture.h
 *
 * @author zw1194
 *
 * @brief 温湿度服务(生产者): 独占 AHT21, 每 10s 触发一次测量, 结果打包成一帧
 *        经 EasyAPP 事件发布.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
#ifndef __CYWATCH_SERVICE_HUMITURE_H__
#define __CYWATCH_SERVICE_HUMITURE_H__

#include <stdint.h>

/* 一帧温湿度数据. EasyAPP 事件 EVT_SERVICE_HUMITURE_DATA 的 event_data 即指向
   本类型 —— 指向服务内部持有的一份"静态最新帧", APP 层按只读使用, 不要 free/
   改所有权; 该帧每测量周期被本任务覆盖, 消费侧应尽快读走 */
typedef struct Humiture_Data
{
    float temperature; /* 温度, ℃ */
    float humidity;    /* 相对湿度, %RH */
} Humiture_Data_t;

/* 温湿度服务初始化: 创建服务任务 */
void service_humiture_init(void);

/* 读"最新一帧"的拷贝快照. 内部锁调度器整份拷贝, 拿到的那份不会被下一个测量周期
 * 覆盖, 也不会跨帧撕裂. 服务未初始化时返回全 0. */
void service_humiture_get_data(Humiture_Data_t *p_out);

/* 休眠/唤醒: AHT21 软复位并释放共享 I2C 上本设备那一份占用.
 * 只置状态位, 服务任务在本轮 osDelay(最长 SERVICE_HUMITURE_PERIOD_MS)结束后
 * 的下一次 switch 才真正睡下; 唤醒走 EVT_HUMITURE_INIT 重走一遍构造 */
void service_humiture_sleep(void);
void service_humiture_wakeup(void);

#endif // __CYWATCH_SERVICE_HUMITURE_H__
