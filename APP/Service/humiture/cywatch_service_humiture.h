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

/* 内部状态枚举: 只服务内部状态机用, 不是页面收的事件.
   @note 状态值必须带 HUMITURE_ 前缀: C 的枚举常量是文件作用域, 而 main.c 同时
         include 多个服务头, 不加前缀会 "redefinition of enumerator" 编译失败 */
typedef enum{
    EVT_HUMITURE_INIT = 0,
    EVT_HUMITURE_MEASURE,
    EVT_HUMITURE_ERROR,
}State_Humiture_Service_t;

/* 一帧温湿度数据. EasyAPP 事件 EVT_SERVICE_HUMITURE_DATA 的 event_data 即指向
   本类型 —— 指向服务内部持有的一份"静态最新帧", APP 层按只读使用, 不要 free/
   改所有权; 该帧每测量周期被本任务覆盖, 消费侧应尽快读走 */
typedef struct Humiture_Data
{
    float temperature; /* 温度, ℃ */
    float humidity;    /* 相对湿度, %RH */
} Humiture_Data_t;

/**
 * @name  service_humiture_init
 * @brief 温湿度服务初始化: 创建服务任务
 * @return 无
 */
void service_humiture_init(void);

#endif // __CYWATCH_SERVICE_HUMITURE_H__
