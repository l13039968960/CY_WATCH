/******************************************************************************
 * Copyright (C) 2026 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_service_power.h
 *
 * @author zw1194
 *
 * @brief 电源服务(生产者): 每 10s 采一次电池电压, 结果打包成一帧经 EasyAPP
 *        事件发布; 充电状态(PB1)当前按同一拍轮询, 翻转时单独发一条事件.
 *
 * Processing flow:
 *
 * 1. main.c 在 osKernelStart() 之前调 service_power_init() —— 只建任务;
 * 2. 任务体内占住共享 ADC1 与 PB1(最多重试5次) → 每 10s 采一次并发布
 *    EVT_SERVICE_POWER_DATA;
 * 3. 需要当前值的模块也可以直接调 service_power_get_mv(), 不必订阅事件.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 本服务与 ADKEY 共用 ADC1(adc_hal 按 ref_count 门控, 互斥量串行化),
 *       电池分压常接、无使能脚 —— 所以本服务**没有** sleep/wakeup 接口.
 *
 * @note 另有 EVT_SERVICE_POWER_CHARGING: **只在充电状态翻转时发一次**(状态不变不发,
 *       不像 DATA 那样每拍都发). 1=充电中(PB1 低电平)/ 0=未充电 放 event_flags,
 *       event_data 为 NULL. 上电后的第一次判定必定会发一帧.
 *       ★节奏当前与 DATA 相同★: 都挂在 10s 那一拍上, 插上充电器最多 10s 才反映.
 *       PB1 的 EXTI 已配好并在跑, 只是暂不拿它当唤醒源 —— 见 .c 里那句 osDelay.
 *****************************************************************************/
#ifndef __CYWATCH_SERVICE_POWER_H__
#define __CYWATCH_SERVICE_POWER_H__

/***********************************Includes***********************************/
#include <stdint.h>
/***********************************Includes***********************************/

/**********************************Declaring***********************************/
/* 内部状态枚举: 只服务内部状态机用, 不是页面收的事件.
   @note 状态值必须带 POWER_ 前缀: C 的枚举常量是文件作用域, 而 main.c 同时
         include 多个服务头, 不加前缀会 "redefinition of enumerator" 编译失败 */
typedef enum{
    EVT_POWER_INIT = 0,
    EVT_POWER_MEASURE,
    EVT_POWER_ERROR,
}State_Power_Service_t;

/* 一帧电源数据. EasyAPP 事件 EVT_SERVICE_POWER_DATA 的 event_data 即指向本类型
   —— 指向服务内部持有的一份"静态最新帧", APP 层按只读使用, 不要 free/改所有权;
   该帧每测量周期被本任务覆盖, 消费侧应尽快读走 */
typedef struct Power_Data
{
    uint16_t voltage_mv; /* 电池电压, 整数毫伏(分压比与 VREF 已在 adapter 里算掉) */
} Power_Data_t;

/**
 * @name  service_power_init
 * @brief 电源服务初始化: 创建服务任务
 * @return 无
 */
void service_power_init(void);

/* 取最近一次采到的电池电压(整数毫伏).
 * @note 不发起新采样, 只是把服务任务最近发布的那一帧拷出来 —— 服务还没跑完
 *       第一轮时读到的是 0
 * @return 0 success */
int8_t service_power_get_mv(uint16_t *p_mv);

/**********************************Declaring***********************************/

#endif // __CYWATCH_SERVICE_POWER_H__
