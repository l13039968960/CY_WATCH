/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_service_HeartRate.h
 *
 * @par dependencies
 * - ServiceFunction/HeartRate.h
 * - ServiceFunction/SpO2.h
 * - stdint.h
 *
 * @author	zw1194
 *
 * @brief 心率/血氧服务(生产者): 独占 MAX30102, 在 SpO2 模式下同时采 RED+IR
 *        两路 PPG, 分别喂给两个纯算法模块, 结果打包成一帧经 EasyAPP 事件发布.
 *
 * Processing flow:
 *
 * 服务任务: 等 FIFO 满中断 → 突发读走整个 FIFO → 逐样本喂
 *   HeartRate 算法(IR 路) 与 SpO2 算法(RED+IR 两路) → 按
 *   EVT_SERVICE_HEARTRATE_DATA 发布最新帧(生产者持有, 0 拷贝).
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 为什么心率与血氧合在一个服务里, 而不是两个服务:
 *       1) MAX30102 的 SpO2 模式本来就一次给出 RED+IR 两路, 心率可由 IR 路算出,
 *          不需要两个任务轮流切 HR/SpO2 模式(切换要清 FIFO, 算法每次都得重新收敛);
 *       2) 软件 I2C 总线不可重入, 且驱动只有一份 static 实例, 两个任务各自
 *          heartrate_bsp_inst() 会互相踩踏.
 *
 *****************************************************************************/
#ifndef __CYWATCH_SERVICE_HEARTRATE_H__
#define __CYWATCH_SERVICE_HEARTRATE_H__

#include <stdint.h>

/* 帧内样本单位与算法说明见两个纯算法模块的头文件 */
#include "ServiceFunction/HeartRate.h"
#include "ServiceFunction/SpO2.h"

/**********************************Declaring***********************************/
/* 内部状态枚举: 只服务内部状态机用, 不是页面收的事件.
   @note 状态值必须带 HEARTRATE_ 前缀: C 的枚举常量是文件作用域, 姿态服务用的是
         无前缀的 EVT_INIT/EVT_SLEEP/..., 而 main.c 同时 include 两个服务头,
         不加前缀会 "redefinition of enumerator 'EVT_INIT'" 编译失败 */
typedef enum{
    EVT_HEARTRATE_INIT = 0,
    EVT_HEARTRATE_MEASURE,
    EVT_HEARTRATE_SLEEP,
    EVT_HEARTRATE_WAKEUP,
    EVT_HEARTRATE_NODONE,
    EVT_HEARTRATE_ERROR,
}State_HeartRate_Service_t;

/* 一帧心率/血氧数据. EasyAPP 事件 EVT_SERVICE_HEARTRATE_DATA 的 event_data
   即指向本类型——指向服务内部持有的一份"静态最新帧", APP 层按只读使用,
   不要 free/改所有权; 该帧每发布周期被本任务覆盖, 消费侧应尽快读走 */
typedef struct HeartRate_Data
{
    uint32_t red;            /* 本帧 RED 通道 18bit 原始计数 */
    uint32_t ir;             /* 本帧 IR  通道 18bit 原始计数 */
    float    heart_rate_bpm; /* 心率, bpm; finger_on==0 时无意义 */
    float    spo2_percent;   /* 血氧饱和度, %; finger_on==0 时无意义 */
    uint8_t  finger_on;      /* 1 = 检测到手指贴合(由直流分量判定) */
} HeartRate_Data_t;

/**
 * @name  service_heartrate_init
 * @brief 心率/血氧服务初始化: 创建服务任务
 * @return 无
 */
void service_heartrate_init(void);

/**********************************Declaring***********************************/

#endif // __CYWATCH_SERVICE_HEARTRATE_H__
