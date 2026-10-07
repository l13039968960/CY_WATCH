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
 * @brief 心率/血氧服务(生产者): 独占 MAX30102, 默认 HR 模式(仅 IR)只跑心率;
 *        血氧测量期间由 service_heartrate_set_spo2_enable() 切到 SpO2 模式
 *        (RED+IR 双路), 结果打包成一帧经 EasyAPP 事件发布.
 *
 * Processing flow:
 *
 * 服务任务: 等 FIFO 满中断 → 突发读走整个 FIFO → 逐样本喂
 *   HeartRate 算法(IR 路); 血氧开着时同时喂 SpO2 算法(RED+IR 两路) → 按
 *   EVT_SERVICE_HEARTRATE_DATA 发布最新帧(生产者持有, 0 拷贝).
 *
 * 模式切换: 血氧服务调 set_spo2_enable() 只置请求位, 真正的写 I2C + 清 FIFO
 *   由本任务在 MEASURE 循环里执行(见该函数的 @note).
 *
 * 另有 EVT_SERVICE_HEARTRATE_WEAR: **只在佩戴状态翻转时发一次**(状态不变不发,
 *   不像 DATA 那样每拍都发). 1=佩戴 / 0=未佩戴 放 event_flags, event_data 为 NULL.
 *   判定依据与 HeartRate_Data_t.finger_on 同源, 连续 3 拍(每拍 = 一个发布周期
 *   500ms)一致才认翻转; 上电后的第一次判定必定会发一帧.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 为什么心率与血氧合在一个服务里, 而不是两个服务:
 *       1) 两者共用同一个传感器与同一条 FIFO —— 工作模式是**器件级**的状态,
 *          分成两个服务就得写同一个 MODE 寄存器, 谁切谁负责说不清;
 *       2) 软件 I2C 总线不可重入, 且驱动只有一份 static 实例, 两个任务各自
 *          heartrate_bsp_inst() 会互相踩踏.
 *
 * @note ★模式是按需切的, 上电默认 HR(血氧关)★ 只有血氧测量期间才切到 SpO2.
 *       切换的代价(见 max30102_change_to_*): 驱动内部要停采样 + 清 FIFO + 重配
 *       模式, 期间 FIFO 断档 —— 心率算法会跳一拍, 血氧算法的窗口冻结(不丢, 只是
 *       不填, 所以重开后不从头收敛). RED 路关掉能省掉那个 LED 的功耗与一半的
 *       FIFO 读出量(6 字节/样本 → 3 字节/样本).
 *****************************************************************************/
#ifndef __CYWATCH_SERVICE_HEARTRATE_H__
#define __CYWATCH_SERVICE_HEARTRATE_H__

#include <stdint.h>

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

/* 心率/血氧服务初始化: 创建服务任务 */
void service_heartrate_init(void);

/* 读"最新一帧"的拷贝快照. 内部锁调度器整份拷贝, 拿到的那份不会被下一个发布周期覆盖,
 * 也不会跨帧撕裂. 服务未初始化时返回全 0. */
void service_heartrate_get_data(HeartRate_Data_t *p_out);

/* 血氧检测开关: 1=开(切 SpO2 模式, RED+IR 双路, 逐样本喂血氧算法);
 *             0=关(切回 HR 模式, 仅 IR, 同时把 spo2_percent 报 0).
 * ★只置请求位★ —— 真正的写 I2C + 清 FIFO 由心率任务在它自己的循环里执行, 所以要
 * 等下一次 FIFO 满(约 320ms)才生效, 不是立即. 开关状态是"目标值"而不是"动作",
 * 重复请求幂等, 也不会因为请求来得比响应快而丢.
 * 上电默认关: 只应由血氧测量服务在测量期间打开, 别的模块不要碰.
 * 可在任意任务上下文调用, 禁止 ISR. */
void service_heartrate_set_spo2_enable(uint8_t enable);

/* 请求休眠/唤醒: 只置状态位, 真正的 MAX30102 hibernating/wakeup 由心率任务执行, 与
 * service_attitudecalculation_sleep/wakeup 同一套路. 注意本服务在 MEASURE 里阻塞等
 * FIFO 满中断, 所以请求最慢要等下一次中断(约 320ms)才生效, 不是立即.
 * 可在任意任务上下文调用, 禁止 ISR; 重复请求幂等. */
void service_heartrate_sleep(void);
void service_heartrate_wakeup(void);

/**********************************Declaring***********************************/

#endif // __CYWATCH_SERVICE_HEARTRATE_H__
