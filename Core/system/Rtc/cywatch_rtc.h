/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_rtc.h
 *
 * @par dependencies
 * - stdint.h
 *
 * @author	zw1194
 *
 * @brief 系统时间源: 驱动 STM32 的 RTC 外设(时钟源 = LSE 32.768kHz), 对外
 *        提供取/设接口, 供 FatFs 等需要时间戳的模块使用.
 *
 * Processing flow:
 *
 * 1. cywatch_rtc_init() 上电调一次(main 在 osKernelStart 之前调): 起 LSE +
 *    选 RTC 时钟源 + 只在"第一次上电"时把日历种成 2025-01-01 00:00:00.
 *    返回非 0 表示 LSE 没起来 / 时间不可信, 由调用方决定是否停机;
 * 2. 时间源(协议对时)拿到可信时间后调 cywatch_rtc_set();
 * 3. 需要时间戳的模块调 cywatch_rtc_get() 拿结构体, 或调
 *    cywatch_rtc_get_packed() 拿 DOS/FAT 编码的 32 位值.
 *
 * @version V2.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 本模块的**接口**(本头文件)仍然零依赖: 只认 stdint.h 与自己那套时间
 *       类型, 不 include FatFs / HAL / CMSIS 任何头. 但**实现**(cywatch_rtc.c)
 *       现在依赖 STM32 HAL 的 RTC 驱动(stm32f4xx_hal_rtc.c)与 CMSIS 设备头 ——
 *       HAL 类型(RTC_HandleTypeDef)不泄漏到本头文件, 所以调用方仍然一行都
 *       不用改.
 *
 * @note 时间值本身仍然**不含日历运算**(不判闰年、不累计天数): 年月日时分秒
 *       是硬件 RTC 各自独立累加的, 取出来直接就是字段值. 唯一的日历运算是
 *       cywatch_rtc_set() 内部用 Zeller 同余算 RTC_DR 的 WeekDay 字段 ——
 *       那个字段**不参与硬件走时**, 只读回给人看, 算错不影响时间.
 *
 * @warning 时间的保持靠的是**备份域**: VDD 不掉时 RTC 日历照走. 所以:
 *            - NRST 热复位 / 软件复位 / 看门狗复位: 时间不丢, 继续走
 *              (靠 RTC_ISR_INITS 判断出"日历已有效", 不重新种值);
 *            - VDD 掉电且 VBAT 没有后备电源(没焊纽扣电池/超级电容): 备份域
 *              复位, 日历被清, 下次上电重新种成 2025-01-01 00:00:00.
 *          也就是说, 没有后备电源时"跨断电保持时间"是不成立的 —— 上电后如果
 *          没有人调 cywatch_rtc_set() 对一次时, 文件时间戳就是 2025-01-01.
 *****************************************************************************/
#ifndef __CYWATCH_RTC_H__
#define __CYWATCH_RTC_H__

#include <stdint.h>

/**********************************Declaring***********************************/

/* 本地时间(无时区概念) */
typedef struct cywatch_rtc_time
{
    uint16_t year;   /* 完整年份, 如 2026; 必须在 2000 ~ 2099(硬件 RTC 只存两位年) */
    uint8_t  month;  /* 1 ~ 12 */
    uint8_t  day;    /* 1 ~ 31 */
    uint8_t  hour;   /* 0 ~ 23 */
    uint8_t  minute; /* 0 ~ 59 */
    uint8_t  second; /* 0 ~ 59(硬件精确到 1 秒; 只有 get_packed 打包成 FAT 格式时才截成 2 秒粒度) */
} cywatch_rtc_time_t;

/* 初始化 RTC: LSE 起振 + 选时钟源 + (仅首次上电)种入 2025-01-01 00:00:00.
   返回码阶梯见 .c. 需在 HAL_Init() 之后、osKernelStart() 之前调一次 */
int8_t cywatch_rtc_init(void);

/* 设置 RTC 当前时间. 时间源(协议对时)拿到可信时间后调. 需先 cywatch_rtc_init() 成功 */
int8_t cywatch_rtc_set(const cywatch_rtc_time_t *p_time);

/* 取 RTC 当前时间. 内部 GetTime+GetDate 成对, **不可重入** */
int8_t cywatch_rtc_get(cywatch_rtc_time_t *p_time);

/* 取已编码的当前时间(DOS/FAT 时间戳格式, 位域说明见 .c).
   @note 与 V1 的"单次对齐 32 位内存读, 天然原子"不同: 现在会真的读 RTC 外设
         (GetTime+GetDate 一对)再重新打包. 返回的值仍是一致的一组(不会半新半旧),
         但本函数**不可重入** —— 并发调用需调用方自备互斥 */
uint32_t cywatch_rtc_get_packed(void);

/**********************************Declaring***********************************/

#endif // __CYWATCH_RTC_H__
