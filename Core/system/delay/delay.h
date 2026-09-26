/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file delay.h
 *
 * @par dependencies
 * - stdint.h
 *
 * @author zw1194
 *
 * @brief Provide the DWT-based delay system functions.
 *
 * Processing flow:
 *
 * call delay_init() after SystemClock_Config(), then use delay_us()/delay_ms().
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
#ifndef __DELAY_H__
#define __DELAY_H__

/***********************************Includes***********************************/
#include <stdint.h>

/***********************************Includes***********************************/

/**********************************Declaring***********************************/
/* 延时初始化(基于DWT周期计数器, 需在系统时钟配置完成后调用) */
void delay_init(void);

/* 微秒延时 */
void delay_us(uint32_t us);

/* 毫秒延时 */
void delay_ms(uint32_t ms);

/**********************************Declaring***********************************/

#endif // __DELAY_H__
