/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file delay.c
 *
 * @par dependencies
 * - delay.h
 * - stm32f4xx_hal.h
 *
 * @author zw1194
 *
 * @brief Implete the DWT-based delay system functions.
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
#include "delay.h"
#include "stm32f4xx_hal.h"
#include "core_cm4.h"

/********************************* 私有变量 *********************************/
/* 每微秒的DWT计数次数, 在 delay_init() 中按实际主频计算 */
static uint32_t dwt_ticks_per_us = 0;

/********************************* 延时函数 *********************************/

/******************************************************************************
 * @name    delay_init
 * @brief   初始化DWT周期计数器, 计算每微秒计数次数
 *
 * @note    必须在系统时钟配置完成后(100MHz)再调用
 *****************************************************************************/
void delay_init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; /* 使能DWT调试跟踪 */
	DWT->CYCCNT = 0;                                /* 清零周期计数器 */
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            /* 启动周期计数器 */

	dwt_ticks_per_us = SystemCoreClock / 1000000U;  /* 每微秒的周期数 */
}

/******************************************************************************
 * @name    delay_us
 * @brief   微秒级延时
 * @param   us[in] 延时的微秒数
 *****************************************************************************/
void delay_us(uint32_t us)
{
	uint32_t start = DWT->CYCCNT;
	uint32_t ticks = us * dwt_ticks_per_us;

	while ((DWT->CYCCNT - start) < ticks)
		;
}

/******************************************************************************
 * @name    delay_ms
 * @brief   毫秒级延时
 * @param   ms[in] 延时的毫秒数
 *****************************************************************************/
void delay_ms(uint32_t ms)
{
	while (ms--)
	{
		delay_us(1000);
	}
}
