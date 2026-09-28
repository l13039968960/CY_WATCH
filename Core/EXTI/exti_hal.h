/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file exti_hal.h
 *
 * @par dependencies
 * - stm32f4xx_hal.h
 * - stdint.h
 *
 * @author zw1194
 *
 * @brief Provide the OO-style HAL APIs of EXTI external interrupt and corresponding opetions.
 *
 * Processing flow:
 *
 * call exti_driver_inst() to construct, attach callback, then use function pointers.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
#ifndef __EXTI_HAL_H__
#define __EXTI_HAL_H__

/***********************************Includes***********************************/
#include "stm32f4xx_hal.h"
#include <stdint.h>

/***********************************Includes***********************************/

/**********************************Declaring***********************************/
/* EXTI外部中断硬件配置 */
typedef struct
{
	GPIO_TypeDef *p_port;      /* GPIO端口 */
	uint16_t pin;              /* 引脚号(GPIO_PIN_x, 同时决定EXTI线号0-15) */
	uint32_t mode;             /* 触发模式(GPIO_MODE_IT_FALLING/RISING/RISING_FALLING) */
	uint32_t pull;             /* 上下拉 */
	IRQn_Type irqn;            /* NVIC中断号(如EXTI3_IRQn) */
	uint32_t preempt_priority; /* 抢占优先级 */
	uint32_t sub_priority;     /* 子优先级 */
} exti_cfg_t;

/* 延时接口 (由调用方注入, 与IIC/GPIO/SPI一致) */
typedef struct
{
	void (*pf_delay_us)(uint32_t us); /* 微秒延时 */
} exti_delay_interface_t;

/* EXTI驱动对象 */
typedef struct exti_driver
{
	exti_cfg_t cfg; /* 硬件配置 */

	exti_delay_interface_t *p_delay_interface; /* 延时接口(由调用方注入) */

	void (*pf_interrupt_cb)(void *p_ctx); /* 上层中断回调(ISR中调用, 仅置标志) */

	/* 构造与析构 */
	int8_t (*pf_inst)(struct exti_driver *p_exti_instance,
					  exti_cfg_t *p_cfg,
					  exti_delay_interface_t *p_delay_interface);
	int8_t (*pf_deinst)(struct exti_driver *p_exti_instance);

	/* 底层EXTI操作 (void *匹配上层接口) */
	int8_t (*pf_enable_interrupt)(void *p_exti_instance);
	int8_t (*pf_disable_interrupt)(void *p_exti_instance);
	int8_t (*pf_attach_callback)(void *p_exti_instance,
								 void (*pf_interrupt_cb)(void *p_ctx));

} exti_driver_t;

/* EXTI驱动构造函数 */
int8_t exti_driver_inst(exti_driver_t *p_exti_instance,
						exti_cfg_t *p_cfg,
						exti_delay_interface_t *p_delay_interface);

/* EXTI中断分发: 由应用层 EXTIx_IRQHandler 调用, 清挂起并按线号反查实例回调 */
void exti_irq_handler(uint16_t pin);

/**********************************Declaring***********************************/

#endif // __EXTI_HAL_H__
