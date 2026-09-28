/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file gpio_hal.h
 *
 * @par dependencies
 * - stm32f4xx_hal.h
 * - stdint.h
 *
 * @author zw1194
 *
 * @brief Provide the OO-style HAL APIs of GPIO and corresponding opetions.
 *
 * Processing flow:
 *
 * call gpio_driver_inst() to construct, then use function pointers.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
#ifndef __GPIO_HAL_H__
#define __GPIO_HAL_H__

/***********************************Includes***********************************/
#include "stm32f4xx_hal.h"
#include <stdint.h>

/***********************************Includes***********************************/

/**********************************Declaring***********************************/
/* GPIO引脚硬件配置 */
typedef struct
{
	GPIO_TypeDef *p_port; /* GPIO端口 */
	uint16_t pin;         /* 引脚号 */
	uint32_t mode;        /* 模式(输入/输出/复用/模拟) */
	uint32_t pull;        /* 上下拉 */
	uint32_t speed;       /* 输出速度 */
} gpio_cfg_t;

/* 延时接口 (由调用方注入, 与IIC/SPI/EXTI一致) */
typedef struct
{
	void (*pf_delay_us)(uint32_t us); /* 微秒延时 */
} gpio_delay_interface_t;

/* GPIO驱动对象 */
typedef struct gpio_driver
{
	gpio_cfg_t cfg; /* 引脚硬件配置 */

	gpio_delay_interface_t *p_delay_interface; /* 延时接口(由调用方注入) */

	/* 构造与析构 */
	int8_t (*pf_inst)(struct gpio_driver *p_gpio_instance,
					  gpio_cfg_t *p_cfg,
					  gpio_delay_interface_t *p_delay_interface);
	int8_t (*pf_deinst)(struct gpio_driver *p_gpio_instance);

	/* 底层GPIO操作 (void *匹配上层接口) */
	int8_t (*pf_write)(void *p_gpio_instance, uint8_t level);
	uint8_t (*pf_read)(void *p_gpio_instance);
	int8_t (*pf_toggle)(void *p_gpio_instance);

} gpio_driver_t;

/* GPIO驱动构造函数 */
int8_t gpio_driver_inst(gpio_driver_t *p_gpio_instance,
						gpio_cfg_t *p_cfg,
						gpio_delay_interface_t *p_delay_interface);

/**********************************Declaring***********************************/

#endif // __GPIO_HAL_H__
