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
 * call gpio_driver_inst() to construct, pf_init() to claim the pins,
 * then use function pointers.
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
	uint16_t pins;        /* 引脚掩码(GPIO_PIN_x 按位或), 可一次配同端口一组同参数脚 */
	uint32_t mode;        /* 模式(输入/输出/复用/模拟/中断) */
	uint32_t pull;        /* 上下拉 */
	uint32_t speed;       /* 输出速度 */
	uint32_t af;          /* 复用号(GPIO_AFx_xxx), 仅复用模式用 */
} gpio_cfg_t;

/* GPIO驱动对象 */
typedef struct gpio_driver
{
	gpio_cfg_t cfg; /* 引脚硬件配置 */

	uint8_t init_state; /* 0=deinit, 1=init */
	uint8_t ref_count;  /* 使用者计数: 0→1 才真正配引脚, 减到 0 才真正收尾 */

	/* 构造与析构: inst 只挂函数指针, 不碰硬件 */
	int8_t (*pf_inst)(struct gpio_driver *p_gpio_instance, gpio_cfg_t *p_cfg);
	int8_t (*pf_deinst)(struct gpio_driver *p_gpio_instance);

	/* 引脚占用/释放. deinit 带低功耗语义: 先 HAL_GPIO_DeInit 复位,
	   再配模拟输入关掉输入缓冲; 端口时钟永不关( ) */
	int8_t (*pf_init)(struct gpio_driver *p_gpio_instance);
	int8_t (*pf_deinit)(struct gpio_driver *p_gpio_instance);

	/* 运行时改模式/上下拉(IIC 要在事务中反复切 SDA 方向).
	   只改 mode/pull, pins 与 speed/af 沿用 cfg; 不写回 cfg, 属临时状态 */
	int8_t (*pf_set_mode)(struct gpio_driver *p_gpio_instance,
						  uint32_t mode,
						  uint32_t pull);

	/* 底层GPIO操作 */
	int8_t (*pf_write)(struct gpio_driver *p_gpio_instance, uint8_t level);
	uint8_t (*pf_read)(struct gpio_driver *p_gpio_instance);
	int8_t (*pf_toggle)(struct gpio_driver *p_gpio_instance);

} gpio_driver_t;

/* GPIO驱动构造函数 */
int8_t gpio_driver_inst(gpio_driver_t *p_gpio_instance, gpio_cfg_t *p_cfg);

/**********************************Declaring***********************************/

#endif // __GPIO_HAL_H__
