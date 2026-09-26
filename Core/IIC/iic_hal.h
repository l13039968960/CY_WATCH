/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file iic_hal.h
 *
 * @par dependencies
 * - stm32f4xx_hal.h
 * - stdint.h
 *
 * @author zw1194
 *
 * @brief Provide the OO-style HAL APIs of software I2C and corresponding opetions.
 *
 * Processing flow:
 *
 * call iic_driver_inst() to construct, then use function pointers.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
#ifndef __IIC_HAL_H__
#define __IIC_HAL_H__

/***********************************Includes***********************************/
#include "stm32f4xx_hal.h"
#include <stdint.h>

/***********************************Includes***********************************/

/**********************************Declaring***********************************/
/* I2C总线硬件配置 */
typedef struct
{
	GPIO_TypeDef *p_sda_port;
	GPIO_TypeDef *p_scl_port;
	uint16_t sda_pin;
	uint16_t scl_pin;
} iic_bus_t;

/* 延时接口 (软件I2C时序需要微秒级延时, 由调用方注入) */
typedef struct
{
	void (*pf_delay_us)(uint32_t us); /* 微秒延时 */
} iic_delay_interface_t;

/* I2C驱动对象 */
typedef struct iic_driver
{
	iic_bus_t bus; /* 总线硬件配置 */

	iic_delay_interface_t *p_delay_interface; /* 延时接口(软件I2C时序) */

	/* 构造与析构 */
	int8_t (*pf_inst)(struct iic_driver *p_iic_instance,
					  iic_bus_t *p_bus,
					  iic_delay_interface_t *p_delay_interface);
	int8_t (*pf_deinst)(struct iic_driver *p_iic_instance);

	/* 底层I2C总线操作 */
	int8_t (*pf_start)(void *p_iic_instance);
	int8_t (*pf_stop)(void *p_iic_instance);
	int8_t (*pf_wait_ack)(void *p_iic_instance);
	int8_t (*pf_send_ack)(void *p_iic_instance);
	int8_t (*pf_send_not_ack)(void *p_iic_instance);
	int8_t (*pf_send_byte)(void *p_iic_instance, uint8_t byte);
	uint8_t (*pf_receive_byte)(void *p_iic_instance);

	/* 原始数据收发*/
	int8_t (*pf_send_bytes)(void *p_iic_instance,
							uint8_t *pdata,
							uint8_t size);
	int8_t (*pf_receive_bytes)(void *p_iic_instance,
							   uint8_t *pdata,
							   uint8_t size);

} iic_driver_t;

/* I2C驱动构造函数 */
int8_t iic_driver_inst(iic_driver_t *p_iic_instance,
					   iic_bus_t *p_bus,
					   iic_delay_interface_t *p_delay_interface);

/**********************************Declaring***********************************/

#endif // __IIC_HAL_H__
