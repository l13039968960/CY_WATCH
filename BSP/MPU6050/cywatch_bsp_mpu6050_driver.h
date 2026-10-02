/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_bsp_mpu6050_driver.h
 *
 * @par dependencies
 * - cywatch_bsp_mpu6050_reg.h
 * - stdio.h
 * - stdint.h
 *
 * @author	zw1194
 *
 * @brief Provide the HAL APIs of AHT21 and corresponding opetions.
 *
 * Processing flow:
 *
 * call directly.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 ******************************************************************************/
#ifndef __CYWATCH_BSP_MPU6050_DRIVER_H__
#define __CYWATCH_BSP_MPU6050_DRIVER_H__

/***********************************Includes***********************************/
#include "cywatch_bsp_mpu6050_reg.h"

#include <stdio.h>
#include <stdint.h>

/***********************************Includes***********************************/

/***********************************Defines************************************/
#define OS_SUPPORTING

/***********************************Defines************************************/

/**********************************Declaring***********************************/
/*iic接口 (无实例参数: 具体I2C实例由适配器层绑定转发) */
typedef struct
{
	int8_t (*pf_start)(void);
	int8_t (*pf_stop)(void);
	int8_t (*pf_wait_ack)(void);
	int8_t (*pf_send_ack)(void);
	int8_t (*pf_send_not_ack)(void);
	int8_t (*pf_send_bytes)(uint8_t *pdata,
							uint8_t size);
	int8_t (*pf_receive_bytes)(uint8_t *pdata,
							   uint8_t size);
	/* 寄存器级读写: 内部完成 START/重复START/器件地址/寄存器地址/STOP.
	   dev_addr 传器件 7 位地址, 读写位由实现自己拼 */
	int8_t (*pf_readreg)(uint8_t dev_addr, uint8_t reg,
						 uint8_t *pdata, uint8_t size);
	int8_t (*pf_writereg)(uint8_t dev_addr, uint8_t reg, uint8_t data);
} mpu6050_iic_interface_t;

/*让出cpu接口*/
typedef struct
{
	void (*pf_yield)(void);
} mpu6050_yield_interface_t;

/*延时函数接口*/
typedef struct
{
	void (*pf_delay)(uint32_t ms);
} mpu6050_delay_interface_t;

/*mpu定义*/
typedef struct bsp_mpu6050_driver
{
	mpu6050_iic_interface_t *p_iic_interface;
	mpu6050_yield_interface_t *p_yield_interface;
	mpu6050_delay_interface_t *p_delay_interface;

	int8_t (*pf_inst)(
		struct bsp_mpu6050_driver *p_mpu6050_instance,

		mpu6050_iic_interface_t *p_iic_interface,
		mpu6050_yield_interface_t *p_yield_interface,
		mpu6050_delay_interface_t *p_delay_interface);

	int8_t (*pf_deinst)(struct bsp_mpu6050_driver *p_mpu6050_instance);

	int8_t (*pf_init)(struct bsp_mpu6050_driver *p_mpu6050_instance);

	int8_t (*pf_deinit)(struct bsp_mpu6050_driver *p_mpu6050_instance);

	int8_t (*pf_read_id)(struct bsp_mpu6050_driver *p_mpu6050_instance);

	int8_t (*pf_read_accel)(struct bsp_mpu6050_driver *p_mpu6050_instance,
							float *p_accel_x, float *p_accel_y, float *p_accel_z);

	int8_t (*pf_read_gyro)(struct bsp_mpu6050_driver *p_mpu6050_instance,
						   float *p_gyro_x, float *p_gyro_y, float *p_gyro_z);

	int8_t (*pf_hibernating)(struct bsp_mpu6050_driver *p_mpu6050_instance);

	int8_t (*pf_wakeup)(struct bsp_mpu6050_driver *p_mpu6050_instance);

} bsp_mpu6050_driver_t;

/*mpu6050构造函数*/
int8_t mpu6050_inst(bsp_mpu6050_driver_t *p_mpu6050_instance,
					mpu6050_iic_interface_t *p_iic_interface,
					mpu6050_yield_interface_t *p_yield_interface,
					mpu6050_delay_interface_t *p_delay_interface);

/**********************************Declaring***********************************/

#endif // __CYWATCH_BSP_MPU6050_DRIVER_H__
