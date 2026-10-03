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
#include "gpio_hal.h"
#include <stdint.h>

/***********************************Includes***********************************/

/**********************************Declaring***********************************/
/* I2C总线硬件配置: 两条线各一份 gpio 配置(SDA/SCL 可能不同端口).
   ★这里填的是初始态★: SDA 收发之间要切方向, 那次切换走运行时的 pf_set_mode,
   不进 cfg(gpio 驱动的 set_mode 是临时状态, 不写回 cfg) */
typedef struct
{
	gpio_cfg_t sda; /* SDA 引脚(初始态: 推挽输出+上拉) */
	gpio_cfg_t scl; /* SCL 引脚(初始态: 推挽输出+上拉) */
} iic_bus_t;

/* 延时接口 */
typedef struct
{
	void (*pf_delay_us)(uint32_t us); /* 微秒延时 */
} iic_delay_interface_t;

/* 互斥量接口 */
typedef struct
{
	int8_t (*pf_lock)(void);   /* 取锁, 0=成功 */
	int8_t (*pf_unlock)(void); /* 放锁, 0=成功 */
} iic_mutex_interface_t;

/* I2C驱动对象 */
typedef struct iic_driver
{
	/* SDA/SCL 两引脚各自的驱动实例(引脚时钟也在它们内部开) */
	gpio_driver_t sda;
	gpio_driver_t scl;

	iic_delay_interface_t *p_delay_interface; /* 延时接口 */
	iic_mutex_interface_t *p_mutex_interface; /* 互斥量接口, NULL=不加锁 */

	uint8_t init_state; /* 初始化状态: 0=deinit, 1=init */
	uint8_t ref_count;  /* 使用本外设的实例数量: 0→1 才真正初始化, 减到 0 才真正反初始化 */

	/* 构造与析构 */
	int8_t (*pf_inst)(struct iic_driver *p_iic_instance,
					  iic_bus_t *p_bus,
					  iic_delay_interface_t *p_delay_interface,
					  iic_mutex_interface_t *p_mutex_interface);
	int8_t (*pf_deinst)(struct iic_driver *p_iic_instance);

	int8_t (*pf_init)(struct iic_driver *p_iic_instance);
	int8_t (*pf_deinit)(struct iic_driver *p_iic_instance);

	/* 底层I2C总线操作 */
	int8_t (*pf_start)(struct iic_driver *p_iic_instance);
	int8_t (*pf_stop)(struct iic_driver *p_iic_instance);
	int8_t (*pf_wait_ack)(struct iic_driver *p_iic_instance);
	int8_t (*pf_send_ack)(struct iic_driver *p_iic_instance);
	int8_t (*pf_send_not_ack)(struct iic_driver *p_iic_instance);

	int8_t (*pf_send_byte)(struct iic_driver *p_iic_instance, uint8_t byte);
	uint8_t (*pf_receive_byte)(struct iic_driver *p_iic_instance);

	/* 原始数据收发*/
	int8_t (*pf_send_bytes)(struct iic_driver *p_iic_instance,
							uint8_t *pdata,
							uint8_t size);
	int8_t (*pf_receive_bytes)(struct iic_driver *p_iic_instance,
							   uint8_t *pdata,
							   uint8_t size);

	/* 寄存器级读写: 内部完成 START/重复START/器件地址/寄存器地址/STOP.
	   整个事务(start..stop)持互斥量, 故这两个接口可被多个任务并发调用 */
	int8_t (*pf_readreg)(struct iic_driver *p_iic_instance,
						 uint8_t dev_addr, uint8_t reg,
						 uint8_t *pdata, uint8_t size);
	int8_t (*pf_writereg)(struct iic_driver *p_iic_instance,
						  uint8_t dev_addr, uint8_t reg,
						  uint8_t data);

	/* 裸帧读写: 器件地址由本层拼, 整段(start..stop)同样持互斥量.
	   给"非寄存器协议"的器件用 —— 命令是多字节裸帧(如 AHT21 的 AC 33 00),
	   或读数根本没有寄存器地址阶段。
	   @note 设备驱动不要再自己用 pf_start/pf_send_bytes 拼帧: 那样绕过了互斥量,
	         位带总线被抢占切开就会和其他设备互相踩。 */
	int8_t (*pf_write_frame)(struct iic_driver *p_iic_instance,
							 uint8_t dev_addr, uint8_t *pdata, uint8_t size);
	int8_t (*pf_read_frame)(struct iic_driver *p_iic_instance,
							uint8_t dev_addr, uint8_t *pdata, uint8_t size);

} iic_driver_t;

/* I2C驱动构造函数 */
int8_t iic_driver_inst(iic_driver_t *p_iic_instance,
					   iic_bus_t *p_bus,
					   iic_delay_interface_t *p_delay_interface,
					   iic_mutex_interface_t *p_mutex_interface);

/**********************************Declaring***********************************/

#endif // __IIC_HAL_H__
