/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_bsp_max30102_driver.h
 *
 * @par dependencies
 * - cywatch_bsp_max30102_reg.h
 * - stdio.h
 * - stdint.h
 *
 * @author	zw1194
 *
 * @brief Provide the HAL APIs of MAX30102 and corresponding opetions.
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
#ifndef __CYWATCH_BSP_MAX30102_DRIVER_H__
#define __CYWATCH_BSP_MAX30102_DRIVER_H__

/***********************************Includes***********************************/
#include "cywatch_bsp_max30102_reg.h"

#include <stdio.h>
#include <stdint.h>

/***********************************Includes***********************************/

/***********************************Defines************************************/
#define OS_SUPPORTING

/***********************************Defines************************************/

/**********************************Declaring***********************************/
/*iic接口 (接口内不保存实例, 具体I2C实例由adapter层以静态转发函数绑定)*/
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
} max30102_iic_interface_t;

#ifdef OS_SUPPORTING
/*让出cpu接口*/
typedef struct
{
	void (*pf_yield)(void);
} max30102_yield_interface_t;

/*OS信号量接口 (中断阻塞等待用; 接口内不保存实例, 信号量实例由adapter层持有并创建)*/
typedef struct
{
	int8_t (*pf_wait)(void);    /*阻塞等待信号量*/
	int8_t (*pf_release)(void); /*释放信号量(ISR中调用)*/
} max30102_semaphore_interface_t;

#endif // OS_SUPPORTING

/*延时函数接口*/
typedef struct
{
	void (*pf_delay)(uint32_t ms);
} max30102_delay_interface_t;

/*时基计数器接口*/
typedef struct
{
	uint32_t (*pf_get_time)(void);
} max30102_timebase_interface_t;

/*底层中断接口 (接口内不保存实例, EXTI实例由adapter层以静态转发函数绑定)*/
typedef struct
{
	void (*pf_enable_interrupt)(void);  /*底层使能中断接口*/
	void (*pf_disable_interrupt)(void); /*底层失能中断接口*/
} max30102_interrupt_interface_t;

/*max30102定义*/
typedef struct bsp_max30102_driver
{
	max30102_iic_interface_t *p_iic_interface;
#ifdef OS_SUPPORTING
	max30102_yield_interface_t *p_yield_interface;
	max30102_semaphore_interface_t *p_semaphore_interface;
#endif // OS_SUPPORTING
	max30102_delay_interface_t *p_delay_interface;
	max30102_timebase_interface_t *p_timebase_interface;
	max30102_interrupt_interface_t *p_interrupt_interface;

	/*裸机中断等待状态 (ISR置位, pf_wait_interrupt轮询)*/
	uint8_t irq_flag;       /*中断标志*/
	uint32_t irq_start_tick; /*中断等待起始时刻*/

	int8_t (*pf_inst)(
		struct bsp_max30102_driver *p_max30102_instance,

		max30102_iic_interface_t *p_iic_interface,
#ifdef OS_SUPPORTING
		max30102_yield_interface_t *p_yield_interface,
		max30102_semaphore_interface_t *p_semaphore_interface,
#endif // OS_SUPPORTING
		max30102_delay_interface_t *p_delay_interface,
		max30102_timebase_interface_t *p_timebase_interface,
		max30102_interrupt_interface_t *p_interrupt_interface);

	int8_t (*pf_deinst)(struct bsp_max30102_driver *p_max30102_instance);

	int8_t (*pf_init)(struct bsp_max30102_driver *p_max30102_instance);

	int8_t (*pf_deinit)(struct bsp_max30102_driver *p_max30102_instance);

	int8_t (*pf_read_id)(struct bsp_max30102_driver *p_max30102_instance);

	int8_t (*pf_change_to_HR)(struct bsp_max30102_driver *p_max30102_instance);

	int8_t (*pf_change_to_spo2)(struct bsp_max30102_driver *p_max30102_instance);

	int8_t (*pf_enable_FIFO_FULL_interrupt)(struct bsp_max30102_driver *p_max30102_instance);

	int8_t (*pf_disable_FIFO_FULL_interrupt)(struct bsp_max30102_driver *p_max30102_instance);

	int8_t (*pf_read_all_FIFO_samples)(struct bsp_max30102_driver *p_max30102_instance,
									   uint32_t *p_red_buff, uint32_t *p_ir_buff);

	int8_t (*pf_read_one_sample)(struct bsp_max30102_driver *p_max30102_instance,
								 uint32_t *p_red, uint32_t *p_i, uint32_t *sample_size);

	int8_t (*pf_hibernating)(struct bsp_max30102_driver *p_max30102_instance);

	int8_t (*pf_wakeup)(struct bsp_max30102_driver *p_max30102_instance);

	int8_t (*pf_wait_interrupt)(struct bsp_max30102_driver *p_max30102_instance);

	int8_t (*pf_interrupt_cb)(struct bsp_max30102_driver *p_max30102_instance);/*该驱动的中断回调函数*/
} bsp_max30102_driver_t;

/*max30102构造函数*/
int8_t max30102_inst(bsp_max30102_driver_t *p_max30102_instance,
					 max30102_iic_interface_t *p_iic_interface,
#ifdef OS_SUPPORTING
					 max30102_yield_interface_t *p_yield_interface,
					 max30102_semaphore_interface_t *p_semaphore_interface,
#endif // OS_SUPPORTING
					 max30102_delay_interface_t *p_delay_interface,
					 max30102_timebase_interface_t *p_timebase_interface,
					 max30102_interrupt_interface_t *p_interrupt_interface);

/**********************************Declaring***********************************/

#endif // __CYWATCH_BSP_MAX30102_DRIVER_H__
