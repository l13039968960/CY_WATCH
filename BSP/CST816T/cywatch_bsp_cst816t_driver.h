/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_bsp_cst816t_driver.h
 *
 * @par dependencies
 * - cywatch_bsp_cst816t_reg.h
 * - stdio.h
 * - stdint.h
 *
 * @author	zw1194
 *
 * @brief Provide the HAL APIs of CST816T and corresponding opetions.
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
#ifndef __CYWATCH_BSP_CST816T_DRIVER_H__
#define __CYWATCH_BSP_CST816T_DRIVER_H__

/***********************************Includes***********************************/
#include "cywatch_bsp_cst816t_reg.h"

#include <stdio.h>
#include <stdint.h>

/***********************************Includes***********************************/

/***********************************Defines************************************/
#define OS_SUPPORTING

/*读取方式 (pf_read_touch 的 block 参数)*/
#define CST816T_READ_POLL 0 /*直接轮询读取(不等中断)*/
#define CST816T_READ_WAIT 1 /*等待中断后读取*/

/***********************************Defines************************************/

/**********************************Declaring***********************************/
/*iic接口*/
typedef struct
{
	/* iic初始化，不关心其他驱动 */
	int8_t (*pf_init)(void);
	/* iic反初始化 */
	int8_t (*pf_deinit)(void);

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
} cst816t_iic_interface_t;

/* 中断阻塞等待用接口 */
typedef struct
{
	int8_t (*pf_wait)(void);    /*阻塞等待*/
	int8_t (*pf_release)(void); /*释放(ISR中调用)*/
} cst816t_semaphore_interface_t;

/*延时函数接口*/
typedef struct
{
	void (*pf_delay)(uint32_t ms);
} cst816t_delay_interface_t;

#ifndef OS_SUPPORTING
/*时基计数器接口 (仅裸机) —— 中断等待靠计数到点超时; OS_SUPPORTING 下这段时间
  由信号量的 pf_wait() 自己兜, 不需要时基, 接口与实例一并省掉*/
typedef struct
{
	uint32_t (*pf_get_time)(void);
} cst816t_timebase_interface_t;
#endif // OS_SUPPORTING

/*底层中断接口*/
typedef struct
{
	void (*pf_enable_interrupt)(void); /*底层使能中断接口*/
	void (*pf_disable_interrupt)(void); /*底层失能中断接口*/
} cst816t_interrupt_interface_t;

/*gpio接口 (RST复位/唤醒引脚)*/
typedef struct
{
	/* gpio初始化 */
	int8_t (*pf_init)(void);
	/* gpio反初始化 */
	int8_t (*pf_deinit)(void);

	/* GPIO电平输出: 0=低电平, 1=高电平 */
	void (*pf_gpio_set_level)(uint8_t level);
} cst816t_gpio_interface_t;

/*cst816t定义*/
typedef struct bsp_cst816t_driver
{
	cst816t_iic_interface_t *p_iic_interface;
	cst816t_gpio_interface_t *p_gpio_interface;
	cst816t_semaphore_interface_t *p_semaphore_interface;
	cst816t_delay_interface_t *p_delay_interface;
#ifndef OS_SUPPORTING
	cst816t_timebase_interface_t *p_timebase_interface;
#endif // OS_SUPPORTING
	cst816t_interrupt_interface_t *p_interrupt_interface;

#ifndef OS_SUPPORTING
	/*裸机中断等待状态 (ISR置位, pf_read_touch轮询) —— OS_SUPPORTING 下等待交给
	  信号量阻塞, 这两个字段没有读者, ISR 也只释放信号量不再置标志
	  @note irq_flag 在 ISR 里写、在 pf_read_touch 的轮询里读, 必须 volatile,
	        否则编译器可以把 while 的读取提到循环外 */
	volatile uint8_t irq_flag; /*中断标志*/
	uint32_t irq_start_tick;   /*中断等待起始时刻*/
#endif // OS_SUPPORTING

	int8_t (*pf_inst)(
		struct bsp_cst816t_driver *p_cst816t_instance,

		cst816t_iic_interface_t *p_iic_interface,
		cst816t_gpio_interface_t *p_gpio_interface,
		cst816t_semaphore_interface_t *p_semaphore_interface,
		cst816t_delay_interface_t *p_delay_interface,
#ifndef OS_SUPPORTING
		cst816t_timebase_interface_t *p_timebase_interface,
#endif // OS_SUPPORTING
		cst816t_interrupt_interface_t *p_interrupt_interface);

	int8_t (*pf_deinst)(struct bsp_cst816t_driver *p_cst816t_instance);

	int8_t (*pf_init)(struct bsp_cst816t_driver *p_cst816t_instance);

	int8_t (*pf_deinit)(struct bsp_cst816t_driver *p_cst816t_instance);

	int8_t (*pf_read_id)(struct bsp_cst816t_driver *p_cst816t_instance);

	int8_t (*pf_read_touch)(struct bsp_cst816t_driver *p_cst816t_instance,
							uint8_t *p_gesture_id, uint8_t *p_finger_num,
							uint16_t *p_x, uint16_t *p_y,
							uint8_t block); /*读取触摸数据: block=CST816T_READ_WAIT等待中断后读, CST816T_READ_POLL直接轮询读*/

	int8_t (*pf_enable_interrupt)(struct bsp_cst816t_driver *p_cst816t_instance);

	int8_t (*pf_disable_interrupt)(struct bsp_cst816t_driver *p_cst816t_instance);

	int8_t (*pf_hibernating)(struct bsp_cst816t_driver *p_cst816t_instance);

	int8_t (*pf_wakeup)(struct bsp_cst816t_driver *p_cst816t_instance); /*唤醒深度休眠(RST复位+重初始化)*/

	void (*pf_interrupt_cb)(struct bsp_cst816t_driver *p_cst816t_instance); /*本层中断回调*/

} bsp_cst816t_driver_t;

/*cst816t构造函数*/
int8_t cst816t_inst(bsp_cst816t_driver_t *p_cst816t_instance,
					cst816t_iic_interface_t *p_iic_interface,
					cst816t_gpio_interface_t *p_gpio_interface,
					cst816t_semaphore_interface_t *p_semaphore_interface,
					cst816t_delay_interface_t *p_delay_interface,
#ifndef OS_SUPPORTING
					cst816t_timebase_interface_t *p_timebase_interface,
#endif // OS_SUPPORTING
					cst816t_interrupt_interface_t *p_interrupt_interface);

/**********************************Declaring***********************************/

#endif // __CYWATCH_BSP_CST816T_DRIVER_H__
