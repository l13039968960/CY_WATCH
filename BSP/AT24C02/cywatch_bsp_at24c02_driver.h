/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_bsp_at24c02_driver.h
 *
 * @par dependencies
 * - cywatch_bsp_at24c02_reg.h
 * - stdio.h
 * - stdint.h
 *
 * @author	zw1194
 *
 * @brief Provide the HAL APIs of AT24C02 and corresponding opetions.
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
#ifndef __CYWATCH_BSP_AT24C02_DRIVER_H__
#define __CYWATCH_BSP_AT24C02_DRIVER_H__

/***********************************Includes***********************************/
#include "cywatch_bsp_at24c02_reg.h"    // 包含自己的寄存器.h文件一定是在最开始

#include <stdio.h>
#include <stdint.h>

/***********************************Includes***********************************/

/***********************************Defines************************************/
#define OS_SUPPORTING

/***********************************Defines************************************/

/**********************************Declaring***********************************/
/*iic接口 (无实例参数: 具体I2C实例由适配器层绑定转发)
  @note 只列本驱动真正用到的3个口。start/stop/send_bytes/receive_bytes 等总线原语
        故意不暴露: 在驱动里自己拼帧会绕过 iic_hal 的互斥量, 位带总线被 tick 抢占
        切在 start..stop 中间就会和其他设备互相踩。拼帧统一交给下面的裸帧接口,
        锁由 iic_hal 在内部整段持有 */
typedef struct
{
	/* 寄存器级读写: 内部完成 START/重复START/器件地址/寄存器地址/STOP.
	   对 EEPROM 而言就是"随机读" —— dev_addr 传器件 7 位地址, reg 传存储地址 */
	int8_t (*pf_readreg)(uint8_t dev_addr, uint8_t reg,
						 uint8_t *pdata, uint8_t size);
	/* 裸帧读写: 器件地址由 iic_hal 拼.
	   pf_write_frame 对 EEPROM 就是"页写": pdata[0] 传存储地址, 其后是数据;
	   size 传 0 时只发器件地址不跟数据 —— 正好用来做 ACK 轮询探测写周期 */
	int8_t (*pf_write_frame)(uint8_t dev_addr, uint8_t *pdata, uint8_t size);
	int8_t (*pf_read_frame)(uint8_t dev_addr, uint8_t *pdata, uint8_t size);
} at24c02_iic_interface_t;

/*延时函数接口*/
typedef struct
{
	void (*pf_delay)(uint32_t ms);
} at24c02_delay_interface_t;

/*时基计数器接口*/
typedef struct
{
	uint32_t (*pf_get_time)(void);
} at24c02_timebase_interface_t;

/*at24c02定义*/
typedef struct bsp_at24c02_driver
{
	at24c02_iic_interface_t *p_iic_interface;
	at24c02_delay_interface_t *p_delay_interface;
	at24c02_timebase_interface_t *p_timebase_interface;

	int8_t (*pf_inst)(
		struct bsp_at24c02_driver *p_at24c02_instance,

		at24c02_iic_interface_t *p_iic_interface,
		at24c02_delay_interface_t *p_delay_interface,
		at24c02_timebase_interface_t *p_timebase_interface);

	int8_t (*pf_deinst)(struct bsp_at24c02_driver *p_at24c02_instance);

	int8_t (*pf_init)(struct bsp_at24c02_driver *p_at24c02_instance);

	int8_t (*pf_deinit)(struct bsp_at24c02_driver *p_at24c02_instance);

	/* 从指定存储地址连续读取(地址自动递增); size 超过单次接收上限(255)自动分块 */
	int8_t (*pf_read)(struct bsp_at24c02_driver *p_at24c02_instance,
					  uint16_t addr, uint8_t *pdata, uint16_t size);

	/* 从指定存储地址写入; 自动按 8 字节页拆分, 支持跨页,
	   每页写完轮询 ACK 等写周期(最大 5ms)结束 */
	int8_t (*pf_write)(struct bsp_at24c02_driver *p_at24c02_instance,
					   uint16_t addr, uint8_t *pdata, uint16_t size);

} bsp_at24c02_driver_t;

/*at24c02构造函数*/
int8_t at24c02_inst(bsp_at24c02_driver_t *p_at24c02_instance,
					at24c02_iic_interface_t *p_iic_interface,
					at24c02_delay_interface_t *p_delay_interface,
					at24c02_timebase_interface_t *p_timebase_interface);

/**********************************Declaring***********************************/

#endif // __CYWATCH_BSP_AT24C02_DRIVER_H__
