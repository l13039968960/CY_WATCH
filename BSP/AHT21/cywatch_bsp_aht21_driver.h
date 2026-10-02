/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_bsp_aht21_driver.h
 *
 * @par dependencies
 * - cywatch_bsp_aht21_reg.h
 * - stdio.h
 * - stdint.h
 *
 * @author zw1194
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
#ifndef __CYWATCH_BSP_AHT21_DRIVER_H__
#define __CYWATCH_BSP_AHT21_DRIVER_H__

/***********************************Includes***********************************/
#include "cywatch_bsp_aht21_reg.h"      // 包含自己的寄存器.h文件一定是在最开始

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
        切在 start..stop 中间就会和其他设备互相踩。拼帧统一交给下面两个裸帧接口,
        锁由 iic_hal 在内部整段持有 */
typedef struct
{
	/* 寄存器级读写: 内部完成 START/重复START/器件地址/寄存器地址/STOP.
	   dev_addr 传器件 7 位地址, 读写位由实现自己拼 */
	int8_t (*pf_readreg)(uint8_t dev_addr, uint8_t reg,
						 uint8_t *pdata, uint8_t size);
	/* 裸帧读写: 器件地址由 iic_hal 拼 */
	int8_t (*pf_write_frame)(uint8_t dev_addr, uint8_t *pdata, uint8_t size);
	int8_t (*pf_read_frame)(uint8_t dev_addr, uint8_t *pdata, uint8_t size);
} aht21_iic_interface_t;

/*延时函数接口*/
typedef struct
{
	void (*pf_delay)(uint32_t ms);
} aht21_delay_interface_t;

/*aht21定义*/
typedef struct bsp_aht21_driver
{
	aht21_iic_interface_t *p_iic_interface;
	aht21_delay_interface_t *p_delay_interface;

	int8_t (*pf_inst)(
		struct bsp_aht21_driver *p_aht21_instance,

		aht21_iic_interface_t *p_iic_interface,
		aht21_delay_interface_t *p_delay_interface);

	int8_t (*pf_deinst)(struct bsp_aht21_driver *p_aht21_instance);

	int8_t (*pf_init)(struct bsp_aht21_driver *p_aht21_instance);

	int8_t (*pf_deinit)(struct bsp_aht21_driver *p_aht21_instance);

	/* AHT21 无 ID 寄存器: 本接口返回状态寄存器 0x71 的值(已屏蔽bit7忙标志),
	   失败返回 -1/-2 */
	int8_t (*pf_read_id)(struct bsp_aht21_driver *p_aht21_instance);

	/* 一次触发同时得到温度(℃)和湿度(%RH), 不可分开取:
	   AHT21 的温湿度由同一次测量产出, 分两次调用要各触发一次(各花 80ms) */
	int8_t (*pf_read_temp_humi)(struct bsp_aht21_driver *p_aht21_instance,
								float *p_temperature, float *p_humidity);

	int8_t (*pf_hibernating)(struct bsp_aht21_driver *p_aht21_instance);

	int8_t (*pf_wakeup)(struct bsp_aht21_driver *p_aht21_instance);

} bsp_aht21_driver_t;

/*aht21构造函数*/
int8_t aht21_inst(bsp_aht21_driver_t *p_aht21_instance,
				  aht21_iic_interface_t *p_iic_interface,
				  aht21_delay_interface_t *p_delay_interface);

/**********************************Declaring***********************************/

#endif // __CYWATCH_BSP_AHT21_DRIVER_H__
