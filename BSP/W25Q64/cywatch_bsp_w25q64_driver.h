/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_bsp_w25q64_driver.h
 *
 * @par dependencies
 * - cywatch_bsp_w25q64_reg.h
 * - stdio.h
 * - stdint.h
 *
 * @author	zw1194
 *
 * @brief Provide the HAL APIs of W25Q64 and corresponding opetions.
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
#ifndef __CYWATCH_BSP_W25Q64_DRIVER_H__
#define __CYWATCH_BSP_W25Q64_DRIVER_H__

/***********************************Includes***********************************/
#include "cywatch_bsp_w25q64_reg.h"

#include <stdio.h>
#include <stdint.h>

/***********************************Includes***********************************/

/***********************************Defines************************************/
#define OS_SUPPORTING

/***********************************Defines************************************/

/**********************************Declaring***********************************/
/*spi接口 (不带实例参数: 具体SPI实例由调用方的转发函数绑定, 见 adapter 层) */
typedef struct
{
	/* spi初始化，不关心其他驱动 */
	int8_t (*pf_init)(void);
	/* spi反初始化 */
	int8_t (*pf_deinit)(void);
	/* 发送字节流(命令/地址/数据), MSB先行 */
	int8_t (*pf_send_bytes)(uint8_t *pdata,
							uint32_t size);

	/* 接收字节流(数据/状态/ID) */
	int8_t (*pf_receive_bytes)(uint8_t *pdata,
							   uint32_t size);
} w25q64_spi_interface_t;

/*gpio接口 (控制CS片选引脚) */
typedef struct
{
	/* gpio初始化 */
	int8_t (*pf_init)(void);
	/* gpio反初始化 */
	int8_t (*pf_deinit)(void);
	/* CS(片选): 0=选中(拉低), 1=释放(拉高) */
	int8_t (*pf_cs_set)(uint8_t level);
} w25q64_gpio_interface_t;

/*延时函数接口*/
typedef struct
{
	void (*pf_delay)(uint32_t ms);
} w25q64_delay_interface_t;


/*w25q64定义*/
typedef struct bsp_w25q64_driver
{
	w25q64_spi_interface_t *p_spi_interface;
	w25q64_gpio_interface_t *p_gpio_interface;
	w25q64_delay_interface_t *p_delay_interface;

	int8_t (*pf_inst)(
		struct bsp_w25q64_driver *p_w25q64_instance,
		w25q64_spi_interface_t *p_spi_interface,
		w25q64_gpio_interface_t *p_gpio_interface,
		w25q64_delay_interface_t *p_delay_interface);

	int8_t (*pf_deinst)(struct bsp_w25q64_driver *p_w25q64_instance);

	int8_t (*pf_init)(struct bsp_w25q64_driver *p_w25q64_instance);

	int8_t (*pf_deinit)(struct bsp_w25q64_driver *p_w25q64_instance);

	int8_t (*pf_read_id)(struct bsp_w25q64_driver *p_w25q64_instance,
						 uint8_t *p_manuf_id, uint8_t *p_memory_type, uint8_t *p_capacity);

	int8_t (*pf_read)(struct bsp_w25q64_driver *p_w25q64_instance,
					  uint32_t addr, uint8_t *pdata, uint32_t size);

	int8_t (*pf_write)(struct bsp_w25q64_driver *p_w25q64_instance,
					   uint32_t addr, uint8_t *pdata, uint32_t size);

	int8_t (*pf_erase_sector)(struct bsp_w25q64_driver *p_w25q64_instance,
							  uint32_t addr);

	int8_t (*pf_erase_block_32k)(struct bsp_w25q64_driver *p_w25q64_instance,
								 uint32_t addr);

	int8_t (*pf_erase_block_64k)(struct bsp_w25q64_driver *p_w25q64_instance,
								 uint32_t addr);

	int8_t (*pf_erase_chip)(struct bsp_w25q64_driver *p_w25q64_instance);

	int8_t (*pf_hibernating)(struct bsp_w25q64_driver *p_w25q64_instance);

	int8_t (*pf_wakeup)(struct bsp_w25q64_driver *p_w25q64_instance);

	int8_t (*pf_wait_busy)(struct bsp_w25q64_driver *p_w25q64_instance); /*等待芯片空闲: 轮询BUSY位*/

} bsp_w25q64_driver_t;

/*w25q64构造函数*/
int8_t w25q64_inst(bsp_w25q64_driver_t *p_w25q64_instance,
				   w25q64_spi_interface_t *p_spi_interface,
				   w25q64_gpio_interface_t *p_gpio_interface,
				   w25q64_delay_interface_t *p_delay_interface);

/**********************************Declaring***********************************/

#endif // __CYWATCH_BSP_W25Q64_DRIVER_H__
