/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_bsp_st7789t3_driver.h
 *
 * @par dependencies
 * - cywatch_bsp_st7789t3_reg.h
 * - stdio.h
 * - stdint.h
 *
 * @author	zw1194
 *
 * @brief Provide the HAL APIs of ST7789T3 and corresponding opetions.
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
#ifndef __CYWATCH_BSP_ST7789T3_DRIVER_H__
#define __CYWATCH_BSP_ST7789T3_DRIVER_H__

/***********************************Includes***********************************/
#include "cywatch_bsp_st7789t3_reg.h"

#include <stdio.h>
#include <stdint.h>

/***********************************Includes***********************************/

/***********************************Defines************************************/
#define OS_SUPPORTING

/***********************************Defines************************************/

/**********************************Declaring***********************************/
/*spi接口*/
typedef struct
{
	/* spi初始化，不关心其他驱动 */
	int8_t (*pf_init)(void);
	/* spi反初始化 */
	int8_t (*pf_deinit)(void);
	/* 发送字节流(命令/参数), MSB先行 */
	int8_t (*pf_send_bytes)(uint8_t *pdata,
							uint32_t size);
	/* DMA中断发送字节流(像素数据, 启动+等待合并), 高字节先行 */
	int8_t (*pf_send_bytes_dma)(uint8_t *pdata,
								uint32_t size);
} st7789t3_spi_interface_t;

/*gpio接口 (控制DC/CS/RST/背光引脚) */
typedef struct
{
	/* gpio初始化 */
	int8_t (*pf_init)(void);
	/* gpio反初始化 */
	int8_t (*pf_deinit)(void);

	/* DC(数据/命令选择): 0=命令, 1=数据 */
	int8_t (*pf_dc_set)(uint8_t level);
	/* CS(片选): 0=选中, 1=释放 */
	int8_t (*pf_cs_set)(uint8_t level);
	/* RST(硬件复位): 0=复位, 1=正常运行 */
	int8_t (*pf_rst_set)(uint8_t level);
} st7789t3_gpio_interface_t;

typedef struct
{
	/* pwm初始化 */
	int8_t (*pf_init)(void);
	/* pwm反初始化 */
	int8_t (*pf_deinit)(void);

	/* 背光调光: 亮度百分比 0~100(0=全灭), 具体怎么实现由底层决定 */
	void (*pf_pwm_set)(uint8_t level);
} st7789t3_pwm_interface_t;

/*延时函数接口*/
typedef struct
{
	void (*pf_delay)(uint32_t ms);
} st7789t3_delay_interface_t;

/*显示方向枚举*/
typedef enum st7789t3_direction
{
	st7789t3_dir_0 = 0,		/* 0°   竖屏 240x280 */
	st7789t3_dir_90 = 90,	/* 90°  横屏 280x240 */
	st7789t3_dir_180 = 180, /* 180° 竖屏 240x280 */
	st7789t3_dir_270 = 270	/* 270° 横屏 280x240 */
} st7789t3_direction_t;

/*st7789t3定义*/
typedef struct bsp_st7789t3_driver
{
	st7789t3_spi_interface_t *p_spi_interface;
	st7789t3_gpio_interface_t *p_gpio_interface;
	st7789t3_delay_interface_t *p_delay_interface;
	st7789t3_pwm_interface_t *p_pwm_interface;

	/* 运行状态 */
	uint16_t width;					/* 当前逻辑宽度 */
	uint16_t height;				/* 当前逻辑高度 */
	st7789t3_direction_t direction; /* 当前显示方向 */

	int8_t (*pf_inst)(
		struct bsp_st7789t3_driver *p_st7789t3_instance,

		st7789t3_spi_interface_t *p_spi_interface,
		st7789t3_gpio_interface_t *p_gpio_interface,
		st7789t3_delay_interface_t *p_delay_interface,
		st7789t3_pwm_interface_t *p_pwm_interface);

	int8_t (*pf_deinst)(struct bsp_st7789t3_driver *p_st7789t3_instance);

	int8_t (*pf_init)(struct bsp_st7789t3_driver *p_st7789t3_instance);

	int8_t (*pf_deinit)(struct bsp_st7789t3_driver *p_st7789t3_instance);

	int8_t (*pf_reset)(struct bsp_st7789t3_driver *p_st7789t3_instance);

	int8_t (*pf_set_direction)(struct bsp_st7789t3_driver *p_st7789t3_instance,
							   st7789t3_direction_t direction);

	int8_t (*pf_display_on)(struct bsp_st7789t3_driver *p_st7789t3_instance);

	int8_t (*pf_display_off)(struct bsp_st7789t3_driver *p_st7789t3_instance);

	int8_t (*pf_sleep)(struct bsp_st7789t3_driver *p_st7789t3_instance);

	int8_t (*pf_wakeup)(struct bsp_st7789t3_driver *p_st7789t3_instance);

	int8_t (*pf_invert_on)(struct bsp_st7789t3_driver *p_st7789t3_instance);

	int8_t (*pf_invert_off)(struct bsp_st7789t3_driver *p_st7789t3_instance);

	int8_t (*pf_set_backlight)(struct bsp_st7789t3_driver *p_st7789t3_instance,
							   uint8_t value);

	int8_t (*pf_set_window)(struct bsp_st7789t3_driver *p_st7789t3_instance,
							uint16_t x0, uint16_t y0,
							uint16_t x1, uint16_t y1);

	int8_t (*pf_write_pixels)(struct bsp_st7789t3_driver *p_st7789t3_instance,
							  uint16_t *pdata, uint32_t size);

	int8_t (*pf_fill)(struct bsp_st7789t3_driver *p_st7789t3_instance,
					  uint16_t x0, uint16_t y0,
					  uint16_t x1, uint16_t y1, uint16_t color);

	int8_t (*pf_clear)(struct bsp_st7789t3_driver *p_st7789t3_instance,
					   uint16_t color);

	int8_t (*pf_draw_point)(struct bsp_st7789t3_driver *p_st7789t3_instance,
							uint16_t x, uint16_t y, uint16_t color);

} bsp_st7789t3_driver_t;

/*st7789t3构造函数*/
int8_t st7789t3_inst(bsp_st7789t3_driver_t *p_st7789t3_instance,
					 st7789t3_spi_interface_t *p_spi_interface,
					 st7789t3_gpio_interface_t *p_gpio_interface,
					 st7789t3_delay_interface_t *p_delay_interface,
					 st7789t3_pwm_interface_t *p_pwm_interface);

/**********************************Declaring***********************************/

#endif // __CYWATCH_BSP_ST7789T3_DRIVER_H__
