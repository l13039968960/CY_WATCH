/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file lv_port_disp.h
 *
 * @par dependencies
 * - lvgl.h
 * - stdint.h
 *
 * @author zw1194
 *
 * @brief LVGL display port: mount ST7789T3(240x280) as the LVGL display device.
 *
 * Processing flow:
 *
 * call lv_port_disp_init() after lv_init()/lv_tick_set_cb(), then create UI widgets.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 ******************************************************************************/
#ifndef __LV_PORT_DISP_H__
#define __LV_PORT_DISP_H__

/***********************************Includes***********************************/
#include "lvgl.h"
#include <stdint.h>

/***********************************Includes***********************************/

/**********************************Declaring***********************************/
/* LVGL显示端口初始化:
 * 构造SPI+ST7789驱动、点亮背光(PA1)、创建240x280 RGB565显示(PARTIAL单缓冲40行).
 * 返回0成功; 负数见 lv_port_disp.c 实现 */
int8_t lv_port_disp_init(void);

/**********************************Declaring***********************************/

#endif // __LV_PORT_DISP_H__
