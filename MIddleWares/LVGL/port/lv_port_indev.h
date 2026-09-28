/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file lv_port_indev.h
 *
 * @par dependencies
 * - lvgl.h
 *
 * @author zw1194
 *
 * @brief LVGL input device port: CST816T touch screen adapter.
 *
 * Processing flow:
 *
 * call lv_port_indev_init() after lv_port_disp_init(), before lv_watch_ui_init().
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 ******************************************************************************/
#ifndef __LV_PORT_INDEV_H__
#define __LV_PORT_INDEV_H__

/***********************************Includes***********************************/
#include "lvgl.h"

/***********************************Includes***********************************/

/**********************************Declaring***********************************/
/* 触摸端口初始化: 构造CST816T驱动(独立I2C PA8/PB4 + INT PB2 + RST PA15)并注册LVGL指针输入设备 */
int8_t lv_port_indev_init(void);

/**********************************Declaring***********************************/

#endif // __LV_PORT_INDEV_H__
