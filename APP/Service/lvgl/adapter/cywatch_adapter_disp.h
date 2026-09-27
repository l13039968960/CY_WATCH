/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_adapter_st7789t3.h
 *
 * @par dependencies
 * - stdint.h
 *
 * @author	zw1194
 *
 * @brief LVGL 服务消费 ST7789T3 的 adapter: 总线实例绑定 + 驱动能力薄转发.
 *
 * Processing flow:
 *
 * lvgl_bsp_disp_inst() 构造(含面板初始化) → set_window/write_pixels 等
 * 逐帧调用 → deinst() 析构.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 命名说明(对 cywatch-bsp-driver skill §11.2 的有意偏离, 非笔误):
 *       adapter 惯例导出函数是 `<consumer>_bsp_<动作>`; 本服务(Lvgl)同时消费
 *       CST816T 与 ST7789T3 两个设备, 只带消费者前缀会出现 `lvgl_bsp_inst` 撞名,
 *       故插入一段区分名写成 `lvgl_bsp_<区分名>_<动作>`, 仍保持「一个 adapter =
 *       一个设备 × 一个服务」的对应关系(grep `lvgl_bsp_` 可一次捞全本服务的
 *       所有外设入口).
 *
 *       ★2026-09-27: 显示这一路的区分名用**角色名 `disp`**(`lvgl_bsp_disp_*`,
 *       与消费者侧 lv_port_disp 对齐); 触摸那一路仍是**设备名 `cst816t`**
 *       (`lvgl_bsp_cst816t_*`). 两侧口径不同★
 *
 * @note 服务/port 层只 include 本头, 不 include 驱动头: 驱动结构体、寄存器、
 *       `st7789t3_direction_t` 等类型不出现在调用方. 显示方向用本文件的
 *       `lvgl_bsp_disp_dir_t` 表达, 由 adapter 内部转换.
 ******************************************************************************/
#ifndef __CYWATCH_ADAPTER_DISP_H__
#define __CYWATCH_ADAPTER_DISP_H__

/***********************************Includes***********************************/
#include <stdint.h>
/***********************************Includes***********************************/

/**********************************Declaring***********************************/
/* 显示方向 */
typedef enum lvgl_bsp_disp_dir
{
	lvgl_bsp_disp_dir_0 = 0,	 /* 0°   竖屏 240x280 */
	lvgl_bsp_disp_dir_90 = 90,	 /* 90°  横屏 280x240 */
	lvgl_bsp_disp_dir_180 = 180, /* 180° 竖屏 240x280 */
	lvgl_bsp_disp_dir_270 = 270	 /* 270° 横屏 280x240 */
} lvgl_bsp_disp_dir_t;

/* 构造: 绑定 main.c 的 SPI1 总线实例 + 背光引脚, 调 st7789t3_inst(含面板初始化) */
int8_t lvgl_bsp_disp_inst(void);
int8_t lvgl_bsp_disp_deinst(void);

/* 方向与电源 */
int8_t lvgl_bsp_disp_set_direction(lvgl_bsp_disp_dir_t direction);
int8_t lvgl_bsp_disp_display_on(void);
int8_t lvgl_bsp_disp_display_off(void);
int8_t lvgl_bsp_disp_sleep(void);
int8_t lvgl_bsp_disp_wakeup(void);

/* 背光: 0=熄灭, 非0=点亮(PA1 为开关型, 不做 PWM 调光) */
int8_t lvgl_bsp_disp_set_backlight(uint8_t value);

/* 画面绘制(坐标含端点, 驱动内部处理面板偏移与 RGB565 字节序) */
int8_t lvgl_bsp_disp_set_window(uint16_t x0, uint16_t y0,
									uint16_t x1, uint16_t y1);
int8_t lvgl_bsp_disp_write_pixels(uint16_t *pdata, uint32_t size);
int8_t lvgl_bsp_disp_fill(uint16_t x0, uint16_t y0,
							  uint16_t x1, uint16_t y1, uint16_t color);
int8_t lvgl_bsp_disp_clear(uint16_t color);
int8_t lvgl_bsp_disp_draw_point(uint16_t x, uint16_t y, uint16_t color);

/* 当前逻辑分辨率(随 set_direction 变化), 供调用方建 LVGL 显示用 */
uint16_t lvgl_bsp_disp_get_width(void);
uint16_t lvgl_bsp_disp_get_height(void);

/**********************************Declaring***********************************/

#endif // __CYWATCH_ADAPTER_DISP_H__
