/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file lv_port_disp.c
 *
 * @par dependencies
 * - cywatch_log.h
 * - lv_port_disp.h
 * - ../../ST7789T3/adapter/cywatch_adapter_st7789t3.h
 *
 * @author zw1194
 *
 * @brief LVGL display port: 把 ST7789T3 adapter 挂成一个 lv_display_t.
 *
 * Processing flow:
 *
 * lv_port_disp_init():
 *   lvgl_bsp_disp_inst()(SPI1总线由 main.c 建/adapter 挂载 + 面板初始化)
 *   → 点亮背光 → lv_display_create(240,280) + RGB565 + PARTIAL单缓冲40行.
 * lcd_flush_cb():
 *   LVGL脏区 → adapter set_window(含端点, 驱动内部+Y20面板偏移)
 *            → adapter write_pixels(驱动内部RGB565高字节先行交换+分块DMA, 阻塞)
 *            → lv_display_flush_ready.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 本文件只做「LVGL 侧胶水」: 出现的类型只有 lv_* 与 adapter 的服务级 API,
 *       不含任何引脚/总线/接口结构体/寄存器 —— 那些都在 ST7789T3/adapter 里.
 *
 * @note 须在 lv_init() 与 lv_tick_set_cb() 之后调用; 且因 adapter 的 inst 内含
 *       osDelay, 必须在 osKernelStart() 之后的任务上下文调用.
 *
 * @note 分辨率常量对应驱动默认方向 st7789t3_dir_0(竖屏 240x280); 若运行时改用
 *       adapter 的 set_direction 切成横屏, 需同步 lv_display_set_resolution,
 *       否则脏区坐标与面板窗口不匹配.
 ******************************************************************************/
#include "lv_port_disp.h"

#include "system/log/cywatch_log.h" /* log_printf() */
#include <stdio.h>
#include "cywatch_adapter_disp.h"

/* 显示尺寸与刷新缓冲(与驱动默认方向 dir_0 竖屏一致) */
#define LV_LCD_HOR_RES   240
#define LV_LCD_VER_RES   280
#define LV_LCD_BUF_LINES 20   /* 240*40*2 = 19.2KB 部分渲染单缓冲 */

/* LVGL渲染缓冲: 16字节对齐(LVGL 9要求), PARTIAL单缓冲 */
static uint8_t s_disp_buf[LV_LCD_HOR_RES * LV_LCD_BUF_LINES * 2]
	__attribute__((aligned(16)));

/******************************************************************************
 * @name    lcd_flush_cb
 * @brief   LVGL显示刷新回调: 脏区→窗口→像素DMA(阻塞)→通知完成
 * @param   p_disp[in]  LVGL显示实例
 * @param   p_area[in]  脏区坐标(含端点)
 * @param   px_map[in]  RGB565像素数据(小端native存储)
 *
 * @return  无
 *
 * @note    ST7789 adapter/驱动内部已处理: 面板+Y20偏移(set_window)与
 *          高字节先行字节交换(write_pixels), 此处勿再交换字节序;
 *          lv_display_flush_ready 必须调用, 漏调会冻结UI刷新
 *****************************************************************************/
static void lcd_flush_cb(lv_display_t *p_disp, const lv_area_t *p_area,
						 uint8_t *px_map)
{
	uint32_t w = (uint32_t)(p_area->x2 - p_area->x1 + 1);
	uint32_t h = (uint32_t)(p_area->y2 - p_area->y1 + 1);

	(void)lvgl_bsp_disp_set_window(p_area->x1, p_area->y1,
									   p_area->x2, p_area->y2);
	(void)lvgl_bsp_disp_write_pixels((uint16_t *)px_map, w * h);

	lv_display_flush_ready(p_disp);
}

/******************************************************************************
 * @name    lv_port_disp_init
 * @brief   LVGL显示端口初始化: ST7789 adapter → 背光 → LVGL显示对象
 * @param   无
 *
 * @return  0 success
 *         -1 st7789 adapter inst failed(总线段或面板初始化失败)
 *         -2 lv_display_create 失败
 *
 * @note    失败时可安全重试: adapter 的 inst 未成功即不会创建 lv_display_t,
 *          重入不会产生第二个显示实例
 *****************************************************************************/
int8_t lv_port_disp_init(void)
{
	int8_t ret = 0;
	lv_display_t *p_disp = NULL;

	/* 1. 构造ST7789T3(总线绑定 + 硬件复位 + 面板寄存器初始化) */
	ret = lvgl_bsp_disp_inst();
	if (0 != ret)
	{
		log_printf("LCD inst fail:%d\r\n", (int)ret);
		return -1;
	}

	/* 2. 点亮背光 */
	(void)lvgl_bsp_disp_set_backlight(1);

	/* 3. 创建LVGL显示: 240x280 RGB565, PARTIAL单缓冲40行(阻塞flush, 双缓冲无收益) */
	p_disp = lv_display_create(LV_LCD_HOR_RES, LV_LCD_VER_RES);
	if (NULL == p_disp)
	{
		log_printf("LCD lv_display_create fail\r\n");
		return -2;
	}

	lv_display_set_color_format(p_disp, LV_COLOR_FORMAT_RGB565);
	lv_display_set_buffers(p_disp, s_disp_buf, NULL, sizeof(s_disp_buf),
						   LV_DISPLAY_RENDER_MODE_PARTIAL);
	lv_display_set_flush_cb(p_disp, lcd_flush_cb);

	return 0;
}
