/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file lv_port_indev.c
 *
 * @par dependencies
 * - cywatch_log.h
 * - lv_port_indev.h
 * - ../../CST816T/adapter/cywatch_adapter_cst816t.h
 *
 * @author zw1194
 *
 * @brief LVGL input device port: 把 CST816T adapter 挂成一个 lv_indev_t.
 *
 * Processing flow:
 *
 * lv_port_indev_init():
 *   lvgl_bsp_indev_inst()(触摸专用位带I2C由 main.c 建/adapter 挂载; RST PA15 +
 *   EXTI PB2 + ChipID 自检) → lv_indev_create(POINTER) + 注册读回调.
 * touch_indev_read_cb():
 *   LVGL每轮(lv_timer_handler)调 adapter 轮询读原始触摸帧(非阻塞)
 *   → 脏帧过滤 → 原始坐标缩放映射到240x280面板 → PRESSED/RELEASED.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 本文件只做「LVGL 侧胶水 + 坐标映射」: 触摸的引脚/总线/EXTI/驱动对接口
 *       都在 CST816T/adapter 里. 坐标缩放留在这里而不是 adapter, 因为
 *       「面板几何 + 屏幕坐标系」是 LVGL 侧知识 —— adapter 只透传设备原始值
 *       (与 cywatch-bsp-driver skill §11.6.4「adapter 只透传, 不做量纲变换」一致);
 *       面板尺寸一旦与 lv_port_disp.c 的显示分辨率解耦(换屏), 只改这里.
 *
 * @note 触摸用独立软件 I2C 实例, 不复用 PB6/PB7 传感器总线;
 *       触摸I2C与显示SPI/其他传感器I2C均独立, 无共享总线竞态.
 ******************************************************************************/
#include "lv_port_indev.h"

#include "system/log/cywatch_log.h" /* log_printf() */
#include <stdio.h>
#include "cywatch_adapter_indev.h"

/* ============================= 面板与坐标映射 ============================= */
/* 本模块CST816T上报面板原生坐标(X≈0~240/Y≈0~280, 非12位满量程).
 * 缩放分母=面板尺寸 → 原始坐标≈屏幕坐标(实测验证). 若换模块上报满量程4096(a),
 * 把 RAW_SPAN 改成 4096 即可. */
#define TOUCH_PANEL_X     240
#define TOUCH_PANEL_Y     280
#define TOUCH_RAW_SPAN_X  240U   /* 原始X满量程 */
#define TOUCH_RAW_SPAN_Y  280U   /* 原始Y满量程 */
/* 触摸轴与屏幕轴映射(硬件验证后按需调整): 1=交换XY / 翻转X / 翻转Y */
#define TOUCH_SWAP_XY   0
#define TOUCH_FLIP_X    0
#define TOUCH_FLIP_Y    0

/******************************************************************************
 * @name    touch_coord_map
 * @brief   将CST816T原始坐标缩放到240x280面板, 并按需交换/翻转轴
 * @param   raw_x[in]  原始X(满量程见TOUCH_RAW_SPAN_X)
 * @param   raw_y[in]  原始Y(满量程见TOUCH_RAW_SPAN_Y)
 * @param   p_x[out]   面板X坐标
 * @param   p_y[out]   面板Y坐标
 *
 * @return  无
 *****************************************************************************/
static void touch_coord_map(uint16_t raw_x, uint16_t raw_y,
							lv_coord_t *p_x, lv_coord_t *p_y)
{
	lv_coord_t x = (lv_coord_t)((uint32_t)raw_x * TOUCH_PANEL_X / TOUCH_RAW_SPAN_X);
	lv_coord_t y = (lv_coord_t)((uint32_t)raw_y * TOUCH_PANEL_Y / TOUCH_RAW_SPAN_Y);

#if (TOUCH_SWAP_XY)
	{
		lv_coord_t t = x;
		x = y;
		y = t;
	}
#endif

#if (TOUCH_FLIP_X)
	x = (lv_coord_t)(TOUCH_PANEL_X - 1) - x;
#endif

#if (TOUCH_FLIP_Y)
	y = (lv_coord_t)(TOUCH_PANEL_Y - 1) - y;
#endif

	*p_x = x;
	*p_y = y;
}

/******************************************************************************
 * @name    touch_indev_read_cb
 * @brief   LVGL指针输入读取回调(主循环 lv_timer_handler 内周期轮询)
 * @param   p_indev[in]  LVGL输入设备实例(未使用)
 * @param   p_data[out]  坐标与按键状态
 *
 * @return  无
 *
 * @note    非阻塞: adapter 固定用轮询模式直接读6字节触摸帧, 不等待中断
 *****************************************************************************/
static void touch_indev_read_cb(lv_indev_t *p_indev, lv_indev_data_t *p_data)
{
	uint8_t gesture = 0;
	uint8_t fingers = 0;
	uint16_t raw_x = 0;
	uint16_t raw_y = 0;
	int8_t ret;

	(void)p_indev;

	ret = lvgl_bsp_indev_read_touch(&gesture, &fingers, &raw_x, &raw_y);

	if (0 == ret && 0 != fingers)
	{
		/* 越界坐标过滤(切屏卡死根因修复): CST816T手势模式下抬手会先上报一帧
		 * "手势帧"——fingers!=0 但坐标是占位最大值 0x1FF/0xFFF=(511,4095)。
		 * 该脏帧进入LVGL(9.3对越界只警告不拦截)会污染指针状态机并叠加近4000px
		 * 伪拖拽矢量, 与切屏动画交错后表现为页面卡死; 此处一律按释放处理 */
		if ((raw_x >= TOUCH_RAW_SPAN_X) || (raw_y >= TOUCH_RAW_SPAN_Y))
		{
			p_data->state = LV_INDEV_STATE_RELEASED;
			return;
		}

		touch_coord_map(raw_x, raw_y, &p_data->point.x, &p_data->point.y);
		p_data->state = LV_INDEV_STATE_PRESSED;
	}
	else
	{
		p_data->state = LV_INDEV_STATE_RELEASED;
	}
}

/******************************************************************************
 * @name    lv_port_indev_init
 * @brief   LVGL触摸端口初始化: CST816T adapter → LVGL输入设备
 * @param   无
 *
 * @return  0 success
 *         -1 cst816t adapter inst failed(总线段/RST/EXTI/ChipID自检)
 *         -2 lv_indev_create 失败
 *
 * @note    须在 lv_init()/lv_tick_set_cb()/lv_port_disp_init() 之后调用;
 *          adapter 内含 osDelay, 必须在 osKernelStart() 之后的任务上下文调用
 *
 * @note    失败时可安全重试: adapter 的 inst 每次都重跑 ChipID 自检;
 *          只有在 inst 成功后才创建 lv_indev_t, 重入不会产生第二个输入设备
 *****************************************************************************/
int8_t lv_port_indev_init(void)
{
	int8_t ret = 0;
	lv_indev_t *p_indev = NULL;

	/* 1. 构造CST816T(总线绑定 + RST复位 + 寄存器配置 + ChipID自检 + EXTI回调) */
	ret = lvgl_bsp_indev_inst();
	if (0 != ret)
	{
		log_printf("TOUCH inst fail:%d\r\n", (int)ret);
		return -1;
	}

	/* 2. 注册LVGL指针输入设备(读回调由 lv_timer_handler 每轮轮询) */
	p_indev = lv_indev_create();
	if (NULL == p_indev)
	{
		log_printf("TOUCH lv_indev_create fail\r\n");
		return -2;
	}

	lv_indev_set_type(p_indev, LV_INDEV_TYPE_POINTER);
	lv_indev_set_read_cb(p_indev, touch_indev_read_cb);

	return 0;
}
