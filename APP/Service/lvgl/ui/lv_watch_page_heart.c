/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file lv_watch_page_heart.c
 *
 * @par dependencies
 * - lv_watch_page.h
 * - lvgl.h
 *
 * @author zw1194
 *
 * @brief 手表UI - 心率页(heart, ID 0x0302): 标题 + 大字心率值 + BPM + 72点折线图
 *        + 全屏手势条. 由 PageMem 页面管理器托管.
 *
 * Processing flow:
 *
 * pf_create : 建独立屏幕 + 控件 + 图表初始波形 + 本页1s刷新定时器;
 * pf_show   : 立即补一帧(推一个图点) + 恢复定时器;
 * pf_hide   : 暂停定时器(隐藏的缓存页不再每秒推图点);
 * pf_destroy: lv_timer_delete + 句柄置 NULL;
 * 手势      : 左滑→home(MOVE_LEFT).
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 本页只"把数据画出来", **不推进**数据 —— 心率值来自 lv_watch_ui.c 的模型
 *       定时器. 所以本页被淘汰重建后, 图表从一条新波形接着当前值画下去, 数值本身
 *       不会归零.
 *
 * @note 图表的数据缓冲由 chart 自己持有, 随 chart 一起被 LVGL 删掉; pf_destroy
 *       里只清句柄, **不要**自己 lv_obj_delete(p_chart) —— 已在递归删除中.
 ******************************************************************************/
#include "lv_watch_page.h"
#include "easyapp_port.h"

#include <stdio.h>

/* 子集字库(实现在 LVGL/assets/fonts, 生成脚本 MDK-ARM/gen_watch_fonts.py):
   数值用20(只需 ASCII), 标题用14(汉字) */
LV_FONT_DECLARE(lv_font_montserrat_regular_20)
LV_FONT_DECLARE(lv_font_alibaba_puhuiti_14)

/***********************************Defines************************************/
#define WATCH_HR_POINTS     72		/* 折线图点数(约72s的可视窗口) */
#define WATCH_HR_MAX        120		/* Y轴上限(BPM) */
#define WATCH_COL_HR_CYAN   0x38E1FF /* 心率绿青(与卡片上的心形同色系) */
#define WATCH_COL_CHART_BG  0x111111 /* 图表底色 */
/***********************************Defines************************************/

/**********************************Declaring***********************************/
/* 心率页: 数值 + 折线图 + 自己的1s刷新定时器 */
typedef struct
{
	page_base_t base;
	lv_obj_t *p_val;			  /* BPM 大字 */
	lv_obj_t *p_chart;			  /* 折线图 */
	lv_chart_series_t *p_series;  /* 图线 */
	lv_timer_t *p_timer;
} watch_heart_t;

static void watch_heart_render(watch_heart_t *p_heart);
static void watch_heart_timer_cb(lv_timer_t *p_timer);

static void watch_heart_swipe_cb(lv_event_t *e);

static void watch_heart_create(page_base_t *p_page);
static void watch_heart_destroy(page_base_t *p_page);
static void watch_heart_show(page_base_t *p_page);
static void watch_heart_hide(page_base_t *p_page);
/**********************************Declaring***********************************/

/*********************************Static Data**********************************/
static watch_heart_t s_heart;

static const page_vtable_t s_heart_vtable =
{
	.pf_create = watch_heart_create,
	.pf_destroy = watch_heart_destroy,
	.pf_show = watch_heart_show,
	.pf_hide = watch_heart_hide,
};
/*********************************Static Data**********************************/

/******************************************************************************
 * @name    watch_heart_render
 * @brief   把模型数据刷到本页控件上(数值 + 图表推进一个点)
 * @param   p_heart[in] 心率页
 *
 * @return  无
 *****************************************************************************/
static void watch_heart_render(watch_heart_t *p_heart)
{
	char buf[16];

	if (NULL == p_heart)
	{
		return;
	}

	if (NULL != p_heart->p_val)
	{
		snprintf(buf, sizeof(buf), "%lu", (unsigned long)watch_model_hr());
		lv_label_set_text(p_heart->p_val, buf);
	}

	if (NULL != p_heart->p_chart && NULL != p_heart->p_series)
	{
		lv_chart_set_next_value(p_heart->p_chart, p_heart->p_series,
								(lv_coord_t)watch_model_hr());
	}
}

/******************************************************************************
 * @name    watch_heart_timer_cb
 * @brief   本页刷新定时器: 每秒刷新数值并推进一个图表点
 * @param   p_timer[in] LVGL定时器(未使用)
 *
 * @return  无
 *****************************************************************************/
static void watch_heart_timer_cb(lv_timer_t *p_timer)
{
	(void)p_timer;

	watch_heart_render(&s_heart);
}

/******************************************************************************
 * @name    watch_heart_swipe_cb
 * @brief   本页手势: 左滑→表盘(MOVE_LEFT), 右滑→血氧页(MOVE_RIGHT)
 * @param   e[in] LVGL事件
 *
 * @return  无
 *
 * @note    ★watch_swipe_track() 每次调用都会读走并清零内部累计位移★ 所以整个回调
 *          里**只能调一次**, 把结果存进局部变量再比较. 若写成上面 if 一次、else if
 *          又调一次, 第二次必然拿到 WATCH_SWIPE_NONE(位移已被第一次清掉) ——
 *          右滑分支就永远不成立, 而且是静默失效
 *****************************************************************************/
static void watch_heart_swipe_cb(lv_event_t *e)
{
	watch_swipe_dir_t dir = watch_swipe_track(e);

	if (WATCH_SWIPE_LEFT == dir)
	{
		watch_switch(WATCH_PAGE_ID_HOME, LV_SCR_LOAD_ANIM_MOVE_LEFT);

		/* 通知 APP 层: 5 = 往左切页(跨页统一的方向旗标, 见
		   APP/APPPAGE/UserPage/cywatch_app_heart_page.c 的旗标表).
		   ★不能放 ISR: 本回调跑在 lvgl 任务, send 内含 osKernelLock, 合法★ */
		x_port_easyapp_event_send(EVT_SERVICE_UIHOME_SWITCH_PAGE, 5, NULL);
	}
	else if (WATCH_SWIPE_RIGHT == dir)
	{
		/* 右滑进血氧页(0x0303), 动画方向与手势一致 */
		watch_switch(WATCH_PAGE_ID_SPO2, LV_SCR_LOAD_ANIM_MOVE_RIGHT);

		/* 3 = 往右切页(与 home 页右滑进本页用的是同一个值, 见
		   cywatch_app_heart_page.c 的旗标表: 3=TO_RIGHT_PAGE) */
		x_port_easyapp_event_send(EVT_SERVICE_UIHOME_SWITCH_PAGE, 3, NULL);
	}
	else
	{
		/* 上/下滑: 本页没有对应页面, 什么都不做 */
	}
}

/******************************************************************************
 * @name    watch_heart_create
 * @brief   建本页: 标题/大字数值/BPM + 折线图 + 手势条 + 刷新定时器
 * @param   p_page[in] 页面基类指针
 *
 * @return  无
 *****************************************************************************/
static void watch_heart_create(page_base_t *p_page)
{
	watch_heart_t *p_heart = (watch_heart_t *)p_page;
	lv_obj_t *scr = lv_obj_create(NULL);
	lv_obj_t *lbl = NULL;
	lv_obj_t *icon = NULL;
	int32_t p = 0;

	/* 清掉可能残留的上一次实例句柄(重建时必须从干净状态开始) */
	p_heart->p_val = NULL;
	p_heart->p_chart = NULL;
	p_heart->p_series = NULL;

	lv_obj_set_style_bg_color(scr, lv_color_hex(WATCH_COL_DARK), 0);

	/* 标题: 心形图 + "心率" */
	icon = lv_image_create(scr);
	lv_image_set_src(icon, &icon_heart_18x18_RGB565A8_NONE);
	lv_obj_align(icon, LV_ALIGN_TOP_LEFT, 20, 24);

	lbl = lv_label_create(scr);
	lv_label_set_text(lbl, "心率");
	lv_obj_set_style_text_font(lbl, &lv_font_alibaba_puhuiti_14, 0);
	lv_obj_set_style_text_color(lbl, lv_color_hex(WATCH_COL_TEXT), 0);
	lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, 52, 26);

	/* 大字心率值 */
	p_heart->p_val = lv_label_create(scr);
	lv_label_set_text(p_heart->p_val, "68");
	lv_obj_set_style_text_font(p_heart->p_val, &lv_font_montserrat_regular_20, 0);
	lv_obj_set_style_text_color(p_heart->p_val, lv_color_hex(WATCH_COL_TEXT), 0);
	lv_obj_set_style_text_align(p_heart->p_val, LV_TEXT_ALIGN_CENTER, 0);
	lv_obj_set_width(p_heart->p_val, 120);
	lv_obj_align(p_heart->p_val, LV_ALIGN_TOP_MID, 0, 52);

	lbl = lv_label_create(scr);
	lv_label_set_text(lbl, "BPM");
	lv_obj_set_style_text_color(lbl, lv_color_hex(WATCH_COL_GRAY), 0);
	lv_obj_align(lbl, LV_ALIGN_TOP_MID, 0, 86);

	/* 72点折线图 */
	p_heart->p_chart = lv_chart_create(scr);
	lv_obj_set_size(p_heart->p_chart, 216, 128);
	lv_obj_align(p_heart->p_chart, LV_ALIGN_TOP_MID, 0, 112);
	lv_chart_set_type(p_heart->p_chart, LV_CHART_TYPE_LINE);
	lv_chart_set_point_count(p_heart->p_chart, WATCH_HR_POINTS);
	lv_chart_set_range(p_heart->p_chart, LV_CHART_AXIS_PRIMARY_X, 0, WATCH_HR_POINTS - 1);
	lv_chart_set_range(p_heart->p_chart, LV_CHART_AXIS_PRIMARY_Y, 0, WATCH_HR_MAX);
	lv_chart_set_div_line_count(p_heart->p_chart, 2, 3);
	lv_obj_set_style_bg_color(p_heart->p_chart, lv_color_hex(WATCH_COL_CHART_BG), 0);
	lv_obj_set_style_border_width(p_heart->p_chart, 0, 0);
	lv_obj_set_style_radius(p_heart->p_chart, 10, 0);

	p_heart->p_series = lv_chart_add_series(p_heart->p_chart,
											lv_color_hex(WATCH_COL_HR_CYAN),
											LV_CHART_AXIS_PRIMARY_Y);
	lv_obj_set_style_line_width(p_heart->p_chart, 2, LV_PART_ITEMS);

	/* 初始填充: 69~79 的一个柔和正弦波形.
	   LVGL9.3 的 lv_trigo_sin 入参是角度(0~360), 返回值范围 LV_TRIGO_SIN_MAX=32768 */
	for (p = 0; p < WATCH_HR_POINTS; p++)
	{
		int32_t v = 74 + (int32_t)(5 * lv_trigo_sin((int16_t)((p * 360) / WATCH_HR_POINTS)) / LV_TRIGO_SIN_MAX);

		lv_chart_set_next_value(p_heart->p_chart, p_heart->p_series, v);
	}

	/* 全屏手势条(页面无点击内容, 整屏可滑): 左滑回表盘 */
	watch_strip_create(scr, WATCH_SCREEN_W, WATCH_SCREEN_H, 0, 0, watch_heart_swipe_cb);

	/* 本页刷新定时器 */
	p_heart->p_timer = lv_timer_create(watch_heart_timer_cb, WATCH_TICK_MS, NULL);

	p_page->obj = scr;
	watch_heart_render(p_heart);

	printf("WATCH page create ->HEART\r\n");
}

/******************************************************************************
 * @name    watch_heart_destroy
 * @brief   销毁本页: 删本页定时器 + 置空控件/图线句柄
 * @param   p_page[in] 页面基类指针
 *
 * @return  无
 *
 * @note    **必须删定时器**: lv_timer 不是 widget, lv_obj_delete 管不到它
 *****************************************************************************/
static void watch_heart_destroy(page_base_t *p_page)
{
	watch_heart_t *p_heart = (watch_heart_t *)p_page;

	if (NULL != p_heart->p_timer)
	{
		lv_timer_delete(p_heart->p_timer);
		p_heart->p_timer = NULL;
	}

	/* 图表的数据缓冲随 chart 一起被 LVGL 删掉, 这里只清句柄 */
	p_heart->p_val = NULL;
	p_heart->p_chart = NULL;
	p_heart->p_series = NULL;

	printf("WATCH page destroy ->HEART\r\n");
}

/******************************************************************************
 * @name    watch_heart_show
 * @brief   本页变为可见: 立即补一帧 + 恢复本页定时器
 * @param   p_page[in] 页面基类指针
 *
 * @return  无
 *
 * @note    lv_timer_reset 不能省: lv_timer_resume 只清 paused 标志, 不重置周期
 *****************************************************************************/
static void watch_heart_show(page_base_t *p_page)
{
	watch_heart_t *p_heart = (watch_heart_t *)p_page;

	watch_heart_render(p_heart);

	if (NULL != p_heart->p_timer)
	{
		lv_timer_reset(p_heart->p_timer);
		lv_timer_resume(p_heart->p_timer);
	}
}

/******************************************************************************
 * @name    watch_heart_hide
 * @brief   本页离开屏幕: 暂停本页定时器
 * @param   p_page[in] 页面基类指针
 *
 * @return  无
 *****************************************************************************/
static void watch_heart_hide(page_base_t *p_page)
{
	watch_heart_t *p_heart = (watch_heart_t *)p_page;

	if (NULL != p_heart->p_timer)
	{
		lv_timer_pause(p_heart->p_timer);
	}
}

/******************************************************************************
 * @name    watch_page_heart_register
 * @brief   把心率页注册进管理器(不创建控件, 切过去时才建)
 * @param   p_mgr[in] 管理器实例
 *
 * @return  page_mgr_register 的返回码(0 成功; -1/-2/-3/-4/-5/-6 见 PageMem.h)
 *****************************************************************************/
int8_t watch_page_heart_register(page_mgr_t *p_mgr)
{
	return page_mgr_register(p_mgr, &s_heart.base, &s_heart_vtable, WATCH_PAGE_ID_HEART);
}
