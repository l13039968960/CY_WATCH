/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file lv_watch_page_home.c
 *
 * @par dependencies
 * - lv_watch_page.h
 * - lvgl.h
 * - ../../system/Rtc/cywatch_rtc.h (cywatch_rtc_time_t, 经 watch_model_datetime 取时间)
 *
 * @author zw1194
 *
 * @brief 手表UI - 表盘页(home, ID 0x0300): 电池图标 + 日期 + 大字时钟 + 三张健康
 *        卡片(步数/心率/卡路里) + 全屏手势条. 由 PageMem 页面管理器托管.
 *
 * Processing flow:
 *
 * pf_create : 建独立屏幕 + 全部控件 + 本页1s刷新定时器(建出来即运行);
 * pf_show   : 立即补一帧 + lv_timer_reset + lv_timer_resume;
 * pf_hide   : lv_timer_pause —— 被缓存的隐藏页不再每秒白刷它的控件;
 * pf_destroy: lv_timer_delete + 控件句柄置 NULL;
 * 手势      : 下滑→menu(OVER_BOTTOM), 右滑→heart(MOVE_RIGHT).
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 本页只"把数据画出来", **不推进**数据. 数据有两路, 都不是本页拥有的状态:
 *       1) **日期与大字时钟**来自硬件 RTC, 经 watch_model_datetime() 每次渲染真读
 *          一次外设(所以它一直走, 与页面在不在屏上无关);
 *       2) **三张卡片**来自 DataModel(cywatch_app_datamodel.h 的 Step/HeartRate/KCal),
 *          由 HeartRate 服务在另一个任务里写.
 *       所以本页被 LRU 淘汰重建后接着画的是最新值, 而不是从 00:00:00 或某个初值重来.
 *
 * @note 日期与时钟**共用一次 RTC 读**(见 watch_home_render): 两者必须同一时刻, 否则
 *       跨秒/跨日那一帧会自相矛盾.
 *
 * @note 时钟格式必须**保持 8 字符 HH:MM:SS** —— 上电自检的 ui_find_clock() 按"恰好
 *       8 字符 + 下标 2/5 是冒号"认时钟. 改成别的形状会让自检第 8 步找不到它.
 *
 * @note 布局全部是**绝对坐标**: 本工程 LV_USE_FLEX=0 / LV_USE_GRID=0, 布局引擎
 *       没编进来, 写 flex 会直接编译不过(移植时踩过).
 ******************************************************************************/
#include "lv_watch_page.h"
#include "../../APP/EasyAPP/port/easyapp_port.h"
#include "../../APP/APPPAGE/DataModel/cywatch_app_datamodel.h"
#include "../../system/Rtc/cywatch_rtc.h"
#include <stdio.h>

/* 子集字库(实现在 LVGL/assets/fonts, 生成脚本 MDK-ARM/gen_watch_fonts.py):
   时钟用20, 卡片数值/日期用16 —— 两者都只需 ASCII */
LV_FONT_DECLARE(lv_font_montserrat_regular_20)
LV_FONT_DECLARE(lv_font_montserrat_regular_16)

/***********************************Defines************************************/
#define WATCH_CARD_CNT      3		 /* 卡片数: 步数/心率/卡路里 */
#define WATCH_COL_PAGE_BG   0x000000 /* 表盘底色: 纯黑(菜单/心率页用 WATCH_COL_DARK) */
#define WATCH_COL_CARD_BG   0x1A1A1A /* 卡片底 */
#define WATCH_COL_GRN       0x4CD964 /* 电池图标绿 */
/***********************************Defines************************************/

/**********************************Declaring***********************************/
/* 表盘页: 时钟 + 三张卡片 + 自己的1s刷新定时器 */
typedef struct
{
	page_base_t base;
	lv_obj_t *p_date;				   /* 日期 YYYY-MM-DD(硬件RTC) */
	lv_obj_t *p_clock;				  /* HH:MM:SS(硬件RTC) */
	lv_obj_t *p_card[WATCH_CARD_CNT]; /* 步数/心率/卡路里 数值 */
	lv_timer_t *p_timer;			  /* 本页刷新(pf_hide 暂停 / pf_show 恢复) */
} watch_home_t;

static void watch_home_render(watch_home_t *p_home);
static void watch_home_timer_cb(lv_timer_t *p_timer);

static void watch_home_swipe_cb(lv_event_t *e);

static void watch_home_create(page_base_t *p_page);
static void watch_home_destroy(page_base_t *p_page);
static void watch_home_show(page_base_t *p_page);
static void watch_home_hide(page_base_t *p_page);
/**********************************Declaring***********************************/

/*********************************Static Data**********************************/
static watch_home_t s_home;

static const page_vtable_t s_home_vtable =
{
	.pf_create = watch_home_create,
	.pf_destroy = watch_home_destroy,
	.pf_show = watch_home_show,
	.pf_hide = watch_home_hide,
};
/*********************************Static Data**********************************/

/******************************************************************************
 * @name    watch_home_render
 * @brief   把模型数据刷到本页控件上(时钟 + 三张卡片)
 * @param   p_home[in] 表盘页
 *
 * @return  无
 *
 * @note    控件指针可能为 NULL(页面已销毁), 每处都判一下; 本函数只读模型
 *****************************************************************************/
static void watch_home_render(watch_home_t *p_home)
{
	char buf[16];
	cywatch_rtc_time_t now;

	if (NULL == p_home)
	{
		return;
	}

	/* 日期与时钟**同源同一刻**: 只读一次 RTC 喂两个控件. 分成两次读会在跨秒/跨日的
	   那一瞬间画出"日期已经是新的一天、时间还停在旧的"这种自相矛盾的一帧 */
	if (0 == watch_model_datetime(&now))
	{
		if (NULL != p_home->p_date)
		{
			snprintf(buf, sizeof(buf), "%04u-%02u-%02u",
					 (unsigned)now.year, (unsigned)now.month, (unsigned)now.day);
			lv_label_set_text(p_home->p_date, buf);
		}

		if (NULL != p_home->p_clock)
		{
			snprintf(buf, sizeof(buf), "%02u:%02u:%02u",
					 (unsigned)now.hour, (unsigned)now.minute, (unsigned)now.second);
			lv_label_set_text(p_home->p_clock, buf);
		}
	}
	else
	{
		/* RTC 不可用(init 失败但调用方放过了): 画占位, **不要**画一个看着正常的假
		   时间 —— USE_FULL_ASSERT 是关的, 静默走偏正是这个模块最该防的失败模式 */
		if (NULL != p_home->p_date)
		{
			lv_label_set_text(p_home->p_date, "-- -- --");
		}
		if (NULL != p_home->p_clock)
		{
			lv_label_set_text(p_home->p_clock, "--:--:--");
		}
	}

	if (NULL != p_home->p_card[0])
	{
		snprintf(buf, sizeof(buf), "%lu", (unsigned long)Step);
		lv_label_set_text(p_home->p_card[0], buf);
	}
	if (NULL != p_home->p_card[1])
	{
		snprintf(buf, sizeof(buf), "%lu", (unsigned long)(HeartRate));
		lv_label_set_text(p_home->p_card[1], buf);
	}
	if (NULL != p_home->p_card[2])
	{
		snprintf(buf, sizeof(buf), "%lu", (unsigned long)KCal);
		lv_label_set_text(p_home->p_card[2], buf);
	}
}

/******************************************************************************
 * @name    watch_home_timer_cb
 * @brief   本页刷新定时器: 每秒重画时钟与卡片
 * @param   p_timer[in] LVGL定时器(未使用)
 *
 * @return  无
 *
 * @note    本页不显示时该定时器被 pf_hide 暂停 —— 隐藏页不再每秒白跑刷新
 ********************************1*********************************************/
static void watch_home_timer_cb(lv_timer_t *p_timer)
{
	(void)p_timer;

	watch_home_render(&s_home);
}

/******************************************************************************
 * @name    watch_home_swipe_cb
 * @brief   本页手势: 下滑→菜单(OVER_BOTTOM), 右滑→心率(MOVE_RIGHT),
 *          左滑→OTA升级页(MOVE_LEFT)
 * @param   e[in] LVGL事件
 *
 * @return  无
 *
 * @note    映射写在**本页自己的**回调里, 所以不必再判"当前是哪一页"
 * @note    ★watch_swipe_track() 只能调一次★ 它每次调用都会**读并清零**累积位移,
 *          所以必须先存进局部变量再逐分支比较(下面 dir 就是); 三个分支各调一次的话
 *          第二、三次必然是 WATCH_SWIPE_NONE
 * @note    表盘是环上唯一有三条出路的页(左 OTA / 右 HEART / 下 MENU) —— 见
 *          lv_watch_page.h 头部的环形拓扑图
 *****************************************************************************/
static void watch_home_swipe_cb(lv_event_t *e)
{
	watch_swipe_dir_t dir = watch_swipe_track(e);

	if (WATCH_SWIPE_DOWN == dir)
	{
		watch_switch(WATCH_PAGE_ID_MENU, LV_SCR_LOAD_ANIM_OVER_BOTTOM);
		x_port_easyapp_event_send(EVT_SERVICE_UIHOME_SWITCH_PAGE,4, NULL);
	}
	else if (WATCH_SWIPE_RIGHT == dir)
	{
		watch_switch(WATCH_PAGE_ID_HEART, LV_SCR_LOAD_ANIM_MOVE_RIGHT);
		x_port_easyapp_event_send(EVT_SERVICE_UIHOME_SWITCH_PAGE, 3, NULL);
	}
	else if (WATCH_SWIPE_LEFT == dir)
	{
		/* 左滑 → OTA升级页(环上 OTA 在表盘左边). 旗标 5 = 目标页在左方 */
		watch_switch(WATCH_PAGE_ID_OTA, LV_SCR_LOAD_ANIM_MOVE_LEFT);
		x_port_easyapp_event_send(EVT_SERVICE_UIHOME_SWITCH_PAGE, 5, NULL);
	}
	else
	{
		/* 上滑: 本页没有对应页面, 什么都不做 */
	}
}

/******************************************************************************
 * @name    watch_home_create
 * @brief   建本页: 电池/日期/大字时钟/三张卡片/手势条 + 刷新定时器
 * @param   p_page[in] 页面基类指针
 *
 * @return  无
 *
 * @note    根对象必须是独立屏幕(lv_obj_create(NULL))并写回 p_page->obj, 否则
 *          page_mgr_switch 判为违约(返回 -6)
 *****************************************************************************/
static void watch_home_create(page_base_t *p_page)
{
	watch_home_t *p_home = (watch_home_t *)p_page; /* 基类是首成员, 零偏移强转 */
	lv_obj_t *scr = lv_obj_create(NULL);
	lv_obj_t *icon = NULL;
	lv_obj_t *lbl = NULL;
	lv_obj_t *card = NULL;
	uint32_t i = 0;

	/* 三张卡片: 标题(英文, 避开缺字形) / 18x18图标图 / X坐标 */
	static const char *const title[WATCH_CARD_CNT] = {"STEPS", "HR", "KCAL"};
	static const lv_image_dsc_t *const card_icon[WATCH_CARD_CNT] =
		{&icon_steps_18x18_RGB565A8_NONE, &icon_heart_18x18_RGB565A8_NONE, &icon_flame_18x18_RGB565A8_NONE};
	static const lv_coord_t card_x[WATCH_CARD_CNT] = {3, 83, 163};

	lv_obj_set_style_bg_color(scr, lv_color_hex(WATCH_COL_PAGE_BG), 0);

	/* 电池图标(右上, 绿色) */
	icon = lv_label_create(scr);
	lv_label_set_text(icon, LV_SYMBOL_CHARGE);
	lv_obj_set_style_text_color(icon, lv_color_hex(WATCH_COL_GRN), 0);
	lv_obj_align(icon, LV_ALIGN_TOP_RIGHT, -10, 5);

	/* 日期(随时钟一起每秒刷新, 所以句柄必须存下来, 不能像以前那样用完就丢) */
	p_home->p_date = lv_label_create(scr);
	lv_label_set_text(p_home->p_date, "-- -- --");
	lv_obj_set_style_text_font(p_home->p_date, &lv_font_montserrat_regular_16, 0);
	lv_obj_set_style_text_color(p_home->p_date, lv_color_hex(WATCH_COL_GRAY), 0);
	lv_obj_align(p_home->p_date, LV_ALIGN_TOP_MID, 0, 16);

	/* 大字时钟(每秒刷新) */
	p_home->p_clock = lv_label_create(scr);
	lv_label_set_text(p_home->p_clock, "00:00:00");
	lv_obj_set_style_text_font(p_home->p_clock, &lv_font_montserrat_regular_20, 0);
	lv_obj_set_style_text_color(p_home->p_clock, lv_color_hex(WATCH_COL_TEXT), 0);
	lv_obj_set_style_text_align(p_home->p_clock, LV_TEXT_ALIGN_CENTER, 0);
	lv_obj_set_width(p_home->p_clock, 200);
	lv_obj_align(p_home->p_clock, LV_ALIGN_TOP_MID, 0, 52);

	/* 三张健康卡片(步数/心率/卡路里) */
	for (i = 0; i < WATCH_CARD_CNT; i++)
	{
		card = lv_obj_create(scr);
		lv_obj_set_pos(card, card_x[i], 196);
		lv_obj_set_size(card, 74, 76);
		lv_obj_set_style_bg_color(card, lv_color_hex(WATCH_COL_CARD_BG), 0);
		lv_obj_set_style_radius(card, 12, 0);
		lv_obj_set_style_border_width(card, 0, 0);
		lv_obj_set_style_pad_all(card, 0, 0);

		/* 卡图标: 步数/心率/卡路里 (18x18图片, 语义正确) */
		lv_obj_t *p_img = lv_image_create(card);
		lv_image_set_src(p_img, card_icon[i]);
		lv_obj_align(p_img, LV_ALIGN_TOP_MID, 0, 8);

		p_home->p_card[i] = lv_label_create(card);
		lv_label_set_text(p_home->p_card[i], "0");
		lv_obj_set_style_text_font(p_home->p_card[i], &lv_font_montserrat_regular_16, 0);
		lv_obj_set_style_text_color(p_home->p_card[i], lv_color_hex(WATCH_COL_TEXT), 0);
		lv_obj_align(p_home->p_card[i], LV_ALIGN_TOP_MID, 0, 36);

		lbl = lv_label_create(card);
		lv_label_set_text(lbl, title[i]);
		lv_obj_set_style_text_color(lbl, lv_color_hex(WATCH_COL_GRAY), 0);
		lv_obj_align(lbl, LV_ALIGN_TOP_MID, 0, 58);
	}

	/* 全屏手势条(卡片只读, 无需点击, 整屏可滑) */
	watch_strip_create(scr, WATCH_SCREEN_W, WATCH_SCREEN_H, 0, 0, watch_home_swipe_cb);

	/* 本页刷新定时器(建出来就是运行的; pf_hide 暂停 / pf_show 恢复) */
	p_home->p_timer = lv_timer_create(watch_home_timer_cb, WATCH_TICK_MS, NULL);

	/* 交回根对象(管理器随后挂 DELETE 回调)并立刻渲染一次, 避免上屏时是占位值 */
	p_page->obj = scr;
	watch_home_render(p_home);

	printf("WATCH page create ->HOME\r\n");
}

/******************************************************************************
 * @name    watch_home_destroy
 * @brief   销毁本页: 删本页定时器 + 置空控件指针
 * @param   p_page[in] 页面基类指针
 *
 * @return  无
 *
 * @note    **必须删定时器**: lv_timer 不是 widget, lv_obj_delete 管不到它, 漏删
 *          会得到一个指向已释放控件的野定时器(PageMem.h 明文约定)
 *****************************************************************************/
static void watch_home_destroy(page_base_t *p_page)
{
	watch_home_t *p_home = (watch_home_t *)p_page;
	uint32_t i = 0;

	if (NULL != p_home->p_timer)
	{
		lv_timer_delete(p_home->p_timer);
		p_home->p_timer = NULL;
	}

	p_home->p_date = NULL;
	p_home->p_clock = NULL;
	for (i = 0; i < WATCH_CARD_CNT; i++)
	{
		p_home->p_card[i] = NULL;
	}

	printf("WATCH page destroy ->HOME\r\n");
}

/******************************************************************************
 * @name    watch_home_show
 * @brief   本页变为可见: 立即补一帧 + 恢复本页定时器
 * @param   p_page[in] 页面基类指针
 *
 * @return  无
 *
 * @note    lv_timer_reset 不能省: lv_timer_resume 只清 paused 标志, **不重置周期**,
 *          页面藏久了之后直接 resume 会在下一次 lv_timer_handler 里立刻触发一次
 *****************************************************************************/
static void watch_home_show(page_base_t *p_page)
{
	watch_home_t *p_home = (watch_home_t *)p_page;

	watch_home_render(p_home);

	if (NULL != p_home->p_timer)
	{
		lv_timer_reset(p_home->p_timer);
		lv_timer_resume(p_home->p_timer);
	}
}

/******************************************************************************
 * @name    watch_home_hide
 * @brief   本页离开屏幕: 暂停本页定时器
 * @param   p_page[in] 页面基类指针
 *
 * @return  无
 *
 * @note    被缓存的页面对象是活着的, 定时器不停会继续每秒刷新看不见的控件
 *****************************************************************************/
static void watch_home_hide(page_base_t *p_page)
{
	watch_home_t *p_home = (watch_home_t *)p_page;

	if (NULL != p_home->p_timer)
	{
		lv_timer_pause(p_home->p_timer);
	}
}

/******************************************************************************
 * @name    watch_page_home_register
 * @brief   把表盘页注册进管理器(不创建控件, 切过去时才建)
 * @param   p_mgr[in] 管理器实例
 *
 * @return  page_mgr_register 的返回码(0 成功; -1/-2/-3/-4/-5/-6 见 PageMem.h)
 *****************************************************************************/
int8_t watch_page_home_register(page_mgr_t *p_mgr)
{
	return page_mgr_register(p_mgr, &s_home.base, &s_home_vtable, WATCH_PAGE_ID_HOME);
}
