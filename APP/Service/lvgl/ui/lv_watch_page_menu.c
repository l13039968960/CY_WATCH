/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file lv_watch_page_menu.c
 *
 * @par dependencies
 * - lv_watch_page.h
 * - lvgl.h
 *
 * @author zw1194
 *
 * @brief 手表UI - 菜单页(menu, ID 0x0301): 3x2 功能按钮(运动/心率/睡眠/音乐/
 *        消息/设置) + 底部手势条. 由 PageMem 页面管理器托管.
 *
 * Processing flow:
 *
 * pf_create : 建独立屏幕 + 6个按钮 + 底部手势条(本页**只有**它, 见下);
 * 手势      : 上滑→home(OUT_TOP); "心率"按钮点击→heart页.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note ★本页实现了一张"最简 vtable": 只有 pf_create★ —— 页面全是静态控件, 没有
 *       定时器(因此没有 pf_destroy 要删的东西)、没有要暂停的刷新、没有要恢复的
 *       状态(因此没有 pf_hide/pf_show). vtable 的四个回调都是**可选**的, 按需实现
 *       即可 —— 这是 PageMem 相对"每个页面都要写四件套"的地方.
 *
 * @note 按钮 → 页面 的绑定靠**事件 user_data 里放目标页 ID**, 不靠按钮文字:
 *       老版本是 strcmp(label, "心率"), 加/改标签就会静默断链, 还要拖进 string.h.
 *       占位按钮 user_data 为 NULL(点击只打印).
 ******************************************************************************/
#include "lv_watch_page.h"
#include "../../APP/EasyAPP/port/easyapp_port.h"

#include <stdio.h>

/* 子集字库(实现在 LVGL/assets/fonts, 生成脚本 MDK-ARM/gen_watch_fonts.py):
   中文标签用14 —— 只含"运动心率睡眠音乐消息设置"这12个字 */
LV_FONT_DECLARE(lv_font_alibaba_puhuiti_14)

/***********************************Defines************************************/
#define WATCH_BTN_CNT   6	/* 3列 x 2行 */
#define WATCH_STRIP_H   64	/* 底部手势条高度(在第二行按钮下方) */
#define WATCH_STRIP_Y   216 /* 手势条Y: 第二行按钮底边(130+82)之下 */
/***********************************Defines************************************/

/**********************************Declaring***********************************/
/* 菜单页: 全是静态按钮, 没有动态内容 —— 所以结构体里除了基类什么都没有 */
typedef struct
{
	page_base_t base;
} watch_menu_t;

static void watch_menu_swipe_cb(lv_event_t *e);
static void watch_menu_btn_cb(lv_event_t *e);

static void watch_menu_create(page_base_t *p_page);
/**********************************Declaring***********************************/

/*********************************Static Data**********************************/
static watch_menu_t s_menu;

/* 只有 pf_create: 纯静态页, 没有定时器/刷新状态需要收尾 */
static const page_vtable_t s_menu_vtable =
{
	.pf_create = watch_menu_create,
};
/*********************************Static Data**********************************/

/******************************************************************************
 * @name    watch_menu_swipe_cb
 * @brief   本页手势: 上滑→表盘(OUT_TOP)
 * @param   e[in] LVGL事件
 *
 * @return  无
 *****************************************************************************/
static void watch_menu_swipe_cb(lv_event_t *e)
{
	if (WATCH_SWIPE_UP == watch_swipe_track(e))
	{
		watch_switch(WATCH_PAGE_ID_HOME, LV_SCR_LOAD_ANIM_OUT_TOP);

		/* 通知 APP 层: 6 = 往上切页(跨页统一的方向旗标, 见
		   APP/APPPAGE/UserPage/cywatch_app_menu_page.c 的旗标表).
		   ★不能放 ISR: 本回调跑在 lvgl 任务, send 内含 osKernelLock, 合法★ */
		x_port_easyapp_event_send(EVT_SERVICE_UIHOME_SWITCH_PAGE, 6, NULL);
	}
}

/******************************************************************************
 * @name    watch_menu_btn_cb
 * @brief   按钮点击: 事件 user_data 里是目标页 ID, 为 NULL 表示占位按钮
 * @param   e[in] LVGL事件(CLICKED)
 *
 * @return  无
 *****************************************************************************/
static void watch_menu_btn_cb(lv_event_t *e)
{
	uint16_t page_id = (uint16_t)(uintptr_t)lv_event_get_user_data(e);

	if (0u != page_id)
	{
		watch_switch(page_id, LV_SCR_LOAD_ANIM_MOVE_RIGHT);

		/* 通知 APP 层: 3 = 往右切页(与表盘右滑同一个旗标值).
		   @note 旗标必须与**目标页**一致而不是与"哪个按钮"一致 —— 当前唯一非占位
		         按钮的 user_data 就是 WATCH_PAGE_ID_HEART(见 watch_menu_create),
		         所以固定 3; 将来加按钮指向别的页时, 这里要按目标页改值.
		         占位按钮(user_data 为 NULL)走 else 分支, 不发事件 */
		x_port_easyapp_event_send(EVT_SERVICE_UIHOME_SWITCH_PAGE, 3, NULL);
	}
	else
	{
		printf("WATCH menu btn (placeholder)\r\n");
	}
}

/******************************************************************************
 * @name    watch_menu_create
 * @brief   建本页: 3x2功能按钮 + 底部手势条
 * @param   p_page[in] 页面基类指针
 *
 * @return  无
 *
 * @note    只有"心率"按钮带目标页 ID(其余是占位); 心率按钮图标用 18x18 心形图
 *          (LVGL9.3 无 LV_SYMBOL_HEART), 所以它的 sym[] 项只是个占位不用
 *****************************************************************************/
static void watch_menu_create(page_base_t *p_page)
{
	static const char *const label[WATCH_BTN_CNT] =
		{"运动", "心率", "睡眠", "音乐", "消息", "设置"};
	static const char *const sym[WATCH_BTN_CNT] =
		{LV_SYMBOL_DRIVE, LV_SYMBOL_HOME, LV_SYMBOL_PAUSE,
		 LV_SYMBOL_AUDIO, LV_SYMBOL_LIST, LV_SYMBOL_SETTINGS};
	static const uint32_t color[WATCH_BTN_CNT] =
		{0xEA580C, 0x0284C7, 0x6D28D9, 0xDB2777, 0x16A34A, 0x64748B};
	static const lv_coord_t btn_x[3] = {12, 86, 160};
	static const lv_coord_t btn_y[2] = {36, 130};
	lv_obj_t *scr = lv_obj_create(NULL);
	lv_obj_t *btn = NULL;
	lv_obj_t *icon = NULL;
	lv_obj_t *lbl = NULL;
	uint32_t i = 0;

	lv_obj_set_style_bg_color(scr, lv_color_hex(WATCH_COL_DARK), 0);

	for (i = 0; i < WATCH_BTN_CNT; i++)
	{
		btn = lv_button_create(scr);
		lv_obj_set_pos(btn, btn_x[i % 3], btn_y[i / 3]);
		lv_obj_set_size(btn, 68, 82);
		lv_obj_set_style_bg_color(btn, lv_color_hex(color[i]), 0);
		lv_obj_set_style_radius(btn, 18, 0);
		lv_obj_set_style_pad_all(btn, 0, 0);

		if (1 == i)
		{
			/* 心率按钮: 18x18心形图(LVGL9.3无LV_SYMBOL_HEART符号) */
			lv_obj_t *p_img = lv_image_create(btn);
			lv_image_set_src(p_img, &icon_heart_18x18_RGB565A8_NONE);
			lv_obj_align(p_img, LV_ALIGN_TOP_MID, 0, 14);
		}
		else
		{
			icon = lv_label_create(btn);
			lv_label_set_text(icon, sym[i]);
			lv_obj_set_style_text_color(icon, lv_color_hex(WATCH_COL_TEXT), 0);
			lv_obj_align(icon, LV_ALIGN_TOP_MID, 0, 16);
		}

		lbl = lv_label_create(btn);
		lv_label_set_text(lbl, label[i]);
		lv_obj_set_style_text_font(lbl, &lv_font_alibaba_puhuiti_14, 0);
		lv_obj_set_style_text_color(lbl, lv_color_hex(WATCH_COL_TEXT), 0);
		lv_obj_align(lbl, LV_ALIGN_TOP_MID, 0, 52);

		/* 心率按钮点击→heart页(目标页ID塞在事件user_data里); 其余占位 */
		lv_obj_add_event_cb(btn, watch_menu_btn_cb, LV_EVENT_CLICKED,
							(1 == i) ? (void *)(uintptr_t)WATCH_PAGE_ID_HEART : NULL);
	}

	/* 底部手势条(按钮下方空白区, 不挡按钮): 上滑回表盘 */
	watch_strip_create(scr, WATCH_SCREEN_W, WATCH_STRIP_H, 0, WATCH_STRIP_Y, watch_menu_swipe_cb);

	p_page->obj = scr;

	printf("WATCH page create ->MENU\r\n");
}

/******************************************************************************
 * @name    watch_page_menu_register
 * @brief   把菜单页注册进管理器(不创建控件, 切过去时才建)
 * @param   p_mgr[in] 管理器实例
 *
 * @return  page_mgr_register 的返回码(0 成功; -1/-2/-3/-4/-5/-6 见 PageMem.h)
 *****************************************************************************/
int8_t watch_page_menu_register(page_mgr_t *p_mgr)
{
	return page_mgr_register(p_mgr, &s_menu.base, &s_menu_vtable, WATCH_PAGE_ID_MENU);
}
