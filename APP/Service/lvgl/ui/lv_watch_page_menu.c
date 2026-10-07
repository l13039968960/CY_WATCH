/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file lv_watch_page_menu.c
 *
 * @par dependencies
 * - cywatch_log.h
 * - lv_watch_page.h
 * - lvgl.h
 *
 * @author zw1194
 *
 * @brief 手表UI - 菜单页(menu, ID 0x0301): 左侧 2x2 功能按钮(消息/睡眠/设置/
 *        音乐) + 右侧背光滑条 + 底部手势条. 由 PageMem 页面管理器托管.
 *
 * Processing flow:
 *
 * pf_create : 建独立屏幕 + 4个按钮 + 背光滑条 + 底部手势条(本页**只有**它, 见下);
 * 手势      : 上滑→home(OUT_TOP);
 * 滑条      : 拖动→按差值降频发 EVT_SERVICE_BACKLIGHT_SET→APP 层写 PWM.
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
 *       ★当前 4 个按钮的 user_data 全是 NULL(占位)★ —— 原来的"心率"按钮已去掉,
 *       心率页改从表盘右滑进(见 lv_watch_page_home.c), 入口没断.
 *
 * @note ★滑条不直接写 PWM★: 背光接口在 adapter 层, 本页只发事件, 由 APP 层
 *       (cywatch_app_ui_menu.c, 跑在 appcore 上下文)去调 —— UI 页只碰对象树.
 *****************************************************************************/
#include "lv_watch_page.h"
#include "easyapp_port.h"

#include "system/log/cywatch_log.h" /* log_printf() */
#include <stdio.h>

/* 子集字库(实现在 LVGL/assets/fonts, 生成脚本 MDK-ARM/gen_watch_fonts.py):
   中文标签用14 —— 只含"运动心率睡眠音乐消息设置"这12个字.
   本轮去掉"运动/心率"两个标签, 剩下 4 个仍在这 12 字里, ★字库不用重生成★ */
LV_FONT_DECLARE(lv_font_alibaba_puhuiti_14)

/***********************************Defines************************************/
#define WATCH_BTN_CNT   4	/* 2列 x 2行, 占屏幕左侧 */
#define WATCH_BTN_W     80	/* 按钮宽 = 高, 保持正方形 */
#define WATCH_BTN_H     80	/* 按钮高 */
#define WATCH_STRIP_H   64	/* 底部手势条高度(在按钮下方) */
#define WATCH_STRIP_Y   216 /* 手势条Y: 第二行按钮底边(116+90)之下 */

/* 背光滑条(屏幕右侧, 垂直, 上=亮). ★下限取 WATCH_BL_MIN 而不是 0★: 拖到全黑时屏幕
   连滑条自己都看不见了, 只能盲拖回来; 而且这样"滑条位置 = 实际亮度"始终一致,
   不用再在写出去的地方钳一道 */
#define WATCH_BL_SLIDER_X   200 /* 右余区(按钮右边 176 .. 屏右 240)居中: 208 - 宽/2 */
#define WATCH_BL_SLIDER_Y   26
#define WATCH_BL_SLIDER_W   16
#define WATCH_BL_SLIDER_H   168
#define WATCH_BL_MIN        10  /* 最低亮度(%), 对应屏幕最暗 */
#define WATCH_BL_MAX        100 /* 最高亮度(%) */
#define WATCH_BL_INIT       100 /* 初值: 与 lv_port_disp 上电点亮背光用的值一致 */

/* 拖动时的事件降频: 亮度变化满这么多才发一次. ★不降频会把事件环灌满★ ——
   LVGL 的 VALUE_CHANGED 是逐像素级的, 一次拖动上百个; appcore 每 10ms 才跑一轮,
   根本消费不完. 松手时另补发最终值, 保证松手后滑条与亮度一致 */
#define WATCH_BL_SEND_STEP  5
/***********************************Defines************************************/

/**********************************Declaring***********************************/
/* 菜单页: 全是静态按钮, 没有动态内容 —— 所以结构体里除了基类什么都没有 */
typedef struct
{
	page_base_t base;
} watch_menu_t;

static void watch_menu_swipe_cb(lv_event_t *e);
static void watch_menu_btn_cb(lv_event_t *e);
static void watch_menu_bl_cb(lv_event_t *e);

static void watch_menu_create(page_base_t *p_page);
/**********************************Declaring***********************************/

/*********************************Static Data**********************************/
static watch_menu_t s_menu;

/* 背光滑条的当前值(页面自己的副本). ★要跨 destroy 保留★: 页面被 LRU 淘汰重建时
   靠它把滑条摆回原位, 否则会显示成初值而背光其实是别的值 */
static uint8_t s_bl_level = WATCH_BL_INIT;

/* 上一次真正发出去的值(只给降频算差值用). 与 s_bl_level 的区别: 拖动中滑条值一直
   在变, 但不是每次变化都值得发一个事件 */
static uint8_t s_bl_sent = WATCH_BL_INIT;

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
 *
 * @note    当前 4 个按钮的 user_data 全是 NULL, 所以本回调实际什么都不做 ——
 *          留着是给"把某个按钮绑到某一页"用的口
 *****************************************************************************/
static void watch_menu_btn_cb(lv_event_t *e)
{
	uint16_t page_id = (uint16_t)(uintptr_t)lv_event_get_user_data(e);

	if (0u != page_id)
	{
		watch_switch(page_id, LV_SCR_LOAD_ANIM_MOVE_RIGHT);

		/* 通知 APP 层: 3 = 往右切页(与表盘右滑同一个旗标值).
		   ★旗标必须与**目标页**一致而不是与"哪个按钮"一致★ —— 将来给按钮绑页面时,
		   这里要按目标页改值. 占位按钮(user_data 为 NULL)什么都不做 */
		x_port_easyapp_event_send(EVT_SERVICE_UIHOME_SWITCH_PAGE, 3, NULL);
	}
}

/******************************************************************************
 * @name    watch_menu_bl_cb
 * @brief   背光滑条: 拖动时按差值降频发事件, 松手时补发最终值
 * @param   e[in] LVGL事件(VALUE_CHANGED / RELEASED)
 *
 * @return  无
 *
 * @note    两种情况都不直接写硬件, 只发 EVT_SERVICE_BACKLIGHT_SET(亮度在
 *          event_flags) 交给 APP 层 —— 见文件头
 *****************************************************************************/
static void watch_menu_bl_cb(lv_event_t *e)
{
	lv_event_code_t code = lv_event_get_code(e);
	int32_t v = lv_slider_get_value(lv_event_get_target(e));
	int32_t diff = v - (int32_t)s_bl_sent;
	uint8_t send = 0U;

	s_bl_level = (uint8_t)v;

	if (LV_EVENT_VALUE_CHANGED == code)
	{
		/* 差值够大才发, 见 WATCH_BL_SEND_STEP */
		if ((diff >= WATCH_BL_SEND_STEP) || (-diff >= WATCH_BL_SEND_STEP))
		{
			send = 1U;
		}
	}
	else /* LV_EVENT_RELEASED */
	{
		/* 松手补发: 攒着没发的最后一段也得落到硬件, 否则松手后亮度与滑条对不上 */
		if (0 != diff)
		{
			send = 1U;
		}
	}

	if (0U != send)
	{
		s_bl_sent = (uint8_t)v;
		x_port_easyapp_event_send(EVT_SERVICE_BACKLIGHT_SET, (uint32_t)v, NULL);
	}
}

/******************************************************************************
 * @name    watch_menu_create
 * @brief   建本页: 左侧 2x2 功能按钮 + 右侧背光滑条 + 底部手势条
 * @param   p_page[in] 页面基类指针
 *
 * @return  无
 *
 * @note    4 个按钮当前全是占位(不带目标页 ID); 中文标签只用到那 12 字子集字库
 * @note    滑条方向: LVGL 垂直 slider 是"下小上大", 所以上=亮 下=暗
 *****************************************************************************/
static void watch_menu_create(page_base_t *p_page)
{
	static const char *const label[WATCH_BTN_CNT] =
		{"消息", "睡眠", "设置", "音乐"};
	static const char *const sym[WATCH_BTN_CNT] =
		{LV_SYMBOL_LIST, LV_SYMBOL_PAUSE, LV_SYMBOL_SETTINGS, LV_SYMBOL_AUDIO};
	static const uint32_t color[WATCH_BTN_CNT] =
		{0x16A34A, 0x6D28D9, 0x64748B, 0xDB2777};
	static const lv_coord_t btn_x[2] = {8, 96};    /* 列距 = 宽80 + 间隙8 */
	static const lv_coord_t btn_y[2] = {24, 116};  /* 行距 92, 使按钮组与滑条竖直对齐(中心均 110) */
	lv_obj_t *scr = lv_obj_create(NULL);
	lv_obj_t *btn = NULL;
	lv_obj_t *icon = NULL;
	lv_obj_t *lbl = NULL;
	lv_obj_t *slider = NULL;
	uint32_t i = 0;

	lv_obj_set_style_bg_color(scr, lv_color_hex(WATCH_COL_DARK), 0);

	for (i = 0; i < WATCH_BTN_CNT; i++)
	{
		btn = lv_button_create(scr);
		lv_obj_set_pos(btn, btn_x[i % 2U], btn_y[i / 2U]);
		lv_obj_set_size(btn, WATCH_BTN_W, WATCH_BTN_H);
		lv_obj_set_style_bg_color(btn, lv_color_hex(color[i]), 0);
		lv_obj_set_style_radius(btn, 18, 0);
		lv_obj_set_style_pad_all(btn, 0, 0);

		icon = lv_label_create(btn);
		lv_label_set_text(icon, sym[i]);
		lv_obj_set_style_text_color(icon, lv_color_hex(WATCH_COL_TEXT), 0);
		lv_obj_align(icon, LV_ALIGN_TOP_MID, 0, 18);

		lbl = lv_label_create(btn);
		lv_label_set_text(lbl, label[i]);
		lv_obj_set_style_text_font(lbl, &lv_font_alibaba_puhuiti_14, 0);
		lv_obj_set_style_text_color(lbl, lv_color_hex(WATCH_COL_TEXT), 0);
		lv_obj_align(lbl, LV_ALIGN_TOP_MID, 0, 48);

		lv_obj_add_event_cb(btn, watch_menu_btn_cb, LV_EVENT_CLICKED, NULL);
	}

	/* 背光滑条(右半屏). 槽用深灰、已填充部分与旋钮用白, 与左边 4 个彩色方块分开 */
	slider = lv_slider_create(scr);
	lv_obj_set_pos(slider, WATCH_BL_SLIDER_X, WATCH_BL_SLIDER_Y);
	lv_obj_set_size(slider, WATCH_BL_SLIDER_W, WATCH_BL_SLIDER_H);
	lv_slider_set_orientation(slider, LV_SLIDER_ORIENTATION_VERTICAL);
	lv_slider_set_range(slider, WATCH_BL_MIN, WATCH_BL_MAX);
	lv_slider_set_value(slider, (int32_t)s_bl_level, LV_ANIM_OFF);
	lv_obj_set_style_bg_color(slider, lv_color_hex(0x2A2A2A), LV_PART_MAIN);
	lv_obj_set_style_bg_color(slider, lv_color_hex(WATCH_COL_TEXT), LV_PART_INDICATOR);
	lv_obj_set_style_bg_color(slider, lv_color_hex(WATCH_COL_TEXT), LV_PART_KNOB);
	lv_obj_set_style_width(slider, 20, LV_PART_KNOB);
	lv_obj_set_style_height(slider, 20, LV_PART_KNOB);
	lv_obj_add_event_cb(slider, watch_menu_bl_cb, LV_EVENT_VALUE_CHANGED, NULL);
	lv_obj_add_event_cb(slider, watch_menu_bl_cb, LV_EVENT_RELEASED, NULL);

	/* 底部手势条(按钮下方空白区, 不挡按钮): 上滑回表盘 */
	watch_strip_create(scr, WATCH_SCREEN_W, WATCH_STRIP_H, 0, WATCH_STRIP_Y, watch_menu_swipe_cb);

	p_page->obj = scr;
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
