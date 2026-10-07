/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file lv_watch_page_home.c
 *
 * @par dependencies
 * - cywatch_log.h
 * - lv_watch_page.h
 * - lvgl.h
 * - ../../system/Rtc/cywatch_rtc.h (cywatch_rtc_time_t, 经 watch_model_datetime 取时间)
 *
 * @author zw1194
 *
 * @brief 手表UI - 表盘页(home, ID 0x0300): 右上角电量区(自绘电池 + 百分比 + 充电
 *        闪电) + 左上角蓝牙标识(自绘) + 日期 + 大字时钟 + 三张健康卡片(步数/心率/
 *        温度) + 全屏手势条. 由 PageMem 页面管理器托管.
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
 * @note 本页只"把数据画出来", **不推进**数据. 数据有两路:
 *       1) **日期与大字时钟**来自硬件 RTC, 经 watch_model_datetime() 每次渲染真读
 *          一次外设(所以它一直走, 与页面在不在屏上无关);
 *       2) **三张卡片**是本页自己持有的值(下面的 step/hr/temp), 由
 *          watch_page_home_set_data() 从别的任务写, 见 lv_watch_page_home.h.
 *       值存在页面结构体里(s_home 是静态的, create/destroy 都不清零),
 *       所以本页被 LRU 淘汰重建后接着画的是最新值, 而不是从初值重来.
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
#include "lv_watch_page_home.h"
#include "easyapp_port.h"
#include "cywatch_rtc.h"
#include "system/log/cywatch_log.h" /* log_printf() */
#include <stdio.h>

/* 子集字库(实现在 LVGL/assets/fonts, 生成脚本 MDK-ARM/gen_watch_fonts.py):
   时钟用20, 卡片数值/日期用16 —— 两者都只需 ASCII */
LV_FONT_DECLARE(lv_font_montserrat_regular_20)
LV_FONT_DECLARE(lv_font_montserrat_regular_16)

/***********************************Defines************************************/
#define WATCH_CARD_CNT      3		 /* 卡片数: 步数/心率/温度 */
#define WATCH_COL_PAGE_BG   0x000000 /* 表盘底色: 纯黑(菜单/心率页用 WATCH_COL_DARK) */
#define WATCH_COL_CARD_BG   0x1A1A1A /* 卡片底 */
#define WATCH_COL_GRN       0x4CD964 /* 电池图标绿 */
#define WATCH_COL_BLUE      0x2196F3 /* 蓝牙已连接的蓝 */

/* 电量区(右上角自绘, 不用图片素材). 绝对坐标, 整簇关于屏幕中轴 x=120 镜像自原来的
   左侧布局, ★顺序也反★: 从左到右 = 闪电 / 数值(右对齐) / 正极帽 / 外壳.
   ★横向和日期重叠是故意的★: 日期居中("2026-10-06" 实测 87px → 占 x 76..163), 全靠纵向
   错开 —— 数值字形底在 y17, 日期字形从 y27 起(日期 label 顶 y24), 余 10px */
#define WATCH_BAT_X         196		 /* 外壳左边缘(右边缘 216, 距屏幕右边 24px) */
#define WATCH_BAT_Y         6		 /* 外壳上边缘 */
#define WATCH_BAT_W         20		 /* 外壳宽 */
#define WATCH_BAT_H         12		 /* 外壳高 */
#define WATCH_BAT_IN_X      198		 /* 填充块左边缘(外壳内缩 1px 边框 + 1px 留白) */
#define WATCH_BAT_IN_Y      8		 /* 填充块上边缘 */
#define WATCH_BAT_IN_W      16		 /* 填充块满档宽 */
#define WATCH_BAT_IN_H      8		 /* 填充块高 */
#define WATCH_BAT_TIER_CNT  6		 /* 档位数: 100/80/60/40/20/0 */
#define WATCH_BAT_TXT_R     49		 /* 数值右边缘距屏幕右边的距离(右对齐, 宽度随内容变) */
#define WATCH_BAT_BOLT_R    92		 /* 闪电右边缘距屏幕右边的距离 */

/* 蓝牙标识(左上角自绘): 竖中线 + 上下两条折线, ★矩形拼不出斜线, 只能用 lv_line★.
   符号 6x12, 与右侧电池外壳(20x12)同高同中线(y 6..18), 左留白 24px 和电池的右留白
   关于中轴对称.
   ★框比符号大 1px★: lv_line 会被裁到对象区域, 不留余量线宽会把左右两个尖角削平 */
#define WATCH_BT_X          23		 /* 框左边缘(符号本身从 24 起) */
#define WATCH_BT_Y          5		 /* 框上边缘(符号本身从 6 起, 与电池外壳对齐) */
#define WATCH_BT_W          8		 /* 框宽 = 符号 6 + 左右各 1px 余量 */
#define WATCH_BT_H          14		 /* 框高 = 符号 12 + 上下各 1px */
#define WATCH_BT_LINE_W     1		 /* 线宽 */
/***********************************Defines************************************/

/**********************************Declaring***********************************/
/* 表盘页: 电量区 + 时钟 + 三张卡片 + 自己的1s刷新定时器 */
typedef struct
{
	page_base_t base;
	lv_obj_t *p_date;				   /* 日期 YYYY-MM-DD(硬件RTC) */
	lv_obj_t *p_clock;				  /* HH:MM:SS(硬件RTC) */
	lv_obj_t *p_card[WATCH_CARD_CNT]; /* 步数/心率/温度 数值 */
	lv_timer_t *p_timer;			  /* 本页刷新(pf_hide 暂停 / pf_show 恢复) */
	/* 电量区(左上角, 自绘): 外壳/正极常驻, 填充块宽度与闪电显隐随数据变 */
	lv_obj_t *p_bat_shell; /* 电池外壳(带边框的空壳) */
	lv_obj_t *p_bat_cap;   /* 正极小凸起 */
	lv_obj_t *p_bat_fill;  /* 内部填充块(宽度 = 档位) */
	lv_obj_t *p_bat_text;  /* 电量数值 "87%" */
	lv_obj_t *p_bat_bolt;  /* 充电闪电(只在充电中显示) */
	/* 蓝牙标识(左上角, 自绘): 符号颜色随状态变, 未连接时叠一道斜杠 */
	lv_obj_t *p_bt_sym;   /* 蓝牙符号(折线) */
	lv_obj_t *p_bt_slash; /* 未连接斜杠 */
	/* 本页的数据面: 由 watch_page_home_set_data() 从外部写, 本页渲染只读.
	   放这里而不是 DataModel 全局 —— 这是本页自己的数据面 */
	uint32_t step;	 /* 步数 */
	uint8_t  hr;	   /* 心率, bpm */
	int16_t  temp;	 /* 温度, 整数摄氏度 */
	uint8_t  battery;  /* 电量, 0~100 */
	uint8_t  charging; /* 1=充电中 */
	uint8_t  bt_conn;  /* 1=蓝牙已连接 */
} watch_home_t;

static void watch_home_render(watch_home_t *p_home);
static void watch_home_timer_cb(lv_timer_t *p_timer);

static uint8_t watch_home_bat_tier(uint8_t pct);

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

/* 六档的下限(含): 0/10/30/50/70/90 → 对应 0/20/40/60/80/100% 六张图 */
static const uint8_t s_bat_tier_min[WATCH_BAT_TIER_CNT] =
	{0U, 10U, 30U, 50U, 70U, 90U};

/* 蓝牙符号(6x12, 整体在框内偏移 1px 让开线宽): 左中→右下→底中→顶中→右中→左下,
   即"竖中线 + 上下两条折线"。形状取自 Feather 的 bluetooth 图标(11x22 等比缩到 6x12) */
static const lv_point_precise_t s_bt_pts[] =
	{{1, 4}, {7, 10}, {4, 13}, {4, 1}, {7, 4}, {1, 10}};

/* 未连接时叠上去的斜杠: 框内左上到右下 */
static const lv_point_precise_t s_bt_slash_pts[] =
	{{1, 1}, {7, 13}};

/******************************************************************************
 * @name    watch_home_bat_tier
 * @brief   电量百分比 → 图标档位下标
 * @param   pct[in] 电量百分比(0~100)
 *
 * @return  0 ~ WATCH_BAT_TIER_CNT-1
 *
 * @note    边界取两档中点(10/30/50/70/90), 所以 87% 落在 80% 档而不是 100% 档
 *****************************************************************************/
static uint8_t watch_home_bat_tier(uint8_t pct)
{
	uint8_t tier = 0U;

	for (tier = 0U; tier + 1U < WATCH_BAT_TIER_CNT; tier++)
	{
		if (pct < s_bat_tier_min[tier + 1U])
		{
			break;
		}
	}

	return tier;
}

/******************************************************************************
 * @name    watch_home_render
 * @brief   把模型数据刷到本页控件上(时钟 + 三张卡片 + 电量区)
 * @param   p_home[in] 表盘页
 *
 * @return  无
 *
 * @note    控件指针可能为 NULL(页面已销毁), 每处都判一下; 本函数只读本页自己那份值
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
		snprintf(buf, sizeof(buf), "%lu", (unsigned long)p_home->step);
		lv_label_set_text(p_home->p_card[0], buf);
	}
	if (NULL != p_home->p_card[1])
	{
		snprintf(buf, sizeof(buf), "%u", (unsigned)p_home->hr);
		lv_label_set_text(p_home->p_card[1], buf);
	}
	if (NULL != p_home->p_card[2])
	{
		snprintf(buf, sizeof(buf), "%d", (int)p_home->temp);
		lv_label_set_text(p_home->p_card[2], buf);
	}

	/* 电量区: 数值 + 填充块宽度(按档位)+ 充电闪电显隐 */
	if (NULL != p_home->p_bat_text)
	{
		snprintf(buf, sizeof(buf), "%u%%", (unsigned)p_home->battery);
		lv_label_set_text(p_home->p_bat_text, buf);
	}
	if (NULL != p_home->p_bat_fill)
	{
		lv_obj_set_width(p_home->p_bat_fill,
						 (lv_coord_t)((uint32_t)WATCH_BAT_IN_W *
									  watch_home_bat_tier(p_home->battery) /
									  (WATCH_BAT_TIER_CNT - 1U)));
	}
	if (NULL != p_home->p_bat_bolt)
	{
		if (0U != p_home->charging)
		{
			lv_obj_remove_flag(p_home->p_bat_bolt, LV_OBJ_FLAG_HIDDEN);
		}
		else
		{
			lv_obj_add_flag(p_home->p_bat_bolt, LV_OBJ_FLAG_HIDDEN);
		}
	}

	/* 蓝牙标识: 已连接=蓝色符号 / 未连接=灰色符号 + 一道斜杠 */
	if (NULL != p_home->p_bt_sym)
	{
		lv_obj_set_style_line_color(p_home->p_bt_sym,
									lv_color_hex(0U != p_home->bt_conn ?
												 WATCH_COL_BLUE : WATCH_COL_GRAY), 0);
	}
	if (NULL != p_home->p_bt_slash)
	{
		if (0U != p_home->bt_conn)
		{
			lv_obj_add_flag(p_home->p_bt_slash, LV_OBJ_FLAG_HIDDEN);
		}
		else
		{
			lv_obj_remove_flag(p_home->p_bt_slash, LV_OBJ_FLAG_HIDDEN);
		}
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
 * @brief   建本页: 电量区/日期/大字时钟/三张卡片/手势条 + 刷新定时器
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
	lv_obj_t *lbl = NULL;
	lv_obj_t *card = NULL;
	uint32_t i = 0;

	/* 三张卡片: 标题(英文, 避开缺字形) / 18x18图标图 / X坐标 */
	static const char *const title[WATCH_CARD_CNT] = {"STEPS", "HR", "TEMP"};
	static const lv_image_dsc_t *const card_icon[WATCH_CARD_CNT] =
		{&icon_steps_18x18_RGB565A8_NONE, &icon_heart_18x18_RGB565A8_NONE, &icon_flame_18x18_RGB565A8_NONE};
	static const lv_coord_t card_x[WATCH_CARD_CNT] = {3, 83, 163};

	lv_obj_set_style_bg_color(scr, lv_color_hex(WATCH_COL_PAGE_BG), 0);

	/* ---- 电量区(右上角; 整簇镜像自左侧, ★顺序也反★ → 闪电在最左, 电池在最右) ----
	   电池是**自绘**的: 外壳/正极/填充块都是空 lv_obj, 全靠 style 画, 不占图片素材,
	   也不像内置 BATTERY_x 符号那样只有 5 档(它凑不齐 100/80/60/40/20/0 这 6 档) */
	p_home->p_bat_shell = lv_obj_create(scr);
	lv_obj_set_pos(p_home->p_bat_shell, WATCH_BAT_X, WATCH_BAT_Y);
	lv_obj_set_size(p_home->p_bat_shell, WATCH_BAT_W, WATCH_BAT_H);
	lv_obj_set_style_bg_opa(p_home->p_bat_shell, LV_OPA_TRANSP, 0);
	lv_obj_set_style_border_width(p_home->p_bat_shell, 1, 0);
	lv_obj_set_style_border_color(p_home->p_bat_shell, lv_color_hex(WATCH_COL_GRN), 0);
	lv_obj_set_style_radius(p_home->p_bat_shell, 2, 0);
	lv_obj_set_style_pad_all(p_home->p_bat_shell, 0, 0);

	/* 正极(镜像后在**外壳左侧**的小凸起 —— 左右跟着一起翻) */
	p_home->p_bat_cap = lv_obj_create(scr);
	lv_obj_set_pos(p_home->p_bat_cap, WATCH_BAT_X - 2, WATCH_BAT_Y + 4);
	lv_obj_set_size(p_home->p_bat_cap, 2, 4);
	lv_obj_set_style_bg_color(p_home->p_bat_cap, lv_color_hex(WATCH_COL_GRN), 0);
	lv_obj_set_style_border_width(p_home->p_bat_cap, 0, 0);
	lv_obj_set_style_radius(p_home->p_bat_cap, 1, 0);
	lv_obj_set_style_pad_all(p_home->p_bat_cap, 0, 0);

	/* 填充块: 宽度由 watch_home_render 按档位改 */
	p_home->p_bat_fill = lv_obj_create(scr);
	lv_obj_set_pos(p_home->p_bat_fill, WATCH_BAT_IN_X, WATCH_BAT_IN_Y);
	lv_obj_set_size(p_home->p_bat_fill, WATCH_BAT_IN_W, WATCH_BAT_IN_H);
	lv_obj_set_style_bg_color(p_home->p_bat_fill, lv_color_hex(WATCH_COL_GRN), 0);
	lv_obj_set_style_border_width(p_home->p_bat_fill, 0, 0);
	lv_obj_set_style_radius(p_home->p_bat_fill, 1, 0);
	lv_obj_set_style_pad_all(p_home->p_bat_fill, 0, 0);

	/* 电量数值(不用子集字库: 数值短, 默认 14 号字与 12px 高的电池图标才配).
	   镜像后数值在图标**左边**, 所以改成右对齐 —— 宽度随内容变("0%" 与 "100%" 差 20px),
	   右边缘固定才不会让数字和电池之间的缝忽宽忽窄 */
	p_home->p_bat_text = lv_label_create(scr);
	lv_label_set_text(p_home->p_bat_text, "0%");
	lv_obj_set_style_text_color(p_home->p_bat_text, lv_color_hex(WATCH_COL_TEXT), 0);
	lv_obj_align(p_home->p_bat_text, LV_ALIGN_TOP_RIGHT,
				 -(lv_coord_t)WATCH_BAT_TXT_R, 4);

	/* 充电闪电(镜像后在整簇最左): LV_SYMBOL_CHARGE 只在默认字体里有 —— 本页那两个
	   子集字库(montserrat_regular_16/20)不含符号区, 所以这里**不能**设 text_font.
	   没充电时隐藏; 布局是绝对坐标, 隐藏不会挤动别的控件 */
	p_home->p_bat_bolt = lv_label_create(scr);
	lv_label_set_text(p_home->p_bat_bolt, LV_SYMBOL_CHARGE);
	lv_obj_set_style_text_color(p_home->p_bat_bolt, lv_color_hex(WATCH_COL_GRN), 0);
	lv_obj_align(p_home->p_bat_bolt, LV_ALIGN_TOP_RIGHT,
				 -(lv_coord_t)WATCH_BAT_BOLT_R, 5);
	lv_obj_add_flag(p_home->p_bat_bolt, LV_OBJ_FLAG_HIDDEN);

	/* ---- 蓝牙标识(左上角, 自绘) ----
	   符号是折线, ★只能用 lv_line★(矩形拼不出斜线, 斜率 2:1 也不是 45°).
	   点数组必须是 static: lv_line_set_points 只存指针、不拷贝点 */
	p_home->p_bt_sym = lv_line_create(scr);
	lv_obj_remove_style_all(p_home->p_bt_sym); /* 主题默认的 bg/border/pad 会挡住 line */
	lv_obj_set_pos(p_home->p_bt_sym, WATCH_BT_X, WATCH_BT_Y);
	lv_obj_set_size(p_home->p_bt_sym, WATCH_BT_W, WATCH_BT_H);
	lv_line_set_points(p_home->p_bt_sym, s_bt_pts, 6);
	lv_obj_set_style_line_width(p_home->p_bt_sym, WATCH_BT_LINE_W, 0);
	lv_obj_set_style_line_rounded(p_home->p_bt_sym, true, 0);
	/* 初值是"未连接", 所以起始色给灰(与下面那道斜杠一致), 别等 render 来纠正 */
	lv_obj_set_style_line_color(p_home->p_bt_sym, lv_color_hex(WATCH_COL_GRAY), 0);

	/* 未连接时叠在符号上的斜杠(已连接时隐藏; 初始值未连接, 所以就建出来可见) */
	p_home->p_bt_slash = lv_line_create(scr);
	lv_obj_remove_style_all(p_home->p_bt_slash);
	lv_obj_set_pos(p_home->p_bt_slash, WATCH_BT_X, WATCH_BT_Y);
	lv_obj_set_size(p_home->p_bt_slash, WATCH_BT_W, WATCH_BT_H);
	lv_line_set_points(p_home->p_bt_slash, s_bt_slash_pts, 2);
	lv_obj_set_style_line_width(p_home->p_bt_slash, WATCH_BT_LINE_W, 0);
	lv_obj_set_style_line_color(p_home->p_bt_slash, lv_color_hex(WATCH_COL_GRAY), 0);

	/* 日期(随时钟一起每秒刷新, 所以句柄必须存下来, 不能像以前那样用完就丢) */
	p_home->p_date = lv_label_create(scr);
	lv_label_set_text(p_home->p_date, "-- -- --");
	lv_obj_set_style_text_font(p_home->p_date, &lv_font_montserrat_regular_16, 0);
	lv_obj_set_style_text_color(p_home->p_date, lv_color_hex(WATCH_COL_GRAY), 0);
	lv_obj_align(p_home->p_date, LV_ALIGN_TOP_MID, 0, 24);

	/* 大字时钟(每秒刷新) */
	p_home->p_clock = lv_label_create(scr);
	lv_label_set_text(p_home->p_clock, "00:00:00");
	lv_obj_set_style_text_font(p_home->p_clock, &lv_font_montserrat_regular_20, 0);
	lv_obj_set_style_text_color(p_home->p_clock, lv_color_hex(WATCH_COL_TEXT), 0);
	lv_obj_set_style_text_align(p_home->p_clock, LV_TEXT_ALIGN_CENTER, 0);
	lv_obj_set_width(p_home->p_clock, 200);
	lv_obj_align(p_home->p_clock, LV_ALIGN_TOP_MID, 0, 60);

	/* 三张健康卡片(步数/心率/温度) */
	for (i = 0; i < WATCH_CARD_CNT; i++)
	{
		card = lv_obj_create(scr);
		lv_obj_set_pos(card, card_x[i], 196);
		lv_obj_set_size(card, 74, 76);
		lv_obj_set_style_bg_color(card, lv_color_hex(WATCH_COL_CARD_BG), 0);
		lv_obj_set_style_radius(card, 12, 0);
		lv_obj_set_style_border_width(card, 0, 0);
		lv_obj_set_style_pad_all(card, 0, 0);

		/* 卡图标: 步数/心率/温度 (18x18图片; 温度暂无温度计图标, 暂沿用火焰) */
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
	p_home->p_bat_shell = NULL;
	p_home->p_bat_cap = NULL;
	p_home->p_bat_fill = NULL;
	p_home->p_bat_text = NULL;
	p_home->p_bat_bolt = NULL;
	p_home->p_bt_sym = NULL;
	p_home->p_bt_slash = NULL;
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

/******************************************************************************
 * @name    watch_page_home_set_data
 * @brief   写本页某一项(异步; 接口说明见 lv_watch_page_home.h)
 * @param   which[in] WATCH_HOME_DATA_STEP / _HR / _TEMP / _BATTERY / _CHARGING / _BT
 * @param   value[in] 新值
 *
 * @return  无
 *
 * @note    只写值, 不碰控件 —— 控件只在 lvgl 任务里动, 本函数可由别的任务调.
 *          下一次本页 1s 定时器(或 pf_show)才把它画上屏
 *****************************************************************************/
void watch_page_home_set_data(watch_home_data_e which, int32_t value)
{
	switch (which)
	{
	case WATCH_HOME_DATA_STEP:
		s_home.step = (uint32_t)value;
		break;
	case WATCH_HOME_DATA_HR:
		s_home.hr = (uint8_t)value;
		break;
	case WATCH_HOME_DATA_TEMP:
		s_home.temp = (int16_t)value;
		break;
	case WATCH_HOME_DATA_BATTERY:
		s_home.battery = (uint8_t)value;
		break;
	case WATCH_HOME_DATA_CHARGING:
		s_home.charging = (uint8_t)(0 != value);
		break;
	case WATCH_HOME_DATA_BT:
		s_home.bt_conn = (uint8_t)(0 != value);
		break;
	default:
		break;
	}
}
