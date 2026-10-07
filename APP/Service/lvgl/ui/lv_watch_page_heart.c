/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file lv_watch_page_heart.c
 *
 * @par dependencies
 * - cywatch_log.h
 * - lv_watch_page.h
 * - lvgl.h
 *
 * @author zw1194
 *
 * @brief 手表UI - 心率页(heart, ID 0x0302): 标题 + 大字心率值 + BPM + 30点折线图
 *        + 全屏手势条. 由 PageMem 页面管理器托管.
 *
 * Processing flow:
 *
 * pf_create : 建独立屏幕 + 控件 + 本页1s刷新定时器(首次刷新就把 hist 整份画上去);
 * pf_show   : 立即补一帧(推一个图点) + 恢复定时器;
 * pf_hide   : 暂停定时器(隐藏的缓存页不再每秒推图点);
 * pf_destroy: lv_timer_delete + 句柄置 NULL;
 * 手势      : 左滑→home(MOVE_LEFT).
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 本页只"把数据画出来", **不推进**数据 —— 心率值与图表历史都由 APP 层经
 *       watch_page_heart_set_hr() / watch_page_heart_push_hr() 投进来(见
 *       lv_watch_page_heart.h), 本页只读.
 *
 * @note ★折线图的历史存在本页结构体里(hist[]), 不在 chart 里★ —— PageMem 只缓存
 *       2 页(PAGE_MGR_BUF_SIZE), 绕几个页面再回来时本页会被淘汰重建, chart 连同它
 *       的数据缓冲一起被删. 所以 hist 才是真源、chart 只是显示副本, 每次重画都从
 *       hist 重建. 不这么做的话重建后**整条线是空的** —— 新 chart 的点全是
 *       LV_CHART_POINT_NONE, 画线时被跳过, 要重新攒 30 秒才长回来.
 *
 * @note 存进 hist 的那一份是给 APP 层常驻记录器用的, 所以即使本页没在显示(被淘汰、
 *       甚至从未创建过), 历史也一直在长.
 *
 * @note 图表的数据缓冲由 chart 自己持有, 随 chart 一起被 LVGL 删掉; pf_destroy
 *       里只清句柄, **不要**自己 lv_obj_delete(p_chart) —— 已在递归删除中.
 ******************************************************************************/
#include "lv_watch_page.h"
#include "lv_watch_page_heart.h"
#include "easyapp_port.h"

#include "system/log/cywatch_log.h" /* log_printf() */
#include <stdio.h>

/* 子集字库(实现在 LVGL/assets/fonts, 生成脚本 MDK-ARM/gen_watch_fonts.py):
   数值用20(只需 ASCII), 标题用14(汉字) */
LV_FONT_DECLARE(lv_font_montserrat_regular_20)
LV_FONT_DECLARE(lv_font_alibaba_puhuiti_14)

/***********************************Defines************************************/
#define WATCH_HR_POINTS     30		/* 折线图点数(约30s的可视窗口) */
#define WATCH_HR_MAX        150		/* Y轴上限(BPM) */
#define WATCH_COL_HR_RED    0xE03030 /* 心率线的红(与血氧页按钮同色) */
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
	/* 本页的数据面: 由 watch_page_heart_set_hr() 从外部写, 本页渲染只读.
	   放这里而不是 lv_watch_ui.c 的模型 —— 这是本页自己的数据面 */
	uint8_t hr;					  /* 心率, bpm; 0 = 没贴手指 */

	/* ★图的真源在这儿, 不在 chart 里★ —— chart 只是个"显示副本", PageMem 淘汰本页时
	   连它的数据缓冲一起删掉, 所以历史必须落在页结构体(文件级 static, 淘汰擦不掉).
	   由 APP 层的常驻记录器每秒 push 一格(见 watch_page_heart_push_hr), 本页只读不推进 */
	uint8_t hist[WATCH_HR_POINTS]; /* 线性: hist[0] 最旧, hist[hist_cnt-1] 最新 */
	uint8_t hist_cnt;			   /* 已写入格数, 封顶 WATCH_HR_POINTS */
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
 * @brief   把本页数据面的值刷到控件上(数值 + 图表推进一个点)
 * @param   p_heart[in] 心率页
 *
 * @return  无
 *****************************************************************************/
static void watch_heart_render(watch_heart_t *p_heart)
{
	char buf[16];
	uint8_t i = 0U;

	if (NULL == p_heart)
	{
		return;
	}

	if (NULL != p_heart->p_val)
	{
		snprintf(buf, sizeof(buf), "%u", (unsigned)p_heart->hr);
		lv_label_set_text(p_heart->p_val, buf);
	}

	if (NULL != p_heart->p_chart && NULL != p_heart->p_series)
	{
		/* ★整体重画, 不是推单点★: set_all_values() 顺手把 chart 自己的环形游标
		   (start_point) 归零(见 lv_chart.c), 所以"清零 + 按时间顺序回放"是幂等的 ——
		   画出来永远是"hist 里那 cnt 格, 最旧在左, 最新在右". 于是本页的 1s 节拍和记录器
		   的 1Hz **不必对齐**: 差半拍、两次重画之间来了两格、一格都没来, 下一次重画都自动
		   纠正. 被淘汰重建走的也是这条路径, 所以不必另写一段"回放"代码 */
		lv_chart_set_all_values(p_heart->p_chart, p_heart->p_series, LV_CHART_POINT_NONE);

		for (i = 0U; i < p_heart->hist_cnt; i++)
		{
			lv_chart_set_next_value(p_heart->p_chart, p_heart->p_series, p_heart->hist[i]);
		}
	}
}

/******************************************************************************
 * @name    watch_heart_timer_cb
 * @brief   本页刷新定时器: 每秒把大字与整张图重画一遍(图的数据由记录器推进)
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
	lv_chart_cursor_t *p_axis = NULL;

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

	/* 30点折线图 */
	p_heart->p_chart = lv_chart_create(scr);
	lv_obj_set_size(p_heart->p_chart, 216, 128);
	lv_obj_align(p_heart->p_chart, LV_ALIGN_TOP_MID, 0, 112);
	lv_chart_set_type(p_heart->p_chart, LV_CHART_TYPE_LINE);
	lv_chart_set_point_count(p_heart->p_chart, WATCH_HR_POINTS);
	lv_chart_set_range(p_heart->p_chart, LV_CHART_AXIS_PRIMARY_X, 0, WATCH_HR_POINTS - 1);
	lv_chart_set_range(p_heart->p_chart, LV_CHART_AXIS_PRIMARY_Y, 0, WATCH_HR_MAX);
	/* 横向网格: 每 50bpm 一条 → 0/50/100/150 共4条.
	   ★这个 4 是"跨度/50+1"算出来的, 改 WATCH_HR_MAX 必须重算★否则网格落在非整刻度上 */
	lv_chart_set_div_line_count(p_heart->p_chart, 4, 0);

	/* 纵向只要原点一条竖线. ★div 线做不到单条★(它按两端等分, 最少 2 条 = 左右边缘),
	   改用 chart 自带的 cursor: LV_DIR_VER 让 y 自动取控件上下边, x 要加上左内边距才落在
	   数据原点; 颜色跟着网格走, 线宽/尺寸要压 —— 主题给 cursor 的是 3px 线 + 一个光标点 */
	p_axis = lv_chart_add_cursor(p_heart->p_chart,
								 lv_obj_get_style_line_color(p_heart->p_chart, 0), LV_DIR_VER);
	lv_obj_set_style_size(p_heart->p_chart, 0, 0, LV_PART_CURSOR);
	lv_obj_set_style_line_width(p_heart->p_chart, 1, LV_PART_CURSOR);
	lv_chart_set_cursor_pos_x(p_heart->p_chart, p_axis,
							   lv_obj_get_style_pad_left(p_heart->p_chart, 0));
	lv_obj_set_style_bg_color(p_heart->p_chart, lv_color_hex(WATCH_COL_CHART_BG), 0);
	lv_obj_set_style_border_width(p_heart->p_chart, 0, 0);
	lv_obj_set_style_radius(p_heart->p_chart, 10, 0);

	p_heart->p_series = lv_chart_add_series(p_heart->p_chart,
											lv_color_hex(WATCH_COL_HR_RED),
											LV_CHART_AXIS_PRIMARY_Y);
	lv_obj_set_style_line_width(p_heart->p_chart, 2, LV_PART_ITEMS);

	/* 折点: 主题默认给的是 8x8 圆点, 会完全盖住上面那条线 */
	lv_obj_set_style_size(p_heart->p_chart, 4, 4, LV_PART_INDICATOR);

	/* 全屏手势条(页面无点击内容, 整屏可滑): 左滑回表盘 */
	watch_strip_create(scr, WATCH_SCREEN_W, WATCH_SCREEN_H, 0, 0, watch_heart_swipe_cb);

	/* 本页刷新定时器 */
	p_heart->p_timer = lv_timer_create(watch_heart_timer_cb, WATCH_TICK_MS, NULL);

	p_page->obj = scr;
	watch_heart_render(p_heart);
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

	/* chart 自己的数据缓冲随 chart 一起被 LVGL 删掉, 这里只清句柄.
	   ★hist / hist_cnt 不能跟着清★ —— 它们是本页图的真源, 清了下一次 pf_create 就
	   画不出历史, 而且 APP 层的记录器还在往里 push */
	p_heart->p_val = NULL;
	p_heart->p_chart = NULL;
	p_heart->p_series = NULL;
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

/******************************************************************************
 * @name    watch_page_heart_set_hr
 * @brief   写本页心率值(异步; 接口说明见 lv_watch_page_heart.h)
 * @param   bpm[in] 心率; 0 = 没有有效读数(没贴手指)
 *
 * @return  无
 *
 * @note    只写值, 不碰控件 —— 控件只在 lvgl 任务里动, 本函数可由别的任务调.
 *          下一次本页 1s 定时器(或 pf_show)才把它画上屏
 *****************************************************************************/
void watch_page_heart_set_hr(uint8_t bpm)
{
	s_heart.hr = bpm;
}

/******************************************************************************
 * @name    watch_page_heart_push_hr
 * @brief   往本页图表历史末尾追加一格(异步; 接口说明见 lv_watch_page_heart.h)
 * @param   bpm[in] 心率; 没贴手指时传 0
 *
 * @return  无
 *
 * @note    只写 hist, 不碰控件 —— 控件只在 lvgl 任务里动, 本函数由 APP 层常驻记录器
 *          在 appcore 任务里调, 所以本页没显示时历史照样在长
 * @note    ★超上限的值在这里钳到顶边★: 不钳的话 LVGL 把线裁在框外, 顶层看着像被
 *          切断. 只钳图线 —— 大字走 watch_page_heart_set_hr(), 不经过这里, 显示真实值
 *****************************************************************************/
void watch_page_heart_push_hr(uint8_t bpm)
{
	uint8_t v = (bpm > WATCH_HR_MAX) ? (uint8_t)WATCH_HR_MAX : bpm;
	uint8_t i = 0U;

	if (s_heart.hist_cnt < WATCH_HR_POINTS)
	{
		/* 还没攒满: 直接追加在队尾, 不用移位 */
		s_heart.hist[s_heart.hist_cnt] = v;
		s_heart.hist_cnt++;
	}
	else
	{
		/* 满了: 左移一格腾出队尾再追加. 30 字节, 每秒一次, 开销可以忽略 */
		for (i = 0U; i + 1U < WATCH_HR_POINTS; i++)
		{
			s_heart.hist[i] = s_heart.hist[i + 1U];
		}
		s_heart.hist[WATCH_HR_POINTS - 1U] = v;
	}
}
