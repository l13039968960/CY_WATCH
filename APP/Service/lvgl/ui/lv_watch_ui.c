/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file lv_watch_ui.c
 *
 * @par dependencies
 * - cywatch_log.h
 * - lv_watch_ui.h
 * - lv_watch_page.h
 * - lvgl.h
 *
 * @author zw1194
 *
 * @brief 手表UI(自研)的**入口**: 页面管理器实例 + 模型数据 + 各页共用
 *        的工具(切页/手势条/拖拽方向) + lv_watch_ui_init(). 三个页面各自一个文件:
 *        lv_watch_page_home.c / _menu.c / _heart.c(接口见 lv_watch_page.h).
 *
 * Processing flow:
 *
 * lv_watch_ui_init():
 *   1. page_mgr_init + 让三页各自 register(此时**一个控件都还没建**);
 *   2. 建模型定时器(1s, 推进步数/卡路里);
 *   3. 切换到首页 —— page_mgr_switch(HOME, 瞬切), home 的 pf_create 在这步才被调到.
 *   menu/heart 的控件**要到第一次切过去时才建**(懒创建 = 省 LVGL 内存池).
 *
 * 数据(本文件持有, 页面只读), 分两类:
 *   1) 模拟量 s_steps/s_kcal —— 由模型定时器每秒推进, 页面经 watch_model_*() 读;
 *   2) 真实时间 —— 日期与大字时钟的真源是**硬件 RTC**(LSE + 备份域), 经
 *      watch_model_datetime() 透传. 它不是"模型", 不需要谁来推进, 所以本文件只转发.
 *   为什么与页面分开: 页面会被 LRU 淘汰重建, 数据不该跟着归零.
 *   (历史上这里还有个 s_sec 假时钟, 2026-09-18 接 RTC 时删掉了 —— 界面已不再有
 *    "开机后累计的秒数"这种东西, 留着它会是个没人读、看着却像时间源的死变量.)
 *
 * 共用工具(页面通过 lv_watch_page.h 调用):
 *   watch_switch()       按 id 切页(各页唯一的导航手段)
 *   watch_strip_create() 建透明置顶手势条
 *   watch_swipe_track()  拖拽方向识别(方向→页面的映射在**各页自己的**回调里)
 *
 * 设计约束(这套UI早期版本曾出现切页卡死, 根因未定位; 下列是当时收敛出来的
 * "别再加回去"清单, 对三页同样有效):
 *   1. 不用大尺寸RGB565A8图片(只有 18x18 小图标, 见 LVGL/assets/image)
 *      —— 大图绘制曾是卡死嫌疑点, 小图未复现;
 *   2. 不使用shadow_width阴影(时钟阴影曾伴随卡死, LVGL阴影走A8层缓冲);
 *   3. 每页仅1个手势对象(自跟踪距离), 不绑LV_EVENT_ALL到多个对象;
 *   4. 字形必须在字库里确实存在 —— 字库是子集, 缺字**不报错只是不显示**
 *      (见 LVGL/assets/README.md 与 MDK-ARM/gen_watch_fonts.py);
 *   5. 不用i18n/lv_anim动画框架(无人调用 lv_anim_start, 无需).
 *
 * @version V2.1
 *
 * @note 1 tab == 4 spaces!
 *
 * @note ★版本历史★
 *       V1.0 三页常驻(init 时全部建好) + 一个全局1s定时器刷新全部页;
 *       V2.0 改成 PageMem 托管: 注册三页 + 懒创建 + 最近3页 LRU, 每页自带刷新
 *            定时器(pf_hide 暂停/pf_show 恢复), 模型数据移到静态变量;
 *       V2.1 三页各拆一个文件. 动机: 三页控件树加起来近千行, 同处一文件时
 *            "表盘的时钟在哪"要在三页之间翻. 拆开后每页的静态实例/vtable/控件
 *            句柄都是文件内 static, 页面之间**没有**任何交叉引用, 本文件只剩
 *            "入口 + 模型 + 共用工具".
 *       V2.2 页面对象缓冲区 3 → 2 槽(用户决定, 2026-09-22): 只留"当前页 + 上一次
 *            显示过的页", 为砍到 20KB 的 LVGL 池留峰值余量. 收益口径、与 3 槽的
 *            行为差异、改值前的推导依据都写在 PageMem.h 的 PAGE_MGR_BUF_SIZE
 *            注释与 PageMem.c 的文件头里(V2.0 那行的"最近3页"是当时的事实, 不改).
 ******************************************************************************/
#include "lv_watch_ui.h"

#include "lv_watch_page.h"

#include "system/log/cywatch_log.h" /* log_printf() */
#include <stdio.h>

/***********************************Defines************************************/
/* 手势判定阈值(累计位移超过它才算一次拖拽) */
#define WATCH_SWIPE_THRESH  50
/***********************************Defines************************************/

/**********************************Declaring***********************************/
static void watch_model_timer_cb(lv_timer_t *p_timer);
/**********************************Declaring***********************************/

/*********************************Static Data**********************************/
/* 页面管理器实例(三个页面的静态实例在各自的 .c 里, 不在这里) */
static page_mgr_t s_watch_mgr;

/* 模型定时器: 只推进数据, 不碰任何控件(与页面是否在屏上无关) */
static lv_timer_t *s_model_timer = NULL;

/* 模拟数据 —— 放静态变量而非页面结构体: 页面被淘汰重建后步数/心率不归零.
   注意这里**没有**时钟: 日期与大字时钟的真源是硬件 RTC, 见 watch_model_datetime() */
static uint32_t s_steps = 0; /* 步数 */
static uint32_t s_kcal = 0;	 /* 卡路里 */
static uint32_t s_tick = 0;	 /* 1s节拍计数(伪随机数据源) */

/* 拖拽距离累计: 同一时刻只有一个手势条在收事件, 所以共用一个累加器 */
static int32_t s_pull_x = 0;
static int32_t s_pull_y = 0;

/* 防重复初始化(重复 init 会让管理器的缓冲区清空, 而旧屏幕还挂在显示上) */
static uint8_t s_inited = 0;
/*********************************Static Data**********************************/

/******************************************************************************
 * @name    watch_switch
 * @brief   切页: 按 id 找页面 -> page_mgr_switch(接口见 lv_watch_page.h)
 * @param   page_id[in] 目标页面 ID(WATCH_PAGE_ID_xxx)
 * @param   anim[in]    切屏动画
 *
 * @return  无
 *
 * @note    **不报错**(返回 void): 调用方都是事件回调(按钮/手势), 那里没有任何可
 *          回滚的事务; 页不存在或切换失败都静默忽略
 * @note    目标页控件若已被淘汰, 管理器会在 page_mgr_switch 内部重建它(懒创建)
 *****************************************************************************/
void watch_switch(uint16_t page_id, lv_screen_load_anim_t anim)
{
	page_base_t *p_page = page_mgr_find(&s_watch_mgr, page_id);

	if (NULL == p_page)
	{
		return;
	}

	(void)page_mgr_switch(&s_watch_mgr, p_page, anim);
}

/******************************************************************************
 * @name    watch_strip_create
 * @brief   给页面创建手势条(透明, 置顶), 并绑定该页自己的拖拽回调
 * @param   parent[in] 页面根对象
 * @param   w[in]     宽度
 * @param   h[in]     高度
 * @param   x[in]     X位置
 * @param   y[in]     Y位置
 * @param   pf_cb[in] 该页的拖拽回调
 *
 * @return  无
 *
 * @note    置顶用 lv_obj_move_to_index(-1): 手势条必须盖在页面内容之上才收得到事件
 *****************************************************************************/
void watch_strip_create(lv_obj_t *parent, lv_coord_t w, lv_coord_t h,
						lv_coord_t x, lv_coord_t y, lv_event_cb_t pf_cb)
{
	lv_obj_t *strip;

	if (NULL == parent || NULL == pf_cb)
	{
		return;
	}

	strip = lv_obj_create(parent);
	lv_obj_set_pos(strip, x, y);
	lv_obj_set_size(strip, w, h);
	lv_obj_set_style_bg_opa(strip, LV_OPA_TRANSP, 0);
	lv_obj_set_style_border_width(strip, 0, 0);
	lv_obj_set_style_pad_all(strip, 0, 0);
	lv_obj_move_to_index(strip, -1);
	lv_obj_add_event_cb(strip, pf_cb, LV_EVENT_ALL, NULL);
}

/******************************************************************************
 * @name    watch_swipe_track
 * @brief   手势条拖拽跟踪: 累计位移, 松手时算出拖拽方向
 * @param   e[in] LVGL事件(PRESSED/PRESSING/RELEASED)
 *
 * @return  拖拽方向; 非松手事件/位移不够时返回 WATCH_SWIPE_NONE
 *
 * @note    自跟踪距离(不用LVGL内置手势), 与已验证过的demo同款方案; 各页的拖拽
 *          回调第一句就是它, 然后按返回值决定切到哪页
 *****************************************************************************/
watch_swipe_dir_t watch_swipe_track(lv_event_t *e)
{
	lv_event_code_t code = lv_event_get_code(e);
	int32_t dx = 0;
	int32_t dy = 0;
	watch_swipe_dir_t dir = WATCH_SWIPE_NONE;

	if (LV_EVENT_PRESSED == code)
	{
		s_pull_x = 0;
		s_pull_y = 0;
		return WATCH_SWIPE_NONE;
	}

	if (LV_EVENT_PRESSING == code)
	{
		lv_indev_t *p_indev = lv_indev_active();

		if (NULL != p_indev)
		{
			lv_point_t v;
			lv_indev_get_vect(p_indev, &v);
			s_pull_x += v.x;
			s_pull_y += v.y;
		}
		return WATCH_SWIPE_NONE;
	}

	if (LV_EVENT_RELEASED != code)
	{
		return WATCH_SWIPE_NONE;
	}

	/* 松手: 取累计值并清零, 按主轴判方向 */
	dx = s_pull_x;
	dy = s_pull_y;
	s_pull_x = 0;
	s_pull_y = 0;

	if (LV_ABS(dx) > LV_ABS(dy))
	{
		if (dx > WATCH_SWIPE_THRESH)
		{
			dir = WATCH_SWIPE_RIGHT;
		}
		else if (dx < -WATCH_SWIPE_THRESH)
		{
			dir = WATCH_SWIPE_LEFT;
		}
	}
	else
	{
		if (dy > WATCH_SWIPE_THRESH)
		{
			dir = WATCH_SWIPE_DOWN;
		}
		else if (dy < -WATCH_SWIPE_THRESH)
		{
			dir = WATCH_SWIPE_UP;
		}
	}

	return dir;
}

/******************************************************************************
 * @name    watch_model_datetime
 * @brief   取硬件 RTC 当前日期时间(接口与只读语义见 lv_watch_page.h)
 * @param   p_time[out] 回填的日期时间
 *
 * @return  0 成功; -1 p_time 为 NULL; 否则透传 cywatch_rtc_get() 的返回码
 *          (-2 未初始化/不可用, -3/-4 读寄存器失败, 详见 cywatch_rtc.h)
 *
 * @note    它与上面几个 watch_model_steps/kcal 的**性质不同**: 那几个是每秒推进的
 *          模拟量, 本函数是穿透到硬件的转发(见 lv_watch_page.h 的类型说明)
 * @note    返回码**逐字透传**, 不做二次映射: -2 是"根本没有时间源"的语义, 上层
 *          (表盘页)要据此画占位, 而不是画一个看着正常的假时间
 *****************************************************************************/
int8_t watch_model_datetime(cywatch_rtc_time_t *p_time)
{
	if (NULL == p_time)
	{
		return -1;
	}

	return cywatch_rtc_get(p_time);
}

/******************************************************************************
 * @name    watch_model_steps
 * @brief   取步数(接口见 lv_watch_page.h)
 * @param   无
 *
 * @return  步数
 *****************************************************************************/
uint32_t watch_model_steps(void)
{
	return s_steps;
}

/******************************************************************************
 * @name    watch_model_kcal
 * @brief   取卡路里(接口见 lv_watch_page.h)
 * @param   无
 *
 * @return  卡路里
 *****************************************************************************/
uint32_t watch_model_kcal(void)
{
	return s_kcal;
}

/******************************************************************************
 * @name    watch_model_timer_cb
 * @brief   模型定时器: 推进步数/心率/卡路里这几个模拟量(与页面在不在屏上无关)
 * @param   p_timer[in] LVGL定时器(未使用)
 *
 * @return  无
 *
 * @note    它与页面刷新定时器的区别: 页面只管"把数据画出来", 数据本身由这里推进
 *          (时间除外 —— 日期/时钟由硬件 RTC 自己走, 本定时器不掺和)
 *****************************************************************************/
static void watch_model_timer_cb(lv_timer_t *p_timer)
{
	(void)p_timer;

	s_tick++;

	/* 确定性伪随机走动(演示用) */
	s_steps += 1 + (s_tick % 3);
	s_kcal += (s_tick % 5);
}

/******************************************************************************
 * @name    lv_watch_ui_init
 * @brief   注册三页并切到表盘首页, 启动模型定时器
 * @param   无
 *
 * @return  0  success
 *         -1  page_mgr_init 失败
 *         -2  某个页面注册失败(表满/ID冲突/重复注册)
 *         -3  首页切换失败(控件创建失败/建的不是屏幕)
 *
 * @note    须在 lv_init/lv_port_disp_init/lv_port_indev_init 之后调用, 且只能在
 *          "lvgl" 任务里调用(见 PageMem.h 的跨任务访问契约)
 * @note    menu/heart 的控件此时**还没建** —— 第一次切过去时才由管理器创建
 *****************************************************************************/
int8_t lv_watch_ui_init(void)
{
	int8_t ret = 0;

	/* 防重复初始化: 重复调会把缓冲区清空, 而旧屏幕还挂在显示上 → 无人认领 */
	if (0 != s_inited)
	{
		return 0;
	}

	ret = page_mgr_init(&s_watch_mgr);
	if (0 != ret)
	{
		return -1;
	}

	/* 三页各自 register(只登记, 不建控件); 每页的实现细节在自己的 .c 里 */
	ret = watch_page_home_register(&s_watch_mgr);
	if (0 != ret)
	{
		return -2;
	}
	ret = watch_page_menu_register(&s_watch_mgr);
	if (0 != ret)
	{
		return -2;
	}
	ret = watch_page_heart_register(&s_watch_mgr);
	if (0 != ret)
	{
		return -2;
	}
	ret = watch_page_spo2_register(&s_watch_mgr);
	if (0 != ret)
	{
		return -2;
	}
	ret = watch_page_ota_register(&s_watch_mgr);
	if (0 != ret)
	{
		return -2;
	}

	/* 模型定时器: 与页面无关的数据推进 */
	if (NULL == s_model_timer)
	{
		s_model_timer = lv_timer_create(watch_model_timer_cb, WATCH_TICK_MS, NULL);
	}

	/* 首页: 瞬切(上电不要动画), home 的 pf_create 在这里才第一次被调到.
	   find 一定命中(上面刚注册成功), 返回 NULL 时 page_mgr_switch 会返回 -2 */
	ret = page_mgr_switch(&s_watch_mgr,
						  page_mgr_find(&s_watch_mgr, WATCH_PAGE_ID_HOME),
						  LV_SCR_LOAD_ANIM_NONE);
	if (0 != ret)
	{
		return -3;
	}

	s_inited = 1;

	return 0;
}

/******************************************************************************
 * @name    lv_watch_ui_mgr
 * @brief   取本UI的页面管理器实例(接口见 lv_watch_ui.h)
 * @param   无
 *
 * @return  管理器指针(恒非 NULL: 它是文件级静态实例)
 *
 * @note    只为自检/诊断开的口子, 见头文件里的 @note
 *****************************************************************************/
page_mgr_t *lv_watch_ui_mgr(void)
{
	return &s_watch_mgr;
}
