/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file lv_watch_selftest.c
 *
 * @par dependencies
 * - lv_watch_selftest.h
 * - lv_watch_page.h
 * - lv_watch_ui.h
 * - lvgl.h
 *
 * @author zw1194
 *
 * @brief 手表UI上电自检: 让"表盘/菜单/心率"三页各走一遍**完整生命周期**
 *        (创建→缓存→隐藏→销毁→重建), 每步断言, 结果打 USART1.
 *
 * Processing flow:
 *
 * lv_watch_selftest_start() 只建一个 lv_timer(100ms), 之后每一步由
 * lv_timer_handler() 带动推进, 走完自动删掉自己并把界面留在表盘页:
 *
 *   1  管理器就绪        三页都在注册表里
 *   2  首页基线          记录: 子控件数 / 定时器普查 / 池余量 / RTC时刻
 *   3  切到菜单          首页转 CACHED 且**对象地址未变**, 首页定时器被暂停
 *   4  切到心率          懒创建, 心率页自己那个定时器出现
 *   5  回菜单缓存        **命中缓存不重建**, 心率定时器被暂停
 *   6  全销毁            page_mgr_clear → 定时器回到基线(泄漏探针)+ 池内存还回来
 *   7  重建首页          新对象地址、子控件数与基线相同、定时器数回到基线+1
 *   8  时钟/日期跟随RTC  时钟与日期标签 == 硬件RTC(证明页面重建后画的是最新值)
 *   9  动画切心率        带动画切页
 *  10  动画后一致        动画结束后 管理器当前页 == lv_screen_active()
 *  11  收尾回表盘        回表盘 + 删自检定时器 + 打汇总
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note ★为什么"全局定时器普查"是这套自检里最强的一根探针★
 *       三页的 `lv_timer_t *p_timer` 都是各自 .c 里的 static, 外部拿不到句柄,
 *       所以没法直接 `lv_timer_get_paused(那一页的定时器)`. 改为每一步都遍历
 *       LVGL 定时器链表, 记下 `(总数, 其中被暂停的个数)` —— 这两个数的**增量**
 *       是可精确预测的, 一次同时抓住两类 bug:
 *         * "pf_hide 没真暂停" → 暂停数该 +1 却没 +1;
 *         * "pf_destroy 漏删定时器" → 第 6 步总数回不到基线(永久泄漏).
 *
 * @note ★第 3~7 步必须用 LV_SCR_LOAD_ANIM_NONE★
 *       已核实 page_mgr_anim_time() 对 NONE 返回 0(PageMem.c:73-76), LVGL 在
 *       time==0 && delay==0 时走"立即切屏"短路分支 → 全程不产生动画 →
 *       动画定时器不会动, 上面那两个计数才是干净的. 改成带动画会让暂停数
 *       每步翻动, 断言全乱.
 *
 * @note ★只比增量, 不要硬编码"暂停数 == 0"★
 *       lv_init() 时就建了动画定时器并把它暂停着(_lv_anim_core_init), 所以
 *       基线 paused 大概率已经是 1. 所有断言都是"相对基线/上一步"的增量.
 *
 * @note 本工程**没有任何断言机制**可用(NDEBUG + MicroLIB + LV_USE_ASSERT_* 全 0
 *       + LV_USE_LOG 0): 违规不会停也不会打印, 只会静默返回 NULL. 所以这里
 *       每一处判断都是显式比较 + printf, 失败**不中断**, 继续跑完剩下的步骤
 *       (与 cywatch_storage_fatfs.c 那个自检同款), 最后一格汇总.
 *
 * @note ★怎么证明这个自检不是"永远 PASS"★
 *       故意注入故障应当看到对应步骤变红(改完记得还原):
 *         * 注释掉 lv_watch_page_home.c 的 watch_home_destroy 里的
 *           lv_timer_delete(p_home->p_timer)  → 第 6 步必须 FAIL;
 *         * 删掉 lv_watch_page_heart.c 的 watch_heart_hide 里的 lv_timer_pause
 *           → 第 5 步必须 FAIL.
 *
 * @note ★只能在 "lvgl" 任务里调, 且在 lv_watch_ui_init() 成功之后★
 *       (LV_USE_OS=LV_OS_NONE, LVGL 内部无锁; 管理器与三页都是 lv_watch_ui_init
 *        建出来的). 见 PageMem.h 的跨任务访问契约.
 ******************************************************************************/
#include "lv_watch_selftest.h"

#include <stdio.h>
#include <string.h>

#include "lv_watch_page.h"
#include "lv_watch_ui.h"

/***********************************Defines************************************/
/* 自检推进节拍: 100ms 一步 —— 比页面自身的 1s 刷新快, 又不至于让每步之间的
   LVGL 处理次数太少(切页/删除需要 lv_timer_handler 迭代才落地) */
#define UI_TICK_MS          (100u)

/* 第 7 步之后等这么久再做第 8 步: 要等够 1 秒以上, 第 8 步才能证明时钟在**走**
   (只证明"某一刻对"是空过 —— 一个冻住的时钟也能在那一刻恰好对上). 1.2s 同时留出
   余量, 免得正好卡在 RTC 的秒边界上 */
#define UI_MODEL_WAIT_TICKS (12u) /* 1.2s */

/* 第 9 步带动画切页后等这么久再断言, 必须 > PAGE_MGR_ANIM_MS(300ms) */
#define UI_ANIM_WAIT_TICKS  (5u) /* 500ms */

/* 时钟标签的形状: "HH:MM:SS" —— 8字符且第3/6个是冒号 */
#define UI_CLOCK_LEN        (8u)

/* 日期标签的形状: "YYYY-MM-DD" —— 10字符且第5/8个是 '-' */
#define UI_DATE_LEN         (10u)
/***********************************Defines************************************/

/**********************************Declaring***********************************/
/* 自检步骤(顺序即执行顺序, s_step_fn[] 必须一一对应) */
typedef enum
{
	UI_STEP_MGR = 0,   /* 1  管理器就绪 */
	UI_STEP_HOME_BASE, /* 2  首页基线 */
	UI_STEP_TO_MENU,   /* 3  切到菜单 */
	UI_STEP_TO_HEART,  /* 4  切到心率 */
	UI_STEP_BACK_MENU, /* 5  回菜单(命中缓存) */
	UI_STEP_CLEAR,     /* 6  全销毁 */
	UI_STEP_REBUILD,   /* 7  重建首页 */
	UI_STEP_CLOCK,     /* 8  时钟/日期跟随RTC */
	UI_STEP_ANIM,      /* 9  动画切心率 */
	UI_STEP_SETTLED,   /* 10 动画后账本一致 */
	UI_STEP_FINISH,    /* 11 收尾回表盘 */
	UI_STEP_TOTAL,
} ui_step_t;

typedef void (*ui_step_fn_t)(void);

static void ui_step_mgr(void);
static void ui_step_home_base(void);
static void ui_step_to_menu(void);
static void ui_step_to_heart(void);
static void ui_step_back_menu(void);
static void ui_step_clear(void);
static void ui_step_rebuild(void);
static void ui_step_clock(void);
static void ui_step_anim(void);
static void ui_step_settled(void);
static void ui_step_finish(void);
static void ui_timer_cb(lv_timer_t *p_timer);
static void ui_report(uint8_t step, const char *p_name, bool ok, const char *p_detail);
/**********************************Declaring***********************************/

/*********************************Static Data**********************************/
static const ui_step_fn_t s_step_fn[UI_STEP_TOTAL] =
{
	ui_step_mgr,	   /* 1 */
	ui_step_home_base, /* 2 */
	ui_step_to_menu,   /* 3 */
	ui_step_to_heart,  /* 4 */
	ui_step_back_menu, /* 5 */
	ui_step_clear,	   /* 6 */
	ui_step_rebuild,   /* 7 */
	ui_step_clock,	   /* 8 */
	ui_step_anim,	   /* 9 */
	ui_step_settled,   /* 10 */
	ui_step_finish,	   /* 11 */
};

static lv_timer_t *s_timer = NULL; /* 自检驱动定时器(收尾时删掉) */
static uint8_t s_step = 0;		   /* 下一步的序号(下标进 s_step_fn) */
static uint8_t s_wait = 0;		   /* >0 时空转这么多拍再走下一步 */
static uint8_t s_pass = 0;
static uint8_t s_fail = 0;
static bool s_running = false; /* 防重复启动 */

/* ---- 跨步骤快照(第 N 步断言第 N-k 步记下的值, 所以必须存成静态) ---- */
static uint32_t s_base_timers = 0;	  /* 第2步: 定时器总数 */
static uint32_t s_base_paused = 0;	  /* 第2步: 其中暂停的个数 */
static uint32_t s_base_pool = 0;	  /* 第2步: LVGL池 free_size */
static uint32_t s_base_child = 0;	  /* 第2步: 首页根对象的直接子控件数 */
static uint32_t s_base_rtc = 0;		  /* 第2步: RTC时刻(单调代理量, 见 ui_rtc_mono) */
static lv_obj_t *s_obj_home = NULL;	  /* 第2步: 首页对象地址(第7步要比它变了) */
static lv_obj_t *s_obj_menu = NULL;	  /* 第3步: 菜单对象地址(第5步比它没变) */
static lv_obj_t *s_obj_heart = NULL;  /* 第4步: 心率对象地址(第5步比它没变) */
static uint32_t s_s4_timers = 0;	  /* 第4步: 定时器总数(第5步比它没变) */
static uint32_t s_s4_paused = 0;	  /* 第4步: 暂停个数(第5步比它 +1) */
/*********************************Static Data**********************************/

/******************************************************************************
 * @name    ui_census
 * @brief   定时器普查: 遍历 LVGL 定时器链表, 数出总数与其中被暂停的个数
 * @param   p_total[out]  定时器总数(含 LVGL 内部那些)
 * @param   p_paused[out] 其中 lv_timer_get_paused() 为真的个数
 *
 * @return  无
 *
 * @note    LVGL 9.3 **没有** lv_timer_get_count(), 只能这样走链表
 * @note    页面自己的定时器句柄是各页 .c 里的 static, 外部拿不到 —— 这正是
 *          本函数存在的理由: 用**全局增量**间接观察"暂停/删除到底发生没有"
 *****************************************************************************/
static void ui_census(uint32_t *p_total, uint32_t *p_paused)
{
	lv_timer_t *p_t = NULL;
	uint32_t total = 0;
	uint32_t paused = 0;

	for (p_t = lv_timer_get_next(NULL); NULL != p_t; p_t = lv_timer_get_next(p_t))
	{
		total++;
		if (lv_timer_get_paused(p_t))
		{
			paused++;
		}
	}

	*p_total = total;
	*p_paused = paused;
}

/******************************************************************************
 * @name    ui_pool_free
 * @brief   取 LVGL 内存池当前空闲字节数
 * @param   无
 *
 * @return  空闲字节数
 *
 * @note    池子 LV_MEM_SIZE=40KB 且 LV_MEM_POOL_EXPAND_SIZE=0(**不可增长**),
 *          耗尽就是静默失败 —— 所以这是"页面销毁到底有没有把内存还回来"的
 *          唯一探针(第6步用)
 *****************************************************************************/
static uint32_t ui_pool_free(void)
{
	lv_mem_monitor_t mon;

	lv_mem_monitor(&mon);

	return (uint32_t)mon.free_size;
}

/******************************************************************************
 * @name    ui_obj_of
 * @brief   取某页当前持有的根对象地址
 * @param   page_id[in] 页面 ID(WATCH_PAGE_ID_xxx)
 *
 * @return  根对象地址; 页面未注册/未创建(obj 为 NULL)时返回 NULL
 *****************************************************************************/
static lv_obj_t *ui_obj_of(uint16_t page_id)
{
	page_base_t *p_page = page_mgr_find(lv_watch_ui_mgr(), page_id);

	return (NULL != p_page) ? p_page->obj : NULL;
}

/******************************************************************************
 * @name    ui_state_of
 * @brief   取某页的生命周期状态
 * @param   page_id[in] 页面 ID
 *
 * @return  PAGE_STATE_xxx; 页面未注册时返回 PAGE_STATE_NONE
 *****************************************************************************/
static page_state_t ui_state_of(uint16_t page_id)
{
	page_base_t *p_page = page_mgr_find(lv_watch_ui_mgr(), page_id);

	return (NULL != p_page) ? p_page->state : PAGE_STATE_NONE;
}

/******************************************************************************
 * @name    ui_isdigit
 * @brief   字符是否为十进制数字
 * @param   c[in] 字符
 *
 * @return  true 是数字
 *****************************************************************************/
static bool ui_isdigit(char c)
{
	return (c >= '0' && c <= '9');
}

/******************************************************************************
 * @name    ui_find_clock
 * @brief   在对象子树里递归找"形如 HH:MM:SS 的标签"
 * @param   p_obj[in] 子树根
 *
 * @return  找到的标签对象; 没有则 NULL
 *
 * @note    **按文本形状找, 不按字体/控件名找** —— 换字库或改版面不该让自检误报.
 *          判据只有三条: 是 label / 长 8 / 第3与第6个字符是 ':'.
 * @note    只会命中表盘页那个时钟(home 页里没有别的 8 字符带双冒号的文本)
 *****************************************************************************/
static lv_obj_t *ui_find_clock(lv_obj_t *p_obj)
{
	uint32_t cnt = 0;
	uint32_t i = 0;
	lv_obj_t *p_hit = NULL;

	if (NULL == p_obj)
	{
		return NULL;
	}

	if (lv_obj_check_type(p_obj, &lv_label_class))
	{
		char *p_txt = lv_label_get_text(p_obj);

		if (NULL != p_txt && UI_CLOCK_LEN == strlen(p_txt) &&
			':' == p_txt[2] && ':' == p_txt[5] &&
			ui_isdigit(p_txt[0]) && ui_isdigit(p_txt[1]) &&
			ui_isdigit(p_txt[3]) && ui_isdigit(p_txt[4]) &&
			ui_isdigit(p_txt[6]) && ui_isdigit(p_txt[7]))
		{
			return p_obj;
		}
	}

	cnt = lv_obj_get_child_count(p_obj);
	for (i = 0; i < cnt; i++)
	{
		p_hit = ui_find_clock(lv_obj_get_child(p_obj, (int32_t)i));
		if (NULL != p_hit)
		{
			return p_hit;
		}
	}

	return NULL;
}

/******************************************************************************
 * @name    ui_parse_hms
 * @brief   把 "HH:MM:SS" 解析成秒数
 * @param   p_txt[in] 文本(调用方已确认形状)
 *
 * @return  HH*3600 + MM*60 + SS
 *****************************************************************************/
static uint32_t ui_parse_hms(const char *p_txt)
{
	uint32_t h = (uint32_t)(p_txt[0] - '0') * 10u + (uint32_t)(p_txt[1] - '0');
	uint32_t m = (uint32_t)(p_txt[3] - '0') * 10u + (uint32_t)(p_txt[4] - '0');
	uint32_t s = (uint32_t)(p_txt[6] - '0') * 10u + (uint32_t)(p_txt[7] - '0');

	return h * 3600u + m * 60u + s;
}

/******************************************************************************
 * @name    ui_find_date
 * @brief   在对象子树里递归找"形如 YYYY-MM-DD 的标签"(表盘页那行日期)
 * @param   p_obj[in] 子树根
 *
 * @return  找到的标签对象; 没有则 NULL
 *
 * @note    与 ui_find_clock 同款"按文本形状找", 不按字体/控件名: 是 label / 长 10 /
 *          第5与第8个字符是 '-' / 其余是数字. 两个形状门**互斥**(一个 8 字符带冒号,
 *          一个 10 字符带横杠), 谁也不会误命中谁
 * @note    RTC 不可用时表盘页画的是 "-- -- --", 这里就找不到 —— 与时钟一样, 交给第 8
 *          步去报"没有时间源", 而不是静默把一个占位当日期读
 *****************************************************************************/
static lv_obj_t *ui_find_date(lv_obj_t *p_obj)
{
	uint32_t cnt = 0;
	uint32_t i = 0;
	lv_obj_t *p_hit = NULL;

	if (NULL == p_obj)
	{
		return NULL;
	}

	if (lv_obj_check_type(p_obj, &lv_label_class))
	{
		char *p_txt = lv_label_get_text(p_obj);

		if (NULL != p_txt && UI_DATE_LEN == strlen(p_txt) &&
			'-' == p_txt[4] && '-' == p_txt[7] &&
			ui_isdigit(p_txt[0]) && ui_isdigit(p_txt[1]) &&
			ui_isdigit(p_txt[2]) && ui_isdigit(p_txt[3]) &&
			ui_isdigit(p_txt[5]) && ui_isdigit(p_txt[6]) &&
			ui_isdigit(p_txt[8]) && ui_isdigit(p_txt[9]))
		{
			return p_obj;
		}
	}

	cnt = lv_obj_get_child_count(p_obj);
	for (i = 0; i < cnt; i++)
	{
		p_hit = ui_find_date(lv_obj_get_child(p_obj, (int32_t)i));
		if (NULL != p_hit)
		{
			return p_hit;
		}
	}

	return NULL;
}

/******************************************************************************
 * @name    ui_parse_ymd
 * @brief   把 "YYYY-MM-DD" 解析成年/月/日(只为逐字段比较, 不做日历运算)
 * @param   p_txt[in] 文本(调用方已确认形状)
 * @param   p_y[out]  年
 * @param   p_m[out]  月
 * @param   p_d[out]  日
 *
 * @return  无
 *****************************************************************************/
static void ui_parse_ymd(const char *p_txt, uint32_t *p_y, uint32_t *p_m, uint32_t *p_d)
{
	*p_y = (uint32_t)(p_txt[0] - '0') * 1000u + (uint32_t)(p_txt[1] - '0') * 100u +
		   (uint32_t)(p_txt[2] - '0') * 10u + (uint32_t)(p_txt[3] - '0');
	*p_m = (uint32_t)(p_txt[5] - '0') * 10u + (uint32_t)(p_txt[6] - '0');
	*p_d = (uint32_t)(p_txt[8] - '0') * 10u + (uint32_t)(p_txt[9] - '0');
}

/******************************************************************************
 * @name    ui_rtc_mono_of
 * @brief   把一个**已读到的**时刻编成单调代理量(式子的来历见 ui_rtc_mono)
 * @param   p_t[in] 时刻快照
 *
 * @return  与 ui_rtc_mono 同式; p_t 为 NULL 时返回 0
 *
 * @note    单独拆出来是为了第 8 步: 那里已经握有一份 RTC 快照(时钟与日期都拿它比对),
 *          不该为了算单调量**再读一次外设** —— 同一份快照才能顺带证明"时钟、日期、
 *          单调量三者出自同一次读"(跨秒瞬间读了两次就会自相矛盾)
 *****************************************************************************/
static uint32_t ui_rtc_mono_of(const cywatch_rtc_time_t *p_t)
{
	if (NULL == p_t)
	{
		return 0u;
	}

	return ((((uint32_t)p_t->month * 31u + (uint32_t)p_t->day) * 24u +
			 (uint32_t)p_t->hour) * 3600u) +
		   ((uint32_t)p_t->minute * 60u) + (uint32_t)p_t->second;
}

/******************************************************************************
 * @name    ui_rtc_mono
 * @brief   取硬件 RTC 的一个**单调代理量**(只为比"只增不减", 不是真实时长)
 * @param   p_ok[out] 可空; RTC 可读时写 true, 否则写 false
 *
 * @return  ((月*31+日)*24+时)*3600 + 分*60 + 秒; RTC 不可读时返回 0
 *
 * @note    为什么不用"当日秒": 那个在午夜会回绕, 第 8 步的"只增不减"会**假失败**.
 *          本式把日期也编进来, 所以只有**跨年**才回绕一次 —— 自检只跑几秒, 可忽略
 * @note    刻意不做真正的日历换算(不判闰年/月长): 它只是给单调性用的标号. uint32
 *          装得下(最大 (12*31+31)*24+23 = 9695, 再 *3600 ≈ 3.5e7)
 *****************************************************************************/
static uint32_t ui_rtc_mono(bool *p_ok)
{
	cywatch_rtc_time_t t;

	if (NULL != p_ok)
	{
		*p_ok = false;
	}

	if (0 != watch_model_datetime(&t))
	{
		return 0u;
	}

	if (NULL != p_ok)
	{
		*p_ok = true;
	}

	return ui_rtc_mono_of(&t);
}

/******************************************************************************
 * @name    ui_report
 * @brief   打一行步骤结果并记账(PASS/FAIL 都记, 失败不中断)
 * @param   step[in]     步骤号(1 起, 打印用)
 * @param   p_name[in]   步骤名(**自带补齐空格**, 见各调用处的对齐)
 * @param   ok[in]       本步是否通过
 * @param   p_detail[in] 实测数值描述
 *
 * @return  无
 *
 * @note    格式与 system/Fatfs/port/cywatch_storage_fatfs.c 的自检一致:
 *          `[TAG] N. 步骤名  PASS/FAIL: <数值>`, 每行 \r\n
 *****************************************************************************/
static void ui_report(uint8_t step, const char *p_name, bool ok, const char *p_detail)
{
	if (ok)
	{
		s_pass++;
	}
	else
	{
		s_fail++;
	}

	printf("[UI] %2u. %s %s: %s\r\n", (unsigned)step, p_name,
		   ok ? "PASS" : "FAIL", p_detail);
}

/******************************************************************************
 * @name    ui_step_mgr
 * @brief   第1步: 管理器就绪 —— 三页都在注册表里
 * @param   无
 *
 * @return  无
 *
 * @note    先决条件检查. 这一步要是不过, 后面的步骤全都没意义(但仍会继续跑,
 *          每一行 FAIL 都是独立信息)
 *****************************************************************************/
static void ui_step_mgr(void)
{
	char d[112];
	page_mgr_t *p_mgr = lv_watch_ui_mgr();
	bool b_home = (NULL != page_mgr_find(p_mgr, WATCH_PAGE_ID_HOME));
	bool b_menu = (NULL != page_mgr_find(p_mgr, WATCH_PAGE_ID_MENU));
	bool b_heart = (NULL != page_mgr_find(p_mgr, WATCH_PAGE_ID_HEART));
	uint32_t found = (uint32_t)b_home + (uint32_t)b_menu + (uint32_t)b_heart;
	bool ok = (NULL != p_mgr) && (3u == found);

	snprintf(d, sizeof(d), "mgr=%s, find(home/menu/heart)=%u/3, 注册表 %u 页",
			 (NULL != p_mgr) ? "ok" : "NULL", (unsigned)found,
			 (NULL != p_mgr) ? (unsigned)p_mgr->page_cnt : 0u);
	ui_report(1u, "管理器就绪  ", ok, d);
}

/******************************************************************************
 * @name    ui_step_home_base
 * @brief   第2步: 首页基线 —— 记录后面各步要比对的基准值
 * @param   无
 *
 * @return  无
 *
 * @note    断言"首页正在显示且是独立屏幕": obj == lv_screen_active() 且
 *          根对象**没有父对象**(这是 PageMem 的载体约定, 违反会被 switch 判 -6)
 *****************************************************************************/
static void ui_step_home_base(void)
{
	char d[112];
	page_mgr_t *p_mgr = lv_watch_ui_mgr();
	page_base_t *p_home = page_mgr_find(p_mgr, WATCH_PAGE_ID_HOME);
	bool ok = false;

	s_obj_home = (NULL != p_home) ? p_home->obj : NULL;
	ui_census(&s_base_timers, &s_base_paused);
	s_base_pool = ui_pool_free();
	s_base_rtc = ui_rtc_mono(NULL);
	s_base_child = (NULL != s_obj_home) ? lv_obj_get_child_count(s_obj_home) : 0u;

	ok = (NULL != p_home) &&
		 (p_home == page_mgr_current(p_mgr)) &&
		 (NULL != s_obj_home) &&
		 (s_obj_home == lv_screen_active()) &&
		 (NULL == lv_obj_get_parent(s_obj_home)) &&
		 (PAGE_STATE_SHOWN == p_home->state) &&
		 (0u != s_base_child);

	snprintf(d, sizeof(d), "子控件 %u, 定时器 %u(暂停 %u), 池 %u, RTC %lu",
			 (unsigned)s_base_child, (unsigned)s_base_timers,
			 (unsigned)s_base_paused, (unsigned)s_base_pool,
			 (unsigned long)s_base_rtc);
	ui_report(2u, "首页基线    ", ok, d);
}

/******************************************************************************
 * @name    ui_step_to_menu
 * @brief   第3步: 切到菜单 —— 首页应被缓存且**对象地址不变**, 其定时器被暂停
 * @param   无
 *
 * @return  无
 *
 * @note    "对象地址不变"是这套UI的核心承诺: 被缓存的页**不重建**, 所以回上一页
 *          是秒开且控件状态还在(见 PageMem.h 的 page_mgr_switch @note).
 *          若这里地址变了, 说明缓存机制退化成了"每次重建".
 *****************************************************************************/
static void ui_step_to_menu(void)
{
	char d[112];
	page_mgr_t *p_mgr = lv_watch_ui_mgr();
	uint32_t timers = 0;
	uint32_t paused = 0;
	bool same_obj = false;
	bool ok = false;

	watch_switch(WATCH_PAGE_ID_MENU, LV_SCR_LOAD_ANIM_NONE);

	s_obj_menu = ui_obj_of(WATCH_PAGE_ID_MENU);
	ui_census(&timers, &paused);
	same_obj = (s_obj_home == ui_obj_of(WATCH_PAGE_ID_HOME));

	ok = (page_mgr_find(p_mgr, WATCH_PAGE_ID_MENU) == page_mgr_current(p_mgr)) &&
		 (NULL != s_obj_menu) &&
		 (PAGE_STATE_CACHED == ui_state_of(WATCH_PAGE_ID_HOME)) &&
		 (same_obj) &&						   /* 缓存页对象没被重建 */
		 (2u == page_mgr_buf_count(p_mgr)) &&
		 (timers == s_base_timers) &&		   /* 菜单页无定时器 → 总数不变 */
		 (paused == s_base_paused + 1u);	   /* 首页 pf_hide 暂停了它的定时器 */

	snprintf(d, sizeof(d), "buf=%u, 首页对象%s, 定时器 %u(%d)/暂停 %u(%d)",
			 (unsigned)page_mgr_buf_count(p_mgr), same_obj ? "未变" : "**变了**",
			 (unsigned)timers, (int)timers - (int)s_base_timers,
			 (unsigned)paused, (int)paused - (int)s_base_paused);
	ui_report(3u, "切到菜单    ", ok, d);
}

/******************************************************************************
 * @name    ui_step_to_heart
 * @brief   第4步: 切到心率 —— 懒创建, 心率页自己的定时器出现
 * @param   无
 *
 * @return  无
 *
 * @note    心率页是"第一次切过去才创建"(懒创建, 省 LVGL 池). 它比菜单页多一个
 *          1s 刷新定时器 —— 但 PAGE_MGR_BUF_SIZE 是 2, 这一步创建心率页正好用掉
 *          缓冲区的第二格, **把首页(队尾)淘汰掉**, 于是首页那个被暂停的定时器连同
 *          它的对象一起销毁. 所以定时器总数**净变化为 0**(+1 心率 / -1 首页),
 *          暂停数也**回到基线**(首页不再是"暂停", 是"没了").
 * @note    ★下面两个期望值是 2026-09-22 把缓冲区 3→2 时**按代码读出来重推**的,
 *          不是跑出来的 —— 本文件是死代码(零调用者, 整个 .o 被链接器 GC), 没有
 *          任何路径能执行到这里. 真要复活这套自检, 请按当时的缓冲区大小把每一步
 *          的期望值重新过一遍.★
 *****************************************************************************/
static void ui_step_to_heart(void)
{
	char d[112];
	uint32_t timers = 0;
	uint32_t paused = 0;
	bool ok = false;

	watch_switch(WATCH_PAGE_ID_HEART, LV_SCR_LOAD_ANIM_NONE);

	s_obj_heart = ui_obj_of(WATCH_PAGE_ID_HEART);
	ui_census(&timers, &paused);
	s_s4_timers = timers;
	s_s4_paused = paused;

	ok = (NULL != s_obj_heart) &&
		 (PAGE_STATE_SHOWN == ui_state_of(WATCH_PAGE_ID_HEART)) &&
		 (2u == page_mgr_buf_count(lv_watch_ui_mgr())) &&
		 (timers == s_base_timers) &&  /* +1(心率) -1(首页被淘汰) = 0 */
		 (paused == s_base_paused);	   /* 首页是被淘汰, 不是被暂停 */

	snprintf(d, sizeof(d), "buf=%u, 定时器 %u(%d)/暂停 %u(%d)",
			 (unsigned)page_mgr_buf_count(lv_watch_ui_mgr()),
			 (unsigned)timers, (int)timers - (int)s_base_timers,
			 (unsigned)paused, (int)paused - (int)s_base_paused);
	ui_report(4u, "切到心率    ", ok, d);
}

/******************************************************************************
 * @name    ui_step_back_menu
 * @brief   第5步: 回菜单 —— **命中缓存, 两页对象都不重建**, 心率定时器被暂停
 * @param   无
 *
 * @return  无
 *
 * @note    这一步是"回上一页秒开"的直接证据: 菜单页还在缓冲区里, 所以
 *          page_mgr_switch 只把它移到队首, 不调 pf_create; 心率页离开屏幕,
 *          pf_hide 暂停它的定时器 → 定时器总数不变而暂停数 +1.
 *****************************************************************************/
static void ui_step_back_menu(void)
{
	char d[112];
	page_mgr_t *p_mgr = lv_watch_ui_mgr();
	uint32_t timers = 0;
	uint32_t paused = 0;
	bool menu_same = false;
	bool heart_same = false;
	bool ok = false;

	watch_switch(WATCH_PAGE_ID_MENU, LV_SCR_LOAD_ANIM_NONE);

	menu_same = (s_obj_menu == ui_obj_of(WATCH_PAGE_ID_MENU));
	heart_same = (s_obj_heart == ui_obj_of(WATCH_PAGE_ID_HEART));
	ui_census(&timers, &paused);

	ok = (page_mgr_find(p_mgr, WATCH_PAGE_ID_MENU) == page_mgr_current(p_mgr)) &&
		 (menu_same) &&						 /* 菜单页没重建 */
		 (heart_same) &&					 /* 心率页没重建 */
		 (PAGE_STATE_CACHED == ui_state_of(WATCH_PAGE_ID_HEART)) &&
		 (2u == page_mgr_buf_count(p_mgr)) &&
		 (timers == s_s4_timers) &&			 /* 没多也没少 */
		 (paused == s_s4_paused + 1u);		 /* 心率 pf_hide 暂停了它的定时器 */

	snprintf(d, sizeof(d), "菜单/心率对象%s, 定时器 %u(%d)/暂停 %u(%d)",
			 (menu_same && heart_same) ? "均未重建" : "**有重建**",
			 (unsigned)timers, (int)timers - (int)s_s4_timers,
			 (unsigned)paused, (int)paused - (int)s_s4_paused);
	ui_report(5u, "回菜单缓存  ", ok, d);
}

/******************************************************************************
 * @name    ui_step_clear
 * @brief   第6步: 全销毁 —— page_mgr_clear 后三页回到"未创建", 定时器与内存还回来
 * @param   无
 *
 * @return  无
 *
 * @note    ★这是整套自检里最硬的一步★ 定时器总数必须**回到基线**: 三页各自的
 *          刷新定时器都该被 pf_destroy 里的 lv_timer_delete 删掉, 少删一个这个
 *          数就降不回基线 —— 这正是一个"永久泄漏"的探针(定时器不是 widget,
 *          lv_obj_delete 不管它, 见 PageMem.h 的说明).
 * @note    池 free_size 必须 **> 基线**: 第2步基线时首页还活着, 现在三页全没了,
 *          它们占的控件内存都该还回池子. 池是不可增长的(EXPAND_SIZE=0), 漏了
 *          就是永久少一块.
 * @note    clear 会连当前显示页一起删(之后 act_scr 为 NULL, 空屏不崩),
 *          所以界面上会看到**约 100ms 的空白**, 下一步重建首页即恢复.
 *****************************************************************************/
static void ui_step_clear(void)
{
	char d[112];
	page_mgr_t *p_mgr = lv_watch_ui_mgr();
	uint32_t timers = 0;
	uint32_t paused = 0;
	uint32_t pool = 0;
	bool all_gone = false;
	bool ok = false;

	page_mgr_clear(p_mgr);

	ui_census(&timers, &paused);
	pool = ui_pool_free();

	all_gone = (NULL == ui_obj_of(WATCH_PAGE_ID_HOME)) &&
			   (NULL == ui_obj_of(WATCH_PAGE_ID_MENU)) &&
			   (NULL == ui_obj_of(WATCH_PAGE_ID_HEART)) &&
			   (PAGE_STATE_NONE == ui_state_of(WATCH_PAGE_ID_HOME)) &&
			   (PAGE_STATE_NONE == ui_state_of(WATCH_PAGE_ID_MENU)) &&
			   (PAGE_STATE_NONE == ui_state_of(WATCH_PAGE_ID_HEART));

	ok = (0u == page_mgr_buf_count(p_mgr)) &&
		 (NULL == page_mgr_current(p_mgr)) &&
		 (all_gone) &&
		 (timers == s_base_timers) && /* ← 定时器泄漏探针 */
		 (paused == s_base_paused) &&
		 (pool > s_base_pool); /* ← 内存归还探针 */

	snprintf(d, sizeof(d), "buf=%u, 三页obj均为NULL=%u, 定时器 %u(基线 %u), 池 %u(基线 %u)",
			 (unsigned)page_mgr_buf_count(p_mgr), (unsigned)all_gone,
			 (unsigned)timers, (unsigned)s_base_timers,
			 (unsigned)pool, (unsigned)s_base_pool);
	ui_report(6u, "全销毁      ", ok, d);
}

/******************************************************************************
 * @name    ui_step_rebuild
 * @brief   第7步: 重建首页 —— 新对象、同样的树、定时器数回到基线+1
 * @param   无
 *
 * @return  无
 *
 * @note    三件独立的事:
 *          1. **对象地址必须变**(上一轮的已销毁, 这是重建而不是"其实没删干净");
 *          2. **直接子控件数必须与基线相同** —— pf_create 每次都建出同一棵树;
 *          3. 定时器数回到基线+1 且暂停数回到基线 —— pf_create 重建了首页的
 *             刷新定时器, 且 pf_show 把它 resume 了(不是留在暂停态).
 * @note    本步末尾设 s_wait, 让**硬件 RTC 走够 1 秒**, 供第8步断言(见其 @note)
 *****************************************************************************/
static void ui_step_rebuild(void)
{
	char d[112];
	uint32_t timers = 0;
	uint32_t paused = 0;
	lv_obj_t *p_obj = NULL;
	uint32_t child = 0;
	bool rebuilt = false;
	bool ok = false;

	watch_switch(WATCH_PAGE_ID_HOME, LV_SCR_LOAD_ANIM_NONE);

	p_obj = ui_obj_of(WATCH_PAGE_ID_HOME);
	child = (NULL != p_obj) ? lv_obj_get_child_count(p_obj) : 0u;
	rebuilt = (NULL != p_obj) && (p_obj != s_obj_home);
	ui_census(&timers, &paused);

	ok = (rebuilt) &&
		 (page_mgr_find(lv_watch_ui_mgr(), WATCH_PAGE_ID_HOME) ==
		  page_mgr_current(lv_watch_ui_mgr())) &&
		 (PAGE_STATE_SHOWN == ui_state_of(WATCH_PAGE_ID_HOME)) &&
		 (p_obj == lv_screen_active()) &&
		 (child == s_base_child) &&		   /* 树建得与基线一模一样 */
		 (timers == s_base_timers + 1u) && /* 首页定时器回来了 */
		 (paused == s_base_paused);		   /* 且是运行态(pf_show resume 过) */

	snprintf(d, sizeof(d), "对象%s, 子控件 %u(基线 %u), 定时器 %u(基线+1=%u)/暂停 %u(基线 %u)",
			 rebuilt ? "已重建" : "**未变**",
			 (unsigned)child, (unsigned)s_base_child,
			 (unsigned)timers, (unsigned)(s_base_timers + 1u),
			 (unsigned)paused, (unsigned)s_base_paused);
	ui_report(7u, "重建首页    ", ok, d);

	s_wait = (uint8_t)UI_MODEL_WAIT_TICKS; /* 等 RTC 走够1秒, 见第8步 */
}

/******************************************************************************
 * @name    ui_step_clock
 * @brief   第8步: 时钟/日期跟随 RTC —— 表盘上的时间与日期 == 硬件 RTC 当前值
 * @param   无
 *
 * @return  无
 *
 * @note    这是"页面被销毁重建后, 接着画的是**最新值**而不是从头开始"的直接验证.
 *          时间源是**硬件 RTC**(完全在页面之外), 且每次渲染都真读一次外设 —— 所以
 *          页面若自己从 0 开始计时、或把日期写死成字面量, 这一步就红了.
 * @note    容差 ±1 秒: RTC 硬件跨秒的那一瞬间与"页面上一帧画的"可能差一拍(首页刷新
 *          定时器 1s 与 RTC 的秒边界不同相). 我们不比"完全相等", 只比"没跑偏" ——
 *          冻住的时钟 / 写死的日期会差出好几秒甚至好几天, 照样抓得住.
 * @note    日期是**逐字段**比对(年/月/日都相等), 不是只比前缀 —— 只比前缀的话
 *          "2025-01-0X" 这种同月不同日会被放过.
 * @note    前面 s_wait 那 1.2s 就是为这一步服务的: 不等够 1 秒, 就只能证明"某一刻
 *          恰好对上"(一个冻住的时钟也做得到), 证明不了它在**走**.
 * @note    RTC 不可用时表盘画的是 "-- -- --" / "--:--:--", 两个形状门都找不到它们,
 *          本步就会报"没有时间源" —— 这正是我们要的(时间源死了必须红).
 *****************************************************************************/
static void ui_step_clock(void)
{
	char d[112];
	lv_obj_t *p_obj = ui_obj_of(WATCH_PAGE_ID_HOME);
	lv_obj_t *p_clock = ui_find_clock(p_obj);
	lv_obj_t *p_date = ui_find_date(p_obj);
	char *p_txt = (NULL != p_clock) ? lv_label_get_text(p_clock) : NULL;
	char *p_dtxt = (NULL != p_date) ? lv_label_get_text(p_date) : NULL;
	cywatch_rtc_time_t now;
	int8_t rtc_rc = watch_model_datetime(&now);
	uint32_t sec_rtc = 0u;
	uint32_t sec_label = (NULL != p_txt) ? ui_parse_hms(p_txt) : 0u;
	int32_t diff = 0;
	uint32_t y = 0u;
	uint32_t mo = 0u;
	uint32_t dy = 0u;
	bool ok = false;

	/* "没有时间源"与"页面画错了"分开报: 两者的修法完全不同, 合在一起报等于没报 */
	if (0 != rtc_rc)
	{
		snprintf(d, sizeof(d), "**RTC 不可用(rc=%d), 表盘应显示占位**", (int)rtc_rc);
	}
	else if ((NULL == p_txt) || (NULL == p_dtxt))
	{
		snprintf(d, sizeof(d), "**表盘上没找到 %s%s形状的标签**",
				 (NULL == p_txt) ? "HH:MM:SS " : "",
				 (NULL == p_dtxt) ? "YYYY-MM-DD " : "");
	}
	else
	{
		/* 单调量与上面两个标签同出**一次** RTC 读(now), 不额外读外设 */
		sec_rtc = ui_rtc_mono_of(&now);
		diff = (int32_t)sec_label - (int32_t)sec_rtc;
		ui_parse_ymd(p_dtxt, &y, &mo, &dy);

		ok = ((diff >= -1) && (diff <= 1)) &&			  /* 时钟跟着 RTC(容差1拍) */
			 ((uint32_t)now.year == y) &&				  /* 日期逐字段比对(不只看前缀) */
			 ((uint32_t)now.month == mo) &&
			 ((uint32_t)now.day == dy) &&
			 ((uint32_t)now.year >= 2000u) &&
			 ((uint32_t)now.year <= 2099u) &&			  /* 挡住"读到垃圾寄存器" */
			 (sec_rtc >= s_base_rtc);					  /* RTC 只增不减 */

		snprintf(d, sizeof(d),
				 "时钟 \"%s\" vs \"%02u:%02u:%02u\"(差 %lds); 日期 \"%s\" vs \"%04u-%02u-%02u\"",
				 p_txt, (unsigned)now.hour, (unsigned)now.minute, (unsigned)now.second,
				 (long)diff, p_dtxt, (unsigned)now.year, (unsigned)now.month, (unsigned)now.day);
	}
	ui_report(8u, "时钟/日期RTC", ok, d);
}

/******************************************************************************
 * @name    ui_step_anim
 * @brief   第9步: 带动画切到心率 —— 管理器账本应当立刻更新
 * @param   无
 *
 * @return  无
 *
 * @note    这一步只发起动画并检查"账本已更新"(page_mgr_switch 是同步改缓冲区的,
 *          不等动画结束); 动画结束后的状态由第10步在等够 500ms 之后断言.
 *****************************************************************************/
static void ui_step_anim(void)
{
	char d[112];
	bool ok = false;

	watch_switch(WATCH_PAGE_ID_HEART, LV_SCR_LOAD_ANIM_OVER_BOTTOM);

	ok = (PAGE_STATE_SHOWN == ui_state_of(WATCH_PAGE_ID_HEART)) &&
		 (PAGE_STATE_CACHED == ui_state_of(WATCH_PAGE_ID_HOME));

	snprintf(d, sizeof(d), "发起 OVER_BOTTOM(%ums), buf=%u, 心率=%d/首页=%d",
			 (unsigned)PAGE_MGR_ANIM_MS,
			 (unsigned)page_mgr_buf_count(lv_watch_ui_mgr()),
			 (int)ui_state_of(WATCH_PAGE_ID_HEART),
			 (int)ui_state_of(WATCH_PAGE_ID_HOME));
	ui_report(9u, "动画切心率  ", ok, d);

	s_wait = (uint8_t)UI_ANIM_WAIT_TICKS; /* 等动画走完再看账本 */
}

/******************************************************************************
 * @name    ui_step_settled
 * @brief   第10步: 动画结束后, 管理器当前页与 LVGL 活动屏幕仍一致
 * @param   无
 *
 * @return  无
 *
 * @note    动画期间 LVGL 内部有 d->prev_scr 之类的中间状态, 动画收尾若没清干净,
 *          最容易表现成"管理器认为在显示心率页, 而 LVGL 的活动屏是别的/空".
 *          这一步跨越完整的 300ms 动画之后比对两者, 就是冲着这类不一致去的.
 *****************************************************************************/
static void ui_step_settled(void)
{
	char d[112];
	page_base_t *p_cur = page_mgr_current(lv_watch_ui_mgr());
	lv_obj_t *p_act = lv_screen_active();
	bool ok = (NULL != p_cur) && (NULL != p_act) && (p_cur->obj == p_act);

	snprintf(d, sizeof(d), "管理器当前页 obj=%s, lv_screen_active=%s",
			 (NULL == p_cur) ? "NULL" : "非NULL",
			 (NULL == p_act) ? "NULL" : "非NULL");
	ui_report(10u, "动画后一致  ", ok, d);
}

/******************************************************************************
 * @name    ui_step_finish
 * @brief   第11步: 收尾 —— 回表盘页 + 删自检定时器 + 打汇总行
 * @param   无
 *
 * @return  无
 *
 * @note    汇总行放在**所有步骤之后**打印, 这样 scrollback 时先看到结论
 *          (与 cywatch_storage_fatfs.c 同款约定)
 * @note    必须把界面留在表盘页: 自检跑完是给人用的, 不能停在心率页
 * @note    自检自己的定时器也要删掉 —— 不删就是给系统永久多留一个 100ms 定时器,
 *          而且第6步那种"定时器回到基线"的账也会被它自己污染
 *****************************************************************************/
static void ui_step_finish(void)
{
	lv_mem_monitor_t mon;
	uint32_t timers = 0;
	uint32_t paused = 0;
	bool ok = false;

	watch_switch(WATCH_PAGE_ID_HOME, LV_SCR_LOAD_ANIM_NONE);

	ok = (PAGE_STATE_SHOWN == ui_state_of(WATCH_PAGE_ID_HOME)) &&
		 (ui_obj_of(WATCH_PAGE_ID_HOME) == lv_screen_active());

	ui_report(11u, "收尾回表盘  ", ok, "已回表盘页");

	/* 汇总: 先停掉自己, 再统计(否则把自己也算进定时器数里) */
	if (NULL != s_timer)
	{
		lv_timer_delete(s_timer);
		s_timer = NULL;
	}
	s_running = false;

	ui_census(&timers, &paused);
	lv_mem_monitor(&mon);

	printf("[UI] ===== 自检结束: %u 项, PASS %u / FAIL %u | 池 free=%u frag=%u%% "
		   "| 定时器 %u(暂停 %u) =====\r\n\r\n",
		   (unsigned)UI_STEP_TOTAL, (unsigned)s_pass, (unsigned)s_fail,
		   (unsigned)mon.free_size, (unsigned)mon.frag_pct,
		   (unsigned)timers, (unsigned)paused);
}

/******************************************************************************
 * @name    ui_timer_cb
 * @brief   自检驱动: 每拍走一步(或按 s_wait 空转)
 * @param   p_timer[in] LVGL定时器(未使用)
 *
 * @return  无
 *
 * @note    当前定时器被自己删掉是安全的: lv_timer_handler 检测到 timer_deleted
 *          就会重新从链表头扫描(lv_timer.c:107-113), 这是 LVGL 支持的用法
 *****************************************************************************/
static void ui_timer_cb(lv_timer_t *p_timer)
{
	(void)p_timer;

	if (s_wait > 0u)
	{
		s_wait--;
		return;
	}

	if (s_step >= (uint8_t)UI_STEP_TOTAL)
	{
		return; /* 不该发生: 走完最后一步就自删了 */
	}

	s_step_fn[s_step]();
	s_step++;
}

/******************************************************************************
 * @name    lv_watch_selftest_start
 * @brief   启动手表UI自检(接口见 lv_watch_selftest.h)
 * @param   无
 *
 * @return  0  已排队(从下一个 lv_timer_handler 开始按步推进)
 *         -1 重复启动
 *
 * @note    返回 0 不等于自检通过, 逐步结果看 USART1 的 [UI] 行
 *****************************************************************************/
int8_t lv_watch_selftest_start(void)
{
	if (s_running)
	{
		printf("[UI] selftest already running\r\n");
		return -1;
	}

	s_step = 0;
	s_wait = 0;
	s_pass = 0;
	s_fail = 0;
	s_running = true;

	printf("\r\n[UI] ===== 手表UI自检开始(页面管理器 + 表盘/菜单/心率 三页生命周期) =====\r\n");
	printf("[UI] 断言口径: 定时器普查=全局定时器链表的(总数,暂停数), 只比增量\r\n");

	s_timer = lv_timer_create(ui_timer_cb, UI_TICK_MS, NULL);
	if (NULL == s_timer)
	{
		printf("[UI] ===== 自检无法启动: lv_timer_create 失败(池耗尽?) =====\r\n\r\n");
		s_running = false;
		return -1;
	}

	return 0;
}
