/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file lv_watch_page_spo2.c
 *
 * @par dependencies
 * - lv_watch_page.h
 * - easyapp_port.h
 * - lv_watch_page_spo2.h
 * - lvgl.h
 *
 * @author zw1194
 *
 * @brief 手表UI - 血氧检测页(spo2, ID 0x0303): 黑底 + 居中红色圆形按钮, 按下开始
 *        测血氧, 最长 1 分钟(可提前结束), 圆内显示倒计时与血氧值. 由 PageMem 托管.
 *
 * Processing flow:
 *
 * pf_create : 建独立屏幕 + 标题 + 状态行 + 全屏手势条 + 红色圆按钮 + 圆内三行标签
 *             + 本页1s刷新定时器(快照一并复位, 见下 @note 重建);
 * pf_show   : 立即补一帧 + 恢复定时器;
 * pf_hide   : 暂停定时器;
 * pf_destroy: lv_timer_delete + 句柄置 NULL;
 * 点击按钮  : 只发 EVT_SERVICE_SPO2_BUTTON, 不做任何判断;
 * 每 1s     : 解开 s_snap 快照 → 重画(见下"三态");
 * 手势      : 左滑→心率页(MOVE_LEFT) / 右滑→OTA升级页(MOVE_RIGHT).
 *
 * ★本页现在只做两件事: 绘制 + 把按钮转成事件★ 业务(状态机/倒计时/可信度)整个
 * 搬进了 cywatch_service_spo2.c 的任务, 它经 watch_page_spo2_post() 把快照写进
 * s_snap, 本页 1s 定时器读出来画. 三态:
 *
 *   IDLE      圆内"开始"                │ 服务任务: 未开始 / 被取消
 *   MEASURING 圆内: 血氧值(或"--")      │ 服务任务: 已起, 等数据 / 攒可信度
 *              + 倒计时 mm:ss           │
 *             圆下: "请贴紧手指"/"测量中"│
 *   DONE      圆内: 定格值 + "%"        │ 服务任务: 到点 / 提前结束
 *             圆下: "已完成"            │ (没测到值则退回"开始")
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note ★数据来自 service_spo2 任务, 不是本页自己采的★ 本页**不** include 任何
 *       服务头, 只借 lv_watch_page_spo2.h 那个零依赖口约好的枚举与签名.
 *
 * @note ★"测量中"门限不在这页了★ doc/心率服务开发问题记录.md 第七节问题 3 记着:
 *       SpO2.c 缺"直流收敛"判据, 手指刚贴合时那几个 80.0/82.2 是直流台阶瞬态造成的
 *       **伪值**, 那个缺陷**没修**(要动已标定的门限). 本页原来的对策是"连续 5 秒都
 *       读到非零才亮数字", 现在整段搬进了 cywatch_service_spo2.c —— ★那边按**毫秒**
 *       累计★(心率事件是 2Hz, 逐帧计数会把 5 秒砍成 2.5 秒). 页面只认结果: value
 *       是 0 就画 "--", 不冒充读数.
 *
 * @note ★两个"顺序/初值"上的坑, 改的时候别踩★
 *       1. **建序承重**: watch_strip_create() 内部会把手势条抬到最顶层
 *          (lv_obj_move_to_index(-1)), 所以圆形按钮必须**在它之后**建, 靠"同级子里
 *          索引更高"在命中测试里压住手势条, 否则按钮点不动. menu 页当年正是为了这个
 *          把手势条缩到按钮下方的空白区(见 lv_watch_page_menu.c 的"不挡按钮").
 *          代价: 起点落在圆内的左滑不生效, 圆外整屏仍可滑.
 *       2. **pf_create 必须清 s_snap**: PAGE_MGR_BUF_SIZE=2 而注册了 4 页, 本页会被
 *          LRU 淘汰后又切回来, 那时走的是重建路径 —— 不清就会带着上一次的"已完成"
 *          和旧数值回来.
 ******************************************************************************/
#include "lv_watch_page.h"
#include "easyapp_port.h"
#include "lv_watch_page_spo2.h"

#include "system/log/cywatch_log.h" /* log_printf() */
#include <stdio.h>

/* 子集字库(实现在 LVGL/assets/fonts, 生成脚本 MDK-ARM/gen_watch_fonts.py):
   数值用20(只需 ASCII), "血氧检测/开始/请贴紧手指/测量中/已完成"这类汉字用14.
   ★写新的中文文案前先确认字在子集里★ —— 缺字**不报错**, 只是那个字不显示 */
LV_FONT_DECLARE(lv_font_montserrat_regular_20)
LV_FONT_DECLARE(lv_font_alibaba_puhuiti_14)

/***********************************Defines************************************/
#define WATCH_SPO2_BTN_D    140			 /* 圆按钮直径 */
#define WATCH_SPO2_BTN_X    50			 /* (240-140)/2, 水平居中 */
#define WATCH_SPO2_BTN_Y    72			 /* 在标题之下、状态行之上 */
#define WATCH_SPO2_STATUS_Y 228			 /* 圆下状态行的 Y */
#define WATCH_COL_SPO2_RED  0xE03030	 /* 按钮红 */

/* s_snap 的四段布局: [7:0]state | [15:8]remain_s | [23:16]value | [31:24]finger.
   ★打包与解包共用这组宏, 别在两边各写一遍移位★ —— 错开一位不会报错, 只是静默画错 */
#define WATCH_SPO2_SNAP_PACK(s, r, v, f)                                \
	((uint32_t)(s) | ((uint32_t)(r) << 8) |                             \
	 ((uint32_t)(v) << 16) | ((uint32_t)(f) << 24))
#define WATCH_SPO2_SNAP_STATE(x)  ((uint32_t)(x) & 0xFFu)
#define WATCH_SPO2_SNAP_REMAIN(x) (((uint32_t)(x) >> 8) & 0xFFu)
#define WATCH_SPO2_SNAP_VALUE(x)  (((uint32_t)(x) >> 16) & 0xFFu)
#define WATCH_SPO2_SNAP_FINGER(x) (((uint32_t)(x) >> 24) & 0xFFu)
/***********************************Defines************************************/

/**********************************Declaring***********************************/
/* 血氧页: 圆按钮 + 圆内三行标签 + 圆下状态行 + 自己的1s刷新定时器.
   ★业务状态不在这里★ —— 全在下面那个打包快照 s_snap 里 */
typedef struct
{
	page_base_t base;
	lv_obj_t *p_btn;	 /* 红色圆形按钮 */
	lv_obj_t *p_idle;	 /* 圆内: "开始"(仅 IDLE / 没测到的 DONE 显示) */
	lv_obj_t *p_val;	 /* 圆内: 血氧值, 未可信时是 "--" */
	lv_obj_t *p_unit;	 /* 圆内: "%" 或倒计时 mm:ss */
	lv_obj_t *p_status;	 /* 圆下: "请贴紧手指"/"测量中"/"已完成" */
	lv_timer_t *p_timer;
} watch_spo2_t;

static void watch_spo2_render(watch_spo2_t *p_spo2);
static void watch_spo2_timer_cb(lv_timer_t *p_timer);
static void watch_spo2_btn_cb(lv_event_t *e);

static void watch_spo2_swipe_cb(lv_event_t *e);

static void watch_spo2_create(page_base_t *p_page);
static void watch_spo2_destroy(page_base_t *p_page);
static void watch_spo2_show(page_base_t *p_page);
static void watch_spo2_hide(page_base_t *p_page);
/**********************************Declaring***********************************/

/*********************************Static Data**********************************/
static watch_spo2_t s_spo2;

/* ★跨任务的唯一通道★ 血氧服务任务写(一条 STR), 本页 1s 定时器读(一条 LDR).
   布局见上面 WATCH_SPO2_SNAP_* 那组宏.
   ★必须打包成一个字★: 分四次投会被抢占(写端在血氧任务、读端在 lvgl 任务, 两者
   同为 osPriorityNormal 且开时间片), 读端会看到"state 已是 DONE 但 value 还是上一
   帧"这种逻辑上不存在的中间态 */
static volatile uint32_t s_snap;

static const page_vtable_t s_spo2_vtable =
{
	.pf_create = watch_spo2_create,
	.pf_destroy = watch_spo2_destroy,
	.pf_show = watch_spo2_show,
	.pf_hide = watch_spo2_hide,
};
/*********************************Static Data**********************************/

/******************************************************************************
 * @name    watch_page_spo2_post
 * @brief   血氧服务任务的写入口: 把一整帧测量快照投给本页(见 lv_watch_page_spo2.h)
 * @param   state   [in] 三态之一
 * @param   remain_s[in] 剩余秒数(MEASURING 时作倒计时)
 * @param   value   [in] 血氧值 %; 0 = 本帧还没有可信读数
 * @param   finger  [in] 1 = 手指贴上了
 *
 * @return  无
 *
 * @note    ★跑在血氧服务任务上下文, 不是 lvgl 任务★ 所以这里只能写 s_snap 这个
 *          字, 绝不能碰任何 lv_obj_t / 调 lv_*(LVGL 无锁, 对象树只归 lvgl 任务)
 *****************************************************************************/
void watch_page_spo2_post(watch_spo2_state_e state, uint8_t remain_s,
						  uint8_t value, uint8_t finger)
{
	/* ★一条 STR 原子落地★ 四段一次写完 */
	s_snap = WATCH_SPO2_SNAP_PACK(state, remain_s, value, finger);
}

/******************************************************************************
 * @name    watch_spo2_render
 * @brief   把当前快照画到圆内三行标签 + 圆下状态行上(纯绘制, 不推进数据)
 * @param   p_spo2[in] 血氧页
 *
 * @return  无
 *
 * @note    p_val 这一行在 MEASURING 下**始终可见**: 可信时是数字, 还不可信时是
 *          "--". 这样圆内版面在整轮测量里不会跳动, 用户也能看出"在算, 只是还没
 *          出数" —— 比"先空着、突然蹦出数字"诚实, 也正好把算法瞬态那一段挡在外面
 *****************************************************************************/
static void watch_spo2_render(watch_spo2_t *p_spo2)
{
	char buf[16];
	uint32_t snap;
	watch_spo2_state_e state;
	uint32_t rem = 0;
	uint32_t val;
	uint32_t fing;

	if (NULL == p_spo2)
	{
		return;
	}

	/* ★只读一次★ 一条 LDR 拿到的就是自洽的一帧; 读两次可能读到两次投递之间 */
	snap  = s_snap;
	state = (watch_spo2_state_e)WATCH_SPO2_SNAP_STATE(snap);
	rem   = WATCH_SPO2_SNAP_REMAIN(snap);
	val   = WATCH_SPO2_SNAP_VALUE(snap);
	fing  = WATCH_SPO2_SNAP_FINGER(snap);

	if (WATCH_SPO2_IDLE == state)
	{
		lv_obj_remove_flag(p_spo2->p_idle, LV_OBJ_FLAG_HIDDEN);
		lv_obj_add_flag(p_spo2->p_val, LV_OBJ_FLAG_HIDDEN);
		lv_obj_add_flag(p_spo2->p_unit, LV_OBJ_FLAG_HIDDEN);
		lv_label_set_text(p_spo2->p_status, "");
		return;
	}

	if (WATCH_SPO2_MEASURING == state)
	{
		lv_obj_add_flag(p_spo2->p_idle, LV_OBJ_FLAG_HIDDEN);
		lv_obj_remove_flag(p_spo2->p_val, LV_OBJ_FLAG_HIDDEN);
		lv_obj_remove_flag(p_spo2->p_unit, LV_OBJ_FLAG_HIDDEN);

		/* 可信度由服务任务算完了: 0 = 还没攒够, 不冒充结果 */
		if (0u == val)
		{
			snprintf(buf, sizeof(buf), "--");
		}
		else
		{
			snprintf(buf, sizeof(buf), "%u", (unsigned)val);
		}
		lv_label_set_text(p_spo2->p_val, buf);

		/* 倒计时(显示剩余时间, 与"最长 1 分钟"的语义一致) */
		snprintf(buf, sizeof(buf), "%02lu:%02lu",
				 (unsigned long)(rem / 60u), (unsigned long)(rem % 60u));
		lv_label_set_text(p_spo2->p_unit, buf);

		/* 手指没贴上就明说, 与"贴着呢但还没出数"区分开 */
		lv_label_set_text(p_spo2->p_status,
						  (0u == fing) ? "请贴紧手指" : "测量中");
		return;
	}

	/* ---- WATCH_SPO2_DONE ---- */
	lv_label_set_text(p_spo2->p_status, "已完成");

	if (0u == val)
	{
		/* 一分钟(或提前结束)都没拿到可信值: 退回"开始"让用户重来 */
		lv_obj_remove_flag(p_spo2->p_idle, LV_OBJ_FLAG_HIDDEN);
		lv_obj_add_flag(p_spo2->p_val, LV_OBJ_FLAG_HIDDEN);
		lv_obj_add_flag(p_spo2->p_unit, LV_OBJ_FLAG_HIDDEN);
		return;
	}

	snprintf(buf, sizeof(buf), "%u", (unsigned)val);
	lv_label_set_text(p_spo2->p_val, buf);
	lv_obj_remove_flag(p_spo2->p_val, LV_OBJ_FLAG_HIDDEN);
	lv_obj_add_flag(p_spo2->p_idle, LV_OBJ_FLAG_HIDDEN);
	lv_label_set_text(p_spo2->p_unit, "%");
	lv_obj_remove_flag(p_spo2->p_unit, LV_OBJ_FLAG_HIDDEN);
}

/******************************************************************************
 * @name    watch_spo2_timer_cb
 * @brief   本页刷新定时器: 把服务任务投来的快照画出来(纯绘制, 不推进任何状态)
 * @param   p_timer[in] LVGL定时器(未使用)
 *
 * @return  无
 *
 * @note    为什么还要轮询而不是"数据一变就叫醒本页": LVGL 是**无锁**的, 服务任务
 *          不能从别的任务里调 lv_* 触发重绘. 1s 的节拍刚好对上倒计时(每秒变一次)
 * @note    IDLE/DONE 下定时器仍在跑, 只是反复画同一帧 —— 不用"按状态暂停/恢复定时器",
 *          省掉一处状态与定时器不一致的可能
 *****************************************************************************/
static void watch_spo2_timer_cb(lv_timer_t *p_timer)
{
	(void)p_timer;

	watch_spo2_render(&s_spo2);
}

/******************************************************************************
 * @name    watch_spo2_btn_cb
 * @brief   圆按钮点击: 只把"按了"转成 EVT_SERVICE_SPO2_BUTTON, 不做任何判断
 * @param   e[in] LVGL事件(CLICKED)
 *
 * @return  无
 *
 * @note    "开始 / 提前结束"的判别整个搬进了 service_spo2_start()(在 appcore 任务
 *          里跑) —— 本页不知道任务在不在跑, 也不该知道
 * @note    ★send 不能放 ISR★ 本回调跑在 lvgl 任务, send 内含 osKernelLock, 合法
 *****************************************************************************/
static void watch_spo2_btn_cb(lv_event_t *e)
{
	(void)e;

	x_port_easyapp_event_send(EVT_SERVICE_SPO2_BUTTON, 0, NULL);
}

/******************************************************************************
 * @name    watch_spo2_swipe_cb
 * @brief   本页手势: 左滑→心率页(MOVE_LEFT), 右滑→OTA升级页(MOVE_RIGHT)
 * @param   e[in] LVGL事件
 *
 * @return  无
 *
 * @note    本页是"心率页右滑进来"的, 所以左滑回去 —— 与来路反向对称.
 *          旗标 5 = 往左切页(跨页统一的方向旗标); 值 5 在心率页那边表示"回表盘",
 *          在本页表示"回心率页": 旗标编码的是**方向**, 目标页由各页自己的 APP 层
 *          switch_page 决定(见 APP/APP/ui/cywatch_app_ui_spo2.c)
 * @note    ★watch_swipe_track() 只能调一次★ 它每次调用都会**读并清零**累积位移.
 *          本回调原来把 track 直接写在 if 条件里(只有一个分支时没问题), 加第二个
 *          分支前必须先改成"先存进局部变量"(同心率页的写法), 否则右滑那支永远
 *          拿到 WATCH_SWIPE_NONE, 静默失效
 * @note    右滑去 OTA 是 2026-09-18 加环时新增的边: 环上 OTA 在血氧页右边
 *****************************************************************************/
static void watch_spo2_swipe_cb(lv_event_t *e)
{
	watch_swipe_dir_t dir = watch_swipe_track(e);

	if (WATCH_SWIPE_LEFT == dir)
	{
		watch_switch(WATCH_PAGE_ID_HEART, LV_SCR_LOAD_ANIM_MOVE_LEFT);

		/* ★不能放 ISR: 本回调跑在 lvgl 任务, send 内含 osKernelLock, 合法★ */
		x_port_easyapp_event_send(EVT_SERVICE_UIHOME_SWITCH_PAGE, 5, NULL);
	}
	else if (WATCH_SWIPE_RIGHT == dir)
	{
		/* 右滑 → OTA升级页(旗标 3 = 目标页在右方) */
		watch_switch(WATCH_PAGE_ID_OTA, LV_SCR_LOAD_ANIM_MOVE_RIGHT);
		x_port_easyapp_event_send(EVT_SERVICE_UIHOME_SWITCH_PAGE, 3, NULL);
	}
	else
	{
		/* 上/下滑: 本页没有对应页面, 什么都不做 */
	}
}

/******************************************************************************
 * @name    watch_spo2_create
 * @brief   建本页: 标题 + 状态行 + 手势条 + 红圆按钮 + 圆内三行标签 + 刷新定时器
 * @param   p_page[in] 页面基类指针
 *
 * @return  无
 *
 * @note    见文件头的"两个坑": 手势条必须**先**建(按钮要压住它), 快照必须复位
 *****************************************************************************/
static void watch_spo2_create(page_base_t *p_page)
{
	watch_spo2_t *p_spo2 = (watch_spo2_t *)p_page;
	lv_obj_t *scr = lv_obj_create(NULL);
	lv_obj_t *lbl = NULL;

	/* 清掉可能残留的上一次实例句柄 + 复位快照(本页会被 LRU 淘汰后重建) */
	p_spo2->p_btn = NULL;
	p_spo2->p_idle = NULL;
	p_spo2->p_val = NULL;
	p_spo2->p_unit = NULL;
	p_spo2->p_status = NULL;
	s_snap = WATCH_SPO2_SNAP_PACK(WATCH_SPO2_IDLE, 0u, 0u, 0u);

	lv_obj_set_style_bg_color(scr, lv_color_hex(WATCH_COL_DARK), 0);

	/* 标题(本页没有专用图标 —— LVGL/assets/image 里只有 步数/心形/火焰 三张) */
	lbl = lv_label_create(scr);
	lv_label_set_text(lbl, "血氧检测");
	lv_obj_set_style_text_font(lbl, &lv_font_alibaba_puhuiti_14, 0);
	lv_obj_set_style_text_color(lbl, lv_color_hex(WATCH_COL_TEXT), 0);
	lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, 20, 26);

	/* 圆下状态行 */
	p_spo2->p_status = lv_label_create(scr);
	lv_label_set_text(p_spo2->p_status, "");
	lv_obj_set_style_text_font(p_spo2->p_status, &lv_font_alibaba_puhuiti_14, 0);
	lv_obj_set_style_text_color(p_spo2->p_status, lv_color_hex(WATCH_COL_GRAY), 0);
	lv_obj_align(p_spo2->p_status, LV_ALIGN_TOP_MID, 0, WATCH_SPO2_STATUS_Y);

	/* ★手势条先建★ 它内部会把自己抬到最顶层, 圆按钮必须建在它之后才压得住它 */
	watch_strip_create(scr, WATCH_SCREEN_W, WATCH_SCREEN_H, 0, 0, watch_spo2_swipe_cb);

	/* 红色圆形按钮 */
	p_spo2->p_btn = lv_button_create(scr);
	lv_obj_set_size(p_spo2->p_btn, WATCH_SPO2_BTN_D, WATCH_SPO2_BTN_D);
	lv_obj_set_pos(p_spo2->p_btn, WATCH_SPO2_BTN_X, WATCH_SPO2_BTN_Y);
	lv_obj_set_style_radius(p_spo2->p_btn, LV_RADIUS_CIRCLE, 0);
	lv_obj_set_style_bg_color(p_spo2->p_btn, lv_color_hex(WATCH_COL_SPO2_RED), 0);
	lv_obj_set_style_pad_all(p_spo2->p_btn, 0, 0);
	/* 默认主题给按钮加了 shadow_width = LV_DPX(3). 阴影在"别再加回去"清单上
	   (见 lv_watch_ui.c 的设计约束 2), 显式关掉 */
	lv_obj_set_style_shadow_width(p_spo2->p_btn, 0, 0);
	lv_obj_add_event_cb(p_spo2->p_btn, watch_spo2_btn_cb, LV_EVENT_CLICKED, NULL);

	/* 圆内三行. 中文用14号(含汉字), 数值用20号(只需 ASCII) —— 靠显示/隐藏切换,
	   不在运行时换字体("开始"是汉字, montserrat 里没有) */
	p_spo2->p_idle = lv_label_create(p_spo2->p_btn);
	lv_label_set_text(p_spo2->p_idle, "开始");
	lv_obj_set_style_text_font(p_spo2->p_idle, &lv_font_alibaba_puhuiti_14, 0);
	lv_obj_set_style_text_color(p_spo2->p_idle, lv_color_hex(WATCH_COL_TEXT), 0);
	lv_obj_align(p_spo2->p_idle, LV_ALIGN_CENTER, 0, 0);

	p_spo2->p_val = lv_label_create(p_spo2->p_btn);
	lv_label_set_text(p_spo2->p_val, "0");
	lv_obj_set_style_text_font(p_spo2->p_val, &lv_font_montserrat_regular_20, 0);
	lv_obj_set_style_text_color(p_spo2->p_val, lv_color_hex(WATCH_COL_TEXT), 0);
	lv_obj_align(p_spo2->p_val, LV_ALIGN_CENTER, 0, -13);

	p_spo2->p_unit = lv_label_create(p_spo2->p_btn);
	lv_label_set_text(p_spo2->p_unit, "");
	lv_obj_set_style_text_font(p_spo2->p_unit, &lv_font_alibaba_puhuiti_14, 0);
	lv_obj_set_style_text_color(p_spo2->p_unit, lv_color_hex(WATCH_COL_TEXT), 0);
	lv_obj_align(p_spo2->p_unit, LV_ALIGN_CENTER, 0, 18);

	/* 本页刷新定时器(1s, 同心率页) */
	p_spo2->p_timer = lv_timer_create(watch_spo2_timer_cb, WATCH_TICK_MS, NULL);

	p_page->obj = scr;
	watch_spo2_render(p_spo2);
}

/******************************************************************************
 * @name    watch_spo2_destroy
 * @brief   销毁本页: 删本页定时器 + 置空控件句柄
 * @param   p_page[in] 页面基类指针
 *
 * @return  无
 *
 * @note    **必须删定时器**: lv_timer 不是 widget, lv_obj_delete 管不到它
 *****************************************************************************/
static void watch_spo2_destroy(page_base_t *p_page)
{
	watch_spo2_t *p_spo2 = (watch_spo2_t *)p_page;

	if (NULL != p_spo2->p_timer)
	{
		lv_timer_delete(p_spo2->p_timer);
		p_spo2->p_timer = NULL;
	}

	p_spo2->p_btn = NULL;
	p_spo2->p_idle = NULL;
	p_spo2->p_val = NULL;
	p_spo2->p_unit = NULL;
	p_spo2->p_status = NULL;
}

/******************************************************************************
 * @name    watch_spo2_show
 * @brief   本页变为可见: 立即补一帧 + 恢复本页定时器
 * @param   p_page[in] 页面基类指针
 *
 * @return  无
 *
 * @note    lv_timer_reset 不能省: lv_timer_resume 只清 paused 标志, 不重置周期
 * @note    ★切走后测量**不再保留**★ 原来的设计是"离开再回来保留进度"(定时器只是
 *          暂停), 那条保证随任务模型消失了: 切页时 APP 层会调 service_spo2_cancel(),
 *          任务投 IDLE 后自退, 再回来看到的是"开始". 想接着测只能重新按按钮
 *****************************************************************************/
static void watch_spo2_show(page_base_t *p_page)
{
	watch_spo2_t *p_spo2 = (watch_spo2_t *)p_page;

	watch_spo2_render(p_spo2);

	if (NULL != p_spo2->p_timer)
	{
		lv_timer_reset(p_spo2->p_timer);
		lv_timer_resume(p_spo2->p_timer);
	}
}

/******************************************************************************
 * @name    watch_spo2_hide
 * @brief   本页离开屏幕: 暂停本页定时器
 * @param   p_page[in] 页面基类指针
 *
 * @return  无
 *****************************************************************************/
static void watch_spo2_hide(page_base_t *p_page)
{
	watch_spo2_t *p_spo2 = (watch_spo2_t *)p_page;

	if (NULL != p_spo2->p_timer)
	{
		lv_timer_pause(p_spo2->p_timer);
	}
}

/******************************************************************************
 * @name    watch_page_spo2_register
 * @brief   把血氧页注册进管理器(不创建控件, 切过去时才建)
 * @param   p_mgr[in] 管理器实例
 *
 * @return  page_mgr_register 的返回码(0 成功; -1/-2/-3/-4/-5/-6 见 PageMem.h)
 *****************************************************************************/
int8_t watch_page_spo2_register(page_mgr_t *p_mgr)
{
	return page_mgr_register(p_mgr, &s_spo2.base, &s_spo2_vtable, WATCH_PAGE_ID_SPO2);
}
