/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file lv_watch_page.h
 *
 * @par dependencies
 * - lvgl.h
 * - ../ui_manger/PageMem.h
 *
 * @author zw1194
 *
 * @brief 手表UI各页面的**内部共享头**: 页面地址(ID)/调色板/共用工具/模型访问器
 *        + 各页自己的注册函数. 只有 LVGL/port 下的手表UI系列文件包含它, 是
 *        模块内部接口, **不是**对外 API(对外只有 lv_watch_ui.h 的 lv_watch_ui_init).
 *
 * Processing flow:
 *
 * 文件分工(每页一个 .c, 实现细节各自私有, 对外只暴露一个 register):
 *
 *   lv_watch_ui.c        UI入口: 管理器 + 模型数据(时钟/步数/心率) + 共用工具
 *                        (切页/手势条/拖拽方向) + lv_watch_ui_init()
 *   lv_watch_page_home.c 表盘页 0x0300 —— 时钟 + 三张健康卡片
 *   lv_watch_page_menu.c 菜单页 0x0301 —— 2x2 功能按钮 + 背光滑条(纯静态, 无定时器)
 *   lv_watch_page_heart.c心率页 0x0302 —— 数值 + 30点折线图
 *   lv_watch_page_spo2.c 血氧页 0x0303 —— 红色圆按钮 + 三态状态机(最长3分钟)
 *   lv_watch_page_ota.c  OTA页  0x0304 —— 进度条 + **唯一一个**按钮. ★纯显示, 无状态★
 *                        (2026-09-22 起): 文案/可点性/色系/进度条全部由 app_core 经
 *                        lv_watch_page_ota.h 的 watch_page_ota_post_*() 投值, 页面
 *                        自己的 100ms 定时器落地. 按钮点击只上报 EVT_SERVICE_OTA_BUTTON.
 *
 * 页面拓扑(2026-09-18 起是**环形闭环**, 不再全是 star):
 *
 *            [MENU]                 左→右 = OTA, HOME, HEART, SPO2
 *              │ 下滑/上滑
 *   [OTA] ── [HOME] ── [HEART] ── [SPO2]
 *     └──────────────────────────────┘          (环: OTA 左滑→SPO2, SPO2 右滑→OTA)
 *
 *   只有 HOME 有两条水平边(左右各一), HEART 是"表盘→心率→血氧"那段的老节点;
 *   每页仍旧**只有一个手势条**, 方向由各页自己的回调映射成"切到哪页".
 *
 * 数据流: lv_watch_ui.c 的模型定时器(1s)推进模型 → 各页的渲染定时器(1s)读模型
 *         并刷新自己的控件. 页面**只读**模型, 不推进 —— 见下方模型访问器.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note ★加一页要动的地方(就这两处)★
 *       1. 新建 lv_watch_page_xxx.c, 参照 home/menu/heart 写: 结构体首成员必须是
 *          page_base_t, 实现 pf_create(必需) + 需要的 pf_show/pf_hide/pf_destroy,
 *          文件末尾给一个 watch_page_xxx_register();
 *       2. 在本文件里加它的 ID 宏 + register 声明, 在 lv_watch_ui.c 的
 *          lv_watch_ui_init() 里加一行注册, 在 watch_page_name() 里加一个名字
 *          (只影响日志).
 *       页面**不需要**自己调 page_mgr_register 之外的任何管理器 API; 切页一律
 *       走 watch_switch().
 ******************************************************************************/
#ifndef __LV_WATCH_PAGE_H__
#define __LV_WATCH_PAGE_H__

/***********************************Includes***********************************/
#include <stdint.h>

#include "lvgl.h"

#include "../ui_manger/PageMem.h"

/* 硬件 RTC(system/Rtc). 本目录第一次 include system/ 下的东西, 可以接受的理由:
   cywatch_rtc.h 是**刻意 HAL-free** 的公开头(它自己的 @par dependencies 只列
   stdint.h), 不会把 HAL 拖进 UI 层 */
#include "cywatch_rtc.h"
/***********************************Includes***********************************/

/***********************************Defines************************************/
/* 屏幕与节拍 */
#define WATCH_SCREEN_W      240 /* 面板宽(ST7789 240x280) */
#define WATCH_SCREEN_H      280 /* 面板高 */
#define WATCH_TICK_MS       1000 /* 模型推进与页面刷新的节拍(1s) */

/* 页面 ID: 切页用的"地址", 全局唯一(见 PageMem.h 的 page_id) */
#define WATCH_PAGE_ID_HOME  0x0300u /* 表盘页 */
#define WATCH_PAGE_ID_MENU  0x0301u /* 菜单页 */
#define WATCH_PAGE_ID_HEART 0x0302u /* 心率页 */
#define WATCH_PAGE_ID_SPO2  0x0303u /* 血氧检测页(心率页右滑进入) */
#define WATCH_PAGE_ID_OTA   0x0304u /* OTA升级页(表盘页左滑 / 血氧页右滑 进入) */

/* 全UI通用配色(各页自己的专属色定义在各自的 .c 里) */
#define WATCH_COL_TEXT      0xFFFFFF /* 主文字 */
#define WATCH_COL_GRAY      0x9A9A9A /* 次级文字(日期/卡片标题/BPM) */
#define WATCH_COL_DARK      0x0A0A0A /* menu/heart 页底色(纯黑留给表盘) */
/***********************************Defines************************************/

/**********************************Declaring***********************************/
/* 18x18 图标(实现在 LVGL/assets/image, LVGL9.3无LV_SYMBOL_HEART/步数/火焰等
   专用符号, 只能用图片). 放在本头里是因为被两页以上共用: home 用全部三张,
   menu/heart 用心形. */
extern const lv_image_dsc_t icon_steps_18x18_RGB565A8_NONE;
extern const lv_image_dsc_t icon_heart_18x18_RGB565A8_NONE;
extern const lv_image_dsc_t icon_flame_18x18_RGB565A8_NONE;

/* 拖拽方向: 手势条回调算出方向, 由**页面自己**映射成"切到哪一页 + 什么动画" */
typedef enum
{
	WATCH_SWIPE_NONE = 0,
	WATCH_SWIPE_LEFT,
	WATCH_SWIPE_RIGHT,
	WATCH_SWIPE_UP,
	WATCH_SWIPE_DOWN,
} watch_swipe_dir_t;

/******************************************************************************
 * @name    watch_switch
 * @brief   切页: 按 id 找页面 -> page_mgr_switch -> 打日志(实现在 lv_watch_ui.c)
 * @param   page_id[in] 目标页面 ID(WATCH_PAGE_ID_xxx)
 * @param   anim[in]    切屏动画
 *
 * @return  无(失败只打日志: 页面回调里已无事务可回滚, 见其实现)
 *
 * @note    这是各页**唯一**的导航手段. 页面里禁止直接调 page_mgr_switch —— 那样
 *          得自己 page_mgr_find + 判返回码 + 打日志, 三份重复.
 * @note    ⚠ 它最终会调 page_mgr_switch, 所以**不能在 pf_create/pf_show/pf_hide/
 *          pf_destroy 里调**(重入, 返回 -4 打日志); 要"进来就跳走"请用
 *          lv_async_call 延到下一帧.
 *****************************************************************************/
void watch_switch(uint16_t page_id, lv_screen_load_anim_t anim);

/******************************************************************************
 * @name    watch_strip_create
 * @brief   给页面创建手势条(透明, 置顶), 并绑定该页自己的拖拽回调
 * @param   parent[in] 页面根对象
 * @param   w[in]     宽度
 * @param   h[in]     高度
 * @param   x[in]     X位置
 * @param   y[in]     Y位置
 * @param   pf_cb[in] 该页的拖拽回调(每页一张映射表, 回调里不必判断"当前是哪页")
 *
 * @return  无
 *
 * @note    每页**只建一个**手势条(绑 LV_EVENT_ALL 到多个对象是早期切页卡死的
 *          嫌疑点之一, 别加回去)
 *****************************************************************************/
void watch_strip_create(lv_obj_t *parent, lv_coord_t w, lv_coord_t h,
						lv_coord_t x, lv_coord_t y, lv_event_cb_t pf_cb);

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
watch_swipe_dir_t watch_swipe_track(lv_event_t *e);

/* ---- 模拟数据(只读) --------------------------------------------------------
   这几个是**模拟量**(假数据): 与"哪一页在显示"无关, 由 lv_watch_ui.c 的模型定时器
   每秒推进一次. 页面只读它并画出来 —— 这样页面被 LRU 淘汰重建后, 接着画的是**最新值**
   而不是从头开始(见 PageMem.h"要跨次保留的状态请放静态变量").

   @note 用访问器而不是 extern 变量: 模型可能在推进中途被读(都在 lvgl 任务里,
         实际不会并发), 但更重要的是**只读语义要写在接口上** —— 页面不该推模型. */
uint32_t watch_model_steps(void); /* 步数 */
uint32_t watch_model_kcal(void);  /* 卡路里 */

/* ---- 真实时间源(硬件 RTC) --------------------------------------------------
   日期与大字时钟的**唯一**来源(表盘页用). 与上面那几个模拟量不同, 本函数**穿透到
   硬件**: 每次调用都真的读一次 RTC 外设(内部 GetTime+GetDate 一对), 所以返回值逐秒
   变化, 且与"哪一页在显示"无关.

   @note 之所以仍包一层、不让页面直接调 cywatch_rtc_get(): 页面只认模型层这一个数据
         出口, 将来换时间源(协议对时 / 外部 RTC)不用改任何页面.
   @note **不可重入**(见 cywatch_rtc.h). 当前唯一调用者是 lvgl 任务(页面渲染与自检都在
         lv_timer_handler 上下文), 不存在并发; 将来多任务要读需自备互斥.
   @return 0 成功(填 *p_time); -1 p_time 为 NULL; -2 RTC 未初始化/不可用;
           -3/-4 读寄存器失败(纯防御). 详见 cywatch_rtc.h */
int8_t watch_model_datetime(cywatch_rtc_time_t *p_time);

/* ---- 各页自己的注册入口(每页一个 .c, 实现细节各自私有) ---- */

/******************************************************************************
 * @name    watch_page_home_register
 * @brief   把表盘页注册进管理器(不创建控件, 切过去时才建)
 * @param   p_mgr[in] 管理器实例
 *
 * @return  page_mgr_register 的返回码, 见 PageMem.h
 *****************************************************************************/
int8_t watch_page_home_register(page_mgr_t *p_mgr);

/******************************************************************************
 * @name    watch_page_menu_register
 * @brief   把菜单页注册进管理器(不创建控件, 切过去时才建)
 * @param   p_mgr[in] 管理器实例
 *
 * @return  page_mgr_register 的返回码, 见 PageMem.h
 *****************************************************************************/
int8_t watch_page_menu_register(page_mgr_t *p_mgr);

/******************************************************************************
 * @name    watch_page_heart_register
 * @brief   把心率页注册进管理器(不创建控件, 切过去时才建)
 * @param   p_mgr[in] 管理器实例
 *
 * @return  page_mgr_register 的返回码, 见 PageMem.h
 *****************************************************************************/
int8_t watch_page_heart_register(page_mgr_t *p_mgr);

/******************************************************************************
 * @name    watch_page_spo2_register
 * @brief   把血氧检测页注册进管理器(不创建控件, 切过去时才建)
 * @param   p_mgr[in] 管理器实例
 *
 * @return  page_mgr_register 的返回码, 见 PageMem.h
 *****************************************************************************/
int8_t watch_page_spo2_register(page_mgr_t *p_mgr);

/******************************************************************************
 * @name    watch_page_ota_register
 * @brief   把 OTA 升级页注册进管理器(不创建控件, 切过去时才建)
 * @param   p_mgr[in] 管理器实例
 *
 * @return  page_mgr_register 的返回码, 见 PageMem.h
 *****************************************************************************/
int8_t watch_page_ota_register(page_mgr_t *p_mgr);

/**********************************Declaring***********************************/

#endif /* __LV_WATCH_PAGE_H__ */
