/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file PageMem.h
 *
 * @par dependencies
 * - lvgl.h
 * - stdbool.h
 * - stdint.h
 *
 * @author	zw1194
 *
 * @brief LVGL 页面管理器: 页面切换接口 + 最近 N 页对象缓冲区(LRU 淘汰).
 *
 * Processing flow:
 *
 * 1. 每个页面一个静态实例, 其结构体**第一个成员必须是 page_base_t**;
 * 2. 页面实现 vtable(pf_create/pf_destroy/pf_show/pf_hide), 调
 *    page_mgr_register() 构造并注册;
 * 3. 切换: page_mgr_switch(p_page, anim) —— 应用层决定"去哪一页", 管理器只做
 *    四件事: 确保目标页有对象 / 把刚离开的页留在缓冲区 / 缓冲满则淘汰最久未用
 *    的页 / 发起切屏动画;
 * 4. 缓冲区: 最多 PAGE_MGR_BUF_SIZE(2)页同时持有 LVGL 对象(= 当前页 + 上一次
 *    显示过的页), 按**最近使用**排列 —— buf[0] 是正在显示的页, 越靠后越久没用.
 *    被淘汰的页**注册关系不变**(还在 pages[] 里), 只是 obj 变 NULL, 下次切到它时重建.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note ★本模块不含导航栈★
 *       没有 push/pop/pop_to, 没有"从哪来回哪去"的层级语义 —— 管理器**不记录**
 *       页面的调用链, 只记录"最近用过哪几页". 因此:
 *         1. 去哪一页完全由应用层决定(按钮/手势/事件回调里调 page_mgr_switch);
 *         2. 想回上一页调 page_mgr_switch_back(), 但它是"切回上一个显示过的页",
 *            不是历史栈(见其 @note);
 *         3. 应用层若需要真正的导航历史(如"设置→网络→WiFi→密码, 逐层返回"),
 *            请自己在应用层维护一个 id 数组, 用 page_mgr_find + page_mgr_switch
 *            实现 —— 那属于业务语义, 不该塞进页面管理器.
 *       好处是管理器职责被压到最小: **页面切换 + 对象生命周期**, 两者都不依赖
 *       "页面之间存在层级关系"这个假设.
 *
 * @note ★载体约定: 屏幕式★
 *       页面根对象是**独立屏幕**(pf_create 里 `p_page->obj = lv_obj_create(NULL)`),
 *       切页由管理器用 lv_screen_load_anim() 完成. switch 会**校验**这一点:
 *       pf_create 建出的根对象若有父对象(即建成了子控件), switch 返回 -6 并把它删掉.
 *       由此带来三条硬约束:
 *         1. 页面全屏独占, 页面之间**不能**共享状态栏/背景 —— 要共享请改用容器式
 *            载体, 但那需要改本文件的校验与删屏逻辑, 不是改一个宏就能切换的;
 *         2. 被缓存的页面**对象还活着**(只是不在屏上), 弹回它是秒回的、控件状态
 *            (滚动位置/选中项)保留; 被淘汰而重建的页面从零开始, pf_create 里
 *            重建的控件状态会丢 —— 要跨次保留的状态请放静态变量而非控件;
 *         3. 页面自己的 lv_timer **不受** lv_obj_delete 管(定时器不是 widget),
 *            必须 pf_destroy() 里自己删; 页面被缓存(CACHED)期间定时器仍在跑,
 *            pf_hide() 里应停掉刷新, pf_show() 再恢复.
 *
 * @note ★跨任务访问契约(必读)★
 *       `LV_USE_OS = LV_OS_NONE`, LVGL 内部**没有任何锁**. 本管理器会调
 *       lv_obj_xxx / lv_screen_load_anim, 因此**所有 API 只能在 "lvgl" 任务里调**
 *       —— 也就是 lv_timer_handler 的上下文, 与 widget 事件回调同一个任务
 *       (见 service/Lvgl/cywatch_service_Lvgl.h 的跨任务访问契约).
 *         1. 别的任务(AppCore/HeartRate/...)要切页, 必须经 osMessageQueue 把请求
 *            投给 lvgl 任务消费, **不能**直接调 page_mgr_switch;
 *         2. ISR 里禁止调用;
 *         3. 页面回调(pf_create/pf_destroy/pf_show/pf_hide)里**禁止**再发起切换
 *            —— 会被拒并返回 -4(重入保护); 需要"进来就跳走"这类逻辑时, 用
 *            lv_async_call / 一次性 lv_timer 延到下一帧.
 *
 * @note 命名说明(对 cywatch-bsp-driver skill 的有意偏离, 非笔误):
 *       本模块不是设备驱动(没有 iic/spi 依赖, 也就没有 `_inst()` 那套接口结构体),
 *       故文件名沿用页面侧的 PageMem; 但命名/返回码/注释规范与 skill 保持一致
 *       (类型 `_t`、指针 `p_`、函数指针 `pf_`、状态码 int8_t 的 0/-1/-2... 阶梯、
 *       NULL 常量在左、每个函数带 @name/@brief/@param/@return/@note 头块).
 ******************************************************************************/
#ifndef __LVGL_PAGE_MANAGER_H__
#define __LVGL_PAGE_MANAGER_H__

/***********************************Includes***********************************/
#include <stdbool.h>
#include <stdint.h>

#include "lvgl.h"

/***********************************Includes***********************************/

/***********************************Defines************************************/
/* 页面对象缓冲区容量: 同时最多这么多页面持有 LVGL 对象, 超出的按 LRU 淘汰.
   本值就是用户可见的"最多几页常驻" —— 调大 = 占 RAM 多/切换快, 调小 = 省 RAM/
   重建频繁.
   ★当前 2 页(2026-09-22 按用户决定由 3 改为 2)★ 语义正好是"当前页 + 上一次显示
   过的页": page_mgr_switch_back() 因此永远能秒回上一页, 而"上上次"那一页下次切回
   时必然走重建路径(重新 pf_create) —— 控件瞬时状态会丢, 要跨次保留的请放静态变量
   (本文件"载体约定"第 2 条, 工程里已有先例: lv_watch_ui.c 的模型数据).
   2 是本值的**最小合法值**: **切页动画**会让"正在离开的页"和"正在进入的页"同时
   存活, 取 1 会在动画还没走完时就把正在出场的那一屏删掉.
   ★收益口径(别记成"省了 RAM")★
     (a) `RW_IRAM1` 只少 4 字节 —— 就是 struct page_mgr 里 buf[3] → buf[2];
     (b) 真正的收益在 **LVGL 池(LV_MEM_SIZE)的峰值**: 少一页常驻 = 少一个屏幕子树
         + 它的字体/样式实例. 池本身是静态数组、不因此缩小, 所以除那 4 字节外
         **没有别的静态 RAM 回到 map 里**; 池峰值只有上板才量得出来.
   ★与 3 槽的一处真实行为差异(改值前已按 lvgl 源码推导, 依据见 PageMem.c 头)★
     "300ms 切页动画没走完就又切到第三页"时: 3 槽淘汰的是索引 2(空闲页), 2 槽淘汰
     的是索引 1 = **正在滑出屏幕的那一页** → 那次动画被提前截断、画面跳一下. */
#define PAGE_MGR_BUF_SIZE 2

/* 可注册页面总数上限(注册表容量, 与缓冲区容量无关) */
#define PAGE_MGR_MAX_PAGES 8

/* 切页动画时长(ms); anim 传 LV_SCR_LOAD_ANIM_NONE 时按瞬切处理, 忽略本值 */
#define PAGE_MGR_ANIM_MS 300

/***********************************Defines************************************/

/***********************************Declaring**********************************/

/* 页面生命周期状态 —— 全部由管理器维护, 页面只读 */
typedef enum
{
	PAGE_STATE_NONE = 0, /* 不在缓冲区: 无 LVGL 对象(obj == NULL), 下次切到时重建 */
	PAGE_STATE_CACHED,	 /* 在缓冲区里但不在屏上(仍在被缓存, 对象活着) */
	PAGE_STATE_SHOWN,	 /* 缓冲区队首, 正在显示 */
} page_state_t;

typedef struct page_base page_base_t;
typedef struct page_mgr page_mgr_t;

/**
 * @brief 页面虚函数表
 * @note  建议每个页面一张 const 静态表(放 flash); 同构页面(如列表项)可共用一张
 */
typedef struct
{
	/**
	 * @brief 创建页面控件 —— **必须实现**
	 * @param p_page[in] 页面基类指针
	 * @note  必须建**屏幕**(lv_obj_create(NULL))并把根屏幕写回 `p_page->obj`,
	 *        子树挂在这个根上; 建出带父对象的控件会被 switch 判为违约(返回 -6)
	 */
	void (*pf_create)(page_base_t *p_page);

	/**
	 * @brief 销毁页面 —— 可选
	 * @param p_page[in] 页面基类指针
	 * @note  由根对象的 LV_EVENT_DELETE 回调调起. 此时**子控件尚未被 LVGL 删除**
	 *        (LVGL 9.3 先发事件后递归删子节点), 可以读子控件句柄, 但**不要**自己
	 *        lv_obj_delete(会与 LVGL 的递归删除打架). 只做非 widget 资源清理:
	 *        lv_timer、malloc 的缓冲区、外部句柄, 并把页面自己的控件指针置 NULL.
	 */
	void (*pf_destroy)(page_base_t *p_page);

	/**
	 * @brief 页面变为可见后的通知 —— 可选
	 * @param p_page[in] 页面基类指针
	 * @note  切屏动画刚发起时调用(不等动画结束); 用来恢复刷新/定时器
	 */
	void (*pf_show)(page_base_t *p_page);

	/**
	 * @brief 页面离开屏幕前的通知 —— 可选
	 * @param p_page[in] 页面基类指针
	 * @note  被缓存的页面仍持有对象且定时器仍在跑, 停刷新的活儿在这里做
	 */
	void (*pf_hide)(page_base_t *p_page);
} page_vtable_t;

/**
 * @brief 页面基类 —— 各页面结构体的**第一个成员**必须是它
 * @note  成员偏移为 0, 所以回调里可以直接 `(my_page_t *)p_page` 强转回派生结构体,
 *        不需要 container_of 之类的偏移计算
 */
struct page_base
{
	lv_obj_t *obj;				 /* 页面根对象(屏幕); pf_create 建立, 管理器回收 */
	const page_vtable_t *vtable; /* 虚函数表 */
	page_mgr_t *p_mgr;			 /* 所属管理器(注册时回填, 兼作"是否已注册"判据) */
	void *p_user_data;			 /* 页面自定义数据(可选, 注册时置 NULL) */
	page_state_t state;			 /* 生命周期状态(管理器维护) */
	uint16_t page_id;			 /* 页面唯一 ID(switch_back / 查找用) */
};

/**
 * @brief 页面管理器实例
 * @note  约 44 字节(buf 8 = 2 槽 × 4 + 注册表 32 + 计数/标志 4); 实例建议放静态存储
 */
struct page_mgr
{
	page_base_t *buf[PAGE_MGR_BUF_SIZE]; /* 对象缓冲区: [0]=正在显示, 越靠后越久未用 */
	page_base_t *pages[PAGE_MGR_MAX_PAGES]; /* 注册表: 所有注册过的页面 */
	uint16_t page_cnt;						 /* 已注册页面数 */
	uint8_t buf_cnt;						 /* 缓冲区里持有对象的页数 */
	bool busy;								 /* 重入保护: 页面回调里再发起切换会被拒绝 */
};

/******************************************************************************
 * @name    page_mgr_init
 * @brief   初始化管理器: 清空缓冲区/注册表/计数(不创建任何页面对象)
 * @param   p_mgr[in] 管理器实例
 *
 * @return  0  success
 *         -1  p_mgr null
 *
 * @note    不碰任何 lv_obj, 但也**不会**回收之前留下的对象 —— 请在还没切过页
 *          (或已 page_mgr_clear)时调用, 否则那些页面对象会变成没人认领的野屏
 *****************************************************************************/
int8_t page_mgr_init(page_mgr_t *p_mgr);

/******************************************************************************
 * @name    page_mgr_register
 * @brief   构造并注册一个页面: 填 page_base_t 各字段 + 挂进注册表(不创建控件)
 * @param   p_mgr[in]    管理器实例
 * @param   p_page[in]   页面基类指针(必须是页面结构体的首成员)
 * @param   p_vtable[in] 页面虚函数表(通常指向 const 静态表)
 * @param   page_id[in]  页面唯一 ID
 *
 * @return  0  success
 *         -1  p_mgr null
 *         -2  p_page null
 *         -3  p_vtable null 或 p_vtable->pf_create null
 *         -4  注册表已满(PAGE_MGR_MAX_PAGES)
 *         -5  p_page 重复注册
 *         -6  page_id 与已注册页面冲突
 *
 * @note    注册只是登记, 页面控件在第一次 switch 到它时才创建
 *****************************************************************************/
int8_t page_mgr_register(page_mgr_t *p_mgr, page_base_t *p_page,
						 const page_vtable_t *p_vtable, uint16_t page_id);

/******************************************************************************
 * @name    page_mgr_switch
 * @brief   切换到指定页面: (必要时)创建对象 → 旧页留在缓冲区 → 满了淘汰最久未用
 *          → 切屏动画
 * @param   p_mgr[in]  管理器实例
 * @param   p_page[in] 目标页面(必须已 register 到本管理器)
 * @param   anim[in]   切屏动画; LV_SCR_LOAD_ANIM_NONE 为瞬切(且不产生动画延时)
 *
 * @return  0  success(含"目标页正在显示"的空操作)
 *         -1  p_mgr null
 *         -2  p_page null
 *         -3  p_page 未经本管理器注册
 *         -4  重入: 在页面回调里发起了切换
 *         -5  页面创建失败(pf_create 没有建出根对象)
 *         -6  pf_create 建出的根对象有父对象(载体约定要求建成独立屏幕)
 *
 * @note    先在缓冲区里找目标页:
 *           * 命中(还在被缓存) → 只做"移到队首", 不淘汰任何页 —— 这是"回上一页
 *             秒开、控件状态还在"的原因;
 *           * 未命中且缓冲区已满 → **淘汰队尾**(最久未用的那一页): 立刻
 *             lv_obj_delete 它的屏幕, 然后目标页插到队首.
 *         被淘汰的页仍在注册表里, 下次切到它会走 pf_create 重建.
 * @note    **失败的顺序保证**: 先建目标页, 成功之后才动旧页与缓冲区, 所以
 *         返回 -5/-6 时当前显示页、缓冲区、动画全部保持原样(不会黑屏或丢页)
 * @note    auto_del 恒为 false: 页面的销毁时机**只**由本管理器决定, LVGL 不会
 *          在动画结束时偷偷删掉页面 —— 否则"留在缓冲区"就成了空话
 *****************************************************************************/
int8_t page_mgr_switch(page_mgr_t *p_mgr, page_base_t *p_page,
					   lv_screen_load_anim_t anim);

/******************************************************************************
 * @name    page_mgr_switch_back
 * @brief   切回"上一个显示过的页"(缓冲区里排第二的那一页)
 * @param   p_mgr[in] 管理器实例
 * @param   anim[in]  切屏动画
 *
 * @return  0  success
 *         -1  p_mgr null
 *         -2  缓冲区里不足 2 页(还没切过页)
 *         -3  内部 page_mgr_switch 失败(返回码见其 @return)
 *
 * @note    ⚠ **这不是历史栈**: 它只是"回到上一个显示过的页", 连按两次会在
 *          最近两页之间来回切(A→B→A→B), 而不是逐层返回. 需要真正的多级返回
 *          (设置→网络→WiFi→密码), 请在应用层自己维护 id 数组 + page_mgr_find
 *****************************************************************************/
int8_t page_mgr_switch_back(page_mgr_t *p_mgr, lv_screen_load_anim_t anim);

/******************************************************************************
 * @name    page_mgr_clear
 * @brief   销毁**所有**缓冲中页面的 LVGL 对象, 缓冲区清空(注册关系保留)
 * @param   p_mgr[in] 管理器实例
 *
 * @return  无
 *
 * @note    ⚠ 会连当前显示页一起删掉, 之后 LVGL 的活动屏幕为 NULL(空屏, 不崩,
 *          见 lv_obj_tree.c 对 act_scr 的置空处理). 所以调用方紧接着应 switch
 *          一个页面, 或者这本来就是关机/休眠前的收尾
 * @note    删除顺序是"从队尾到队首", 正在显示的那一页**最后**被删
 * @note    被删页面的 pf_hide **不会**被调用(删对象不走 unload 流程), 有收尾
 *          动作请放在 pf_destroy 里
 *****************************************************************************/
void page_mgr_clear(page_mgr_t *p_mgr);

/******************************************************************************
 * @name    page_mgr_current
 * @brief   取当前正在显示的页面
 * @param   p_mgr[in] 管理器实例
 *
 * @return  栈顶页面指针; p_mgr null 或还没切过页时返回 NULL
 *****************************************************************************/
page_base_t *page_mgr_current(const page_mgr_t *p_mgr);

/******************************************************************************
 * @name    page_mgr_buf_at
 * @brief   按"最近使用"顺序取缓冲区里的第 index 页
 * @param   p_mgr[in]   管理器实例
 * @param   index[in]   0 = 正在显示的页, 1 = 上一个显示过的, 依此类推
 *
 * @return  页面指针; 越界/p_mgr null 返回 NULL
 *
 * @note    给调试打印/看护UI用(例如把 "BUF: A B C" 显示到屏幕上)
 *****************************************************************************/
page_base_t *page_mgr_buf_at(const page_mgr_t *p_mgr, uint8_t index);

/******************************************************************************
 * @name    page_mgr_buf_count
 * @brief   取缓冲区里持有对象的页数
 * @param   p_mgr[in] 管理器实例
 *
 * @return  页数(0..PAGE_MGR_BUF_SIZE); p_mgr null 时返回 0
 *****************************************************************************/
uint8_t page_mgr_buf_count(const page_mgr_t *p_mgr);

/******************************************************************************
 * @name    page_mgr_find
 * @brief   按 page_id 在注册表里查页面
 * @param   p_mgr[in]   管理器实例
 * @param   page_id[in] 页面 ID
 *
 * @return  页面指针; 未注册/找不到返回 NULL
 *****************************************************************************/
page_base_t *page_mgr_find(const page_mgr_t *p_mgr, uint16_t page_id);

/***********************************Declaring**********************************/

#endif /* __LVGL_PAGE_MANAGER_H__ */
