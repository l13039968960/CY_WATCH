/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file PageMem.c
 *
 * @par dependencies
 * - PageMem.h
 *
 * @author	zw1194
 *
 * @brief LVGL 页面管理器实现: 页面切换 + 最近 N 页对象缓冲区(LRU).
 *
 * Processing flow:
 *
 * page_mgr_switch(p_page):
 *   1. 校验(注册过 / 不在重入中 / 不是当前页)
 *   2. page_mgr_ensure_created(): obj 为 NULL 才调 pf_create, 建完挂删除事件
 *      —— 失败直接返回, 此时还没动过任何既有状态
 *   3. 在 buf[] 里找目标页:
 *        命中 at i>0 → 把 [0, i-1] 右移一格, 目标页放到 buf[0]  (不淘汰)
 *        未命中且满 → page_mgr_delete_obj(队尾) 淘汰最久未用的一页, 再插入队首
 *        未命中未满 → 直接插入队首
 *   4. 刷新各页 state(SHOWN/CACHED)
 *   5. pf_hide(旧页) → lv_screen_load_anim(auto_del = false) → pf_show(新页)
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note ★设计要点: 只有一个页面回收入口★
 *       无论是 LRU 淘汰还是 page_mgr_clear, 页面的销毁都走 lv_obj_delete, 而
 *       LVGL 会在删对象时同步发 LV_EVENT_DELETE —— 于是所有销毁路径都汇进
 *       page_mgr_root_delete_cb(), 在那里统一做: 摘缓冲区 → 清 obj → pf_destroy.
 *       好处是不需要"待删队列"或自建定时器; 哪怕有人绕过管理器直接
 *       lv_obj_delete(页面根屏幕), 缓冲区也照样收敛(不会留下悬空指针).
 *
 * @note ★为什么 auto_del 恒为 false★
 *       lv_screen_load_anim 的 auto_del=true 会在**动画结束时**删掉切屏前的那个
 *       屏幕, 看起来正好是我们要的"延迟销毁", 但它删的是"上一次的上一屏",
 *       和我们的缓冲区账本对不上(动画期间连点两次就会漏账). 本管理器自己管
 *       删除时机: 淘汰队尾时立即删, 其余一律保留 —— LVGL 只负责画动画.
 *
 * @note ★为什么删被缓存的页面是安全的★
 *       淘汰的永远是队尾, 而正在显示的那一屏是队首 buf[0] —— 所以永远删不到活动
 *       屏幕. 即便被删页面上还挂着动画, lv_obj_destructor() 里的
 *       lv_anim_delete(obj, NULL) 会把它摘掉, 不留野动画.
 *
 * @note ★PAGE_MGR_BUF_SIZE == 2 时, 上一条要按这两点重新读★
 *       (3 槽时的原推理是"淘汰索引 >= 2 ⇒ 那一格必然是空闲页", 2 槽淘汰的是索引 1,
 *        而索引 1 在**动画进行中**恰好就是正在出场的那一屏 —— 推理不再成立.)
 *         1. 它**确实**会被提前删掉: 300ms 动画没走完再切到第三个页时, 那次动画被
 *            截断、画面跳一下. 这是可见的行为变化, 不是崩溃, 也不需要加锁.
 *         2. 之所以只是"跳一下"而不是野指针 —— 两条都读过源码:
 *            · LVGL 一次切屏起**两条**动画: a_new(var = 进场页) 与 a_old(var = 出场
 *              页, 见 lv_display.c 的 lv_screen_load_anim). 被删的那个页面正是
 *              a_old 的 var ⇒ 随 lv_obj_destructor 的 lv_anim_delete(obj, NULL) 一起消失.
 *            · lv_anim_delete() 只回调 deleted_cb、**不回调 completed_cb**(lv_anim.c),
 *              而切屏这两条只设了 start_cb/completed_cb ⇒ 动画被截断**不会**反过来
 *              执行 scr_anim_completed() 去读 lv_display 里的 prev_scr.
 *              何况删页之后紧接着的 lv_screen_load_anim 自己就会把 prev_scr 清成
 *              NULL, 而且清之前只把它当布尔量判断、从不解引用 —— 因为本管理器
 *              auto_del 恒为 false, 那句 `lv_obj_delete(d->prev_scr)` 永远走不到.
 ******************************************************************************/
#include "PageMem.h"

/**********************************Declaring***********************************/
/* 内部静态函数 ——— 定义在本文件后半部分, 这里集中前置声明 */
static void page_mgr_buf_remove(page_mgr_t *p_mgr, page_base_t *p_page);
static void page_mgr_refresh_state(page_mgr_t *p_mgr);
static void page_mgr_root_delete_cb(lv_event_t *p_event);
static void page_mgr_delete_obj(page_mgr_t *p_mgr, page_base_t *p_page);
static int8_t page_mgr_ensure_created(page_base_t *p_page);
static uint32_t page_mgr_anim_time(lv_screen_load_anim_t anim);
/**********************************Declaring***********************************/

/******************************************************************************
 * @name    page_mgr_anim_time
 * @brief   把切屏动画类型翻译成动画时长
 * @param   anim[in] LVGL 切屏动画类型
 *
 * @return  动画时长(ms); LV_SCR_LOAD_ANIM_NONE 时为 0
 *
 * @note    LVGL 在 time==0 && delay==0 时走"立即切屏"的短路分支, 所以瞬切既
 *          不产生动画, 也不会有 300ms 的延迟感
 *****************************************************************************/
static uint32_t page_mgr_anim_time(lv_screen_load_anim_t anim)
{
	return (LV_SCR_LOAD_ANIM_NONE == anim) ? 0u : (uint32_t)PAGE_MGR_ANIM_MS;
}

/******************************************************************************
 * @name    page_mgr_refresh_state
 * @brief   按缓冲区顺序重算各页 state: buf[0] = SHOWN, 其余 = CACHED
 * @param   p_mgr[in] 管理器实例
 *
 * @return  无
 *
 * @note    状态的唯一真相是"它在缓冲区里的位置", 所以每次动完 buf[] 都整体重算,
 *          比在各个分支里分别维护 state 更不容易错
 *****************************************************************************/
static void page_mgr_refresh_state(page_mgr_t *p_mgr)
{
	uint8_t i = 0;

	for (i = 0; i < p_mgr->buf_cnt; i++)
	{
		p_mgr->buf[i]->state = (0 == i) ? PAGE_STATE_SHOWN : PAGE_STATE_CACHED;
	}
}

/******************************************************************************
 * @name    page_mgr_buf_remove
 * @brief   把一个页面从缓冲区里摘掉(页数减一, 后面的页前移), 幂等
 * @param   p_mgr[in]  管理器实例
 * @param   p_page[in] 页面
 *
 * @return  无
 *
 * @note    不在缓冲区时直接返回 —— 因此可以在删除回调里放心调用(管理器自己
 *          发起的删除已经先摘过一次了)
 *****************************************************************************/
static void page_mgr_buf_remove(page_mgr_t *p_mgr, page_base_t *p_page)
{
	uint8_t i = 0;
	uint8_t idx = 0;

	if (NULL == p_mgr || NULL == p_page)
	{
		return;
	}

	/* 查下标: 找不到就用 buf_cnt 当哨兵 */
	idx = p_mgr->buf_cnt;
	for (i = 0; i < p_mgr->buf_cnt; i++)
	{
		if (p_mgr->buf[i] == p_page)
		{
			idx = i;
			break;
		}
	}
	if (idx >= p_mgr->buf_cnt)
	{
		return;
	}

	for (i = idx; (uint8_t)(i + 1u) < p_mgr->buf_cnt; i++)
	{
		p_mgr->buf[i] = p_mgr->buf[i + 1u];
	}
	p_mgr->buf_cnt--;
	p_mgr->buf[p_mgr->buf_cnt] = NULL;

	p_page->state = PAGE_STATE_NONE;
}

/******************************************************************************
 * @name    page_mgr_root_delete_cb
 * @brief   页面根对象的 LV_EVENT_DELETE 回调 —— 本模块**唯一**的页面回收入口
 * @param   p_event[in] LVGL 事件(目标 = 页面根屏幕, user_data = page_base_t*)
 *
 * @return  无
 *
 * @note    触发时机有两类: 管理器淘汰队尾时主动 lv_obj_delete, 或页面自己/
 *          别人直接删了这个屏幕. 两条路都在这里收敛
 * @note    LVGL 9.3 的顺序是**先发本事件、之后才递归删子控件**, 所以此刻子控件
 *          句柄仍然有效(pf_destroy 可以读), 但**不能**自己删它们
 *****************************************************************************/
static void page_mgr_root_delete_cb(lv_event_t *p_event)
{
	lv_obj_t *p_obj = (lv_obj_t *)lv_event_get_target(p_event);
	page_base_t *p_page = (page_base_t *)lv_event_get_user_data(p_event);

	if (NULL == p_obj || NULL == p_page)
	{
		return;
	}

	/* 1. 从缓冲区摘账(幂等) */
	if (NULL != p_page->p_mgr)
	{
		page_mgr_buf_remove(p_page->p_mgr, p_page);
	}

	/* 2. 清根指针 —— 不变式: state == PAGE_STATE_NONE 等价于 obj == NULL */
	if (p_obj == p_page->obj)
	{
		p_page->obj = NULL;
	}

	/* 3. 页面自己的非 widget 资源(lv_timer / 缓冲区 / 外部句柄)在这里释放 */
	if (NULL != p_page->vtable && NULL != p_page->vtable->pf_destroy)
	{
		p_page->vtable->pf_destroy(p_page);
	}

	p_page->state = PAGE_STATE_NONE;
}

/******************************************************************************
 * @name    page_mgr_delete_obj
 * @brief   销毁一个页面的 LVGL 对象(先摘缓冲区, 再 lv_obj_delete)
 * @param   p_mgr[in]  管理器实例
 * @param   p_page[in] 页面
 *
 * @return  无
 *
 * @note    顺序不能反: lv_obj_delete 会**同步**回调 page_mgr_root_delete_cb,
 *          那时缓冲区数组必须已经收敛, 否则回调会在数组挪动到一半时再改数组
 * @note    真正的清理(obj=NULL / pf_destroy / state=NONE)在回调里做, 所以本函数
 *          只管"摘账 + 删对象"
 *****************************************************************************/
static void page_mgr_delete_obj(page_mgr_t *p_mgr, page_base_t *p_page)
{
	lv_obj_t *p_obj = NULL;

	if (NULL == p_mgr || NULL == p_page)
	{
		return;
	}

	p_obj = p_page->obj;

	page_mgr_buf_remove(p_mgr, p_page);

	if (NULL != p_obj)
	{
		lv_obj_delete(p_obj);
	}
	else
	{
		p_page->state = PAGE_STATE_NONE;
	}
}

/******************************************************************************
 * @name    page_mgr_ensure_created
 * @brief   确保页面持有 LVGL 对象: 没有就调 pf_create 建, 建完挂删除回调
 * @param   p_page[in] 页面
 *
 * @return  0   已经有对象(命中缓冲)或本次创建成功
 *         -5  pf_create 没有建出根对象(p_page->obj 仍为 NULL)
 *         -6  建出的根对象有父对象 —— 载体约定要求页面根是独立屏幕
 *
 * @note    返回码直接沿用 page_mgr_switch 的阶梯(本函数只有它一个调用者), 不再
 *          另立一套再翻译一遍
 * @note    -6 时会把违规建出来的对象删掉再返回, 不留野控件(此时还没挂删除回调,
 *          所以这次删除不会误触发 pf_destroy)
 *****************************************************************************/
static int8_t page_mgr_ensure_created(page_base_t *p_page)
{
	if (NULL != p_page->obj)
	{
		return 0; /* 还在缓冲区里, 直接复用 */
	}

	if (NULL != p_page->vtable && NULL != p_page->vtable->pf_create)
	{
		p_page->vtable->pf_create(p_page);
	}

	if (NULL == p_page->obj)
	{
		return -5;
	}

	if (NULL != lv_obj_get_parent(p_page->obj))
	{
		lv_obj_delete(p_page->obj);
		p_page->obj = NULL;
		return -6;
	}

	/* 挂回收回调: 此后这个屏幕无论被谁删, 账本都会在回调里收敛 */
	lv_obj_add_event_cb(p_page->obj, page_mgr_root_delete_cb, LV_EVENT_DELETE, p_page);
	p_page->state = PAGE_STATE_CACHED;

	return 0;
}

/******************************************************************************
 * @name    page_mgr_init
 * @brief   初始化管理器: 清空缓冲区/注册表/计数
 * @param   p_mgr[in] 管理器实例
 *
 * @return  0  success
 *         -1  p_mgr null
 *****************************************************************************/
int8_t page_mgr_init(page_mgr_t *p_mgr)
{
	uint8_t i = 0;

	if (NULL == p_mgr)
	{
		return -1;
	}

	for (i = 0; i < PAGE_MGR_BUF_SIZE; i++)
	{
		p_mgr->buf[i] = NULL;
	}
	for (i = 0; i < PAGE_MGR_MAX_PAGES; i++)
	{
		p_mgr->pages[i] = NULL;
	}

	p_mgr->page_cnt = 0;
	p_mgr->buf_cnt = 0;
	p_mgr->busy = false;

	return 0;
}

/******************************************************************************
 * @name    page_mgr_register
 * @brief   构造并注册一个页面(只登记, 不创建控件)
 * @param   p_mgr[in]    管理器实例
 * @param   p_page[in]   页面基类指针(必须是页面结构体的首成员)
 * @param   p_vtable[in] 页面虚函数表
 * @param   page_id[in]  页面唯一 ID
 *
 * @return  0  success
 *         -1  p_mgr null
 *         -2  p_page null
 *         -3  p_vtable null 或 p_vtable->pf_create null
 *         -4  注册表已满(PAGE_MGR_MAX_PAGES)
 *         -5  p_page 重复注册
 *         -6  page_id 与已注册页面冲突
 *****************************************************************************/
int8_t page_mgr_register(page_mgr_t *p_mgr, page_base_t *p_page,
						 const page_vtable_t *p_vtable, uint16_t page_id)
{
	uint16_t i = 0;

	if (NULL == p_mgr)
	{
		return -1;
	}
	if (NULL == p_page)
	{
		return -2;
	}
	if (NULL == p_vtable || NULL == p_vtable->pf_create)
	{
		return -3;
	}
	if (p_mgr->page_cnt >= PAGE_MGR_MAX_PAGES)
	{
		return -4;
	}

	for (i = 0; i < p_mgr->page_cnt; i++)
	{
		if (p_mgr->pages[i] == p_page)
		{
			return -5;
		}
		if (p_mgr->pages[i]->page_id == page_id)
		{
			return -6;
		}
	}

	/* 构造: 把基类字段填齐(pf_create 里页面自己会把控件指针填进派生成员) */
	p_page->obj = NULL;
	p_page->vtable = p_vtable;
	p_page->p_mgr = p_mgr;
	p_page->p_user_data = NULL;
	p_page->state = PAGE_STATE_NONE;
	p_page->page_id = page_id;

	p_mgr->pages[p_mgr->page_cnt] = p_page;
	p_mgr->page_cnt++;

	return 0;
}

/******************************************************************************
 * @name    page_mgr_switch
 * @brief   切换到指定页面: 必要时创建 → 旧页留在缓冲区 → 满了淘汰最久未用 →
 *          切屏动画
 * @param   p_mgr[in]  管理器实例
 * @param   p_page[in] 目标页面(必须已注册到本管理器)
 * @param   anim[in]   切屏动画
 *
 * @return  0  success(含"目标页正在显示"的空操作)
 *         -1  p_mgr null
 *         -2  p_page null
 *         -3  p_page 未经本管理器注册
 *         -4  重入: 在页面回调里发起了切换
 *         -5  页面创建失败(pf_create 没有建出根对象)
 *         -6  pf_create 建出的根对象有父对象(必须是独立屏幕)
 *****************************************************************************/
int8_t page_mgr_switch(page_mgr_t *p_mgr, page_base_t *p_page,
					   lv_screen_load_anim_t anim)
{
	int8_t ret = 0;
	uint8_t i = 0;
	uint8_t ins = 0;
	page_base_t *p_old = NULL;

	if (NULL == p_mgr)
	{
		return -1;
	}
	if (NULL == p_page)
	{
		return -2;
	}
	/* p_mgr 字段就是"注册到哪个管理器"的凭据, 顺带挡住别的管理器的页面 */
	if (p_mgr != p_page->p_mgr)
	{
		return -3;
	}

	/* 目标页正在显示: 空操作(放行重入检查, 免得回调里"切到自己"被当成错误) */
	if (0 != p_mgr->buf_cnt && p_mgr->buf[0] == p_page)
	{
		return 0;
	}

	if (true == p_mgr->busy)
	{
		return -4;
	}
	p_mgr->busy = true;

	/* 1. 先建/复用目标页对象 —— 失败就原样退出, 当前显示页不受影响(不黑屏) */
	ret = page_mgr_ensure_created(p_page);

	if (0 == ret)
	{
		p_old = (0 != p_mgr->buf_cnt) ? p_mgr->buf[0] : NULL;

		/* 2. 在缓冲区里找目标页 */
		ins = p_mgr->buf_cnt;
		for (i = 0; i < p_mgr->buf_cnt; i++)
		{
			if (p_mgr->buf[i] == p_page)
			{
				ins = i;
				break;
			}
		}

		/* 3. 未命中且缓冲区已满: 淘汰队尾(最久未用)那一页, 腾出一格 */
		if (p_mgr->buf_cnt == ins && p_mgr->buf_cnt >= PAGE_MGR_BUF_SIZE)
		{
			page_mgr_delete_obj(p_mgr, p_mgr->buf[p_mgr->buf_cnt - 1u]);
			ins = p_mgr->buf_cnt;
		}

		/* 4. 把 [0, ins-1] 右移一格, 目标页放到队首(命中时 ins < buf_cnt, 页数不变) */
		for (i = ins; i > 0; i--)
		{
			p_mgr->buf[i] = p_mgr->buf[i - 1u];
		}
		p_mgr->buf[0] = p_page;
		if (p_mgr->buf_cnt == ins)
		{
			p_mgr->buf_cnt++;
		}

		page_mgr_refresh_state(p_mgr);

		/* 5. 旧页停刷新 → 切屏 → 新页恢复刷新 */
		if (NULL != p_old && NULL != p_old->vtable && NULL != p_old->vtable->pf_hide)
		{
			p_old->vtable->pf_hide(p_old);
		}

		/* auto_del = false: 页面的销毁时机只由本管理器决定, 见文件头 @note */
		lv_screen_load_anim(p_page->obj, anim, page_mgr_anim_time(anim), 0, false);

		if (NULL != p_page->vtable && NULL != p_page->vtable->pf_show)
		{
			p_page->vtable->pf_show(p_page);
		}
	}

	p_mgr->busy = false;
	return ret;
}

/******************************************************************************
 * @name    page_mgr_switch_back
 * @brief   切回缓冲区里排第二的那一页(上一个显示过的页)
 * @param   p_mgr[in] 管理器实例
 * @param   anim[in]  切屏动画
 *
 * @return  0  success
 *         -1  p_mgr null
 *         -2  缓冲区里不足 2 页(还没切过页)
 *         -3  内部 page_mgr_switch 失败(其返回码见 page_mgr_switch)
 *****************************************************************************/
int8_t page_mgr_switch_back(page_mgr_t *p_mgr, lv_screen_load_anim_t anim)
{
	int8_t ret = 0;

	if (NULL == p_mgr)
	{
		return -1;
	}
	if (p_mgr->buf_cnt < 2u || NULL == p_mgr->buf[1])
	{
		return -2;
	}

	ret = page_mgr_switch(p_mgr, p_mgr->buf[1], anim);
	if (0 != ret)
	{
		return -3;
	}

	return 0;
}

/******************************************************************************
 * @name    page_mgr_clear
 * @brief   销毁缓冲区里所有页面的 LVGL 对象, 缓冲区清空(注册关系保留)
 * @param   p_mgr[in] 管理器实例
 *
 * @return  无
 *
 * @note    反复删队尾: 每删一个 buf_cnt 自减, 所以循环结束时缓冲区必然为空,
 *          且正在显示的那一页是**最后**一个被删的
 *****************************************************************************/
void page_mgr_clear(page_mgr_t *p_mgr)
{
	if (NULL == p_mgr)
	{
		return;
	}

	while (p_mgr->buf_cnt > 0)
	{
		page_mgr_delete_obj(p_mgr, p_mgr->buf[p_mgr->buf_cnt - 1u]);
	}

	p_mgr->buf_cnt = 0;
	p_mgr->busy = false;
}

/******************************************************************************
 * @name    page_mgr_current
 * @brief   取当前正在显示的页面
 * @param   p_mgr[in] 管理器实例
 *
 * @return  页面指针; 未初始化/还没切过页时返回 NULL
 *****************************************************************************/
page_base_t *page_mgr_current(const page_mgr_t *p_mgr)
{
	if (NULL == p_mgr || 0 == p_mgr->buf_cnt)
	{
		return NULL;
	}

	return p_mgr->buf[0];
}

/******************************************************************************
 * @name    page_mgr_buf_at
 * @brief   按"最近使用"顺序取缓冲区里的第 index 页
 * @param   p_mgr[in] 管理器实例
 * @param   index[in] 0 = 正在显示, 1 = 上一个显示过的, 依此类推
 *
 * @return  页面指针; 越界或 p_mgr null 返回 NULL
 *****************************************************************************/
page_base_t *page_mgr_buf_at(const page_mgr_t *p_mgr, uint8_t index)
{
	if (NULL == p_mgr || index >= p_mgr->buf_cnt)
	{
		return NULL;
	}

	return p_mgr->buf[index];
}

/******************************************************************************
 * @name    page_mgr_buf_count
 * @brief   取缓冲区里持有对象的页数
 * @param   p_mgr[in] 管理器实例
 *
 * @return  页数(0..PAGE_MGR_BUF_SIZE); p_mgr null 时返回 0
 *****************************************************************************/
uint8_t page_mgr_buf_count(const page_mgr_t *p_mgr)
{
	if (NULL == p_mgr)
	{
		return 0;
	}

	return p_mgr->buf_cnt;
}

/******************************************************************************
 * @name    page_mgr_find
 * @brief   按 page_id 在注册表里查页面
 * @param   p_mgr[in]   管理器实例
 * @param   page_id[in] 页面 ID
 *
 * @return  页面指针; 未注册/找不到返回 NULL
 *****************************************************************************/
page_base_t *page_mgr_find(const page_mgr_t *p_mgr, uint16_t page_id)
{
	uint16_t i = 0;

	if (NULL == p_mgr)
	{
		return NULL;
	}

	for (i = 0; i < p_mgr->page_cnt; i++)
	{
		if (p_mgr->pages[i]->page_id == page_id)
		{
			return p_mgr->pages[i];
		}
	}

	return NULL;
}
