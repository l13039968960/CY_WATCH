/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file lv_watch_page_ota.c
 *
 * @par dependencies
 * - lv_watch_page_ota.h
 * - lv_watch_page.h
 * - ../../APP/EasyAPP/port/easyapp_port.h
 * - lvgl.h
 * - stdio.h
 * - string.h
 *
 * @author zw1194
 *
 * @brief 手表UI - OTA 固件升级页(ota, ID 0x0304): 黑底 + 进度条 + **唯一一个**按钮.
 *        本页是**纯显示**: 它不产生任何 OTA 状态, 只把 app_core 投进来的"一组显示值"
 *        画到控件上. 进度条和按钮长什么样, 全部由 lv_watch_page_ota.h 里那几个
 *        watch_page_ota_post_*() 说了算. 由 PageMem 托管.
 *
 * Processing flow:
 *
 * 发起方(别的任务, 通常是 app_core 任务)
 *   --lv_watch_page_ota.h 的 post 族--> 本文件静态双缓冲槽(只写值, 不碰控件)
 *   --最迟 WATCH_OTA_TICK_MS(100ms), 本页 lv_timer--> watch_ota_apply()
 *                                                      └─> lv_bar_set_value / lv_label_set_text
 * 按钮点击   : watch_ota_btn_cb 只上报 EVT_SERVICE_OTA_BUTTON —— **本页不解释**这一下
 *              是什么意思, 由 app_core 决定(见 cywatch_app_ota_page.c)
 * 手势       : 左滑→血氧页(MOVE_LEFT) / 右滑→表盘页(MOVE_RIGHT)
 *
 * pf_create : 建独立屏幕 + 标题 + 全屏手势条 + 进度条 + 按钮(+ 标签) + 100ms 定时器,
 *             建完立刻按当前槽画一帧(页面不在时投进来的值不丢);
 * pf_show   : 立即补一帧 + 恢复定时器;
 * pf_hide   : 暂停定时器;
 * pf_destroy: lv_timer_delete + 句柄置 NULL.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note ★★本页**没有任何 OTA 状态机**, 这是有意的★★ 2026-09-22 之前这里是一个七态
 *       模拟页(IDLE/CHECKING/UPTODATE/FOUND/DOWNLOADING/DONE/FAILED), 自己造数据自己画.
 *       用户明确要求把它**彻底删掉**, 理由和分层有关: 状态属于 app_core(将来属于
 *       OTA 服务), 页面只负责显示. 于是"检查/下载/进度百分比"这些**语义**全部搬到了
 *       cywatch_app_ota_page.c, 本页只剩"把一组值画上去"这一件事.
 *       ⚠ 别因为"看着空"就顺手把状态挪回来: 页面会被 LRU 淘汰, 放在页面里的状态
 *         一重建就归零, 而真实下载在页面之外还在继续.
 *
 * @note ★★"一个按钮干两件事"这个结论仍然成立, 但机理搬家了★★ 原来蓝/绿底色是页面
 *       按当前状态自己推出来的; 现在页面无从推断, 只能由持有语义的一方明确告诉它 ——
 *       这就是 watch_page_ota_post_btn_green() 这个字段存在的原因(见该函数 @note).
 *       一次点击 = 一个**不透明**的步进, 页面不解释它的含义. 别把它简化为
 *       "点一下就检查" —— 那正是 app_core 该决定的事.
 *
 * @note ★异步通道: 双缓冲 + 两个 volatile 标量, 单写者单读者★ 见下面 Static Data
 *       那一段的注释. 三条承重规则(写者两条 store 的顺序、读者只读一次索引、必须
 *       字段级拷贝)全部写在代码旁边, **别"顺手整理"它们**.
 *
 * @note ★关于"离开本页"★ 原来 pf_hide 暂停定时器意味着"下载冻结"、pf_destroy 意味着
 *       "回来是干净的 IDLE". 现在两条都不成立了, 取而代之的性质是:
 *       1. pf_hide 暂停定时器**只是省电** —— app_core 在页面不可见期间投的值照收不误,
 *          切回来 pf_show 立刻补一帧, 显示的是 app_core 的当前值;
 *       2. LRU 淘汰重建后, pf_create 按当前槽重建, 显示的还是 app_core 的当前值
 *          (而不是"干净的起点"). 因为状态本来就不在本页手里.
 *       ⇒ 两条路径的正确性都由"槽会合并 + 重建时重画"保证, 与定时器在不在跑无关.
 *
 * @note ★三个顺序/标志上的坑, 改的时候别踩★
 *       1. **建序承重**: watch_strip_create() 内部会把手势条抬到最顶层
 *          (lv_obj_move_to_index(-1)), 所以**进度条和按钮都必须建在它之后**,
 *          靠"同级子里索引更高"在命中测试里压住手势条, 否则点不动. 这与血氧页
 *          那个圆按钮是同一个坑.
 *       2. **进度条要建在按钮之前, 并且必须摘掉 LV_OBJ_FLAG_CLICKABLE**: lv_bar
 *          的构造函数只摘了 CHECKABLE/SCROLLABLE(见 lvgl/src/widgets/bar/lv_bar.c),
 *          CLICKABLE 是基类 lv_obj 给的、**留着**. 不摘的话进度条那一条横带会把
 *          触摸从手势条手里抢走(条上起步的滑动失效); 而它若建得比按钮晚, 更会在
 *          命中测试里**压住按钮 —— 静默点不动, 编译链接毫无异常**. 本页只有一个
 *          按钮, 这条**更要紧**(压住它就等于整页点不动).
 *       3. **pf_create 必须复位 applied_valid**: PAGE_MGR_BUF_SIZE=2 而注册了 5 页,
 *          走一圈 home→heart→spo2→ota→home 就会淘汰重建. 不复位的话, "变化才写"
 *          的比较基准还停在上一轮的绿按钮/旧文案上, 重建出来的页面就会残着旧样子.
 ******************************************************************************/
#include "lv_watch_page_ota.h"
#include "lv_watch_page.h"
#include "easyapp_port.h"

#include <stdio.h>
#include <string.h>

/* 子集字库(实现在 LVGL/assets/fonts, 生成脚本 MDK-ARM/gen_watch_fonts.py):
   本页文案全是汉字 + ASCII 数字/百分号, 一个 14 号字库就够了.
   ★现在文案是 app_core 投进来的, 改文案的地方不在本文件里了★ —— 但下面这条一样
   适用, 且已经抄了一份到 lv_watch_page_ota.h 的 @note 里: 缺字**不报错**、不告警、
   不链接失败, 只是那个字**渲染成空白**, 会一路伪装成"页面正常" */
LV_FONT_DECLARE(lv_font_alibaba_puhuiti_14)

/***********************************Defines************************************/
#define WATCH_OTA_TICK_MS     100u	 /* 本页节拍. ★它现在的身份是"异步通道的延迟
										上限"★: app_core 投的值最迟这么多毫秒落地.
										仍比别页的 WATCH_TICK_MS(1s) 快 —— 真实下载
										进度要求秒级以内的响应 */
#define WATCH_OTA_BAR_W       200
#define WATCH_OTA_BAR_H       14	 /* >= 10 才不会被 LV_BAR_SIZE_MIN 强撑 */
#define WATCH_OTA_BAR_R       7u	 /* = BAR_H/2. ★MAIN 与 INDICATOR 必须同值★ */
#define WATCH_OTA_BAR_X       20
#define WATCH_OTA_BAR_Y       80
#define WATCH_OTA_BTN_X       40
#define WATCH_OTA_BTN_W       160	 /* 塞得下最长那条文案"已是最新版本"(6 字 x 14px) */
#define WATCH_OTA_BTN_H       44
#define WATCH_OTA_BTN_Y       164	 /* 在"进度条下沿(94)..屏底(280)"这段里居中 */
#define WATCH_OTA_BTN_R       8u

/* 本页专属配色(全UI通用色在 lv_watch_page.h 里) */
#define WATCH_COL_OTA_TRACK	  0x2A2A2A	 /* 进度条槽: 深灰, 在黑底上要看得出来 */
#define WATCH_COL_OTA_FILL	  0x4CD964	 /* 下载族的绿. ★两处都用它★: 进度条指示条,
										    以及 btn_green 为真时的按钮底色(原来是
										    两件不同的事共用一个色值, 现在由
										    watch_ota_apply 一处消费) */
#define WATCH_COL_OTA_BTN	  0x2F6FED	 /* 按钮"检查族"的底色: 蓝 */
/***********************************Defines************************************/

/**********************************Declaring***********************************/
/* OTA页: 进度条 + **唯一**按钮(带标签) + 自己的 100ms 定时器.
   ★注意这里没有任何"当前状态"字段★ —— 显示的真相在下面的 s_cmd 槽里, 本结构只存
   一份"上一次画上去的是什么", 用来免掉每拍的重复样式写 */
typedef struct
{
	page_base_t base;
	lv_obj_t *p_bar;	/* 进度条 */
	lv_obj_t *p_btn;	/* 唯一的按钮: 文案/可点性/色系全部由 post 族决定 */
	lv_obj_t *p_lbl;	/* 按钮上的字 */
	lv_timer_t *p_timer;
	watch_page_ota_state_t applied;	/* 上一次真正画上去的那一组值(非 volatile) */
	uint8_t applied_valid;			/* 0 = 还没画过 → 下一帧全部无条件写 */
} watch_ota_t;

static void watch_ota_cmd_load(watch_page_ota_state_t *p_out);
static void watch_ota_cmd_store(volatile watch_page_ota_state_t *p_dst,
								const watch_page_ota_state_t *p_src);
static void watch_ota_publish(watch_page_ota_state_t *p_cmd);
static void watch_ota_sync_now(void);
static void watch_ota_apply(const watch_page_ota_state_t *p_cmd);
static void watch_ota_timer_cb(lv_timer_t *p_timer);
static void watch_ota_btn_cb(lv_event_t *e);

static void watch_ota_swipe_cb(lv_event_t *e);

static void watch_ota_create(page_base_t *p_page);
static void watch_ota_destroy(page_base_t *p_page);
static void watch_ota_show(page_base_t *p_page);
static void watch_ota_hide(page_base_t *p_page);
/**********************************Declaring***********************************/

/*********************************Static Data**********************************/
static watch_ota_t s_ota;

/* ---- 异步通道 ============================================================
   形态: **双缓冲 + 两个 volatile 标量**. 写者只碰"备用格", 读者只碰"生效格",
   两者永不重叠, 所以不需要锁、不需要自旋、不需要在 lvgl 任务里等任何东西.

     s_active ──> s_cmd[0] / s_cmd[1] 里的哪一格是"当前对外值"
     s_dirty  ──  1 = 有新值还没画

   为什么两条索引不合成一个结构体/不用位域: 它们在 s_active 那一格里必须是**两次
   独立的字节写**, 顺序才有意义(见 watch_ota_publish). 位域会变成整字读改写, 把
   两次发布并成一次非原子访问, 静默毁掉下面这个顺序保证.

   ★为什么"全 volatile + 单核"就够了★ 编译器只保证"volatile 访问之间的程序顺序",
   这正是需要的全部 —— 硬件那边单核 Cortex-M4 无 cache、写缓冲按序, 没有 store 会
   被重排到更早的 store 前面. ⚠ 这条推理依赖"单核 + 无 cache": 若哪天 s_cmd 挪到
   DMA 可见的 RAM 里、或上了第二个核, 必须在 s_active 之后补 __DMB().
   ★不需要 seqlock/重试★ 因为写者永远不碰生效格 —— 读者读到的整格内容始终是
   一次完整发布的结果(前提是遵守 watch_ota_cmd_load 里"索引只读一次"那条).

   ★不需要"单写者"以外的任何假设★ 但必须**只有一个写者**(本批 = appcore 任务):
   多一个写者时, 两个写者可能同时挑中同一格备用格, 读改写互相覆盖. 不会崩, 只会
   静默丢更新. 谁要加第二个写者, 先给它加锁.
   ========================================================================= */
static volatile watch_page_ota_state_t s_cmd[2] =
{
	/* 两格的初值必须一样: 开机时页面还没建, 万一此时有人读, 读到的应该是"起点"
	   而不是垃圾. 字段顺序与 watch_page_ota_state_t 一致:
	   progress / enabled / bar_visible / btn_green / text */
	{ 0u, 1u, 0u, 0u, "检查新版本" },
	{ 0u, 1u, 0u, 0u, "检查新版本" },
};
static volatile uint8_t s_active;	/* 读者取用这一格 */
static volatile uint8_t s_dirty;	/* 1 = 有新值待落地 */

static const page_vtable_t s_ota_vtable =
{
	.pf_create = watch_ota_create,
	.pf_destroy = watch_ota_destroy,
	.pf_show = watch_ota_show,
	.pf_hide = watch_ota_hide,
};
/*********************************Static Data**********************************/

/******************************************************************************
 * @name    watch_ota_cmd_load
 * @brief   把"当前生效格"读进一个非 volatile 局部(volatile → 本地)
 * @param   p_out[out] 读出的一组值
 *
 * @return  无
 *
 * @note    ★★索引 s_active 必须只读一次★★ 先存进局部 slot, 后面全部通过 slot
 *          访问. 若写成 `p_out->x = s_cmd[s_active].x;` 那样每次现读, 而读的期间
 *          恰好有投值把 s_active 翻过去, 就会画出"进度取自 A 格、文案取自 B 格"的
 *          **跨格撕裂** —— 这是双缓冲唯一防不住的失效模式, 只能靠这条规矩堵.
 * @note    抄进局部还有两个好处: 后面 watch_ota_apply 拿到的是普通指针, 不会把
 *          volatile char* 递给 LVGL(那要靠抹掉限定符才编得过, 而 MiscControls 里有
 *          -Wno-cast-qual, **编译器不会警告**); 以及给 watch_ota_apply 一个稳定的
 *          比较基准
 *****************************************************************************/
static void watch_ota_cmd_load(watch_page_ota_state_t *p_out)
{
	const volatile watch_page_ota_state_t *p_src;
	uint8_t slot = s_active;	/* ★只读这一次★ 见 @note */
	uint8_t i;

	p_src = &s_cmd[slot];

	p_out->progress	   = p_src->progress;
	p_out->enabled	   = p_src->enabled;
	p_out->bar_visible = p_src->bar_visible;
	p_out->btn_green   = p_src->btn_green;

	/* 32 字节**无条件拷满**, 不扫 '\0': 两格都由静态初始化给定(字面量会把后面补 0),
	   之后每次也都是整段拷, 所以 32 个字节恒有定义. 顺带白拿一个 NUL 结尾保证 */
	for (i = 0u; i < WATCH_OTA_TEXT_SIZE; i++)
	{
		p_out->text[i] = p_src->text[i];
	}
}

/******************************************************************************
 * @name    watch_ota_cmd_store
 * @brief   把一个本地值写进指定格(本地 → volatile)
 * @param   p_dst[out] 目标格(必定是"备用格")
 * @param   p_src[in]  源
 *
 * @return  无
 *
 * @note    ★★必须是这种逐字段的写法, 不能 memcpy, 更不能 `*p_dst = *p_src;`★★
 *          1. `memcpy(p_dst, (const void *)p_src, n)` 要把 volatile 抹掉 —— 抹掉
 *             之后编译器有权把整段拷贝**整个挪到** s_active = idx 那条 store 后面,
 *             于是读者可能看到新索引配旧内容. 而 -Wno-cast-qual 让这个错误**无声无息**
 *          2. `*p_dst = *p_src;` 看起来最自然, 恰恰是最坏的: C 标准没有规定 volatile
 *             聚合体赋值的访问粒度, GCC 逐成员拷、armclang 整体拷 —— 同一份源码在
 *             一个编译器上对、另一个上错
 *          所以老老实实一个字段一条 volatile 赋值
 *****************************************************************************/
static void watch_ota_cmd_store(volatile watch_page_ota_state_t *p_dst,
								const watch_page_ota_state_t *p_src)
{
	uint8_t i;

	p_dst->progress	   = p_src->progress;
	p_dst->enabled	   = p_src->enabled;
	p_dst->bar_visible = p_src->bar_visible;
	p_dst->btn_green   = p_src->btn_green;

	for (i = 0u; i < WATCH_OTA_TEXT_SIZE; i++)
	{
		p_dst->text[i] = p_src->text[i];
	}
}

/******************************************************************************
 * @name    watch_ota_publish
 * @brief   发布一组显示值: 归一化 → 写备用格 → 翻索引 → 置脏(写者侧唯一入口)
 * @param   p_cmd[in,out] 要发布的一组值; 会被就地归一化
 *
 * @return  无
 *
 * @note    ★只有 app_core 任务会调到这里 —— 单写者是这个机制的前提★
 * @note    归一化放在这里而不是各 post 函数里: ① 三个布尔字段收敛成 0/1, 免得
 *          enabled=2 与 enabled=1 被当成"变了"而白刷一次样式; ② progress 统一截到
 *          0..100; ③ 强制 text 末字节为 '\0'. 集中一处, 五个单字段函数就都不用管
 * @note    ★★最后两行的顺序不可换★★
 *          正确的是 `s_active = idx;` 再 `s_dirty = 1;`(先发布内容, 再报"有新值").
 *          换成反的顺序就会**丢更新**: 写者置 dirty → 读者看到 dirty 并清零 →
 *          读者此时读到的是**旧的** s_active, 于是画的是旧值 → 写者这才翻索引.
 *          结果是新值已经就位、s_dirty 却是 0, 一直等到下一次投值才被画出来.
 *          按现在的顺序则不可能: 读者一旦看到 dirty==1, 那一格**必定已经发布**,
 *          而且读者是"清完 dirty 才去读索引".
 *****************************************************************************/
static void watch_ota_publish(watch_page_ota_state_t *p_cmd)
{
	uint8_t idx;

	p_cmd->enabled	   = (0u != p_cmd->enabled)	  ? 1u : 0u;
	p_cmd->bar_visible = (0u != p_cmd->bar_visible) ? 1u : 0u;
	p_cmd->btn_green   = (0u != p_cmd->btn_green)	  ? 1u : 0u;
	if (p_cmd->progress > (uint8_t)WATCH_OTA_PROGRESS_MAX)
	{
		p_cmd->progress = (uint8_t)WATCH_OTA_PROGRESS_MAX;
	}
	p_cmd->text[WATCH_OTA_TEXT_SIZE - 1u] = '\0';

	/* 只写备用格: 生效格读者可能正在读, 一个字都不许碰 */
	idx = (uint8_t)(1u - s_active);
	watch_ota_cmd_store(&s_cmd[idx], p_cmd);

	s_active = idx;	   /* ★先发布索引★ */
	s_dirty	 = 1u;	   /* ★再置脏★ 顺序不可换, 见 @note */
}

/******************************************************************************
 * @name    watch_ota_sync_now
 * @brief   立刻(不经过定时器)按当前槽把控件重画一遍
 *
 * @return  无
 *
 * @note    pf_create / pf_show 用. 它们跑在 lvgl 任务里, 直接调 lv_xxx 是合法的
 *          (跨任务契约只禁止**别的**任务碰对象树)
 * @note    ★故意**不碰 s_dirty**★ 清了就重开 watch_ota_publish 注释里那个丢更新
 *          的窗口(读者清 dirty 与读索引之间存在缝). 让定时器下一拍再幂等地画一次,
 *          代价为零
 *****************************************************************************/
static void watch_ota_sync_now(void)
{
	watch_page_ota_state_t cmd;

	watch_ota_cmd_load(&cmd);
	watch_ota_apply(&cmd);
}

/******************************************************************************
 * @name    watch_ota_apply
 * @brief   把一组显示值画到按钮/进度条上(**唯一**碰本页控件的地方)
 * @param   p_cmd[in] 要显示的一组值
 *
 * @return  无
 *
 * @note    ★每项都"变了才写"★ 不是优化洁癖: 真实链路投一次"下载中"可能是几十 Hz,
 *          而 lv_obj_set_style_bg_color / lv_label_set_text 都会触发样式刷新与重绘
 *          (lv_obj_set_local_style_prop 根本不看新旧值是否相同). 不比较就是每拍白刷.
 *          ⚠ applied_valid == 0(刚 pf_create)时**全部无条件写** —— 重建后的控件
 *          带的是主题默认样式, 没有任何"基准"可比较, 必须强制来一遍
 * @note    ★这里不许 printf★ 它现在跑在真实链路的投值频率上(定时器内), 而 printf
 *          是阻塞式 polled 发送, 会拖住整个 lvgl 任务(所有页面一起卡). 日志只留在
 *          pf_create / pf_destroy
 *****************************************************************************/
static void watch_ota_apply(const watch_page_ota_state_t *p_cmd)
{
	watch_ota_t *p_ota = &s_ota;
	uint8_t force = (0u == p_ota->applied_valid) ? 1u : 0u;

	/* 控件还没建或已销毁(定时器可能比控件多活一拍): 什么都不做.
	   注意此时**不**更新 applied_valid —— 等控件真建起来时还得强制写一遍 */
	if ((NULL == p_ota->p_btn) || (NULL == p_ota->p_bar) || (NULL == p_ota->p_lbl))
	{
		return;
	}

	/* ---- 1. 按钮文案 ---- */
	if ((0u != force) || (0 != strcmp(p_ota->applied.text, p_cmd->text)))
	{
		lv_label_set_text(p_ota->p_lbl, p_cmd->text);
	}

	/* ---- 2. 按钮可点性 ----
	   禁用用 LV_STATE_DISABLED(9.x 是 lv_obj_add_state/remove_state, 没有 v8 那个
	   lv_obj_clear_state). 默认主题的 disabled 样式会给它加一层 recolor, 所以"置灰"
	   是看得见的; 而且 LVGL 对 DISABLED 的控件**根本不派发 CLICKED** —— 置灰是真的
	   点不动, 不是画着好看 */
	if ((0u != force) || (p_ota->applied.enabled != p_cmd->enabled))
	{
		if (0u != p_cmd->enabled)
		{
			lv_obj_remove_state(p_ota->p_btn, LV_STATE_DISABLED);
		}
		else
		{
			lv_obj_add_state(p_ota->p_btn, LV_STATE_DISABLED);
		}
	}

	/* ---- 3. 按钮色系 ----
	   蓝 = 检查族, 绿 = 下载族. 一个按钮干两件事, 光靠文案区分不够醒目, 底色再补一层.
	   @note ★为什么这一项要由外面投★ 原来它是本页按当前状态推出来的; 状态机删掉之后
	         本页无从推断 —— 页面只认识"值", 不认识"含义". 见 watch_page_ota_post_btn_green */
	if ((0u != force) || (p_ota->applied.btn_green != p_cmd->btn_green))
	{
		lv_obj_set_style_bg_color(p_ota->p_btn,
			lv_color_hex((0u != p_cmd->btn_green) ? WATCH_COL_OTA_FILL : WATCH_COL_OTA_BTN), 0);
	}

	/* ---- 4. 进度条显隐 ---- */
	if ((0u != force) || (p_ota->applied.bar_visible != p_cmd->bar_visible))
	{
		if (0u != p_cmd->bar_visible)
		{
			lv_obj_remove_flag(p_ota->p_bar, LV_OBJ_FLAG_HIDDEN);
		}
		else
		{
			lv_obj_add_flag(p_ota->p_bar, LV_OBJ_FLAG_HIDDEN);
		}
	}

	/* ---- 5. 进度值 ----
	   ★LV_ANIM_OFF 是有意的: 但理由是"运行时不补间", 不是"省 flash"★
	   开了动画会走 lv_bar_set_value_with_anim 的 else 分支(lv_anim_init +
	   lv_anim_start + 一个自定义回调), 于是每次设值都要建一次补间动画, 进度条显示的
	   就是被动画拖后的旧值 —— 而进度本来就是硬件推的, 直接显示真值即可.
	   ⚠ 别以为它省 flash: lv_bar_set_value 是**无条件**调 lv_bar_set_value_with_anim 的
	   (见 lvgl/src/widgets/bar/lv_bar.c), 而链接器会保留被保留函数引用到的一切,
	   所以 lv_bar_anim / lv_bar_anim_completed / lv_anim_start 照样进镜像
	   (已在 AHT21_TEST.map 里核实). */
	if ((0u != force) || (p_ota->applied.progress != p_cmd->progress))
	{
		lv_bar_set_value(p_ota->p_bar, (int32_t)p_cmd->progress, LV_ANIM_OFF);
	}

	/* 记下"画上去的是什么", 供下一拍比较. ★必须在所有写之后★ */
	p_ota->applied = *p_cmd;
	p_ota->applied_valid = 1u;
}

/******************************************************************************
 * @name    watch_ota_timer_cb
 * @brief   本页 100ms 定时器: 把 app_core 投进来的值落地(**读者侧唯一入口**)
 * @param   p_timer[in] LVGL定时器(未使用)
 *
 * @return  无
 *
 * @note    ★顺序是"先清脏、再取索引"★ 与 watch_ota_publish 里的"先发布内容、再置脏"
 *          配对才有那个不丢更新的保证, 两处要一起读
 * @note    没新值时一次早退 —— 空闲时这个定时器的成本就是一次 volatile 读
 * @note    连续两次投值会被**合并**: 写者两次都在同一个备用格上改, 读者醒来时只看到
 *          最终那一组, 中间态**永不回放**. 这正是进度显示想要的(不需要看历史), 也是
 *          为什么这里**不该**换成队列
 *****************************************************************************/
static void watch_ota_timer_cb(lv_timer_t *p_timer)
{
	watch_page_ota_state_t cmd;

	(void)p_timer;

	if (0u == s_dirty)
	{
		return;
	}

	s_dirty = 0u;	 /* ★先清脏★ */

	watch_ota_cmd_load(&cmd);	/* ★再取索引(内部只读一次)★ */
	watch_ota_apply(&cmd);
}

/******************************************************************************
 * @name    watch_ota_btn_cb
 * @brief   唯一按钮的点击: **只上报**, 不解释
 * @param   e[in] LVGL事件(CLICKED)
 *
 * @return  无
 *
 * @note    ★本页不再决定这一下是什么意思★ 原来这里按当前状态分支(是"开始下载"还是
 *          "开始检查"); 现在语义属于 app_core, 页面发出的只是一个不带参数的
 *          "按钮被按了". 谁按下、当前该干什么, 由 cywatch_app_ota_page.c 判断
 * @note    event_flags 填 0: 本事件不用旗标(点击就是"往前走一步"), 见 easyapp_page.h
 * @note    ★不能放 ISR: 本回调跑在 lvgl 任务, send 内含 osKernelLock, 合法★
 *          (与下面的手势回调同理)
 * @note    按钮被置灰时 LVGL 不派发 CLICKED, 所以这里不需要"禁用中就别上报"的兜底
 *****************************************************************************/
static void watch_ota_btn_cb(lv_event_t *e)
{
	(void)e;

	x_port_easyapp_event_send(EVT_SERVICE_OTA_BUTTON, 0, NULL);
}

/******************************************************************************
 * @name    watch_ota_swipe_cb
 * @brief   本页手势: 左滑→血氧页(MOVE_LEFT) / 右滑→表盘页(MOVE_RIGHT)
 * @param   e[in] LVGL事件
 *
 * @return  无
 *
 * @note    ★watch_swipe_track() 只能调一次★ 它每次调用都会**读并清零**累积位移,
 *          两个分支各调一次的话第二次必然是 WATCH_SWIPE_NONE. 所以照心率页的写法
 *          先存进局部变量再比较
 * @note    四页成环: 左→右 = OTA, HOME, HEART, SPO2. 本页在环上, 两个方向都有去处
 *          (与血氧页/心率页那两条"单边"不同)
 * @note    旗标 3/5 编码的是**方向**(目标页在右/左), 不是目标页本身 —— 目标由各页
 *          自己的 APP 层 switch_page 决定, 见 cywatch_app_ota_page.c
 *****************************************************************************/
static void watch_ota_swipe_cb(lv_event_t *e)
{
	watch_swipe_dir_t dir = watch_swipe_track(e);

	if (WATCH_SWIPE_LEFT == dir)
	{
		/* 左滑 → 血氧页(它在环上的左边) */
		watch_switch(WATCH_PAGE_ID_SPO2, LV_SCR_LOAD_ANIM_MOVE_LEFT);

		/* ★不能放 ISR: 本回调跑在 lvgl 任务, send 内含 osKernelLock, 合法★ */
		x_port_easyapp_event_send(EVT_SERVICE_UIHOME_SWITCH_PAGE, 5, NULL);
	}
	else if (WATCH_SWIPE_RIGHT == dir)
	{
		/* 右滑 → 表盘页 */
		watch_switch(WATCH_PAGE_ID_HOME, LV_SCR_LOAD_ANIM_MOVE_RIGHT);
		x_port_easyapp_event_send(EVT_SERVICE_UIHOME_SWITCH_PAGE, 3, NULL);
	}
	else
	{
		/* 上/下滑: 本页没有对应页面, 什么都不做 */
	}
}

/******************************************************************************
 * @name    watch_ota_create
 * @brief   建本页: 标题 + 手势条 + 进度条 + 唯一按钮(带标签) + 100ms 定时器
 * @param   p_page[in] 页面基类指针
 *
 * @return  无
 *
 * @note    见文件头的三个坑: 手势条必须**最先**建(进度条和按钮要压住它), 进度条
 *          必须建在按钮**之前**且摘掉 CLICKABLE, applied_valid 必须复位
 *****************************************************************************/
static void watch_ota_create(page_base_t *p_page)
{
	watch_ota_t *p_ota = (watch_ota_t *)p_page;
	lv_obj_t *scr = lv_obj_create(NULL);
	lv_obj_t *lbl = NULL;

	/* 清掉可能残留的上一次实例句柄 */
	p_ota->p_bar = NULL;
	p_ota->p_btn = NULL;
	p_ota->p_lbl = NULL;

	/* ---- 复位瞬时状态(本页会被 PAGE_MGR_BUF_SIZE=2 的 LRU 淘汰后重建) ----
	   ★applied_valid = 0 是关键★: 它让紧随其后的 watch_ota_sync_now() **无条件**写一遍
	   全部五项(文案/可点性/色系/进度条显隐/进度值), 于是重建后的页面显示的是 app_core
	   当前那一组值, 不会残留上一轮的绿按钮或"升级完成".
	   它取代了原来 btn_green = 0xFFu 那个哨兵 —— 状态机搬走之后需要强制写的字段从一个
	   变成了五个, 一个显式标志比五个哨兵值清楚.
	   @note applied 的内容在 applied_valid=0 时无所谓(会被整组覆盖), 这里仍逐项写死,
	         只是不留一个悬着未定义的结构体 */
	p_ota->applied.progress	   = 0u;
	p_ota->applied.enabled	   = 1u;
	p_ota->applied.bar_visible = 0u;
	p_ota->applied.btn_green   = 0u;
	p_ota->applied.text[0]	   = '\0';
	p_ota->applied_valid	   = 0u;

	lv_obj_set_style_bg_color(scr, lv_color_hex(WATCH_COL_DARK), 0);

	/* 标题 */
	lbl = lv_label_create(scr);
	lv_label_set_text(lbl, "系统更新");
	lv_obj_set_style_text_font(lbl, &lv_font_alibaba_puhuiti_14, 0);
	lv_obj_set_style_text_color(lbl, lv_color_hex(WATCH_COL_TEXT), 0);
	lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, 20, 26);

	/* ★手势条先建★ 它内部会把自己抬到最顶层, 后建的控件才压得住它 */
	watch_strip_create(scr, WATCH_SCREEN_W, WATCH_SCREEN_H, 0, 0, watch_ota_swipe_cb);

	/* ---- 进度条(★建在按钮之前, 且必须摘 CLICKABLE★ 见文件头坑 2) ---- */
	p_ota->p_bar = lv_bar_create(scr);
	lv_obj_set_size(p_ota->p_bar, WATCH_OTA_BAR_W, WATCH_OTA_BAR_H);
	lv_obj_set_pos(p_ota->p_bar, WATCH_OTA_BAR_X, WATCH_OTA_BAR_Y);
	lv_obj_remove_flag(p_ota->p_bar, LV_OBJ_FLAG_CLICKABLE);

	/* 默认主题给 bar 的是"主色 + 20% 不透明"当槽(lv_theme_default.c 的
	   bg_color_primary_muted), 压在 0x0A0A0A 上几乎看不见, 所以槽和指示条都自己配色.
	   @note MAIN 与 INDICATOR 的圆角**必须同值**、且不要给 bar 加 padding: 两者圆角
	         不同(或指示条被 padding 挤窄)会让 lv_bar 判定 radius_issue, 转而去
	         create 一张 ARGB8888 的 mask 图层 —— 那是**每次重绘都 malloc 一次**
	         draw buffer 进 LVGL 池, 而本工程 LV_DRAW_SW_SUPPORT_ARGB8888 = 0 */
	lv_obj_set_style_bg_color(p_ota->p_bar, lv_color_hex(WATCH_COL_OTA_TRACK), LV_PART_MAIN);
	lv_obj_set_style_bg_opa(p_ota->p_bar, LV_OPA_COVER, LV_PART_MAIN);
	lv_obj_set_style_radius(p_ota->p_bar, WATCH_OTA_BAR_R, LV_PART_MAIN);
	lv_obj_set_style_bg_color(p_ota->p_bar, lv_color_hex(WATCH_COL_OTA_FILL), LV_PART_INDICATOR);
	lv_obj_set_style_bg_opa(p_ota->p_bar, LV_OPA_COVER, LV_PART_INDICATOR);
	lv_obj_set_style_radius(p_ota->p_bar, WATCH_OTA_BAR_R, LV_PART_INDICATOR);
	lv_bar_set_range(p_ota->p_bar, 0, 100);

	/* ---- 唯一的按钮(文案/色系稍后由 watch_ota_sync_now 整组写一遍, 这里只给初值) ---- */
	p_ota->p_btn = lv_button_create(scr);
	lv_obj_set_size(p_ota->p_btn, WATCH_OTA_BTN_W, WATCH_OTA_BTN_H);
	lv_obj_set_pos(p_ota->p_btn, WATCH_OTA_BTN_X, WATCH_OTA_BTN_Y);
	lv_obj_set_style_radius(p_ota->p_btn, WATCH_OTA_BTN_R, 0);
	lv_obj_set_style_bg_color(p_ota->p_btn, lv_color_hex(WATCH_COL_OTA_BTN), 0);
	/* 默认主题给按钮加了 shadow_width = LV_DPX(3). 阴影在"别再加回去"清单上
	   (见 lv_watch_ui.c 的设计约束), 显式关掉 */
	lv_obj_set_style_shadow_width(p_ota->p_btn, 0, 0);
	lv_obj_add_event_cb(p_ota->p_btn, watch_ota_btn_cb, LV_EVENT_CLICKED, NULL);

	p_ota->p_lbl = lv_label_create(p_ota->p_btn);
	lv_label_set_text(p_ota->p_lbl, "检查新版本");
	lv_obj_set_style_text_font(p_ota->p_lbl, &lv_font_alibaba_puhuiti_14, 0);
	lv_obj_set_style_text_color(p_ota->p_lbl, lv_color_hex(WATCH_COL_TEXT), 0);
	lv_obj_align(p_ota->p_lbl, LV_ALIGN_CENTER, 0, 0);

	/* 本页刷新定时器: 它现在是**异步通道的落地端**(见文件头 Processing flow) */
	p_ota->p_timer = lv_timer_create(watch_ota_timer_cb, WATCH_OTA_TICK_MS, NULL);

	p_page->obj = scr;

	/* 按当前槽画第一帧 —— app_core 在本页还不存在时投的值, 到这里才第一次露面 */
	watch_ota_sync_now();

	printf("WATCH page create ->OTA\r\n");
}

/******************************************************************************
 * @name    watch_ota_destroy
 * @brief   销毁本页: 删本页定时器 + 置空控件句柄
 * @param   p_page[in] 页面基类指针
 *
 * @return  无
 *
 * @note    **必须删定时器**: lv_timer 不是 widget, lv_obj_delete 管不到它
 * @note    这里只置空句柄, 不碰控件 —— 控件树此时已被 PageMem 删掉了.
 *          置空之后 watch_ota_apply 会因为句柄为 NULL 而早退(见那里的守卫)
 * @note    ★app_core 的当前值**不在这里清掉**★ 它是页面之外的事实, 页面被淘汰不代表
 *          下载停了 —— 下次 pf_create 会把同一个槽重画出来
 *****************************************************************************/
static void watch_ota_destroy(page_base_t *p_page)
{
	watch_ota_t *p_ota = (watch_ota_t *)p_page;

	if (NULL != p_ota->p_timer)
	{
		lv_timer_delete(p_ota->p_timer);
		p_ota->p_timer = NULL;
	}

	p_ota->p_bar = NULL;
	p_ota->p_btn = NULL;
	p_ota->p_lbl = NULL;
	p_ota->applied_valid = 0u;	/* 下次重建要强制全写 */

	printf("WATCH page destroy ->OTA\r\n");
}

/******************************************************************************
 * @name    watch_ota_show
 * @brief   本页变为可见: 立即按当前值补一帧 + 恢复本页定时器
 * @param   p_page[in] 页面基类指针
 *
 * @return  无
 *
 * @note    lv_timer_reset 不能省: lv_timer_resume 只清 paused 标志, 不重置周期
 * @note    补这一帧是必须的: 页面不可见期间定时器是暂停的, 而 app_core 一直在往槽里
 *          投值 —— 不补的话要等下一次投值才刷新, 一进来可能看到的是旧样子
 *****************************************************************************/
static void watch_ota_show(page_base_t *p_page)
{
	watch_ota_t *p_ota = (watch_ota_t *)p_page;

	watch_ota_sync_now();

	if (NULL != p_ota->p_timer)
	{
		lv_timer_reset(p_ota->p_timer);
		lv_timer_resume(p_ota->p_timer);
	}
}

/******************************************************************************
 * @name    watch_ota_hide
 * @brief   本页离开屏幕: 暂停本页定时器
 * @param   p_page[in] 页面基类指针
 *
 * @return  无
 *
 * @note    ★暂停**只是省电**, 不再是"冻结下载"★ app_core 在页面不可见期间投的值
 *          照收不误(写者只碰备用格, 与定时器在不在跑无关), 回来时 pf_show 立刻补一帧.
 *          见文件头 @note 关于"离开本页"的说明
 *****************************************************************************/
static void watch_ota_hide(page_base_t *p_page)
{
	watch_ota_t *p_ota = (watch_ota_t *)p_page;

	if (NULL != p_ota->p_timer)
	{
		lv_timer_pause(p_ota->p_timer);
	}
}

/******************************************************************************
 * @name    watch_page_ota_register
 * @brief   把 OTA 升级页注册进管理器(不创建控件, 切过去时才建)
 * @param   p_mgr[in] 管理器实例
 *
 * @return  page_mgr_register 的返回码(0 成功; -1/-2/-3/-4/-5/-6 见 PageMem.h)
 *****************************************************************************/
int8_t watch_page_ota_register(page_mgr_t *p_mgr)
{
	return page_mgr_register(p_mgr, &s_ota.base, &s_ota_vtable, WATCH_PAGE_ID_OTA);
}

/**************************** 异步设置口(对外) *********************************
 * 下面 6 个是 lv_watch_page_ota.h 里声明的全部对外函数. 它们**只写值, 不碰控件** ——
 * 这正是"异步"的含义, 也是 APP 层能安全调它们的原因(APP 层连 lvgl.h 都拿不到).
 * 调用方只允许一个(见 s_cmd 那段注释).
 ******************************************************************************/

/******************************************************************************
 * @name    watch_page_ota_post_progress
 * @brief   投一个进度条百分比(异步)
 * @param   percent[in] 0..100, >100 截到 100
 *
 * @return  无
 *****************************************************************************/
void watch_page_ota_post_progress(uint8_t percent)
{
	watch_page_ota_state_t cmd;

	watch_ota_cmd_load(&cmd);	/* 取当前值, 只改这一项 */
	cmd.progress = percent;		/* 范围截断在 watch_ota_publish 里统一做 */
	watch_ota_publish(&cmd);
}

/******************************************************************************
 * @name    watch_page_ota_post_text
 * @brief   投一条按钮文案(异步)
 * @param   p_text[in] UTF-8 文案, 必须以 '\0' 结尾; 超长静默截断
 *
 * @return  0  成功(含被截断)
 *         -1 p_text 为空
 *
 * @note    ★必须逐字节拷进来, 不能只存指针★ 所以调用方给个局部缓冲区就行, 函数
 *          返回后可以立刻复用. 见 lv_watch_page_ota.h
 *****************************************************************************/
int8_t watch_page_ota_post_text(const char *p_text)
{
	watch_page_ota_state_t cmd;
	uint8_t i;

	if (NULL == p_text)
	{
		return -1;
	}

	watch_ota_cmd_load(&cmd);

	/* 只拷到倒数第二个字节, 最后一个字节恒留给 '\0' ⇒ 超长自动截断且仍然合法 */
	for (i = 0u; (i < (WATCH_OTA_TEXT_SIZE - 1u)) && ('\0' != p_text[i]); i++)
	{
		cmd.text[i] = p_text[i];
	}
	/* 把尾巴清干净, 免得短文案背后挂着上一次的残字 */
	for (; i < WATCH_OTA_TEXT_SIZE; i++)
	{
		cmd.text[i] = '\0';
	}

	watch_ota_publish(&cmd);

	return 0;
}

/******************************************************************************
 * @name    watch_page_ota_post_enabled
 * @brief   投按钮的可点性(异步)
 * @param   enabled[in] 0 = 置灰, 非 0 = 可点
 *
 * @return  无
 *
 * @note    ★置灰就再点不动了★ LVGL 对 DISABLED 的控件不派发 CLICKED, 所以页面
 *          自己救不回来, 只有 app_core 能把它打开. ⇒ app_core 必须在"检查/下载"
 *          这类进行态上**自己带超时兜底**, 否则用户会被永久卡在一个死按钮上
 *          (页面只剩手势可走)
 *****************************************************************************/
void watch_page_ota_post_enabled(uint8_t enabled)
{
	watch_page_ota_state_t cmd;

	watch_ota_cmd_load(&cmd);
	cmd.enabled = enabled;
	watch_ota_publish(&cmd);
}

/******************************************************************************
 * @name    watch_page_ota_post_bar_visible
 * @brief   投进度条的显隐(异步)
 * @param   visible[in] 0 = 隐藏, 非 0 = 显示
 *
 * @return  无
 *
 * @note    "没更新/还没开始下载"时不该摆一条空进度条出来, 由 app_core 决定
 *****************************************************************************/
void watch_page_ota_post_bar_visible(uint8_t visible)
{
	watch_page_ota_state_t cmd;

	watch_ota_cmd_load(&cmd);
	cmd.bar_visible = visible;
	watch_ota_publish(&cmd);
}

/******************************************************************************
 * @name    watch_page_ota_post_btn_green
 * @brief   投按钮的色系(异步)
 * @param   green[in] 0 = 蓝("检查"族), 非 0 = 绿("下载"族)
 *
 * @return  无
 *
 * @note    ★这一个字段是"删掉页面状态机"的直接后果★ 原来蓝/绿是页面自己按当前状态
 *          推出来的; 状态机搬走之后页面无从推断, 只能由持有语义的一方明确告诉它
 *****************************************************************************/
void watch_page_ota_post_btn_green(uint8_t green)
{
	watch_page_ota_state_t cmd;

	watch_ota_cmd_load(&cmd);
	cmd.btn_green = green;
	watch_ota_publish(&cmd);
}

/******************************************************************************
 * @name    watch_page_ota_post_state
 * @brief   一次提交一整组显示值(异步, 整组同时落地)
 * @param   p_state[in] 想显示的一组值
 *
 * @return  0  成功
 *         -1 p_state 为空
 *
 * @note    ★改一组互相关联的值就该用它, 而不是连调 4 个单字段函数★ appcore 任务与
 *          lvgl 任务同为 osPriorityNormal 且开了时间片, 分 4 次单独 post 会被抢占,
 *          渲染出一个**逻辑上不存在**的中间态(例如文案已经是"下载中"、进度条却还隐藏
 *          着). 双缓冲本来就是为了解决这个 —— 这个函数才是它的正式用法
 *****************************************************************************/
int8_t watch_page_ota_post_state(const watch_page_ota_state_t *p_state)
{
	watch_page_ota_state_t cmd;

	if (NULL == p_state)
	{
		return -1;
	}

	cmd = *p_state;	  /* 源是调用方的普通对象, 整体拷贝; text 一起过来了 */
	cmd.text[WATCH_OTA_TEXT_SIZE - 1u] = '\0';	/* 防御: 调用方没清干净也别越界 */

	watch_ota_publish(&cmd);

	return 0;
}
