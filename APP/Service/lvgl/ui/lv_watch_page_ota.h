/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file lv_watch_page_ota.h
 *
 * @par dependencies
 * - stdint.h
 *
 * @author	zw1194
 *
 * @brief OTA 升级页(lv_watch_page_ota.c)的**异步设置口**: 让别的任务(app_core)
 *        能把"进度条值 / 按钮文案 / 按钮可点性 / 进度条显隐 / 按钮色系"投进来,
 *        而真正的 lv_xxx() 调用留在本页自己的 100ms 定时器里做.
 *
 * Processing flow:
 *
 * 调用方(单写者, 通常是 app_core 任务)          lvgl 任务
 *   watch_page_ota_post_progress(45)  ┐
 *   watch_page_ota_post_text("...")   ├─> 写静态槽 ─> 本页 100ms lv_timer 取用
 *   watch_page_ota_post_state(&cmd)   ┘              └─> lv_bar_set_value / lv_label_set_text
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note ★★本头文件**刻意零依赖**(只 include stdint.h), 别往里加 lvgl.h 或
 *       lv_watch_page.h★★ 理由是**分层**, 不是洁癖: lv_watch_page.h 第 63 行就把
 *       lvgl.h 拖进来了, 而 APP 层的 handler 跑在 appcore 任务上下文里, **绝不能拿到
 *       lv_xxx() 的原型** —— LV_USE_OS = LV_OS_NONE 时 LVGL 无锁, 全工程只有 "lvgl"
 *       任务能碰对象树(见 service/Lvgl/cywatch_service_Lvgl.h 头注释的跨任务契约)。
 *       本头只暴露"值"和"设置值的函数", 一个 vendor 类型都不带, 从源头上让 APP 层
 *       想调 lv_xxx() 也调不到. 范本: system/Rtc/cywatch_rtc.h(同为"接口零依赖").
 *
 * @note ★异步, 不是立刻生效★ post 只把值写进静态槽就返回, 实际渲染最迟在
 *       WATCH_OTA_TICK_MS(100ms) 之后由本页定时器完成. 页面**还没创建**时投的值
 *       也不丢: 下次 pf_create 会把当前槽直接画上去.
 *
 * @note ★只允许一个任务调用★ 内部是"双缓冲 + 单索引", 单写者是它成立的前提;
 *       多一个写者会静默失效(不会崩, 只会画出混合了两次投值的中间态).
 *
 * @note ★★中文文案必须在 LVGL/assets/fonts 的 CJK_SYMBOLS 子集里★★ 文案现在从
 *       app_core 侧传进来, 而缺字**不报错、不告警、不链接失败**, 只是那个字**渲染成
 *       空白**, 会一路伪装成"页面正常". 加新文案前先跑
 *       MDK-ARM/gen_watch_fonts.py 重新生成子集字库, 或确认字已在子集里.
 ******************************************************************************/
#ifndef __LV_WATCH_PAGE_OTA_H__
#define __LV_WATCH_PAGE_OTA_H__

/***********************************Includes***********************************/
#include <stdint.h>
/***********************************Includes***********************************/

/***********************************Defines************************************/
/* 按钮文案缓冲长度. UTF-8 下一个汉字 3 字节, 所以 32B ≈ 10 个汉字
   (现有最长文案"已是最新版本" = 6 字 = 18B, 还留着余量).
   超长由 post 侧**截断**(算成功, 不返回错误), 并保证 NUL 结尾. */
#define WATCH_OTA_TEXT_SIZE   32u

/* 进度条取值范围(与页面里的 lv_bar_set_range(0, 100) 一致) */
#define WATCH_OTA_PROGRESS_MIN 0u
#define WATCH_OTA_PROGRESS_MAX 100u
/***********************************Defines************************************/

/**********************************Declaring***********************************/
/* 一次可以提交的一组显示值. 5 个单字段函数各自只改其中一项, 但"改一组互相关联的
   值"(如"按钮转绿 + 文案变下载中 + 进度条从隐藏变显示")应该用 post_state 一次提交.

   @note ★为什么需要整组提交★ appcore 任务与 lvgl 任务同为 osPriorityNormal 且开了
         时间片, 分 4 次单独 post 会被抢占, 渲染出一个**逻辑上不存在**的中间态
         (例如文案已经是"下载中"、进度条却还隐藏着). 双缓冲本来就是为了解决这个,
         post_state 才是它的正式用法.
   @note 字段全是 uint8_t/char, 无位域 —— 位域会变成整字读改写, 破坏下面的发布顺序 */
typedef struct watch_page_ota_state
{
	uint8_t progress;					/* 0..100, post 侧会截到这个范围 */
	uint8_t enabled;					/* 0 = 按钮置灰(收不到点击); 非 0 = 可点 */
	uint8_t bar_visible;				/* 0 = 进度条隐藏; 非 0 = 显示 */
	uint8_t btn_green;					/* 0 = 按钮"检查族"的蓝; 非 0 = 下载族的绿 */
	char	text[WATCH_OTA_TEXT_SIZE];	/* UTF-8 按钮文案, 恒以 '\0' 结尾 */
} watch_page_ota_state_t;

/* ---- 五个单字段设置(每个只改一项, 其余沿用当前值) ---- */

/******************************************************************************
 * @name    watch_page_ota_post_progress
 * @brief   投一个进度条百分比(异步, 100ms 内生效)
 * @param   percent[in] 0..100, >100 截到 100
 *
 * @return  无
 *
 * @note    恒成功, 所以返回 void(与 void watch_switch(...) 同一取舍)
 *****************************************************************************/
void watch_page_ota_post_progress(uint8_t percent);

/******************************************************************************
 * @name    watch_page_ota_post_text
 * @brief   投一条按钮文案(异步, 100ms 内生效)
 * @param   p_text[in] UTF-8 文案, 必须以 '\0' 结尾; 超长静默截断
 *
 * @return  0  成功(含超长被截断)
 *         -1 p_text 为空
 *
 * @note    ★必须拷贝进来, 不能只存指针★ 所以调用方给个栈上/局部缓冲区就行,
 *          函数返回后可以立刻复用.
 * @note    汉字必须在 CJK_SYMBOLS 子集里, 见文件头 @note
 *****************************************************************************/
int8_t watch_page_ota_post_text(const char *p_text);

/******************************************************************************
 * @name    watch_page_ota_post_enabled
 * @brief   投按钮的可点性(异步, 100ms 内生效)
 * @param   enabled[in] 0 = 置灰, 非 0 = 可点
 *
 * @return  无
 *
 * @note    ★置灰就再点不动了★ 只有 app_core 能把它放回来, 所以 app_core 必须在
 *          "检查/下载"这类进行态上自己带超时兜底, 否则用户会被永久卡在一个死按钮上
 *          (页面只剩手势可走)
 *****************************************************************************/
void watch_page_ota_post_enabled(uint8_t enabled);

/******************************************************************************
 * @name    watch_page_ota_post_bar_visible
 * @brief   投进度条的显隐(异步, 100ms 内生效)
 * @param   visible[in] 0 = 隐藏, 非 0 = 显示
 *
 * @return  无
 *
 * @note    "没更新/还没开始下载"时不该摆一条空进度条出来, 由 app_core 决定
 *****************************************************************************/
void watch_page_ota_post_bar_visible(uint8_t visible);

/******************************************************************************
 * @name    watch_page_ota_post_btn_green
 * @brief   投按钮的色系(异步, 100ms 内生效)
 * @param   green[in] 0 = 蓝("检查"族), 非 0 = 绿("下载"族)
 *
 * @return  无
 *
 * @note    ★这一个字段是"删掉页面状态机"的直接后果★ 原来蓝/绿是页面自己按当前
 *          状态推出来的; 状态机搬走之后, 页面无从推断, 只能由持有语义的一方(app_core)
 *          明确告诉它. 一个按钮干两件事, 光靠文案区分不够醒目, 底色再补一层
 *****************************************************************************/
void watch_page_ota_post_btn_green(uint8_t green);

/* ---- 整组提交 ---- */

/******************************************************************************
 * @name    watch_page_ota_post_state
 * @brief   一次提交一整组显示值(异步, 100ms 内生效; 一次全部落地, 不出现中间态)
 * @param   p_state[in] 想显示的一组值
 *
 * @return  0  成功
 *         -1 p_state 为空
 *
 * @note    ★改一组互相关联的值就该用它, 而不是连调 4 个单字段函数★ 理由见上面
 *          watch_page_ota_state_t 的 @note
 * @note    p_state 里的 text 会被拷贝, 函数返回后调用方的缓冲区可复用
 *****************************************************************************/
int8_t watch_page_ota_post_state(const watch_page_ota_state_t *p_state);
/**********************************Declaring***********************************/

#endif // __LV_WATCH_PAGE_OTA_H__
