/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file lv_watch_page_spo2.h
 *
 * @par dependencies
 * - stdint.h
 *
 * @author	zw1194
 *
 * @brief 血氧页(lv_watch_page_spo2.c)的设置口: 让血氧服务任务把"一帧测量快照"
 *        投进来, 真正的圆内大字/倒计时/状态行留在本页 1s 刷新定时器里画.
 *        范本: lv_watch_page_home.h.
 *
 * Processing flow:
 *
 * 调用方(血氧服务任务)                          lvgl 任务
 *   watch_page_spo2_post(MEASURING,60,0,0) ──> 写本页快照 ──> 本页 1s 定时器渲染
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note ★本头**刻意零依赖**(只 include stdint.h)★ 理由与 lv_watch_page_home.h 同:
 *       调用方跑在血氧服务任务里, **绝不能拿到 lv_xxx() 的原型** —— LVGL 无锁,
 *       全工程只有 "lvgl" 任务能碰对象树.
 *
 * @note ★状态枚举必须由本头共享★: 服务侧写、页面侧读的是同一份值. 若两边各写
 *       一份"数值碰巧一样"的私有枚举, 将来改错一个**照样编译通过**, 只是静默画错态.
 ******************************************************************************/
#ifndef __LV_WATCH_PAGE_SPO2_H__
#define __LV_WATCH_PAGE_SPO2_H__

/***********************************Includes***********************************/
#include <stdint.h>
/***********************************Includes***********************************/

/**********************************Declaring***********************************/
/* 血氧页三态: 见 lv_watch_page_spo2.c 文件头的状态机 */
typedef enum
{
	WATCH_SPO2_IDLE = 0,  /* 圆内"开始" */
	WATCH_SPO2_MEASURING, /* 圆内血氧值(未可信时"--") + 倒计时 */
	WATCH_SPO2_DONE,	  /* 圆内定格值 + "%" */
} watch_spo2_state_e;

/******************************************************************************
 * @name    watch_page_spo2_post
 * @brief   投一整帧测量快照到血氧页(异步, 最迟下一个 1s 节拍生效)
 * @param   state    [in] 三态之一
 * @param   remain_s [in] 剩余秒数(MEASURING 时作倒计时)
 * @param   value    [in] 血氧值 %; 0 = 本帧还没有可信读数, 圆内显示 "--"
 * @param   finger   [in] 1 = 手指贴上了, 决定状态行写"测量中"还是"请贴紧手指"
 *
 * @return  无
 *
 * @note    ★四个字段一次投, 不许拆成四次★ 拆开投会被抢占(写端在 appcore 任务,
 *          读端在 lvgl 任务, 两者同优先级且开时间片), 读端可能看到"state 已是 DONE
 *          但 value 还是上一帧"这种逻辑上不存在的中间态. 实现把四个字段打包进一个
 *          volatile uint32 一条指令写完, 读端天然拿到自洽的一帧
 *****************************************************************************/
void watch_page_spo2_post(watch_spo2_state_e state, uint8_t remain_s,
						  uint8_t value, uint8_t finger);
/**********************************Declaring***********************************/

#endif /* __LV_WATCH_PAGE_SPO2_H__ */
