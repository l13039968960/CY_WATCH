/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file lv_watch_page_home.h
 *
 * @par dependencies
 * - stdint.h
 *
 * @author	zw1194
 *
 * @brief 表盘页(lv_watch_page_home.c)的设置口: 让别的任务把卡片(步数/心率/温度)、
 *        电量与蓝牙状态投进来, 真正的 lv_label_set_text 留在本页 1s 刷新定时器里做.
 *        范本: lv_watch_page_ota.h.
 *
 * Processing flow:
 *
 * 调用方(APP 层, app_core 任务)                lvgl 任务
 *   watch_page_home_set_data(TEMP, 25)  ──> 写本页结构体 ──> 本页 1s 定时器渲染
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note ★本头**刻意零依赖**(只 include stdint.h), 别往里加 lvgl.h 或
 *       lv_watch_page.h★ 理由与 lv_watch_page_ota.h 同: 调用方跑在 appcore 任务
 *       上下文里, **绝不能拿到 lv_xxx() 的原型** —— LVGL 无锁, 全工程只有
 *       "lvgl" 任务能碰对象树(见 service/Lvgl/cywatch_service_lvgl.h 跨任务契约).
 *
 * @note ★异步, 不是立刻生效★ set 只把值写进本页就返回, 实际渲染最迟在本页
 *       下一个 1s 刷新节拍(或 pf_show 补帧)完成. 页面**还没创建**时投的值也不丢:
 *       值存在页面结构体里, pf_create 会直接画出当前值.
 ******************************************************************************/
#ifndef __LV_WATCH_PAGE_HOME_H__
#define __LV_WATCH_PAGE_HOME_H__

/***********************************Includes***********************************/
#include <stdint.h>
/***********************************Includes***********************************/

/**********************************Declaring***********************************/
/* 表盘页可写的那几项 */
typedef enum
{
	WATCH_HOME_DATA_STEP = 0, /* 步数 */
	WATCH_HOME_DATA_HR,		  /* 心率, bpm */
	WATCH_HOME_DATA_TEMP,	  /* 温度, 整数摄氏度 */
	WATCH_HOME_DATA_BATTERY,  /* 电量, 0~100 百分比(由调用方换算好) */
	WATCH_HOME_DATA_CHARGING, /* 充电状态: 1=充电中, 0=未充电 */
	WATCH_HOME_DATA_BT,		  /* 蓝牙连接: 1=已连接, 0=未连接 */
} watch_home_data_e;

/******************************************************************************
 * @name    watch_page_home_set_data
 * @brief   写表盘页三张卡片中的一张(异步, 最迟下一个 1s 节拍生效)
 * @param   which[in] 要写哪一项(WATCH_HOME_DATA_xxx)
 * @param   value[in] 新值: 步数原值 / 心率 bpm / 整数摄氏度 / 电量 0~100 / 充电 1或0 /
 *                     蓝牙连接 1或0
 *
 * @return  无(恒成功, 所以返回 void, 与 watch_switch(...) 同一取舍)
 *
 * @note    which 越界静默忽略; 值超出该项类型范围会被截断
 *****************************************************************************/
void watch_page_home_set_data(watch_home_data_e which, int32_t value);
/**********************************Declaring***********************************/

#endif /* __LV_WATCH_PAGE_HOME_H__ */
