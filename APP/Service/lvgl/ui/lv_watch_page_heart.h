/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file lv_watch_page_heart.h
 *
 * @par dependencies
 * - stdint.h
 *
 * @author	zw1194
 *
 * @brief 心率页(lv_watch_page_heart.c)的设置口: 让别的任务把心率值投进来,
 *        真正的大字 label 与折线图留在本页 1s 刷新定时器里画.
 *        范本: lv_watch_page_home.h.
 *
 * Processing flow:
 *
 * 调用方(APP 层, app_core 任务)                lvgl 任务
 *   watch_page_heart_set_hr(76)   ──┐
 *   watch_page_heart_push_hr(76)  ──┴──> 写本页静态值 ──> 本页 1s 定时器渲染
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note ★本头**刻意零依赖**(只 include stdint.h), 别往里加 lvgl.h 或
 *       lv_watch_page.h★ 理由与 lv_watch_page_home.h 同: 调用方跑在 appcore 任务
 *       上下文里, **绝不能拿到 lv_xxx() 的原型** —— LVGL 无锁, 全工程只有
 *       "lvgl" 任务能碰对象树(见 service/Lvgl/cywatch_service_lvgl.h 跨任务契约).
 *
 * @note ★异步, 不是立刻生效★ set 只把值写进本页就返回, 实际渲染最迟在本页
 *       下一个 1s 节拍(或 pf_show 补帧)完成. 页面**还没创建**时投的值也不丢:
 *       值存在页面结构体里, pf_create 会直接画出当前值.
 *
 * @note ★折线图的历史(最近30点)跨重建保留★: 它存在页面结构体里, 不依赖 chart 对象
 *       —— PageMem 淘汰本页后 chart 被删, 但下次重画会从历史整份重建, 波形接得上而
 *       不必重新攒. 详见 lv_watch_page_heart.c 的 hist 说明.
 *
 * @note ★历史由 watch_page_heart_push_hr() 推进, 与"本页是否在显示"无关★: 调用方是
 *       APP 层一个**常驻(永远 enable)**的记录器, 它不管当前显示的是哪一页, 所以
 *       切走之后历史照样在长 —— 而 set_hr 那个口挂在心率页自己的 handler 上, 页面
 *       一失能就收不到事件了. 两个口的分工: set_hr 管"大字现在显示什么", push_hr
 *       管"历史里追加一格".
 ******************************************************************************/
#ifndef __LV_WATCH_PAGE_HEART_H__
#define __LV_WATCH_PAGE_HEART_H__

/***********************************Includes***********************************/
#include <stdint.h>
/***********************************Includes***********************************/

/**********************************Declaring***********************************/
/******************************************************************************
 * @name    watch_page_heart_set_hr
 * @brief   写心率页的大字(异步, 最迟下一个 1s 节拍生效)
 * @param   bpm[in] 心率; 0 = 没有有效读数(没贴手指), 大写 0
 *
 * @return  无(恒成功, 所以返回 void, 与 watch_page_home_set_data 同一取舍)
 *****************************************************************************/
void watch_page_heart_set_hr(uint8_t bpm);

/******************************************************************************
 * @name    watch_page_heart_push_hr
 * @brief   往折线图历史的末尾追加一格(异步, 最迟下一个 1s 节拍画上去)
 * @param   bpm[in] 心率; 0 = 没有有效读数(没贴手指), 图线落到底
 *
 * @return  无
 *
 * @note    实现只写页面结构体里那份 hist, **不碰任何控件**, 所以可以由别的任务调
 *          (appcore 任务里的常驻记录器就是这么用的)
 * @note    调用节奏决定图的横向窗口: 图固定 30 格, 每秒 push 一次 = 30 秒窗
 *****************************************************************************/
void watch_page_heart_push_hr(uint8_t bpm);
/**********************************Declaring***********************************/

#endif /* __LV_WATCH_PAGE_HEART_H__ */
