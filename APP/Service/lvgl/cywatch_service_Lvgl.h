/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_service_lvgl.h
 *
 * @par dependencies
 * - stdint.h
 *
 * @author	zw1194
 *
 * @brief LVGL 服务: 托管 LVGL 运行时(显示 + 触摸输入 + 时基), 一个任务里跑 UI 泵.
 *
 * Processing flow:
 *
 * service_lvgl_init()(本文件) 只建任务;
 * 任务体内: lv_init → lv_tick_set_cb → lv_port_disp_init(ST7789 adapter)
 *          → lv_port_indev_init(CST816T adapter) → lv_watch_ui_init
 *          → while(1){ lv_timer_handler(); osDelay(LOOP_TICK); }.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note ★跨任务访问契约(必读)★
 *       `LV_USE_OS = LV_OS_NONE`, LVGL 内部**没有任何锁**. 因此:
 *         1. 所有 `lv_*` API 只能在**本服务的 lvgl 任务**里调用 —— 也就是
 *            lv_timer_handler 的上下文. 页面/其他任务的 LVGL widget 回调
 *            (lv_obj_add_event_cb 注册的那些)正好也在该上下文里执行, 可以安全
 *            调 lv_* 与 x_port_easyapp_event_send(无阻塞环形push);
 *         2. 任何**别的任务**想改 UI, 必须经 osMessageQueue 把请求投给 lvgl 任务
 *            消费, 不能直接调 lv_*(会与 lv_timer_handler 并发撕裂对象树);
 *         3. ISR 里禁止调任何 lv_*, 也禁止在 ISR 里做触摸坐标换算(软 I2C 不可重入);
 *         4. widget 回调里禁止 osDelay/等信号量/长耗时 I2C 读 —— 那会卡住整个 UI.
 *       将来若有第二个真正的 LVGL 消费者, 再把 lv_conf.h 的 LV_USE_OS 改成
 *       LV_OS_FREERTOS 引入递归互斥, 现在不必.
 *
 * @note 本服务**不发 EasyAPP 事件**, 也不注册页面: 触摸事件由各页面的 LVGL
 *       widget 回调直接发出(见上). 故 `easyapp_page.h` 的事件枚举无需改动.
 ******************************************************************************/
#ifndef __CYWATCH_SERVICE_LVGL_H__
#define __CYWATCH_SERVICE_LVGL_H__

/***********************************Includes***********************************/
#include <stdint.h>
/***********************************Includes***********************************/

/**********************************Declaring***********************************/
/* 启动LVGL服务: 只创建 "lvgl" 任务(栈4KB, Normal优先级), 不做任何设备访问.
 * 真正的 lv_init/显示/触摸初始化在任务体内跑 —— adapter 的 inst 内含 osDelay,
 * 必须等 osKernelStart() 之后.
 *
 * 返回 0 success; -1 任务创建失败 */
int8_t service_lvgl_init(void);
/**********************************Declaring***********************************/

#endif // __CYWATCH_SERVICE_LVGL_H__
