/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_service_spo2.h
 *
 * @par dependencies
 * - stdint.h
 *
 * @author  zw1194
 *
 * @brief 血氧测量服务: 一个**按需创建、用完自退**的任务, 负责一轮血氧测量的全部
 *        业务 —— 推进 IDLE/MEASURING/DONE、算倒计时、按可信度决定要不要亮数字,
 *        并把每帧快照投给血氧页(经 lv_watch_page_spo2.h 那个零依赖口).
 *
 * Processing flow:
 *
 * 血氧页圆按钮 ──EVT_SERVICE_SPO2_BUTTON──> APP 层 ──> service_spo2_start()
 *   ──> osThreadNew ──> 任务阻塞在 osMessageQueueGet 上
 * 心率服务 ──EVT_SERVICE_HEARTRATE_DATA──> APP 层 ──> service_spo2_feed()
 *   ──> osMessageQueuePut ──> 任务醒来算一帧 ──> watch_page_spo2_post()
 * 到点 / 提前结束 / 切走页 ──> 任务投终态后自行 osThreadExit()
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note ★本服务**没有** service_spo2_init()★ 别的服务在 main.c 里挂一次初始化
 *       建常驻任务; 本任务是按需建、用完自退的 —— 没在测血氧时它不该占着栈.
 *       所以也不用往 main.c 里加任何东西.
 *
 * @note ★.c 里禁止浮点/长串格式化★ 任务栈只有 1024, 而
 *       configCHECK_FOR_STACK_OVERFLOW == 0 —— 溢出是静默踩坏内核堆, 不报错.
 *       需要格式化的数字一律留在页面侧(那边是 lvgl 任务, 4KB 栈).
 ******************************************************************************/
#ifndef __CYWATCH_SERVICE_SPO2_H__
#define __CYWATCH_SERVICE_SPO2_H__

/***********************************Includes***********************************/
#include <stdint.h>
/***********************************Includes***********************************/

/**********************************Declaring***********************************/
/******************************************************************************
 * @name    service_spo2_start
 * @brief   开始一轮测量; 已经在测就当作"提前结束"(与页面按钮是同一个语义)
 *
 * @return  无
 *
 * @note    内部按任务是否存活分三条路走, 调用方不用自己判断"在不在跑"
 * @note    ★任务正在退出的那 100ms 窗口里再按也能接住★: 不处理的话那次点击会被
 *          "正在测量"吞掉, 用户得再按一次才有反应
 *****************************************************************************/
void service_spo2_start(void);

/******************************************************************************
 * @name    service_spo2_finish
 * @brief   提前结束本轮并定格当前读数(圆内定格 → DONE)
 *
 * @return  无
 *
 * @note    没任务在跑时是空操作
 *****************************************************************************/
void service_spo2_finish(void);

/******************************************************************************
 * @name    service_spo2_cancel
 * @brief   取消本轮并让血氧页回到初始态(IDLE); 离开血氧页时调
 *
 * @return  无
 *
 * @note    ★终态是 IDLE 而不是 DONE★: 页面被 PageMem 缓存着, 下次 pf_show 会照
 *          快照重画 —— 不投 IDLE 的话切回来会看到一个冻结的"测量中"倒计时
 * @note    与 finish() 的区别只在终态: finish 是"测完了", cancel 是"这事没发生过"
 *****************************************************************************/
void service_spo2_cancel(void);

/******************************************************************************
 * @name    service_spo2_feed
 * @brief   投一帧血氧数据进队列(APP 层收到 EVT_SERVICE_HEARTRATE_DATA 时调)
 * @param   spo2_percent[in] 血氧饱和度 %, 由心率服务的帧里取
 * @param   finger_on   [in] 1 = 检测到手指贴合
 *
 * @return  无
 *
 * @note    ★按值传, 不传指针★ 心率服务那份静态帧会在下一个发布周期被覆盖
 * @note    没任务在跑就静默丢弃; 队列满也直接丢(不等), 调用方是 appcore 不能阻塞
 *****************************************************************************/
void service_spo2_feed(float spo2_percent, uint8_t finger_on);
/**********************************Declaring***********************************/

#endif /* __CYWATCH_SERVICE_SPO2_H__ */
