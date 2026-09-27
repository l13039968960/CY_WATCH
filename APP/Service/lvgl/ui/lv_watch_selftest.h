/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file lv_watch_selftest.h
 *
 * @par dependencies
 * - lvgl.h
 * - lv_watch_page.h
 * - lv_watch_ui.h
 *
 * @author zw1194
 *
 * @brief 手表UI上电自检: 把"表盘/菜单/心率"三页的真实生命周期(创建→缓存→
 *        隐藏→销毁→重建)连同定时器与LVGL内存池的账目一起断言掉, 结果打printf.
 *        实现在 lv_watch_selftest.c, 步骤与判据的完整说明也在那里.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note ★只能在 "lvgl" 任务里调, 且必须在 lv_watch_ui_init() 成功之后★
 *       (它要观察的管理器与三个页面都是 lv_watch_ui_init 建出来的;
 *        LV_USE_OS=LV_OS_NONE, LVGL 内部无锁, 跨任务调用没有保护).
 *
 * @note 本自检**始终编进固件、上电自动跑**(用户选择, 没有编译开关).
 *       要撤掉就是三处: 删本文件与 .c、删服务里那一行调用、删工程组条目.
 ******************************************************************************/
#ifndef __LV_WATCH_SELFTEST_H__
#define __LV_WATCH_SELFTEST_H__

/***********************************Includes***********************************/
#include <stdint.h>
/***********************************Includes***********************************/

/**********************************Declaring***********************************/

/******************************************************************************
 * @name    lv_watch_selftest_start
 * @brief   启动手表UI自检(只注册驱动定时器, 不立刻执行第一步)
 * @param   无
 *
 * @return  0  success(自检已排队, 从下一个 lv_timer_handler 开始按步推进)
 *         -1  重复启动(已经跑过/正在跑)
 *
 * @note    返回 0 **不代表自检通过** —— 它只是"排上队了". 逐步结果与最终结论
 *          都从 USART1(PA9) 的 printf 看: 步骤行 `[UI] N. 步骤名 PASS/FAIL: ...`,
 *          汇总行 `[UI] ===== 自检结束: ... =====` 在所有步骤之后打印.
 * @note    推进由 lv_timer_handler() 带动, 所以调用后必须继续跑 UI 泵
 *****************************************************************************/
int8_t lv_watch_selftest_start(void);

#endif /* __LV_WATCH_SELFTEST_H__ */
