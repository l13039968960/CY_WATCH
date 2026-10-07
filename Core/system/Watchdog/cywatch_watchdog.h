/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_watchdog.h
 *
 * @par dependencies
 * - stdint.h
 *
 * @author	zw1194
 *
 * @brief 独立看门狗(IWDG): 对外只有"启动"与"喂狗"两个接口.
 *
 * Processing flow:
 *
 * 1. cywatch_watchdog_init() 启动看门狗, 超时约 4s, 返回前已喂过一口;
 * 2. 之后由调用方(将来的看门狗服务)按远短于超时的周期调
 *    cywatch_watchdog_feed().
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @warning ★init 一旦返回, 看门狗就在跑, 软件再也关不掉★: 启动位只能由复位
 *          清除, 没有 stop 接口. 所以调它之前必须已经安排好谁会周期喂狗,
 *          否则 MCU 约 4s 后复位. 调试时要停掉它只能改代码不调 init 重新下载.
 *
 * @note 时钟源是内部 LSI(≈32kHz), 与 HSI/PLL 无关: 主时钟跑飞时它照走, 这正是
 *       选 IWDG 的理由. 代价是 LSI 容差大(真实超时约 2.8s ~ 7.7s), 喂狗周期
 *       要留足余量, 别贴着标称的 4s 做.
 *****************************************************************************/
#ifndef __CYWATCH_WATCHDOG_H__
#define __CYWATCH_WATCHDOG_H__

#include <stdint.h>

/**********************************Declaring***********************************/

/* 启动独立看门狗(IWDG). 需在 HAL_Init() 之后调一次
   @return  0  成功
           -1 HAL_IWDG_Init 失败(SR 的 PVU/RVU 一直不落下, 即 LSI 没起振) */
int8_t cywatch_watchdog_init(void);

/* 喂狗(重装计数), 由调用方周期性调. 本函数可在任意上下文调用 */
void cywatch_watchdog_feed(void);

/**********************************Declaring***********************************/

#endif // __CYWATCH_WATCHDOG_H__
