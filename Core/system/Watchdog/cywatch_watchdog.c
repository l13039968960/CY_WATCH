/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_watchdog.c
 *
 * @par dependencies
 * - cywatch_watchdog.h
 * - stm32f4xx_hal.h (HAL 的 IWDG 驱动; 需要 hal_conf.h 里把
 *   HAL_IWDG_MODULE_ENABLED 打开, 否则 IWDG_HandleTypeDef 根本不定义)
 *
 * @author	zw1194
 *
 * @brief 独立看门狗的实现: STM32 的 IWDG 外设(时钟源 = LSI ≈32kHz).
 *
 * Processing flow:
 *
 * call directly.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 超时 = (RLR + 1) / (LSI / 预分频) = 4096 / (32000 / 32) = 4096 × 1ms
 *       ≈ 4.096s. 即 PR=IWDG_PRESCALER_32 把 32kHz 分成 1kHz(1ms 一 tick),
 *       再由 RLR=4095 数 4096 个 tick. 这颗 F411 的 IWDG 最大档是 256 分频
 *       + RLR=4095 ≈ 32.7s.
 *
 * @note 走 HAL 而不直接写寄存器, 与工程里其它外设一致. 关键是 HAL_IWDG_Init
 *       结尾那段"等 SR 的 PVU/RVU 落下去": PR/RLR 是同步寄存器, 写下去要几个
 *       LSI 周期才真正生效, 不等就喂狗会按旧参数重装.
 *****************************************************************************/
#include "cywatch_watchdog.h"

#include <stm32f4xx_hal.h>

/***********************************Defines************************************/
/* 预分频与重装值: 合起来 ≈4.096s, 见文件头超时算法 */
#define CYWATCH_IWDG_PRESCALER       (IWDG_PRESCALER_32)
#define CYWATCH_IWDG_RELOAD          (4095U)
/***********************************Defines************************************/

/**********************************Declaring***********************************/
/* 本模块独占的 HAL 句柄. static: IWDG 全芯片只有一个, 不做多实例.
   静态存储保证 Init 之外的成员(Lock 等)是 0, 不用逐个赋 */
static IWDG_HandleTypeDef s_iwdg_handle;
/**********************************Declaring***********************************/

/******************************************************************************
 * @name    cywatch_watchdog_init
 * @brief   启动独立看门狗(≈4.096s), 返回前已喂过第一口
 * @param   无
 *
 * @return  0  成功
 *          -1 寄存器同步超时(LSI 没起振, PR/RLR 写不进去)
 *
 * @warning 这里是"启动"而不是"配置": HAL_IWDG_Init 第一句就写 KR=0xCCCC 把
 *          看门狗打开(同时自动使能 LSI), 之后软件再也关不掉. 所以即便本函数
 *          返回 -1, 看门狗可能已经在跑了 —— 调用方若不停机, 必须有喂狗者.
 *****************************************************************************/
int8_t cywatch_watchdog_init(void)
{
    s_iwdg_handle.Instance       = IWDG;
    s_iwdg_handle.Init.Prescaler = CYWATCH_IWDG_PRESCALER;
    s_iwdg_handle.Init.Reload    = CYWATCH_IWDG_RELOAD;

    if (HAL_OK != HAL_IWDG_Init(&s_iwdg_handle))
    {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @name    cywatch_watchdog_feed
 * @brief   喂狗: 把 IWDG 计数器按 RLR 重装
 * @param   无
 *
 * @return  无
 *
 * @note    返回值只有"句柄为 NULL"一种失败(句柄由本模块自己传, 不会发生),
 *          故忽略; 也不能在这里判断返回值 —— 判断本身也救不了, 只剩复位
 *****************************************************************************/
void cywatch_watchdog_feed(void)
{
    (void)HAL_IWDG_Refresh(&s_iwdg_handle);
}
