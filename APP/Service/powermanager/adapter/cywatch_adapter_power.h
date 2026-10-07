/******************************************************************************
 * Copyright (C) 2026 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_adapter_power.h
 *
 * @par dependencies
 * - stdint.h
 *
 * @author zw1194
 *
 * @brief 电源适配层: 用共享的 ADC1 读电池分压, 换算成整数毫伏; 另管 PB1 的
 *        充电检测(EXTI 双边沿 + 信号量).
 *
 * Processing flow:
 *
 * power_bsp_inst()      : 占住共享 ADC1 + 把 PB0 配成模拟输入 + 采一次自检
 *                         + 开 PB1 的 EXTI(双边沿)与唤醒信号量
 * power_bsp_read_battery_mv(): 采一次并换算
 * power_bsp_is_charging()     : 读 PB1 当前电平(充电=低)
 * power_bsp_wait_charge()     : 阻塞等一次充电状态边沿(带超时), 由 ISR 放信号量唤醒
 * power_bsp_deinst()    : 收引脚 + 放掉共享 ADC1 的占用
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 本层不含周期、不含事件 —— 那些在 cywatch_service_power.c 里.
 *****************************************************************************/
#ifndef __CYWATCH_ADAPTER_POWER_H__
#define __CYWATCH_ADAPTER_POWER_H__

/***********************************Includes***********************************/
#include <stdint.h>
/***********************************Includes***********************************/

/**********************************Declaring***********************************/
/* 占住共享 ADC1(PB0/ADC1_IN8 配成模拟输入)并采一次自检
   @return  0  成功
           -1  自检采样失败(见 adc_hal 的 pf_read: 时钟没起/引脚没配) */
int8_t power_bsp_inst(void);

/* 收引脚 + 放掉共享 ADC1 上本设备那一份占用. @return 恒 0 */
int8_t power_bsp_deinst(void);

/* 读一次电池电压
   @param  p_mv[out] 电池电压, 整数毫伏(分压比与 VREF 已在本层算掉)
   @return 0  成功
           -2 采样失败(见 adc_hal 的 pf_read) */
int8_t power_bsp_read_battery_mv(uint16_t *p_mv);

/* 读充电检测脚 PB1 的当前电平(充电时低电平)
   @return 1 充电中 / 0 未充电(引脚还没占用或已被释放时也返回 0) */
uint8_t power_bsp_is_charging(void);

/* 等 PB1 上出现一次充电状态边沿, 或等到超时
   @param  timeout_ms[in] 最长等待毫秒数
   @return 0  被边沿叫醒(状态可能刚变, 再调 power_bsp_is_charging 读真值)
           -1 超时, 或信号量还没建起来
   @note   只能任务上下文调用; 返回 -1 只代表超时, 不代表出错 */
int8_t power_bsp_wait_charge(uint32_t timeout_ms);
/**********************************Declaring***********************************/

#endif // __CYWATCH_ADAPTER_POWER_H__
