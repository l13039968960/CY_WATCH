/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_adapter_adkey.h
 *
 * @par dependencies
 * - stdint.h
 *
 * @author	zw1194
 *
 * @brief AD 按键的 adapter: 把 BSP 驱动挂到本工程的硬件上, 并向上导出按键读取.
 *
 * Processing flow:
 *
 * key_bsp_inst() 构造(内置 ADC 初始化与采样自检)
 *   → key_bsp_read_key() 逐轮读取键号
 *   → key_bsp_deinst() 析构.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 本层把驱动挂到本工程的硬件上: ADC 侧通过 `adkey_adc_interface_t` 转发给
 *       Core/ADC/adc_hal, 延时侧用 osDelay. ADC 驱动实例由应用层 main.c 创建,
 *       本 adapter 只 extern 引用(与 W25Q64 adapter 引用 spi2_instance 同一写法),
 *       不自己再建 ADC 实例. BSP 驱动本身不碰 HAL, 只认那两个接口结构体.
 *
 * @note key_bsp_read_key() 在键号变化时会 osDelay(ADKEY_DEBOUNCE_MS) 复采 →
 *       只能在任务上下文调用, 不能放在中断里.
 *
 * @note 命名说明(非笔误): 文件名按 adapter 惯例用设备名(cywatch_adapter_adkey),
 *       导出符号按消费者名写成 `key_bsp_*`, 与 cywatch_adapter_cst816t.h 里
 *       导出 `lvgl_bsp_indev_*` 是同一套做法 —— 文件名保设备, 符号保服务.
 *****************************************************************************/
#ifndef __CYWATCH_ADAPTER_ADKEY_H__
#define __CYWATCH_ADAPTER_ADKEY_H__

/***********************************Includes***********************************/
#include <stdint.h>
/***********************************Includes***********************************/

/**********************************Declaring***********************************/
/* 构造: 构造 ADC 实例并装配 adkey_adc_interface_t / adkey_delay_interface_t 两个
   接口实例, 然后调 adkey_inst (内含 ADC 初始化与采样自检).

   返回 0 success;
   -1 ADC 实例构造失败(基地址非法, 正常运行不会出现);
   -4 ADC 初始化失败 / -5 采样自检失败 (adkey_inst 的错误码, 见驱动 @return) */
int8_t key_bsp_inst(void);

/* 析构: 反初始化 ADC 并拆掉驱动实例 */
int8_t key_bsp_deinst(void);

/* 读一次键(阻塞):
 *   p_key[out] 0 = 无按键, 1..3 = 键号(阈值见驱动头文件 ADKEY_KEYn_MIN/MAX)
 *
 * 返回 0 表示本次采样成功(不代表有按键);
 * -1 instance null / -2 p_key null / -3 首次采样失败 / -4 复采失败(见驱动 @return) */
int8_t key_bsp_read_key(uint16_t *p_key);

/**********************************Declaring***********************************/

#endif // __CYWATCH_ADAPTER_ADKEY_H__
