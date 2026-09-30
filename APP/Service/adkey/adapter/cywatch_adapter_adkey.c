/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_adapter_adkey.c
 *
 * @par dependencies
 * - cywatch_adapter_adkey.h
 * - cywatch_bsp_adkey_driver.h
 * - adc_hal.h
 * - cmsis_os2.h
 *
 * @author zw1194
 *
 * @brief Implete the AD adapter: 装配 BSP 驱动的两个接口, 并转发按键读取.
 *
 * Processing flow:
 *
 * key_bsp_inst() 构造 ADC 实例 → 挂接口 → adkey_inst (内含 ADC 初始化与采样自检)
 *   → key_bsp_read_key() 转发到驱动的 pf_read_key
 *   → key_bsp_deinst() 转发到驱动的 pf_deinst.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note ADKEY 独占 ADC1_IN2 (PA2), 实例在本层创建(不与别的设备共享, 故不像 W25Q64
 *       adapter 那样 extern 应用层的总线实例). PA2 的模拟模式在 HAL_ADC_MspInit
 *       里配置, 由 adc_init 内部的 HAL_ADC_Init 回调触发。
 *
 * @note 本层只做"转发 + 接口装配", 不含按键阈值判定与去抖 —— 那两样在 BSP 驱动里。
 *****************************************************************************/
#include "cywatch_adapter_adkey.h"

#include "cywatch_bsp_adkey_driver.h"
#include "adc_hal.h"   /* adc_driver_t: ADKEY 独占的 ADC1 */
#include "cmsis_os2.h" /* osDelay */

/***********************************Defines************************************/
/* ADKEY 接在 ADC1_IN2 (PA2). 采样时间 84 周期: AD 按键是电阻分压, 给短了读数会偏低 */
#define ADKEY_ADC_BASE     ADC1
#define ADKEY_ADC_CHANNEL  ADC_CHANNEL_2
#define ADKEY_ADC_SAMPLING ADC_SAMPLETIME_84CYCLES

/***********************************Defines************************************/

/**********************************Declaring***********************************/
static bsp_adkey_driver_t adkey_instance;

/* ADKEY 独占的 ADC 实例 */
static adc_driver_t adkey_adc_instance;

static adkey_adc_interface_t   adkey_adc_interface_instance;
static adkey_delay_interface_t adkey_delay_interface_instance;

/******************************************************************************
 * @name    adkey_adc_init
 * @brief   adkey_adc_interface_t.pf_init 的实现: 转发到 adc_hal
 * @param   无
 *
 * @return  见 adc_hal 的 pf_init
 *****************************************************************************/
static int8_t adkey_adc_init(void)
{
	return adkey_adc_instance.pf_init(&adkey_adc_instance);
}

/******************************************************************************
 * @name    adkey_adc_deinit
 * @brief   adkey_adc_interface_t.pf_deinit 的实现: 转发到 adc_hal
 * @param   无
 *
 * @return  见 adc_hal 的 pf_deinit
 *****************************************************************************/
static int8_t adkey_adc_deinit(void)
{
	return adkey_adc_instance.pf_deinit(&adkey_adc_instance);
}

/******************************************************************************
 * @name    adkey_adc_get_value
 * @brief   adkey_adc_interface_t.pf_get_value 的实现: 采一次, 转发到 adc_hal
 * @param   value[out] 转换结果
 *
 * @return  见 adc_hal 的 pf_read
 *****************************************************************************/
static int8_t adkey_adc_get_value(uint16_t *value)
{
	return adkey_adc_instance.pf_read(&adkey_adc_instance, value);
}

/******************************************************************************
 * @name    adkey_delay_cb
 * @brief   adkey_delay_interface_t.pf_delay 的实现: 去抖复采的延时
 * @param   ms[in] 延时长度(ms)
 *
 * @return  无
 *****************************************************************************/
static void adkey_delay_cb(uint32_t ms)
{
	(void)osDelay(ms);
}

/******************************************************************************
 * @name    key_bsp_inst
 * @brief   构造 ADC 实例 → 挂两个接口 → 构造 ADKEY 驱动(内含初始化与自检)
 * @param   无
 *
 * @return  0 success
 *         -1 ADC实例构造失败
 *         其余为 adkey_inst 的错误码(见驱动 @return)
 *
 * @note    adc_driver_inst 与 adkey_inst 都不碰硬件(只装配结构体), 真正开 ADC1 时钟
 *          与配 PA2 模拟模式发生在 adkey_inst 内部的 pf_init → adc_init 里。
 *****************************************************************************/
int8_t key_bsp_inst(void)
{
	adc_cfg_t adc_cfg = {0};

	/* 1. 构造 ADC 实例(单通道, 软件启动, 每次 pf_read 转一次) */
	adc_cfg.p_adc_base    = ADKEY_ADC_BASE;
	adc_cfg.channel       = ADKEY_ADC_CHANNEL;
	adc_cfg.sampling_time = ADKEY_ADC_SAMPLING;

	adc_cfg.init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
	adc_cfg.init.Resolution            = ADC_RESOLUTION_12B;
	adc_cfg.init.DataAlign             = ADC_DATAALIGN_RIGHT;
	adc_cfg.init.ScanConvMode          = DISABLE;
	adc_cfg.init.EOCSelection          = ADC_EOC_SINGLE_CONV;
	adc_cfg.init.ContinuousConvMode    = DISABLE;
	adc_cfg.init.NbrOfConversion       = 1U;
	adc_cfg.init.DiscontinuousConvMode = DISABLE;
	adc_cfg.init.NbrOfDiscConversion   = 0U;
	adc_cfg.init.ExternalTrigConv      = ADC_SOFTWARE_START;
	adc_cfg.init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
	adc_cfg.init.DMAContinuousRequests = DISABLE;

	if (0 != adc_driver_inst(&adkey_adc_instance, &adc_cfg))
	{
		return -1;
	}

	/* 2. 挂两个接口(每个都对应驱动 adkey_inst 的一处非空校验) */
	adkey_adc_interface_instance.pf_init      = adkey_adc_init;
	adkey_adc_interface_instance.pf_deinit    = adkey_adc_deinit;
	adkey_adc_interface_instance.pf_get_value = adkey_adc_get_value;

	adkey_delay_interface_instance.pf_delay   = adkey_delay_cb;

	/* 3. 构造驱动: 内部回调 adc 的 pf_init(初始化 ADC1 + 配 PA2 模拟模式), 再采一次
	      做自检, 失败即在此返回 */
	return adkey_inst(&adkey_instance,
					  &adkey_adc_interface_instance,
					  &adkey_delay_interface_instance);
}

/******************************************************************************
 * @name    key_bsp_deinst
 * @brief   析构: 转发到驱动的 pf_deinst
 * @param   无
 *
 * @return  见驱动的 pf_deinst
 *****************************************************************************/
int8_t key_bsp_deinst(void)
{
	return adkey_instance.pf_deinst(&adkey_instance);
}

/******************************************************************************
 * @name    key_bsp_read_key
 * @brief   读一次键: 转发到驱动的 pf_read_key
 * @param   p_key[out] 0 = 无按键, 1..3 = 键号
 *
 * @return  见驱动的 pf_read_key
 *****************************************************************************/
int8_t key_bsp_read_key(uint16_t *p_key)
{
	return adkey_instance.pf_read_key(&adkey_instance, p_key);
}
