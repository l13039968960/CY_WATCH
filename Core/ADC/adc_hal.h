/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file adc_hal.h
 *
 * @par dependencies
 * - stm32f4xx_hal.h
 * - stdint.h
 *
 * @author zw1194
 *
 * @brief Provide the OO-style HAL APIs of ADC and corresponding opetions.
 *
 * Processing flow:
 *
 * call adc_driver_inst() to construct, then use function pointers.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
#ifndef __ADC_HAL_H__
#define __ADC_HAL_H__

/***********************************Includes***********************************/
#include "stm32f4xx_hal.h"
#include "gpio_hal.h"
#include <stdint.h>

/***********************************Includes***********************************/

/***********************************Defines************************************/
/* 单次转换的轮询超时(ms). 12bit 单通道一次转换只有几 us, 10ms 已是极大余量;
 * 它真正防的是"ADC 压根没启动"(时钟没使能 / 引脚没配), 那种情况下
 * HAL_ADC_PollForConversion 会一直返回 BUSY. */
#define ADC_POLL_TIMEOUT_MS       10

/* 转换结果满量程(12bit 分辨率). 换算电压时用:
 *   mv = value * VREF_MV / ADC_FULL_SCALE
 * ADC 只出原始码值, 参考电压是板级知识, 不由本层承担. */
#define ADC_FULL_SCALE            4095U

/***********************************Defines************************************/

/**********************************Declaring***********************************/
/* ADC硬件配置 */
typedef struct
{
	ADC_TypeDef *p_adc_base;      /* ADC外设基地址(如ADC1) */
	ADC_InitTypeDef init;         /* ADC初始化参数(分辨率/扫描/连续/触发/对齐等) */
	uint32_t channel;             /* 规则组通道(ADC_CHANNEL_x) */
	uint32_t sampling_time;       /* 采样时间(ADC_SAMPLETIME_xCycles_x) */

	/* 通道所在 GPIO 的引脚配置(模拟输入, mode 填 GPIO_MODE_ANALOG).
	   本层**不用 ST 的 MSP 回调**, 时钟与引脚都交给 gpio_hal */
	gpio_cfg_t gpio;

	/* @note 单通道时 init.ScanConvMode 必须 DISABLE、init.NbrOfConversion 必须 1,
	 *       本层会按此设定规则组的 Rank = 1.
	 * @note 采样时间要配足: 信号源阻抗越高需要越长. 分压电阻一类的场合用
	 *       ADC_SAMPLETIME_84CYCLES 起步; 内部通道(VREFINT/TEMPSENSOR)有最小
	 *       采样时间要求(约 10us), 给得太短读数会偏低. */
} adc_cfg_t;

/* ADC驱动对象 */
typedef struct adc_driver
{
	/* 第一个成员: 内嵌ADC句柄(按值). 与 spi_driver_t 保持同样的布局, 但轮询
	 * 模式没有中断回调, 因此不依赖 container_of 反查本实例 */
	ADC_HandleTypeDef hadc;
	gpio_driver_t gpio; /* 通道引脚驱动实例 */
	adc_cfg_t cfg;      /* 硬件配置 */

	uint8_t init_state; /* 初始化状态: 0=deinit, 1=init */
	uint8_t ref_count;  /* 使用本外设的实例数量: 0→1 才真正初始化, 减到 0 才真正反初始化 */

	/* 构造与析构 */
	int8_t (*pf_inst)(struct adc_driver *p_adc_instance,
					  adc_cfg_t *p_cfg);
	int8_t (*pf_deinst)(struct adc_driver *p_adc_instance);

	int8_t (*pf_init)(struct adc_driver *p_adc_instance);
	int8_t (*pf_deinit)(struct adc_driver *p_adc_instance);

	/* 采集: 启动转换 + 等待完成 + 取码值, 三步合并成一次阻塞调用.
	 * 返回的 p_value 是 0..ADC_FULL_SCALE 的原始码值. */
	int8_t (*pf_read)(struct adc_driver *p_adc_instance,
					  uint16_t *p_value);

} adc_driver_t;

/* ADC驱动构造函数 */
int8_t adc_driver_inst(adc_driver_t *p_adc_instance,
					   adc_cfg_t *p_cfg);

/**********************************Declaring***********************************/

#endif // __ADC_HAL_H__
