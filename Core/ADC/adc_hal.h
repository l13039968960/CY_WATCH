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
 * @version V2.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note V2(2026-10-03) 把本驱动改成**可共享**: 一个实例服务多个通道/
 *       多个设备. 为此做了三处改动:
 *         1. cfg 里**去掉 channel / sampling_time / gpio** —— 通道与采样时间
 *            变成 pf_read 的参数, 引脚由各调用方自己用 gpio_hal 配成模拟输入;
 *         2. pf_read 每次读之前按传入的通道重配规则组(ADC1 全芯片只有一个转换器,
 *            同一时刻只能有一路在转换);
 *         3. 加互斥量接口, 由调用方注入 —— 两个任务同时读会互相踩通道配置与
 *            ADC_DR, 必须串行化.
 *       实例本身仍然由外部(main.c)持有, 各 adapter 经 extern 引用并按
 *       ref_count 各占各放, 与 iic_hal 同一套模式.
 *****************************************************************************/
#ifndef __ADC_HAL_H__
#define __ADC_HAL_H__

/***********************************Includes***********************************/
#include "stm32f4xx_hal.h"
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
/* 互斥量接口(与 iic_hal 同构): 两个函数都不带参数, 句柄由注入方自己持有.
   不注入(NULL)时本层不加锁 */
typedef struct adc_mutex_interface
{
	int8_t (*pf_lock)(void);
	int8_t (*pf_unlock)(void);
} adc_mutex_interface_t;

/* ADC硬件配置 —— 只有"整个外设级"的参数.
   @note 通道与采样时间**不在这里**: 它们是 pf_read 的入参. 同一个 ADC1 被多个
         设备按不同通道共用, 配置是每读一次覆盖一次的.
   @note 引脚也**不在这里**: 引脚跟着通道走, 属于各自的调用方, 由它们自己用
         gpio_hal 配成 GPIO_MODE_ANALOG. 本层不碰任何 GPIO. */
typedef struct
{
	ADC_TypeDef *p_adc_base;      /* ADC外设基地址(如ADC1) */
	ADC_InitTypeDef init;         /* ADC初始化参数(分辨率/扫描/连续/触发/对齐等) */

	/* 互斥量接口, NULL = 不加锁.
	   @note 只保护 pf_read: 它要改规则组再转换, 中途被另一个任务插进来会读错通道.
	         与 iic_hal 一致, init/deinit 不加锁(它们的调用时机由调用方的
	         claim/release 决定, 本就不该并发) */
	adc_mutex_interface_t *p_mutex_interface;
} adc_cfg_t;

/* ADC驱动对象 */
typedef struct adc_driver
{
	/* 第一个成员: 内嵌ADC句柄(按值). 与 spi_driver_t 保持同样的布局, 但轮询
	 * 模式没有中断回调, 因此不依赖 container_of 反查本实例 */
	ADC_HandleTypeDef hadc;
	adc_cfg_t cfg;      /* 硬件配置 */

	uint8_t init_state; /* 初始化状态: 0=deinit, 1=init */
	uint8_t ref_count;  /* 使用本外设的设备数量: 0→1 才真正初始化, 减到 0 才真正反初始化 */

	/* 构造与析构 */
	int8_t (*pf_inst)(struct adc_driver *p_adc_instance,
					  adc_cfg_t *p_cfg);
	int8_t (*pf_deinst)(struct adc_driver *p_adc_instance);

	int8_t (*pf_init)(struct adc_driver *p_adc_instance);
	int8_t (*pf_deinit)(struct adc_driver *p_adc_instance);

	/* 采集: 按给定通道重配规则组 + 启动转换 + 等待完成 + 取码值, 合并成一次
	 * 阻塞调用. 返回的 p_value 是 0..ADC_FULL_SCALE 的原始码值.
	 *
	 * @param channel[in]       本次要采的通道(ADC_CHANNEL_x)
	 * @param sampling_time[in] 采样时间(ADC_SAMPLETIME_xCycles_x). 由调用方给:
	 *                          信号源阻抗越高要越长(分压电阻一类的场合 84 周期
	 *                          起步), 各设备的要求不一样 */
	int8_t (*pf_read)(struct adc_driver *p_adc_instance,
					  uint32_t channel, uint32_t sampling_time,
					  uint16_t *p_value);

} adc_driver_t;

/* ADC驱动构造函数 */
int8_t adc_driver_inst(adc_driver_t *p_adc_instance,
					   adc_cfg_t *p_cfg);

/**********************************Declaring***********************************/

#endif // __ADC_HAL_H__
