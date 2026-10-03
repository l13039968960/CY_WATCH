/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file pwm_hal.h
 *
 * @par dependencies
 * - stm32f4xx_hal.h
 * - stdint.h
 *
 * @author	zw1194
 *
 * @brief Provide the OO-style HAL APIs of PWM and corresponding opetions.
 *
 * Processing flow:
 *
 * call pwm_driver_inst() to construct, then use function pointers.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 一个实例管一个 TIM 的**全部**通道: 同一定时器的各通道共享时基(PSC/ARR),
 *       所以几路必然同频, 只能各自调占空比。要两路不同频得用两个定时器。
 *
 *****************************************************************************/
#ifndef __PWM_HAL_H__
#define __PWM_HAL_H__

/***********************************Includes***********************************/
#include "stm32f4xx_hal.h"
#include "gpio_hal.h"
#include <stdint.h>

/***********************************Includes***********************************/

/***********************************Defines************************************/
/* 通道位掩码: cfg.channels 与 pf_set_duty 的 channel 参数都用它 */
#define PWM_CHANNEL_1              (1U << 0)
#define PWM_CHANNEL_2              (1U << 1)
#define PWM_CHANNEL_3              (1U << 2)
#define PWM_CHANNEL_4              (1U << 3)

/* 通道数量: 也是 duty_percent[] 的下标范围(0=CH1 ... 3=CH4) */
#define PWM_CHANNEL_NUM            4U

/* 占空比满量程(百分比). pf_set_duty 传 0~100, 超出的返回值见函数说明 */
#define PWM_DUTY_FULL              100U
/***********************************Defines************************************/

/**********************************Declaring***********************************/
/* PWM硬件配置 */
typedef struct
{
	TIM_TypeDef *p_tim_base;    /* TIM外设基地址(如TIM4) */
	TIM_Base_InitTypeDef init;  /* 时基参数: 只填计数模式/时钟分频/自动重装预装载,
	                               Prescaler 与 Period 由驱动按 freq_hz 重算 */
	uint32_t tim_clock_hz;      /* 定时器输入时钟(Hz). 挂 APB1/APB2 的定时器在
	                               对应预分频≠1 时是 PCLK×2, 由调用方算好写进来 */
	uint32_t freq_hz;           /* PWM频率(Hz), 驱动据此算 Prescaler/Period
	                               (取值必须 1 ~ tim_clock_hz). ★只在 pf_init 时
	                               生效: 本层不提供运行中改频率的接口★ */
	uint32_t channels;          /* 启用的通道: PWM_CHANNEL_x 按位或 */
	uint8_t duty_percent[PWM_CHANNEL_NUM]; /* 各通道初始占空比(0~100), 下标0=CH1 */

	/* 通道所在 GPIO 的引脚配置(复用推挽). 本层**不用 ST 的 MSP 回调**,
	   时钟与引脚都交给 gpio_hal.
	   一条总线的多个通道通常共用一个端口, 这里也只有一组(见 @note 本驱动只管
	   一个 TIM 的全部通道) */
	gpio_cfg_t gpio;
} pwm_cfg_t;

/* PWM驱动对象 */
typedef struct pwm_driver
{
	/* 第一个成员: 内嵌TIM句柄(按值). PWM 没有中断回调, 与 adc_driver_t 一样
	 * 不需要 container_of 反查本实例 */
	TIM_HandleTypeDef htim;
	gpio_driver_t gpio; /* 通道引脚驱动实例 */
	pwm_cfg_t cfg;      /* 硬件配置 */

	uint8_t init_state; /* 初始化状态: 0=deinit, 1=init */
	uint8_t ref_count;  /* 使用者数量: 0→1 才真正初始化, 减到 0 才真正反初始化 */

	/* 构造与析构 */
	int8_t (*pf_inst)(struct pwm_driver *p_pwm_instance,
					  pwm_cfg_t *p_cfg);
	int8_t (*pf_deinst)(struct pwm_driver *p_pwm_instance);

	int8_t (*pf_init)(struct pwm_driver *p_pwm_instance);
	/* 关TIM外设并把引脚置低功耗态(模拟输入): 之后那几脚不再是 PWM 输出 */
	int8_t (*pf_deinit)(struct pwm_driver *p_pwm_instance);

	/* 改单个通道占空比: channel 传 PWM_CHANNEL_x(单个位), duty 传 0~100.
	 * 只写一次CCR加一次字节, 两个任务并发调不同通道/同一通道都不会坏 —— 不需要锁 */
	int8_t (*pf_set_duty)(struct pwm_driver *p_pwm_instance,
						  uint32_t channel,
						  uint8_t duty_percent);

} pwm_driver_t;

/* PWM驱动构造函数 */
int8_t pwm_driver_inst(pwm_driver_t *p_pwm_instance,
					   pwm_cfg_t *p_cfg);

/**********************************Declaring***********************************/

#endif // __PWM_HAL_H__
