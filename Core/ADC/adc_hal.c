/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file adc_hal.c
 *
 * @par dependencies
 * - adc_hal.h
 *
 * @author zw1194
 *
 * @brief Implete the OO-style HAL operations of ADC.
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
#include "adc_hal.h"

/********************************* 前向声明 *********************************/
static int8_t adc_deinst(adc_driver_t *p_adc_instance);
static int8_t adc_init(adc_driver_t *p_adc_instance);
static int8_t adc_deinit(adc_driver_t *p_adc_instance);
static int8_t adc_read(adc_driver_t *p_adc_instance, uint16_t *p_value);

/********************************* 私有辅助 *********************************/

/******************************************************************************
 * @name    adc_clk_enable
 * @brief   开本 ADC 外设的时钟(引脚时钟由 gpio_hal 负责)
 * @param   p_adc[in]   ADC外设基地址
 *
 * @note    只能按基地址分派: RCC 的时钟使能宏连寄存器位都是编译期固定的
 *          (__HAL_RCC_ADC1_CLK_ENABLE 直接写 APB2ENR 的 ADC1EN), HAL 没有
 *          "按基地址查位" 的通用 API.
 *****************************************************************************/
static void adc_clk_enable(ADC_TypeDef *p_adc)
{
	if (ADC1 == p_adc) { __HAL_RCC_ADC1_CLK_ENABLE(); }
}

/******************************************************************************
 * @name    adc_clk_disable
 * @brief   关本 ADC 的时钟(pf_deinit 收尾用)
 * @param   p_adc[in] ADC外设基地址
 *
 * @note    ★只关 ADC 外设时钟, 不关 GPIO 端口时钟★ —— 端口上还有 UART/EXTI/IIC
 *          共用, 连端口时钟一起关会把它们一起打死.
 *****************************************************************************/
static void adc_clk_disable(ADC_TypeDef *p_adc)
{
	if (ADC1 == p_adc) { __HAL_RCC_ADC1_CLK_DISABLE(); }
}

/********************************* 构造与析构 *********************************/

/******************************************************************************
 * @name    adc_init
 * @brief   ADC驱动初始化: 初始化ADC外设 + 配规则组通道
 * @param   p_adc_instance[in]
 *
 * @return  0 success
 *         -1 adc_instance null
 *         -2 HAL_ADC_Init error
 *         -3 通道配置失败
 *
 * @note    ADC 的时钟与引脚都在本函数里配, 不走 ST 的 MSP 回调
 *          (HAL_ADC_MspInit 未实现, HAL 自带的那个空弱函数不会动 GPIO).
 *
 * @note    引用计数: ref_count 由 0→1 时才真正初始化外设, 已有使用者时只累加计数
 *          (同一实例被重复init不会把外设重配一遍)。
 *****************************************************************************/
static int8_t adc_init(adc_driver_t *p_adc_instance)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	if (NULL == p_adc_instance)
	{
		return -1;
	}

	/* 首个使用者才真正初始化ADC外设 */
	if (0 == p_adc_instance->ref_count)
	{
		/* 时钟与引脚都在这里配: 后面 HAL_ADC_Init 内部那次 HAL_ADC_MspInit
		   是空弱函数, 不会再动 GPIO —— 所以这一步不能省, 也不能挪到后面 */
		adc_clk_enable(p_adc_instance->cfg.p_adc_base);

		(void)p_adc_instance->gpio.pf_init(&p_adc_instance->gpio);

		if (HAL_OK != HAL_ADC_Init(&p_adc_instance->hadc))
		{
			return -2;
		}

		/* 单通道: 规则组只有一项, Rank固定为1 */
		sConfig.Channel = p_adc_instance->cfg.channel;
		sConfig.Rank = 1;
		sConfig.SamplingTime = p_adc_instance->cfg.sampling_time;
		sConfig.Offset = 0;
		if (HAL_OK != HAL_ADC_ConfigChannel(&p_adc_instance->hadc, &sConfig))
		{
			return -3;
		}

		p_adc_instance->init_state = 1;
	}

	p_adc_instance->ref_count++;

	return 0;
}

/******************************************************************************
 * @name    adc_deinit
 * @brief   ADC驱动反初始化: 关闭ADC外设
 * @param   p_adc_instance[in]
 *
 * @return  0 success
 *         -1 adc_instance null
 *
 * @note    HAL_ADC_DeInit 回调的 MspDeInit 是空弱函数, 引脚与时钟由本函数自己收尾.
 * @note    本函数有**低功耗语义**: 引脚不能只 DeInit 到复位态 ——
 *          F4 的 GPIO 复位值是**浮空输入**(不是模拟), 施密特触发器还开着, 悬空脚
 *          会随噪声来回翻转、白耗电。要再配成模拟输入才算真的关掉输入缓冲。
 * @note    引用计数: 每个使用者退出时减1, 减到0才真正关闭外设。
 *****************************************************************************/
static int8_t adc_deinit(adc_driver_t *p_adc_instance)
{
	if (NULL == p_adc_instance)
	{
		return -1;
	}

	/* 退出一个使用者(已在0则不再减, 防uint8下溢后永远回不到0) */
	if (p_adc_instance->ref_count > 0)
	{
		p_adc_instance->ref_count--;
	}

	/* 最后一个使用者退出时才真正关闭ADC外设 */
	if (0 == p_adc_instance->ref_count)
	{
		HAL_ADC_DeInit(&p_adc_instance->hadc);

		/* 引脚收尾(DeInit → 模拟输入, 顺序在 gpio_hal 里) */
		(void)p_adc_instance->gpio.pf_deinit(&p_adc_instance->gpio);

		/* ★只关 ADC 时钟★: 端口上还有 UART/EXTI/IIC 共用, 端口时钟不能关 */
		adc_clk_disable(p_adc_instance->cfg.p_adc_base);

		p_adc_instance->init_state = 0;
	}

	return 0;
}

/******************************************************************************
 * @name    adc_deinst
 * @brief   ADC驱动析构: 清除函数指针
 * @param   p_adc_instance[in]
 *
 * @return  0 success
 *         -1 adc_instance null
 *         -2 仍有使用者(ref_count != 0), 本次不析构
 *
 * @note    本函数不关外设 —— 关闭ADC外设走pf_deinit。析构会清空pf_read等全部
 *          指针, 等于废掉整个实例, 所以只在没有使用者时才允许执行。
 *****************************************************************************/
static int8_t adc_deinst(adc_driver_t *p_adc_instance)
{
	if (NULL == p_adc_instance)
	{
		return -1;
	}

	/* 仍有使用者: 清指针会把复用本实例的其他设备一起废掉 */
	if (0 != p_adc_instance->ref_count)
	{
		return -2;
	}

	p_adc_instance->init_state = 0;
	p_adc_instance->ref_count = 0;

	p_adc_instance->pf_inst = NULL;
	p_adc_instance->pf_deinst = NULL;
	p_adc_instance->pf_init = NULL;
	p_adc_instance->pf_deinit = NULL;
	p_adc_instance->pf_read = NULL;

	return 0;
}

/********************************* 采集 *********************************/

/******************************************************************************
 * @name    adc_read
 * @brief   采一次: 启动转换 -> 等待完成 -> 取码值
 * @param   p_adc_instance[in]
 * @param   p_value[out] 转换结果(0..ADC_FULL_SCALE)
 *
 * @return  0 success
 *         -1 adc_instance null
 *         -2 p_value null
 *         -3 未初始化(pf_init 没调过)
 *         -4 HAL_ADC_Start error
 *         -5 转换超时
 *
 * @note    超时会返回而不是死等: ADC没启动(时钟没使能/引脚没配成模拟)时
 *          HAL_ADC_PollForConversion 会一直返回BUSY, 有超时才能暴露出来。
 *          超时路径上也调了 Stop, 免得ADC停在运行态影响下一次采集。
 *****************************************************************************/
static int8_t adc_read(adc_driver_t *p_adc_instance, uint16_t *p_value)
{
	if (NULL == p_adc_instance)
	{
		return -1;
	}

	if (NULL == p_value)
	{
		return -2;
	}

	if (0 == p_adc_instance->init_state)
	{
		return -3;
	}

	if (HAL_OK != HAL_ADC_Start(&p_adc_instance->hadc))
	{
		return -4;
	}

	if (HAL_OK != HAL_ADC_PollForConversion(&p_adc_instance->hadc, ADC_POLL_TIMEOUT_MS))
	{
		(void)HAL_ADC_Stop(&p_adc_instance->hadc);
		return -5;
	}

	*p_value = (uint16_t)HAL_ADC_GetValue(&p_adc_instance->hadc);

	(void)HAL_ADC_Stop(&p_adc_instance->hadc);

	return 0;
}

/********************************* 构造与析构 *********************************/

/******************************************************************************
 * @name    adc_driver_inst
 * @brief   ADC驱动构造函数: 加载配置、装配内嵌句柄、挂载函数指针
 *
 *          本函数不碰硬件, ADC外设的真正初始化在pf_init()里做。
 *
 * @param   p_adc_instance[out] ADC驱动实例
 * @param   p_cfg[in]           ADC硬件配置
 *
 * @return  0 success
 *         -1 adc_instance null
 *         -2 cfg null
 *         -3 adc base null
 *         -4 gpio 实例构造失败
 *****************************************************************************/
int8_t adc_driver_inst(adc_driver_t *p_adc_instance,
					   adc_cfg_t *p_cfg)
{
	if (NULL == p_adc_instance)
	{
		return -1;
	}

	if (NULL == p_cfg)
	{
		return -2;
	}

	if (NULL == p_cfg->p_adc_base)
	{
		return -3;
	}

	if (0 != gpio_driver_inst(&p_adc_instance->gpio, &p_cfg->gpio))
	{
		return -4;
	}

	/* 加载硬件配置 */
	p_adc_instance->cfg = *p_cfg;

	/* 实例状态: 尚未初始化, 无使用者 */
	p_adc_instance->init_state = 0;
	p_adc_instance->ref_count = 0;

	/* 构建内嵌ADC句柄 */
	p_adc_instance->hadc.Instance = p_cfg->p_adc_base;
	p_adc_instance->hadc.Init = p_cfg->init;

	/* 挂载函数指针 (形参类型与头文件字段一致: adc_driver_t *, 可直接赋值;
	 * 若写成 void *, AC6 会以 incompatible-function-pointer-types **报错**) */
	p_adc_instance->pf_inst = adc_driver_inst;
	p_adc_instance->pf_deinst = adc_deinst;
	p_adc_instance->pf_init = adc_init;
	p_adc_instance->pf_deinit = adc_deinit;
	p_adc_instance->pf_read = adc_read;

	return 0;
}
