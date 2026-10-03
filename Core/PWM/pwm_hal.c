/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file pwm_hal.c
 *
 * @par dependencies
 * - pwm_hal.h
 *
 * @author zw1194
 *
 * @brief Implete the OO-style HAL operations of PWM.
 *
 * Processing flow:
 *
 * call pwm_driver_inst() to construct, then use function pointers.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
#include "pwm_hal.h"

/********************************* 私有常量 *********************************/
/* HAL 通道常量表: 下标 0..3 对应 PWM_CHANNEL_1..4 */
static const uint32_t pwm_hal_channel[PWM_CHANNEL_NUM] =
{
	TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
};

/********************************* 前向声明 *********************************/
static int8_t pwm_deinst(pwm_driver_t *p_pwm_instance);
static int8_t pwm_init(pwm_driver_t *p_pwm_instance);
static int8_t pwm_deinit(pwm_driver_t *p_pwm_instance);
static int8_t pwm_set_duty(pwm_driver_t *p_pwm_instance,
						   uint32_t channel, uint8_t duty_percent);

/********************************* 私有辅助 *********************************/

/******************************************************************************
 * @name    pwm_clk_enable
 * @brief   开本 TIM 外设的时钟(引脚时钟由 gpio_hal 负责)
 * @param   p_tim[in]   TIM外设基地址
 *
 * @note    只能按基地址分派: RCC 的时钟使能宏连寄存器位都是编译期固定的
 *          (__HAL_RCC_TIM4_CLK_ENABLE 直接写 APB1ENR 的 TIM4EN), HAL 没有
 *          "按基地址查位" 的通用 API.
 *****************************************************************************/
static void pwm_clk_enable(TIM_TypeDef *p_tim)
{
	if (TIM1 == p_tim)      { __HAL_RCC_TIM1_CLK_ENABLE(); }
	else if (TIM2 == p_tim) { __HAL_RCC_TIM2_CLK_ENABLE(); }
	else if (TIM3 == p_tim) { __HAL_RCC_TIM3_CLK_ENABLE(); }
	else if (TIM4 == p_tim) { __HAL_RCC_TIM4_CLK_ENABLE(); }
	else if (TIM5 == p_tim) { __HAL_RCC_TIM5_CLK_ENABLE(); }
}

/******************************************************************************
 * @name    pwm_clk_disable
 * @brief   关本 TIM 外设的时钟(pf_deinit 收尾用)
 * @param   p_tim[in]   TIM外设基地址
 *
 * @note    ★只关 TIM 外设时钟, 不关 GPIO 端口时钟★ —— 端口是多外设共用的,
 *          连它一起关会把同端口上的 ADC/UART/IIC 一起打死.
 *****************************************************************************/
static void pwm_clk_disable(TIM_TypeDef *p_tim)
{
	if (TIM1 == p_tim)      { __HAL_RCC_TIM1_CLK_DISABLE(); }
	else if (TIM2 == p_tim) { __HAL_RCC_TIM2_CLK_DISABLE(); }
	else if (TIM3 == p_tim) { __HAL_RCC_TIM3_CLK_DISABLE(); }
	else if (TIM4 == p_tim) { __HAL_RCC_TIM4_CLK_DISABLE(); }
	else if (TIM5 == p_tim) { __HAL_RCC_TIM5_CLK_DISABLE(); }
}

/******************************************************************************
 * @name    pwm_calc_psc_arr
 * @brief   按目标频率反算 Prescaler/Period(ARR 寄存器值)
 * @param   tim_clock_hz[in] 定时器输入时钟(Hz)
 * @param   freq_hz[in]      目标PWM频率(Hz), 必须 1 ~ tim_clock_hz
 * @param   p_psc[out]       PSC 寄存器值(= 预分频系数 - 1)
 * @param   p_arr[out]       ARR 寄存器值(= 每周期计数 - 1)
 *
 * @note    用"总步数"来拆, 不写 freq*65536 —— 那个中间量会撑爆 uint32:
 *             ticks = 计数时钟 / 频率     (一个周期总共要走的计数)
 *             pre   = ceil(ticks/65536)   (16位ARR装不下时需要的预分频系数)
 *             arr   = ticks / pre
 *          freq=1kHz, clk=100MHz 时得 pre=2/arr=50000, 实际 100e6/(2*50000)=1kHz.
 *****************************************************************************/
static void pwm_calc_psc_arr(uint32_t tim_clock_hz, uint32_t freq_hz,
							 uint32_t *p_psc, uint32_t *p_arr)
{
	uint32_t ticks = tim_clock_hz / freq_hz;
	uint32_t pre   = (ticks + 65535U) / 65536U;

	if (0U == pre)
	{
		pre = 1U;
	}

	*p_psc = pre - 1U;
	*p_arr = (ticks / pre) - 1U;
}

/******************************************************************************
 * @name    pwm_channel_index
 * @brief   把通道位掩码换成 0..3 的下标
 * @param   channel[in] PWM_CHANNEL_x(必须是单个位)
 * @param   p_index[out] 下标(0=CH1)
 * @return  0 成功 / -1 不是单个已知的通道位
 *****************************************************************************/
static int8_t pwm_channel_index(uint32_t channel, uint8_t *p_index)
{
	uint8_t i;

	for (i = 0U; i < PWM_CHANNEL_NUM; i++)
	{
		if (channel == (1U << i))
		{
			*p_index = i;
			return 0;
		}
	}

	return -1;
}

/********************************* 构造与析构 *********************************/

/******************************************************************************
 * @name    pwm_init
 * @brief   算好时基 -> 初始化TIM -> 逐通道配PWM并启动输出
 * @param   p_pwm_instance[in]
 *
 * @return  0 success
 *         -1 pwm_instance null
 *         -2 cfg.freq_hz 越界(0 或 > tim_clock_hz)
 *         -3 HAL_TIM_PWM_Init error
 *         -4 通道配置失败
 *         -5 通道启动失败
 *
 * @note    TIM/GPIO 的时钟与引脚都是本函数自己配的, 不走 ST 的 MSP 回调
 *          (HAL_TIM_PWM_MspInit 未实现, HAL 自带的那个空弱函数不会动 GPIO).
 *
 * @note    引用计数: ref_count 由 0→1 时才真正初始化外设, 已有使用者时只累加计数.
 *****************************************************************************/
static int8_t pwm_init(pwm_driver_t *p_pwm_instance)
{
	TIM_OC_InitTypeDef sConfig = {0};
	uint32_t psc = 0U;
	uint32_t arr = 0U;
	uint8_t i;

	if (NULL == p_pwm_instance)
	{
		return -1;
	}

	if ((0U == p_pwm_instance->cfg.freq_hz) ||
		(p_pwm_instance->cfg.freq_hz > p_pwm_instance->cfg.tim_clock_hz))
	{
		return -2;
	}

	/* 首个使用者才真正初始化TIM外设 */
	if (0 == p_pwm_instance->ref_count)
	{
		/* 时钟与引脚都在这里配: 后面 HAL_TIM_PWM_Init 内部那次 HAL_TIM_PWM_MspInit
		   是空弱函数, 不会再动 GPIO —— 所以这一步不能省, 也不能挪到后面 */
		pwm_clk_enable(p_pwm_instance->cfg.p_tim_base);

		(void)p_pwm_instance->gpio.pf_init(&p_pwm_instance->gpio);

		pwm_calc_psc_arr(p_pwm_instance->cfg.tim_clock_hz,
						 p_pwm_instance->cfg.freq_hz, &psc, &arr);
		p_pwm_instance->htim.Init.Prescaler = psc;
		p_pwm_instance->htim.Init.Period    = arr;

		if (HAL_OK != HAL_TIM_PWM_Init(&p_pwm_instance->htim))
		{
			return -3;
		}

		sConfig.OCMode       = TIM_OCMODE_PWM1;
		sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
		sConfig.OCFastMode   = TIM_OCFAST_DISABLE;

		for (i = 0U; i < PWM_CHANNEL_NUM; i++)
		{
			if (0U == (p_pwm_instance->cfg.channels & (1U << i)))
			{
				continue;
			}

			/* duty=100 时 CCR=ARR, 实际占空比 ARR/(ARR+1), 差一个计数可忽略 */
			sConfig.Pulse = ((arr * p_pwm_instance->cfg.duty_percent[i])
							 / PWM_DUTY_FULL);

			if (HAL_OK != HAL_TIM_PWM_ConfigChannel(&p_pwm_instance->htim,
													&sConfig,
													pwm_hal_channel[i]))
			{
				return -4;
			}

			if (HAL_OK != HAL_TIM_PWM_Start(&p_pwm_instance->htim,
											pwm_hal_channel[i]))
			{
				return -5;
			}
		}

		p_pwm_instance->init_state = 1;
	}

	p_pwm_instance->ref_count++;

	return 0;
}

/******************************************************************************
 * @name    pwm_deinit
 * @brief   停掉各通道输出、关闭TIM外设, 并把引脚置低功耗态(模拟输入)
 * @param   p_pwm_instance[in]
 *
 * @return  0 success
 *         -1 pwm_instance null
 *
 * @note    HAL_TIM_PWM_DeInit 回调的 MspDeInit 是空弱函数, 引脚与时钟由本函数自己收尾.
 * @note    本函数有**低功耗语义**: 引脚不能只 DeInit 到复位态 ——
 *          F4 的 GPIO 复位值是**浮空输入**(不是模拟), 施密特触发器还开着, 悬空脚
 *          会随噪声来回翻转、白耗电。要再配成模拟输入才算真的关掉输入缓冲。
 * @note    引用计数: 每个使用者退出时减1, 减到0才真正关闭外设.
 *****************************************************************************/
static int8_t pwm_deinit(pwm_driver_t *p_pwm_instance)
{
	uint8_t i;

	if (NULL == p_pwm_instance)
	{
		return -1;
	}

	/* 退出一个使用者(已在0则不再减, 防uint8下溢后永远回不到0) */
	if (p_pwm_instance->ref_count > 0)
	{
		p_pwm_instance->ref_count--;
	}

	/* 最后一个使用者退出时才真正关闭TIM外设 */
	if (0 == p_pwm_instance->ref_count)
	{
		for (i = 0U; i < PWM_CHANNEL_NUM; i++)
		{
			if (0U != (p_pwm_instance->cfg.channels & (1U << i)))
			{
				(void)HAL_TIM_PWM_Stop(&p_pwm_instance->htim,
									   pwm_hal_channel[i]);
			}
		}

		HAL_TIM_PWM_DeInit(&p_pwm_instance->htim);

		/* 引脚收尾(DeInit → 模拟输入, 顺序在 gpio_hal 里) */
		(void)p_pwm_instance->gpio.pf_deinit(&p_pwm_instance->gpio);

		pwm_clk_disable(p_pwm_instance->cfg.p_tim_base);

		p_pwm_instance->init_state = 0;
	}

	return 0;
}

/******************************************************************************
 * @name    pwm_deinst
 * @brief   PWM驱动析构: 清除函数指针
 * @param   p_pwm_instance[in]
 *
 * @return  0 success
 *         -1 pwm_instance null
 *         -2 仍有使用者(ref_count != 0), 本次不析构
 *
 * @note    本函数不关外设 —— 关闭TIM外设走 pf_deinit.
 *****************************************************************************/
static int8_t pwm_deinst(pwm_driver_t *p_pwm_instance)
{
	if (NULL == p_pwm_instance)
	{
		return -1;
	}

	/* 仍有使用者: 清指针会把复用本实例的其他设备一起废掉 */
	if (0 != p_pwm_instance->ref_count)
	{
		return -2;
	}

	p_pwm_instance->init_state = 0;
	p_pwm_instance->ref_count = 0;

	p_pwm_instance->pf_inst = NULL;
	p_pwm_instance->pf_deinst = NULL;
	p_pwm_instance->pf_init = NULL;
	p_pwm_instance->pf_deinit = NULL;
	p_pwm_instance->pf_set_duty = NULL;

	return 0;
}

/*********************************** 占空比 ***********************************/

/******************************************************************************
 * @name    pwm_set_duty
 * @brief   改一个通道的占空比, 输出不停
 * @param   p_pwm_instance[in]
 * @param   channel[in]      PWM_CHANNEL_x(单个位)
 * @param   duty_percent[in] 0~100
 *
 * @return  0 success
 *         -1 pwm_instance null
 *         -2 未初始化(pf_init 没调过)
 *         -3 channel 不是单个已知通道位
 *         -4 duty_percent 超过 100
 *
 * @note    只是改 CCR, 不重配时基 —— 改完立即生效, 不打断正在出的波形.
 *****************************************************************************/
static int8_t pwm_set_duty(pwm_driver_t *p_pwm_instance,
						   uint32_t channel, uint8_t duty_percent)
{
	uint8_t index = 0U;

	if (NULL == p_pwm_instance)
	{
		return -1;
	}

	if (0 == p_pwm_instance->init_state)
	{
		return -2;
	}

	if (0 != pwm_channel_index(channel, &index))
	{
		return -3;
	}

	if (duty_percent > PWM_DUTY_FULL)
	{
		return -4;
	}

	__HAL_TIM_SET_COMPARE(&p_pwm_instance->htim, pwm_hal_channel[index],
						  (uint32_t)((__HAL_TIM_GET_AUTORELOAD(
										 &p_pwm_instance->htim)
									  * duty_percent) / PWM_DUTY_FULL));

	return 0;
}

/********************************* 构造与析构 *********************************/

/******************************************************************************
 * @name    pwm_driver_inst
 * @brief   PWM驱动构造函数: 加载配置、装配内嵌句柄、挂载函数指针
 *
 *          本函数不碰硬件, TIM外设的真正初始化在pf_init()里做。
 *
 * @param   p_pwm_instance[out] PWM驱动实例
 * @param   p_cfg[in]           PWM硬件配置
 *
 * @return  0 success
 *         -1 pwm_instance null
 *         -2 cfg null
 *         -3 tim base null
 *         -4 gpio 实例构造失败
 *****************************************************************************/
int8_t pwm_driver_inst(pwm_driver_t *p_pwm_instance,
					   pwm_cfg_t *p_cfg)
{
	if (NULL == p_pwm_instance)
	{
		return -1;
	}

	if (NULL == p_cfg)
	{
		return -2;
	}

	if (NULL == p_cfg->p_tim_base)
	{
		return -3;
	}

	if (0 != gpio_driver_inst(&p_pwm_instance->gpio, &p_cfg->gpio))
	{
		return -4;
	}

	/* 加载硬件配置 */
	p_pwm_instance->cfg = *p_cfg;

	/* 实例状态: 尚未初始化, 无使用者 */
	p_pwm_instance->init_state = 0;
	p_pwm_instance->ref_count = 0;

	/* 构建内嵌TIM句柄(Prescaler/Period 在 pf_init 里按 freq_hz 覆盖) */
	p_pwm_instance->htim.Instance = p_cfg->p_tim_base;
	p_pwm_instance->htim.Init = p_cfg->init;

	/* 挂载函数指针 (形参类型与头文件字段一致: pwm_driver_t *, 可直接赋值;
	 * 若写成 void *, AC6 会以 incompatible-function-pointer-types **报错**) */
	p_pwm_instance->pf_inst = pwm_driver_inst;
	p_pwm_instance->pf_deinst = pwm_deinst;
	p_pwm_instance->pf_init = pwm_init;
	p_pwm_instance->pf_deinit = pwm_deinit;
	p_pwm_instance->pf_set_duty = pwm_set_duty;

	return 0;
}
