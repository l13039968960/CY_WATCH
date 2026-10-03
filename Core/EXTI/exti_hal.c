/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file exti_hal.c
 *
 * @par dependencies
 * - exti_hal.h
 *
 * @author zw1194
 *
 * @brief Implete the OO-style HAL operations of EXTI external interrupt.
 *
 * Processing flow:
 *
 * exti_driver_inst() 构造(不碰硬件) -> pf_init() 配引脚并注册分发表
 * -> attach callback -> 用函数指针.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
#include "exti_hal.h"

/********************************* 前向声明 *********************************/
static int8_t exti_deinst(exti_driver_t *p_exti_instance);
static int8_t exti_init(exti_driver_t *p_exti_instance);
static int8_t exti_deinit(exti_driver_t *p_exti_instance);

/* 底层EXTI操作 (对外接口, 使用void *匹配上层接口) */
static int8_t exti_enable_interrupt(void *p_ctx);
static int8_t exti_disable_interrupt(void *p_ctx);
static int8_t exti_attach_callback(void *p_ctx,
								   void (*pf_interrupt_cb)(void *p_ctx));

/* EXTI实例注册表: 按EXTI线号(0-15)索引, 供中断分发反查实例(多实例支持) */
static exti_driver_t *g_exti_instances[16] = { NULL };

/********************************* 内部工具 *********************************/

/******************************************************************************
 * @name    exti_pin_to_line
 * @brief   将GPIO_PIN_x(单比特掩码)换算为EXTI线号(0-15)
 * @param   pin[in] GPIO_PIN_x
 *
 * @return  EXTI线号(0-15), 非法返回0xFF
 *****************************************************************************/
static uint8_t exti_pin_to_line(uint16_t pin)
{
	uint8_t line = 0;

	while (0U == (pin & 0x0001U))
	{
		pin >>= 1U;
		line++;

		if (line > 15U)
		{
			return 0xFF;
		}
	}

	return line;
}

/********************************* 中断分发 *********************************/

/******************************************************************************
 * @name    exti_irq_handler
 * @brief   EXTI中断分发: 清除挂起标志, 按线号反查实例并回调上层
 * @param   pin[in] GPIO_PIN_x(由应用层 EXTIx_IRQHandler 传入)
 *
 * @note    共享中断线(EXTI5-9 / EXTI10-15)需对每个候选引脚各调用一次;
 *          中断上下文内仅回调(上层只置标志), 不做 I2C/SPI/printf 等耗时操作
 *****************************************************************************/
void exti_irq_handler(uint16_t pin)
{
	uint8_t line;

	/* 仅处理确实挂起的中断 */
	if (RESET == __HAL_GPIO_EXTI_GET_IT(pin))
	{
		return;
	}

	/* 清除挂起标志 */
	__HAL_GPIO_EXTI_CLEAR_IT(pin);

	line = exti_pin_to_line(pin);
	if (line < 16U &&
		NULL != g_exti_instances[line] &&
		NULL != g_exti_instances[line]->pf_interrupt_cb)
	{
		g_exti_instances[line]->pf_interrupt_cb(
			(void *)g_exti_instances[line]);
	}
}

/********************************* EXTI操作 *********************************/

/******************************************************************************
 * @name    exti_enable_interrupt
 * @brief   使能EXTI中断(设置NVIC优先级并使能)
 * @param   p_ctx[in] void * → exti_driver_t *
 *
 * @return  0 success
 *         -1 exti_instance null
 *****************************************************************************/
static int8_t exti_enable_interrupt(void *p_ctx)
{
	exti_driver_t *p_exti_instance = (exti_driver_t *)p_ctx;

	if (NULL == p_exti_instance)
	{
		return -1;
	}

	HAL_NVIC_SetPriority(p_exti_instance->cfg.irqn,
						 p_exti_instance->cfg.preempt_priority,
						 p_exti_instance->cfg.sub_priority);
	HAL_NVIC_EnableIRQ(p_exti_instance->cfg.irqn);

	return 0;
}

/******************************************************************************
 * @name    exti_disable_interrupt
 * @brief   失能EXTI中断
 * @param   p_ctx[in] void * → exti_driver_t *
 *
 * @return  0 success
 *         -1 exti_instance null
 *****************************************************************************/
static int8_t exti_disable_interrupt(void *p_ctx)
{
	exti_driver_t *p_exti_instance = (exti_driver_t *)p_ctx;

	if (NULL == p_exti_instance)
	{
		return -1;
	}

	HAL_NVIC_DisableIRQ(p_exti_instance->cfg.irqn);

	return 0;
}

/******************************************************************************
 * @name    exti_attach_callback
 * @brief   挂载上层中断回调(由中断分发在ISR中调用)
 * @param   p_ctx[in]           void * → exti_driver_t *
 * @param   pf_interrupt_cb[in] 上层中断回调
 *
 * @return  0 success
 *         -1 exti_instance null
 *****************************************************************************/
static int8_t exti_attach_callback(void *p_ctx,
								   void (*pf_interrupt_cb)(void *p_ctx))
{
	exti_driver_t *p_exti_instance = (exti_driver_t *)p_ctx;

	if (NULL == p_exti_instance)
	{
		return -1;
	}

	p_exti_instance->pf_interrupt_cb = pf_interrupt_cb;

	return 0;
}

/********************************* 初始化与收尾 *********************************/

/******************************************************************************
 * @name    exti_init
 * @brief   占用: 配引脚为中断模式 + 注册到中断分发表
 * @param   p_exti_instance[in]
 *
 * @return  0 success
 *         -1 exti_instance null
 *****************************************************************************/
static int8_t exti_init(exti_driver_t *p_exti_instance)
{
	uint8_t line;

	if (NULL == p_exti_instance)
	{
		return -1;
	}

	/* 配置GPIO为外部中断模式. HAL_GPIO_Init 内部会配 SYSCFG_EXTICR 的线映射,
	   并按 mode 里的 IT 位写 EXTI 的 IMR/RTSR/FTSR —— 这些仍归它管 */
	(void)p_exti_instance->gpio.pf_init(&p_exti_instance->gpio);

	/* 注册到分发表(多实例按线号反查) */
	line = exti_pin_to_line(p_exti_instance->cfg.gpio.pins);
	g_exti_instances[line] = p_exti_instance;

	return 0;
}

/******************************************************************************
 * @name    exti_deinit
 * @brief   释放: 关NVIC + 从分发表摘除 + 反初始化引脚
 * @param   p_exti_instance[in]
 *
 * @return  0 success
 *         -1 exti_instance null
 *****************************************************************************/
static int8_t exti_deinit(exti_driver_t *p_exti_instance)
{
	uint8_t line;

	if (NULL == p_exti_instance)
	{
		return -1;
	}

	HAL_NVIC_DisableIRQ(p_exti_instance->cfg.irqn);

	/* 先摘表再放引脚: 引脚一悬空, 边沿可能立刻触发一次中断 */
	line = exti_pin_to_line(p_exti_instance->cfg.gpio.pins);
	if (line < 16U && g_exti_instances[line] == p_exti_instance)
	{
		g_exti_instances[line] = NULL;
	}

	/* 反初始化引脚(DeInit → 模拟输入, 顺序在 gpio_hal 里) */
	(void)p_exti_instance->gpio.pf_deinit(&p_exti_instance->gpio);

	return 0;
}

/********************************* 构造与析构 *********************************/

/******************************************************************************
 * @name    exti_deinst
 * @brief   EXTI驱动析构: 清除接口、回调与函数指针(硬件由 pf_deinit 释放)
 * @param   p_exti_instance[in]
 *
 * @return  0 success
 *         -1 exti_instance null
 *****************************************************************************/
static int8_t exti_deinst(exti_driver_t *p_exti_instance)
{
	if (NULL == p_exti_instance)
	{
		return -1;
	}

	/* 清除接口、回调与函数指针 */
	p_exti_instance->p_delay_interface = NULL;
	p_exti_instance->pf_interrupt_cb = NULL;
	p_exti_instance->pf_inst = NULL;
	p_exti_instance->pf_deinst = NULL;
	p_exti_instance->pf_init = NULL;
	p_exti_instance->pf_deinit = NULL;
	p_exti_instance->pf_enable_interrupt = NULL;
	p_exti_instance->pf_disable_interrupt = NULL;
	p_exti_instance->pf_attach_callback = NULL;

	return 0;
}

/******************************************************************************
 * @name    exti_driver_inst
 * @brief   EXTI驱动构造函数: 存配置、注入延时接口、挂载函数指针(不碰硬件)
 * @param   p_exti_instance[out]   EXTI驱动实例
 * @param   p_cfg[in]             EXTI硬件配置
 * @param   p_delay_interface[in]  延时接口(由调用方注入)
 *
 * @return  0 success
 *         -1 exti_instance null
 *         -2 cfg null
 *         -3 gpio 实例构造失败(端口空)
 *         -4 pin invalid (非单一有效引脚位)
 *         -5 delay interface null
 *****************************************************************************/
int8_t exti_driver_inst(exti_driver_t *p_exti_instance,
						exti_cfg_t *p_cfg,
						exti_delay_interface_t *p_delay_interface)
{
	if (NULL == p_exti_instance)
	{
		return -1;
	}

	if (NULL == p_cfg)
	{
		return -2;
	}

	if (0 != gpio_driver_inst(&p_exti_instance->gpio, &p_cfg->gpio))
	{
		return -3;
	}

	/* 校验引脚为单一有效位(EXTI线0-15) */
	if (0U == p_cfg->gpio.pins || 0U != (p_cfg->gpio.pins & (p_cfg->gpio.pins - 1U)))
	{
		return -4;
	}

	if (NULL == p_delay_interface || NULL == p_delay_interface->pf_delay_us)
	{
		return -5;
	}

	/* 注入延时接口并加载硬件配置 */
	p_exti_instance->p_delay_interface = p_delay_interface;
	p_exti_instance->cfg = *p_cfg;
	p_exti_instance->pf_interrupt_cb = NULL;

	/* 挂载函数指针 */
	p_exti_instance->pf_inst = exti_driver_inst;
	p_exti_instance->pf_deinst = exti_deinst;
	p_exti_instance->pf_init = exti_init;
	p_exti_instance->pf_deinit = exti_deinit;
	p_exti_instance->pf_enable_interrupt = exti_enable_interrupt;
	p_exti_instance->pf_disable_interrupt = exti_disable_interrupt;
	p_exti_instance->pf_attach_callback = exti_attach_callback;

	return 0;
}
