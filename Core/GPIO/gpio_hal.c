/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file gpio_hal.c
 *
 * @par dependencies
 * - gpio_hal.h
 *
 * @author zw1194
 *
 * @brief Implete the OO-style HAL operations of GPIO.
 *
 * Processing flow:
 *
 * call gpio_driver_inst() to construct, then use function pointers.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
#include "gpio_hal.h"

/********************************* 前向声明 *********************************/
static int8_t gpio_deinst(gpio_driver_t *p_gpio_instance);

/* 底层GPIO操作 (对外接口, 使用void *匹配上层接口) */
static int8_t gpio_write(void *p_ctx, uint8_t level);
static uint8_t gpio_read(void *p_ctx);
static int8_t gpio_toggle(void *p_ctx);

/********************************* GPIO操作 *********************************/

/******************************************************************************
 * @name    gpio_write
 * @brief   输出引脚电平
 * @param   p_ctx[in] void * → gpio_driver_t *
 * @param   level[in] 0=低电平, 非0=高电平
 *
 * @return  0 success
 *         -1 gpio_instance null
 *****************************************************************************/
static int8_t gpio_write(void *p_ctx, uint8_t level)
{
	gpio_driver_t *p_gpio_instance = (gpio_driver_t *)p_ctx;

	if (NULL == p_gpio_instance)
	{
		return -1;
	}

	HAL_GPIO_WritePin(p_gpio_instance->cfg.p_port, p_gpio_instance->cfg.pin,
					  level ? GPIO_PIN_SET : GPIO_PIN_RESET);

	return 0;
}

/******************************************************************************
 * @name    gpio_read
 * @brief   读取引脚电平
 * @param   p_ctx[in] void * → gpio_driver_t *
 *
 * @return  1=高电平, 0=低电平(实例为空时返回0)
 *****************************************************************************/
static uint8_t gpio_read(void *p_ctx)
{
	gpio_driver_t *p_gpio_instance = (gpio_driver_t *)p_ctx;

	if (NULL == p_gpio_instance)
	{
		return 0;
	}

	if (HAL_GPIO_ReadPin(p_gpio_instance->cfg.p_port,
						 p_gpio_instance->cfg.pin) == GPIO_PIN_SET)
	{
		return 1;
	}

	return 0;
}

/******************************************************************************
 * @name    gpio_toggle
 * @brief   翻转引脚电平
 * @param   p_ctx[in] void * → gpio_driver_t *
 *
 * @return  0 success
 *         -1 gpio_instance null
 *****************************************************************************/
static int8_t gpio_toggle(void *p_ctx)
{
	gpio_driver_t *p_gpio_instance = (gpio_driver_t *)p_ctx;

	if (NULL == p_gpio_instance)
	{
		return -1;
	}

	HAL_GPIO_TogglePin(p_gpio_instance->cfg.p_port, p_gpio_instance->cfg.pin);

	return 0;
}

/********************************* 构造与析构 *********************************/

/******************************************************************************
 * @name    gpio_deinst
 * @brief   GPIO驱动析构: 复位引脚配置并清除函数指针
 * @param   p_gpio_instance[in]
 *
 * @return  0 success
 *         -1 gpio_instance null
 *****************************************************************************/
static int8_t gpio_deinst(gpio_driver_t *p_gpio_instance)
{
	if (NULL == p_gpio_instance)
	{
		return -1;
	}

	/* 复位引脚配置 */
	HAL_GPIO_DeInit(p_gpio_instance->cfg.p_port, p_gpio_instance->cfg.pin);

	/* 清除接口与函数指针 */
	p_gpio_instance->p_delay_interface = NULL;

	p_gpio_instance->pf_inst = NULL;
	p_gpio_instance->pf_inst = NULL;
	p_gpio_instance->pf_deinst = NULL;
	p_gpio_instance->pf_write = NULL;
	p_gpio_instance->pf_read = NULL;
	p_gpio_instance->pf_toggle = NULL;

	return 0;
}

/******************************************************************************
 * @name    gpio_driver_inst
 * @brief   GPIO驱动构造函数: 配置引脚、注入延时接口、挂载函数指针
 * @param   p_gpio_instance[out]   GPIO驱动实例
 * @param   p_cfg[in]              GPIO引脚配置
 * @param   p_delay_interface[in]  延时接口(由调用方注入)
 *
 * @return  0 success
 *         -1 gpio_instance null
 *         -2 cfg null
 *         -3 port null
 *         -4 delay interface null
 *****************************************************************************/
int8_t gpio_driver_inst(gpio_driver_t *p_gpio_instance,
						gpio_cfg_t *p_cfg,
						gpio_delay_interface_t *p_delay_interface)
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};

	if (NULL == p_gpio_instance)
	{
		return -1;
	}

	if (NULL == p_cfg)
	{
		return -2;
	}

	if (NULL == p_cfg->p_port)
	{
		return -3;
	}

	if (NULL == p_delay_interface || NULL == p_delay_interface->pf_delay_us)
	{
		return -4;
	}

	/* 注入延时接口 */
	p_gpio_instance->p_delay_interface = p_delay_interface;

	/* 加载硬件配置 */
	p_gpio_instance->cfg = *p_cfg;

	/* 初始化GPIO */
	GPIO_InitStructure.Pin = p_gpio_instance->cfg.pin;
	GPIO_InitStructure.Mode = p_gpio_instance->cfg.mode;
	GPIO_InitStructure.Pull = p_gpio_instance->cfg.pull;
	GPIO_InitStructure.Speed = p_gpio_instance->cfg.speed;
	HAL_GPIO_Init(p_gpio_instance->cfg.p_port, &GPIO_InitStructure);

	/* 挂载函数指针 */
	p_gpio_instance->pf_inst = gpio_driver_inst;
	p_gpio_instance->pf_deinst = gpio_deinst;
	p_gpio_instance->pf_write = gpio_write;
	p_gpio_instance->pf_read = gpio_read;
	p_gpio_instance->pf_toggle = gpio_toggle;

	return 0;
}
