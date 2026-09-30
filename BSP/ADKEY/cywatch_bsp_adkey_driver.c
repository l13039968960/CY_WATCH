/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_bsp_adkey_driver.c
 *
 * @par dependencies
 * - cywatch_bsp_adkey_driver.h
 *
 * @author zw1194
 *
 * @brief Implete the HAL operations of ADKEY and corresponding opetions.
 *
 * Processing flow:
 *
 * call directly.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
#include "cywatch_bsp_adkey_driver.h"

static int8_t adkey_deinst(struct bsp_adkey_driver *p_adkey_instance);
static int8_t adkey_init(struct bsp_adkey_driver *p_adkey_instance);
static int8_t adkey_deinit(struct bsp_adkey_driver *p_adkey_instance);
static int8_t adkey_read_key(struct bsp_adkey_driver *p_adkey_instance,
							 uint16_t *p_key);

/******************************************************************************
 * @name    adkey_match_key
 * @brief   ADC码值 → 键号
 * @param   value[in] ADC码值
 *
 * @return  0 无按键, 1..3 键号
 *
 * @note    落在任何一个区间外都判为无按键, 所以阈值区间的上下沿必须盖住实测值。
 *          阈值宏在头文件里, 目前是占位值。
 *****************************************************************************/
static uint16_t adkey_match_key(uint16_t value)
{
	if (value >= ADKEY_KEY1_MIN && value <= ADKEY_KEY1_MAX)
	{
		return 1U;
	}

	if (value >= ADKEY_KEY2_MIN && value <= ADKEY_KEY2_MAX)
	{
		return 2U;
	}

	if (value >= ADKEY_KEY3_MIN && value <= ADKEY_KEY3_MAX)
	{
		return 3U;
	}

	return 0U;
}

/******************************************************************************
 * @name    adkey_inst
 * @brief   ADKEY构造函数: 校验接口 → 存接口 → 挂函数指针 → 初始化 → 自检
 * @param   p_adkey_instance[out] ADKEY驱动实例
 * @param   p_adc_interface[in]   ADC接口
 * @param   p_delay_interface[in] 延时接口
 *
 * @return  0 success
 *         -1 adkey_instance null
 *         -2 adc接口为null或其pf_*不完整
 *         -3 delay接口为null或pf_delay为null
 *         -4 初始化失败
 *         -5 自检失败
 *
 * @note    自检只确认"采样通路读得回来"(pf_get_value 返回0), 不校验具体码值 ——
 *          此刻按键可能正按着, 也可能确实停在 0 或满量程, 都属正常。
 *****************************************************************************/
int8_t adkey_inst(bsp_adkey_driver_t *p_adkey_instance,
				  adkey_adc_interface_t *p_adc_interface,
				  adkey_delay_interface_t *p_delay_interface)
{
	uint16_t value = 0;

	if (NULL == p_adkey_instance)
	{
		return -1;
	}

	if (NULL == p_adc_interface)
	{
		return -2;
	}
	else
	{
		if (NULL == p_adc_interface->pf_init ||
			NULL == p_adc_interface->pf_deinit ||
			NULL == p_adc_interface->pf_get_value)
		{
			return -2;
		}
	}

	if (NULL == p_delay_interface)
	{
		return -3;
	}
	else
	{
		if (NULL == p_delay_interface->pf_delay)
		{
			return -3;
		}
	}

	p_adkey_instance->p_adc_interface = p_adc_interface;
	p_adkey_instance->p_delay_interface = p_delay_interface;
	p_adkey_instance->last_key = 0;

	p_adkey_instance->pf_inst = adkey_inst;
	p_adkey_instance->pf_deinst = adkey_deinst;
	p_adkey_instance->pf_init = adkey_init;
	p_adkey_instance->pf_deinit = adkey_deinit;
	p_adkey_instance->pf_read_key = adkey_read_key;

	/* 初始化 */
	if (0 != p_adkey_instance->pf_init(p_adkey_instance))
	{
		p_adkey_instance->pf_deinst(p_adkey_instance);
		return -4;
	}

	/* 自检: 采一次ADC, 确认采样通路可用 */
	if (0 != p_adkey_instance->p_adc_interface->pf_get_value(&value))
	{
		p_adkey_instance->pf_deinst(p_adkey_instance);
		return -5;
	}

	return 0;
}

/******************************************************************************
 * @name    adkey_deinst
 * @brief   析构ADKEY实例: 反初始化并清空接口指针与函数指针
 * @param   p_adkey_instance[in]
 *
 * @return  0 success
 *         -1 adkey_instance null
 *****************************************************************************/
static int8_t adkey_deinst(struct bsp_adkey_driver *p_adkey_instance)
{
	if (NULL == p_adkey_instance)
	{
		return -1;
	}

	p_adkey_instance->pf_deinit(p_adkey_instance);

	p_adkey_instance->p_adc_interface = NULL;
	p_adkey_instance->p_delay_interface = NULL;
	p_adkey_instance->last_key = 0;

	p_adkey_instance->pf_inst = NULL;
	p_adkey_instance->pf_deinst = NULL;
	p_adkey_instance->pf_init = NULL;
	p_adkey_instance->pf_deinit = NULL;
	p_adkey_instance->pf_read_key = NULL;

	return 0;
}

/******************************************************************************
 * @name    adkey_init
 * @brief   ADKEY初始化: 初始化ADC接口, 并清掉上次键号
 * @param   p_adkey_instance[in]
 *
 * @return  0 success
 *         -1 adkey_instance null
 *         -2 adc接口初始化失败
 *
 * @note    last_key 必须随初始化一起清零: 它代表"上一次判定结果", 重新初始化后
 *          没有历史状态, 留着旧值会让第一次 pf_read_key 结果失真。
 *****************************************************************************/
static int8_t adkey_init(struct bsp_adkey_driver *p_adkey_instance)
{
	if (NULL == p_adkey_instance)
	{
		return -1;
	}

	if (0 != p_adkey_instance->p_adc_interface->pf_init())
	{
		return -2;
	}

	p_adkey_instance->last_key = 0;

	return 0;
}

/******************************************************************************
 * @name    adkey_deinit
 * @brief   ADKEY反初始化: 反初始化ADC接口
 * @param   p_adkey_instance[in]
 *
 * @return  0 success
 *         -1 adkey_instance null
 *****************************************************************************/
static int8_t adkey_deinit(struct bsp_adkey_driver *p_adkey_instance)
{
	if (NULL == p_adkey_instance)
	{
		return -1;
	}

	p_adkey_instance->p_adc_interface->pf_deinit();

	p_adkey_instance->last_key = 0;

	return 0;
}

/******************************************************************************
 * @name    adkey_read_key
 * @brief   读一次键: 采ADC → 判定键号; 结果与上次不同时延时复采, 一致才认
 * @param   p_adkey_instance[in]
 * @param   p_key[out] 0=无按键, 1..3=键号
 *
 * @return  0 success
 *         -1 adkey_instance null
 *         -2 p_key null
 *         -3 首次采样失败
 *         -4 复采失败
 *
 * @note    去抖只在"判定结果和上次不一样"时才多花 ADKEY_DEBOUNCE_MS: 复采结果与
 *          首次判定一致才认新键并更新 last_key, 否则维持 last_key(当作抖动丢弃)。
 *          因此按下/松开瞬间最坏阻塞 ADKEY_DEBOUNCE_MS。
 *****************************************************************************/
static int8_t adkey_read_key(struct bsp_adkey_driver *p_adkey_instance,
							 uint16_t *p_key)
{
	uint16_t value = 0;
	uint16_t key = 0;

	if (NULL == p_adkey_instance)
	{
		return -1;
	}

	if (NULL == p_key)
	{
		return -2;
	}

	if (0 != p_adkey_instance->p_adc_interface->pf_get_value(&value))
	{
		return -3;
	}

	key = adkey_match_key(value);

	/* 与上次不同: 可能是真按键, 也可能是抖动 */
	if (key != p_adkey_instance->last_key)
	{
		p_adkey_instance->p_delay_interface->pf_delay(ADKEY_DEBOUNCE_MS);

		if (0 != p_adkey_instance->p_adc_interface->pf_get_value(&value))
		{
			return -4;
		}

		/* 复采一致才认新键, 否则维持上次结果 */
		if (adkey_match_key(value) == key)
		{
			p_adkey_instance->last_key = key;
		}
		else
		{
			key = p_adkey_instance->last_key;
		}
	}

	*p_key = key;

	return 0;
}
