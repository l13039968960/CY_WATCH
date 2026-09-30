/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_bsp_adkey_driver.h
 *
 * @par dependencies
 * - stdint.h
 * - stddef.h
 *
 * @author	zw1194
 *
 * @brief Provide the HAL APIs of ADKEY and corresponding opetions.
 *
 * Processing flow:
 *
 * call adkey_inst() to construct, then use function pointers.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 ******************************************************************************/
#ifndef __CYWATCH_BSP_ADKEY_DRIVER_H__
#define __CYWATCH_BSP_ADKEY_DRIVER_H__

/***********************************Includes***********************************/
#include <stdint.h>
#include <stddef.h>

/***********************************Includes***********************************/

/***********************************Defines************************************/
#define OS_SUPPORTING

/* 按键阈值: 每键一组 ADC 码值区间, 落在任何区间外即无按键
 * @note 占位值! 上电后逐个键按下、用 pf_get_value 读实测码值, 再按实测值改这里,
 *       每个区间两端各留约 100 的余量. */
#define ADKEY_KEY1_MIN 2500U
#define ADKEY_KEY1_MAX 2960U
#define ADKEY_KEY2_MIN 1150U
#define ADKEY_KEY2_MAX 1580U
#define ADKEY_KEY3_MIN 0U
#define ADKEY_KEY3_MAX 200U


/* 去抖延时(ms): pf_read_key 判定结果与上次不同时, 延时这么久再采一次, 一致才认 */
#define ADKEY_DEBOUNCE_MS 20U

/***********************************Defines************************************/

/**********************************Declaring***********************************/
/*adc接口*/
typedef struct
{
	/* adc初始化，不关心其他驱动 */
	int8_t (*pf_init)(void);
	/* adc反初始化 */
	int8_t (*pf_deinit)(void);
	/* 获取一次转换值 */
	int8_t (*pf_get_value)(uint16_t *value);
} adkey_adc_interface_t;

/*延时函数接口 (去抖复采用)*/
typedef struct
{
	void (*pf_delay)(uint32_t ms);
} adkey_delay_interface_t;

/*adkey定义*/
typedef struct bsp_adkey_driver
{
	adkey_adc_interface_t *p_adc_interface;
	adkey_delay_interface_t *p_delay_interface;

	uint16_t last_key; /*上次判定出的键号, 去抖比对用*/

	int8_t (*pf_inst)(
		struct bsp_adkey_driver *p_adkey_instance,

		adkey_adc_interface_t *p_adc_interface,
		adkey_delay_interface_t *p_delay_interface);

	int8_t (*pf_deinst)(struct bsp_adkey_driver *p_adkey_instance);

	int8_t (*pf_init)(struct bsp_adkey_driver *p_adkey_instance);

	int8_t (*pf_deinit)(struct bsp_adkey_driver *p_adkey_instance);

	/* 读键: *p_key 输出 0=无按键、1..3=键号. 判定结果与上次不同时延
	 * ADKEY_DEBOUNCE_MS 再采一次, 一致才认, 因此按下瞬间最坏阻塞这么久. */
	int8_t (*pf_read_key)(struct bsp_adkey_driver *p_adkey_instance,
						  uint16_t *p_key);

} bsp_adkey_driver_t;

/*adkey构造函数*/
int8_t adkey_inst(bsp_adkey_driver_t *p_adkey_instance,
				  adkey_adc_interface_t *p_adc_interface,
				  adkey_delay_interface_t *p_delay_interface);

/**********************************Declaring***********************************/

#endif // __CYWATCH_BSP_ADKEY_DRIVER_H__
