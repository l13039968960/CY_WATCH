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
 * - gpio_hal.h
 * - cmsis_os2.h
 *
 * @author zw1194
 *
 * @brief Implete the AD adapter: 装配 BSP 驱动的两个接口, 并转发按键读取.
 *
 * Processing flow:
 *
 * key_bsp_inst() 挂接口 → adkey_inst (内含占住 ADC1 + 配 PA2 + 采样自检)
 *   → key_bsp_read_key() 转发到驱动的 pf_read_key
 *   → key_bsp_deinst() 转发到驱动的 pf_deinst.
 *
 * @version V2.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note V2(2026-10-03): ADC1 改成与电源电压检测(IN8/PB0)共用, 实例移到 main.c
 *       (extern adc_instance), 本层只给"用哪个通道 + 采样时间".
 *       ★引脚本层自己配★: adc_hal 不再管引脚 —— 引脚跟着通道走, 归各自设备,
 *       所以 PA2 的模拟模式在 adkey_adc_init 里配、deinit 里收尾。
 *
 * @note 本层只做"转发 + 接口装配", 不含按键阈值判定与去抖 —— 那两样在 BSP 驱动里。
 *****************************************************************************/
#include "cywatch_adapter_adkey.h"

#include "cywatch_bsp_adkey_driver.h"
#include "adc_hal.h"   /* adc_driver_t: 与电源电压检测共用的 ADC1 */
#include "gpio_hal.h"  /* 本设备那一路 AD 引脚(PA2) */
#include "cmsis_os2.h" /* osDelay */

/***********************************Defines************************************/
/* ADKEY 接在 ADC1_IN2 (PA2). 采样时间 84 周期: AD 按键是电阻分压, 给短了读数会偏低.
   @note 外设基地址/分辨率/触发方式那些**外设级**参数不在这里, 在 main.c 的
         adc_shared_cfg 里 —— 两个设备共用同一份 */
#define ADKEY_ADC_CHANNEL  ADC_CHANNEL_2
#define ADKEY_ADC_SAMPLING ADC_SAMPLETIME_84CYCLES
#define ADKEY_ADC_PORT     GPIOA
#define ADKEY_ADC_PIN      GPIO_PIN_2

/***********************************Defines************************************/

/**********************************Declaring***********************************/
static bsp_adkey_driver_t adkey_instance;

/* 共用的 ADC1 实例(定义在 main.c)。谁用谁 claim/release */
extern adc_driver_t adc_instance;

/* 本设备那一路 AD 引脚(PA2, 模拟输入). ADC 驱动不碰 GPIO, 由本层自己管 */
static gpio_driver_t adkey_adc_pin;
static gpio_cfg_t adkey_adc_pin_cfg =
{
    .p_port = ADKEY_ADC_PORT,
    .pins   = ADKEY_ADC_PIN,
    .mode   = GPIO_MODE_ANALOG,
    .pull   = GPIO_NOPULL,
};

static adkey_adc_interface_t   adkey_adc_interface_instance;
static adkey_delay_interface_t adkey_delay_interface_instance;

/******************************************************************************
 * @name    adkey_adc_init
 * @brief   adkey_adc_interface_t.pf_init 的实现: 占住共享 ADC1 + 配本路引脚
 * @param   无
 *
 * @return  见 adc_hal 的 pf_init
 *
 * @note    共享外设按 ref_count 门控: 本设备这一份占用与电源电压检测那一份
 *          各记一次, 都放掉才真的关 ADC 与时钟
 *****************************************************************************/
static int8_t adkey_adc_init(void)
{
    int8_t ret = adc_instance.pf_init(&adc_instance);

    /* 引脚归本层: HAL_ADC_MspInit 是空弱函数, ADC 驱动不会碰 GPIO */
    (void)adkey_adc_pin.pf_init(&adkey_adc_pin);

    return ret;
}

/******************************************************************************
 * @name    adkey_adc_deinit
 * @brief   adkey_adc_interface_t.pf_deinit 的实现: 收引脚 + 放掉共享 ADC1 的占用
 * @param   无
 *
 * @return  见 adc_hal 的 pf_deinit
 *
 * @note    顺序: 先收引脚再放外设 —— 反过来的话外设已经关了, 引脚还挂在模拟
 *          输入上, 低功耗语义就落空了
 *****************************************************************************/
static int8_t adkey_adc_deinit(void)
{
    int8_t ret;

    (void)adkey_adc_pin.pf_deinit(&adkey_adc_pin);
    ret = adc_instance.pf_deinit(&adc_instance);

    return ret;
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
    return adc_instance.pf_read(&adc_instance,
                                ADKEY_ADC_CHANNEL, ADKEY_ADC_SAMPLING,
                                value);
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
 * @brief   挂两个接口 → 构造 ADKEY 驱动(内含占 ADC + 配引脚 + 采样自检)
 * @param   无
 *
 * @return  0 success
 *         -1 本层的引脚实例构造失败(配置非法)
 *         其余为 adkey_inst 的错误码(见驱动 @return)
 *
 * @note    adkey_inst 内部回调 adc 的 pf_init(占住 ADC1 + 配 PA2 模拟模式)并采
 *          一次做自检。ADC 实例本身已在 main.c 里建好, 本层不构造实例
 *****************************************************************************/
int8_t key_bsp_inst(void)
{
    /* 0. 构造本路引脚实例(只装配结构体, 不碰硬件) */
    if (0 != gpio_driver_inst(&adkey_adc_pin, &adkey_adc_pin_cfg))
    {
        return -1;
    }

    /* 挂两个接口(每个都对应驱动 adkey_inst 的一处非空校验) */
    adkey_adc_interface_instance.pf_init      = adkey_adc_init;
    adkey_adc_interface_instance.pf_deinit    = adkey_adc_deinit;
    adkey_adc_interface_instance.pf_get_value = adkey_adc_get_value;

    adkey_delay_interface_instance.pf_delay   = adkey_delay_cb;

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
