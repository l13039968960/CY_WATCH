/******************************************************************************
 * Copyright (C) 2026 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_adapter_power.c
 *
 * @par dependencies
 * - cywatch_adapter_power.h
 * - adc_hal.h
 * - gpio_hal.h
 * - exti_hal.h
 * - delay.h
 * - cmsis_os2.h
 *
 * @author zw1194
 *
 * @brief 电源适配层的实现: 共享 ADC1 的第二个使用者, 读 PB0 上的电池分压;
 *        另管 PB1 的充电检测(EXTI1 双边沿 → 信号量).
 *
 * Processing flow:
 *
 * call directly.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 与 ADKEY 共用 ADC1(实例在 main.c), 但走**另一个通道**(IN8/PB0) ——
 *       adc_hal 的 pf_read 每次读之前会按传入的通道重配规则组.
 *
 * @note 分压电阻是**常接**的(没有通断控制脚): 100k+100k 这一档会常流约 18µA,
 *       本层不省这个电. 要省就得加个 MOS 使能脚, 那是硬件改动.
 *
 * @note 充电检测(PB1)只做两件事: **把边沿变成一次信号量唤醒**(power_bsp_wait_charge),
 *       和**读当前电平**(power_bsp_is_charging). 状态判断与事件发布都在服务里 ——
 *       事件入队的 x_port_easyapp_event_send 用 osKernelLock 当临界区, ISR 里调不了,
 *       所以 ISR 只能放信号量, 不能直接发事件.
 *****************************************************************************/
#include "cywatch_adapter_power.h"

#include "adc_hal.h"   /* adc_driver_t: 与 ADKEY 共用的 ADC1 */
#include "gpio_hal.h"  /* 本设备那两路引脚(PB0 模拟 / PB1 中断) */
#include "exti_hal.h"  /* exti_driver_t: 充电检测 PB1 */
#include "delay.h"     /* delay_us: exti_driver_inst 会校验 pf_delay_us 非空 */
#include "cmsis_os2.h" /* osSemaphoreNew/Acquire/Release */

/***********************************Defines************************************/
/* 电池分压接在 ADC1_IN8 (PB0) */
#define POWER_ADC_CHANNEL   ADC_CHANNEL_8
#define POWER_ADC_PORT      GPIOB
#define POWER_ADC_PIN       GPIO_PIN_0

/* 采样时间取最大档(480 周期 @25MHz ≈ 19.2us): 分压电阻的具体值未知, 源阻抗高时
   采样窗口不够读数会偏低, 而本路 10s 才采一次, 多花这点时间无所谓 */
#define POWER_ADC_SAMPLING  ADC_SAMPLETIME_480CYCLES

/* ADC 参考电压. 电池经 LDO 稳压到 3.3V 给 MCU 供电, 所以 VREF 是定值.
   @note 若哪天改成电池直供 MCU, 这个数就不再是常数 —— 读数会随电量漂,
         那时得改读内部 VREFINT 通道(ADC1_IN17)反推真实 VDDA */
#define POWER_VREF_MV       (3300U)

/* 分压比 = 1/2: V_adc = V_bat / 2, 所以 V_bat = V_adc × 2.
   3.7V 锂电池满电 4.2V, 分压后最高 2.1V, 不超 VREF */
#define POWER_DIV_NUM       (2U)
#define POWER_DIV_DEN       (1U)

/* 充电检测脚 PB1, 充电时低电平. 开内部上拉是为了没插线(或充电 IC 是开漏输出)时
   引脚不会悬空乱触发边沿 */
#define POWER_CHG_PORT      GPIOB
#define POWER_CHG_PIN       GPIO_PIN_1

/* EXTI 抢占优先级: 必须 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(5),
   因为该 ISR 内会调 osSemaphoreRelease(中断安全 API) */
#define POWER_CHG_PREEMPT_PRIO  6

/***********************************Defines************************************/

/**********************************Declaring***********************************/
/* 共用的 ADC1 实例(定义在 main.c)。谁用谁 claim/release */
extern adc_driver_t adc_instance;

/* 本设备那一路 AD 引脚(PB0, 模拟输入). ADC 驱动不碰 GPIO, 由本层自己管 */
static gpio_driver_t power_adc_pin;
static gpio_cfg_t power_adc_pin_cfg =
{
    .p_port = POWER_ADC_PORT,
    .pins   = POWER_ADC_PIN,
    .mode   = GPIO_MODE_ANALOG,
    .pull   = GPIO_NOPULL,
};

/* 共享 ADC1 的占用标记. ★必需, 不是冗余★: 服务的 EVT_POWER_INIT 是重试循环
   (最多调 5 次 inst), 没有它每失败一次就多抬一分, 而释放只发生一次 ——
   计数只增不减, 外设永远回不到 0, 释放出口形同虚设.
   (与共享 IIC 上那三个 adapter 的 s_iic_claimed 是同一回事) */
static uint8_t s_adc_claimed = 0U;

static void adc_claim(void)
{
    if (0U == s_adc_claimed)
    {
        (void)adc_instance.pf_init(&adc_instance);
        (void)power_adc_pin.pf_init(&power_adc_pin);
        s_adc_claimed = 1U;
    }
}

static void adc_release(void)
{
    if (0U != s_adc_claimed)
    {
        (void)power_adc_pin.pf_deinit(&power_adc_pin);
        (void)adc_instance.pf_deinit(&adc_instance);
        s_adc_claimed = 0U;
    }
}

/* ---- 充电检测(PB1/EXTI1) ----
   信号量由本层持有、ISR 里释放, 服务阻塞在上面等 —— 与 MAX30102 那条链同一套路.
   引脚由 exti 驱动内嵌的 gpio 持有, 本层不再另建一份(同一根脚只配一次) */
static exti_cfg_t power_chg_exti_cfg =
{
    .gpio =
    {
        .p_port = POWER_CHG_PORT,
        .pins   = POWER_CHG_PIN,
        .mode   = GPIO_MODE_IT_RISING_FALLING, /* 插上是下降沿, 拔下是上升沿, 两个都要 */
        .pull   = GPIO_PULLUP,
    },
    .irqn             = EXTI1_IRQn,
    .preempt_priority = POWER_CHG_PREEMPT_PRIO,
    .sub_priority     = 0,
};
static exti_delay_interface_t power_chg_delay_instance =
{
    .pf_delay_us = delay_us,
};
static exti_driver_t   power_chg_exti;
static osSemaphoreId_t s_chg_sem     = NULL;
static uint8_t         s_chg_claimed = 0U;

/* ISR: 只放信号量. 不读电平、不发事件 —— 读电平留给服务(读到的才是真值),
   发事件的那个口在中断里用不了(见文件头注释) */
static void power_chg_exti_cb(void *p_ctx)
{
    (void)p_ctx;

    if (NULL != s_chg_sem)
    {
        (void)osSemaphoreRelease(s_chg_sem);
    }
}

/* 同一套路里的占用标记. ★必需★: 服务的 EVT_POWER_INIT 是重试循环, 没有它每失败
   一次就多建一遍实例与信号量 */
static int8_t charge_claim(void)
{
    if (0U != s_chg_claimed)
    {
        return 0;
    }

    if (NULL == s_chg_sem)
    {
        s_chg_sem = osSemaphoreNew(1U, 0U, NULL);
        if (NULL == s_chg_sem)
        {
            return -1;
        }
    }

    if (0 != exti_driver_inst(&power_chg_exti, &power_chg_exti_cfg,
                              &power_chg_delay_instance))
    {
        return -1;
    }
    if (0 != power_chg_exti.pf_init(&power_chg_exti))
    {
        return -1;
    }
    if (0 != power_chg_exti.pf_attach_callback(&power_chg_exti, power_chg_exti_cb))
    {
        return -1;
    }
    /* pf_init 只配了边沿与分发表, NVIC 要这里单独开 —— 不开就永远不进 ISR */
    if (0 != power_chg_exti.pf_enable_interrupt(&power_chg_exti))
    {
        return -1;
    }

    s_chg_claimed = 1U;

    return 0;
}

static void charge_release(void)
{
    if (0U != s_chg_claimed)
    {
        (void)power_chg_exti.pf_deinit(&power_chg_exti);
        s_chg_claimed = 0U;
    }
}
/**********************************Declaring***********************************/

/******************************************************************************
 * @name    power_bsp_inst
 * @brief   占住共享 ADC1 + 配 PB0 模拟输入 + 采一次自检
 * @param   无
 *
 * @return  0 success
 *         -1 引脚实例构造失败(配置非法)
 *         -2 充电检测脚(PB1/EXTI1)占用失败
 *         -3 自检采样失败
 *
 * @note    自检是必要的: PB0 的模拟模式配没配对、分压通不通, 采一次就知道;
 *          而 ADKEY 那一份占用还在, 所以本层失败**不会**把 ADC 时钟关掉
 *****************************************************************************/
int8_t power_bsp_inst(void)
{
    uint16_t dummy = 0U;

    /* 构造本路引脚实例(只装配结构体, 不碰硬件) */
    if (0 != gpio_driver_inst(&power_adc_pin, &power_adc_pin_cfg))
    {
        return -1;
    }

    if (0 != charge_claim())
    {
        return -2;
    }

    adc_claim();

    if (0 != adc_instance.pf_read(&adc_instance, POWER_ADC_CHANNEL,
                                  POWER_ADC_SAMPLING, &dummy))
    {
        return -3;
    }

    return 0;
}

/******************************************************************************
 * @name    power_bsp_deinst
 * @brief   收引脚(含充电检测)+ 放掉共享 ADC1 上本设备那一份占用
 * @param   无
 *
 * @return  恒 0
 *****************************************************************************/
int8_t power_bsp_deinst(void)
{
    charge_release();
    adc_release();

    return 0;
}

/******************************************************************************
 * @name    power_bsp_read_battery_mv
 * @brief   采一次电池分压并换算成毫伏
 * @param   p_mv[out] 电池电压(整数毫伏)
 *
 * @return  0 success / -2 采样失败
 *
 * @note    先按 VREF 换成电压再乘分压比: 中转要用 uint32 ——
 *          code × 3300 最大约 1.35e7, 放不进 16 位
 *****************************************************************************/
int8_t power_bsp_read_battery_mv(uint16_t *p_mv)
{
    uint16_t code = 0U;

    if (0 != adc_instance.pf_read(&adc_instance, POWER_ADC_CHANNEL,
                                  POWER_ADC_SAMPLING, &code))
    {
        return -2;
    }

    *p_mv = (uint16_t)(((uint32_t)code * POWER_VREF_MV * POWER_DIV_NUM)
                       / (ADC_FULL_SCALE * POWER_DIV_DEN));

    return 0;
}

/******************************************************************************
 * @name    power_bsp_is_charging
 * @brief   读 PB1 当前电平: 低 = 充电中
 * @param   无
 *
 * @return  1 充电中 / 0 未充电
 *
 * @note    引脚没占用(inst 失败过/已 deinst)时一律报 0 —— 那时读到的电平没有意义
 *****************************************************************************/
uint8_t power_bsp_is_charging(void)
{
    if (0U == s_chg_claimed)
    {
        return 0U;
    }

    /* 读 exti 实例内嵌的那份 gpio: 同一根引脚只配一次, 不再另建实例 */
    return (0U == power_chg_exti.gpio.pf_read(&power_chg_exti.gpio)) ? 1U : 0U;
}

/******************************************************************************
 * @name    power_bsp_wait_charge
 * @brief   等一次充电状态边沿(ISR 放信号量唤醒), 或等到超时
 * @param   timeout_ms[in] 最长等待毫秒数
 *
 * @return  0 被边沿叫醒 / -1 超时或信号量还没建
 *
 * @note    只能任务上下文调用. -1 只是"这段时间没有边沿", 不是故障
 * @note    ★当前无人调用★: 服务暂时改用 osDelay 轮询(见 cywatch_service_power.c),
 *          这个口连同 PB1 的 EXTI 一起留着, 切回边沿驱动时直接换掉那句 osDelay
 *****************************************************************************/
int8_t power_bsp_wait_charge(uint32_t timeout_ms)
{
    if (NULL == s_chg_sem)
    {
        return -1;
    }

    return (osOK == osSemaphoreAcquire(s_chg_sem, timeout_ms)) ? 0 : -1;
}
