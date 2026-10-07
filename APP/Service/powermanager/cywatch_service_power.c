/******************************************************************************
 * Copyright (C) 2026 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_service_power.c
 *
 * @author zw1194
 *
 * @brief 电源服务(生产者): 电池电压 + 充电状态, 任务 + 状态机.
 *
 * Processing flow:
 *
 * EVT_POWER_INIT:    占住共享 ADC1(PB0 分压)与充电检测脚 PB1(EXTI1), 最多重试5次
 *                    → 报一次充电状态(上电即插着充电器时没有边沿可等)
 *                    → EVT_POWER_MEASURE
 * EVT_POWER_MEASURE: 采一次 → 发布 EVT_SERVICE_POWER_DATA
 *                    → 等满 10s → 查一次充电脚, 翻转才发 CHARGING
 * EVT_POWER_ERROR:   发布 BUSY+ERROR, 1s 后自动回 INIT 重试(自愈)
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 电池电压是分钟级的缓变量, 10s 一拍与温湿度同节奏. 本服务与 ADKEY
 *       共用 ADC1, 但两者周期差得远(ADKEY 是 10ms 级轮询), 互斥量只在每次
 *       转换的那几十微秒内被持有, 不会互相拖慢.
 *
 * @note ★充电状态当前是轮询的★: 每 10s 那一拍末尾读一次 PB1, 翻转才发事件 ——
 *       插上充电器最多要等 10s 才反映到界面.
 *       EXTI 那一整套仍在工作(PB1 已配成双边沿中断, ISR 会放信号量), 只是本任务
 *       不拿它当唤醒源. 想切回边沿驱动(插拔立刻响应), 把 EVT_POWER_MEASURE 里那句
 *       osDelay 换成 power_bsp_wait_charge(SERVICE_POWER_PERIOD_MS) 即可.
 *****************************************************************************/
#include "cywatch_service_power.h"

#include "cywatch_adapter_power.h"
#include "cmsis_os2.h"
#include "easyapp_port.h"
#include "system/log/cywatch_log.h"

/***********************************Defines************************************/
/* 测量周期. 3.7V 锂电的电压随负载有波动, 采样密了只是在采噪声 */
#define SERVICE_POWER_PERIOD_MS      (10000u)

/* 实例化(占 ADC + 自检)重试次数上限 */
#define SERVICE_POWER_INST_RETRY     (5)
/* 连续多少次读失败判为错误态(单次失败多为总线/调度毛刺) */
#define SERVICE_POWER_ERROR_LIMIT    (3)

/***********************************Defines************************************/

/**********************************Declaring***********************************/
static State_Power_Service_t ServiceState;

/* 服务独占写的"最新帧": 发布前先填这里, 再把地址随事件发出去(0 拷贝, 生产者持有).
   注意每个测量周期被本任务覆盖, 消费侧应尽快读走 */
static Power_Data_t s_latest_power;

static const osThreadAttr_t g_service_power_attr =
{
    .name       = "power",
    .attr_bits  = 0,
    .cb_mem     = NULL,
    .cb_size    = 0,
    .stack_mem  = NULL,
    .stack_size = 1024,
    .priority   = osPriorityNormal,
};

/* 充电状态已上报值. 初值取 0xFF(未知)而不是 0(未充电): 上电时就插着充电器的话
   一个边沿都不会来, 只有靠这个初值逼出第一帧, 消费侧才知道当前状态 */
static uint8_t s_charge_reported = 0xFFU;

static void service_power_run(void *pvParameters);

/******************************************************************************
 * @name    power_charge_report
 * @brief   充电状态翻转时发一次 EVT_SERVICE_POWER_CHARGING(状态在 event_flags)
 * @param   无
 *
 * @return  无
 *
 * @note    每拍都判一次(当前即 10s 那一拍), 但**只在状态与上次上报的不同时才发**
 *          —— 状态没变不该刷事件
 *****************************************************************************/
static void power_charge_report(void)
{
    uint8_t charging = power_bsp_is_charging();

    if (charging != s_charge_reported)
    {
        s_charge_reported = charging;
        (void)x_port_easyapp_event_send(EVT_SERVICE_POWER_CHARGING,
                                        (uint32_t)charging, NULL);
    }
}
/**********************************Declaring***********************************/

/******************************************************************************
 * @name    service_power_run
 * @brief   电源服务任务: 占 ADC → 每 10s 采一次电池电压并发布
 * @param   pvParameters[in] 未使用
 *
 * @return  无(不返回)
 *****************************************************************************/
static void service_power_run(void *pvParameters)
{
    int8_t  ret = 0;
    uint8_t inst_retry = 0;
    uint8_t error_cnt = 0;

    (void)pvParameters;

    while(1)
    {
        switch (ServiceState)
        {
        case EVT_POWER_INIT:
            x_port_easyapp_event_send(EVT_SERVICE_POWER_BUSY, 0, NULL);
            inst_retry = 0;
            while (0 != power_bsp_inst())
            {
                osDelay(500);
                inst_retry++;
                if (inst_retry >= SERVICE_POWER_INST_RETRY)
                {
                    ServiceState = EVT_POWER_ERROR;
                    break;
                }
            }
            /* 重试全失败已切到 EVT_POWER_ERROR: 跳过下面的正常初始化,
               否则 ServiceState 会被覆盖回 EVT_POWER_MEASURE */
            if (EVT_POWER_ERROR == ServiceState)
            {
                break;
            }

            error_cnt = 0;
            ServiceState = EVT_POWER_MEASURE;
            /* 上电时可能已经插着充电器: 那种情况没有边沿可等, 这里主动报一次 */
            power_charge_report();
            /* 空闲补丁: 只在状态跳变时发一次, 不逐帧刷(逐帧刷会淹没事件环) */
            x_port_easyapp_event_send(EVT_SERVICE_POWER_NOBUSY, 0, NULL);
            break;

        case EVT_POWER_MEASURE:
            /* adapter 换算失败时一个字节都不写, 所以出错不会在已发布的帧里
               留下半截脏数据 */
            ret = power_bsp_read_battery_mv(&s_latest_power.voltage_mv);
            if (0 != ret)
            {
                error_cnt++;
                if (error_cnt >= SERVICE_POWER_ERROR_LIMIT)
                {
                    ServiceState = EVT_POWER_ERROR;
                }
            }
            else
            {
                error_cnt = 0;
                /* 发指针: 数据由本任务持续更新, 事件里不复制 */
                x_port_easyapp_event_send(EVT_SERVICE_POWER_DATA,
                                          0U, (void *)&s_latest_power);
            }

            /* ★这一行是轮询/边沿驱动的唯一开关★ 当前是轮询: 傻等一拍, 下一轮再读
               充电脚. PB1 的 EXTI 与 power_bsp_wait_charge 都还在也还在跑, 只是不拿它
               当唤醒源 —— 切回边沿驱动就把它换成
               power_bsp_wait_charge(SERVICE_POWER_PERIOD_MS) */
            osDelay(SERVICE_POWER_PERIOD_MS);

            power_charge_report();
            break;

        case EVT_POWER_ERROR:
            x_port_easyapp_event_send(EVT_SERVICE_POWER_BUSY, 0, NULL);
            x_port_easyapp_event_send(EVT_SERVICE_POWER_ERROR, 0, NULL);
            /* 自愈: 传感器/接线可能是临时故障, 等 1s 回 INIT 重试 */
            osDelay(1000);
            ServiceState = EVT_POWER_INIT;
            break;

        default:
            break;
        }
    }
}

/******************************************************************************
 * @name    service_power_init
 * @brief   启动电源服务(只建任务, 不碰设备)
 * @param   无
 *
 * @return  无
 *
 * @note    在 osKernelInitialize() 之后、osKernelStart() 之前调; 设备占用
 *          (adc_claim)在任务体内做
 *****************************************************************************/
void service_power_init(void)
{
    ServiceState = EVT_POWER_INIT;

    if (NULL == osThreadNew(service_power_run, NULL, &g_service_power_attr))
    {
        /* 任务创建失败(堆不足等), 无电源服务可跑 */
        for (;;)
        {
        }
    }
}

/******************************************************************************
 * @name    service_power_get_mv
 * @brief   取最近一次采到的电池电压
 * @param   p_mv[out] 电池电压(整数毫伏)
 *
 * @return  0 success
 *
 * @note    不发起新采样, 只是把最新帧拷出来. 服务还没跑完第一轮时读到的是 0
 *****************************************************************************/
int8_t service_power_get_mv(uint16_t *p_mv)
{
    *p_mv = s_latest_power.voltage_mv;

    return 0;
}
