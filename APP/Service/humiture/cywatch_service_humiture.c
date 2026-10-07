/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_service_humiture.c
 *
 * @author zw1194
 *
 * @brief 温湿度服务(生产者): AHT21 任务 + 状态机.
 *
 * Processing flow:
 *
 * EVT_HUMITURE_INIT:    实例化 AHT21(最多重试5次) → EVT_HUMITURE_MEASURE
 * EVT_HUMITURE_MEASURE: 触发一次测量 → 发布 EVT_SERVICE_HUMITURE_DATA
 *                       → osDelay(10s) 原地等下一拍
 * EVT_HUMITURE_ERROR:   发布 BUSY+ERROR, 1s 后自动回 INIT 重试(自愈)
 * EVT_HUMITURE_SLEEP:   休眠请求 → AHT21 软复位 + 放共享总线 → 停在 NODONE,
 *                       等 service_humiture_wakeup() 把状态改回 INIT
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
#include "cywatch_service_humiture.h"
#include "cywatch_adapter_aht21.h"
#include "cmsis_os2.h"
#include "easyapp_port.h"
#include "system/log/cywatch_log.h"

/* ===== 闭环测试打印开关 =====
   1 = 每个测量周期(10s)打一行温湿度, 并在构造/读失败时打一行原因; ★测完改回 0★
   (main.c 里的宏跨不了翻译单元, 本文件的开关只对本文件生效) */
#define HUMITURE_SVC_LOOPBACK_TEST 0

/* 测量周期: AHT21 一次触发测量约 80ms, 器件自发热会让温度读数随采样率升高,
   手册建议间隔不低于 1s; 温湿度本身是分钟级缓变量, 且总线与姿态/心率共用 */
#define SERVICE_HUMITURE_PERIOD_MS   (5000u)

/* 实例化重试次数上限 */
#define SERVICE_HUMITURE_INST_RETRY  (5)
/* 连续多少次读失败判为错误态(单次失败多为总线毛刺, 累积到 3 次才重初始化) */
#define SERVICE_HUMITURE_ERROR_LIMIT (3)

/* 服务内部状态机取值, 不是页面收的事件 */
typedef enum{
    EVT_HUMITURE_INIT = 0,
    EVT_HUMITURE_MEASURE,
    EVT_HUMITURE_SLEEP,
    EVT_HUMITURE_NODONE,
    EVT_HUMITURE_ERROR,
}State_Humiture_Service_t;

/* 必须 volatile: 由 service_humiture_sleep/wakeup 从别的任务改, 而任务循环里所有调用
   都是外部函数(改不了本 TU 的 static), -O2 下编译器会把非 volatile 的值缓存进寄存器,
   导致状态切换永远读不到 */
static volatile State_Humiture_Service_t ServiceState;

/* 服务独占写的"最新帧": 发布前先填这里, 再把地址随事件发出去(0 拷贝, 生产者持有).
   注意每个测量周期被本任务覆盖, 消费侧应尽快读走 */
static Humiture_Data_t s_latest_humiture;

static const osThreadAttr_t g_service_humiture_attr =
{
    .name       = "humiture",
    .attr_bits  = 0,
    .cb_mem     = NULL,
    .cb_size    = 0,
    .stack_mem  = NULL,
    .stack_size = 1024,
    .priority   = osPriorityNormal,
};

static void service_humiture_run(void *pvParameters)
{
    int8_t  ret = 0;
    uint8_t inst_retry = 0;
    uint8_t error_cnt = 0;

    (void)pvParameters;

    while(1)
    {
        switch (ServiceState)
        {
        case EVT_HUMITURE_INIT:
            x_port_easyapp_event_send(EVT_SERVICE_HUMITURE_BUSY, 0, NULL);
            inst_retry = 0;
            while (0 != humiture_bsp_inst())
            {
                osDelay(500);
                inst_retry++;
                if (inst_retry >= SERVICE_HUMITURE_INST_RETRY)
                {
                    ServiceState = EVT_HUMITURE_ERROR;
#if HUMITURE_SVC_LOOPBACK_TEST
                    log_printf("[HUMITURE] AHT21 连续 %d 次构造失败, 1s 后重试(查接线/供电/上拉)\r\n",
                               (int)SERVICE_HUMITURE_INST_RETRY);
#endif
                    break;
                }
            }
            /* 重试全失败已切到 EVT_HUMITURE_ERROR: 跳过下面的正常初始化,
               否则 ServiceState 会被覆盖回 EVT_HUMITURE_MEASURE, 错误分支形同虚设 */
            if (EVT_HUMITURE_ERROR == ServiceState)
            {
                break;
            }

            error_cnt = 0;
            ServiceState = EVT_HUMITURE_MEASURE;
            /* 空闲补丁: 只在状态跳变时发一次, 不逐帧刷(逐帧刷会淹没事件环) */
            x_port_easyapp_event_send(EVT_SERVICE_HUMITURE_NOBUSY, 0, NULL);
#if HUMITURE_SVC_LOOPBACK_TEST
            log_printf("[HUMITURE] AHT21 构造成功, 每 %u ms 测一次\r\n",
                       (unsigned)SERVICE_HUMITURE_PERIOD_MS);
#endif
            break;

        case EVT_HUMITURE_MEASURE:
            /* 驱动直接把结果写进最新帧(失败时一个字节都不写), 所以出错不会
               在已发布的帧里留下半截脏数据 */
            ret = humiture_bsp_read_temp_humi(&s_latest_humiture.temperature,
                                              &s_latest_humiture.humidity);
            if (0 != ret)
            {
                error_cnt++;
#if HUMITURE_SVC_LOOPBACK_TEST
                log_printf("[HUMITURE] 读温湿度失败 rc=%d (连续 %u/%u 次)\r\n",
                           (int)ret, (unsigned)error_cnt,
                           (unsigned)SERVICE_HUMITURE_ERROR_LIMIT);
#endif
                if (error_cnt >= SERVICE_HUMITURE_ERROR_LIMIT)
                {
                    ServiceState = EVT_HUMITURE_ERROR;
                }
            }
            else
            {
                error_cnt = 0;
                /* 发指针: 数据由本任务持续更新, 事件里不复制 */
                x_port_easyapp_event_send(EVT_SERVICE_HUMITURE_DATA, 0U, (void *)&s_latest_humiture);
#if HUMITURE_SVC_LOOPBACK_TEST
                /* MicroLIB 无 %f, 放大成整数打(与心率服务同一处理) */
                log_printf("[HUMITURE] temp=%d humi=%d (均放大100倍)\r\n",
                           (int)(s_latest_humiture.temperature * 100.0f),
                           (int)(s_latest_humiture.humidity * 100.0f));
#endif
            }

            osDelay(SERVICE_HUMITURE_PERIOD_MS);
            break;

        case EVT_HUMITURE_SLEEP:
            /* 睡: AHT21 软复位 + 释放共享 I2C 上本设备那一份占用 */
            x_port_easyapp_event_send(EVT_SERVICE_HUMITURE_BUSY, 0, NULL);
            (void)humiture_bsp_hibernating();
            ServiceState = EVT_HUMITURE_NODONE;
            break;

        case EVT_HUMITURE_NODONE:
            x_port_easyapp_event_send(EVT_SERVICE_HUMITURE_NOBUSY, 0, NULL);
            x_port_easyapp_event_send(EVT_SERVICE_HUMITURE_NODONE, 0, NULL);
            /* 停在这里等 service_humiture_wakeup(); 唤醒回 EVT_HUMITURE_INIT
               重走构造 —— hibernating 已把器件与总线的占用都放掉了 */
            osDelay(1000);
            break;

        case EVT_HUMITURE_ERROR:
            x_port_easyapp_event_send(EVT_SERVICE_HUMITURE_BUSY, 0, NULL);
            x_port_easyapp_event_send(EVT_SERVICE_HUMITURE_ERROR, 0, NULL);
            /* 自愈: 传感器/I2C 可能是临时故障(接线松动、上电时序), 等 1s 回 INIT 重试 */
            osDelay(1000);
            ServiceState = EVT_HUMITURE_INIT;
            break;

        default:
            break;
        }
    }
}

void service_humiture_init(void)
{
    ServiceState = EVT_HUMITURE_INIT;

    if (NULL == osThreadNew(service_humiture_run, NULL, &g_service_humiture_attr))
    {
        /* 任务创建失败(堆不足等), 无温湿度服务可跑 */
        for (;;)
        {
        }
    }
}

void service_humiture_sleep(void)
{
    /* @warning 只置状态位. 任务若正卡在 EVT_HUMITURE_INIT 的重试循环或
       EVT_HUMITURE_ERROR 的 1s 等待里, 那次请求会被后面的状态赋值覆盖掉 */
    ServiceState = EVT_HUMITURE_SLEEP;
}

void service_humiture_wakeup(void)
{
    /* 回 INIT 重走构造: hibernating 已把器件软复位并放掉了共享总线的占用 */
    ServiceState = EVT_HUMITURE_INIT;
}

void service_humiture_get_data(Humiture_Data_t *p_out)
{
    /* 锁调度器再整份拷贝: 温湿度任务每测量周期覆写 s_latest_humiture, 不锁会跨帧撕裂 */
    (void)osKernelLock();
    *p_out = s_latest_humiture;
    (void)osKernelUnlock();
}
