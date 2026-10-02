/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_service_HeartRate.c
 *
 * @par dependencies
 * - cywatch_service_HeartRate.h
 * - ServiceFunction/HeartRate.h
 * - ServiceFunction/SpO2.h
 * - ../../MAX30102/adapter/cywatch_adapter_max30102.h
 * - cmsis_os2.h
 * - ../../APP/EasyAPP/port/easyapp_port.h
 *
 * @author	zw1194
 *
 * @brief Implete the heart rate / SpO2 producer service (task + state machine).
 *
 * Processing flow:
 *
 * EVT_HEARTRATE_INIT: 实例化 MAX30102(最多重试5次) → 切 SpO2 模式 → 使能 FIFO 满中断
 *           → EVT_HEARTRATE_MEASURE
 * EVT_HEARTRATE_MEASURE: 阻塞等 FIFO 满中断 → 突发读走整个 FIFO → 逐样本喂两个算法
 *              → 按 SERVICE_HEARTRATE_PUBLISH_MS 节流发布 EVT_SERVICE_HEARTRATE_DATA
 * EVT_HEARTRATE_ERROR: 发布 BUSY+ERROR, 1s 后自动回 EVT_HEARTRATE_INIT 重试(自愈)
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
#include "cywatch_service_HeartRate.h"
#include "ServiceFunction/HeartRate.h"
#include "ServiceFunction/SpO2.h"
#include "cywatch_adapter_max30102.h"
#include "cmsis_os2.h"
#include "easyapp_port.h"
#include "system/log/cywatch_log.h"

/* ===== 闭环测试打印开关 =====
   1 = 每个发布周期(500ms)打一行心率/血氧; ★测完改回 0★
   (main.c 里的宏跨不了翻译单元, 本文件的开关只对本文件生效) */
#define HEARTRATE_SVC_LOOPBACK_TEST 0

static State_HeartRate_Service_t ServiceState;
osMessageQueueId_t HeartRateMsgQueue;

/* 服务独占写的"最新帧": 发布前先填这里, 再把地址随事件发出去(0 拷贝, 生产者持有).
   注意每个发布周期被本任务覆盖, 消费侧应尽快读走 */
static HeartRate_Data_t s_latest_heartrate;

/* FIFO 突发读缓冲: 驱动 pf_read_all_FIFO_samples 要求容量 >= FIFO 深度(32) */
#define SERVICE_HEARTRATE_FIFO_DEPTH        (32u)
static uint32_t s_red_buf[SERVICE_HEARTRATE_FIFO_DEPTH];
static uint32_t s_ir_buf[SERVICE_HEARTRATE_FIFO_DEPTH];

/* FIFO 有效样本率 = SR / SMP_AVE = 100Hz / 1 = 100Hz(见驱动 MAX30102_DEFAULT_SMP_AVE).
   必须与驱动实际配置一致, 否则心率按样本号换算的时间整体失真 */
#define SERVICE_HEARTRATE_SAMPLE_RATE_HZ    (100.0f)

/* 发布节流: FIFO 每 320ms 满一次(32样本/100Hz), 若每次都发 DATA 就是 ~3 帧/s,
   而心率/血氧本身是秒级缓变量. 这里降到 2 帧/s, 避免无谓占用事件环与分发带宽 */
#define SERVICE_HEARTRATE_PUBLISH_MS        (500u)

/* 实例化重试次数上限 */
#define SERVICE_HEARTRATE_INST_RETRY        (5)
/* 连续 I2C/中断失败多少拍判为错误态 */
#define SERVICE_HEARTRATE_ERROR_LIMIT       (5)

static const osThreadAttr_t g_service_heartrate_attr =
{
    .name       = "heartrate",
    .attr_bits  = 0,
    .cb_mem     = NULL,
    .cb_size    = 0,
    .stack_mem  = NULL,
    .stack_size = 1024,
    .priority   = osPriorityNormal,
};

static void service_heartrate_run(void *pvParameters)
{
    heartrate_result_t hr = {0};
    spo2_result_t      sp = {0};
    int8_t  ret = 0;
    int8_t  sample_num = 0;
    int8_t  i = 0;
    int8_t  error_cnt = 0;
    uint8_t inst_retry = 0;
    uint32_t tick_pub = 0;

    (void)pvParameters;

    while(1)
    {
        State_HeartRate_Service_t ReceiveState;
        ret = osMessageQueueGet(HeartRateMsgQueue, &ReceiveState, NULL, 0);
        if(ret == osOK)
            ServiceState = ReceiveState;
        switch (ServiceState)
        {
        case EVT_HEARTRATE_INIT:
            x_port_easyapp_event_send(EVT_SERVICE_HEARTRATE_BUSY, 0, NULL);
            inst_retry = 0;
            while (0 != heartrate_bsp_inst())
            {
                osDelay(500);
                inst_retry++;
                if (inst_retry >= SERVICE_HEARTRATE_INST_RETRY)
                {
                    ServiceState = EVT_HEARTRATE_ERROR;
                    break;
                }
            }
            /* 重试全失败已切到 EVT_HEARTRATE_ERROR: 跳过下面的正常初始化, 交给 EVT_HEARTRATE_ERROR 分支处理,
               否则 ServiceState 会被覆盖回 EVT_HEARTRATE_MEASURE, 错误分支形同虚设 */
            if (EVT_HEARTRATE_ERROR == ServiceState)
            {
#if HEARTRATE_SVC_LOOPBACK_TEST
                log_printf("[HEARTRATE] MAX30102 连续 5 次构造失败, 1s 后重试(查接线/供电/上拉)\r\n");
#endif
                break;
            }

            heartrate_algo_init(SERVICE_HEARTRATE_SAMPLE_RATE_HZ);
            spo2_algo_init(SERVICE_HEARTRATE_SAMPLE_RATE_HZ);

            /* SpO2 模式: RED+IR 双路一起采. 心率从 IR 路算, 血氧用两路比值,
               一个模式同时满足两个算法, 不用轮流切换(切换要清 FIFO, 算法得重新收敛) */
            ret = heartrate_bsp_change_to_spo2();
            if (0 == ret)
            {
                ret = heartrate_bsp_enable_FIFO_FULL_interrupt();
            }
            if (0 != ret)
            {
#if HEARTRATE_SVC_LOOPBACK_TEST
                log_printf("[HEARTRATE] 切 SpO2 模式或开 FIFO 满中断失败, rc=%d\r\n", (int)ret);
#endif
                ServiceState = EVT_HEARTRATE_ERROR;
                break;
            }

            tick_pub = osKernelGetTickCount();
            error_cnt = 0;
            ServiceState = EVT_HEARTRATE_MEASURE;
            /* 空闲补丁: 只在状态跳变时发一次, 不逐帧刷(逐帧刷会淹没事件环) */
            x_port_easyapp_event_send(EVT_SERVICE_HEARTRATE_NOBUSY, 0, NULL);
#if HEARTRATE_SVC_LOOPBACK_TEST
            log_printf("[HEARTRATE] MAX30102 构造成功 + SpO2 模式 + FIFO 满中断已开, 等中断\r\n");
#endif
            break;

        case EVT_HEARTRATE_MEASURE:
            /* 阻塞等 FIFO 满中断(信号量由 adapter 持有, ISR 里释放).
               正常情况下 100Hz / 32 样本 = 每 320ms 醒一次 */
            ret = heartrate_bsp_wait_interrupt();
            if (0 != ret)
            {
                error_cnt++;
                if (error_cnt >= SERVICE_HEARTRATE_ERROR_LIMIT)
                {
#if HEARTRATE_SVC_LOOPBACK_TEST
                    log_printf("[HEARTRATE] 连续 5 次等 FIFO 满中断超时(INT 线断开? IFG 没清?), 重试\r\n");
#endif
                    ServiceState = EVT_HEARTRATE_ERROR;
                }
                break;
            }

            /* 一次突发读走整个 FIFO: 逐样本 feed 两个算法(不能跳样本, 否则时间轴失真) */
            sample_num = heartrate_bsp_read_all_FIFO_samples(s_red_buf, s_ir_buf);
            if (sample_num < 0)
            {
                error_cnt++;
                if (error_cnt >= SERVICE_HEARTRATE_ERROR_LIMIT)
                {
#if HEARTRATE_SVC_LOOPBACK_TEST
                    log_printf("[HEARTRATE] 连续 5 次突发读 FIFO 失败, 重试\r\n");
#endif
                    ServiceState = EVT_HEARTRATE_ERROR;
                }
                break;
            }

            if (sample_num > 0)
            {
                error_cnt = 0;

                for (i = 0; i < sample_num; i++)
                {
                    (void)heartrate_algo_update(s_ir_buf[i], &hr);
                    (void)spo2_algo_update(s_red_buf[i], s_ir_buf[i], &sp);
                }

                if ((osKernelGetTickCount() - tick_pub) >= SERVICE_HEARTRATE_PUBLISH_MS)
                {
                    tick_pub = osKernelGetTickCount();

                    /* 先更新最新帧: 末样本原值 + 两个算法的当前结果 */
                    s_latest_heartrate.red = s_red_buf[sample_num - 1];
                    s_latest_heartrate.ir  = s_ir_buf[sample_num - 1];
                    s_latest_heartrate.finger_on =
                        (hr.dc >= HEARTRATE_DC_MIN) ? 1U : 0U;
                    s_latest_heartrate.heart_rate_bpm =
                        (0 != hr.valid) ? hr.bpm : 0.0f;
                    s_latest_heartrate.spo2_percent =
                        (0 != sp.valid) ? sp.spo2 : 0.0f;

                    /* 再发指针: 数据由本任务持续更新, 事件里不复制 */
                    x_port_easyapp_event_send(EVT_SERVICE_HEARTRATE_DATA,
                                              0U, (void *)&s_latest_heartrate);
#if HEARTRATE_SVC_LOOPBACK_TEST
                    /* 本分支本身已按 500ms 节流, 直接打; MicroLIB 无 %f, 放大成整数 */
                    log_printf("[HEARTRATE] red=%u ir=%u 手指=%u bpm=%d spo2=%d (均放大100倍) "
                               "HR_valid=%u SpO2_valid=%u\r\n",
                               (unsigned)s_latest_heartrate.red,
                               (unsigned)s_latest_heartrate.ir,
                               (unsigned)s_latest_heartrate.finger_on,
                               (int)(s_latest_heartrate.heart_rate_bpm * 100.0f),
                               (int)(s_latest_heartrate.spo2_percent * 100.0f),
                               (unsigned)(0 != hr.valid), (unsigned)(0 != sp.valid));
#endif
                }
            }
            break;

        case EVT_HEARTRATE_SLEEP:
            (void)heartrate_bsp_hibernating();
            ServiceState = EVT_HEARTRATE_NODONE;
            break;

        case EVT_HEARTRATE_WAKEUP:
            (void)heartrate_bsp_wakeup();
            /* 休眠期间 FIFO 已断档, 算法状态作废, 醒后重新收敛 */
            (void)heartrate_algo_reset();
            (void)spo2_algo_reset();
            tick_pub = osKernelGetTickCount();
            error_cnt = 0;
            ServiceState = EVT_HEARTRATE_MEASURE;
            break;

        case EVT_HEARTRATE_NODONE:
            x_port_easyapp_event_send(EVT_SERVICE_HEARTRATE_NODONE, 0, NULL);
            osDelay(100);
            break;

        case EVT_HEARTRATE_ERROR:
            x_port_easyapp_event_send(EVT_SERVICE_HEARTRATE_BUSY, 0, NULL);
            x_port_easyapp_event_send(EVT_SERVICE_HEARTRATE_ERROR, 0, NULL);
            /* 自愈: 传感器/I2C 可能是临时故障(接线松动、上电时序), 等 1s 回 INIT 重试.
               与姿态服务"死在 ERROR 不动"不同——心率是常驻测量, 必须能自己恢复 */
            osDelay(1000);
            ServiceState = EVT_HEARTRATE_INIT;
            break;

        default:
            break;
        }
    }
}

void service_heartrate_init(void)
{
    ServiceState = EVT_HEARTRATE_INIT;
    HeartRateMsgQueue = osMessageQueueNew(1, sizeof(State_HeartRate_Service_t), NULL);
    if (NULL == osThreadNew(service_heartrate_run, NULL, &g_service_heartrate_attr))
    {
        /* 任务创建失败(堆不足等), 无心率服务可跑 */
        for (;;)
        {
        }
    }
}
