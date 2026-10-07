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
 * EVT_HEARTRATE_INIT: 实例化 MAX30102(最多重试5次) → 切 HR 模式 → 使能 FIFO 满中断
 *           → EVT_HEARTRATE_MEASURE
 * EVT_HEARTRATE_MEASURE: 按血氧开关同步 MAX30102 工作模式(见 set_spo2_enable) →
 *              阻塞等 FIFO 满中断 → 突发读走整个 FIFO → 逐样本喂心率算法
 *              (血氧开着时同时喂血氧算法) → 按 SERVICE_HEARTRATE_PUBLISH_MS 节流
 *              发布 EVT_SERVICE_HEARTRATE_DATA
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

/**********************************Declaring***********************************/
/* 服务内部状态机取值, 不是页面收的事件 */
typedef enum{
    EVT_HEARTRATE_INIT = 0,
    EVT_HEARTRATE_MEASURE,
    EVT_HEARTRATE_SLEEP,
    EVT_HEARTRATE_WAKEUP,
    EVT_HEARTRATE_NODONE,
    EVT_HEARTRATE_ERROR,
}State_HeartRate_Service_t;

/* 必须 volatile: 由 service_heartrate_sleep/wakeup 从别的任务改, 而任务循环里所有调用
   都是外部函数(改不了本 TU 的 static), -O2 下编译器会把非 volatile 的值缓存进寄存器,
   导致状态切换永远读不到 */
static volatile State_HeartRate_Service_t ServiceState;

/* 服务独占写的"最新帧": 发布前先填这里, 再把地址随事件发出去(0 拷贝, 生产者持有).
   注意每个发布周期被本任务覆盖, 消费侧应尽快读走 */
static HeartRate_Data_t s_latest_heartrate;

/* 佩戴状态的边沿检测. 已上报值初值取 0xFF(未知)而不是 0(未佩戴): 这样上电后
   第一次判定无论如何都会发一帧, 消费者才知道当前状态 */
static uint8_t s_wear_reported = 0xFFU;
static uint8_t s_wear_cand     = 0xFFU;  /* 正在连续计数的候选状态 */
static uint8_t s_wear_cand_cnt = 0U;

/* 血氧检测开关. ★req 必须 volatile★: 它由血氧服务任务改, 而本任务循环里每一句
   都是外部函数调用(改不了本 TU 的 static) —— -O2 下编译器会把非 volatile 的值
   缓存进寄存器, 请求永远读不到(同 ServiceState 那个坑).
   两者是"目标值"与"已切到的模式", 不相等时才需要动硬件 */
static volatile uint8_t s_spo2_req = 0U; /* 血氧服务要的: 0=关 1=开 */
static uint8_t          s_spo2_on  = 0U; /* 本任务已切到: 0=HR 1=SpO2(只本任务读写) */

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

/* 佩戴状态去抖: 连续多少拍(每拍 = 一个发布周期)判定一致才承认翻转.
   finger_on 来自直流分量阈值, 手指微动就在阈值上来回跳, 不去抖会刷屏事件 */
#define SERVICE_HEARTRATE_WEAR_DEBOUNCE     (3u)

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

/******************************************************************************
 * @name    heartrate_wear_report
 * @brief   佩戴状态翻转时发一次 EVT_SERVICE_HEARTRATE_WEAR(状态在 event_flags)
 * @param   worn[in] 本拍判定结果(1=佩戴)
 *
 * @return  无
 *
 * @note    连续 SERVICE_HEARTRATE_WEAR_DEBOUNCE 拍一致才认翻转, 且只在与上次
 *          上报的状态不同时才发 —— 状态不变就不发
 *****************************************************************************/
static void heartrate_wear_report(uint8_t worn)
{
    if (worn != s_wear_cand)
    {
        s_wear_cand     = worn;
        s_wear_cand_cnt = 1U;
    }
    else if (s_wear_cand_cnt < SERVICE_HEARTRATE_WEAR_DEBOUNCE)
    {
        s_wear_cand_cnt++;
    }

    if ((s_wear_cand_cnt >= SERVICE_HEARTRATE_WEAR_DEBOUNCE) &&
        (s_wear_cand != s_wear_reported))
    {
        s_wear_reported = s_wear_cand;
        (void)x_port_easyapp_event_send(EVT_SERVICE_HEARTRATE_WEAR,
                                        (uint32_t)s_wear_reported, NULL);
    }
}

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

            /* 起手停在 HR 模式(血氧关): 血氧只有血氧页测量期间才用得上, RED 路关着
               能省掉那个 LED 的功耗和一半的 FIFO 读出量(6 字节/样本 → 3 字节/样本).
               血氧服务要开时经 s_spo2_req 切过去.
               ★这里不碰 s_spo2_req★: 初始化这些毫秒里血氧任务若已经请求开, MEASURE
               循环第一轮就会切过去; 覆盖它反而把那次请求丢了 */
            ret = heartrate_bsp_change_to_HR();
            if (0 == ret)
            {
                ret = heartrate_bsp_enable_FIFO_FULL_interrupt();
            }
            if (0 != ret)
            {
#if HEARTRATE_SVC_LOOPBACK_TEST
                log_printf("[HEARTRATE] 切 HR 模式或开 FIFO 满中断失败, rc=%d\r\n", (int)ret);
#endif
                ServiceState = EVT_HEARTRATE_ERROR;
                break;
            }

            s_spo2_on = 0U; /* 硬件确实停在 HR 上了 */
            tick_pub = osKernelGetTickCount();
            error_cnt = 0;
            ServiceState = EVT_HEARTRATE_MEASURE;
#if HEARTRATE_SVC_LOOPBACK_TEST
            log_printf("[HEARTRATE] MAX30102 构造成功 + HR 模式 + FIFO 满中断已开, 等中断\r\n");
#endif
            break;

        case EVT_HEARTRATE_MEASURE:
            x_port_easyapp_event_send(EVT_SERVICE_HEARTRATE_NOBUSY, 0, NULL);

            /* 血氧开关: 请求和当前模式不符就切过去. ★只能在这里做★ —— 写 I2C +
               驱动内部清 FIFO 是慢操作, 而这是本循环唯一的空闲点(等中断之前).
               切完 FIFO 是空的, 接着的 wait_interrupt 会等下一个满中断 */
            if (s_spo2_on != s_spo2_req)
            {
                if (0U != s_spo2_req)
                {
                    ret = heartrate_bsp_change_to_spo2();

                    /* ★把上一轮的读数作废★: sp 是本函数的局部量、跨循环留着, 不清的话
                       从重开到血氧算法自己算出新值之间, 会一直拿几十秒前的旧结果往外报
                       —— 而它落在 [50,100] 里, 会一路骗过血氧服务那道 5 秒可信度门槛 */
                    sp.valid = 0U;
                }
                else
                {
                    ret = heartrate_bsp_change_to_HR();
                }

                if (0 != ret)
                {
                    error_cnt++;
                    if (error_cnt >= SERVICE_HEARTRATE_ERROR_LIMIT)
                    {
                        ServiceState = EVT_HEARTRATE_ERROR;
                    }
                    break;
                }

                s_spo2_on = s_spo2_req; /* 切成功才认账; 失败留着下轮重试 */
            }

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

                    /* 血氧关着就跳过: HR 模式下驱动把 s_red_buf 全填 0(见
                       max30102_read_all_FIFO_samples), 喂进去必然判 invalid, 纯白算.
                       省的是 100Hz 的逐样本浮点运算 */
                    if (0U != s_spo2_on)
                    {
                        (void)spo2_algo_update(s_red_buf[i], s_ir_buf[i], &sp);
                    }
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
                        ((0U != s_spo2_on) && (0 != sp.valid)) ? sp.spo2 : 0.0f;

                    /* 再发指针: 数据由本任务持续更新, 事件里不复制 */
                    x_port_easyapp_event_send(EVT_SERVICE_HEARTRATE_DATA, 0U, (void *)&s_latest_heartrate);

                    /* 佩戴状态: 每拍判一次, 只在翻转时发(去抖见宏) */
                    heartrate_wear_report(s_latest_heartrate.finger_on);
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
            x_port_easyapp_event_send(EVT_SERVICE_HEARTRATE_BUSY, 0, NULL);
            (void)heartrate_bsp_hibernating();
            ServiceState = EVT_HEARTRATE_NODONE;
            break;

        case EVT_HEARTRATE_WAKEUP:
            x_port_easyapp_event_send(EVT_SERVICE_HEARTRATE_BUSY, 0, NULL);
            (void)heartrate_bsp_wakeup();
            /* ★唤醒会把工作模式写回驱动的默认(HR)★ —— 见 max30102_wakeup, 所以本地
               缓存必须跟着清零, 否则血氧开着时会以为还停在 SpO2 上, 再也不切回去.
               置 0 后若血氧正开着, MEASURE 第一轮就会看到 on != req 并切回来 */
            s_spo2_on = 0U;
            /* 休眠期间 FIFO 已断档, 算法状态作废, 醒后重新收敛 */
            (void)heartrate_algo_reset();
            (void)spo2_algo_reset();
            tick_pub = osKernelGetTickCount();
            error_cnt = 0;
            ServiceState = EVT_HEARTRATE_MEASURE;
            break;

        case EVT_HEARTRATE_NODONE:
            x_port_easyapp_event_send(EVT_SERVICE_HEARTRATE_NOBUSY, 0, NULL);
            x_port_easyapp_event_send(EVT_SERVICE_HEARTRATE_NODONE, 0, NULL);
            osDelay(1000);
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
    if (NULL == osThreadNew(service_heartrate_run, NULL, &g_service_heartrate_attr))
    {
        /* 任务创建失败(堆不足等), 无心率服务可跑 */
        for (;;)
        {
        }
    }
}

void service_heartrate_sleep(void)
{
    /* 直接置状态位: 任务下一轮 switch 取到 EVT_HEARTRATE_SLEEP → hibernating.
       不用 MsgQueue: 全项目没有 osMessageQueuePut 的生产者, 队列只读不写是死通道;
       走队列还会在队满时静默丢请求, 而休眠请求不能丢 */
    ServiceState = EVT_HEARTRATE_SLEEP;
}

void service_heartrate_wakeup(void)
{
    ServiceState = EVT_HEARTRATE_WAKEUP;
}

void service_heartrate_get_data(HeartRate_Data_t *p_out)
{
    /* 锁调度器再整份拷贝: 心率任务每发布周期覆写 s_latest_heartrate, 不锁会跨帧撕裂 */
    (void)osKernelLock();
    *p_out = s_latest_heartrate;
    (void)osKernelUnlock();
}

void service_heartrate_set_spo2_enable(uint8_t enable)
{
    /* 只置目标值. 真正的写 I2C + 清 FIFO 由心率任务在 MEASURE 循环里做 —— 中间设备
       是器件级的共享资源, 而且 I2C 是慢操作, 不能直接在本函数(别的任务上下文)里干 */
    s_spo2_req = (0U != enable) ? 1U : 0U;
}
