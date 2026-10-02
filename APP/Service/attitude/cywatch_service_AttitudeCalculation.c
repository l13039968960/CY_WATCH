#include "cywatch_service_AttitudeCalculation.h"
#include "ServiceFunction/AttitudeCalculation.h"
#include "cywatch_adapter_mpu6050.h"
#include "cmsis_os2.h"
#include "easyapp_port.h"
#include "system/log/cywatch_log.h"

/* ===== 闭环测试打印开关 =====
   1 = 每 500ms 打一行姿态角与 6 轴原始值; ★测完改回 0★
   (main.c 里的宏跨不了翻译单元, 本文件的开关只对本文件生效) */
#define ATTITUDE_SVC_LOOPBACK_TEST  0

static State_AttitudeCalculation_Setvice_t ServiceState;
osMessageQueueId_t AttitudeCalculationMsgQueue;

/* 服务独占写的"最新帧": 每次姿态更新后先填这里, 再把地址随事件发出去(0 拷贝, 生产者持有),
   APP 层拿到指针读到的一直是最新一份. 注意每 10ms 被本任务覆盖, 消费侧应尽快读走 */
static Attitude_Data_t s_latest_attitude;

/* 姿态采样节拍: 内核 tick 1kHz == 1ms, 每拍 10ms */
#define SERVICE_ATTITUDE_LOOP_TICK       (10u)

static const osThreadAttr_t g_service_attitude_attr =
{
    .name       = "attitude",
    .attr_bits  = 0,
    .cb_mem     = NULL,
    .cb_size    = 0,
    .stack_mem  = NULL,
    .stack_size = 1024,
    .priority   = osPriorityNormal,
};

static void service_attitudecalculation_run(void *pvParameters)
{
    attitude_imu_sample_t attitude_sample = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; /* 本次6轴样本 */
    float roll  = 0.0f; /* 滤波后横滚角, deg */
    float pitch = 0.0f; /* 滤波后俯仰角, deg */
    float yaw   = 0.0f; /* 航向角, deg(仅Z陀螺积分, 会漂移) */
    uint32_t tick_prev = 0;  /* 上一拍成功帧的内核 tick */
    uint32_t tick_now  = 0;  /* 本拍内核 tick */
    float dt_s = 0.0f;       /* 距上一成功帧的真实秒数 */
    int8_t ret = 0;
    int8_t error_cnt = 0;
#if ATTITUDE_SVC_LOOPBACK_TEST
    uint32_t print_div = 0;
#endif

    (void)pvParameters;

    while(1)
    {
        State_AttitudeCalculation_Setvice_t ReceiveState;
        ret = osMessageQueueGet(AttitudeCalculationMsgQueue, &ReceiveState, NULL, 0);
        if(ret == osOK)
            ServiceState = ReceiveState;
        switch (ServiceState)
        {
        case EVT_INIT:
            x_port_easyapp_event_send(EVT_SERVICE_ATTITUDE_BUSY, 0, NULL);
            error_cnt = 0;
            while (0 != attitudecalculation_bsp_inst())
            {
                osDelay(500);
                error_cnt++;
                if (error_cnt >= 5)
                {
                    ServiceState = EVT_ERROR;
                    break;
                }
            }
            /* 5 次实例化全失败已切到 EVT_ERROR: 跳过下面的正常初始化, 交给 EVT_ERROR 分支处理,
               否则 ServiceState 会被覆盖回 EVT_CALCULATION, 错误分支形同虚设 */
            if (EVT_ERROR == ServiceState)
            {
#if ATTITUDE_SVC_LOOPBACK_TEST
                log_printf("[ATTITUDE] MPU6050 连续 5 次构造失败, 进错误态(查接线/供电/上拉)\r\n");
#endif
                break;
            }
            attitude_init();
            tick_prev = osKernelGetTickCount();
            ServiceState = EVT_CALCULATION;
            error_cnt = 0;
#if ATTITUDE_SVC_LOOPBACK_TEST
            log_printf("[ATTITUDE] MPU6050 构造成功, 开始 10ms 周期采样\r\n");
#endif
            break;
        case EVT_CALCULATION:
            /* 读一次 6 轴(加速度单位 g, 角速度单位 deg/s, 已由 adapter 换算) */
            ret = attitudecalculation_bsp_read_accel(&attitude_sample.ax, &attitude_sample.ay, &attitude_sample.az);
            if (0 == ret)
            {
                ret = attitudecalculation_bsp_read_gyro(&attitude_sample.gx, &attitude_sample.gy, &attitude_sample.gz);
            }
            if (0 == ret)
            {
                tick_now = osKernelGetTickCount();
                dt_s = (float)(tick_now - tick_prev) * 0.001f;
                tick_prev = tick_now;

                attitude_update(&attitude_sample, dt_s, &roll, &pitch, &yaw);

                /* 先更新最新帧: 6轴样本 + 滤波后欧拉角 */
                s_latest_attitude.attitude_sample = attitude_sample;
                s_latest_attitude.roll  = roll;
                s_latest_attitude.pitch = pitch;
                s_latest_attitude.yaw   = yaw;
                /* 再发指针: 数据由本任务持续更新, 事件里不复制 */
                x_port_easyapp_event_send(EVT_SERVICE_ATTITUDE_DATA, 0U, (void *)&s_latest_attitude);
#if ATTITUDE_SVC_LOOPBACK_TEST
                /* 每 50 帧(10ms/帧 ≈ 500ms)打一行; MicroLIB 无 %f, 角度放大 100 倍打整数 */
                if (++print_div >= 50u)
                {
                    print_div = 0;
                    log_printf("[ATTITUDE] roll=%d pitch=%d yaw=%d (0.01deg) | "
                               "acc x=%d y=%d z=%d (0.01g) gyro x=%d y=%d z=%d (0.01dps)\r\n",
                               (int)(roll * 100.0f), (int)(pitch * 100.0f), (int)(yaw * 100.0f),
                               (int)(attitude_sample.ax * 100.0f),
                               (int)(attitude_sample.ay * 100.0f),
                               (int)(attitude_sample.az * 100.0f),
                               (int)(attitude_sample.gx * 100.0f),
                               (int)(attitude_sample.gy * 100.0f),
                               (int)(attitude_sample.gz * 100.0f));
                }
#endif
            }
            else
            {
                error_cnt++;
                if(error_cnt >= 5)
                {
#if ATTITUDE_SVC_LOOPBACK_TEST
                    log_printf("[ATTITUDE] 连续 5 帧 I2C 读失败, 进错误态(读的是共享 I2C, 查接线)\r\n");
#endif
                    ServiceState = EVT_ERROR;
                    break;
                }
                /* I2C 偶发失败: 丢本帧并重基准, 避免把长时间断档当成大步长 dt */
                tick_prev = osKernelGetTickCount();
            }
            x_port_easyapp_event_send(EVT_SERVICE_ATTITUDE_NOBUSY, 0, NULL);
            osDelay(10);
            break;
        case EVT_SLEEP:
            attitudecalculation_bsp_hibernating();
            ServiceState = EVT_NODONE;
            break;
        case EVT_WAKEUP:
            attitudecalculation_bsp_wakeup();
            ServiceState = EVT_CALCULATION;
            break;
        case EVT_NODONE:
            /*NOTING DONE*/
            x_port_easyapp_event_send(EVT_SERVICE_ATTITUDE_NODONE, 0, NULL);
            osDelay(100);
            break;
        case EVT_ERROR:
            x_port_easyapp_event_send(EVT_SERVICE_ATTITUDE_BUSY, 0, NULL);
            /*发送服务出错事件*/
            x_port_easyapp_event_send(EVT_SERVICE_ATTITUDE_ERROR, 0, NULL);
            osDelay(100);
            break;
        default:
            break;
        }   


    }
}


void service_attitudecalculation_init(void)
{
    ServiceState = EVT_INIT;
    AttitudeCalculationMsgQueue = osMessageQueueNew(1, sizeof(State_AttitudeCalculation_Setvice_t), NULL);
    if (NULL == osThreadNew(service_attitudecalculation_run, NULL, &g_service_attitude_attr))
    {
        /* 任务创建失败(堆不足等), 无姿态服务可跑 */
        for (;;)
        {
        }
    }
}
