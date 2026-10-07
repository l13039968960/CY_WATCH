#ifndef __CYWATCH_SERVICE_ATTITUDECALCULATION_H__
#define __CYWATCH_SERVICE_ATTITUDECALCULATION_H__

/* Attitude_Data_t 的成员类型来源, 下面两个都要 */
#include "ServiceFunction/AttitudeCalculation.h"
#include "ServiceFunction/StepDetection.h"

/* 一帧姿态数据: 6 轴样本(驱动已换算为 g / deg-s) + 滤波后欧拉角(deg) + 计步结果.
   整帧经 service_attitudecalculation_get_data() 拉取; 事件只发步数, 即
   EVT_SERVICE_STEP_DATA 的 event_data 指向本类型里那份"静态最新帧"的 step 字段,
   APP 层按只读用, 别 free/改所有权. */
typedef struct Attitude_Data
{
    attitude_imu_sample_t attitude_sample; /* 本帧 6 轴: 加速度(g), 角速度(deg/s) */
    float roll;   /* 滤波后横滚角, deg */
    float pitch;  /* 滤波后俯仰角, deg */
    float yaw;    /* 航向角, deg(仅 Z 陀螺积分, 会漂移) */
    step_result_t step; /* 计步结果: 累计步数/步频(步-分)/三态(静止-走路-跑步) */
} Attitude_Data_t;

/* 姿态服务初始化: 创建姿态任务 */
void service_attitudecalculation_init(void);

/* 读"最新一帧"的拷贝快照(6轴 + 欧拉角 + 计步). 内部锁调度器整份拷贝, 拿到的那份不会
 * 被下一帧覆盖, 也不会跨帧撕裂. 服务未初始化时返回全 0. */
void service_attitudecalculation_get_data(Attitude_Data_t *p_out);

/* 请求休眠/唤醒: 只置状态位, 真正的 MPU6050 hibernating/wakeup 由姿态任务执行, 最慢一个
 * 采样节拍(10ms)后生效. 与 service_lvgl_sleep/wakeup 同一套路.
 * 可在任意任务上下文调用, 禁止 ISR; 重复请求幂等. */
void service_attitudecalculation_sleep(void);
void service_attitudecalculation_wakeup(void);

#endif
