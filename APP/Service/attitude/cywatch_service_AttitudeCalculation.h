#ifndef __CYWATCH_SERVICE_ATTITUDECALCULATION_H__
#define __CYWATCH_SERVICE_ATTITUDECALCULATION_H__

/* 样本类型(attitude_imu_sample_t)来自纯算法模块 */
#include "ServiceFunction/AttitudeCalculation.h"

typedef enum{
    EVT_INIT = 0,
    EVT_CALCULATION,
    EVT_SLEEP,
    EVT_WAKEUP,
    EVT_NODONE,
    EVT_ERROR,
}State_AttitudeCalculation_Setvice_t;

/* 一帧姿态数据: 本帧 6 轴样本(已由驱动换算为 g / deg-s) + 滤波后欧拉角(deg).
   EasyAPP 事件 EVT_SERVICE_ATTITUDE_DATA 的 event_data 即指向本类型——
   指向服务内部持有的一份"静态最新帧", APP 层按只读使用, 不要 free/改所有权 */
typedef struct Attitude_Data
{
    attitude_imu_sample_t attitude_sample; /* 本帧 6 轴: 加速度(g), 角速度(deg/s) */
    float roll;   /* 滤波后横滚角, deg */
    float pitch;  /* 滤波后俯仰角, deg */
    float yaw;    /* 航向角, deg(仅 Z 陀螺积分, 会漂移) */
} Attitude_Data_t;

/**
 * @name  service_attitudecalculation_init
 * @brief 姿态服务初始化: 创建姿态任务
 * @return 无
 */
void service_attitudecalculation_init(void);

#endif
