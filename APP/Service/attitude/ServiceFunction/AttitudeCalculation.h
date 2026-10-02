/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file AttitudeCalculation.h
 *
 * @par dependencies
 * - stdint.h / stddef.h
 * - 数学库(libm, 用到 atan2f/sqrtf)
 *
 * @author  -
 *
 * @brief 姿态解算(卡尔曼融合 加速度计+陀螺仪 → roll/pitch/yaw, 单位: 度).
 *        纯算法模块: 不依赖也不认识任何具体传感器驱动(如MPU6050), 自身不做任何
 *        I/O. 调用方负责采集 IMU 数据(加速度 单位g、角速度 单位deg/s), 每次通过
 *        attitude_update() 把样本与步长喂进来, 该函数一步完成滤波计算并回填
 *        最新欧拉角; 本模块只负责计算, 无单独"取结果"的接口.
 *
 * Processing flow:
 * 1. attitude_init() 把噪声参数复位为默认并清空姿态状态;
 * 2. (可选) attitude_set_filter_noise() 运行时调整噪声参数;
 * 3. 按固定节拍调用 attitude_update(&sample, dt_s, &roll, &pitch, &yaw):
 *    dt_s 为距上次 update 的秒数(例如 10ms 任务传 0.010f). 每次调用既消耗样本
 *    又通过输出指针回填滤波后的姿态角. 首次调用仅用加速度计建立初值.
 *
 * 算法说明:
 * - roll/pitch: 绕X/Y轴的"一维角+零偏"两状态卡尔曼滤波, 量测为加速度计
 *   反解出的倾斜角(静止时重力方向), 短时动态由陀螺积分, 长时间由加速度计校正,
 *   同时在线估计X/Y轴陀螺零偏(初值为0, 静止后自动收敛). 忽略横滚/俯仰间交叉
 *   耦合, 大角度(>60°)时有误差;
 * - yaw: 航向没有绝对参考(无磁力计/外部罗盘), 只能用Z轴陀螺积分, 会随时间
 *   漂移. 建议: 要么只做"短时相对航向", 要么静止时用 attitude_set_yaw() 手动
 *   对准初始航向.
 *
 * 坐标系/安装方向: 默认以 IMU 数据手册封装坐标为准(平放正面朝上时 az=+1g).
 * 若板子安装方向与芯片坐标不一致导致 roll/pitch 符号相反, 把加速度计角度公式或
 * 对应陀螺轴取反即可(见 .c 内注释).
 *
 * @version V1.1
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 单实例模块(文件级静态状态), 同一时刻只服务一路姿态; 内部无I/O但非线程
 *       安全, 需在单个任务上下文串行调用(不要在多任务/ISR 里并发调 update).
 ******************************************************************************/
#ifndef __ATTITUDE_CALCULATION_H__
#define __ATTITUDE_CALCULATION_H__

/***********************************Includes***********************************/
#include <stdint.h>
#include <stddef.h>
/***********************************Includes***********************************/

/***********************************Defines************************************/
/* 卡尔曼滤波器默认噪声参数(可通过 set_filter_noise 运行时调整) */
#define ATTITUDE_KALMAN_Q_ANGLE_DEFAULT     0.001f  /* 角度过程噪声 */
#define ATTITUDE_KALMAN_Q_BIAS_DEFAULT      0.003f  /* 零偏过程噪声 */
#define ATTITUDE_KALMAN_R_MEASURE_DEFAULT   0.03f   /* 量测(加速度计角度)噪声 */

/* 单步 dt 的上限保护(秒): 调用方传入的 dt_s 超过此值会钳位, 防止因调度抖动/
   卡顿导致陀螺积分跳变. 需要可设为更大值或自行节拍管理 */
#define ATTITUDE_KALMAN_DT_MAX_S            0.2f
/***********************************Defines************************************/

/**********************************Declaring***********************************/
/* 一次采样: 加速度(g) + 角速度(deg/s). 单位/量程由调用方在采集处统一换算 */
typedef struct
{
    float ax; /* X轴加速度, g */
    float ay; /* Y轴加速度, g */
    float az; /* Z轴加速度, g */
    float gx; /* X轴角速度, deg/s */
    float gy; /* Y轴角速度, deg/s */
    float gz; /* Z轴角速度, deg/s */
} attitude_imu_sample_t;

/* 初始化: 噪声参数复位默认并清空姿态状态. 无失败可能, 无返回值.
 * 不初始化直接 update 亦可(静态零值即为默认), 此函数用于显式/反复初始化. */
void attitude_init(void);

/* 运行时调整滤波器噪声参数(<=0的参数保持不变). 无返回值. */
void attitude_set_filter_noise(float q_angle,
                               float q_bias,
                               float r_measure);

/* 姿态解算单步(纯计算, 无I/O): 喂入一次 IMU 样本与距上次调用的秒数 dt_s,
 * 一步完成滤波计算并把最新 roll/pitch/yaw 回填到输出指针.
 * roll/pitch 做"陀螺积分预测+加速度计量测校正"卡尔曼; yaw 积分Z轴陀螺.
 * 首次调用(或 reset 后)仅用加速度计建立 roll/pitch 初值, 忽略 dt_s.
 * 应在单任务上下文按节拍调用(如每 10ms).
 * @param p_sample[in]     本次加速度计/陀螺仪样本(不可为空)
 * @param dt_s[in]         距上次 update 的秒数(>ATTITUDE_KALMAN_DT_MAX_S 会被钳位)
 * @param p_roll_deg[out]  滤波后横滚角, 度, [-180,180](可传 NULL 表示不取)
 * @param p_pitch_deg[out] 滤波后俯仰角, 度, [-90,90](可传 NULL 表示不取)
 * @param p_yaw_deg[out]   航向角, 度, [-180,180](可传 NULL 表示不取)
 *
 * @return  0 success
 *         -1 p_sample null
 *         -2 三个输出指针全为空(至少要一个有效输出)
 */
int8_t attitude_update(const attitude_imu_sample_t *p_sample,
                       float dt_s,
                       float *p_roll_deg,
                       float *p_pitch_deg,
                       float *p_yaw_deg);

/* 手动设定航向角(度), 例如静止对准后用外部罗盘或约定方向重置 yaw 参考. */
void attitude_set_yaw(float yaw_deg);

/* 姿态清零并重新收敛: 复位 roll/pitch 滤波器与 yaw, 下次 update 将重新以
 * 加速度计建立初值. 保留噪声参数设置. */
void attitude_reset(void);
/**********************************Declaring***********************************/

#endif /* __ATTITUDE_CALCULATION_H__ */
