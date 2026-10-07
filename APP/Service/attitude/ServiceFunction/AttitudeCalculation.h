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
 * @brief 姿态解算(加速度计+陀螺仪 → roll/pitch/yaw, 单位: 度).
 *        纯算法模块: 不依赖任何具体传感器驱动, 自身不做 I/O. 调用方采集 IMU
 *        样本(加速度 g、角速度 deg/s), 每次经 attitude_update() 喂入样本与步长,
 *        该函数一步完成滤波并回填欧拉角.
 *
 * 算法:
 * - roll/pitch: 绕X/Y轴的"一维角+零偏"两状态卡尔曼(陀螺积分预测 + 加速度计
 *   量测校正, 在线估零偏). 忽略两轴交叉耦合, 大角度(>60°)有误差;
 * - yaw: 无绝对参考(无磁力计), 只能 Z 轴陀螺积分, 会漂移 —— 要么只做短时相对
 *   航向, 要么静止时用 attitude_set_yaw() 手动对准.
 *
 * 坐标系: 以 IMU 封装坐标为准, 平放正面朝上时 az=+1g. 安装方向不同导致
 * roll/pitch 符号相反时, 把 .c 里对应公式取反即可(不要改量程/单位).
 *
 * @version V1.1
 *
 * @note 1 tab == 4 spaces!
 * @note 单实例模块(文件级静态状态), 非线程安全, 须在单个任务上下文串行调用.
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

/* 单步 dt 上限(秒): 超过则钳位, 防调度抖动/卡顿让陀螺积分跳变 */
#define ATTITUDE_KALMAN_DT_MAX_S            0.2f
/***********************************Defines************************************/

/**********************************Declaring***********************************/
/* 一次采样: 加速度(g) + 角速度(deg/s) */
typedef struct
{
    float ax; /* X轴加速度, g */
    float ay; /* Y轴加速度, g */
    float az; /* Z轴加速度, g */
    float gx; /* X轴角速度, deg/s */
    float gy; /* Y轴角速度, deg/s */
    float gz; /* Z轴角速度, deg/s */
} attitude_imu_sample_t;

/* 初始化: 噪声参数复位默认 + 清空姿态状态. 不初始化直接 update 亦可(静态零值
 * 即默认), 本函数用于显式/反复初始化. */
void attitude_init(void);

/* 运行时调整滤波器噪声参数(<=0 的参数保持不变). */
void attitude_set_filter_noise(float q_angle,
                               float q_bias,
                               float r_measure);

/* 姿态解算单步(纯计算): 喂入一次 IMU 样本与距上次调用的秒数 dt_s, 一步完成滤波
 * 并把最新 roll/pitch/yaw 回填到输出指针. 首次调用(或 reset 后)只用加速度计建立
 * roll/pitch 初值, 忽略 dt_s. 应在单任务上下文按节拍调用(如每 10ms).
 * 三个输出指针可传 NULL 表示不取; dt_s 超上限会被钳位.
 * @return 0 成功 / -1 p_sample 为空 / -2 三个输出指针全为空
 */
int8_t attitude_update(const attitude_imu_sample_t *p_sample,
                       float dt_s,
                       float *p_roll_deg,
                       float *p_pitch_deg,
                       float *p_yaw_deg);

/* 手动设定航向角(度), 例如静止对准后用外部罗盘或约定方向重置 yaw 参考. */
void attitude_set_yaw(float yaw_deg);

/* 姿态清零并重新收敛: 复位 roll/pitch 滤波器与 yaw, 下次 update 将重新以加速度计
 * 建立初值. 保留噪声参数设置. */
void attitude_reset(void);
/**********************************Declaring***********************************/

#endif /* __ATTITUDE_CALCULATION_H__ */
