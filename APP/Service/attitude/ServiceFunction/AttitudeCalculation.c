/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file AttitudeCalculation.c
 *
 * @par dependencies
 * - stddef.h / stdint.h(经头文件)
 * - 数学库(libm, 用到 atan2f/sqrtf, 链接需加 -lm 或工程勾选 Use MicroLIB)
 *
 * @author  -
 *
 * @brief 姿态解算实现: 纯算法, 与具体传感器驱动解耦.
 *        模块不知道也不关心数据从哪来(MPU6050/其他 IMU/仿真), 调用方把一次
 *        加速度计(g)+陀螺仪(deg/s)样本与步长 dt 交给 attitude_update() 即可,
 *        本模块只负责卡尔曼滤波/积分, 一步回填最新欧拉角. 模块内无任何 I/O.
 *
 * Processing flow:
 * 1. attitude_init(): 噪声参数复位默认 + 清空姿态(可选, 静态零值即默认);
 * 2. (可选) attitude_set_filter_noise(): 调噪声;
 * 3. 调用方按节拍(如 10ms 任务)采集 IMU 数据 → 填 attitude_imu_sample_t →
 *    attitude_update(&sample, dt_s, &roll, &pitch, &yaw), dt_s 由调用方给
 *    (10ms 任务即 0.010f); 一次调用同时完成滤波与取角;
 * 4. 若要复位/对准, 用 attitude_reset() / attitude_set_yaw().
 *
 * roll/pitch 解算原理(每轴一个"角度+零偏"两状态卡尔曼):
 * - 预测: 角度 += (陀螺转速 - 估计零偏) * dt, 零偏不变, 协方差按 q_angle/q_bias
 *   扩散(过程噪声);
 * - 校正: 量测为加速度计反解的重力倾角(静止时准确, 动态时有线加速度干扰),
 *   用 r_measure 表征量测噪声, 计算卡尔曼增益后同时修正角度与在线零偏估计;
 * 由此实现"短时信陀螺、长时信加速度计", 并在静止时自动收敛 X/Y 轴零偏.
 * yaw: 仅有 Z 轴陀螺, 无绝对参考, 直接角速度积分(会漂移, 见头文件说明).
 *
 * 角度/坐标系约定: 平放正面朝上 az=+1g; roll/pitch 用数据手册右手系公式.
 * 若板子安装方向导致符号相反, 改 accel_to_roll_deg/accel_to_pitch_deg 与
 * update() 里的陀螺轴代入顺序即可(不要改量程/单位).
 *
 * @version V1.1
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 本文件内全部为浮点纯计算, 无阻塞/I2C, 可在任务上下文按节拍调用;
 *       但文件级状态单实例、非线程安全, 不要在多任务/ISR 里并发调用.
 ******************************************************************************/

/***********************************Includes************************************/
#include <math.h>
#include "AttitudeCalculation.h"
/***********************************Includes************************************/

/***********************************Defines*************************************/
/* 度 <-> 弧度换算 */
#define ATTITUDE_RAD2DEG        (57.29577951308232087679f) /* 180/PI */
#define ATTITUDE_DEG360         (360.0f)
#define ATTITUDE_DEG180         (180.0f)
/***********************************Defines*************************************/

/************************************Types**************************************/
/* 一维(单轴)"角度+零偏"两状态卡尔曼滤波器参数 */
typedef struct
{
    float angle; /* 滤波后角度, deg */
    float bias;  /* 该轴陀螺零偏估计, deg/s */
    float p00;   /* 协方差 [0][0] */
    float p01;   /* 协方差 [0][1] */
    float p10;   /* 协方差 [1][0] */
    float p11;   /* 协方差 [1][1] */
} kalman_1d_t;
/************************************Types**************************************/

/***********************************Variables***********************************/
static kalman_1d_t s_kf_roll;   /* 横滚轴滤波器, deg */
static kalman_1d_t s_kf_pitch;  /* 俯仰轴滤波器, deg */
static float       s_yaw_deg = 0.0f; /* 航向, deg(仅 Z 陀螺积分, 会漂移) */

/* 滤波器噪声参数(默认见头文件宏; <=0 时 set_filter_noise 不改动) */
static float s_q_angle   = ATTITUDE_KALMAN_Q_ANGLE_DEFAULT;   /* 角度过程噪声 */
static float s_q_bias    = ATTITUDE_KALMAN_Q_BIAS_DEFAULT;    /* 零偏过程噪声 */
static float s_r_measure = ATTITUDE_KALMAN_R_MEASURE_DEFAULT; /* 量测噪声 */

/* 首步标记: 0 = 尚未用加速度计建立初值(下次 update 先初始化) */
static uint8_t s_started = 0;
/***********************************Variables***********************************/

/******************************Function Prototypes******************************/
static void kalman_1d_reset(kalman_1d_t *p_kf);
static void kalman_1d_update(kalman_1d_t *p_kf,
                             float rate_dps,
                             float meas_deg,
                             float dt_s);
static float accel_to_roll_deg(float ax_g, float ay_g, float az_g);
static float accel_to_pitch_deg(float ax_g, float ay_g, float az_g);
static float wrap_deg(float angle_deg);
/******************************Function Prototypes******************************/

/**
 * @name  kalman_1d_reset
 * @brief 清空单轴卡尔曼滤波器(角度/零偏/协方差全部归零). 归零协方差意味着
 *        下次 kalman_1d_update 前的校正会强烈信任量测, 适合重新开始收敛.
 * @param p_kf[out] 滤波器实例
 * @return 无
 * @note  静态函数, 仅供本文件内部调用.
 */
static void kalman_1d_reset(kalman_1d_t *p_kf)
{
    if (NULL == p_kf)
    {
        return;
    }

    p_kf->angle = 0.0f;
    p_kf->bias  = 0.0f;
    p_kf->p00   = 0.0f;
    p_kf->p01   = 0.0f;
    p_kf->p10   = 0.0f;
    p_kf->p11   = 0.0f;
}

/**
 * @name  kalman_1d_update
 * @brief 单轴卡尔曼一步(预测+校正): 用该轴陀螺转速(deg/s)对角度做短时预测,
 *        再用加速度计量测角度(deg)校正, 在线修正零偏.
 * @param p_kf[in/out] 滤波器实例(读上一步状态, 写本步状态)
 * @param rate_dps[in] 该轴角速度, deg/s
 * @param meas_deg[in] 该轴加速度计量测角度, deg
 * @param dt_s[in]     距上一步的秒数
 * @return 无
 * @note  经典"角度+零偏"离散卡尔曼, 实现来自公开的 gyro+accel Kalman 参考.
 *        噪声参数(过程/量测)取自模块级 s_q_* 全局, 便于 set_filter_noise 调整.
 */
static void kalman_1d_update(kalman_1d_t *p_kf,
                             float rate_dps,
                             float meas_deg,
                             float dt_s)
{
    float rate = 0.0f; /* 去除零偏后的角速度 */
    float s    = 0.0f; /* 新息方差 S = P00 + R */
    float k0   = 0.0f; /* 角度增益 */
    float k1   = 0.0f; /* 零偏增益 */
    float y    = 0.0f; /* 新息(量测-预测) */

    /* ---- 预测: 角度按(转速-零偏)积分, 零偏视为缓变量 ---- */
    rate = rate_dps - p_kf->bias;
    p_kf->angle += dt_s * rate;

    /* 协方差预测: FPF^T + Q (Q_angle 扩角度方差, Q_bias 扩零偏方差) */
    p_kf->p00 += dt_s * (dt_s * p_kf->p11 - p_kf->p01 - p_kf->p10 + s_q_angle);
    p_kf->p01 -= dt_s * p_kf->p11;
    p_kf->p10 -= dt_s * p_kf->p11;
    p_kf->p11 += s_q_bias * dt_s;

    /* ---- 校正: 加速度计量测修正角度与零偏 ---- */
    s  = p_kf->p00 + s_r_measure;
    k0 = p_kf->p00 / s;
    k1 = p_kf->p10 / s;

    y            = meas_deg - p_kf->angle;
    p_kf->angle += k0 * y;
    p_kf->bias  += k1 * y;

    /* 协方差校正 */
    p_kf->p00 -= k0 * p_kf->p00;
    p_kf->p01 -= k0 * p_kf->p01;
    p_kf->p10 -= k1 * p_kf->p00;
    p_kf->p11 -= k1 * p_kf->p01;
}

/**
 * @name  accel_to_roll_deg
 * @brief 由加速度计三分量(g)反解横滚角: 绕X轴倾斜. 平放 az=+1g 时 roll=0,
 *        左/右倾时按右手系绕X轴旋转得到.
 * @param ax_g[in] X轴加速度, g
 * @param ay_g[in] Y轴加速度, g
 * @param az_g[in] Z轴加速度, g
 * @return 横滚角, 度, [-180,180]
 * @note  仅静止/准静态准确, 有平移线加速度时引入误差(卡尔曼以 r_measure
 *        部分抑制). 安装方向不同可在此调符号.
 */
static float accel_to_roll_deg(float ax_g, float ay_g, float az_g)
{
    (void)ax_g;
    return atan2f(ay_g, az_g) * ATTITUDE_RAD2DEG;
}

/**
 * @name  accel_to_pitch_deg
 * @brief 由加速度计三分量(g)反解俯仰角: 绕Y轴倾斜. 平放 az=+1g 时 pitch=0.
 * @param ax_g[in] X轴加速度, g
 * @param ay_g[in] Y轴加速度, g
 * @param az_g[in] Z轴加速度, g
 * @return 俯仰角, 度, [-90,90](用水平面投影求反正切, 俯仰不随横滚奇异)
 * @note  安装方向不同可在此调符号.
 */
static float accel_to_pitch_deg(float ax_g, float ay_g, float az_g)
{
    return atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * ATTITUDE_RAD2DEG;
}

/**
 * @name  wrap_deg
 * @brief 把角度折回 [-180,180).
 * @param angle_deg[in] 任意角度, 度
 * @return 折回后的角度, 度, [-180,180)
 */
static float wrap_deg(float angle_deg)
{
    angle_deg = fmodf(angle_deg, ATTITUDE_DEG360);
    if (angle_deg >= ATTITUDE_DEG180)
    {
        angle_deg -= ATTITUDE_DEG360;
    }
    else if (angle_deg < -ATTITUDE_DEG180)
    {
        angle_deg += ATTITUDE_DEG360;
    }
    return angle_deg;
}

/**
 * @name  attitude_init
 * @brief 初始化姿态模块: 噪声参数复位为头文件默认并清空姿态状态.
 *        静态零值本身即默认, 此函数用于显式/反复初始化(如再次上电对准前).
 * @return 无
 */
void attitude_init(void)
{
    s_q_angle   = ATTITUDE_KALMAN_Q_ANGLE_DEFAULT;
    s_q_bias    = ATTITUDE_KALMAN_Q_BIAS_DEFAULT;
    s_r_measure = ATTITUDE_KALMAN_R_MEASURE_DEFAULT;
    attitude_reset();
}

/**
 * @name  attitude_set_filter_noise
 * @brief 运行时调整滤波器噪声参数; 传入 <=0 的参数保持不变(便于只改其中一两个).
 * @param q_angle[in]   角度过程噪声(越大越信加速度计/越跟手但有毛刺)
 * @param q_bias[in]    零偏过程噪声(越大零偏收敛越快, 但会把扰动误当零偏)
 * @param r_measure[in] 量测噪声(越大越信陀螺积分, 输出越平滑但动态滞后)
 * @return 无
 */
void attitude_set_filter_noise(float q_angle,
                               float q_bias,
                               float r_measure)
{
    if (q_angle > 0.0f)
    {
        s_q_angle = q_angle;
    }
    if (q_bias > 0.0f)
    {
        s_q_bias = q_bias;
    }
    if (r_measure > 0.0f)
    {
        s_r_measure = r_measure;
    }
}

/**
 * @name  attitude_update
 * @brief 姿态解算单步(纯计算): 喂入一次 IMU 样本与距上次调用的秒数 dt_s,
 *        一步完成滤波并回填最新欧拉角(融合了原 attitude_get_angles).
 *        roll/pitch 分别做卡尔曼(陀螺积分预测 + 加速度计量测校正, 在线估零偏);
 *        yaw 仅积分 Z 轴陀螺(无绝对参考, 会漂移).
 *        首次调用(或 attitude_reset 后)只用加速度计建立 roll/pitch 初值并回填,
 *        此时忽略 dt_s; 之后每次调用消耗一个样本与一步 dt.
 * @param p_sample[in]     本次 6 轴样本(加速度 g + 角速度 deg/s), 不可为空
 * @param dt_s[in]         距上次 update 的秒数(例如 10ms 任务传 0.010f);
 *                         超过 ATTITUDE_KALMAN_DT_MAX_S 会被钳位以抗调度抖动,
 *                         负值按 0 处理
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
                       float *p_yaw_deg)
{
    float acc_roll_deg  = 0.0f; /* 加速度计量测横滚角 */
    float acc_pitch_deg = 0.0f; /* 加速度计量测俯仰角 */

    if (NULL == p_sample)
    {
        return -1;
    }
    if ((NULL == p_roll_deg) && (NULL == p_pitch_deg) && (NULL == p_yaw_deg))
    {
        return -2;
    }

    /* 步长保护: 只接受 (0, DT_MAX_S] 内的正步长 */
    if (dt_s < 0.0f)
    {
        dt_s = 0.0f;
    }
    else if (dt_s > ATTITUDE_KALMAN_DT_MAX_S)
    {
        dt_s = ATTITUDE_KALMAN_DT_MAX_S;
    }

    /* 加速度计先反解出两个参考角(卡尔曼量测) */
    acc_roll_deg  = accel_to_roll_deg(p_sample->ax, p_sample->ay, p_sample->az);
    acc_pitch_deg = accel_to_pitch_deg(p_sample->ax, p_sample->ay, p_sample->az);

    /* 首步: 用当前加速度计倾角直接建立初值, 不做滤波/积分 */
    if (0 == s_started)
    {
        kalman_1d_reset(&s_kf_roll);
        kalman_1d_reset(&s_kf_pitch);
        s_kf_roll.angle  = acc_roll_deg;
        s_kf_pitch.angle = acc_pitch_deg;
        s_started = 1;
    }
    else
    {
        /* 常规步: roll 融合 X 陀螺, pitch 融合 Y 陀螺 */
        kalman_1d_update(&s_kf_roll,  p_sample->gx, acc_roll_deg,  dt_s);
        kalman_1d_update(&s_kf_pitch, p_sample->gy, acc_pitch_deg, dt_s);

        /* yaw 无参考, 仅 Z 轴陀螺积分并折回 [-180,180) */
        s_yaw_deg = wrap_deg(s_yaw_deg + p_sample->gz * dt_s);
    }

    /* 回填滤波结果(输出指针允许部分为 NULL) */
    if (NULL != p_roll_deg)
    {
        *p_roll_deg = s_kf_roll.angle;
    }
    if (NULL != p_pitch_deg)
    {
        *p_pitch_deg = s_kf_pitch.angle;
    }
    if (NULL != p_yaw_deg)
    {
        *p_yaw_deg = s_yaw_deg;
    }

    return 0;
}

/**
 * @name  attitude_set_yaw
 * @brief 手动设定当前航向(度). 用于静止时给无参考的 yaw 一个初值/对准:
 *        例: 静止朝正北 → attitude_set_yaw(0.0f); 内部会折回 [-180,180).
 * @param yaw_deg[in] 目标航向, 度
 * @return 无
 */
void attitude_set_yaw(float yaw_deg)
{
    s_yaw_deg = wrap_deg(yaw_deg);
}

/**
 * @name  attitude_reset
 * @brief 姿态清零并重新收敛: 清空 roll/pitch 滤波器与 yaw, 置回"未开始"态,
 *        下次 attitude_update 会重新以加速度计建立初值. 保留噪声参数设置
 *        (set_filter_noise 的结果不丢).
 * @return 无
 */
void attitude_reset(void)
{
    kalman_1d_reset(&s_kf_roll);
    kalman_1d_reset(&s_kf_pitch);
    s_yaw_deg = 0.0f;
    s_started = 0;
}
