/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file SpO2.c
 *
 * @par dependencies
 * - SpO2.h
 * - math.h
 * - stddef.h
 *
 * @author	zw1194
 *
 * @brief Implete the SpO2 pure algorithm (ratio of ratios).
 *
 * Processing flow:
 *
 * call spo2_algo_init() once, then spo2_algo_update() per RED/IR sample pair.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
#include "SpO2.h"

#include <math.h>
#include <stddef.h>

/***********************************Defines************************************/
#define SPO2_TWO_PI             (6.28318530718f)  /* 2π */

/* Maxim AN6400 针对 MAX3010x 评估板的经验二次曲线系数 */
#define SPO2_CURVE_A            (-45.06f)
#define SPO2_CURVE_B            (30.354f)
#define SPO2_CURVE_C            (94.845f)

/* 经典线性近似 SpO2 = 110 - 25·R */
#define SPO2_LINEAR_B           (-25.0f)
#define SPO2_LINEAR_C           (110.0f)
/***********************************Defines************************************/

/********************************* 内部状态 ***********************************/
/* 滤波器系数(init 时按采样率算好) */
static float s_fs_hz = 0.0f;
static float s_k_dc  = 0.0f;   /* 直流跟踪器 IIR 系数 */
static float s_k_ac  = 0.0f;   /* 交流低通 IIR 系数 */

/* 两路滤波器状态 */
static float s_dc_red = 0.0f;  /* RED 直流分量估计(原始计数) */
static float s_ac_red = 0.0f;  /* RED 带通后交流分量 */
static float s_dc_ir  = 0.0f;  /* IR  直流分量估计 */
static float s_ac_ir  = 0.0f;  /* IR  带通后交流分量 */

/* 当前窗口的峰峰值统计 */
static float   s_min_red = 0.0f;
static float   s_max_red = 0.0f;
static float   s_min_ir  = 0.0f;
static float   s_max_ir  = 0.0f;
static uint8_t s_win_filled = 0;   /* 本窗口是否已有样本(决定 min/max 是赋初值还是比较) */
static uint32_t s_win_cnt = 0;     /* 本窗口累计样本数 */
static uint32_t s_win_len = 0;     /* 窗口长度(样本数), init 时按 fs 算出 */

/* 输出 */
static float   s_spo2  = 0.0f;
static float   s_ratio = 0.0f;
static uint8_t s_valid = 0;

static uint8_t s_init_done = 0;
/********************************* 内部状态 ***********************************/

/********************************* 内部工具 ***********************************/

/******************************************************************************
 * @name    spo2_reset_window
 * @brief   重置当前计算窗口的峰峰值统计
 * @param   无
 *
 * @return  无
 *****************************************************************************/
static void spo2_reset_window(void)
{
    s_min_red = 0.0f;
    s_max_red = 0.0f;
    s_min_ir = 0.0f;
    s_max_ir = 0.0f;
    s_win_filled = 0;
    s_win_cnt = 0;
}

/******************************************************************************
 * @name    spo2_compute_window
 * @brief   窗口结束: 由两路 AC 峰峰值与 DC 算 R, 再按标定曲线换算 SpO2
 * @param   无
 *
 * @return  无
 *
 * @note    任一前置条件不满足(直流过低/交流过小/灌注指数不足)时置 valid=0,
 *          但保留上一次的 s_spo2, 便于上层区分"没算"与"算出来是 0"
 *****************************************************************************/
static void spo2_compute_window(void)
{
    float ac_red = 0.0f;
    float ac_ir  = 0.0f;
    float pi_ir  = 0.0f;
    float ratio  = 0.0f;
    float spo2   = 0.0f;

    /* 1. 前置条件: 手指贴合 + 两路直流有效 */
    if (s_dc_red < SPO2_DC_MIN || s_dc_ir < SPO2_DC_MIN)
    {
        s_valid = 0;
        return;
    }

    /* 2. 两路交流峰峰值 */
    ac_red = s_max_red - s_min_red;
    ac_ir  = s_max_ir - s_min_ir;

    if (ac_red <= 0.0f || ac_ir <= 0.0f)
    {
        s_valid = 0;
        return;
    }

    /* 3. 灌注指数把关: IR 路交流太小时 R 的除数被噪声主导 */
    pi_ir = ac_ir / s_dc_ir;
    if (pi_ir < SPO2_PI_MIN)
    {
        s_valid = 0;
        return;
    }

    /* 4. 比值-比值法: R = (ACred/DCred) / (ACir/DCir) */
    ratio = (ac_red / s_dc_red) / (ac_ir / s_dc_ir);

    /* 5. 标定曲线换算 */
#if SPO2_USE_LINEAR_CURVE
    spo2 = SPO2_LINEAR_B * ratio + SPO2_LINEAR_C;
#else
    spo2 = SPO2_CURVE_A * ratio * ratio + SPO2_CURVE_B * ratio + SPO2_CURVE_C;
#endif

    /* 6. 限幅到生理合理范围 */
    if (spo2 > SPO2_OUT_MAX)
    {
        spo2 = SPO2_OUT_MAX;
    }
    else if (spo2 < SPO2_OUT_MIN)
    {
        spo2 = SPO2_OUT_MIN;
    }

    s_ratio = ratio;
    s_spo2 = spo2;
    s_valid = 1;
}

/********************************* 对外接口 ***********************************/

/******************************************************************************
 * @name    spo2_algo_init
 * @brief   初始化血氧算法: 按采样率算好 IIR 系数与窗口长度, 清空状态
 * @param   fs_hz[in] 采样率(Hz), 须与 FIFO 有效样本率一致
 *                    (MAX30102: SR / SMP_AVE; 本工程 100Hz / 1 = 100)
 *
 * @return  0 success
 *         -1 fs_hz 非法(<=0)
 *****************************************************************************/
int8_t spo2_algo_init(float fs_hz)
{
    if (fs_hz <= 0.0f)
    {
        return -1;
    }

    s_fs_hz = fs_hz;

    /* 一阶 IIR 系数: k = 1 - e^(-2π·fc/fs) */
    s_k_dc = 1.0f - expf(-SPO2_TWO_PI * SPO2_DC_FC_HZ / fs_hz);
    s_k_ac = 1.0f - expf(-SPO2_TWO_PI * SPO2_AC_FC_HZ / fs_hz);

    /* 窗口长度: 向上取整到样本数, 并夹到上限内 */
    s_win_len = (uint32_t)(fs_hz * SPO2_WINDOW_SEC + 0.5f);
    if (s_win_len < 2U)
    {
        s_win_len = 2U;
    }
    else if (s_win_len > SPO2_MAX_WINDOW_SAMPLES)
    {
        s_win_len = SPO2_MAX_WINDOW_SAMPLES;
    }

    s_init_done = 1;

    (void)spo2_algo_reset();

    return 0;
}

/******************************************************************************
 * @name    spo2_algo_reset
 * @brief   清空全部状态(含滤波器与窗口), 采样率与系数保留
 * @param   无
 *
 * @return  0 success
 *         -1 尚未 init
 *****************************************************************************/
int8_t spo2_algo_reset(void)
{
    if (0 == s_init_done)
    {
        return -1;
    }

    s_dc_red = 0.0f;
    s_ac_red = 0.0f;
    s_dc_ir  = 0.0f;
    s_ac_ir  = 0.0f;

    s_spo2 = 0.0f;
    s_ratio = 0.0f;
    s_valid = 0;

    spo2_reset_window();

    return 0;
}

/******************************************************************************
 * @name    spo2_algo_update
 * @brief   喂入一对 RED/IR 原始样本, 推进滤波与窗口统计, 回填当前血氧结果
 * @param   red_sample[in] MAX30102 RED 通道 18bit 原始计数
 * @param   ir_sample[in]  MAX30102 IR  通道 18bit 原始计数
 * @param   p_result[out]  当前结果(每次调用都会被重写)
 *
 * @return  0 success
 *         -1 p_result null
 *         -2 尚未 init
 *
 * @note    SpO2 只在窗口边界(每 SPO2_WINDOW_SEC 秒)刷新, 期间回填上一次结果;
 *          本函数必须与采集同节奏调用, 且两路样本须来自同一次 FIFO 突发读
 *****************************************************************************/
int8_t spo2_algo_update(uint32_t red_sample, uint32_t ir_sample, spo2_result_t *p_result)
{
    float x_red = 0.0f;
    float x_ir  = 0.0f;

    if (NULL == p_result)
    {
        return -1;
    }

    if (0 == s_init_done)
    {
        return -2;
    }

    x_red = (float)red_sample;
    x_ir  = (float)ir_sample;

    /* 1. 两路各自做 0.5~5Hz 带通 */
    s_dc_red += (x_red - s_dc_red) * s_k_dc;
    s_ac_red += ((x_red - s_dc_red) - s_ac_red) * s_k_ac;

    s_dc_ir += (x_ir - s_dc_ir) * s_k_dc;
    s_ac_ir += ((x_ir - s_dc_ir) - s_ac_ir) * s_k_ac;

    /* 2. 累积本窗口两路 AC 的极值 */
    if (0 == s_win_filled)
    {
        s_min_red = s_ac_red;
        s_max_red = s_ac_red;
        s_min_ir = s_ac_ir;
        s_max_ir = s_ac_ir;
        s_win_filled = 1;
    }
    else
    {
        if (s_ac_red < s_min_red) { s_min_red = s_ac_red; }
        if (s_ac_red > s_max_red) { s_max_red = s_ac_red; }
        if (s_ac_ir < s_min_ir)   { s_min_ir = s_ac_ir; }
        if (s_ac_ir > s_max_ir)   { s_max_ir = s_ac_ir; }
    }

    s_win_cnt++;

    /* 3. 窗口结束: 算一次 R 与 SpO2, 然后开新窗口 */
    if (s_win_cnt >= s_win_len)
    {
        spo2_compute_window();
        spo2_reset_window();
    }

    p_result->spo2 = s_spo2;
    p_result->ratio = s_ratio;
    p_result->valid = s_valid;

    return 0;
}
