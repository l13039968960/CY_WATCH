/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file HeartRate.c
 *
 * @par dependencies
 * - HeartRate.h
 * - math.h
 * - stddef.h
 *
 * @author	zw1194
 *
 * @brief Implete the PPG heart rate pure algorithm.
 *
 * Processing flow:
 *
 * call heartrate_algo_init() once, then heartrate_algo_update() per sample.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
#include "HeartRate.h"

#include <math.h>
#include <stddef.h>

/***********************************Defines************************************/
#define HEARTRATE_TWO_PI        (6.28318530718f)  /* 2π */
#define HEARTRATE_MS_PER_MIN    (60000.0f)        /* 1 分钟的毫秒数 */
#define HEARTRATE_MS_PER_S      (1000.0f)         /* 1 秒的毫秒数 */
/***********************************Defines************************************/

/********************************* 内部状态 ***********************************/
/* 滤波器系数(init 时按采样率算好) */
static float s_fs_hz     = 0.0f;   /* 采样率, Hz */
static float s_k_dc      = 0.0f;   /* 直流跟踪器 IIR 系数 */
static float s_k_ac      = 0.0f;   /* 交流低通 IIR 系数 */
static float s_k_ref     = 0.0f;   /* 慢速直流参考 IIR 系数(只用于判锁定) */
static float s_amp_decay = 0.0f;   /* 动态幅度估计的每样本衰减因子 */

/* 滤波器状态 */
static float s_dc     = 0.0f;      /* 直流分量估计(原始计数) */
static float s_dc_ref = 0.0f;      /* 慢速直流参考(原始计数), 判直流锁没锁上 */
static float s_ac     = 0.0f;      /* 带通后交流分量(原始计数) */
static float s_amp    = 0.0f;      /* 交流幅度动态估计(衰减峰值) */

/* 峰值检测状态机 */
static uint8_t  s_rising = 0;        /* 0=等待上穿门限, 1=等待回落过零 */
static uint32_t s_sample_idx = 0;    /* 样本计数(换算时间用) */
static uint32_t s_beat_idx = 0;      /* 本次上穿门限的样本号 */
static uint32_t s_last_beat_idx = 0; /* 上一次心搏的样本号 */
static uint8_t  s_have_last_beat = 0;/* 是否已有可算间期的上一拍 */

/* 心搏间期滑窗(环形) */
static float   s_ibi_ring[HEARTRATE_IBI_RING_LEN];
static uint8_t s_ibi_cnt  = 0;     /* 窗内已入有效间期数 */
static uint8_t s_ibi_head = 0;     /* 环形写指针 */

/* 输出 */
static float   s_bpm   = 0.0f;
static uint8_t s_valid = 0;

static uint8_t s_init_done = 0;
/********************************* 内部状态 ***********************************/

/********************************* 内部工具 ***********************************/

/******************************************************************************
 * @name    heartrate_median
 * @brief   求数组中位数(插入排序, 长度最多 HEARTRATE_IBI_RING_LEN=5)
 * @param   p_data[in] 数据数组
 * @param   n[in]      元素个数(1~5)
 *
 * @return  中位数; n==0 时返回 0
 *****************************************************************************/
static float heartrate_median(const float *p_data, uint8_t n)
{
    float tmp[HEARTRATE_IBI_RING_LEN];
    float key;
    uint8_t i = 0;
    uint8_t j = 0;

    if (0 == n)
    {
        return 0.0f;
    }

    for (i = 0; i < n; i++)
    {
        tmp[i] = p_data[i];
    }

    for (i = 1; i < n; i++)
    {
        key = tmp[i];
        j = i;
        while (j > 0 && tmp[j - 1] > key)
        {
            tmp[j] = tmp[j - 1];
            j--;
        }
        tmp[j] = key;
    }

    if (0 != (n & 1U))
    {
        return tmp[n / 2];
    }

    return 0.5f * (tmp[n / 2 - 1] + tmp[n / 2]);
}

/******************************************************************************
 * @name    heartrate_reset_detector
 * @brief   清空峰值检测/间期状态, 保留滤波器状态(s_dc/s_ac 继续收敛)
 * @param   无
 *
 * @return  无
 *
 * @note    手指移开或信号失效时调用: 下一次贴合需重新攒够 HEARTRATE_BEATS_FOR_VALID
 *          次心搏才会再输出有效心率, 避免把断档前后的间期拼成一个假心率
 *
 * @note    本函数**不清 s_amp**(幅度估计): s_amp 的清除由调用方负责——
 *          heartrate_algo_update 第 4 步判为"直流未锁定/手指未贴合"时会把
 *          s_amp 一并清 0. 这样估计器总是从当前真实信号电平重新起步, 不会把
 *          直流台阶瞬态(上万个计数)当成交流峰值、把门限顶高十几秒
 *****************************************************************************/
static void heartrate_reset_detector(void)
{
    uint8_t i = 0;

    s_rising = 0;
    s_have_last_beat = 0;
    s_beat_idx = 0;
    s_last_beat_idx = 0;
    s_ibi_cnt = 0;
    s_ibi_head = 0;
    s_bpm = 0.0f;
    s_valid = 0;

    for (i = 0; i < HEARTRATE_IBI_RING_LEN; i++)
    {
        s_ibi_ring[i] = 0.0f;
    }
}

/******************************************************************************
 * @name    heartrate_accept_ibi
 * @brief   心搏间期入窗判决: 生理范围 + 滑窗中位数一致性双重剔除
 * @param   ibi_ms[in] 本次心搏间期(ms)
 *
 * @return  无
 *****************************************************************************/
static void heartrate_accept_ibi(float ibi_ms)
{
    float median = 0.0f;

    /* 1. 生理范围(30~200bpm): 同时覆盖了最短不应期 */
    if (ibi_ms < HEARTRATE_IBI_MIN_MS || ibi_ms > HEARTRATE_IBI_MAX_MS)
    {
        return;
    }

    /* 2. 滑窗已满时, 偏离中位数过大的判为伪迹(抖动双峰/漏峰) */
    if (s_ibi_cnt >= HEARTRATE_IBI_RING_LEN)
    {
        median = heartrate_median(s_ibi_ring, s_ibi_cnt);
        if (fabsf(ibi_ms - median) > HEARTRATE_IBI_TOLERANCE * median)
        {
            return;
        }
    }

    s_ibi_ring[s_ibi_head] = ibi_ms;
    s_ibi_head = (uint8_t)((s_ibi_head + 1U) % HEARTRATE_IBI_RING_LEN);
    if (s_ibi_cnt < HEARTRATE_IBI_RING_LEN)
    {
        s_ibi_cnt++;
    }
}

/******************************************************************************
 * @name    heartrate_update_bpm
 * @brief   由滑窗内间期均值更新心率, 并做灌注指数与均值范围把关
 * @param   无
 *
 * @return  无
 *****************************************************************************/
static void heartrate_update_bpm(void)
{
    float sum = 0.0f;
    float mean = 0.0f;
    float pi = 0.0f;
    uint8_t i = 0;

    if (s_ibi_cnt < HEARTRATE_BEATS_FOR_VALID)
    {
        s_valid = 0;
        s_bpm = 0.0f;
        return;
    }

    /* 灌注指数 = AC 峰峰 / DC: 手指偏凉/压力过大时太小, 此时间期不可信 */
    pi = (2.0f * s_amp) / s_dc;
    if (pi < HEARTRATE_PI_MIN)
    {
        s_valid = 0;
        s_bpm = 0.0f;
        return;
    }

    for (i = 0; i < s_ibi_cnt; i++)
    {
        sum += s_ibi_ring[i];
    }
    mean = sum / (float)s_ibi_cnt;

    if (mean < HEARTRATE_IBI_MIN_MS || mean > HEARTRATE_IBI_MAX_MS)
    {
        s_valid = 0;
        s_bpm = 0.0f;
        return;
    }

    s_bpm = HEARTRATE_MS_PER_MIN / mean;
    s_valid = 1;
}

/********************************* 对外接口 ***********************************/

/******************************************************************************
 * @name    heartrate_algo_init
 * @brief   初始化心率算法: 按采样率算好各级 IIR 系数并清空状态
 * @param   fs_hz[in] 采样率(Hz), 须与 FIFO 有效样本率一致
 *                    (MAX30102: SR / SMP_AVE; 本工程 100Hz / 1 = 100)
 *
 * @return  0 success
 *         -1 fs_hz 非法(<=0)
 *****************************************************************************/
int8_t heartrate_algo_init(float fs_hz)
{
    if (fs_hz <= 0.0f)
    {
        return -1;
    }

    s_fs_hz = fs_hz;

    /* 一阶 IIR 系数: k = 1 - e^(-2π·fc/fs) */
    s_k_dc  = 1.0f - expf(-HEARTRATE_TWO_PI * HEARTRATE_DC_FC_HZ / fs_hz);
    s_k_ac  = 1.0f - expf(-HEARTRATE_TWO_PI * HEARTRATE_AC_FC_HZ / fs_hz);
    s_k_ref = 1.0f - expf(-HEARTRATE_TWO_PI * HEARTRATE_DC_REF_FC_HZ / fs_hz);

    /* 衰减峰值: 时间常数 TAU, 每样本乘 e^(-1/(fs·TAU)) */
    s_amp_decay = expf(-1.0f / (fs_hz * HEARTRATE_PEAK_TAU_S));

    s_init_done = 1;

    (void)heartrate_algo_reset();

    return 0;
}

/******************************************************************************
 * @name    heartrate_algo_reset
 * @brief   清空全部状态(含滤波器), 采样率与系数保留
 * @param   无
 *
 * @return  0 success
 *         -1 尚未 init
 *****************************************************************************/
int8_t heartrate_algo_reset(void)
{
    if (0 == s_init_done)
    {
        return -1;
    }

    s_dc     = 0.0f;
    s_dc_ref = 0.0f;
    s_ac     = 0.0f;
    s_sample_idx = 0;

    heartrate_reset_detector();

    return 0;
}

/******************************************************************************
 * @name    heartrate_algo_update
 * @brief   喂入一个 IR 原始样本, 推进滤波与峰值检测, 回填本样本的判决结果
 * @param   ir_sample[in]  MAX30102 IR 通道 18bit 原始计数
 * @param   p_result[out]  本样本结果(每次调用都会被重写)
 *
 * @return  0 success
 *         -1 p_result null
 *         -2 尚未 init
 *
 * @note    本函数必须与采集同节奏调用(每个 FIFO 样本一次), 不能跳样本,
 *          否则间期按样本号换算出的时间会整体失真
 *****************************************************************************/
int8_t heartrate_algo_update(uint32_t ir_sample, heartrate_result_t *p_result)
{
    float x = 0.0f;
    float a = 0.0f;
    float threshold = 0.0f;
    float ibi_ms = 0.0f;
    uint8_t beat = 0;

    if (NULL == p_result)
    {
        return -1;
    }

    if (0 == s_init_done)
    {
        return -2;
    }

    x = (float)ir_sample;

    /* 1. 直流跟踪 + 去直流(等效 0.5Hz 高通) */
    s_dc += (x - s_dc) * s_k_dc;

    /* 2. 交流低通(fc≈5Hz): 与上一步合起来构成 0.5~5Hz 带通 */
    s_ac += ((x - s_dc) - s_ac) * s_k_ac;

    /* 3. 慢速直流参考: 只用来判"直流跟踪器锁没锁上", 不参与去直流.
       比跟踪器慢一档(0.3Hz vs 0.5Hz)才反映得出台阶, 但稳态下仍紧紧跟着脉搏基线 */
    s_dc_ref += (x - s_dc_ref) * s_k_ref;

    s_sample_idx++;

    /* 4. 直流尚未锁定 / 手指未贴合: 把滤波器吸附到当前电平, 清空检测与幅度估计后返回.
       @note 判据不能只看电平(s_dc < HEARTRATE_DC_MIN). 手指靠近是"斜坡"不是"台阶",
             电平守卫在 dc 爬过 HEARTRATE_DC_MIN 的那一刻就撒手了, 此后 x - s_dc 是上万
             的台阶差, 被 5Hz 低通吃成 s_ac, s_amp 随之冲到近万; 真实脉搏波只有十几计数,
             门限(0.5×s_amp)要等 HEARTRATE_PEAK_TAU_S 衰减 ln(瞬态/脉搏)≈6.5 倍时间常数
             (≈13s)才降到脉搏波之下 —— 这就是"贴合后十几秒才出心率"的全部原因.
       @note 判据也不能用 |x - s_dc| 对 dc 的固定比例(试过 0.15, 只治一半, 残余仍能把
             s_amp 灌到几百): 那个尺度的基准是 dc, 而真实灌注指数只有 dc 的 0.1%,
             判据比被保护的信号大几百倍; 调小到能挡住残余, 又会在强信号(PI 3%,
             |x-s_dc| 达 dc 的 1.5%)上反复误触发、把 s_ac 清掉, 心率永远出不来.
             换成"跟踪器 vs 慢速参考"后, 判据的尺度自动跟着信号走, 强弱通吃 */
    if (s_dc < HEARTRATE_DC_MIN ||
        fabsf(s_dc - s_dc_ref) > HEARTRATE_DC_LOCK_RATIO * s_dc_ref)
    {
        s_dc = x;          /* 吸附: 立刻满足"跟踪器跟得上参考"这一半 */
        s_ac = 0.0f;
        s_amp = 0.0f;

        heartrate_reset_detector();

        p_result->bpm = 0.0f;
        p_result->dc = s_dc;
        p_result->amplitude = s_amp;
        p_result->beat = 0;
        p_result->valid = 0;

        return 0;
    }

    /* 5. 动态幅度估计: 衰减峰值, 随信号强弱自动升降 */
    a = fabsf(s_ac);
    if (a > s_amp)
    {
        s_amp = a;
    }
    else
    {
        s_amp *= s_amp_decay;
    }

    threshold = HEARTRATE_AMP_RATIO * s_amp;

    /* 6. 峰值判决状态机: 上穿门限记时刻 → 回落过零确认心搏 */
    if (0 == s_rising)
    {
        if (s_ac > threshold)
        {
            s_rising = 1;
            s_beat_idx = s_sample_idx;
        }
    }
    else if (s_ac < 0.0f)
    {
        beat = 1;

        if (0 != s_have_last_beat)
        {
            ibi_ms = (float)(s_beat_idx - s_last_beat_idx) * HEARTRATE_MS_PER_S / s_fs_hz;
            heartrate_accept_ibi(ibi_ms);
        }

        s_last_beat_idx = s_beat_idx;
        s_have_last_beat = 1;
        s_rising = 0;
    }

    /* 7. 有新心搏才重算心率 */
    if (0 != beat)
    {
        heartrate_update_bpm();
    }

    p_result->bpm = s_bpm;
    p_result->dc = s_dc;
    p_result->amplitude = s_amp;
    p_result->beat = beat;
    p_result->valid = s_valid;

    return 0;
}

