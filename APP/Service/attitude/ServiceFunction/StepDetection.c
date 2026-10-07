/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file StepDetection.c
 *
 * @par dependencies
 * - stdint.h(经头文件)
 * - 数学库(libm, 用到 sqrtf, 链接需加 -lm 或工程勾选 Use MicroLIB)
 *
 * @author  -
 *
 * @brief 计步/运动状态识别实现: 纯算法, 只用加速度计, 与传感器驱动解耦.
 *        算法逐级说明见头文件. 本文件只用浮点, 无阻塞/无 I2C.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 * @note 单实例、非线程安全, 须在同一任务上下文串行调用.
 ******************************************************************************/

/***********************************Includes************************************/
#include <math.h>
#include "StepDetection.h"
/***********************************Includes************************************/

/***********************************Variables***********************************/
static float    s_baseline;                          /* 慢低通重力基线, g */
static float    s_fast;                              /* 快低通后的交流分量, g */
static float    s_env_buf[STEP_ENV_WINDOW_SAMPLES];  /* 包络滑窗(环形) */
static uint16_t s_env_idx;                           /* 滑窗写指针 */
static uint16_t s_env_cnt;                           /* 滑窗已填充个数(≤长度) */

static uint8_t  s_armed;        /* 1 = 已越过上门限, 等下降沿计步 */
static uint32_t s_steps;        /* 累计步数 */
static float    s_time_ms;      /* 内部时间累加器, ms(dt_s 累加得到) */
static float    s_last_step_ms; /* 上一步时刻, ms */
static uint8_t  s_has_step;     /* 1 = 间隔平滑已起算(首次/重新起步后为 0) */
static float    s_interval_lp;  /* 平滑后的步间隔, ms */
static step_state_e s_state;    /* 当前运动状态 */
/***********************************Variables***********************************/

/* 清零重启: 滤波状态/滑窗/计步/状态全部复位. 换人佩戴或重新计步时调. */
void step_detect_init(void)
{
    uint16_t i;

    s_baseline = 0.0f;
    s_fast     = 0.0f;
    for (i = 0u; i < STEP_ENV_WINDOW_SAMPLES; i++)
    {
        s_env_buf[i] = 0.0f;
    }
    s_env_idx      = 0u;
    s_env_cnt      = 0u;
    s_armed        = 0u;
    s_steps        = 0u;
    s_time_ms      = 0.0f;
    s_last_step_ms = 0.0f;
    s_has_step     = 0u;
    s_interval_lp  = 0.0f;
    s_state        = STEP_STATE_IDLE;
}

/* 计步单步: 三轴加速度 → 去直流 → 滑窗包络 → 迟滞峰值 + 频率门 → 计步; 再由平滑
 * 步频分三态并回填 p_out. 间隔过短丢弃(跑步一个步态周期有触地峰+蹬地峰, 会数成两步),
 * 过长按重新起步. */
void step_detect_update(float ax, float ay, float az, float dt_s, step_result_t *p_out)
{
    float mag, hp, amp, thr, hyst, interval, cadence, vmin, vmax, v;
    uint16_t i;

    s_time_ms += dt_s * 1000.0f;

    /* 1) 模值(与安装朝向无关) → 去重力基线 → 抗毛刺 */
    mag = sqrtf(ax * ax + ay * ay + az * az);
    s_baseline += STEP_LP_SLOW_ALPHA * (mag - s_baseline);
    hp = mag - s_baseline;
    s_fast += STEP_LP_FAST_ALPHA * (hp - s_fast);

    /* 2) 滑窗包络: 入窗后重扫 min/max, 得幅度与门限 */
    s_env_buf[s_env_idx] = s_fast;
    s_env_idx = (uint16_t)((s_env_idx + 1u) % STEP_ENV_WINDOW_SAMPLES);
    if (s_env_cnt < STEP_ENV_WINDOW_SAMPLES)
    {
        s_env_cnt++;
    }
    vmin = s_env_buf[0];
    vmax = s_env_buf[0];
    for (i = 1u; i < s_env_cnt; i++)
    {
        v = s_env_buf[i];
        if (v < vmin) { vmin = v; }
        if (v > vmax) { vmax = v; }
    }
    amp = vmax - vmin;
    thr = 0.5f * (vmax + vmin);

    /* 3) 迟滞峰值检测 + 频率门 */
    p_out->new_step = 0u;
    if (amp < STEP_MIN_AMP_G)
    {
        /* 幅度不足判静止: 解除武装, 噪声不会触发计步 */
        s_armed = 0u;
    }
    else
    {
        hyst = STEP_HYST_RATIO * amp;
        if (0u == s_armed)
        {
            if (s_fast > (thr + hyst))
            {
                s_armed = 1u;
            }
        }
        else if (s_fast < (thr - hyst))
        {
            s_armed   = 0u;
            interval  = s_time_ms - s_last_step_ms;
            if (interval < STEP_MIN_INTERVAL_MS)
            {
                /* 双峰, 丢弃: 不计步也不更新上步时刻 */
            }
            else
            {
                if (interval > STEP_MAX_INTERVAL_MS)
                {
                    /* 重新起步: 步频立刻反映"刚动起来", 下一步重新起算平滑 */
                    s_interval_lp = interval;
                    s_has_step    = 0u;
                }
                else if (0u == s_has_step)
                {
                    s_interval_lp = interval; /* 首次有效步, 起算 */
                    s_has_step    = 1u;
                }
                else
                {
                    s_interval_lp += STEP_INTERVAL_LP_ALPHA * (interval - s_interval_lp);
                }
                s_last_step_ms  = s_time_ms;
                s_steps++;
                p_out->new_step = 1u;
            }
        }
    }

    /* 4) 平滑步频 → 三态(带滞回) */
    if ((0u == s_has_step) || ((s_time_ms - s_last_step_ms) > STEP_CADENCE_TIMEOUT_MS))
    {
        cadence = 0.0f;
        s_state = STEP_STATE_IDLE;
    }
    else
    {
        cadence = 60000.0f / s_interval_lp;
        switch (s_state)
        {
        case STEP_STATE_IDLE:
            if (cadence >= STEP_CAD_WALK_ENTER) { s_state = STEP_STATE_WALK; }
            break;
        case STEP_STATE_WALK:
            if (cadence >= STEP_CAD_RUN_ENTER)  { s_state = STEP_STATE_RUN; }
            else if (cadence < STEP_CAD_IDLE_EXIT) { s_state = STEP_STATE_IDLE; }
            break;
        case STEP_STATE_RUN:
            if (cadence < STEP_CAD_RUN_EXIT) { s_state = STEP_STATE_WALK; }
            break;
        }
    }

    p_out->steps       = s_steps;
    p_out->cadence_spm = (uint16_t)cadence;
    p_out->state       = s_state;
}
