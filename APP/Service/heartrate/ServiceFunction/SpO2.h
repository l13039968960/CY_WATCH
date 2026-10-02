/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file SpO2.h
 *
 * @par dependencies
 * - stdint.h
 * - math.h
 *
 * @author	zw1194
 *
 * @brief 血氧饱和度(SpO2)纯算法: 从 RED/IR 双路光电容积脉搏波原始 ADC 序列
 *        按"比值-比值法(Ratio of Ratios)"估算 SpO2, 输出单位 %.
 *
 * Processing flow:
 *
 * 1. spo2_algo_init(fs_hz) 只调一次, 设定采样率;
 * 2. 每收到一对样本调一次 spo2_algo_update(red_sample, ir_sample, &result);
 * 3. result.valid == 1 时 result.spo2 才可信; 该结果每 SPO2_WINDOW_SEC 秒
 *    才刷新一次, 两次刷新之间回填的是上一次的结果.
 *
 * 算法原理(比值-比值法):
 *
 *   两路各自做与心率模块相同的 0.5~5Hz 带通(去直流 + 低通去噪):
 *     核心思想: 动脉搏动带来的交流分量(AC)与组织/静脉的直流分量(DC)之比,
 *     在 RED 与 IR 两个波长上不同, 这个差异只与血氧饱和度有关.
 *
 *   在 SPO2_WINDOW_SEC 秒窗口内取两路 AC 的峰峰值(ACpp)与窗口末的 DC:
 *     R = (ACpp_red / DC_red) / (ACpp_ir / DC_ir)
 *     SpO2 = A·R² + B·R + C
 *
 *   窗口取 2s 是为了至少包住一个完整心动周期(心率 ≥30bpm 时周期 ≤2s), 否则
 *   峰峰值会被截断, R 偏小、SpO2 偏高. 代价是血氧每 2s 才刷新一次.
 *
 * 单位约定:
 *   - 入参 red_sample/ir_sample: MAX30102 FIFO 解析后的 18bit 原始计数;
 *   - 出参 spo2: 百分比(%), 典型生理范围 95~100;
 *   - 出参 ratio: 无量纲 R 值, 供现场标定曲线用.
 *
 * 标定曲线(可换):
 *   默认用 Maxim AN6400 针对 MAX3010x 评估板给出的二次曲线:
 *     SpO2 = -45.06·R² + 30.354·R + 94.845
 *   若改用经典线性近似(医疗器械上更常见的粗估), 把 SPO2_USE_LINEAR_CURVE 置 1:
 *     SpO2 = 110 - 25·R
 *   注意: 两条曲线都是经验标定结果, 不构成医疗级精度; 换手指/结构/LED 电流后
 *         需用血氧仪对照重新拟合, 见文件尾"现场标定".
 *
 * 现场标定:
 *   - SPO2_DC_MIN: 与心率模块同一判据, 手指贴合时应远高于该值;
 *   - SPO2_PI_MIN: 灌注指数下限, IR 路 ACpp/DC 低于此值判为信号太弱;
 *   - 标定时把 result.ratio 打印出来, 对照参考血氧仪的读数拟合 A/B/C.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 单实例 + 非线程安全: 状态全在文件级 static, 只能由同一个任务上下文
 *       串行调用(本工程里归心率服务任务独占), 不可多任务并发.
 *
 *****************************************************************************/
#ifndef __SPO2_H__
#define __SPO2_H__

#include <stdint.h>

/***********************************Defines************************************/
/* ---- 标定曲线选择 ---- */
#define SPO2_USE_LINEAR_CURVE   0          /* 1 = 用 SpO2 = 110 - 25·R 近似 */

/* ---- 判决门限(现场标定用) ---- */
#define SPO2_DC_MIN             10000.0f   /* 直流下限: 低于此值判为未贴手指(18bit原始计数) */
#define SPO2_PI_MIN             0.001f     /* 灌注指数(ACpp/DC)下限, 低于此值结果不可信 */

/* ---- 信号链参数 ---- */
#define SPO2_DC_FC_HZ           0.5f       /* 直流跟踪器截止频率(Hz) */
#define SPO2_AC_FC_HZ           5.0f       /* 交流低通截止频率(Hz) */
#define SPO2_WINDOW_SEC         2.0f       /* 计算窗口长度(s), 须覆盖一个完整心动周期 */
#define SPO2_MAX_WINDOW_SAMPLES 2000       /* 窗口样本数上限(防采样率误设时溢出计数) */

/* ---- 输出限幅(生理合理范围) ---- */
#define SPO2_OUT_MIN            70.0f
#define SPO2_OUT_MAX            100.0f
/***********************************Defines************************************/

/**********************************Declaring***********************************/
/* 一次 update 的输出: 有效标志 + 血氧 + 中间量 R(供标定) */
typedef struct spo2_result
{
    float   spo2;   /* 血氧饱和度, %; valid==0 时无意义 */
    float   ratio;  /* 比值-比值法算出的 R, 无量纲 */
    uint8_t valid;  /* 1 = spo2 字段有效 */
} spo2_result_t;

/* 血氧算法对外接口 */
int8_t spo2_algo_init(float fs_hz);
int8_t spo2_algo_reset(void);
int8_t spo2_algo_update(uint32_t red_sample, uint32_t ir_sample, spo2_result_t *p_result);

/**********************************Declaring***********************************/

#endif // __SPO2_H__
