/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file HeartRate.h
 *
 * @par dependencies
 * - stdint.h
 * - math.h
 *
 * @author	zw1194
 *
 * @brief PPG 心率(heart rate)纯算法: 从单路(IR)光电容积脉搏波原始 ADC 序列
 *        估算心率, 输出单位 bpm(次/分).
 *
 * Processing flow:
 *
 * 1. heartrate_algo_init(fs_hz) 只调一次, 设定采样率并清状态;
 * 2. 每收到一个样本调一次 heartrate_algo_update(ir_sample, &result);
 * 3. result.valid == 1 时 result.bpm 才可信; result.beat 标出本样本处检测到
 *    一次心搏, 供上层做波形/心跳闪烁显示.
 *
 * 算法原理(信号链):
 *
 *   原始 ADC x
 *     ├─ 一阶 IIR 直流跟踪器(fc≈0.5Hz) ──> dc        (判手指/灌注)
 *     ├─ 一阶 IIR 慢速参考  (fc≈0.3Hz) ──> dc_ref    (只用于判"直流锁没锁上")
 *     └─ ac = x - dc                      (等效 0.5Hz 高通, 去掉直流与呼吸基线)
 *          └─ 一阶 IIR 低通(fc≈5Hz) ──> ac   (压制高频噪声, 带通 0.5~5Hz)
 *
 *   带通带宽 0.5~5Hz 对应 30~300bpm, 覆盖生理心率范围.
 *
 *   直流锁定判据: |dc - dc_ref| 超过 HEARTRATE_DC_LOCK_RATIO×dc_ref, 或 dc 低于
 *   HEARTRATE_DC_MIN(未贴手指), 即判为"直流还没跟上"——上电(直流从 0 爬升)、
 *   刚贴合、手指挪位都属于此列. 此时把 dc 吸附到当前样本并清 ac/amp, 不让这个
 *   台阶瞬态进入幅度估计器.
 *   @note 为什么必须挡住它: 0.5Hz 高通对一个台阶的过冲约是台阶高度的 31%, 而真实
 *         脉搏波只有直流值的 ~0.1%, 两者差约 300 倍. 一旦瞬态进了 s_amp, 门限
 *         (= HEARTRATE_AMP_RATIO×s_amp) 要按 HEARTRATE_PEAK_TAU_S 自由衰减
 *         ln(300)≈5.7 个时间常数(≈13s)才降到脉搏波之下, 之后还要攒
 *         HEARTRATE_BEATS_FOR_VALID 个心搏——实测"贴合后 16.5s 才出心率"即由此而来.
 *   @note 判据的基准必须是 dc_ref 而不是 dc 本身: 前者让判据的尺度自动跟着信号
 *         强弱走(稳态差 0.2%、台阶差几十个百分点), 后者在弱信号上形同虚设、在强
 *         信号上又会被真实的脉搏波反复误触发.
 *
 *   峰值检测: 自适应门限 = HEARTRATE_AMP_RATIO × 动态幅度估计 amp, 其中 amp 为
 *   ac 绝对值的衰减峰值(时间常数 HEARTRATE_PEAK_TAU_S), 随信号强弱自动升降.
 *   状态机: ac 上穿门限 → 记时刻, 进入等待回落; ac 回落过零 → 确认一次心搏.
 *   确认后算心搏间期 IBI, 经三重剔除后再入滑窗:
 *     a) 间期须落在 [IBI_MIN, IBI_MAX] (生理范围 30~200bpm);
 *     b) 间期与滑窗中位数偏差超过 HEARTRATE_IBI_TOLERANCE 判为伪迹丢弃
 *        (手指抖动/运动导致的双峰或漏峰);
 *     c) 上穿门限前的绝对不应期由 a) 的 IBI_MIN 一并覆盖.
 *   心率 = 60000 / 滑窗内 IBI 均值; 攒够 HEARTRATE_BEATS_FOR_VALID 次才置 valid.
 *
 * 单位约定:
 *   - 入参 ir_sample: MAX30102 FIFO 解析后的 18bit 原始计数(0~262143), 不是电流/电压;
 *   - 出参 bpm: 次/分; dc/amplitude: 同输入量纲的原始计数, 仅用于判断信号强弱.
 *
 * 现场标定(换手指/换结构/换 LED 电流后需复标):
 *   - HEARTRATE_DC_MIN: 手指贴合时 dc 应远高于该值, 未贴合(仅环境光)时低于它.
 *     实测方法: 贴与不贴各打印一次 result.dc, 取两者之间的对数中点;
 *   - HEARTRATE_PI_MIN: 灌注指数下限, 手指偏凉/压力过大时 AC 变小, 会因此判无效.
 *     实测方法: 打印 result.amplitude 与 result.dc, PI = 2×amplitude/dc, 门限取实测值
 *     的 1/5 左右. **注意门限若落进实测值的摆动区间, 心率会"出了又灭"**;
 *   - HEARTRATE_AMP_RATIO: 抬高则漏检(更稳), 降低则误检(更灵敏).
 *
 * 本机实测(MAX30102 LED 7.2mA / ADC 4096nA / 100Hz, 指腹轻压):
 *   - dc ≈ 26600;  amplitude ≈ 12~20 → PI ≈ 0.11%(远低于常见手指的 1~3%);
 *   - 直流锁定后 beats 稳定增长、HR 83~96bpm 可复现, 但信号余量很小.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 单实例 + 非线程安全: 状态全在文件级 static, 只能由同一个任务上下文
 *       串行调用(本工程里归心率服务任务独占), 不可多任务并发.
 *
 *****************************************************************************/
#ifndef __HEARTRATE_H__
#define __HEARTRATE_H__

#include <stdint.h>

/***********************************Defines************************************/
/* ---- 判决门限(现场标定用, 见文件头"现场标定") ---- */
#define HEARTRATE_DC_MIN        10000.0f   /* 直流下限: 低于此值判为未贴手指(18bit原始计数) */
#define HEARTRATE_PI_MIN        0.0002f    /* 灌注指数(AC峰峰/DC)下限: 低于此值信号太弱.
                                              实测本机手指出力只有 ~0.11%(弱于常见手指
                                              1~3%), 原取值 0.001 恰好卡在 s_amp 的摆动
                                              区间中央, 每次心搏的 valid 像掷硬币, 心率
                                              出了又灭. 0.02% 只滤"彻底没信号".
                                              @note 这是**比值**, 加大 LED 电流改不了它 */
#define HEARTRATE_AMP_RATIO     0.5f       /* 峰值门限 = 该比例 × 动态幅度估计 */

/* ---- 信号链参数(一般不用改) ---- */
#define HEARTRATE_DC_FC_HZ      0.5f       /* 直流跟踪器截止频率(Hz) */
#define HEARTRATE_AC_FC_HZ      5.0f       /* 交流低通截止频率(Hz) */
#define HEARTRATE_PEAK_TAU_S    2.0f       /* 动态幅度估计衰减时间常数(s) */

/* ---- 直流锁定判据(决定上电/挪指后多久才出心率, 核心参数) ---- */
#define HEARTRATE_DC_REF_FC_HZ  0.3f       /* 慢速直流参考的截止频率(Hz). 必须比
                                              HEARTRATE_DC_FC_HZ(0.5Hz)慢才体现得出
                                              台阶, 又不能太慢(锁定等待 = τ·ln(台阶/
                                              判据), 越慢等越久). 0.3Hz 约 1.7 倍分离 */
#define HEARTRATE_DC_LOCK_RATIO 0.03f      /* 锁定判据: |s_dc - s_dc_ref| 超过参考值的
                                              这个比例即判"直流还在跑"、未锁定, 此时吸附
                                              s_dc 并清 s_ac/s_amp.
                                              取值依据: 稳态下两者之差 ≈ 0.12×交流幅度,
                                              即使灌注指数 3% 也只有直流值的 0.2%; 而
                                              台阶/挪指时相差几十个百分点. 3% 留了 15 倍余量.
                                              @note 不能用 |x-s_dc| 对 dc 的固定比例: 那个
                                              尺度的基准是 dc, 比被保护的脉搏波(直流值的
                                              0.1%)大几百倍; 调小到能挡住残余又会在强信号
                                              (PI 3%, |x-s_dc| 达直流值 1.5%)上反复误触发 */

/* ---- 间期判决参数 ---- */
#define HEARTRATE_IBI_MIN_MS    300.0f     /* 最短心搏间期(200bpm) */
#define HEARTRATE_IBI_MAX_MS    2000.0f    /* 最长心搏间期(30bpm) */
#define HEARTRATE_IBI_TOLERANCE 0.30f      /* 偏离滑窗中位数超过该比例判为伪迹 */
#define HEARTRATE_IBI_RING_LEN  5          /* 间期滑窗长度 */
#define HEARTRATE_BEATS_FOR_VALID 3        /* 攒够几次间期才输出有效心率 */
/***********************************Defines************************************/

/**********************************Declaring***********************************/
/* 一次 update 的输出: 有效标志 + 心率 + 供上层观察的信号强弱 */
typedef struct heartrate_result
{
    float   bpm;       /* 心率, 次/分; valid==0 时无意义 */
    float   dc;        /* 直流分量估计(原始计数), 判手指贴合 */
    float   amplitude; /* 交流分量动态幅度估计(原始计数), 判信号强弱 */
    uint8_t beat;      /* 1 = 本次调用检测到一次心搏 */
    uint8_t valid;     /* 1 = bpm 字段有效 */
} heartrate_result_t;

/* 心率算法对外接口 */
int8_t heartrate_algo_init(float fs_hz);
int8_t heartrate_algo_reset(void);
int8_t heartrate_algo_update(uint32_t ir_sample, heartrate_result_t *p_result);

/**********************************Declaring***********************************/

#endif // __HEARTRATE_H__
