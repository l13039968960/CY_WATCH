/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file StepDetection.h
 *
 * @par dependencies
 * - stdint.h
 * - 数学库(libm, 用到 sqrtf)
 *
 * @author  -
 *
 * @brief 计步与运动状态识别(纯算法, 只用加速度计). 输入三轴加速度(g)与步长 dt,
 *        输出累计步数、步频(步/分)与三态运动状态(静止/走路/跑步). 自身无 I/O.
 *
 * 算法(每次 update 串行完成):
 * - 合加速度模值 → 慢低通估重力基线并相减(去直流, 免疫安装朝向) → 快低通抗毛刺;
 * - 1.5 秒滑窗取 min/max 得自适应包络, 门限取窗内中值, 迟滞带宽 = 幅度 × 比例;
 * - 上升越"门限+迟滞"武装, 下降破"门限-迟滞"计一步; 间隔过短判为一次步态里的第二
 *   个加速度峰(不计), 过长按重新起步;
 * - 按平滑步频分档并带滞回(走路≥60/退回<40, 跑步≥150/退出<125), 超时无新步回静止.
 *   ★走跑只按步频区分, 不按幅度★ —— 绝对幅度门限需上板标定, 更脆.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 * @note 单实例模块(文件级静态状态), 非线程安全, 须在同一任务上下文串行调用.
 * @note 采样率假定 100Hz(见 STEP_ENV_WINDOW_SAMPLES), 节拍不同请同步改参数.
 ******************************************************************************/
#ifndef __STEP_DETECTION_H__
#define __STEP_DETECTION_H__

/***********************************Includes***********************************/
#include <stdint.h>
/***********************************Includes***********************************/

/***********************************Defines************************************/
/* ---- 参数全部集中在此, 便于上板标定 ---- */
/* 包络滑窗长度(采样点): 1.5s @100Hz. 用滑窗而非指数衰减, 是为了走/跑切换时包络能
   在 1.5s 内跟上, 不会像指数衰减那样拖 10 秒 */
#define STEP_ENV_WINDOW_SAMPLES     150

#define STEP_LP_SLOW_ALPHA          0.02f   /* 慢低通系数(估重力基线), ≈0.3Hz */
#define STEP_LP_FAST_ALPHA          0.36f   /* 快低通系数(抗毛刺), ≈7Hz */
#define STEP_HYST_RATIO             0.15f   /* 迟滞带宽 = 该比例 × 当前包络幅度 */
#define STEP_MIN_AMP_G              0.05f   /* 最小包络幅度(g), 低于此判为静止 */

#define STEP_MIN_INTERVAL_MS        280.0f  /* 一步最短间隔(≈3.6Hz), 更短判为双峰 */
#define STEP_MAX_INTERVAL_MS        900.0f  /* 一步最长间隔(≈1.1Hz), 更长按重新起步 */
#define STEP_INTERVAL_LP_ALPHA      0.30f   /* 步间隔指数平滑系数 */
#define STEP_CADENCE_TIMEOUT_MS     2000.0f /* 超此时长无新步 → 步频清零, 判静止 */

#define STEP_CAD_WALK_ENTER         60.0f   /* 步频≥此值: 静止 → 走路 */
#define STEP_CAD_IDLE_EXIT          40.0f   /* 步频<此值: 走路 → 静止 */
#define STEP_CAD_RUN_ENTER          150.0f  /* 步频≥此值: 走路 → 跑步 */
#define STEP_CAD_RUN_EXIT           125.0f  /* 步频<此值: 跑步 → 走路 */
/***********************************Defines************************************/

/**********************************Declaring***********************************/
/* 运动状态(三态) */
typedef enum
{
    STEP_STATE_IDLE = 0, /* 静止 */
    STEP_STATE_WALK,     /* 走路 */
    STEP_STATE_RUN       /* 跑步 */
} step_state_e;

/* 一次 update 的输出快照 */
typedef struct
{
    uint32_t     steps;       /* 累计步数(只增, 直到 init 清零) */
    uint16_t     cadence_spm; /* 当前步频(步/分), 静止时为 0 */
    step_state_e state;       /* 静止/走路/跑步 */
    uint8_t      new_step;    /* 1 = 本次 update 新增了一步 */
} step_result_t;

/* 清零重启: 步数、包络滑窗、状态全部复位. 不初始化直接 update 亦可(静态零值即默认). */
void step_detect_init(void);

/* 计步单步(纯计算): 喂入三轴加速度(g)与距上次调用的秒数 dt_s, 结果回填 p_out
 * (不可为空). 应在单任务上下文按固定节拍调用(如每 10ms). */
void step_detect_update(float ax, float ay, float az, float dt_s, step_result_t *p_out);
/**********************************Declaring***********************************/

#endif /* __STEP_DETECTION_H__ */
