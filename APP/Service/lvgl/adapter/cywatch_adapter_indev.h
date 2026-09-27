/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_adapter_cst816t.h
 *
 * @par dependencies
 * - stdint.h
 *
 * @author	zw1194
 *
 * @brief LVGL 服务消费 CST816T 的 adapter: 总线/中断实例绑定 + 驱动能力薄转发.
 *
 * Processing flow:
 *
 * lvgl_bsp_indev_inst() 构造(RST复位 → 寄存器配置 → ChipID自检)
 *   → lvgl_bsp_indev_read_touch() 逐轮轮询读原始触摸帧
 *   → lvgl_bsp_indev_deinst() 析构.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 命名说明(对 cywatch-bsp-driver skill §11.2 的有意偏离, 非笔误):
 *       adapter 惯例导出函数是 `<consumer>_bsp_<动作>`; 本服务(Lvgl)同时消费
 *       CST816T 与 ST7789T3 两个设备, 只带消费者前缀会出现 `lvgl_bsp_inst` 撞名,
 *       故插入一段区分名写成 `lvgl_bsp_<区分名>_<动作>`, 仍保持「一个 adapter =
 *       一个设备 × 一个服务」的对应关系(grep `lvgl_bsp_` 可一次捞全本服务的
 *       所有外设入口).
 *
 *       ★2026-09-27: 区分名一律用**角色名**(与消费者侧 lv_port_* 对齐) ——
 *       触摸这一路是 `indev`(`lvgl_bsp_indev_*`, 对齐 lv_port_indev), 显示那
 *       一路是 `disp`(`lvgl_bsp_disp_*`, 对齐 lv_port_disp). 两侧口径一致;
 *       文件名仍保留设备名(cywatch_adapter_cst816t.{h,c}), 改名只落在符号上★
 *
 * @note 本层只输出**原始坐标**: 缩放/轴交换翻转属于「屏幕坐标系」知识, 归调用方
 *       (lv_port_indev.c) —— 与 skill §11.6.4「adapter 只透传, 不做量纲变换」
 *       一致。驱动的 `block` 模式参数也不外泄: 本层固定用轮询(CST816T_READ_POLL),
 *       调用方不需要知道 POLL/WAIT 的存在。
 ******************************************************************************/
#ifndef __CYWATCH_ADAPTER_INDEV_H__
#define __CYWATCH_ADAPTER_INDEV_H__

/***********************************Includes***********************************/
#include <stdint.h>
/***********************************Includes***********************************/

/**********************************Declaring***********************************/
/* 构造: 绑定 main.c 的触摸专用位带 I2C 总线 + RST(PA15)/INT(PB2), 调 cst816t_inst
   (内含 ChipID 自检); 内部已把 EXTI 回调接到驱动的 pf_interrupt_cb */
int8_t lvgl_bsp_indev_inst(void);
int8_t lvgl_bsp_indev_deinst(void);

/* 读取ChipID(自检用; 驱动返回 -1/-2 表示通信失败) */
int8_t lvgl_bsp_indev_read_id(void);

/* 轮询读一帧触摸数据(非阻塞):
 *   p_gesture_id[out] 手势编码(见驱动 cywatch_bsp_cst816t_reg.h 的 CST816T_GESTURE_*)
 *   p_finger_num[out] 触摸点数, 0 = 无手指(未按下)
 *   p_x[out]/p_y[out] 面板原始坐标(未缩放, 满量程见调用方)
 *
 * 返回 0 表示本次 I2C 读取成功(不代表有手指);
 * -1 instance null / -2 I2C 读失败 (见驱动 @return) */
int8_t lvgl_bsp_indev_read_touch(uint8_t *p_gesture_id, uint8_t *p_finger_num,
								   uint16_t *p_x, uint16_t *p_y);

/* 中断使能/失能(触摸 EXTI, PB2 下降沿) */
int8_t lvgl_bsp_indev_enable_interrupt(void);
int8_t lvgl_bsp_indev_disable_interrupt(void);

/* 深度休眠(仅外部复位可唤醒) / 复位唤醒并重新初始化 */
int8_t lvgl_bsp_indev_hibernating(void);
int8_t lvgl_bsp_indev_wakeup(void);

/**********************************Declaring***********************************/

#endif // __CYWATCH_ADAPTER_INDEV_H__
