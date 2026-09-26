/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_bsp_cst816t_reg.h
 *
 * @author
 *
 * @brief Provide the address of cst816t's registers.
 *
 * Processing flow:
 *
 * call directly.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
#ifndef __CYWATCH_BSP_CST816T_REG_H__
#define __CYWATCH_BSP_CST816T_REG_H__

/*CST816T I2C地址*/
#define CST816T_I2C_ADDR        0x15

/*============================ 寄存器地址 ============================*/

/*---- 触摸数据寄存器 (0x01~0x06, 只读) ----*/
#define CST816T_GESTURE_ID      0x01    /* 手势识别结果 */
#define CST816T_FINGER_NUM      0x02    /* 触摸手指数量 */
#define CST816T_XPOS_H          0x03    /* X坐标高4位 */
#define CST816T_XPOS_L          0x04    /* X坐标低8位 */
#define CST816T_YPOS_H          0x05    /* Y坐标高4位 */
#define CST816T_YPOS_L          0x06    /* Y坐标低8位 */

/*---- 芯片信息寄存器 (0xA7~0xAA, 只读) ----*/
#define CST816T_CHIP_ID         0xA7    /* 芯片型号ID */
#define CST816T_PROJ_ID         0xA8    /* 工程编号 */
#define CST816T_FW_VERSION      0xA9    /* 固件版本 */
#define CST816T_FACTORY_ID      0xAA    /* 触控屏厂商ID */

/*---- 系统控制寄存器 (0xE5~0xEF) ----*/
#define CST816T_SLEEP_MODE          0xE5    /* 休眠模式控制 */
#define CST816T_ERR_RESET_CTL       0xEA    /* 异常复位控制 */
#define CST816T_LONG_PRESS_TICK     0xEB    /* 长按时间门限 */
#define CST816T_MOTION_MASK         0xEC    /* 手势使能掩码 */
#define CST816T_IRQ_PULSE_WIDTH     0xED    /* 中断脉冲宽度 */
#define CST816T_NOR_SCAN_PER        0xEE    /* 正常扫描周期 */
#define CST816T_MOTION_SL_ANGLE     0xEF    /* 手势滑动角度控制 */

/*---- 低功耗配置寄存器 (0xF0~0xF9) ----*/
#define CST816T_LP_SCAN_RAW1_H      0xF0    /* 低功耗扫描1通道基准值高8位 */
#define CST816T_LP_SCAN_RAW1_L      0xF1    /* 低功耗扫描1通道基准值低8位 */
#define CST816T_LP_SCAN_RAW2_H      0xF2    /* 低功耗扫描2通道基准值高8位 */
#define CST816T_LP_SCAN_RAW2_L      0xF3    /* 低功耗扫描2通道基准值低8位 */
#define CST816T_LP_AUTO_WAKE_TIME   0xF4    /* 低功耗自动重校正周期 */
#define CST816T_LP_SCAN_TH          0xF5    /* 低功耗扫描唤醒门限 */
#define CST816T_LP_SCAN_WIN         0xF6    /* 低功耗扫描量程 */
#define CST816T_LP_SCAN_FREQ        0xF7    /* 低功耗扫描频率 */
#define CST816T_LP_SCAN_IDAC        0xF8    /* 低功耗扫描电流 */
#define CST816T_AUTO_SLEEP_TIME     0xF9    /* 自动休眠时间 */

/*---- 中断与IO控制寄存器 (0xFA~0xFE) ----*/
#define CST816T_IRQ_CTL             0xFA    /* 中断控制 */
#define CST816T_DEBOUNCE_TIME       0xFB    /* 无有效手势自动复位时间 */
#define CST816T_LONG_PRESS_TIME     0xFC    /* 长按自动复位时间 */
#define CST816T_IO_CTL              0xFD    /* IO控制 */
#define CST816T_DIS_AUTO_SLEEP      0xFE    /* 自动休眠禁用 */

/*============================ 位域掩码与取值 ============================*/

/*---- GESTURE_ID (0x01) 手势编码 ----*/
#define CST816T_GESTURE_NONE        0x00    /* 无手势 */
#define CST816T_GESTURE_SLIDE_UP    0x01    /* 上滑 */
#define CST816T_GESTURE_SLIDE_DOWN  0x02    /* 下滑 */
#define CST816T_GESTURE_SLIDE_LEFT  0x03    /* 左滑 */
#define CST816T_GESTURE_SLIDE_RIGHT 0x04    /* 右滑 */
#define CST816T_GESTURE_SINGLE_CLICK 0x05   /* 单击 */
#define CST816T_GESTURE_DOUBLE_CLICK 0x0B   /* 双击 */
#define CST816T_GESTURE_LONG_PRESS  0x0C    /* 长按 */

/*---- FINGER_NUM (0x02) ----*/
#define CST816T_FINGER_NUM_MASK     0x0F    /* bit3-0: 触摸点数 */

/*---- SLEEP_MODE (0xE5) ----*/
#define CST816T_SLEEP_MODE_DEEP_SLEEP 0x03  /* 深度休眠(仅外部复位可唤醒) */

/*---- MOTION_MASK (0xEC) ----*/
#define CST816T_MOTION_EN_D_CLICK   0x01    /* bit0: 双击手势 */
#define CST816T_MOTION_EN_CON_UD    0x02    /* bit1: 连续上下滑动 */
#define CST816T_MOTION_EN_CON_LR    0x04    /* bit2: 连续左右滑动 */

/*---- IRQ_CTL (0xFA) ----*/
#define CST816T_IRQ_EN_TEST         0x80    /* bit7: 中断测试模式 */
#define CST816T_IRQ_EN_TOUCH        0x20    /* bit5: 触摸中断 */
#define CST816T_IRQ_EN_CHANGE       0x10    /* bit4: 状态变化中断 */
#define CST816T_IRQ_EN_MOTION       0x08    /* bit3: 手势中断 */
#define CST816T_IRQ_ONCE_WLP        0x01    /* bit0: 长按仅单脉冲 */

/*---- IO_CTL (0xFD) ----*/
#define CST816T_IO_SOFT_RST         0x04    /* bit2: IRQ软复位 */
#define CST816T_IO_IIC_OD           0x02    /* bit1: I2C开漏模式 */
#define CST816T_IO_EN_1V8           0x01    /* bit0: 1.8V电平 */

/*============================ 触摸数据 ============================*/
#define CST816T_TOUCH_DATA_SIZE     6       /* 触摸数据帧字节数(0x01~0x06) */
#define CST816T_COORD_H_MASK        0x0F    /* 坐标高4位掩码(XposH/YposH bit3-0有效) */

#endif //__CYWATCH_BSP_CST816T_REG_H__
