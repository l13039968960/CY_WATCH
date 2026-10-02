/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_bsp_max30102_reg.h
 *
 * @author
 *
 * @brief Provide the address of max30102's registers.
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
#ifndef __CYWATCH_BSP_MAX30102_REG_H__
#define __CYWATCH_BSP_MAX30102_REG_H__

/*MAX30102 I2C地址*/
#define MAX30102_I2C_ADDR       0x57

/*ID*/
#define MAX30102_PART_ID        0x15

/*============================ 寄存器地址 ============================*/

/*---- 中断状态与使能 ----*/
#define MAX30102_INTR_STATUS_1      0x00    /* 中断状态1(只读) */
#define MAX30102_INTR_STATUS_2      0x01    /* 中断状态2(只读) */
#define MAX30102_INTR_ENABLE_1      0x02    /* 中断使能1 */
#define MAX30102_INTR_ENABLE_2      0x03    /* 中断使能2 */

/*---- FIFO寄存器 ----*/
#define MAX30102_FIFO_WR_PTR        0x04    /* FIFO写指针 */
#define MAX30102_OVF_COUNTER        0x05    /* FIFO溢出计数器 */
#define MAX30102_FIFO_RD_PTR        0x06    /* FIFO读指针 */
#define MAX30102_FIFO_DATA          0x07    /* FIFO数据(突发读) */

/*---- 配置寄存器 ----*/
#define MAX30102_FIFO_CONFIG        0x08    /* FIFO配置 */
#define MAX30102_MODE_CONFIG        0x09    /* 模式配置 */
#define MAX30102_SPO2_CONFIG        0x0A    /* SpO2配置 */

/*---- LED脉冲幅度 ----*/
#define MAX30102_LED1_PA            0x0C    /* LED1(RED)脉冲幅度 */
#define MAX30102_LED2_PA            0x0D    /* LED2(IR)脉冲幅度 */

/*---- 多LED模式控制 ----*/
#define MAX30102_MULTI_LED_CTRL1    0x0F    /* 多LED模式控制1 */
#define MAX30102_MULTI_LED_CTRL2    0x10    /* 多LED模式控制2 */

/*---- 温度寄存器 ----*/
#define MAX30102_DIE_TEMP_INT       0x1F    /* 温度整数(只读) */
#define MAX30102_DIE_TEMP_FRAC      0x20    /* 温度小数(只读) */
#define MAX30102_DIE_TEMP_CONFIG    0x21    /* 温度配置 */

/*---- 器件ID ----*/
#define MAX30102_REV_ID             0xFE    /* 版本ID(只读) */
#define MAX30102_PART_ID_REG        0xFF    /* 器件ID(固定0x15) */

/*============================ 位域掩码与取值 ============================*/

/*---- INTR_STATUS_1 (0x00) ----*/
#define MAX30102_INTR1_A_FULL       0x80    /* bit7: FIFO近满 */
#define MAX30102_INTR1_PPG_RDY      0x40    /* bit6: PPG数据就绪 */
#define MAX30102_INTR1_ALC_OVF      0x20    /* bit5: 环境光消除溢出 */
#define MAX30102_INTR1_PWR_RDY      0x01    /* bit0: 电源就绪 */

/*---- INTR_STATUS_2 (0x01) ----*/
#define MAX30102_INTR2_DIE_TEMP_RDY 0x04    /* bit2: 温度转换就绪 */

/*---- INTR_ENABLE_1 (0x02) ----*/
#define MAX30102_INTR1_A_FULL_EN    0x80    /* bit7: 使能近满中断 */
#define MAX30102_INTR1_PPG_RDY_EN   0x40    /* bit6: 使能PPG就绪中断 */
#define MAX30102_INTR1_ALC_OVF_EN   0x20    /* bit5: 使能环境光溢出中断 */

/*---- INTR_ENABLE_2 (0x03) ----*/
#define MAX30102_INTR2_DIE_TEMP_RDY_EN 0x04 /* bit2: 使能温度就绪中断 */

/*---- FIFO_CONFIG (0x08) ----*/
#define MAX30102_FIFO_SMP_AVE_MASK      0xE0    /* bit7-5: 采样平均 */
#define MAX30102_FIFO_SMP_AVE_1         0x00    /* 1次平均 */
#define MAX30102_FIFO_SMP_AVE_2         0x20    /* 2次平均 */
#define MAX30102_FIFO_SMP_AVE_4         0x40    /* 4次平均 */
#define MAX30102_FIFO_SMP_AVE_8         0x60    /* 8次平均 */
#define MAX30102_FIFO_SMP_AVE_16        0x80    /* 16次平均 */
#define MAX30102_FIFO_SMP_AVE_32        0xA0    /* 32次平均 */
#define MAX30102_FIFO_ROLLOVER_EN       0x10    /* bit4: FIFO满后循环覆盖 */
#define MAX30102_FIFO_A_FULL_MASK       0x0F    /* bit3-0: FIFO近满阈值 */

/*---- MODE_CONFIG (0x09) ----*/
#define MAX30102_MODE_SHDN          0x80    /* bit7: 关断模式 */
#define MAX30102_MODE_RESET         0x40    /* bit6: 软件复位 */
#define MAX30102_MODE_MASK          0x07    /* bit2-0: 工作模式 */
#define MAX30102_MODE_HR            0x02    /* 心率模式(仅IR) */
#define MAX30102_MODE_SPO2          0x03    /* SpO2模式(RED+IR) */
#define MAX30102_MODE_MULTILED      0x07    /* 多LED模式 */

/*---- SPO2_CONFIG (0x0A) ----*/
#define MAX30102_SPO2_ADC_RANGE_MASK    0x60    /* bit6-5: ADC满量程 */
#define MAX30102_SPO2_ADC_RANGE_2048    0x00    /* 2048nA */
#define MAX30102_SPO2_ADC_RANGE_4096    0x20    /* 4096nA */
#define MAX30102_SPO2_ADC_RANGE_8192    0x40    /* 8192nA */
#define MAX30102_SPO2_ADC_RANGE_16384   0x60    /* 16384nA */
#define MAX30102_SPO2_SR_MASK           0x1C    /* bit4-2: 采样率 */
#define MAX30102_SPO2_SR_50             0x00    /* 50Hz */
#define MAX30102_SPO2_SR_100            0x04    /* 100Hz */
#define MAX30102_SPO2_SR_200            0x08    /* 200Hz */
#define MAX30102_SPO2_SR_400            0x0C    /* 400Hz */
#define MAX30102_SPO2_SR_800            0x10    /* 800Hz */
#define MAX30102_SPO2_SR_1000           0x14    /* 1000Hz */
#define MAX30102_SPO2_SR_1600           0x18    /* 1600Hz */
#define MAX30102_SPO2_SR_3200           0x1C    /* 3200Hz */
#define MAX30102_SPO2_LED_PW_MASK       0x03    /* bit1-0: LED脉宽/ADC分辨率 */
#define MAX30102_SPO2_LED_PW_69US       0x00    /* 69us (15-bit) */
#define MAX30102_SPO2_LED_PW_118US      0x01    /* 118us (16-bit) */
#define MAX30102_SPO2_LED_PW_215US      0x02    /* 215us (17-bit) */
#define MAX30102_SPO2_LED_PW_411US      0x03    /* 411us (18-bit) */

/*---- MULTI_LED_MODE_CONTROL (0x0F/0x10) ----*/
#define MAX30102_SLOT2_MASK        0x70    /* bit6-4: Slot2 LED选择 */
#define MAX30102_SLOT1_MASK        0x07    /* bit2-0: Slot1 LED选择 */
#define MAX30102_SLOT4_MASK        0x70    /* bit6-4: Slot4 LED选择 */
#define MAX30102_SLOT3_MASK        0x07    /* bit2-0: Slot3 LED选择 */
#define MAX30102_SLOT_DISABLED     0x00    /* 关闭 */
#define MAX30102_SLOT_LED1         0x01    /* LED1(RED) */
#define MAX30102_SLOT_LED2         0x02    /* LED2(IR) */
#define MAX30102_SLOT_PILOT        0x03    /* Pilot模式 */

/*---- DIE_TEMP_CONFIG (0x21) ----*/
#define MAX30102_DIE_TEMP_EN       0x01    /* bit0: 启动单次温度转换 */
#define MAX30102_DIE_TEMP_FRAC_MASK 0x0F   /* 温度小数部分(4位有效) */

/*============================ FIFO数据 ============================*/
/* SpO2模式每样本6字节: RED[23:16], RED[15:8], RED[7:0], IR[23:16], IR[15:8], IR[7:0] */
#define MAX30102_FIFO_SPO2_SAMPLE_SIZE  6

#endif //__CYWATCH_BSP_MAX30102_REG_H__
