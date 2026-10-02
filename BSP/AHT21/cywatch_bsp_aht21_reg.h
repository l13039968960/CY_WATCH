/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_bsp_aht21_reg.h
 *
 * @author zw1194
 *
 * @brief Provide the address of AHT21's registers.
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
#ifndef __CYWATCH_BSP_AHT21_REG_H__
#define __CYWATCH_BSP_AHT21_REG_H__

/*============================ 器件地址 ============================*/
/* AHT21 的 7 位 I2C 地址, 固定不可配 */
#define AHT21_IIC_ADDR          0x38

/*============================ 寄存器 ============================*/
/* 状态寄存器(只读): bit7=忙, bit3=已校准 */
#define AHT21_STATUS_REG        0x71

#define AHT21_STATUS_BUSY       0x80    /* bit7: 1=正在测量 */
#define AHT21_STATUS_CALI       0x08    /* bit3: 1=已校准 */

/*============================ 命令 ============================*/
/* @note AHT21 的命令是"无寄存器地址"的裸帧: [DevAddr] + 命令体, 一次事务内发完.
         用 pf_writereg 发不了(它只带 1 字节数据), 需用总线原语自己拼帧 */
#define AHT21_CMD_TRIGGER       0xAC    /* 触发测量: AC 33 00 */
#define AHT21_CMD_TRIGGER_ARG1  0x33
#define AHT21_CMD_TRIGGER_ARG2  0x00

#define AHT21_CMD_INIT          0xE1    /* 初始化/自校准: E1 08 00 */
#define AHT21_CMD_INIT_ARG1     0x08
#define AHT21_CMD_INIT_ARG2     0x00

#define AHT21_CMD_SOFT_RESET    0xBA    /* 软复位(单字节) */

/*============================ 换算与时序 ============================*/
/* 温湿度原始值都是 20 位, 满量程 2^20 */
#define AHT21_RAW_FULL_SCALE    1048576.0f

#define AHT21_POWER_ON_DELAY_MS 100     /* 上电后需 >=100ms 才能通信 */
#define AHT21_CALIBRATE_DELAY_MS 40     /* 自校准耗时 */
#define AHT21_MEASURE_DELAY_MS  80      /* 一次测量耗时 */
#define AHT21_RESET_DELAY_MS    20      /* 软复位后恢复时间 */

#endif //__CYWATCH_BSP_AHT21_REG_H__
