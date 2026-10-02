/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_bsp_mpu60050_reg.h
 *
 * @author
 *
 * @brief Provide the address of mpu60050's registers.
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
#ifndef __CYWATCH_BSP_MPU60050_REG_H__
#define __CYWATCH_BSP_MPU60050_REG_H__

/*MPU6050 I2C地址*/
#define MPU6050_I2C_ADDR        0x68

/*ID*/
#define MPU6050_ID              0x68

/*============================ 寄存器地址 ============================*/

/*---- 基础配置寄存器 ----*/
#define MPU6050_AUX_VDDIO       0x01    /* 辅助I2C IO电平选择 */
#define MPU6050_SMPLRT_DIV      0x19    /* 采样率分频 */
#define MPU6050_CONFIG          0x1A    /* DLPF与FSYNC配置 */
#define MPU6050_GYRO_CONFIG     0x1B    /* 陀螺仪量程与自检 */
#define MPU6050_ACCEL_CONFIG    0x1C    /* 加速度量程与自检 */

/*---- 运动检测寄存器 ----*/
#define MPU6050_FF_THR          0x1D    /* 自由落体阈值 */
#define MPU6050_FF_DUR          0x1E    /* 自由落体持续时间 */
#define MPU6050_MOT_THR         0x1F    /* 运动检测阈值 */
#define MPU6050_MOT_DUR         0x20    /* 运动持续时长 */
#define MPU6050_ZRMOT_THR       0x21    /* 静止检测阈值 */
#define MPU6050_ZRMOT_DUR       0x22    /* 静止持续时长 */

/*---- FIFO配置 ----*/
#define MPU6050_FIFO_EN         0x23    /* FIFO数据源使能 */

/*---- 辅助I2C主机控制 ----*/
#define MPU6050_I2C_MST_CTRL    0x24    /* I2C主机模式控制 */
#define MPU6050_I2C_SLV0_ADDR   0x25    /* Slave0 I2C地址 */
#define MPU6050_I2C_SLV0_REG    0x26    /* Slave0寄存器地址 */
#define MPU6050_I2C_SLV0_CTRL   0x27    /* Slave0传输控制 */
#define MPU6050_I2C_SLV1_ADDR   0x28    /* Slave1 I2C地址 */
#define MPU6050_I2C_SLV1_REG    0x29    /* Slave1寄存器地址 */
#define MPU6050_I2C_SLV1_CTRL   0x2A    /* Slave1传输控制 */
#define MPU6050_I2C_SLV2_ADDR   0x2B    /* Slave2 I2C地址 */
#define MPU6050_I2C_SLV2_REG    0x2C    /* Slave2寄存器地址 */
#define MPU6050_I2C_SLV2_CTRL   0x2D    /* Slave2传输控制 */
#define MPU6050_I2C_SLV3_ADDR   0x2E    /* Slave3 I2C地址 */
#define MPU6050_I2C_SLV3_REG    0x2F    /* Slave3寄存器地址 */
#define MPU6050_I2C_SLV3_CTRL   0x30    /* Slave3传输控制 */
#define MPU6050_I2C_SLV4_ADDR   0x31    /* Slave4 I2C地址 */
#define MPU6050_I2C_SLV4_REG    0x32    /* Slave4寄存器地址 */
#define MPU6050_I2C_SLV4_DO     0x33    /* Slave4写输出数据 */
#define MPU6050_I2C_SLV4_CTRL   0x34    /* Slave4传输控制 */
#define MPU6050_I2C_SLV4_DI     0x35    /* Slave4读输入数据 */

/*---- I2C主机状态 ----*/
#define MPU6050_I2C_MST_STATUS  0x36    /* I2C主机状态(只读) */

/*---- 中断配置与状态 ----*/
#define MPU6050_INT_PIN_CFG     0x37    /* INT引脚与I2C旁路配置 */
#define MPU6050_INT_ENABLE      0x38    /* 中断源总使能 */
#define MPU6050_INT_STATUS      0x3A    /* 中断状态(只读) */

/*---- 加速度原始数据 (只读, 大端16位) ----*/
#define MPU6050_ACCEL_XOUT_H    0x3B    /* X轴加速度高8位 */
#define MPU6050_ACCEL_XOUT_L    0x3C    /* X轴加速度低8位 */
#define MPU6050_ACCEL_YOUT_H    0x3D    /* Y轴加速度高8位 */
#define MPU6050_ACCEL_YOUT_L    0x3E    /* Y轴加速度低8位 */
#define MPU6050_ACCEL_ZOUT_H    0x3F    /* Z轴加速度高8位 */
#define MPU6050_ACCEL_ZOUT_L    0x40    /* Z轴加速度低8位 */

/*---- 温度原始数据 (只读, 大端16位) ----*/
#define MPU6050_TEMP_OUT_H      0x41    /* 温度高8位 */
#define MPU6050_TEMP_OUT_L      0x42    /* 温度低8位 */

/*---- 陀螺仪原始数据 (只读, 大端16位) ----*/
#define MPU6050_GYRO_XOUT_H     0x43    /* X轴陀螺仪高8位 */
#define MPU6050_GYRO_XOUT_L     0x44    /* X轴陀螺仪低8位 */
#define MPU6050_GYRO_YOUT_H     0x45    /* Y轴陀螺仪高8位 */
#define MPU6050_GYRO_YOUT_L     0x46    /* Y轴陀螺仪低8位 */
#define MPU6050_GYRO_ZOUT_H     0x47    /* Z轴陀螺仪高8位 */
#define MPU6050_GYRO_ZOUT_L     0x48    /* Z轴陀螺仪低8位 */

/*---- 外部传感器数据缓存 (只读) ----*/
#define MPU6050_EXT_SENS_DATA_00    0x49
#define MPU6050_EXT_SENS_DATA_01    0x4A
#define MPU6050_EXT_SENS_DATA_02    0x4B
#define MPU6050_EXT_SENS_DATA_03    0x4C
#define MPU6050_EXT_SENS_DATA_04    0x4D
#define MPU6050_EXT_SENS_DATA_05    0x4E
#define MPU6050_EXT_SENS_DATA_06    0x4F
#define MPU6050_EXT_SENS_DATA_07    0x50
#define MPU6050_EXT_SENS_DATA_08    0x51
#define MPU6050_EXT_SENS_DATA_09    0x52
#define MPU6050_EXT_SENS_DATA_10    0x53
#define MPU6050_EXT_SENS_DATA_11    0x54
#define MPU6050_EXT_SENS_DATA_12    0x55
#define MPU6050_EXT_SENS_DATA_13    0x56
#define MPU6050_EXT_SENS_DATA_14    0x57
#define MPU6050_EXT_SENS_DATA_15    0x58
#define MPU6050_EXT_SENS_DATA_16    0x59
#define MPU6050_EXT_SENS_DATA_17    0x5A
#define MPU6050_EXT_SENS_DATA_18    0x5B
#define MPU6050_EXT_SENS_DATA_19    0x5C
#define MPU6050_EXT_SENS_DATA_20    0x5D
#define MPU6050_EXT_SENS_DATA_21    0x5E
#define MPU6050_EXT_SENS_DATA_22    0x5F
#define MPU6050_EXT_SENS_DATA_23    0x60

/*---- 运动检测状态 ----*/
#define MPU6050_MOT_DETECT_STATUS   0x61    /* 运动/静止状态(只读) */

/*---- Slave写输出缓存 ----*/
#define MPU6050_I2C_SLV0_DO     0x63    /* Slave0写输出数据 */
#define MPU6050_I2C_SLV1_DO     0x64    /* Slave1写输出数据 */
#define MPU6050_I2C_SLV2_DO     0x65    /* Slave2写输出数据 */
#define MPU6050_I2C_SLV3_DO     0x66    /* Slave3写输出数据 */

/*---- I2C从机访问分频 ----*/
#define MPU6050_I2C_MST_DELAY_CTRL  0x67    /* 从机访问分频控制 */

/*---- 信号通路与检测控制 ----*/
#define MPU6050_SIGNAL_PATH_RESET   0x68    /* 信号通路软复位(只写) */
#define MPU6050_MOT_DETECT_CTRL     0x69    /* 运动检测计数器配置 */

/*---- 用户控制与电源管理 ----*/
#define MPU6050_USER_CTRL       0x6A    /* FIFO/I2C/复位控制 */
#define MPU6050_PWR_MGMT_1      0x6B    /* 电源管理1: 复位/睡眠/时钟 */
#define MPU6050_PWR_MGMT_2      0x6C    /* 电源管理2: 唤醒频率/各轴待机 */

/*---- FIFO计数与数据 ----*/
#define MPU6050_FIFO_COUNT_H    0x72    /* FIFO剩余字节数高8位 */
#define MPU6050_FIFO_COUNT_L    0x73    /* FIFO剩余字节数低8位 */
#define MPU6050_FIFO_R_W        0x74    /* FIFO读写端口 */

/*---- 设备ID ----*/
#define MPU6050_WHO_AM_I        0x75    /* 设备识别(固定0x68) */

/*============================ 位域掩码与偏移 ============================*/

/*---- PWR_MGMT_1 (0x6B) ----*/
#define MPU6050_PWR1_DEVICE_RESET   0x80    /* bit7: 整机软复位, 硬件自动清零 */
#define MPU6050_PWR1_SLEEP          0x40    /* bit6: 睡眠模式 */
#define MPU6050_PWR1_CYCLE          0x20    /* bit5: 低功耗循环采样 */
#define MPU6050_PWR1_TEMP_DIS       0x08    /* bit3: 温度传感器关闭 */
#define MPU6050_PWR1_CLKSEL_MASK    0x07    /* bit2-0: 时钟源选择 */
#define MPU6050_PWR1_CLKSEL_INTERNAL    0x00    /* 内部8MHz */
#define MPU6050_PWR1_CLKSEL_PLL_X       0x01    /* PLL X轴陀螺仪参考 */
#define MPU6050_PWR1_CLKSEL_PLL_Y       0x02    /* PLL Y轴陀螺仪参考 */
#define MPU6050_PWR1_CLKSEL_PLL_Z       0x03    /* PLL Z轴陀螺仪参考 */
#define MPU6050_PWR1_CLKSEL_EXT_32K     0x04    /* 外部32.768kHz */
#define MPU6050_PWR1_CLKSEL_EXT_19M     0x05    /* 外部19.2MHz */
#define MPU6050_PWR1_CLKSEL_STOP        0x07    /* 停止时钟 */

/*---- PWR_MGMT_2 (0x6C) ----*/
#define MPU6050_PWR2_LP_WAKE_CTRL_MASK  0xC0    /* bit7-6: 低功耗唤醒频率 */
#define MPU6050_PWR2_LP_WAKE_1_25HZ     0x00    /* 1.25Hz */
#define MPU6050_PWR2_LP_WAKE_2_5HZ      0x40    /* 2.5Hz */
#define MPU6050_PWR2_LP_WAKE_5HZ        0x80    /* 5Hz */
#define MPU6050_PWR2_LP_WAKE_10HZ       0xC0    /* 10Hz */
#define MPU6050_PWR2_STBY_XA            0x20    /* bit5: X加速度待机 */
#define MPU6050_PWR2_STBY_YA            0x10    /* bit4: Y加速度待机 */
#define MPU6050_PWR2_STBY_ZA            0x08    /* bit3: Z加速度待机 */
#define MPU6050_PWR2_STBY_XG            0x04    /* bit2: X陀螺仪待机 */
#define MPU6050_PWR2_STBY_YG            0x02    /* bit1: Y陀螺仪待机 */
#define MPU6050_PWR2_STBY_ZG            0x01    /* bit0: Z陀螺仪待机 */

/*---- CONFIG (0x1A) ----*/
#define MPU6050_CONFIG_EXT_SYNC_MASK    0x38    /* bit5-3: FSYNC同步配置 */
#define MPU6050_CONFIG_EXT_SYNC_DIS     0x00    /* 禁用FSYNC */
#define MPU6050_CONFIG_DLPF_MASK        0x07    /* bit2-0: DLPF配置 */
#define MPU6050_CONFIG_DLPF_260HZ       0x00    /* Accel:260Hz Gyro:256Hz(8kHz) */
#define MPU6050_CONFIG_DLPF_184HZ       0x01    /* Accel:184Hz Gyro:188Hz(1kHz) */
#define MPU6050_CONFIG_DLPF_94HZ        0x02    /* Accel:94Hz  Gyro:98Hz(1kHz)  */
#define MPU6050_CONFIG_DLPF_44HZ        0x03    /* Accel:44Hz  Gyro:42Hz(1kHz)  */
#define MPU6050_CONFIG_DLPF_21HZ        0x04    /* Accel:21Hz  Gyro:20Hz(1kHz)  */
#define MPU6050_CONFIG_DLPF_10HZ        0x05    /* Accel:10Hz  Gyro:10Hz(1kHz)  */
#define MPU6050_CONFIG_DLPF_5HZ         0x06    /* Accel:5Hz   Gyro:5Hz(1kHz)   */

/*---- GYRO_CONFIG (0x1B) ----*/
#define MPU6050_GCONFIG_XG_ST           0x80    /* bit7: X轴陀螺仪自检 */
#define MPU6050_GCONFIG_YG_ST           0x40    /* bit6: Y轴陀螺仪自检 */
#define MPU6050_GCONFIG_ZG_ST           0x20    /* bit5: Z轴陀螺仪自检 */
#define MPU6050_GCONFIG_FS_SEL_MASK     0x18    /* bit4-3: 量程选择 */
#define MPU6050_GCONFIG_FS_SEL_250      0x00    /* ±250°/s  */
#define MPU6050_GCONFIG_FS_SEL_500      0x08    /* ±500°/s  */
#define MPU6050_GCONFIG_FS_SEL_1000     0x10    /* ±1000°/s */
#define MPU6050_GCONFIG_FS_SEL_2000     0x18    /* ±2000°/s */

/*---- ACCEL_CONFIG (0x1C) ----*/
#define MPU6050_ACONFIG_XA_ST           0x80    /* bit7: X加速度自检 */
#define MPU6050_ACONFIG_YA_ST           0x40    /* bit6: Y加速度自检 */
#define MPU6050_ACONFIG_ZA_ST           0x20    /* bit5: Z加速度自检 */
#define MPU6050_ACONFIG_AFS_SEL_MASK    0x18    /* bit4-3: 量程选择 */
#define MPU6050_ACONFIG_AFS_SEL_2G      0x00    /* ±2g  */
#define MPU6050_ACONFIG_AFS_SEL_4G      0x08    /* ±4g  */
#define MPU6050_ACONFIG_AFS_SEL_8G      0x10    /* ±8g  */
#define MPU6050_ACONFIG_AFS_SEL_16G     0x18    /* ±16g */
#define MPU6050_ACONFIG_HPF_MASK        0x07    /* bit2-0: 高通滤波配置 */

/*---- USER_CTRL (0x6A) ----*/
#define MPU6050_USER_FIFO_EN            0x40    /* bit6: FIFO总使能 */
#define MPU6050_USER_I2C_MST_EN         0x20    /* bit5: 辅助I2C主机使能 */
#define MPU6050_USER_I2C_IF_DIS         0x10    /* bit4: I2C禁用(SPI使能) */
#define MPU6050_USER_FIFO_RST           0x04    /* bit2: FIFO复位 */
#define MPU6050_USER_I2C_MST_RST        0x02    /* bit1: I2C主机复位 */
#define MPU6050_USER_SIG_COND_RST       0x01    /* bit0: 信号通路复位 */

/*---- INT_PIN_CFG (0x37) ----*/
#define MPU6050_INTCFG_INT_LEVEL        0x80    /* bit7: 0=低有效 1=高有效 */
#define MPU6050_INTCFG_INT_OPEN         0x40    /* bit6: 0=推挽 1=开漏 */
#define MPU6050_INTCFG_LATCH_INT_EN     0x20    /* bit5: 中断锁存使能 */
#define MPU6050_INTCFG_INT_RD_CLEAR     0x10    /* bit4: 读INT_STATUS清中断 */
#define MPU6050_INTCFG_FSYNC_INT_LEVEL  0x08    /* bit3: FSYNC中断电平 */
#define MPU6050_INTCFG_FSYNC_INT_EN     0x04    /* bit2: FSYNC中断使能 */
#define MPU6050_INTCFG_I2C_BYPASS_EN    0x02    /* bit1: I2C旁路使能 */
#define MPU6050_INTCFG_CLKOUT_EN        0x01    /* bit0: 时钟输出使能 */

/*---- INT_ENABLE (0x38) ----*/
#define MPU6050_INT_FF_EN               0x80    /* bit7: 自由落体中断 */
#define MPU6050_INT_MOT_EN              0x40    /* bit6: 运动检测中断 */
#define MPU6050_INT_ZMOT_EN             0x20    /* bit5: 静止检测中断 */
#define MPU6050_INT_FIFO_OFLOW_EN       0x10    /* bit4: FIFO溢出中断 */
#define MPU6050_INT_I2C_MST_EN          0x08    /* bit3: I2C主机中断 */
#define MPU6050_INT_DATA_RDY_EN         0x01    /* bit0: 数据就绪中断 */

/*============================ 传感器灵敏度 ============================*/
/* 陀螺仪 LSB/(°/s) */
#define MPU6050_GYRO_SENS_250       131.0f
#define MPU6050_GYRO_SENS_500       65.5f
#define MPU6050_GYRO_SENS_1000      32.8f
#define MPU6050_GYRO_SENS_2000      16.4f

/* 加速度 LSB/g */
#define MPU6050_ACCEL_SENS_2G       16384.0f
#define MPU6050_ACCEL_SENS_4G       8192.0f
#define MPU6050_ACCEL_SENS_8G       4096.0f
#define MPU6050_ACCEL_SENS_16G      2048.0f

#endif //__CYWATCH_BSP_AHT21_REG_H__
