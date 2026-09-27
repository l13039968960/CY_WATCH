/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_bsp_st7789t3_reg.h
 *
 * @author zw1194
 *
 * @brief Provide the address of st7789t3's registers.
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
#ifndef __CYWATCH_BSP_ST7789T3_REG_H__
#define __CYWATCH_BSP_ST7789T3_REG_H__

/*============================ 面板尺寸 ============================*/
#define ST7789T3_WIDTH          240     /* 面板物理宽(竖屏) */
#define ST7789T3_HEIGHT         280     /* 面板物理高(竖屏) */

/*============================ 显示区域偏移 ============================*/
/* 1.69寸240x280屏映射在ST7789T3的240x320 GRAM中, 280方向偏移20 */
#define ST7789T3_OFFSET_X       0       /* 横向偏移 */
#define ST7789T3_OFFSET_Y       20      /* 纵向偏移(可见区从GRAM第20行开始) */

/*============================ 系统控制命令 ============================*/
#define ST7789T3_SWRESET        0x01    /* 软件复位 */
#define ST7789T3_SLPIN          0x10    /* 进入睡眠模式 */
#define ST7789T3_SLPOUT         0x11    /* 退出睡眠模式 */
#define ST7789T3_DISPOFF        0x28    /* 关闭显示输出 */
#define ST7789T3_DISPON         0x29    /* 开启显示输出 */

/*============================ 显示效果控制 ============================*/
#define ST7789T3_INVOFF         0x20    /* 关闭显示反转 */
#define ST7789T3_INVON          0x21    /* 开启显示反转(IPS面板需要) */
#define ST7789T3_MADCTL         0x36    /* 内存访问控制(旋转/镜像/颜色顺序) */
#define ST7789T3_COLMOD         0x3A    /* 接口像素格式设置 */

/*============================ 窗口与显存读写 ============================*/
#define ST7789T3_CASET          0x2A    /* 列地址设置 */
#define ST7789T3_RASET          0x2B    /* 行地址设置 */
#define ST7789T3_RAMWR          0x2C    /* 显存写入命令 */
#define ST7789T3_RAMRD          0x2E    /* 显存读取命令 */

/*============================ 电源与帧率控制 ============================*/
#define ST7789T3_PORCTRL        0xB2    /* 栅极升压控制 */
#define ST7789T3_VCOMS          0xBB    /* VCOM电压设置 */
#define ST7789T3_LCMCTRL        0xC2    /* LCM控制寄存器 */
#define ST7789T3_VRHSET         0xC3    /* 正电压调节 */
#define ST7789T3_VDVSET         0xC4    /* 负电压调节 */
#define ST7789T3_VCMOFSET       0xC5    /* VCOM偏移校准 */
#define ST7789T3_FRCTRL2        0xC6    /* 正常模式帧率控制 */
#define ST7789T3_PWCTRL1        0xD0    /* 电源控制1 */

/*============================ Gamma校正 ============================*/
#define ST7789T3_GMCTRP1        0xE0    /* 正向Gamma曲线设置 */
#define ST7789T3_GMCTRN1        0xE1    /* 负向Gamma曲线设置 */

/*============================ 扩展功能 ============================*/
#define ST7789T3_PTLAR          0x30    /* 局部显示区域设置 */
#define ST7789T3_VSCRDEF        0x33    /* 垂直滚动区域定义 */
#define ST7789T3_TEOFF          0x34    /* 撕裂效果线关闭 */
#define ST7789T3_TEON           0x35    /* 撕裂效果线开启 */
#define ST7789T3_VSCSAD         0x37    /* 垂直滚动起始地址 */

/*============================ MADCTL (0x36) 位定义 ============================*/
#define ST7789T3_MADCTL_MY      0x80    /* bit7: 行地址顺序 1=从下到上 */
#define ST7789T3_MADCTL_MX      0x40    /* bit6: 列地址顺序 1=从右到左 */
#define ST7789T3_MADCTL_MV      0x20    /* bit5: 行列交换 1=横屏 */
#define ST7789T3_MADCTL_ML      0x10    /* bit4: 垂直扫描方向 1=反向 */
#define ST7789T3_MADCTL_BGR     0x08    /* bit3: 颜色顺序 1=BGR */
#define ST7789T3_MADCTL_MH      0x04    /* bit2: 水平数据顺序 1=反向 */

/* 常用旋转方向组合值(数据手册典型值) */
#define ST7789T3_MADCTL_ROT_0       0x00    /* 0°   竖屏 240x280 */
#define ST7789T3_MADCTL_ROT_90      0x60    /* 90°  横屏 280x240 (MX|MV) */
#define ST7789T3_MADCTL_ROT_180     0xC0    /* 180° 竖屏 240x280 (MY|MX) */
#define ST7789T3_MADCTL_ROT_270     0xA0    /* 270° 横屏 280x240 (MY|MV) */

/*============================ COLMOD (0x3A) 像素格式 ============================*/
#define ST7789T3_COLMOD_RGB444      0x33    /* 12bit RGB444 */
#define ST7789T3_COLMOD_RGB565      0x55    /* 16bit RGB565 (推荐) */
#define ST7789T3_COLMOD_RGB666      0x66    /* 18bit RGB666 */

#endif // __CYWATCH_BSP_ST7789T3_REG_H__
