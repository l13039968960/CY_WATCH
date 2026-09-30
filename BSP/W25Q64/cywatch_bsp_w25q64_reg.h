/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_bsp_w25q64_reg.h
 *
 * @author
 *
 * @brief Provide the instruction set and register definitions of w25q64.
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
#ifndef __CYWATCH_BSP_W25Q64_REG_H__
#define __CYWATCH_BSP_W25Q64_REG_H__

/*============================ 器件ID (JEDEC) ============================*/
#define W25Q64_MANUFACTURER_ID      0xEF    /* 厂商ID(华邦Winbond) */
#define W25Q64_MEMORY_TYPE_ID       0x40    /* 存储类型(SPI NOR Flash) */
#define W25Q64_CAPACITY_ID          0x17    /* 容量ID(64M-bit) */

/*============================ 存储组织 ============================*/
#define W25Q64_PAGE_SIZE            256     /* 页大小(字节) */
#define W25Q64_SECTOR_SIZE          4096    /* 扇区大小(4KB) */
#define W25Q64_BLOCK_32K_SIZE       32768   /* 32KB块大小 */
#define W25Q64_BLOCK_64K_SIZE       65536   /* 64KB块大小 */
#define W25Q64_TOTAL_SIZE           8388608 /* 总容量(8MB = 64M-bit) */
#define W25Q64_ADDR_BYTES           3       /* 24位地址(A23~A0) */

/*============================ 通用控制指令 ============================*/
#define W25Q64_CMD_WRITE_ENABLE         0x06    /* 写使能(置位WEL) */
#define W25Q64_CMD_WRITE_DISABLE        0x04    /* 写禁止(清零WEL) */
#define W25Q64_CMD_READ_JEDEC_ID        0x9F    /* 读JEDEC ID(厂商+类型+容量) */
#define W25Q64_CMD_POWER_DOWN           0xB9    /* 深度掉电 */
#define W25Q64_CMD_RELEASE_POWER_DOWN   0xAB    /* 释放掉电/读设备ID */

/*============================ 读取数据指令 ============================*/
#define W25Q64_CMD_READ_DATA            0x03    /* 普通读(无哑周期, ≤50MHz) */
#define W25Q64_CMD_FAST_READ            0x0B    /* 快速读(需1字节哑周期, ≤133MHz) */

/*============================ 编程写入指令 ============================*/
#define W25Q64_CMD_PAGE_PROGRAM         0x02    /* 页编程(单次≤256字节, 跨页回卷) */

/*============================ 擦除操作指令 ============================*/
#define W25Q64_CMD_SECTOR_ERASE         0x20    /* 4KB扇区擦除 */
#define W25Q64_CMD_BLOCK_ERASE_32K      0x52    /* 32KB块擦除 */
#define W25Q64_CMD_BLOCK_ERASE_64K      0xD8    /* 64KB块擦除 */
#define W25Q64_CMD_CHIP_ERASE           0xC7    /* 整片擦除 */

/*============================ 状态寄存器指令 ============================*/
#define W25Q64_CMD_READ_STATUS_REG1     0x05    /* 读状态寄存器1 */
#define W25Q64_CMD_WRITE_STATUS_REG1    0x01    /* 写状态寄存器1 */
#define W25Q64_CMD_READ_STATUS_REG2     0x35    /* 读状态寄存器2 */
#define W25Q64_CMD_WRITE_STATUS_REG2    0x31    /* 写状态寄存器2 */
#define W25Q64_CMD_READ_STATUS_REG3     0x15    /* 读状态寄存器3 */
#define W25Q64_CMD_WRITE_STATUS_REG3    0x11    /* 写状态寄存器3 */

/*============================ 状态寄存器1 (SR1) 位定义 ============================*/
#define W25Q64_SR1_BUSY                 0x01    /* bit0: 忙标志 1=正在擦除/编程 */
#define W25Q64_SR1_WEL                  0x02    /* bit1: 写使能锁存 1=已使能 */
#define W25Q64_SR1_BP0                  0x04    /* bit2: 块保护位0 */
#define W25Q64_SR1_BP1                  0x08    /* bit3: 块保护位1 */
#define W25Q64_SR1_BP2                  0x10    /* bit4: 块保护位2 */
#define W25Q64_SR1_TB                   0x20    /* bit5: 保护方向 0=顶部 1=底部 */
#define W25Q64_SR1_SEC                  0x40    /* bit6: 保护粒度 0=64KB块 1=4KB扇区 */
#define W25Q64_SR1_SRP0                 0x80    /* bit7: 状态寄存器保护位0 */

/*============================ 状态寄存器2 (SR2) 位定义 ============================*/
#define W25Q64_SR2_SRP1                 0x01    /* bit0: 状态寄存器保护位1 */
#define W25Q64_SR2_QE                   0x02    /* bit1: 四线模式使能 1=启用Quad SPI */

#endif // __CYWATCH_BSP_W25Q64_REG_H__
