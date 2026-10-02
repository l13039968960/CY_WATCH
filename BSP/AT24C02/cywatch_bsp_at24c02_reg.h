/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_bsp_at24c02_reg.h
 *
 * @author
 *
 * @brief Provide the I2C address and memory organization of at24c02.
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
#ifndef __CYWATCH_BSP_AT24C02_REG_H__
#define __CYWATCH_BSP_AT24C02_REG_H__

/*============================ 器件I2C地址 ============================*/
#define AT24C02_I2C_ADDR        0x50    /* 7位地址(高4位1010, A2/A1/A0全接地) */

/*============================ 存储组织 ============================*/
#define AT24C02_PAGE_SIZE       8       /* 页大小(字节), 共32页 */
#define AT24C02_PAGE_NUM        32      /* 页数量 */
#define AT24C02_TOTAL_SIZE      256     /* 总容量(256字节 = 2K-bit) */
#define AT24C02_ADDR_BYTES      1       /* 存储地址宽度(8位, 0x00~0xFF) */

/*============================ 说明 ============================*/
/* AT24C02为纯存储型EEPROM, 无控制寄存器、状态寄存器与独立指令码。
 * 所有操作均通过I2C帧格式(器件地址读写位 + 存储地址)实现;
 * 写周期忙状态通过I2C ACK轮询检测(见驱动层 pf_wait_busy)。 */

#endif // __CYWATCH_BSP_AT24C02_REG_H__
