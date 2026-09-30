/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_adapter_w25q64.h
 *
 * @par dependencies
 * - stdint.h
 *
 * @author	zw1194
 *
 * @brief Provide the storage-level APIs of W25Q64 (adapter 层).
 *
 * Processing flow:
 *
 * storage_bsp_w25q64_inst()  : 挂 SPI/GPIO/delay 三个接口 + 构造驱动(内部先配 CS(PB12)、
 *                              初始化 SPI2 总线, 再做 JEDEC ID 自检)
 * storage_bsp_w25q64_read/write/erase_* : 转发到驱动的 pf_*
 * storage_bsp_w25q64_deinst(): 析构
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 本层把驱动挂到本工程的硬件上: SPI2(PB13=SCK/PB14=MISO/PB15=MOSI) + CS(PB12).
 *       总线实例 `spi2_instance` 由应用层 main.c 创建, 本 adapter 只 extern 引用
 *       (与 ST7789T3 adapter 引用 `lcd_spi_instance` 同一写法), 不自己再建总线.
 *
 * @note `storage_bsp_w25q64_inst()` 内含 osDelay(芯片上电等待与 wait_busy 轮询) →
 *       必须在 osKernelStart() 之后的**任务上下文**调用, 不能放在 main 启动内核之前.
 *****************************************************************************/
#ifndef __CYWATCH_ADAPTER_W25Q64_H__
#define __CYWATCH_ADAPTER_W25Q64_H__

/***********************************Includes***********************************/
#include <stdint.h>

/**********************************Declaring***********************************/
int8_t storage_bsp_w25q64_inst(void);
int8_t storage_bsp_w25q64_deinst(void);
int8_t storage_bsp_w25q64_read_id(uint8_t *p_manuf_id,
								  uint8_t *p_memory_type,
								  uint8_t *p_capacity);
int8_t storage_bsp_w25q64_read(uint32_t addr, uint8_t *pdata, uint32_t size);
int8_t storage_bsp_w25q64_write(uint32_t addr, uint8_t *pdata, uint32_t size);
int8_t storage_bsp_w25q64_erase_sector(uint32_t addr);
int8_t storage_bsp_w25q64_erase_block_32k(uint32_t addr);
int8_t storage_bsp_w25q64_erase_block_64k(uint32_t addr);
int8_t storage_bsp_w25q64_erase_chip(void);
int8_t storage_bsp_w25q64_wait_busy(void);
int8_t storage_bsp_w25q64_hibernating(void);
int8_t storage_bsp_w25q64_wakeup(void);
/**********************************Declaring***********************************/

#endif // __CYWATCH_ADAPTER_W25Q64_H__
