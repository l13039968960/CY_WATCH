/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file spi_hal.h
 *
 * @par dependencies
 * - stm32f4xx_hal.h
 * - stdint.h
 *
 * @author zw1194
 *
 * @brief Provide the OO-style HAL APIs of hardware SPI and corresponding opetions.
 *
 * Processing flow:
 *
 * call spi_driver_inst() to construct, then use function pointers.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
#ifndef __SPI_HAL_H__
#define __SPI_HAL_H__

/***********************************Includes***********************************/
#include "stm32f4xx_hal.h"
#include <stdint.h>

/***********************************Includes***********************************/

/***********************************Defines************************************/
#define OS_SUPPORTING            /* 有 RTOS 时定义; 裸机也可保留但 yield 空实现 */

/* 阻塞发送的RX回波scratch容量(全双工下发送用TransmitReceive收走回波防OVR).
 * 须 ≥ 单次最大发送字节数: W25Q64页编程≤256B / LCD命令≤4B, 取256. */
#define SPI_TX_RX_SCRATCH_SIZE   256

/***********************************Defines************************************/

/**********************************Declaring***********************************/
/* SPI硬件配置 */
typedef struct
{
	SPI_TypeDef *p_spi_base;      /* SPI外设基地址(如SPI1) */
	SPI_InitTypeDef init;         /* SPI初始化参数(模式/极性/相位/预分频等) */
	DMA_HandleTypeDef *p_hdma_tx; /* TX DMA句柄(可为NULL, 不使用DMA发送) */
	uint32_t tx_timeout_tick;     /* 裸机DMA发送超时计数(ms) */
} spi_cfg_t;

/* 中断阻塞等待和释放接口 */
typedef struct
{
	int8_t (*pf_wait)(void);    /* 阻塞等待信号量 */
	int8_t (*pf_release)(void); /* 释放信号量(ISR中调用) */
} spi_semaphore_interface_t;

/* 延时接口*/
typedef struct
{
	void (*pf_delay_us)(uint32_t us); /* 微秒延时 */
} spi_delay_interface_t;

/* 时基计数器接口 */
typedef struct
{
	uint32_t (*pf_get_time)(void); /* 毫秒时间戳 */
} spi_timebase_interface_t;

/* SPI驱动对象 */
typedef struct spi_driver
{
	/* 第一个成员: 内嵌SPI句柄(按值), 用于在 HAL_SPI_TxCpltCallback 中
	 * 通过 container_of(首个成员偏移为0) 强转反查本实例, 实现多实例支持 */
	SPI_HandleTypeDef hspi;
	spi_cfg_t cfg; /* 硬件配置 */

	spi_semaphore_interface_t *p_semaphore_interface; /* OS信号量 */
	spi_delay_interface_t *p_delay_interface;         /* 延时接口*/

#ifndef OS_SUPPORTING
	spi_timebase_interface_t *p_timebase_interface;   /* 裸机时基计数器 */

	/* 裸机DMA发送状态(ISR置位 tx_complete, 由 pf_get_time 计时) */
	volatile uint8_t tx_complete; /* 发送完成标志 */
	uint32_t tx_start_tick;       /* 发送启动时刻(用于超时计数) */
#endif /* OS_SUPPORTING */

	/* 构造与析构 */
	int8_t (*pf_inst)(struct spi_driver *p_spi_instance,
					  spi_cfg_t *p_cfg,
					  spi_semaphore_interface_t *p_semaphore_interface,
					  spi_delay_interface_t *p_delay_interface,
#ifndef OS_SUPPORTING
					  spi_timebase_interface_t *p_timebase_interface
#endif /* OS_SUPPORTING */											);
	
	int8_t (*pf_deinst)(struct spi_driver *p_spi_instance);

	/* SPI传输操作*/
	int8_t (*pf_transmit)(struct spi_driver *p_spi_instance,
						  uint8_t *pdata,
						  uint32_t size);
	int8_t (*pf_receive)(struct spi_driver *p_spi_instance,
						 uint8_t *pdata,
						 uint32_t size);
	int8_t (*pf_transmit_receive)(struct spi_driver *p_spi_instance,
								  uint8_t *ptx,
								  uint8_t *prx,
								  uint32_t size);

	/* DMA中断发送(启动+等待合并): 阻塞等信号量 / 裸机计数器轮询 */
	int8_t (*pf_transmit_dma)(struct spi_driver *p_spi_instance,
							  uint8_t *pdata,
							  uint32_t size);

} spi_driver_t;

/* SPI驱动构造函数 */
int8_t spi_driver_inst(spi_driver_t *p_spi_instance,
					   spi_cfg_t *p_cfg,
					   spi_semaphore_interface_t *p_semaphore_interface,
					   spi_delay_interface_t *p_delay_interface,
#ifndef OS_SUPPORTING
					   spi_timebase_interface_t *p_timebase_interface
#endif /* OS_SUPPORTING */											);

/**********************************Declaring***********************************/

#endif // __SPI_HAL_H__
