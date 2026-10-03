/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file dma_hal.h
 *
 * @par dependencies
 * - stm32f4xx_hal.h
 * - stdint.h
 *
 * @author zw1194
 *
 * @brief Provide the OO-style HAL APIs of DMA and corresponding opetions.
 *
 * Processing flow:
 *
 * call dma_driver_inst() to construct, pf_init() to build the stream,
 * then the peripheral driver links the handle and sets its own callbacks.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
#ifndef __DMA_HAL_H__
#define __DMA_HAL_H__

/***********************************Includes***********************************/
#include "stm32f4xx_hal.h"
#include <stdint.h>

/***********************************Includes***********************************/

/**********************************Declaring***********************************/
/* 一条DMA流描述 */
typedef struct
{
	DMA_Stream_TypeDef *p_stream; /* DMA流(如DMA2_Stream3) */
	uint32_t channel;             /* 流请求通道(DMA_CHANNEL_x) */
	IRQn_Type irqn;               /* 该流的中断号(如DMA2_Stream3_IRQn) */
	uint32_t direction;           /* DMA_MEMORY_TO_PERIPH(发) / DMA_PERIPH_TO_MEMORY(收) */
	uint32_t mode;                /* DMA_NORMAL / DMA_CIRCULAR */
	uint32_t priority;            /* 流之间的硬件仲裁(DMA_PRIORITY_x) */
	uint32_t nvic_priority;       /* NVIC抢占优先级. ★必须 ≥
	                                 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(5)★,
	                                 否则完成回调里的 osSemaphoreRelease 会破坏内核 */
} dma_cfg_t;

/* DMA驱动对象 */
typedef struct dma_driver
{
	DMA_HandleTypeDef *p_hdma; /* 调用方内嵌的句柄, 本驱动只填不改归属 */
	dma_cfg_t cfg;             /* 流描述 */

	/* 构造: 存配置 + 挂函数指针(不碰硬件); 析构: 清句柄与指针
	   ★不碰 Parent 与 XferCpltCallback★ —— 收完要干什么只有外设驱动知道 */
	int8_t (*pf_inst)(struct dma_driver *p_dma_instance,
					  DMA_HandleTypeDef *p_hdma,
					  dma_cfg_t *p_cfg);
	int8_t (*pf_deinst)(struct dma_driver *p_dma_instance);

	/* 占用: 开控制器时钟 + 建流 + 配NVIC + 登记中断分发表;
	   释放: 反登记 + 关NVIC + 停流 + 归还时钟(该控制器最后一个流才真关) */
	int8_t (*pf_init)(struct dma_driver *p_dma_instance);
	int8_t (*pf_deinit)(struct dma_driver *p_dma_instance);

} dma_driver_t;

/* DMA驱动构造函数: 装配一条流 */
int8_t dma_driver_inst(dma_driver_t *p_dma_instance,
					   DMA_HandleTypeDef *p_hdma,
					   dma_cfg_t *p_cfg);

/* DMA中断分发: 由 it.c / adapter 的 DMAx_Streamy_IRQHandler 调用, 按流反查句柄,
   未登记的流直接返回 */
void dma_irq_handler(DMA_Stream_TypeDef *p_stream);

/**********************************Declaring***********************************/

#endif // __DMA_HAL_H__
