/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file dma_hal.c
 *
 * @par dependencies
 * - dma_hal.h
 *
 * @author zw1194
 *
 * @brief Implete the OO-style HAL operations of DMA.
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
#include "dma_hal.h"

/********************************* 前向声明 *********************************/
static int8_t dma_deinst(dma_driver_t *p_dma_instance);
static int8_t dma_init(dma_driver_t *p_dma_instance);
static int8_t dma_deinit(dma_driver_t *p_dma_instance);

/********************************* 私有辅助 *********************************/

/* DMA1 八条流占 0-7, DMA2 八条流占 8-15 —— 直接用流指针反查下标 */
#define DMA_STREAM_COUNT   16

static const DMA_Stream_TypeDef * const g_dma_streams[DMA_STREAM_COUNT] =
{
	DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3,
	DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7,
	DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3,
	DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7,
};

/* 中断分发表: 按流下标存句柄, 供 dma_irq_handler 反查 */
static DMA_HandleTypeDef *g_dma_handles[DMA_STREAM_COUNT] = { NULL };

/******************************************************************************
 * @name    dma_stream_index
 * @brief   流指针 → 下标(0-15)
 * @param   p_stream[in] DMA流
 *
 * @return  0-15, 不是本芯片的流则 -1
 *****************************************************************************/
static int8_t dma_stream_index(DMA_Stream_TypeDef *p_stream)
{
	uint8_t i;

	for (i = 0U; i < DMA_STREAM_COUNT; i++)
	{
		if (p_stream == g_dma_streams[i])
		{
			return (int8_t)i;
		}
	}

	return -1;
}

/* 控制器级使用者计数: 0=DMA1, 1=DMA2 */
#define DMA_CTRL_NUM 2U

static uint8_t s_dma_ref_count[DMA_CTRL_NUM];

/******************************************************************************
 * @name    dma_clk_claim
 * @brief   取用该流所属控制器的时钟: 控制器第一个流才真正开, 计数累加
 * @param   index[in] 流下标(0-15, 调用前已校验有效)
 *
 * @note    ★按控制器计数, 不是按流★ —— DMA2 上同时挂着 SPI1_TX 和 UART1 收发,
 *          哪个流先拆完就关时钟会打死同控制器上的其他流.
 *****************************************************************************/
static void dma_clk_claim(int8_t index)
{
	uint8_t ctrl = (index < 8) ? 0U : 1U;

	if (0U == s_dma_ref_count[ctrl])
	{
		if (0U == ctrl)
		{
			__HAL_RCC_DMA1_CLK_ENABLE();
		}
		else
		{
			__HAL_RCC_DMA2_CLK_ENABLE();
		}
	}

	s_dma_ref_count[ctrl]++;
}

/******************************************************************************
 * @name    dma_clk_release
 * @brief   归还控制器时钟: 该控制器最后一个流才真正关, 计数递减
 * @param   index[in] 流下标(0-15)
 *
 * @note    与 claim 严格成对.
 *****************************************************************************/
static void dma_clk_release(int8_t index)
{
	uint8_t ctrl = (index < 8) ? 0U : 1U;

	if (0U == s_dma_ref_count[ctrl])
	{
		return;
	}

	s_dma_ref_count[ctrl]--;

	if (0U == s_dma_ref_count[ctrl])
	{
		if (0U == ctrl)
		{
			__HAL_RCC_DMA1_CLK_DISABLE();
		}
		else
		{
			__HAL_RCC_DMA2_CLK_DISABLE();
		}
	}
}

/********************************* 初始化与收尾 *********************************/

/******************************************************************************
 * @name    dma_init
 * @brief   占用一条流: 开控制器时钟、HAL_DMA_Init、配NVIC、登记中断分发
 * @param   p_dma_instance[in]
 *
 * @return  0 success
 *         -1 dma_instance null
 *         -2 未装配过(句柄空)
 *         -3 不是本芯片的流
 *         -4 HAL_DMA_Init error
 *
 * @note    对齐/增量是写死的(外设不自增、内存自增、字节宽度、FIFO关):
 *          本工程 SPI 与 UART 对 DMA 只有这一种配法, 故不放进 cfg.
 *****************************************************************************/
static int8_t dma_init(dma_driver_t *p_dma_instance)
{
	DMA_HandleTypeDef *p_hdma;
	int8_t index;

	if (NULL == p_dma_instance)
	{
		return -1;
	}

	if (NULL == p_dma_instance->p_hdma)
	{
		return -2;
	}

	index = dma_stream_index(p_dma_instance->cfg.p_stream);
	if (index < 0)
	{
		return -3;
	}

	p_hdma = p_dma_instance->p_hdma;

	dma_clk_claim(index);

	p_hdma->Instance = p_dma_instance->cfg.p_stream;
	p_hdma->Init.Channel             = p_dma_instance->cfg.channel;
	p_hdma->Init.Direction           = p_dma_instance->cfg.direction;
	p_hdma->Init.PeriphInc           = DMA_PINC_DISABLE;
	p_hdma->Init.MemInc              = DMA_MINC_ENABLE;
	p_hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	p_hdma->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
	p_hdma->Init.Mode                = p_dma_instance->cfg.mode;
	p_hdma->Init.Priority            = p_dma_instance->cfg.priority;
	p_hdma->Init.FIFOMode            = DMA_FIFOMODE_DISABLE;

	if (HAL_OK != HAL_DMA_Init(p_hdma))
	{
		return -4;
	}

	HAL_NVIC_SetPriority(p_dma_instance->cfg.irqn,
						 p_dma_instance->cfg.nvic_priority, 0);
	HAL_NVIC_EnableIRQ(p_dma_instance->cfg.irqn);

	/* 最后才登记: 登记之后中断才可能进来 */
	g_dma_handles[index] = p_hdma;

	return 0;
}

/******************************************************************************
 * @name    dma_deinit
 * @brief   释放一条流: 从分发表摘除、关NVIC、停流、归还控制器时钟
 * @param   p_dma_instance[in]
 *
 * @return  0 success
 *         -1 dma_instance null
 *         -2 句柄空(未装配或已释放)
 *
 * @note    ★控制器时钟按控制器计数关★: 同一控制器上还有别的流在用就不关.
 *****************************************************************************/
static int8_t dma_deinit(dma_driver_t *p_dma_instance)
{
	int8_t index;

	if (NULL == p_dma_instance)
	{
		return -1;
	}

	if (NULL == p_dma_instance->p_hdma)
	{
		return -2;
	}

	/* 先摘表再停流: 反过来的话中断可能落到正在拆除的句柄上 */
	index = dma_stream_index(p_dma_instance->cfg.p_stream);
	if ((index >= 0) && (g_dma_handles[index] == p_dma_instance->p_hdma))
	{
		g_dma_handles[index] = NULL;
	}

	HAL_NVIC_DisableIRQ(p_dma_instance->cfg.irqn);
	(void)HAL_DMA_Abort(p_dma_instance->p_hdma);
	(void)HAL_DMA_DeInit(p_dma_instance->p_hdma);

	if (index >= 0)
	{
		dma_clk_release(index);
	}

	return 0;
}

/********************************* 构造与析构 *********************************/

/******************************************************************************
 * @name    dma_driver_inst
 * @brief   DMA驱动构造函数: 存配置、挂函数指针(不碰硬件, 流由 pf_init 建)
 * @param   p_dma_instance[out] DMA驱动实例
 * @param   p_hdma[in]          调用方内嵌的DMA句柄(按值)
 * @param   p_cfg[in]           流描述
 *
 * @return  0 success
 *         -1 dma_instance null
 *         -2 hdma null
 *         -3 cfg null
 *         -4 stream null
 *         -5 不是本芯片的流
 *
 * @note    ★不设 Parent, 不设 XferCpltCallback★: 这两样由外设驱动自己做,
 *          只有它知道收完要释放哪个信号量、要反查到哪个实例.
 *****************************************************************************/
int8_t dma_driver_inst(dma_driver_t *p_dma_instance,
					   DMA_HandleTypeDef *p_hdma,
					   dma_cfg_t *p_cfg)
{
	if (NULL == p_dma_instance)
	{
		return -1;
	}

	if (NULL == p_hdma)
	{
		return -2;
	}

	if (NULL == p_cfg)
	{
		return -3;
	}

	if (NULL == p_cfg->p_stream)
	{
		return -4;
	}

	if (dma_stream_index(p_cfg->p_stream) < 0)
	{
		return -5;
	}

	p_dma_instance->p_hdma = p_hdma;
	p_dma_instance->cfg    = *p_cfg;

	p_dma_instance->pf_inst   = dma_driver_inst;
	p_dma_instance->pf_deinst = dma_deinst;
	p_dma_instance->pf_init   = dma_init;
	p_dma_instance->pf_deinit = dma_deinit;

	return 0;
}

/******************************************************************************
 * @name    dma_deinst
 * @brief   DMA驱动析构: 清句柄与函数指针(不碰硬件, 流由 pf_deinit 释放)
 * @param   p_dma_instance[in]
 *
 * @return  0 success
 *         -1 dma_instance null
 *****************************************************************************/
static int8_t dma_deinst(dma_driver_t *p_dma_instance)
{
	if (NULL == p_dma_instance)
	{
		return -1;
	}

	p_dma_instance->p_hdma    = NULL;
	p_dma_instance->pf_inst   = NULL;
	p_dma_instance->pf_deinst = NULL;
	p_dma_instance->pf_init   = NULL;
	p_dma_instance->pf_deinit = NULL;

	return 0;
}

/********************************* 中断分发 *********************************/

/******************************************************************************
 * @name    dma_irq_handler
 * @brief   DMA中断分发: 按流反查句柄并转给 HAL_DMA_IRQHandler
 * @param   p_stream[in] DMA流(由 DMAx_Streamy_IRQHandler 传入)
 *
 * @note    F411 每条流有独立中断号, 不存在共享线, 所以按流反查就够;
 *          未登记的流直接返回(中断可能来自尚未装配的流)
 *****************************************************************************/
void dma_irq_handler(DMA_Stream_TypeDef *p_stream)
{
	int8_t index = dma_stream_index(p_stream);

	if (index < 0)
	{
		return;
	}

	if (NULL != g_dma_handles[index])
	{
		HAL_DMA_IRQHandler(g_dma_handles[index]);
	}
}
