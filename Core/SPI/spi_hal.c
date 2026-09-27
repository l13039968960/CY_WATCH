/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file spi_hal.c
 *
 * @par dependencies
 * - spi_hal.h
 *
 * @author zw1194
 *
 * @brief Implete the OO-style HAL operations of hardware SPI.
 *
 * Processing flow:
 *
 * call spi_driver_inst() to construct, then use function pointers.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 与 spi_hal.h 的两条接口契约(本文件的形参/调用点与头文件逐字对齐):
 *       1) spi_semaphore_interface_t 只有 pf_wait(void)/pf_release(void) 两个
 *          **无形参**回调, 结构体里没有实例成员 —— 信号量句柄归注入方自己持有
 *          (本工程是 main.c 的 lcd_spi_sem_handle; 裸机下可为空实现).
 *       2) 四个传输函数的实例形参是 struct spi_driver *, 不再是 void *.
 *****************************************************************************/
#include "spi_hal.h"

static int8_t spi_deinst(spi_driver_t *p_spi_instance);
static int8_t spi_transmit(spi_driver_t *p_spi_instance, uint8_t *pdata, uint32_t size);
static int8_t spi_receive(spi_driver_t *p_spi_instance, uint8_t *pdata, uint32_t size);
static int8_t spi_transmit_receive(spi_driver_t *p_spi_instance, uint8_t *ptx, uint8_t *prx, uint32_t size);
static int8_t spi_transmit_dma(spi_driver_t *p_spi_instance, uint8_t *pdata, uint32_t size);


/******************************************************************************
 * @name    HAL_SPI_TxCpltCallback
 * @brief   重写HAL弱函数: SPI TX DMA完成回调(在DMA中断上下文执行)
 * @param   hspi[in] SPI句柄(即 spi_driver_t 首个成员 &p_spi->hspi)
 *
 * @note    hspi 为 spi_driver_t 首个成员(按值嵌入, 偏移为0), 直接强转反查实例,
 *          实现多实例; 中断上下文内仅释放信号量/置完成标志, 不做耗时操作
 *****************************************************************************/
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	/* container_of: hspi 位于实例偏移0处, 强转即得实例指针 */
	spi_driver_t *p_spi = (spi_driver_t *)hspi;

	/* OS: 释放信号量, 唤醒等待的发送任务
	 * 裸机: 置标志位, 唤醒阻塞状态 */
	if (NULL != p_spi->p_semaphore_interface &&
		NULL != p_spi->p_semaphore_interface->pf_release)
	{
		(void)p_spi->p_semaphore_interface->pf_release();
	}
}

/********************************* SPI传输 *********************************/

/******************************************************************************
 * @name    spi_transmit
 * @brief   阻塞发送字节流
 * @param   p_spi_instance[in] SPI驱动实例
 * @param   pdata[in] 数据缓冲区
 * @param   size[in]  字节数(全双工下须≤SPI_TX_RX_SCRATCH_SIZE)
 *
 * @return  0 success
 *         -1 spi_instance null
 *         -2 spi transmit error
 *         -3 size exceeds scratch buffer
 *
 * @note    全双工(2LINES)下 HAL_SPI_Transmit 纯发送不收: 多字节(>2, RX FIFO 2深)
 *          会触发 OVR 溢出并把脏数据留在 FIFO, 后续 HAL_SPI_Receive 被污染甚至
 *          因 BSY 状态卡死(HAL_MAX_DELAY 无限等)。因此发送一律走 TransmitReceive,
 *          边发边把回波收进 scratch(丢弃), 保证无 OVR、FIFO 始终干净。
 *****************************************************************************/
static int8_t spi_transmit(spi_driver_t *p_spi_instance, uint8_t *pdata, uint32_t size)
{
	static uint8_t s_rx_scratch[SPI_TX_RX_SCRATCH_SIZE];  /* 收走回波, 丢弃 */

	if (NULL == p_spi_instance)
	{
		return -1;
	}

	if (size > sizeof(s_rx_scratch))
	{
		return -3;
	}

	if (HAL_OK != HAL_SPI_TransmitReceive(&p_spi_instance->hspi,
										   pdata, s_rx_scratch,
										   (uint16_t)size, HAL_MAX_DELAY))
	{
		return -2;
	}

	return 0;
}

/******************************************************************************
 * @name    spi_receive
 * @brief   阻塞接收字节流
 * @param   p_spi_instance[in] SPI驱动实例
 * @param   pdata[out] 数据缓冲区
 * @param   size[in]   字节数
 *
 * @return  0 success
 *         -1 spi_instance null
 *         -2 spi receive error
 *****************************************************************************/
static int8_t spi_receive(spi_driver_t *p_spi_instance, uint8_t *pdata, uint32_t size)
{
	if (NULL == p_spi_instance)
	{
		return -1;
	}

	if (HAL_OK != HAL_SPI_Receive(&p_spi_instance->hspi,
								  pdata, (uint16_t)size, HAL_MAX_DELAY))
	{
		return -2;
	}

	return 0;
}

/******************************************************************************
 * @name    spi_transmit_receive
 * @brief   阻塞全双工收发
 * @param   p_spi_instance[in] SPI驱动实例
 * @param   ptx[in]    发送缓冲区
 * @param   prx[out]   接收缓冲区
 * @param   size[in]   字节数
 *
 * @return  0 success
 *         -1 spi_instance null
 *         -2 spi transmit receive error
 *****************************************************************************/
static int8_t spi_transmit_receive(spi_driver_t *p_spi_instance,
								   uint8_t *ptx,
								   uint8_t *prx,
								   uint32_t size)
{
	if (NULL == p_spi_instance)
	{
		return -1;
	}

	if (HAL_OK != HAL_SPI_TransmitReceive(&p_spi_instance->hspi,
										  ptx, prx, (uint16_t)size, HAL_MAX_DELAY))
	{
		return -2;
	}

	return 0;
}

/******************************************************************************
 * @name    spi_transmit_dma
 * @brief   DMA中断方式发送字节流(启动+等待合并, 阻塞至完成/超时)
 * @param   p_spi_instance[in] SPI驱动实例
 * @param   pdata[in] 数据缓冲区
 * @param   size[in]  字节数
 *
 * @return  0 success (发送完成)
 *         -1 spi_instance null
 *         -2 dma tx handle null
 *         -3 wait interface null (OS: 信号量 / 裸机: 时基)
 *         -4 spi transmit dma error
 *         -5 wait error (OS: 信号量等待失败 / 裸机: 超时)
 *
 * @note    OS环境 → 阻塞等待信号量(ISR释放); 裸机 → 计数器轮询直到完成/超时
 *          OS分支的等待**无超时**: 时限由注入方的 pf_wait 自己决定(本工程
 *          main.c 里给的是 200ms, 见那里的注释)
 *****************************************************************************/
static int8_t spi_transmit_dma(spi_driver_t *p_spi_instance, uint8_t *pdata, uint32_t size)
{
	if (NULL == p_spi_instance)
	{
		return -1;
	}

	if (NULL == p_spi_instance->hspi.hdmatx)
	{
		return -2;
	}

#ifdef OS_SUPPORTING
	if (NULL == p_spi_instance->p_semaphore_interface ||
		NULL == p_spi_instance->p_semaphore_interface->pf_wait)
	{
		return -3;
	}
#else
	if (NULL == p_spi_instance->p_timebase_interface ||
		NULL == p_spi_instance->p_timebase_interface->pf_get_time)
	{
		return -3;
	}
#endif // OS_SUPPORTING

	/* 清完成标志 */
	p_spi_instance->tx_complete = 0;

#ifdef OS_SUPPORTING
	/* OS: 启动DMA后阻塞等待信号量(由HAL_SPI_TxCpltCallback释放) */
	if (HAL_OK != HAL_SPI_Transmit_DMA(&p_spi_instance->hspi,
									   pdata, (uint16_t)size))
	{
		return -4;
	}

	/* pf_wait() 无形参: 句柄与超时策略都在注入方(见其头文件注释) */
	if (0 != p_spi_instance->p_semaphore_interface->pf_wait())
	{
		return -5;
	}

	return 0;
#else
	/* 裸机: 记录启动时刻, 启动DMA后计数器轮询直到完成/超时 */
	p_spi_instance->tx_start_tick =
		p_spi_instance->p_timebase_interface->pf_get_time();

	if (HAL_OK != HAL_SPI_Transmit_DMA(&p_spi_instance->hspi,
									   pdata, (uint16_t)size))
	{
		return -4;
	}

	while (0 == p_spi_instance->tx_complete)
	{
		if ((p_spi_instance->p_timebase_interface->pf_get_time() -
			 p_spi_instance->tx_start_tick) >= p_spi_instance->cfg.tx_timeout_tick)
		{
			return -5; /* 超时 */
		}
	}

	return 0;
#endif // OS_SUPPORTING
}

/********************************* 构造与析构 *********************************/

/******************************************************************************
 * @name    spi_deinst
 * @brief   SPI驱动析构: 反初始化外设并清除函数指针
 * @param   p_spi_instance[in]
 *
 * @return  0 success
 *         -1 spi_instance null
 *****************************************************************************/
static int8_t spi_deinst(spi_driver_t *p_spi_instance)
{
	if (NULL == p_spi_instance)
	{
		return -1;
	}

	HAL_SPI_DeInit(&p_spi_instance->hspi);

#ifdef OS_SUPPORTING
	p_spi_instance->p_semaphore_interface = NULL;
#endif // OS_SUPPORTING
	p_spi_instance->p_delay_interface = NULL;
	p_spi_instance->p_timebase_interface = NULL;
	p_spi_instance->tx_complete = 0;
	p_spi_instance->tx_start_tick = 0;

	p_spi_instance->pf_inst = NULL;
	p_spi_instance->pf_deinst = NULL;
	p_spi_instance->pf_transmit = NULL;
	p_spi_instance->pf_receive = NULL;
	p_spi_instance->pf_transmit_receive = NULL;
	p_spi_instance->pf_transmit_dma = NULL;

	return 0;
}

/******************************************************************************
 * @name    spi_driver_inst
 * @brief   SPI驱动构造函数: 加载配置、注入接口、初始化内嵌句柄、挂载函数指针
 * @param   p_spi_instance[out]       SPI驱动实例
 * @param   p_cfg[in]                SPI硬件配置
 * @param   p_semaphore_interface[in] OS信号量接口(OS_SUPPORTING)
 * @param   p_delay_interface[in]     延时接口(由调用方注入)
 * @param   p_timebase_interface[in]  时基计数器接口
 *
 * @return  0 success
 *         -1 spi_instance null
 *         -2 cfg null
 *         -3 spi base null
 *         -4 semaphore null (OS_SUPPORTING)
 *         -5 delay null
 *         -6 timebase null
 *         -7 spi init error
 *****************************************************************************/
int8_t spi_driver_inst(spi_driver_t *p_spi_instance,
					   spi_cfg_t *p_cfg,
#ifdef OS_SUPPORTING
					   spi_semaphore_interface_t *p_semaphore_interface,
#endif // OS_SUPPORTING
					   spi_delay_interface_t *p_delay_interface,
					   spi_timebase_interface_t *p_timebase_interface)
{
	if (NULL == p_spi_instance)
	{
		return -1;
	}

	if (NULL == p_cfg)
	{
		return -2;
	}

	if (NULL == p_cfg->p_spi_base)
	{
		return -3;
	}

#ifdef OS_SUPPORTING
	if (NULL == p_semaphore_interface ||
		NULL == p_semaphore_interface->pf_wait ||
		NULL == p_semaphore_interface->pf_release)
	{
		return -4;
	}
#endif // OS_SUPPORTING

	if (NULL == p_delay_interface ||
		NULL == p_delay_interface->pf_delay_us)
	{
		return -5;
	}

	if (NULL == p_timebase_interface ||
		NULL == p_timebase_interface->pf_get_time)
	{
		return -6;
	}

	/* 加载硬件配置 */
	p_spi_instance->cfg = *p_cfg;

	/* 构建内嵌SPI句柄(hspi 为首个成员, 供DMA完成回调反查实例) */
	p_spi_instance->hspi.Instance = p_cfg->p_spi_base;
	p_spi_instance->hspi.Init = p_cfg->init;
	p_spi_instance->hspi.hdmatx = p_cfg->p_hdma_tx;
	p_spi_instance->hspi.hdmarx = NULL;
	if (NULL != p_cfg->p_hdma_tx)
	{
		/* 让DMA句柄通过Parent回指内嵌SPI句柄, 使DMA中断能反查到本实例 */
		p_cfg->p_hdma_tx->Parent = &p_spi_instance->hspi;
	}

	/* 初始化SPI外设 */
	if (HAL_OK != HAL_SPI_Init(&p_spi_instance->hspi))
	{
		return -7;
	}

	/* 注入接口 */
#ifdef OS_SUPPORTING
	p_spi_instance->p_semaphore_interface = p_semaphore_interface;
#endif // OS_SUPPORTING
	p_spi_instance->p_delay_interface = p_delay_interface;
	p_spi_instance->p_timebase_interface = p_timebase_interface;
	p_spi_instance->tx_complete = 0;
	p_spi_instance->tx_start_tick = 0;

	/* 挂载函数指针 (形参类型与头文件字段一致: spi_driver_t *, 可直接赋值;
	 * 若仍写成 void *, AC6 会以 incompatible-function-pointer-types **报错**) */
	p_spi_instance->pf_inst = spi_driver_inst;
	p_spi_instance->pf_deinst = spi_deinst;
	p_spi_instance->pf_transmit = spi_transmit;
	p_spi_instance->pf_receive = spi_receive;
	p_spi_instance->pf_transmit_receive = spi_transmit_receive;
	p_spi_instance->pf_transmit_dma = spi_transmit_dma;

	return 0;
}
