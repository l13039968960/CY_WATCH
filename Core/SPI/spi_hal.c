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
static int8_t spi_init(spi_driver_t *p_spi_instance);
static int8_t spi_deinit(spi_driver_t *p_spi_instance);
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
	/* 裸机: 清完成标志, 记录启动时刻, 启动DMA后计数器轮询直到完成/超时 */
	p_spi_instance->tx_complete = 0;
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
 * @name    spi_init
 * @brief   SPI驱动初始化: 初始化SPI外设到运行态
 * @param   p_spi_instance[in]
 *
 * @return  0 success
 *         -1 spi_instance null
 *         -2 spi init error
 *
 * @note    引脚(PA5/PA7)、TX DMA、NVIC与SPI1时钟都在 HAL_SPI_MspInit 里配置,
 *          由 HAL_SPI_Init 内部回调完成, 本函数不用重复开时钟。
 *
 * @note    引用计数: ref_count 由 0→1 时才真正初始化外设, 已有使用者时只累加计数
 *          (同一实例被重复init不会把外设重配一遍)。
 *****************************************************************************/
static int8_t spi_init(spi_driver_t *p_spi_instance)
{
	if (NULL == p_spi_instance)
	{
		return -1;
	}

	/* 首个使用者才真正初始化SPI外设(MspInit里配引脚/DMA/NVIC/时钟) */
	if (0 == p_spi_instance->ref_count)
	{
		if (HAL_OK != HAL_SPI_Init(&p_spi_instance->hspi))
		{
			return -2;
		}

		p_spi_instance->init_state = 1;
	}

	p_spi_instance->ref_count++;

	return 0;
}

/******************************************************************************
 * @name    spi_deinit
 * @brief   SPI驱动反初始化: 关闭SPI外设
 * @param   p_spi_instance[in]
 *
 * @return  0 success
 *         -1 spi_instance null
 *
 * @note    HAL_SPI_DeInit 会回调 HAL_SPI_MspDeInit: 关SPI1时钟、DeInit PA5/PA7、
 *          HAL_DMA_DeInit。SPI1 是LCD独占, 关时钟不会误伤其他外设。
 *
 * @note    引用计数: 每个使用者退出时减1, 减到0才真正关闭外设。
 *****************************************************************************/
static int8_t spi_deinit(spi_driver_t *p_spi_instance)
{
	if (NULL == p_spi_instance)
	{
		return -1;
	}

	/* 退出一个使用者(已在0则不再减, 防uint8下溢后永远回不到0) */
	if (p_spi_instance->ref_count > 0)
	{
		p_spi_instance->ref_count--;
	}

	/* 最后一个使用者退出时才真正关闭SPI外设 */
	if (0 == p_spi_instance->ref_count)
	{
		HAL_SPI_DeInit(&p_spi_instance->hspi);

		p_spi_instance->init_state = 0;
	}

	return 0;
}

/******************************************************************************
 * @name    spi_deinst
 * @brief   SPI驱动析构: 清除函数指针
 * @param   p_spi_instance[in]
 *
 * @return  0 success
 *         -1 spi_instance null
 *         -2 仍有使用者(ref_count != 0), 本次不析构
 *
 * @note    本函数不关外设 —— 关闭SPI外设走pf_deinit。析构会清空pf_transmit等
 *          全部指针, 等于废掉整条总线, 所以只在没有使用者时才允许执行。
 *****************************************************************************/
static int8_t spi_deinst(spi_driver_t *p_spi_instance)
{
	if (NULL == p_spi_instance)
	{
		return -1;
	}

	/* 仍有使用者: 清指针会把复用本实例的其他设备一起废掉 */
	if (0 != p_spi_instance->ref_count)
	{
		return -2;
	}

	p_spi_instance->p_semaphore_interface = NULL;
	p_spi_instance->p_delay_interface = NULL;
#ifndef OS_SUPPORTING
	p_spi_instance->p_timebase_interface = NULL;
	p_spi_instance->tx_complete = 0;
	p_spi_instance->tx_start_tick = 0;
#endif // OS_SUPPORTING

	p_spi_instance->init_state = 0;
	p_spi_instance->ref_count = 0;

	p_spi_instance->pf_inst = NULL;
	p_spi_instance->pf_deinst = NULL;
	p_spi_instance->pf_init = NULL;
	p_spi_instance->pf_deinit = NULL;
	p_spi_instance->pf_transmit = NULL;
	p_spi_instance->pf_receive = NULL;
	p_spi_instance->pf_transmit_receive = NULL;
	p_spi_instance->pf_transmit_dma = NULL;

	return 0;
}

/******************************************************************************
 * @name    spi_driver_inst
 * @brief   SPI驱动构造函数: 加载配置、装配内嵌句柄、注入接口、挂载函数指针
 *
 *          本函数不碰硬件, SPI外设的真正初始化在pf_init()里做。
 *
 * @param   p_spi_instance[out]       SPI驱动实例
 * @param   p_cfg[in]                SPI硬件配置
 * @param   p_semaphore_interface[in] OS信号量接口(两种模式都收, 裸机模式下不参与发送)
 * @param   p_delay_interface[in]     延时接口(由调用方注入)
 * @param   p_timebase_interface[in]  时基计数器接口(仅裸机模式, OS_SUPPORTING 下不传)
 *
 * @return  0 success
 *         -1 spi_instance null
 *         -2 cfg null
 *         -3 spi base null
 *         -4 semaphore null
 *         -5 delay null
 *         -6 timebase null (仅裸机模式, OS_SUPPORTING 下不会返回)
 *****************************************************************************/
int8_t spi_driver_inst(spi_driver_t *p_spi_instance,
					   spi_cfg_t *p_cfg,
					   spi_semaphore_interface_t *p_semaphore_interface,
					   spi_delay_interface_t *p_delay_interface
#ifndef OS_SUPPORTING
					   , spi_timebase_interface_t *p_timebase_interface
#endif // OS_SUPPORTING
)
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

	if (NULL == p_semaphore_interface ||
		NULL == p_semaphore_interface->pf_wait ||
		NULL == p_semaphore_interface->pf_release)
	{
		return -4;
	}

	if (NULL == p_delay_interface ||
		NULL == p_delay_interface->pf_delay_us)
	{
		return -5;
	}

#ifndef OS_SUPPORTING
	if (NULL == p_timebase_interface ||
		NULL == p_timebase_interface->pf_get_time)
	{
		return -6;
	}
#endif // OS_SUPPORTING

	/* 加载硬件配置 */
	p_spi_instance->cfg = *p_cfg;

	/* 实例状态: 尚未初始化, 无使用者 */
	p_spi_instance->init_state = 0;
	p_spi_instance->ref_count = 0;

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

	/* 注入接口 */
	p_spi_instance->p_semaphore_interface = p_semaphore_interface;
	p_spi_instance->p_delay_interface = p_delay_interface;
#ifndef OS_SUPPORTING
	p_spi_instance->p_timebase_interface = p_timebase_interface;
	p_spi_instance->tx_complete = 0;
	p_spi_instance->tx_start_tick = 0;
#endif // OS_SUPPORTING

	/* 挂载函数指针 (形参类型与头文件字段一致: spi_driver_t *, 可直接赋值;
	 * 若仍写成 void *, AC6 会以 incompatible-function-pointer-types **报错**) */
	p_spi_instance->pf_inst = spi_driver_inst;
	p_spi_instance->pf_deinst = spi_deinst;
	p_spi_instance->pf_init = spi_init;
	p_spi_instance->pf_deinit = spi_deinit;
	p_spi_instance->pf_transmit = spi_transmit;
	p_spi_instance->pf_receive = spi_receive;
	p_spi_instance->pf_transmit_receive = spi_transmit_receive;
	p_spi_instance->pf_transmit_dma = spi_transmit_dma;

	return 0;
}
