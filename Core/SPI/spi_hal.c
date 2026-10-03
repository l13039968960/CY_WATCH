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
static int8_t spi_receive_dma(spi_driver_t *p_spi_instance, uint8_t *pdata, uint32_t size);

/********************************* 私有辅助 *********************************/

/******************************************************************************
 * @name    spi_clk_enable
 * @brief   开本 SPI 外设的时钟(引脚时钟由 gpio_hal 负责)
 * @param   p_spi[in]  SPI外设基地址
 *
 * @note    只能按基地址分派: RCC 的时钟使能宏连寄存器位都是编译期固定的
 *          (__HAL_RCC_SPI1_CLK_ENABLE 直接写 APB2ENR 的 SPI1EN), HAL 没有
 *          "按基地址查位" 的通用 API.
 *****************************************************************************/
static void spi_clk_enable(SPI_TypeDef *p_spi)
{
	if (SPI1 == p_spi)      { __HAL_RCC_SPI1_CLK_ENABLE(); }
	else if (SPI2 == p_spi) { __HAL_RCC_SPI2_CLK_ENABLE(); }
	else if (SPI3 == p_spi) { __HAL_RCC_SPI3_CLK_ENABLE(); }
}

/******************************************************************************
 * @name    spi_clk_disable
 * @brief   关本 SPI 外设的时钟(pf_deinit 收尾用)
 * @param   p_spi[in] SPI外设基地址
 *
 * @note    ★只关 SPI 外设时钟★: GPIO 端口与 DMA 控制器都是多外设共用的
 *          (GPIOA 上还有 ADC/EXTI/UART, DMA2 上还有 USART1 的两条流),
 *          连它们一起关会把别的设备一起打死.
 *****************************************************************************/
static void spi_clk_disable(SPI_TypeDef *p_spi)
{
	if (SPI1 == p_spi)      { __HAL_RCC_SPI1_CLK_DISABLE(); }
	else if (SPI2 == p_spi) { __HAL_RCC_SPI2_CLK_DISABLE(); }
	else if (SPI3 == p_spi) { __HAL_RCC_SPI3_CLK_DISABLE(); }
}

/******************************************************************************
 * @name    spi_dma_init_stream
 * @brief   交给 dma_hal 装配一条流, 再把句柄链到内嵌的 hspi 上
 * @param   p_spi_instance[in]
 * @param   p_dma_cfg[in]    该方向的流描述
 * @param   p_dma_driver[in] 驱动内嵌的流实例(dma_tx 或 dma_rx)
 * @param   p_hdma[in]       驱动内嵌的DMA句柄(hdma_tx 或 hdma_rx)
 * @param   is_tx[in]        1=发送 0=接收(只决定往 hspi 的哪个槽挂)
 *
 * @return  0 success
 *         -1 该方向未配(p_stream 为 NULL), 不建流
 *         -2 dma_hal 装配失败
 *         -3 dma_hal 建流失败
 *
 * @note    Parent 与 Link 留在本层: DMA 中断要靠 Parent 反查到本实例,
 *          再由 HAL 转成 HAL_SPI_TxCpltCallback / RxCpltCallback.
 *****************************************************************************/
static int8_t spi_dma_init_stream(spi_driver_t *p_spi_instance,
								  dma_cfg_t *p_dma_cfg,
								  dma_driver_t *p_dma_driver,
								  DMA_HandleTypeDef *p_hdma,
								  uint8_t is_tx)
{
	if (NULL == p_dma_cfg->p_stream)
	{
		return -1;
	}

	if (0 != dma_driver_inst(p_dma_driver, p_hdma, p_dma_cfg))
	{
		return -2;
	}

	if (0 != p_dma_driver->pf_init(p_dma_driver))
	{
		return -3;
	}

	/* 回指内嵌SPI句柄: DMA中断靠它反查到本实例 */
	p_hdma->Parent = &p_spi_instance->hspi;

	if (0 != is_tx)
	{
		p_spi_instance->hspi.hdmatx = p_hdma;
	}
	else
	{
		p_spi_instance->hspi.hdmarx = p_hdma;
	}

	return 0;
}

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

/******************************************************************************
 * @name    HAL_SPI_RxCpltCallback
 * @brief   重写HAL弱函数: SPI RX DMA完成回调(在DMA中断上下文执行)
 * @param   hspi[in] SPI句柄(即 spi_driver_t 首个成员 &p_spi->hspi)
 *
 * @note    与 TxCplt 同理由 container_of 反查实例, 中断内只释放信号量.
 * @note    2LINES+MASTER 下 HAL_SPI_Receive_DMA 实际走的是 TransmitReceive,
 *          该路径只把这个回调挂到 hdmarx 上、hdmatx 的完成回调被置 NULL,
 *          所以收发两侧不会串到对方的信号量上.
 *****************************************************************************/
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	/* container_of: hspi 位于实例偏移0处, 强转即得实例指针 */
	spi_driver_t *p_spi = (spi_driver_t *)hspi;

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

/******************************************************************************
 * @name    spi_receive_dma
 * @brief   DMA中断方式接收字节流(启动+等待合并, 阻塞至完成/超时)
 * @param   p_spi_instance[in] SPI驱动实例
 * @param   pdata[out] 数据缓冲区
 * @param   size[in]   字节数
 *
 * @return  0 success (接收完成)
 *         -1 spi_instance null
 *         -2 dma rx handle null (cfg.dma_rx 没配, 或 pf_init 没调过)
 *         -3 semaphore interface null
 *         -4 spi receive dma error
 *         -5 wait error (信号量等待失败/超时)
 *
 * @note    本工程是 OS 环境(OS_SUPPORTING), 等待走注入的信号量, 完成回调在
 *          HAL_SPI_RxCpltCallback. 超时时限由注入方的 pf_wait 决定(本工程 200ms).
 * @note    返回时 HAL 已完成收尾(等到 BSY 清零), 片选可以安全拉高.
 *****************************************************************************/
static int8_t spi_receive_dma(spi_driver_t *p_spi_instance, uint8_t *pdata, uint32_t size)
{
	if (NULL == p_spi_instance)
	{
		return -1;
	}

	if (NULL == p_spi_instance->hspi.hdmarx)
	{
		return -2;
	}

	if ((NULL == p_spi_instance->p_semaphore_interface) ||
		(NULL == p_spi_instance->p_semaphore_interface->pf_wait))
	{
		return -3;
	}

	if (HAL_OK != HAL_SPI_Receive_DMA(&p_spi_instance->hspi,
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
 * @note    SPI/GPIO/DMA 的时钟、引脚、两条DMA流与它们的NVIC都在本函数里配,
 *          不走 ST 的 MSP 回调(该回调已从 stm32f4xx_hal_msp.c 删除).
 *          顺序: 时钟 → 引脚 → DMA流+NVIC → HAL_SPI_Init.
 *          ★不用手工 __HAL_SPI_ENABLE★: HAL 在启动 DMA 传输时自己会开.
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

	/* 首个使用者才真正初始化SPI外设 */
	if (0 == p_spi_instance->ref_count)
	{
		/* 时钟与引脚都在这里配: 后面 HAL_SPI_Init 内部那次 HAL_SPI_MspInit
		   是空弱函数, 不会再动 GPIO —— 所以这一步不能省, 也不能挪到后面 */
		spi_clk_enable(p_spi_instance->cfg.p_spi_base);

		(void)p_spi_instance->gpio.pf_init(&p_spi_instance->gpio);

		/* 两条流: 未配的方向(p_stream=NULL)内部直接跳过、不动句柄(句柄在构造时
		   已清成NULL). 建流失败不回滚 —— 与 PWM/ADC 一样, 失败由调用方的传输
		   函数以 -2 暴露出来 */
		(void)spi_dma_init_stream(p_spi_instance, &p_spi_instance->cfg.dma_tx,
								  &p_spi_instance->dma_tx,
								  &p_spi_instance->hdma_tx, 1);
		(void)spi_dma_init_stream(p_spi_instance, &p_spi_instance->cfg.dma_rx,
								  &p_spi_instance->dma_rx,
								  &p_spi_instance->hdma_rx, 0);

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
 * @brief   SPI驱动反初始化: 停DMA、关闭SPI外设, 并把引脚置低功耗态(模拟输入)
 * @param   p_spi_instance[in]
 *
 * @return  0 success
 *         -1 spi_instance null
 *
 * @note    HAL_SPI_DeInit 回调的 MspDeInit 是空弱函数, 引脚/时钟/DMA 由本函数自己收尾.
 * @note    本函数有**低功耗语义**: 引脚不能只 DeInit 到复位态 ——
 *          F4 的 GPIO 复位值是**浮空输入**(不是模拟), 施密特触发器还开着, 悬空脚
 *          会随噪声来回翻转、白耗电。要再配成模拟输入才算真的关掉输入缓冲。
 * @note    ★只关 SPI 外设时钟★: GPIO 端口与 DMA 控制器都是多外设共用的,
 *          关 GPIOA/GPIOB 会打死 PA2(ADC)/PA9,PA10(UART)/PB12(CS) 等,
 *          关 DMA2 会打死 USART1 的两条流。
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
		/* 先停 DMA 再关 SPI: 反过来的话 DMA 请求还挂在 SPI 上, 关外设时钟时
		   那些请求没有应答方, 流会卡在使能态 */
		if (NULL != p_spi_instance->hspi.hdmatx)
		{
			(void)p_spi_instance->dma_tx.pf_deinit(&p_spi_instance->dma_tx);
			p_spi_instance->hspi.hdmatx = NULL;
		}

		if (NULL != p_spi_instance->hspi.hdmarx)
		{
			(void)p_spi_instance->dma_rx.pf_deinit(&p_spi_instance->dma_rx);
			p_spi_instance->hspi.hdmarx = NULL;
		}

		HAL_SPI_DeInit(&p_spi_instance->hspi);

		/* 引脚收尾(DeInit → 模拟输入, 顺序在 gpio_hal 里) */
		(void)p_spi_instance->gpio.pf_deinit(&p_spi_instance->gpio);

		spi_clk_disable(p_spi_instance->cfg.p_spi_base);

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
	p_spi_instance->pf_receive_dma = NULL;

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
 *         -7 gpio port null
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

	/* 引脚驱动(端口空/非法由 gpio_hal 拦) */
	if (0 != gpio_driver_inst(&p_spi_instance->gpio, &p_cfg->gpio))
	{
		return -7;
	}

	/* 加载硬件配置 */
	p_spi_instance->cfg = *p_cfg;

	/* 实例状态: 尚未初始化, 无使用者 */
	p_spi_instance->init_state = 0;
	p_spi_instance->ref_count = 0;

	/* 构建内嵌SPI句柄(hspi 为首个成员, 供DMA完成回调反查实例).
	   hdmatx/hdmarx 先清空: 两条DMA流由 pf_init 建好后回填(未配的方向保持NULL) */
	p_spi_instance->hspi.Instance = p_cfg->p_spi_base;
	p_spi_instance->hspi.Init = p_cfg->init;
	p_spi_instance->hspi.hdmatx = NULL;
	p_spi_instance->hspi.hdmarx = NULL;

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
	p_spi_instance->pf_receive_dma = spi_receive_dma;

	return 0;
}
