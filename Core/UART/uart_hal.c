/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file uart_hal.c
 *
 * @par dependencies
 * - uart_hal.h
 *
 * @author zw1194
 *
 * @brief Implete the OO-style HAL operations of hardware UART.
 *
 * Processing flow: uart_driver_inst() 构造(不碰硬件) -> pf_init() 起外设 ->
 * pf_rx_start() 启动 IDLE+DMA 接收 -> 应用 ISR 调 pf_*_irq_handler ->
 * 主循环 pf_rx_read()/pf_tx_send() 收发, pf_wait_* 做同步.
 *
 * 硬件接线要求(不满足会丢数据/丢事件):
 * - TX DMA流须 NORMAL 模式, RX DMA流须 CIRCULAR 模式;
 * - RX DMA流中断优先级必须高于 USART 中断优先级.
 *
 * @version V1.2
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
#include "uart_hal.h"

/********************************* 前向声明 *********************************/
static int8_t uart_init(uart_driver_t *p_uart_instance);
static int8_t uart_deinit(uart_driver_t *p_uart_instance);
static int8_t uart_deinst(uart_driver_t *p_uart_instance);
static int8_t uart_tx_kick(uart_driver_t *p_uart_instance);
static int8_t uart_wait_txcplt(uart_driver_t *p_uart_instance, uint32_t timeout_ms);
static int8_t uart_wait_rxcplt(uart_driver_t *p_uart_instance, uint32_t timeout_ms);

/* 底层UART操作 (无阻塞收发) */
static int8_t uart_tx_send(uart_driver_t *p_uart_instance, uint8_t *pdata, uint32_t size);
static uint32_t uart_tx_get_free(uart_driver_t *p_uart_instance);
static int8_t uart_rx_read(uart_driver_t *p_uart_instance, uint8_t *pdata, uint32_t size);
static uint32_t uart_rx_get_count(uart_driver_t *p_uart_instance);
static int8_t uart_rx_start(uart_driver_t *p_uart_instance);
static void uart_irq_handler(uart_driver_t *p_uart_instance);

/********************************* 私有辅助 *********************************/

/******************************************************************************
 * @name    uart_clk_enable
 * @brief   开本 UART 外设的时钟(引脚时钟由 gpio_hal 负责)
 * @param   p_uart[in] UART外设基地址
 *
 * @note    只能按基地址分派: RCC 的时钟使能宏连寄存器位都是编译期固定的
 *          (__HAL_RCC_USART1_CLK_ENABLE 直接写 APB2ENR 的 USART1EN), HAL 没有
 *          "按基地址查位" 的通用 API.
 *****************************************************************************/
static void uart_clk_enable(USART_TypeDef *p_uart)
{
	if (USART1 == p_uart)      { __HAL_RCC_USART1_CLK_ENABLE(); }
	else if (USART2 == p_uart) { __HAL_RCC_USART2_CLK_ENABLE(); }
	else if (USART6 == p_uart) { __HAL_RCC_USART6_CLK_ENABLE(); }
}

/******************************************************************************
 * @name    uart_clk_disable
 * @brief   关本 UART 的时钟(pf_deinit 收尾用)
 * @param   p_uart[in] UART外设基地址
 *
 * @note    ★只关 UART 外设时钟, 不关 GPIO 端口时钟★ —— 端口上还有 ADC/EXTI/IIC
 *          共用, 连端口时钟一起关会把它们一起打死.
 *****************************************************************************/
static void uart_clk_disable(USART_TypeDef *p_uart)
{
	if (USART1 == p_uart)      { __HAL_RCC_USART1_CLK_DISABLE(); }
	else if (USART2 == p_uart) { __HAL_RCC_USART2_CLK_DISABLE(); }
	else if (USART6 == p_uart) { __HAL_RCC_USART6_CLK_DISABLE(); }
}

/********************************* 环形缓冲操作 *********************************/

/******************************************************************************
 * @brief   向环形缓冲区写入cnt字节(调用方保证空间足够), 返回新写索引
 *****************************************************************************/
static uint16_t uart_ring_write(uint8_t *p_buf, uint16_t size, uint16_t head,
								const uint8_t *p_src, uint16_t cnt)
{
	uint16_t i;

	for (i = 0; i < cnt; i++)
	{
		p_buf[head] = p_src[i];
		head = (uint16_t)((head + 1) % size);
	}

	return head;
}

/******************************************************************************
 * @brief   从环形缓冲区读出cnt字节(调用方保证有数据), 返回新读索引
 *****************************************************************************/
static uint16_t uart_ring_read(uint8_t *p_buf, uint16_t size, uint16_t tail,
							   uint8_t *p_dst, uint16_t cnt)
{
	uint16_t i;

	for (i = 0; i < cnt; i++)
	{
		p_dst[i] = p_buf[tail];
		tail = (uint16_t)((tail + 1) % size);
	}

	return tail;
}

/******************************************************************************
 * @brief   环形缓冲区剩余可写字节数(空余一格, 容量=size-1)
 *****************************************************************************/
static uint16_t uart_ring_free(uint16_t size, uint16_t head, uint16_t tail)
{
	int32_t free = (int32_t)tail - (int32_t)head - 1 + size;

	if (free >= size)
	{
		free -= size;
	}

	return (uint16_t)free;
}

/******************************************************************************
 * @brief   环形缓冲区可读字节数
 *****************************************************************************/
static uint16_t uart_ring_count(uint16_t size, uint16_t head, uint16_t tail)
{
	return (uint16_t)(((int32_t)head - (int32_t)tail + size) % size);
}

/********************************* 发送实现 *********************************/

/******************************************************************************
 * @brief   从TX环取一段连续数据启动DMA发送; 环空则清busy
 * @return  0 成功 / -1 实例空 / -2 启动DMA失败(内部已复位环与busy)
 *****************************************************************************/
static int8_t uart_tx_kick(uart_driver_t *p_uart_instance)
{
	uint16_t size, head, tail, count, chunk;
	uint8_t *p_start;

	if (NULL == p_uart_instance)
	{
		return -1;
	}

	size  = UART_TX_RING_SIZE;
	head  = p_uart_instance->tx_ring_head;
	tail  = p_uart_instance->tx_ring_tail;
	count = uart_ring_count(size, head, tail);

	if (0 == count)
	{
		/* 环空, 发送结束 */
		p_uart_instance->tx_busy = 0;
		return 0;
	}

	/* 只发送连续一段, 跨环尾部分由下一次TC回调续传 */
	chunk   = (tail < head) ? (uint16_t)(head - tail) : (uint16_t)(size - tail);
	p_start = &p_uart_instance->tx_ring_buf[tail];
	p_uart_instance->tx_dma_len = chunk;
	p_uart_instance->tx_busy = 1;

	if (HAL_OK != HAL_UART_Transmit_DMA(&p_uart_instance->huart, p_start, chunk))
	{
		/* 启动失败: 丢弃未发数据, 复位为空闲 */
		p_uart_instance->tx_ring_tail = head;
		p_uart_instance->tx_dma_len = 0;
		p_uart_instance->tx_busy = 0;
		return -2;
	}

	return 0;
}

/******************************************************************************
 * @brief   将数据写入TX环并触发DMA发送(主循环调用, 非阻塞)
 * @return  0 成功 / -1 实例空 / -2 pdata空 / -3 size超环容量 / -4 TX环空间不足
 * @note    空间不足整包拒绝(-4), 可先查 pf_tx_get_free();
 *          先更新head再查busy, 保证ISR能取到新数据(无加锁设计)
 *****************************************************************************/
static int8_t uart_tx_send(uart_driver_t *p_uart_instance, uint8_t *pdata, uint32_t size)
{
	uint16_t ring_size, n, seg;

	if (NULL == p_uart_instance)
	{
		return -1;
	}

	if ((NULL == pdata) && (size > 0))
	{
		return -2;
	}

	ring_size = UART_TX_RING_SIZE;

	if (size > (uint32_t)(ring_size - 1))
	{
		return -3;
	}

	if (size > (uint32_t)uart_ring_free(ring_size,
										p_uart_instance->tx_ring_head,
										p_uart_instance->tx_ring_tail))
	{
		return -4;
	}

	/* 写入TX环(可能跨环尾); 先更新head再检查busy, 保证ISR可见新数据 */
	n = (uint16_t)size;
	seg = (uint16_t)((p_uart_instance->tx_ring_head + n <= ring_size) ?
					 n : (ring_size - p_uart_instance->tx_ring_head));
	p_uart_instance->tx_ring_head =
		uart_ring_write(p_uart_instance->tx_ring_buf, ring_size,
						p_uart_instance->tx_ring_head, pdata, seg);
	if (n > seg)
	{
		p_uart_instance->tx_ring_head =
			uart_ring_write(p_uart_instance->tx_ring_buf, ring_size,
							p_uart_instance->tx_ring_head, &pdata[seg],
							(uint16_t)(n - seg));
	}

	/* DMA空闲则立即触发发送 */
	if (0 == p_uart_instance->tx_busy)
	{
		(void)uart_tx_kick(p_uart_instance);
	}

	return 0;
}

/********************************* 接收实现 *********************************/

/******************************************************************************
 * @brief   启动IDLE+DMA循环接收(RX DMA须CIRCULAR模式; 构造后调用一次)
 * @return  0 成功(或已在接收中) / -1 实例空 / -2 启动失败
 * @note    错误回调中断内也会自动重启
 *****************************************************************************/
static int8_t uart_rx_start(uart_driver_t *p_uart_instance)
{
	if (NULL == p_uart_instance)
	{
		return -1;
	}

	if (HAL_UART_STATE_BUSY_RX == p_uart_instance->huart.RxState)
	{
		return 0; /* 已在接收中 */
	}

	p_uart_instance->rx_dma_pos = 0;

	if (HAL_OK != HAL_UARTEx_ReceiveToIdle_DMA(&p_uart_instance->huart,
												p_uart_instance->rx_dma_buf,
												UART_RX_DMA_BUF_SIZE))
	{
		return -2;
	}

	return 0;
}

/******************************************************************************
 * @brief   从RX环读数据(主循环调用, 非阻塞; 实际读取=min(size, 可读))
 * @return  0 成功 / -1 实例空 / -2 pdata空; 未读尽不算错误
 *****************************************************************************/
static int8_t uart_rx_read(uart_driver_t *p_uart_instance, uint8_t *pdata, uint32_t size)
{
	uint16_t ring_size, tail, cnt;

	if (NULL == p_uart_instance)
	{
		return -1;
	}

	if ((NULL == pdata) && (size > 0))
	{
		return -2;
	}

	ring_size = UART_RX_RING_SIZE;
	tail = p_uart_instance->rx_ring_tail;
	cnt  = uart_ring_count(ring_size, p_uart_instance->rx_ring_head, tail);

	if (size > cnt)
	{
		size = cnt;
	}

	if (size > 0)
	{
		p_uart_instance->rx_ring_tail =
			uart_ring_read(p_uart_instance->rx_ring_buf, ring_size,
						   tail, pdata, (uint16_t)size);
	}

	return 0;
}

/******************************************************************************
 * @brief   重写HAL弱函数: IDLE/半传输/写满接收事件回调(中断上下文)
 *         huart = uart_driver_t 首个成员; Size = DMA直通缓冲当前写入位置
 *         (写满=缓冲长度, 半传输=长度/2, IDLE=已收字节)
 * @note    只做内存搬运; 要求RX DMA流中断优先级高于USART, 事件才能按序处理
 *****************************************************************************/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	uart_driver_t *p_uart = (uart_driver_t *)huart;
	uint16_t buf_size, ring_size, start, n, seg, free, head;

	if (0 == Size)
	{
		return;
	}

	buf_size  = UART_RX_DMA_BUF_SIZE;
	ring_size = UART_RX_RING_SIZE;
	start     = p_uart->rx_dma_pos;
	head      = p_uart->rx_ring_head;

	/* 计算本次新到达字节数(相对上次搬运位置) */
	n = (uint16_t)((Size + buf_size - start) % buf_size);
	if (0 == n)
	{
		return;
	}

	/* 按RX环剩余空间截断, 丢弃超出的部分 */
	free = uart_ring_free(ring_size, head, p_uart->rx_ring_tail);
	if (n > free)
	{
		p_uart->rx_overflow_cnt += (uint32_t)(n - free);
		n = free;
	}

	/* 从DMA直通缓冲[start]搬运n字节进RX环(源/目的都可能跨环尾) */
	seg = (uint16_t)((start + n <= buf_size) ? n : (buf_size - start));
	head = uart_ring_write(p_uart->rx_ring_buf, ring_size, head,
						   &p_uart->rx_dma_buf[start], seg);
	if (n > seg)
	{
		head = uart_ring_write(p_uart->rx_ring_buf, ring_size, head,
							   &p_uart->rx_dma_buf[0], (uint16_t)(n - seg));
	}

	p_uart->rx_ring_head = head;
	p_uart->rx_dma_pos = (uint16_t)((start + n) % buf_size);

	/* 释放RX事件信号量, 通知等待新数据的任务(pf_wait_rxcplt) */
	if ((NULL != p_uart->p_rx_semaphore_interface) &&
		(NULL != p_uart->p_rx_semaphore_interface->pf_release))
	{
		(void)p_uart->p_rx_semaphore_interface->pf_release();
	}
}

/********************************* 中断入口 *********************************/

/******************************************************************************
 * @brief   USARTx_IRQHandler 入口: 处理IDLE/TC/错误等UART中断
 *****************************************************************************/
static void uart_irq_handler(uart_driver_t *p_uart_instance)
{
	if (NULL == p_uart_instance)
	{
		return;
	}

	HAL_UART_IRQHandler(&p_uart_instance->huart);
}

/********************************* HAL回调 *********************************/

/******************************************************************************
 * @brief   重写HAL弱函数: UART TX发送完成回调(USART TC中断上下文)
 * @note    推进TX环读索引并续传下一段, 再释放TX信号量; 中断内不做耗时操作
 *****************************************************************************/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	uart_driver_t *p_uart = (uart_driver_t *)huart;

	/* 推进读索引到已发送的连续段之后 */
	p_uart->tx_ring_tail = (uint16_t)((p_uart->tx_ring_tail + p_uart->tx_dma_len)
										% UART_TX_RING_SIZE);
	p_uart->tx_dma_len = 0;

	/* 若环中仍有数据, 续传下一段 */
	(void)uart_tx_kick(p_uart);

	/* 释放TX完成信号量, 通知发送任务(pf_wait_txcplt): 段完成可续写环 */
	if ((NULL != p_uart->p_tx_semaphore_interface) &&
		(NULL != p_uart->p_tx_semaphore_interface->pf_release))
	{
		(void)p_uart->p_tx_semaphore_interface->pf_release();
	}
}

/******************************************************************************
 * @brief   重写HAL弱函数: UART/DMA错误回调(中断上下文)
 * @note    清错误标志后, 若DMA传输错误(RX流已停)直接重启IDLE接收;
 *          不调阻塞的 HAL_UART_Abort, 遵循中断纪律
 *****************************************************************************/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	uart_driver_t *p_uart = (uart_driver_t *)huart;

	/* 清UART错误标志: 过载/帧/噪声/校验 */
	__HAL_UART_CLEAR_FLAG(huart,
						  UART_FLAG_ORE | UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_PE);

	/* DMA传输错误: TX复位发送状态; RX(若已停止)直接重启 */
	if ((huart->ErrorCode & HAL_UART_ERROR_DMA) != 0U)
	{
		p_uart->tx_busy = 0;
		p_uart->tx_dma_len = 0;
		p_uart->tx_ring_tail = p_uart->tx_ring_head; /* 丢弃未发完数据 */

		if (HAL_UART_STATE_READY == p_uart->huart.RxState)
		{
			p_uart->rx_dma_pos = 0;
			(void)HAL_UARTEx_ReceiveToIdle_DMA(&p_uart->huart,
												p_uart->rx_dma_buf,
												UART_RX_DMA_BUF_SIZE);
		}
	}

	huart->ErrorCode = HAL_UART_ERROR_NONE;
}

/********************************* 查询接口 *********************************/

/******************************************************************************
 * @brief   查询TX环剩余可写字节(实例空则0)
 *****************************************************************************/
static uint32_t uart_tx_get_free(uart_driver_t *p_uart_instance)
{
	if (NULL == p_uart_instance)
	{
		return 0;
	}

	return (uint32_t)uart_ring_free(UART_TX_RING_SIZE,
									p_uart_instance->tx_ring_head,
									p_uart_instance->tx_ring_tail);
}

/******************************************************************************
 * @brief   查询RX环可读字节(实例空则0)
 *****************************************************************************/
static uint32_t uart_rx_get_count(uart_driver_t *p_uart_instance)
{
	if (NULL == p_uart_instance)
	{
		return 0;
	}

	return (uint32_t)uart_ring_count(UART_RX_RING_SIZE,
									 p_uart_instance->rx_ring_head,
									 p_uart_instance->rx_ring_tail);
}

/********************************* 事件等待接口 *********************************/

/******************************************************************************
 * @brief   阻塞等待TX整批发完(flush): 等TX环空且DMA停, 即此前所有send数据全部上线
 * @return  0 成功 / -1 实例空
 * @note    ★timeout_ms 本实现忽略★(信号量等待无超时参数, 与 spi_hal 同);
 *          每次唤醒后重查真实状态, 计数信号量下自校正, 不会误返回/死锁;
 *          严禁在ISR上下文调用
 *****************************************************************************/
static int8_t uart_wait_txcplt(uart_driver_t *p_uart_instance, uint32_t timeout_ms)
{
	(void)timeout_ms;

	if (NULL == p_uart_instance)
	{
		return -1;
	}

	while ((0 != uart_ring_count(UART_TX_RING_SIZE,
								  p_uart_instance->tx_ring_head,
								  p_uart_instance->tx_ring_tail)) ||
		   (0 != p_uart_instance->tx_busy))
	{
		/* 阻塞等TC完成信号量 */
		if ((NULL != p_uart_instance->p_tx_semaphore_interface) &&
			(NULL != p_uart_instance->p_tx_semaphore_interface->pf_wait))
		{
			(void)p_uart_instance->p_tx_semaphore_interface->pf_wait();
		}
	}

	return 0;
}

/******************************************************************************
 * @brief   阻塞等待RX新数据入环(至少一次RxEvent搬入新字节)
 * @return  0 成功 / -1 实例空
 * @note    ★timeout_ms 本实现忽略★(同 pf_wait_txcplt); 返回后由调用方用
 *          pf_rx_get_count()/pf_rx_read() 消费数据; 严禁在ISR上下文调用
 *****************************************************************************/
static int8_t uart_wait_rxcplt(uart_driver_t *p_uart_instance, uint32_t timeout_ms)
{
	(void)timeout_ms;

	if (NULL == p_uart_instance)
	{
		return -1;
	}

	while (0 == uart_ring_count(UART_RX_RING_SIZE,
								p_uart_instance->rx_ring_head,
								p_uart_instance->rx_ring_tail))
	{
		/* 阻塞等RX事件信号量 */
		if ((NULL != p_uart_instance->p_rx_semaphore_interface) &&
			(NULL != p_uart_instance->p_rx_semaphore_interface->pf_wait))
		{
			(void)p_uart_instance->p_rx_semaphore_interface->pf_wait();
		}
	}

	return 0;
}

/********************************* 构造与析构 *********************************/

/******************************************************************************
 * @brief   UART驱动初始化: 初始化UART外设到运行态
 * @return  0 成功 / -1 实例空 / -2 初始化失败
 * @note    UART 的时钟与引脚都在本函数里配, 不走 ST 的 MSP 回调
 *          (HAL_UART_MspInit 未实现, HAL 自带的那个空弱函数不会动 GPIO).
 * @note    ★与 Core/system/uart/uart.c 的 UART_Init() 是重复配置★: 那一处用全局
 *          huart1 服务 printf, 必须留着。两处配的是同一组引脚、同样的参数, 且
 *          RCC 使能是置位幂等, 重复执行没有副作用。
 * @note    TX/RX DMA 流与它们的 NVIC 由 adapter 的 nordic_bsp_dma_init() 配,
 *          必须在本函数之前跑完(DMA 句柄由 cfg 注入)。
 * @note    引用计数: ref_count 由 0→1 时才真正初始化外设(重复init不会重配一遍)
 *****************************************************************************/
static int8_t uart_init(uart_driver_t *p_uart_instance)
{
	if (NULL == p_uart_instance)
	{
		return -1;
	}

	/* 首个使用者才真正初始化UART外设 */
	if (0 == p_uart_instance->ref_count)
	{
		/* 时钟与引脚都在这里配: 后面 HAL_UART_Init 内部那次 HAL_UART_MspInit
		   是空弱函数, 不会再动 GPIO —— 所以这一步不能省, 也不能挪到后面 */
		uart_clk_enable(p_uart_instance->cfg.p_uart_base);

		(void)p_uart_instance->gpio.pf_init(&p_uart_instance->gpio);

		if (HAL_OK != HAL_UART_Init(&p_uart_instance->huart))
		{
			return -2;
		}

		p_uart_instance->init_state = 1;
	}

	p_uart_instance->ref_count++;

	return 0;
}

/******************************************************************************
 * @brief   UART驱动反初始化: 关闭UART外设, 并把引脚置低功耗态(模拟输入)
 * @return  0 成功 / -1 实例空
 * @note    HAL_UART_DeInit 回调的 MspDeInit 是空弱函数, 引脚与时钟由本函数自己收尾.
 * @note    本函数有**低功耗语义**: 引脚不能只 DeInit 到复位态 ——
 *          F4 的 GPIO 复位值是**浮空输入**(不是模拟), 施密特触发器还开着, 悬空脚
 *          会随噪声来回翻转、白耗电。要再配成模拟输入才算真的关掉输入缓冲。
 * @note    ★DMA 流与 NVIC 不在这里停★: 那些由调用方(adapter 的
 *          nordic_bsp_deinst)自己 DisableIRQ / DeInit, 见其 @note。
 * @note    调用本函数后 PA9/PA10 不再是串口, printf 跟着失效 —— 本函数只在
 *          nordic 服务初始化失败的路径上被调, 正常流程不受影响。
 * @note    引用计数: 每个使用者退出时减1, 减到0才真正关闭外设
 *****************************************************************************/
static int8_t uart_deinit(uart_driver_t *p_uart_instance)
{
	if (NULL == p_uart_instance)
	{
		return -1;
	}

	/* 退出一个使用者(已在0则不再减, 防uint8下溢后永远回不到0) */
	if (p_uart_instance->ref_count > 0)
	{
		p_uart_instance->ref_count--;
	}

	/* 最后一个使用者退出时才真正关闭UART外设 */
	if (0 == p_uart_instance->ref_count)
	{
		HAL_UART_DeInit(&p_uart_instance->huart);

		/* 引脚收尾(DeInit → 模拟输入, 顺序在 gpio_hal 里) */
		(void)p_uart_instance->gpio.pf_deinit(&p_uart_instance->gpio);

		/* ★只关 UART 时钟★: 端口上还有 ADC/EXTI/IIC 共用, 端口时钟不能关 */
		uart_clk_disable(p_uart_instance->cfg.p_uart_base);

		p_uart_instance->init_state = 0;
	}

	return 0;
}

/******************************************************************************
 * @brief   UART驱动析构: 清空全部函数指针与环形缓冲状态
 * @return  0 成功 / -1 实例空 / -2 仍有使用者(ref_count!=0), 本次不析构
 * @note    本函数不关外设(关外设走 pf_deinit); 清指针会废掉整条链路, 故仅在无使用者时执行
 *****************************************************************************/
static int8_t uart_deinst(uart_driver_t *p_uart_instance)
{
	if (NULL == p_uart_instance)
	{
		return -1;
	}

	/* 仍有使用者: 清指针会把复用本实例的其他链路一起废掉 */
	if (0 != p_uart_instance->ref_count)
	{
		return -2;
	}

	p_uart_instance->p_tx_semaphore_interface = NULL;
	p_uart_instance->p_rx_semaphore_interface = NULL;
	p_uart_instance->p_delay_interface = NULL;
	p_uart_instance->tx_ring_head = 0;
	p_uart_instance->tx_ring_tail = 0;
	p_uart_instance->tx_dma_len = 0;
	p_uart_instance->tx_busy = 0;
	p_uart_instance->rx_ring_head = 0;
	p_uart_instance->rx_ring_tail = 0;
	p_uart_instance->rx_dma_pos = 0;
	p_uart_instance->rx_overflow_cnt = 0;

	p_uart_instance->init_state = 0;
	p_uart_instance->ref_count = 0;

	p_uart_instance->pf_inst = NULL;
	p_uart_instance->pf_deinst = NULL;
	p_uart_instance->pf_init = NULL;
	p_uart_instance->pf_deinit = NULL;
	p_uart_instance->pf_tx_send = NULL;
	p_uart_instance->pf_tx_get_free = NULL;
	p_uart_instance->pf_rx_read = NULL;
	p_uart_instance->pf_rx_get_count = NULL;
	p_uart_instance->pf_rx_start = NULL;
	p_uart_instance->pf_wait_txcplt = NULL;
	p_uart_instance->pf_wait_rxcplt = NULL;
	p_uart_instance->pf_irq_handler = NULL;

	return 0;
}

/******************************************************************************
 * @brief   UART驱动构造函数: 加载配置、装配内嵌句柄、注入接口、挂载函数指针
 *          (不碰硬件; 外设真正初始化在 pf_init() 里做)。
 *          p_uart_instance 须为全局变量(环形缓冲内嵌其中)。
 * @return  0 成功 / -1 实例空 / -2 cfg空 / -3 基地址空 / -4 TX DMA空
 *          / -5 RX DMA空 / -6 信号量接口空 / -7 延时接口空 / -8 GPIO端口空
 * @note    构造后由应用调 pf_init() 起外设、pf_rx_start() 启动接收;
 *          TX/RX DMA流须由应用先行初始化(见文件头接线要求)
 *****************************************************************************/
int8_t uart_driver_inst(uart_driver_t *p_uart_instance,
						uart_cfg_t *p_cfg,
						uart_semaphore_interface_t *p_tx_semaphore_interface,
						uart_semaphore_interface_t *p_rx_semaphore_interface,
						uart_delay_interface_t *p_delay_interface)
{
	if (NULL == p_uart_instance)
	{
		return -1;
	}

	if (NULL == p_cfg)
	{
		return -2;
	}

	if (NULL == p_cfg->p_uart_base)
	{
		return -3;
	}

	if (NULL == p_cfg->p_hdma_tx)
	{
		return -4;
	}

	if (NULL == p_cfg->p_hdma_rx)
	{
		return -5;
	}

	if ((NULL == p_tx_semaphore_interface) ||
		(NULL == p_tx_semaphore_interface->pf_wait) ||
		(NULL == p_tx_semaphore_interface->pf_release) ||
		(NULL == p_rx_semaphore_interface) ||
		(NULL == p_rx_semaphore_interface->pf_wait) ||
		(NULL == p_rx_semaphore_interface->pf_release))
	{
		return -6;
	}

	if (NULL == p_delay_interface ||
		NULL == p_delay_interface->pf_delay_us)
	{
		return -7;
	}

	/* 引脚驱动(端口空/非法由 gpio_hal 拦) */
	if (0 != gpio_driver_inst(&p_uart_instance->gpio, &p_cfg->gpio))
	{
		return -8;
	}

	/* 加载硬件配置 */
	p_uart_instance->cfg = *p_cfg;

	/* 构建内嵌UART句柄(huart为首个成员, 供回调container_of反查实例) */
	p_uart_instance->huart.Instance = p_cfg->p_uart_base;
	p_uart_instance->huart.Init = p_cfg->init;
	p_uart_instance->huart.hdmatx = p_cfg->p_hdma_tx;
	p_uart_instance->huart.hdmarx = p_cfg->p_hdma_rx;
	/* Parent回指内嵌句柄, 让DMA中断能反查到本实例(两个DMA句柄上面已判非空) */
	p_cfg->p_hdma_tx->Parent = &p_uart_instance->huart;
	p_cfg->p_hdma_rx->Parent = &p_uart_instance->huart;

	/* 注入接口 */
	p_uart_instance->p_tx_semaphore_interface = p_tx_semaphore_interface;
	p_uart_instance->p_rx_semaphore_interface = p_rx_semaphore_interface;
	p_uart_instance->p_delay_interface = p_delay_interface;
	p_uart_instance->init_state = 0;
	p_uart_instance->ref_count = 0;
	p_uart_instance->tx_ring_head = 0;
	p_uart_instance->tx_ring_tail = 0;
	p_uart_instance->tx_dma_len = 0;
	p_uart_instance->tx_busy = 0;
	p_uart_instance->rx_ring_head = 0;
	p_uart_instance->rx_ring_tail = 0;
	p_uart_instance->rx_dma_pos = 0;
	p_uart_instance->rx_overflow_cnt = 0;

	/* 挂载函数指针 (无阻塞收发) */
	p_uart_instance->pf_inst = uart_driver_inst;
	p_uart_instance->pf_deinst = uart_deinst;
	p_uart_instance->pf_init = uart_init;
	p_uart_instance->pf_deinit = uart_deinit;
	p_uart_instance->pf_tx_send = uart_tx_send;
	p_uart_instance->pf_tx_get_free = uart_tx_get_free;
	p_uart_instance->pf_rx_read = uart_rx_read;
	p_uart_instance->pf_rx_get_count = uart_rx_get_count;
	p_uart_instance->pf_rx_start = uart_rx_start;
	p_uart_instance->pf_wait_txcplt = uart_wait_txcplt;
	p_uart_instance->pf_wait_rxcplt = uart_wait_rxcplt;
	p_uart_instance->pf_irq_handler = uart_irq_handler;

	return 0;
}
