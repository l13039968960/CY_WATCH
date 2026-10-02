/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file uart_hal.h
 *
 * @par dependencies
 * - stm32f4xx_hal.h
 * - stdint.h
 *
 * @author zw1194
 *
 * @brief Provide the OO-style HAL APIs of hardware UART and corresponding opetions.
 *
 * Processing flow: uart_driver_inst() 构造(不碰硬件) -> pf_init() 起外设
 * -> pf_rx_start() 启动接收 -> 主循环用 pf_* 收发.
 * 环形缓冲由HAL层持有维护(内嵌于驱动实例); 不提供阻塞收发接口.
 *
 * @version V1.2
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
#ifndef __UART_HAL_H__
#define __UART_HAL_H__

/***********************************Includes***********************************/
#include "stm32f4xx_hal.h"
#include <stdint.h>
/***********************************Includes***********************************/

/***********************************Defines************************************/
/* 环形缓冲大小(由HAL层持有维护, 内嵌于驱动实例; 可在此处按需调整) */
#define UART_TX_RING_SIZE       256    /* 发送环形缓冲字节数(实际容量=size-1) */
#define UART_RX_RING_SIZE       512    /* 接收环形缓冲字节数(建议>=2*DMA缓冲) */
#define UART_RX_DMA_BUF_SIZE    128    /* 接收DMA直通缓冲字节数(须CIRCULAR) */
/***********************************Defines************************************/

/**********************************Declaring***********************************/
/* UART硬件配置 */
typedef struct
{
	USART_TypeDef *p_uart_base;   /* UART外设基地址(如USART1) */
	UART_InitTypeDef init;        /* UART初始化参数(波特率/数据位/校验/停止位等) */
	DMA_HandleTypeDef *p_hdma_tx; /* TX DMA句柄(须NORMAL模式, 环发送用) */
	DMA_HandleTypeDef *p_hdma_rx; /* RX DMA句柄(须CIRCULAR模式, IDLE接收用) */
} uart_cfg_t;

typedef struct
{
	int8_t (*pf_wait)(void);    /* 阻塞等待信号量 */
	int8_t (*pf_release)(void); /* 释放信号量(ISR中调用) */
} uart_semaphore_interface_t;

/* 延时接口 (由调用方注入, 与IIC/GPIO/SPI/EXTI一致) */
typedef struct
{
	void (*pf_delay_us)(uint32_t us);      /* 微秒延时 */
} uart_delay_interface_t;

/* UART驱动对象 */
typedef struct uart_driver
{
	/* 第一个成员: 内嵌UART句柄(按值), 用于在HAL回调中container_of反查本实例 */
	UART_HandleTypeDef huart;

	uart_cfg_t cfg; /* 硬件配置 */

	/* 环形缓冲(由HAL层持有维护, 大小见UART_*_SIZE宏; 实例须为全局变量) */
	uint8_t tx_ring_buf[UART_TX_RING_SIZE];   /* 发送环形缓冲 */
	uint8_t rx_ring_buf[UART_RX_RING_SIZE];   /* 接收环形缓冲 */
	uint8_t rx_dma_buf[UART_RX_DMA_BUF_SIZE]; /* 接收DMA直通缓冲(CIRCULAR) */

	uart_semaphore_interface_t *p_tx_semaphore_interface; /* OS发送完成信号量(TC回调释放) */
	uart_semaphore_interface_t *p_rx_semaphore_interface; /* OS接收事件信号量(RxEvent回调释放) */
	uart_delay_interface_t *p_delay_interface;         /* 延时接口(由调用方注入) */

	/* 环形缓冲区状态(单生产者-单消费者, 无需加锁) */
	volatile uint16_t tx_ring_head; /* TX环写索引(主循环写) */
	volatile uint16_t tx_ring_tail; /* TX环读索引(DMA发送完成推进, ISR写) */
	volatile uint16_t tx_dma_len;   /* 在途DMA发送长度(TX环连续段) */
	volatile uint8_t  tx_busy;      /* TX DMA发送忙标志 */
	volatile uint16_t rx_ring_head; /* RX环写索引(ISR写) */
	volatile uint16_t rx_ring_tail; /* RX环读索引(主循环读) */
	uint16_t rx_dma_pos;            /* 已从DMA直通缓冲搬运到的位置(ISR自用) */
	volatile uint32_t rx_overflow_cnt; /* RX环溢出丢弃字节计数 */

	uint8_t init_state; /* 初始化状态: 0=deinit, 1=init */
	uint8_t ref_count;  /* 使用本外设的实例数量: 0→1 才真正初始化, 减到 0 才真正反初始化 */

	/* 构造与析构 */
	int8_t (*pf_inst)(struct uart_driver *p_uart_instance,
					  uart_cfg_t *p_cfg,
					  uart_semaphore_interface_t *p_tx_semaphore_interface,
					  uart_semaphore_interface_t *p_rx_semaphore_interface,
					  uart_delay_interface_t *p_delay_interface);
	int8_t (*pf_deinst)(struct uart_driver *p_uart_instance);

	int8_t (*pf_init)(struct uart_driver *p_uart_instance);
	int8_t (*pf_deinit)(struct uart_driver *p_uart_instance);

	int8_t   (*pf_tx_send)(struct uart_driver *p_uart_instance, uint8_t *pdata, uint32_t size); /* 写TX环并触发DMA发送 */
	uint32_t (*pf_tx_get_free)(struct uart_driver *p_uart_instance);   /* TX环剩余可写字节 */

	int8_t   (*pf_rx_read)(struct uart_driver *p_uart_instance, uint8_t *pdata, uint32_t size); /* 读RX环 */
	uint32_t (*pf_rx_get_count)(struct uart_driver *p_uart_instance);  /* RX环可读字节 */
	int8_t   (*pf_rx_start)(struct uart_driver *p_uart_instance);      /* 启动IDLE+DMA循环接收 */

	/* 事件等待接口 (阻塞等待; ★timeout_ms 本实现忽略★; 严禁在ISR中调用) */
	int8_t   (*pf_wait_txcplt)(struct uart_driver *p_uart_instance, uint32_t timeout_ms); /* 等TX整批发完(flush) */
	int8_t   (*pf_wait_rxcplt)(struct uart_driver *p_uart_instance, uint32_t timeout_ms); /* 等RX新数据入环 */

	/* 中断入口(应用ISR调用) */
	void (*pf_irq_handler)(struct uart_driver *p_uart_instance);        /* USARTx_IRQHandler: IDLE/TC等 */
	void (*pf_dma_tx_irq_handler)(struct uart_driver *p_uart_instance); /* DMA TX流IRQ */
	void (*pf_dma_rx_irq_handler)(struct uart_driver *p_uart_instance); /* DMA RX流IRQ */
} uart_driver_t;

/* UART驱动构造函数 */
int8_t uart_driver_inst(uart_driver_t *p_uart_instance,
						uart_cfg_t *p_cfg,
						uart_semaphore_interface_t *p_tx_semaphore_interface,
						uart_semaphore_interface_t *p_rx_semaphore_interface,
						uart_delay_interface_t *p_delay_interface);

/**********************************Declaring***********************************/

#endif // __UART_HAL_H__
