/******************************************************************************
 * Copyright (C) 2026 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file uart.h
 *
 * @par dependencies
 * - main.h
 *
 * @author zw1194
 *
 * @brief Declare the USART1 init and printf retarget interfaces.
 *
 * Processing flow:
 *
 * call MX_USART1_UART_Init() in main(), then use printf() freely.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
#ifndef __UART_H__
#define __UART_H__

/***********************************Includes***********************************/
/* 引入 stm32f4xx_hal.h (UART_HandleTypeDef / uint8_t) 与 Error_Handler 声明 */
#include "main.h"

/***********************************Includes***********************************/

/**********************************Declaring***********************************/
/* USART1 初始化(PA9=TX, PA10=RX, 115200 8N1), 需在首次 printf 前调用 */
void UART_Init(void);

/**********************************Declaring***********************************/

#endif // __UART_H__
