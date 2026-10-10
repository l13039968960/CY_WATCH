/******************************************************************************
 * Copyright (C) 2026 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file uart.c
 *
 * @par dependencies
 * - uart.h
 * - stm32f4xx_hal.h
 *
 * @author zw1194
 *
 * @brief ★日志口★ USART6 init (PA11=TX, PA12=RX, 115200 8N1) and printf retarget.
 *
 * Processing flow:
 * call UART_Init() in main() before the first printf. printf 无闸门: 每个字符直接
 * 走 HAL_UART_Transmit。若在 UART_Init() 之前调 printf, huart6.gState 还是 RESET,
 * HAL 会直接返回而不发送(不会崩)。
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note ★printf 走 USART6 而不是 USART1★: USART1(PA9/PA10) 整条让给 Nordic 链路,
 *       两个口分开后 printf 不再往协议线里插字节。选 PA11/PA12(AF8) 而不是 PC6/PC7:
 *       UFQFPN48 没引出 PC6/PC7。PA11/PA12 的另一复用是 USB DM/DP, 本工程不用 USB。
 *****************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "uart.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>

UART_HandleTypeDef huart6;      /* 日志口(printf). 全工程只有本文件引用它 */

/* printf 重定向. AC5(armcc)/AC6(armclang)/GCC 统一走 fputc(MicroLIB printf 钩子).
   AC6 定义 __GNUC__, 旧 __GNUC__ 分支会误选 _io_putchar 钩子使 printf 静默失效, 故去掉编译分支 */
int fputc(int ch, FILE *f)
{
    uint8_t b = (uint8_t)ch;

    (void)HAL_UART_Transmit(&huart6, &b, 1, 0xFFFF);

    return ch;
}

/* AC6(armclang)+MicroLIB 备选钩子; 未被引用时链接器自动丢弃, 无副作用.
   直接发送不走 fputc(ch,NULL): 标准库将 FILE* 标 nonnull, 传 NULL 会触发 -Wnonnull */
int __io_putchar(int ch)
{
    uint8_t b = (uint8_t)ch;

    (void)HAL_UART_Transmit(&huart6, &b, 1, 0xFFFF);

    return ch;
}
/* USER CODE END 0 */

/* 日志口 init function */

void UART_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_USART6_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  /**USART6 GPIO Configuration
  PA11     ------> USART6_TX
  PA12     ------> USART6_RX
  */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
}


/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
