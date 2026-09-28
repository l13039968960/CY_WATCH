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
 * @brief USART1 init (PA9=TX, PA10=RX, 115200 8N1) and printf retarget.
 *
 * Processing flow:
 * call UART_Init() in main() before the first printf. printf 无闸门: 每个字符直接
 * 走 HAL_UART_Transmit。若在 UART_Init() 之前调 printf, huart1.gState 还是 RESET,
 * HAL 会直接返回而不发送(不会崩)。
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "uart.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>

UART_HandleTypeDef huart1;

/* printf 重定向. AC5(armcc)/AC6(armclang)/GCC 统一走 fputc(MicroLIB printf 钩子).
   AC6 定义 __GNUC__, 旧 __GNUC__ 分支会误选 _io_putchar 钩子使 printf 静默失效, 故去掉编译分支 */
int fputc(int ch, FILE *f)
{
    uint8_t b = (uint8_t)ch;

    (void)HAL_UART_Transmit(&huart1, &b, 1, 0xFFFF);

    return ch;
}

/* AC6(armclang)+MicroLIB 备选钩子; 未被引用时链接器自动丢弃, 无副作用.
   直接发送不走 fputc(ch,NULL): 标准库将 FILE* 标 nonnull, 传 NULL 会触发 -Wnonnull */
int __io_putchar(int ch)
{
    uint8_t b = (uint8_t)ch;

    (void)HAL_UART_Transmit(&huart1, &b, 1, 0xFFFF);

    return ch;
}
/* USER CODE END 0 */

/* USART1 init function */

void UART_Init(void)
{
  /* USER CODE BEGIN USART1_Init 0 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  /**USART1 GPIO Configuration
  PA9     ------> USART1_TX
  PA10    ------> USART1_RX
  */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */

}


/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
