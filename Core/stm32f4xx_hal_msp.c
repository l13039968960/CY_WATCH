/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32f4xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/* USER CODE BEGIN 1 */

/* 注意: 本工程**故意不实现** HAL_SPI_MspInit/MspDeInit、HAL_ADC_MspInit/MspDeInit、
   HAL_TIM_PWM_MspInit/MspDeInit、HAL_UART_MspInit/MspDeInit。
   这几个外设的时钟、引脚、(SPI 的)DMA 流与 NVIC 全在各自驱动里做:
     Core/SPI/spi_hal.c  的 spi_init / spi_deinit
     Core/ADC/adc_hal.c  的 adc_init / adc_deinit
     Core/PWM/pwm_hal.c  的 pwm_init / pwm_deinit
     Core/UART/uart_hal.c 的 uart_init / uart_deinit
   做法都是按基地址分派开时钟 + HAL_GPIO_Init, HAL 自带的空弱函数留在原位。
   UART 是唯一的例外: Core/system/uart/uart.c 的 UART_Init() 也在配同一组引脚
   (printf 要用那个全局 huart1), 两处参数一致、RCC 使能置位幂等, 重复无害。 */

/* USER CODE END 1 */
