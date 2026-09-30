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
/* SPI1 TX DMA 句柄: 非 static, main.c 的 spi_cfg_t 与 it.c 的 DMA 中断都要它 */
DMA_HandleTypeDef hdma_spi1_tx;
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

/**
  * @brief  SPI 的 MSP 初始化: SPI1 服务 LCD(ST7789T3), SPI2 服务 W25Q64
  *
  *         SPI1: PA5=SCK / PA7=MOSI 配成 AF5; 同时把 TX DMA(DMA2_Stream3_Channel3)
  *         建好并链接到 SPI 句柄 —— 像素数据由 spi_hal 的 pf_transmit_dma 分块发。
  *         SPI2: PB13=SCK / PB14=MISO / PB15=MOSI 配成 AF5, 无 DMA(阻塞传输)。
  *
  * @note DMA 抢占优先级 5 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(5),
  *       所以 HAL_SPI_TxCpltCallback 里释放信号量是合法的 FromISR 调用。
  *       若把这个优先级调到 5 以下(数值更大), 会直接触发 FreeRTOS 断言。
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef *spiHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (spiHandle->Instance == SPI1)
  {
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = LCD_SCL_Pin | LCD_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* SPI1 TX DMA Init: DMA2_Stream3_Channel3, 用于像素数据中断发送 */
    __HAL_RCC_DMA2_CLK_ENABLE();

    hdma_spi1_tx.Instance = DMA2_Stream3;
    hdma_spi1_tx.Init.Channel = DMA_CHANNEL_3;
    hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_tx.Init.Mode = DMA_NORMAL;
    hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle, hdmatx, hdma_spi1_tx);

    /* SPI1 TX DMA中断使能 */
    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  }
  else if (spiHandle->Instance == SPI2)
  {
    /* SPI2 的 MSP 初始化: W25Q64 独占的存储总线
       PB13=SCK / PB14=MISO / PB15=MOSI 配成 AF5; CS=PB12 是普通 GPIO,
       由 adapter 的 w25q64_cs_gpio_init 配置(不在这里)。

       @note 不走 DMA: W25Q64 的读写都是阻塞传输(spi_hal 的 pf_transmit /
             pf_receive), 单次最长 256B(页编程), 无需 DMA 与信号量。
       @note 调用时机: 由 W25Q64 驱动在 w25q64_init 里回调 spi 接口的 pf_init 触发
             (adapter 转发到 spi2_instance.pf_init), 详见 cywatch_adapter_w25q64.c。 */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI2 GPIO Configuration
    PB13    ------> SPI2_SCK
    PB14    ------> SPI2_MISO
    PB15    ------> SPI2_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *spiHandle)
{
  if (spiHandle->Instance == SPI1)
  {
    __HAL_RCC_SPI1_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOA, LCD_SCL_Pin | LCD_SDA_Pin);

    HAL_DMA_DeInit(&hdma_spi1_tx);
  }
  else if (spiHandle->Instance == SPI2)
  {
    __HAL_RCC_SPI2_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
  }
}

/**
  * @brief  ADC 的 MSP 初始化: ADC1 服务 ADKEY(PA2 = ADC1_IN2)
  *
  * @note 调用时机: 由 ADKEY adapter 在 key_bsp_inst → adkey_inst → adc_init 里
  *       回调 HAL_ADC_Init 触发, 那时已在 "key" 任务体内。
  * @note 模拟输入只配 GPIO_MODE_ANALOG: 上下拉与速度在模拟模式下无意义, 一律不配。
  *       本工程没有 MX_GPIO_Init, GPIOA 时钟必须在这里开 —— 没开时对 PA2 的配置
  *       会被静默丢弃, 引脚停在复位态, ADC 读回的是一根悬空脚。
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (hadc->Instance == ADC1)
  {
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA2     ------> ADC1_IN2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    __HAL_RCC_ADC1_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2);
  }
}

/* USER CODE END 1 */
