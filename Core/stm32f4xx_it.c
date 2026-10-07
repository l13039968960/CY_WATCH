/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "exti_hal.h"
#include "dma_hal.h"   /* DMA 中断统一转给 dma_hal 按流分发 */

/* FreeRTOS 端口的真实 ISR(portable/GCC/ARM_CM4F/port.c), 头文件无声明, 在此补原型 */
extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */
  /* FreeRTOS: SVC 异常由内核端口处理(启动首个任务/任务切换的软中断通道) */
  vPortSVCHandler();
  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */
  /* FreeRTOS: PendSV 承载上下文切换, 转发到内核端口实现 */
  xPortPendSVHandler();
  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  /* HAL 1ms 时基: HAL_Delay() / HAL_GetTick() 依赖此调用 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  /* FreeRTOS 与 HAL 共用本 SysTick(HAL 时基 1ms 照走): 调度器启动后把 tick 喂给内核.
     HAL_InitTick 与 xPortStartScheduler 都会写 SysTick 重装值, 两者同为
     configTICK_RATE_HZ=1kHz(100MHz/1000-1), 配置一致故共用无冲突 */
  if (taskSCHEDULER_NOT_STARTED != xTaskGetSchedulerState())
  {
    xPortSysTickHandler();
  }
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f411xe.s).                  */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/**
  * @brief DMA2_Stream3 中断: SPI1(LCD) TX 完成
  *
  *        统一转给 dma_hal 按流反查句柄, 最终调 HAL_SPI_TxCpltCallback 释放 LCD 信号量。
  */
void DMA2_Stream3_IRQHandler(void)
{
  dma_irq_handler(DMA2_Stream3);
}

/**
  * @brief DMA1_Stream3 中断: SPI2(W25Q64) RX 完成
  *
  *        统一转给 dma_hal 按流反查句柄, 最终调 HAL_SPI_RxCpltCallback 释放 W25Q64 信号量。
  */
void DMA1_Stream3_IRQHandler(void)
{
  dma_irq_handler(DMA1_Stream3);
}

/**
  * @brief DMA1_Stream4 中断: SPI2(W25Q64) TX 完成
  *
  *        出空字节用: 全双工主模式下 HAL 的接收走 TransmitReceive, 发送侧的完成
  *        回调被置 NULL(不释放任何信号量), 但流本身仍要挂中断, 否则标志位不清、
  *        DMA 报 TE/HT 错误后流会停死。
  */
void DMA1_Stream4_IRQHandler(void)
{
  dma_irq_handler(DMA1_Stream4);
}

/**
  * @brief EXTI2 中断: CST816T 触摸 INT(PB2)
  *
  *        转发到 exti_hal 分发(按线号 2 反查实例) → CST816T 驱动 pf_interrupt_cb,
  *        仅释放信号量, 不做任何 I2C/printf(软件 I2C 不可重入); 实际触摸数据
  *        由 lvgl 任务内的 LVGL 触摸读回调取走。
  */
void EXTI2_IRQHandler(void)
{
  exti_irq_handler(GPIO_PIN_2);
}

/**
  * @brief EXTI3 中断: MAX30102 INT(PA3, FIFO 满下降沿)
  *
  *        转发到 exti_hal 分发(按线号 3 反查实例) → main.c 的 max30102_exti_cb
  *        → adapter heartrate_bsp_interrupt_cb() → 驱动 pf_interrupt_cb:
  *        只释放信号量, 不做任何 I2C/printf(软件 I2C 不可重入).
  *        FIFO 数据由心率服务任务醒来后用软件 I2C 读走.
  *
  * @note 优先级 6 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(5):
  *       ISR 内会调 osSemaphoreRelease(FromISR 安全 API)
  */
void EXTI3_IRQHandler(void)
{
  exti_irq_handler(GPIO_PIN_3);
}

/**
  * @brief EXTI1 中断: 充电检测 PB1(充电时低电平, 双边沿)
  *
  *        转发到 exti_hal 分发(按线号 1 反查实例) → power adapter 的
  *        power_chg_exti_cb: 只 osSemaphoreRelease, 不读电平、不发事件
  *        (事件入队口用 osKernelLock, 中断里用不了).
  *        电源服务被唤醒后读 PB1 真值, 翻转才发 EVT_SERVICE_POWER_CHARGING.
  *
  * @note 优先级 6 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(5):
  *       ISR 内会调 osSemaphoreRelease(中断安全 API)
  */
void EXTI1_IRQHandler(void)
{
  exti_irq_handler(GPIO_PIN_1);
}

/* USER CODE END 1 */
