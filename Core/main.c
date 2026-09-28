/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "system/delay/delay.h"
#include "system/uart/uart.h"
#include "system/rtstats/rtstats.h" /* rtstats_start() */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "core_cm4.h"
#include "spi_hal.h"                /* LCD 的 SPI1 总线 */
#include "iic_hal.h"                /* 触摸的位带 I2C 总线 */
#include "cywatch_rtc.h"            /* 表盘页的日期/时钟来源 */
#include "cywatch_service_Lvgl.h"   /* service_lvgl_init() */
#include "cmsis_os2.h"              /* osKernelInitialize / osKernelStart / osSemaphoreNew */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 两条总线各只有一个使用者, 实例都在本层创建, adapter 只 extern 引用:
     1) SPI1(PA5=SCK / PA7=MOSI + TX DMA2_Stream3) → ST7789 LCD 独占(只发不收);
     2) PA8=SCL / PB4=SDA  位带 I2C               → CST816T 触摸 独占。
   @note PB4 复位后是 NJTRST: 当普通 GPIO 用会占掉 JTAG 的复位脚, SWD(PA13/PA14)
         不受影响, 调试器按 SW-DP(SWD) 连接即可 */

/* SPI1 TX DMA 句柄由 stm32f4xx_hal_msp.c 定义(引脚/DMA/NVIC 在 HAL_SPI_MspInit 里配) */
extern DMA_HandleTypeDef hdma_spi1_tx;

/* ---- LCD: SPI1 总线实例 ----
   参数与 CubeMX 生成的 MX_SPI1_Init() 逐项一致(Mode0, /2 → APB2 100MHz/2 = 50MHz)。 */
spi_driver_t lcd_spi_instance; /* 非 static: adapter 内 extern 引用 */
static spi_cfg_t lcd_spi_cfg =
{
    .p_spi_base = SPI1,
    .init =
    {
        .Mode              = SPI_MODE_MASTER,
        .Direction         = SPI_DIRECTION_2LINES, /* 面板只接 MOSI, 保留双线以复用驱动 */
        .DataSize          = SPI_DATASIZE_8BIT,
        .CLKPolarity       = SPI_POLARITY_LOW,  /* CPOL=0 */
        .CLKPhase          = SPI_PHASE_1EDGE,   /* CPHA=0 → Mode 0 */
        .NSS               = SPI_NSS_SOFT,      /* 片选由 adapter 用 LCD_CS_Pin 手动驱动 */
        .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2, /* APB2 100MHz/2 = 50MHz */
        .FirstBit          = SPI_FIRSTBIT_MSB,
        .TIMode            = SPI_TIMODE_DISABLE,
        .CRCCalculation    = SPI_CRCCALCULATION_DISABLE,
        .CRCPolynomial     = 10,
    },
    .p_hdma_tx       = &hdma_spi1_tx, /* 像素数据分块 DMA 发送 */
    .tx_timeout_tick = 200,           /* 仅裸机路径用; OS 路径的超时由信号量给 */
};
static spi_delay_interface_t lcd_spi_delay_instance =
{
    .pf_delay_us = delay_us,
};

/* LCD DMA 发送信号量: 带超时的真信号量。
   spi_hal 在 OS_SUPPORTING 下的 pf_transmit_dma 是"启动 DMA 后无限等信号量",
   而本工程没有 HAL_SPI_ErrorCallback: DMA 一旦出错, 完成回调永不到来, LVGL 任务
   会当场死等(整屏定格且无日志)。把 200ms 超时做进 pf_wait 后, 这条路径会退化成
   spi_hal 的超时返回码, 上层继续跑、日志可见。
   余量: 单次最大传输 8KB @50MHz ≈ 1.4ms, 200ms 是百倍 */
#define LCD_SPI_TX_TIMEOUT_MS   (200u)
static osSemaphoreId_t lcd_spi_sem_handle = NULL;

/* @note 句柄归本文件持有(不塞进接口): spi_semaphore_interface_t 的两个回调
         **没有形参、也没有实例成员**, 所以下面两个函数直接读这个静态句柄 */
static int8_t lcd_spi_sem_wait(void)
{
    if (NULL == lcd_spi_sem_handle)
    {
        return -1;
    }

    if (osOK != osSemaphoreAcquire(lcd_spi_sem_handle, LCD_SPI_TX_TIMEOUT_MS))
    {
        /* 超时: 顺手清掉可能"迟到"的完成令牌, 否则下一帧表观是"秒回成功"而实际
           只发了一半像素(局部花屏且不报错)。200ms 都没完成说明 DMA 已真错,
           迟到令牌不会再来, 这次零等待 acquire 不会偷到好数据 */
        (void)osSemaphoreAcquire(lcd_spi_sem_handle, 0u);
        return -1;
    }

    return 0;
}

/* 由 HAL_SPI_TxCpltCallback(DMA 中断上下文)调用: 只释放信号量 */
static int8_t lcd_spi_sem_release(void)
{
    if (NULL == lcd_spi_sem_handle)
    {
        return -1;
    }

    return (osOK == osSemaphoreRelease(lcd_spi_sem_handle)) ? 0 : -1;
}

static spi_semaphore_interface_t lcd_spi_semaphore_instance =
{
    .pf_wait    = lcd_spi_sem_wait,
    .pf_release = lcd_spi_sem_release,
};

/* ---- 触摸: 独立位带 I2C 总线实例(PA8=SCL, PB4=SDA) ----
   ⚠ 它是一条独立总线, 不与任何别的设备共用: CST816T 的读事务在 lvgl 任务里按
     ~33ms 周期跑, 若与别的设备共用同一条不可重入的位带总线, 就会被 tick 抢占切在
     start..stop 事务中间而互相踩总线 */
iic_driver_t touch_iic_instance; /* 非 static: adapter 内 extern 引用 */
static iic_bus_t touch_iic_bus_instance =
{
    .p_sda_port = GPIOB,
    .sda_pin    = GPIO_PIN_4,
    .p_scl_port = GPIOA,
    .scl_pin    = GPIO_PIN_8,
};
static iic_delay_interface_t touch_iic_delay_instance =
{
    .pf_delay_us = delay_us,
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	/* 必须在 SystemClock_Config() 之后: delay_init() 按当前主频换算DWT计数,
	   放在时钟配置前会拿到复位默认的16MHz, 延时偏快 */
	delay_init();
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */

	/* USER CODE BEGIN 2 */
	/* USART1: PA9=TX/PA10=RX, 115200 8N1. 初始化后 printf 即发往 PA9 */
    UART_Init();
	/* ===== 硬件 RTC: 表盘页的日期/时钟来源 =====
	   放最前面: 失败要尽早知道, 别先把 SPI/I2C/LCD 都配一遍再停。
	   与 FreeRTOS 无关(纯寄存器操作, 没有 osDelay), 所以能在 osKernelStart() 之前调。
	   耗时 ~10ms(LSE 已起振) ~ 5000ms(LSE 起不来, 等超时) */
	{
		int8_t rtc_rc = cywatch_rtc_init();
		if (0 != rtc_rc)
		{
			printf("\r\n[RTC] init failed, rc=%d\r\n", (int)rtc_rc);
			printf("[RTC] -1=LSE 起振失败(板上没焊 32.768kHz 晶振?) -2=RTCSEL 非 LSE "
				   "-3=HAL_RTC_Init 超时 -4=种初值失败 -5=回读不一致\r\n");
			/* 不停机: 时间恒为种子值 2025-01-01, 表盘页照常起来, 便于区分"RTC 挂"
			   和"LVGL 起不来"这两种都会黑屏的情况 */
			printf("[RTC] 不停机, 时间恒为 2025-01-01\r\n");
		}
		else
		{
			printf("[RTC] init ok(LSE 起振正常, 表盘页有真实日期/时钟)\r\n");
		}
	}

	/* ===== LVGL 总线 1: LCD 的 SPI1 =====
	   spi_driver_inst 只做参数校验 + 装配内嵌 hspi 句柄 + 挂函数指针, 不碰硬件, 也不依赖
	   调度器, 可在 osKernelStart 前调用。SPI 外设的真正初始化(HAL_SPI_Init → MspInit 配
	   引脚/DMA/NVIC)不在这里做: 由 ST7789T3 设备驱动在 st7789t3_init 里回调 spi 接口的
	   pf_init 触发, 那时已在 lvgl 任务体内 —— 调用链是 lv_port_disp_init →
	   lvgl_bsp_disp_inst() → st7789t3_inst() → st7789t3_init()(含各外设 init、硬件复位
	   与面板寄存器初始化, 内含 osDelay) */
	lcd_spi_sem_handle = osSemaphoreNew(1u, 0u, NULL); /* 二值语义: max=1, 初值=0 */
	if (NULL == lcd_spi_sem_handle)
	{
		Error_Handler(); /* 内核堆不足, 起不来 DMA 发送 */
	}
	if (0 != spi_driver_inst(&lcd_spi_instance,
	                         &lcd_spi_cfg,
	                         &lcd_spi_semaphore_instance,
	                         &lcd_spi_delay_instance))
	{
		Error_Handler(); /* SPI1 配置非法(基地址/信号量/延时) */
	}

	/* ===== LVGL 总线 2: CST816T 触摸的独立位带 I2C(PA8=SCL, PB4=SDA) =====
	   iic_driver_inst 只做参数校验 + 挂函数指针, 不碰硬件, 不含 I2C 时序, 也不依赖
	   调度器, 可在 osKernelStart 前调用。SDA/SCL 的真正配置不在这里做: 由 CST816T
	   设备驱动在 cst816t_init 里回调 iic 接口的 pf_init 触发 —— 调用链是
	   lv_port_indev_init → lvgl_bsp_indev_inst() → cst816t_inst() → cst816t_init()
	   (含各外设 init、RST 复位、寄存器配置与 ChipID 自检, 内含 osDelay) */
	/* 端口的时钟仍然必须在这里、在调度器起来之前开: pf_init 内部要配 PA8/PB4 的
	   MODER, 而端口时钟没开时对外设寄存器的写入会被直接丢弃, 引脚就停在复位默认态
	   —— PB4=NJTRST、PA15=JTDI 这类脚复位后归 JTAG 而不是 GPIO, 连 BSRR 都拉不动,
	   I2C 的 START 条件根本发不出来(表现为 cst816t_inst 失败、屏幕空白)。
	   本工程没有 MX_GPIO_Init(参考工程那份迁移时未搬入), 没人替我们开这两个时钟 */
	__HAL_RCC_GPIOA_CLK_ENABLE(); /* SCL = PA8 */
	__HAL_RCC_GPIOB_CLK_ENABLE(); /* SDA = PB4 */

	if (0 != iic_driver_inst(&touch_iic_instance,
	                         &touch_iic_bus_instance,
	                         &touch_iic_delay_instance))
	{
		Error_Handler(); /* 总线/延时接口参数非法 */
	}

	/* ===== FreeRTOS/CMSIS-RTOS v2: 只启动 LVGL 一个服务 =====
	   @note osKernelInitialize() 必须在任何 osThreadNew/osMessageQueueNew 之前;
	         service_lvgl_init() 只创建 "lvgl" 任务(不碰任何设备), 所以可以放在
	         osKernelStart() 之前 —— 设备实例化在两个 adapter 的 inst 里,
	         它们内含 osDelay, 必须等调度器跑起来 */
	osKernelInitialize();

	/* ===== LVGL 服务: 显示(ST7789 adapter) + 触摸(CST816T adapter) + UI 泵 =====
	   @warning 返回值必须看: 非 0 = "lvgl" 任务创建失败, 只可能是 FreeRTOS 内核堆不足
	            (24KB heap_4), 而 configASSERT 与 configUSE_MALLOC_FAILED_HOOK 都是关的
	            —— 分配失败是**静默**的, 表现为"屏幕全黑、串口一个字都没有" */
	{
		int8_t lvgl_rc = service_lvgl_init();
		if (0 != lvgl_rc)
		{
			printf("\r\n[LVGL] service_lvgl_init failed, rc=%d(内核堆不足?)\r\n", (int)lvgl_rc);
			Error_Handler();
		}
		printf("[LVGL] service init ok(\"lvgl\" 任务已建; lv_init/显示/触摸/UI 在任务体内做)\r\n");
		printf("[LVGL] 预期: 屏幕亮起并显示表盘页, 可左右/下滑切页\r\n");
	}

	/* ===== 运行统计打印任务: 每 5 秒打一次内核堆 + 任务表(栈水位) + CPU 占比 =====
	   时基 TIM5 由内核在 vTaskStartScheduler() 里启动(见 FreeRTOSConfig.h)
	   @note 这里失败**不** Error_Handler: 它只是调试辅助, 建不起来不该让手表停机,
	         打一行警告继续跑 */
	if (0 != rtstats_start())
	{
		printf("\r\n[RTSTATS] rtstats_start failed(内核堆不足?); 统计日志将缺失\r\n");
	}

	osKernelStart(); /* 成功进入调度器, 不再返回 */
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
