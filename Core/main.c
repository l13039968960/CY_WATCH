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
#include "system/log/cywatch_log.h" /* log_init() / log_printf() */
#include "system/rtstats/rtstats.h" /* rtstats_start() */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "core_cm4.h"
#include "spi_hal.h"                /* LCD 的 SPI1 总线 */
#include "iic_hal.h"                /* 触摸的位带 I2C 总线 */
#include "exti_hal.h"               /* exti_driver_t (MAX30102 INT 的占位实例用) */
#include "pwm_hal.h"                /* TIM4 CH3/CH4 (PB8/PB9) 的两路 PWM */
#include "adc_hal.h"                /* ADC1: ADKEY(IN2) 与电源电压检测(IN8) 共用 */
#include "cywatch_rtc.h"            /* 表盘页的日期/时钟来源 */
#include "cywatch_service_lvgl.h"   /* service_lvgl_init() */
#include "cywatch_service_fatfs.h"  /* service_fatfs_init() */
#include "cywatch_service_key.h"    /* service_key_init() */
#include "cywatch_service_nordicprotocol.h" /* service_nordicprotocol_init() */
#include "cywatch_service_ota.h"            /* service_ota_init() (只注册特征回调) */
#include "cywatch_service_AttitudeCalculation.h" /* service_attitudecalculation_init() */
#include "cywatch_service_HeartRate.h"      /* service_heartrate_init() */
#include "cywatch_service_humiture.h"       /* service_humiture_init() */
#include "cywatch_service_power.h"          /* service_power_init() */
#include "cywatch_service_flags.h"          /* service_flags_init() */
#include "cywatch_service_watchdog.h"       /* service_watchdog_init() */
#include "cywatch_service_appcore.h"        /* service_appcore_init() */
#include "cywatch_adapter_mpu6050.h"        /* attitudecalculation_bsp_*() (自检用) */
#include "cywatch_adapter_max30102.h"       /* heartrate_bsp_*() (自检/EXTI 回调用) */
#include "cmsis_os2.h"              /* osKernelInitialize / osKernelStart / osSemaphoreNew */
#include "FreeRTOS.h"               /* 必须先于 task.h */
#include "task.h"                   /* xPortGetFreeHeapSize() (自检打印内核堆余量) */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ===== MPU6050 / MAX30102 服务层闭环测试脚手架 =====
   1 = 建一个一次性自检任务: 先用 adapter 的 xxx_bsp_*() 把 BSP 层打出来(WHO_AM_I /
   PART_ID / 单次 6 轴 / 单次 FIFO 样本), 自检完再把两条真实服务任务拉起来.
   ★测完请改回 0★ —— 两个服务源码里另有各自的打印开关(见两个 service .c 顶部) */
#define SENSOR_SVC_LOOPBACK_TEST   0

/* ===== 看门狗服务开关 =====
   1 = 建 "watchdog" 任务, 任务体内启动 IWDG(超时≈4.096s)并每 1s 喂一口;
   0 = 整个服务不建(调试期默认).
   ★为什么默认关★: IWDG 一旦启动软件关不掉, 只有复位能清 —— 挂调试器打断点、
   单步、或任何让内核停住超过 4s 的操作都会把板子复位, 根本调不了.
   ★脱离调试/上板前记得改回 1★ (改回来后 cywatch_watchdog.c 才会被链进来) */
#define WATCHDOG_SERVICE_ENABLE    0
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

/* ---- LCD: SPI1 总线实例 ----
   SPI1 的时钟/引脚/DMA 全在 spi_hal 的 pf_init 里配(不走 MSP), 所以下面 cfg
   要把 GPIO 与两条 DMA 流的描述都填齐。
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
    .tx_timeout_tick = 200,           /* 仅裸机路径用; OS 路径的超时由信号量给 */

    .gpio =
    {
        .p_port = GPIOA,
        .pins   = LCD_SCL_Pin | LCD_SDA_Pin, /* PA5=SCK / PA7=MOSI */
        .mode   = GPIO_MODE_AF_PP,
        .pull   = GPIO_NOPULL,
        .speed  = GPIO_SPEED_FREQ_VERY_HIGH, /* 50MHz, 引脚翻转要跟上 */
        .af     = GPIO_AF5_SPI1,
    },

    /* TX: DMA2_Stream3_Channel3, 像素数据分块发送 */
    .dma_tx =
    {
        .p_stream          = DMA2_Stream3,
        .channel           = DMA_CHANNEL_3,
        .irqn              = DMA2_Stream3_IRQn,
        .direction         = DMA_MEMORY_TO_PERIPH,
        .mode              = DMA_NORMAL,
        .priority          = DMA_PRIORITY_LOW, /* 流之间的总线仲裁 */
        .nvic_priority     = 5,                /* 须 ≥ configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY */
    },
    /* RX: 面板只接 MOSI, 不收 —— p_stream 为 NULL 表示不建这条流 */
    .dma_rx =
    {
        .p_stream          = NULL,
    },
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

/* ---- W25Q64: SPI2 总线实例 ----
   W25Q64 独占 SPI2(PB13=SCK/PB14=MISO/PB15=MOSI) + CS(PB12)。
   小传输走阻塞接口(pf_transmit / pf_receive), 大块接收走 pf_receive_dma ——
   按字节数分派, 见 adapter 的 w25q64_spi_receive_bytes。命令与状态轮询都是几字节,
   单独为它们起一次 DMA 不划算。
   参数与 CubeMX 会生成的 MX_SPI2_Init() 逐项一致, Mode0。
   @note 速率只能到 25MHz, 不是 50MHz: SPI2 挂 APB1(本工程 APB1 = 50MHz), 而 STM32 的
         SPI 波特率预分频**最小就是 /2**(BR 位域 000, HAL 里根本没有
         SPI_BAUDRATEPRESCALER_1 这个宏), 所以 SPI2 的物理上限是 50/2 = 25MHz。
         想要 50MHz 只能上 SPI1(APB2 100MHz), 但 SPI1 已被 LCD 独占。
         25MHz 对 W25Q64(标准版读 104MHz)完全够用, 不是瓶颈。 */
spi_driver_t spi2_instance; /* 非 static: adapter 内 extern 引用 */
static spi_cfg_t w25q64_spi_cfg =
{
    .p_spi_base = SPI2,
    .init =
    {
        .Mode              = SPI_MODE_MASTER,
        .Direction         = SPI_DIRECTION_2LINES,
        .DataSize          = SPI_DATASIZE_8BIT,
        .CLKPolarity       = SPI_POLARITY_LOW,  /* CPOL=0 */
        .CLKPhase          = SPI_PHASE_1EDGE,   /* CPHA=0 → Mode 0 */
        .NSS               = SPI_NSS_SOFT,      /* 片选由 adapter 的 pf_cs_set 驱动 PB12 */
        .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2, /* APB1 50MHz/2 = 25MHz */
        .FirstBit          = SPI_FIRSTBIT_MSB,
        .TIMode            = SPI_TIMODE_DISABLE,
        .CRCCalculation    = SPI_CRCCALCULATION_DISABLE,
        .CRCPolynomial     = 10,
    },
    .tx_timeout_tick = 0,    /* 仅裸机 DMA 路径用; OS 路径的超时由信号量给 */

    .gpio =
    {
        .p_port = GPIOB,
        .pins   = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, /* PB13=SCK/PB14=MISO/PB15=MOSI */
        .mode   = GPIO_MODE_AF_PP,
        .pull   = GPIO_NOPULL,
        .speed  = GPIO_SPEED_FREQ_VERY_HIGH, /* 25MHz */
        .af     = GPIO_AF5_SPI2,
    },

    /* ★收发两条流都必须建★: 全双工主模式下 HAL_SPI_Receive_DMA 会转调
       HAL_SPI_TransmitReceive_DMA, 后者解引用 hspi->hdmatx, 只配 RX 会 HardFault。
       发送侧的哑字节走 TX 流, SCLK 由它带出来 —— 不建 RX 也收不到时钟。
       TX = DMA1_Stream4_Ch0 / RX = DMA1_Stream3_Ch0 (RM0383 Table 27)。
       DMA1 的八条流此前全空闲, 时钟由 dma_hal 在装配流时打开 */
    .dma_tx =
    {
        .p_stream          = DMA1_Stream4,
        .channel           = DMA_CHANNEL_0,
        .irqn              = DMA1_Stream4_IRQn,
        .direction         = DMA_MEMORY_TO_PERIPH,
        .mode              = DMA_NORMAL,
        .priority          = DMA_PRIORITY_LOW, /* 流之间的总线仲裁: DMA1 上只有这两条流 */
        .nvic_priority     = 6,                /* 须 ≥ configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY */
    },
    .dma_rx =
    {
        .p_stream          = DMA1_Stream3,
        .channel           = DMA_CHANNEL_0,
        .irqn              = DMA1_Stream3_IRQn,
        .direction         = DMA_PERIPH_TO_MEMORY,
        .mode              = DMA_NORMAL,
        .priority          = DMA_PRIORITY_LOW,
        .nvic_priority     = 6,
    },
};
static spi_delay_interface_t w25q64_spi_delay_instance =
{
    .pf_delay_us = delay_us,
};

/* W25Q64 的 SPI2 信号量: 现在**真的有人等了** —— w25q64_spi_receive_bytes 在
   大块接收时走 pf_receive_dma, 完成回调 HAL_SPI_RxCpltCallback 释放本信号量。
   超时余量: 单次最大 60KB @25MHz ≈ 20ms, 200ms 是十倍。超时同样清迟到令牌,
   理由与 lcd_spi_sem_wait 一致(工程没有 HAL_SPI_ErrorCallback, 出错时完成回调
   永不到来, 只能靠超时兜底)。 */
#define W25Q64_SPI_TX_TIMEOUT_MS   (200u)
static osSemaphoreId_t w25q64_spi_sem_handle = NULL;

static int8_t w25q64_spi_sem_wait(void)
{
    if (NULL == w25q64_spi_sem_handle)
    {
        return -1;
    }

    if (osOK != osSemaphoreAcquire(w25q64_spi_sem_handle, W25Q64_SPI_TX_TIMEOUT_MS))
    {
        /* 超时: 顺手清掉可能"迟到"的完成令牌, 理由同 lcd_spi_sem_wait */
        (void)osSemaphoreAcquire(w25q64_spi_sem_handle, 0u);
        return -1;
    }

    return 0;
}

/* 由 HAL_SPI_RxCpltCallback(DMA 中断上下文)调用: 只释放信号量 ——
   本总线只做 DMA 接收, 发送侧的完成回调在 TransmitReceive 路径里被置 NULL */
static int8_t w25q64_spi_sem_release(void)
{
    if (NULL == w25q64_spi_sem_handle)
    {
        return -1;
    }

    return (osOK == osSemaphoreRelease(w25q64_spi_sem_handle)) ? 0 : -1;
}

static spi_semaphore_interface_t w25q64_spi_semaphore_instance =
{
    .pf_wait    = w25q64_spi_sem_wait,
    .pf_release = w25q64_spi_sem_release,
};

/* ---- MPU6050 / MAX30102 / AHT21 共用的位带 I2C(SCL=PB6, SDA=PB7) ----
   三个 adapter 在文件内 `extern iic_driver_t iic_instance;`, 不给定义就链接失败。
   ★ 全工程只有这条总线被多个器件共用, 所以只有它需要由外部(main.c)持有 ——
     独占总线(CST816T 的 PA8/PB4、AT24C02 的 PB10/PB3)的实例和锁都归各自的 adapter。
   ★ 这条总线被姿态(10ms 周期)与心率(突发)两个任务共用, 而位带 iic_hal 不可重入
     —— 所以它必须注入互斥量, 由 pf_readreg/pf_writereg 整段持锁。
   @warning 本轮只搭总线, 不建任务: 没有任何代码路径会发起事务。
            谁要接服务, 得自己补任务与 adapter 的 bsp_inst()。 */
iic_driver_t  iic_instance;            /* MPU6050 + MAX30102 + AHT21 共用的软 I2C */
static iic_bus_t iic_bus_instance =
{
    .sda = { .p_port = GPIOB, .pins = GPIO_PIN_7, .mode = GPIO_MODE_OUTPUT_PP,
             .pull = GPIO_PULLUP, .speed = GPIO_SPEED_FREQ_HIGH },
    .scl = { .p_port = GPIOB, .pins = GPIO_PIN_6, .mode = GPIO_MODE_OUTPUT_PP,
             .pull = GPIO_PULLUP, .speed = GPIO_SPEED_FREQ_HIGH },
};
static iic_delay_interface_t iic_delay_instance =
{
    .pf_delay_us = delay_us,
};

/* 共享总线的互斥量: 理由同上, 句柄同样归本文件持有 */
static osMutexId_t iic_mutex_handle = NULL;

static int8_t iic_mutex_lock(void)
{
    if (NULL == iic_mutex_handle)
    {
        return -1;
    }

    return (osOK == osMutexAcquire(iic_mutex_handle, osWaitForever)) ? 0 : -1;
}

static int8_t iic_mutex_unlock(void)
{
    if (NULL == iic_mutex_handle)
    {
        return -1;
    }

    return (osOK == osMutexRelease(iic_mutex_handle)) ? 0 : -1;
}

static iic_mutex_interface_t iic_mutex_instance =
{
    .pf_lock   = iic_mutex_lock,
    .pf_unlock = iic_mutex_unlock,
};

/* ---- ADC1: ADKEY(IN2/PA2) 与 电源电压检测(IN8/PB0) 共用 ----
   两个 adapter 在文件内 `extern adc_driver_t adc_instance;`。
   ★ADC1 全芯片只有一个转换器, 但两个设备各用各的通道, 所以实例由 main.c 持有、
     通道由 pf_read 的入参给 —— 这与 iic 那种"共用同一组引脚"的共享不一样。
   ★两个通道的采样互斥: 换通道改的是外设级的规则组寄存器, 结果也在同一个 ADC_DR,
     交错会读成对方的值, 所以必须注入互斥量。
   ★引脚不在 cfg 里: 通道跟着引脚走, 由各 adapter 自己 claim/release 时用 gpio_hal
     配成模拟输入。 */
adc_driver_t  adc_instance;            /* ADKEY + 电源电压检测共用的 ADC1 */

/* 共享 ADC 的互斥量: 句柄同样归本文件持有 */
static osMutexId_t adc_mutex_handle = NULL;

static int8_t adc_mutex_lock(void)
{
    if (NULL == adc_mutex_handle)
    {
        return -1;
    }

    return (osOK == osMutexAcquire(adc_mutex_handle, osWaitForever)) ? 0 : -1;
}

static int8_t adc_mutex_unlock(void)
{
    if (NULL == adc_mutex_handle)
    {
        return -1;
    }

    return (osOK == osMutexRelease(adc_mutex_handle)) ? 0 : -1;
}

static adc_mutex_interface_t adc_mutex_instance =
{
    .pf_lock   = adc_mutex_lock,
    .pf_unlock = adc_mutex_unlock,
};

/* ADC 外设级参数(与通道无关): 12bit 单次软件触发. 通道与采样时间见各自 adapter */
static adc_cfg_t adc_shared_cfg =
{
    .p_adc_base = ADC1,
    .init =
    {
        .ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4,
        .Resolution            = ADC_RESOLUTION_12B,
        .DataAlign             = ADC_DATAALIGN_RIGHT,
        .ScanConvMode          = DISABLE,
        .EOCSelection          = ADC_EOC_SINGLE_CONV,
        .ContinuousConvMode    = DISABLE,
        .NbrOfConversion       = 1U,
        .DiscontinuousConvMode = DISABLE,
        .NbrOfDiscConversion   = 0U,
        .ExternalTrigConv      = ADC_SOFTWARE_START,
        .ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE,
        .DMAContinuousRequests = DISABLE,
    },
    .p_mutex_interface = &adc_mutex_instance,
};

/* ---- MAX30102 INT(PA3/EXTI3): FIFO 满(下降沿)通知 ----
   @note 配置照触摸那条链(CST816T INT PB2)写; 抢占优先级 6 >= configLIBRARY_
         MAX_SYSCALL_INTERRUPT_PRIORITY(5) —— ISR 里要调 osSemaphoreRelease。
   @note PA3 在 GPIOA, 时钟由上面触摸的 SCL(PA8) 那段一并打开 */
#define MAX30102_INT_PREEMPT_PRIO   6
static exti_cfg_t max30102_exti_cfg =
{
    .gpio =
    {
        .p_port = GPIOA,
        .pins   = GPIO_PIN_3,
        .mode   = GPIO_MODE_IT_FALLING,
        .pull   = GPIO_PULLUP, /* INT 空闲高电平, 防悬空误触发 */
        .speed  = GPIO_SPEED_FREQ_HIGH,
    },
    .irqn             = EXTI3_IRQn,
    .preempt_priority = MAX30102_INT_PREEMPT_PRIO,
    .sub_priority     = 0,
};
static exti_delay_interface_t max30102_exti_delay_instance =
{
    .pf_delay_us = delay_us,
};

/* EXTI3 的 ISR 回调: 只转发到 adapter, 绝不做 I2C/printf(软件 I2C 不可重入) */
static void max30102_exti_cb(void *p_ctx)
{
    (void)p_ctx;
    (void)heartrate_bsp_interrupt_cb();
}

exti_driver_t max30102_exti_instance; /* 非 static: adapter 内 extern 引用 */

/* ---- PWM: TIM4 的 CH3/CH4(PB8 / PB9) ----
   @note ★一个实例管 TIM4 的全部通道★: 同一定时器的各通道共享时基(PSC/ARR),
         所以 CH3/CH4 必然同频, 只能各自调占空比。要两路不同频得用两个定时器,
         而 TIM5 已被 rtstats 占用。
   @note TIM4 挂 APB1(本工程 APB1 = 50MHz): 该预分频≠1 时定时器时钟 = PCLK1×2
         = 100MHz, 即下面 cfg 里的 tim_clock_hz。这个数由调用方算好写进来,
         驱动不去猜"本定时器挂在哪条总线上"。
   @note 构造与 init 都放在 osKernelStart() 之前: HAL_TIM_PWM_Init/Start 是纯寄存器
         操作, 无中断、无 osDelay, 不像 SPI/ADC 那样要等调度器。
   @note ★不走 ST 的 MSP★: TIM 的时钟与 PB8/PB9 的复用配置都在驱动的 pf_init
         里做, stm32f4xx_hal_msp.c 里**没有** HAL_TIM_PWM_MspInit。引脚交给
         gpio_hal, 所以下面 cfg 的 gpio.pins/af 必须给全, 少一个引脚就不会被配成
         复用(示波器上什么也量不到)。
   @warning 本轮只到驱动, 没有消费者: PB8/PB9 上真的在出波形, 但没有任何任务会去调
            pf_set_duty —— 谁要接服务(振动马达/蜂鸣器), 自己补 adapter 与服务。
   @note 频率只在 pf_init 时定死(cfg.freq_hz), 本层没有运行中改频的接口: 于是
         pf_set_duty 只剩两次独立写(CCR + 一个字节), 天然线程安全, 不需要锁。
         将来真要动态调频, 那段"读改写 ARR 再刷各通道 CCR"必须整体加互斥。 */
pwm_driver_t pwm_instance; /* 本轮无 adapter, 仍留非 static 方便将来服务直接 extern */
static pwm_cfg_t pwm_cfg =
{
	.p_tim_base = TIM4,
	.init =
	{
		.CounterMode       = TIM_COUNTERMODE_UP,
		.ClockDivision     = TIM_CLOCKDIVISION_DIV1,
		.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE,
		/* Prescaler/Period 留 0: 由驱动按 freq_hz 重算后覆盖 */
	},
	.tim_clock_hz = 100000000U,                /* APB1 50MHz × 2 */
	.freq_hz      = 1000U,                     /* 1kHz: 马达/背光/无源蜂鸣器通吃的起点 */
	.channels     = PWM_CHANNEL_3 | PWM_CHANNEL_4,
	.duty_percent = { 0U, 0U, 50U, 50U },      /* CH3/CH4 各 50% */
	.gpio =
	{
		.p_port = GPIOB,                     /* PB8=TIM4_CH3 / PB9=TIM4_CH4 */
		.pins   = GPIO_PIN_8 | GPIO_PIN_9,
		.mode   = GPIO_MODE_AF_PP,
		.pull   = GPIO_NOPULL,
		.speed  = GPIO_SPEED_FREQ_LOW,
		.af     = GPIO_AF2_TIM4,
	},
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if SENSOR_SVC_LOOPBACK_TEST
static const osThreadAttr_t g_sensor_selftest_attr =
{
    .name       = "sensor_test",
    .attr_bits  = 0,
    .cb_mem     = NULL,
    .cb_size    = 0,
    .stack_mem  = NULL,
    .stack_size = 1024,
    .priority   = osPriorityNormal,
};

/* 每 50 帧(10ms/帧 = 500ms)打一行姿态, 逐帧打会淹没 115200 的串口 */
#define SENSOR_SELFTEST_ATTITUDE_DIV    50

/**
 * @brief BSP 层自检: 直调 adapter 把两条链的器件层打出来, 完了再拉起两条服务
 *
 * @note 自检必须跑在服务之前 —— 两侧共用同一份 static 设备实例, 同时跑会互相重配器件
 * @note 浮点一律放大成整数打: 本工程是 MicroLIB(uvprojx useUlib=1), printf 不支持 %f
 */
static void sensor_selftest_run(void *pvParameters)
{
    int8_t   rc = 0;
    int8_t   max_inst_rc = 0; /* MAX30102 构造结果: 失败时实例指针已被清空, 探针不能再调 */
    float    ax = 0.0f, ay = 0.0f, az = 0.0f;
    float    gx = 0.0f, gy = 0.0f, gz = 0.0f;
    uint32_t red = 0, ir = 0, n = 0;

    (void)pvParameters;

    log_printf("\r\n===== 传感器 BSP 层自检 =====\r\n");

    /* ---------- MPU6050: I2C 读写 + WHO_AM_I + 单次 6 轴 ---------- */
    rc = attitudecalculation_bsp_inst();
    log_printf("[MPU6050] inst rc=%d (0=成功; -5=WHO_AM_I 不是 0x68, 多为接线/器件)\r\n", (int)rc);
    if (0 == rc)
    {
        log_printf("[MPU6050] WHO_AM_I=0x%02X (期望 0x68; 0xFF 表示读失败)\r\n",
                   (unsigned)attitudecalculation_bsp_read_id() & 0xFFU);

        rc = attitudecalculation_bsp_read_accel(&ax, &ay, &az);
        log_printf("[MPU6050] accel rc=%d  x=%d y=%d z=%d (单位 0.01g; 静止 z 应约 100)\r\n",
                   (int)rc, (int)(ax * 100.0f), (int)(ay * 100.0f), (int)(az * 100.0f));

        rc = attitudecalculation_bsp_read_gyro(&gx, &gy, &gz);
        log_printf("[MPU6050] gyro  rc=%d  x=%d y=%d z=%d (单位 0.01deg/s; 静止应接近 0)\r\n",
                   (int)rc, (int)(gx * 100.0f), (int)(gy * 100.0f), (int)(gz * 100.0f));
    }

    /* ---------- MAX30102: PART_ID + 模式切换 + 单次 FIFO 样本 ---------- */
    max_inst_rc = heartrate_bsp_inst();
    log_printf("[MAX30102] inst rc=%d (0=成功; -8=init 失败 -9=PART_ID 不是 0x15)\r\n", (int)max_inst_rc);
    if (0 == max_inst_rc)
    {
        log_printf("[MAX30102] PART_ID=0x%02X (期望 0x15; 0xFF 表示读失败)\r\n",
                   (unsigned)heartrate_bsp_read_id() & 0xFFU);

        rc = heartrate_bsp_change_to_spo2();
        log_printf("[MAX30102] change_to_spo2 rc=%d\r\n", (int)rc);

        rc = heartrate_bsp_read_one_sample(&red, &ir, &n);
        log_printf("[MAX30102] one_sample rc=%d  red=%u ir=%u 每样本=%u 字节\r\n",
                   (int)rc, (unsigned)red, (unsigned)ir, (unsigned)n);
        log_printf("[MAX30102] (FIFO 空时 red/ir 读回 0, 属正常; 有手指贴合应是万级以上计数)\r\n");
    }

    /* ---------- MAX30102 硬件探针: 自己开 A_FULL, 看器件到底有没有断言 ----------
       心率服务卡在"等 FIFO 满中断超时", 要分清是"器件没断言"还是"断言了但 INT 线
       没通到 MCU". 设备寄存器直接经 iic_instance 读, 不绕 MAX30102 驱动.
       ★A_FULL 是"拉低并保持": 直到读了 INTR_STATUS_1 才释放 —— 所以必须先读引脚
         电平, 再读状态寄存器(读操作会清标志), 顺序反了就什么都看不到★
       @note 构造失败时实例指针已被 pf_deinst 清空, 进来就是空指针跳转 */
    if (0 == max_inst_rc)
    {
        uint8_t mode = 0, wr1 = 0, rd1 = 0, st = 0, wr2 = 0, ovf = 0;
        int8_t  int_rc = 0;

        /* a. 开中断前的基线: 器件在采样? 引脚空闲电平对不对? */
        (void)iic_instance.pf_readreg(&iic_instance, 0x57, 0x09, &mode, 1); /* MODE_CONFIG */
        (void)iic_instance.pf_readreg(&iic_instance, 0x57, 0x04, &wr1, 1);  /* FIFO_WR_PTR */
        (void)iic_instance.pf_readreg(&iic_instance, 0x57, 0x06, &rd1, 1);  /* FIFO_RD_PTR */
        log_printf("[PROBE] MODE=0x%02X(期望0x03) WR_PTR=%u RD_PTR=%u INT(PA3)=%u(空闲应=1)\r\n",
                   (unsigned)mode, (unsigned)wr1, (unsigned)rd1,
                   (unsigned)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3));

        /* b. 打开 A_FULL 中断源 + EXTI3 NVIC —— 与心率服务同一条链, 只是提前做 */
        int_rc = heartrate_bsp_enable_FIFO_FULL_interrupt();
        log_printf("[PROBE] enable_FIFO_FULL_interrupt rc=%d(0=器件与EXTI都已开)\r\n", (int)int_rc);

        /* c. 等 FIFO 填满: 100Hz/32 深 → 320ms 满一次, 1s 内必然断言过 */
        osDelay(1000);

        /* d. 先看引脚(A_FULL 会把它拉低并保持), 再读状态寄存器(读走才释放) */
        log_printf("[PROBE] 开中断1s后 INT(PA3)=%u (0=器件确实在断言)\r\n",
                   (unsigned)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3));
        (void)iic_instance.pf_readreg(&iic_instance, 0x57, 0x00, &st, 1);  /* INTR_STATUS_1 */
        (void)iic_instance.pf_readreg(&iic_instance, 0x57, 0x04, &wr2, 1);
        (void)iic_instance.pf_readreg(&iic_instance, 0x57, 0x05, &ovf, 1);
        log_printf("[PROBE] INTR_ST1=0x%02X(bit7=A_FULL) WR_PTR=%u OVF=%u 清标志后 INT(PA3)=%u\r\n",
                   (unsigned)st, (unsigned)wr2, (unsigned)ovf,
                   (unsigned)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3));
        log_printf("[PROBE] 判读: bit7=1 且 引脚由0变1=器件+线都好; "
                   "bit7=1 而引脚恒1=INT 线没通; bit7=0=器件没断言(FIFO_A_FULL 阈值/使能)\r\n");
    }

    log_printf("===== BSP 自检结束, 拉起三个传感器服务 =====\r\n");

    /* 内核堆只有 24KB(FreeRTOSConfig.h), 任务栈全部从这里切 —— 逐条打印余量,
       哪一条把堆吃光、谁建任务失败, 一眼可见 */
    log_printf("[SENSOR] 起服务前 内核堆余量 = %u 字节\r\n",
               (unsigned)xPortGetFreeHeapSize());

    /* 自检完再把真实服务任务拉起来, 之后由它们持有设备 */
    service_attitudecalculation_init();
    log_printf("[SENSOR] attitude 服务拉起后 内核堆余量 = %u 字节\r\n",
               (unsigned)xPortGetFreeHeapSize());

    service_heartrate_init();
    log_printf("[SENSOR] heartrate 服务拉起后 内核堆余量 = %u 字节\r\n",
               (unsigned)xPortGetFreeHeapSize());

    service_humiture_init();
    log_printf("[SENSOR] humiture 服务拉起后 内核堆余量 = %u 字节\r\n",
               (unsigned)xPortGetFreeHeapSize());

    osThreadExit(); /* 一次性任务, 用完退出 */
}
#endif
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
	/* 日志口 USART6: PA11=TX/PA12=RX, 115200 8N1. 初始化后 printf 即发往 PA11
	   ★与 Nordic 的 USART1(PA9/PA10) 是两个独立的口, 日志不再插进协议线★ */
    UART_Init();

	/* ===== 日志互斥量: 把整条 printf 跨任务串行化 =====
	   fputc 是每字符一次阻塞 HAL_UART_Transmit 且全程无闸门, 两个任务同时打会让
	   后者在 HAL 的 __HAL_LOCK 上拿到 HAL_BUSY 并被 (void) 静默丢字符。
	   在这里(第一个 printf 之前)把锁建好; 任务里的日志走 log_printf()。
	   @note 失败即停: 锁建不出来说明内核堆已经不够, 后面建服务任务同样会失败 */
    if (0 != log_init())
    {
        Error_Handler();
    }
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

	/* ===== 总线 2: MPU6050 / MAX30102 / AHT21 共用的位带 I2C(SCL=PB6, SDA=PB7) =====
	   这是全工程唯一一条**共享**总线, 所以实例留在这里由 main.c 建, 三个 adapter
	   经 extern 引用。
	   @note 这里只建实例与锁, **不调 pf_init**: SDA/SCL 由三个设备的 adapter 在各自
	         bsp_inst() 里 iic_claim() 抬起、在 hibernating()/deinst() 里
	         iic_release() 放下, iic_hal 按 ref_count 门控, 三个设备都退出才真的
	         释放引脚。留一次无条件 pf_init 会让计数永远回不到 0, 释放出口形同虚设。
	   @note 独占总线不走这里: CST816T 的(PA8/PB4)在 lvgl adapter 里, AT24C02 的
	         (PB10/PB3)在 flags adapter 里, 各自管各自的引脚与锁。
	   @warning 设备实例化都在 osKernelStart() 之后的任务里, 所以启动内核之前这条
	            总线的引脚还没配 —— 而这个阶段也没有任何代码会发起事务 */
	iic_mutex_handle = osMutexNew(NULL);
	if (NULL == iic_mutex_handle)
	{
		Error_Handler(); /* 内核堆不足 */
	}
	if (0 != iic_driver_inst(&iic_instance,
	                         &iic_bus_instance,
	                         &iic_delay_instance,
	                         &iic_mutex_instance))
	{
		Error_Handler(); /* 总线/延时/互斥量接口参数非法 */
	}

	/* ===== ADC1: ADKEY(IN2/PA2) 与 电源电压检测(IN8/PB0) 共用 =====
	   @note 这里只建实例与锁, **不调 pf_init**: 由两个设备各自 claim/release,
	         adc_hal 按 ref_count 门控, 两个都退出才真的关 ADC 与时钟。留一次
	         无条件 pf_init 会让计数永远回不到 0(与 iic 那次同一个坑)。
	   @note 通道与采样时间不在这里: 它们是 pf_read 的入参, 由各 adapter 给。
	   @warning 两个设备都在 osKernelStart() 之后的任务里才 claim, 启动内核之前
	            这个外设还没初始化 —— 而这个阶段也没有任何代码会采 AD */
	adc_mutex_handle = osMutexNew(NULL);
	if (NULL == adc_mutex_handle)
	{
		Error_Handler(); /* 内核堆不足 */
	}
	if (0 != adc_driver_inst(&adc_instance, &adc_shared_cfg))
	{
		Error_Handler(); /* ADC 配置非法(基地址为空) */
	}

	/* ===== MAX30102 INT 线: PA3/EXTI3(FIFO 满下降沿) =====
	   @note pf_init 经由 gpio_hal 配 PA3(端口时钟也由它开), 并把这个实例按线号
	         注册进 exti_hal 的分发表; 可在 osKernelStart 前调(不依赖调度器)。
	   @note 只挂实例与回调, **不开中断**: NVIC 使能在 exti_enable_interrupt 里,
	         由心率服务的 heartrate_bsp_enable_FIFO_FULL_interrupt() 在任务里触发
	         —— 那时 max30102 实例已构造, ISR 进来不会打到空回调 */
	if (0 != exti_driver_inst(&max30102_exti_instance,
	                          &max30102_exti_cfg,
	                          &max30102_exti_delay_instance))
	{
		Error_Handler();
	}
	if (0 != max30102_exti_instance.pf_init(&max30102_exti_instance))
	{
		Error_Handler();
	}
	(void)max30102_exti_instance.pf_attach_callback(&max30102_exti_instance, max30102_exti_cb);

	/* ===== 总线 3: W25Q64 存储的 SPI2(PB13=SCK / PB14=MISO / PB15=MOSI + CS=PB12) =====
	   spi_driver_inst 只做参数校验 + 装配内嵌 hspi 句柄 + 挂函数指针, 不碰硬件, 也不依赖
	   调度器, 可在 osKernelStart 前调用。SPI2 外设的真正初始化(HAL_SPI_Init → MspInit 配
	   PB13/14/15)不在这里做: 由 W25Q64 驱动在 w25q64_init 里回调 spi 接口的 pf_init 触发
	   (adapter 把它转发到 spi2_instance.pf_init), 那时已在调用方的任务体内 —— 与 SPI1 由
	   st7789t3_init 回调是同一条路径。
	   @note SPI2 的 CS(PB12) 由 adapter 的 gpio 接口 pf_init 配置(本工程没有
	         MX_GPIO_Init), 详见 cywatch_adapter_w25q64.c 的文件头 @note。
	   @note 本段只搭总线, **不**构造 W25Q64 设备: storage_bsp_w25q64_inst() 内含 osDelay,
	         必须在 osKernelStart() 之后的任务上下文里调。 */
	w25q64_spi_sem_handle = osSemaphoreNew(1u, 0u, NULL); /* 当前无人 wait, 见 PV 区注释 */
	if (NULL == w25q64_spi_sem_handle)
	{
		Error_Handler(); /* 内核堆不足 */
	}
	if (0 != spi_driver_inst(&spi2_instance,
	                         &w25q64_spi_cfg,
	                         &w25q64_spi_semaphore_instance,
	                         &w25q64_spi_delay_instance))
	{
		Error_Handler(); /* SPI2 配置非法(基地址/信号量/延时) */
	}

	/* ===== PWM: TIM4 的 CH3/CH4(两路, 同频 1kHz / 各 50%) =====
	   pwm_driver_inst 只做参数校验 + 装配内嵌 htim 句柄 + 挂函数指针, 不碰硬件;
	   真正的初始化(开 TIM4/GPIOB 时钟 + 配 PB8/PB9 复用 → 算 PSC/ARR →
	   HAL_TIM_PWM_Init → 逐通道 ConfigChannel + Start)全在 pf_init 里, 是纯寄存器
	   操作、不依赖调度器, 所以直接在这里调, 不必等 osKernelStart。
	   @note PB8/PB9 上没有负载: 要验波形得拿示波器/万用表量这两只脚
	   @warning 返回值必须看: 非 0 = 参数非法或 TIM 没配起来(PB8/PB9 上静默无输出) */
	if (0 != pwm_driver_inst(&pwm_instance, &pwm_cfg))
	{
		Error_Handler(); /* PWM 配置非法(基地址/freq 越界) */
	}
	if (0 != pwm_instance.pf_init(&pwm_instance))
	{
		Error_Handler(); /* TIM4 初始化失败 */
	}
	printf("[PWM] init ok(TIM4_CH3/CH4 @ PB8/PB9, %uHz, CH3/CH4 = 50%%)\r\n",
		   (unsigned int)pwm_cfg.freq_hz);

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

	/* ===== FatFs 服务: W25Q64(SPI2) 上挂一个 FAT 卷 =====
	   @note service_fatfs_init() 只建一个**一次性任务**(不碰设备), 所以可以和
	         service_lvgl_init() 一样放在 osKernelStart() 之前。真正的
	         真正的挂载在那个任务体内跑 —— 它要经 SPI2 访问 W25Q64,
	         而 adapter 的 pf_delay 是 osDelay, 必须等调度器起来。
	   @note 挂载是异步的: 本函数返回时卷还没挂上。要确认挂好了就用
	         service_fatfs_is_mounted(), 或者直接调 service_fatfs_* 看返不返 -3
	         (SERVICE_FATFS_ERR_NOT_MOUNTED)。
	   @warning 返回值必须看: 非 0 = "fatfs" 任务创建失败(内核堆不足), 表现是
	            "文件 API 全部返回 -1, 串口上一个字都没有" */
	{
		int8_t fatfs_rc = service_fatfs_init();
		if (0 != fatfs_rc)
		{
			printf("\r\n[FATFS] service_fatfs_init failed, rc=%d(内核堆不足?)\r\n", (int)fatfs_rc);
			Error_Handler();
		}
		printf("[FATFS] service init ok(\"fatfs\" 任务已建; 挂载/自检在任务体内做)\r\n");
	}

	/* ===== AD 按键服务: 建 "key" 任务, 每 50ms 读一次键(PA2=ADC1_IN2) =====
	   ADC1 的时钟与 PA2 的模拟模式在 adc_hal 的 pf_init 里配(不走 MSP), 由任务
	   体内的 key_bsp_inst → adkey_inst → adc_init 触发
	   @note 本段只建任务, 不碰设备: key_bsp_inst() 里的 ADC 初始化与采样自检都要在
	         任务上下文做
	   @note 返回 void: 任务建不起来时它内部自己 for(;;) 兜住(与姿态/心率/温湿度一致) */
	service_key_init();
	printf("[KEY] service init ok(\"key\" 任务已建; ADKEY 构造/轮询在任务体内做)\r\n");

	/* ===== Nordic 协议服务: USART1(115200 8N1) + DMA2_Stream7(TX)/Stream2(RX) =====
	   一条龙里含板级绑定(DMA/NVIC/USART1 驱动)与 BSP 构造, 都在 osKernelStart 之前
	   可调; RX/TX 两个任务只在这里挂上, 任务体进调度器后才跑
	   @warning USART1 已整条让给链路: printf 走 USART6(PA11/PA12, 见 UART_Init)。
	            往 PA9/PA10 上写任何字节都是往协议线里注入, 日志一律走 log_printf()
	   @warning 返回值必须看: -4 = NVIC 优先级违反 FreeRTOS 约束(上电即静默崩),
	            -2/-7 = 内核堆不足, 都表现为"串口一声不响" */
	{
		int8_t nordic_rc = service_nordicprotocol_init();
		if (0 != nordic_rc)
		{
			printf("\r\n[NORDIC] service_nordicprotocol_init failed, rc=%d(-4:NVIC -2/-7:内核堆?)\r\n",
				   (int)nordic_rc);
			Error_Handler();
		}
		printf("[NORDIC] service init ok(\"nordic_rx\"/\"nordic_tx\" 任务已建)\r\n");
	}

	/* ===== OTA 服务: 只注册两个特征回调, 不建任务(下载任务按需起、用完自退) =====
	   @note ★必须紧跟在 service_nordicprotocol_init() 后面、osKernelStart() 之前★
	         BSP 那张特征回调表不做并发保护(只被 RX 任务读), 注册要赶在 RX 任务真正
	         跑起来之前写完 —— 这个位置调度器还没起, 时间点天然满足
	   @note 这里注册的是 feature 0x02(应用层应答) 与 0x81(镜像块). 一个特征只有
	         一个回调槽位, 后注册的会顶掉先注册的 */
	service_ota_init();
	printf("[OTA] feature 0x02/0x81 rx callbacks registered\r\n");

	/* ===== MPU6050(姿态) / MAX30102(心率) / AHT21(温湿度) 服务 =====
	   @note 三个 service_*_init() 都只建任务、不碰设备: 设备的实例化在三个 adapter
	         的 inst 里, 内含 osDelay, 必须等调度器跑起来(在任务体内做)
	   @note 这三个 init 返回 void(不像其它服务返回状态码): 任务建不起来时它们内部
	         自己 for(;;) 兜住. 代价是失败**静默** —— 只表现为"串口没有姿态/心率/温湿度
	         数据", 排查手段看 rtstats(每 5s 一行): 内核堆余量与任务表都在里面 */
#if SENSOR_SVC_LOOPBACK_TEST
	/* 闭环测试: 先跑一次 BSP 自检任务, 它自检完自己把三条服务拉起来再退出
	   (原因: 自检与服务共用同一份 static 设备实例, 同时跑会互相重配器件) */
	if (NULL == osThreadNew(sensor_selftest_run, NULL, &g_sensor_selftest_attr))
	{
		printf("\r\n[SENSOR] sensor_selftest 任务创建失败(内核堆不足?)\r\n");
		Error_Handler();
	}
	printf("[SENSOR] 自检任务已建; BSP 自检完会自动拉起姿态/心率/温湿度服务\r\n");
#else
	service_attitudecalculation_init();
	printf("[ATTITUDE] service init ok(\"attitude\" 任务已建; MPU6050 构造在任务体内做)\r\n");

	service_heartrate_init();
	printf("[HEARTRATE] service init ok(\"heartrate\" 任务已建; MAX30102 构造在任务体内做)\r\n");

	service_humiture_init();
	printf("[HUMITURE] service init ok(\"humiture\" 任务已建; AHT21 构造在任务体内做)\r\n");
#endif

	/* ===== 电源服务: 每 10s 采一次电池电压(PB0/ADC1_IN8, 分压 1/2) =====
	   @note service_power_init() 只建任务、不碰设备: 占住共享 ADC1 那一步在
	         任务体内做(adc_hal 的 pf_init 会开时钟, 但本函数在 osKernelStart()
	         之前调, 那时内核还没跑 —— 所以 claim 必须留在任务里)。
	   @note 与 ADKEY 共用 ADC1(adc_hal 按 ref_count 门控, 互斥量串行化),
	         走的是另一个通道, 引脚 PA2/PB0 互不干涉。
	   @note 返回 void: 任务建不起来时它内部死循环(与 attitude/heartrate/
	         humiture 三个服务一致) */
	service_power_init();
	printf("[POWER] service init ok(\"power\" 任务已建; ADC1 占用在任务体内做)\r\n");

	/* ===== app_core 服务: EasyAPP 事件总线唯一的消费者 =====
	   @note 只建任务, 循环里就是 easyapp_core_run() + osDelay(10): 不需要设备,
	         也不依赖调度器之外的东西, 但分发在任务上下文做, 所以照例放这里
	   @note 返回 void: 任务建不起来时它内部死循环(与 attitude/heartrate/
	         humiture 三个服务一致)
	   @warning 没有它整条总线是死的 —— 服务发的事件只进队列不出队, 页面处理函数
	         永远不会被调用(表盘三张卡片恒 0 就是这么来的) */
	service_appcore_init();
	printf("[APPCORE] service init ok(\"appcore\" 任务已建; 事件开始分发)\r\n");

	/* ===== flags 服务: AT24C02(独占位带 I2C, PB10=SCL / PB3=SDA) =====
	   @note service_flags_init() 只建一个**一次性任务**(不碰设备): 真正的构造在那个
	         任务体内跑 —— 它内含 osDelay(上电延时 + 写周期 ACK 探测), 必须等调度器
	         起来。构造完任务就 osThreadExit, 之后 service_flags_read/write 同步返回。
	   @note 构造是异步的: 本函数返回时 AT24C02 还没好。要确认就调
	         service_flags_is_ready(), 或者直接调 service_flags_* 看返不返 -3
	         (SERVICE_FLAGS_ERR_NOT_READY)。
	   @note 本服务**没有消费者**: 读写的 API 目前全工程无人调用, 上电只会构造一次
	         器件(驱动 init 里那次 ACK 探测), 之后不会自发产生任何 I2C 事务。
	   @warning 返回值必须看: 非 0 = "flags" 任务创建失败(内核堆不足) */
	{
		int8_t flags_rc = service_flags_init();

		if (0 != flags_rc)
		{
			printf("\r\n[FLAGS] service_flags_init failed, rc=%d(内核堆不足?)\r\n", (int)flags_rc);
			Error_Handler();
		}
		printf("[FLAGS] service init ok(\"flags\" 任务已建; AT24C02 构造在任务体内做)\r\n");
	}

	/* ===== 运行统计打印任务: 每 5 秒打一次内核堆 + 任务表(栈水位) + CPU 占比 =====
	   时基 TIM5 由内核在 vTaskStartScheduler() 里启动(见 FreeRTOSConfig.h)
	   @note 这里失败**不** Error_Handler: 它只是调试辅助, 建不起来不该让手表停机,
	         打一行警告继续跑 */
	if (0 != rtstats_start())
	{
		printf("\r\n[RTSTATS] rtstats_start failed(内核堆不足?); 统计日志将缺失\r\n");
	}

#if WATCHDOG_SERVICE_ENABLE
	/* ===== 看门狗服务: 高优先级任务, 每 1s 喂一口 IWDG(超时≈4.096s) =====
	   @note 开关在文件顶部的 PD 区(WATCHDOG_SERVICE_ENABLE), 调试期为 0 整段不编译。
	   @note 放最后一个建: 前面任一服务失败会走 Error_Handler 死等, 那时看门狗
	         要是已经跑起来, 板子会被反复复位, 反而看不到串口上打印的失败原因。
	   @note IWDG 不在 main 里启动, 在任务体内启动 —— 本函数返回到任务首次被调度
	         之间的启动阶段不计时; 否则启动阶段(LCD 重试、fatfs 首次挂载/格式化)
	         就会被算进超时(见 cywatch_service_watchdog.h 的 @warning)。
	   @warning ★从这里往后看门狗就在跑了★: IWDG 一旦启动软件关不掉, 只有复位能清。
	   @warning 返回值必须看: 非 0 = "watchdog" 任务创建失败(内核堆不足) */
	{
		int8_t watchdog_rc = service_watchdog_init();

		if (0 != watchdog_rc)
		{
			printf("\r\n[WATCHDOG] service_watchdog_init failed, rc=%d(内核堆不足?)\r\n", (int)watchdog_rc);
			Error_Handler();
		}
		printf("[WATCHDOG] service init ok(\"watchdog\" 任务已建; IWDG 在任务体内启动)\r\n");
	}
#endif

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
