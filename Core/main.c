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
#include "cywatch_rtc.h"            /* 表盘页的日期/时钟来源 */
#include "cywatch_service_lvgl.h"   /* service_lvgl_init() */
#include "cywatch_service_fatfs.h"  /* service_fatfs_init() */
#include "cywatch_service_key.h"    /* service_key_init() */
#include "cywatch_service_nordicprotocol.h" /* service_nordicprotocol_init() */
#include "cywatch_service_AttitudeCalculation.h" /* service_attitudecalculation_init() */
#include "cywatch_service_HeartRate.h"      /* service_heartrate_init() */
#include "cywatch_service_humiture.h"       /* service_humiture_init() */
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
/* ===== Nordic 服务层闭环测试脚手架 =====
   1 = 注册回显: 收到已注册特征的数据, 原样回一条同特征同内容的.
   ★测完请改回 0★ —— 打开时它让"收到就回"成为产品行为, 且每次收帧都占用 RX 任务
   做一次 send(协议核的回调里) */
#define NORDIC_SVC_LOOPBACK_TEST   0

/* ===== MPU6050 / MAX30102 服务层闭环测试脚手架 =====
   1 = 建一个一次性自检任务: 先用 adapter 的 xxx_bsp_*() 把 BSP 层打出来(WHO_AM_I /
   PART_ID / 单次 6 轴 / 单次 FIFO 样本), 自检完再把两条真实服务任务拉起来.
   ★测完请改回 0★ —— 两个服务源码里另有各自的打印开关(见两个 service .c 顶部) */
#define SENSOR_SVC_LOOPBACK_TEST   0
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

/* 触摸 I2C 的互斥量: 位带总线不可重入, 一次事务(start..stop)被 tick 抢占切开会踩总线。
   句柄归本文件持有(理由同 lcd_spi_sem_handle): 接口的两个回调没有形参 */
static osMutexId_t touch_iic_mutex_handle = NULL;

static int8_t touch_iic_mutex_lock(void)
{
    if (NULL == touch_iic_mutex_handle)
    {
        return -1;
    }

    return (osOK == osMutexAcquire(touch_iic_mutex_handle, osWaitForever)) ? 0 : -1;
}

static int8_t touch_iic_mutex_unlock(void)
{
    if (NULL == touch_iic_mutex_handle)
    {
        return -1;
    }

    return (osOK == osMutexRelease(touch_iic_mutex_handle)) ? 0 : -1;
}

static iic_mutex_interface_t touch_iic_mutex_instance =
{
    .pf_lock   = touch_iic_mutex_lock,
    .pf_unlock = touch_iic_mutex_unlock,
};

/* ---- W25Q64: SPI2 总线实例 ----
   W25Q64 独占 SPI2(PB13=SCK/PB14=MISO/PB15=MOSI) + CS(PB12), 不走 DMA:
   读写都是阻塞传输(spi_hal 的 pf_transmit / pf_receive, 单次最长 256B 页编程)。
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
    .p_hdma_tx       = NULL, /* W25Q64 不使用 DMA 发送 */
    .tx_timeout_tick = 0,    /* 仅裸机 DMA 路径用; 本总线不发 DMA */
};
static spi_delay_interface_t w25q64_spi_delay_instance =
{
    .pf_delay_us = delay_us,
};

/* W25Q64 的 SPI2 信号量: 当前**无人等待** —— W25Q64 走阻塞传输(pf_transmit /
   pf_receive), 只有 pf_transmit_dma 才会等信号量, 而本总线不发 DMA。
   之所以仍然要建: spi_driver_inst 强制要求 pf_wait/pf_release 非空(否则返回 -4),
   这是接口契约, 不是可选装饰。将来若 W25Q64 改走 DMA(大块读), 这对回调直接可用。 */
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

/* 由 HAL_SPI_TxCpltCallback(DMA 中断上下文)调用: 只释放信号量 */
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

/* ---- MPU6050 / MAX30102 共用的位带 I2C(SCL=PB6, SDA=PB7) ----
   两个 adapter 在文件内 `extern iic_driver_t iic_instance;`, 不给定义就链接失败。
   ★ 这条总线被姿态(10ms 周期)与心率(突发)两个任务共用, 而位带 iic_hal 不可重入
     —— 所以它必须注入互斥量, 由 pf_readreg/pf_writereg 整段持锁。
   @warning 本轮只搭总线, 不建任务: 没有任何代码路径会发起事务。
            谁要接服务, 得自己补任务与 adapter 的 bsp_inst()。 */
iic_driver_t  iic_instance;            /* MPU6050 + MAX30102 共用的软 I2C */
static iic_bus_t iic_bus_instance =
{
    .p_sda_port = GPIOB,
    .sda_pin    = GPIO_PIN_7,
    .p_scl_port = GPIOB,
    .scl_pin    = GPIO_PIN_6,
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

/* ---- AT24C02 独占的位带 I2C(SCL=PB10, SDA=PB3) ----
   adapter 在文件内 `extern iic_driver_t at24c02_iic_instance;`, 不给定义就链接失败。
   @note 线上只有 AT24C02 一个器件, 且当前没有任务访问它 —— 按 iic_hal 的
         "单任务独占总线可省锁"约定, 建这条总线时不注入互斥量(p_mutex_interface 传
         NULL)。将来若有多任务读写, 必须照 iic_instance 那条补上互斥量, 否则位带
         时序会被 tick 切在 start..stop 中间。
   @note PB3 复位后是 JTDO: 写 GPIO_MODER 就能释放为普通 GPIO, F4 不需要 SYSCFG
         重映射; 代价是丢掉 JTAG 的 JTDO/SWO 跟踪, SWD(PA13/PA14)不受影响。
   @warning 本轮只搭总线, 不建任务: 没有任何代码路径会发起事务, 也没有人调
            storage_bsp_at24c02_inst()。谁要接服务, 得自己补任务与 bsp_inst()。 */
iic_driver_t  at24c02_iic_instance;    /* AT24C02 独占的软 I2C */
static iic_bus_t at24c02_iic_bus_instance =
{
    .p_sda_port = GPIOB,
    .sda_pin    = GPIO_PIN_3,
    .p_scl_port = GPIOB,
    .scl_pin    = GPIO_PIN_10,
};
static iic_delay_interface_t at24c02_iic_delay_instance =
{
    .pf_delay_us = delay_us,
};

/* ---- MAX30102 INT(PA3/EXTI3): FIFO 满(下降沿)通知 ----
   @note 配置照触摸那条链(CST816T INT PB2)写; 抢占优先级 6 >= configLIBRARY_
         MAX_SYSCALL_INTERRUPT_PRIORITY(5) —— ISR 里要调 osSemaphoreRelease。
   @note PA3 在 GPIOA, 时钟由上面触摸的 SCL(PA8) 那段一并打开 */
#define MAX30102_INT_PREEMPT_PRIO   6
static exti_cfg_t max30102_exti_cfg =
{
    .p_port           = GPIOA,
    .pin              = GPIO_PIN_3,
    .mode             = GPIO_MODE_IT_FALLING,
    .pull             = GPIO_PULLUP, /* INT 空闲高电平, 防悬空误触发 */
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if NORDIC_SVC_LOOPBACK_TEST
/**
 * @brief  闭环测试回显: 收到什么特征的数据, 就原样回一条同特征同内容的.
 * @note   运行在 RX 任务上下文(协议核的接收回调), 所以 send 必须非阻塞 —— 它确实是.
 * @note   ★不要回 ACK(0x00)与 0x11/0x10 控制帧★: 那几类根本到不了这里(协议核自用).
 *         也正因如此, 本回显不会与对端形成无限循环.
 */
static void nordic_test_echo(uint8_t feature, uint8_t *pdata, uint16_t len)
{
	(void)service_nordicprotocol_send(feature, pdata, len, NULL);
}
#endif

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
    rc = heartrate_bsp_inst();
    log_printf("[MAX30102] inst rc=%d (0=成功; -8=init 失败 -9=PART_ID 不是 0x15)\r\n", (int)rc);
    if (0 == rc)
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
         电平, 再读状态寄存器(读操作会清标志), 顺序反了就什么都看不到★ */
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

    log_printf("===== BSP 自检结束, 拉起姿态/心率两个服务 =====\r\n");

    /* 内核堆只有 24KB(FreeRTOSConfig.h), 任务栈全部从这里切 —— 逐条打印余量,
       哪一条把堆吃光、谁建任务失败, 一眼可见 */
    log_printf("[SENSOR] 起服务前 内核堆余量 = %u 字节\r\n",
               (unsigned)xPortGetFreeHeapSize());

    /* 自检完再把三条真实服务任务拉起来, 之后由它们持有设备 */
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
	/* USART1: PA9=TX/PA10=RX, 115200 8N1. 初始化后 printf 即发往 PA9 */
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

	touch_iic_mutex_handle = osMutexNew(NULL);
	if (NULL == touch_iic_mutex_handle)
	{
		Error_Handler(); /* 内核堆不足 */
	}
	if (0 != iic_driver_inst(&touch_iic_instance,
	                         &touch_iic_bus_instance,
	                         &touch_iic_delay_instance,
	                         &touch_iic_mutex_instance))
	{
		Error_Handler(); /* 总线/延时/互斥量接口参数非法 */
	}

	/* ===== 总线 2b: MPU6050 + MAX30102 共用的位带 I2C(SCL=PB6, SDA=PB7) =====
	   @note GPIOB 的时钟在上面触摸那段已开(PB4 与 PB6/PB7 同端口)。
	   @note iic_init 只配 GPIO, 不含 I2C 时序也没有 osDelay, 可在 osKernelStart 前调;
	         本总线没有设备驱动替它回调 pf_init(MPU6050/MAX30102 的 iic 接口里没有
	         pf_init 这个口), 所以只能在这里自己调一次。
	   @warning 没有任何任务在跑, 这条总线接上电后不会自己发事务 */
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
	if (0 != iic_instance.pf_init(&iic_instance))
	{
		Error_Handler(); /* 参数非法; 正常不会返回非0 */
	}

	/* ===== 总线 2c: AT24C02 独占的位带 I2C(SCL=PB10, SDA=PB3) =====
	   @note GPIOB 的时钟在上面触摸那段已开(PB3/PB10 与 PB4 同端口)。
	   @note 不注入互斥量: 这条线上只有 AT24C02 一个器件且无任务访问(见 PV 区说明)。
	   @note iic_init 只配 GPIO, 不含 I2C 时序也没有 osDelay, 可在 osKernelStart 前调;
	         本总线同样没有设备驱动替它回调 pf_init, 只能在这里自己调一次;
	         真正构造 AT24C02 实例的 storage_bsp_at24c02_inst() 要等有任务时再调
	         (它内含 osDelay)。 */
	if (0 != iic_driver_inst(&at24c02_iic_instance,
	                         &at24c02_iic_bus_instance,
	                         &at24c02_iic_delay_instance,
	                         NULL))
	{
		Error_Handler(); /* 总线/延时接口参数非法 */
	}
	if (0 != at24c02_iic_instance.pf_init(&at24c02_iic_instance))
	{
		Error_Handler(); /* 参数非法; 正常不会返回非0 */
	}

	/* ===== MAX30102 INT 线: PA3/EXTI3(FIFO 满下降沿) =====
	   @note exti_driver_inst 内部自己调 HAL_GPIO_Init 配 PA3, 并把这个实例按线号
	         注册进 exti_hal 的分发表; 可在 osKernelStart 前调(不依赖调度器)。
	   @note 只挂实例与回调, **不开中断**: NVIC 使能在 exti_enable_interrupt 里,
	         由心率服务的 heartrate_bsp_enable_FIFO_FULL_interrupt() 在任务里触发
	         —— 那时 max30102 实例已构造, ISR 进来不会打到空回调 */
	__HAL_RCC_GPIOA_CLK_ENABLE(); /* PA3(INT) 与 PA8(触摸 SCL) 同端口, 这里再开一次无副作用 */
	if (0 != exti_driver_inst(&max30102_exti_instance,
	                          &max30102_exti_cfg,
	                          &max30102_exti_delay_instance))
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

	/* ===== AD 按键服务: 建 "key" 任务, 每 100ms 读一次键(PA2=ADC1_IN2) =====
	   ADC1 的时钟与 PA2 的模拟模式在 HAL_ADC_MspInit 里配, 由任务体内的
	   key_bsp_inst → adkey_inst → adc_init → HAL_ADC_Init 回调触发
	   @note 本段只建任务, 不碰设备: key_bsp_inst() 里的 ADC 初始化与采样自检都要在
	         任务上下文做
	   @warning 返回值必须看: 非 0 = "key" 任务创建失败(内核堆不足) */
	{
		int8_t key_rc = service_key_init();
		if (0 != key_rc)
		{
			printf("\r\n[KEY] service_key_init failed, rc=%d(内核堆不足?)\r\n", (int)key_rc);
			Error_Handler();
		}
		printf("[KEY] service init ok(\"key\" 任务已建; ADKEY 构造/轮询在任务体内做)\r\n");
	}

	/* ===== Nordic 协议服务: USART1(115200 8N1) + DMA2_Stream7(TX)/Stream2(RX) =====
	   一条龙里含板级绑定(DMA/NVIC/USART1 驱动)与 BSP 构造, 都在 osKernelStart 之前
	   可调; RX/TX 两个任务只在这里挂上, 任务体进调度器后才跑
	   @warning ★USART1 与 printf 共用★: 参考工程把 printf 挪到 USART2, 但本板 PA2
	            已被 ADKEY 占用, 故按现状两个口都落在 USART1 —— printf 与 Nordic
	            帧会互相插字节, 且同一 USART1 上有两个 UART_HandleTypeDef
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

#if NORDIC_SVC_LOOPBACK_TEST
		/* 闭环测试: 单帧区间(0x02)与消息区间(0x81)各注册一个回显.
		   @note 必须在 init() 之后调 —— 协议核实例由它构造; 也必须在收到第一帧之前调完
		         (协议核的回调表不做并发保护, 见服务头的 @note) */
		(void)service_nordicprotocol_register_feature(0x02u, nordic_test_echo);
		(void)service_nordicprotocol_register_feature(0x81u, nordic_test_echo);
#endif
	}

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
