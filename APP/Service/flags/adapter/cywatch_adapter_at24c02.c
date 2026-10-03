/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_adapter_at24c02.c
 *
 * @brief AT24C02 适配器: 把 BSP 驱动的无实例接口绑到应用层的独占 I2C 总线实例上.
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
#include "cywatch_adapter_at24c02.h"
#include "cywatch_bsp_at24c02_driver.h"
#include "iic_hal.h" /* iic_driver_t (AT24C02 独占的软件I2C总线, 本 adapter 自持) */
#include "delay.h"   /* delay_us: 总线的延时接口 */
/* 延时/时基改用 CMSIS-RTOS v2 (FreeRTOS 封装): osDelay 参数=内核 tick(本工程 1kHz==1ms) */
#include "cmsis_os2.h"

static bsp_at24c02_driver_t at24c02_instance;

/* ---- AT24C02 独占的位带软件 I2C(PB10=SCL / PB3=SDA) ----
   @note 线上只有 AT24C02 一个器件, 是一条独立总线, 所以实例与配置都归本层,
         与 main.c 里那条被 MPU6050/MAX30102/AHT21 共用的 PB6/PB7 无关。
   @note 不注入互斥量(NULL): 单任务访问。若将来有多个任务读写本适配器, 必须补上
         互斥量, 否则位带时序会被 tick 切在 start..stop 中间。
   @note PB3 复位后是 JTDO: 拿来当 SDA 会占用 JTAG 的 JTDO/SWO 跟踪功能,
         SWD(PA13/PA14)不受影响, 调试器按 SW-DP(SWD)连接即可;
         F4 系写 GPIO_MODER 就能把该脚从复用功能释放为普通输出(iic 的 pf_init
         经 gpio_hal 完成), 无需 SYSCFG 重映射 */
static iic_driver_t at24c02_iic_instance;
static iic_bus_t at24c02_iic_bus_cfg =
{
	.sda = { .p_port = GPIOB, .pins = GPIO_PIN_3,  .mode = GPIO_MODE_OUTPUT_PP,
			 .pull = GPIO_PULLUP, .speed = GPIO_SPEED_FREQ_HIGH },
	.scl = { .p_port = GPIOB, .pins = GPIO_PIN_10, .mode = GPIO_MODE_OUTPUT_PP,
			 .pull = GPIO_PULLUP, .speed = GPIO_SPEED_FREQ_HIGH },
};
static iic_delay_interface_t at24c02_iic_delay_instance =
{
	.pf_delay_us = delay_us,
};

static at24c02_iic_interface_t      at24c02_iic_interface_instance;
static at24c02_delay_interface_t    at24c02_delay_instance;
static at24c02_timebase_interface_t at24c02_timebase_instance;

/* cmsis osDelay 返回 osStatus_t, 驱动 delay 接口要求 void(*)(uint32_t), 包一层丢弃返回值.
   @note osDelay 依赖调度器运行——storage_bsp_at24c02_inst() 内含上电延时与
         wait_busy 轮询, 必须在 osKernelStart() 之后的任务上下文中调用
         (不能放在 main 启动内核之前) */
static void at24c02_delay_cb(uint32_t ms)
{
    (void)osDelay(ms);
}

/* 时基: 供 at24c02_wait_busy 的写周期轮询超时计数使用(超时门限 10ms) */
static uint32_t at24c02_get_time_cb(void)
{
    return osKernelGetTickCount();
}

/* I2C转发: 驱动接口不带实例, 静态转发到本层的 at24c02_iic_instance */
static int8_t iic_readreg(uint8_t dev_addr, uint8_t reg,
                          uint8_t *pdata, uint8_t size)
{
    return at24c02_iic_instance.pf_readreg(&at24c02_iic_instance,
                                           dev_addr, reg, pdata, size);
}

static int8_t iic_write_frame(uint8_t dev_addr, uint8_t *pdata, uint8_t size)
{
    return at24c02_iic_instance.pf_write_frame(&at24c02_iic_instance,
                                               dev_addr, pdata, size);
}

static int8_t iic_read_frame(uint8_t dev_addr, uint8_t *pdata, uint8_t size)
{
    return at24c02_iic_instance.pf_read_frame(&at24c02_iic_instance,
                                              dev_addr, pdata, size);
}

int8_t storage_bsp_at24c02_inst(void)
{
    /* 1. 构造总线实例 + 配引脚(PB3/PB10): iic_init 只配 GPIO, 不含 I2C 时序
          也没有 osDelay, 可在任务上下文里直接调 */
    if (0 != iic_driver_inst(&at24c02_iic_instance, &at24c02_iic_bus_cfg,
                             &at24c02_iic_delay_instance, NULL))
    {
        return -1;
    }

    if (0 != at24c02_iic_instance.pf_init(&at24c02_iic_instance))
    {
        return -1;
    }

    /* 2. 挂接口并构造设备实例 */
    at24c02_iic_interface_instance.pf_readreg     = iic_readreg;
    at24c02_iic_interface_instance.pf_write_frame = iic_write_frame;
    at24c02_iic_interface_instance.pf_read_frame  = iic_read_frame;

    at24c02_delay_instance.pf_delay = at24c02_delay_cb;

    at24c02_timebase_instance.pf_get_time = at24c02_get_time_cb;

    /* AT24C02 无 ID 寄存器(EEPROM), 驱动自检走 init 内的 ACK 探测:
       at24c02_inst 内部调 pf_init, 器件无应答时返回 -5 */
    return at24c02_inst(&at24c02_instance,
                        &at24c02_iic_interface_instance,
                        &at24c02_delay_instance,
                        &at24c02_timebase_instance);
}

int8_t storage_bsp_at24c02_deinst(void)
{
    return at24c02_instance.pf_deinst(&at24c02_instance);
}

int8_t storage_bsp_at24c02_read(uint16_t addr, uint8_t *pdata, uint16_t size)
{
    return at24c02_instance.pf_read(&at24c02_instance, addr, pdata, size);
}

int8_t storage_bsp_at24c02_write(uint16_t addr, uint8_t *pdata, uint16_t size)
{
    return at24c02_instance.pf_write(&at24c02_instance, addr, pdata, size);
}
