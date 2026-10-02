/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_adapter_aht21.c
 *
 * @brief AHT21 适配器: 把 BSP 驱动的无实例接口绑到应用层的共享 I2C 总线实例上.
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
#include "cywatch_adapter_aht21.h"
#include "cywatch_bsp_aht21_driver.h"
#include "iic_hal.h"  /* iic_driver_t (共享软件I2C总线, 由 main.c 提供) */
/* 延时改用 CMSIS-RTOS v2: osDelay 参数 = 内核 tick(本工程 1kHz == 1ms) */
#include "cmsis_os2.h"

static bsp_aht21_driver_t aht21_instance;

/* 应用层(main.c)提供的总线实例. AHT21 与 MPU6050/MAX30102 共用同一条软件I2C
   总线(PB6=SCL / PB7=SDA), 该总线不可重入 —— 并发访问由 iic_hal 在
   readreg/write_frame/read_frame 内部整段(start..stop)持互斥量兜住 */
extern iic_driver_t iic_instance;

static aht21_iic_interface_t   aht21_iic_interface_instance;
static aht21_delay_interface_t aht21_delay_instance;

/* cmsis osDelay 返回 osStatus_t, 驱动 delay 接口要求 void(*)(uint32_t), 包一层丢返回值.
   @note osDelay 依赖调度器 —— humiture_bsp_inst() 内含上百毫秒延时(上电等待 100ms +
         自校准 40ms), 必须在 osKernelStart() 之后的任务上下文里调用 */
static void aht21_delay_cb(uint32_t ms)
{
    (void)osDelay(ms);
}

/* I2C转发: 驱动接口不带实例, 静态转发到 main.c 的总线实例 iic_instance */
static int8_t iic_readreg(uint8_t dev_addr, uint8_t reg,
                          uint8_t *pdata, uint8_t size)
{
    return iic_instance.pf_readreg(&iic_instance, dev_addr, reg, pdata, size);
}

static int8_t iic_write_frame(uint8_t dev_addr, uint8_t *pdata, uint8_t size)
{
    return iic_instance.pf_write_frame(&iic_instance, dev_addr, pdata, size);
}

static int8_t iic_read_frame(uint8_t dev_addr, uint8_t *pdata, uint8_t size)
{
    return iic_instance.pf_read_frame(&iic_instance, dev_addr, pdata, size);
}

int8_t humiture_bsp_inst(void)
{
    aht21_iic_interface_instance.pf_readreg     = iic_readreg;
    aht21_iic_interface_instance.pf_write_frame = iic_write_frame;
    aht21_iic_interface_instance.pf_read_frame  = iic_read_frame;

    aht21_delay_instance.pf_delay = aht21_delay_cb;

    return aht21_inst(&aht21_instance,
                      &aht21_iic_interface_instance,
                      &aht21_delay_instance);
}

int8_t humiture_bsp_deinst(void)
{
    return aht21_instance.pf_deinst(&aht21_instance);
}

int8_t humiture_bsp_read_id(void)
{
    return aht21_instance.pf_read_id(&aht21_instance);
}

int8_t humiture_bsp_read_temp_humi(float *p_temperature, float *p_humidity)
{
    return aht21_instance.pf_read_temp_humi(&aht21_instance,
                                            p_temperature, p_humidity);
}

int8_t humiture_bsp_hibernating(void)
{
    return aht21_instance.pf_hibernating(&aht21_instance);
}

int8_t humiture_bsp_wakeup(void)
{
    return aht21_instance.pf_wakeup(&aht21_instance);
}
