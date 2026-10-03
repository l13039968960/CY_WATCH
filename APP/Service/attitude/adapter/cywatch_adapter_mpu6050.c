#include "cywatch_adapter_mpu6050.h"
#include "cywatch_bsp_mpu6050_driver.h"
#include "iic_hal.h"
/* 延时改用 CMSIS-RTOS v2 (FreeRTOS 封装): osDelay 参数=内核 tick(本工程 1kHz==1ms) */
#include "cmsis_os2.h"

static bsp_mpu6050_driver_t mpu6050_instance;
extern iic_driver_t iic_instance;

static mpu6050_iic_interface_t mpu6050_iic_interface_instance;
static mpu6050_yield_interface_t mpu6050_yield_instance;
static mpu6050_delay_interface_t mpu6050_delay_instance;

/* 共享总线(PB6/PB7)的占用标记. iic_hal 按 ref_count 门控 —— 计数归零才真的
   释放 SDA/SCL 引脚, 所以 MPU6050/MAX30102/AHT21 三个设备必须各占各放.
   @note 标记不是冗余: 服务的 EVT_INIT 是重试循环(最多调 5 次 inst), 没有它每
         失败一次就多抬一分, 而休眠/析构只放一次 —— 计数只增不减, 总线永远
         回不到 0, 释放出口形同虚设 */
static uint8_t s_iic_claimed = 0U;

static void iic_claim(void)
{
    if (0U == s_iic_claimed)
    {
        (void)iic_instance.pf_init(&iic_instance);
        s_iic_claimed = 1U;
    }
}

static void iic_release(void)
{
    if (0U != s_iic_claimed)
    {
        (void)iic_instance.pf_deinit(&iic_instance);
        s_iic_claimed = 0U;
    }
}

static void yield(void)
{

}

/* cmsis osDelay 返回 osStatus_t, 驱动 delay 接口要求 void(*)(uint32_t), 包一层丢弃返回值.
   @note osDelay 依赖调度器运行——attitudecalculation_bsp_inst() 内含延时, 必须在
         osKernelStart() 之后的任务上下文中调用(不能放在 main 启动内核之前) */
static void mpu6050_delay_cb(uint32_t ms)
{
    (void)osDelay(ms);
}

static int8_t iic_start(void)
{
    return iic_instance.pf_start(&iic_instance);
}

static int8_t iic_stop(void)
{
    return iic_instance.pf_stop(&iic_instance);
}

static int8_t iic_wait_ack(void)
{
    return iic_instance.pf_wait_ack(&iic_instance);
}

static int8_t iic_send_ack(void)
{
    return iic_instance.pf_send_ack(&iic_instance);
}

static int8_t iic_send_no_ack(void)
{
    return iic_instance.pf_send_not_ack(&iic_instance);
}

static int8_t iic_send_bytes(uint8_t *pdata, uint8_t size)
{
    return iic_instance.pf_send_bytes(&iic_instance, pdata, size);
}

static int8_t iic_receive_bytes(uint8_t *pdata, uint8_t size)
{
    return iic_instance.pf_receive_bytes(&iic_instance, pdata, size);
}

static int8_t iic_readreg(uint8_t dev_addr, uint8_t reg,
                          uint8_t *pdata, uint8_t size)
{
    return iic_instance.pf_readreg(&iic_instance, dev_addr, reg, pdata, size);
}

static int8_t iic_writereg(uint8_t dev_addr, uint8_t reg, uint8_t data)
{
    return iic_instance.pf_writereg(&iic_instance, dev_addr, reg, data);
}

int8_t attitudecalculation_bsp_inst(void)
{
    /* 先占住共享总线: 下面的 mpu6050_inst 里就有 I2C 读写(WHO_AM_I) */
    iic_claim();

    mpu6050_iic_interface_instance.pf_start         = iic_start;
    mpu6050_iic_interface_instance.pf_stop          = iic_stop;
    mpu6050_iic_interface_instance.pf_wait_ack      = iic_wait_ack;
    mpu6050_iic_interface_instance.pf_send_ack      = iic_send_ack;
    mpu6050_iic_interface_instance.pf_send_not_ack  = iic_send_no_ack;
    mpu6050_iic_interface_instance.pf_send_bytes    = iic_send_bytes;
    mpu6050_iic_interface_instance.pf_receive_bytes = iic_receive_bytes;
    mpu6050_iic_interface_instance.pf_readreg       = iic_readreg;
    mpu6050_iic_interface_instance.pf_writereg      = iic_writereg;

    mpu6050_yield_instance.pf_yield = yield;

    mpu6050_delay_instance.pf_delay = mpu6050_delay_cb;

    return mpu6050_inst(&mpu6050_instance, &mpu6050_iic_interface_instance, &mpu6050_yield_instance, &mpu6050_delay_instance);
}

int8_t attitudecalculation_bsp_deinst(void)
{
    int8_t ret;

    ret = mpu6050_instance.pf_deinst(&mpu6050_instance);

    /* 析构完再放总线: pf_deinst 里还要经 I2C 把器件送进睡眠 */
    iic_release();

    return ret;
}

int8_t attitudecalculation_bsp_read_id(void)
{
    return mpu6050_instance.pf_read_id(&mpu6050_instance);
}

int8_t attitudecalculation_bsp_read_accel(float *p_accel_x, float *p_accel_y, float *p_accel_z)
{
    return mpu6050_instance.pf_read_accel(&mpu6050_instance, p_accel_x, p_accel_y, p_accel_z);
}

int8_t attitudecalculation_bsp_read_gyro(float *p_gyro_x, float *p_gyro_y, float *p_gyro_z)
{
    return mpu6050_instance.pf_read_gyro(&mpu6050_instance, p_gyro_x, p_gyro_y, p_gyro_z);
}

int8_t attitudecalculation_bsp_hibernating(void)
{
    int8_t ret;

    ret = mpu6050_instance.pf_hibernating(&mpu6050_instance);

    /* 器件已睡, 本设备退出共享总线(计数减一, 归零才真的放引脚) */
    iic_release();

    return ret;
}

int8_t attitudecalculation_bsp_wakeup(void)
{
    int8_t ret;

    /* 先占回总线再唤醒: pf_wakeup 里要经 I2C 写器件 */
    iic_claim();

    ret = mpu6050_instance.pf_wakeup(&mpu6050_instance);

    return ret;
}

