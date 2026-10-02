#include "cywatch_adapter_max30102.h"
#include "cywatch_bsp_max30102_driver.h"
#include "iic_hal.h"   /* iic_driver_t (共享软件I2C总线, 由 main.c 提供) */
#include "exti_hal.h" /* exti_driver_t (MAX30102 INT PA3/EXTI3, 由 main.c 提供) */
/* 延时/信号量/时基改用 CMSIS-RTOS v2 (FreeRTOS 封装): osDelay 参数=内核 tick(本工程 1kHz==1ms) */
#include "cmsis_os2.h"

static bsp_max30102_driver_t max30102_instance;

/* 应用层(main.c)提供的总线与中断实例. MAX30102 与 MPU6050 共用同一条软件I2C总线,
   该总线不可重入, 两个服务不要并发访问; 中断链路: EXTI3 → exti_irq_handler →
   应用层回调 → heartrate_bsp_interrupt_cb() → max30102_irq_cb(释放信号量/置标志) */
extern iic_driver_t  iic_instance;           /* 共享软件I2C总线 */
extern exti_driver_t max30102_exti_instance; /* INT PA3 的 EXTI3 实例 */

/* 信号量由本 adapter 层持有并创建(驱动接口不保存实例), 首次 inst 时创建后复用 */
static osSemaphoreId_t max30102_sem_instance;

static max30102_iic_interface_t       max30102_iic_interface_instance;
static max30102_yield_interface_t     max30102_yield_instance;
static max30102_semaphore_interface_t max30102_semaphore_instance;
static max30102_delay_interface_t     max30102_delay_instance;
static max30102_timebase_interface_t  max30102_timebase_instance;
static max30102_interrupt_interface_t max30102_interrupt_instance;

static void yield(void)
{

}

/* cmsis osDelay 返回 osStatus_t, 驱动 delay 接口要求 void(*)(uint32_t), 包一层丢弃返回值.
   @note osDelay 依赖调度器运行——heartrate_bsp_inst() 内含延时, 必须在
         osKernelStart() 之后的任务上下文中调用(不能放在 main 启动内核之前) */
static void max30102_delay_cb(uint32_t ms)
{
    (void)osDelay(ms);
}

/* 时基: OS 模式下 pf_wait_interrupt 走信号量分支, 时基仅需非空(inst 会校验) */
static uint32_t max30102_get_time_cb(void)
{
    return osKernelGetTickCount();
}

/* I2C转发: 驱动接口不带实例, 静态转发到 main.c 的总线实例 iic_instance */
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

/* 信号量等待超时(ms): FIFO 满中断周期 = FIFO深度 / 有效样本率 = 32/100Hz = 320ms.
   留 6 倍余量后仍未等到, 说明中断链断了(EXTI 配置丢失/INT 线脱落/传感器掉电),
   此时必须让 pf_wait_interrupt 返回错误而不是永久挂起——否则服务任务会
   无声卡死在信号量上, 既测不到数据也进不了错误分支, 无法自愈 */
#define MAX30102_SEM_WAIT_MS    (2000U)

/* 信号量: cmsis_os2 → 驱动接口(int8_t).
   pf_wait 在任务上下文阻塞等待(带超时), pf_release 在 ISR 中调用(osSemaphoreRelease 中断安全) */
static int8_t max30102_sem_wait(void)
{
    if (NULL == max30102_sem_instance)
    {
        return -1;
    }

    return (osOK == osSemaphoreAcquire(max30102_sem_instance, MAX30102_SEM_WAIT_MS)) ? 0 : -1;
}

static int8_t max30102_sem_release(void)
{
    if (NULL == max30102_sem_instance)
    {
        return -1;
    }

    return (osOK == osSemaphoreRelease(max30102_sem_instance)) ? 0 : -1;
}

/* EXTI包装: exti_hal pf_enable/disable(int8_t) → max30102_interrupt_interface_t(void) */
static void max30102_int_enable(void)
{
    (void)max30102_exti_instance.pf_enable_interrupt(&max30102_exti_instance);
}

static void max30102_int_disable(void)
{
    (void)max30102_exti_instance.pf_disable_interrupt(&max30102_exti_instance);
}

int8_t heartrate_bsp_inst(void)
{
    /* 信号量由本层持有: 仅首次创建, 服务重试 inst 时复用(不重复 new) */
    if (NULL == max30102_sem_instance)
    {
        max30102_sem_instance = osSemaphoreNew(1U, 0U, NULL);
        if (NULL == max30102_sem_instance)
        {
            return -1; /* 信号量创建失败(堆不足等), 无中断等待能力 */
        }
    }

    max30102_iic_interface_instance.pf_start         = iic_start;
    max30102_iic_interface_instance.pf_stop          = iic_stop;
    max30102_iic_interface_instance.pf_wait_ack      = iic_wait_ack;
    max30102_iic_interface_instance.pf_send_ack      = iic_send_ack;
    max30102_iic_interface_instance.pf_send_not_ack  = iic_send_no_ack;
    max30102_iic_interface_instance.pf_send_bytes    = iic_send_bytes;
    max30102_iic_interface_instance.pf_receive_bytes = iic_receive_bytes;
    max30102_iic_interface_instance.pf_readreg       = iic_readreg;
    max30102_iic_interface_instance.pf_writereg      = iic_writereg;

    max30102_yield_instance.pf_yield = yield;

    max30102_semaphore_instance.pf_wait    = max30102_sem_wait;
    max30102_semaphore_instance.pf_release = max30102_sem_release;

    max30102_delay_instance.pf_delay = max30102_delay_cb;

    max30102_timebase_instance.pf_get_time = max30102_get_time_cb;

    max30102_interrupt_instance.pf_enable_interrupt  = max30102_int_enable;
    max30102_interrupt_instance.pf_disable_interrupt = max30102_int_disable;

    return max30102_inst(&max30102_instance,
                         &max30102_iic_interface_instance,
                         &max30102_yield_instance,
                         &max30102_semaphore_instance,
                         &max30102_delay_instance,
                         &max30102_timebase_instance,
                         &max30102_interrupt_instance);
}

int8_t heartrate_bsp_deinst(void)
{
    return max30102_instance.pf_deinst(&max30102_instance);
}

int8_t heartrate_bsp_read_id(void)
{
    return max30102_instance.pf_read_id(&max30102_instance);
}

int8_t heartrate_bsp_change_to_HR(void)
{
    return max30102_instance.pf_change_to_HR(&max30102_instance);
}

int8_t heartrate_bsp_change_to_spo2(void)
{
    return max30102_instance.pf_change_to_spo2(&max30102_instance);
}

int8_t heartrate_bsp_enable_FIFO_FULL_interrupt(void)
{
    return max30102_instance.pf_enable_FIFO_FULL_interrupt(&max30102_instance);
}

int8_t heartrate_bsp_disable_FIFO_FULL_interrupt(void)
{
    return max30102_instance.pf_disable_FIFO_FULL_interrupt(&max30102_instance);
}

int8_t heartrate_bsp_read_all_FIFO_samples(uint32_t *p_red_buff, uint32_t *p_ir_buff)
{
    return max30102_instance.pf_read_all_FIFO_samples(&max30102_instance, p_red_buff, p_ir_buff);
}

int8_t heartrate_bsp_read_one_sample(uint32_t *p_red, uint32_t *p_ir, uint32_t *sample_size)
{
    return max30102_instance.pf_read_one_sample(&max30102_instance, p_red, p_ir, sample_size);
}

int8_t heartrate_bsp_wait_interrupt(void)
{
    return max30102_instance.pf_wait_interrupt(&max30102_instance);
}

int8_t heartrate_bsp_hibernating(void)
{
    return max30102_instance.pf_hibernating(&max30102_instance);
}

int8_t heartrate_bsp_wakeup(void)
{
    return max30102_instance.pf_wakeup(&max30102_instance);
}

/* ISR 上下文: 仅转驱动 pf_interrupt_cb(释放信号量/置标志), 不做 I2C/printf.
   @note EXTI 回调在 main 中先于 heartrate_bsp_inst() 挂载, 中断若早于实例化到来,
         pf_interrupt_cb 仍为 NULL, 必须判空(调用空函数指针直接 HardFault) */
int8_t heartrate_bsp_interrupt_cb(void)
{
    if (NULL == max30102_instance.pf_interrupt_cb)
    {
        return -1;
    }

    return max30102_instance.pf_interrupt_cb(&max30102_instance);
}
