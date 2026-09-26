/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_adapter_cst816t.c
 *
 * @par dependencies
 * - cywatch_adapter_cst816t.h
 * - ../hal_driver/Inc/cywatch_bsp_cst816t_driver.h
 * - ../../IIC/iic_hal.h
 * - ../../GPIO/gpio_hal.h
 * - ../../EXTI/exti_hal.h
 * - cmsis_os2.h
 *
 * @author	zw1194
 *
 * @brief LVGL 服务消费 CST816T 的 adapter 实现.
 *
 * Processing flow:
 *
 * 1. lvgl_bsp_cst816t_inst(): RST(PA15) GPIO → 填 IIC/GPIO/中断/semaphore/
 *    yield/delay/timebase 七个接口 → cst816t_inst()(寄存器配置 + ChipID 自检)
 *    → 挂 EXTI2 回调(PB2 下降沿);
 * 2. 之后每次 lvgl_bsp_cst816t_read_touch() 轮询读 6 字节触摸帧(非阻塞);
 * 3. lvgl_bsp_cst816t_deinst() 析构.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 总线归属: 触摸专用位带 I2C(PA8=SCL/PB4=SDA)实例 `touch_iic_instance` 由
 *       应用层 main.c 创建, 本 adapter 只 extern 引用(与 MPU6050/W25Q64 adapter
 *       同一写法), **不自己再建总线**。
 *
 *       ⚠ 该符号绝不能叫 `iic_instance`: 那个名字被 MPU6050/adapter 占用, 指的是
 *         传感器总线 PB6/PB7。同名会让触摸驱动静默地去操作 MPU 那条总线——
 *         编译链接全过, 运行时读到垃圾数据, 极难定位。
 *
 * @note 设备自身的控制脚(RST PA15 / INT PB2)由本 adapter 自己配置, 不上交 main.c:
 *       与 W25Q64/adapter 自持 CS(PB12)同一做法。这四个脚都不在 MX_GPIO_Init 里
 *       (CubeMX 未纳管), 必须由本层 __HAL_RCC_GPIOx_CLK_ENABLE() + 配好。
 *
 * @note `lvgl_bsp_cst816t_inst()` 内含 osDelay(复位等待) → 必须在 osKernelStart()
 *       之后的**任务上下文**调用, 不能放在 main 启动内核之前.
 *
 * @note 中断纪律: EXTI2 回调在 ISR 内只做两件事——转发驱动的 pf_interrupt_cb
 *       (释放信号量 + 置 irq_flag), 绝不做 I2C/printf。软 I2C 不可重入, 禁止在
 *       ISR 中访问总线。
 *
 * @note 轮询 vs 中断: 本层读触摸固定用轮询(CST816T_READ_POLL), 所以 PB2 的 EXTI
 *       实际上「接了但不用」——留着是因为 cst816t_inst() 在 OS_SUPPORTING 下强制
 *       校验中断接口非空(否则返回 -8), 且将来做低功耗要切 CST816T_READ_WAIT。
 ******************************************************************************/
#include "cywatch_adapter_cst816t.h"

#include "../hal_driver/Inc/cywatch_bsp_cst816t_driver.h"
#include "../../IIC/iic_hal.h"   /* iic_driver_t: 触摸专用位带 I2C, 由 main.c 提供 */
#include "../../GPIO/gpio_hal.h" /* gpio_driver_t: RST(PA15) */
#include "../../EXTI/exti_hal.h" /* exti_driver_t: INT(PB2) */
#include "../../system/Delay/delay.h"   /* delay_us: GPIO/EXTI 驱动 inst 会校验 pf_delay_us 非空 */
#include "main.h"                /* __HAL_RCC_GPIOx_CLK_ENABLE */
#include "cmsis_os2.h"           /* osDelay / osKernelGetTickCount / osSemaphore* */

/***********************************Defines************************************/
/* 触摸接线(用户确认, 均不在 CubeMX 纳管范围):
   RST=PA15(GPIO), INT=PB2(EXTI2下降沿); SCL/SDA 见 main.c 的 touch_iic_instance */
#define CST816T_RST_PORT GPIOA
#define CST816T_RST_PIN  GPIO_PIN_15
#define CST816T_INT_PORT GPIOB
#define CST816T_INT_PIN  GPIO_PIN_2

/* EXTI 抢占优先级: 必须 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(5),
   因为该 ISR 内会经 pf_release 调 osSemaphoreRelease(FromISR 安全 API) */
#define CST816T_INT_PREEMPT_PRIO 6
/***********************************Defines************************************/

/**********************************Declaring***********************************/
static bsp_cst816t_driver_t cst816t_instance;

/* 应用层(main.c)提供的总线实例: 触摸独占位带 I2C(PA8=SCL/PB4=SDA) */
extern iic_driver_t touch_iic_instance;

/* 设备自身控制脚(RST/INT)的底层实例, 归本层所有 */
static gpio_driver_t touch_rst_gpio;
static exti_driver_t touch_exti;

static cst816t_iic_interface_t         cst816t_iic_interface_instance;
static cst816t_gpio_interface_t        cst816t_gpio_interface_instance;
static cst816t_yield_interface_t       cst816t_yield_instance;
static cst816t_semaphore_interface_t   cst816t_semaphore_instance;
static cst816t_delay_interface_t       cst816t_delay_instance;
static cst816t_timebase_interface_t    cst816t_timebase_instance;
static cst816t_interrupt_interface_t   cst816t_interrupt_instance;

/* 中断信号量: 轮询读模式下不会被 wait, 但 ISR 会 release(见 cst816t_irq_cb),
   故必须是真实可用的内核对象, 不能是自旋桩 */
static osSemaphoreId_t cst816t_sem_handle = NULL;

/* ============================= 底层实例配置 ============================= */
static gpio_cfg_t touch_rst_gpio_cfg =
{
	.p_port = CST816T_RST_PORT,
	.pin    = CST816T_RST_PIN,
	.mode   = GPIO_MODE_OUTPUT_PP,
	.pull   = GPIO_NOPULL,
	.speed  = GPIO_SPEED_FREQ_LOW,
};

static gpio_delay_interface_t touch_gpio_delay_instance =
{
	.pf_delay_us = delay_us,
};

static exti_cfg_t touch_exti_cfg =
{
	.p_port           = CST816T_INT_PORT,
	.pin              = CST816T_INT_PIN,
	.mode             = GPIO_MODE_IT_FALLING,
	.pull             = GPIO_PULLUP, /* CST816T INT 空闲高电平, 防悬空误触发 */
	.irqn             = EXTI2_IRQn,
	.preempt_priority = CST816T_INT_PREEMPT_PRIO,
	.sub_priority     = 0,
};

static exti_delay_interface_t touch_exti_delay_instance =
{
	.pf_delay_us = delay_us,
};

/* ============================= 接口包装 ============================= */

/******************************************************************************
 * @name    touch_rst_set_level
 * @brief   RST电平写: gpio_hal 的 pf_write(int8_t) → 驱动要求的 void 回调
 * @param   p_gpio_instance[in] gpio实例(gpio_driver_t*)
 * @param   level[in] 0=低(复位), 1=高(释放复位)
 *
 * @return  无
 *****************************************************************************/
static void touch_rst_set_level(void *p_gpio_instance, uint8_t level)
{
	(void)touch_rst_gpio.pf_write(p_gpio_instance, level);
}

/******************************************************************************
 * @name    touch_int_enable
 * @brief   EXTI使能: exti_hal 的 pf_enable_interrupt(int8_t) → 驱动要求的 void 回调
 * @param   p_interrupt_instance[in] EXTI实例(exti_driver_t*)
 *
 * @return  无
 *****************************************************************************/
static void touch_int_enable(void *p_interrupt_instance)
{
	(void)touch_exti.pf_enable_interrupt(p_interrupt_instance);
}

/******************************************************************************
 * @name    touch_int_disable
 * @brief   EXTI失能: exti_hal 的 pf_disable_interrupt(int8_t) → 驱动要求的 void 回调
 * @param   p_interrupt_instance[in] EXTI实例(exti_driver_t*)
 *
 * @return  无
 *****************************************************************************/
static void touch_int_disable(void *p_interrupt_instance)
{
	(void)touch_exti.pf_disable_interrupt(p_interrupt_instance);
}

/******************************************************************************
 * @name    touch_exti_cb
 * @brief   EXTI2中断回调(ISR上下文): 转发到驱动的 pf_interrupt_cb
 * @param   p_ctx[in] 未使用
 *
 * @return  无
 *
 * @note    ISR内只释放信号量 + 置 irq_flag, 不做任何 I2C(软 I2C 不可重入)
 *****************************************************************************/
static void touch_exti_cb(void *p_ctx)
{
	(void)p_ctx;
	cst816t_instance.pf_interrupt_cb(&cst816t_instance);
}

/******************************************************************************
 * @name    yield
 * @brief   让出CPU接口的空实现(驱动要求非空, 本工程由 RTOS 调度)
 * @param   无
 *
 * @return  无
 *****************************************************************************/
static void yield(void)
{
}

/******************************************************************************
 * @name    cst816t_delay_cb
 * @brief   延时: osDelay 返回 osStatus_t, 驱动要求 void(*)(uint32_t), 包一层丢弃返回值
 * @param   ms[in] 毫秒
 *
 * @return  无
 *
 * @note    依赖调度器运行 —— inst 内含复位等待, 必须在任务上下文调用
 *****************************************************************************/
static void cst816t_delay_cb(uint32_t ms)
{
	(void)osDelay(ms);
}

/******************************************************************************
 * @name    cst816t_get_time_cb
 * @brief   时基: 内核 tick 计数(供驱动的中断等待超时计数使用)
 * @param   无
 *
 * @return  当前内核 tick(ms)
 *****************************************************************************/
static uint32_t cst816t_get_time_cb(void)
{
	return osKernelGetTickCount();
}

/* ---- 信号量接口(ISR 中 release, 任务上下文 wait) ---- */

/******************************************************************************
 * @name    cst816t_sem_wait
 * @brief   阻塞等待中断信号量(仅 CST816T_READ_WAIT 模式会走到)
 * @param   p_semaphore_instance[in] osSemaphoreId_t 句柄
 *
 * @return  0 success
 *         -1 句柄空或等待失败
 *****************************************************************************/
static int8_t cst816t_sem_wait(void *p_semaphore_instance)
{
	osSemaphoreId_t sem = (osSemaphoreId_t)p_semaphore_instance;

	if (NULL == sem)
	{
		return -1;
	}

	return (osOK == osSemaphoreAcquire(sem, osWaitForever)) ? 0 : -1;
}

/******************************************************************************
 * @name    cst816t_sem_release
 * @brief   释放中断信号量(在 EXTI ISR 中调用)
 * @param   p_semaphore_instance[in] osSemaphoreId_t 句柄
 *
 * @return  0 success
 *         -1 句柄空或释放失败
 *
 * @note    调用方 ISR 优先级 6 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(5),
 *          满足 FreeRTOS 对 FromISR API 的约束
 *****************************************************************************/
static int8_t cst816t_sem_release(void *p_semaphore_instance)
{
	osSemaphoreId_t sem = (osSemaphoreId_t)p_semaphore_instance;

	if (NULL == sem)
	{
		return -1;
	}

	return (osOK == osSemaphoreRelease(sem)) ? 0 : -1;
}

/* ---- I2C 转发(驱动接口自带实例参数, 直接透传到 main.c 的总线实例) ---- */

static int8_t touch_iic_start(void *p_iic_instance)
{
	return touch_iic_instance.pf_start(p_iic_instance);
}

static int8_t touch_iic_stop(void *p_iic_instance)
{
	return touch_iic_instance.pf_stop(p_iic_instance);
}

static int8_t touch_iic_wait_ack(void *p_iic_instance)
{
	return touch_iic_instance.pf_wait_ack(p_iic_instance);
}

static int8_t touch_iic_send_ack(void *p_iic_instance)
{
	return touch_iic_instance.pf_send_ack(p_iic_instance);
}

static int8_t touch_iic_send_not_ack(void *p_iic_instance)
{
	return touch_iic_instance.pf_send_not_ack(p_iic_instance);
}

static int8_t touch_iic_send_bytes(void *p_iic_instance, uint8_t *pdata, uint8_t size)
{
	return touch_iic_instance.pf_send_bytes(p_iic_instance, pdata, size);
}

static int8_t touch_iic_receive_bytes(void *p_iic_instance, uint8_t *pdata, uint8_t size)
{
	return touch_iic_instance.pf_receive_bytes(p_iic_instance, pdata, size);
}

/* ============================= 构造/析构 ============================= */

/******************************************************************************
 * @name    lvgl_bsp_cst816t_inst
 * @brief   构造CST816T实例: RST GPIO → 挂八个驱动接口 → cst816t_inst → 挂EXTI回调
 * @param   无
 *
 * @return  0 success
 *         -1 RST GPIO 构造失败
 *         -2 EXTI 构造失败
 *         -3 信号量创建失败
 *         -4 cst816t_inst 失败(负值语义见驱动头文件 @return, -10 = ChipID自检失败)
 *
 * @note    须在 osKernelStart() 之后的任务上下文调用(内部含 osDelay);
 *          可重复调用用于失败重试——每次都会重跑 ChipID 自检
 *****************************************************************************/
int8_t lvgl_bsp_cst816t_inst(void)
{
	int8_t ret = 0;

	/* 1. RST引脚(PA15): 初始化为输出并置高释放复位(CST816T 低电平复位) */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	ret = gpio_driver_inst(&touch_rst_gpio, &touch_rst_gpio_cfg,
						   &touch_gpio_delay_instance);
	if (0 != ret)
	{
		return -1;
	}
	(void)touch_rst_gpio.pf_write(&touch_rst_gpio, 1);

	/* 2. EXTI中断(PB2下降沿): 触摸事件通知, ISR仅转发驱动的 pf_interrupt_cb */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	ret = exti_driver_inst(&touch_exti, &touch_exti_cfg,
						   &touch_exti_delay_instance);
	if (0 != ret)
	{
		return -2;
	}

	/* 3. 中断信号量: 只创建一次(inst 可被重试, 不能每次泄漏一个内核对象) */
	if (NULL == cst816t_sem_handle)
	{
		cst816t_sem_handle = osSemaphoreNew(1U, 0U, NULL);
		if (NULL == cst816t_sem_handle)
		{
			return -3;
		}
	}

	/* 4. 挂I2C接口(转发到 main.c 的 touch_iic_instance) */
	cst816t_iic_interface_instance.p_iic_instance   = (void *)&touch_iic_instance;
	cst816t_iic_interface_instance.pf_start         = touch_iic_start;
	cst816t_iic_interface_instance.pf_stop          = touch_iic_stop;
	cst816t_iic_interface_instance.pf_wait_ack      = touch_iic_wait_ack;
	cst816t_iic_interface_instance.pf_send_ack      = touch_iic_send_ack;
	cst816t_iic_interface_instance.pf_send_not_ack  = touch_iic_send_not_ack;
	cst816t_iic_interface_instance.pf_send_bytes    = touch_iic_send_bytes;
	cst816t_iic_interface_instance.pf_receive_bytes = touch_iic_receive_bytes;

	/* 5. 挂GPIO(RST)接口 */
	cst816t_gpio_interface_instance.p_gpio_instance   = (void *)&touch_rst_gpio;
	cst816t_gpio_interface_instance.pf_gpio_set_level = touch_rst_set_level;

	/* 6. 挂yield/信号量/延时/时基接口 */
	cst816t_yield_instance.pf_yield = yield;

	cst816t_semaphore_instance.p_semaphore_instance = (void *)cst816t_sem_handle;
	cst816t_semaphore_instance.pf_wait              = cst816t_sem_wait;
	cst816t_semaphore_instance.pf_release           = cst816t_sem_release;

	cst816t_delay_instance.pf_delay = cst816t_delay_cb;

	cst816t_timebase_instance.pf_get_time = cst816t_get_time_cb;

	/* 7. 挂中断接口 */
	cst816t_interrupt_instance.p_interrupt_instance = (void *)&touch_exti;
	cst816t_interrupt_instance.pf_enable_interrupt  = touch_int_enable;
	cst816t_interrupt_instance.pf_disable_interrupt = touch_int_disable;

	/* 8. 构造驱动: 七接口校验 → 寄存器配置(中断源/手势/时序) → ChipID 自检 */
	ret = cst816t_inst(&cst816t_instance,
					   &cst816t_iic_interface_instance,
					   &cst816t_gpio_interface_instance,
					   &cst816t_yield_instance,
					   &cst816t_semaphore_instance,
					   &cst816t_delay_instance,
					   &cst816t_timebase_instance,
					   &cst816t_interrupt_instance);
	if (0 != ret)
	{
		return -4;
	}

	/* 9. 挂EXTI回调(ISR → cst816t_irq_cb 释放信号量 + 置 irq_flag)。
	   @note 这里**只挂回调, 不使能中断**: LVGL 侧走的是轮询(CST816T_READ_POLL),
	         不依赖中断, 所以 EXTI2 的 NVIC/IMR 一直没开, cst816t_irq_cb 与上面的
	         信号量都处于"备而未用"的状态(留着是为了支持驱动的 CST816T_READ_WAIT
	         模式, 那段代码仍被编译进来, 只占几十字节).
	         若要切到中断模式: 先挂回调(此处已做)再调
	         lvgl_bsp_cst816t_enable_interrupt(), 顺序反过来会让中断在回调为空时
	         进来——EXTI2 ISR 优先级 6 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_
	         PRIORITY(5), FromISR 调用合法 */
	(void)touch_exti.pf_attach_callback(&touch_exti, touch_exti_cb);

	return 0;
}

/******************************************************************************
 * @name    lvgl_bsp_cst816t_deinst
 * @brief   析构CST816T实例(先关中断, 再清驱动)
 * @param   无
 *
 * @return  0 success
 *         -1 驱动返回的实例空指针错误
 *****************************************************************************/
int8_t lvgl_bsp_cst816t_deinst(void)
{
	(void)touch_exti.pf_disable_interrupt(&touch_exti);

	return cst816t_instance.pf_deinst(&cst816t_instance);
}

/* ============================= 能力转发 ============================= */

/******************************************************************************
 * @name    lvgl_bsp_cst816t_read_id
 * @brief   读取ChipID(通信自检)
 * @param   无
 *
 * @return  >=0 ChipID
 *         -1 instance null
 *         -2 I2C 读失败
 *****************************************************************************/
int8_t lvgl_bsp_cst816t_read_id(void)
{
	return cst816t_instance.pf_read_id(&cst816t_instance);
}

/******************************************************************************
 * @name    lvgl_bsp_cst816t_read_touch
 * @brief   轮询读一帧触摸数据(固定 POLL 模式, 非阻塞, 不等待中断)
 * @param   p_gesture_id[out] 手势编码
 * @param   p_finger_num[out] 触摸点数(0=无手指)
 * @param   p_x[out] 面板原始X坐标(未缩放)
 * @param   p_y[out] 面板原始Y坐标(未缩放)
 *
 * @return  0 success (本次I2C读成功, 不代表有手指)
 *         -1 cst816t_instance null
 *         -2 I2C 读失败
 *         -3 出参空指针
 *****************************************************************************/
int8_t lvgl_bsp_cst816t_read_touch(uint8_t *p_gesture_id, uint8_t *p_finger_num,
								   uint16_t *p_x, uint16_t *p_y)
{
	if (NULL == p_gesture_id || NULL == p_finger_num ||
		NULL == p_x || NULL == p_y)
	{
		return -3;
	}

	return cst816t_instance.pf_read_touch(&cst816t_instance,
										  p_gesture_id, p_finger_num,
										  p_x, p_y,
										  CST816T_READ_POLL);
}

/******************************************************************************
 * @name    lvgl_bsp_cst816t_enable_interrupt
 * @brief   使能触摸中断(EXTI2)
 * @param   无
 *
 * @return  0 success / -1 instance null / -2.. 见驱动 @return
 *****************************************************************************/
int8_t lvgl_bsp_cst816t_enable_interrupt(void)
{
	return cst816t_instance.pf_enable_interrupt(&cst816t_instance);
}

/******************************************************************************
 * @name    lvgl_bsp_cst816t_disable_interrupt
 * @brief   失能触摸中断(EXTI2)
 * @param   无
 *
 * @return  0 success / -1 instance null / -2.. 见驱动 @return
 *****************************************************************************/
int8_t lvgl_bsp_cst816t_disable_interrupt(void)
{
	return cst816t_instance.pf_disable_interrupt(&cst816t_instance);
}

/******************************************************************************
 * @name    lvgl_bsp_cst816t_hibernating
 * @brief   进入深度休眠(仅外部复位可唤醒)
 * @param   无
 *
 * @return  0 success / -1 instance null / -2 I2C 写失败
 *****************************************************************************/
int8_t lvgl_bsp_cst816t_hibernating(void)
{
	return cst816t_instance.pf_hibernating(&cst816t_instance);
}

/******************************************************************************
 * @name    lvgl_bsp_cst816t_wakeup
 * @brief   复位唤醒并重新初始化
 * @param   无
 *
 * @return  0 success / -1 instance null / -2 gpio null / -3 re-init failed
 *****************************************************************************/
int8_t lvgl_bsp_cst816t_wakeup(void)
{
	return cst816t_instance.pf_wakeup(&cst816t_instance);
}
