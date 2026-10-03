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
 * 1. lvgl_bsp_indev_inst(): 填 IIC/GPIO/中断/semaphore/delay 五个接口, 裸机再加
 *    timebase(OS_SUPPORTING 下该接口不存在)
 *    (IIC 与 GPIO 两个接口各带 pf_init/pf_deinit) → cst816t_inst()(内部先回调各
 *    外设 init, 再做 RST 复位/寄存器配置/ChipID 自检) → 挂 EXTI2 回调(PB2 下降沿);
 * 2. 之后每次 lvgl_bsp_indev_read_touch() 轮询读 6 字节触摸帧(非阻塞);
 * 3. lvgl_bsp_indev_deinst() 析构.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 总线归属: 触摸那条位带 I2C(PA8=SCL/PB4=SDA)是**独占**总线(线上只有
 *       CST816T), 所以实例与配置都归本 adapter, 在 lvgl_bsp_indev_inst() 里
 *       自己构造。只有被多器件共用的总线才需要外部提供实例: 那是 main.c 里的
 *       `iic_instance`(PB6/PB7, MPU6050/MAX30102/AHT21 共用), 与本文件无关。
 *
 * @note 设备自身的控制脚(RST PA15 / INT PB2)由本 adapter 自己配置, 不上交 main.c:
 *       与 W25Q64/adapter 自持 CS(PB12)同一做法。本工程**没有** MX_GPIO_Init(参考工程
 *       Dirver_Test/Core/Src/gpio.c 那份迁移时未搬入, main.c 的调用点也是空的), 所以
 *       没有任何别的地方会配这些脚。
 *       RST(PA15) 的配置现已并入 gpio 接口的 pf_init(裸 HAL: 开 GPIOA 时钟 + 先写 ODR
 *       再配 MODER), 由驱动在 cst816t_init 里回调, 不再在 inst 里内联。
 *
 * @note `lvgl_bsp_indev_inst()` 内含 osDelay(复位等待) → 必须在 osKernelStart()
 *       之后的**任务上下文**调用, 不能放在 main 启动内核之前.
 *
 * @note 中断纪律: EXTI2 回调在 ISR 内只做两件事——转发驱动的 pf_interrupt_cb
 *       (释放信号量 + 置 irq_flag), 绝不做 I2C/printf。软 I2C 不可重入, 禁止在
 *       ISR 中访问总线。
 *
 * @note 轮询 vs 中断: 本层读触摸固定用轮询(CST816T_READ_POLL), 所以 PB2 的 EXTI
 *       实际上「接了但不用」——留着是因为 cst816t_inst() 在 OS_SUPPORTING 下强制
 *       校验中断接口非空(否则返回 -7), 且将来做低功耗要切 CST816T_READ_WAIT。
 *
 * @note 实例归属: 驱动接口结构体一律**不带实例成员**(见驱动头文件), 于是本层改用
 *       **文件静态量**兜住四个"实例"——自持的 `touch_iic_instance`(总线)与
 *       `touch_rst_gpio`/`touch_exti`(两根控制脚)、`cst816t_sem_handle`(信号量句柄)。
 *       因此所有转发函数都不带实例形参。⚠ 形参表必须与接口结构体逐字一致: 多写一个
 *       void * 就是 -Wincompatible-function-pointer-types 编译失败(本工程里这是
 *       错误而不是警告)。
 ******************************************************************************/
#include "cywatch_adapter_indev.h"

#include "cywatch_bsp_cst816t_driver.h"
#include "iic_hal.h"   /* iic_driver_t: 触摸专用位带 I2C, 本 adapter 自持 */
#include "gpio_hal.h" /* gpio_driver_t: RST(PA15) */
#include "exti_hal.h" /* exti_driver_t: INT(PB2) */
#include "delay.h"   /* delay_us: EXTI 驱动 inst 会校验 pf_delay_us 非空 */
#include "main.h"                /* GPIO_TypeDef / GPIO_MODE_* / GPIO_PIN_* */
#include "cmsis_os2.h"           /* osDelay / osKernelGetTickCount / osSemaphore* */

/***********************************Defines************************************/
/* 触摸接线(用户确认, 均不在 CubeMX 纳管范围):
   RST=PA15(GPIO), INT=PB2(EXTI2下降沿), SCL=PA8 / SDA=PB4(独占位带 I2C) */
#define CST816T_RST_PORT GPIOA
#define CST816T_RST_PIN  GPIO_PIN_15
#define CST816T_INT_PORT GPIOB
#define CST816T_INT_PIN  GPIO_PIN_2
#define CST816T_SCL_PORT GPIOA
#define CST816T_SCL_PIN  GPIO_PIN_8
#define CST816T_SDA_PORT GPIOB
#define CST816T_SDA_PIN  GPIO_PIN_4

/* EXTI 抢占优先级: 必须 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(5),
   因为该 ISR 内会经 pf_release 调 osSemaphoreRelease(FromISR 安全 API) */
#define CST816T_INT_PREEMPT_PRIO 6
/***********************************Defines************************************/

/**********************************Declaring***********************************/
static bsp_cst816t_driver_t cst816t_instance;

/* ---- 触摸总线: 独占位带 I2C(PA8=SCL / PB4=SDA) ----
   线上只有 CST816T 一个器件, 是一条独立总线, 所以实例与配置都归本层, 与自持的
   RST(PA15)/INT(PB2) 同一条理。被多器件共用的总线(如 main.c 的 PB6/PB7 那条)
   才需要外部提供实例。 */
static iic_driver_t touch_iic_instance;
static iic_bus_t touch_iic_bus_cfg =
{
	/* 初始态: 推挽输出+上拉(与 iic_hal 里 SDA 收发切换时的参数无关) */
	.sda = { .p_port = CST816T_SDA_PORT, .pins = CST816T_SDA_PIN,
			 .mode = GPIO_MODE_OUTPUT_PP, .pull = GPIO_PULLUP,
			 .speed = GPIO_SPEED_FREQ_HIGH },
	.scl = { .p_port = CST816T_SCL_PORT, .pins = CST816T_SCL_PIN,
			 .mode = GPIO_MODE_OUTPUT_PP, .pull = GPIO_PULLUP,
			 .speed = GPIO_SPEED_FREQ_HIGH },
};
static iic_delay_interface_t touch_iic_delay_instance =
{
	.pf_delay_us = delay_us,
};

/* 触摸总线的互斥量: 位带总线不可重入, 读事务在 lvgl 任务里按 ~33ms 周期跑,
   一次事务(start..stop)被 tick 抢占切开会踩总线。
   句柄归本层持有: 接口的两个回调没有形参 */
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

/* 设备自身控制脚(RST/INT)的底层实例, 归本层所有 */
static gpio_driver_t touch_rst_gpio;
static exti_driver_t touch_exti;

static cst816t_iic_interface_t         cst816t_iic_interface_instance;
static cst816t_gpio_interface_t        cst816t_gpio_interface_instance;
static cst816t_semaphore_interface_t   cst816t_semaphore_instance;
static cst816t_delay_interface_t       cst816t_delay_instance;
#ifndef OS_SUPPORTING
/* 时基实例只有裸机的中断轮询等待会读(见驱动头文件); OS_SUPPORTING 下本层
   连类型都不存在, 实例一并省掉 */
static cst816t_timebase_interface_t    cst816t_timebase_instance;
#endif // OS_SUPPORTING
static cst816t_interrupt_interface_t   cst816t_interrupt_instance;

/* 中断信号量: 轮询读模式下不会被 wait, 但 ISR 会 release(见 cst816t_irq_cb),
   故必须是真实可用的内核对象, 不能是自旋桩 */
static osSemaphoreId_t cst816t_sem_handle = NULL;

/* ============================= 底层实例配置 ============================= */
static gpio_cfg_t touch_rst_gpio_cfg =
{
	.p_port = CST816T_RST_PORT,
	.pins   = CST816T_RST_PIN,
	.mode   = GPIO_MODE_OUTPUT_PP,
	.pull   = GPIO_NOPULL,
	.speed  = GPIO_SPEED_FREQ_LOW,
	.af     = 0,
};

static exti_cfg_t touch_exti_cfg =
{
	.gpio =
	{
		.p_port = CST816T_INT_PORT,
		.pins   = CST816T_INT_PIN,
		.mode   = GPIO_MODE_IT_FALLING,
		.pull   = GPIO_PULLUP, /* CST816T INT 空闲高电平, 防悬空误触发 */
		.speed  = GPIO_SPEED_FREQ_HIGH,
	},
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
 * @param   level[in] 0=低(复位), 1=高(释放复位)
 *
 * @return  无
 *
 * @note    接口不带实例形参, 固定操作本层的 touch_rst_gpio(PA15)
 *****************************************************************************/
static void touch_rst_set_level(uint8_t level)
{
	(void)touch_rst_gpio.pf_write(&touch_rst_gpio, level);
}

/******************************************************************************
 * @name    touch_int_enable
 * @brief   EXTI使能: exti_hal 的 pf_enable_interrupt(int8_t) → 驱动要求的 void 回调
 * @param   无
 *
 * @return  无
 *
 * @note    接口不带实例形参, 固定操作本层的 touch_exti(PB2)
 *****************************************************************************/
static void touch_int_enable(void)
{
	(void)touch_exti.pf_enable_interrupt(&touch_exti);
}

/******************************************************************************
 * @name    touch_int_disable
 * @brief   EXTI失能: exti_hal 的 pf_disable_interrupt(int8_t) → 驱动要求的 void 回调
 * @param   无
 *
 * @return  无
 *
 * @note    接口不带实例形参, 固定操作本层的 touch_exti(PB2)
 *****************************************************************************/
static void touch_int_disable(void)
{
	(void)touch_exti.pf_disable_interrupt(&touch_exti);
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

#ifndef OS_SUPPORTING
/******************************************************************************
 * @name    cst816t_get_time_cb
 * @brief   时基: 内核 tick 计数(供驱动的中断等待超时计数使用)
 * @param   无
 *
 * @return  当前内核 tick(ms)
 *
 * @note    仅裸机编入: 只被 cst816t_timebase_instance 引用, 而该实例在
 *          OS_SUPPORTING 下不存在, 留着会变成 -Wunused-function
 *****************************************************************************/
static uint32_t cst816t_get_time_cb(void)
{
	return osKernelGetTickCount();
}
#endif // OS_SUPPORTING

/* ---- 信号量接口(ISR 中 release, 任务上下文 wait) ---- */

/******************************************************************************
 * @name    cst816t_sem_wait
 * @brief   阻塞等待中断信号量(仅 CST816T_READ_WAIT 模式会走到)
 * @param   无
 *
 * @return  0 success
 *         -1 句柄空或等待失败
 *
 * @note    接口不带实例形参: 句柄改由本层的文件静态量 `cst816t_sem_handle` 持有
 *          (与 main.c 里 lcd_spi_sem_handle 兜住 spi_hal 信号量的写法一致)
 *****************************************************************************/
static int8_t cst816t_sem_wait(void)
{
	if (NULL == cst816t_sem_handle)
	{
		return -1;
	}

	return (osOK == osSemaphoreAcquire(cst816t_sem_handle, osWaitForever)) ? 0 : -1;
}

/******************************************************************************
 * @name    cst816t_sem_release
 * @brief   释放中断信号量(在 EXTI ISR 中调用)
 * @param   无
 *
 * @return  0 success
 *         -1 句柄空或释放失败
 *
 * @note    调用方 ISR 优先级 6 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(5),
 *          满足 FreeRTOS 对 FromISR API 的约束
 *****************************************************************************/
static int8_t cst816t_sem_release(void)
{
	if (NULL == cst816t_sem_handle)
	{
		return -1;
	}

	return (osOK == osSemaphoreRelease(cst816t_sem_handle)) ? 0 : -1;
}

/* ---- I2C 转发(接口不带实例形参, 固定转发到本层的 touch_iic_instance) ----
   @note iic_hal 那一侧是"带实例形参"的方言, 所以这里显式把 &touch_iic_instance
         当作它的第一个实参传下去; 本适配器只服务这一条触摸总线, 不会传别的实例 */

static int8_t touch_iic_start(void)
{
	return touch_iic_instance.pf_start(&touch_iic_instance);
}

static int8_t touch_iic_stop(void)
{
	return touch_iic_instance.pf_stop(&touch_iic_instance);
}

static int8_t touch_iic_wait_ack(void)
{
	return touch_iic_instance.pf_wait_ack(&touch_iic_instance);
}

static int8_t touch_iic_send_ack(void)
{
	return touch_iic_instance.pf_send_ack(&touch_iic_instance);
}

static int8_t touch_iic_send_not_ack(void)
{
	return touch_iic_instance.pf_send_not_ack(&touch_iic_instance);
}

static int8_t touch_iic_send_bytes(uint8_t *pdata, uint8_t size)
{
	return touch_iic_instance.pf_send_bytes(&touch_iic_instance, pdata, size);
}

static int8_t touch_iic_receive_bytes(uint8_t *pdata, uint8_t size)
{
	return touch_iic_instance.pf_receive_bytes(&touch_iic_instance, pdata, size);
}

static int8_t touch_iic_readreg(uint8_t dev_addr, uint8_t reg,
                                uint8_t *pdata, uint8_t size)
{
	return touch_iic_instance.pf_readreg(&touch_iic_instance, dev_addr, reg, pdata, size);
}

static int8_t touch_iic_writereg(uint8_t dev_addr, uint8_t reg, uint8_t data)
{
	return touch_iic_instance.pf_writereg(&touch_iic_instance, dev_addr, reg, data);
}

/******************************************************************************
 * @name    cst816t_iic_init
 * @brief   I2C总线初始化: 转发到本层的 touch_iic_instance
 *
 * @return  0 success, 非0 见 iic_hal 的 pf_init
 *
 * @note    驱动在 cst816t_init 里回调本函数。iic_hal 那侧带引用计数, 若总线已被
 *          别的使用者init过, 这里只累加计数、不会把 PA8/PB4 重配一遍。
 *****************************************************************************/
static int8_t cst816t_iic_init(void)
{
	return touch_iic_instance.pf_init(&touch_iic_instance);
}

/******************************************************************************
 * @name    cst816t_iic_deinit
 * @brief   I2C总线反初始化: 转发到本层的 touch_iic_instance
 *
 * @return  0 success, 非0 见 iic_hal 的 pf_deinit
 *
 * @note    引用计数减到 0 才真正把 PA8/PB4 释放成模拟输入; 之后器件只能靠
 *          RST 引脚唤醒(见 cst816t_wakeup), 要再通信必须先 pf_init 把总线开回来。
 *****************************************************************************/
static int8_t cst816t_iic_deinit(void)
{
	return touch_iic_instance.pf_deinit(&touch_iic_instance);
}

/******************************************************************************
 * @name    lvgl_bsp_indev_gpio_init
 * @brief   RST引脚(PA15)配置到运行态: 输出推挽 + 置高释放复位
 *
 * @return  0 success
 *         -1 引脚配置失败
 *
 * @note    CST816T 低电平复位, 所以初始电平必须是高。先写 ODR 再配 MODER:
 *          输出锁存器预置成"释放复位", 引脚一变成输出就立刻是高, 不会给芯片
 *          打出一个假复位脉冲(gpio_write 直写 BSRR, 输入态下也有效)。
 *****************************************************************************/
static int8_t lvgl_bsp_indev_gpio_init(void)
{
	(void)touch_rst_gpio.pf_write(&touch_rst_gpio, 1);

	return touch_rst_gpio.pf_init(&touch_rst_gpio);
}

/******************************************************************************
 * @name    lvgl_bsp_indev_gpio_deinit
 * @brief   释放RST引脚(PA15), 恢复复位默认态并按低功耗收尾
 *
 * @return  0 success
 *         -1 实例为空
 *
 * @note    端口时钟由 gpio_hal 统一管理且永不关闭: 同一端口上还挂着位带 I2C 的
 *          SCL(PA8)、LCD 的 DC/CS/SCK/MOSI/背光等。
 *****************************************************************************/
static int8_t lvgl_bsp_indev_gpio_deinit(void)
{
	return touch_rst_gpio.pf_deinit(&touch_rst_gpio);
}

/* ============================= 构造/析构 ============================= */

/******************************************************************************
 * @name    lvgl_bsp_indev_inst
 * @brief   构造CST816T实例: 总线 → RST GPIO → 挂六个驱动接口 → cst816t_inst → 挂EXTI回调
 * @param   无
 *
 * @return  0 success
 *         -1 触摸总线构造失败(互斥量创建失败或 iic_driver_inst 参数非法)
 *         -2 RST GPIO 构造失败
 *         -3 EXTI 构造失败
 *         -4 信号量创建失败
 *         <0  cst816t_inst 失败, 透传驱动的返回码(负值语义见驱动头文件, -9 = ChipID自检失败)
 *
 * @note    须在 osKernelStart() 之后的任务上下文调用(内部含 osDelay);
 *          可重复调用用于失败重试——每次都会重跑 ChipID 自检
 *****************************************************************************/
int8_t lvgl_bsp_indev_inst(void)
{
	int8_t ret = 0;

	/* 0. 触摸总线(PA8/PB4): 只构造, 不碰硬件 —— 配引脚由下方 I2C 接口的 pf_init
	      (cst816t_iic_init)在 CST816T 驱动初始化时做。
	      互斥量只创建一次(inst 可被重试, 不能每次泄漏一个内核对象) */
	if (NULL == touch_iic_mutex_handle)
	{
		touch_iic_mutex_handle = osMutexNew(NULL);
		if (NULL == touch_iic_mutex_handle)
		{
			return -1;
		}
	}

	ret = iic_driver_inst(&touch_iic_instance, &touch_iic_bus_cfg,
						  &touch_iic_delay_instance, &touch_iic_mutex_instance);
	if (0 != ret)
	{
		return -1;
	}

	/* 1. RST引脚(PA15): 只构造, 不碰硬件 —— 真正配引脚由下方接口的 pf_init
	      (lvgl_bsp_indev_gpio_init)在 CST816T 驱动初始化时做 */
	ret = gpio_driver_inst(&touch_rst_gpio, &touch_rst_gpio_cfg);
	if (0 != ret)
	{
		return -2;
	}

	/* 2. EXTI中断(PB2下降沿): 触摸事件通知, ISR仅转发驱动的 pf_interrupt_cb
	      (引脚与 GPIOB 端口时钟都由 exti 内部的 gpio 实例开, 这里不用手工开) */
	ret = exti_driver_inst(&touch_exti, &touch_exti_cfg,
						   &touch_exti_delay_instance);
	if (0 != ret)
	{
		return -3;
	}

	ret = touch_exti.pf_init(&touch_exti);
	if (0 != ret)
	{
		return -3;
	}

	/* 3. 中断信号量: 只创建一次(inst 可被重试, 不能每次泄漏一个内核对象) */
	if (NULL == cst816t_sem_handle)
	{
		cst816t_sem_handle = osSemaphoreNew(1U, 0U, NULL);
		if (NULL == cst816t_sem_handle)
		{
			return -4;
		}
	}

	/* 4. 挂I2C接口(转发到本层的 touch_iic_instance) */
	cst816t_iic_interface_instance.pf_init          = cst816t_iic_init;
	cst816t_iic_interface_instance.pf_deinit        = cst816t_iic_deinit;
	cst816t_iic_interface_instance.pf_start         = touch_iic_start;
	cst816t_iic_interface_instance.pf_stop          = touch_iic_stop;
	cst816t_iic_interface_instance.pf_wait_ack      = touch_iic_wait_ack;
	cst816t_iic_interface_instance.pf_send_ack      = touch_iic_send_ack;
	cst816t_iic_interface_instance.pf_send_not_ack  = touch_iic_send_not_ack;
	cst816t_iic_interface_instance.pf_send_bytes    = touch_iic_send_bytes;
	cst816t_iic_interface_instance.pf_receive_bytes = touch_iic_receive_bytes;
	cst816t_iic_interface_instance.pf_readreg       = touch_iic_readreg;
	cst816t_iic_interface_instance.pf_writereg      = touch_iic_writereg;

	/* 5. 挂GPIO(RST)接口 */
	cst816t_gpio_interface_instance.pf_init            = lvgl_bsp_indev_gpio_init;
	cst816t_gpio_interface_instance.pf_deinit          = lvgl_bsp_indev_gpio_deinit;
	cst816t_gpio_interface_instance.pf_gpio_set_level  = touch_rst_set_level;

	/* 6. 挂信号量/延时/时基接口
	   @note 信号量句柄由本层静态量 cst816t_sem_handle 直接持有, 不再经接口结构体
	         传递(接口已无实例成员); yield 接口随驱动头文件一并删除 */
	cst816t_semaphore_instance.pf_wait              = cst816t_sem_wait;
	cst816t_semaphore_instance.pf_release           = cst816t_sem_release;

	cst816t_delay_instance.pf_delay = cst816t_delay_cb;

#ifndef OS_SUPPORTING
	cst816t_timebase_instance.pf_get_time = cst816t_get_time_cb;
#endif // OS_SUPPORTING

	/* 7. 挂中断接口 */
	cst816t_interrupt_instance.pf_enable_interrupt  = touch_int_enable;
	cst816t_interrupt_instance.pf_disable_interrupt = touch_int_disable;

	/* 8. 构造驱动: 接口校验 → 寄存器配置(中断源/手势/时序) → ChipID 自检 */
	ret = cst816t_inst(&cst816t_instance,
					   &cst816t_iic_interface_instance,
					   &cst816t_gpio_interface_instance,
					   &cst816t_semaphore_instance,
					   &cst816t_delay_instance,
#ifndef OS_SUPPORTING
					   &cst816t_timebase_instance,
#endif // OS_SUPPORTING
					   &cst816t_interrupt_instance);
	if (0 != ret)
	{
		return ret;
	}

	/* 9. 挂EXTI回调(ISR → cst816t_irq_cb 释放信号量 + 置 irq_flag)。
	   @note 这里**只挂回调, 不使能中断**: LVGL 侧走的是轮询(CST816T_READ_POLL),
	         不依赖中断, 所以 EXTI2 的 NVIC/IMR 一直没开, cst816t_irq_cb 与上面的
	         信号量都处于"备而未用"的状态(留着是为了支持驱动的 CST816T_READ_WAIT
	         模式, 那段代码仍被编译进来, 只占几十字节).
	         若要切到中断模式: 先挂回调(此处已做)再调
	         lvgl_bsp_indev_enable_interrupt(), 顺序反过来会让中断在回调为空时
	         进来——EXTI2 ISR 优先级 6 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_
	         PRIORITY(5), FromISR 调用合法 */
	(void)touch_exti.pf_attach_callback(&touch_exti, touch_exti_cb);

	return 0;
}

/******************************************************************************
 * @name    lvgl_bsp_indev_deinst
 * @brief   析构CST816T实例(先关中断, 再清驱动)
 * @param   无
 *
 * @return  0 success
 *         -1 驱动返回的实例空指针错误
 *****************************************************************************/
int8_t lvgl_bsp_indev_deinst(void)
{
	(void)touch_exti.pf_disable_interrupt(&touch_exti);

	return cst816t_instance.pf_deinst(&cst816t_instance);
}

/* ============================= 能力转发 ============================= */

/******************************************************************************
 * @name    lvgl_bsp_indev_read_id
 * @brief   读取ChipID(通信自检)
 * @param   无
 *
 * @return  >=0 ChipID
 *         -1 instance null
 *         -2 I2C 读失败
 *****************************************************************************/
int8_t lvgl_bsp_indev_read_id(void)
{
	return cst816t_instance.pf_read_id(&cst816t_instance);
}

/******************************************************************************
 * @name    lvgl_bsp_indev_read_touch
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
int8_t lvgl_bsp_indev_read_touch(uint8_t *p_gesture_id, uint8_t *p_finger_num,
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
 * @name    lvgl_bsp_indev_enable_interrupt
 * @brief   使能触摸中断(EXTI2)
 * @param   无
 *
 * @return  0 success / -1 instance null / -2.. 见驱动 @return
 *****************************************************************************/
int8_t lvgl_bsp_indev_enable_interrupt(void)
{
	return cst816t_instance.pf_enable_interrupt(&cst816t_instance);
}

/******************************************************************************
 * @name    lvgl_bsp_indev_disable_interrupt
 * @brief   失能触摸中断(EXTI2)
 * @param   无
 *
 * @return  0 success / -1 instance null / -2.. 见驱动 @return
 *****************************************************************************/
int8_t lvgl_bsp_indev_disable_interrupt(void)
{
	return cst816t_instance.pf_disable_interrupt(&cst816t_instance);
}

/******************************************************************************
 * @name    lvgl_bsp_indev_hibernating
 * @brief   进入深度休眠(仅外部复位可唤醒)
 * @param   无
 *
 * @return  0 success / -1 instance null / -2 I2C 写失败
 *****************************************************************************/
int8_t lvgl_bsp_indev_hibernating(void)
{
	return cst816t_instance.pf_hibernating(&cst816t_instance);
}

/******************************************************************************
 * @name    lvgl_bsp_indev_wakeup
 * @brief   复位唤醒并重新初始化
 * @param   无
 *
 * @return  0 success / -1 instance null / -2 gpio null / -3 re-init failed
 *****************************************************************************/
int8_t lvgl_bsp_indev_wakeup(void)
{
	return cst816t_instance.pf_wakeup(&cst816t_instance);
}
