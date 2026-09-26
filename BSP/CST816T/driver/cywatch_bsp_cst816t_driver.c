/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_bsp_cst816t_driver.c
 *
 * @par dependencies
 * - cywatch_bsp_cst816t_driver.h
 *
 * @author zw1194
 *
 * @brief Implete the HAL operations of CST816T and corresponding opetions.
 *
 * Processing flow:
 *
 * call directly.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
#include "../Inc/cywatch_bsp_cst816t_driver.h"

/* 默认配置 */
#define CST816T_STARTUP_DELAY_MS 100  /* 复位/上电后等待初始化完成 */
#define CST816T_RESET_PULSE_MS 10     /* RST低电平复位脉冲宽度 */
#define CST816T_IRQ_TIMEOUT_MS 1000   /* 裸机中断等待超时计数(ms) */

/* IrqCtl 默认: 仅使能触摸中断.
 * 手势中断已禁用: 手势模式下抬手会先上报 fingers!=0 + 占位坐标(0x1FF/0xFFF)的
 * "手势帧", 进入LVGL会污染指针状态机导致切屏卡死(见 lv_port_indev.c 越界过滤);
 * 手表UI 自跟踪拖拽距离判断手势, 不需要芯片手势检测 */
#define CST816T_DEFAULT_IRQ_CTL (CST816T_IRQ_EN_TOUCH)

/* MotionMask 默认: 关闭全部手势检测(双击/连续上下/连续左右), 原因见上方说明 */
#define CST816T_DEFAULT_MOTION_MASK (0)

/* 时序参数默认值(数据手册默认) */
#define CST816T_DEFAULT_LONG_PRESS_TICK 100 /* 长按时间门限 */
#define CST816T_DEFAULT_NOR_SCAN_PER    1   /* 正常扫描周期 */
#define CST816T_DEFAULT_AUTO_SLEEP_TIME 2   /* 自动休眠时间 */

#define CST816T_I2C_ADDR_W (CST816T_I2C_ADDR << 1)
#define CST816T_I2C_ADDR_R ((CST816T_I2C_ADDR << 1) | 0x01)

static int8_t cst816t_deinst(struct bsp_cst816t_driver *p_cst816t_instance);
static int8_t cst816t_init(struct bsp_cst816t_driver *p_cst816t_instance);
static int8_t cst816t_deinit(struct bsp_cst816t_driver *p_cst816t_instance);
static int8_t cst816t_read_id(struct bsp_cst816t_driver *p_cst816t_instance);
static int8_t cst816t_read_touch(struct bsp_cst816t_driver *p_cst816t_instance,
								 uint8_t *p_gesture_id, uint8_t *p_finger_num,
								 uint16_t *p_x, uint16_t *p_y,
								 uint8_t block);
static int8_t cst816t_enable_interrupt(struct bsp_cst816t_driver *p_cst816t_instance);
static int8_t cst816t_disable_interrupt(struct bsp_cst816t_driver *p_cst816t_instance);
static int8_t cst816t_hibernating(struct bsp_cst816t_driver *p_cst816t_instance);
static int8_t cst816t_wakeup(struct bsp_cst816t_driver *p_cst816t_instance);

/******************************************************************************
 * @name    cst816t_write_reg
 * @brief   向CST816T指定寄存器写入单字节数据
 * @param   p_cst816t_instance[in]
 * @param   reg[in] 寄存器地址
 * @param   data[in] 写入数据
 *
 * @return  0 success
 *         -1 cst816t_instance null
 *         -2 i2c write error
 *****************************************************************************/
static int8_t cst816t_write_reg(struct bsp_cst816t_driver *p_cst816t_instance,
								uint8_t reg, uint8_t data)
{
	cst816t_iic_interface_t *p_iic;
	uint8_t buf[3];

	if (NULL == p_cst816t_instance)
	{
		return -1;
	}

	p_iic = p_cst816t_instance->p_iic_interface;

	/* 组装I2C帧: [DevAddr(W), RegAddr, Data] */
	buf[0] = CST816T_I2C_ADDR_W;
	buf[1] = reg;
	buf[2] = data;

	if (0 != p_iic->pf_start(p_iic->p_iic_instance))
	{
		return -2;
	}

	if (0 != p_iic->pf_send_bytes(p_iic->p_iic_instance, buf, 3))
	{
		p_iic->pf_stop(p_iic->p_iic_instance);
		return -2;
	}

	p_iic->pf_stop(p_iic->p_iic_instance);

	return 0;
}

/******************************************************************************
 * @name    cst816t_read_reg
 * @brief   从CST816T指定寄存器读取单字节数据
 * @param   p_cst816t_instance[in]
 * @param   reg[in] 寄存器地址
 * @param   p_data[out] 读取数据缓冲区
 *
 * @return  0 success
 *         -1 cst816t_instance null
 *         -2 i2c read error
 *****************************************************************************/
static int8_t cst816t_read_reg(struct bsp_cst816t_driver *p_cst816t_instance,
							   uint8_t reg, uint8_t *p_data)
{
	cst816t_iic_interface_t *p_iic;
	uint8_t buf[2];

	if (NULL == p_cst816t_instance)
	{
		return -1;
	}

	p_iic = p_cst816t_instance->p_iic_interface;

	/* Phase 1: 写寄存器地址 [DevAddr(W), RegAddr] */
	buf[0] = CST816T_I2C_ADDR_W;
	buf[1] = reg;

	if (0 != p_iic->pf_start(p_iic->p_iic_instance))
	{
		return -2;
	}

	if (0 != p_iic->pf_send_bytes(p_iic->p_iic_instance, buf, 2))
	{
		p_iic->pf_stop(p_iic->p_iic_instance);
		return -2;
	}

	/* Phase 2: 重复起始 + 读 [DevAddr(R)] → 接收数据 */
	buf[0] = CST816T_I2C_ADDR_R;

	if (0 != p_iic->pf_start(p_iic->p_iic_instance))
	{
		p_iic->pf_stop(p_iic->p_iic_instance);
		return -2;
	}

	if (0 != p_iic->pf_send_bytes(p_iic->p_iic_instance, buf, 1))
	{
		p_iic->pf_stop(p_iic->p_iic_instance);
		return -2;
	}

	if (0 != p_iic->pf_receive_bytes(p_iic->p_iic_instance, p_data, 1))
	{
		p_iic->pf_stop(p_iic->p_iic_instance);
		return -2;
	}

	p_iic->pf_stop(p_iic->p_iic_instance);

	return 0;
}

/******************************************************************************
 * @name    cst816t_read_multi_reg
 * @brief   从CST816T连续读取多字节数据(支持自动地址递增)
 * @param   p_cst816t_instance[in]
 * @param   reg[in] 起始寄存器地址
 * @param   p_data[out] 读取数据缓冲区
 * @param   size[in] 读取字节数
 *
 * @return  0 success
 *         -1 cst816t_instance null
 *         -2 i2c read error
 *****************************************************************************/
static int8_t cst816t_read_multi_reg(struct bsp_cst816t_driver *p_cst816t_instance,
									 uint8_t reg, uint8_t *p_data, uint8_t size)
{
	cst816t_iic_interface_t *p_iic;
	uint8_t buf[2];

	if (NULL == p_cst816t_instance)
	{
		return -1;
	}

	p_iic = p_cst816t_instance->p_iic_interface;

	/* Phase 1: 写寄存器地址 [DevAddr(W), RegAddr] */
	buf[0] = CST816T_I2C_ADDR_W;
	buf[1] = reg;

	if (0 != p_iic->pf_start(p_iic->p_iic_instance))
	{
		return -2;
	}

	if (0 != p_iic->pf_send_bytes(p_iic->p_iic_instance, buf, 2))
	{
		p_iic->pf_stop(p_iic->p_iic_instance);
		return -2;
	}

	/* Phase 2: 重复起始 + 读 [DevAddr(R)] → 接收多字节 */
	buf[0] = CST816T_I2C_ADDR_R;

	if (0 != p_iic->pf_start(p_iic->p_iic_instance))
	{
		p_iic->pf_stop(p_iic->p_iic_instance);
		return -2;
	}

	if (0 != p_iic->pf_send_bytes(p_iic->p_iic_instance, buf, 1))
	{
		p_iic->pf_stop(p_iic->p_iic_instance);
		return -2;
	}

	if (0 != p_iic->pf_receive_bytes(p_iic->p_iic_instance, p_data, size))
	{
		p_iic->pf_stop(p_iic->p_iic_instance);
		return -2;
	}

	p_iic->pf_stop(p_iic->p_iic_instance);

	return 0;
}

/******************************************************************************
 * @name    cst816t_irq_cb
 * @brief   本层中断服务函数(由底层EXTI在ISR中回调)
 * @param   p_cst816t_instance[in] 本层驱动实例
 *
 * @note    在中断上下文调用, OS释放信号量/裸机置irq_flag, 供 pf_read_touch
 *          轮询。本函数不做任何I2C读操作, 避免与主循环I2C并发导致总线损坏;
 *          实际触摸数据由主循环调用 pf_read_touch 读取。
 *****************************************************************************/
static void cst816t_irq_cb(bsp_cst816t_driver_t *p_cst816t_instance)
{
	if (NULL == p_cst816t_instance)
	{
		return;
	}

	/* OS: 释放信号量唤醒等待任务; 裸机: 置中断标志供 pf_read_touch 轮询 */
#ifdef OS_SUPPORTING
	if (NULL != p_cst816t_instance->p_semaphore_interface &&
		NULL != p_cst816t_instance->p_semaphore_interface->pf_release)
	{
		p_cst816t_instance->p_semaphore_interface->pf_release(
			p_cst816t_instance->p_semaphore_interface->p_semaphore_instance);
	}
#endif // OS_SUPPORTING
	p_cst816t_instance->irq_flag = 1;
}

/******************************************************************************
 * @name    cst816t_inst
 * @brief   instancetiate the CST816T instance
 * @param   p_cst816t_instance[in]
 * @param   p_iic_interface[in]
 * @param   p_gpio_interface[in]
 * @param   p_yield_interface[in] (OS_SUPPORTING)
 * @param   p_semaphore_interface[in] (OS_SUPPORTING)
 * @param   p_delay_interface[in]
 * @param   p_timebase_interface[in]
 * @param   p_interrupt_interface[in]
 *
 * @return  0 success
 *         -1 cst816t_instance null
 *         -2 iic_interface null
 *         -3 gpio_interface null
 *         -4 rtos_yield null
 *         -5 semaphore null
 *         -6 delay null
 *         -7 timebase null
 *         -8 interrupt null
 *         -9 init failed
 *         -10 cst816t ChipID read failed
 *****************************************************************************/
int8_t cst816t_inst(bsp_cst816t_driver_t *p_cst816t_instance,
					cst816t_iic_interface_t *p_iic_interface,
					cst816t_gpio_interface_t *p_gpio_interface,
#ifdef OS_SUPPORTING
					cst816t_yield_interface_t *p_yield_interface,
					cst816t_semaphore_interface_t *p_semaphore_interface,
#endif // OS_SUPPORTING
					cst816t_delay_interface_t *p_delay_interface,
					cst816t_timebase_interface_t *p_timebase_interface,
					cst816t_interrupt_interface_t *p_interrupt_interface)
{
	int8_t id = 0;

	if (NULL == p_cst816t_instance)
	{
		return -1;
	}

	/* 构造 */
	if (NULL == p_iic_interface)
	{
		return -2;
	}
	else
	{
		if (NULL == p_iic_interface->pf_receive_bytes ||
			NULL == p_iic_interface->pf_send_ack ||
			NULL == p_iic_interface->pf_send_bytes ||
			NULL == p_iic_interface->pf_send_not_ack ||
			NULL == p_iic_interface->pf_start ||
			NULL == p_iic_interface->pf_stop ||
			NULL == p_iic_interface->pf_wait_ack ||
			NULL == p_iic_interface->p_iic_instance)
		{
			return -2;
		}
	}

	if (NULL == p_gpio_interface)
	{
		return -3;
	}
	else
	{
		if (NULL == p_gpio_interface->pf_gpio_set_level ||
			NULL == p_gpio_interface->p_gpio_instance)
		{
			return -3;
		}
	}

#ifdef OS_SUPPORTING
	if (NULL == p_yield_interface)
	{
		return -4;
	}
	else
	{
		if (NULL == p_yield_interface->pf_yield)
		{
			return -4;
		}
	}

	if (NULL == p_semaphore_interface)
	{
		return -5;
	}
	else
	{
		if (NULL == p_semaphore_interface->pf_wait ||
			NULL == p_semaphore_interface->pf_release)
		{
			return -5;
		}
	}
#endif // OS_SUPPORTING

	if (NULL == p_delay_interface)
	{
		return -6;
	}
	else
	{
		if (NULL == p_delay_interface->pf_delay)
		{
			return -6;
		}
	}

	if (NULL == p_timebase_interface)
	{
		return -7;
	}
	else
	{
		if (NULL == p_timebase_interface->pf_get_time)
		{
			return -7;
		}
	}

	if (NULL == p_interrupt_interface)
	{
		return -8;
	}
	else
	{
		if (NULL == p_interrupt_interface->pf_enable_interrupt)
		{
			return -8;
		}
	}

	p_cst816t_instance->p_iic_interface = p_iic_interface;
	p_cst816t_instance->p_gpio_interface = p_gpio_interface;
#ifdef OS_SUPPORTING
	p_cst816t_instance->p_yield_interface = p_yield_interface;
	p_cst816t_instance->p_semaphore_interface = p_semaphore_interface;
#endif // OS_SUPPORTING
	p_cst816t_instance->p_delay_interface = p_delay_interface;
	p_cst816t_instance->p_timebase_interface = p_timebase_interface;
	p_cst816t_instance->p_interrupt_interface = p_interrupt_interface;
	p_cst816t_instance->irq_flag = 0;
	p_cst816t_instance->irq_start_tick = 0;

	p_cst816t_instance->pf_inst = cst816t_inst;
	p_cst816t_instance->pf_deinst = cst816t_deinst;
	p_cst816t_instance->pf_init = cst816t_init;
	p_cst816t_instance->pf_deinit = cst816t_deinit;
	p_cst816t_instance->pf_read_id = cst816t_read_id;
	p_cst816t_instance->pf_read_touch = cst816t_read_touch;
	p_cst816t_instance->pf_enable_interrupt = cst816t_enable_interrupt;
	p_cst816t_instance->pf_disable_interrupt = cst816t_disable_interrupt;
	p_cst816t_instance->pf_hibernating = cst816t_hibernating;
	p_cst816t_instance->pf_wakeup = cst816t_wakeup;
	p_cst816t_instance->pf_interrupt_cb = cst816t_irq_cb;

	/* 初始化 */
	if (0 != p_cst816t_instance->pf_init(p_cst816t_instance))
	{
		p_cst816t_instance->pf_deinst(p_cst816t_instance);
		return -9;
	}

	/* 自检: 读取ChipID确认I2C通信正常(ChipID由固件决定, 不校验具体值) */
	id = p_cst816t_instance->pf_read_id(p_cst816t_instance);
	if (-1 == id || -2 == id)
	{
		p_cst816t_instance->pf_deinst(p_cst816t_instance);
		return -10;
	}
	else
	{
		return 0;
	}
}

/******************************************************************************
 * @name    cst816t_deinst
 * @brief   析构CST816T实例
 * @param   p_cst816t_instance[in]
 *
 * @return  0 success
 *         -1 cst816t_instance null
 *****************************************************************************/
static int8_t cst816t_deinst(struct bsp_cst816t_driver *p_cst816t_instance)
{
	if (NULL == p_cst816t_instance)
		return -1;

	p_cst816t_instance->pf_deinit(p_cst816t_instance);

	p_cst816t_instance->p_iic_interface = NULL;
	p_cst816t_instance->p_gpio_interface = NULL;
#ifdef OS_SUPPORTING
	p_cst816t_instance->p_yield_interface = NULL;
	p_cst816t_instance->p_semaphore_interface = NULL;
#endif // OS_SUPPORTING
	p_cst816t_instance->p_delay_interface = NULL;
	p_cst816t_instance->p_timebase_interface = NULL;
	p_cst816t_instance->p_interrupt_interface = NULL;
	p_cst816t_instance->irq_flag = 0;
	p_cst816t_instance->irq_start_tick = 0;
	p_cst816t_instance->pf_inst = NULL;
	p_cst816t_instance->pf_deinst = NULL;
	p_cst816t_instance->pf_init = NULL;
	p_cst816t_instance->pf_deinit = NULL;
	p_cst816t_instance->pf_read_id = NULL;
	p_cst816t_instance->pf_read_touch = NULL;
	p_cst816t_instance->pf_enable_interrupt = NULL;
	p_cst816t_instance->pf_disable_interrupt = NULL;
	p_cst816t_instance->pf_hibernating = NULL;
	p_cst816t_instance->pf_wakeup = NULL;
	p_cst816t_instance->pf_interrupt_cb = NULL;

	return 0;
}

/******************************************************************************
 * @name    cst816t_init
 * @brief   CST816T初始化: 等待上电/复位完成、配置中断源、手势掩码与扫描时序
 * @param   p_cst816t_instance[in]
 *
 * @return  0 success
 *         -1 cst816t_instance null
 *         -2 irq ctl config failed
 *         -3 motion mask config failed
 *         -4 long press tick config failed
 *         -5 nor scan per config failed
 *         -6 auto sleep time config failed
 *****************************************************************************/
static int8_t cst816t_init(struct bsp_cst816t_driver *p_cst816t_instance)
{
	int8_t ret = 0;

	if (NULL == p_cst816t_instance)
	{
		return -1;
	}

	/* 1. 等待芯片上电/复位完成(≥100ms) */
	p_cst816t_instance->p_delay_interface->pf_delay(CST816T_STARTUP_DELAY_MS);

	/* 2. 配置中断控制(使能触摸中断 + 手势中断) */
	ret = cst816t_write_reg(p_cst816t_instance,
							CST816T_IRQ_CTL,
							CST816T_DEFAULT_IRQ_CTL);
	if (0 != ret)
	{
		return -2;
	}

	/* 3. 配置手势使能掩码 */
	ret = cst816t_write_reg(p_cst816t_instance,
							CST816T_MOTION_MASK,
							CST816T_DEFAULT_MOTION_MASK);
	if (0 != ret)
	{
		return -3;
	}

	/* 4. 配置长按时间门限 */
	ret = cst816t_write_reg(p_cst816t_instance,
							CST816T_LONG_PRESS_TICK,
							CST816T_DEFAULT_LONG_PRESS_TICK);
	if (0 != ret)
	{
		return -4;
	}

	/* 5. 配置正常扫描周期 */
	ret = cst816t_write_reg(p_cst816t_instance,
							CST816T_NOR_SCAN_PER,
							CST816T_DEFAULT_NOR_SCAN_PER);
	if (0 != ret)
	{
		return -5;
	}

	/* 6. 配置自动休眠时间 */
	ret = cst816t_write_reg(p_cst816t_instance,
							CST816T_AUTO_SLEEP_TIME,
							CST816T_DEFAULT_AUTO_SLEEP_TIME);
	if (0 != ret)
	{
		return -6;
	}

	return 0;
}

/******************************************************************************
 * @name    cst816t_deinit
 * @brief   CST816T去初始化: 进入深度休眠模式
 * @param   p_cst816t_instance[in]
 *
 * @return  0 success
 *         -1 cst816t_instance null
 *         -2 deep sleep failed
 *
 * @note    深度休眠后器件停止扫描, 仅外部复位(RST引脚)可唤醒, I2C无法恢复;
 *          重新使用需复位后再次调用 pf_init。
 *****************************************************************************/
static int8_t cst816t_deinit(struct bsp_cst816t_driver *p_cst816t_instance)
{
	int8_t ret = 0;

	if (NULL == p_cst816t_instance)
	{
		return -1;
	}

	/* 进入深度休眠, 关闭扫描以降低功耗 */
	ret = cst816t_write_reg(p_cst816t_instance,
							CST816T_SLEEP_MODE,
							CST816T_SLEEP_MODE_DEEP_SLEEP);
	if (0 != ret)
	{
		return -2;
	}

	return 0;
}

/******************************************************************************
 * @name    cst816t_read_id
 * @brief   读取CST816T芯片型号ID
 * @param   p_cst816t_instance[in]
 *
 * @return  读取到的ChipID原始值(0x00~0xFF, 由固件决定; 0x80~0xFF在int8_t下
 *          表现为负, 属正常)
 *         -1 cst816t_instance null
 *         -2 i2c read error
 *****************************************************************************/
static int8_t cst816t_read_id(struct bsp_cst816t_driver *p_cst816t_instance)
{
	uint8_t id = 0;

	if (NULL == p_cst816t_instance)
	{
		return -1;
	}

	if (0 != cst816t_read_reg(p_cst816t_instance, CST816T_CHIP_ID, &id))
	{
		return -2;
	}

	return id;
}

/******************************************************************************
 * @name    cst816t_read_touch
 * @brief   读取一次触摸数据帧(手势 + 手指数量 + 12位坐标)
 * @param   p_cst816t_instance[in]
 * @param   p_gesture_id[out] 手势识别结果(见 CST816T_GESTURE_* 编码)
 * @param   p_finger_num[out] 触摸手指数量
 * @param   p_x[out]          X坐标(12位, 0~4095)
 * @param   p_y[out]          Y坐标(12位, 0~4095)
 * @param   block[in]         CST816T_READ_WAIT=等待中断后读, CST816T_READ_POLL=直接读
 *
 * @return  0 success
 *         -1 cst816t_instance null
 *         -2 output buffer null
 *         -3 wait interface null (仅block=CST816T_READ_WAIT)
 *         -4 wait error / timeout (仅block=CST816T_READ_WAIT)
 *         -5 i2c read error
 *
 * @note    block=CST816T_READ_WAIT 时先等待触摸中断(OS阻塞信号量/裸机轮询
 *          irq_flag)再读; block=CST816T_READ_POLL 时直接读。数据帧连续读取
 *          0x01~0x06共6字节; X = (XposH[3:0] << 8) | XposL,
 *          Y = (YposH[3:0] << 8) | YposL。
 *****************************************************************************/
static int8_t cst816t_read_touch(struct bsp_cst816t_driver *p_cst816t_instance,
								 uint8_t *p_gesture_id, uint8_t *p_finger_num,
								 uint16_t *p_x, uint16_t *p_y,
								 uint8_t block)
{
	uint8_t buf[CST816T_TOUCH_DATA_SIZE];

	if (NULL == p_cst816t_instance)
	{
		return -1;
	}

	if (NULL == p_gesture_id || NULL == p_finger_num || NULL == p_x || NULL == p_y)
	{
		return -2;
	}

	/* 阻塞模式: 先等待触摸中断(由 cst816t_irq_cb 在ISR中置位/释放信号量) */
	if (0 != block)
	{
#ifdef OS_SUPPORTING
		if (NULL == p_cst816t_instance->p_semaphore_interface ||
			NULL == p_cst816t_instance->p_semaphore_interface->pf_wait)
		{
			return -3;
		}

		if (0 != p_cst816t_instance->p_semaphore_interface->pf_wait(
				p_cst816t_instance->p_semaphore_interface->p_semaphore_instance))
		{
			return -4;
		}
#else
		if (NULL == p_cst816t_instance->p_timebase_interface ||
			NULL == p_cst816t_instance->p_timebase_interface->pf_get_time)
		{
			return -3;
		}

		while (0 == p_cst816t_instance->irq_flag)
		{
			if ((p_cst816t_instance->p_timebase_interface->pf_get_time() -
				 p_cst816t_instance->irq_start_tick) >= CST816T_IRQ_TIMEOUT_MS)
			{
				return -4; /* 超时 */
			}
		}
#endif // OS_SUPPORTING
	}

	/* 读取触摸数据帧 */
	if (0 != cst816t_read_multi_reg(p_cst816t_instance,
									CST816T_GESTURE_ID,
									buf,
									CST816T_TOUCH_DATA_SIZE))
	{
		return -5;
	}

	*p_gesture_id = buf[0];
	*p_finger_num = buf[1] & CST816T_FINGER_NUM_MASK;
	*p_x = ((uint16_t)(buf[2] & CST816T_COORD_H_MASK) << 8) | (uint16_t)buf[3];
	*p_y = ((uint16_t)(buf[4] & CST816T_COORD_H_MASK) << 8) | (uint16_t)buf[5];

	return 0;
}

/******************************************************************************
 * @name    cst816t_enable_interrupt
 * @brief   使能触摸中断
 * @param   p_cst816t_instance[in]
 *
 * @note    使能器件内部触摸/手势中断源(IrqCtl), 并通过 interrupt_interface
 *          使能底层外部中断(EXTI/GPIO); 底层中断触发后调用本驱动的
 *          pf_interrupt_cb(cst816t_irq_cb), 释放信号量/置irq_flag。
 *
 * @return  0 success
 *         -1 cst816t_instance null
 *         -2 device interrupt enable failed
 *         -3 lower interrupt enable failed
 *****************************************************************************/
static int8_t cst816t_enable_interrupt(struct bsp_cst816t_driver *p_cst816t_instance)
{
	int8_t ret = 0;
	cst816t_interrupt_interface_t *p_interrupt;

	if (NULL == p_cst816t_instance)
	{
		return -1;
	}

	/* 0. 复位中断等待状态(供 pf_read_touch 轮询, 须在使能中断前复位避免竞态) */
	p_cst816t_instance->irq_flag = 0;
	if (NULL != p_cst816t_instance->p_timebase_interface &&
		NULL != p_cst816t_instance->p_timebase_interface->pf_get_time)
	{
		p_cst816t_instance->irq_start_tick =
			p_cst816t_instance->p_timebase_interface->pf_get_time();
	}

	/* 1. 使能器件内部触摸/手势中断源 */
	ret = cst816t_write_reg(p_cst816t_instance,
							CST816T_IRQ_CTL,
							CST816T_DEFAULT_IRQ_CTL);
	if (0 != ret)
	{
		return -2;
	}

	/* 2. 使能底层外部中断 */
	p_interrupt = p_cst816t_instance->p_interrupt_interface;
	if (NULL == p_interrupt || NULL == p_interrupt->pf_enable_interrupt)
	{
		return -3;
	}

	p_interrupt->pf_enable_interrupt(p_interrupt->p_interrupt_instance);

	return 0;
}

/******************************************************************************
 * @name    cst816t_disable_interrupt
 * @brief   关闭触摸中断
 * @param   p_cst816t_instance[in]
 *
 * @return  0 success
 *         -1 cst816t_instance null
 *         -2 device interrupt disable failed
 *         -3 lower interrupt disable failed
 *****************************************************************************/
static int8_t cst816t_disable_interrupt(struct bsp_cst816t_driver *p_cst816t_instance)
{
	int8_t ret = 0;
	cst816t_interrupt_interface_t *p_interrupt;

	if (NULL == p_cst816t_instance)
	{
		return -1;
	}

	/* 1. 关闭器件内部触摸/手势中断源 */
	ret = cst816t_write_reg(p_cst816t_instance,
							CST816T_IRQ_CTL,
							0x00);
	if (0 != ret)
	{
		return -2;
	}

	/* 2. 失能底层外部中断 */
	p_interrupt = p_cst816t_instance->p_interrupt_interface;
	if (NULL == p_interrupt || NULL == p_interrupt->pf_disable_interrupt)
	{
		return -3;
	}

	p_interrupt->pf_disable_interrupt(p_interrupt->p_interrupt_instance);

	return 0;
}

/******************************************************************************
 * @name    cst816t_hibernating
 * @brief   使CST816T进入深度休眠模式, 关闭扫描以降低功耗
 * @param   p_cst816t_instance[in]
 *
 * @return  0 success
 *         -1 cst816t_instance null
 *         -2 deep sleep failed
 *
 * @note    深度休眠后仅外部复位(RST引脚)可唤醒, I2C无法恢复; 与 pf_deinit
 *          等价, 重新使用请调用 pf_wakeup(RST复位+重初始化)。
 *****************************************************************************/
static int8_t cst816t_hibernating(struct bsp_cst816t_driver *p_cst816t_instance)
{
	int8_t ret = 0;

	if (NULL == p_cst816t_instance)
	{
		return -1;
	}

	/* 进入深度休眠 */
	ret = cst816t_write_reg(p_cst816t_instance,
							CST816T_SLEEP_MODE,
							CST816T_SLEEP_MODE_DEEP_SLEEP);
	if (0 != ret)
	{
		return -2;
	}

	return 0;
}

/******************************************************************************
 * @name    cst816t_wakeup
 * @brief   通过RST引脚复位唤醒深度休眠的CST816T, 并重新初始化
 * @param   p_cst816t_instance[in]
 *
 * @return  0 success
 *         -1 cst816t_instance null
 *         -2 gpio interface null
 *         -3 re-init failed
 *
 * @note    深度休眠后I2C失效, 无法用I2C唤醒; 本函数通过RST引脚低脉冲复位
 *          芯片, 待其重启后重新配置寄存器(pf_init), 使器件恢复到可触摸状态。
 *****************************************************************************/
static int8_t cst816t_wakeup(struct bsp_cst816t_driver *p_cst816t_instance)
{
	cst816t_gpio_interface_t *p_gpio;

	if (NULL == p_cst816t_instance)
	{
		return -1;
	}

	p_gpio = p_cst816t_instance->p_gpio_interface;
	if (NULL == p_gpio || NULL == p_gpio->pf_gpio_set_level ||
		NULL == p_gpio->p_gpio_instance)
	{
		return -2;
	}

	/* 1. RST低脉冲复位, 唤醒深度休眠的芯片 */
	p_gpio->pf_gpio_set_level(p_gpio->p_gpio_instance, 0);
	p_cst816t_instance->p_delay_interface->pf_delay(CST816T_RESET_PULSE_MS);
	p_gpio->pf_gpio_set_level(p_gpio->p_gpio_instance, 1);

	/* 2. 复位后重新初始化(等待上电完成 + 恢复寄存器配置) */
	if (0 != p_cst816t_instance->pf_init(p_cst816t_instance))
	{
		return -3;
	}

	return 0;
}
