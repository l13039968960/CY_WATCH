/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_bsp_max30102_driver.c
 *
 * @par dependencies
 * - cywatch_bsp_max30102_driver.h
 *
 * @author zw1194
 *
 * @brief Implete the HAL operations of MAX30102 and corresponding opetions.
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
#include "cywatch_bsp_max30102_driver.h"

/* 默认配置 */
/* 采样平均=1: FIFO 有效样本率 = SR / SMP_AVE. 默认 SR_100 配 SMP_AVE_4 时实际只有
   25 sps, PPG 峰值定位精度仅 40ms(心率误差约±3bpm), 运动伪影下易漏峰; 取 1 得到
   100 sps, 定位精度 10ms. 代价是 I2C 吞吐 ×4(600B/s, 对位带 I2C 仍余量充足) */
#define MAX30102_DEFAULT_SMP_AVE MAX30102_FIFO_SMP_AVE_1
#define MAX30102_DEFAULT_ROLLOVER MAX30102_FIFO_ROLLOVER_EN
#define MAX30102_DEFAULT_MODE MAX30102_MODE_HR
#define MAX30102_DEFAULT_ADC_RANGE MAX30102_SPO2_ADC_RANGE_4096
#define MAX30102_DEFAULT_SAMPLE_RATE MAX30102_SPO2_SR_100
#define MAX30102_DEFAULT_LED_PW MAX30102_SPO2_LED_PW_411US
#define MAX30102_DEFAULT_LED1_PA 0x24 /* RED ~7.2mA */
#define MAX30102_DEFAULT_LED2_PA 0x24 /* IR  ~7.2mA */
#define MAX30102_STARTUP_DELAY_MS 100
#define MAX30102_IRQ_TIMEOUT_MS 1000   /* 裸机中断等待超时计数(ms) */

/* FIFO */
#define MAX30102_FIFO_DEPTH 32		   /* FIFO深度(样本数) */
#define MAX30102_FIFO_HR_SAMPLE_SIZE 3 /* HR模式每样本3字节(仅IR) */


static int8_t max30102_deinst(struct bsp_max30102_driver *p_max30102_instance);
static int8_t max30102_init(struct bsp_max30102_driver *p_max30102_instance);
static int8_t max30102_deinit(struct bsp_max30102_driver *p_max30102_instance);
static int8_t max30102_read_id(struct bsp_max30102_driver *p_max30102_instance);
static int8_t max30102_get_sample_size(struct bsp_max30102_driver *p_max30102_instance);
static int8_t max30102_change_to_HR(struct bsp_max30102_driver *p_max30102_instance);
static int8_t max30102_change_to_spo2(struct bsp_max30102_driver *p_max30102_instance);
static int8_t max30102_enable_FIFO_FULL_interrupt(struct bsp_max30102_driver *p_max30102_instance);
static int8_t max30102_disable_FIFO_FULL_interrupt(struct bsp_max30102_driver *p_max30102_instance);
static int8_t max30102_read_all_FIFO_samples(struct bsp_max30102_driver *p_max30102_instance,
											 uint32_t *p_red_buff, uint32_t *p_ir_buff);
static int8_t max30102_read_one_sample(struct bsp_max30102_driver *p_max30102_instance,
									   uint32_t *p_red, uint32_t *p_ir, uint32_t *sample_size);
static int8_t max30102_hibernating(struct bsp_max30102_driver *p_max30102_instance);
static int8_t max30102_wakeup(struct bsp_max30102_driver *p_max30102_instance);
static int8_t max30102_wait_interrupt(struct bsp_max30102_driver *p_max30102_instance);

/******************************************************************************
 * @name    max30102_irq_cb
 * @brief   本层中断服务函数(挂载到 bsp_max30102_driver_t.pf_interrupt_cb)
 * @param   p_max30102_instance[in]
 *
 * @return  0 success
 *         -1 max30102_instance null
 *
 * @note    在中断上下文调用, OS释放信号量唤醒等待任务 / 裸机置irq_flag供
 *          pf_wait_interrupt 轮询。本函数不做任何I2C读操作, 避免与主循环
 *          I2C并发导致总线损坏; A_FULL标志的清理由 read_all_FIFO_samples
 *          读INTR_STATUS_1完成。
 *****************************************************************************/
static int8_t max30102_irq_cb(bsp_max30102_driver_t *p_max30102_instance)
{
	if (NULL == p_max30102_instance)
	{
		return -1;
	}

	/* OS: 释放信号量唤醒等待任务; 裸机: 置中断标志供 pf_wait_interrupt 轮询 */
#ifdef OS_SUPPORTING
	if (NULL != p_max30102_instance->p_semaphore_interface &&
		NULL != p_max30102_instance->p_semaphore_interface->pf_release)
	{
		p_max30102_instance->p_semaphore_interface->pf_release();
	}
#endif // OS_SUPPORTING
	p_max30102_instance->irq_flag = 1;

	return 0;
}

/******************************************************************************
 * @name    max30102_wait_interrupt
 * @brief   等待FIFO中断: OS阻塞等待信号量 / 裸机计数器轮询
 * @param   p_max30102_instance[in]
 *
 * @return  0 success (收到中断)
 *          1 not yet complete (裸机: 未收到且未超时, 上层据此直接返回)
 *         -1 max30102_instance null
 *         -2 interface null
 *         -3 wait error / timeout
 *****************************************************************************/
static int8_t max30102_wait_interrupt(struct bsp_max30102_driver *p_max30102_instance)
{
	if (NULL == p_max30102_instance)
	{
		return -1;
	}

#ifdef OS_SUPPORTING
	if (NULL == p_max30102_instance->p_semaphore_interface ||
		NULL == p_max30102_instance->p_semaphore_interface->pf_wait)
	{
		return -2;
	}

	if (0 != p_max30102_instance->p_semaphore_interface->pf_wait())
	{
		return -3;
	}

	return 0;
#else
	if (NULL == p_max30102_instance->p_timebase_interface ||
		NULL == p_max30102_instance->p_timebase_interface->pf_get_time)
	{
		return -2;
	}

	if (0 != p_max30102_instance->irq_flag)
	{
		return 0; /* 收到中断 */
	}

	if ((p_max30102_instance->p_timebase_interface->pf_get_time() -
		 p_max30102_instance->irq_start_tick) >= MAX30102_IRQ_TIMEOUT_MS)
	{
		return -3; /* 超时 */
	}

	return 1; /* 尚未完成 */
#endif // OS_SUPPORTING
}

/******************************************************************************
 * @name    max30102_inst
 * @brief   instancetiate the MAX30102 instance
 * @param   p_max30102_instance[in]
 * @param   p_iic_interface[in]
 * @param   p_yield_interface[in] (OS_SUPPORTING)
 * @param   p_semaphore_interface[in] (OS_SUPPORTING)
 * @param   p_delay_interface[in]
 * @param   p_timebase_interface[in]
 * @param   p_interrupt_interface[in]
 *
 * @return  0 success
 *         -1 max30102_instance null
 *         -2 iic_interface null
 *         -3 rtos_yield null
 *         -4 semaphore null
 *         -5 delay null
 *         -6 timebase null
 *         -7 interrupt null
 *         -8 init failed
 *         -9 max30102 PART_ID error
 *****************************************************************************/
int8_t max30102_inst(bsp_max30102_driver_t *p_max30102_instance,
					 max30102_iic_interface_t *p_iic_interface,
#ifdef OS_SUPPORTING
					 max30102_yield_interface_t *p_yield_interface,
					 max30102_semaphore_interface_t *p_semaphore_interface,
#endif // OS_SUPPORTING
					 max30102_delay_interface_t *p_delay_interface,
					 max30102_timebase_interface_t *p_timebase_interface,
					 max30102_interrupt_interface_t *p_interrupt_interface)
{
	if (NULL == p_max30102_instance)
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
			NULL == p_iic_interface->pf_wait_ack)
		{
			return -2;
		}
	}

#ifdef OS_SUPPORTING
	if (NULL == p_yield_interface)
	{
		return -3;
	}
	else
	{
		if (NULL == p_yield_interface->pf_yield)
		{
			return -3;
		}
	}

	if (NULL == p_semaphore_interface)
	{
		return -4;
	}
	else
	{
		if (NULL == p_semaphore_interface->pf_wait ||
			NULL == p_semaphore_interface->pf_release)
		{
			return -4;
		}
	}
#endif // OS_SUPPORTING

	if (NULL == p_delay_interface)
	{
		return -5;
	}
	else
	{
		if (NULL == p_delay_interface->pf_delay)
		{
			return -5;
		}
	}

	if (NULL == p_timebase_interface)
	{
		return -6;
	}
	else
	{
		if (NULL == p_timebase_interface->pf_get_time)
		{
			return -6;
		}
	}

	if (NULL == p_interrupt_interface)
	{
		return -7;
	}
	else
	{
		if (NULL == p_interrupt_interface->pf_enable_interrupt)
		{
			return -7;
		}
	}

	p_max30102_instance->p_iic_interface = p_iic_interface;
#ifdef OS_SUPPORTING
	p_max30102_instance->p_yield_interface = p_yield_interface;
	p_max30102_instance->p_semaphore_interface = p_semaphore_interface;
#endif // OS_SUPPORTING
	p_max30102_instance->p_delay_interface = p_delay_interface;
	p_max30102_instance->p_timebase_interface = p_timebase_interface;
	p_max30102_instance->p_interrupt_interface = p_interrupt_interface;
	p_max30102_instance->irq_flag = 0;
	p_max30102_instance->irq_start_tick = 0;

	p_max30102_instance->pf_inst = max30102_inst;
	p_max30102_instance->pf_deinst = max30102_deinst;
	p_max30102_instance->pf_init = max30102_init;
	p_max30102_instance->pf_deinit = max30102_deinit;
	p_max30102_instance->pf_read_id = max30102_read_id;
	p_max30102_instance->pf_change_to_HR = max30102_change_to_HR;
	p_max30102_instance->pf_change_to_spo2 = max30102_change_to_spo2;
	p_max30102_instance->pf_enable_FIFO_FULL_interrupt = max30102_enable_FIFO_FULL_interrupt;
	p_max30102_instance->pf_disable_FIFO_FULL_interrupt = max30102_disable_FIFO_FULL_interrupt;
	p_max30102_instance->pf_read_all_FIFO_samples = max30102_read_all_FIFO_samples;
	p_max30102_instance->pf_read_one_sample = max30102_read_one_sample;
	p_max30102_instance->pf_hibernating = max30102_hibernating;
	p_max30102_instance->pf_wakeup = max30102_wakeup;
	p_max30102_instance->pf_wait_interrupt = max30102_wait_interrupt;
	p_max30102_instance->pf_interrupt_cb = max30102_irq_cb;

	/* 初始化 */
	if (0 != p_max30102_instance->pf_init(p_max30102_instance))
	{
		p_max30102_instance->pf_deinst(p_max30102_instance);
		return -8;
	}

	if (p_max30102_instance->pf_read_id(p_max30102_instance) != MAX30102_PART_ID)
	{
		p_max30102_instance->pf_deinst(p_max30102_instance);
		return -9;
	}
	else
	{
		return 0;
	}
}

/******************************************************************************
 * @name    max30102_deinst
 * @brief   析构MAX30102实例
 * @param   p_max30102_instance[in]
 *
 * @return  0 success
 *         -1 max30102_instance null
 *****************************************************************************/
static int8_t max30102_deinst(struct bsp_max30102_driver *p_max30102_instance)
{
	if (NULL == p_max30102_instance)
		return -1;

	p_max30102_instance->pf_deinit(p_max30102_instance);

	p_max30102_instance->p_iic_interface = NULL;
#ifdef OS_SUPPORTING
	p_max30102_instance->p_yield_interface = NULL;
	p_max30102_instance->p_semaphore_interface = NULL;
#endif // OS_SUPPORTING
	p_max30102_instance->p_delay_interface = NULL;
	p_max30102_instance->p_timebase_interface = NULL;
	p_max30102_instance->p_interrupt_interface = NULL;
	p_max30102_instance->irq_flag = 0;
	p_max30102_instance->irq_start_tick = 0;
	p_max30102_instance->pf_inst = NULL;
	p_max30102_instance->pf_deinst = NULL;
	p_max30102_instance->pf_init = NULL;
	p_max30102_instance->pf_deinit = NULL;
	p_max30102_instance->pf_read_id = NULL;
	p_max30102_instance->pf_change_to_HR = NULL;
	p_max30102_instance->pf_change_to_spo2 = NULL;
	p_max30102_instance->pf_enable_FIFO_FULL_interrupt = NULL;
	p_max30102_instance->pf_disable_FIFO_FULL_interrupt = NULL;
	p_max30102_instance->pf_read_all_FIFO_samples = NULL;
	p_max30102_instance->pf_read_one_sample = NULL;
	p_max30102_instance->pf_hibernating = NULL;
	p_max30102_instance->pf_wakeup = NULL;
	p_max30102_instance->pf_wait_interrupt = NULL;
	p_max30102_instance->pf_interrupt_cb = NULL;

	return 0;
}

/******************************************************************************
 * @name    max30102_init
 * @brief   MAX30102初始化: 软复位、配置FIFO、SpO2采样参数、LED电流、
 *          复位FIFO指针、进入SpO2模式
 * @param   p_max30102_instance[in]
 *
 * @return  0 success
 *         -1 max30102_instance null
 *         -2 device reset failed
 *         -3 fifo config failed
 *         -4 spo2 config failed
 *         -5 led1 current config failed
 *         -6 led2 current config failed
 *         -7 fifo wr ptr reset failed
 *         -8 fifo rd ptr reset failed
 *         -9 mode config failed
 *****************************************************************************/
static int8_t max30102_init(struct bsp_max30102_driver *p_max30102_instance)
{
	int8_t ret = 0;

	if (NULL == p_max30102_instance)
	{
		return -1;
	}

	/* 1. 软件复位设备 */
	ret = p_max30102_instance->p_iic_interface->pf_writereg(MAX30102_I2C_ADDR,
							 MAX30102_MODE_CONFIG,
							 MAX30102_MODE_RESET);
	if (0 != ret)
	{
		return -2;
	}

	/* 等待复位完成 (RESET位复位完成后自动清零) */
	p_max30102_instance->p_delay_interface->pf_delay(MAX30102_STARTUP_DELAY_MS);

	/* 2. 配置FIFO: 采样平均 + 循环覆盖 */
	ret = p_max30102_instance->p_iic_interface->pf_writereg(MAX30102_I2C_ADDR,
							 MAX30102_FIFO_CONFIG,
							 MAX30102_DEFAULT_SMP_AVE |
								 MAX30102_DEFAULT_ROLLOVER);
	if (0 != ret)
	{
		return -3;
	}

	/* 3. 配置SpO2: ADC量程 + 采样率 + LED脉宽 */
	ret = p_max30102_instance->p_iic_interface->pf_writereg(MAX30102_I2C_ADDR,
							 MAX30102_SPO2_CONFIG,
							 MAX30102_DEFAULT_ADC_RANGE |
								 MAX30102_DEFAULT_SAMPLE_RATE |
								 MAX30102_DEFAULT_LED_PW);
	if (0 != ret)
	{
		return -4;
	}

	/* 4. 配置LED电流 */
	ret = p_max30102_instance->p_iic_interface->pf_writereg(MAX30102_I2C_ADDR,
							 MAX30102_LED1_PA,
							 MAX30102_DEFAULT_LED1_PA);
	if (0 != ret)
	{
		return -5;
	}

	ret = p_max30102_instance->p_iic_interface->pf_writereg(MAX30102_I2C_ADDR,
							 MAX30102_LED2_PA,
							 MAX30102_DEFAULT_LED2_PA);
	if (0 != ret)
	{
		return -6;
	}

	/* 5. 复位FIFO读写指针 */
	ret = p_max30102_instance->p_iic_interface->pf_writereg(MAX30102_I2C_ADDR,
							 MAX30102_FIFO_WR_PTR, 0x00);
	if (0 != ret)
	{
		return -7;
	}

	ret = p_max30102_instance->p_iic_interface->pf_writereg(MAX30102_I2C_ADDR,
							 MAX30102_FIFO_RD_PTR, 0x00);
	if (0 != ret)
	{
		return -8;
	}

	/* 6. 进入HR默认模式 */
	ret = p_max30102_instance->p_iic_interface->pf_writereg(MAX30102_I2C_ADDR,
							 MAX30102_MODE_CONFIG,
							 MAX30102_DEFAULT_MODE);
	if (0 != ret)
	{
		return -9;
	}

	return 0;
}

/******************************************************************************
 * @name    max30102_deinit
 * @brief   MAX30102去初始化: 进入关断模式
 * @param   p_max30102_instance[in]
 *
 * @return  0 success
 *         -1 max30102_instance null
 *         -2 shutdown failed
 *****************************************************************************/
static int8_t max30102_deinit(struct bsp_max30102_driver *p_max30102_instance)
{
	int8_t ret = 0;

	if (NULL == p_max30102_instance)
	{
		return -1;
	}

	/* 进入关断模式, 关闭所有模拟电路 */
	ret = p_max30102_instance->p_iic_interface->pf_writereg(MAX30102_I2C_ADDR,
							 MAX30102_MODE_CONFIG,
							 MAX30102_MODE_SHDN);
	if (0 != ret)
	{
		return -2;
	}

	return 0;
}

/******************************************************************************
 * @name    max30102_read_id
 * @brief   读取MAX30102器件ID
 * @param   p_max30102_instance[in]
 *
 * @return  读取到的ID值
 *         -1 max30102_instance null
 *         -2 i2c read error
 *****************************************************************************/
static int8_t max30102_read_id(struct bsp_max30102_driver *p_max30102_instance)
{
	uint8_t id = 0;

	if (NULL == p_max30102_instance)
	{
		return -1;
	}

	if (0 != p_max30102_instance->p_iic_interface->pf_readreg(MAX30102_I2C_ADDR, MAX30102_PART_ID_REG, &id, 1))
	{
		return -2;
	}

	return id;
}

/******************************************************************************
 * @name    max30102_get_sample_size
 * @brief   读取当前工作模式, 返回每样本的FIFO字节数
 * @param   p_max30102_instance[in]
 *
 * @return  3  HR模式(仅IR)
 *          6  SpO2模式(RED + IR)
 *         -1 max30102_instance null
 *         -2 i2c read error
 *****************************************************************************/
static int8_t max30102_get_sample_size(struct bsp_max30102_driver *p_max30102_instance)
{
	uint8_t mode = 0;

	if (NULL == p_max30102_instance)
	{
		return -1;
	}

	if (0 != p_max30102_instance->p_iic_interface->pf_readreg(MAX30102_I2C_ADDR, MAX30102_MODE_CONFIG, &mode, 1))
	{
		return -2;
	}

	if (MAX30102_MODE_HR == (mode & MAX30102_MODE_MASK))
	{
		return MAX30102_FIFO_HR_SAMPLE_SIZE;
	}
	else
	{
		return MAX30102_FIFO_SPO2_SAMPLE_SIZE;
	}
}

/******************************************************************************
 * @name    max30102_change_to_HR
 * @brief   切换MAX30102到心率模式(仅IR LED, 每样本3字节), 切换前清空FIFO
 * @param   p_max30102_instance[in]
 *
 * @return  0 success
 *         -1 max30102_instance null
 *         -2 stop sampling failed
 *         -3 fifo wr ptr reset failed
 *         -4 ovf counter reset failed
 *         -5 fifo rd ptr reset failed
 *         -6 mode config failed
 *****************************************************************************/
static int8_t max30102_change_to_HR(struct bsp_max30102_driver *p_max30102_instance)
{
	int8_t ret = 0;

	if (NULL == p_max30102_instance)
	{
		return -1;
	}

	/* 1. 停止采样, 便于清空FIFO */
	ret = p_max30102_instance->p_iic_interface->pf_writereg(MAX30102_I2C_ADDR,
							 MAX30102_MODE_CONFIG,
							 0x00);
	if (0 != ret)
	{
		return -2;
	}

	/* 2. 清空FIFO */
	ret = p_max30102_instance->p_iic_interface->pf_writereg(MAX30102_I2C_ADDR,
							 MAX30102_FIFO_WR_PTR, 0x00);
	if (0 != ret)
	{
		return -3;
	}

	ret = p_max30102_instance->p_iic_interface->pf_writereg(MAX30102_I2C_ADDR,
							 MAX30102_OVF_COUNTER, 0x00);
	if (0 != ret)
	{
		return -4;
	}

	ret = p_max30102_instance->p_iic_interface->pf_writereg(MAX30102_I2C_ADDR,
							 MAX30102_FIFO_RD_PTR, 0x00);
	if (0 != ret)
	{
		return -5;
	}

	/* 3. 切换到心率模式 */
	ret = p_max30102_instance->p_iic_interface->pf_writereg(MAX30102_I2C_ADDR,
							 MAX30102_MODE_CONFIG,
							 MAX30102_MODE_HR);
	if (0 != ret)
	{
		return -6;
	}

	return 0;
}

/******************************************************************************
 * @name    max30102_change_to_spo2
 * @brief   切换MAX30102到SpO2模式(RED + IR, 每样本6字节), 切换前清空FIFO
 * @param   p_max30102_instance[in]
 *
 * @return  0 success
 *         -1 max30102_instance null
 *         -2 stop sampling failed
 *         -3 fifo wr ptr reset failed
 *         -4 ovf counter reset failed
 *         -5 fifo rd ptr reset failed
 *         -6 mode config failed
 *****************************************************************************/
static int8_t max30102_change_to_spo2(struct bsp_max30102_driver *p_max30102_instance)
{
	int8_t ret = 0;

	if (NULL == p_max30102_instance)
	{
		return -1;
	}

	/* 1. 停止采样, 便于清空FIFO */
	ret = p_max30102_instance->p_iic_interface->pf_writereg(MAX30102_I2C_ADDR,
							 MAX30102_MODE_CONFIG,
							 0x00);
	if (0 != ret)
	{
		return -2;
	}

	/* 2. 清空FIFO */
	ret = p_max30102_instance->p_iic_interface->pf_writereg(MAX30102_I2C_ADDR,
							 MAX30102_FIFO_WR_PTR, 0x00);
	if (0 != ret)
	{
		return -3;
	}

	ret = p_max30102_instance->p_iic_interface->pf_writereg(MAX30102_I2C_ADDR,
							 MAX30102_OVF_COUNTER, 0x00);
	if (0 != ret)
	{
		return -4;
	}

	ret = p_max30102_instance->p_iic_interface->pf_writereg(MAX30102_I2C_ADDR,
							 MAX30102_FIFO_RD_PTR, 0x00);
	if (0 != ret)
	{
		return -5;
	}

	/* 3. 切换到SpO2模式 */
	ret = p_max30102_instance->p_iic_interface->pf_writereg(MAX30102_I2C_ADDR,
							 MAX30102_MODE_CONFIG,
							 MAX30102_MODE_SPO2);
	if (0 != ret)
	{
		return -6;
	}

	return 0;
}

/******************************************************************************
 * @name    max30102_enable_FIFO_FULL_interrupt
 * @brief   使能FIFO近满(A_FULL)中断
 * @param   p_max30102_instance[in]
 *
 * @note    使能器件内部A_FULL中断源(INTR_ENABLE_1 bit7), 并通过
 *          interrupt_interface 使能底层外部中断(EXTI/GPIO); 底层触发后由
 *          本驱动的 pf_interrupt_cb(max30102_irq_cb) 处理(释放信号量/置标志)。
 *
 * @return  0 success
 *         -1 max30102_instance null
 *         -2 device interrupt enable failed
 *         -3 lower interrupt enable failed
 *****************************************************************************/
static int8_t max30102_enable_FIFO_FULL_interrupt(struct bsp_max30102_driver *p_max30102_instance)
{
	int8_t ret = 0;
	max30102_interrupt_interface_t *p_interrupt;

	if (NULL == p_max30102_instance)
	{
		return -1;
	}

	/* 0. 复位中断等待状态(供 pf_wait_interrupt 轮询, 须在使能中断前复位避免竞态) */
	p_max30102_instance->irq_flag = 0;
	if (NULL != p_max30102_instance->p_timebase_interface &&
		NULL != p_max30102_instance->p_timebase_interface->pf_get_time)
	{
		p_max30102_instance->irq_start_tick =
			p_max30102_instance->p_timebase_interface->pf_get_time();
	}

	/* 1. 使能器件内部FIFO近满(A_FULL)中断 */
	ret = p_max30102_instance->p_iic_interface->pf_writereg(MAX30102_I2C_ADDR,
							 MAX30102_INTR_ENABLE_1,
							 MAX30102_INTR1_A_FULL_EN);
	if (0 != ret)
	{
		return -2;
	}

	/* 2. 使能底层外部中断 */
	p_interrupt = p_max30102_instance->p_interrupt_interface;
	if (NULL == p_interrupt || NULL == p_interrupt->pf_enable_interrupt)
	{
		return -3;
	}

	p_interrupt->pf_enable_interrupt();

	return 0;
}

/******************************************************************************
 * @name    max30102_disable_FIFO_FULL_interrupt
 * @brief   关闭FIFO近满(A_FULL)中断
 * @param   p_max30102_instance[in]
 *
 * @return  0 success
 *         -1 max30102_instance null
 *         -2 interrupt disable failed
 *****************************************************************************/
static int8_t max30102_disable_FIFO_FULL_interrupt(struct bsp_max30102_driver *p_max30102_instance)
{
	int8_t ret = 0;
	max30102_interrupt_interface_t *p_interrupt;

	if (NULL == p_max30102_instance)
	{
		return -1;
	}

	/* 关闭FIFO近满(A_FULL)中断 */
	ret = p_max30102_instance->p_iic_interface->pf_writereg(MAX30102_I2C_ADDR,
							 MAX30102_INTR_ENABLE_1,
							 0x00);
	if (0 != ret)
	{
		return -2;
	}

	/* 2.失能底层外部中断 */
	p_interrupt = p_max30102_instance->p_interrupt_interface;
	if (NULL == p_interrupt || NULL == p_interrupt->pf_disable_interrupt)
	{
		return -3;
	}

	p_interrupt->pf_disable_interrupt();

	return 0;
}

/******************************************************************************
 * @name    max30102_read_all_FIFO_samples
 * @brief   读取当前FIFO中所有样本数据
 * @param   p_max30102_instance[in]
 * @param   p_red_buff[out] RED数据缓冲区(容量>=32, HR模式下被置0)
 * @param   p_ir_buff[out]  IR数据缓冲区(容量>=32)
 *
 * @return  >0 读取到的样本数(0表示FIFO空)
 *         -1 max30102_instance null
 *         -2 output buffer null
 *         -3 interrupt status / sample size read failed
 *         -4 fifo ptr read failed
 *         -5 i2c fifo read error
 *****************************************************************************/
static int8_t max30102_read_all_FIFO_samples(struct bsp_max30102_driver *p_max30102_instance,
											 uint32_t *p_red_buff, uint32_t *p_ir_buff)
{
	int8_t sample_size = 0;
	uint8_t wr_ptr = 0;
	uint8_t rd_ptr = 0;
	uint8_t available = 0;
	uint8_t status = 0;
	uint8_t i = 0;
	uint8_t buf[MAX30102_FIFO_DEPTH * MAX30102_FIFO_SPO2_SAMPLE_SIZE];

	if (NULL == p_max30102_instance)
	{
		return -1;
	}

	if (NULL == p_red_buff || NULL == p_ir_buff)
	{
		return -2;
	}

	/* 1. 读中断状态1(读操作自动清除A_FULL标志, 释放INT引脚) */
	if (0 != p_max30102_instance->p_iic_interface->pf_readreg(MAX30102_I2C_ADDR, MAX30102_INTR_STATUS_1, &status, 1))
	{
		return -3;
	}

	/* 2. 获取当前模式对应的每样本字节数 */
	sample_size = max30102_get_sample_size(p_max30102_instance);
	if (sample_size < 0)
	{
		return -3;
	}

	/* 3. 计算当前FIFO中可读样本数 */
	if (0 != p_max30102_instance->p_iic_interface->pf_readreg(MAX30102_I2C_ADDR, MAX30102_FIFO_WR_PTR, &wr_ptr, 1) ||
		0 != p_max30102_instance->p_iic_interface->pf_readreg(MAX30102_I2C_ADDR, MAX30102_FIFO_RD_PTR, &rd_ptr, 1))
	{
		return -4;
	}

	available = (wr_ptr - rd_ptr) & (MAX30102_FIFO_DEPTH - 1);

	/* FIFO满(32样本)时WR_PTR==RD_PTR, 需结合A_FULL标志判断是满而非空 */
	if (0 == available && (MAX30102_INTR1_A_FULL == (status & MAX30102_INTR1_A_FULL)))
	{
		available = MAX30102_FIFO_DEPTH;
	}

	if (0 == available)
	{
		return 0; /* FIFO空 */
	}

	/* 3. 突发读取全部样本数据 */
	if (0 != p_max30102_instance->p_iic_interface->pf_readreg(MAX30102_I2C_ADDR,
									 MAX30102_FIFO_DATA,
									 buf,
									 available * sample_size))
	{
		return -5;
	}

	/* 4. 解析数据 (18位左对齐, 取高18位) */
	if (MAX30102_FIFO_HR_SAMPLE_SIZE == sample_size)
	{
		/* HR模式: 每样本3字节, 仅IR数据 */
		for (i = 0; i < available; i++)
		{
			p_red_buff[i] = 0;
			p_ir_buff[i] = ((uint32_t)buf[i * 3] << 16 |
							(uint32_t)buf[i * 3 + 1] << 8 |
							(uint32_t)buf[i * 3 + 2]) >>
						   6;
		}
	}
	else
	{
		/* SpO2模式: 每样本6字节, RED + IR */
		for (i = 0; i < available; i++)
		{
			p_red_buff[i] = ((uint32_t)buf[i * 6] << 16 |
							 (uint32_t)buf[i * 6 + 1] << 8 |
							 (uint32_t)buf[i * 6 + 2]) >>
							6;
			p_ir_buff[i] = ((uint32_t)buf[i * 6 + 3] << 16 |
							(uint32_t)buf[i * 6 + 4] << 8 |
							(uint32_t)buf[i * 6 + 5]) >>
						   6;
		}
	}

	return available;
}

/******************************************************************************
 * @name    max30102_read_one_sample
 * @brief   从FIFO读取一个样本(RED + IR)
 * @param   p_max30102_instance[in]
 * @param   p_red[out]       RED 18位原始数据
 * @param   p_ir[out]        IR 18位原始数据
 * @param   sample_size[out] 每样本字节数(3=HR, 6=SpO2), 可为NULL
 *
 * @note    本函数不判断FIFO是否为空, FIFO空时读到的数据为0; 调用方需自行
 *          通过读写指针判断是否有新数据(或在中断回调中读取)。
 *
 * @return  0 success
 *         -1 max30102_instance null
 *         -2 output buffer null
 *         -3 sample size read failed
 *         -4 i2c fifo read error
 *****************************************************************************/
static int8_t max30102_read_one_sample(struct bsp_max30102_driver *p_max30102_instance,
									   uint32_t *p_red, uint32_t *p_ir, uint32_t *sample_size)
{
	int8_t size = 0;
	uint8_t buf[MAX30102_FIFO_SPO2_SAMPLE_SIZE];

	if (NULL == p_max30102_instance)
	{
		return -1;
	}

	if (NULL == p_red || NULL == p_ir)
	{
		return -2;
	}

	size = max30102_get_sample_size(p_max30102_instance);
	if (size < 0)
	{
		return -3;
	}

	/* 输出每样本字节数 */
	if (NULL != sample_size)
	{
		*sample_size = (uint32_t)size;
	}

	/* 突发读取一个样本 */
	if (0 != p_max30102_instance->p_iic_interface->pf_readreg(MAX30102_I2C_ADDR,
									 MAX30102_FIFO_DATA,
									 buf,
									 size))
	{
		return -4;
	}

	/* 18位数据左对齐, 取高18位 */
	if (MAX30102_FIFO_HR_SAMPLE_SIZE == size)
	{
		/* HR模式: 仅IR数据 */
		*p_red = 0;
		*p_ir = ((uint32_t)buf[0] << 16 | (uint32_t)buf[1] << 8 | (uint32_t)buf[2]) >> 6;
	}
	else
	{
		/* SpO2模式: RED + IR */
		*p_red = ((uint32_t)buf[0] << 16 | (uint32_t)buf[1] << 8 | (uint32_t)buf[2]) >> 6;
		*p_ir = ((uint32_t)buf[3] << 16 | (uint32_t)buf[4] << 8 | (uint32_t)buf[5]) >> 6;
	}

	return 0;
}

/******************************************************************************
 * @name    max30102_hibernating
 * @brief   使MAX30102进入休眠(关断)模式, 关闭模拟电路以降低功耗
 * @param   p_max30102_instance[in]
 *
 * @return  0 success
 *         -1 max30102_instance null
 *         -2 shutdown failed
 *****************************************************************************/
static int8_t max30102_hibernating(struct bsp_max30102_driver *p_max30102_instance)
{
	int8_t ret = 0;

	if (NULL == p_max30102_instance)
	{
		return -1;
	}

	/* 进入休眠/关断模式 */
	ret = p_max30102_instance->p_iic_interface->pf_writereg(MAX30102_I2C_ADDR,
							 MAX30102_MODE_CONFIG,
							 MAX30102_MODE_SHDN);
	if (0 != ret)
	{
		return -2;
	}

	return 0;
}

/******************************************************************************
 * @name    max30102_wakeup
 * @brief   唤醒MAX30102, 退出关断模式并恢复HR工作模式
 * @param   p_max30102_instance[in]
 *
 * @return  0 success
 *         -1 max30102_instance null
 *         -2 mode config failed
 *****************************************************************************/
static int8_t max30102_wakeup(struct bsp_max30102_driver *p_max30102_instance)
{
	int8_t ret = 0;

	if (NULL == p_max30102_instance)
	{
		return -1;
	}

	/* 退出关断模式, 恢复默认模式 */
	ret = p_max30102_instance->p_iic_interface->pf_writereg(MAX30102_I2C_ADDR,
							 MAX30102_MODE_CONFIG,
							 MAX30102_DEFAULT_MODE);
	if (0 != ret)
	{
		return -2;
	}

	return 0;
}
