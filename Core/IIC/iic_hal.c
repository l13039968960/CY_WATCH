/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file iic_hal.c
 *
 * @par dependencies
 * - iic_hal.h
 *
 * @author zw1194
 *
 * @brief Implete the OO-style HAL operations of software I2C.
 *
 * Processing flow:
 *
 * call iic_driver_inst() to construct, then use function pointers.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
#include "iic_hal.h"

/********************************* 前向声明 *********************************/
static int8_t iic_deinst(iic_driver_t *p_iic_instance);
static int8_t iic_init(iic_driver_t *p_iic_instance);
static int8_t iic_deinit(iic_driver_t *p_iic_instance);

/* 底层GPIO操作 (内部调用, 使用iic_driver_t *) */
static void iic_sda_input_mode(iic_driver_t *p_iic_instance);
static void iic_sda_output_mode(iic_driver_t *p_iic_instance);
static void iic_sda_output(iic_driver_t *p_iic_instance, uint16_t val);
static void iic_scl_output(iic_driver_t *p_iic_instance, uint16_t val);
static uint8_t iic_sda_input(iic_driver_t *p_iic_instance);

/* 总线信号 (对外接口, 使用void *匹配iic_interface_t) */
static int8_t iic_start(struct iic_driver *p_iic_instance);
static int8_t iic_stop(struct iic_driver *p_iic_instance);
static int8_t iic_wait_ack(struct iic_driver *p_iic_instance);
static int8_t iic_send_ack(struct iic_driver *p_iic_instance);
static int8_t iic_send_not_ack(struct iic_driver *p_iic_instance);
static int8_t iic_send_byte(struct iic_driver *p_iic_instance, uint8_t byte);
static uint8_t iic_receive_byte(struct iic_driver *p_iic_instance);

/* 原始数据收发 (不含器件地址与寄存器, 由上层组装I2C帧) */
static int8_t iic_send_bytes(struct iic_driver *p_iic_instance,
							 uint8_t *pdata,
							 uint8_t size);
static int8_t iic_receive_bytes(struct iic_driver *p_iic_instance,
								uint8_t *pdata,
								uint8_t size);

/* 寄存器级读写 (器件地址/寄存器地址的组装都在本层完成) */
static int8_t iic_readreg(struct iic_driver *p_iic_instance,
						  uint8_t dev_addr, uint8_t reg,
						  uint8_t *pdata, uint8_t size);
static int8_t iic_writereg(struct iic_driver *p_iic_instance,
						   uint8_t dev_addr, uint8_t reg,
						   uint8_t data);

/* 裸帧读写 (器件地址由本层拼) */
static int8_t iic_write_frame(struct iic_driver *p_iic_instance,
							  uint8_t dev_addr, uint8_t *pdata, uint8_t size);
static int8_t iic_read_frame(struct iic_driver *p_iic_instance,
							 uint8_t dev_addr, uint8_t *pdata, uint8_t size);

/* 互斥量 (未注入 p_mutex_interface 时为空操作) */
static int8_t iic_mutex_lock(iic_driver_t *p_iic_instance);
static void iic_mutex_unlock(iic_driver_t *p_iic_instance);

/********************************* 底层GPIO操作 *********************************/

/******************************************************************************
 * @name    iic_sda_input_mode
 * @brief   SDA线配置为输入模式(开漏, 上拉)
 * @param   p_iic_instance[in]
 *****************************************************************************/
static void iic_sda_input_mode(iic_driver_t *p_iic_instance)
{
	(void)p_iic_instance->sda.pf_set_mode(&p_iic_instance->sda,
										  GPIO_MODE_INPUT, GPIO_PULLUP);
}

/******************************************************************************
 * @name    iic_sda_output_mode
 * @brief   SDA线配置为输出模式(开漏, 无上下拉)
 * @param   p_iic_instance[in]
 *****************************************************************************/
static void iic_sda_output_mode(iic_driver_t *p_iic_instance)
{
	(void)p_iic_instance->sda.pf_set_mode(&p_iic_instance->sda,
										  GPIO_MODE_OUTPUT_OD, GPIO_NOPULL);
}

/******************************************************************************
 * @name    iic_sda_output
 * @brief   SDA线输出一位
 * @param   p_iic_instance[in]
 * @param   val[in] 0=低电平, 非0=高电平
 *****************************************************************************/
static void iic_sda_output(iic_driver_t *p_iic_instance, uint16_t val)
{
	(void)p_iic_instance->sda.pf_write(&p_iic_instance->sda,
									   (0U != val) ? 1U : 0U);
}

/******************************************************************************
 * @name    iic_scl_output
 * @brief   SCL线输出一位
 * @param   p_iic_instance[in]
 * @param   val[in] 0=低电平, 非0=高电平
 *****************************************************************************/
static void iic_scl_output(iic_driver_t *p_iic_instance, uint16_t val)
{
	(void)p_iic_instance->scl.pf_write(&p_iic_instance->scl,
									   (0U != val) ? 1U : 0U);
}

/******************************************************************************
 * @name    iic_sda_input
 * @brief   读取SDA线电平
 * @param   p_iic_instance[in]
 * @return  1=高电平, 0=低电平
 *****************************************************************************/
static uint8_t iic_sda_input(iic_driver_t *p_iic_instance)
{
	return p_iic_instance->sda.pf_read(&p_iic_instance->sda);
}

/********************************* 总线信号 *********************************/

/******************************************************************************
 * @name    iic_start
 * @brief   发送I2C起始信号
 * @param   p_ctx[in] void * → iic_driver_t *
 * @return  0 success
 *****************************************************************************/
static int8_t iic_start(struct iic_driver *p_iic_instance)
{
	iic_sda_output(p_iic_instance, 1);
	p_iic_instance->p_delay_interface->pf_delay_us(2);
	iic_scl_output(p_iic_instance, 1);
	p_iic_instance->p_delay_interface->pf_delay_us(1);
	iic_sda_output(p_iic_instance, 0);
	p_iic_instance->p_delay_interface->pf_delay_us(1);
	iic_scl_output(p_iic_instance, 0);
	p_iic_instance->p_delay_interface->pf_delay_us(1);

	return 0;
}

/******************************************************************************
 * @name    iic_stop
 * @brief   发送I2C停止信号
 * @param   p_ctx[in] iic_driver_t *
 * @return  0 success
 *****************************************************************************/
static int8_t iic_stop(struct iic_driver *p_iic_instance)
{
	iic_scl_output(p_iic_instance, 0);
	p_iic_instance->p_delay_interface->pf_delay_us(2);
	iic_sda_output(p_iic_instance, 0);
	p_iic_instance->p_delay_interface->pf_delay_us(1);
	iic_scl_output(p_iic_instance, 1);
	p_iic_instance->p_delay_interface->pf_delay_us(1);
	iic_sda_output(p_iic_instance, 1);
	p_iic_instance->p_delay_interface->pf_delay_us(1);

	return 0;
}

/******************************************************************************
 * @name    iic_wait_ack
 * @brief   等待从机应答信号
 * @param   p_ctx[in] void * → iic_driver_t *
 * @return  0 success (收到ACK)
 *         -1 timeout (未收到ACK)
 *****************************************************************************/
static int8_t iic_wait_ack(struct iic_driver *p_iic_instance)
{
	uint16_t cErrTime = 5;

	iic_sda_input_mode(p_iic_instance);
	iic_scl_output(p_iic_instance, 1);

	while (iic_sda_input(p_iic_instance))
	{
		cErrTime--;
		p_iic_instance->p_delay_interface->pf_delay_us(1);
		if (0 == cErrTime)
		{
			iic_sda_output_mode(p_iic_instance);
			iic_stop(p_iic_instance);
			return -1;
		}
	}
	iic_sda_output_mode(p_iic_instance);
	iic_scl_output(p_iic_instance, 0);
	p_iic_instance->p_delay_interface->pf_delay_us(2);

	return 0;
}

/******************************************************************************
 * @name    iic_send_ack
 * @brief   发送应答信号(ACK)
 * @param   p_ctx[in] void * → iic_driver_t *
 * @return  0 success
 *****************************************************************************/
static int8_t iic_send_ack(struct iic_driver *p_iic_instance)
{

	iic_sda_output(p_iic_instance, 0);
	p_iic_instance->p_delay_interface->pf_delay_us(1);
	iic_scl_output(p_iic_instance, 1);
	p_iic_instance->p_delay_interface->pf_delay_us(1);
	iic_scl_output(p_iic_instance, 0);
	p_iic_instance->p_delay_interface->pf_delay_us(1);

	return 0;
}

/******************************************************************************
 * @name    iic_send_not_ack
 * @brief   发送非应答信号(NACK)
 * @param   p_ctx[in] void * → iic_driver_t *
 * @return  0 success
 *****************************************************************************/
static int8_t iic_send_not_ack(struct iic_driver *p_iic_instance)
{
	iic_sda_output(p_iic_instance, 1);
	p_iic_instance->p_delay_interface->pf_delay_us(1);
	iic_scl_output(p_iic_instance, 1);
	p_iic_instance->p_delay_interface->pf_delay_us(1);
	iic_scl_output(p_iic_instance, 0);
	p_iic_instance->p_delay_interface->pf_delay_us(2);

	return 0;
}

/******************************************************************************
 * @name    iic_send_byte
 * @brief   发送一个字节 (MSB先出)
 * @param   p_ctx[in]  void * → iic_driver_t *
 * @param   byte[in]   待发送的字节
 * @return  0 success
 *****************************************************************************/
static int8_t iic_send_byte(struct iic_driver *p_iic_instance, uint8_t byte)
{
	uint8_t i = 8;

	while (i--)
	{
		iic_scl_output(p_iic_instance, 0);
		p_iic_instance->p_delay_interface->pf_delay_us(2);
		iic_sda_output(p_iic_instance, byte & 0x80);
		p_iic_instance->p_delay_interface->pf_delay_us(1);
		byte += byte;
		p_iic_instance->p_delay_interface->pf_delay_us(1);
		iic_scl_output(p_iic_instance, 1);
		p_iic_instance->p_delay_interface->pf_delay_us(1);
	}
	iic_scl_output(p_iic_instance, 0);
	p_iic_instance->p_delay_interface->pf_delay_us(2);

	return 0;
}

/******************************************************************************
 * @name    iic_receive_byte
 * @brief   接收一个字节 (MSB先收)
 * @param   p_ctx[in] void * → iic_driver_t *
 * @return  接收到的字节
 *****************************************************************************/
static uint8_t iic_receive_byte(struct iic_driver *p_iic_instance)
{
	uint8_t i = 8;
	uint8_t cR_Byte = 0;

	iic_sda_input_mode(p_iic_instance);

	while (i--)
	{
		cR_Byte += cR_Byte;
		iic_scl_output(p_iic_instance, 0);
		p_iic_instance->p_delay_interface->pf_delay_us(2);
		iic_scl_output(p_iic_instance, 1);
		p_iic_instance->p_delay_interface->pf_delay_us(1);
		cR_Byte |= iic_sda_input(p_iic_instance);
	}
	iic_scl_output(p_iic_instance, 0);
	iic_sda_output_mode(p_iic_instance);

	return cR_Byte;
}

/********************************* 原始数据收发 *********************************/

/******************************************************************************
 * @name    iic_send_bytes
 * @brief   连续发送多字节数据 (每字节后等待ACK)
 *
 *          不含START/STOP/器件地址/寄存器地址, 由上层组装I2C帧。
 *          调用方应在调用前发送START, 调用后发送STOP。
 *
 * @param   p_ctx[in]  void * → iic_driver_t *
 * @param   pdata[in]  数据缓冲区
 * @param   size[in]   发送字节数
 * @return  0 success
 *         -1 ack error
 *****************************************************************************/
static int8_t iic_send_bytes(struct iic_driver *p_iic_instance,
							 uint8_t *pdata,
							 uint8_t size)
{
	uint8_t i;

	for (i = 0; i < size; i++)
	{
		iic_send_byte(p_iic_instance, pdata[i]);
		if (0 != iic_wait_ack(p_iic_instance))
		{
			return -1;
		}
	}

	return 0;
}

/******************************************************************************
 * @name    iic_receive_bytes
 * @brief   连续接收多字节数据 (前N-1字节ACK, 末字节NACK)
 *
 *          不含START/STOP/器件地址/寄存器地址, 由上层组装I2C帧。
 *          调用方应在调用前完成寻址和方向位发送, 调用后发送STOP。
 *
 * @param   p_ctx[in]   void * → iic_driver_t *
 * @param   pdata[out]  数据缓冲区
 * @param   size[in]    接收字节数
 * @return  0 success
 *         -1 ack error
 *****************************************************************************/
static int8_t iic_receive_bytes(struct iic_driver *p_iic_instance,
								uint8_t *pdata,
								uint8_t size)
{
	uint8_t i;

	for (i = 0; i < size; i++)
	{
		pdata[i] = iic_receive_byte(p_iic_instance);
		if (i < (size - 1))
		{
			iic_send_ack(p_iic_instance);
		}
	}
	iic_send_not_ack(p_iic_instance);

	return 0;
}

/******************************************************************************
 * @name    iic_mutex_lock
 * @brief   取总线锁
 * @param   p_iic_instance[in]
 * @return  0 success / -3 取锁失败
 * @note    未注入互斥量接口时为空操作(单任务独占总线可省锁)
 *****************************************************************************/
static int8_t iic_mutex_lock(iic_driver_t *p_iic_instance)
{
	if (NULL == p_iic_instance->p_mutex_interface)
	{
		return 0;
	}

	return p_iic_instance->p_mutex_interface->pf_lock();
}

/******************************************************************************
 * @name    iic_mutex_unlock
 * @brief   放总线锁(空操作同上)
 *****************************************************************************/
static void iic_mutex_unlock(iic_driver_t *p_iic_instance)
{
	if (NULL == p_iic_instance->p_mutex_interface)
	{
		return;
	}

	(void)p_iic_instance->p_mutex_interface->pf_unlock();
}

/******************************************************************************
 * @name    iic_readreg
 * @brief   读寄存器: 写[DevAddr(W),Reg] → 重复START → 读[DevAddr(R)] → 收size字节
 * @param   p_iic_instance[in]
 * @param   dev_addr[in] 器件7位地址(内部自己拼读写位)
 * @param   reg[in]      寄存器地址
 * @param   pdata[out]   接收缓冲区
 * @param   size[in]     接收字节数
 *
 * @return  0 success
 *         -1 p_iic_instance null
 *         -2 i2c error
 *         -3 取总线锁失败
 *
 * @note    整个事务(start..stop)持锁: 位带总线不可重入, 多任务共用时必须原子
 *****************************************************************************/
static int8_t iic_readreg(struct iic_driver *p_iic_instance,
						  uint8_t dev_addr, uint8_t reg,
						  uint8_t *pdata, uint8_t size)
{
	uint8_t buf[2];

	if (NULL == p_iic_instance)
	{
		return -1;
	}

	if (0 != iic_mutex_lock(p_iic_instance))
	{
		return -3;
	}

	/* Phase 1: 写寄存器地址 [DevAddr(W), RegAddr] */
	buf[0] = (uint8_t)(dev_addr << 1);
	buf[1] = reg;

	if (0 != iic_start(p_iic_instance))
	{
		iic_mutex_unlock(p_iic_instance);
		return -2;
	}

	if (0 != iic_send_bytes(p_iic_instance, buf, 2))
	{
		iic_stop(p_iic_instance);
		iic_mutex_unlock(p_iic_instance);
		return -2;
	}

	/* Phase 2: 重复起始 + 读 [DevAddr(R)] → 接收数据 */
	buf[0] = (uint8_t)((dev_addr << 1) | 0x01);

	if (0 != iic_start(p_iic_instance))
	{
		iic_stop(p_iic_instance);
		iic_mutex_unlock(p_iic_instance);
		return -2;
	}

	if (0 != iic_send_bytes(p_iic_instance, buf, 1))
	{
		iic_stop(p_iic_instance);
		iic_mutex_unlock(p_iic_instance);
		return -2;
	}

	if (0 != iic_receive_bytes(p_iic_instance, pdata, size))
	{
		iic_stop(p_iic_instance);
		iic_mutex_unlock(p_iic_instance);
		return -2;
	}

	iic_stop(p_iic_instance);
	iic_mutex_unlock(p_iic_instance);

	return 0;
}

/******************************************************************************
 * @name    iic_writereg
 * @brief   写寄存器: [DevAddr(W),RegAddr,Data] → STOP
 * @param   p_iic_instance[in]
 * @param   dev_addr[in] 器件7位地址(内部自己拼写位)
 * @param   reg[in]      寄存器地址
 * @param   data[in]     写入数据
 *
 * @return  0 success
 *         -1 p_iic_instance null
 *         -2 i2c error
 *         -3 取总线锁失败
 *
 * @note    整个事务(start..stop)持锁, 理由同 iic_readreg
 *****************************************************************************/
static int8_t iic_writereg(struct iic_driver *p_iic_instance,
						   uint8_t dev_addr, uint8_t reg,
						   uint8_t data)
{
	uint8_t buf[3];

	if (NULL == p_iic_instance)
	{
		return -1;
	}

	if (0 != iic_mutex_lock(p_iic_instance))
	{
		return -3;
	}

	/* 组装I2C帧: [DevAddr(W), RegAddr, Data] */
	buf[0] = (uint8_t)(dev_addr << 1);
	buf[1] = reg;
	buf[2] = data;

	if (0 != iic_start(p_iic_instance))
	{
		iic_mutex_unlock(p_iic_instance);
		return -2;
	}

	if (0 != iic_send_bytes(p_iic_instance, buf, 3))
	{
		iic_stop(p_iic_instance);
		iic_mutex_unlock(p_iic_instance);
		return -2;
	}

	iic_stop(p_iic_instance);
	iic_mutex_unlock(p_iic_instance);

	return 0;
}

/******************************************************************************
 * @name    iic_write_frame
 * @brief   写裸帧: START → [DevAddr(W), data...] → STOP
 * @param   p_iic_instance[in]
 * @param   dev_addr[in] 器件7位地址(内部自己拼写位)
 * @param   pdata[in]    数据区(不含器件地址), size为0时不发送
 * @param   size[in]     数据字节数
 *
 * @return  0 success
 *         -1 p_iic_instance null
 *         -2 i2c error
 *         -3 取总线锁失败
 *
 * @note    给命令是"无寄存器地址"裸帧的器件用(如 AHT21 的 AC 33 00)。
 *          地址字节与数据分两次 send_bytes 发出, 线上波形与拼成一整段完全相同
 *          (send_bytes 只是逐字节发送+等ACK), 故不需要额外缓冲区。
 *****************************************************************************/
static int8_t iic_write_frame(struct iic_driver *p_iic_instance,
							  uint8_t dev_addr, uint8_t *pdata, uint8_t size)
{
	uint8_t addr_byte = (uint8_t)(dev_addr << 1);

	if (NULL == p_iic_instance)
	{
		return -1;
	}

	if (0 != iic_mutex_lock(p_iic_instance))
	{
		return -3;
	}

	iic_start(p_iic_instance);

	if (0 != iic_send_bytes(p_iic_instance, &addr_byte, 1) ||
		0 != iic_send_bytes(p_iic_instance, pdata, size))
	{
		iic_stop(p_iic_instance);
		iic_mutex_unlock(p_iic_instance);
		return -2;
	}

	iic_stop(p_iic_instance);
	iic_mutex_unlock(p_iic_instance);

	return 0;
}

/******************************************************************************
 * @name    iic_read_frame
 * @brief   读裸帧: START → [DevAddr(R)] → 收size字节 → STOP
 * @param   p_iic_instance[in]
 * @param   dev_addr[in] 器件7位地址(内部自己拼读位)
 * @param   pdata[out]   接收缓冲区
 * @param   size[in]     接收字节数
 *
 * @return  0 success
 *         -1 p_iic_instance null
 *         -2 i2c error
 *         -3 取总线锁失败
 *
 * @note    给"读数没有寄存器地址阶段"的器件用(如 AHT21 触发后的 7 字节测量结果)
 *****************************************************************************/
static int8_t iic_read_frame(struct iic_driver *p_iic_instance,
							 uint8_t dev_addr, uint8_t *pdata, uint8_t size)
{
	uint8_t addr_byte = (uint8_t)((dev_addr << 1) | 0x01);

	if (NULL == p_iic_instance)
	{
		return -1;
	}

	if (0 != iic_mutex_lock(p_iic_instance))
	{
		return -3;
	}

	iic_start(p_iic_instance);

	if (0 != iic_send_bytes(p_iic_instance, &addr_byte, 1) ||
		0 != iic_receive_bytes(p_iic_instance, pdata, size))
	{
		iic_stop(p_iic_instance);
		iic_mutex_unlock(p_iic_instance);
		return -2;
	}

	iic_stop(p_iic_instance);
	iic_mutex_unlock(p_iic_instance);

	return 0;
}

/********************************* 构造与析构 *********************************/

/******************************************************************************
 * @name    iic_init
 * @brief   I2C驱动初始化: 把SDA/SCL配置到运行态(GPIO + 总线空闲电平)
 * @param   p_iic_instance[in]
 *
 * @return  0 success
 *         -1 p_iic_instance null
 *
 * @note    SDA/SCL 的端口时钟由 gpio_hal 在自己的 pf_init 里开; 端口没配好
 *          (时钟没开/MODER 没写)就拉不动线, I2C 的 START 条件发不出来。
 *
 * @note    引用计数: ref_count 由 0→1 时才真正配置GPIO, 已有使用者时只累加计数
 *          (同一实例被重复init不会把总线重配一遍)。
 *****************************************************************************/
static int8_t iic_init(iic_driver_t *p_iic_instance)
{
	if (NULL == p_iic_instance)
	{
		return -1;
	}

	/* 首个使用者才真正把SDA/SCL配到运行态 */
	if (0 == p_iic_instance->ref_count)
	{
		/* 1. SDA/SCL均配为推挽输出 + 上拉(cfg 里的初始态) */
		(void)p_iic_instance->sda.pf_init(&p_iic_instance->sda);
		(void)p_iic_instance->scl.pf_init(&p_iic_instance->scl);

		/* 2. 总线初始状态: SDA/SCL均置高 */
		iic_sda_output(p_iic_instance, 1);
		iic_scl_output(p_iic_instance, 1);

		p_iic_instance->init_state = 1;
	}

	p_iic_instance->ref_count++;

	return 0;
}

/******************************************************************************
 * @name    iic_deinit
 * @brief   I2C驱动反初始化: 释放SDA/SCL引脚, 恢复复位默认态
 * @param   p_iic_instance[in]
 *
 * @return  0 success
 *         -1 p_iic_instance null
 *
 * @note    引脚收尾(DeInit → 模拟输入)在 gpio_hal 里, 端口时钟不动: 本总线所在
 *          端口可能与其他外设共用(如LCD也挂在GPIOA上), 关时钟会连带打死它们。
 *
 * @note    引用计数: 每个使用者退出时减1, 减到0才真正释放引脚。
 *****************************************************************************/
static int8_t iic_deinit(iic_driver_t *p_iic_instance)
{
	if (NULL == p_iic_instance)
	{
		return -1;
	}

	/* 退出一个使用者(已在0则不再减, 防uint8下溢后永远回不到0) */
	if (p_iic_instance->ref_count > 0)
	{
		p_iic_instance->ref_count--;
	}

	/* 最后一个使用者退出时才真正释放SDA/SCL引脚 */
	if (0 == p_iic_instance->ref_count)
	{
		(void)p_iic_instance->sda.pf_deinit(&p_iic_instance->sda);
		(void)p_iic_instance->scl.pf_deinit(&p_iic_instance->scl);

		p_iic_instance->init_state = 0;
	}

	return 0;
}

/******************************************************************************
 * @name    iic_deinst
 * @brief   I2C驱动析构: 清除所有函数指针
 * @param   p_iic_instance[in]
 *
 * @return  0 success
 *         -1 p_iic_instance null
 *         -2 仍有使用者(ref_count != 0), 本次不析构
 *
 * @note    本函数不关外设 —— 释放SDA/SCL引脚走pf_deinit。析构会清空pf_start等
 *          全部指针, 等于废掉整条总线, 所以只在没有使用者时才允许执行。
 *****************************************************************************/
static int8_t iic_deinst(iic_driver_t *p_iic_instance)
{
	if (NULL == p_iic_instance)
	{
		return -1;
	}

	/* 仍有使用者: 清指针会把复用本实例的其他设备一起废掉 */
	if (0 != p_iic_instance->ref_count)
	{
		return -2;
	}

	p_iic_instance->p_delay_interface = NULL;
	p_iic_instance->p_mutex_interface = NULL;

	p_iic_instance->init_state = 0;
	p_iic_instance->ref_count = 0;

	p_iic_instance->pf_inst = NULL;
	p_iic_instance->pf_deinst = NULL;
	p_iic_instance->pf_init = NULL;
	p_iic_instance->pf_deinit = NULL;
	p_iic_instance->pf_start = NULL;
	p_iic_instance->pf_stop = NULL;
	p_iic_instance->pf_wait_ack = NULL;
	p_iic_instance->pf_send_ack = NULL;
	p_iic_instance->pf_send_not_ack = NULL;
	p_iic_instance->pf_send_byte = NULL;
	p_iic_instance->pf_receive_byte = NULL;
	p_iic_instance->pf_send_bytes = NULL;
	p_iic_instance->pf_receive_bytes = NULL;
	p_iic_instance->pf_readreg = NULL;
	p_iic_instance->pf_writereg = NULL;
	p_iic_instance->pf_write_frame = NULL;
	p_iic_instance->pf_read_frame = NULL;

	return 0;
}

/******************************************************************************
 * @name    iic_driver_inst
 * @brief   I2C驱动构造函数: 加载总线配置、注入延时接口、挂载函数指针
 *
 *          本函数不碰硬件, GPIO的配置在pf_init()里做。
 *
 * @param   p_iic_instance[out]   I2C驱动实例
 * @param   p_bus[in]            GPIO总线配置
 * @param   p_delay_interface[in] 延时接口(软件I2C时序需微秒级延时)
 * @param   p_mutex_interface[in] 互斥量接口(可传NULL=不加锁, 用于独占总线)
 *
 * @return  0 success
 *         -1 p_iic_instance null
 *         -2 p_bus null
 *         -3 delay interface null
 *         -4 mutex interface 非空但回调缺失
 *         -5 SDA/SCL 引脚实例构造失败(端口空)
 *****************************************************************************/
int8_t iic_driver_inst(iic_driver_t *p_iic_instance,
					   iic_bus_t *p_bus,
					   iic_delay_interface_t *p_delay_interface,
					   iic_mutex_interface_t *p_mutex_interface)
{
	if (NULL == p_iic_instance)
	{
		return -1;
	}

	if (NULL == p_bus)
	{
		return -2;
	}

	if (NULL == p_delay_interface || NULL == p_delay_interface->pf_delay_us)
	{
		return -3;
	}

	if (NULL != p_mutex_interface &&
		(NULL == p_mutex_interface->pf_lock || NULL == p_mutex_interface->pf_unlock))
	{
		return -4;
	}

	/* 注入延时接口 */
	p_iic_instance->p_delay_interface = p_delay_interface;

	/* 注入互斥量接口 */
	p_iic_instance->p_mutex_interface = p_mutex_interface;

	/* 实例状态: 尚未初始化, 无使用者 */
	p_iic_instance->init_state = 0;
	p_iic_instance->ref_count = 0;

	/* 装配两条线的引脚实例(不碰硬件, 端口时钟与配置都在 pf_init 时做) */
	if ((0 != gpio_driver_inst(&p_iic_instance->sda, &p_bus->sda)) ||
		(0 != gpio_driver_inst(&p_iic_instance->scl, &p_bus->scl)))
	{
		return -5;
	}

	/* 挂载函数指针 */
	p_iic_instance->pf_inst = iic_driver_inst;
	p_iic_instance->pf_deinst = iic_deinst;
	p_iic_instance->pf_init = iic_init;
	p_iic_instance->pf_deinit = iic_deinit;
	p_iic_instance->pf_start = iic_start;
	p_iic_instance->pf_stop = iic_stop;
	p_iic_instance->pf_wait_ack = iic_wait_ack;
	p_iic_instance->pf_send_ack = iic_send_ack;
	p_iic_instance->pf_send_not_ack = iic_send_not_ack;
	p_iic_instance->pf_send_byte = iic_send_byte;
	p_iic_instance->pf_receive_byte = iic_receive_byte;
	p_iic_instance->pf_send_bytes = iic_send_bytes;
	p_iic_instance->pf_receive_bytes = iic_receive_bytes;
	p_iic_instance->pf_readreg = iic_readreg;
	p_iic_instance->pf_writereg = iic_writereg;
	p_iic_instance->pf_write_frame = iic_write_frame;
	p_iic_instance->pf_read_frame = iic_read_frame;

	return 0;
}
