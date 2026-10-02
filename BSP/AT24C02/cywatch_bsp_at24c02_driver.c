/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_bsp_at24c02_driver.c
 *
 * @par dependencies
 * - cywatch_bsp_at24c02_driver.h
 *
 * @author zw1194
 *
 * @brief Implete the HAL operations of AT24C02 and corresponding opetions.
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
#include "cywatch_bsp_at24c02_driver.h"

/* 默认配置 */
#define AT24C02_POWER_UP_DELAY_MS   1   /* 上电稳定等待(≥1ms) */
#define AT24C02_WRITE_TIMEOUT_MS    10  /* 写周期ACK轮询超时(最大5ms, 留余量) */
#define AT24C02_WRITE_POLL_DELAY_MS 1   /* 写周期ACK轮询间隔(ms) */

static int8_t at24c02_deinst(bsp_at24c02_driver_t *p_at24c02_instance);
static int8_t at24c02_init(bsp_at24c02_driver_t *p_at24c02_instance);
static int8_t at24c02_deinit(bsp_at24c02_driver_t *p_at24c02_instance);
static int8_t at24c02_read(bsp_at24c02_driver_t *p_at24c02_instance,
						   uint16_t addr, uint8_t *pdata, uint16_t size);
static int8_t at24c02_write(bsp_at24c02_driver_t *p_at24c02_instance,
							uint16_t addr, uint8_t *pdata, uint16_t size);

/* 私有辅助 */
static int8_t at24c02_page_write(bsp_at24c02_driver_t *p_at24c02_instance,
								 uint8_t addr, uint8_t *pdata, uint8_t size);
static int8_t at24c02_random_read(bsp_at24c02_driver_t *p_at24c02_instance,
								  uint8_t addr, uint8_t *pdata, uint8_t size);
static int8_t at24c02_wait_busy(bsp_at24c02_driver_t *p_at24c02_instance);

/******************************************************************************
 * @name    at24c02_page_write
 * @brief   页写入: 从指定地址写入≤8字节(不可跨页, 调用方保证)
 * @param   p_at24c02_instance[in]
 * @param   addr[in] 写入起始存储地址(0x00~0xFF)
 * @param   pdata[in] 数据缓冲区
 * @param   size[in] 写入字节数(≤8)
 *
 * @return  0 success
 *         -1 at24c02_instance null
 *         -2 invalid data / size
 *         -3 i2c write error
 *
 * @note    帧数据是 [MemAddr, Data...] 连成一串发出的, 器件地址与读写位由
 *          iic_hal 在 pf_write_frame 里拼 —— 线上波形和原来自己拼 [DevAddr(W),
 *          MemAddr, Data...] 完全一致, 但整段 start..stop 由 iic_hal 持锁
 *****************************************************************************/
static int8_t at24c02_page_write(bsp_at24c02_driver_t *p_at24c02_instance,
								 uint8_t addr, uint8_t *pdata, uint8_t size)
{
	uint8_t buf[AT24C02_PAGE_SIZE + 1];
	uint8_t i;

	if (NULL == p_at24c02_instance)
	{
		return -1;
	}

	if (NULL == pdata || 0 == size || size > AT24C02_PAGE_SIZE)
	{
		return -2;
	}

	buf[0] = addr;
	for (i = 0; i < size; i++)
	{
		buf[1 + i] = pdata[i];
	}

	if (0 != p_at24c02_instance->p_iic_interface->pf_write_frame(
				 AT24C02_I2C_ADDR, buf, (uint8_t)(size + 1)))
	{
		return -3;
	}

	return 0;
}

/******************************************************************************
 * @name    at24c02_random_read
 * @brief   随机地址读取: 从指定地址连续读取多字节(地址自动递增)
 * @param   p_at24c02_instance[in]
 * @param   addr[in] 读取起始存储地址(0x00~0xFF)
 * @param   pdata[out] 数据缓冲区
 * @param   size[in] 读取字节数(≤255)
 *
 * @return  0 success
 *         -1 at24c02_instance null
 *         -2 i2c read error
 *
 * @note    pf_readreg 的波形正是 EEPROM 随机读: 伪写 [DevAddr(W), MemAddr] →
 *          重复START → [DevAddr(R)] → 收 size 字节
 *****************************************************************************/
static int8_t at24c02_random_read(bsp_at24c02_driver_t *p_at24c02_instance,
								  uint8_t addr, uint8_t *pdata, uint8_t size)
{
	if (NULL == p_at24c02_instance)
	{
		return -1;
	}

	if (0 != p_at24c02_instance->p_iic_interface->pf_readreg(
				 AT24C02_I2C_ADDR, addr, pdata, size))
	{
		return -2;
	}

	return 0;
}

/******************************************************************************
 * @name    at24c02_wait_busy
 * @brief   ACK轮询等待写周期完成(带超时)
 *
 *          写停止条件后芯片进入自定时写周期(最大5ms), 此期间不响应ACK;
 *          反复发送器件写地址检测ACK: ACK=写完成, NACK=写进行中。
 *
 * @param   p_at24c02_instance[in]
 *
 * @return  0 success (写完成/芯片空闲)
 *         -1 at24c02_instance null
 *         -2 wait timeout (器件无应答)
 *
 * @note    NACK 时不需要调用方收尾: pf_write_frame 内部无论成败都会发 STOP
 *          (自己拼帧时漏发会把这根线的 SCL 停在低电平, 后续访问全吊死)
 *****************************************************************************/
static int8_t at24c02_wait_busy(bsp_at24c02_driver_t *p_at24c02_instance)
{
	uint32_t start = 0;
	uint8_t dummy = 0;
	int8_t ret = 0;

	if (NULL == p_at24c02_instance)
	{
		return -1;
	}

	start = p_at24c02_instance->p_timebase_interface->pf_get_time();

	while (1)
	{
		/* size=0: 只发器件写地址、不跟数据, 用 ACK 判断写周期是否结束 */
		ret = p_at24c02_instance->p_iic_interface->pf_write_frame(
				  AT24C02_I2C_ADDR, &dummy, 0);

		if (0 == ret)
		{
			return 0;
		}

		if ((p_at24c02_instance->p_timebase_interface->pf_get_time() - start) >=
			AT24C02_WRITE_TIMEOUT_MS)
		{
			return -2;
		}

		p_at24c02_instance->p_delay_interface->pf_delay(AT24C02_WRITE_POLL_DELAY_MS);
	}
}

/******************************************************************************
 * @name    at24c02_inst
 * @brief   instancetiate the AT24C02 instance
 * @param   p_at24c02_instance[in]
 * @param   p_iic_interface[in]
 * @param   p_delay_interface[in]
 * @param   p_timebase_interface[in]
 *
 * @return  0 success
 *         -1 at24c02_instance null
 *         -2 iic_interface null
 *         -3 delay null
 *         -4 timebase null
 *         -5 init failed (器件无应答)
 *****************************************************************************/
int8_t at24c02_inst(bsp_at24c02_driver_t *p_at24c02_instance,
					at24c02_iic_interface_t *p_iic_interface,
					at24c02_delay_interface_t *p_delay_interface,
					at24c02_timebase_interface_t *p_timebase_interface)
{
	if (NULL == p_at24c02_instance)
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
		if (NULL == p_iic_interface->pf_readreg ||
			NULL == p_iic_interface->pf_write_frame ||
			NULL == p_iic_interface->pf_read_frame)
		{
			return -2;
		}
	}

	if (NULL == p_delay_interface)
	{
		return -3;
	}
	else
	{
		if (NULL == p_delay_interface->pf_delay)
		{
			return -3;
		}
	}

	if (NULL == p_timebase_interface)
	{
		return -4;
	}
	else
	{
		if (NULL == p_timebase_interface->pf_get_time)
		{
			return -4;
		}
	}

	p_at24c02_instance->p_iic_interface = p_iic_interface;
	p_at24c02_instance->p_delay_interface = p_delay_interface;
	p_at24c02_instance->p_timebase_interface = p_timebase_interface;

	p_at24c02_instance->pf_inst = at24c02_inst;
	p_at24c02_instance->pf_deinst = at24c02_deinst;
	p_at24c02_instance->pf_init = at24c02_init;
	p_at24c02_instance->pf_deinit = at24c02_deinit;
	p_at24c02_instance->pf_read = at24c02_read;
	p_at24c02_instance->pf_write = at24c02_write;

	/* 初始化(含ACK探测自检) */
	if (0 != p_at24c02_instance->pf_init(p_at24c02_instance))
	{
		p_at24c02_instance->pf_deinst(p_at24c02_instance);
		return -5;
	}

	return 0;
}

/******************************************************************************
 * @name    at24c02_deinst
 * @brief   析构AT24C02实例
 * @param   p_at24c02_instance[in]
 *
 * @return  0 success
 *         -1 at24c02_instance null
 *****************************************************************************/
static int8_t at24c02_deinst(bsp_at24c02_driver_t *p_at24c02_instance)
{
	if (NULL == p_at24c02_instance)
		return -1;

	p_at24c02_instance->pf_deinit(p_at24c02_instance);

	p_at24c02_instance->p_iic_interface = NULL;
	p_at24c02_instance->p_delay_interface = NULL;
	p_at24c02_instance->p_timebase_interface = NULL;

	p_at24c02_instance->pf_inst = NULL;
	p_at24c02_instance->pf_deinst = NULL;
	p_at24c02_instance->pf_init = NULL;
	p_at24c02_instance->pf_deinit = NULL;
	p_at24c02_instance->pf_read = NULL;
	p_at24c02_instance->pf_write = NULL;

	return 0;
}

/******************************************************************************
 * @name    at24c02_init
 * @brief   AT24C02初始化: 等待上电稳定、ACK探测确认器件存在且空闲
 * @param   p_at24c02_instance[in]
 *
 * @return  0 success
 *         -1 at24c02_instance null
 *         -2 device not ready (ACK探测超时)
 *****************************************************************************/
static int8_t at24c02_init(bsp_at24c02_driver_t *p_at24c02_instance)
{
	if (NULL == p_at24c02_instance)
	{
		return -1;
	}

	/* 1. 等待上电稳定(≥1ms) */
	p_at24c02_instance->p_delay_interface->pf_delay(AT24C02_POWER_UP_DELAY_MS);

	/* 2. ACK探测: 芯片空闲时立即应答, 确认器件存在且无残留写周期 */
	if (0 != at24c02_wait_busy(p_at24c02_instance))
	{
		return -2;
	}

	return 0;
}

/******************************************************************************
 * @name    at24c02_deinit
 * @brief   AT24C02去初始化
 * @param   p_at24c02_instance[in]
 *
 * @return  0 success
 *         -1 at24c02_instance null
 *
 * @note    AT24C02为被动EEPROM, 无掉电模式指令, 空闲即待机(0.1μA),
 *          无需额外操作; 此处仅做占位, 保持生命周期接口完整。
 *****************************************************************************/
static int8_t at24c02_deinit(bsp_at24c02_driver_t *p_at24c02_instance)
{
	if (NULL == p_at24c02_instance)
	{
		return -1;
	}

	return 0;
}

/******************************************************************************
 * @name    at24c02_read
 * @brief   从指定地址连续读取数据(地址自动递增, 到达0xFF回卷)
 * @param   p_at24c02_instance[in]
 * @param   addr[in] 读取起始地址
 * @param   pdata[out] 数据缓冲区
 * @param   size[in] 读取字节数
 *
 * @return  0 success
 *         -1 at24c02_instance null
 *         -2 invalid data / size
 *         -3 address out of range
 *         -4 i2c read error
 *****************************************************************************/
static int8_t at24c02_read(bsp_at24c02_driver_t *p_at24c02_instance,
						   uint16_t addr, uint8_t *pdata, uint16_t size)
{
	uint16_t offset = 0;
	uint8_t chunk = 0;

	if (NULL == p_at24c02_instance)
	{
		return -1;
	}

	if (NULL == pdata || 0 == size)
	{
		return -2;
	}

	if (addr >= AT24C02_TOTAL_SIZE || (addr + size) > AT24C02_TOTAL_SIZE)
	{
		return -3;
	}

	/* 底层I2C单次接收上限为uint8_t(255), 超过时分块读取 */
	while (size > 0)
	{
		chunk = (size > 255) ? 255 : (uint8_t)size;

		if (0 != at24c02_random_read(p_at24c02_instance,
									 (uint8_t)addr, pdata + offset, chunk))
		{
			return -4;
		}

		addr += chunk;
		offset += chunk;
		size -= chunk;
	}

	return 0;
}

/******************************************************************************
 * @name    at24c02_write
 * @brief   从指定地址写入数据(自动按8字节页拆分, 支持跨页)
 * @param   p_at24c02_instance[in]
 * @param   addr[in] 写入起始地址
 * @param   pdata[in] 数据缓冲区
 * @param   size[in] 写入字节数
 *
 * @return  0 success
 *         -1 at24c02_instance null
 *         -2 invalid data / size
 *         -3 address out of range
 *         -4 page write failed
 *         -5 wait busy timeout
 *****************************************************************************/
static int8_t at24c02_write(bsp_at24c02_driver_t *p_at24c02_instance,
							uint16_t addr, uint8_t *pdata, uint16_t size)
{
	uint16_t offset = 0;
	uint16_t remain = size;
	uint16_t chunk = 0;

	if (NULL == p_at24c02_instance)
	{
		return -1;
	}

	if (NULL == pdata || 0 == size)
	{
		return -2;
	}

	if (addr >= AT24C02_TOTAL_SIZE || (addr + size) > AT24C02_TOTAL_SIZE)
	{
		return -3;
	}

	while (remain > 0)
	{
		/* 计算本次写入字节数(不超过当前页剩余空间) */
		chunk = AT24C02_PAGE_SIZE - (addr & (AT24C02_PAGE_SIZE - 1));
		if (chunk > remain)
		{
			chunk = remain;
		}

		/* 页写入 */
		if (0 != at24c02_page_write(p_at24c02_instance,
									(uint8_t)addr, pdata + offset, (uint8_t)chunk))
		{
			return -4;
		}

		/* 等待写周期完成 */
		if (0 != at24c02_wait_busy(p_at24c02_instance))
		{
			return -5;
		}

		addr += chunk;
		offset += chunk;
		remain -= chunk;
	}

	return 0;
}
