/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_bsp_w25q64_driver.c
 *
 * @par dependencies
 * - cywatch_bsp_w25q64_driver.h
 *
 * @author zw1194
 *
 * @brief Implete the HAL operations of W25Q64 and corresponding opetions.
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
#include "cywatch_bsp_w25q64_driver.h"

/* 默认配置 */
#define W25Q64_POWER_UP_DELAY_MS  1       /* 上电稳定等待(tPUW≥5μs) */
#define W25Q64_BUSY_MAX_POLL_CNT  5000    /* 忙等待轮询次数上限(次, 每次间隔 W25Q64_BUSY_POLL_DELAY_MS, 约5s; 扇区擦除≤400ms/块擦除≤3s, 卡死快速恢复) */
#define W25Q64_CHIP_ERASE_MAX_POLL_CNT 120000 /* 整片擦除轮询次数上限(次, 约120s, 典型20s/最大100s, 留余量) */
#define W25Q64_BUSY_POLL_DELAY_MS 1       /* 忙等待轮询间隔(ms) */

static int8_t w25q64_deinst(bsp_w25q64_driver_t *p_w25q64_instance);
static int8_t w25q64_init(bsp_w25q64_driver_t *p_w25q64_instance);
static int8_t w25q64_deinit(bsp_w25q64_driver_t *p_w25q64_instance);
static int8_t w25q64_read_id(bsp_w25q64_driver_t *p_w25q64_instance,
							 uint8_t *p_manuf_id, uint8_t *p_memory_type, uint8_t *p_capacity);
static int8_t w25q64_read(bsp_w25q64_driver_t *p_w25q64_instance,
						  uint32_t addr, uint8_t *pdata, uint32_t size);
static int8_t w25q64_write(bsp_w25q64_driver_t *p_w25q64_instance,
						   uint32_t addr, uint8_t *pdata, uint32_t size);
static int8_t w25q64_erase_sector(bsp_w25q64_driver_t *p_w25q64_instance, uint32_t addr);
static int8_t w25q64_erase_block_32k(bsp_w25q64_driver_t *p_w25q64_instance, uint32_t addr);
static int8_t w25q64_erase_block_64k(bsp_w25q64_driver_t *p_w25q64_instance, uint32_t addr);
static int8_t w25q64_erase_chip(bsp_w25q64_driver_t *p_w25q64_instance);
static int8_t w25q64_hibernating(bsp_w25q64_driver_t *p_w25q64_instance);
static int8_t w25q64_wakeup(bsp_w25q64_driver_t *p_w25q64_instance);
static int8_t w25q64_wait_busy(bsp_w25q64_driver_t *p_w25q64_instance);

/* 私有辅助 */
static int8_t w25q64_wait_busy_timeout(bsp_w25q64_driver_t *p_w25q64_instance,
									   uint32_t max_poll_cnt);
static int8_t w25q64_cs_low(bsp_w25q64_driver_t *p_w25q64_instance);
static int8_t w25q64_cs_high(bsp_w25q64_driver_t *p_w25q64_instance);
static int8_t w25q64_write_enable(bsp_w25q64_driver_t *p_w25q64_instance);
static int8_t w25q64_read_status_register(bsp_w25q64_driver_t *p_w25q64_instance,
										  uint8_t *p_status);
static int8_t w25q64_page_program(bsp_w25q64_driver_t *p_w25q64_instance,
								  uint32_t addr, uint8_t *pdata, uint32_t size);
static int8_t w25q64_erase(bsp_w25q64_driver_t *p_w25q64_instance,
						   uint8_t cmd, uint32_t addr);

/******************************************************************************
 * @name    w25q64_cs_low
 * @brief   拉低CS片选, 开始一次SPI传输
 * @param   p_w25q64_instance[in]
 *
 * @return  0 success
 *         -1 w25q64_instance null
 *****************************************************************************/
static int8_t w25q64_cs_low(bsp_w25q64_driver_t *p_w25q64_instance)
{
	if (NULL == p_w25q64_instance)
	{
		return -1;
	}

	p_w25q64_instance->p_gpio_interface->pf_cs_set(0);

	return 0;
}

/******************************************************************************
 * @name    w25q64_cs_high
 * @brief   拉高CS片选, 结束一次SPI传输
 * @param   p_w25q64_instance[in]
 *
 * @return  0 success
 *         -1 w25q64_instance null
 *****************************************************************************/
static int8_t w25q64_cs_high(bsp_w25q64_driver_t *p_w25q64_instance)
{
	if (NULL == p_w25q64_instance)
	{
		return -1;
	}

	p_w25q64_instance->p_gpio_interface->pf_cs_set(1);

	return 0;
}

/******************************************************************************
 * @name    w25q64_write_enable
 * @brief   发送写使能指令, 置位WEL标志(所有擦除/编程前必须执行)
 * @param   p_w25q64_instance[in]
 *
 * @return  0 success
 *         -1 w25q64_instance null
 *         -2 spi write error
 *****************************************************************************/
static int8_t w25q64_write_enable(bsp_w25q64_driver_t *p_w25q64_instance)
{
	uint8_t cmd = W25Q64_CMD_WRITE_ENABLE;

	if (NULL == p_w25q64_instance)
	{
		return -1;
	}

	w25q64_cs_low(p_w25q64_instance);
	if (0 != p_w25q64_instance->p_spi_interface->pf_send_bytes(&cmd, 1))
	{
		w25q64_cs_high(p_w25q64_instance);
		return -2;
	}
	w25q64_cs_high(p_w25q64_instance);

	return 0;
}

/******************************************************************************
 * @name    w25q64_read_status_register
 * @brief   读取状态寄存器1
 * @param   p_w25q64_instance[in]
 * @param   p_status[out] 状态寄存器1值
 *
 * @return  0 success
 *         -1 w25q64_instance null
 *         -2 spi read error
 *****************************************************************************/
static int8_t w25q64_read_status_register(bsp_w25q64_driver_t *p_w25q64_instance,
										  uint8_t *p_status)
{
	uint8_t cmd = W25Q64_CMD_READ_STATUS_REG1;

	if (NULL == p_w25q64_instance)
	{
		return -1;
	}

	w25q64_cs_low(p_w25q64_instance);
	if (0 != p_w25q64_instance->p_spi_interface->pf_send_bytes(&cmd, 1) ||
		0 != p_w25q64_instance->p_spi_interface->pf_receive_bytes(p_status, 1))
	{
		w25q64_cs_high(p_w25q64_instance);
		return -2;
	}
	w25q64_cs_high(p_w25q64_instance);

	return 0;
}

/******************************************************************************
 * @name    w25q64_page_program
 * @brief   页编程: 单次写入≤256字节(地址不得跨页, 调用方保证)
 * @param   p_w25q64_instance[in]
 * @param   addr[in] 写入起始地址
 * @param   pdata[in] 数据缓冲区
 * @param   size[in] 写入字节数(≤256)
 *
 * @return  0 success
 *         -1 w25q64_instance null
 *         -2 invalid data / size
 *         -3 spi write error
 *****************************************************************************/
static int8_t w25q64_page_program(bsp_w25q64_driver_t *p_w25q64_instance,
								  uint32_t addr, uint8_t *pdata, uint32_t size)
{
	uint8_t frame[4];

	if (NULL == p_w25q64_instance)
	{
		return -1;
	}

	if (NULL == pdata || 0 == size || size > W25Q64_PAGE_SIZE)
	{
		return -2;
	}

	/* 组装帧头 [cmd, A23~A16, A15~A8, A7~A0] */
	frame[0] = W25Q64_CMD_PAGE_PROGRAM;
	frame[1] = (uint8_t)(addr >> 16);
	frame[2] = (uint8_t)(addr >> 8);
	frame[3] = (uint8_t)(addr);

	/* CS保持低电平: 先发指令+地址, 再发数据 */
	w25q64_cs_low(p_w25q64_instance);
	if (0 != p_w25q64_instance->p_spi_interface->pf_send_bytes(frame, 4) ||
		0 != p_w25q64_instance->p_spi_interface->pf_send_bytes(pdata, size))
	{
		w25q64_cs_high(p_w25q64_instance);
		return -3;
	}
	w25q64_cs_high(p_w25q64_instance);

	return 0;
}

/******************************************************************************
 * @name    w25q64_erase
 * @brief   擦除操作(扇区/块): 写使能→指令+地址→等待完成
 * @param   p_w25q64_instance[in]
 * @param   cmd[in] 擦除指令码
 * @param   addr[in] 目标地址
 *
 * @return  0 success
 *         -1 w25q64_instance null
 *         -2 write enable failed
 *         -3 spi write error
 *         -4 wait busy timeout
 *****************************************************************************/
static int8_t w25q64_erase(bsp_w25q64_driver_t *p_w25q64_instance,
						   uint8_t cmd, uint32_t addr)
{
	uint8_t frame[4];

	if (NULL == p_w25q64_instance)
	{
		return -1;
	}

	/* 1. 写使能 */
	if (0 != w25q64_write_enable(p_w25q64_instance))
	{
		return -2;
	}

	/* 2. 组装帧 [cmd, A23~A16, A15~A8, A7~A0] */
	frame[0] = cmd;
	frame[1] = (uint8_t)(addr >> 16);
	frame[2] = (uint8_t)(addr >> 8);
	frame[3] = (uint8_t)(addr);

	/* 3. 发送擦除指令+地址 */
	w25q64_cs_low(p_w25q64_instance);
	if (0 != p_w25q64_instance->p_spi_interface->pf_send_bytes(frame, 4))
	{
		w25q64_cs_high(p_w25q64_instance);
		return -3;
	}
	w25q64_cs_high(p_w25q64_instance);

	/* 4. 等待擦除完成 */
	if (0 != w25q64_wait_busy(p_w25q64_instance))
	{
		return -4;
	}

	return 0;
}

/******************************************************************************
 * @name    w25q64_inst
 * @brief   instancetiate the W25Q64 instance
 * @param   p_w25q64_instance[in]
 * @param   p_spi_interface[in]
 * @param   p_gpio_interface[in]
 * @param   p_delay_interface[in]
 *
 * @return  0 success
 *         -1 w25q64_instance null
 *         -2 spi_interface null (含 pf_init/pf_deinit/pf_send_bytes/pf_receive_bytes 非空校验)
 *         -3 gpio_interface null (含 pf_init/pf_deinit/pf_cs_set 非空校验)
 *         -4 delay null
 *         -7 init failed
 *         -8 w25q64 JEDEC ID error
 *
 * @note    两个带 pf_init/pf_deinit 的接口(SPI/GPIO)在这里就把四个函数指针一起校验掉:
 *          它们由 w25q64_init/w25q64_deinit 在内部回调, 若留到那时才发现是空的,
 *          已经在"发完 SPI 流量"之后了 —— 校验放在构造期, 只做一次。
 *****************************************************************************/
int8_t w25q64_inst(bsp_w25q64_driver_t *p_w25q64_instance,
				   w25q64_spi_interface_t *p_spi_interface,
				   w25q64_gpio_interface_t *p_gpio_interface,
				   w25q64_delay_interface_t *p_delay_interface)
{
	uint8_t manuf_id = 0;
	uint8_t memory_type = 0;
	uint8_t capacity = 0;

	if (NULL == p_w25q64_instance)
	{
		return -1;
	}

	/* 构造 */
	if (NULL == p_spi_interface)
	{
		return -2;
	}
	else
	{
		if (NULL == p_spi_interface->pf_init ||
			NULL == p_spi_interface->pf_deinit ||
			NULL == p_spi_interface->pf_send_bytes ||
			NULL == p_spi_interface->pf_receive_bytes)
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
		if (NULL == p_gpio_interface->pf_init ||
			NULL == p_gpio_interface->pf_deinit ||
			NULL == p_gpio_interface->pf_cs_set)
		{
			return -3;
		}
	}

	if (NULL == p_delay_interface)
	{
		return -4;
	}
	else
	{
		if (NULL == p_delay_interface->pf_delay)
		{
			return -4;
		}
	}

	p_w25q64_instance->p_spi_interface = p_spi_interface;
	p_w25q64_instance->p_gpio_interface = p_gpio_interface;
	p_w25q64_instance->p_delay_interface = p_delay_interface;

	p_w25q64_instance->pf_inst = w25q64_inst;
	p_w25q64_instance->pf_deinst = w25q64_deinst;
	p_w25q64_instance->pf_init = w25q64_init;
	p_w25q64_instance->pf_deinit = w25q64_deinit;
	p_w25q64_instance->pf_read_id = w25q64_read_id;
	p_w25q64_instance->pf_read = w25q64_read;
	p_w25q64_instance->pf_write = w25q64_write;
	p_w25q64_instance->pf_erase_sector = w25q64_erase_sector;
	p_w25q64_instance->pf_erase_block_32k = w25q64_erase_block_32k;
	p_w25q64_instance->pf_erase_block_64k = w25q64_erase_block_64k;
	p_w25q64_instance->pf_erase_chip = w25q64_erase_chip;
	p_w25q64_instance->pf_hibernating = w25q64_hibernating;
	p_w25q64_instance->pf_wakeup = w25q64_wakeup;
	p_w25q64_instance->pf_wait_busy = w25q64_wait_busy;

	/* 初始化 */
	if (0 != p_w25q64_instance->pf_init(p_w25q64_instance))
	{
		p_w25q64_instance->pf_deinst(p_w25q64_instance);
		return -7;
	}

	/* 自检: 校验JEDEC ID (厂商0xEF + 类型0x40 + 容量0x17) */
	if (0 != p_w25q64_instance->pf_read_id(p_w25q64_instance,
										  &manuf_id, &memory_type, &capacity))
	{
		p_w25q64_instance->pf_deinst(p_w25q64_instance);
		return -8;
	}

	if (W25Q64_MANUFACTURER_ID != manuf_id ||
		W25Q64_MEMORY_TYPE_ID != memory_type ||
		W25Q64_CAPACITY_ID != capacity)
	{
		p_w25q64_instance->pf_deinst(p_w25q64_instance);
		return -8;
	}

	return 0;
}

/******************************************************************************
 * @name    w25q64_deinst
 * @brief   析构W25Q64实例
 * @param   p_w25q64_instance[in]
 *
 * @return  0 success
 *         -1 w25q64_instance null
 *****************************************************************************/
static int8_t w25q64_deinst(bsp_w25q64_driver_t *p_w25q64_instance)
{
	if (NULL == p_w25q64_instance)
		return -1;

	p_w25q64_instance->pf_deinit(p_w25q64_instance);

	p_w25q64_instance->p_spi_interface = NULL;
	p_w25q64_instance->p_gpio_interface = NULL;
	p_w25q64_instance->p_delay_interface = NULL;

	p_w25q64_instance->pf_inst = NULL;
	p_w25q64_instance->pf_deinst = NULL;
	p_w25q64_instance->pf_init = NULL;
	p_w25q64_instance->pf_deinit = NULL;
	p_w25q64_instance->pf_read_id = NULL;
	p_w25q64_instance->pf_read = NULL;
	p_w25q64_instance->pf_write = NULL;
	p_w25q64_instance->pf_erase_sector = NULL;
	p_w25q64_instance->pf_erase_block_32k = NULL;
	p_w25q64_instance->pf_erase_block_64k = NULL;
	p_w25q64_instance->pf_erase_chip = NULL;
	p_w25q64_instance->pf_hibernating = NULL;
	p_w25q64_instance->pf_wakeup = NULL;
	p_w25q64_instance->pf_wait_busy = NULL;

	return 0;
}

/******************************************************************************
 * @name    w25q64_init
 * @brief   W25Q64初始化: 等待上电稳定、确认芯片空闲
 * @param   p_w25q64_instance[in]
 *
 * @return  0 success
 *         -1 w25q64_instance null
 *         -2 gpio init failed
 *         -3 spi init failed
 *         -4 read status failed
 *         -5 wait busy timeout
 *
 * @note    顺序: GPIO(CS) → SPI → 上电等待 → 读状态. 先配 CS 的理由见下面第 1 步
 *****************************************************************************/
static int8_t w25q64_init(bsp_w25q64_driver_t *p_w25q64_instance)
{
	int8_t ret = 0;
	uint8_t status = 0;

	if (NULL == p_w25q64_instance)
	{
		return -1;
	}

	/* 1. CS 先配好并拉高, 再让 SPI 的 SCK/MOSI 变成复用输出
	      @note 顺序是刻意的: CS 停在复位默认的输入浮空态时, 它靠模块自身上拉保持
	            未选中 —— 那是**外部**决定的, 引脚一有扰动就可能被读成选中。先把它
	            配成推挽输出并预置为高(adapter 的 gpio pf_init 里先写 ODR 再配 MODER),
	            片选才是由我们主动释放的。反过来的话, 引脚切到 AF 时的那几个跳变有被
	            当成命令收进去的风险
	      @note 引脚/时钟等寄存器一律不在这里碰, 全部在 adapter 的 gpio pf_init 里 */
	ret = p_w25q64_instance->p_gpio_interface->pf_init();
	if (0 != ret)
	{
		return -2;
	}

	/* 2. SPI 外设初始化(HAL_SPI_Init → MspInit 配 SCK/MISO/MOSI 与时钟) */
	ret = p_w25q64_instance->p_spi_interface->pf_init();
	if (0 != ret)
	{
		return -3;
	}

	/* 3. 等待上电稳定(tPUW) */
	p_w25q64_instance->p_delay_interface->pf_delay(W25Q64_POWER_UP_DELAY_MS);

	/* 4. 读取状态寄存器确认芯片空闲(无残留擦除/编程操作) */
	ret = w25q64_read_status_register(p_w25q64_instance, &status);
	if (0 != ret)
	{
		return -4;
	}

	/* 5. 若芯片忙, 等待其空闲 */
	if (0 != (status & W25Q64_SR1_BUSY))
	{
		if (0 != w25q64_wait_busy(p_w25q64_instance))
		{
			return -5;
		}
	}

	return 0;
}

/******************************************************************************
 * @name    w25q64_deinit
 * @brief   W25Q64去初始化: 进入深度掉电模式
 * @param   p_w25q64_instance[in]
 *
 * @return  0 success
 *         -1 w25q64_instance null
 *         -2 power down failed
 *         -3 spi deinit failed
 *         -4 gpio deinit failed
 *
 * @note    先让芯片进掉电(这一步还要用 SPI 发命令), 再按 init 的反序放掉 SPI/GPIO
 *****************************************************************************/
static int8_t w25q64_deinit(bsp_w25q64_driver_t *p_w25q64_instance)
{
	uint8_t cmd = W25Q64_CMD_POWER_DOWN;
	int8_t  ret = 0;

	if (NULL == p_w25q64_instance)
	{
		return -1;
	}

	w25q64_cs_low(p_w25q64_instance);
	if (0 != p_w25q64_instance->p_spi_interface->pf_send_bytes(&cmd, 1))
	{
		w25q64_cs_high(p_w25q64_instance);
		return -2;
	}
	w25q64_cs_high(p_w25q64_instance);

	/* 反序放掉(init 是 GPIO → SPI): 芯片已掉电, 这两步失败不影响它已进入的状态,
	   但仍如实往上返 —— 调用方(inst 失败路径里的 deinst)需要知道有没有收干净 */
	ret = p_w25q64_instance->p_spi_interface->pf_deinit();
	if (0 != ret)
	{
		return -3;
	}

	ret = p_w25q64_instance->p_gpio_interface->pf_deinit();
	if (0 != ret)
	{
		return -4;
	}

	return 0;
}

/******************************************************************************
 * @name    w25q64_read_id
 * @brief   读取JEDEC ID(厂商ID + 存储类型 + 容量)
 * @param   p_w25q64_instance[in]
 * @param   p_manuf_id[out]   厂商ID(0xEF)
 * @param   p_memory_type[out] 存储类型(0x40)
 * @param   p_capacity[out]   容量(0x17)
 *
 * @return  0 success
 *         -1 w25q64_instance null
 *         -2 output buffer null
 *         -3 spi read error
 *****************************************************************************/
static int8_t w25q64_read_id(bsp_w25q64_driver_t *p_w25q64_instance,
							 uint8_t *p_manuf_id, uint8_t *p_memory_type, uint8_t *p_capacity)
{
	uint8_t cmd = W25Q64_CMD_READ_JEDEC_ID;
	uint8_t id[3];

	if (NULL == p_w25q64_instance)
	{
		return -1;
	}

	if (NULL == p_manuf_id || NULL == p_memory_type || NULL == p_capacity)
	{
		return -2;
	}

	w25q64_cs_low(p_w25q64_instance);
	if (0 != p_w25q64_instance->p_spi_interface->pf_send_bytes(&cmd, 1) ||
		0 != p_w25q64_instance->p_spi_interface->pf_receive_bytes(id, 3))
	{
		w25q64_cs_high(p_w25q64_instance);
		return -3;
	}
	w25q64_cs_high(p_w25q64_instance);

	*p_manuf_id = id[0];
	*p_memory_type = id[1];
	*p_capacity = id[2];

	return 0;
}

/******************************************************************************
 * @name    w25q64_read
 * @brief   从指定地址连续读取数据
 * @param   p_w25q64_instance[in]
 * @param   addr[in] 读取起始地址
 * @param   pdata[out] 数据缓冲区
 * @param   size[in] 读取字节数
 *
 * @return  0 success
 *         -1 w25q64_instance null
 *         -2 invalid data / size
 *         -3 address out of range
 *         -4 spi read error
 *****************************************************************************/
static int8_t w25q64_read(bsp_w25q64_driver_t *p_w25q64_instance,
						  uint32_t addr, uint8_t *pdata, uint32_t size)
{
	uint8_t frame[4];

	if (NULL == p_w25q64_instance)
	{
		return -1;
	}

	if (NULL == pdata || 0 == size)
	{
		return -2;
	}

	if (addr >= W25Q64_TOTAL_SIZE || (addr + size) > W25Q64_TOTAL_SIZE)
	{
		return -3;
	}

	/* 组装帧 [cmd, A23~A16, A15~A8, A7~A0] */
	frame[0] = W25Q64_CMD_READ_DATA;
	frame[1] = (uint8_t)(addr >> 16);
	frame[2] = (uint8_t)(addr >> 8);
	frame[3] = (uint8_t)(addr);

	/* CS保持低电平: 发指令+地址后连续接收数据 */
	w25q64_cs_low(p_w25q64_instance);
	if (0 != p_w25q64_instance->p_spi_interface->pf_send_bytes(frame, 4) ||
		0 != p_w25q64_instance->p_spi_interface->pf_receive_bytes(pdata, size))
	{
		w25q64_cs_high(p_w25q64_instance);
		return -4;
	}
	w25q64_cs_high(p_w25q64_instance);

	return 0;
}

/******************************************************************************
 * @name    w25q64_write
 * @brief   从指定地址写入数据(自动按页拆分, 支持跨页)
 * @param   p_w25q64_instance[in]
 * @param   addr[in] 写入起始地址
 * @param   pdata[in] 数据缓冲区
 * @param   size[in] 写入字节数
 *
 * @return  0 success
 *         -1 w25q64_instance null
 *         -2 invalid data / size
 *         -3 address out of range
 *         -4 write enable failed
 *         -5 page program failed
 *         -6 wait busy timeout
 *****************************************************************************/
static int8_t w25q64_write(bsp_w25q64_driver_t *p_w25q64_instance,
						   uint32_t addr, uint8_t *pdata, uint32_t size)
{
	uint32_t offset = 0;
	uint32_t remain = size;
	uint32_t chunk = 0;

	if (NULL == p_w25q64_instance)
	{
		return -1;
	}

	if (NULL == pdata || 0 == size)
	{
		return -2;
	}

	if (addr >= W25Q64_TOTAL_SIZE || (addr + size) > W25Q64_TOTAL_SIZE)
	{
		return -3;
	}

	while (remain > 0)
	{
		/* 计算本次写入字节数(不超过当前页剩余空间) */
		chunk = W25Q64_PAGE_SIZE - (addr & (W25Q64_PAGE_SIZE - 1));
		if (chunk > remain)
		{
			chunk = remain;
		}

		/* 写使能 */
		if (0 != w25q64_write_enable(p_w25q64_instance))
		{
			return -4;
		}

		/* 页编程 */
		if (0 != w25q64_page_program(p_w25q64_instance, addr, pdata + offset, chunk))
		{
			return -5;
		}

		/* 等待编程完成 */
		if (0 != w25q64_wait_busy(p_w25q64_instance))
		{
			return -6;
		}

		addr += chunk;
		offset += chunk;
		remain -= chunk;
	}

	return 0;
}

/******************************************************************************
 * @name    w25q64_erase_sector
 * @brief   擦除4KB扇区
 * @param   p_w25q64_instance[in]
 * @param   addr[in] 目标地址(落在待擦除扇区内即可)
 *
 * @return  0 success
 *         -1 w25q64_instance null
 *         -2 write enable failed
 *         -3 spi write error
 *         -4 wait busy timeout
 *****************************************************************************/
static int8_t w25q64_erase_sector(bsp_w25q64_driver_t *p_w25q64_instance, uint32_t addr)
{
	return w25q64_erase(p_w25q64_instance, W25Q64_CMD_SECTOR_ERASE, addr);
}

/******************************************************************************
 * @name    w25q64_erase_block_32k
 * @brief   擦除32KB块
 * @param   p_w25q64_instance[in]
 * @param   addr[in] 目标地址(落在待擦除块内即可)
 *
 * @return  0 success
 *         -1 w25q64_instance null
 *         -2 write enable failed
 *         -3 spi write error
 *         -4 wait busy timeout
 *****************************************************************************/
static int8_t w25q64_erase_block_32k(bsp_w25q64_driver_t *p_w25q64_instance, uint32_t addr)
{
	return w25q64_erase(p_w25q64_instance, W25Q64_CMD_BLOCK_ERASE_32K, addr);
}

/******************************************************************************
 * @name    w25q64_erase_block_64k
 * @brief   擦除64KB块
 * @param   p_w25q64_instance[in]
 * @param   addr[in] 目标地址(落在待擦除块内即可)
 *
 * @return  0 success
 *         -1 w25q64_instance null
 *         -2 write enable failed
 *         -3 spi write error
 *         -4 wait busy timeout
 *****************************************************************************/
static int8_t w25q64_erase_block_64k(bsp_w25q64_driver_t *p_w25q64_instance, uint32_t addr)
{
	return w25q64_erase(p_w25q64_instance, W25Q64_CMD_BLOCK_ERASE_64K, addr);
}

/******************************************************************************
 * @name    w25q64_erase_chip
 * @brief   整片擦除(全片置0xFF)
 * @param   p_w25q64_instance[in]
 *
 * @return  0 success
 *         -1 w25q64_instance null
 *         -2 write enable failed
 *         -3 spi write error
 *         -4 wait busy timeout
 *
 * @note    整片擦除典型20s、最大100s, 本函数阻塞等待全程(靠 pf_delay 让出CPU);
 *          轮询次数上限用 W25Q64_CHIP_ERASE_MAX_POLL_CNT, 不是默认的 W25Q64_BUSY_MAX_POLL_CNT
 *****************************************************************************/
static int8_t w25q64_erase_chip(bsp_w25q64_driver_t *p_w25q64_instance)
{
	uint8_t cmd = W25Q64_CMD_CHIP_ERASE;

	if (NULL == p_w25q64_instance)
	{
		return -1;
	}

	/* 1. 写使能 */
	if (0 != w25q64_write_enable(p_w25q64_instance))
	{
		return -2;
	}

	/* 2. 发送整片擦除指令(无地址) */
	w25q64_cs_low(p_w25q64_instance);
	if (0 != p_w25q64_instance->p_spi_interface->pf_send_bytes(&cmd, 1))
	{
		w25q64_cs_high(p_w25q64_instance);
		return -3;
	}
	w25q64_cs_high(p_w25q64_instance);

	/* 3. 等待擦除完成: 整片擦除典型20s/最大100s, 必须用长上限 */
	if (0 != w25q64_wait_busy_timeout(p_w25q64_instance,
									  W25Q64_CHIP_ERASE_MAX_POLL_CNT))
	{
		return -4;
	}

	return 0;
}

/******************************************************************************
 * @name    w25q64_hibernating
 * @brief   使W25Q64进入深度掉电模式, 大幅降低功耗
 * @param   p_w25q64_instance[in]
 *
 * @return  0 success
 *         -1 w25q64_instance null
 *         -2 power down failed
 *         -3 spi interface deinit failed
 *         -4 gpio interface deinit failed
 *
 * @note    掉电后仅响应释放掉电指令(0xAB), 其余指令均不响应。
 * @note    外设反初始化在命令之后执行, 顺序与 w25q64_init 相反(SPI → GPIO):
 *          0xB9 还得靠 SPI 与 CS 活着才发得出去。只发命令不放总线的话, 芯片是
 *          ~1uA, 但 SPI2 外设和 PB12 还配着 —— MCU 侧那部分省不到
 * @note    外设码段追加在末尾(-3/-4), 不平移原有的 -1/-2(与 ST7789T3 同约定)
 *****************************************************************************/
static int8_t w25q64_hibernating(bsp_w25q64_driver_t *p_w25q64_instance)
{
	uint8_t cmd = W25Q64_CMD_POWER_DOWN;
	int8_t  ret = 0;

	if (NULL == p_w25q64_instance)
	{
		return -1;
	}

	w25q64_cs_low(p_w25q64_instance);
	if (0 != p_w25q64_instance->p_spi_interface->pf_send_bytes(&cmd, 1))
	{
		w25q64_cs_high(p_w25q64_instance);
		return -2;
	}
	w25q64_cs_high(p_w25q64_instance);

	/* 各外设反初始化(SPI总线 → GPIO) */
	ret = p_w25q64_instance->p_spi_interface->pf_deinit();
	if (0 != ret)
	{
		return -3;
	}

	ret = p_w25q64_instance->p_gpio_interface->pf_deinit();
	if (0 != ret)
	{
		return -4;
	}

	return 0;
}

/******************************************************************************
 * @name    w25q64_wakeup
 * @brief   释放深度掉电, 唤醒W25Q64
 * @param   p_w25q64_instance[in]
 *
 * @return  0 success
 *         -1 w25q64_instance null
 *         -2 spi write error
 *         -3 gpio interface init failed
 *         -4 spi interface init failed
 *
 * @note    掉电模式下芯片只响应0xAB, 其余指令均不响应 —— 所以这条命令发得出去,
 *          本身就证明它刚从掉电态被唤醒
 * @note    外设初始化在0xAB之前执行, 顺序与 w25q64_init 相同(GPIO(CS) → SPI):
 *          命令得靠 SPI 与 CS 活着才发得出去; CS 先配好也避免引脚切到 AF 时的
 *          跳变被当成命令(理由同 w25q64_init 的第 1 步)
 * @note    外设码段追加在末尾(-3/-4), 不平移原有的 -1/-2(与 ST7789T3 同约定)
 *****************************************************************************/
static int8_t w25q64_wakeup(bsp_w25q64_driver_t *p_w25q64_instance)
{
	uint8_t cmd = W25Q64_CMD_RELEASE_POWER_DOWN;
	int8_t  ret = 0;

	if (NULL == p_w25q64_instance)
	{
		return -1;
	}

	/* 各外设初始化: GPIO(CS) → SPI总线 */
	ret = p_w25q64_instance->p_gpio_interface->pf_init();
	if (0 != ret)
	{
		return -3;
	}

	ret = p_w25q64_instance->p_spi_interface->pf_init();
	if (0 != ret)
	{
		return -4;
	}

	w25q64_cs_low(p_w25q64_instance);
	if (0 != p_w25q64_instance->p_spi_interface->pf_send_bytes(&cmd, 1))
	{
		w25q64_cs_high(p_w25q64_instance);
		return -2;
	}
	w25q64_cs_high(p_w25q64_instance);

	return 0;
}

/******************************************************************************
 * @name    w25q64_wait_busy_timeout
 * @brief   轮询状态寄存器BUSY位等待芯片空闲, 轮询次数上限由调用方指定
 * @param   p_w25q64_instance[in]
 * @param   max_poll_cnt[in] 轮询次数上限(次)
 *
 * @return  0 success (芯片空闲)
 *         -1 w25q64_instance null
 *         -2 read status failed
 *         -3 wait timeout
 *
 * @note    计数而不是计时: 本驱动不再要求时基接口, 超时按"轮到第几次"判定, 每次
 *          间隔 W25Q64_BUSY_POLL_DELAY_MS(1ms) —— 所以 max_poll_cnt 在数值上约等于
 *          毫秒, 但**不是严格等时**: 下层的 pf_delay 若延时偏长, 实际超时会比标称长。
 *          对"卡死时快速恢复"这个用途足够
 * @note    普通擦除/编程用 W25Q64_BUSY_MAX_POLL_CNT(约5s, 卡死快速恢复);
 *          整片擦除耗时长(典型20s/最大100s), 必须用 W25Q64_CHIP_ERASE_MAX_POLL_CNT,
 *          否则 w25q64_erase_chip 必然误报超时
 *****************************************************************************/
static int8_t w25q64_wait_busy_timeout(bsp_w25q64_driver_t *p_w25q64_instance,
									   uint32_t max_poll_cnt)
{
	uint32_t poll_cnt = 0;
	uint8_t  status   = 0;

	if (NULL == p_w25q64_instance)
	{
		return -1;
	}

	while (1)
	{
		if (0 != w25q64_read_status_register(p_w25q64_instance, &status))
		{
			return -2;
		}

		if (0 == (status & W25Q64_SR1_BUSY))
		{
			return 0; /* 芯片空闲 */
		}

		poll_cnt++;
		if (poll_cnt >= max_poll_cnt)
		{
			return -3; /* 超时 */
		}

		p_w25q64_instance->p_delay_interface->pf_delay(W25Q64_BUSY_POLL_DELAY_MS);
	}
}

/******************************************************************************
 * @name    w25q64_wait_busy
 * @brief   轮询状态寄存器BUSY位, 等待芯片空闲(默认上限 W25Q64_BUSY_MAX_POLL_CNT 次)
 * @param   p_w25q64_instance[in]
 *
 * @return  0 success (芯片空闲)
 *         -1 w25q64_instance null
 *         -2 read status failed
 *         -3 wait timeout
 *
 * @note    公开接口(pf_wait_busy 结构体成员), 签名不可变; 整片擦除等长耗时
 *          场景内部改走 w25q64_wait_busy_timeout
 *****************************************************************************/
static int8_t w25q64_wait_busy(bsp_w25q64_driver_t *p_w25q64_instance)
{
	return w25q64_wait_busy_timeout(p_w25q64_instance, W25Q64_BUSY_MAX_POLL_CNT);
}
