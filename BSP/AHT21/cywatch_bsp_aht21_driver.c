/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_bsp_aht21_driver.c
 *
 * @par dependencies
 * - cywatch_bsp_aht21_driver.h
 *
 * @author zw1194
 *
 * @brief Implete the HAL operations of AHT21 and corresponding opetions.
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
#include "cywatch_bsp_aht21_driver.h"

/* 一次测量读回的 7 字节: [0]=状态, [1..3]=湿度20位, [4..6]=温度20位 */
#define AHT21_MEASURE_DATA_LEN  7

static int8_t aht21_deinst(struct bsp_aht21_driver *p_aht21_instance);
static int8_t aht21_init(struct bsp_aht21_driver *p_aht21_instance);
static int8_t aht21_deinit(struct bsp_aht21_driver *p_aht21_instance);
static int8_t aht21_read_id(struct bsp_aht21_driver *p_aht21_instance);
static int8_t aht21_read_temp_humi(struct bsp_aht21_driver *p_aht21_instance,
								   float *p_temperature, float *p_humidity);
static int8_t aht21_hibernating(struct bsp_aht21_driver *p_aht21_instance);
static int8_t aht21_wakeup(struct bsp_aht21_driver *p_aht21_instance);

/******************************************************************************
 * @name    aht21_inst
 * @brief   instancetiate the AHT21 instance
 * @param   p_aht21_instance[in]
 * @param   p_iic_interface[in]
 * @param   p_delay_interface[in]
 *
 * @return  0 success
 *         -1 aht21_instance null
 *         -2 iic_interface null
 *         -3 delay null
 *         -4 init failed
 *         -5 aht21 ADDR/ID error
 *****************************************************************************/
int8_t aht21_inst(bsp_aht21_driver_t *p_aht21_instance,
				  aht21_iic_interface_t *p_iic_interface,
				  aht21_delay_interface_t *p_delay_interface)
{
	int8_t ret = 0;

	if (NULL == p_aht21_instance)
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

	p_aht21_instance->p_iic_interface = p_iic_interface;
	p_aht21_instance->p_delay_interface = p_delay_interface;

	p_aht21_instance->pf_inst = aht21_inst;
	p_aht21_instance->pf_deinst = aht21_deinst;
	p_aht21_instance->pf_init = aht21_init;
	p_aht21_instance->pf_deinit = aht21_deinit;
	p_aht21_instance->pf_read_id = aht21_read_id;
	p_aht21_instance->pf_read_temp_humi = aht21_read_temp_humi;
	p_aht21_instance->pf_hibernating = aht21_hibernating;
	p_aht21_instance->pf_wakeup = aht21_wakeup;

	/* 初始化 */
	if (0 != p_aht21_instance->pf_init(p_aht21_instance))
	{
		p_aht21_instance->pf_deinst(p_aht21_instance);
		return -4;
	}

	/* 自检: AHT21 无 ID 寄存器, 用状态寄存器 0x71 是否应答 + 校准位
	   是否已置位作为"器件在位且可用"的判据 */
	ret = p_aht21_instance->pf_read_id(p_aht21_instance);

	if (ret < 0 || 0 == (ret & AHT21_STATUS_CALI))
	{
		p_aht21_instance->pf_deinst(p_aht21_instance);
		return -5;
	}

	return 0;
}

/******************************************************************************
 * @name    aht21_deinst
 * @brief   析构AHT21实例
 * @param   p_aht21_instance[in]
 *
 * @return  0 success
 *         -1 aht21_instance null
 *****************************************************************************/
static int8_t aht21_deinst(struct bsp_aht21_driver *p_aht21_instance)
{
	if (NULL == p_aht21_instance)
		return -1;

	p_aht21_instance->pf_deinit(p_aht21_instance);

	p_aht21_instance->p_iic_interface = NULL;
	p_aht21_instance->p_delay_interface = NULL;
	p_aht21_instance->pf_inst = NULL;
	p_aht21_instance->pf_deinst = NULL;
	p_aht21_instance->pf_init = NULL;
	p_aht21_instance->pf_deinit = NULL;
	p_aht21_instance->pf_read_id = NULL;
	p_aht21_instance->pf_read_temp_humi = NULL;
	p_aht21_instance->pf_hibernating = NULL;
	p_aht21_instance->pf_wakeup = NULL;

	return 0;
}

/******************************************************************************
 * @name    aht21_init
 * @brief   AHT21初始化: 上电等待、未校准则发 E1 08 00 触发自校准
 * @param   p_aht21_instance[in]
 *
 * @return  0 success
 *         -1 aht21_instance null
 *         -2 status read failed
 *         -3 calibrate cmd failed
 *****************************************************************************/
static int8_t aht21_init(struct bsp_aht21_driver *p_aht21_instance)
{
	uint8_t status = 0;
	uint8_t init_cmd[3] = { AHT21_CMD_INIT,
							AHT21_CMD_INIT_ARG1,
							AHT21_CMD_INIT_ARG2 };

	if (NULL == p_aht21_instance)
	{
		return -1;
	}

	/* 1. 上电稳定: 期间器件不应答 */
	p_aht21_instance->p_delay_interface->pf_delay(AHT21_POWER_ON_DELAY_MS);

	/* 2. 读状态寄存器, 看校准位 */
	if (0 != p_aht21_instance->p_iic_interface->pf_readreg(AHT21_IIC_ADDR,
							AHT21_STATUS_REG, &status, 1))
	{
		return -2;
	}

	/* 3. 未校准(如首次上电)才发初始化命令, 已校准的不重复校准 */
	if (0 == (status & AHT21_STATUS_CALI))
	{
		if (0 != p_aht21_instance->p_iic_interface->pf_write_frame(AHT21_IIC_ADDR,
																   init_cmd, 3))
		{
			return -3;
		}

		p_aht21_instance->p_delay_interface->pf_delay(AHT21_CALIBRATE_DELAY_MS);
	}

	return 0;
}

/******************************************************************************
 * @name    aht21_deinit
 * @brief   AHT21去初始化: 发软复位进入待机
 * @param   p_aht21_instance[in]
 *
 * @return  0 success
 *         -1 aht21_instance null
 *         -2 soft reset failed
 *****************************************************************************/
static int8_t aht21_deinit(struct bsp_aht21_driver *p_aht21_instance)
{
	uint8_t reset_cmd[1] = { AHT21_CMD_SOFT_RESET };

	if (NULL == p_aht21_instance)
	{
		return -1;
	}

	/* AHT21 没有掉电命令, 软复位是能做到的最低功耗状态 */
	if (0 != p_aht21_instance->p_iic_interface->pf_write_frame(AHT21_IIC_ADDR,
															   reset_cmd, 1))
	{
		return -2;
	}

	p_aht21_instance->p_delay_interface->pf_delay(AHT21_RESET_DELAY_MS);

	return 0;
}

/******************************************************************************
 * @name    aht21_read_id
 * @brief   读状态寄存器 0x71 作为器件应答凭据
 * @param   p_aht21_instance[in]
 *
 * @return  状态寄存器值(已屏蔽bit7忙标志)
 *         -1 aht21_instance null
 *         -2 i2c read error
 *
 * @note    位7("正在测量")会把这个 int8_t 返回值顶成负数, 与 -1/-2 混淆,
 *          故屏蔽掉再返回
 *****************************************************************************/
static int8_t aht21_read_id(struct bsp_aht21_driver *p_aht21_instance)
{
	uint8_t status = 0;

	if (NULL == p_aht21_instance)
	{
		return -1;
	}

	if (0 != p_aht21_instance->p_iic_interface->pf_readreg(AHT21_IIC_ADDR,
							AHT21_STATUS_REG, &status, 1))
	{
		return -2;
	}

	return (int8_t)(status & 0x7F);
}

/******************************************************************************
 * @name    aht21_read_temp_humi
 * @brief   触发一次测量, 同时读回温度(℃)与湿度(%RH)
 * @param   p_aht21_instance[in]
 * @param   p_temperature[out] 温度, 单位 ℃
 * @param   p_humidity[out]    相对湿度, 单位 %RH
 *
 * @return  0 success
 *         -1 aht21_instance null
 *         -2 trigger cmd failed
 *         -3 data frame read failed
 *         -4 still busy
 *****************************************************************************/
static int8_t aht21_read_temp_humi(struct bsp_aht21_driver *p_aht21_instance,
								   float *p_temperature, float *p_humidity)
{
	uint8_t buf[AHT21_MEASURE_DATA_LEN];
	uint8_t trigger_cmd[3] = { AHT21_CMD_TRIGGER,
							   AHT21_CMD_TRIGGER_ARG1,
							   AHT21_CMD_TRIGGER_ARG2 };
	uint32_t raw_humi = 0;
	uint32_t raw_temp = 0;

	if (NULL == p_aht21_instance)
	{
		return -1;
	}

	/* 1. 触发测量 AC 33 00 */
	if (0 != p_aht21_instance->p_iic_interface->pf_write_frame(AHT21_IIC_ADDR,
															   trigger_cmd, 3))
	{
		return -2;
	}

	/* 2. 等测量完成(典型 80ms) */
	p_aht21_instance->p_delay_interface->pf_delay(AHT21_MEASURE_DELAY_MS);

	/* 3. 读 7 字节(测量结果不是寄存器, 没有寄存器地址阶段) */
	if (0 != p_aht21_instance->p_iic_interface->pf_read_frame(AHT21_IIC_ADDR,
															  buf,
															  AHT21_MEASURE_DATA_LEN))
	{
		return -3;
	}

	/* 4. 忙标志还在 → 这帧不是有效测量结果 */
	if (0 != (buf[0] & AHT21_STATUS_BUSY))
	{
		return -4;
	}

	/* 5. 解析: 温湿度各 20 位, 且**共用第 3 字节** —— 湿度占 buf[3] 高 4 位,
	      温度占 buf[3] 低 4 位. buf[6] 不参与任何计算(读它是为了把帧取完).
	      两者都是 2^20 满量程
	      @note 温度若从 buf[4] 起取就整体错位一字节: buf[4]低4位会当成高位,
	            权重 12.5℃/LSB, 读数在 ±80℃ 内乱跳而湿度却正常 —— 别这么写 */
	raw_humi = ((uint32_t)buf[1] << 16 |
				(uint32_t)buf[2] << 8 |
				(uint32_t)buf[3]) >> 4;
	raw_temp = ((uint32_t)buf[3] & 0x0F) << 16 |
			   (uint32_t)buf[4] << 8 |
			   (uint32_t)buf[5];

	*p_humidity = (float)raw_humi * 100.0f / AHT21_RAW_FULL_SCALE;
	*p_temperature = (float)raw_temp * 200.0f / AHT21_RAW_FULL_SCALE - 50.0f;

	return 0;
}

/******************************************************************************
 * @name    aht21_hibernating
 * @brief   使AHT21进入待机: 发软复位 BA
 * @param   p_aht21_instance[in]
 *
 * @return  0 success
 *         -1 aht21_instance null
 *         -2 soft reset failed
 *
 * @note    AHT21 无休眠命令, 软复位后器件停止测量等唤醒
 *****************************************************************************/
static int8_t aht21_hibernating(struct bsp_aht21_driver *p_aht21_instance)
{
	return aht21_deinit(p_aht21_instance);
}

/******************************************************************************
 * @name    aht21_wakeup
 * @brief   唤醒AHT21: 重新发初始化命令 E1 08 00 恢复校准
 * @param   p_aht21_instance[in]
 *
 * @return  0 success
 *         -1 aht21_instance null
 *         -2 init cmd failed
 *
 * @note    软复位会清掉校准状态, 唤醒必须重新初始化, 否则读回的数据不可信
 *****************************************************************************/
static int8_t aht21_wakeup(struct bsp_aht21_driver *p_aht21_instance)
{
	uint8_t init_cmd[3] = { AHT21_CMD_INIT,
							AHT21_CMD_INIT_ARG1,
							AHT21_CMD_INIT_ARG2 };

	if (NULL == p_aht21_instance)
	{
		return -1;
	}

	if (0 != p_aht21_instance->p_iic_interface->pf_write_frame(AHT21_IIC_ADDR,
															   init_cmd, 3))
	{
		return -2;
	}

	p_aht21_instance->p_delay_interface->pf_delay(AHT21_CALIBRATE_DELAY_MS);

	return 0;
}
