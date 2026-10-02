/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_bsp_mpu6050_driver.c
 *
 * @par dependencies
 * - cywatch_bsp_mpu6050_driver.h
 *
 * @author zw1194
 *
 * @brief Implete the HAL operations of MPU6050 and corresponding opetions.
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
#include "cywatch_bsp_mpu6050_driver.h"

/* 默认配置 */
#define MPU6050_DEFAULT_GYRO_FS MPU6050_GCONFIG_FS_SEL_250
#define MPU6050_DEFAULT_ACCEL_FS MPU6050_ACONFIG_AFS_SEL_2G
#define MPU6050_DEFAULT_DLPF MPU6050_CONFIG_DLPF_44HZ
#define MPU6050_DEFAULT_SMPLRT_DIV 0x00
#define MPU6050_STARTUP_DELAY_MS 100
#define MPU6050_I2C_ADDR           0x68 /* AD0=0 时默认7位地址 */

static int8_t mpu6050_deinst(struct bsp_mpu6050_driver *p_mpu6050_instance);
static int8_t mpu6050_init(struct bsp_mpu6050_driver *p_mpu6050_instance);
static int8_t mpu6050_deinit(struct bsp_mpu6050_driver *p_mpu6050_instance);
static int8_t mpu6050_read_id(struct bsp_mpu6050_driver *p_mpu6050_instance);
static int8_t mpu6050_read_accel(struct bsp_mpu6050_driver *p_mpu6050_instance,
								 float *p_accel_x, float *p_accel_y, float *p_accel_z);
static int8_t mpu6050_read_gyro(struct bsp_mpu6050_driver *p_mpu6050_instance,
								float *p_gyro_x, float *p_gyro_y, float *p_gyro_z);
static int8_t mpu6050_hibernating(struct bsp_mpu6050_driver *p_mpu6050_instance);
static int8_t mpu6050_wakeup(struct bsp_mpu6050_driver *p_mpu6050_instance);

/******************************************************************************
 * @name    mpu6050_inst
 * @brief   instancetiate the MPU6050 instance
 * @param   p_mpu6050_instance[in]
 * @param   p_iic_interface[in]
 * @param   p_yield_interface[in]
 * @param   p_delay_interface[in]
 *
 * @return  0 success
 *         -1 mpu6050_instance null
 *         -2 iic_interface null
 *         -3 rtos_yield null
 *         -4 delay null
 *         -5 mpu6050 ADDR error
 *****************************************************************************/
int8_t mpu6050_inst(bsp_mpu6050_driver_t *p_mpu6050_instance,
					mpu6050_iic_interface_t *p_iic_interface,
					mpu6050_yield_interface_t *p_yield_interface,
					mpu6050_delay_interface_t *p_delay_interface)
{
	if (NULL == p_mpu6050_instance)
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

	p_mpu6050_instance->p_iic_interface = p_iic_interface;
	p_mpu6050_instance->p_yield_interface = p_yield_interface;
	p_mpu6050_instance->p_delay_interface = p_delay_interface;

	p_mpu6050_instance->pf_inst = mpu6050_inst;
	p_mpu6050_instance->pf_deinst = mpu6050_deinst;
	p_mpu6050_instance->pf_init = mpu6050_init;
	p_mpu6050_instance->pf_deinit = mpu6050_deinit;
	p_mpu6050_instance->pf_read_id = mpu6050_read_id;
	p_mpu6050_instance->pf_read_accel = mpu6050_read_accel;
	p_mpu6050_instance->pf_read_gyro = mpu6050_read_gyro;
	p_mpu6050_instance->pf_hibernating = mpu6050_hibernating;
	p_mpu6050_instance->pf_wakeup = mpu6050_wakeup;

	/* 初始化 */
	p_mpu6050_instance->pf_init(p_mpu6050_instance);

	if (p_mpu6050_instance->pf_read_id(p_mpu6050_instance) != MPU6050_ID)
	{
		p_mpu6050_instance->pf_deinst(p_mpu6050_instance);
		return -5;
	}
	else
	{
		return 0;
	}
}

/******************************************************************************
 * @name    mpu6050_deinst
 * @brief   析构MPU6050实例
 * @param   p_mpu6050_instance[in]
 *
 * @return  0 success
 *         -1 mpu6050_instance null
 *****************************************************************************/
static int8_t mpu6050_deinst(struct bsp_mpu6050_driver *p_mpu6050_instance)
{
	if (NULL == p_mpu6050_instance)
		return -1;

	p_mpu6050_instance->pf_deinit(p_mpu6050_instance);

	p_mpu6050_instance->p_iic_interface = NULL;
	p_mpu6050_instance->p_yield_interface = NULL;
	p_mpu6050_instance->p_delay_interface = NULL;
	p_mpu6050_instance->pf_inst = NULL;
	p_mpu6050_instance->pf_deinst = NULL;
	p_mpu6050_instance->pf_init = NULL;
	p_mpu6050_instance->pf_deinit = NULL;
	p_mpu6050_instance->pf_read_id = NULL;
	p_mpu6050_instance->pf_read_accel = NULL;
	p_mpu6050_instance->pf_read_gyro = NULL;
	p_mpu6050_instance->pf_hibernating = NULL;
	p_mpu6050_instance->pf_wakeup = NULL;

	return 0;
}

/******************************************************************************
 * @name    mpu6050_init
 * @brief   MPU6050初始化: 唤醒设备、配置DLPF、陀螺仪和加速度计量程、
 *          采样率分频
 * @param   p_mpu6050_instance[in]
 *
 * @return  0 success
 *         -1 mpu6050_instance null
 *         -2 device reset failed
 *         -3 wakeup failed
 *         -4 dlpf config failed
 *         -5 gyro config failed
 *         -6 accel config failed
 *         -7 sample rate config failed
 *****************************************************************************/
static int8_t mpu6050_init(struct bsp_mpu6050_driver *p_mpu6050_instance)
{
	int8_t ret = 0;

	if (NULL == p_mpu6050_instance)
	{
		return -1;
	}

	/* 1. 软件复位设备 */
	ret = p_mpu6050_instance->p_iic_interface->pf_writereg(MPU6050_I2C_ADDR,
							MPU6050_PWR_MGMT_1,
							MPU6050_PWR1_DEVICE_RESET);
	if (0 != ret)
	{
		return -2;
	}

	/* 等待复位完成 (芯片复位后自动清除DEVICE_RESET位) */
	p_mpu6050_instance->p_delay_interface->pf_delay(MPU6050_STARTUP_DELAY_MS);

	// #ifdef OS_SUPPORTING
	// 	p_mpu6050_instance->p_yield_interface->pf_yield();
	// #endif /* OS_SUPPORTING */

	/* 验证复位后寄存器值 (SLEEP=1, 其余为0 → 0x40) */
	/* TBD: 如需要可读取PWR_MGMT_1验证 */

	/* 2. 唤醒设备, 选择PLL X轴陀螺仪作为时钟源以获得最佳稳定性 */
	ret = p_mpu6050_instance->p_iic_interface->pf_writereg(MPU6050_I2C_ADDR,
							MPU6050_PWR_MGMT_1,
							MPU6050_PWR1_CLKSEL_PLL_X);
	if (0 != ret)
	{
		return -3;
	}

	/* 等待时钟稳定 */
	p_mpu6050_instance->p_delay_interface->pf_delay(MPU6050_STARTUP_DELAY_MS);

	// #ifdef OS_SUPPORTING
	// 	p_mpu6050_instance->p_yield_interface->pf_yield();
	// #endif /* OS_SUPPORTING */

	/* 3. 配置DLPF: 禁用FSYNC, 设置低通滤波带宽 */
	ret = p_mpu6050_instance->p_iic_interface->pf_writereg(MPU6050_I2C_ADDR,
							MPU6050_CONFIG,
							MPU6050_DEFAULT_DLPF);
	if (0 != ret)
	{
		return -4;
	}

	/* 4. 配置陀螺仪量程 */
	ret = p_mpu6050_instance->p_iic_interface->pf_writereg(MPU6050_I2C_ADDR,
							MPU6050_GYRO_CONFIG,
							MPU6050_DEFAULT_GYRO_FS);
	if (0 != ret)
	{
		return -5;
	}

	/* 5. 配置加速度计量程 */
	ret = p_mpu6050_instance->p_iic_interface->pf_writereg(MPU6050_I2C_ADDR,
							MPU6050_ACCEL_CONFIG,
							MPU6050_DEFAULT_ACCEL_FS);
	if (0 != ret)
	{
		return -6;
	}

	/* 6. 配置采样率分频 (Sample Rate = Gyro Output Rate / (1 + DIV)) */
	ret = p_mpu6050_instance->p_iic_interface->pf_writereg(MPU6050_I2C_ADDR,
							MPU6050_SMPLRT_DIV,
							MPU6050_DEFAULT_SMPLRT_DIV);
	if (0 != ret)
	{
		return -7;
	}

	return 0;
}

/******************************************************************************
 * @name    mpu6050_deinit
 * @brief   MPU6050去初始化: 使设备进入睡眠模式
 * @param   p_mpu6050_instance[in]
 *
 * @return  0 success
 *         -1 mpu6050_instance null
 *         -2 sleep failed
 *****************************************************************************/
static int8_t mpu6050_deinit(struct bsp_mpu6050_driver *p_mpu6050_instance)
{
	int8_t ret = 0;

	if (NULL == p_mpu6050_instance)
	{
		return -1;
	}

	/* 设置睡眠位, 保持当前时钟源配置 */
	ret = p_mpu6050_instance->p_iic_interface->pf_writereg(MPU6050_I2C_ADDR,
							MPU6050_PWR_MGMT_1,
							MPU6050_PWR1_SLEEP);
	if (0 != ret)
	{
		return -2;
	}

	return 0;
}

/******************************************************************************
 * @name    mpu6050_read_id
 * @brief   读取MPU6050设备ID
 * @param   p_mpu6050_instance[in]
 *
 * @return  读取到的ID值
 *         -1 mpu6050_instance null
 *         -2 i2c read error
 *****************************************************************************/
static int8_t mpu6050_read_id(struct bsp_mpu6050_driver *p_mpu6050_instance)
{
	uint8_t id = 0;

	if (NULL == p_mpu6050_instance)
	{
		return -1;
	}

	if (0 != p_mpu6050_instance->p_iic_interface->pf_readreg(MPU6050_I2C_ADDR, MPU6050_WHO_AM_I, &id, 1))
	{
		return -2;
	}

	return id;
}

/******************************************************************************
 * @name    mpu6050_read_accel
 * @brief   读取MPU6050加速度计数据(三轴, 单位: g)
 * @param   p_mpu6050_instance[in]
 * @param   p_accel_x[out] X轴加速度
 * @param   p_accel_y[out] Y轴加速度
 * @param   p_accel_z[out] Z轴加速度
 *
 * @return  0 success
 *         -1 mpu6050_instance null
 *         -2 i2c read error
 *****************************************************************************/
static int8_t mpu6050_read_accel(struct bsp_mpu6050_driver *p_mpu6050_instance,
								 float *p_accel_x, float *p_accel_y, float *p_accel_z)
{
	uint8_t buf[6];
	int16_t raw_x, raw_y, raw_z;

	if (NULL == p_mpu6050_instance)
	{
		return -1;
	}

	if (0 != p_mpu6050_instance->p_iic_interface->pf_readreg(MPU6050_I2C_ADDR,
									MPU6050_ACCEL_XOUT_H, buf, 6))
	{
		return -2;
	}

	/* 大端转小端 (MPU6050内部为大端序) */
	raw_x = (int16_t)((buf[0] << 8) | buf[1]);
	raw_y = (int16_t)((buf[2] << 8) | buf[3]);
	raw_z = (int16_t)((buf[4] << 8) | buf[5]);

	/* 转换为 g (使用 ±2g 量程灵敏度) */
	*p_accel_x = (float)raw_x / MPU6050_ACCEL_SENS_2G;
	*p_accel_y = (float)raw_y / MPU6050_ACCEL_SENS_2G;
	*p_accel_z = (float)raw_z / MPU6050_ACCEL_SENS_2G;

	return 0;
}

/******************************************************************************
 * @name    mpu6050_read_gyro
 * @brief   读取MPU6050陀螺仪数据(三轴, 单位: °/s)
 * @param   p_mpu6050_instance[in]
 * @param   p_gyro_x[out] X轴角速度
 * @param   p_gyro_y[out] Y轴角速度
 * @param   p_gyro_z[out] Z轴角速度
 *
 * @return  0 success
 *         -1 mpu6050_instance null
 *         -2 i2c read error
 *****************************************************************************/
static int8_t mpu6050_read_gyro(struct bsp_mpu6050_driver *p_mpu6050_instance,
								float *p_gyro_x, float *p_gyro_y, float *p_gyro_z)
{
	uint8_t buf[6];
	int16_t raw_x, raw_y, raw_z;

	if (NULL == p_mpu6050_instance)
	{
		return -1;
	}

	if (0 != p_mpu6050_instance->p_iic_interface->pf_readreg(MPU6050_I2C_ADDR,
									MPU6050_GYRO_XOUT_H, buf, 6))
	{
		return -2;
	}

	/* 大端转小端 */
	raw_x = (int16_t)((buf[0] << 8) | buf[1]);
	raw_y = (int16_t)((buf[2] << 8) | buf[3]);
	raw_z = (int16_t)((buf[4] << 8) | buf[5]);

	/* 转换为 °/s (使用 ±250°/s 量程灵敏度) */
	*p_gyro_x = (float)raw_x / MPU6050_GYRO_SENS_250;
	*p_gyro_y = (float)raw_y / MPU6050_GYRO_SENS_250;
	*p_gyro_z = (float)raw_z / MPU6050_GYRO_SENS_250;

	return 0;
}

/******************************************************************************
 * @name    mpu6050_hibernating
 * @brief   使MPU6050进入休眠模式(保留时钟源配置)
 * @param   p_mpu6050_instance[in]
 *
 * @return  0 success
 *         -1 mpu6050_instance null
 *         -2 sleep failed
 *****************************************************************************/
static int8_t mpu6050_hibernating(struct bsp_mpu6050_driver *p_mpu6050_instance)
{
	int8_t ret = 0;

	if (NULL == p_mpu6050_instance)
	{
		return -1;
	}

	/* 设置SLEEP位, 保留当前时钟源配置 */
	ret = p_mpu6050_instance->p_iic_interface->pf_writereg(MPU6050_I2C_ADDR,
							MPU6050_PWR_MGMT_1,
							MPU6050_PWR1_SLEEP);
	if (0 != ret)
	{
		return -2;
	}

	return 0;
}

/******************************************************************************
 * @name    mpu6050_wakeup
 * @brief   唤醒MPU6050(清除睡眠位, 保留时钟源配置)
 * @param   p_mpu6050_instance[in]
 *
 * @return  0 success
 *         -1 mpu6050_instance null
 *         -2 wakeup failed
 *****************************************************************************/
static int8_t mpu6050_wakeup(struct bsp_mpu6050_driver *p_mpu6050_instance)
{
	int8_t ret = 0;

	if (NULL == p_mpu6050_instance)
	{
		return -1;
	}

	/* 清除SLEEP位, 使用PLL X轴陀螺仪时钟 */
	ret = p_mpu6050_instance->p_iic_interface->pf_writereg(MPU6050_I2C_ADDR,
							MPU6050_PWR_MGMT_1,
							MPU6050_PWR1_CLKSEL_PLL_X);
	if (0 != ret)
	{
		return -2;
	}

	/* 等待时钟稳定 */
	p_mpu6050_instance->p_delay_interface->pf_delay(MPU6050_STARTUP_DELAY_MS);

	return 0;
}
