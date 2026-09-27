/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_bsp_st7789t3_driver.c
 *
 * @par dependencies
 * - cywatch_bsp_st7789t3_driver.h
 *
 * @author zw1194
 *
 * @brief Implete the HAL operations of ST7789T3 and corresponding opetions.
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
#include "../Inc/cywatch_bsp_st7789t3_driver.h"

/* 默认配置 */
#define ST7789T3_DEFAULT_DIRECTION   st7789t3_dir_0   /* 默认 0° 竖屏 */
#define ST7789T3_PIXEL_CHUNK_SIZE    4096             /* 单次DMA发送像素数(8KB缓冲); 原8192为腾RAM给LVGL内存池缩半, 仅影响DMA分块次数 */

/* 时序(单位ms) */
#define ST7789T3_RESET_LOW_MS        10      /* 复位低电平脉冲 */
#define ST7789T3_RESET_HIGH_MS       10      /* 复位释放后保持 */
#define ST7789T3_RESET_STABLE_MS     5       /* 复位后首条命令前等待 */
#define ST7789T3_SLEEP_OUT_MS        120     /* SLPOUT后电源/时钟稳定 */
#define ST7789T3_SLEEP_IN_MS         120     /* SLPIN后进入睡眠 */
#define ST7789T3_DISPLAY_ON_MS       10      /* DISPON后 */

/* 初始化参数表 (取自ST7789T3数据手册典型值) */
static uint8_t st7789t3_colmod_value[]   = {0x55};
static uint8_t st7789t3_porctrl_value[]  = {0x0C, 0x0C, 0x00, 0x33, 0x33};
static uint8_t st7789t3_vcoms_value[]    = {0x1F};
static uint8_t st7789t3_lcmctrl_value[]  = {0x01};
static uint8_t st7789t3_vrhset_value[]   = {0x13};
static uint8_t st7789t3_vdvset_value[]   = {0x20};
static uint8_t st7789t3_vcmofset_value[] = {0x00};
static uint8_t st7789t3_frctrl2_value[]  = {0x0F};
static uint8_t st7789t3_pwctrl1_value[]  = {0xA4, 0xA1};
static uint8_t st7789t3_gmctrp1_value[]  = {0xF0, 0x05, 0x0A, 0x06, 0x06, 0x03, 0x2B, 0x32,
											0x43, 0x39, 0x17, 0x15, 0x2E, 0x35};
static uint8_t st7789t3_gmctrn1_value[]  = {0xF0, 0x08, 0x0C, 0x0A, 0x0A, 0x26, 0x2B, 0x22,
											0x25, 0x2A, 0x29, 0x2F, 0x3C, 0x3F};

static int8_t st7789t3_deinst(bsp_st7789t3_driver_t *p_st7789t3_instance);
static int8_t st7789t3_init(bsp_st7789t3_driver_t *p_st7789t3_instance);
static int8_t st7789t3_deinit(bsp_st7789t3_driver_t *p_st7789t3_instance);
static int8_t st7789t3_reset(bsp_st7789t3_driver_t *p_st7789t3_instance);
static int8_t st7789t3_set_direction(bsp_st7789t3_driver_t *p_st7789t3_instance,
									 st7789t3_direction_t direction);
static int8_t st7789t3_display_on(bsp_st7789t3_driver_t *p_st7789t3_instance);
static int8_t st7789t3_display_off(bsp_st7789t3_driver_t *p_st7789t3_instance);
static int8_t st7789t3_sleep(bsp_st7789t3_driver_t *p_st7789t3_instance);
static int8_t st7789t3_wakeup(bsp_st7789t3_driver_t *p_st7789t3_instance);
static int8_t st7789t3_invert_on(bsp_st7789t3_driver_t *p_st7789t3_instance);
static int8_t st7789t3_invert_off(bsp_st7789t3_driver_t *p_st7789t3_instance);
static int8_t st7789t3_set_backlight(bsp_st7789t3_driver_t *p_st7789t3_instance,
									 uint8_t value);
static int8_t st7789t3_set_window(bsp_st7789t3_driver_t *p_st7789t3_instance,
								  uint16_t x0, uint16_t y0,
								  uint16_t x1, uint16_t y1);
static int8_t st7789t3_write_pixels(bsp_st7789t3_driver_t *p_st7789t3_instance,
									uint16_t *pdata, uint32_t size);
static int8_t st7789t3_fill(bsp_st7789t3_driver_t *p_st7789t3_instance,
							uint16_t x0, uint16_t y0,
							uint16_t x1, uint16_t y1, uint16_t color);
static int8_t st7789t3_clear(bsp_st7789t3_driver_t *p_st7789t3_instance,
							 uint16_t color);
static int8_t st7789t3_draw_point(bsp_st7789t3_driver_t *p_st7789t3_instance,
								  uint16_t x, uint16_t y, uint16_t color);

static int8_t st7789t3_cs_low(bsp_st7789t3_driver_t *p_st7789t3_instance);
static int8_t st7789t3_cs_high(bsp_st7789t3_driver_t *p_st7789t3_instance);
static int8_t st7789t3_send_command(bsp_st7789t3_driver_t *p_st7789t3_instance,
									uint8_t cmd);
static int8_t st7789t3_send_data(bsp_st7789t3_driver_t *p_st7789t3_instance,
								 uint8_t *pdata, uint32_t size);
static int8_t st7789t3_send_pixels(bsp_st7789t3_driver_t *p_st7789t3_instance,
								   uint16_t *pdata, uint32_t size);
static int8_t st7789t3_write_command(bsp_st7789t3_driver_t *p_st7789t3_instance,
									 uint8_t cmd);
static int8_t st7789t3_write_command_data(bsp_st7789t3_driver_t *p_st7789t3_instance,
										  uint8_t cmd, uint8_t *pdata, uint32_t size);
static int8_t st7789t3_apply_direction(bsp_st7789t3_driver_t *p_st7789t3_instance,
									   st7789t3_direction_t direction, uint8_t *p_madctl);

/******************************************************************************
 * @name    st7789t3_cs_low
 * @brief   拉低CS片选, 开始一次传输
 * @param   p_st7789t3_instance[in]
 *
 * @return  0 success
 *         -1 st7789t3_instance null
 *****************************************************************************/
static int8_t st7789t3_cs_low(bsp_st7789t3_driver_t *p_st7789t3_instance)
{
	if (NULL == p_st7789t3_instance)
	{
		return -1;
	}

	p_st7789t3_instance->p_gpio_interface->pf_cs_set(0);

	return 0;
}

/******************************************************************************
 * @name    st7789t3_cs_high
 * @brief   拉高CS片选, 结束一次传输
 * @param   p_st7789t3_instance[in]
 *
 * @return  0 success
 *         -1 st7789t3_instance null
 *****************************************************************************/
static int8_t st7789t3_cs_high(bsp_st7789t3_driver_t *p_st7789t3_instance)
{
	if (NULL == p_st7789t3_instance)
	{
		return -1;
	}

	p_st7789t3_instance->p_gpio_interface->pf_cs_set(1);

	return 0;
}

/******************************************************************************
 * @name    st7789t3_send_command
 * @brief   DC拉低, 发送单字节命令
 * @param   p_st7789t3_instance[in]
 * @param   cmd[in] 命令字节
 *
 * @return  0 success
 *         -1 st7789t3_instance null
 *         -2 spi write error
 *****************************************************************************/
static int8_t st7789t3_send_command(bsp_st7789t3_driver_t *p_st7789t3_instance,
									uint8_t cmd)
{
	if (NULL == p_st7789t3_instance)
	{
		return -1;
	}

	p_st7789t3_instance->p_gpio_interface->pf_dc_set(0);

	if (0 != p_st7789t3_instance->p_spi_interface->pf_send_bytes(&cmd, 1))
	{
		return -2;
	}

	return 0;
}

/******************************************************************************
 * @name    st7789t3_send_data
 * @brief   DC拉高, 发送数据字节流(参数)
 * @param   p_st7789t3_instance[in]
 * @param   pdata[in] 数据缓冲区
 * @param   size[in] 字节数
 *
 * @return  0 success
 *         -1 st7789t3_instance null
 *         -2 spi write error
 *****************************************************************************/
static int8_t st7789t3_send_data(bsp_st7789t3_driver_t *p_st7789t3_instance,
								 uint8_t *pdata, uint32_t size)
{
	if (NULL == p_st7789t3_instance)
	{
		return -1;
	}

	p_st7789t3_instance->p_gpio_interface->pf_dc_set(1);

	if (0 != p_st7789t3_instance->p_spi_interface->pf_send_bytes(pdata, size))
	{
		return -2;
	}

	return 0;
}

/******************************************************************************
 * @name    st7789t3_send_pixels
 * @brief   DC拉高, 将RGB565像素流转换为高字节先行字节流后经DMA发送
 * @param   p_st7789t3_instance[in]
 * @param   pdata[in] 像素缓冲区(RGB565)
 * @param   size[in] 像素个数
 *
 * @return  0 success
 *         -1 st7789t3_instance null
 *         -2 spi dma write error
 *
 * @note    RGB565为16位小端存储, SPI需高字节先行, 此处手动交换后分块DMA发送;
 *          底层 spi_hal 的 pf_send_bytes_dma 内部完成启动+等待
 *****************************************************************************/
static int8_t st7789t3_send_pixels(bsp_st7789t3_driver_t *p_st7789t3_instance,
								   uint16_t *pdata, uint32_t size)
{
	static uint8_t buf[ST7789T3_PIXEL_CHUNK_SIZE * 2];
	uint32_t sent = 0;

	if (NULL == p_st7789t3_instance)
	{
		return -1;
	}

	p_st7789t3_instance->p_gpio_interface->pf_dc_set(1);

	while (sent < size)
	{
		uint32_t n = size - sent;
		uint32_t i;

		if (n > ST7789T3_PIXEL_CHUNK_SIZE)
		{
			n = ST7789T3_PIXEL_CHUNK_SIZE;
		}

		/* RGB565高字节先行(STM32小端存储需手动交换) */
		for (i = 0; i < n; i++)
		{
			buf[i * 2]     = (uint8_t)(pdata[sent + i] >> 8);
			buf[i * 2 + 1] = (uint8_t)(pdata[sent + i] & 0xFF);
		}

		if (0 != p_st7789t3_instance->p_spi_interface->pf_send_bytes_dma(buf, n * 2))
		{
			return -2;
		}

		sent += n;
	}

	return 0;
}

/******************************************************************************
 * @name    st7789t3_write_command
 * @brief   发送单条命令(无参数): CS拉低→命令→CS拉高
 * @param   p_st7789t3_instance[in]
 * @param   cmd[in] 命令字节
 *
 * @return  0 success
 *         -1 st7789t3_instance null
 *         -2 spi write error
 *****************************************************************************/
static int8_t st7789t3_write_command(bsp_st7789t3_driver_t *p_st7789t3_instance,
									 uint8_t cmd)
{
	int8_t ret = 0;

	if (NULL == p_st7789t3_instance)
	{
		return -1;
	}

	st7789t3_cs_low(p_st7789t3_instance);
	ret = st7789t3_send_command(p_st7789t3_instance, cmd);
	st7789t3_cs_high(p_st7789t3_instance);

	return ret;
}

/******************************************************************************
 * @name    st7789t3_write_command_data
 * @brief   发送命令+参数: CS拉低→命令→数据→CS拉高
 * @param   p_st7789t3_instance[in]
 * @param   cmd[in] 命令字节
 * @param   pdata[in] 参数字节
 * @param   size[in] 参数个数
 *
 * @return  0 success
 *         -1 st7789t3_instance null
 *         -2 spi write error
 *****************************************************************************/
static int8_t st7789t3_write_command_data(bsp_st7789t3_driver_t *p_st7789t3_instance,
										  uint8_t cmd, uint8_t *pdata, uint32_t size)
{
	int8_t ret = 0;

	if (NULL == p_st7789t3_instance)
	{
		return -1;
	}

	st7789t3_cs_low(p_st7789t3_instance);

	ret = st7789t3_send_command(p_st7789t3_instance, cmd);
	if (0 == ret)
	{
		ret = st7789t3_send_data(p_st7789t3_instance, pdata, size);
	}

	st7789t3_cs_high(p_st7789t3_instance);

	return ret;
}

/******************************************************************************
 * @name    st7789t3_apply_direction
 * @brief   根据方向计算MADCTL值与逻辑宽高, 并更新实例状态
 * @param   p_st7789t3_instance[in]
 * @param   direction[in] 显示方向
 * @param   p_madctl[out] 计算出的MADCTL寄存器值
 *
 * @return  0 success
 *         -1 st7789t3_instance null
 *         -2 invalid direction
 *****************************************************************************/
static int8_t st7789t3_apply_direction(bsp_st7789t3_driver_t *p_st7789t3_instance,
									   st7789t3_direction_t direction, uint8_t *p_madctl)
{
	if (NULL == p_st7789t3_instance)
	{
		return -1;
	}

	switch (direction)
	{
	case st7789t3_dir_0:
		*p_madctl = ST7789T3_MADCTL_ROT_0;
		p_st7789t3_instance->width = ST7789T3_WIDTH;
		p_st7789t3_instance->height = ST7789T3_HEIGHT;
		break;

	case st7789t3_dir_90:
		*p_madctl = ST7789T3_MADCTL_ROT_90;
		p_st7789t3_instance->width = ST7789T3_HEIGHT;
		p_st7789t3_instance->height = ST7789T3_WIDTH;
		break;

	case st7789t3_dir_180:
		*p_madctl = ST7789T3_MADCTL_ROT_180;
		p_st7789t3_instance->width = ST7789T3_WIDTH;
		p_st7789t3_instance->height = ST7789T3_HEIGHT;
		break;

	case st7789t3_dir_270:
		*p_madctl = ST7789T3_MADCTL_ROT_270;
		p_st7789t3_instance->width = ST7789T3_HEIGHT;
		p_st7789t3_instance->height = ST7789T3_WIDTH;
		break;

	default:
		return -2;
	}

	p_st7789t3_instance->direction = direction;

	return 0;
}

/******************************************************************************
 * @name    st7789t3_inst
 * @brief   instancetiate the ST7789T3 instance
 * @param   p_st7789t3_instance[in]
 * @param   p_spi_interface[in]
 * @param   p_gpio_interface[in]
 * @param   p_delay_interface[in]
 * @param   p_pwm_interface[in]
 *
 * @return  0 success
 *         -1 st7789t3_instance null
 *         -2 spi_interface null
 *         -3 gpio_interface null
 *         -4 delay null
 *         -5 pwm_interface null
 *         -6 init failed
 *
 * @note    接口结构体一律不带实例成员(见头文件), 因此这边只校验 pf_* 非空;
 *          原来的 -4 rtos_yield null 一档随 st7789t3_yield_interface_t 一并删除
 *          (该接口此前只被非空校验、从未被调用), 其后各档按"错误出现顺序递增"
 *          的约定整体上移一位
 *****************************************************************************/
int8_t st7789t3_inst(bsp_st7789t3_driver_t *p_st7789t3_instance,
					 st7789t3_spi_interface_t *p_spi_interface,
					 st7789t3_gpio_interface_t *p_gpio_interface,
					 st7789t3_delay_interface_t *p_delay_interface,
					 st7789t3_pwm_interface_t *p_pwm_interface)
{
	if (NULL == p_st7789t3_instance)
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
		if (NULL == p_spi_interface->pf_send_bytes ||
			NULL == p_spi_interface->pf_send_bytes_dma)
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
		if (NULL == p_gpio_interface->pf_dc_set ||
			NULL == p_gpio_interface->pf_cs_set ||
			NULL == p_gpio_interface->pf_rst_set)
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

	if (NULL == p_pwm_interface)
	{
		return -5;
	}
	else
	{
		if (NULL == p_pwm_interface->pf_backlight_set)
		{
			return -5;
		}
	}

	p_st7789t3_instance->p_spi_interface = p_spi_interface;
	p_st7789t3_instance->p_gpio_interface = p_gpio_interface;
	p_st7789t3_instance->p_delay_interface = p_delay_interface;
	p_st7789t3_instance->p_pwm_interface = p_pwm_interface;

	p_st7789t3_instance->pf_inst = st7789t3_inst;
	p_st7789t3_instance->pf_deinst = st7789t3_deinst;
	p_st7789t3_instance->pf_init = st7789t3_init;
	p_st7789t3_instance->pf_deinit = st7789t3_deinit;
	p_st7789t3_instance->pf_reset = st7789t3_reset;
	p_st7789t3_instance->pf_set_direction = st7789t3_set_direction;
	p_st7789t3_instance->pf_display_on = st7789t3_display_on;
	p_st7789t3_instance->pf_display_off = st7789t3_display_off;
	p_st7789t3_instance->pf_sleep = st7789t3_sleep;
	p_st7789t3_instance->pf_wakeup = st7789t3_wakeup;
	p_st7789t3_instance->pf_invert_on = st7789t3_invert_on;
	p_st7789t3_instance->pf_invert_off = st7789t3_invert_off;
	p_st7789t3_instance->pf_set_backlight = st7789t3_set_backlight;
	p_st7789t3_instance->pf_set_window = st7789t3_set_window;
	p_st7789t3_instance->pf_write_pixels = st7789t3_write_pixels;
	p_st7789t3_instance->pf_fill = st7789t3_fill;
	p_st7789t3_instance->pf_clear = st7789t3_clear;
	p_st7789t3_instance->pf_draw_point = st7789t3_draw_point;

	/* 初始化 */
	if (0 != p_st7789t3_instance->pf_init(p_st7789t3_instance))
	{
		p_st7789t3_instance->pf_deinst(p_st7789t3_instance);
		return -6;
	}

	return 0;
}

/******************************************************************************
 * @name    st7789t3_deinst
 * @brief   析构ST7789T3实例
 * @param   p_st7789t3_instance[in]
 *
 * @return  0 success
 *         -1 st7789t3_instance null
 *****************************************************************************/
static int8_t st7789t3_deinst(bsp_st7789t3_driver_t *p_st7789t3_instance)
{
	if (NULL == p_st7789t3_instance)
	{
		return -1;
	}

	p_st7789t3_instance->pf_deinit(p_st7789t3_instance);

	p_st7789t3_instance->p_spi_interface = NULL;
	p_st7789t3_instance->p_gpio_interface = NULL;
	p_st7789t3_instance->p_delay_interface = NULL;
	p_st7789t3_instance->p_pwm_interface = NULL;
	p_st7789t3_instance->width = 0;
	p_st7789t3_instance->height = 0;
	p_st7789t3_instance->direction = st7789t3_dir_0;

	p_st7789t3_instance->pf_inst = NULL;
	p_st7789t3_instance->pf_deinst = NULL;
	p_st7789t3_instance->pf_init = NULL;
	p_st7789t3_instance->pf_deinit = NULL;
	p_st7789t3_instance->pf_reset = NULL;
	p_st7789t3_instance->pf_set_direction = NULL;
	p_st7789t3_instance->pf_display_on = NULL;
	p_st7789t3_instance->pf_display_off = NULL;
	p_st7789t3_instance->pf_sleep = NULL;
	p_st7789t3_instance->pf_wakeup = NULL;
	p_st7789t3_instance->pf_invert_on = NULL;
	p_st7789t3_instance->pf_invert_off = NULL;
	p_st7789t3_instance->pf_set_backlight = NULL;
	p_st7789t3_instance->pf_set_window = NULL;
	p_st7789t3_instance->pf_write_pixels = NULL;
	p_st7789t3_instance->pf_fill = NULL;
	p_st7789t3_instance->pf_clear = NULL;
	p_st7789t3_instance->pf_draw_point = NULL;

	return 0;
}

/******************************************************************************
 * @name    st7789t3_init
 * @brief   ST7789T3初始化: 硬件复位、退出睡眠、配置像素格式/方向/电源/Gamma,
 *          开启反转与显示输出
 * @param   p_st7789t3_instance[in]
 *
 * @return  0 success
 *         -1 st7789t3_instance null
 *         -2 hardware reset failed
 *         -3 sleep out failed
 *         -4 colmod config failed
 *         -5 madctl config failed
 *         -6 porctrl config failed
 *         -7 vcoms config failed
 *         -8 lcmctrl config failed
 *         -9 vrhset config failed
 *         -10 vdvset config failed
 *         -11 vcmofset config failed
 *         -12 frctrl2 config failed
 *         -13 pwctrl1 config failed
 *         -14 gmctrp1 config failed
 *         -15 gmctrn1 config failed
 *         -16 invert on failed
 *         -17 display on failed
 *****************************************************************************/
static int8_t st7789t3_init(bsp_st7789t3_driver_t *p_st7789t3_instance)
{
	int8_t ret = 0;
	uint8_t madctl = 0;

	if (NULL == p_st7789t3_instance)
	{
		return -1;
	}

	/* 1. 硬件复位 */
	ret = st7789t3_reset(p_st7789t3_instance);
	if (0 != ret)
	{
		return -2;
	}

	/* 2. 退出睡眠模式(初始化必须的第一条命令) */
	ret = st7789t3_write_command(p_st7789t3_instance, ST7789T3_SLPOUT);
	if (0 != ret)
	{
		return -3;
	}

	/* 等待电源与时钟稳定 */
	p_st7789t3_instance->p_delay_interface->pf_delay(ST7789T3_SLEEP_OUT_MS);

	/* 3. 配置像素格式 RGB565 */
	ret = st7789t3_write_command_data(p_st7789t3_instance, ST7789T3_COLMOD,
									  st7789t3_colmod_value, sizeof(st7789t3_colmod_value));
	if (0 != ret)
	{
		return -4;
	}

	/* 4. 配置显示方向(同时更新实例的width/height) */
	if (0 != st7789t3_apply_direction(p_st7789t3_instance,
									  ST7789T3_DEFAULT_DIRECTION, &madctl))
	{
		return -5;
	}

	ret = st7789t3_write_command_data(p_st7789t3_instance, ST7789T3_MADCTL,
									  &madctl, 1);
	if (0 != ret)
	{
		return -5;
	}

	/* 5. 栅极升压控制 */
	ret = st7789t3_write_command_data(p_st7789t3_instance, ST7789T3_PORCTRL,
									  st7789t3_porctrl_value, sizeof(st7789t3_porctrl_value));
	if (0 != ret)
	{
		return -6;
	}

	/* 6. VCOM电压设置 */
	ret = st7789t3_write_command_data(p_st7789t3_instance, ST7789T3_VCOMS,
									  st7789t3_vcoms_value, sizeof(st7789t3_vcoms_value));
	if (0 != ret)
	{
		return -7;
	}

	/* 7. LCM控制 */
	ret = st7789t3_write_command_data(p_st7789t3_instance, ST7789T3_LCMCTRL,
									  st7789t3_lcmctrl_value, sizeof(st7789t3_lcmctrl_value));
	if (0 != ret)
	{
		return -8;
	}

	/* 8. 正电压调节 */
	ret = st7789t3_write_command_data(p_st7789t3_instance, ST7789T3_VRHSET,
									  st7789t3_vrhset_value, sizeof(st7789t3_vrhset_value));
	if (0 != ret)
	{
		return -9;
	}

	/* 9. 负电压调节 */
	ret = st7789t3_write_command_data(p_st7789t3_instance, ST7789T3_VDVSET,
									  st7789t3_vdvset_value, sizeof(st7789t3_vdvset_value));
	if (0 != ret)
	{
		return -10;
	}

	/* 10. VCOM偏移校准 */
	ret = st7789t3_write_command_data(p_st7789t3_instance, ST7789T3_VCMOFSET,
									  st7789t3_vcmofset_value, sizeof(st7789t3_vcmofset_value));
	if (0 != ret)
	{
		return -11;
	}

	/* 11. 正常模式帧率控制 */
	ret = st7789t3_write_command_data(p_st7789t3_instance, ST7789T3_FRCTRL2,
									  st7789t3_frctrl2_value, sizeof(st7789t3_frctrl2_value));
	if (0 != ret)
	{
		return -12;
	}

	/* 12. 电源控制1 */
	ret = st7789t3_write_command_data(p_st7789t3_instance, ST7789T3_PWCTRL1,
									  st7789t3_pwctrl1_value, sizeof(st7789t3_pwctrl1_value));
	if (0 != ret)
	{
		return -13;
	}

	/* 13. 正向Gamma校正 */
	ret = st7789t3_write_command_data(p_st7789t3_instance, ST7789T3_GMCTRP1,
									  st7789t3_gmctrp1_value, sizeof(st7789t3_gmctrp1_value));
	if (0 != ret)
	{
		return -14;
	}

	/* 14. 负向Gamma校正 */
	ret = st7789t3_write_command_data(p_st7789t3_instance, ST7789T3_GMCTRN1,
									  st7789t3_gmctrn1_value, sizeof(st7789t3_gmctrn1_value));
	if (0 != ret)
	{
		return -15;
	}

	/* 15. 开启显示反转(IPS面板需要, 否则明暗反向) */
	ret = st7789t3_write_command(p_st7789t3_instance, ST7789T3_INVON);
	if (0 != ret)
	{
		return -16;
	}

	/* 16. 开启显示输出(初始化最后一条命令) */
	ret = st7789t3_write_command(p_st7789t3_instance, ST7789T3_DISPON);
	if (0 != ret)
	{
		return -17;
	}

	p_st7789t3_instance->p_delay_interface->pf_delay(ST7789T3_DISPLAY_ON_MS);

	return 0;
}

/******************************************************************************
 * @name    st7789t3_deinit
 * @brief   ST7789T3去初始化: 关闭显示并进入睡眠模式
 * @param   p_st7789t3_instance[in]
 *
 * @return  0 success
 *         -1 st7789t3_instance null
 *         -2 display off failed
 *         -3 sleep in failed
 *****************************************************************************/
static int8_t st7789t3_deinit(bsp_st7789t3_driver_t *p_st7789t3_instance)
{
	int8_t ret = 0;

	if (NULL == p_st7789t3_instance)
	{
		return -1;
	}

	/* 关闭显示输出 */
	ret = st7789t3_write_command(p_st7789t3_instance, ST7789T3_DISPOFF);
	if (0 != ret)
	{
		return -2;
	}

	/* 进入睡眠模式 */
	ret = st7789t3_write_command(p_st7789t3_instance, ST7789T3_SLPIN);
	if (0 != ret)
	{
		return -3;
	}

	p_st7789t3_instance->p_delay_interface->pf_delay(ST7789T3_SLEEP_IN_MS);

	return 0;
}

/******************************************************************************
 * @name    st7789t3_reset
 * @brief   硬件复位: RST引脚低电平脉冲后等待稳定
 * @param   p_st7789t3_instance[in]
 *
 * @return  0 success
 *         -1 st7789t3_instance null
 *****************************************************************************/
static int8_t st7789t3_reset(bsp_st7789t3_driver_t *p_st7789t3_instance)
{
	st7789t3_gpio_interface_t *p_gpio;

	if (NULL == p_st7789t3_instance)
	{
		return -1;
	}

	p_gpio = p_st7789t3_instance->p_gpio_interface;

	/* RST低电平脉冲 */
	p_gpio->pf_rst_set(0);
	p_st7789t3_instance->p_delay_interface->pf_delay(ST7789T3_RESET_LOW_MS);
	p_gpio->pf_rst_set(1);
	p_st7789t3_instance->p_delay_interface->pf_delay(ST7789T3_RESET_HIGH_MS);

	/* 复位后等待稳定再发首条命令 */
	p_st7789t3_instance->p_delay_interface->pf_delay(ST7789T3_RESET_STABLE_MS);

	return 0;
}

/******************************************************************************
 * @name    st7789t3_set_direction
 * @brief   设置显示方向(旋转), 并更新实例逻辑宽高
 * @param   p_st7789t3_instance[in]
 * @param   direction[in] 显示方向
 *
 * @return  0 success
 *         -1 st7789t3_instance null
 *         -2 invalid direction
 *         -3 spi write error
 *****************************************************************************/
static int8_t st7789t3_set_direction(bsp_st7789t3_driver_t *p_st7789t3_instance,
									 st7789t3_direction_t direction)
{
	uint8_t madctl = 0;

	if (NULL == p_st7789t3_instance)
	{
		return -1;
	}

	if (0 != st7789t3_apply_direction(p_st7789t3_instance, direction, &madctl))
	{
		return -2;
	}

	if (0 != st7789t3_write_command_data(p_st7789t3_instance, ST7789T3_MADCTL,
										 &madctl, 1))
	{
		return -3;
	}

	return 0;
}

/******************************************************************************
 * @name    st7789t3_display_on
 * @brief   开启显示输出
 * @param   p_st7789t3_instance[in]
 *
 * @return  0 success
 *         -1 st7789t3_instance null
 *         -2 spi write error
 *****************************************************************************/
static int8_t st7789t3_display_on(bsp_st7789t3_driver_t *p_st7789t3_instance)
{
	if (NULL == p_st7789t3_instance)
	{
		return -1;
	}

	if (0 != st7789t3_write_command(p_st7789t3_instance, ST7789T3_DISPON))
	{
		return -2;
	}

	p_st7789t3_instance->p_delay_interface->pf_delay(ST7789T3_DISPLAY_ON_MS);

	return 0;
}

/******************************************************************************
 * @name    st7789t3_display_off
 * @brief   关闭显示输出(GRAM内容保留)
 * @param   p_st7789t3_instance[in]
 *
 * @return  0 success
 *         -1 st7789t3_instance null
 *         -2 spi write error
 *****************************************************************************/
static int8_t st7789t3_display_off(bsp_st7789t3_driver_t *p_st7789t3_instance)
{
	if (NULL == p_st7789t3_instance)
	{
		return -1;
	}

	if (0 != st7789t3_write_command(p_st7789t3_instance, ST7789T3_DISPOFF))
	{
		return -2;
	}

	return 0;
}

/******************************************************************************
 * @name    st7789t3_sleep
 * @brief   进入睡眠模式(关闭内部振荡器, 低功耗)
 * @param   p_st7789t3_instance[in]
 *
 * @return  0 success
 *         -1 st7789t3_instance null
 *         -2 spi write error
 *****************************************************************************/
static int8_t st7789t3_sleep(bsp_st7789t3_driver_t *p_st7789t3_instance)
{
	if (NULL == p_st7789t3_instance)
	{
		return -1;
	}

	if (0 != st7789t3_write_command(p_st7789t3_instance, ST7789T3_SLPIN))
	{
		return -2;
	}

	p_st7789t3_instance->p_delay_interface->pf_delay(ST7789T3_SLEEP_IN_MS);

	return 0;
}

/******************************************************************************
 * @name    st7789t3_wakeup
 * @brief   退出睡眠模式
 * @param   p_st7789t3_instance[in]
 *
 * @return  0 success
 *         -1 st7789t3_instance null
 *         -2 spi write error
 *****************************************************************************/
static int8_t st7789t3_wakeup(bsp_st7789t3_driver_t *p_st7789t3_instance)
{
	if (NULL == p_st7789t3_instance)
	{
		return -1;
	}

	if (0 != st7789t3_write_command(p_st7789t3_instance, ST7789T3_SLPOUT))
	{
		return -2;
	}

	p_st7789t3_instance->p_delay_interface->pf_delay(ST7789T3_SLEEP_OUT_MS);

	return 0;
}

/******************************************************************************
 * @name    st7789t3_invert_on
 * @brief   开启显示反转(IPS面板通常需要)
 * @param   p_st7789t3_instance[in]
 *
 * @return  0 success
 *         -1 st7789t3_instance null
 *         -2 spi write error
 *****************************************************************************/
static int8_t st7789t3_invert_on(bsp_st7789t3_driver_t *p_st7789t3_instance)
{
	if (NULL == p_st7789t3_instance)
	{
		return -1;
	}

	if (0 != st7789t3_write_command(p_st7789t3_instance, ST7789T3_INVON))
	{
		return -2;
	}

	return 0;
}

/******************************************************************************
 * @name    st7789t3_invert_off
 * @brief   关闭显示反转
 * @param   p_st7789t3_instance[in]
 *
 * @return  0 success
 *         -1 st7789t3_instance null
 *         -2 spi write error
 *****************************************************************************/
static int8_t st7789t3_invert_off(bsp_st7789t3_driver_t *p_st7789t3_instance)
{
	if (NULL == p_st7789t3_instance)
	{
		return -1;
	}

	if (0 != st7789t3_write_command(p_st7789t3_instance, ST7789T3_INVOFF))
	{
		return -2;
	}

	return 0;
}

/******************************************************************************
 * @name    st7789t3_set_backlight
 * @brief   设置背光亮度(透传给底层, 具体占空比由底层实现)
 * @param   p_st7789t3_instance[in]
 * @param   value[in] 背光值(0=关闭, 非0=点亮)
 *
 * @return  0 success
 *         -1 st7789t3_instance null
 *****************************************************************************/
static int8_t st7789t3_set_backlight(bsp_st7789t3_driver_t *p_st7789t3_instance,
									 uint8_t value)
{
	if (NULL == p_st7789t3_instance)
	{
		return -1;
	}

	p_st7789t3_instance->p_pwm_interface->pf_backlight_set(value);

	return 0;
}

/******************************************************************************
 * @name    st7789t3_set_window
 * @brief   设置显示窗口(坐标含端点), 内部自动应用面板偏移并发送RAMWR命令
 * @param   p_st7789t3_instance[in]
 * @param   x0[in] 起始列(含)
 * @param   y0[in] 起始行(含)
 * @param   x1[in] 结束列(含)
 * @param   y1[in] 结束行(含)
 *
 * @return  0 success
 *         -1 st7789t3_instance null
 *         -2 invalid coordinate
 *         -3 out of range
 *         -4 spi write error
 *****************************************************************************/
static int8_t st7789t3_set_window(bsp_st7789t3_driver_t *p_st7789t3_instance,
								  uint16_t x0, uint16_t y0,
								  uint16_t x1, uint16_t y1)
{
	uint8_t caset[4];
	uint8_t raset[4];
	uint16_t x_offset = 0;
	uint16_t y_offset = 0;

	if (NULL == p_st7789t3_instance)
	{
		return -1;
	}

	if (x0 > x1 || y0 > y1)
	{
		return -2;
	}

	if (x1 >= p_st7789t3_instance->width || y1 >= p_st7789t3_instance->height)
	{
		return -3;
	}

	/* 面板偏移: 竖屏偏移在Y方向, 横屏偏移在X方向 */
	if (st7789t3_dir_0 == p_st7789t3_instance->direction ||
		st7789t3_dir_180 == p_st7789t3_instance->direction)
	{
		y_offset = ST7789T3_OFFSET_Y;
	}
	else
	{
		x_offset = ST7789T3_OFFSET_Y;
	}

	x0 += x_offset;
	x1 += x_offset;
	y0 += y_offset;
	y1 += y_offset;

	/* 组装CASET: [XS_H, XS_L, XE_H, XE_L] */
	caset[0] = (uint8_t)(x0 >> 8);
	caset[1] = (uint8_t)(x0 & 0xFF);
	caset[2] = (uint8_t)(x1 >> 8);
	caset[3] = (uint8_t)(x1 & 0xFF);

	/* 组装RASET: [YS_H, YS_L, YE_H, YE_L] */
	raset[0] = (uint8_t)(y0 >> 8);
	raset[1] = (uint8_t)(y0 & 0xFF);
	raset[2] = (uint8_t)(y1 >> 8);
	raset[3] = (uint8_t)(y1 & 0xFF);

	/* CS保持低电平, 连续发送 CASET + RASET + RAMWR */
	st7789t3_cs_low(p_st7789t3_instance);

	if (0 != st7789t3_send_command(p_st7789t3_instance, ST7789T3_CASET) ||
		0 != st7789t3_send_data(p_st7789t3_instance, caset, 4) ||
		0 != st7789t3_send_command(p_st7789t3_instance, ST7789T3_RASET) ||
		0 != st7789t3_send_data(p_st7789t3_instance, raset, 4) ||
		0 != st7789t3_send_command(p_st7789t3_instance, ST7789T3_RAMWR))
	{
		st7789t3_cs_high(p_st7789t3_instance);
		return -4;
	}

	st7789t3_cs_high(p_st7789t3_instance);

	return 0;
}

/******************************************************************************
 * @name    st7789t3_write_pixels
 * @brief   写入像素数据(需先调用pf_set_window发送RAMWR)
 * @param   p_st7789t3_instance[in]
 * @param   pdata[in] RGB565像素缓冲区
 * @param   size[in] 像素个数
 *
 * @return  0 success
 *         -1 st7789t3_instance null
 *         -2 invalid data
 *         -3 spi write error
 *****************************************************************************/
static int8_t st7789t3_write_pixels(bsp_st7789t3_driver_t *p_st7789t3_instance,
									uint16_t *pdata, uint32_t size)
{
	if (NULL == p_st7789t3_instance)
	{
		return -1;
	}

	if (NULL == pdata || 0 == size)
	{
		return -2;
	}

	st7789t3_cs_low(p_st7789t3_instance);

	if (0 != st7789t3_send_pixels(p_st7789t3_instance, pdata, size))
	{
		st7789t3_cs_high(p_st7789t3_instance);
		return -3;
	}

	st7789t3_cs_high(p_st7789t3_instance);

	return 0;
}

/******************************************************************************
 * @name    st7789t3_fill
 * @brief   以纯色填充矩形区域(坐标含端点)
 * @param   p_st7789t3_instance[in]
 * @param   x0[in] 起始列(含)
 * @param   y0[in] 起始行(含)
 * @param   x1[in] 结束列(含)
 * @param   y1[in] 结束行(含)
 * @param   color[in] RGB565颜色
 *
 * @return  0 success
 *         -1 st7789t3_instance null
 *         -2 invalid coordinate
 *         -3 out of range
 *         -4 set window failed
 *         -5 spi write error
 *****************************************************************************/
static int8_t st7789t3_fill(bsp_st7789t3_driver_t *p_st7789t3_instance,
							uint16_t x0, uint16_t y0,
							uint16_t x1, uint16_t y1, uint16_t color)
{
	static uint8_t fill_buf[ST7789T3_PIXEL_CHUNK_SIZE * 2];
	uint8_t hi = (uint8_t)(color >> 8);
	uint8_t lo = (uint8_t)(color & 0xFF);
	uint32_t total;
	uint32_t i;

	if (NULL == p_st7789t3_instance)
	{
		return -1;
	}

	if (x0 > x1 || y0 > y1)
	{
		return -2;
	}

	if (x1 >= p_st7789t3_instance->width || y1 >= p_st7789t3_instance->height)
	{
		return -3;
	}

	/* 预交换RGB565为高字节先行字节流, 用重复模式填满DMA缓冲(省去逐像素字节交换) */
	for (i = 0; i < sizeof(fill_buf); i += 2)
	{
		fill_buf[i]     = hi;
		fill_buf[i + 1] = lo;
	}

	/* 设置窗口(含RAMWR) */
	if (0 != st7789t3_set_window(p_st7789t3_instance, x0, y0, x1, y1))
	{
		return -4;
	}

	total = (uint32_t)(x1 - x0 + 1) * (uint32_t)(y1 - y0 + 1);

	/* 保持CS低电平、DC为数据态分块DMA写入, 避免频繁拉取CS */
	st7789t3_cs_low(p_st7789t3_instance);
	p_st7789t3_instance->p_gpio_interface->pf_dc_set(1);

	while (total > 0)
	{
		uint32_t chunk_pixels = (total > ST7789T3_PIXEL_CHUNK_SIZE) ?
									ST7789T3_PIXEL_CHUNK_SIZE : total;

		if (0 != p_st7789t3_instance->p_spi_interface->pf_send_bytes_dma(
					fill_buf, chunk_pixels * 2))
		{
			st7789t3_cs_high(p_st7789t3_instance);
			return -5;
		}

		total -= chunk_pixels;
	}

	st7789t3_cs_high(p_st7789t3_instance);

	return 0;
}

/******************************************************************************
 * @name    st7789t3_clear
 * @brief   全屏清屏为纯色
 * @param   p_st7789t3_instance[in]
 * @param   color[in] RGB565颜色
 *
 * @return  0 success
 *         -1 st7789t3_instance null
 *         -2 fill failed
 *****************************************************************************/
static int8_t st7789t3_clear(bsp_st7789t3_driver_t *p_st7789t3_instance,
							 uint16_t color)
{
	if (NULL == p_st7789t3_instance)
	{
		return -1;
	}

	if (0 != st7789t3_fill(p_st7789t3_instance, 0, 0,
						   p_st7789t3_instance->width - 1,
						   p_st7789t3_instance->height - 1, color))
	{
		return -2;
	}

	return 0;
}

/******************************************************************************
 * @name    st7789t3_draw_point
 * @brief   在指定坐标绘制一个像素点
 * @param   p_st7789t3_instance[in]
 * @param   x[in] 列坐标
 * @param   y[in] 行坐标
 * @param   color[in] RGB565颜色
 *
 * @return  0 success
 *         -1 st7789t3_instance null
 *         -2 out of range
 *         -3 set window failed
 *         -4 write pixels failed
 *****************************************************************************/
static int8_t st7789t3_draw_point(bsp_st7789t3_driver_t *p_st7789t3_instance,
								  uint16_t x, uint16_t y, uint16_t color)
{
	if (NULL == p_st7789t3_instance)
	{
		return -1;
	}

	if (x >= p_st7789t3_instance->width || y >= p_st7789t3_instance->height)
	{
		return -2;
	}

	if (0 != st7789t3_set_window(p_st7789t3_instance, x, y, x, y))
	{
		return -3;
	}

	if (0 != st7789t3_write_pixels(p_st7789t3_instance, &color, 1))
	{
		return -4;
	}

	return 0;
}
