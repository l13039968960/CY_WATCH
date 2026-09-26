/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_adapter_st7789t3.c
 *
 * @par dependencies
 * - cywatch_adapter_st7789t3.h
 * - ../hal_driver/Inc/cywatch_bsp_st7789t3_driver.h
 * - ../../SPI/spi_hal.h
 * - main.h
 * - cmsis_os2.h
 *
 * @author	zw1194
 *
 * @brief LVGL 服务消费 ST7789T3 的 adapter 实现.
 *
 * Processing flow:
 *
 * 1. lvgl_bsp_st7789t3_inst(): 背光 PA1 GPIO → 填 SPI/GPIO/yield/delay/pwm
 *    五个接口 → st7789t3_inst()(含硬件复位与面板寄存器初始化);
 * 2. 之后每次画面刷新: lvgl_bsp_st7789t3_set_window() + _write_pixels();
 * 3. lvgl_bsp_st7789t3_deinst() 析构.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 总线归属: SPI1(PA5=SCK/PA7=MOSI + DMA2_Stream3)实例 `lcd_spi_instance` 由
 *       应用层 main.c 创建, 本 adapter 只 extern 引用(与 W25Q64 adapter 引用
 *       `spi2_instance` 同一写法), 不自己再建总线.
 *
 * @note `lvgl_bsp_st7789t3_inst()` 内含 osDelay(复位/上电等待) → 必须在
 *       osKernelStart() 之后的**任务上下文**调用, 不能放在 main 启动内核之前.
 *
 * @note 中断纪律: SPI 的 DMA 完成回调在 SPI/spi_hal.c 的 HAL_SPI_TxCpltCallback
 *       (ISR)内释放信号量, 本层不含任何 ISR 代码; 阻塞等待在
 *       spi_driver.pf_transmit_dma 内部完成.
 ******************************************************************************/
#include "cywatch_adapter_st7789t3.h"

#include "../hal_driver/Inc/cywatch_bsp_st7789t3_driver.h"
#include "../../SPI/spi_hal.h" /* spi_driver_t: 硬件 SPI1 总线, 由 main.c 提供 */
#include "main.h"              /* LCD_DC_Pin / LCD_CS_Pin / LCD_RST_Pin / LCD_PWR_Pin */
#include "cmsis_os2.h"         /* osDelay */

/***********************************Defines************************************/
/* 背光: PA1 为开/关型, 不做 PWM 调光(与 main.h 的 LCD_PWR_Pin 一致) */
#define ST7789T3_BL_PORT  LCD_PWR_GPIO_Port
#define ST7789T3_BL_PIN   LCD_PWR_Pin
/***********************************Defines************************************/

/**********************************Declaring***********************************/
static bsp_st7789t3_driver_t st7789t3_instance;

/* 应用层(main.c)提供的总线实例: LCD 独占 SPI1 + DMA2_Stream3 */
extern spi_driver_t lcd_spi_instance;

/* ============================= 引脚描述符 ============================= */
/* DC/CS/RST 三个引脚共用同一个写回调, 故各自带一个描述符作为不透明实例传入
   (st7789t3_gpio_interface_t 为三根线各留了一个 p_gpio_xxx_instance) */
typedef struct
{
	GPIO_TypeDef *p_port;
	uint16_t pin;
} st7789t3_pin_t;

static st7789t3_pin_t s_lcd_dc_pin  = { LCD_DC_GPIO_Port,  LCD_DC_Pin  };
static st7789t3_pin_t s_lcd_cs_pin  = { LCD_CS_GPIO_Port,  LCD_CS_Pin  };
static st7789t3_pin_t s_lcd_rst_pin = { LCD_RST_GPIO_Port, LCD_RST_Pin };
static st7789t3_pin_t s_lcd_bl_pin  = { ST7789T3_BL_PORT,  ST7789T3_BL_PIN };

static st7789t3_spi_interface_t         st7789t3_spi_interface_instance;
static st7789t3_gpio_interface_t        st7789t3_gpio_interface_instance;
static st7789t3_yield_interface_t       st7789t3_yield_instance;
static st7789t3_delay_interface_t       st7789t3_delay_instance;
static st7789t3_pwm_interface_t         st7789t3_pwm_interface_instance;

static void yield(void);

/******************************************************************************
 * @name    lvgl_bsp_st7789t3_gpio_set
 * @brief   DC/CS/RST 通用GPIO写(0=低电平, 非0=高电平)
 * @param   p_pin[in] 引脚描述符(st7789t3_pin_t*, 由接口结构体传入)
 * @param   level[in] 电平
 *
 * @return  0 success
 *         -1 pin instance null
 *****************************************************************************/
static int8_t lvgl_bsp_st7789t3_gpio_set(void *p_pin, uint8_t level)
{
	st7789t3_pin_t *p = (st7789t3_pin_t *)p_pin;

	if (NULL == p)
	{
		return -1;
	}

	HAL_GPIO_WritePin(p->p_port, p->pin,
					  (0 == level) ? GPIO_PIN_RESET : GPIO_PIN_SET);

	return 0;
}

/******************************************************************************
 * @name    lvgl_bsp_st7789t3_backlight_set
 * @brief   背光写(0=熄灭, 非0=点亮); PA1 为开/关型, 不做PWM调光
 * @param   p_pin[in] 引脚描述符(st7789t3_pin_t*)
 * @param   level[in] 背光值
 *
 * @return  无
 *****************************************************************************/
static void lvgl_bsp_st7789t3_backlight_set(void *p_pin, uint8_t level)
{
	st7789t3_pin_t *p = (st7789t3_pin_t *)p_pin;

	if (NULL == p)
	{
		return;
	}

	HAL_GPIO_WritePin(p->p_port, p->pin,
					  (0 == level) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/******************************************************************************
 * @name    yield
 * @brief   让出CPU接口的空实现(驱动要求非空, 本工程由 RTOS 调度, 无需主动让出)
 * @param   无
 *
 * @return  无
 *****************************************************************************/
static void yield(void)
{
}

/******************************************************************************
 * @name    lvgl_bsp_st7789t3_delay_cb
 * @brief   延时: osDelay 返回 osStatus_t, 驱动要求 void(*)(uint32_t), 包一层丢弃返回值
 * @param   ms[in] 毫秒
 *
 * @return  无
 *
 * @note    依赖调度器运行 —— inst 内含复位等待, 必须在任务上下文调用
 *****************************************************************************/
static void lvgl_bsp_st7789t3_delay_cb(uint32_t ms)
{
	(void)osDelay(ms);
}

/* ============================= 驱动接口转发 ============================= */
/* 驱动接口**带**实例参数 `p_spi_instance`(与 st7789t3_spi_interface_t 签名一致),
   而 SPI 总线实例(lcd_spi_instance)由 main.c 创建、本层 extern 引用: 本适配器
   只服务这一块 LCD, 故收到的 p_spi_instance 恒等于 &lcd_spi_instance, 一律忽略
   形参、直接用 extern 的实例转发.
   @note 一处易错点: 别照抄"无实例参数"的转发形态(如 MPU6050 adapter), 函数指针
         类型不匹配会直接 -Wincompatible-function-pointer-types 编译失败 */

/******************************************************************************
 * @name    st7789t3_spi_send_bytes
 * @brief   阻塞发送字节流(命令/参数)
 * @param   p_spi_instance[in] SPI实例(恒为 lcd_spi_instance, 忽略)
 * @param   pdata[in] 数据
 * @param   size[in]  字节数
 *
 * @return  0 success
 *         -1 spi instance null
 *         -2 busy / -3 timeout (见 spi_hal.c)
 *****************************************************************************/
static int8_t st7789t3_spi_send_bytes(void *p_spi_instance,
									  uint8_t *pdata,
									  uint32_t size)
{
	(void)p_spi_instance;

	return lcd_spi_instance.pf_transmit(&lcd_spi_instance, pdata, size);
}

/******************************************************************************
 * @name    st7789t3_spi_send_bytes_dma
 * @brief   DMA发送字节流(像素数据, 启动+等待合并, 阻塞至完成或超时)
 * @param   p_spi_instance[in] SPI实例(恒为 lcd_spi_instance, 忽略)
 * @param   pdata[in] 数据
 * @param   size[in]  字节数
 *
 * @return  0 success
 *         -1 spi instance null
 *         -2 hdmatx null / -3 等待接口 null / -4 启动失败 / -5 等待超时
 *****************************************************************************/
static int8_t st7789t3_spi_send_bytes_dma(void *p_spi_instance,
										  uint8_t *pdata,
										  uint32_t size)
{
	(void)p_spi_instance;

	return lcd_spi_instance.pf_transmit_dma(&lcd_spi_instance, pdata, size);
}

/* ============================= 构造/析构 ============================= */

/******************************************************************************
 * @name    lvgl_bsp_st7789t3_inst
 * @brief   构造ST7789T3实例: 背光GPIO → 挂五个驱动接口 → st7789t3_inst(含面板初始化)
 * @param   无
 *
 * @return  0 success
 *         -1 背光GPIO初始化失败
 *         -2 st7789t3_inst 失败(负值语义见驱动头文件 @return, -7 = 面板初始化失败)
 *
 * @note    须在 osKernelStart() 之后的任务上下文调用(内部 osDelay);
 *          SPI1 的 PA5/PA7 与 DMA2_Stream3 由 spi_hal 的 HAL_SPI_MspInit 配置,
 *          调用方不要再调 MX_SPI1_Init
 *****************************************************************************/
int8_t lvgl_bsp_st7789t3_inst(void)
{
	int8_t ret = 0;
	GPIO_InitTypeDef gpio = { 0 };

	/* 1. 背光引脚(PA1): MX_GPIO_Init 未纳管此脚, 先初始化为输出并熄灭 */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	gpio.Pin   = ST7789T3_BL_PIN;
	gpio.Mode  = GPIO_MODE_OUTPUT_PP;
	gpio.Pull  = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ST7789T3_BL_PORT, &gpio);
	HAL_GPIO_WritePin(ST7789T3_BL_PORT, ST7789T3_BL_PIN, GPIO_PIN_RESET);

	/* 2. 挂SPI接口(转发到 main.c 的 lcd_spi_instance) */
	st7789t3_spi_interface_instance.p_spi_instance    = (void *)&lcd_spi_instance;
	st7789t3_spi_interface_instance.pf_send_bytes     = st7789t3_spi_send_bytes;
	st7789t3_spi_interface_instance.pf_send_bytes_dma = st7789t3_spi_send_bytes_dma;

	/* 3. 挂GPIO接口(DC/CS/RST 三根线共用同一写回调) */
	st7789t3_gpio_interface_instance.p_gpio_dc_instance  = (void *)&s_lcd_dc_pin;
	st7789t3_gpio_interface_instance.p_gpio_cs_instance  = (void *)&s_lcd_cs_pin;
	st7789t3_gpio_interface_instance.p_gpio_rst_instance = (void *)&s_lcd_rst_pin;
	st7789t3_gpio_interface_instance.pf_dc_set           = lvgl_bsp_st7789t3_gpio_set;
	st7789t3_gpio_interface_instance.pf_cs_set           = lvgl_bsp_st7789t3_gpio_set;
	st7789t3_gpio_interface_instance.pf_rst_set          = lvgl_bsp_st7789t3_gpio_set;

	/* 4. 挂yield/延时/背光接口 */
	st7789t3_yield_instance.pf_yield = yield;

	st7789t3_delay_instance.pf_delay = lvgl_bsp_st7789t3_delay_cb;

	st7789t3_pwm_interface_instance.p_pwm_instance     = (void *)&s_lcd_bl_pin;
	st7789t3_pwm_interface_instance.pf_backlight_set   = lvgl_bsp_st7789t3_backlight_set;

	/* 5. 构造驱动: 硬件复位 → SLPOUT/COLMOD/MADCTL/Gamma/INVON/DISPON(默认竖屏) */
	ret = st7789t3_inst(&st7789t3_instance,
						&st7789t3_spi_interface_instance,
						&st7789t3_gpio_interface_instance,
						&st7789t3_yield_instance,
						&st7789t3_delay_instance,
						&st7789t3_pwm_interface_instance);
	if (0 != ret)
	{
		return -2;
	}

	return 0;
}

/******************************************************************************
 * @name    lvgl_bsp_st7789t3_deinst
 * @brief   析构ST7789T3实例
 * @param   无
 *
 * @return  0 success
 *         -1 驱动返回的实例空指针错误
 *****************************************************************************/
int8_t lvgl_bsp_st7789t3_deinst(void)
{
	return st7789t3_instance.pf_deinst(&st7789t3_instance);
}

/* ============================= 能力转发 ============================= */

/******************************************************************************
 * @name    lvgl_bsp_st7789t3_set_direction
 * @brief   设置显示方向(内部转换为驱动枚举)
 * @param   direction[in] 服务级方向枚举
 *
 * @return  0 success
 *         -1 st7789t3_instance null
 *         -2 方向非法 (见驱动 @return)
 *****************************************************************************/
int8_t lvgl_bsp_st7789t3_set_direction(lvgl_bsp_st7789t3_dir_t direction)
{
	return st7789t3_instance.pf_set_direction(&st7789t3_instance,
											  (st7789t3_direction_t)direction);
}

/******************************************************************************
 * @name    lvgl_bsp_st7789t3_display_on
 * @brief   打开显示
 * @param   无
 *
 * @return  0 success / -1 instance null / -2 spi error (见驱动 @return)
 *****************************************************************************/
int8_t lvgl_bsp_st7789t3_display_on(void)
{
	return st7789t3_instance.pf_display_on(&st7789t3_instance);
}

/******************************************************************************
 * @name    lvgl_bsp_st7789t3_display_off
 * @brief   关闭显示
 * @param   无
 *
 * @return  0 success / -1 instance null / -2 spi error (见驱动 @return)
 *****************************************************************************/
int8_t lvgl_bsp_st7789t3_display_off(void)
{
	return st7789t3_instance.pf_display_off(&st7789t3_instance);
}

/******************************************************************************
 * @name    lvgl_bsp_st7789t3_sleep
 * @brief   进入睡眠(显示关闭 + SLPIN)
 * @param   无
 *
 * @return  0 success / -1 instance null / -2 spi error (见驱动 @return)
 *****************************************************************************/
int8_t lvgl_bsp_st7789t3_sleep(void)
{
	return st7789t3_instance.pf_sleep(&st7789t3_instance);
}

/******************************************************************************
 * @name    lvgl_bsp_st7789t3_wakeup
 * @brief   唤醒(SLPOUT + 打开显示)
 * @param   无
 *
 * @return  0 success / -1 instance null / -2 spi error (见驱动 @return)
 *****************************************************************************/
int8_t lvgl_bsp_st7789t3_wakeup(void)
{
	return st7789t3_instance.pf_wakeup(&st7789t3_instance);
}

/******************************************************************************
 * @name    lvgl_bsp_st7789t3_set_backlight
 * @brief   背光开关
 * @param   value[in] 0=熄灭, 非0=点亮
 *
 * @return  0 success / -1 instance null (见驱动 @return)
 *****************************************************************************/
int8_t lvgl_bsp_st7789t3_set_backlight(uint8_t value)
{
	return st7789t3_instance.pf_set_backlight(&st7789t3_instance, value);
}

/******************************************************************************
 * @name    lvgl_bsp_st7789t3_set_window
 * @brief   设置绘制窗口(坐标含端点, 驱动内部处理面板偏移)
 * @param   x0[in] 左上X
 * @param   y0[in] 左上Y
 * @param   x1[in] 右下X
 * @param   y1[in] 右下Y
 *
 * @return  0 success / -1 instance null / -2.. 见驱动 @return
 *****************************************************************************/
int8_t lvgl_bsp_st7789t3_set_window(uint16_t x0, uint16_t y0,
									uint16_t x1, uint16_t y1)
{
	return st7789t3_instance.pf_set_window(&st7789t3_instance, x0, y0, x1, y1);
}

/******************************************************************************
 * @name    lvgl_bsp_st7789t3_write_pixels
 * @brief   向已设窗口写入RGB565像素(驱动内部做高字节先行交换与分块DMA)
 * @param   pdata[in] 像素数据(小端native存储)
 * @param   size[in]  像素个数
 *
 * @return  0 success / -1 instance null / -2.. 见驱动 @return
 *****************************************************************************/
int8_t lvgl_bsp_st7789t3_write_pixels(uint16_t *pdata, uint32_t size)
{
	return st7789t3_instance.pf_write_pixels(&st7789t3_instance, pdata, size);
}

/******************************************************************************
 * @name    lvgl_bsp_st7789t3_fill
 * @brief   用单色填充矩形
 * @param   x0[in] 左上X
 * @param   y0[in] 左上Y
 * @param   x1[in] 右下X
 * @param   y1[in] 右下Y
 * @param   color[in] RGB565颜色
 *
 * @return  0 success / -1 instance null / -2.. 见驱动 @return
 *****************************************************************************/
int8_t lvgl_bsp_st7789t3_fill(uint16_t x0, uint16_t y0,
							  uint16_t x1, uint16_t y1, uint16_t color)
{
	return st7789t3_instance.pf_fill(&st7789t3_instance, x0, y0, x1, y1, color);
}

/******************************************************************************
 * @name    lvgl_bsp_st7789t3_clear
 * @brief   整屏清屏
 * @param   color[in] RGB565颜色
 *
 * @return  0 success / -1 instance null / -2.. 见驱动 @return
 *****************************************************************************/
int8_t lvgl_bsp_st7789t3_clear(uint16_t color)
{
	return st7789t3_instance.pf_clear(&st7789t3_instance, color);
}

/******************************************************************************
 * @name    lvgl_bsp_st7789t3_draw_point
 * @brief   画单点
 * @param   x[in] X坐标
 * @param   y[in] Y坐标
 * @param   color[in] RGB565颜色
 *
 * @return  0 success / -1 instance null / -2.. 见驱动 @return
 *****************************************************************************/
int8_t lvgl_bsp_st7789t3_draw_point(uint16_t x, uint16_t y, uint16_t color)
{
	return st7789t3_instance.pf_draw_point(&st7789t3_instance, x, y, color);
}

/******************************************************************************
 * @name    lvgl_bsp_st7789t3_get_width
 * @brief   读取当前逻辑宽度(随显示方向变化)
 * @param   无
 *
 * @return  当前宽度(实例未构造时为0)
 *****************************************************************************/
uint16_t lvgl_bsp_st7789t3_get_width(void)
{
	return st7789t3_instance.width;
}

/******************************************************************************
 * @name    lvgl_bsp_st7789t3_get_height
 * @brief   读取当前逻辑高度(随显示方向变化)
 * @param   无
 *
 * @return  当前高度(实例未构造时为0)
 *****************************************************************************/
uint16_t lvgl_bsp_st7789t3_get_height(void)
{
	return st7789t3_instance.height;
}
