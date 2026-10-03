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
 * 1. lvgl_bsp_disp_inst(): 填 SPI/GPIO/delay/pwm 四个接口(每个接口都带 pf_init/
 *    pf_deinit, 四根控制脚 DC/CS/RST/背光 的 GPIO 配置在 gpio 的 pf_init 里)
 *    → st7789t3_inst()(内部依次回调各外设 init, 再做硬件复位与面板寄存器初始化);
 * 2. 之后每次画面刷新: lvgl_bsp_disp_set_window() + _write_pixels();
 * 3. lvgl_bsp_disp_deinst() 析构.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 总线归属: SPI1(PA5=SCK/PA7=MOSI + DMA2_Stream3)实例 `lcd_spi_instance` 由
 *       应用层 main.c 创建, 本 adapter 只 extern 引用(与 W25Q64 adapter 引用
 *       `spi2_instance` 同一写法), 不自己再建总线.
 *
 * @note `lvgl_bsp_disp_inst()` 内含 osDelay(复位/上电等待) → 必须在
 *       osKernelStart() 之后的**任务上下文**调用, 不能放在 main 启动内核之前.
 *
 * @note 中断纪律: SPI 的 DMA 完成回调在 SPI/spi_hal.c 的 HAL_SPI_TxCpltCallback
 *       (ISR)内释放信号量, 本层不含任何 ISR 代码; 阻塞等待在
 *       spi_driver.pf_transmit_dma 内部完成.
 ******************************************************************************/
#include "cywatch_adapter_disp.h"

#include "cywatch_bsp_st7789t3_driver.h"
#include "spi_hal.h" /* spi_driver_t: 硬件 SPI1 总线, 由 main.c 提供 */
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

/* ============================= 引脚配置 ============================= */
/* DC/CS/RST/背光 每根线一个 gpio 实例 —— 必须各自独立: 合成一个掩码写就变成
   三根线同时翻转. 速度取 LOW: 它们只在命令间/帧间翻转, 不跟 SCK 的边沿 */
static gpio_cfg_t s_lcd_dc_cfg =
{
	.p_port = LCD_DC_GPIO_Port,  .pins = LCD_DC_Pin,
	.mode   = GPIO_MODE_OUTPUT_PP, .pull = GPIO_NOPULL, .speed = GPIO_SPEED_FREQ_LOW,
};
static gpio_cfg_t s_lcd_cs_cfg =
{
	.p_port = LCD_CS_GPIO_Port,  .pins = LCD_CS_Pin,
	.mode   = GPIO_MODE_OUTPUT_PP, .pull = GPIO_NOPULL, .speed = GPIO_SPEED_FREQ_LOW,
};
static gpio_cfg_t s_lcd_rst_cfg =
{
	.p_port = LCD_RST_GPIO_Port, .pins = LCD_RST_Pin,
	.mode   = GPIO_MODE_OUTPUT_PP, .pull = GPIO_NOPULL, .speed = GPIO_SPEED_FREQ_LOW,
};
static gpio_cfg_t s_lcd_bl_cfg =
{
	.p_port = ST7789T3_BL_PORT,  .pins = ST7789T3_BL_PIN,
	.mode   = GPIO_MODE_OUTPUT_PP, .pull = GPIO_NOPULL, .speed = GPIO_SPEED_FREQ_LOW,
};

static gpio_driver_t s_lcd_dc_pin;
static gpio_driver_t s_lcd_cs_pin;
static gpio_driver_t s_lcd_rst_pin;
static gpio_driver_t s_lcd_bl_pin;

static st7789t3_spi_interface_t         st7789t3_spi_interface_instance;
static st7789t3_gpio_interface_t        st7789t3_gpio_interface_instance;
static st7789t3_delay_interface_t       st7789t3_delay_instance;
static st7789t3_pwm_interface_t         st7789t3_pwm_interface_instance;

/******************************************************************************
 * @name    lvgl_bsp_disp_pin_write
 * @brief   引脚写(0=低电平, 非0=高电平)
 * @param   p_pin[in] 引脚 gpio 实例
 * @param   level[in] 电平
 *
 * @return  无
 *****************************************************************************/
static void lvgl_bsp_disp_pin_write(gpio_driver_t *p_pin, uint8_t level)
{
	if (NULL == p_pin)
	{
		return;
	}

	(void)p_pin->pf_write(p_pin, (0U == level) ? 0U : 1U);
}

/******************************************************************************
 * @name    lvgl_bsp_disp_dc_set
 * @brief   DC(数据/命令选择)写: 0=命令, 非0=数据
 * @param   level[in] 电平
 *
 * @return  0 success
 *****************************************************************************/
static int8_t lvgl_bsp_disp_dc_set(uint8_t level)
{
	lvgl_bsp_disp_pin_write(&s_lcd_dc_pin, level);

	return 0;
}

/******************************************************************************
 * @name    lvgl_bsp_disp_cs_set
 * @brief   CS(片选)写: 0=选中, 非0=释放
 * @param   level[in] 电平
 *
 * @return  0 success
 *****************************************************************************/
static int8_t lvgl_bsp_disp_cs_set(uint8_t level)
{
	lvgl_bsp_disp_pin_write(&s_lcd_cs_pin, level);

	return 0;
}

/******************************************************************************
 * @name    lvgl_bsp_disp_rst_set
 * @brief   RST(硬件复位)写: 0=复位, 非0=正常运行
 * @param   level[in] 电平
 *
 * @return  0 success
 *****************************************************************************/
static int8_t lvgl_bsp_disp_rst_set(uint8_t level)
{
	lvgl_bsp_disp_pin_write(&s_lcd_rst_pin, level);

	return 0;
}

/******************************************************************************
 * @name    lvgl_bsp_disp_backlight_set
 * @brief   背光写(0=熄灭, 非0=点亮); PA1 为开/关型, 不做PWM调光
 * @param   level[in] 背光值
 *
 * @return  无
 *****************************************************************************/
static void lvgl_bsp_disp_backlight_set(uint8_t level)
{
	lvgl_bsp_disp_pin_write(&s_lcd_bl_pin, level);
}

/******************************************************************************
 * @name    lvgl_bsp_disp_gpio_init
 * @brief   四根控制脚(DC/CS/RST/背光)配置到运行态
 *
 * @return  0 success
 *
 * @note    四根控制脚(DC=PA6 / CS=PA4 / RST=PC13 / 背光=PA1)全由本层自配, 不上交
 *          main.c: 与触摸 adapter 自持 RST(PA15)/INT(PB2) 是同一做法。
 *          @note 本工程**没有** MX_GPIO_Init —— 参考工程 Dirver_Test/Core/Src/gpio.c
 *                里那个函数在迁移时没搬进来, main.c 的调用点("Initialize all
 *                configured peripherals" 下面)也是空的。所以这四根线除了这里, 没有
 *                任何地方会配它们。漏配的后果: 引脚停在复位默认的**输入浮空**态
 *                (PA4/PA6 在 GPIOA, MODER 复位值 0xA8000000; PC13 在 GPIOC, MODER
 *                复位值 0), 此时 HAL_GPIO_WritePin 只写 BSRR/ODR, 输出驱动级是断开
 *                的 —— 电平等同于没写。于是 CS 被模块自身上拉拉高 → 面板忽略全部
 *                SPI 流量, RST 复位脉冲也发不出去 → 整屏不亮, 而串口日志一切正常
 *                (驱动只发不收, 没有回读校验)。
 *          @note PC13 在 GPIOC: 端口时钟由 gpio_hal 在自己的 pf_init 里开。
 *          @note 先写 ODR 再配 MODER(与参考工程同序): 输出锁存器预置成空闲电平,
 *                引脚一变成输出就立刻是 RST/CS/DC 的高电平, 不会产生一次假复位/
 *                假片选。
 *          @note 驱动在 st7789t3_init 里回调本函数, 早于复位脉冲与任何SPI流量。
 *****************************************************************************/
static int8_t lvgl_bsp_disp_gpio_init(void)
{
	(void)s_lcd_rst_pin.pf_write(&s_lcd_rst_pin, 1U);
	(void)s_lcd_cs_pin.pf_write(&s_lcd_cs_pin, 1U);
	(void)s_lcd_dc_pin.pf_write(&s_lcd_dc_pin, 1U);
	(void)s_lcd_bl_pin.pf_write(&s_lcd_bl_pin, 0U);

	(void)s_lcd_rst_pin.pf_init(&s_lcd_rst_pin);
	(void)s_lcd_cs_pin.pf_init(&s_lcd_cs_pin);
	(void)s_lcd_dc_pin.pf_init(&s_lcd_dc_pin);

	return s_lcd_bl_pin.pf_init(&s_lcd_bl_pin);
}

/******************************************************************************
 * @name    lvgl_bsp_disp_gpio_deinit
 * @brief   释放四根控制脚, 置低功耗态模拟输入
 *
 * @return  0 success
 *
 * @note    不动端口时钟: GPIOA 上还挂着 SPI1 的 PA5/PA7 与 I2C 的 PA8,
 *          GPIOC 也只有 PC13 这一根线, 关时钟会误伤。
 *****************************************************************************/
static int8_t lvgl_bsp_disp_gpio_deinit(void)
{
	(void)s_lcd_rst_pin.pf_deinit(&s_lcd_rst_pin);
	(void)s_lcd_cs_pin.pf_deinit(&s_lcd_cs_pin);
	(void)s_lcd_dc_pin.pf_deinit(&s_lcd_dc_pin);

	return s_lcd_bl_pin.pf_deinit(&s_lcd_bl_pin);
}

/******************************************************************************
 * @name    lvgl_bsp_disp_pwm_init
 * @brief   背光调光外设初始化
 *
 * @return  0 success
 *
 * @note    背光是 PA1 的开关型 GPIO(归 gpio 接口的 s_lcd_bl_pin 管), 当前没有
 *          PWM 外设, 这里先做空实现占位; 以后真上定时器调光时在此配置。
 *****************************************************************************/
static int8_t lvgl_bsp_disp_pwm_init(void)
{
	return 0;
}

/******************************************************************************
 * @name    lvgl_bsp_disp_pwm_deinit
 * @brief   背光调光外设反初始化
 *
 * @return  0 success
 *
 * @note    同 pf_init: 当前无 PWM 外设, 空实现占位。
 *****************************************************************************/
static int8_t lvgl_bsp_disp_pwm_deinit(void)
{
	return 0;
}

/******************************************************************************
 * @name    lvgl_bsp_disp_delay_cb
 * @brief   延时: osDelay 返回 osStatus_t, 驱动要求 void(*)(uint32_t), 包一层丢弃返回值
 * @param   ms[in] 毫秒
 *
 * @return  无
 *
 * @note    依赖调度器运行 —— inst 内含复位等待, 必须在任务上下文调用
 *****************************************************************************/
static void lvgl_bsp_disp_delay_cb(uint32_t ms)
{
	(void)osDelay(ms);
}

/* ============================= 驱动接口转发 ============================= */
/* 驱动接口一律**不带**实例参数(见驱动头文件), 而 SPI 总线实例(lcd_spi_instance)
   由 main.c 创建、本层 extern 引用: 本适配器只服务这一块 LCD, 转发函数直接用
   extern 的实例调 spi_hal 的 pf_*(该实例正是 spi_hal 强类型接口的第一个实参).
   @note 一处易错点: 形参表必须与接口结构体逐字一致 —— 多写一个 void * 就会
         -Wincompatible-function-pointer-types 编译失败(本工程里这是错误不是警告) */

/******************************************************************************
 * @name    st7789t3_spi_init
 * @brief   SPI总线初始化: 转发到 main.c 的 lcd_spi_instance
 *
 * @return  0 success
 *         -1 spi instance null
 *         -2 spi init error (见 spi_hal.c)
 *
 * @note    驱动在 st7789t3_init 里回调本函数。spi_hal 那侧带引用计数, 若总线
 *          已被别的使用者init过, 这里只累加计数、不会把外设重配一遍。
 *****************************************************************************/
static int8_t st7789t3_spi_init(void)
{
	return lcd_spi_instance.pf_init(&lcd_spi_instance);
}

/******************************************************************************
 * @name    st7789t3_spi_deinit
 * @brief   SPI总线反初始化: 转发到 main.c 的 lcd_spi_instance
 *
 * @return  0 success
 *         -1 spi instance null (见 spi_hal.c)
 *****************************************************************************/
static int8_t st7789t3_spi_deinit(void)
{
	return lcd_spi_instance.pf_deinit(&lcd_spi_instance);
}

/******************************************************************************
 * @name    st7789t3_spi_send_bytes
 * @brief   阻塞发送字节流(命令/参数)
 * @param   pdata[in] 数据
 * @param   size[in]  字节数
 *
 * @return  0 success
 *         -1 spi instance null
 *         -2 busy / -3 timeout (见 spi_hal.c)
 *****************************************************************************/
static int8_t st7789t3_spi_send_bytes(uint8_t *pdata, uint32_t size)
{
	return lcd_spi_instance.pf_transmit(&lcd_spi_instance, pdata, size);
}

/******************************************************************************
 * @name    st7789t3_spi_send_bytes_dma
 * @brief   DMA发送字节流(像素数据, 启动+等待合并, 阻塞至完成或超时)
 * @param   pdata[in] 数据
 * @param   size[in]  字节数
 *
 * @return  0 success
 *         -1 spi instance null
 *         -2 hdmatx null / -3 等待接口 null / -4 启动失败 / -5 等待超时
 *****************************************************************************/
static int8_t st7789t3_spi_send_bytes_dma(uint8_t *pdata, uint32_t size)
{
	return lcd_spi_instance.pf_transmit_dma(&lcd_spi_instance, pdata, size);
}

/* ============================= 构造/析构 ============================= */

/******************************************************************************
 * @name    lvgl_bsp_disp_inst
 * @brief   构造ST7789T3实例: 四根控制脚GPIO → 挂四个驱动接口 → st7789t3_inst(含面板初始化)
 * @param   无
 *
 * @return  0 success
 *         -2 st7789t3_inst 失败(负值语义见驱动头文件 @return, -6 = 面板初始化失败)
 *
 * @note    须在 osKernelStart() 之后的任务上下文调用(内部 osDelay);
 *          SPI1 的时钟/PA5/PA7/DMA2_Stream3 都由 spi_hal 的 pf_init 配, 调用方不要再动
 *****************************************************************************/
int8_t lvgl_bsp_disp_inst(void)
{
	int8_t ret = 0;

	/* 0. 装配四根控制脚各自的 gpio 实例(不碰硬件, 真正配置在 gpio 的 pf_init 里) */
	(void)gpio_driver_inst(&s_lcd_dc_pin,  &s_lcd_dc_cfg);
	(void)gpio_driver_inst(&s_lcd_cs_pin,  &s_lcd_cs_cfg);
	(void)gpio_driver_inst(&s_lcd_rst_pin, &s_lcd_rst_cfg);
	(void)gpio_driver_inst(&s_lcd_bl_pin,  &s_lcd_bl_cfg);

	/* 1. 挂SPI接口(转发到 main.c 的 lcd_spi_instance)
	      四根控制脚的 GPIO 配置不在这里做, 已搬进 lvgl_bsp_disp_gpio_init, 由驱动在
	      st7789t3_init 里回调 —— 见那里的 @note */
	st7789t3_spi_interface_instance.pf_init           = st7789t3_spi_init;
	st7789t3_spi_interface_instance.pf_deinit         = st7789t3_spi_deinit;
	st7789t3_spi_interface_instance.pf_send_bytes     = st7789t3_spi_send_bytes;
	st7789t3_spi_interface_instance.pf_send_bytes_dma = st7789t3_spi_send_bytes_dma;

	/* 2. 挂GPIO接口(DC/CS/RST 各自一根线、各自一个回调) */
	st7789t3_gpio_interface_instance.pf_init    = lvgl_bsp_disp_gpio_init;
	st7789t3_gpio_interface_instance.pf_deinit  = lvgl_bsp_disp_gpio_deinit;
	st7789t3_gpio_interface_instance.pf_dc_set  = lvgl_bsp_disp_dc_set;
	st7789t3_gpio_interface_instance.pf_cs_set  = lvgl_bsp_disp_cs_set;
	st7789t3_gpio_interface_instance.pf_rst_set = lvgl_bsp_disp_rst_set;

	/* 3. 挂延时/PWM接口(yield 接口已随驱动头文件一并删除: 它此前只被非空校验) */
	st7789t3_delay_instance.pf_delay = lvgl_bsp_disp_delay_cb;

	st7789t3_pwm_interface_instance.pf_init     = lvgl_bsp_disp_pwm_init;
	st7789t3_pwm_interface_instance.pf_deinit   = lvgl_bsp_disp_pwm_deinit;
	st7789t3_pwm_interface_instance.pf_pwm_set  = lvgl_bsp_disp_backlight_set;

	/* 4. 构造驱动: 各外设 init → 硬件复位 → SLPOUT/COLMOD/MADCTL/Gamma/INVON/DISPON(默认竖屏) */
	ret = st7789t3_inst(&st7789t3_instance,
						&st7789t3_spi_interface_instance,
						&st7789t3_gpio_interface_instance,
						&st7789t3_delay_instance,
						&st7789t3_pwm_interface_instance);
	if (0 != ret)
	{
		return -2;
	}

	return 0;
}

/******************************************************************************
 * @name    lvgl_bsp_disp_deinst
 * @brief   析构ST7789T3实例
 * @param   无
 *
 * @return  0 success
 *         -1 驱动返回的实例空指针错误
 *****************************************************************************/
int8_t lvgl_bsp_disp_deinst(void)
{
	return st7789t3_instance.pf_deinst(&st7789t3_instance);
}

/* ============================= 能力转发 ============================= */

/******************************************************************************
 * @name    lvgl_bsp_disp_set_direction
 * @brief   设置显示方向(内部转换为驱动枚举)
 * @param   direction[in] 服务级方向枚举
 *
 * @return  0 success
 *         -1 st7789t3_instance null
 *         -2 方向非法 (见驱动 @return)
 *****************************************************************************/
int8_t lvgl_bsp_disp_set_direction(lvgl_bsp_disp_dir_t direction)
{
	return st7789t3_instance.pf_set_direction(&st7789t3_instance,
											  (st7789t3_direction_t)direction);
}

/******************************************************************************
 * @name    lvgl_bsp_disp_display_on
 * @brief   打开显示
 * @param   无
 *
 * @return  0 success / -1 instance null / -2 spi error (见驱动 @return)
 *****************************************************************************/
int8_t lvgl_bsp_disp_display_on(void)
{
	return st7789t3_instance.pf_display_on(&st7789t3_instance);
}

/******************************************************************************
 * @name    lvgl_bsp_disp_display_off
 * @brief   关闭显示
 * @param   无
 *
 * @return  0 success / -1 instance null / -2 spi error (见驱动 @return)
 *****************************************************************************/
int8_t lvgl_bsp_disp_display_off(void)
{
	return st7789t3_instance.pf_display_off(&st7789t3_instance);
}

/******************************************************************************
 * @name    lvgl_bsp_disp_sleep
 * @brief   进入睡眠(显示关闭 + SLPIN)
 * @param   无
 *
 * @return  0 success / -1 instance null / -2 spi error (见驱动 @return)
 *****************************************************************************/
int8_t lvgl_bsp_disp_sleep(void)
{
	return st7789t3_instance.pf_sleep(&st7789t3_instance);
}

/******************************************************************************
 * @name    lvgl_bsp_disp_wakeup
 * @brief   唤醒(SLPOUT + 打开显示)
 * @param   无
 *
 * @return  0 success / -1 instance null / -2 spi error (见驱动 @return)
 *****************************************************************************/
int8_t lvgl_bsp_disp_wakeup(void)
{
	return st7789t3_instance.pf_wakeup(&st7789t3_instance);
}

/******************************************************************************
 * @name    lvgl_bsp_disp_set_backlight
 * @brief   背光开关
 * @param   value[in] 0=熄灭, 非0=点亮
 *
 * @return  0 success / -1 instance null (见驱动 @return)
 *****************************************************************************/
int8_t lvgl_bsp_disp_set_backlight(uint8_t value)
{
	return st7789t3_instance.pf_set_backlight(&st7789t3_instance, value);
}

/******************************************************************************
 * @name    lvgl_bsp_disp_set_window
 * @brief   设置绘制窗口(坐标含端点, 驱动内部处理面板偏移)
 * @param   x0[in] 左上X
 * @param   y0[in] 左上Y
 * @param   x1[in] 右下X
 * @param   y1[in] 右下Y
 *
 * @return  0 success / -1 instance null / -2.. 见驱动 @return
 *****************************************************************************/
int8_t lvgl_bsp_disp_set_window(uint16_t x0, uint16_t y0,
									uint16_t x1, uint16_t y1)
{
	return st7789t3_instance.pf_set_window(&st7789t3_instance, x0, y0, x1, y1);
}

/******************************************************************************
 * @name    lvgl_bsp_disp_write_pixels
 * @brief   向已设窗口写入RGB565像素(驱动内部做高字节先行交换与分块DMA)
 * @param   pdata[in] 像素数据(小端native存储)
 * @param   size[in]  像素个数
 *
 * @return  0 success / -1 instance null / -2.. 见驱动 @return
 *****************************************************************************/
int8_t lvgl_bsp_disp_write_pixels(uint16_t *pdata, uint32_t size)
{
	return st7789t3_instance.pf_write_pixels(&st7789t3_instance, pdata, size);
}

/******************************************************************************
 * @name    lvgl_bsp_disp_fill
 * @brief   用单色填充矩形
 * @param   x0[in] 左上X
 * @param   y0[in] 左上Y
 * @param   x1[in] 右下X
 * @param   y1[in] 右下Y
 * @param   color[in] RGB565颜色
 *
 * @return  0 success / -1 instance null / -2.. 见驱动 @return
 *****************************************************************************/
int8_t lvgl_bsp_disp_fill(uint16_t x0, uint16_t y0,
							  uint16_t x1, uint16_t y1, uint16_t color)
{
	return st7789t3_instance.pf_fill(&st7789t3_instance, x0, y0, x1, y1, color);
}

/******************************************************************************
 * @name    lvgl_bsp_disp_clear
 * @brief   整屏清屏
 * @param   color[in] RGB565颜色
 *
 * @return  0 success / -1 instance null / -2.. 见驱动 @return
 *****************************************************************************/
int8_t lvgl_bsp_disp_clear(uint16_t color)
{
	return st7789t3_instance.pf_clear(&st7789t3_instance, color);
}

/******************************************************************************
 * @name    lvgl_bsp_disp_draw_point
 * @brief   画单点
 * @param   x[in] X坐标
 * @param   y[in] Y坐标
 * @param   color[in] RGB565颜色
 *
 * @return  0 success / -1 instance null / -2.. 见驱动 @return
 *****************************************************************************/
int8_t lvgl_bsp_disp_draw_point(uint16_t x, uint16_t y, uint16_t color)
{
	return st7789t3_instance.pf_draw_point(&st7789t3_instance, x, y, color);
}

/******************************************************************************
 * @name    lvgl_bsp_disp_get_width
 * @brief   读取当前逻辑宽度(随显示方向变化)
 * @param   无
 *
 * @return  当前宽度(实例未构造时为0)
 *****************************************************************************/
uint16_t lvgl_bsp_disp_get_width(void)
{
	return st7789t3_instance.width;
}

/******************************************************************************
 * @name    lvgl_bsp_disp_get_height
 * @brief   读取当前逻辑高度(随显示方向变化)
 * @param   无
 *
 * @return  当前高度(实例未构造时为0)
 *****************************************************************************/
uint16_t lvgl_bsp_disp_get_height(void)
{
	return st7789t3_instance.height;
}
