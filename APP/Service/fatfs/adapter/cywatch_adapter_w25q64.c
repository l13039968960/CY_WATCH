/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_adapter_w25q64.c
 *
 * @par dependencies
 * - cywatch_adapter_w25q64.h
 * - cywatch_bsp_w25q64_driver.h
 * - spi_hal.h
 * - main.h
 * - cmsis_os2.h
 *
 * @author	zw1194
 *
 * @brief 把 W25Q64 驱动挂到本工程的 SPI2 总线上.
 *
 * Processing flow:
 *
 * storage_bsp_w25q64_inst():
 *   填三个接口(SPI 的 pf_init/pf_deinit 转发到 spi2_instance; GPIO 的 pf_init 配
 *   CS=PB12; delay 走 osDelay) → w25q64_inst()
 *   (驱动内部回调 gpio/spi 的 pf_init, 再做芯片初始化 + 读 JEDEC ID 自检 0xEF/0x40/0x17);
 * 之后即可用 read/write/erase_* 转发到驱动的 pf_*.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 总线归属: SPI2(PB13=SCK/PB14=MISO/PB15=MOSI)实例 `spi2_instance` 由应用层
 *       main.c 创建, 本 adapter 只 extern 引用(与 ST7789T3 adapter 引用
 *       `lcd_spi_instance` 同一写法), 不自己再建总线. CS=PB12 是普通 GPIO,
 *       由本层的 pf_cs_set 直接驱动.
 *
 * @note `storage_bsp_w25q64_inst()` 内含 osDelay(芯片上电等待与 wait_busy 轮询) →
 *       必须在 osKernelStart() 之后的**任务上下文**调用, 不能放在 main 启动内核之前.
 *
 * @note ★初始化时机归驱动★: spi/gpio 两个接口各自带 pf_init/pf_deinit, 由驱动的
 *       w25q64_init/w25q64_deinit 回调(顺序 GPIO → SPI, 理由见驱动的同名函数) ——
 *       所以本层**不再**自己算"什么时候调 spi2_instance.pf_init", 只负责把函数挂上.
 *       这与 ST7789T3 的 adapter 是同一个形状.
 *
 * @note 与参考工程 Dirver_Test 的差异(两处必补, 否则芯片一个字节都不应答):
 *       1. CS(PB12) 的 GPIO 配置 —— 参考工程由 gpio.c 的 MX_GPIO_Init 代劳,
 *          本工程**没有**该函数, 见 w25q64_cs_gpio_init 的 @note;
 *       2. SPI2 外设初始化 —— 参考工程里 SPI2 从未真正初始化过(main.c 里的
 *          `spi_driver_t spi2_instance;` 是个全零桩, 只为满足链接器),
 *          本工程由本层的 spi pf_init 转发到 spi2_instance.pf_init 补上,
 *          见 w25q64_spi_init 的 @note.
 ******************************************************************************/
#include "cywatch_adapter_w25q64.h"

#include "cywatch_bsp_w25q64_driver.h"
#include "spi_hal.h"  /* spi_driver_t: 硬件 SPI2 总线, 由 main.c 提供 */
#include "main.h"     /* GPIO_TypeDef / GPIO_PIN_* */
#include "cmsis_os2.h" /* osDelay */

/***********************************Defines************************************/
/* CS 引脚: PB12 推挽输出, 低有效 */
#define W25Q64_CS_PORT  GPIOB
#define W25Q64_CS_PIN   GPIO_PIN_12
/***********************************Defines************************************/

/**********************************Declaring***********************************/
static bsp_w25q64_driver_t w25q64_instance;

/* 应用层(main.c)提供的总线实例: W25Q64 独占 SPI2 + CS(PB12), 不用 DMA */
extern spi_driver_t spi2_instance;

static w25q64_spi_interface_t   w25q64_spi_interface_instance;
static w25q64_gpio_interface_t  w25q64_gpio_interface_instance;
static w25q64_delay_interface_t w25q64_delay_instance;

/******************************************************************************
 * @name    w25q64_spi_init
 * @brief   SPI 外设初始化: 转发到应用层的 SPI2 总线实例(spi 接口的 pf_init)
 * @param   无
 *
 * @return  0 success / 其余见 spi_hal 的 pf_init
 *
 * @note    由驱动的 w25q64_init 回调, 本层不自己算调用时机。
 * @note    这一步是**本工程相对参考工程必须补的**: 参考工程里 SPI2 从未真正初始化过
 *          (main.c 的 `spi_driver_t spi2_instance;` 只是个全零桩, 只为满足链接器),
 *          少了它 SPI2 寄存器全是复位默认值(SPE=0), 一个字节也发不出去 —— 表现为
 *          读 JEDEC ID 得 0x00, w25q64_inst 卡在 -8。
 * @note    spi_hal 带引用计数: 若将来还有别的设备挂 SPI2, 这里只累加计数, 不会把
 *          外设重配一遍。
 *****************************************************************************/
static int8_t w25q64_spi_init(void)
{
	return spi2_instance.pf_init(&spi2_instance);
}

/******************************************************************************
 * @name    w25q64_spi_deinit
 * @brief   SPI 外设反初始化: 转发到 SPI2 总线实例(spi 接口的 pf_deinit)
 * @param   无
 *
 * @return  0 success / 其余见 spi_hal 的 pf_deinit
 *
 * @note    由驱动的 w25q64_deinit 回调, 在芯片进掉电之后
 *****************************************************************************/
static int8_t w25q64_spi_deinit(void)
{
	return spi2_instance.pf_deinit(&spi2_instance);
}

/******************************************************************************
 * @name    w25q64_cs_gpio_init
 * @brief   CS(PB12)配置成推挽输出, 空闲拉高(gpio 接口的 pf_init)
 * @param   无
 *
 * @return  0 success (本函数不做失败判定, 恒返回 0)
 *
 * @note    参考工程里这步由 MX_GPIO_Init(gpio.c) 代劳, 本工程**没有** MX_GPIO_Init
 *          —— 参考工程 Core/Src/gpio.c 里那个函数在迁移时没搬进来, main.c 的调用点
 *          ("Initialize all configured peripherals" 下面)也是空的。所以 PB12 除了这里,
 *          没有任何地方会配它。漏配的后果: 引脚停在复位默认的**输入浮空**态, 此时
 *          HAL_GPIO_WritePin 只写 BSRR/ODR, 输出驱动级是断开的 —— 电平等同于没写,
 *          CS 被模块自身上拉拉高 → W25Q64 忽略全部 SPI 流量 → w25q64_inst() 里读
 *          JEDEC ID 得到 0xFF/0x00, 卡在 -8, 而串口日志看不出任何 SPI 异常。
 * @note    先写 ODR 再配 MODER(与 ST7789T3 adapter 同序): 输出锁存器预置成空闲高,
 *          引脚一变成输出就是释放态, 不会产生一次假片选。
 * @note    Speed 取 VERY_HIGH: CS 是 SPI 时序的一部分, 边沿要跟得上 SCK(本总线 25MHz);
 *          ST7789T3 的四根控制脚用 LOW 是因为它们只在帧间翻转, 与这里不同。
 * @note    驱动在 w25q64_init 的**第一步**回调本函数, 早于 SPI 引脚切到 AF 和任何
 *          SPI 流量 —— 理由见驱动里 w25q64_init 的 @note。
 *****************************************************************************/
static int8_t w25q64_cs_gpio_init(void)
{
	GPIO_InitTypeDef gpio = { 0 };

	__HAL_RCC_GPIOB_CLK_ENABLE();

	HAL_GPIO_WritePin(W25Q64_CS_PORT, W25Q64_CS_PIN, GPIO_PIN_SET);

	gpio.Pin   = W25Q64_CS_PIN;
	gpio.Mode  = GPIO_MODE_OUTPUT_PP;
	gpio.Pull  = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(W25Q64_CS_PORT, &gpio);

	return 0;
}

/******************************************************************************
 * @name    w25q64_gpio_deinit
 * @brief   释放 CS(PB12), 恢复复位默认态(gpio 接口的 pf_deinit)
 * @param   无
 *
 * @return  0 success
 *
 * @note    不动 GPIOB 的端口时钟: 同一端口上还挂着 SPI2 的 PB13/14/15,
 *          关时钟会误伤(与 ST7789T3 adapter 的 gpio_deinit 同一考虑)
 *****************************************************************************/
static int8_t w25q64_gpio_deinit(void)
{
	HAL_GPIO_DeInit(W25Q64_CS_PORT, W25Q64_CS_PIN);

	return 0;
}

/******************************************************************************
 * @name    w25q64_delay_cb
 * @brief   延时: osDelay 返回 osStatus_t, 驱动要求 void(*)(uint32_t), 包一层丢弃返回值
 * @param   ms[in] 毫秒
 *
 * @return  无
 *
 * @note    依赖调度器运行 —— inst 内含芯片上电等待, 必须在任务上下文调用
 *****************************************************************************/
static void w25q64_delay_cb(uint32_t ms)
{
	(void)osDelay(ms);
}

/******************************************************************************
 * @name    w25q64_cs_set
 * @brief   CS(片选)写: 0=选中(拉低), 1=释放(拉高)
 * @param   level[in] 电平
 *
 * @return  0 success
 *****************************************************************************/
static int8_t w25q64_cs_set(uint8_t level)
{
	HAL_GPIO_WritePin(W25Q64_CS_PORT, W25Q64_CS_PIN,
					  (0 == level) ? GPIO_PIN_RESET : GPIO_PIN_SET);

	return 0;
}

/* ============================= 驱动接口转发 ============================= */
/* 驱动接口一律**不带**实例参数(见驱动头文件), 而 SPI 总线实例(spi2_instance)由
   main.c 创建、本层 extern 引用: 本适配器只服务这一片 Flash, 转发函数直接用 extern
   的实例调 spi_hal 的 pf_*(该实例正是 spi_hal 强类型接口的第一个实参). */

/******************************************************************************
 * @name    w25q64_spi_send_bytes
 * @brief   阻塞发送字节流(命令/地址/数据)
 * @param   pdata[in] 数据
 * @param   size[in]  字节数
 *
 * @return  0 success
 *         -1 spi instance null
 *         -2 HAL 传输失败 / -3 size 超过内部回波scratch(256B)
 *
 * @note    spi_hal 的 pf_transmit 走 HAL_SPI_TransmitReceive 收走回波防 OVR,
 *          单次≤SPI_TX_RX_SCRATCH_SIZE(256B) —— 正好覆盖 W25Q64 页编程的 256B
 *****************************************************************************/
static int8_t w25q64_spi_send_bytes(uint8_t *pdata, uint32_t size)
{
	return spi2_instance.pf_transmit(&spi2_instance, pdata, size);
}

/******************************************************************************
 * @name    w25q64_spi_receive_bytes
 * @brief   阻塞接收字节流(数据/状态/ID)
 * @param   pdata[in] 接收缓冲
 * @param   size[in]  字节数
 *
 * @return  0 success
 *         -1 spi instance null
 *         -2 HAL 接收失败
 *****************************************************************************/
static int8_t w25q64_spi_receive_bytes(uint8_t *pdata, uint32_t size)
{
	return spi2_instance.pf_receive(&spi2_instance, pdata, size);
}

/* ============================= 构造/析构 ============================= */

/******************************************************************************
 * @name    storage_bsp_w25q64_inst
 * @brief   构造W25Q64实例: 挂三个接口 → w25q64_inst(内部回调GPIO/SPI初始化, 含ID自检)
 * @param   无
 *
 * @return  0 success
 *         负值语义见驱动头文件: -1 实例空指针 / -2 SPI接口 / -3 GPIO接口 / -4 delay接口 /
 *         -7 芯片初始化失败(GPIO 或 SPI 的 pf_init 失败也在其中) /
 *         -8 JEDEC ID 自检不通过(芯片没接、接线错、供电不足、或 CS 没配)
 *
 * @note    须在 osKernelStart() 之后的任务上下文调用(内部 osDelay);
 *          SPI2 的 PB13/14/15 由 spi_hal 的 HAL_SPI_MspInit 配置, 调用方不要再调 MX_SPI2_Init
 * @note    初始化顺序(GPIO → SPI)与"为什么要这个顺序"都写在驱动的 w25q64_init 里,
 *          本层只把两个函数挂上
 *****************************************************************************/
int8_t storage_bsp_w25q64_inst(void)
{
	/* 1. 挂三个接口(每个都对应驱动 w25q64_inst 的一处非空校验) */
	w25q64_spi_interface_instance.pf_init          = w25q64_spi_init;
	w25q64_spi_interface_instance.pf_deinit        = w25q64_spi_deinit;
	w25q64_spi_interface_instance.pf_send_bytes    = w25q64_spi_send_bytes;
	w25q64_spi_interface_instance.pf_receive_bytes = w25q64_spi_receive_bytes;

	w25q64_gpio_interface_instance.pf_init         = w25q64_cs_gpio_init;
	w25q64_gpio_interface_instance.pf_deinit       = w25q64_gpio_deinit;
	w25q64_gpio_interface_instance.pf_cs_set       = w25q64_cs_set;

	w25q64_delay_instance.pf_delay                 = w25q64_delay_cb;

	/* 2. 构造驱动: 内部先回调 gpio/spi 的 pf_init, 再做芯片初始化 + 读 JEDEC ID
	      自检(0xEF/0x40/0x17), 失败即在此返回 */
	return w25q64_inst(&w25q64_instance,
					   &w25q64_spi_interface_instance,
					   &w25q64_gpio_interface_instance,
					   &w25q64_delay_instance);
}

/******************************************************************************
 * @name    storage_bsp_w25q64_deinst
 * @brief   析构W25Q64实例
 * @param   无
 *
 * @return  0 success
 *         -1 驱动返回的实例空指针错误
 *
 * @note    SPI2 经驱动 deinit 回调本层的 spi pf_deinit → spi_hal 的引用计数减 1,
 *          不是无条件关总线; 本工程 SPI2 只服务这一片 Flash, 计数归 0 时 spi_hal
 *          自己会 deinit 外设
 *****************************************************************************/
int8_t storage_bsp_w25q64_deinst(void)
{
	return w25q64_instance.pf_deinst(&w25q64_instance);
}

/* ============================= 能力转发 ============================= */

/******************************************************************************
 * @name    storage_bsp_w25q64_read_id
 * @brief   读 JEDEC ID(厂商/类型/容量)
 * @param   p_manuf_id[out]    厂商 ID(W25Q64 为 0xEF)
 * @param   p_memory_type[out] 存储类型(W25Q64 为 0x40)
 * @param   p_capacity[out]    容量(W25Q64 为 0x17, 即 8MB)
 *
 * @return  0 success / -1 实例空指针 / -2 SPI error (见驱动 @return)
 *****************************************************************************/
int8_t storage_bsp_w25q64_read_id(uint8_t *p_manuf_id,
								  uint8_t *p_memory_type,
								  uint8_t *p_capacity)
{
	return w25q64_instance.pf_read_id(&w25q64_instance,
									  p_manuf_id, p_memory_type, p_capacity);
}

/******************************************************************************
 * @name    storage_bsp_w25q64_read
 * @brief   读数据(任意长度, 驱动内部自动跨页)
 * @param   addr[in]  起始地址(24位)
 * @param   pdata[out] 接收缓冲
 * @param   size[in]  字节数
 *
 * @return  0 success / -1 实例空指针 / -2.. 见驱动 @return
 *****************************************************************************/
int8_t storage_bsp_w25q64_read(uint32_t addr, uint8_t *pdata, uint32_t size)
{
	return w25q64_instance.pf_read(&w25q64_instance, addr, pdata, size);
}

/******************************************************************************
 * @name    storage_bsp_w25q64_write
 * @brief   写数据(任意长度, 驱动内部自动分页; 目标区须先擦除)
 * @param   addr[in]  起始地址(24位)
 * @param   pdata[in] 数据
 * @param   size[in]  字节数
 *
 * @return  0 success / -1 实例空指针 / -2.. 见驱动 @return
 *
 * @note    本调用会阻塞至写入完成(页编程典型 0.7ms/最大 3ms, 靠 pf_delay→osDelay
 *          让出CPU), 调用方需保证所在任务优先级/栈能容忍
 *****************************************************************************/
int8_t storage_bsp_w25q64_write(uint32_t addr, uint8_t *pdata, uint32_t size)
{
	return w25q64_instance.pf_write(&w25q64_instance, addr, pdata, size);
}

/******************************************************************************
 * @name    storage_bsp_w25q64_erase_sector
 * @brief   擦除 4KB 扇区(典型 45ms/最大 400ms)
 * @param   addr[in] 扇区内任意地址
 *
 * @return  0 success / -1 实例空指针 / -2.. 见驱动 @return
 *****************************************************************************/
int8_t storage_bsp_w25q64_erase_sector(uint32_t addr)
{
	return w25q64_instance.pf_erase_sector(&w25q64_instance, addr);
}

/******************************************************************************
 * @name    storage_bsp_w25q64_erase_block_32k
 * @brief   擦除 32KB 块(典型 120ms/最大 1.6s)
 * @param   addr[in] 块内任意地址
 *
 * @return  0 success / -1 实例空指针 / -2.. 见驱动 @return
 *****************************************************************************/
int8_t storage_bsp_w25q64_erase_block_32k(uint32_t addr)
{
	return w25q64_instance.pf_erase_block_32k(&w25q64_instance, addr);
}

/******************************************************************************
 * @name    storage_bsp_w25q64_erase_block_64k
 * @brief   擦除 64KB 块(典型 150ms/最大 2s)
 * @param   addr[in] 块内任意地址
 *
 * @return  0 success / -1 实例空指针 / -2.. 见驱动 @return
 *****************************************************************************/
int8_t storage_bsp_w25q64_erase_block_64k(uint32_t addr)
{
	return w25q64_instance.pf_erase_block_64k(&w25q64_instance, addr);
}

/******************************************************************************
 * @name    storage_bsp_w25q64_erase_chip
 * @brief   整片擦除(典型 20s/最大 100s)
 * @param   无
 *
 * @return  0 success / -1 实例空指针 / -2.. 见驱动 @return
 *
 * @note    本调用会阻塞全程(靠 pf_delay→osDelay 让出CPU), 超时门限 120s 在驱动内设定,
 *          调用方需保证所在任务栈/看门狗能容忍长阻塞
 *****************************************************************************/
int8_t storage_bsp_w25q64_erase_chip(void)
{
	return w25q64_instance.pf_erase_chip(&w25q64_instance);
}

/******************************************************************************
 * @name    storage_bsp_w25q64_wait_busy
 * @brief   等待芯片空闲(轮询 BUSY 位, 超时 5s)
 * @param   无
 *
 * @return  0 success / -1 实例空指针 / -2.. 见驱动 @return
 *****************************************************************************/
int8_t storage_bsp_w25q64_wait_busy(void)
{
	return w25q64_instance.pf_wait_busy(&w25q64_instance);
}

/******************************************************************************
 * @name    storage_bsp_w25q64_hibernating
 * @brief   进入掉电(hibernation)模式, 电流降到 ~1uA
 * @param   无
 *
 * @return  0 success / -1 实例空指针 / -2.. 见驱动 @return
 *****************************************************************************/
int8_t storage_bsp_w25q64_hibernating(void)
{
	return w25q64_instance.pf_hibernating(&w25q64_instance);
}

/******************************************************************************
 * @name    storage_bsp_w25q64_wakeup
 * @brief   从掉电模式唤醒
 * @param   无
 *
 * @return  0 success / -1 实例空指针 / -2.. 见驱动 @return
 *****************************************************************************/
int8_t storage_bsp_w25q64_wakeup(void)
{
	return w25q64_instance.pf_wakeup(&w25q64_instance);
}
