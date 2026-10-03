/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file gpio_hal.c
 *
 * @par dependencies
 * - gpio_hal.h
 *
 * @author zw1194
 *
 * @brief Implete the OO-style HAL operations of GPIO.
 *
 * Processing flow:
 *
 * call gpio_driver_inst() to construct, pf_init() to claim the pins,
 * then use function pointers.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
#include "gpio_hal.h"

/********************************* 前向声明 *********************************/
static int8_t gpio_deinst(gpio_driver_t *p_gpio_instance);
static int8_t gpio_init(gpio_driver_t *p_gpio_instance);
static int8_t gpio_deinit(gpio_driver_t *p_gpio_instance);
static int8_t gpio_set_mode(gpio_driver_t *p_gpio_instance,
							uint32_t mode,
							uint32_t pull);

static int8_t gpio_write(gpio_driver_t *p_gpio_instance, uint8_t level);
static uint8_t gpio_read(gpio_driver_t *p_gpio_instance);
static int8_t gpio_toggle(gpio_driver_t *p_gpio_instance);

/********************************* 私有辅助 *********************************/

#define GPIO_PORT_NUM 6U /* 本芯片实际引出的端口数: A/B/C/D/E/H */

/* 端口级使用者计数: 同一个端口上所有实例共用一格 */
static uint8_t s_port_ref_count[GPIO_PORT_NUM];

/******************************************************************************
 * @name    gpio_port_index
 * @brief   端口基地址 -> 计数表下标
 * @param   p_port[in] GPIO端口
 *
 * @return  >=0 下标; -1 不在表内
 *****************************************************************************/
static int8_t gpio_port_index(GPIO_TypeDef *p_port)
{
	if (GPIOA == p_port)      { return 0; }
	if (GPIOB == p_port)      { return 1; }
	if (GPIOC == p_port)      { return 2; }
	if (GPIOD == p_port)      { return 3; }
	if (GPIOE == p_port)      { return 4; }
	if (GPIOH == p_port)      { return 5; }

	return -1;
}

/******************************************************************************
 * @name    gpio_port_clk_claim
 * @brief   取用端口时钟: 该端口第一个使用者才真正开, 计数累加
 * @param   p_port[in] GPIO端口
 *
 * @note    ★按端口计数, 不是按实例★ —— GPIOA/GPIOB 上挂着多个外设, 谁先归零谁
 *          关时钟会打死同端口的其他外设(如 w25q64_deinit 关掉 GPIOB → PWM/IIC 全灭).
 *          分派只能按基地址: RCC 时钟使能宏连寄存器位都是编译期固定的.
 *****************************************************************************/
static void gpio_port_clk_claim(GPIO_TypeDef *p_port)
{
	int8_t idx = gpio_port_index(p_port);

	if (idx < 0)
	{
		return;
	}

	if (0U == s_port_ref_count[idx])
	{
		if (GPIOA == p_port)      { __HAL_RCC_GPIOA_CLK_ENABLE(); }
		else if (GPIOB == p_port) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
		else if (GPIOC == p_port) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
		else if (GPIOD == p_port) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
		else if (GPIOE == p_port) { __HAL_RCC_GPIOE_CLK_ENABLE(); }
		else                      { __HAL_RCC_GPIOH_CLK_ENABLE(); }
	}

	s_port_ref_count[idx]++;
}

/******************************************************************************
 * @name    gpio_port_clk_release
 * @brief   归还端口时钟: 该端口最后一个使用者才真正关, 计数递减
 * @param   p_port[in] GPIO端口
 *
 * @note    ★只关本端口★, 别顺手关别的端口; 与 claim 严格成对.
 *****************************************************************************/
static void gpio_port_clk_release(GPIO_TypeDef *p_port)
{
	int8_t idx = gpio_port_index(p_port);

	if (idx < 0)
	{
		return;
	}

	if (0U == s_port_ref_count[idx])
	{
		return;
	}

	s_port_ref_count[idx]--;

	if (0U == s_port_ref_count[idx])
	{
		if (GPIOA == p_port)      { __HAL_RCC_GPIOA_CLK_DISABLE(); }
		else if (GPIOB == p_port) { __HAL_RCC_GPIOB_CLK_DISABLE(); }
		else if (GPIOC == p_port) { __HAL_RCC_GPIOC_CLK_DISABLE(); }
		else if (GPIOD == p_port) { __HAL_RCC_GPIOD_CLK_DISABLE(); }
		else if (GPIOE == p_port) { __HAL_RCC_GPIOE_CLK_DISABLE(); }
		else                      { __HAL_RCC_GPIOH_CLK_DISABLE(); }
	}
}

/******************************************************************************
 * @name    gpio_apply
 * @brief   按给定 mode/pull 写引脚配置, pins/speed/af 取 cfg
 * @param   p_gpio_instance[in]
 * @param   mode[in] 模式
 * @param   pull[in] 上下拉
 *
 * @note    ★模拟模式下 Pull 是空转★: HAL 在 MODE_ANALOG 时会跳过 PUPDR 配置
 *          (stm32f4xx_hal_gpio.c 那个 != MODE_ANALOG 判断), 所以低功耗收尾必须
 *          先 DeInit 把 PUPDR 清掉, 顺序不能反.
 *****************************************************************************/
static void gpio_apply(gpio_driver_t *p_gpio_instance, uint32_t mode, uint32_t pull)
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};

	GPIO_InitStructure.Pin       = p_gpio_instance->cfg.pins;
	GPIO_InitStructure.Mode      = mode;
	GPIO_InitStructure.Pull      = pull;
	GPIO_InitStructure.Speed     = p_gpio_instance->cfg.speed;
	GPIO_InitStructure.Alternate = p_gpio_instance->cfg.af;
	HAL_GPIO_Init(p_gpio_instance->cfg.p_port, &GPIO_InitStructure);
}

/********************************* GPIO操作 *********************************/

/******************************************************************************
 * @name    gpio_write
 * @brief   输出引脚电平
 * @param   p_gpio_instance[in]
 * @param   level[in] 0=低电平, 非0=高电平
 *
 * @return  0 success
 *         -1 gpio_instance null
 *
 * @note    直写 BSRR, 引脚还是输入态时也有效(锁存进 ODR, 转输出即生效).
 *          需要"上电即高"的输出脚在 pf_init 之前先调本函数.
 *****************************************************************************/
static int8_t gpio_write(gpio_driver_t *p_gpio_instance, uint8_t level)
{
	if (NULL == p_gpio_instance)
	{
		return -1;
	}

	HAL_GPIO_WritePin(p_gpio_instance->cfg.p_port, p_gpio_instance->cfg.pins,
					  level ? GPIO_PIN_SET : GPIO_PIN_RESET);

	return 0;
}

/******************************************************************************
 * @name    gpio_read
 * @brief   读取引脚电平
 * @param   p_gpio_instance[in]
 *
 * @return  1=高电平, 0=低电平(实例为空时返回0)
 *
 * @note    掩码含多个引脚时, 只要有一个为高即返回1(HAL_GPIO_ReadPin 语义)
 *****************************************************************************/
static uint8_t gpio_read(gpio_driver_t *p_gpio_instance)
{
	if (NULL == p_gpio_instance)
	{
		return 0;
	}

	if (HAL_GPIO_ReadPin(p_gpio_instance->cfg.p_port,
						 p_gpio_instance->cfg.pins) == GPIO_PIN_SET)
	{
		return 1;
	}

	return 0;
}

/******************************************************************************
 * @name    gpio_toggle
 * @brief   翻转引脚电平
 * @param   p_gpio_instance[in]
 *
 * @return  0 success
 *         -1 gpio_instance null
 *****************************************************************************/
static int8_t gpio_toggle(gpio_driver_t *p_gpio_instance)
{
	if (NULL == p_gpio_instance)
	{
		return -1;
	}

	HAL_GPIO_TogglePin(p_gpio_instance->cfg.p_port, p_gpio_instance->cfg.pins);

	return 0;
}

/********************************* 初始化与收尾 *********************************/

/******************************************************************************
 * @name    gpio_init
 * @brief   占用引脚(0→1 时才真正配), 引用计数累加
 * @param   p_gpio_instance[in]
 *
 * @return  0 success
 *         -1 gpio_instance null
 *****************************************************************************/
static int8_t gpio_init(gpio_driver_t *p_gpio_instance)
{
	if (NULL == p_gpio_instance)
	{
		return -1;
	}

	if (0U == p_gpio_instance->init_state)
	{
		gpio_port_clk_claim(p_gpio_instance->cfg.p_port);
		gpio_apply(p_gpio_instance, p_gpio_instance->cfg.mode,
				   p_gpio_instance->cfg.pull);
		p_gpio_instance->init_state = 1U;
	}

	p_gpio_instance->ref_count++;

	return 0;
}

/******************************************************************************
 * @name    gpio_deinit
 * @brief   释放引脚(减到 0 时才真正收尾), 引用计数递减
 * @param   p_gpio_instance[in]
 *
 * @return  0 success
 *         -1 gpio_instance null
 *
 * @note    ★收尾顺序★: 先 DeInit 再配模拟输入 —— F4 复位态是浮空输入,
 *          只 DeInit 施密特触发器仍开着; 而模拟模式下 HAL 不写 PUPDR,
 *          上下拉只能靠前一步 DeInit 清掉. 配完再归还端口时钟(写寄存器要靠它).
 * @note    ★端口时钟按端口计数关★: 本实例释放完了, 同端口还有别人在用就不关.
 *****************************************************************************/
static int8_t gpio_deinit(gpio_driver_t *p_gpio_instance)
{
	if (NULL == p_gpio_instance)
	{
		return -1;
	}

	if (p_gpio_instance->ref_count > 0U)
	{
		p_gpio_instance->ref_count--;
	}

	if ((0U == p_gpio_instance->ref_count) && (1U == p_gpio_instance->init_state))
	{
		HAL_GPIO_DeInit(p_gpio_instance->cfg.p_port, p_gpio_instance->cfg.pins);
		gpio_apply(p_gpio_instance, GPIO_MODE_ANALOG, GPIO_NOPULL);
		p_gpio_instance->init_state = 0U;

		gpio_port_clk_release(p_gpio_instance->cfg.p_port);
	}

	return 0;
}

/******************************************************************************
 * @name    gpio_set_mode
 * @brief   运行时改引脚模式与上下拉(如软件I2C切SDA收/发方向)
 * @param   p_gpio_instance[in]
 * @param   mode[in] 新模式(GPIO_MODE_INPUT / GPIO_MODE_OUTPUT_OD ...)
 * @param   pull[in] 新上下拉
 *
 * @return  0 success
 *         -1 gpio_instance null
 *
 * @note    临时状态, 不写回 cfg; 下次 pf_deinit/pf_init 会恢复成 cfg 里的配置.
 *****************************************************************************/
static int8_t gpio_set_mode(gpio_driver_t *p_gpio_instance,
							uint32_t mode,
							uint32_t pull)
{
	if (NULL == p_gpio_instance)
	{
		return -1;
	}

	gpio_apply(p_gpio_instance, mode, pull);

	return 0;
}

/********************************* 构造与析构 *********************************/

/******************************************************************************
 * @name    gpio_deinst
 * @brief   GPIO驱动析构: 清除函数指针(不碰硬件, 引脚由 pf_deinit 释放)
 * @param   p_gpio_instance[in]
 *
 * @return  0 success
 *         -1 gpio_instance null
 *****************************************************************************/
static int8_t gpio_deinst(gpio_driver_t *p_gpio_instance)
{
	if (NULL == p_gpio_instance)
	{
		return -1;
	}

	p_gpio_instance->pf_inst = NULL;
	p_gpio_instance->pf_deinst = NULL;
	p_gpio_instance->pf_init = NULL;
	p_gpio_instance->pf_deinit = NULL;
	p_gpio_instance->pf_set_mode = NULL;
	p_gpio_instance->pf_write = NULL;
	p_gpio_instance->pf_read = NULL;
	p_gpio_instance->pf_toggle = NULL;

	return 0;
}

/******************************************************************************
 * @name    gpio_driver_inst
 * @brief   GPIO驱动构造函数: 存配置、挂函数指针(不碰硬件, 引脚由 pf_init 配)
 * @param   p_gpio_instance[out] GPIO驱动实例
 * @param   p_cfg[in]            GPIO引脚配置
 *
 * @return  0 success
 *         -1 gpio_instance null
 *         -2 cfg null
 *         -3 port null
 *****************************************************************************/
int8_t gpio_driver_inst(gpio_driver_t *p_gpio_instance, gpio_cfg_t *p_cfg)
{
	if (NULL == p_gpio_instance)
	{
		return -1;
	}

	if (NULL == p_cfg)
	{
		return -2;
	}

	if (NULL == p_cfg->p_port)
	{
		return -3;
	}

	p_gpio_instance->cfg        = *p_cfg;
	p_gpio_instance->init_state = 0U;
	p_gpio_instance->ref_count  = 0U;

	p_gpio_instance->pf_inst     = gpio_driver_inst;
	p_gpio_instance->pf_deinst   = gpio_deinst;
	p_gpio_instance->pf_init     = gpio_init;
	p_gpio_instance->pf_deinit   = gpio_deinit;
	p_gpio_instance->pf_set_mode = gpio_set_mode;
	p_gpio_instance->pf_write    = gpio_write;
	p_gpio_instance->pf_read     = gpio_read;
	p_gpio_instance->pf_toggle   = gpio_toggle;

	return 0;
}
