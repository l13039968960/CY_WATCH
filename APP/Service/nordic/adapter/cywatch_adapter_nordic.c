/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_adapter_nordic.c
 *
 * @par dependencies
 * - cywatch_adapter_nordic.h
 * - UART/uart_hal.h
 * - system/Delay/delay.h
 * - cmsis_os2.h
 * - stdio.h / string.h
 *
 * @author	zw1194
 *
 * @brief Nordic 链路 BSP(Nordic/hal_driver) + 服务(service/NordicProtocol)之间的
 *        adapter, 见同名 .h 的说明. 本文件合并了原先分散在两处的两半:
 *          · service/UartProtocol/cywatch_service_UartProtocol.c 的 4 个 RTOS 对象
 *            与 8 个注入实现;
 *          · UART/adapter/cywatch_adapter_uart.c 的板级绑定与三个中断入口.
 *        理由是这两半描述的是同一件事(把这条链路接到 USART1 上), 分在两处只会
 *        让"谁持有信号量 / 谁套超时换算"需要跨文件追.
 *
 * Processing flow:
 *
 * nordic_bsp_inst(&cfg)
 *   → 建 4 个 RTOS 对象       → 句柄填进 s_sem/s_mtx/s_time
 *   → nordic_bsp_dma_init()  → DMA2 时钟 + 两个流 + NVIC(5/6/7)
 *   → nordic_bsp_check_nvic_prio()
 *   → uart_driver_inst()     → 拿到 g_uart1_driver(只装配, 不碰硬件)
 *   → g_uart1_driver.pf_init()→ HAL_UART_Init, 真正把 USART1 配起来
 *   → nordic_inst(&s_nordic, &cfg, &s_uart, &s_sem, &s_mtx, &s_time)
 * 之后服务再调 nordic_bsp_register_feature(特征号, 回调) 逐个注册要收的特征
 * 【本函数不建任务】调用方(服务)随后用 osThreadNew 挂两个入口:
 *   nordic_bsp_rx_entry → nordic_bsp_rx_start() 武装 RX + IDLE → pf_rx_task()
 *   nordic_bsp_tx_entry → pf_tx_task()
 * 之后: BSP 经四个注入接口回调本层 → USART1
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note ★include 全裸名, 相对路径已改掉★
 *       `uart_hal.h` 靠 IncludePath 里的 `..\..\Core\UART`,
 *       `system/delay/delay.h` 靠 `..\..\Core`(与 main.c 的写法一致)。
 *       反过来说: 本文件里看到 HAL/RTOS 类型是正常的, **不要**据此外推别处也可以。★
 *
 * @note ★中断优先级(本文件最容易被改坏的地方)★
 *       sem_rx 的释放发生在 **UART 接收中断**里(驱动 HAL_UARTEx_RxEventCallback →
 *       pf_release → osSemaphoreRelease). 中断里调 FreeRTOS API 的前提是抢占
 *       优先级数值 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY —— 本工程该
 *       值为 5(OS/FreeRTOS/FreeRTOSConfig.h:71). 所以这里三个口取 5/6/7, 并保持
 *       "RX DMA 流 > USART" 的顺序(驱动 UART/uart_hal.c 顶部注释的硬性要求:
 *       IDLE/半传输事件要按序处理).
 *       @warning 本工程 configASSERT 是**关闭**的(FreeRTOSConfig.h),
 *                所以优先级配错了不会断言, 而是**静默破坏内核**. 这就是下面
 *                nordic_bsp_check_nvic_prio() 存在的原因 —— 它把这条约束变成运行期
 *                可检测的返回值. 改了优先级表就要同步改 NORDIC_BSP_NVIC_PRIO_MIN.
 *       @note 旧 OTA 测试固件(已删)用的是 1/0/2(裸机时代, 无 RTOS), 照抄必挂.
 *
 * @note ★发送完成通知来自 USART1 的 TC 中断, 不是 DMA2_Stream7★
 *       HAL_UART_Transmit_DMA 发完后由 UART_DMATransmitCplt 打开 TCIE, 再由
 *       USART1_IRQHandler → UART_EndTransmit_IT → HAL_UART_TxCpltCallback 推进
 *       TX 环. 排障时往 USART1 找, 不要在 DMA2_Stream7 上找.
 *
 * @note 为什么 USART1 只能有这一个数据通路属主: 驱动实例内嵌**自己的**
 *       UART_HandleTypeDef(uart_hal.h:76). 任何别处拿一个指向 USART1 的句柄做
 *       HAL_UART_Transmit 都会绕过驱动直接写 DR —— 在 DMA 发送在途时就是抢寄存器、
 *       往协议线上注入。
 *       ★printf 已不在这里★: 日志口是 USART6(PA11/PA12), 见 Core/system/uart/uart.c
 *       —— 与 USART1 是两个独立外设, 互不抢寄存器。
 *
 * @note 驱动回调 HAL_UARTEx_RxEventCallback 靠 "huart 是驱动实例首成员" 做
 *       container_of 强转(uart_hal.c:358). 因此**任何其它 UART 句柄都不得进入
 *       ReceiveToIdle 模式**(printf 的 huart6 是阻塞 TX, 不涉及).
 ******************************************************************************/
#include "cywatch_adapter_nordic.h"
#include "uart_hal.h"
#include "dma_hal.h"
#include "system/delay/delay.h"
#include "cmsis_os2.h"

#include <stdio.h>
#include <string.h>

/***********************************Defines************************************/
/* 三个中断的抢占优先级(数值越大优先级越低). 约束见文件头 @note:
 * >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(5) 且 RX DMA > USART */
#define NORDIC_BSP_NVIC_PRIO_RX_DMA   5u   /* DMA2_Stream2 (RX) */
#define NORDIC_BSP_NVIC_PRIO_TX_DMA   6u   /* DMA2_Stream7 (TX) */
#define NORDIC_BSP_NVIC_PRIO_USART    7u   /* USART1(TC/IDLE 等) */
/* 允许的最小优先级数值(= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY) */
#define NORDIC_BSP_NVIC_PRIO_MIN      5u

/* 两个计数信号量的计满上限. 协议核每个"有活干"的边沿最多 release 一次, 任务醒来
 * 后一次性排空, 所以上限 ≥1 就够; 给 16/8 是留连续多次 release 的余量. 取小了
 * 不会出错(release 被内核拒掉, 驱动按"事件被合并"忽略), 但可能丢唤醒, 别往下调 */
#define NORDIC_BSP_SEM_RX_MAX         16u
#define NORDIC_BSP_SEM_TX_MAX         8u
/***********************************Defines************************************/

/**********************************Declaring***********************************/
/* ---- 本 adapter 独占的板级实例(文件私有: 服务只经 .h 的 8 个函数访问) ---- */
static uart_driver_t          g_uart1_driver;    /* 驱动实例(内嵌 huart + 三条环形缓冲) */
static DMA_HandleTypeDef      g_uart1_hdma_tx;   /* DMA2_Stream7 / Ch4 / NORMAL */
static DMA_HandleTypeDef      g_uart1_hdma_rx;   /* DMA2_Stream2 / Ch4 / CIRCULAR */
static dma_driver_t           g_uart1_dma_tx;    /* 上面两条流的驱动实例 */
static dma_driver_t           g_uart1_dma_rx;
static uart_cfg_t             g_uart1_cfg;
static uart_delay_interface_t g_uart1_delay;

/* ---- 4 个 RTOS 对象(本层创建, 句柄同时注入 BSP 与 UART 驱动) ---- */
static osSemaphoreId_t s_sem_rx;      /* 计数: RX 环形缓冲非空(驱动中断释放) */
static osSemaphoreId_t s_sem_tx_evt;  /* 计数: 发送侧有事件(含"查完成") */
static osMutexId_t     s_mtx_bus_tx;  /* 互斥: 总线写口(RX 直发 ACK / TX 发帧) */
static osMutexId_t     s_mtx_ring;    /* 互斥: 协议核发送环的写入口 */

/* ---- 驱动侧的信号量镜像(驱动类型, 与协议核那份是两个类型名) ----
   ★必须常驻★: 驱动实例里存的是这两个 static 结构体的**地址** */
static uart_semaphore_interface_t g_drv_sem_rx;
static uart_semaphore_interface_t g_drv_sem_tx;

/* ---- 协议核实例 + 四个注入接口(★必须常驻: BSP 存的是它们的地址★, 不能是调用方
        的栈变量). 实例约 10KB(5KB 发送环 + 4KB 重组缓冲 + 256 项回调表), 放 .bss ---- */
static bsp_nordic_driver_t          s_nordic;
static nordic_uart_interface_t      s_uart;
static nordic_semaphore_interface_t s_sem;
static nordic_mutex_interface_t     s_mtx;
static nordic_timebase_interface_t  s_time;

/* 幂等标志: uart_driver_inst / nordic_inst 都没有重复调用保护, 本层必须有 */
static uint8_t g_nordic_bsp_inited = 0u;

/**********************************Functions***********************************/

/**
 * @name  nordic_bsp_check_nvic_prio
 * @brief 复核三个中断的抢占优先级是否满足 FreeRTOS 约束(见文件头 @note)
 * @param 无
 * @return 0 三个口都合规 / -1 至少一个口的优先级低于 NORDIC_BSP_NVIC_PRIO_MIN
 * @note   ★阈值不能左移★: CMSIS 的 NVIC_GetPriority 是
 *         `__NVIC_GetPriority`(core_cm4.h:1838) 的别名, 它把 NVIC->IP[] 里的
 *         寄存器原值 **右移回低位**(>> (8 - __NVIC_PRIO_BITS)), 返回的是 0..15
 *         的归一化优先级 —— 而寄存器原值才是左移 4 位的那份.
 *         所以这里直接与 NORDIC_BSP_NVIC_PRIO_MIN(=5) 比较.
 * @warning 这里踩过一个坑, 记下来: 第一版按"NVIC_GetPriority 返回左移后的值"写,
 *          阈值取 `5u << (8 - __NVIC_PRIO_BITS)`(=80). 由于该函数是 __STATIC_INLINE,
 *          编译器能看出返回值 ≤ 15 < 80, 于是把三个比较**全部折叠成恒真**,
 *          整个函数被折叠成唯一的 `返 -4`(反汇编可见: 只剩一次 volatile 读 IPR,
 *          紧跟无条件 `MOVS r0,#0xfc`). 后果不是"校验失效"而是**上电即死**:
 *          nordic_bsp_inst 恒返 -4 → service_nordicprotocol_init 返 -4 → main 里
 *          Error_Handler() 停机, 而串口上只会看到一行 init failed.
 *          —— 校验代码本身永远不会"静默失效", 它只会静默地反着来.
 *          改这里请顺手用 fromelf 反汇编确认分支还在.
 */
static int8_t nordic_bsp_check_nvic_prio(void)
{
	if (NVIC_GetPriority(DMA2_Stream2_IRQn) < (uint32_t)NORDIC_BSP_NVIC_PRIO_MIN)
	{
		return -1;
	}
	if (NVIC_GetPriority(DMA2_Stream7_IRQn) < (uint32_t)NORDIC_BSP_NVIC_PRIO_MIN)
	{
		return -1;
	}
	if (NVIC_GetPriority(USART1_IRQn) < (uint32_t)NORDIC_BSP_NVIC_PRIO_MIN)
	{
		return -1;
	}
	return 0;
}

/**
 * @name  nordic_bsp_dma_init
 * @brief 装配 USART1 的两个 DMA 流(TX=NORMAL, RX=CIRCULAR)并配 NVIC
 * @param 无
 * @return 0 成功 / -1 流装配失败
 * @note   DMA2 时钟由 dma_hal 在装配流时打开
 * @note   HAL_DMA_Init 是阻塞的且用 HAL_GetTick 做超时, 故必须在 HAL_Init()
 *         之后调用(本函数经 nordic_bsp_inst 由 main 的 USER CODE 2 区调用, 满足)
 * @note   Parent 不在这里设: HAL_UART_Transmit_DMA/Receive_DMA 内部的
 *         __HAL_LINKDMA 会把它回指到 huart
 */
static int8_t nordic_bsp_dma_init(void)
{
	dma_cfg_t cfg_tx;
	dma_cfg_t cfg_rx;

	/* TX: DMA2_Stream7_Ch4, NORMAL, 内存->外设(驱动按"整段"发起) */
	cfg_tx.p_stream      = DMA2_Stream7;
	cfg_tx.channel       = DMA_CHANNEL_4;
	cfg_tx.irqn          = DMA2_Stream7_IRQn;
	cfg_tx.direction     = DMA_MEMORY_TO_PERIPH;
	cfg_tx.mode          = DMA_NORMAL;
	cfg_tx.priority      = DMA_PRIORITY_MEDIUM;
	cfg_tx.nvic_priority = NORDIC_BSP_NVIC_PRIO_TX_DMA;

	if (0 != dma_driver_inst(&g_uart1_dma_tx, &g_uart1_hdma_tx, &cfg_tx))
	{
		return -1;
	}

	if (0 != g_uart1_dma_tx.pf_init(&g_uart1_dma_tx))
	{
		return -1;
	}

	/* RX: DMA2_Stream2_Ch4, CIRCULAR, 外设->内存(IDLE 接收靠它) */
	cfg_rx.p_stream      = DMA2_Stream2;
	cfg_rx.channel       = DMA_CHANNEL_4;
	cfg_rx.irqn          = DMA2_Stream2_IRQn;
	cfg_rx.direction     = DMA_PERIPH_TO_MEMORY;
	cfg_rx.mode          = DMA_CIRCULAR;
	cfg_rx.priority      = DMA_PRIORITY_HIGH;
	cfg_rx.nvic_priority = NORDIC_BSP_NVIC_PRIO_RX_DMA;

	if (0 != dma_driver_inst(&g_uart1_dma_rx, &g_uart1_hdma_rx, &cfg_rx))
	{
		return -1;
	}

	if (0 != g_uart1_dma_rx.pf_init(&g_uart1_dma_rx))
	{
		return -1;
	}

	HAL_NVIC_SetPriority(USART1_IRQn, NORDIC_BSP_NVIC_PRIO_USART, 0u);
	HAL_NVIC_EnableIRQ(USART1_IRQn);

	return 0;
}

/**
 * @name  nordic_bsp_to_os_timeout
 * @brief 把协议核的超时语义(0=永久等)换算成 CMSIS 的(0=不等待)
 * @param timeout_ms[in] 协议核视角的超时(毫秒)
 * @return osWaitForever(当 timeout_ms==0) 或 timeout_ms
 * @note   ★★★ 这行代码看着多余, 删了就假死 ★★★
 *         协议核契约(cywatch_bsp_nordic_driver.h 的 pf_sem_acquire 说明):
 *         timeout_ms == 0 表示**永久等待**; 而 CMSIS 的 osSemaphoreAcquire /
 *         osMutexAcquire 把 timeout **直接当 tick**, 0 = 不等, 永久等是 osWaitForever
 *         —— 两者语义正好相反. 少了这一层翻译, RX 任务(挂载优先级
 *         osPriorityAboveNormal, 全工程最高)会在 sem_rx 上空转压死全家, 表现为
 *         上电假死, 且本工程 configASSERT 关着, **没有任何报错**.
 *         全工程最容易被"顺手简化掉"的一行就是这个函数体.
 */
static uint32_t nordic_bsp_to_os_timeout(uint32_t timeout_ms)
{
	return (0u == timeout_ms) ? (uint32_t)osWaitForever : timeout_ms;
}

/**
 * @name  nordic_bsp_sem_acquire
 * @brief 取信号量的内部实现(带 0→osWaitForever 换算), 上面 8 个无参包装用它
 * @param p_sem[in]     信号量句柄(osSemaphoreId_t)
 * @param timeout_ms[in] 超时(0=永久等)
 * @return 0 成功 / -1 失败或超时
 */
static int8_t nordic_bsp_sem_acquire(void *p_sem, uint32_t timeout_ms)
{
	if (NULL == p_sem)
	{
		return -1;
	}

	if (osOK != osSemaphoreAcquire((osSemaphoreId_t)p_sem,
								   nordic_bsp_to_os_timeout(timeout_ms)))
	{
		return -1;
	}
	return 0;
}

/**
 * @name  nordic_bsp_sem_wait_forever
 * @brief 取信号量并永久等待的内部实现(两个无参 uart_wait_* 包装用它)
 * @param p_sem[in] 信号量句柄
 * @return 0 成功 / -1 失败
 * @note  驱动只在 pf_wait_txcplt/pf_wait_rxcplt 里调它, 本链路不调用这两个口;
 *        存在只为满足 uart_driver_inst 的非空校验
 */
static int8_t nordic_bsp_sem_wait_forever(void *p_sem)
{
	if (NULL == p_sem)
	{
		return -1;
	}

	if (osOK != osSemaphoreAcquire((osSemaphoreId_t)p_sem, osWaitForever))
	{
		return -1;
	}
	return 0;
}

/**
 * @name  nordic_bsp_sem_release
 * @brief 信号量释放的内部实现: UART 驱动与协议核的无参包装都调它
 * @param p_sem[in] 信号量句柄
 * @return 0 成功 / -1 失败
 * @note  ★本函数会被 UART 接收中断调用★(驱动 HAL_UARTEx_RxEventCallback →
 *        pf_release, 经 nordic_bsp_uart_release_rx). osSemaphoreRelease 自身分支
 *        处理 ISR 上下文(ISR 里走 xSemaphoreGiveFromISR + 条件 PendSV), 是 ISR
 *        安全的; 前提是中断优先级满足 FreeRTOS 约束(见文件头 @note).
 */
static int8_t nordic_bsp_sem_release(void *p_sem)
{
	if (NULL == p_sem)
	{
		return -1;
	}

	if (osOK != osSemaphoreRelease((osSemaphoreId_t)p_sem))
	{
		return -1; /* 计数已满(正常现象, 说明有事件被合并了) */
	}
	return 0;
}

/**
 * @name  nordic_bsp_uart_wait_* / nordic_bsp_uart_release_*
 * @brief UART 驱动的无参信号量接口(四个)
 * @return 同 nordic_bsp_sem_wait_forever / nordic_bsp_sem_release
 * @note  ★uart_semaphore_interface_t 已经没有实例字段了★(见 uart_hal.h): 接口函数
 *        不带句柄, "释放/等待哪一个信号量"只能由包装函数各自闭包进去 —— 所以这里
 *        是四个而不是两个. ★不要合并成"用一个全局变量记住当前句柄"★: RX 与 TX 是
 *        两个不同的对象, 驱动在中断里同时可能用到它们.
 * @note  RX 那对才是承重的: 中断里释放 s_sem_rx(RX 任务的唯一唤醒源), 且必须与
 *        协议核 pf_rx_* 那一对是同一个对象.
 */
static int8_t nordic_bsp_uart_wait_rx(void)
{
	return nordic_bsp_sem_wait_forever((void *)s_sem_rx);
}

static int8_t nordic_bsp_uart_release_rx(void)
{
	return nordic_bsp_sem_release((void *)s_sem_rx);
}

static int8_t nordic_bsp_uart_wait_tx(void)
{
	return nordic_bsp_sem_wait_forever((void *)s_sem_tx_evt);
}

static int8_t nordic_bsp_uart_release_tx(void)
{
	return nordic_bsp_sem_release((void *)s_sem_tx_evt);
}

/**
 * @name  nordic_bsp_sem_rx/tx_acquire / _release
 * @brief 协议核的四个无句柄信号量取放口
 * @return 同 nordic_bsp_sem_acquire / nordic_bsp_sem_release
 * @note  ★与上面四个 uart_wait/release_* 同一个道理★: BSP 的注入结构体已经没有句柄
 *        字段了, "取/放哪一个"只能由包装函数各自闭包进去. ★不要合并成"用一个全局
 *        变量记住当前句柄"★: RX 与 TX 是两个不同的对象.
 * @note  这四个与 UART 驱动的那两对**共用同一对对象**(s_sem_rx / s_sem_tx_evt):
 *        中断里释放的就是任务里等的那个, 必须是同一个.
 */
static int8_t nordic_bsp_sem_rx_acquire(uint32_t timeout_ms)
{
	return nordic_bsp_sem_acquire((void *)s_sem_rx, timeout_ms);
}

static int8_t nordic_bsp_sem_rx_release(void)
{
	return nordic_bsp_sem_release((void *)s_sem_rx);
}

static int8_t nordic_bsp_sem_tx_acquire(uint32_t timeout_ms)
{
	return nordic_bsp_sem_acquire((void *)s_sem_tx_evt, timeout_ms);
}

static int8_t nordic_bsp_sem_tx_release(void)
{
	return nordic_bsp_sem_release((void *)s_sem_tx_evt);
}

/**
 * @name  nordic_bsp_mutex_acquire
 * @brief 取互斥量的内部实现(与信号量同样的 0→osWaitForever 换算)
 * @param p_mtx[in]      互斥量句柄(osMutexId_t)
 * @param timeout_ms[in] 超时(0=永久等)
 * @return 0 成功 / -1 失败或超时
 */
static int8_t nordic_bsp_mutex_acquire(void *p_mtx, uint32_t timeout_ms)
{
	if (NULL == p_mtx)
	{
		return -1;
	}

	if (osOK != osMutexAcquire((osMutexId_t)p_mtx,
							   nordic_bsp_to_os_timeout(timeout_ms)))
	{
		return -1;
	}
	return 0;
}

/**
 * @name  nordic_bsp_mutex_release
 * @brief 释放互斥量的内部实现
 * @param p_mtx[in] 互斥量句柄
 * @return 0 成功 / -1 失败
 */
static int8_t nordic_bsp_mutex_release(void *p_mtx)
{
	if (NULL == p_mtx)
	{
		return -1;
	}

	if (osOK != osMutexRelease((osMutexId_t)p_mtx))
	{
		return -1;
	}
	return 0;
}

/**
 * @name  nordic_bsp_mtx_bus/ring_acquire / _release
 * @brief 协议核的四个无句柄互斥量取放口
 * @return 同 nordic_bsp_mutex_acquire / nordic_bsp_mutex_release
 * @note  ★与上面信号量那四个同一个道理★: 无句柄、各自闭包. 总线互斥量的两个写者
 *        (RX 直发 ACK / TX 发帧)会同时用到它, 所以不能"用全局变量记住当前句柄".
 * @note  环互斥量的写者有应用、服务、以及 RX 任务(控制帧入环).
 */
static int8_t nordic_bsp_mtx_bus_acquire(uint32_t timeout_ms)
{
	return nordic_bsp_mutex_acquire((void *)s_mtx_bus_tx, timeout_ms);
}

static int8_t nordic_bsp_mtx_bus_release(void)
{
	return nordic_bsp_mutex_release((void *)s_mtx_bus_tx);
}

static int8_t nordic_bsp_mtx_ring_acquire(uint32_t timeout_ms)
{
	return nordic_bsp_mutex_acquire((void *)s_mtx_ring, timeout_ms);
}

static int8_t nordic_bsp_mtx_ring_release(void)
{
	return nordic_bsp_mutex_release((void *)s_mtx_ring);
}

/**
 * @name  nordic_bsp_get_time
 * @brief 协议核 pf_get_time: 毫秒时基
 * @return 内核 tick(本工程 1kHz → 1ms)
 * @note  本链路所有超时都走内核 tick, 不要换成 HAL_GetTick(两者起点差一个常量)
 */
static uint32_t nordic_bsp_get_time(void)
{
	return osKernelGetTickCount();
}

/**********************************Functions***********************************/
/* 以下三个是 nordic_uart_interface_t 的签名匹配实现: 直接填函数指针即可, 服务与
   BSP 之间不需要中间的转发胶水. 形参不带实例指针 —— 本层只有一个 UART 实例 */

/**
 * @name  nordic_bsp_uart_tx_send
 * @brief 把一整帧拷进驱动发送环并启动 DMA(pf_tx_send 的实现)
 * @param pdata[in] 整帧字节
 * @param size[in]  长度
 * @return 0 成功 / 其它 = 驱动 pf_tx_send 的返回码(-1 未实例化等)
 */
static int8_t nordic_bsp_uart_tx_send(uint8_t *pdata, uint32_t size)
{
	if (0u == g_nordic_bsp_inited)
	{
		return -1;
	}

	return g_uart1_driver.pf_tx_send(&g_uart1_driver, pdata, size);
}

/**
 * @name  nordic_bsp_uart_rx_get_count
 * @brief 驱动接收环里已收到的字节数(pf_rx_get_count 的实现)
 * @return 字节数(未实例化返回 0)
 */
static uint32_t nordic_bsp_uart_rx_get_count(void)
{
	if (0u == g_nordic_bsp_inited)
	{
		return 0u;
	}

	return g_uart1_driver.pf_rx_get_count(&g_uart1_driver);
}

/**
 * @name  nordic_bsp_uart_rx_read
 * @brief 从驱动接收环取走 size 字节(pf_rx_read 的实现, 短读不报错)
 * @param pdata[out] 取出的字节
 * @param size[in]   想要取的字节数
 * @return 0 成功 / -1 未实例化 / 其它 = 驱动 pf_rx_read 的返回码
 */
static int8_t nordic_bsp_uart_rx_read(uint8_t *pdata, uint32_t size)
{
	if (0u == g_nordic_bsp_inited)
	{
		return -1;
	}

	return g_uart1_driver.pf_rx_read(&g_uart1_driver, pdata, size);
}

/**********************************Functions***********************************/

int8_t nordic_bsp_inst(nordic_cfg_t *p_cfg)
{
	int8_t ret;

	if (NULL == p_cfg)
	{
		return -1;
	}

	if (0u != g_nordic_bsp_inited)
	{
		return 0; /* 幂等: 重复调用不重复初始化(见 .h 的 @note) */
	}

	/* ---- 1. 四个 RTOS 对象 ---- */
	s_sem_rx     = osSemaphoreNew(NORDIC_BSP_SEM_RX_MAX, 0u, NULL);
	s_sem_tx_evt = osSemaphoreNew(NORDIC_BSP_SEM_TX_MAX, 0u, NULL);
	s_mtx_bus_tx = osMutexNew(NULL);
	s_mtx_ring   = osMutexNew(NULL);
	if ((NULL == s_sem_rx) || (NULL == s_sem_tx_evt) ||
		(NULL == s_mtx_bus_tx) || (NULL == s_mtx_ring))
	{
		return -2; /* 内核堆不足(24KB heap_4), 失败是静默的, 只能在这里查 */
	}

	/* ---- 2. 板级绑定 ---- */
	/* RX 包装闭包的就是 s_sem_rx: 释放它的是 UART 接收中断, 而等待它的是 BSP 的
	   RX 任务 —— 两边必须是**同一个对象**, 否则中断释放了没人醒.
	   TX 包装闭包的是 s_sem_tx_evt: 驱动在发送完成回调里释放它, 正对应 BSP
	   "sem_tx_evt 的唤醒源包含 查完成" 的约定; 多出来的唤醒只会让 TX 任务空转一轮
	   (它自己会重算超时), 无害.
	   ★四个口都必须给★: uart_driver_inst 对 tx/rx 的 pf_wait 与 pf_release 逐个
	   做非空校验, 缺一即返 -6 */
	g_drv_sem_rx.pf_wait    = nordic_bsp_uart_wait_rx;
	g_drv_sem_rx.pf_release = nordic_bsp_uart_release_rx;

	g_drv_sem_tx.pf_wait    = nordic_bsp_uart_wait_tx;
	g_drv_sem_tx.pf_release = nordic_bsp_uart_release_tx;

	if (0 != nordic_bsp_dma_init())
	{
		return -3;
	}

	if (0 != nordic_bsp_check_nvic_prio())
	{
		return -4; /* 优先级不满足 FreeRTOS 约束(见文件头 @note) */
	}

	/* 驱动配置: USART1 115200 8N1 */
	g_uart1_cfg.p_uart_base       = USART1;
	g_uart1_cfg.init.BaudRate     = 115200;
	g_uart1_cfg.init.WordLength   = UART_WORDLENGTH_8B;
	g_uart1_cfg.init.StopBits     = UART_STOPBITS_1;
	g_uart1_cfg.init.Parity       = UART_PARITY_NONE;
	g_uart1_cfg.init.Mode         = UART_MODE_TX_RX;
	g_uart1_cfg.init.HwFlowCtl    = UART_HWCONTROL_NONE;
	g_uart1_cfg.init.OverSampling = UART_OVERSAMPLING_16;
	g_uart1_cfg.p_hdma_tx         = &g_uart1_hdma_tx;
	g_uart1_cfg.p_hdma_rx         = &g_uart1_hdma_rx;

	/* 引脚由本层给: uart_hal 不走 MSP, USART1 时钟与 PA9/PA10 在 pf_init 里配.
	   日志口已改到 USART6(PA11/PA12), 与这里不再重叠 */
	g_uart1_cfg.gpio.p_port = GPIOA;
	g_uart1_cfg.gpio.pins   = GPIO_PIN_9 | GPIO_PIN_10;
	g_uart1_cfg.gpio.mode   = GPIO_MODE_AF_PP;
	g_uart1_cfg.gpio.pull   = GPIO_NOPULL;
	g_uart1_cfg.gpio.speed  = GPIO_SPEED_FREQ_VERY_HIGH;
	g_uart1_cfg.gpio.af     = GPIO_AF7_USART1;

	/* 延时接口: 只填驱动要求的那一个口 */
	g_uart1_delay.pf_delay_us = delay_us;

	ret = uart_driver_inst(&g_uart1_driver, &g_uart1_cfg,
						   &g_drv_sem_tx, &g_drv_sem_rx, &g_uart1_delay);
	if (0 != ret)
	{
		return -5;
	}

	/* uart_driver_inst 只装配不碰硬件, USART1 的真正初始化在这里 */
	ret = g_uart1_driver.pf_init(&g_uart1_driver);
	if (0 != ret)
	{
		return -5; /* 见 .h: 与 inst 合并为"UART 驱动起不来" */
	}

	/* ---- 3. BSP 的四个注入接口(★必须常驻★: 协议核存的是它们的地址) ---- */
	/* 总线注入: 上面三个函数的签名与 nordic_uart_interface_t 逐字匹配, 直接填 */
	s_uart.pf_tx_send      = nordic_bsp_uart_tx_send;
	s_uart.pf_rx_get_count = nordic_bsp_uart_rx_get_count;
	s_uart.pf_rx_read      = nordic_bsp_uart_rx_read;

	/* 信号量注入: 四个无句柄口(谁是谁由包装函数闭包进去).
	   ★这里**没有 pf_thread_new**★ —— 任务创建已上移到服务层, BSP 不再自持任务 */
	s_sem.pf_rx_acquire = nordic_bsp_sem_rx_acquire;
	s_sem.pf_rx_release = nordic_bsp_sem_rx_release;
	s_sem.pf_tx_acquire = nordic_bsp_sem_tx_acquire;
	s_sem.pf_tx_release = nordic_bsp_sem_tx_release;

	/* 互斥量注入: 同样是四个无句柄口 */
	s_mtx.pf_bus_acquire  = nordic_bsp_mtx_bus_acquire;
	s_mtx.pf_bus_release  = nordic_bsp_mtx_bus_release;
	s_mtx.pf_ring_acquire = nordic_bsp_mtx_ring_acquire;
	s_mtx.pf_ring_release = nordic_bsp_mtx_ring_release;

	/* 时基注入 */
	s_time.pf_get_time = nordic_bsp_get_time;

	/* ---- 4. 构造协议核(校验四个注入接口 + 复位全部协议状态) ---- */
	ret = nordic_inst(&s_nordic, p_cfg, &s_uart, &s_sem, &s_mtx, &s_time);
	if (0 != ret)
	{
		return -6; /* 透传 BSP 的 -1 实例 / -2 cfg / -3 uart / -4 sem / -5 mtx / -6 time */
	}

	g_nordic_bsp_inited = 1u;
	return 0;
}

int8_t nordic_bsp_deinst(void)
{
	if (0u == g_nordic_bsp_inited)
	{
		return -1;
	}

	/* ★先叫醒两个任务, 再拆硬件★: 协议核的 deinst 会释放 sem_rx/sem_tx_evt, 阻塞着
	   的任务醒来见到"未初始化"就自行 return(入口包装接着 osThreadExit).
	   反过来先拆 UART 会让任务停在半拆的驱动上. */
	(void)s_nordic.pf_deinst(&s_nordic);

	HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
	HAL_NVIC_DisableIRQ(DMA2_Stream7_IRQn);
	HAL_NVIC_DisableIRQ(USART1_IRQn);

	/* 先放掉两条流再关 UART: 反过来的话 DMA 请求还挂在 USART1 上, 关外设时钟时
	   那些请求没有应答方, 流会卡在使能态. 顺带归还 DMA2 的使用计数 */
	(void)g_uart1_dma_rx.pf_deinit(&g_uart1_dma_rx);
	(void)g_uart1_dma_tx.pf_deinit(&g_uart1_dma_tx);
	(void)g_uart1_dma_rx.pf_deinst(&g_uart1_dma_rx);
	(void)g_uart1_dma_tx.pf_deinst(&g_uart1_dma_tx);

	/* 按 spi/iic 规范: 关外设走 pf_deinit, 析构走 pf_deinst. 反序调用的话
	   pf_deinst 见到 ref_count != 0 会直接返回 -2, USART1 就拆不掉 */
	(void)g_uart1_driver.pf_deinit(&g_uart1_driver); /* HAL_UART_DeInit */
	(void)g_uart1_driver.pf_deinst(&g_uart1_driver);

	g_nordic_bsp_inited = 0u;
	return 0;
}

int8_t nordic_bsp_rx_start(void)
{
	if (0u == g_nordic_bsp_inited)
	{
		return -1;
	}

	return g_uart1_driver.pf_rx_start(&g_uart1_driver);
}

/**
 * @name  nordic_bsp_rx_entry
 * @brief RX 任务入口(服务用 osThreadNew 挂它): 武装 RX → BSP 任务体 → 退出
 * @param p_arg[in] 原样透传(协议核不用, 恒为 NULL)
 * @return 无(正常路径永不返回)
 * @note  ★为什么武装放在这里而不是 main 的启动序列里★
 *        放在 osKernelStart() 之前同样安全(osSemaphoreRelease 在无任务等待时不会
 *        触发 PendSV, 且 HAL_UART_ReceiveToIdle_DMA 全程不依赖调度器/HAL tick),
 *        但上电瞬间 PA10(浮空输入)极易被噪声触发起始位, 那些噪声字节会被灌进
 *        解析器. 放到"RX 任务第一句"让噪声窗口与解析窗口完全重合.
 * @note  ★osThreadExit() 不能漏★: CMSIS-RTOS v2 下从线程函数 return 是未定义行为
 *        (底层是 FreeRTOS 的 vTaskDelete(NULL)). 宿主回归测**抓不到**这一条
 *        (替身里是 pthread, return 合法), 只能靠审查与上板.
 */
void nordic_bsp_rx_entry(void *p_arg)
{
	int8_t ret;

	(void)p_arg;

	ret = nordic_bsp_rx_start();
	if (0 != ret)
	{
		printf("[NORDIC] rx_start fail rc=%d (链路收方向不会工作)\r\n", (int)ret);
	}

	s_nordic.pf_rx_task(&s_nordic);

	osThreadExit(); /* 任务体返回 = 已 deinst, 自行退出 */
}

/**
 * @name  nordic_bsp_tx_entry
 * @brief TX 任务入口(服务用 osThreadNew 挂它): BSP 任务体 → 退出
 * @param p_arg[in] 原样透传(协议核不用, 恒为 NULL)
 * @return 无(正常路径永不返回)
 * @note  同 nordic_bsp_rx_entry: osThreadExit() 不能漏
 */
void nordic_bsp_tx_entry(void *p_arg)
{
	(void)p_arg;

	s_nordic.pf_tx_task(&s_nordic);

	osThreadExit();
}

int8_t nordic_bsp_register_feature(uint8_t feature, nordic_rx_cb_t pf_cb)
{
	if (0u == g_nordic_bsp_inited)
	{
		return -1;
	}

	return s_nordic.pf_register_feature(&s_nordic, feature, pf_cb);
}

int8_t nordic_bsp_send(uint8_t feature, const uint8_t *pdata, uint16_t len,
					   nordic_tx_done_cb_t pf_done)
{
	/* 未实例化时直接拒: 协议核自己也会返 -1, 这里再挡一道是为了不让"未初始化"这条
	   契约依赖 BSP 内部实现(它改了, 本层的返回值不变) */
	if (0u == g_nordic_bsp_inited)
	{
		return -1;
	}

	return s_nordic.pf_send(&s_nordic, feature, pdata, len, pf_done);
}

/*********************************中断入口*************************************/
/* @note AHT21_TEST.ioc **没有**使能这三个 NVIC(见文件头), 所以 CubeMX 不会生成
 *       它们的 IRQHandler, 也就不会与 Core/Src/stm32f4xx_it.c 撞名. 三个函数
 *       只做转发, 不做任何数据处理(中断纪律: 见 cywatch-bsp-driver skill) */

void USART1_IRQHandler(void)
{
	if (NULL != g_uart1_driver.pf_irq_handler)
	{
		g_uart1_driver.pf_irq_handler(&g_uart1_driver); /* IDLE/TC 等 */
	}
}

void DMA2_Stream7_IRQHandler(void)
{
	dma_irq_handler(DMA2_Stream7); /* TX 流 */
}

void DMA2_Stream2_IRQHandler(void)
{
	dma_irq_handler(DMA2_Stream2); /* RX 流 */
}
