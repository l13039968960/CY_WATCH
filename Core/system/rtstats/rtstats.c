/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file rtstats.c
 *
 * @par dependencies
 * - rtstats.h
 * - stm32f4xx_hal.h
 * - FreeRTOS.h / task.h (vTaskList / vTaskGetRunTimeStats / 内核堆查询)
 * - cmsis_os2.h
 *
 * @author zw1194
 *
 * @brief Implete the FreeRTOS run-time statistics: timebase (TIM5 free running)
 *        + printer task.
 *
 * Processing flow:
 *
 * rtstats_init()  : 内核在 vTaskStartScheduler() 里调一次, 启动时基(TIM5)
 * rtstats_start() : 在 osKernelStart() 之前调一次, 建 "rtstats" 任务
 * "rtstats" 任务体: 每 5 秒打印一次内核堆 + 任务表(栈水位) + CPU 占比
 *
 * @version V1.0
 *
 * @note 为什么统计打印是一个**独立任务**而不是塞在别的任务里: 一屏约 700 字节,
 *       115200 波特率下 printf 要阻塞约 60ms, 塞进 lvgl 任务会让触摸采样和渲染
 *       出现肉眼可见的停顿. 独立成任务后, 它优先级最低, 那 60ms 只会让日志自己
 *       晚一点, 不挡任何人.
 *
 * @note 为什么不用 DWT->CYCCNT 当这个时基: vTaskGetRunTimeStats 算百分比时是拿
 * @note 为什么不用 DWT->CYCCNT 当这个时基: vTaskGetRunTimeStats 算百分比时是拿
 *       **时基计数器的原始值**当分母的(tasks.c 里 ulTotalRunTime = 当前计数值),
 *       100MHz 的 32 位计数 42.9 秒就回绕一次 —— 系统跑 100 秒, 一个只占 3% 的
 *       任务会算出 98%. 也修不好: 软件除法只是把数值变粗, 回绕周期不变. 必须换
 *       一个**更慢的硬件计数器**. 本模块用 TIM5 自由计数到 10kHz, 回绕周期
 *       119.3 小时.
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
#include "rtstats.h"
#include "stm32f4xx_hal.h"

#include <stdio.h>
#include "FreeRTOS.h" /* 必须先于 task.h */
#include "task.h"
#include "cmsis_os2.h"

/***********************************Defines************************************/
/* HAL_TIM 模块在本工程是关着的(stm32f4xx_hal_conf.h 里 HAL_TIM_MODULE_ENABLED 被
   注释掉), 拿不到 HAL 的 TIM_CR1_CEN / TIM_EGR_UG 宏, 所以本模块走纯寄存器,
   下面两个位定义是照参考手册补的 */
#define RTSTATS_TIM_CR1_CEN     (0x00000001U) /* CR1.CEN: 计数器使能 */
#define RTSTATS_TIM_EGR_UG      (0x00000001U) /* EGR.UG: 软件产生更新事件, 把 PSC 立即装进影子寄存器 */

/* 分频: APB1 定时器时钟 100MHz / 10000 = 10kHz. 刚好是 tick(1kHz) 的 10 倍,
   满足 FreeRTOS 要求的"统计计数器至少比 tick 快 10 倍"(见 tasks.c 里
   configGENERATE_RUN_TIME_STATS 那段的注释) */
#define RTSTATS_TIM_PRESCALER   (10000U - 1U)

/* 打印周期(ms) */
#define RTSTATS_PERIOD_MS       (5000u)

/* 任务栈(字节): 给 newlib printf 及 vTaskList 内部的 sprintf 留余量.
   注意 vTaskList/vTaskGetRunTimeStats 的输出缓冲区是 static 的, 不占这里 */
#define RTSTATS_STACK_SIZE      (1024u)

/* 输出缓冲区大小. static 且要够大: 这两个函数按"任务数翻倍"估用量(见 tasks.c
   里的注释), 放局部数组会吃掉本任务栈的一半 */
#define RTSTATS_BUF_SIZE        (1024u)
/***********************************Defines************************************/

/**********************************Declaring***********************************/
static void rtstats_task(void *pvParameters);
static void rtstats_dump(void);

/* 任务属性: 优先级压到 Low —— 它每 5 秒要占串口约 60ms, 必须抢不过任何业务 */
static const osThreadAttr_t g_rtstats_attr =
{
	.name       = "rtstats",
	.attr_bits  = 0,
	.cb_mem     = NULL,
	.cb_size    = 0,
	.stack_mem  = NULL,
	.stack_size = RTSTATS_STACK_SIZE,
	.priority   = osPriorityLow,
};
/**********************************Declaring***********************************/

/******************************************************************************
 * @name    rtstats_init
 * @brief   启动 TIM5 自由计数(10kHz, 无中断, 不用引脚)
 * @param   无
 *
 * @return  无
 *
 * @note    TIM5 在 STM32F4 上是 32 位定时器, 10kHz 下 2^32 个计数 = 119.3 小时
 *          才回绕, 所以"统计结果有效期"就是 119.3 小时, 一次调试会话足够.
 * @note    不用引脚: 选的是内部时钟源(默认 SMS=000), 只当计数器用
 * @note    不开中断: 计数器一直跑, 读 CNT 拿值, 没有 ISR/没有 NVIC 配置
 *****************************************************************************/
void rtstats_init(void)
{
	__HAL_RCC_TIM5_CLK_ENABLE();

	TIM5->PSC = RTSTATS_TIM_PRESCALER; /* 100MHz -> 10kHz */
	TIM5->ARR = 0xFFFFFFFFU;           /* 满量程自由计数 */
	TIM5->EGR = RTSTATS_TIM_EGR_UG;    /* 产生更新事件, PSC 立即生效并清零 CNT */
	TIM5->SR  = 0;                     /* 清 UG 顺带置起的 UIF, 免得以后有人开中断时吃到幽灵中断 */
	TIM5->CR1 = RTSTATS_TIM_CR1_CEN;   /* 启动 */
}

/******************************************************************************
 * @name    rtstats_get_counter
 * @brief   读时基当前计数值
 * @param   无
 *
 * @return  TIM5 的 CNT(10kHz 自由计数, 满量程回绕)
 *
 * @note    由 portGET_RUN_TIME_COUNTER_VALUE() 调用, 在任务切换点被频繁读取,
 *          所以只做一次寄存器读、不加任何判断
 *****************************************************************************/
uint32_t rtstats_get_counter(void)
{
	return TIM5->CNT;
}

/******************************************************************************
 * @name    rtstats_dump
 * @brief   打印一次运行统计: 内核堆 + 任务表(含栈水位) + CPU 占比
 * @param   无
 *
 * @return  无
 *
 * @note    堆**先打**: vTaskGetRunTimeStats 内部要 pvPortMalloc 一个
 *          TaskStatus_t 数组, 会临时压低 free —— 先打才能看到不含统计自身
 *          开销的那个空闲值
 * @note    vTaskList/vTaskGetRunTimeStats **不打印表头**, 表头得自己补
 * @note    单位: Stack 列是 uxTaskGetStackHighWaterMark 的**字(word)**数, 即
 *          该任务历史最小剩余栈; Abs Time 列是 TIM5 的原始计数值(1 个=100us),
 *          % Time 列才是要看的; Heap 是字节
 * @note    本任务自己也会出现在表里("rtstats") —— 正好可以照它看自己的栈给够没有
 *****************************************************************************/
static void rtstats_dump(void)
{
	static char buf[RTSTATS_BUF_SIZE];

	printf("\r\n===== FreeRTOS runtime stats =====\r\n");
	printf("Heap: free %u B, min-ever-free %u B\r\n",
		   xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());

	printf("Task\t\tState\tPrio\tStack\tNum\r\n");
	vTaskList(buf);
	printf("%s", buf);

	printf("Task\t\tAbs Time\t%% Time\r\n");
	vTaskGetRunTimeStats(buf);
	printf("%s", buf);
}

/******************************************************************************
 * @name    rtstats_task
 * @brief   运行统计打印任务体
 * @param   pvParameters[in] 未使用
 *
 * @return  无(不返回)
 *
 * @note    先 osDelay 再打印: 让启动阶段其他任务的日志先完整输出, 而且内核刚
 *          起来时各任务的栈水位/CPU 占比都还没有信息量
 *****************************************************************************/
static void rtstats_task(void *pvParameters)
{
	(void)pvParameters;

	while (1)
	{
		osDelay(RTSTATS_PERIOD_MS);
		rtstats_dump();
	}
}

/******************************************************************************
 * @name    rtstats_start
 * @brief   建运行统计打印任务("rtstats")
 * @param   无
 *
 * @return  0 success
 *         -1 任务创建失败(FreeRTOS 内核堆不足)
 *****************************************************************************/
int8_t rtstats_start(void)
{
	if (NULL == osThreadNew(rtstats_task, NULL, &g_rtstats_attr))
	{
		printf("RTSTATS: osThreadNew failed\r\n");
		return -1;
	}

	return 0;
}
