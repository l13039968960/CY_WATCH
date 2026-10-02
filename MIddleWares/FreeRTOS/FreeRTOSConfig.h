/******************************************************************************
 * @file   FreeRTOSConfig.h
 * @brief  FreeRTOS 内核配置 (STM32F411CEU6 @100MHz, Keil AC6/armclang)
 *
 * @note   Cortex-M4F: configCPU_CLOCK_HZ 与 SysTick 均以 HCLK=100MHz 为基准.
 *         与 CMSIS-RTOS v2 封装(OS/FreeRTOS/CMSIS_RTOS_V2)配套:
 *         - configMAX_PRIORITIES 必须=56 且 configUSE_PORT_OPTIMISED_TASK_SELECTION=0
 *           (freertos_os2.h 强制检查)
 *         - USE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1: 不让 cmsis_os2.c 自产
 *           SysTick_Handler(会与 stm32f4xx_it.c 里"共用SysTick"的 HAL_IncTick 冲突),
 *           改由 it.c 自定义 SysTick_Handler 同时喂 HAL 时基与 FreeRTOS tick.
 *
 *         本文件不随 CubeMX 再生成, 属 OS 层自有文件.
 ******************************************************************************/
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/***********************************Includes***********************************/
/* 无: 保持自包含. CMSIS 设备头由 CMSIS_RTOS_V2/freertos_os2.h 经
   CMSIS_device_header 宏自行引入 */

/************************************Config************************************/
/* 时钟与节拍 ------------------------------------------------------------- */
#define configCPU_CLOCK_HZ                      ( ( unsigned long ) 100000000 ) /* HCLK: HSI->PLL 100MHz */
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 )         /* 与 HAL 时基共用 SysTick, 同为 1kHz */
#define configUSE_16_BIT_TICKS                  0                               /* CM4F 用 32 位 tick */

/* 调度 ---------------------------------------------------------------- */
#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0 /* CMSIS-RTOS2 要求 56 优先级, 不能走 32 位优化选路 */
#define configMAX_PRIORITIES                    ( 56 )                          /* CMSIS-RTOS2 要求 */
#define configMINIMAL_STACK_SIZE                ( ( unsigned short ) 128 )      /* 128 word = 512 B */
#define configMAX_TASK_NAME_LEN                 ( 16 )
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TASK_NOTIFICATIONS            1

/* 内存管理 ------------------------------------------------------------ */
#define configSUPPORT_DYNAMIC_ALLOCATION        1 /* heap_4: pvPortMalloc 动态堆 */
#define configSUPPORT_STATIC_ALLOCATION         0
#define configTOTAL_HEAP_SIZE                   ( ( size_t ) ( 32 * 1024 ) )    /* 32 KB 内核堆 */
/* @note 由 16KB → 24KB(FatFs 自检任务要 4096B 栈) → 32KB: 再加姿态(2048)与心率
   (2048)两个服务任务后, 24KB 时实测"起服务前余 3816B, 拉起姿态后只剩 1560B",
   心率任务 2048B 建不起来 → osThreadNew 返回 NULL, 服务内自己的 for(;;) 空转,
   还把同优先级的 rtstats 一起饿死. 32KB 后余量充足.
   多出的 8KB 是 .bss: RW+ZI 共 ~104KB, 芯片 128KB, 仍余 ~24KB.
   @warning configCHECK_FOR_STACK_OVERFLOW == 0: 栈溢出**不报错**, 只踩坏相邻
   堆块 —— 所以这里宁可多留余量. 心跳堆余量看 rtstats(每 5s 一行) */
#define configAPPLICATION_ALLOCATED_HEAP        0
#define configQUEUE_REGISTRY_SIZE               8

/* 内核对象 ------------------------------------------------------------ */
#define configUSE_MUTEXES                       1   /* CMSIS-RTOS2 Mutex 需要 */
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES           1   /* CMSIS-RTOS2 MemoryPool 需要 */
#define configUSE_EVENT_GROUPS                  1   /* CMSIS-RTOS2 EventFlags 需要(event_groups.c) */
#define configUSE_TIMERS                        1   /* CMSIS-RTOS2 osTimer / ISR event flag 需要 */
#define configTIMER_TASK_PRIORITY               ( 2 )
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            ( configMINIMAL_STACK_SIZE * 2 )

/* Hook / 跟踪 (全部关闭, 需要时再开) ----------------------------------- */
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0
#define configCHECK_FOR_STACK_OVERFLOW          0
#define configUSE_MALLOC_FAILED_HOOK            0
#define configUSE_TRACE_FACILITY                1   /* CMSIS-RTOS2 osThreadEnumerate 需要 */
#define configUSE_TICKLESS_IDLE                 0

/* 运行时间统计 (2026-09-28 开) ----------------------------------------- */
/* 打开后 vTaskGetRunTimeStats()/vTaskList() 才有定义. 注意 vTaskList 的
   "Stack" 列就是 uxTaskGetStackHighWaterMark 的水位值(单位: word), 所以
   栈水位检测不需要 configCHECK_FOR_STACK_OVERFLOW, 保持它的 0 即可.
   @note configRUN_TIME_COUNTER_TYPE 故意不定义: FreeRTOS.h 默认就是 uint32_t,
         与 TIM5->CNT 宽度一致. 若改成 64 位, vTaskGetRunTimeStats 会静默截断 */
#define configGENERATE_RUN_TIME_STATS           1
/* vTaskList/vTaskGetRunTimeStats 是"便利函数"不是内核, 由本开关单独控制;
   =1 会让 tasks.c #include <stdio.h>(用 sprintf 排版), =2 则只编函数不带头文件 */
#define configUSE_STATS_FORMATTING_FUNCTIONS    1

/* 统计时基 = TIM5 自由计数 10kHz (见 Core/system/rtstats/rtstats.c).
   由内核在 vTaskStartScheduler() 里调一次(见 tasks.c 里
   portCONFIGURE_TIMER_FOR_RUN_TIME_STATS), 所以"统计起点 = 调度器启动时刻",
   且不依赖 main.c 与内核的调用先后.
   @note **不能用 DWT->CYCCNT 当这个时基**: 百分比的分母就是这里返回的原始
         计数值(tasks.c 里 ulTotalRunTime = 当前计数值), 而 100MHz 的 32 位
         计数 42.9 秒就回绕一次 —— 系统跑 100 秒, 只占 3% 的任务会算出 98%.
         软件除法也救不了(数值变粗, 回绕周期不变). 必须换更慢的硬件计数器,
         详见 rtstats.c 文件头. TIM5 是 32 位, 10kHz 下 119.3 小时才回绕.
   @note 这两个宏只把函数名声明出来, 不 include rtstats.h —— 本文件要保持
         自包含(不引 CMSIS/工程头), tasks.c 那边也拿不到 stm32f4xx.h,
         所以这里不能直接写 TIM5->CNT */
extern void     rtstats_init(void);
extern uint32_t rtstats_get_counter(void);
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()  rtstats_init()
#define portGET_RUN_TIME_COUNTER_VALUE()          rtstats_get_counter()

/* 中断优先级 (STM32F4: 4 位优先级) ------------------------------------- */
#define configPRIO_BITS                         ( 4 )
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY         ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )

/* CMSIS-RTOS v2 封装相关 -------------------------------------------------- */
/* 让 cmsis_os2.c 不要生成 SysTick_Handler: 本工程 SysTick 为 HAL+FreeRTOS 共用,
   由 Core/Src/stm32f4xx_it.c 自定义 SysTick_Handler(HAL_IncTick + xPortSysTickHandler) */
#define USE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION 1
/* CMSIS 设备头文件: freertos_os2.h #include CMSIS_device_header 用 */
#define CMSIS_device_header                     "stm32f4xx.h"

/* CMSIS-RTOS2 API 依赖的 FreeRTOS 函数开关(freertos_os2.h 逐一 #error 检查) ---- */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xSemaphoreGetMutexHolder        1
#define INCLUDE_xTimerPendFunctionCall          1

/******************************************************************************/
/* FreeRTOS.h 需要把所有配置宏定义为 1 或 0 供 #if 使用; 个别文件里出现 1/0 之
   外的值是历史遗留(被 FreeRTOS.h 自身 #undef/#define 归一), 可忽略                */
/******************************************************************************/

/* assert 默认为空(Keil 下便于调试验证, 需要时可改成自定义上报) */
/* #define configASSERT( x )  ...  暂时关闭 */

#endif /* FREERTOS_CONFIG_H */
