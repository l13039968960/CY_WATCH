#include <stdint.h>
#include "easyapp_port.h"
#include "../inc/easyapp_event.h"

/*
 * =====================================================================
 *  临界区实现选择 (由 easyapp_port.h 里的 OSSUPPORT 决定)
 * =====================================================================
 */
#if OSSUPPORT
    /* CMSIS-RTOS v2: osKernelLock/osKernelUnlock 声明(调度器挂起/恢复作临界区,
       等价 vTaskSuspendAll/xTaskResumeAll). 只能在任务上下文调用, 不能在 ISR 里调用! */
    #include "cmsis_os2.h"

    #define EASYAPP_ENTER_CRITICAL()    (void)osKernelLock()
    #define EASYAPP_EXIT_CRITICAL()     (void)osKernelUnlock()
#else
    /* ---------------- 裸机: 关中断即可 (Cortex-M) ----------------
     * __get_PRIMASK / __disable_irq / __set_PRIMASK 由 CMSIS 内核头(core_cm*.h)提供。
     * Keil 工程通常已通过器件头(如 stm32f4xx.h)间接包含; 若本文件报未定义,
     * 就在本文件顶部 #include 你器件的 CMSIS 头。
     */
#endif/*OSSUPPORT*/

int32_t x_port_easyapp_event_send(EASYAPP_RIGISTERED_EVENTS_t event_id, uint32_t event_flags, void *event_data)
{
#if APP_LAYER_BYPASS
    /* ---- 临时测试台(2026-09-23): APP 层短路 ----
       一个字都不入环, 连临界区都不进(osKernelLock 也不取). 返回 0 而不是 -1:
       -1 的语义是"ring 满, 本事件被丢", 生产者(NordicProtocol 的 s_event_drop_cnt)
       会把它记成丢包, 看上去像溢出 —— 短路不该制造这种假象.

       ★放到 #else 里去的是 int32_t ret 的**声明**★: 留在这里会造成
       "unused variable 'ret'", 而 uvprojx 的 MiscControls 只压制了
       -Wno-unused-parameter/-Wno-unused-function, **没有** -Wno-unused-variable,
       wLevel=All Warnings ⇒ 一条新警告就破坏"0 Error / 9 Warning"的验收门槛. */
    (void)event_id;
    (void)event_flags;
    (void)event_data;

    return 0;
#else
    int32_t ret;

#if OSSUPPORT
    EASYAPP_ENTER_CRITICAL();
#else
    uint32_t primask = __get_PRIMASK();   /* 保存旧的中断屏蔽状态 */
    __disable_irq();                       /* 关中断 */
#endif

    ret = x_easyapp_event_send(event_id, event_flags, event_data);

#if OSSUPPORT
    EASYAPP_EXIT_CRITICAL();
#else
    __set_PRIMASK(primask);                /* 恢复旧状态(不是__enable_irq), 嵌套调用也安全 */
#endif

    return ret;
#endif /* APP_LAYER_BYPASS */
}
