#include <stdint.h>
#include "easyapp_port.h"
#include "../inc/easyapp_event.h"
#include "cmsis_os2.h"


int32_t x_port_easyapp_event_send(EASYAPP_RIGISTERED_EVENTS_t event_id, uint32_t event_flags, void *event_data)
{
    int32_t ret;

    /* 挂起/恢复调度器当临界区(等价 vTaskSuspendAll/xTaskResumeAll).
       只能在任务上下文调用, 中断里发事件不能用它 */
    (void)osKernelLock();

    ret = x_easyapp_event_send(event_id, event_flags, event_data);

    (void)osKernelUnlock();

    return ret;
}
