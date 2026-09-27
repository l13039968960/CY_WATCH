#ifndef EASYAPP_PORT_H__
#define EASYAPP_PORT_H__

#include "../inc/easyapp_page.h"
#include "../inc/easyapp_core.h"
#include "../inc/easyapp_event.h"

/*
 * 运行环境选择:
 *   OSSUPPORT = 1  -> 运行在 RTOS 下, 临界区使用 OS 自带接口(见 easyapp_port.c 顶部)
 *   OSSUPPORT = 0  -> 裸机, 临界区使用"关中断"(Cortex-M PRIMASK, 保存/恢复)
 */
#define OSSUPPORT   1

/*
 * =====================================================================
 *  ★临时测试台: APP 层整体短路 (2026-09-23, 逐个服务单独上板测试用)★
 * =====================================================================
 *   APP_LAYER_BYPASS = 0 -> 正常: 事件入 128 深环, 由 app_core 分发到页面.
 *   APP_LAYER_BYPASS = 1 -> 短路: 本函数直接 return 0, 一个字都不入环
 *                          (见 easyapp_port.c). 配套地, Core/Src/main.c 里
 *                          service_appcore_init() 与 easyapp_page_enable() 也
 *                          被 #if !APP_LAYER_BYPASS 关掉 —— 那一半的开关在
 *                          main.c 的 USER CODE BEGIN PD 说明块里.
 *
 *   ★测完改回 0 即整体恢复原状★(不留其它痕迹); 删掉本段与 .c 里那段 #if
 *   就回到本批之前的样子. 也可用 -D APP_LAYER_BYPASS=1 覆盖.
 *
 *   @warning ★两条副作用, 别看成 bug★
 *            1) 页面(含 OTA 页的下载相位机)**全都不会跑** —— 下载循环靠"自己给
 *               自己发 EVT_*_DONE"推进, 短路后它每步都返回成功却永远停在
 *               DL_OPENING/DL_WAITING_BLOCK. 即"短路 app 层"与"测 OTA 下载"互斥.
 *            2) LVGL 的切屏动画照样播(那是 LVGL 侧 watch_switch 干的, 与
 *               EasyAPP 无关) ⇒ 现象是"画面能动、按下去没反应", 这是预期.
 *
 *   @note 为什么开关在这个头里而不是 main.c: 本函数实现在 easyapp_port.c,
 *         那是**另一个编译单元** —— main.c 里 #define 它、或写进 uvprojx 的
 *         Define 框都改不到 .c. 要单一落点, 只能在被 .c 读到的这个头里.
 *         main.c 侧另有一道 #ifndef APP_LAYER_BYPASS -> #error 守着"半态".
 */
#ifndef APP_LAYER_BYPASS
#define APP_LAYER_BYPASS    1
#endif

int32_t x_port_easyapp_event_send(EASYAPP_RIGISTERED_EVENTS_t event_id, uint32_t event_flags, void *event_data);

#endif
