#ifndef __EASYAPP_PAGE_H__
#define __EASYAPP_PAGE_H__

#include <stdint.h>
#include <stddef.h>

typedef int8_t (*page_event_func)(void *);

typedef enum EASYAPP_REGISTERED_EVENTS
{
    ALL_PAGES_HEAD = -1,
    PAGE_EVENT_HEAD = 0,
    EVT_SERVICE_ATTITUDE_NOBUSY,
    EVT_SERVICE_ATTITUDE_BUSY,
    EVT_SERVICE_ATTITUDE_DATA,
    EVT_SERVICE_ATTITUDE_NODONE,
    EVT_SERVICE_ATTITUDE_ERROR,
    EVT_SERVICE_HEARTRATE_NOBUSY,
    EVT_SERVICE_HEARTRATE_BUSY,
    EVT_SERVICE_HEARTRATE_DATA,
    EVT_SERVICE_HEARTRATE_NODONE,
    EVT_SERVICE_HEARTRATE_ERROR,
    EVT_SERVICE_UIHOME_IDLE,
    EVT_SERVICE_UIHOME_BUSY,
    EVT_SERVICE_UIHOME_GET_DATA,
    EVT_SERVICE_UIHOME_SWITCH_PAGE,
    EVT_SERVICE_UIHOME_NODONE,
    EVT_SERVICE_UIHOME_ERROR,
    /* 链路协议(service/NordicProtocol): 五类帧对应五个事件.
       顺序与 NordicProtocol_FrameKind_t 一一对应, 但两者是独立枚举, 不要互相赋值 */
    EVT_SERVICE_NORDIC_DATA,
    EVT_SERVICE_NORDIC_MESSAGE,
    EVT_SERVICE_NORDIC_EVENT,
    EVT_SERVICE_NORDIC_TXDONE,
    EVT_SERVICE_NORDIC_ERROR,
    /* OTA 页(APP/APPPAGE/UserPage): LVGL 页的**按钮点击上报**, 是本工程第一个
       "LVGL 页 -> app_core" 方向的事件(其余都是服务 -> UI 页, UIHOME 那一族是页间切页).
       event_flags 恒 0(不使用), event_data 恒 NULL —— 点一下就是"往前走一步",
       至于这一步是什么含义, 由 app_core 决定(见 cywatch_app_ota_page.c) */
    EVT_SERVICE_OTA_BUTTON,
    /* OTA 页: 协议服务(service/NordicProtocol)对 app_core 的**结果上报**, 方向与上面
       那条相反(TX 任务 / RX 任务 -> app_core). 两条都是"戳一下", **载荷一律为空**
       (event_flags/event_data 都不用): 具体数值由页面**调协议服务的 getter**取,
       页面不碰任何字节 —— 见 cywatch_service_nordicprotocol.h 的
       service_nordicprotocol_get_ota_*(). ★不复用 EVT_SERVICE_NORDIC_TXDONE★
       (那条是"链路帧发出去了", 语义不同) */
    EVT_SERVICE_OTA_CMD_DONE, /* 一条 OTA 命令发送结束(成败由 get_ota_result 报) */
    EVT_SERVICE_OTA_REPLY,    /* 对端的 CHECK 应答已到(值由 get_ota_* 取); flags = NordicProtocol_OtaRsp_t */
    /* OTA 页: **FatFs 服务的一条异步操作结束了**(开/写/关). 方向: fatfs 任务 -> app_core.
       event_flags = op 号(1=open / 2=write / 3=close), event_data 恒 NULL ——
       成败由页面读自己传给服务的那个静态结果量(服务不把它塞进事件).
       ★不复用 EVT_SERVICE_OTA_CMD_DONE★: 那条是"协议帧发完了"(TX 任务), 这条是
       "文件落盘了"(fatfs 任务), 两个服务、两个方向、两种时序, 混用会连环误判.
       ★这是"下载主循环"的节拍器★: 页面在收到它之后才请求下一块(见 cywatch_app_ota_page.c) */
    EVT_SERVICE_OTA_STORE_DONE,
    PAGE_EVENT_TAIL,
    ALL_PAGES_TAIL,
}EASYAPP_RIGISTERED_EVENTS_t;

typedef struct easyapp_page
{
    page_event_func func;
    uint32_t flag;
    EASYAPP_RIGISTERED_EVENTS_t page_event_lists[];
}easyapp_page_t;

#define easyapp_page_register(name, fun, ...)                             \
    __attribute__((used, section(".app_page_adr")))                       \
    easyapp_page_t easyapp_page_##name = {                                \
        .func = fun,                                                      \
        .flag = 0,                                                        \
        .page_event_lists = { __VA_ARGS__ }                               \
    }

int8_t easyapp_page_enable(easyapp_page_t * page);

int8_t easyapp_page_disable(easyapp_page_t * page);

#endif
