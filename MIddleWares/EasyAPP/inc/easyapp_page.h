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
    /* 温湿度服务(APP/Service/humiture): AHT21 挂在 MPU6050/MAX30102 同一条
       软件I2C总线上, 每 10s 触发一次测量 */
    EVT_SERVICE_HUMITURE_NOBUSY,
    EVT_SERVICE_HUMITURE_BUSY,
    EVT_SERVICE_HUMITURE_DATA,
    EVT_SERVICE_HUMITURE_ERROR,
    EVT_SERVICE_UIHOME_IDLE,
    EVT_SERVICE_UIHOME_BUSY,
    EVT_SERVICE_UIHOME_GET_DATA,
    EVT_SERVICE_UIHOME_SWITCH_PAGE,
    EVT_SERVICE_UIHOME_NODONE,
    EVT_SERVICE_UIHOME_ERROR,
    /* OTA 页(APP/Service/lvgl/ui/lv_watch_page_ota.c): LVGL 页的**按钮点击上报**,
       是本工程第一个 "LVGL 页 -> app_core" 方向的事件(其余都是服务 -> UI 页,
       UIHOME 那一族是页间切页). event_flags 恒 0(不使用), event_data 恒 NULL ——
       点一下就是"往前走一步", 至于这一步是什么含义, 由 app_core 决定 */
    EVT_SERVICE_OTA_BUTTON,
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
