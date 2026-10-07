#ifndef __EASYAPP_PAGE_H__
#define __EASYAPP_PAGE_H__

#include <stdint.h>
#include <stddef.h>

/**
 * @brief  页面事件处理函数原型
 * @param  event  事件对象(easyapp_event_t *), 页面只应读取
 * @return 1 = 事件已消费, 停止分发; 0 = 未消费, 继续发给后面的页面
 */
typedef int8_t (*page_event_func)(void *);

typedef enum page_flag{
    PAGE_FLAG_DISABLE,
    PAGE_FLAG_ENABLE,
}page_flag_t;

typedef enum EASYAPP_REGISTERED_EVENTS
{
    /* CY_WATCH 项目事件. 都插在 PAGE_EVENT_TAIL 之前 —— TAIL 是订阅列表的终止符,
       不能当事件 ID 用 */
    EVT_SERVICE_ATTITUDE_NOBUSY,
    EVT_SERVICE_ATTITUDE_BUSY,
    EVT_SERVICE_ATTITUDE_DATA,
    EVT_SERVICE_ATTITUDE_NODONE,
    EVT_SERVICE_ATTITUDE_ERROR,
    EVT_SERVICE_STEP_DATA,
    EVT_SERVICE_HEARTRATE_NOBUSY,
    EVT_SERVICE_HEARTRATE_BUSY,
    EVT_SERVICE_HEARTRATE_DATA,
    EVT_SERVICE_HEARTRATE_NODONE,
    EVT_SERVICE_HEARTRATE_ERROR,
    /* 佩戴状态翻转时发一次: 1=佩戴/0=未佩戴 放 event_flags, event_data 为 NULL.
       状态不变不发, 详见 cywatch_service_HeartRate.h */
    EVT_SERVICE_HEARTRATE_WEAR,
    EVT_SERVICE_HUMITURE_NOBUSY,
    EVT_SERVICE_HUMITURE_BUSY,
    EVT_SERVICE_HUMITURE_DATA,
    EVT_SERVICE_HUMITURE_NODONE,
    EVT_SERVICE_HUMITURE_ERROR,
    EVT_SERVICE_POWER_NOBUSY,
    EVT_SERVICE_POWER_BUSY,
    EVT_SERVICE_POWER_DATA,
    /* 充电状态翻转时发一次: 1=充电中(PB1 低电平)/0=未充电 放 event_flags,
       event_data 为 NULL. 状态不变不发, 详见 cywatch_service_power.h */
    EVT_SERVICE_POWER_CHARGING,
    EVT_SERVICE_POWER_ERROR,
    EVT_SERVICE_UIHOME_SWITCH_PAGE,
    EVT_SERVICE_OTA_BUTTON,
    /* AD 按键(PA2): 一条事件, 键值(1/2/3)在 event_flags, 手势在 event_data,
       见 cywatch_service_key.h */
    EVT_SERVICE_KEY_PRESS,
    /* 血氧页圆按钮被按下(不带数据). 语义是"切换": 没在测就开始, 在测就提前结束
       —— 判别在 service_spo2_start() 里, 发事件的 lvgl 任务不做判断 */
    EVT_SERVICE_SPO2_BUTTON,
    /* menu 页背光滑条: 亮度百分比放 event_flags, event_data 为 NULL.
       ★页面侧已经按差值降频了★(一次拖动不是每个像素都发), 收端只管写硬件 */
    EVT_SERVICE_BACKLIGHT_SET,

    PAGE_EVENT_TAIL
}EASYAPP_RIGISTERED_EVENTS_t;

typedef struct easyapp_page
{
    page_event_func func;
    EASYAPP_RIGISTERED_EVENTS_t page_event_lists[];
}easyapp_page_t;

typedef struct easyapp_page_flag
{
    const easyapp_page_t* page_addr;
    page_flag_t flag;
}easyapp_page_flag_t;

/* 编译期约定 —— 这几条被 easyapp_core.c 的指针自增遍历和 stress_test.c 的
 * 按字扫描依赖, 改动任何一个都会静默破坏分发。放在这里让改动立刻编译失败。
 * (枚举 4 字节是 armclang 的默认行为; 若有人加了 -fshort-enums, 这里会报错。) */
_Static_assert(sizeof(EASYAPP_RIGISTERED_EVENTS_t) == 4, "事件枚举必须为 4 字节");
_Static_assert(sizeof(page_event_func) == 4,             "函数指针必须为 4 字节");
_Static_assert(sizeof(easyapp_page_flag_t) == 8,         "页标志必须为 8 字节");

/**
 * @brief  注册一个页面到 flash 页表
 * @param  name   页面名, 生成两个对象 easyapp_page_<name> / easyapp_page_flag_<name>
 * @param  fun    页面事件处理函数(page_event_func)
 * @param  state  初始使能状态(PAGE_FLAG_ENABLE / PAGE_FLAG_DISABLE)
 * @param  ...    该页订阅的事件列表. PAGE_EVENT_TAIL 由宏自动补, 不用手写;
 *                但最后一项后面**必须留一个逗号** —— 宏是按 "args TAIL"
 *                拼接的, 少了那个逗号会编不过
 */
#define easyapp_page_register(name, fun, state, ...)                      \
    __attribute__((used, section(".app_page_events")))                    \
    const easyapp_page_t easyapp_page_##name = {                          \
        .func = fun,                                                      \
        .page_event_lists = { __VA_ARGS__ PAGE_EVENT_TAIL }                               \
    };                                                                    \
    __attribute__((used, section(".app_page_flag")))                      \
    easyapp_page_flag_t easyapp_page_flag_##name = {                      \
        .page_addr = &easyapp_page_##name,                                \
        .flag = state                                                     \
    }

/**
 * @brief  把页面提到分发表最前, 优先接收事件
 * @param  page  页面标志对象
 * @retval 0     成功(已在最前时也返回 0, 不做任何改动)
 * @retval -1    page 为 NULL, 或不是数组里的真实槽位(锚点/野指针)
 * @note   只改变分发的遍历起点, 页面记录本身不搬动, 其余页面保持注册顺序。
 *         同时最多只有一个页面被提升 —— 它是"单槽"的: 连续提升 A 再 B,
 *         只有 B 在前, A 回到它原来的次序(不是 B,A 都在前)。
 *         重复提升同一页是幂等的。
 *         提升不改变使能状态: 被提升的页面照样要用 enable()/disable() 控制。
 * @note   在页面处理函数里调用它, 从**下一次** easyapp_core_run() 起生效 ——
 *         分发器在一次遍历开始时锁存起点, 中途改变不会影响正在进行的这一次。
 */
int8_t easyapp_page_to_front(easyapp_page_flag_t * page);

/**
 * @brief  使能页面, 开始接收事件
 * @param  page  页面标志对象
 * @retval 0     成功
 * @retval -1    page 为 NULL
 */
int8_t easyapp_page_enable(easyapp_page_flag_t * page);

/**
 * @brief  失能页面, 停止接收事件
 * @param  page  页面标志对象
 * @retval 0     成功
 * @retval -1    page 为 NULL
 */
int8_t easyapp_page_disable(easyapp_page_flag_t * page);

#endif
