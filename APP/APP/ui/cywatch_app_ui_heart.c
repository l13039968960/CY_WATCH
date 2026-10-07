/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_app_ui_heart.c
 *
 * @brief 心率页(heart, ID 0x0302)的 APP 层: 收心率服务的数据 + 切页通知 + 按键事件
 *        + 一个常驻的"后台记录器"页(无界面, 只订阅心率事件, 存图表历史).
 *
 * Processing flow:
 *
 * 心率服务 ──EVT_SERVICE_HEARTRATE_DATA(event_data = 服务持有的最新帧)──> 本页 handler
 *   ──> watch_page_heart_set_hr(bpm) ──> lvgl 任务下次 1s 节拍渲染
 *   ★同一事件还有第二个消费者★: 常驻的 app_hr_bg_handler(见下面第二个 register),
 *   节流到 1Hz 后经 push_hr() 往图的历史里存一格 —— 它永远 enable, 所以本页
 *   没在显示时也在存
 * LVGL 手势 ──EVT_SERVICE_UIHOME_SWITCH_PAGE(flags = 方向旗标)──> 本页 handler
 *   ──> app_ui_heart_switch_page(): 使能目标页 + 提到最前 → 失能自己
 * 按键服务 ──EVT_SERVICE_KEY_PRESS(flags = 键值, data = 手势)──> app_ui_heart_key()
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 本文件跑在 appcore 任务上下文. 要动控件只能走 lv_watch_page_*.h 那种零依赖的
 *       口, ★绝不 include lvgl 头★ —— LVGL 无锁, 对象树只能在 "lvgl" 任务里碰.
 *
 * @note ★本页不是开机页★: 注册成 PAGE_FLAG_DISABLE, 等 LVGL 从 home 切过来时由
 *       home 的 app_ui_home_switch_page() 使能. 见 cywatch_app_ui_home.c.
 *
 * @note ★数据只走 lv_watch_page_heart.h 那个零依赖口★ 本页绝不 include lvgl 头 ——
 *       同一帧里还有 spo2_percent, 但血氧页这一轮先不接(见 cywatch_app_ui_spo2.c).
 *****************************************************************************/
#include "easyapp_port.h"
#include "easyapp_event.h"
#include "cywatch_service_key.h"
#include "cywatch_service_HeartRate.h"
#include "lv_watch_page_heart.h"
#include "system/log/cywatch_log.h"
#include "cmsis_os2.h" /* osKernelGetTickCount: 后台记录器的 1Hz 节流 */

static int8_t app_hr_bg_handler(void *event);
static int8_t app_ui_heart_handler(void *event);
static int8_t app_ui_heart_key(easyapp_event_t *event);
static void app_ui_heart_switch_page(easyapp_event_t *event);

/* 后台记录器的记录周期. 心率事件本身是 2Hz(cywatch_service_HeartRate.c 的
   SERVICE_HEARTRATE_PUBLISH_MS = 500), 而图是"30 格"固定的 —— 照原速存窗口会从
   30 秒砍到 15 秒, 所以这里自己节流到 1Hz, 保持 30 格 = 30 秒 */
#define APP_HR_RECORD_MS (1000U)

/* 上一次记录的时刻(记录器唯一的私有状态) */
static uint32_t s_hr_rec_tick = 0U;

/* 本页要切过去的页(flag 对象由各自的 easyapp_page_register 生成).
   ★只 extern flag 那个, 别 extern easyapp_page_xxx —— 后者在宏里是 const 的★ */
extern easyapp_page_flag_t easyapp_page_flag_ui_home;
extern easyapp_page_flag_t easyapp_page_flag_ui_spo2;

/* 框架里"被提到最前"的那一页, 只读. ★它没在任何头文件里声明★(只有 easyapp_core.c
   自己 extern 用), 这里跟着声明一份 —— 按键探针用它核对落在本页的按键是否来自前台页 */
extern easyapp_page_flag_t *easyapp_page_flag_front;

/* 切页方向旗标: LVGL 侧发 EVT_SERVICE_UIHOME_SWITCH_PAGE 时填在 event_flags 里.
   ★五个 APP 页各有一份同名枚举, 数值必须完全一致★(3/4/5/6 是 LVGL 侧已经发出去的
   值, 改了要同时动五个 LVGL 页).
   ★别拿 watch_swipe_dir_t 的数值来对★: 那是 LEFT=1/RIGHT=2/UP=3/DOWN=4, 另一套命名空间 */
typedef enum
{
    APP_PAGE_TO_RIGHT  = 3, /* 目标页在右方 */
    APP_PAGE_TO_BOTTOM = 4, /* 目标页在下方 */
    APP_PAGE_TO_LEFT   = 5, /* 目标页在左方 */
    APP_PAGE_TO_TOP    = 6, /* 目标页在上方 */
} app_page_dir_e;

easyapp_page_register(ui_heart, app_ui_heart_handler, PAGE_FLAG_DISABLE,
    EVT_SERVICE_HEARTRATE_DATA,
    EVT_SERVICE_UIHOME_SWITCH_PAGE,
    EVT_SERVICE_KEY_PRESS,
);

/* 常驻的"后台记录器": 无界面, 只订阅心率事件, ★永远 ENABLE★ —— 它不随切页被
   disable, 所以心率页没显示时也一直在往图的历史缓冲里存点(见 push_hr 的说明).
   ★它必须 return 0★: 只是"顺手抄一份", 返回 1 会把分发截断, 排在后面的表盘页心率
   卡片和心率页大字就都收不到了 */
easyapp_page_register(hr_bg, app_hr_bg_handler, PAGE_FLAG_ENABLE,
    EVT_SERVICE_HEARTRATE_DATA,
);

/******************************************************************************
 * @name    app_hr_bg_handler
 * @brief   后台记录器: 把心率按 1Hz 追加进心率页的图表历史(页面没显示时也存)
 * @param   event[in] EVT_SERVICE_HEARTRATE_DATA
 *
 * @return  恒 0(不消费) —— 心率事件还有别的消费者(表盘卡片 / 心率页大字)
 *
 * @note    节流到 1Hz 的理由: 事件是 2Hz, 而图只有 30 格, 照原速存窗口会减半
 * @note    ★存的是心率页结构体里那份 hist, 不是本层自己的缓冲★: 页面被 PageMem
 *          淘汰时控件(chart 的数据缓冲在里面)会没, 但页结构体是文件级 static, 擦不掉,
 *          所以历史能跨切页留下来. 本层没有"哪一页在显示"的概念, 只能存那儿
 * @note    没贴手指时 heart_rate_bpm 无意义, 存 0(与表盘卡片同一口径), 图会落到底
 *****************************************************************************/
static int8_t app_hr_bg_handler(void *event)
{
    easyapp_event_t *p_event = (easyapp_event_t *)event;
    uint32_t         now     = osKernelGetTickCount();

    if ((now - s_hr_rec_tick) < APP_HR_RECORD_MS)
    {
        return 0;
    }
    s_hr_rec_tick = now;

    HeartRate_Data_t *p_hr = (HeartRate_Data_t *)p_event->event_data;
    watch_page_heart_push_hr(
        p_hr->finger_on ? (uint8_t)(p_hr->heart_rate_bpm + 0.5f) : 0U);

    return 0;
}

/******************************************************************************
 * @name    app_ui_heart_handler
 * @brief   本页事件处理机: 切页跟随 + 按键, 其余待填
 * @param   event[in] EasyAPP 事件
 *
 * @return  1 = 消费
 *
 * @note    只订阅了切页与按键两条, 别的事件收不到
 *****************************************************************************/
static int8_t app_ui_heart_handler(void *event)
{
    easyapp_event_t *p_event = (easyapp_event_t *)event;

    switch (p_event->event_id)
    {
        case EVT_SERVICE_HEARTRATE_DATA:
        {
            /* event_data 指向心率服务最新帧: 只读, 别改所有权.
               没贴手指时 heart_rate_bpm 无意义, 投 0(与表盘卡片同一口径) */
            HeartRate_Data_t *p_hr = (HeartRate_Data_t *)p_event->event_data;
            watch_page_heart_set_hr(
                p_hr->finger_on ? (uint8_t)(p_hr->heart_rate_bpm + 0.5f) : 0U);
            break;
        }

        case EVT_SERVICE_UIHOME_SWITCH_PAGE:
        {
            app_ui_heart_switch_page(p_event);
            return 1;
            break;
        }

        case EVT_SERVICE_KEY_PRESS:
        {
            return app_ui_heart_key(p_event);
            break;
        }

        /* TODO: 本页要收的其他事件在这里分派 */
        default:
            break;
    }

    return 0;
}

/******************************************************************************
 * @name    app_ui_heart_key
 * @brief   按键处理机: 先按键值分派, 每档里再按手势分派(具体动作待填)
 * @param   event[in] EVT_SERVICE_KEY_PRESS 事件
 *
 * @return  恒 1(消费)
 *
 * @note    ★同一时刻只有一个 APP 页处于使能态★(切页时使能目标页、失能自己), 按键
 *          因此天然只落到当前那一页; 五页都返回 1 是防止同一次按键被重复处理, 别改回 0
 * @note    事件形状见 cywatch_service_key.h: event_flags = 键值(1/2/3),
 *          event_data = 手势(按值塞在指针里, service_key_action_t)
 * @note    本函数跑在 appcore 任务, LVGL 无锁 —— 要动控件只能走
 *          lv_watch_page_*.h 那组零依赖的口, 不能在这里直接碰对象树
 *****************************************************************************/
static int8_t app_ui_heart_key(easyapp_event_t *event)
{
    uint32_t             key    = event->event_flags;
    service_key_action_t action = (service_key_action_t)(uint32_t)event->event_data;

    switch (key)
    {
        case 1U: /* 键1 */
            switch (action)
            {
                case SERVICE_KEY_ACTION_CLICK:
                    log_printf("APP key1: page=HEART is_front=%d\r\n",
                               easyapp_page_flag_front == &easyapp_page_flag_ui_heart);
                    break;

                case SERVICE_KEY_ACTION_DOUBLE:
                    break;

                case SERVICE_KEY_ACTION_LONG:
                    break;

                default:
                    break;
            }
            break;

        case 2U: /* 键2 */
            switch (action)
            {
                case SERVICE_KEY_ACTION_CLICK:
                    break;

                case SERVICE_KEY_ACTION_DOUBLE:
                    break;

                case SERVICE_KEY_ACTION_LONG:
                    break;

                default:
                    break;
            }
            break;

        case 3U: /* 键3 */
            switch (action)
            {
                case SERVICE_KEY_ACTION_CLICK:
                    break;

                case SERVICE_KEY_ACTION_DOUBLE:
                    break;

                case SERVICE_KEY_ACTION_LONG:
                    break;

                default:
                    break;
            }
            break;

        default:
            break;
    }

    return 1;
}

/******************************************************************************
 * @name    app_ui_heart_switch_page
 * @brief   LVGL 侧切页通知: 使能目标页 + 提到最前, 再失能自己
 * @param   event[in] EVT_SERVICE_UIHOME_SWITCH_PAGE(event_flags = 方向旗标)
 *
 * @return  无
 *
 * @note    ★顺序不能反: 先使能目标页, 再失能自己★ —— 失能的页面收不到任何事件,
 *          五页同时失能就再没有任何代码能把它们打开, 只能由"当前还收得到事件的这一页"
 *          去开目标页
 * @note    ★to_front 是给按键用的★: 分发按注册顺序走, 提到最前才优先拿到按键.
 *          页面切换本身靠 enable/disable 就已经互斥, 不靠 to_front
 * @note    拓扑(见 lv_watch_page.h): 左滑→表盘, 右滑→血氧页; 上/下无去处
 *****************************************************************************/
static void app_ui_heart_switch_page(easyapp_event_t *event)
{
    switch ((app_page_dir_e)event->event_flags)
    {
        case APP_PAGE_TO_LEFT: /* 5: 左滑 → 表盘页 */
            easyapp_page_enable(&easyapp_page_flag_ui_home);
            easyapp_page_to_front(&easyapp_page_flag_ui_home);
            easyapp_page_disable(&easyapp_page_flag_ui_heart);
            break;

        case APP_PAGE_TO_RIGHT: /* 3: 右滑 → 血氧页 */
            easyapp_page_enable(&easyapp_page_flag_ui_spo2);
            easyapp_page_to_front(&easyapp_page_flag_ui_spo2);
            easyapp_page_disable(&easyapp_page_flag_ui_heart);
            break;

        default: /* 4/6: 心率页没有上下方向的去处 */
            break;
    }
}
