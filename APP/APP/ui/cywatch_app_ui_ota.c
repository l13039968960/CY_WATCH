/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_app_ui_ota.c
 *
 * @brief OTA 页(ota, ID 0x0304)的 APP 层: 收切页通知、按键、圆按钮三类事件.
 *
 * Processing flow:
 *
 * LVGL 手势 ──EVT_SERVICE_UIHOME_SWITCH_PAGE(flags = 方向旗标)──> 本页 handler
 *   ──> app_ui_ota_switch_page(): 使能目标页 + 提到最前 → 失能自己
 * 按键服务 ──EVT_SERVICE_KEY_PRESS(flags = 键值, data = 手势)──> app_ui_ota_key()
 * LVGL 圆按钮 ──EVT_SERVICE_OTA_BUTTON──> service_ota_start() 起下载任务
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 本文件跑在 appcore 任务上下文. 要动控件只能走 lv_watch_page_*.h 那种零依赖的
 *       口, ★绝不 include lvgl 头★ —— LVGL 无锁, 对象树只能在 "lvgl" 任务里碰.
 *
 * @note ★本页不是开机页★: 注册成 PAGE_FLAG_DISABLE, 等 LVGL 从表盘页/血氧页切过来时
 *       由那一页的 switch_page 使能.
 *
 * @note ★本页只转发, 不做任何判断★: 圆按钮接到 service_ota_start() 上就结束了 ——
 *       "这一下是检查、是下载、还是重来"全由服务按相位分路. ★别在下面的按钮分支里
 *       再投任何显示值★: OTA 页的写者已经归 OTA 任务, 加第二个写者会静默丢更新.
 *       服务已经是**真实路径**(发 0x01/0x03/0x05, 收 0x02/0x04/整块, 写 ota/).
 *****************************************************************************/
#include "easyapp_port.h"
#include "easyapp_event.h"
#include "cywatch_service_key.h"
#include "cywatch_service_ota.h"
#include "system/log/cywatch_log.h"

static int8_t app_ui_ota_handler(void *event);
static int8_t app_ui_ota_key(easyapp_event_t *event);
static void app_ui_ota_switch_page(easyapp_event_t *event);

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

easyapp_page_register(ui_ota, app_ui_ota_handler, PAGE_FLAG_DISABLE,
    EVT_SERVICE_UIHOME_SWITCH_PAGE,
    EVT_SERVICE_KEY_PRESS,
    EVT_SERVICE_OTA_BUTTON,
);

/******************************************************************************
 * @name    app_ui_ota_handler
 * @brief   本页事件处理机: 切页跟随 + 按键 + 圆按钮
 * @param   event[in] EasyAPP 事件
 *
 * @return  1 = 消费
 *
 * @note    只订阅了切页/按键/圆按钮三条, 别的事件收不到
 *****************************************************************************/
static int8_t app_ui_ota_handler(void *event)
{
    easyapp_event_t *p_event = (easyapp_event_t *)event;

    switch (p_event->event_id)
    {
        case EVT_SERVICE_UIHOME_SWITCH_PAGE:
        {
            app_ui_ota_switch_page(p_event);
            return 1;
            break;
        }

        case EVT_SERVICE_KEY_PRESS:
        {
            return app_ui_ota_key(p_event);
            break;
        }

        case EVT_SERVICE_OTA_BUTTON:
        {
            /* 页面已经把"在跑就取消、没跑就开始"这类判别留给服务侧, 这里无脑起 ——
               服务的 start() 自己按任务存活分路 */
            service_ota_start();
            return 1;
            break;
        }

        /* TODO: 本页要收的其他事件在这里分派 */
        default:
            break;
    }

    return 0;
}

/******************************************************************************
 * @name    app_ui_ota_key
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
static int8_t app_ui_ota_key(easyapp_event_t *event)
{
    uint32_t             key    = event->event_flags;
    service_key_action_t action = (service_key_action_t)(uint32_t)event->event_data;

    switch (key)
    {
        case 1U: /* 键1 */
            switch (action)
            {
                case SERVICE_KEY_ACTION_CLICK:
                    log_printf("APP key1: page=OTA is_front=%d\r\n",
                               easyapp_page_flag_front == &easyapp_page_flag_ui_ota);
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
 * @name    app_ui_ota_switch_page
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
 * @note    拓扑(见 lv_watch_page.h): 左滑→血氧页(环上接回最右), 右滑→表盘页;
 *          上/下无去处
 *****************************************************************************/
static void app_ui_ota_switch_page(easyapp_event_t *event)
{
    switch ((app_page_dir_e)event->event_flags)
    {
        case APP_PAGE_TO_LEFT: /* 5: 左滑 → 血氧页 */
            easyapp_page_enable(&easyapp_page_flag_ui_spo2);
            easyapp_page_to_front(&easyapp_page_flag_ui_spo2);
            easyapp_page_disable(&easyapp_page_flag_ui_ota);
            break;

        case APP_PAGE_TO_RIGHT: /* 3: 右滑 → 表盘页 */
            easyapp_page_enable(&easyapp_page_flag_ui_home);
            easyapp_page_to_front(&easyapp_page_flag_ui_home);
            easyapp_page_disable(&easyapp_page_flag_ui_ota);
            break;

        default: /* 4/6: OTA 页没有上下方向的去处 */
            break;
    }
}
