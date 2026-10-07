/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_app_ui_home.c
 *
 * @brief 表盘页的 APP 层: 收各服务的数据事件, 把值投给主页自己的三张卡片与
 *        左上角电量区; 另收按键事件(处理机骨架已搭, 具体动作待填).
 *
 * Processing flow:
 *
 * 服务任务 ──EVT_SERVICE_xxx_DATA(event_data = 服务持有的最新帧)──> 本页 handler
 *   ──> watch_page_home_set_data(哪一项, 值) ──> lvgl 任务下次 1s 节拍渲染
 * 按键服务 ──EVT_SERVICE_KEY_PRESS(flags = 键值, data = 手势)──> 本页 handler
 *   ──> app_ui_home_key() 按键值 + 手势两级分派
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 本文件跑在 appcore 任务上下文, 只调 lv_watch_page_home.h 那个零依赖的口,
 *       绝不 include lvgl 头 —— LVGL 无锁, 对象树只能在 "lvgl" 任务里碰.
 *
 * @note 电池那条链有**两条事件**: 电压 EVT_SERVICE_POWER_DATA 每 10s 发一帧, 充电状态
 *       EVT_SERVICE_POWER_CHARGING 只在翻转时发. 两者都投给左上角那个电量区.
 *       ★充电时端电压被钉在 4.2V, 换算不出电量★ —— 那一段显示值改走一条斜坡,
 *       见 BAT_CHARGE_FULL_MS 与 s_chg_* 那组静态变量.
 *
 * @note ★切页跟随★: LVGL 侧每次切页都发 EVT_SERVICE_UIHOME_SWITCH_PAGE(event_flags =
 *       方向旗标 3/4/5/6). 本页据此**使能目标页、失能自己**, 所以同一时刻只有一页
 *       处于使能态 —— 事件(含按键)天然只落到当前显示的那一页. 见 app_ui_home_switch_page.
 *****************************************************************************/
#include "easyapp_port.h"
#include "easyapp_event.h"
#include "lv_watch_page_home.h"
#include "StepDetection.h"
#include "cywatch_service_HeartRate.h"
#include "cywatch_service_humiture.h"
#include "cywatch_service_power.h"
#include "cywatch_service_key.h"
#include "system/log/cywatch_log.h"
#include "cmsis_os2.h" /* osKernelGetTickCount: 充电时的电量爬升要一个真实时间基准 */

static int8_t app_ui_home_handler(void *event);
static int8_t key_handler(easyapp_event_t *event);
static void app_ui_home_switch_page(easyapp_event_t *event);

/* 本页要切过去的另外三页(flag 对象由各自的 easyapp_page_register 生成).
   ★只 extern flag 那个, 别 extern easyapp_page_xxx —— 后者在宏里是 const 的★ */
extern easyapp_page_flag_t easyapp_page_flag_ui_menu;
extern easyapp_page_flag_t easyapp_page_flag_ui_heart;
extern easyapp_page_flag_t easyapp_page_flag_ui_ota;

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

easyapp_page_register(ui_home, app_ui_home_handler, PAGE_FLAG_ENABLE,
    EVT_SERVICE_STEP_DATA,
    EVT_SERVICE_HEARTRATE_DATA,
    EVT_SERVICE_HUMITURE_DATA,
    EVT_SERVICE_POWER_DATA,
    EVT_SERVICE_POWER_CHARGING,
    EVT_SERVICE_UIHOME_SWITCH_PAGE,
    EVT_SERVICE_KEY_PRESS,
);

/* 3.7V 锂电的空载放电曲线(分段, 电压从高到低). 中段 3.7~3.8V 那一大段平台是锂电的
   特征, 线性换算会让中段掉得比实际快, 所以用查表 */
#define BAT_TBL_CNT (10U)

/* 假定的 0→100% 充电时长(1小时). ★真实充电时长固件测不出来★: 没有电流检测, 而且
   PB1 那根 STAT 脚上"充满"和"已拔线"都是高电平, 分不开. 只能假定一个 */
#define BAT_CHARGE_FULL_MS (3600000U)

static const uint16_t s_bat_mv[BAT_TBL_CNT]  =
    {4200U, 4100U, 4000U, 3900U, 3800U, 3700U, 3600U, 3500U, 3400U, 3300U};
static const uint8_t  s_bat_pct[BAT_TBL_CNT] =
    {100U,   90U,   80U,   65U,   50U,   35U,   20U,   10U,    5U,    0U};

/* 充电时的电量爬升状态. 只在充电中用来生成显示值, 一拔线立刻回到实测换算 */
static int32_t  s_bat_disp  = 0; /* 最近一次投给页面的百分比 */
static uint8_t  s_chg_on    = 0; /* 当前是否在充电(来自 CHARGING 事件) */
static int32_t  s_chg_pct0  = 0; /* 插上充电器那一刻的显示值 */
static uint32_t s_chg_tick0 = 0; /* 那一刻的 tick */

/******************************************************************************
 * @name    battery_mv_to_percent
 * @brief   电池毫伏 → 百分比(0~100), 表内相邻两点之间线性插值
 * @param   mv[in] 电池电压, 整数毫伏
 *
 * @return  0~100; 超出表范围时钳到两端
 *
 * @note    表按空载标定, 带载时读数偏低, 百分比也会跟着偏低一点
 *****************************************************************************/
static int32_t battery_mv_to_percent(uint16_t mv)
{
    uint32_t i = 0U;

    if (mv >= s_bat_mv[0])
    {
        return 100;
    }
    if (mv <= s_bat_mv[BAT_TBL_CNT - 1U])
    {
        return 0;
    }

    for (i = 0U; i + 1U < BAT_TBL_CNT; i++)
    {
        if (mv >= s_bat_mv[i + 1U])
        {
            uint32_t span_mv  = (uint32_t)s_bat_mv[i] - (uint32_t)s_bat_mv[i + 1U];
            uint32_t span_pct = (uint32_t)s_bat_pct[i] - (uint32_t)s_bat_pct[i + 1U];
            uint32_t offset   = (uint32_t)mv - (uint32_t)s_bat_mv[i + 1U];

            return (int32_t)((uint32_t)s_bat_pct[i + 1U] +
                             (offset * span_pct) / span_mv);
        }
    }

    return 0;
}

static int8_t app_ui_home_handler(void *event)
{
    easyapp_event_t *p_event = (easyapp_event_t *)event;

    switch (p_event->event_id)
    {
        case EVT_SERVICE_STEP_DATA:
        {
            /* event_data 指向姿态服务最新帧里的 step 字段: 只读, 别改所有权 */
            step_result_t *p_step = (step_result_t *)p_event->event_data;
            watch_page_home_set_data(WATCH_HOME_DATA_STEP, (int32_t)p_step->steps);
            break;
        }

        case EVT_SERVICE_HEARTRATE_DATA:
        {
            /* 没贴手指时 heart_rate_bpm 无意义, 投 0; 贴上才四舍五入到整数 bpm */
            HeartRate_Data_t *p_hr = (HeartRate_Data_t *)p_event->event_data;
            watch_page_home_set_data(
                WATCH_HOME_DATA_HR,
                p_hr->finger_on ? (int32_t)(p_hr->heart_rate_bpm + 0.5f) : 0);
            break;
        }

        case EVT_SERVICE_HUMITURE_DATA:
        {
            /* 四舍五入到整数摄氏度. 负数要往远离 0 的方向加, 否则 -3.4 截断成 -3 再偏 +1 */
            float t = ((Humiture_Data_t *)p_event->event_data)->temperature;
            watch_page_home_set_data(WATCH_HOME_DATA_TEMP,
                                     (int32_t)(t >= 0.0f ? t + 0.5f : t - 0.5f));
            break;
        }

        case EVT_SERVICE_POWER_DATA:
        {
            /* event_data 指向电源服务最新帧: 只读. 电压→百分比在本层换算 ——
               页面只负责画, 不该知道电池的放电特性 */
            Power_Data_t *p_pwr = (Power_Data_t *)p_event->event_data;

            if (0U == s_chg_on)
            {
                s_bat_disp = battery_mv_to_percent(p_pwr->voltage_mv);
            }
            else
            {
                /* ★充电时端电压被钉在 4.2V 附近, 换算出来恒 100%, 没有信息★ ——
                   改成从插上那一刻的值, 按 BAT_CHARGE_FULL_MS 线性爬到 100%.
                   ★先把 elapsed 钳到 FULL_MS 再乘★: 不钳的话充够 12 小时以上乘 100
                   会溢出 uint32; 钳住既防溢出, 又顺带把结果封顶在 100 */
                uint32_t elapsed = osKernelGetTickCount() - s_chg_tick0;

                if (elapsed > BAT_CHARGE_FULL_MS)
                {
                    elapsed = BAT_CHARGE_FULL_MS;
                }
                s_bat_disp = s_chg_pct0 +
                             (int32_t)(((uint32_t)(100 - s_chg_pct0) * elapsed) / BAT_CHARGE_FULL_MS);
            }

            watch_page_home_set_data(WATCH_HOME_DATA_BATTERY, s_bat_disp);
            break;
        }

        case EVT_SERVICE_POWER_CHARGING:
        {
            /* 充电状态按值塞在 event_flags 里(1=充电中), 见 cywatch_service_power.h.
               本事件只在翻转时发, 所以下面这段 0→1 每个充电周期只走一次 */
            uint8_t on = (uint8_t)(0U != p_event->event_flags);

            if (0U != on && 0U == s_chg_on)
            {
                /* 记冻结点: 爬升的起点就是刚才显示的那个值 —— 所以从 90% 插上约 6
                   分钟见顶, 从 20% 插上才真的爬满一小时 */
                s_chg_pct0  = s_bat_disp;
                s_chg_tick0 = osKernelGetTickCount();
            }
            s_chg_on = on;

            watch_page_home_set_data(WATCH_HOME_DATA_CHARGING, (int32_t)on);
            break;
        }

        case EVT_SERVICE_UIHOME_SWITCH_PAGE:
        {
            app_ui_home_switch_page(p_event);
            return 1;
            break;
        }

        case EVT_SERVICE_KEY_PRESS:
        {
            return key_handler(p_event);
            break;
        }

        default:
            break;
    }

    return 0;
}

/******************************************************************************
 * @name    app_ui_home_key
 * @brief   按键处理机: 先按键值分派, 每档里再按手势分派(具体动作待填)
 * @param   event[in] EVT_SERVICE_KEY_PRESS 事件
 *
 * @return  恒 1(消费): 分发到本页即截断, 排在后面的页面收不到这次按键
 *
 * @note    ★同一时刻只有一个 APP 页处于使能态★(切页时由 app_ui_home_switch_page
 *          使能目标页、失能自己), 按键因此天然只落到当前那一页; 五页都返回 1 是
 *          防止同一次按键被重复处理, 别改回 0
 * @note    事件形状见 cywatch_service_key.h: event_flags = 键值(1/2/3),
 *          event_data = 手势(按值塞在指针里, service_key_action_t)
 * @note    本函数跑在 appcore 任务, LVGL 无锁 —— 要动控件只能走
 *          lv_watch_page_home.h 那组零依赖的口, 不能在这里直接碰对象树
 *****************************************************************************/
static int8_t key_handler(easyapp_event_t *event)
{
    uint32_t             key    = event->event_flags;
    service_key_action_t action = (service_key_action_t)(uint32_t)event->event_data;

    switch (key)
    {
        case 1U: /* 键1 */
            switch (action)
            {
                case SERVICE_KEY_ACTION_CLICK:
                    log_printf("APP key1: page=HOME is_front=%d\r\n",
                               easyapp_page_flag_front == &easyapp_page_flag_ui_home);
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
 * @name    app_ui_home_switch_page
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
 * @note    拓扑(见 lv_watch_page.h): 下滑→菜单, 右滑→心率, 左滑→OTA; 上滑无去处
 *****************************************************************************/
static void app_ui_home_switch_page(easyapp_event_t *event)
{
    switch ((app_page_dir_e)event->event_flags)
    {
        case APP_PAGE_TO_BOTTOM: /* 4: 下滑 → 菜单页 */
            easyapp_page_enable(&easyapp_page_flag_ui_menu);
            easyapp_page_to_front(&easyapp_page_flag_ui_menu);
            easyapp_page_disable(&easyapp_page_flag_ui_home);
            break;

        case APP_PAGE_TO_RIGHT: /* 3: 右滑 → 心率页 */
            easyapp_page_enable(&easyapp_page_flag_ui_heart);
            easyapp_page_to_front(&easyapp_page_flag_ui_heart);
            easyapp_page_disable(&easyapp_page_flag_ui_home);
            break;

        case APP_PAGE_TO_LEFT: /* 5: 左滑 → OTA 页 */
            easyapp_page_enable(&easyapp_page_flag_ui_ota);
            easyapp_page_to_front(&easyapp_page_flag_ui_ota);
            easyapp_page_disable(&easyapp_page_flag_ui_home);
            break;

        default: /* 上滑: LVGL 侧没绑页面 */
            break;
    }
}
