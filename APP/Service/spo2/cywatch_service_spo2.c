/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_service_spo2.c
 *
 * @par dependencies
 * - cywatch_service_spo2.h
 * - lv_watch_page_spo2.h
 * - cywatch_service_HeartRate.h
 * - cmsis_os2.h
 *
 * @author  zw1194
 *
 * @brief 血氧测量服务(消费者): 按需创建的任务, 阻塞等 APP 层投来的血氧帧, 推
 *        IDLE/MEASURING/DONE 三态, 并把每帧快照投给血氧页.
 *
 * Processing flow:
 *
 * service_spo2_start() ──> osThreadNew ──> service_spo2_run:
 *
 *   ┌─ 一轮测量 ────────────────────────────────────────────────┐
 *   │ osMessageQueueReset + 记 t0 + 投 MEASURING                │
 *   │   ↻ osMessageQueueGet(100ms)                              │
 *   │      ├ 收到有效帧 → hold_ms += 500, 攒够 5000 才认这个数  │
 *   │      ├ 收到无效帧 → hold_ms 归零                          │
 *   │      ├ 超时       → 什么都不动(见 @note 超时不算"没数据") │
 *   │      └ 投 MEASURING(remain,value,finger)                  │
 *   │   直到 停止请求 或 满 60 秒 → 投终态 → break              │
 *   │                                                           │
 *   └─ s_restart ? 再开一轮 : 退出 ─────────────────────────────┘
 *
 *   s_alive = 0 → osThreadExit()
 *
 * 心率侧的模式开关: 心率服务默认停在 HR 模式(只跑心率, 血氧报 0). start() 会调
 *   service_heartrate_set_spo2_enable(1) 让它切到 SpO2 模式(RED+IR 双路)并喂血氧
 *   算法, 退出时调 (0) 关回去 —— 不测血氧时就不白烧 RED LED 和那一半 FIFO 读出量.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note ★超时 != 没数据★ 本任务的队列超时只有 100ms, 而心率帧是 500ms 一帧 ——
 *       每 5 次 get 里有 4 次必然超时. 所以超时那一轮**只**用来检查停止请求和到点,
 *       绝不能拿本地残留的 sample 去动 hold/value(会重复计数), 也不能因此清 hold
 *       (那 hold 永远攒不满). 而且 osMessageQueueGet 超时时不写 sample, 里面还是
 *       上一帧的内容, 更碰不得.
 *
 * @note ★退出顺序写死, 不能颠倒★: 投终态 → s_alive = 0 → osThreadExit.
 *       s_alive 置 0 之后不得再写任何共享量 —— 那一刻新任务可能已经被 start()
 *       建起来, 两边会抢同一份状态.
 *
 * @note ★s_alive 由 start() 置 1, 不在这里置★ osThreadNew 返回到任务真正跑起来
 *       之间有个窗口, 若在任务入口才置 1, 窗口内再按按钮会**建出第二个任务**.
 *****************************************************************************/
#include "cywatch_service_spo2.h"
#include "lv_watch_page_spo2.h"
#include "cywatch_service_HeartRate.h"
#include "cmsis_os2.h"

/***********************************Defines************************************/
/* 一轮测量的时长上限(秒) */
#define SERVICE_SPO2_MEASURE_S   (60U)

/* 连续读到有效值多久才认这个数字(毫秒). ★这是为了挡 SpO2 算法的直流台阶瞬态伪值★
   —— ServiceFunction/SpO2.c 缺"直流收敛"判据, 手指刚贴合时那几个 80/82 是伪值,
   详见 lv_watch_page_spo2.c 文件头.
   ★按毫秒累计而不是数帧★: 心率事件是 2Hz, 数帧的话这个 5s 会被砍成 2.5s */
#define SERVICE_SPO2_HOLD_MS     (5000U)

/* 一帧的有效期, 用来把"收到几帧"折算成毫秒. 与心率服务的
   SERVICE_HEARTRATE_PUBLISH_MS(500) 对应, 那边改了这里要跟着改 */
#define SERVICE_SPO2_FRAME_MS    (500U)

/* 阻塞超时. ★它就是停止请求与到点检查的最坏响应延迟★ —— 100ms 时按钮按下到
   画面定格最多等这么久. 别为了省电调大: 那是用户能感觉到的延迟 */
#define SERVICE_SPO2_POLL_MS     (100U)

/* 队列深度: 生产 2Hz, 消费最快 10Hz, 只有消费者已经没了才可能满 */
#define SERVICE_SPO2_QUEUE_LEN   (4U)

/* 有效血氧值的判据. ★不能只看 != 0★: 算法输出不保证范围, 而 float→uint8_t 越界
   转换是 UB(画面可能出乱码). 卡在 [50,100] 既滤掉伪值, 又顺带把转换范围管住了 */
#define SERVICE_SPO2_MIN_PCT     (50.0f)
#define SERVICE_SPO2_MAX_PCT     (100.0f)

/* 停止请求的三种取值 */
#define SERVICE_SPO2_STOP_NONE   (0U)
#define SERVICE_SPO2_STOP_FINISH (1U) /* 提前结束: 定格 → DONE */
#define SERVICE_SPO2_STOP_CANCEL (2U) /* 取消(切页): 回初始 → IDLE */
/***********************************Defines************************************/

/**********************************Variables***********************************/
/* 一帧里本服务要的那部分. ★按值投进队列, 不存指针★ —— 心率服务那份静态帧会在
   下一个发布周期被覆盖 */
typedef struct
{
    float   spo2_percent;
    uint8_t finger_on;
} spo2_sample_t;

static osMessageQueueId_t s_q;       /* 惰性建一次, 建完不删(见 service_spo2_start) */
static volatile uint8_t   s_stop;    /* SERVICE_SPO2_STOP_xxx; appcore 写, 任务读 */
static volatile uint8_t   s_alive;   /* start() 置 1, 任务退出前置 0 */
static volatile uint8_t   s_restart; /* 任务正在退出的窗口里又按了按钮 → 排队重启 */

/* 任务属性. 优先级压到 Low: 它只是等数据 + 算几个数, 不该跟 LVGL / 心率抢.
   栈 1024 够用(对齐心率/按键任务), 前提是**本文件不出现 printf 家族** */
static const osThreadAttr_t s_spo2_attr =
{
    .name       = "spo2",
    .attr_bits  = 0U,
    .cb_mem     = NULL,
    .cb_size    = 0U,
    .stack_mem  = NULL,
    .stack_size = 1024U,
    .priority   = osPriorityLow,
};
/**********************************Variables***********************************/

/******************************************************************************
 * @name    service_spo2_run
 * @brief   测量任务: 阻塞等帧 → 推状态 → 投快照; 到点或收到停止请求后自退
 * @param   p_arg[in] 未使用
 *
 * @return  无(退出走 osThreadExit; 从线程函数 return 是未定义行为)
 *
 * @note    两层循环: 外层 = 一轮测量, 内层 = 帧循环. "排队重启"因此不用 goto
 *****************************************************************************/
static void service_spo2_run(void *p_arg)
{
    spo2_sample_t sample;
    osStatus_t    st;
    uint32_t      t0;      /* 本轮起点 tick */
    uint32_t      hold_ms; /* 连续读到有效值的毫秒数 */
    uint32_t      elapsed_s;
    uint32_t      remain_s;
    uint8_t       finger;  /* 最近一帧的手指状态 */
    uint8_t       value;   /* 最近一次可信读数, 0 = 还没有 */

    (void)p_arg;

    while (1U)
    {
        /* 本轮起点. ★必须冲队列★: 队列是跨轮复用的, 上一轮残留的帧会把新一轮的
           hold 和倒计时带歪 */
        osMessageQueueReset(s_q);
        t0      = osKernelGetTickCount();
        hold_ms = 0U;
        finger  = 0U;
        value   = 0U;

        /* 起手先投一帧, 免得"按钮按下"到"第一帧数据"之间画面还是旧的 */
        watch_page_spo2_post(WATCH_SPO2_MEASURING, (uint8_t)SERVICE_SPO2_MEASURE_S, 0U, 0U);

        while (1U)
        {
            st = osMessageQueueGet(s_q, &sample, NULL, SERVICE_SPO2_POLL_MS);

            elapsed_s = (osKernelGetTickCount() - t0) / 1000U;

            /* ---- 停止请求(提前结束 / 切页取消) ---- */
            if (SERVICE_SPO2_STOP_NONE != s_stop)
            {
                if (SERVICE_SPO2_STOP_CANCEL == s_stop)
                {
                    /* ★必须投 IDLE★: 本页还在 PageMem 的 LRU 缓存里, 下次 pf_show
                       会照这份快照重画 —— 不投的话切回来会看到一个冻结的"测量中"
                       倒计时, 而背后已经没有任务了 */
                    watch_page_spo2_post(WATCH_SPO2_IDLE, 0U, 0U, 0U);
                }
                else
                {
                    /* 提前结束: 定格当时的值(还没攒够可信度就当没测到) */
                    watch_page_spo2_post(WATCH_SPO2_DONE, 0U, value, 0U);
                }
                break;
            }

            /* ---- 到点: 与"提前结束"同一种收尾 ---- */
            if (elapsed_s >= SERVICE_SPO2_MEASURE_S)
            {
                watch_page_spo2_post(WATCH_SPO2_DONE, 0U, value, 0U);
                break;
            }

            /* ---- 只有真收到一帧才动 hold/value/finger ---- */
            if (osOK == st)
            {
                finger = sample.finger_on;

                if ((0U != sample.finger_on) &&
                    (sample.spo2_percent >= SERVICE_SPO2_MIN_PCT) &&
                    (sample.spo2_percent <= SERVICE_SPO2_MAX_PCT))
                {
                    hold_ms += SERVICE_SPO2_FRAME_MS;
                    value    = (uint8_t)(sample.spo2_percent + 0.5f);
                }
                else
                {
                    hold_ms = 0U;
                    value   = 0U;
                }
            }

            remain_s = SERVICE_SPO2_MEASURE_S - elapsed_s;

            /* 攒够之前投 0: 由页面显示成 "--", 不冒充结果 */
            watch_page_spo2_post(WATCH_SPO2_MEASURING, (uint8_t)remain_s,
                                 (hold_ms >= SERVICE_SPO2_HOLD_MS) ? value : 0U,
                                 finger);
        }

        /* ---- 本轮结束: 退出窗口期里被按过按钮就接着开一轮 ---- */
        if (0U == s_restart)
        {
            break;
        }
        s_restart = 0U;
        s_stop    = SERVICE_SPO2_STOP_NONE;
    }

    /* ★先关血氧★: 关模式也是"碰共享量", 必须在 s_alive 置 0 之前做 —— 置 0 之后
       新任务可能已经被 start() 建起来, 两边会抢同一份状态.
       关掉心率服务就切回 HR 模式, 省掉 RED LED 的功耗与一半 FIFO 读出量 */
    service_heartrate_set_spo2_enable(0U);

    /* 收尾. ★顺序不能颠倒, s_alive 置 0 之后不许再碰共享量★ */
    s_alive = 0U;
    osThreadExit();
}

/**********************************Functions***********************************/
void service_spo2_start(void)
{
    osThreadId_t h;

    if (0U == s_alive)
    {
        if (NULL == s_q)
        {
            s_q = osMessageQueueNew(SERVICE_SPO2_QUEUE_LEN, sizeof(spo2_sample_t), NULL);
        }

        /* ★建不出队列/任务必须把页面拉回 IDLE★: 32KB 内核堆(fatfs 的一次性任务
           要走 4.6KB), 失败是真会发生的. 不拉回的话页面永远卡在"测量中", 而且
           后续每次 start() 都会走"正在停"那条分支 —— 按钮从此完全失效 */
        if (NULL == s_q)
        {
            watch_page_spo2_post(WATCH_SPO2_IDLE, 0U, 0U, 0U);
            return;
        }

        s_stop    = SERVICE_SPO2_STOP_NONE;
        s_restart = 0U;

        h = osThreadNew(service_spo2_run, NULL, &s_spo2_attr);
        if (NULL == h)
        {
            watch_page_spo2_post(WATCH_SPO2_IDLE, 0U, 0U, 0U);
            return;
        }

        /* ★在这儿置 1, 不在任务入口置★: 关掉 osThreadNew 返回到任务真正跑起来
           之间的窗口, 否则窗口内再按一次会建出第二个任务 */
        s_alive = 1U;

        /* ★让心率服务切到 SpO2 模式★(RED+IR 双路): 不切的话心率帧里
           spo2_percent 恒为 0, 本服务永远等不到可信值.
           只置请求位, 真正的切由心率任务在它自己的循环里做, 最慢一次
           FIFO 满(约 320ms)后生效 */
        service_heartrate_set_spo2_enable(1U);
        return;
    }

    if (SERVICE_SPO2_STOP_NONE == s_stop)
    {
        s_stop = SERVICE_SPO2_STOP_FINISH; /* 测量中再按 = 提前结束 */
    }
    else
    {
        s_restart = 1U; /* 正在退出 → 排队重启, 不建第二个任务 */
    }
}

void service_spo2_finish(void)
{
    if (0U != s_alive)
    {
        s_stop = SERVICE_SPO2_STOP_FINISH;
    }
}

void service_spo2_cancel(void)
{
    /* 没任务在跑时置了也无害: 下次 start() 走"新建"那条路会把 s_stop 清掉 */
    s_stop = SERVICE_SPO2_STOP_CANCEL;
}

void service_spo2_feed(float spo2_percent, uint8_t finger_on)
{
    spo2_sample_t sample;

    /* ★没任务在跑就丢★: 队列可能还没建(NULL), 也可能刚被停掉. 心率事件是 2Hz
       一直在发的, 不守卫的话会往一个没有消费者的队列里灌 */
    if ((0U == s_alive) || (NULL == s_q))
    {
        return;
    }

    sample.spo2_percent = spo2_percent;
    sample.finger_on    = finger_on;

    /* 满就丢, 不等(timeout = 0): 消费端最快 100ms 取一次, 真满了说明消费者已经
       不在, 丢帧是对的. 也绝不能阻塞 —— 调用方是 appcore */
    (void)osMessageQueuePut(s_q, &sample, 0U, 0U);
}
/**********************************Functions***********************************/
