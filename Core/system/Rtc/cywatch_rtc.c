/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_rtc.c
 *
 * @par dependencies
 * - cywatch_rtc.h
 * - stddef.h
 * - stm32f4xx_hal.h (HAL 的 RTC 驱动与 RCC/PWR; 需要 hal_conf.h 里把
 *   HAL_RTC_MODULE_ENABLED 打开, 否则 RTC_HandleTypeDef 根本不定义)
 *
 * @author	zw1194
 *
 * @brief 系统时间源的实现: STM32 的 RTC 外设(时钟源 = LSE 32.768kHz) +
 *        取/设接口.
 *
 * Processing flow:
 *
 * call directly.
 *
 * @version V2.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 时间的真源是硬件 RTC 的日历寄存器, 本文件**不持有任何"当前时间"缓存**
 *       —— 每次取都真的去读外设. 这样就不存在"缓存和硬件谁对"的问题, 代价是
 *       cywatch_rtc_get() 不可重入(见它的说明).
 *
 * @note 本文件从 system/Fatfs/port/cywatch_fatfs_os.c 搬来(原 static DWORD
 *       s_fattime + storage_fatfs_set_time()). 编码与原实现逐位一致, 只把
 *       归属从"FatFs 的 OS 粘合层"挪到"系统时间源", 并去掉对 ff.h 的依赖
 *       (原来的 DWORD 换成 uint32_t, 缺省年份不再取自 ffconf.h).
 *
 * @note V2(2026-09-18) 把存储从"一个 RAM 变量"换成硬件 RTC: 接口一行没改, 但
 *       get_packed() 不再原子、set() 会真的写外设、init() 的返回类型从 void
 *       变成 int8_t 并改变语义(从"重置为缺省"变成"拉起外设").
 *       对照物见同目录的 cywatch_rtc.c.bak_ram_v1.
 *****************************************************************************/
#include "cywatch_rtc.h"

#include <stddef.h>
#include <stm32f4xx_hal.h>

/***********************************Defines************************************/
/* DOS/FAT 编码的年份基准: 位域里存的是"年份 - 1980".
   @note 这是 **FAT 文件系统格式**的纪元, 与硬件 RTC 的解释基准
         (CYWATCH_RTC_YEAR_HW_BASE) 是两件事, 别合并 */
#define CYWATCH_RTC_YEAR_BASE        (1980U)

/* 硬件 RTC 的年份基准: RTC_DR 只存两位年(0~99), 由固件决定怎么解释成世纪.
   @note 取 2000 而不是复用 CYWATCH_RTC_YEAR_BASE(1980) 的理由:
         1) 1980 属于 FAT 格式, 挂在密码学上无关的硬件时钟上会让"这个常量
            到底在说什么"变得靠猜;
         2) 2000~2099 与 ST 的 HAL 示例/CubeMX 惯例一致, 用调试器直接看
            RTC_DR 时的读数与直觉相符;
         3) 两个基准各占一处, 将来要改世纪策略只动一个, 不会连带 FAT 编码 */
#define CYWATCH_RTC_YEAR_HW_BASE     (2000U)
#define CYWATCH_RTC_YEAR_HW_MAX      (2099U)

/* 缺省年份 = 2025.
   @note 这个数原本取自 FatFs 的 ffconf.h(FF_NORTC_YEAR). 本模块不再依赖 FatFs
         头, 所以写死在这里. 它的含义只是"一个看起来正常的日期", 不是配置项,
         所以将来改 ffconf.h 的 FF_NORTC_YEAR 时这里**不需要**跟.
   @note 现在它同时是"硬件 RTC 首次上电的种入年份", 所以它必须落在
         CYWATCH_RTC_YEAR_HW_BASE..CYWATCH_RTC_YEAR_HW_MAX 之内 */
#define CYWATCH_RTC_DEFAULT_YEAR     (2025U)

/* 把"年月日时分秒"编码成 DOS/FAT 时间戳:
   bit31-25 年(-1980) | bit24-21 月 | bit20-16 日 | bit15-11 时 | bit10-5 分 | bit4-0 秒/2

   @note 秒只有 5 位, 所以精度是 2 秒 —— 这是 FAT 文件系统的格式限制, 不是
         本模块的取舍
   @note V2 起本模块**不再用这个格式存时间**: 它只在 cywatch_rtc_get_packed()
         的出口用一次, 把硬件读回来的一组值转成 FatFs 要的 32 位. 位域定义与
         V1 逐位一致, 没有动 */
#define CYWATCH_RTC_TIME_PACK(y, mo, d, h, mi, s)                        \
    ( ((((uint32_t)(y) - CYWATCH_RTC_YEAR_BASE)) & 0x7FU) << 25          \
    | (((uint32_t)(mo) & 0x0FU) << 21)                                   \
    | (((uint32_t)(d)  & 0x1FU) << 16)                                   \
    | (((uint32_t)(h)  & 0x1FU) << 11)                                   \
    | (((uint32_t)(mi) & 0x3FU) <<  5)                                   \
    | (((uint32_t)(s)  & 0x3EU) >>  1) )

/* 缺省时间 = 缺省年 1 月 1 日 00:00:00.
   @note 有两处用途, 别删: ① cywatch_rtc_init() 首次上电时的种入值;
         ② cywatch_rtc_get_packed() 读硬件失败时的回退值 */
#define CYWATCH_RTC_TIME_DEFAULT     \
    (CYWATCH_RTC_TIME_PACK(CYWATCH_RTC_DEFAULT_YEAR, 1U, 1U, 0U, 0U, 0U))

/* RTC 的预分频: (AsynchPrediv+1)*(SynchPrediv+1) 必须等于 LSE 的 32768Hz,
   商才是 1Hz. (127+1)*(255+1) = 128*256 = 32768 ✓
   @note 这也正好是 RTC_PRER 的复位值 0x007F00FF, 不是巧合 */
#define CYWATCH_RTC_ASYNCH_PREDIV    (127U)
#define CYWATCH_RTC_SYNCH_PREDIV     (255U)
/***********************************Defines************************************/

/********************************* 内部状态 ***********************************/
/* RTC 句柄. 只在本文件用, 所以 static.
   @note 零初始化即 State = HAL_RTC_STATE_RESET, Lock = HAL_UNLOCKED(= 0),
         两个值都正是 HAL 期望的初值 —— 尤其 Lock, 热复位那条路会跳过
         HAL_RTC_Init()(它才是平时设 Lock 的地方), 靠零初始化顶上 */
static RTC_HandleTypeDef s_rtc_handle;

/* 0 = cywatch_rtc_init() 没成功(或还没调), 1 = RTC 可用.
   @note 为什么必须有: init 失败后如果调用方选择"打印并继续", RTC 是**没有
         时钟**的, 此时 HAL_RTC_SetTime 会在 RTC_EnterInitMode 里白等 1000ms
         再返回 HAL_ERROR. 用这个标志把"没起来"提前挡掉, 让失败快速可见 */
static uint8_t s_rtc_ready = 0U;
/********************************* 内部状态 ***********************************/

/********************************* 内部函数 ***********************************/

/******************************************************************************
 * @name    cywatch_rtc_weekday
 * @brief   用 Zeller 同余算某一天是星期几(RTC_DR 的 WDU 字段要这个值)
 * @param   year[in]  完整年份(2000~2099, 与硬件 RTC 的年份范围一致)
 * @param   month[in] 月(1~12)
 * @param   day[in]   日(1~31)
 *
 * @return  1~7, 1=周一 ... 7=周日(与 HAL 的 RTC_WEEKDAY_MONDAY..SUNDAY 对齐)
 *
 * @note    这个值只被写进 RTC_DR 的 bit15:13(WDU), **不参与硬件走时** ——
 *          RTC 的日历是一条 BCD 计数器链, 跨月/跨年由它自己进位, 跟 WDU 无关.
 *          所以这里算错了顶多是"星期读出来不对", 时间不会错. 也正因为如此,
 *          本模块不需要引入闰年表之类的日历算法
 *
 * @note    只对公历有效. 1、2 月按 Zeller 的惯例当作上一年的 13、14 月处理,
 *          这样闰年规则可以统一写在年份除 4 上, 不用为 1、2 月开特例
 *
 * @note    自检: 2025-01-01 → 3(周三), 2026-09-18 → 5(周五)
 *****************************************************************************/
static uint8_t cywatch_rtc_weekday(uint16_t year, uint8_t month, uint8_t day)
{
    uint32_t y = (uint32_t)year;
    uint32_t m = (uint32_t)month;
    uint32_t k;
    uint32_t j;
    uint32_t h;

    if (3U > m)
    {
        m += 12U;       /* 1、2 月当作上一年的 13、14 月 */
        y -= 1U;
    }

    k = y % 100U;       /* 年份在世纪内的部分 K */
    j = y / 100U;       /* 世纪数 J */

    /* h = (日 + floor(13(m+1)/5) + K + floor(K/4) + floor(J/4) + 5J) mod 7
       取值含义: 0=周六 1=周日 2=周一 3=周二 4=周三 5=周四 6=周五 */
    h = ((uint32_t)day + ((13U * (m + 1U)) / 5U) + k + (k / 4U) + (j / 4U) + (5U * j)) % 7U;

    return (uint8_t)(((h + 5U) % 7U) + 1U);   /* 平移成 1=周一 ... 7=周日 */
}

/********************************* 内部函数 ***********************************/

/********************************* 对外接口 ***********************************/

/******************************************************************************
 * @name    cywatch_rtc_init
 * @brief   把 STM32 的 RTC 外设拉起来(LSE 起振 + 选 RTC 时钟源 + 上电初始化),
 *          并保证"第一次上电"的日历初值是 2025-01-01 00:00:00
 * @param   无
 *
 * @return  0  success
 *         -1  LSE 起振失败(HAL_RCC_OscConfig 返回非 HAL_OK; 最常见的原因是
 *             板上没有 32.768kHz 晶振 → 等满 5000ms 超时)
 *         -2  RTC 时钟源不是 LSE(后备域没复位前 RTCSEL 撬不动)
 *         -3  HAL_RTC_Init 失败(等 INITF / 等 RSF 超时)
 *         -4  种入缺省时间失败(见 cywatch_rtc_set 的返回码, 本函数不细分)
 *         -5  回读校验不一致(硬件没接受写入的日期/时间)
 *
 * @note    必须在 HAL_Init() 之后调用: HAL 内部等 LSERDY/INITF/RSF 的超时
 *          都以 HAL_GetTick() 为时基
 *
 * @note    与 FreeRTOS 无关(纯寄存器与短时自旋, 没有 osDelay), 所以可以在
 *          osKernelStart() 之前、调度器还没跑的时候调 —— main() 就是这么用的
 *
 * @note    幂等, 且**跨热复位非破坏性**: 用 RTC_ISR_INITS 判断要不要种初值.
 *          INITS 是硬件自己的"日历已经有过有效值"标志, 只有备份域复位才清零.
 *          热复位时它已经是 1, 本函数就只走时钟那几步, 一个日历寄存器都不碰
 *          —— 时间继续走, 不会被刷回 2025-01-01
 *
 * @note    为什么 LSE / RTCSEL / RTCEN 这几步可以重复调: 它们写的是**备份域**
 *          里的配置位, 热复位(VDD 不掉)时备份域不复位, 值本来就是对的;
 *          __HAL_RCC_LSE_CONFIG 与 __HAL_RCC_RTC_CONFIG 都是"或上位"操作,
 *          写同一个值等于什么都没写
 *
 * @warning 绝对不要用 __HAL_RCC_BACKUPRESET_FORCE() 来"先清干净再配":
 *          它复位整个备份域, 会清掉日历与 BKP0R~BKP19R, 时间直接回 2025-01-01
 *
 * @note    本函数**不自旋死等**: LSE 起不来就把错误码交回调用方, 由 main()
 *          决定是停机(本工程的选择)还是降级继续
 *****************************************************************************/
int8_t cywatch_rtc_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    cywatch_rtc_time_t seed;
    cywatch_rtc_time_t back;
    uint32_t           rtcsel;
    uint8_t            need_seed;

    /* ---- 步骤 1: 打开 PWR 时钟 + 解锁备份域写保护 ----
       @note PWR 时钟 SystemClock_Config() 里已经开过, 这里再来一次是幂等的.
             显式写出来是因为下面 RTCSEL/RTCEN 都要靠 PWR_CR.DBP=1 才写得进去,
             不要依赖"HAL_RCC_OscConfig 内部顺手开过"这种隐式前提(它确实会开,
             但那是它的实现细节) */
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();          /* PWR_CR.DBP = 1, 幂等 */

    /* ---- 步骤 2: 只启动 LSE, 等 LSERDY ----
       @note 只置 OscillatorType 的 LSE 位 → HSE/HSI/LSI 一律不动;
             PLL.PLLState 保持零初始化(= RCC_PLL_NONE) → HAL 完全跳过 PLL 段.
             这是**必须**的: 若照抄 SystemClock_Config 的 PLL 字段, 而此刻
             SYSCLK 已经是 PLL, HAL 会逐字段比对 PLLCFGR, 任一不符就
             return HAL_ERROR —— 于是"LSE 坏了"和"PLL 参数不符"会共用同一个
             返回值, 现场无法区分
       @note 超时: RCC_LSE_TIMEOUT_VALUE = LSE_STARTUP_TIMEOUT = 5000ms.
             板上没有晶振时就在这里耗满 5 秒然后返回 HAL_TIMEOUT */
    osc.OscillatorType = RCC_OSCILLATORTYPE_LSE;
    osc.LSEState       = RCC_LSE_ON;
    if (HAL_OK != HAL_RCC_OscConfig(&osc))
    {
        return -1;
    }

    /* ---- 步骤 3: 选 RTC 时钟源 = LSE, 并使能 RTC 时钟 ----
       @note RTCSEL 一旦被写过(且 RTCEN=1), 硬件就忽略后续写入, 只有备份域复位
             或上电复位才解锁. 所以这里**先读再决定**:
               - 已经是 LSE  → 什么都不做(热复位的正常路径);
               - 是无时钟    → 写 LSE(首次上电路径);
               - 是别的源    → 报 -2. 硬写也改不动, 与其假装成功不如让调用方知道 */
    rtcsel = __HAL_RCC_GET_RTC_SOURCE();
    if (RCC_RTCCLKSOURCE_LSE != rtcsel)
    {
        if (RCC_RTCCLKSOURCE_NO_CLK != rtcsel)
        {
            return -2;
        }
        __HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSE);
    }
    __HAL_RCC_RTC_ENABLE();

    /* ---- 步骤 4: 判断要不要种初值(必须在上面的 RTCEN 之后读) ----
       @note 顺序不能反: RTCEN=0 时 RTC 的 APB 接口没有时钟, ISR 读出来恒为 0,
             会把"热复位"误判成"首次上电", 于是每次复位都把时间刷回缺省值 ——
             这正是本函数最要防的失败 */
    need_seed = (0U == (RTC->ISR & RTC_ISR_INITS)) ? 1U : 0U;

    s_rtc_handle.Instance = RTC;

    if (0U != need_seed)
    {
        /* ---- 4a. 首次上电: 配 RTC 参数, 然后种缺省时间 ---- */
        s_rtc_handle.Init.HourFormat     = RTC_HOURFORMAT_24;
        s_rtc_handle.Init.AsynchPrediv   = CYWATCH_RTC_ASYNCH_PREDIV;
        s_rtc_handle.Init.SynchPrediv    = CYWATCH_RTC_SYNCH_PREDIV;
        s_rtc_handle.Init.OutPut         = RTC_OUTPUT_DISABLE;  /* ★ 见下 */
        s_rtc_handle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
        s_rtc_handle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;

        /* @warning ★ OutPut 必须是 RTC_OUTPUT_DISABLE ★
               F4 的 RTC 闹钟/校准输出是复用功能 RTC_AF1, 落在 **PC13** 上, 而
               PC13 在本工程里是 LCD_RST(Core/Inc/main.h 定义; 本工程没有
               MX_GPIO_Init, 那个脚由显示 adapter 自己配成输出推挽). 一旦把
               OutPut 设成 RTC_OUTPUT_ALARMA/B, PC13 会被 RTC 抢走, LCD_RST
               失控 → 屏幕复位异常, 而且现象与"屏幕坏了"一模一样 */
        if (HAL_OK != HAL_RTC_Init(&s_rtc_handle))
        {
            return -3;
        }

        seed.year   = CYWATCH_RTC_DEFAULT_YEAR;
        seed.month  = 1U;
        seed.day    = 1U;
        seed.hour   = 0U;
        seed.minute = 0U;
        seed.second = 0U;

        s_rtc_ready = 1U;                /* 先放行, 让 cywatch_rtc_set 能进来 */
        if (0 != cywatch_rtc_set(&seed))
        {
            s_rtc_ready = 0U;
            return -4;
        }
    }
    else
    {
        /* ---- 4b. 热复位: 日历已经有效, 一个寄存器都不写 ----
           @note 这里**故意不调 HAL_RTC_Init()**: 它内部的 RTC_EnterInitMode 会置
                 INIT 位, 而 INIT=1 期间日历的更新被冻结、亚秒计数器归零, 释放
                 INIT 后才继续 —— 也就是每调一次白丢 ≤1 秒. 这是把"每次上电都
                 调一遍 HAL_RTC_Init"当成常规做法的代价, 也是本设计用 INITS
                 分岔的真正理由(不是省几条指令).
                 备份域里的 CR/PRER 本来就是上次配好的值; 连 PRER 的复位值
                 0x007F00FF 都正好是 LSE 下的 (127,255) = 1Hz */
        s_rtc_handle.State = HAL_RTC_STATE_READY;   /* 与 HAL_RTC_Init 收尾一致 */
        s_rtc_ready        = 1U;
    }

    /* ---- 步骤 5: 种了值就回读一次, 确认硬件真的收下了 ----
       @note 这一步是**故意留的自检**, 抓的是"写进去了但值是错的"这一类静默故障:
             RTC_FORMAT_BIN 写成 BCD、忘了 SetDate 只调 SetTime、WeekDay 越界、
             日期字段错一位 —— 全都不会报错, 只会让时间静默走偏. 根源是
             USE_FULL_ASSERT 没打开, HAL 里的 assert_param 全是空操作.
             嫌代码多可以整段删掉, 删了就少了这层保险
       @note 只比较"分"及以上: 秒在读之前可能已经被硬件走掉 1 拍, 所以放宽到
             0/1/2 秒(缺省时间的秒域恒为 0) */
    if (0U != need_seed)
    {
        if ((0 != cywatch_rtc_get(&back))                            ||
            (back.year   != seed.year)   || (back.month  != seed.month) ||
            (back.day    != seed.day)    || (back.hour   != seed.hour)  ||
            (back.minute != seed.minute) || (2U < back.second))
        {
            s_rtc_ready = 0U;
            return -5;
        }
    }

    return 0;
}

/******************************************************************************
 * @name    cywatch_rtc_set
 * @brief   设置 RTC 的当前时间(时间源预留接口的写数端)
 * @param   p_time[in] 年月日时分秒(本地时间, 无时区概念)
 *
 * @return  0 success
 *         -1 p_time null
 *         -2 字段越界(年份不在 2000~2099 / 月不在 1~12 / 日不在 1~31 /
 *                      时不在 0~23 / 分或秒不在 0~59)
 *         -3 RTC 未初始化(cywatch_rtc_init 没成功)
 *         -4 HAL_RTC_SetTime 失败
 *         -5 HAL_RTC_SetDate 失败
 *
 * @note    本函数**会写硬件**(APB 寄存器, 不是 I2C/SPI), 耗时几十微秒; 可在
 *          任务上下文随时调, 不可在 ISR 里调(保持"ISR 不做业务"的纪律)
 *
 * @note    写保护不用调用方管: HAL_RTC_SetTime/SetDate 内部各自做了
 *          __HAL_RTC_WRITEPROTECTION_DISABLE/ENABLE 与 EnterInitMode/ExitInitMode
 *
 * @note    只校验字段范围, **不校验日期是否真实存在**(比如 2 月 31 日会被接受).
 *          这与原实现一致: 编码格式本身表达不了"非法日期", 而多做一层日历校验
 *          就要引入闰年表. 时间源应当只送来真实存在的日期
 *
 * @note    年份上限 2099 不是本模块的取舍, 而是硬件 RTC_DR 只存两位年(0~99).
 *          超范围的年份**必须**在这里挡掉, 否则会被静默截断成另一个世纪
 *****************************************************************************/
int8_t cywatch_rtc_set(const cywatch_rtc_time_t *p_time)
{
    RTC_TimeTypeDef t = {0};
    RTC_DateTypeDef d = {0};

    if (NULL == p_time)
    {
        return -1;
    }

    if (CYWATCH_RTC_YEAR_HW_BASE > p_time->year ||
        CYWATCH_RTC_YEAR_HW_MAX  < p_time->year ||
        12U < p_time->month  || 1U > p_time->month  ||
        31U < p_time->day    || 1U > p_time->day    ||
        23U < p_time->hour   ||
        59U < p_time->minute ||
        59U < p_time->second)
    {
        return -2;
    }

    if (0U == s_rtc_ready)
    {
        return -3;
    }

    /* 时分秒: RTC_FORMAT_BIN 让 HAL 替我们做十进制 → BCD 的转换 */
    t.Hours          = p_time->hour;
    t.Minutes        = p_time->minute;
    t.Seconds        = p_time->second;
    t.TimeFormat     = RTC_HOURFORMAT12_AM;      /* 24 小时制下 HAL 会把它清 0 */
    t.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;  /* 与零初始化同值, 显式写出 */
    t.StoreOperation = RTC_STOREOPERATION_RESET; /* 是为了表明"夏令时那套我们不用" */
    t.SubSeconds     = 0U;
    t.SecondFraction = 0U;

    if (HAL_OK != HAL_RTC_SetTime(&s_rtc_handle, &t, RTC_FORMAT_BIN))
    {
        return -4;
    }

    /* 年月日: WeekDay 必须给, HAL 里 assert_param(IS_RTC_WEEKDAY(...)) 要求 1~7 */
    d.WeekDay = cywatch_rtc_weekday(p_time->year, p_time->month, p_time->day);
    d.Month   = p_time->month;
    d.Date    = p_time->day;
    d.Year    = (uint8_t)(p_time->year - CYWATCH_RTC_YEAR_HW_BASE);   /* 0..99 */

    if (HAL_OK != HAL_RTC_SetDate(&s_rtc_handle, &d, RTC_FORMAT_BIN))
    {
        return -5;
    }

    return 0;
}

/******************************************************************************
 * @name    cywatch_rtc_get
 * @brief   取 RTC 的当前时间
 * @param   p_time[out] 年月日时分秒
 *
 * @return  0 success
 *         -1 p_time null
 *         -2 RTC 未初始化(cywatch_rtc_init 没成功)
 *         -3 HAL_RTC_GetTime 失败
 *         -4 HAL_RTC_GetDate 失败
 *
 * @note    ★ GetTime 与 GetDate 是**一对**, 顺序不能反也不能拆开 ★
 *          HAL_RTC_GetTime 读 TR 会把日历影子寄存器闩住(保证年月日时分秒是同
 *          一瞬的值), 由紧接着的 HAL_RTC_GetDate 读 DR 来解锁. 这是 HAL 函数
 *          注释里写死的约束. 只调一个 → 留下过期的闩锁状态, 下一次读到的可能
 *          是上一次的快照
 *
 * @note    年份还原: HAL 给的是两位年(0~99), 本模块按 CYWATCH_RTC_YEAR_HW_BASE
 *          解释成 2000~2099. 这个基准与 FAT 编码用的 CYWATCH_RTC_YEAR_BASE(1980)
 *          是**两个不同的东西**, 别合并
 *
 * @note    本函数**不可重入**: GetTime/GetDate 这一对中间若插进另一个调用者的
 *          GetTime, 两者的闩锁会串在一起, 有概率拿到"秒来自新采样、日期来自旧
 *          采样"的组合(只在跨秒/跨日的那一瞬间发生). 目前全工程的调用者都在
 *          main-loop 里, 不存在并发; 将来若多个任务都要读, 调用方需要自备互斥
 *
 * @note    错误码 -3/-4 是**纯防御**: 本 HAL 版本的 HAL_RTC_GetTime 与
 *          HAL_RTC_GetDate 都没有失败路径(无条件 return HAL_OK), 永远不会触发.
 *          留着是为了将来换 HAL 版本时不至于没有出口
 *****************************************************************************/
int8_t cywatch_rtc_get(cywatch_rtc_time_t *p_time)
{
    RTC_TimeTypeDef t = {0};
    RTC_DateTypeDef d = {0};

    if (NULL == p_time)
    {
        return -1;
    }

    if (0U == s_rtc_ready)
    {
        return -2;
    }

    if (HAL_OK != HAL_RTC_GetTime(&s_rtc_handle, &t, RTC_FORMAT_BIN))
    {
        return -3;
    }

    if (HAL_OK != HAL_RTC_GetDate(&s_rtc_handle, &d, RTC_FORMAT_BIN))
    {
        return -4;
    }

    p_time->year   = (uint16_t)(CYWATCH_RTC_YEAR_HW_BASE + (uint16_t)d.Year);
    p_time->month  = d.Month;
    p_time->day    = d.Date;
    p_time->hour   = t.Hours;
    p_time->minute = t.Minutes;
    p_time->second = t.Seconds;

    return 0;
}

/******************************************************************************
 * @name    cywatch_rtc_get_packed
 * @brief   取当前的 DOS/FAT 编码时间戳
 * @param   无
 *
 * @return  打包时间戳(位域说明见 CYWATCH_RTC_TIME_PACK); **读失败时返回缺省值
 *          (2025-01-01 00:00:00 的编码)**
 *
 * @note    FatFs 的 get_fattime() 就是转发本函数 —— ff.c 建/改文件时回调它.
 *          本函数刻意**不返回错误**: FatFs 那个回调的契约就是一个 DWORD, 没有
 *          错误通道. 给一个合法的旧时间戳, 比给 0(FAT 里代表非法/1980)更不容易
 *          把磁盘工具和上层逻辑带偏
 *
 * @note    ★ 与 V1 不同: 现在**不再**是"一次对齐 32 位内存读" ★ 本函数会真的去
 *          读 RTC 外设(走 cywatch_rtc_get 的 GetTime+GetDate 一对), 再重新打包.
 *          所以:
 *            - 返回的值仍然是一致的一组(年月日时分秒来自同一次采样, 打包在局部
 *              变量里完成, 不会出现"年是新值秒是旧值"这种半新半旧);
 *            - 但**本函数不可重入**, 理由与 cywatch_rtc_get 相同
 *
 * @note    调用方若在 system/Fatfs/port/cywatch_fatfs_os.c 里转发本函数, 那句
 *          "转发是安全的(不需要临界区): 一次对齐 32 位读, Cortex-M4 上原子"
 *          的注释已随 V2 失效, 恢复存储链时要一并改
 *
 * @note    秒被截断成 2 秒粒度: 这是 FAT 格式本身的限制(只有 5 位秒域), 不是
 *          本模块的取舍. cywatch_rtc_get() 拿到的秒是精确到 1 秒的
 *
 * @note    返回 uint32_t 而不是 FatFs 的 DWORD: 本模块不依赖 ff.h. 两者在
 *          STM32 上都是 32 位无符号, FatFs 侧做一次显式转换即可
 *****************************************************************************/
uint32_t cywatch_rtc_get_packed(void)
{
    cywatch_rtc_time_t now;

    if (0 != cywatch_rtc_get(&now))
    {
        /* 读不到就给缺省值(理由见 @return).
           @note 这里**不要**返回 0: 0 在 DOS/FAT 编码里等于 1980-00-00
                 00:00:00, 是个非法日期, 比一个"看起来正常的旧时间"更容易给
                 上层制造麻烦 */
        return CYWATCH_RTC_TIME_DEFAULT;
    }

    return CYWATCH_RTC_TIME_PACK(now.year, now.month, now.day,
                                 now.hour, now.minute, now.second);
}

/********************************* 对外接口 ***********************************/
