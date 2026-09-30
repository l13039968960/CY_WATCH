/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_fatfs_os.c
 *
 * @par dependencies
 * - ff.h
 * - cywatch_rtc.h      (Core/system/Rtc)
 * - cmsis_os2.h        (MIddleWares/FreeRTOS/CMSIS_RTOS_V2)
 *
 * @author	zw1194
 *
 * @brief FatFs R0.16 的 OS 粘合层: 可重入互斥 + get_fattime() 的转发.
 *
 * Processing flow:
 *
 * FatFs 侧(ff.c) 在 FF_FS_REENTRANT == 1 时回调本文件的 4 个函数:
 *   f_mount  -> ff_mutex_create / ff_mutex_delete   (建/删每卷一把互斥量)
 *   各 API   -> ff_mutex_take / ff_mutex_give       (每次文件操作进出各一次)
 * 时间戳侧: ff.c 建/改文件时调 get_fattime(), 本文件把它转发给系统时间源
 * (system/Rtc/cywatch_rtc.c). **时间值不归本文件管** —— 本文件只做转换.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 为什么不用 ChaN 附带的 ffsystem.c: 那个样例的 OS_TYPE == 4(CMSIS-RTOS)
 *       分支用的是 **CMSIS-RTOS v1** 的 osMutexCreate/osMutexWait/osMutexRelease,
 *       本工程只有 CMSIS-RTOS v2(cmsis_os2.h), 那三个函数不存在; 而它的缺省
 *       OS_TYPE == 0 是 Win32. 所以照它的形状自己实现一份(v2 版),
 *       ffsystem.c 本身不进工程.
 *
 * @note R0.16 需要的回调名是 ff_mutex_create/delete/take/give. 老版本 FatFs 的
 *       ff_req_grant/ff_rel_grant 在 R0.16 已不存在, 不要按老资料写.
 *
 * @note 本文件是**卷锁的唯一咽喉**: ff.c 里所有 API 的互斥最终都落到这两个函数
 *       (take/give). 所以"卷锁到底有没有起作用"这件事只能在这里量 —— 2026-09-13
 *       的一套 3 任务并发测试就是在这两个函数上埋的计数(自然持锁包线 + 注入超时),
 *       量到过"最长持锁 174ms / 余量 5x / take 与 give_ok 配平".
 *       @note 那套测试与其探针**已删除**(见 system/Fatfs/README.md 的"线程安全"
 *             一节). 上面这些数是**当时板上量到的**, 不是估算; 若要复验, 只需在
 *             本文件这两个函数里重新插计数 —— 它们是唯一咽喉这一点没有变.
 *****************************************************************************/
#include "ff.h"

#include "cywatch_rtc.h"
#include "cmsis_os2.h"

/***********************************Defines************************************/
/* 互斥量表项数: FF_VOLUMES 个卷 + 1 个"系统锁".
   @note 系统锁那一项的下标就是 FF_VOLUMES(见 ff.c 的 lock_volume), 只在
         FF_FS_LOCK != 0 时才会被真的创建/使用; 本工程 FF_FS_LOCK == 0,
         所以那一项永远为 NULL, 但表仍按 FF_VOLUMES+1 开, 免得将来打开
         FF_FS_LOCK 时越界 */
#define FATFS_MUTEX_CNT             (FF_VOLUMES + 1)

/* @note 原本这里还有 FATFS_TIME_PACK / FATFS_TIME_DEFAULT_PACK / 年份基准三个宏,
         以及 static DWORD s_fattime. 现在时间值的归属已移到系统层
         (system/Rtc/cywatch_rtc.c), 本文件只剩"把 FatFs 的 get_fattime()
         转发过去"这一件事, 所以那套编码不再需要留在这里 */
/***********************************Defines************************************/

/********************************* 内部状态 ***********************************/
/* 每卷(含系统锁)一把互斥量句柄 */
static osMutexId_t s_mutex[FATFS_MUTEX_CNT];

/* @note 用**普通互斥量**, 不用递归互斥量: 与 ChaN 的样例(ffsystem.c 的
   FreeRTOS/CMSIS 分支)保持一致. 递归量会把"同一任务在一次 API 里重复加锁"
   这种真 bug 悄悄吃掉; 普通量配上 FF_FS_TIMEOUT 只会退化成一次
   FR_TIMEOUT(返回码), 是能看见、能查的失败, 而不是死锁 */
static const osMutexAttr_t s_mutex_attr =
{
    .name      = "fatfs",
    .attr_bits = 0,         /* 0 = 普通(非递归)互斥量 */
    .cb_mem    = NULL,      /* 控制块与句柄由内核堆分配 */
    .cb_size   = 0,
};

/* osMutexRelease 非 osOK 的次数 —— 稳态必须是 0.
   @note 为什么留着一个"没人读"的计数器: 它没有优雅的消费方(不能在热路径 printf,
         也不该为了它多一个任务), 但它记的是**卷锁 take/give 不配对**这一件事,
         而那是本文件唯一能观察到、且会静默毁数据的东西. 留在这里用调试器看一眼
         比什么都不留强. 每次 f_mount 重建互斥量时不清零(它统计的是全程)
   @note 板上实测: 正常负载下恒为 0; 把 ff_mutex_take 改成"假装取到锁但不真取"
         之后立刻涨到 21(当时那个测试的负向验证) */
static volatile uint32_t s_give_bad = 0;

/********************************* 内部状态 ***********************************/

/********************************* 对外接口 ***********************************/

/******************************************************************************
 * @name    ff_mutex_create
 * @brief   建一个卷互斥量(FatFs 回调, 见 ff.c 的 f_mount)
 * @param   vol[in] 卷号; 等于 FF_VOLUMES 时表示"系统锁"
 *
 * @return  1 成功 / 0 失败(下标越界或内核堆不足)
 *
 * @note    形参是 int 不是 BYTE: 原型由 ff.h 给出(ff.h:398), 照抄即可;
 *          负数也要挡住(ff.h 的声明不保证调用方只传合法值)
 *
 * @note    幂等: 若该槽位已有句柄, 先删掉再建(ff.c 在 f_mount 里也会先 delete,
 *          这里是防御性处理, 不依赖调用序)
 *****************************************************************************/
int ff_mutex_create(int vol)
{
    if (0 > vol || FATFS_MUTEX_CNT <= vol)
    {
        return 0;
    }

    if (NULL != s_mutex[vol])
    {
        (void)osMutexDelete(s_mutex[vol]);
        s_mutex[vol] = NULL;
    }

    s_mutex[vol] = osMutexNew(&s_mutex_attr);

    return (NULL != s_mutex[vol]) ? 1 : 0;
}

/******************************************************************************
 * @name    ff_mutex_delete
 * @brief   删一个卷互斥量(FatFs 回调, 见 ff.c 的 f_mount 卸载分支)
 * @param   vol[in] 卷号
 *
 * @return  无
 *
 * @note    f_mount 传 NULL 卸载时会调到这里, 之后该卷号可以被重新 create
 *****************************************************************************/
void ff_mutex_delete(int vol)
{
    if (0 > vol || FATFS_MUTEX_CNT <= vol || NULL == s_mutex[vol])
    {
        return;
    }

    (void)osMutexDelete(s_mutex[vol]);
    s_mutex[vol] = NULL;
}

/******************************************************************************
 * @name    ff_mutex_take
 * @brief   取卷互斥量(阻塞, 带超时) —— FatFs 每次文件操作进入时调
 * @param   vol[in] 卷号
 *
 * @return  1 成功 / 0 超时或该卷未初始化
 *
 * @note    FF_FS_TIMEOUT 的单位: CMSIS-RTOS v2 的 osMutexAcquire 超时用
 *          **内核 tick**; 本工程 configTICK_RATE_HZ == 1000, 所以
 *          FF_FS_TIMEOUT(1000) 恰好 = 1000ms = 1s, 与 ChaN 的 FreeRTOS 样例
 *          (xSemaphoreTake 也按 tick 解释)语义一致. 若将来改了 tick 频率,
 *          这个"1s"会跟着变, 需要一起复核
 *
 * @note    返回 0 就是"锁真的没取到"(而不是"取到后用不了") —— 上层看到的
 *          FR_TIMEOUT 唯一来源就在这里, 板上实测过: 持锁者不放时, 句柄类
 *          f_write 与路径类 f_open 都会如实阻塞满 1000ms 再返回超时(见 README)
 *****************************************************************************/
int ff_mutex_take(int vol)
{
    if (0 > vol || FATFS_MUTEX_CNT <= vol || NULL == s_mutex[vol])
    {
        return 0;
    }

    if (osOK != osMutexAcquire(s_mutex[vol], FF_FS_TIMEOUT))
    {
        return 0;
    }

    return 1;
}

/******************************************************************************
 * @name    ff_mutex_give
 * @brief   还卷互斥量 —— FatFs 每次文件操作退出时调
 * @param   vol[in] 卷号
 *
 * @return  无
 *
 * @note    FreeRTOS 的 xSemaphoreGive 对互斥量要求"调用者就是持有者", 非持有者
 *          释放会失败, 所以那个返回值**不是**能安全丢掉的: 它一旦非 osOK, 说明
 *          ff.c 那边 take/give 不配对了. 这里是全工程唯一能观察到这件事的地方,
 *          保留显式判错比 (void) 丢掉有价值(见下)
 *
 * @note    2026-09-13 那套并发测试曾把 osMutexRelease 的返回码计成一个计数器
 *          (give_bad), 常态 0、且把卷锁"假装取到但不真取"时会立刻涨到 21 ——
 *          这是那个计数有牙的实证. 测试已删(见 README 的"线程安全"一节), 但
 *          **判错的必要性没变**, 所以下面保留一条可诊断路径而不是静默丢弃:
 *          出错时置 s_give_bad(可由调试器观察; 不要在这里 printf,
 *          本函数在文件操作热路径上, 且 fputc 无锁)
 *
 * @note    已知的一个**死代码**陷阱(当时逐条核对 ff.c 后得到的结论, 免得后人再
 *          花一遍): 曾以为这里能抓到"ff.c:927 的 res != FR_TIMEOUT 保护被删掉",
 *          实际那条保护在本配置下走不到 —— 两个产生 FR_TIMEOUT 的点
 *          (mount_volume / validate)返回前都已把 *rfs 置 0, 于是调用方 LEAVE_FF
 *          传进去的 fs 恒为 NULL. 想验证超时语义请从**返回码**看(见 README)
 *****************************************************************************/
void ff_mutex_give(int vol)
{
    if (0 > vol || FATFS_MUTEX_CNT <= vol || NULL == s_mutex[vol])
    {
        return;
    }

    if (osOK != osMutexRelease(s_mutex[vol]))
    {
        s_give_bad++;
    }
}

/******************************************************************************
 * @name    get_fattime
 * @brief   取当前时间戳给 FatFs 建/改文件用(FF_FS_NORTC == 0 时 ff.c 回调)
 * @param   无
 *
 * @return  FatFs 打包时间戳(DOS/FAT 编码, 位域说明见 cywatch_rtc.c)
 *
 * @note    本函数只是**转发**: 时间值归系统时间源(system/Rtc/cywatch_rtc.c)
 *          所有, 这里做一次 uint32_t -> DWORD 的显式转换. 要设时间请调
 *          cywatch_rtc_set(), 不要指望本文件存任何东西
 *
 * @note    转发是安全的(不需要临界区): cywatch_rtc_get_packed() 内部是一次
 *          对齐 32 位读, Cortex-M4 上原子
 *****************************************************************************/
DWORD get_fattime(void)
{
    return (DWORD)cywatch_rtc_get_packed();
}

/* ---------------------------------------------------------------------------
 * 原 storage_fatfs_set_time() 与 storage_fatfs_time_t 已移到系统层:
 *   Core/system/Rtc/cywatch_rtc.h   的 cywatch_rtc_time_t / cywatch_rtc_set()
 *   Core/system/Rtc/cywatch_rtc.c   的实现(校验与编码逐行照搬, 未改行为)
 * 要设时间请调 cywatch_rtc_set(), 不要再在本层找这个接口.
 * ------------------------------------------------------------------------- */
