/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_service_fatfs.c
 *
 * @par dependencies
 * - cywatch_log.h
 * - cywatch_service_fatfs.h
 * - cywatch_adapter_w25q64.h  (BSP/W25Q64)
 * - cmsis_os2.h
 * - stdio.h
 *
 * @author	zw1194
 *
 * @brief FatFs 服务: 在 W25Q64 上挂载一个 FAT 卷, 并对上层提供同步阻塞的
 *        文件接口 + 一套上电自检.
 *
 * Processing flow:
 *
 * 1. service_fatfs_init() 建一个一次性任务;
 * 2. 该任务: fatfs_mount() -> (开关打开时)fatfs_selftest() -> osThreadExit();
 * 3. 此后任意任务调 service_fatfs_*, 每个都是"查状态 -> 转一次 FatFs API ->
 *    转一次返回码", 同步返回.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 内存开销(文件级 static, 进 .bss): s_fatfs 卷对象 ~0.6KB, s_mkfs_work
 *       0.5KB. 另有一块 4KB 在 cywatch_fatfs_diskio.c 的 s_rmw_block(RMW 的擦除
 *       块缓冲), 那一块**不能**挪到栈上: 栈溢出不会报错, 只会踩坏内核堆块.
 *
 * @note 扇区 512B 比 W25Q64 的擦除单位(4096B)小 8 倍, 所以每次 disk_write 都要
 *       读-改-写. 细节见 cywatch_fatfs_diskio.c.
 *
 * @note 串口打印中文的坑: 源码与 API 字符串都是 UTF-8(FF_LFN_UNICODE == 2), 所以
 *       文件名字节流也是 UTF-8. PC 端串口工具若按 GBK 解码会显示成乱码 —— 那是
 *       显示端的设置问题, 文件系统里存的名字是对的.
 *****************************************************************************/
/***********************************Includes***********************************/
#include "cywatch_service_fatfs.h"
#include "cywatch_adapter_w25q64.h" /* 整片擦除没有扇区语义(diskio 只暴露扇区擦除) */
#include "cmsis_os2.h"
#include "system/log/cywatch_log.h" /* log_printf() */
#include <stdio.h>
/***********************************Includes***********************************/

/***********************************Defines************************************/
/* 挂载点: "" = 逻辑盘 0(本工程 FF_VOLUMES == 1) */
#define FATFS_DRIVE                 ""

/* 自检文件名. 中文名是**有意**加的: 它验证 FF_USE_LFN == 2 + FF_LFN_UNICODE == 2
   这条 UTF-8 长文件名链路. 这一步挂了先查源文件编码, 不是查磁盘层 */
#define FATFS_TEST_FILE             "selftest.txt"
#define FATFS_TEST_LFN              "目录测试.txt"

/* 簇大小 = 4096B, 与 W25Q64 擦除单位一致.
   为什么不交给 f_mkfs 自动选: 自动选会得到 FAT16 + 1024B 簇(8128 个簇), FAT 表要
   32 个扇区; 指定 4096B 得到 FAT12 + 2035 个簇, FAT 表只要 8 个扇区, 元数据更集中.
   必须是 2 的幂且 >= 一个扇区 */
#define FATFS_AU_SIZE               (4096UL)

/* 自检写入 36 x 256B = 跨 3 个 4KB 簇, 顺带验证"多簇分配 + FAT 链表 + 多扇区读写" */
#define FATFS_TEST_SIZE             (9216UL)

/* 自检的读写块大小(必须整除 FATFS_TEST_SIZE).
   **故意**保持 256B 这种小片写: 扇区(512B)小于擦除单位(4096B), 小片写每次都走
   disk_write 的读-改-写慢路径, 而慢路径正是最容易写错的一段(少读邻居 -> 静默丢
   数据). 改成 4096 整块就只走"擦+写"快路径, 等于没测 RMW.
   代价(板上实测): 这 9216B 触发约 28 次 disk_write, 自检第 1 步多花约 2.2s */
#define FATFS_TEST_CHUNK            (256U)

/* f_mkfs 的工作缓冲. f_mkfs 对它的唯一校验是 sz_buf == 0 -> FR_NOT_ENOUGH_CORE,
   没有上限要求; 这里 512 是**最保守**的一档(本工程 FF_MIN_SS == FF_MAX_SS == 512,
   于是 sz_buf == 1), 纯粹为了省 RAM.
   代价(只在真走了一次格式化的那一次付): 元数据约 41 个扇区, 从整块快路径(73ms/块)
   变成每扇区一次读-改-写(79ms/扇区), 约 2~3s 变成约 3~4s.
   想换回快的: 改成 4096(仍然合法), 但要先确认 RW_IRAM1 有 3584B 余量 */
#define FATFS_MKFS_WORK_SIZE        (512U)
/***********************************Defines************************************/

/**********************************Variables***********************************/
/* 卷对象. 必须 static: 挂载后整个生命周期共用, 放栈上等于挂完就丢 */
static FATFS s_fatfs;

static BYTE s_mkfs_work[FATFS_MKFS_WORK_SIZE];

/* 格式化参数. 只固定簇大小, 其余(fmt/n_fat/align/n_root)交给 f_mkfs.
   @warning .fmt 必须显式给 FM_ANY(0x07): 写成 `= {0}` 时 fmt 就是 0, ff.c 会因为
            `!(fsopt & FM_FAT)` 直接返回 FR_INVALID_PARAMETER(6). 注意 FS_FAT12/
            FS_FAT16 是**读盘**判出来的结果类型, 写盘要用的是 FM_* 那一组, 两组数值
            不通用.
   @note 卷是 FAT12 还是 FAT16 由**簇数**决定, 不由参数决定(ff.c 里是级联赋值, 最后
         成立的那个胜出). 8MB 上做不出 FAT32: 它要求 RootEntCnt == 0, 而读盘路径对
         非 FAT32 又要求 RootEntCnt != 0, 需要约 33MB 才能自洽, 硬来会 FR_MKFS_ABORTED(14) */
static const MKFS_PARM s_mkfs_opt =
{
    .fmt     = FM_ANY,
    .n_fat   = 0,                        /* 0 = 缺省(1 份 FAT) */
    .align   = 0,                        /* 0 = 问 disk_ioctl(GET_BLOCK_SIZE) */
    .n_root  = 0,                        /* 0 = 缺省(512 个根目录项) */
    .au_size = FATFS_AU_SIZE,
};

/* 服务状态. 都是 uint8_t: Cortex-M4 上单字节读写天然原子, 不需要临界区 */
static uint8_t s_inited  = 0U;   /* service_fatfs_init() 是否已经建过任务 */
static uint8_t s_mounted = 0U;   /* 卷挂上没 —— 只从 0 变成 1 */

/* 一次性任务属性. 常量, 放 flash. cb_mem/stack_mem 留 NULL: 让 CMSIS-RTOS2 从
   FreeRTOS 内核堆里分配 TCB 与栈 */
static const osThreadAttr_t g_fatfs_boot_attr =
{
    .name       = "fatfs",
    .attr_bits  = 0U,
    .cb_mem     = NULL,
    .cb_size    = 0U,
    .stack_mem  = NULL,
    .stack_size = SERVICE_FATFS_BOOT_STACK,
    .priority   = osPriorityNormal,
};
/**********************************Variables***********************************/

/******************************Static Functions********************************/
/* 检查"能不能碰卷". 返回 0 = 可以, <0 = SERVICE_FATFS_ERR_*.
   空指针检查不在这里 —— 每个 API 的入参个数不同, 混在一起反而看不清 */
static int8_t service_fatfs_state_check(void)
{
    if (0U == s_inited)  { return SERVICE_FATFS_ERR_NO_INIT; }
    if (0U == s_mounted) { return SERVICE_FATFS_ERR_NOT_MOUNTED; }
    return SERVICE_FATFS_OK;
}

/******************************************************************************
 * @name    fatfs_mkfs_and_remount
 * @brief   f_mkfs 重建文件系统, 然后重新挂载
 * @return  0 成功 / -1 f_mkfs 失败 / -2 重新挂载失败
 *
 * @note    被两个地方共用: fatfs_mount() 的自动格式化分支, 和
 *          service_fatfs_format(). 两处的参数必须一致, 所以共用一份 s_mkfs_opt.
 * @note    f_mkfs 后 fs_type 已被清零, 必须重挂才能读到新的 VBR.
 *****************************************************************************/
static int8_t fatfs_mkfs_and_remount(void)
{
    if (FR_OK != f_mkfs(FATFS_DRIVE, &s_mkfs_opt, s_mkfs_work,
                        (UINT)sizeof(s_mkfs_work)))
    {
        return -1;
    }

    if (FR_OK != f_mount(&s_fatfs, FATFS_DRIVE, 1))
    {
        return -2;
    }

    return 0;
}

/******************************************************************************
 * @name    fatfs_mount
 * @brief   挂载卷 0; 读不出文件系统时自动格式化
 * @return  0 成功 / 非 0 失败(细节已 printf)
 *
 * @note    自动格式化只在"读不出 VBR"(FR_NO_FILESYSTEM)时才走. 新片或被整片擦过
 *          的片子第一次上电会走到这里, 属正常流程; 卷一旦成形, 之后每次上电都直接
 *          "挂载成功", 不会再格式化.
 * @warning 自动格式化会盖掉偏移 0 起的 OTA 暂存区, 见服务头文件的 @warning.
 *****************************************************************************/
static int8_t fatfs_mount(void)
{
    FRESULT fr = f_mount(&s_fatfs, FATFS_DRIVE, 1);

    if (FR_OK == fr)
    {
        log_printf("[FS] 挂载成功\r\n");
        return 0;
    }

    if (FR_NO_FILESYSTEM != fr)
    {
        log_printf("[FS] 挂载失败 fr=%d (非'无文件系统', 查 SPI2/芯片接线)\r\n", (int)fr);
        return -1;
    }

    log_printf("[FS] 卷上没有文件系统(fr=%d)\r\n", (int)fr);
    log_printf("[FS] 警告: 即将格式化整片 W25Q64, 会破坏偏移 0 起的 OTA 暂存区\r\n");

    if (0 != fatfs_mkfs_and_remount())
    {
        log_printf("[FS] 自动格式化失败\r\n");
        return -1;
    }

    log_printf("[FS] 格式化完成并已挂载\r\n");
    return 0;
}

/******************************************************************************
 * @name    fatfs_pattern
 * @brief   自检数据的确定性伪随机图案: 任意偏移都能重算
 * @param   index[in] 该字节在文件内的偏移
 * @return  该偏移处应有的字节值
 *
 * @note    用函数而不是大数组生成数据, 是为了读写两侧都不需要额外缓存: 回读校验
 *          时按偏移重算即可, 省下一份 9216B 的缓冲.
 *****************************************************************************/
static uint8_t fatfs_pattern(uint32_t index)
{
    return (uint8_t)(((index * 31UL) + 7UL) & 0xFFUL);
}

/******************************************************************************
 * @name    fatfs_type_name
 * @brief   把 FATFS.fs_type 的数值转成可读字符串
 * @param   fs_type[in] FATFS.fs_type
 * @return  卷型名字
 *
 * @note    本工程按 4096B 簇格式化出来的是 FAT12(2042 个簇, 未超过 MAX_FAT12 =
 *          4085), 这是 FatFs 对这种容量的正常选择, 不是故障. 打印名字是为了让
 *          日志里一眼看懂, 免得看到 "卷型=1" 还要去查枚举.
 *****************************************************************************/
static const char *fatfs_type_name(BYTE fs_type)
{
    switch (fs_type)
    {
    case FS_FAT12:  return "FAT12";
    case FS_FAT16:  return "FAT16";
    case FS_FAT32:  return "FAT32";
    case FS_EXFAT:  return "exFAT";
    default:        return "unknown";
    }
}

/******************************************************************************
 * @name    fatfs_write_test_file
 * @brief   建/清空一个文件并写入 FATFS_TEST_SIZE 字节的自检图案
 * @param   p_fil[in]     调用方的文件对象
 * @param   p_name[in]    文件名(UTF-8)
 * @param   p_written[out] 实际写入字节数; 可为 NULL
 * @return  FRESULT; FR_OK 且 *p_written == FATFS_TEST_SIZE 才算成功
 *****************************************************************************/
static FRESULT fatfs_write_test_file(FIL *p_fil, const TCHAR *p_name,
                                     UINT *p_written)
{
    FRESULT fr    = FR_OK;
    UINT    total = 0;
    UINT    bw    = 0;
    UINT    i     = 0;
    BYTE    chunk[FATFS_TEST_CHUNK];

    /* FA_CREATE_ALWAYS: 每次上电都重建, 免得多次自检把文件越滚越大 */
    fr = f_open(p_fil, p_name, (BYTE)(FA_CREATE_ALWAYS | FA_WRITE));
    if (FR_OK != fr)
    {
        return fr;
    }

    while (total < (UINT)FATFS_TEST_SIZE)
    {
        for (i = 0; i < FATFS_TEST_CHUNK; i++)
        {
            chunk[i] = fatfs_pattern((uint32_t)total + (uint32_t)i);
        }

        fr = f_write(p_fil, chunk, FATFS_TEST_CHUNK, &bw);
        if (FR_OK != fr || FATFS_TEST_CHUNK != bw)
        {
            break;
        }

        total += bw;
    }

    (void)f_close(p_fil);

    if (NULL != p_written)
    {
        *p_written = total;
    }

    return fr;
}

/******************************************************************************
 * @name    fatfs_verify_test_file
 * @brief   读回一个文件并与自检图案逐字节比对
 * @param   p_fil[in]          调用方的文件对象
 * @param   p_name[in]         文件名(UTF-8)
 * @param   p_read[out]        实际读到的字节数; 可为 NULL
 * @param   p_mismatch_at[out] 首个不符字节的偏移; 可为 NULL
 * @return  FR_OK 长度与内容都对
 *          FR_INT_ERR 内容不符或长度不对(不是 FatFs 的标准返回码, 用作本函数自己
 *                     的"校验失败"信号)
 *          其它       f_open / f_read 的原样返回
 *****************************************************************************/
static FRESULT fatfs_verify_test_file(FIL *p_fil, const TCHAR *p_name,
                                      UINT *p_read, uint32_t *p_mismatch_at)
{
    FRESULT fr    = FR_OK;
    UINT    total = 0;
    UINT    br    = 0;
    UINT    i     = 0;
    BYTE    chunk[FATFS_TEST_CHUNK];

    fr = f_open(p_fil, p_name, (BYTE)FA_READ);
    if (FR_OK != fr)
    {
        return fr;
    }

    while (total < (UINT)FATFS_TEST_SIZE)
    {
        fr = f_read(p_fil, chunk, FATFS_TEST_CHUNK, &br);
        if (FR_OK != fr)
        {
            break;
        }

        if (0 == br)    /* 提前 EOF: 文件短于预期 */
        {
            break;
        }

        for (i = 0; i < br; i++)
        {
            if (chunk[i] != fatfs_pattern((uint32_t)total + (uint32_t)i))
            {
                if (NULL != p_mismatch_at)
                {
                    *p_mismatch_at = (uint32_t)total + (uint32_t)i;
                }

                /* 这条提前返回路径也必须回填 *p_read: 否则调用方看到的是它自己那个
                   初值 0, 日志会打成"读到了 0 字节", 把"内容在第 9000 字节开始不对"
                   误导成"一个字节都没读到", 排障时白绕一圈 */
                if (NULL != p_read)
                {
                    *p_read = total;
                }

                (void)f_close(p_fil);
                return FR_INT_ERR;
            }
        }

        total += br;
    }

    (void)f_close(p_fil);

    if (NULL != p_read)
    {
        *p_read = total;
    }

    if ((UINT)FATFS_TEST_SIZE != total)
    {
        return FR_INT_ERR;
    }

    return FR_OK;
}

/******************************************************************************
 * @name    fatfs_dump_root
 * @brief   列一遍根目录并打印(顺手证明 f_readdir 的长文件名返回是好的)
 * @return  0 成功 / -1 打开或遍历失败
 *
 * @note    打印宽度按**字节**算, 中文名的对齐会不齐, 这是 %-24s 的固有行为, 不是 bug.
 *****************************************************************************/
static int8_t fatfs_dump_root(void)
{
    FRESULT fr = FR_OK;
    DIR     dir;
    FILINFO fno;

    log_printf("[FS] 6. 根目录列表:\r\n");

    fr = f_opendir(&dir, FATFS_DRIVE);
    if (FR_OK != fr)
    {
        log_printf("        FAIL: f_opendir fr=%d\r\n", (int)fr);
        return -1;
    }

    for (;;)
    {
        fr = f_readdir(&dir, &fno);

        /* 读失败, 或读到名字为空(目录结束) */
        if (FR_OK != fr || 0 == fno.fname[0])
        {
            break;
        }

        log_printf("        %-24s %8lu B  attr=0x%02X\r\n",
               fno.fname,
               (unsigned long)fno.fsize,
               (unsigned)fno.fattrib);
    }

    (void)f_closedir(&dir);

    if (FR_OK != fr)
    {
        log_printf("        FAIL: f_readdir fr=%d\r\n", (int)fr);
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @name    fatfs_selftest
 * @brief   上电自检: 写 -> 读回校验 -> f_stat -> f_getfree -> 中文名 -> 列目录
 * @return  0 成功(1~6 步与栈余量全 PASS)
 *         -1 卷未挂载 / -2 写文件失败 / -3 读回校验失败
 *         -4 f_stat 失败 / -5 f_getfree 失败 / -6 列根目录失败
 *
 * @note    第 5 步"中文长文件名"**不计入**返回码: 它验证的是源文件编码 +
 *          FF_LFN_UNICODE 这条链路, 与磁盘层无关. 它的结果单独打一行, 并在末尾的
 *          总结行里标出, 不会被漏看.
 * @note    文件对象 FIL 开在**本函数栈上**(不是文件级 static): 它只在自检期间用,
 *          常驻 .bss 不值当. 任务栈给 4608 就是为了带上它那 512B.
 *****************************************************************************/
static int8_t fatfs_selftest(void)
{
    FRESULT fr        = FR_OK;
    FIL     fil;
    FILINFO fno;
    FATFS  *p_fs      = NULL;
    DWORD   free_clst = 0;
    UINT    total     = 0;
    uint32_t bad_at   = 0;
    uint32_t sz_kb    = 0;
    int8_t  ret       = 0;
    uint8_t lfn_fail  = 0;

    log_printf("\r\n===== FatFs 自检开始(介质: W25Q64 8MB @SPI2, 扇区 512B, 擦除块 4096B) =====\r\n");

    /* 前置条件: 卷已挂载(fs_type != 0). 没挂上就没什么可自检的 */
    if (0 == s_fatfs.fs_type)
    {
        log_printf("[FS] 卷未挂载, 自检中止\r\n");
        return -1;
    }

    /* 1. 写文件 */
    fr = fatfs_write_test_file(&fil, FATFS_TEST_FILE, &total);
    if (FR_OK != fr || (UINT)FATFS_TEST_SIZE != total)
    {
        log_printf("[FS] 1. 写文件        FAIL: fr=%d, 写入 %u/%lu B\r\n",
               (int)fr, (unsigned)total, (unsigned long)FATFS_TEST_SIZE);
        return -2;
    }
    log_printf("[FS] 1. 写文件        PASS: %s, %lu B\r\n",
           FATFS_TEST_FILE, (unsigned long)total);

    /* 2. 读回逐字节校验 */
    total = 0;
    fr = fatfs_verify_test_file(&fil, FATFS_TEST_FILE, &total, &bad_at);
    if (FR_OK != fr)
    {
        log_printf("[FS] 2. 读回校验      FAIL: fr=%d, 读到 %u B, 首个不符偏移 %lu\r\n",
               (int)fr, (unsigned)total, (unsigned long)bad_at);
        return -3;
    }
    log_printf("[FS] 2. 读回校验      PASS: %lu B 逐字节一致\r\n", (unsigned long)total);

    /* 3. f_stat */
    fr = f_stat(FATFS_TEST_FILE, &fno);
    if (FR_OK != fr)
    {
        log_printf("[FS] 3. f_stat        FAIL: fr=%d\r\n", (int)fr);
        return -4;
    }
    log_printf("[FS] 3. f_stat        PASS: size=%lu 属性=0x%02X 时间=%04X/%04X\r\n",
           (unsigned long)fno.fsize, (unsigned)fno.fattrib,
           (unsigned)fno.fdate, (unsigned)fno.ftime);

    /* 4. 容量/剩余空间 */
    fr = f_getfree(FATFS_DRIVE, &free_clst, &p_fs);
    if (FR_OK != fr || NULL == p_fs)
    {
        log_printf("[FS] 4. f_getfree     FAIL: fr=%d\r\n", (int)fr);
        return -5;
    }

    /* 字节数 = 簇数 x 每簇扇区数 x 每扇区字节数; 8MB 量级用 32 位够.
       扇区大小用 FF_MAX_SS 而不是 p_fs->ssize: FATFS 里的 ssize 成员只在
       FF_MAX_SS != FF_MIN_SS(变扇区模式)时才存在, 本工程两个都是 512, 成员就没了 */
    sz_kb = (((uint32_t)(p_fs->n_fatent - 2U) * (uint32_t)p_fs->csize *
              (uint32_t)FF_MAX_SS) / 1024UL);
    log_printf("[FS] 4. f_getfree     PASS: 总 %lu KB / 空闲 %lu KB "
           "(卷型=%s, 簇=%u 个 x %u B, 扇区=%u B)\r\n",
           (unsigned long)sz_kb,
           (unsigned long)(((uint32_t)free_clst * (uint32_t)p_fs->csize *
                            (uint32_t)FF_MAX_SS) / 1024UL),
           fatfs_type_name(p_fs->fs_type),
           (unsigned)(p_fs->n_fatent - 2U),
           (unsigned)(p_fs->csize * (WORD)FF_MAX_SS),
           (unsigned)FF_MAX_SS);

    /* 5. 中文长文件名(UTF-8 API). 失败只告警, 不影响磁盘层结论 */
    fr = fatfs_write_test_file(&fil, FATFS_TEST_LFN, NULL);
    if (FR_OK == fr)
    {
        log_printf("[FS] 5. 中文长文件名  PASS: %s\r\n", FATFS_TEST_LFN);
    }
    else
    {
        lfn_fail = 1;
        log_printf("[FS] 5. 中文长文件名  FAIL: fr=%d "
               "(先查源文件是否为 UTF-8 编码, 再查 FF_LFN_UNICODE)\r\n", (int)fr);
    }

    /* 6. 列根目录 */
    ret = fatfs_dump_root();
    if (0 != ret)
    {
        return -6;
    }

    /* 7. 栈余量(高水位). 放在最后一行, 量到的就是**全流程**的高水位.
       余量若小于 ~1KB, 就把 SERVICE_FATFS_BOOT_STACK 调大 */
    log_printf("[FS] 7. 栈余量        %lu B / %u B (全流程高水位)\r\n",
           (unsigned long)osThreadGetStackSpace(osThreadGetId()),
           (unsigned)SERVICE_FATFS_BOOT_STACK);

    /* 总结行放在**所有**步骤之后, 这样它才是真的总结(scrollback 时先看到结论) */
    log_printf("[FS] ===== 自检结束: 文件系统 PASS | 中文长文件名 %s =====\r\n\r\n",
           (0 == lfn_fail) ? "PASS" : "FAIL");

    return 0;
}

/* ===== 一次性诊断开关 =====
   1 = 本次上电先"整片擦除 + f_mkfs", 再走正常挂载 + 自检.
   目的: 自检第 6 步发现根目录首 4096B(一个擦除块)是 0xFF 空白, 128 个幻影目录项
   排在两个真实文件前面. 整片擦回全 0xFF 再格式化, 就能区分是"卷里残留的旧数据"
   还是"f_mkfs 的根目录零填充真的漏了第一块". ★查完改回 0★
   代价: 整片擦除典型 20s / 最大 100s, 期间不能断电
   @note 结论(2026-10-02 板上验证): 置 1 跑过一次后幻影项全部消失, 根目录只剩 2 项.
         说明 f_mkfs 的零填充是好的 —— 那 128 个幻影项是**旧卷**留在 flash 上的状态
         (根目录头一个擦除块仍是空白 0xFF), 不是当前格式化路径的缺陷. */
#define FATFS_BOOT_ERASE_CHIP_TEST  0

/******************************************************************************
 * @name    service_fatfs_boot_task
 * @brief   一次性任务体: (诊断擦除 +) 挂载 + 自检, 然后退出
 * @param   p_arg[in] 未使用
 * @return  无(末尾 osThreadExit)
 *
 * @note    跑完就 osThreadExit(), 而不是常驻空转: 自检不需要周期执行, 4.6KB 栈
 *          一直被占着不值当; 退出后空闲任务会把它归还内核堆.
 * @note    退出后卷仍是挂载状态(s_fatfs 是文件级 static), 后续消费者直接复用即可.
 * @note    挂载失败时不跑自检: 自检内部第一件事就是看卷挂上没有, 挂了也是白跑.
 *****************************************************************************/
static void service_fatfs_boot_task(void *p_arg)
{
    (void)p_arg;

#if (0 != FATFS_BOOT_ERASE_CHIP_TEST)
    if (SERVICE_FATFS_OK != service_fatfs_format(1U))
    {
        log_printf("[FS] 诊断: 整片擦除 + f_mkfs 失败, 仍按原流程继续\r\n");
    }
#endif

    s_mounted = (0 == fatfs_mount()) ? 1U : 0U;

#if (0 != SERVICE_FATFS_SELFTEST)
    if (0U != s_mounted)
    {
        (void)fatfs_selftest();
    }
#endif

    osThreadExit();
}
/******************************Static Functions********************************/

/**********************************Functions***********************************/
int8_t service_fatfs_init(void)
{
    /* 幂等: 重复调用直接当成功, 不会建出第二个任务 */
    if (0U != s_inited)
    {
        return SERVICE_FATFS_OK;
    }

    if (NULL == osThreadNew(service_fatfs_boot_task, NULL, &g_fatfs_boot_attr))
    {
        return -1;
    }

    s_inited = 1U;
    return SERVICE_FATFS_OK;
}

uint8_t service_fatfs_is_mounted(void)
{
    return s_mounted;
}

int8_t service_fatfs_open(fatfs_file_t *p_file, const char *p_path, BYTE mode)
{
    int8_t ret;

    if ((NULL == p_file) || (NULL == p_path))
    {
        return SERVICE_FATFS_ERR_NULL_ARG;
    }

    ret = service_fatfs_state_check();
    if (SERVICE_FATFS_OK != ret)
    {
        return ret;
    }

    return (int8_t)f_open(p_file, p_path, mode);
}

int8_t service_fatfs_close(fatfs_file_t *p_file)
{
    int8_t ret;

    if (NULL == p_file)
    {
        return SERVICE_FATFS_ERR_NULL_ARG;
    }

    ret = service_fatfs_state_check();
    if (SERVICE_FATFS_OK != ret)
    {
        return ret;
    }

    return (int8_t)f_close(p_file);
}

int8_t service_fatfs_read(fatfs_file_t *p_file, void *p_buf, uint32_t len,
                          uint32_t *p_read)
{
    int8_t ret;
    UINT   br = 0U;

    if ((NULL == p_file) || (NULL == p_buf) || (NULL == p_read))
    {
        return SERVICE_FATFS_ERR_NULL_ARG;
    }

    /* 先清零: 下面任何一条提前返回的路径上, 调用方都能看到一个确定的值 */
    *p_read = 0U;

    ret = service_fatfs_state_check();
    if (SERVICE_FATFS_OK != ret)
    {
        return ret;
    }

    ret = (int8_t)f_read(p_file, p_buf, (UINT)len, &br);

    /* 出错时把已读到的字节数也带出去: 调用方可以据此判断"丢了哪一段" */
    *p_read = (uint32_t)br;

    return ret;
}

int8_t service_fatfs_write(fatfs_file_t *p_file, const void *p_buf, uint32_t len,
                           uint32_t *p_written)
{
    int8_t ret;
    UINT   bw = 0U;

    if ((NULL == p_file) || (NULL == p_buf) || (NULL == p_written))
    {
        return SERVICE_FATFS_ERR_NULL_ARG;
    }

    *p_written = 0U;

    ret = service_fatfs_state_check();
    if (SERVICE_FATFS_OK != ret)
    {
        return ret;
    }

    ret = (int8_t)f_write(p_file, p_buf, (UINT)len, &bw);
    *p_written = (uint32_t)bw;

    return ret;
}

int8_t service_fatfs_seek(fatfs_file_t *p_file, uint32_t offset)
{
    int8_t ret;

    if (NULL == p_file)
    {
        return SERVICE_FATFS_ERR_NULL_ARG;
    }

    ret = service_fatfs_state_check();
    if (SERVICE_FATFS_OK != ret)
    {
        return ret;
    }

    return (int8_t)f_lseek(p_file, (FSIZE_t)offset);
}

int8_t service_fatfs_size(fatfs_file_t *p_file, uint32_t *p_size)
{
    int8_t ret;

    if ((NULL == p_file) || (NULL == p_size))
    {
        return SERVICE_FATFS_ERR_NULL_ARG;
    }

    *p_size = 0U;

    ret = service_fatfs_state_check();
    if (SERVICE_FATFS_OK != ret)
    {
        return ret;
    }

    /* f_size() 是宏(读 fp->obj.objsize), 不会失败, 所以这里固定返回 OK */
    *p_size = (uint32_t)f_size(p_file);

    return SERVICE_FATFS_OK;
}

int8_t service_fatfs_remove(const char *p_path)
{
    int8_t ret;

    if (NULL == p_path)
    {
        return SERVICE_FATFS_ERR_NULL_ARG;
    }

    ret = service_fatfs_state_check();
    if (SERVICE_FATFS_OK != ret)
    {
        return ret;
    }

    return (int8_t)f_unlink(p_path);
}

int8_t service_fatfs_list(const char *p_path, fatfs_dirent_cb_t pf_cb, void *p_arg)
{
    DIR      dir;
    FILINFO  fno;
    FRESULT  fr;
    int8_t   ret;

    if ((NULL == p_path) || (NULL == pf_cb))
    {
        return SERVICE_FATFS_ERR_NULL_ARG;
    }

    ret = service_fatfs_state_check();
    if (SERVICE_FATFS_OK != ret)
    {
        return ret;
    }

    fr = f_opendir(&dir, p_path);
    if (FR_OK != fr)
    {
        return (int8_t)fr;
    }

    for (;;)
    {
        fr = f_readdir(&dir, &fno);

        /* 两种结束方式: 出错(fr != FR_OK), 或读完(f_readdir 返回 FR_OK 但把
           fname[0] 写成 '\0'). 两种情况都跳出, 由结尾那次 return 统一报出去 */
        if ((FR_OK != fr) || ('\0' == fno.fname[0]))
        {
            break;
        }

        pf_cb(fno.fname,
              (uint32_t)fno.fsize,
              (0U != (fno.fattrib & AM_DIR)) ? 1U : 0U,
              p_arg);
    }

    (void)f_closedir(&dir);

    return (int8_t)fr;
}

int8_t service_fatfs_format(uint8_t erase_chip_first)
{
    int8_t ret;

    log_printf("[FS] ===== 开始格式化 (模式: %s) =====\r\n",
           (0 != erase_chip_first) ? "整片擦除 + f_mkfs" : "仅 f_mkfs");

    /* 先把卷对象注册好: f_mkfs 要求 FatFs[0] 非空, 否则返回 FR_NOT_ENABLED.
       opt = 0 意为"只注册, 不读盘": 不做任何磁盘访问, 对已挂载的情形也安全
       (会先注销再重注册, 卷互斥量随之重建) */
    (void)f_mount(&s_fatfs, FATFS_DRIVE, 0);

    /* 整片擦除是可选的: 只有它需要绕过 FatFs 直接找 W25Q64 adapter(f_mkfs 自己会
       调 disk_initialize, 所以"仅 f_mkfs"分支不需要这步) */
    if (0 != erase_chip_first)
    {
        /* 整片擦除没有扇区语义, diskio 层只暴露扇区擦除, 所以这里直接碰适配层.
           adapter 的 inst 内含 read_id 自检与 osDelay, 重复调用是安全的 */
        if (0 != storage_bsp_w25q64_inst())
        {
            log_printf("[FS] W25Q64 adapter 实例化失败, 放弃格式化\r\n");
            return SERVICE_FATFS_ERR_FMT_ADAPTER;
        }

        /* 典型 20s / 最大 100s. adapter 的 pf_wait_busy 用的是 120s 门限, 不是普通
           擦除的 5s —— 否则这里必然误报超时 */
        log_printf("[FS] 1/3 整片擦除中(典型 20s, 最大 100s, 请勿断电)...\r\n");
        if (0 != storage_bsp_w25q64_erase_chip())
        {
            log_printf("[FS] 整片擦除失败\r\n");
            return SERVICE_FATFS_ERR_FMT_ERASE;
        }
        log_printf("[FS] 1/3 整片擦除完成\r\n");
    }
    else
    {
        /* 不做物理擦除: 旧数据仍留在 flash 上, 只是文件系统看不见了 */
        log_printf("[FS] 1/3 跳过整片擦除(旧数据仍留在数据区, 只有元数据被重写)\r\n");
    }

    log_printf("[FS] 2/3 重建文件系统(f_mkfs)...\r\n");
    ret = fatfs_mkfs_and_remount();
    if (0 > ret)
    {
        /* 常见 FRESULT: 6 = FR_INVALID_PARAMETER(s_mkfs_opt.fmt 没给 FM_ANY),
                          14 = FR_MKFS_ABORTED(参数组合凑不出合法几何) */
        if (-1 == ret)
        {
            log_printf("[FS] 2/3 f_mkfs 失败\r\n");
            return SERVICE_FATFS_ERR_FMT_MKFS;
        }

        log_printf("[FS] 3/3 重新挂载失败\r\n");
        return SERVICE_FATFS_ERR_FMT_MOUNT;
    }

    log_printf("[FS] 3/3 重新挂载成功: 空的 FAT12 卷(4096B 簇)已就绪\r\n\r\n");

    /* 底层已经挂好了, 把状态同步过来 —— 否则调用方刚格式化完再 open 会拿到
       -3(未挂载), 那是假报错 */
    s_mounted = 1U;

    return SERVICE_FATFS_OK;
}
/**********************************Functions***********************************/
