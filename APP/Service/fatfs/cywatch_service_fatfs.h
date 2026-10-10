/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_service_fatfs.h
 *
 * @par dependencies
 * - stdint.h
 * - ff.h
 *
 * @author	zw1194
 *
 * @brief FatFs 服务: 同步阻塞的薄封装 —— 自动挂载 + 统一返回码.
 *
 * Processing flow:
 *
 * 1. main.c 在 osKernelStart() 之前调 service_fatfs_init();
 * 2. 服务建一个一次性任务: 挂载 -> (开关打开时)自检 -> osThreadExit;
 * 3. 之后任意任务调 service_fatfs_open/read/write/close, 同步返回.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 本服务只做三件事: 卷的挂载与状态、统一返回码、空参数保护. 文件对象由
 *       **调用方自己声明**, 所以 ff.h 直接暴露出去 —— 没有句柄池、没有注册表、
 *       没有锁、没有所有权.
 *
 * @note 多任务并发是安全的: FF_FS_REENTRANT == 1, ff.c 每次 API 进出都会取/放
 *       一把卷级互斥量. 前提是**每个任务用各自的 fatfs_file_t** —— 共用同一个
 *       文件对象, 读写位置必然互踩, 加什么锁都救不了.
 *
 * @note 所有 API 都要在 osKernelStart() 之后的任务上下文里调: 底层经 W25Q64
 *       adapter 访问 flash, 而它的 pf_delay 是 osDelay, 调度器没起来会卡死.
 *
 * @warning 本卷占据整片 W25Q64, 起始偏移 = 0, 与 OTA 暂存区完全重叠. 格式化会
 *          盖掉 OTA 镜像头(erase_chip_first != 0 时整片端掉), 挂载在读不出文件
 *          系统时也会**自动格式化**. Bootloader/ 与 OTA/ 是独立工程, 链接期
 *          发现不了. 在 OTA 改成"用 FatFs 存镜像文件"之前, 两条路径别同时用.
 *****************************************************************************/
#ifndef __CYWATCH_SERVICE_FATFS_H__
#define __CYWATCH_SERVICE_FATFS_H__

/***********************************Includes***********************************/
#include <stdint.h>
#include "ff.h"     /* TCHAR(UTF-8 char) / FIL / BYTE / FA_* / FRESULT / FR_* */
/***********************************Includes***********************************/

/***********************************Defines************************************/
/* 上电自检开关: 1 = 挂载成功后跑一遍自检(结果经 printf 打出来), 0 = 只挂载不打印.
   自检内容: 写 9216B -> 读回逐字节校验 -> f_stat -> f_getfree -> 中文长文件名 ->
   列根目录 -> 栈余量. 上电一眼就能确认 SPI2 / W25Q64 / 接线是通的.
   代价: 每次上电重写同一批文件(约 9KB), 消耗一次擦写; 不想要就改成 0. */
#define SERVICE_FATFS_SELFTEST          (1U)

/* 一次性任务的栈.
   4096 是 ff.c 侧的用量: FF_USE_LFN == 2 时它在**调用者栈**上开 LFN 工作缓冲
   (512B), 再叠一个 FILINFO(内含 fname[256], 约 300B).
   +512 是自检那个 FIL(内含 buf[512], 因为 FF_FS_TINY == 0).
   本工程 configCHECK_FOR_STACK_OVERFLOW == 0, 栈溢出**不会报错**, 只会踩坏相邻的
   内核堆块 —— 所以宁大勿小. */
#define SERVICE_FATFS_BOOT_STACK        (4608U)

/* ---- 统一返回码 ----
   0  成功
   >0 FatFs 的 FRESULT 原样透传(FR_* 是 1..19, FR_OK == 0)
   <0 本服务的码
   符号位就是"谁报的错"的分界: 正数一定来自 FatFs, 负数一定来自本服务. */
#define SERVICE_FATFS_OK                (0)     /* 成功 */
#define SERVICE_FATFS_ERR_NO_INIT       (-1)    /* 没调 service_fatfs_init(), 或它失败了 */
#define SERVICE_FATFS_ERR_NULL_ARG      (-2)    /* 传了空指针 */
#define SERVICE_FATFS_ERR_NOT_MOUNTED   (-3)    /* 卷还没挂上(任务还没跑到, 或挂载失败) */
/* 下面四个只有 service_fatfs_format 会返回 */
#define SERVICE_FATFS_ERR_FMT_ADAPTER   (-4)    /* 格式化: W25Q64 adapter 实例化失败 */
#define SERVICE_FATFS_ERR_FMT_ERASE     (-5)    /* 格式化: 整片擦除失败 */
#define SERVICE_FATFS_ERR_FMT_MKFS      (-6)    /* 格式化: f_mkfs 失败 */
#define SERVICE_FATFS_ERR_FMT_MOUNT     (-7)    /* 格式化: 重建后重新挂载失败 */
/***********************************Defines************************************/

/**********************************Declaring***********************************/
/* 文件对象 = FatFs 的 FIL. 调用方自己声明一个(栈上或 static 都行), 交给
   service_fatfs_open 即可; 用完必须 service_fatfs_close.
   @warning 约 584B. 放栈上时注意任务栈够不够, 拿不准就做成 static. */
typedef FIL fatfs_file_t;

/**
 * @name  fatfs_dirent_cb_t
 * @brief service_fatfs_list 每找到一个目录项就调一次
 * @param p_name[in] 文件名(UTF-8). 目录也走这个回调. 只在回调返回前有效, 要留得自己拷
 * @param fsize[in]  文件字节数(目录项此值无效)
 * @param is_dir[in] 1 = 目录 / 0 = 文件
 * @param p_arg[in]  调 service_fatfs_list 时原样带回来的用户参数
 * @return 无
 * @note 回调同步跑在调用 service_fatfs_list 的那个任务里, 里面随便干什么都行.
 */
typedef void (*fatfs_dirent_cb_t)(const char *p_name, uint32_t fsize,
                                  uint8_t is_dir, void *p_arg);

/**
 * @name  service_fatfs_init
 * @brief 建"挂载 + 自检"的一次性任务. 幂等
 * @return 0 成功(含重复调用) / -1 任务创建失败(内核堆不足)
 *
 * @note 可在 osKernelStart() 之前调用(跟 service_lvgl_init 同一个位置): 本函数
 *       只建任务对象, 不碰设备.
 * @note 返回值必须看: -1 只可能是内核堆不足, 而 configASSERT 与
 *       configUSE_MALLOC_FAILED_HOOK 都是关的 —— 分配失败是静默的, 表现是
 *       "文件 API 全部返回 -1, 串口上一个字都没有".
 * @note 挂载是异步的: 本函数返回时卷**还没挂上**, 看 service_fatfs_is_mounted().
 */
int8_t service_fatfs_init(void);

/**
 * @name  service_fatfs_is_mounted
 * @brief 查卷挂上没
 * @return 1 = 已挂载 / 0 = 还没挂上(任务还没跑到, 或挂载失败)
 * @note 无锁: 单字节读写原子, 读到"稍旧的 0"只意味着"还没挂上", 那是真话不是错报.
 */
uint8_t service_fatfs_is_mounted(void);

/**
 * @name  service_fatfs_open
 * @brief 打开文件. 内部就是 f_open
 * @param p_file[in,out] 调用方自己声明的文件对象
 * @param p_path[in]     路径(UTF-8). 根目录写 "a.txt"; 子目录 "dir/a.txt"
 * @param mode[in]       直接给 FatFs 的 FA_* 常量, 常见组合:
 *                         FA_READ                      只读打开
 *                         FA_READ | FA_WRITE           读写打开(文件须已存在)
 *                         FA_WRITE | FA_CREATE_ALWAYS  新建/截断重建
 *                         FA_WRITE | FA_CREATE_NEW     只新建, 已存在则失败
 *                         FA_WRITE | FA_OPEN_ALWAYS    有就打开, 没有就建
 *                         FA_WRITE | FA_OPEN_APPEND    追加写
 * @return 0 成功 / <0 见 SERVICE_FATFS_ERR_* / >0 见 FR_*
 * @note 常见失败码: FR_NO_FILE(4) 文件不存在 / FR_NO_PATH(5) 目录不存在 /
 *       FR_DENIED(7) 权限不符 / FR_NOT_READY(3) 卷没挂上.
 * @note FA_OPEN_APPEND 会把读写指针都放到文件末尾, 不要在追加模式下读整个文件.
 */
int8_t service_fatfs_open(fatfs_file_t *p_file, const char *p_path, BYTE mode);

/**
 * @name  service_fatfs_close
 * @brief 关闭文件. 写完必须调它, 否则数据可能还在缓冲里没落盘
 * @param p_file[in] 由 service_fatfs_open 得到的文件对象
 * @return 0 成功 / <0 见 SERVICE_FATFS_ERR_* / >0 见 FR_*
 * @note 失败(FR_*)时文件对象仍然有效, 可以重试 close —— f_close 失败意味着数据
 *       可能还没落盘, 此时丢掉对象等于把错误咽下去.
 */
int8_t service_fatfs_close(fatfs_file_t *p_file);

/**
 * @name  service_fatfs_read
 * @brief 从当前位置读. 内部就是 f_read
 * @param p_file[in]  文件对象
 * @param p_buf[out]  接收缓冲
 * @param len[in]     想读多少字节
 * @param p_read[out] 实际读到多少(出错时也会写, 通常 0)
 * @return 0 成功 / <0 见 SERVICE_FATFS_ERR_* / >0 见 FR_*
 * @note 读到的比要的少不是错误: 已在文件末尾时 *p_read == 0 而返回 0. 判断"读完没"
 *       要看 *p_read == 0, 不是看返回值.
 */
int8_t service_fatfs_read(fatfs_file_t *p_file, void *p_buf, uint32_t len,
                          uint32_t *p_read);

/**
 * @name  service_fatfs_write
 * @brief 从当前位置写. 内部就是 f_write
 * @param p_file[in]     文件对象
 * @param p_buf[in]      待写数据
 * @param len[in]        想写多少字节
 * @param p_written[out] 实际写入多少
 * @return 0 成功 / <0 见 SERVICE_FATFS_ERR_* / >0 见 FR_*
 * @note 必须比对 *p_written 与 len: FatFs 在空间不足时返回 FR_OK 但只写一部分 ——
 *       这是它唯一一处"成功返回里藏着失败"的地方.
 * @note 返回 0 只代表数据进了 FatFs 的窗口缓冲, 不等于已落盘. 要确保落盘得走到
 *       service_fatfs_close() 并看到它返回 0.
 */
int8_t service_fatfs_write(fatfs_file_t *p_file, const void *p_buf, uint32_t len,
                           uint32_t *p_written);

/**
 * @name  service_fatfs_seek
 * @brief 移动读写指针. 内部就是 f_lseek
 * @param p_file[in] 文件对象
 * @param offset[in] 目标偏移(从文件头算, 字节)
 * @return 0 成功 / <0 见 SERVICE_FATFS_ERR_* / >0 见 FR_*
 * @note 允许 seek 超过文件末尾, 中间那段读出来是 0.
 */
int8_t service_fatfs_seek(fatfs_file_t *p_file, uint32_t offset);

/**
 * @name  service_fatfs_size
 * @brief 取文件大小
 * @param p_file[in]  文件对象
 * @param p_size[out] 文件字节数(出错时写 0)
 * @return 0 成功 / <0 见 SERVICE_FATFS_ERR_*
 */
int8_t service_fatfs_size(fatfs_file_t *p_file, uint32_t *p_size);

/**
 * @name  service_fatfs_remove
 * @brief 删除文件或空目录. 内部就是 f_unlink
 * @param p_path[in] 路径
 * @return 0 成功 / <0 见 SERVICE_FATFS_ERR_* / >0 见 FR_*
 * @note FR_NO_FILE(4) 文件不存在 / FR_DENIED(7) 目录非空或只读.
 */
int8_t service_fatfs_remove(const char *p_path);

/**
 * @name  service_fatfs_mkdir
 * @brief 建目录. 内部就是 f_mkdir
 * @param p_path[in] 目录路径(UTF-8), 如 "ota"
 * @return 0 成功(**含目录已存在**) / <0 见 SERVICE_FATFS_ERR_* / >0 见 FR_*
 * @note ★FR_EXIST(8) 当成功★: 目录已存在是正常情形(第二轮下载/续传), 原样报错会让
 *       调用方把"目录本来就在"误判成"建不出来". f_mkdir 对已存在的目录返回 FR_EXIST.
 * @note 不建中间层: "a/b" 要求 "a" 已存在, 否则 FR_NO_PATH(5).
 */
int8_t service_fatfs_mkdir(const char *p_path);

/**
 * @name  service_fatfs_list
 * @brief 列目录: 每找到一个条目就调一次 pf_cb
 * @param p_path[in] 目录路径(根目录写 "")
 * @param pf_cb[in]  目录项回调(不可为 NULL)
 * @param p_arg[in]  用户参数, 回调时原样带回
 * @return 0 成功 / <0 见 SERVICE_FATFS_ERR_* / >0 见 FR_*
 * @note 回调式的好处: 不用关心目录里有多少文件, 也不用给缓冲区.
 * @note 遍历期间别的任务若改了这个目录, 结果不保证是某一时刻的完整快照 —— 这是
 *       FatFs 的固有性质, 不是本服务的限制.
 */
int8_t service_fatfs_list(const char *p_path, fatfs_dirent_cb_t pf_cb, void *p_arg);

/**
 * @name  service_fatfs_format
 * @brief 格式化整片 W25Q64 并重新挂载(很慢, 阻塞)
 * @param erase_chip_first[in] 0 = 只 f_mkfs(约 2~3s, **不擦数据区**: 旧文件内容仍
 *                                 留在 flash 上, 读得到, 只是文件系统看不见)
 *                             1 = 先整片擦成 0xFF 再 f_mkfs(真正擦净, 典型 20s /
 *                                 最大 100s)
 * @return 0 成功(此时卷已重新挂好) / -4 ~ -7 见 SERVICE_FATFS_ERR_FMT_*
 * @note 整片擦除是阻塞式忙等(最长 120s), 靠 osDelay 让出 CPU, 只挂住本任务 ——
 *       但调用方若在界面/命令上下文, 要有"最长两分钟不响应"的心理准备.
 * @note 本函数不做"卷是否被占用"的检查, 也不会关别人已打开的文件. 有并发消费者
 *       时格式化, 结果未定义.
 * @warning 见文件头的 @warning: 会盖掉偏移 0 起的 OTA 暂存区.
 */
int8_t service_fatfs_format(uint8_t erase_chip_first);
/**********************************Declaring***********************************/

#endif // __CYWATCH_SERVICE_FATFS_H__
