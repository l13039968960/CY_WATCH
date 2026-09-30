/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_fatfs_diskio.c
 *
 * @par dependencies
 * - ff.h
 * - diskio.h
 * - stddef.h (只为 NULL: ff.h/diskio.h 都不带标准头, 见下)
 * - cywatch_adapter_w25q64.h  (BSP/W25Q64)
 * - cywatch_bsp_w25q64_reg.h  (BSP/W25Q64)
 *
 * @author	zw1194
 *
 * @brief FatFs R0.16 的底层磁盘接口(diskio)移植实现:
 *        把**整片** W25Q64(8MB) 当作一个 16384 扇区 x 512B 的块设备交给 FatFs.
 *
 * Processing flow:
 *
 * f_mount/f_open/f_read/f_write/f_mkfs  (ff.c)
 *   -> disk_initialize / disk_read / disk_write / disk_ioctl   (本文件)
 *     -> storage_bsp_w25q64_*()                                (BSP/W25Q64 adapter)
 *       -> bsp_w25q64_driver_t  (BSP/W25Q64, SPI2 @25MHz)
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 本文件实现的是 ChaN 定义的第三方接口(diskio.h), 所以返回类型与错误码
 *       必须照 DSTATUS/DRESULT, **不套用**本工程的 int8_t/0/-1/-2 约定;
 *       内部调用 adapter 时仍按本工程约定判 `if (0 != ret)`.
 *
 * @note 扇区取标准 LBA 尺寸 512B, 比 W25Q64 的擦除单位(4096B)小 8 倍. 由此得到
 *       本移植最重要的一条性质: **每次 disk_write 都必须做读-改-写(RMW)** ——
 *       先把整个 4KB 擦除块读回, 改掉其中 512B, 再擦除整块写回. 详见 disk_write
 *       的 @note. 代价: 写一个扇区 = 1 次 4KB 读 + 1 次擦除 + 1 次 4KB 编程
 *       (板上实测见下面 disk_write 的 @note), 这是 SPI NOR 上跑 FAT 的固有开销,
 *       不是实现缺陷.
 *
 * @note 512B 是**标准 LBA 尺寸**(ATA/SCSI/SD 的默认块长). 选它而不是 4096B, 代价
 *       就是上面这条 RMW; 换来的是卷的几何与通用块设备一致 —— PC 端工具、以及
 *       将来任何把这片 NOR 当作标准块设备暴露出去的路径, 都不必做 512/4096 重排.
 *
 * @note 卷偏移固定为 0(整片 8MB 都给文件系统). 本工程没有 OTA/Bootloader,
 *       不存在与别的裸偏移使用者抢同一片芯片的问题.
 *
 * @note 本文件的所有函数都依赖 adapter 的 storage_bsp_w25q64_inst(), 其内含
 *       osDelay —— 因此整条 FatFs 调用链**只能在 osKernelStart() 之后的任务
 *       上下文**里跑, 不能在 main 启动调度器之前调 f_mount.
 *****************************************************************************/
#include "ff.h"
#include "diskio.h"

/* @note ff.h 与 diskio.h 都**不带标准头**(ChaN 的原版样例靠它自己的 platform.h/
   storage.h 顺带引入 stdio/stdlib, 本移植没有那两个文件), 而磁盘层只用得到 NULL,
   所以这里显式引入 stddef.h. 少这一个就是 6 个 "use of undeclared identifier 'NULL'" */
#include <stddef.h>

/* @note RMW 里用 memcpy 把新数据打进擦除块缓冲 */
#include <string.h>

#include "cywatch_adapter_w25q64.h"
#include "cywatch_bsp_w25q64_reg.h"

/***********************************Defines************************************/
/* 本移植只挂 1 个物理盘(对应 FF_VOLUMES = 1, 逻辑盘 0) */
#define FATFS_DISK_PDRV             (0U)

/* 卷在介质上的起始字节偏移: 0 = 整片 W25Q64 都给文件系统
   @warning 若将来把卷挪到非 0 偏移, 这个值**必须**是擦除块大小(4096)的整数倍.
            理由: disk_write 的 RMW 是拿"绝对字节地址"去算擦除块边界的
            (blk_addr = addr & ~(ERASE_BYTES - 1)), 只有卷起点本身对齐, 卷内
            第 0 个扇区才落在擦除块首; 否则第一个擦除块会被卷外的数据共享,
            RMW 会去读写不属于本卷的字节 */
#define FATFS_DISK_BASE_ADDR        (0UL)

/* FatFs 的"扇区" = 标准 LBA 尺寸 512B.
   @warning 它**小于**介质的擦除单位(4096B), 这是本移植引入 RMW 的唯一原因:
            一个擦除块装着 8 个逻辑扇区, 写其中任何一个都必须把另外 7 个先读回
            来再一起写回去(见 disk_write). 若把这里改回 4096, RMW 立刻消失 */
#define FATFS_DISK_SECTOR_SIZE      (512UL)

/* 介质擦除单位(字节). W25Q64_SECTOR_SIZE == 4096 */
#define FATFS_DISK_ERASE_BYTES      (W25Q64_SECTOR_SIZE)

/* 一个擦除块 = 几个逻辑扇区 = 8.
   @note 必须整除 FATFS_DISK_ERASE_BYTES, 否则 RMW 的"整块"分支判不出边界 */
#define FATFS_DISK_ERASE_BLOCK_SS   (FATFS_DISK_ERASE_BYTES / FATFS_DISK_SECTOR_SIZE)

/* 卷扇区总数 = 8MB / 512B = 16384 */
#define FATFS_DISK_SECTOR_COUNT     (W25Q64_TOTAL_SIZE / FATFS_DISK_SECTOR_SIZE)

/* 单次 SPI 事务能搬的最大字节数 = 61440.
   @note 这个上限不是 W25Q64 的, 是底层 spi_hal.c 的: spi_receive() 把 size
         强转成 uint16_t 再交给 HAL_SPI_Receive, 超过 65535 会**静默截断**
         (少读数据而不报错). 由此 disk_read 的突发读必须自己分块.
   @note 取 61440 = 15 x 4096, 既能整除扇区大小(120 个扇区, 分块后仍扇区对齐),
         也刚好等于整个擦除块的 15 倍, 不至于把 RMW 的整块遍历切得太碎
   @note FatFs 在 csize(每簇扇区数)== 1 时 count 恒为 1, 但本移植 au_size = 4096B
         => csize == 8, 所以这个上限是真会被用到的, 不是摆设 */
#define FATFS_DISK_MAX_XFER_BYTES   (61440UL)
/***********************************Defines************************************/

/********************************* 内部状态 ***********************************/
/* 介质状态位组合: STA_NOINIT / STA_NODISK / STA_PROTECT.
   初值 = 未初始化, FatFs 的第一次 disk_initialize 会把它清掉 */
static DSTATUS s_disk_status = STA_NOINIT;

/* RMW 的工作缓冲, 大小 == 一个完整擦除块.
   @note 为什么不放栈上: 调用链 disk_write <- f_write/f_mkfs <- 自检任务, 而任务
         的栈是 osThreadNew 时按 g_storage_fatfs_attr.stack_size 给的; 再加
         4KB 进去会直接把栈吃穿, 而 configCHECK_FOR_STACK_OVERFLOW == 0 意味着
         **溢出不报错、只踩坏相邻堆块**. 这里放 BSS 是唯一安全的选择.
   @note 单线程安全前提: FatFs 的 FF_FS_REENTRANT == 1 已经把对同一卷的并发访问
         串行化了(cywatch_fatfs_os.c 的 ff_mutex_*), 所以这个缓冲不会被重入
   @note 上面这条前提**曾经在板上被运行时验证过**: 2026-09-13 用一套 3 任务 x 各自
         文件的并发测试量到过"本层内峰值深度 1, 违例 0", 该测试与其探针现已删除
         (见 system/Fatfs/README.md 的"线程安全"一节). 也就是说这条注释是**量过
         的**, 不是推理 —— 但它只对"任务上下文、单卷"成立 */
static BYTE s_rmw_block[FATFS_DISK_ERASE_BYTES];
/********************************* 内部状态 ***********************************/

/********************************* 内部工具 ***********************************/

/******************************************************************************
 * @name    fatfs_disk_sector_addr
 * @brief   把 FatFs 的扇区号换算成 W25Q64 的 24bit 字节地址
 * @param   sector[in] FatFs 扇区号(0 ~ FATFS_DISK_SECTOR_COUNT-1)
 *
 * @return  W25Q64 字节地址
 *
 * @note    调用前必须已做过范围检查, 本函数不做越界保护
 *****************************************************************************/
static uint32_t fatfs_disk_sector_addr(LBA_t sector)
{
    return (uint32_t)(FATFS_DISK_BASE_ADDR +
                      ((uint32_t)sector * (uint32_t)FATFS_DISK_SECTOR_SIZE));
}

/******************************************************************************
 * @name    fatfs_disk_range_ok
 * @brief   检查 [sector, sector+count) 是否整个落在卷内
 * @param   sector[in] 起始扇区号
 * @param   count[in]  扇区个数(>= 1)
 *
 * @return  1 合法 / 0 越界
 *
 * @note    越界必须挡住而不是钳位: FatFs 传进来的地址一旦出卷, 就会踩到
 *          Bootloader / OTA 用的裸偏移区, 或者干脆戳到芯片地址空间外面
 *****************************************************************************/
static uint8_t fatfs_disk_range_ok(LBA_t sector, UINT count)
{
    if ((LBA_t)FATFS_DISK_SECTOR_COUNT <= sector)
    {
        return 0;
    }

    if (((LBA_t)FATFS_DISK_SECTOR_COUNT - sector) < (LBA_t)count)
    {
        return 0;
    }

    return 1;
}

/********************************* 对外接口 ***********************************/

/******************************************************************************
 * @name    disk_status
 * @brief   取介质状态(diskio.h 接口)
 * @param   pdrv[in] 物理盘号(本移植只认 0)
 *
 * @return  DSTATUS 状态位组合
 *****************************************************************************/
DSTATUS disk_status(BYTE pdrv)
{
    if (FATFS_DISK_PDRV != pdrv)
    {
        return (DSTATUS)(STA_NOINIT | STA_NODISK);
    }

    return s_disk_status;
}

/******************************************************************************
 * @name    disk_initialize
 * @brief   初始化介质(diskio.h 接口): 实例化 W25Q64 并核对 JEDEC ID
 * @param   pdrv[in] 物理盘号(本移植只认 0)
 *
 * @return  DSTATUS 状态位组合; 0 = 就绪
 *
 * @note    幂等: 已初始化过就直接返回, 不重复探测(FatFs 每次 f_mount 都会调)
 *
 * @note    这条路径里有 osDelay(adapter 的 inst 与擦除轮询), **必须在
 *          osKernelStart() 之后的任务上下文调用**
 *****************************************************************************/
DSTATUS disk_initialize(BYTE pdrv)
{
    uint8_t manuf_id    = 0;
    uint8_t memory_type = 0;
    uint8_t capacity    = 0;
    int8_t  ret         = 0;

    if (FATFS_DISK_PDRV != pdrv)
    {
        return (DSTATUS)(STA_NOINIT | STA_NODISK);
    }

    /* 已就绪: 幂等返回 */
    if (0 == (s_disk_status & STA_NOINIT))
    {
        return s_disk_status;
    }

    /* 1. 实例化 W25Q64(内含 SPI2 收发自检) */
    ret = storage_bsp_w25q64_inst();
    if (0 != ret)
    {
        s_disk_status = STA_NOINIT;
        return s_disk_status;
    }

    /* 2. 核对 JEDEC ID: 确认挂上来的确实是 8MB 的 W25Q64.
          本移植按 W25Q64_TOTAL_SIZE 算扇区数, 如果实际是别的容量芯片,
          卷尾的扇区就会写到芯片地址空间外面去, 必须在这里挡住 */
    ret = storage_bsp_w25q64_read_id(&manuf_id, &memory_type, &capacity);
    if (0 != ret ||
        W25Q64_MANUFACTURER_ID != manuf_id ||
        W25Q64_MEMORY_TYPE_ID  != memory_type ||
        W25Q64_CAPACITY_ID     != capacity)
    {
        s_disk_status = (DSTATUS)(STA_NOINIT | STA_NODISK);
        return s_disk_status;
    }

    s_disk_status = 0;

    return s_disk_status;
}

/******************************************************************************
 * @name    disk_read
 * @brief   读扇区(diskio.h 接口)
 * @param   pdrv[in]   物理盘号(本移植只认 0)
 * @param   buff[out]  读缓冲, 容量须 >= count x 512B(FatFs 保证)
 * @param   sector[in] 起始扇区号
 * @param   count[in]  扇区个数
 *
 * @return  RES_OK success
 *         RES_PARERR 盘号错 / 缓冲空 / count 为 0 / 越界
 *         RES_NOTRDY 介质未初始化
 *         RES_ERROR  SPI 读失败
 *
 * @note    FatFs 传来的扇区一定是连续的, 所以合并成**一次**突发读, 不必
 *          一扇区一次 CS 事务; 但单次事务有上限(见 FATFS_DISK_MAX_XFER_BYTES),
 *          超过就分几次读. 分块点取在上限而非块边界, 且上限是扇区大小的整数倍,
 *          所以每一块都仍然扇区对齐
 *
 * @note    底层读走的是 HAL_SPI_Receive(..., HAL_MAX_DELAY): 若芯片没接好/
 *          没供电, 本函数会**挂死**而不是返回 RES_ERROR. 排查"FatFs 卡住"时
 *          先量 SPI2 的 MISO/CS 与芯片供电, 别怀疑文件系统
 *****************************************************************************/
DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
    uint32_t addr     = 0;
    uint32_t size     = 0;
    uint32_t max_xfer = 0;
    uint32_t xfer     = 0;

    if (FATFS_DISK_PDRV != pdrv)
    {
        return RES_PARERR;
    }

    if (0 != (s_disk_status & STA_NOINIT))
    {
        return RES_NOTRDY;
    }

    if (NULL == buff || 0 == count)
    {
        return RES_PARERR;
    }

    if (0 == fatfs_disk_range_ok(sector, count))
    {
        return RES_PARERR;
    }

    addr     = fatfs_disk_sector_addr(sector);
    size     = (uint32_t)count * (uint32_t)FATFS_DISK_SECTOR_SIZE;
    max_xfer = (uint32_t)FATFS_DISK_MAX_XFER_BYTES;

    /* 读整簇时 count == 8, 一共 4096B < 61440, 这个循环跑一次就结束 */
    while (0 != size)
    {
        xfer = (size < max_xfer) ? size : max_xfer;

        if (0 != storage_bsp_w25q64_read(addr, (uint8_t *)buff, xfer))
        {
            return RES_ERROR;
        }

        addr += xfer;
        buff += xfer;
        size -= xfer;
    }

    return RES_OK;
}

/******************************************************************************
 * @name    disk_write
 * @brief   写扇区(diskio.h 接口): 以擦除块为单位的"读-改-写"(RMW)
 * @param   pdrv[in]   物理盘号(本移植只认 0)
 * @param   buff[in]   待写数据, 容量须 >= count x 512B(FatFs 保证)
 * @param   sector[in] 起始扇区号
 * @param   count[in]  扇区个数
 *
 * @return  RES_OK success
 *         RES_PARERR 盘号错 / 缓冲空 / count 为 0 / 越界
 *         RES_NOTRDY 介质未初始化
 *         RES_WRPRT  介质写保护(本移植恒不返回, 保留位)
 *         RES_ERROR  读回 / 擦除 / 页编程任一失败
 *
 * @note    为什么必须擦: W25Q64 的页编程只能把 bit 由 1 变 0, 不擦除直接覆写
 *          等于把新数据 AND 到旧数据上, 第二次写同一扇区必然是错的.
 *
 * @note    为什么必须读-改-写: FatFs 的扇区是 512B, 而擦除单位是 4096B —— 一个
 *          擦除块装着 8 个逻辑扇区, 写其中 1 个也**必须**把另外 7 个一起处理.
 *          直接擦掉整个块只写这 512B, 邻居会被留成 0xFF(全 1), 数据静默丢失.
 *          所以顺序固定是: 读回整块 -> 在缓冲区里覆盖这 512B -> 擦除整块 -> 写回整块.
 *
 * @note    快路径: 当这次写的范围**正好覆盖一个完整擦除块**(起始扇区对齐到块首,
 *          且 count == 8)时, 调用方缓冲里已经是整块的新数据, 读回是多余的, 直接
 *          "擦 + 写". 走这条路的只有 f_mkfs 建卷和 FatFs 整簇连续写大块数据.
 *
 * @note    代价: 写 1 个扇区 = 1 次 4KB 读 + 1 次擦除 + 1 次 4KB 编程(16 页) +
 *          1 次 4KB 写回. 这是整条链路上最慢的操作, 上层写文件的停顿全在这里.
 *
 * @note    **板上实测**(2026-09-13, 自检第 7 步打印): 单扇区 RMW **79 ms/次**,
 *          整擦除块写 **73 ms/8 扇区**. 两者只差 6 ms —— 那 6 ms 就是慢路径多出来
 *          的 1 次 4KB 读回(4096B @12.5MHz 约 2.6ms, 加命令开销与 wait_busy 轮询).
 *          => **代价在擦除, 不在读回**: 擦一次整块要几十 ms, 是 79ms 里的大头.
 *          => 想优化 RMW, 只要不减少**擦除次数**就没什么可省的(上界 6/79 ≈ 8%),
 *             "把读回的块缓存起来"这条路基本可以放弃.
 *          写一个 9KB 文件(自检那样每 256B 一次 f_write)触发 28 次 disk_write
 *          = 28 次擦除, 其中 23 次要先读回 4KB => 28 x 79ms ≈ **2.2s**.
 *          @note 簇大小帮不上忙: FatFs 只在"fptr 落在簇边界且一次给够整簇"时才
 *                发大块写, 上层按 256B 喂数据时它攒满 512B(一个扇区)就刷出去,
 *                所以每一次都还是单扇区 RMW. 真正决定开销的是**写了多少个扇区**.
 *
 * @note    这是**朴素 RMW**, 不缓存擦除块. 若将来实测延迟不可接受, 可选优化是
 *          把 s_rmw_block 变成一个"当前块缓存 + 脏标志": 同一块被连续修改时只
 *          擦写一次. 注意那会让 disk_write 与 disk_read 必须走同一份缓存, 复杂
 *          度明显上升, 且掉电时缓存里的内容会丢 —— 现在这版没有这些负担.
 *
 * @note    每次 SPI 事务都是 4096B, 低于底层 uint16_t 的传输上限(见
 *          FATFS_DISK_MAX_XFER_BYTES), 所以不需要像 disk_read 那样再分块
 *****************************************************************************/
DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
    /* @note 慢路径会写 s_rmw_block 这个**全文件共用**的 4KB 缓冲, 两个任务同时进来
       就是互相踩 —— 这一层正是"卷锁到底有没有真的生效"要看的那个点. 板上量过:
       峰值深度 1 / 违例 0(见上面 s_rmw_block 的 @note) */
    UINT     i        = 0;
    uint32_t addr     = 0;
    uint32_t blk_addr = 0;
    uint32_t off      = 0;
    UINT     in_blk   = 0;
    UINT     n        = 0;
    const BYTE *p_src = NULL;

    if (FATFS_DISK_PDRV != pdrv)
    {
        return RES_PARERR;
    }

    if (0 != (s_disk_status & STA_NOINIT))
    {
        return RES_NOTRDY;
    }

    if (0 != (s_disk_status & STA_PROTECT))
    {
        return RES_WRPRT;
    }

    if (NULL == buff || 0 == count)
    {
        return RES_PARERR;
    }

    if (0 == fatfs_disk_range_ok(sector, count))
    {
        return RES_PARERR;
    }

    /* 按"落在同一个擦除块内的连续段"推进. 用循环而不是单次处理, 因为一次
       disk_write 可能横跨两个擦除块(例如写扇区 6~7: 从块内偏移 3072 开始,
       写到块尾就断, 剩下的落在下一个块里) */
    while (i < count)
    {
        addr     = fatfs_disk_sector_addr(sector + (LBA_t)i);
        blk_addr = addr & ~((uint32_t)FATFS_DISK_ERASE_BYTES - 1U);
        off      = addr - blk_addr;

        /* 本擦除块内还剩几个扇区可写(off 必是扇区对齐的, 所以不会算出 0) */
        in_blk = (UINT)((FATFS_DISK_ERASE_BYTES - off) /
                        FATFS_DISK_SECTOR_SIZE);
        n      = ((count - i) < in_blk) ? (count - i) : in_blk;

        p_src = &buff[(uint32_t)i * (uint32_t)FATFS_DISK_SECTOR_SIZE];

        if (0 == off && FATFS_DISK_ERASE_BLOCK_SS == n)
        {
            /* 快路径: 正好一个整块, 调用方缓冲里就是整块新数据, 不必读回.
               @note 这么写也顺带避开了"读回自己刚要被覆盖的内容"的无用功 */

            /* 1. 擦除整块 */
            if (0 != storage_bsp_w25q64_erase_sector(blk_addr))
            {
                return RES_ERROR;
            }

            /* 2. 页编程写整块. adapter 的写接口形参不是 const, 这里显式脱掉
                  const: 底层只读这块内存, 不会改写它 */
            if (0 != storage_bsp_w25q64_write(blk_addr,
                                              (uint8_t *)(void *)p_src,
                                              (uint32_t)FATFS_DISK_ERASE_BYTES))
            {
                return RES_ERROR;
            }
        }
        else
        {
            /* 慢路径 —— 文件系统跑起来之后, 绝大多数 disk_write 都走这里 */

            /* 1. 读回整个擦除块(4KB), 邻居的数据必须先拿到手 */
            if (0 != storage_bsp_w25q64_read(blk_addr, s_rmw_block,
                                             (uint32_t)FATFS_DISK_ERASE_BYTES))
            {
                return RES_ERROR;
            }

            /* 2. 在缓冲区里把要写的那 n 个扇区覆盖掉, 其余字节原样保留 */
            memcpy(&s_rmw_block[off], p_src,
                   (uint32_t)n * (uint32_t)FATFS_DISK_SECTOR_SIZE);

            /* 3. 擦除整块 */
            if (0 != storage_bsp_w25q64_erase_sector(blk_addr))
            {
                return RES_ERROR;
            }

            /* 4. 整块写回: 新数据 + 原样保留的邻居 */
            if (0 != storage_bsp_w25q64_write(blk_addr, s_rmw_block,
                                              (uint32_t)FATFS_DISK_ERASE_BYTES))
            {
                return RES_ERROR;
            }
        }

        i += n;
    }

    return RES_OK;
}

/******************************************************************************
 * @name    disk_ioctl
 * @brief   杂项控制(diskio.h 接口)
 * @param   pdrv[in] 物理盘号(本移植只认 0)
 * @param   cmd[in]  控制码(CTRL_SYNC / GET_SECTOR_COUNT / GET_SECTOR_SIZE /
 *                   GET_BLOCK_SIZE / CTRL_TRIM)
 * @param   buff[out] 与 cmd 对应的缓冲; CTRL_SYNC/CTRL_TRIM 按约定传 NULL
 *
 * @return  RES_OK success
 *         RES_PARERR 盘号错 / 未知 cmd / 该 cmd 需要 buff 但传了 NULL
 *         RES_NOTRDY 介质未初始化
 *         RES_ERROR  CTRL_SYNC 等芯片就绪失败
 *****************************************************************************/
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    /* @note ioctl 不碰任何共享缓冲, 但它的 CTRL_SYNC 会在**同一条 SPI2** 上等
       芯片忙(wait_busy 是轮询), 期间有别的任务进来读写就是两个 SPI 事务交错.
       本层的真实前提是"没有两个任务同时在这一层内", 不只是"没同时进 RMW" */
    if (FATFS_DISK_PDRV != pdrv)
    {
        return RES_PARERR;
    }

    if (0 != (s_disk_status & STA_NOINIT))
    {
        return RES_NOTRDY;
    }

    switch (cmd)
    {
    /* 等待挂起的写完成. 本移植的 adapter 在每次擦/写结束都已 pf_wait_busy
       等芯片就绪, 没有挂在芯片内部的写缓冲, 所以这里只需再确认一次"芯片空闲".
       @note buff 按约定是 NULL, 所以本分支必须排在下面 buff 判空**之前** */
    case CTRL_SYNC:
        return (0 == storage_bsp_w25q64_wait_busy()) ? RES_OK : RES_ERROR;

    /* 介质容量, 单位 = 扇区 */
    case GET_SECTOR_COUNT:
        if (NULL == buff)
        {
            return RES_PARERR;
        }
        *(LBA_t *)buff = (LBA_t)FATFS_DISK_SECTOR_COUNT;
        return RES_OK;

    /* 扇区大小, 单位 = 字节. FF_MAX_SS != FF_MIN_SS 时 ff.c 会来问.
       @note 本工程的 ffconf.h 已把 FF_MIN_SS == FF_MAX_SS == 512, 所以 ff.c 走的是
             编译期定值, **根本不会调到这里**. 实现留着是为了两件事: 一是万一将来
             把 FF_MAX_SS 放大(重新支持变扇区), 这里已经是对的; 二是让"本盘的扇区
             就是 512B"这件事在代码里有一处可读的出处, 不必翻 ffconf.h */
    case GET_SECTOR_SIZE:
        if (NULL == buff)
        {
            return RES_PARERR;
        }
        *(WORD *)buff = (WORD)FATFS_DISK_SECTOR_SIZE;
        return RES_OK;

    /* 擦除粒度, 单位 = 扇区 = 8. f_mkfs 用它给数据区(簇)起始位置做对齐,
       使数据区不与前面 FAT/根目录区挤在同一个擦除块里.
       @note 这个值是 8 而不是 1, 正是"扇区(512B) != 擦除单位(4096B)"的直接体现 */
    case GET_BLOCK_SIZE:
        if (NULL == buff)
        {
            return RES_PARERR;
        }
        *(DWORD *)buff = FATFS_DISK_ERASE_BLOCK_SS;
        return RES_OK;

    /* CTRL_TRIM: buff 指向 LBA_t lba[2], 是 [起, 止]**闭区间**, 表示这段数据
       已作废、可提前擦除.
       @note 本移植**故意不做物理擦除**, 直接回 RES_OK = "知道了".
             理由: 提前擦对 RMW 一点用都没有 —— RMW 每次写都要先把整块读回来,
             块里是不是 0xFF 都改变不了"擦一次"这个事实, 省不下任何一次擦除;
             而唯一会真的调 CTRL_TRIM 的是 f_mkfs 对整卷的一次 trim —— 真按物理
             擦除实现, 就是 16384 个扇区 / 2048 个擦除块(板上实测块擦 73ms,
             合计约 150s)的格式化等待, 换来的是"第一次写略微快一点".
             好处: f_unlink / f_mkfs 立刻返回, 不做无谓的整片擦除 */
    case CTRL_TRIM:
        return RES_OK;

    default:
        return RES_PARERR;
    }
}
