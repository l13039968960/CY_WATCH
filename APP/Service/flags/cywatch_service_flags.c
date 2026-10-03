/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_service_flags.c
 *
 * @par dependencies
 * - cywatch_service_flags.h
 * - cywatch_adapter_at24c02.h  (BSP/AT24C02)
 * - cywatch_log.h
 * - cmsis_os2.h
 *
 * @author	zw1194
 *
 * @brief flags 服务: 在 AT24C02(256B EEPROM) 上提供同步阻塞的字节读写 + 一套上电自检.
 *
 * Processing flow:
 *
 * 1. service_flags_init() 建一个一次性任务;
 * 2. 该任务: storage_bsp_at24c02_inst() -> (开关打开时)flags_selftest() -> osThreadExit();
 * 3. 此后任意任务调 service_flags_read/write, 每个都是"查状态 -> 转一次驱动 API ->
 *    转一次返回码", 同步返回.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 自检的两块 256B 缓冲是文件级 static(进 .bss, 共 512B), 不放栈上: 本工程
 *       configCHECK_FOR_STACK_OVERFLOW == 0, 栈溢出**不会报错**, 只会踩坏相邻的
 *       内核堆块. 一次性的自检没必要为此把任务栈撑到 1.5KB.
 *****************************************************************************/
/***********************************Includes***********************************/
#include "cywatch_service_flags.h"
#include "cywatch_adapter_at24c02.h"
#include "cywatch_bsp_at24c02_reg.h" /* AT24C02_TOTAL_SIZE */
#include "cmsis_os2.h"
#include "system/log/cywatch_log.h" /* log_printf() */
/***********************************Includes***********************************/

/***********************************Defines************************************/
/* 一次性任务的栈. 构造 AT24C02 的调用链很短(wait_busy 里只有 osDelay), 自检的
   两块大缓冲又都是 static, 1KB 有一倍余量 */
#define FLAGS_BOOT_STACK            (1024U)
/***********************************Defines************************************/

/**********************************Variables***********************************/
/* 服务状态. 都是 uint8_t: Cortex-M4 上单字节读写天然原子, 不需要临界区 */
static uint8_t s_inited = 0U;   /* service_flags_init() 是否已经建过任务 */
static uint8_t s_ready  = 0U;   /* AT24C02 构造好没 —— 只从 0 变成 1 */

/* 一次性任务属性. 常量, 放 flash. cb_mem/stack_mem 留 NULL: 让 CMSIS-RTOS2 从
   FreeRTOS 内核堆里分配 TCB 与栈 */
static const osThreadAttr_t g_flags_boot_attr =
{
    .name       = "flags",
    .attr_bits  = 0U,
    .cb_mem     = NULL,
    .cb_size    = 0U,
    .stack_mem  = NULL,
    .stack_size = FLAGS_BOOT_STACK,
    .priority   = osPriorityNormal,
};
/**********************************Variables***********************************/

/******************************Static Functions********************************/
#if (0 != SERVICE_FLAGS_SELFTEST)
/******************************************************************************
 * @name    flags_selftest
 * @brief   读写闭环: 整片 256B 写递增模式 -> 读回比对, 再验一次非对齐跨页写
 * @param   无
 * @return  无(结果打串口)
 *
 * @note    第 2 步**故意**从 offset 5 起写 20B: 它跨 3 个页(5~7 / 8~15 / 16~24),
 *          走的是驱动里"按页拆分"那条路径. 只测对齐写的话, 页拆分逻辑等于没测.
 *****************************************************************************/
static void flags_selftest(void)
{
    static uint8_t s_wbuf[AT24C02_TOTAL_SIZE];
    static uint8_t s_rbuf[AT24C02_TOTAL_SIZE];
    uint32_t t0 = 0U;
    uint32_t dt = 0U;
    uint16_t i  = 0U;

    /* ---- 1. 整片 256B: 写递增模式 -> 读回逐字节比对 ---- */
    for (i = 0U; i < AT24C02_TOTAL_SIZE; i++)
    {
        s_wbuf[i] = (uint8_t)i;
    }

    t0 = osKernelGetTickCount();
    if (SERVICE_FLAGS_OK != service_flags_write(0U, s_wbuf, AT24C02_TOTAL_SIZE))
    {
        log_printf("[FLAGS] 自检: 整片写 256B 失败(查接线/供电/上拉)\r\n");
        return;
    }
    dt = osKernelGetTickCount() - t0; /* 32 页, 每页最多 5ms 写周期 */

    if (SERVICE_FLAGS_OK != service_flags_read(0U, s_rbuf, AT24C02_TOTAL_SIZE))
    {
        log_printf("[FLAGS] 自检: 整片读 256B 失败\r\n");
        return;
    }

    for (i = 0U; i < AT24C02_TOTAL_SIZE; i++)
    {
        if (s_rbuf[i] != (uint8_t)i)
        {
            log_printf("[FLAGS] 自检: 整片比对失败 @%u 期望=%02X 实读=%02X\r\n",
                       (unsigned)i, (unsigned)(uint8_t)i, (unsigned)s_rbuf[i]);
            return;
        }
    }
    log_printf("[FLAGS] 自检: 整片 256B 读写一致(写耗时 %u ms)\r\n", (unsigned)dt);

    /* ---- 2. 非对齐跨页: offset 5 起写 20B(跨 3 页) -> 读回比对 ---- */
    for (i = 0U; i < 20U; i++)
    {
        s_wbuf[i] = (uint8_t)(0xA0U + i);
    }

    if (SERVICE_FLAGS_OK != service_flags_write(5U, s_wbuf, 20U))
    {
        log_printf("[FLAGS] 自检: 非对齐写(offset=5, 20B)失败\r\n");
        return;
    }

    if (SERVICE_FLAGS_OK != service_flags_read(5U, s_rbuf, 20U))
    {
        log_printf("[FLAGS] 自检: 非对齐读失败\r\n");
        return;
    }

    for (i = 0U; i < 20U; i++)
    {
        if (s_rbuf[i] != (uint8_t)(0xA0U + i))
        {
            log_printf("[FLAGS] 自检: 非对齐比对失败 @%u 期望=%02X 实读=%02X\r\n",
                       (unsigned)(5U + i), (unsigned)(uint8_t)(0xA0U + i),
                       (unsigned)s_rbuf[i]);
            return;
        }
    }
    log_printf("[FLAGS] 自检: 非对齐跨页(offset=5, 20B)读写一致\r\n");
}
#endif /* SERVICE_FLAGS_SELFTEST */

/******************************************************************************
 * @name    service_flags_boot_task
 * @brief   一次性任务体: 构造 AT24C02 实例 (+自检), 然后退出
 * @param   p_arg[in] 未使用
 * @return  无(末尾 osThreadExit)
 *
 * @note    跑完就 osThreadExit(), 而不是常驻空转: 存储服务不需要周期执行, 1KB 栈
 *          一直被占着不值当; 退出后空闲任务会把它归还内核堆.
 * @note    构造成功后器件实例仍是有效的(adapter 里是文件级 static), 后续消费者
 *          直接复用即可.
 * @note    构造失败不跑自检: 自检第一步就是读写, 器件不在只会再失败一次.
 *****************************************************************************/
static void service_flags_boot_task(void *p_arg)
{
    (void)p_arg;

    /* 内含上电延时(1ms)与写周期 ACK 探测, 都走 osDelay —— 必须在任务上下文里调 */
    s_ready = (0 == storage_bsp_at24c02_inst()) ? 1U : 0U;

#if (0 != SERVICE_FLAGS_SELFTEST)
    if (0U != s_ready)
    {
        flags_selftest();
    }
    else
    {
        log_printf("[FLAGS] 自检: AT24C02 构造失败, 跳过读写闭环\r\n");
    }
#endif

    osThreadExit();
}
/******************************Static Functions********************************/

/**********************************Functions***********************************/
int8_t service_flags_init(void)
{
    /* 幂等: 重复调用直接当成功, 不会建出第二个任务 */
    if (0U != s_inited)
    {
        return SERVICE_FLAGS_OK;
    }

    if (NULL == osThreadNew(service_flags_boot_task, NULL, &g_flags_boot_attr))
    {
        return -1;
    }

    s_inited = 1U;
    return SERVICE_FLAGS_OK;
}

uint8_t service_flags_is_ready(void)
{
    return s_ready;
}

int8_t service_flags_read(uint16_t offset, uint8_t *p_buf, uint16_t len)
{
    if (0U == s_inited)
    {
        return SERVICE_FLAGS_ERR_NO_INIT;
    }

    if (NULL == p_buf || 0U == len)
    {
        return SERVICE_FLAGS_ERR_NULL_ARG;
    }

    if (0U == s_ready)
    {
        return SERVICE_FLAGS_ERR_NOT_READY;
    }

    return (0 == storage_bsp_at24c02_read(offset, p_buf, len)) ?
           SERVICE_FLAGS_OK : SERVICE_FLAGS_ERR_BUS;
}

int8_t service_flags_write(uint16_t offset, uint8_t *p_buf, uint16_t len)
{
    if (0U == s_inited)
    {
        return SERVICE_FLAGS_ERR_NO_INIT;
    }

    if (NULL == p_buf || 0U == len)
    {
        return SERVICE_FLAGS_ERR_NULL_ARG;
    }

    if (0U == s_ready)
    {
        return SERVICE_FLAGS_ERR_NOT_READY;
    }

    return (0 == storage_bsp_at24c02_write(offset, p_buf, len)) ?
           SERVICE_FLAGS_OK : SERVICE_FLAGS_ERR_BUS;
}
/**********************************Functions***********************************/
