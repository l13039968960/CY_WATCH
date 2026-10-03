/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_service_flags.h
 *
 * @par dependencies
 * - stdint.h
 *
 * @author	zw1194
 *
 * @brief flags 服务: AT24C02(256B EEPROM) 的同步阻塞薄封装 —— 建实例 + 统一返回码.
 *
 * Processing flow:
 *
 * 1. main.c 在 osKernelStart() 之前调 service_flags_init();
 * 2. 服务建一个一次性任务: 构造 AT24C02 实例 -> (开关打开时)自检 -> osThreadExit;
 * 3. 之后任意任务调 service_flags_read/write, 同步返回.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 本服务只做三件事: AT24C02 实例的构造与状态、统一返回码、空参数保护.
 *       存什么、怎么布局由调用方自己定 —— 整片 256B 从 offset 0 起都可用.
 *       地址越界不在这里拦: 驱动的 at24c02_read/write 已经查过, 会返回 SERVICE_
 *       FLAGS_ERR_BUS. 驱动内部是哪一步失败(-1 空参数 / -3 越界 / -4 -5 I2C)
 *       没有细分, 需要时看驱动头文件的返回码表.
 *
 * @note 初始化的异步的: service_flags_init() 返回时 AT24C02 **还没构造好**,
 *       看 service_flags_is_ready(). 在那之前调读写一律返回 NOT_READY.
 *
 * @note **不保证并发安全**: AT24C02 独占那条软件 I2C(PB10=SCL / PB3=SDA) 线上只有
 *       它一个器件, main.c 建这条总线时没注入互斥量(理由见 main.c 里那段说明).
 *       多任务同时读写本服务会踩位带时序 —— 要并发就得先给那条总线补上互斥量.
 *
 * @note 所有 API 都要在 osKernelStart() 之后的任务上下文里调: 底层经 AT24C02
 *       adapter 访问 EEPROM, 而它的 pf_delay 是 osDelay, 调度器没起来会卡死.
 *****************************************************************************/
#ifndef __CYWATCH_SERVICE_FLAGS_H__
#define __CYWATCH_SERVICE_FLAGS_H__

/***********************************Includes***********************************/
#include <stdint.h>
/***********************************Includes***********************************/

/***********************************Defines************************************/
/* 上电自检开关: 1 = 构造成功后跑一遍读写闭环(结果经串口打出来), 0 = 只构造不打印.
   自检内容: 整片 256B 写递增模式 -> 读回逐字节比对 -> 再从 offset 5 写 20B(跨 3 页,
   验驱动按页拆分) -> 读回比对. 上电一眼就能确认 I2C 接线与 EEPROM 是通的.
   ★代价: 每次上电重写整片, 消耗一次擦写寿命(AT24C02 约 100 万次, 影响很小);
     测完改回 0★ */
#define SERVICE_FLAGS_SELFTEST          (0U)

/* ---- 统一返回码 ----
   全为负数: 内部一律把驱动的失败压成一个值, 调用方不需要认识驱动的码表 */
#define SERVICE_FLAGS_OK                (0)     /* 成功 */
#define SERVICE_FLAGS_ERR_NO_INIT       (-1)    /* 没调 service_flags_init(), 或它失败了 */
#define SERVICE_FLAGS_ERR_NULL_ARG      (-2)    /* 传了空指针 / len 为 0 */
#define SERVICE_FLAGS_ERR_NOT_READY     (-3)    /* AT24C02 实例还没构造好(任务还没跑到) */
#define SERVICE_FLAGS_ERR_BUS           (-4)    /* 底层读写失败: 器件无应答, 或地址越界 */
/***********************************Defines************************************/

/**********************************Declaring***********************************/
/**
 * @name  service_flags_init
 * @brief 建"构造 AT24C02 + 自检"的一次性任务. 幂等
 * @return 0 成功(含重复调用) / -1 任务创建失败(内核堆不足)
 *
 * @note 可在 osKernelStart() 之前调用: 本函数只建任务对象, 不碰设备.
 * @note 返回值必须看: -1 只可能是内核堆不足, 而 configASSERT 与
 *       configUSE_MALLOC_FAILED_HOOK 都是关的 —— 分配失败是静默的, 表现是
 *       "读写 API 全部返回 -3, 串口上一个字都没有".
 */
int8_t service_flags_init(void);

/**
 * @name  service_flags_is_ready
 * @brief 查 AT24C02 构造好没
 * @return 1 = 可用 / 0 = 还没好(任务还没跑到, 或构造失败——器件无应答)
 * @note 无锁: 单字节读写原子, 读到"稍旧的 0"只意味着"还没好", 那是真话不是错报.
 */
uint8_t service_flags_is_ready(void);

/**
 * @name  service_flags_read
 * @brief 从 EEPROM 的指定偏移读一段. 地址自动递增
 * @param offset[in]  EEPROM 内偏移(0 ~ 255)
 * @param p_buf[out]  接收缓冲
 * @param len[in]     读取字节数
 * @return 0 成功 / <0 见 SERVICE_FLAGS_ERR_*
 */
int8_t service_flags_read(uint16_t offset, uint8_t *p_buf, uint16_t len);

/**
 * @name  service_flags_write
 * @brief 往 EEPROM 的指定偏移写一段. 自动按 8 字节页拆分, 支持跨页
 * @param offset[in]  EEPROM 内偏移(0 ~ 255)
 * @param p_buf[in]   待写数据
 * @param len[in]     写入字节数
 * @return 0 成功 / <0 见 SERVICE_FLAGS_ERR_*
 *
 * @note 返回 0 代表写周期已结束(驱动每页写完都轮询过 ACK), 数据已经真正落进 EEPROM,
 *       不像 FatFs 那样还有一层缓冲.
 * @note 整片写完要跨 32 页, 每页最多等 5ms 写周期 —— 写满 256B 最坏约 160ms.
 */
int8_t service_flags_write(uint16_t offset, uint8_t *p_buf, uint16_t len);
/**********************************Declaring***********************************/

#endif // __CYWATCH_SERVICE_FLAGS_H__
