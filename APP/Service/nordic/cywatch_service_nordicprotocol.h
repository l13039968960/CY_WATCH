/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_service_nordicprotocol.h
 *
 * @par dependencies
 * - cywatch_adapter_nordic.h
 * - stdint.h
 *
 * @author	zw1194
 *
 * @brief Nordic 链路服务: 把协议核的两个阻塞式任务体挂到本工程的 RTOS 上, 并把
 *        "注册接收回调"与"发一条数据"两个口转给应用.
 *        本层**不解析协议、不组帧、不碰总线** —— 解析与停等重传全在 BSP/Nordic,
 *        4 个 RTOS 对象与板级绑定全在 Nordic/adapter.
 *
 * Processing flow:
 *
 *   service_nordicprotocol_init()
 *     → nordic_bsp_inst(&cfg)             adapter: 4 个 RTOS 对象 + 板级绑定 + 协议核构造
 *     → osThreadNew(nordic_bsp_rx_entry)  RX 任务(第一句武装 RX DMA + IDLE)
 *     → osThreadNew(nordic_bsp_tx_entry)  TX 任务
 *
 *   之后应用只用下面两个口:
 *     service_nordicprotocol_register_feature(特征号, 回调)  每个要收的特征注册一次
 *     service_nordicprotocol_send(特征号, 数据, 长度, 回调)   单帧还是消息由特征号区间决定
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note ★本服务建**两个**任务(偏离"一个服务一个 osThreadNew"的惯例)★: 两个任务体是
 *       同一个状态机的两半(共享发送环与那 4 个 RTOS 对象), 靠"RX 收字节解析 / TX 组帧
 *       停等重传"分工; 而 ★RX 优先级必须高于 TX★ 是协议核的硬性要求(否则"单轮工作
 *       有界"那条保证失效, 见 cywatch_bsp_nordic_driver.h 的 NORDIC_RX_BURST 说明).
 *       合并成一个任务 = 改协议行为. 任务体在 BSP 层(本层不碰), 本层只分配栈与优先级.
 *
 * @note ★本头只 include adapter 头, 不直接 include BSP 驱动头★: adapter 头已经把
 *       nordic_cfg_t / nordic_rx_cb_t / nordic_tx_done_cb_t 转发出来了. 本层因此一个
 *       HAL 类型、一个 pf_*、一个寄存器都不碰.
 *
 * @note ★注册与挂任务之间没有互斥★: 协议核的回调表只被 RX 任务读、不做并发保护, 而
 *       init() 返回时两个任务已经挂上了. 建议 init() 一返回就把该注册的注册完 ——
 *       这之前到来的帧因表项为空被协议核丢弃(★ACK 照发★, 链路层看不出异常), 不会出错.
 *****************************************************************************/
#ifndef __CYWATCH_SERVICE_NORDICPROTOCOL_H__
#define __CYWATCH_SERVICE_NORDICPROTOCOL_H__

/***********************************Includes***********************************/
#include <stdint.h>
#include "cywatch_adapter_nordic.h"
/***********************************Includes***********************************/

/**********************************Declaring***********************************/
/**
 * @name  service_nordicprotocol_init
 * @brief 链路协议服务初始化: adapter 一条龙(建 4 个 RTOS 对象 + 板级绑定 + 协议核构造)
 *        + 挂 RX/TX 两个任务
 * @param 无
 * @return 0 成功
 *         -1 配置为空(本服务内部不会, 走到这里说明代码被改过)
 *         -2 4 个 RTOS 对象创建失败(FreeRTOS 内核堆不足)
 *         -3 DMA 初始化失败
 *         -4 NVIC 中断优先级不满足 FreeRTOS 约束(★"上电即死"路径, 上板排障先看它★)
 *         -5 UART 驱动起不来
 *         -6 协议核构造失败(BSP 的 -1 实例/-2 cfg/-3 uart/-4 sem/-5 mutex/-6 timebase)
 *         -7 任务创建失败(内核堆不足; 会先 deinst 掉已建的那一半, 见 .c 说明)
 * @note 可在 osKernelStart() 之前调用(只建对象/配寄存器, 不做阻塞动作; HAL_DMA_Init
 *       用 HAL tick, 而 HAL_Init() 早于本调用).
 *       ★幂等★: 重复调用直接返回 0(重复初始化会造出 4 个任务抢同一批信号量, 字节流
 *       被分走且没有任何报错).
 * @note 本函数**不**武装 RX 接收: 那一步在 RX 任务第一句做, 理由是上电瞬间 PA10 悬空
 *       易被噪声触发起始位, 把手收字节的时间推后到"解析器已经就位"那一刻.
 * @note 返回码 -1..-6 由 adapter **原样透传**, 本层不做二次映射 —— 保住上板排障的
 *       粒度(尤其是 -4).
 */
int8_t service_nordicprotocol_init(void);

/**
 * @name  service_nordicprotocol_register_feature
 * @brief 注册某个特征号的接收回调(收到单帧整包 / 一条消息重组完成时触发)
 * @param feature[in] 特征号(应用可用 0x02-0xFF; 0x00/0x01 是协议自用)
 * @param pf_cb[in]   回调, NULL 表示不关心该特征
 * @return 0 成功 / -1 未实例化
 * @note ★回调运行在 RX 任务上下文, 且 pdata 指向协议核内部缓冲 —— 返回后即可被下一条
 *       覆盖, 要留用必须自己拷走★.
 * @note ★该回调里只许做有界小活★(拷走 + 发事件): RX 任务的优先级高于 TX, 在里面写
 *       flash 或 printf 会把整条链路的发送方向一起拖住.
 */
int8_t service_nordicprotocol_register_feature(uint8_t feature, nordic_rx_cb_t pf_cb);

/**
 * @name  service_nordicprotocol_send
 * @brief 发一条数据(应用入口, 非阻塞). 走单帧还是消息由特征号所在区间决定, 调用方
 *        不用选: 0x02-0x80 = 单帧数据包 / 0x81-0xFF = 分包消息
 * @param feature[in] 特征号
 * @param pdata[in]   数据
 * @param len[in]     长度(≤ NORDIC_TX_MSG_MAX = 4096)
 * @param pf_done[in] 发送完成回调(可为 NULL), 在 **TX 任务上下文**触发
 * @return 0 成功入环 / -1 未实例化或未初始化 / -2 参数错 / -3 超长 / -4 环空间不够
 * @note ★非阻塞★: 环里放不下立即返回, 绝不挂起调用方. 入环即拷贝, 返回后 pdata
 *       可立刻复用.
 * @note ★特征号 ≤ 0x80 时 len 必须 ≤ 32★(单帧的 payload 就是数据本身, 超了会溢出发
 *       送侧的栈缓冲). 本层与协议核**都不做这个校验**, 违背 = 未定义行为 —— 要发长
 *       数据就把特征号换到 0x81-0xFF.
 * @note pf_done 的参数只说明本条的结果(result: 0 成功 / -5 帧级失败 / -6 消息级失败),
 *       ★不带特征号也不带编号★ —— 要区分是哪一条, 就各传各的回调函数.
 */
int8_t service_nordicprotocol_send(uint8_t feature, const uint8_t *pdata, uint16_t len,
								   nordic_tx_done_cb_t pf_done);

/**********************************Declaring***********************************/

#endif // __CYWATCH_SERVICE_NORDICPROTOCOL_H__
