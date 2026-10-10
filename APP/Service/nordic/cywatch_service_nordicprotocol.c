/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_service_nordicprotocol.c
 *
 * @par dependencies
 * - cywatch_service_nordicprotocol.h
 * - cywatch_adapter_nordic.h
 * - cmsis_os2.h
 *
 * @author	zw1194
 *
 * @brief Nordic 链路服务的薄壳, 见同名 .h 的说明. 本文件只有三件事:
 *        ① 挂两个任务(nordic_bsp_rx_entry / nordic_bsp_tx_entry);
 *        ② 转发"注册接收回调";
 *        ③ 转发"发一条数据".
 *        4 个 RTOS 对象、注入实现、板级绑定都在 Nordic/adapter/cywatch_adapter_nordic.c.
 *
 * Processing flow:
 *
 * 见同名 .h 的 Processing flow.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note ★本文件对三个头用的是"裸名 include"★(`"cywatch_adapter_nordic.h"` /
 *       `"cmsis_os2.h"`). Keil 侧靠工程 IncludePath 解析; 宿主回归测靠 -I 顺序换成
 *       替身. **相对路径的 include 拦不住 -I**(相对路径是按"包含它的文件"所在目录
 *       解析的), 所以这里必须是裸名 —— 这是这套代码能在 PC 上被真实编译测试的前提,
 *       不是风格问题, 改回相对路径会让宿主测编不过.
 *****************************************************************************/
#include "cywatch_service_nordicprotocol.h"
#include "cywatch_adapter_nordic.h"
#include "cmsis_os2.h"

/***********************************Defines************************************/
/* RX 任务栈: 最坏路径 = 重扫嵌套(深度上限 8, 每层一个 135B 的 rescan 数组 → 约
   1.1KB)里的解析 + 重组 + 回调, 合计约 1.8KB, 且 Cortex-M 的**中断借用任务栈**
   (RX DMA 5 / USART1 7 可能再叠约 200B). 给 3072, 余量约 1.2KB ——
   configCHECK_FOR_STACK_OVERFLOW 是 0, 栈溢出在本工程是**静默踩堆**;
   帧长上限(NORDIC_MAX_FRAME)再涨就回来重算这个数. */
#define SVC_NORDIC_RX_STACK           3072u
#define SVC_NORDIC_TX_STACK           2048u
/***********************************Defines************************************/

/**********************************Declaring***********************************/
static uint8_t s_inited = 0u;   /* 幂等守卫 */
/**********************************Functions***********************************/

int8_t service_nordicprotocol_init(void)
{
	osThreadAttr_t attr = {0};
	nordic_cfg_t cfg;
	int8_t ret;

	if (0u != s_inited)
	{
		return 0; /* 幂等(见 .h 的 @note: 重复初始化会造出 4 个任务抢同一批信号量) */
	}

	/* ---- 1. 协议参数: 取协议建议值(doc/协议.txt); 协议核构造时会**拷一份**进实例,
	       所以本缓冲放栈上即可(见 BSP 头的 nordic_cfg_t 注释) ---- */
	cfg.frame_timeout_ms = 100u;  /* 帧级 ACK 超时 */
	cfg.frame_max_retry  = 5u;    /* 帧级重传上限 */

	/* ---- 2. adapter 一条龙: 4 个 RTOS 对象 → 板级绑定 → 协议核构造 ---- */
	ret = nordic_bsp_inst(&cfg);
	if (0 != ret)
	{
		/* 原样透传 adapter 的 -1..-6, 不做二次映射(见 .h 的 @note: 保住 -4
		   "NVIC 优先级不合规"这条上板排障路径的粒度) */
		return ret;
	}

	/* ---- 3. 挂两个任务(任务体在 BSP 层, 本层只分栈与优先级) ---- */
	attr.name       = "nordic_rx";
	attr.stack_size = SVC_NORDIC_RX_STACK;
	attr.priority   = osPriorityAboveNormal; /* ★RX 必须高于 TX: 协议核硬性要求★ */
	if (NULL == osThreadNew(nordic_bsp_rx_entry, NULL, &attr))
	{
		/* 建成了一半也要收干净: deinst 会释放两个信号量把(此刻还没起来的)RX 任务
		   叫醒, 它醒来见到未初始化就自行 return —— 不留永久阻塞在信号量上的线程 */
		(void)nordic_bsp_deinst();
		return -7; /* 内核堆不足时静默失败(configASSERT 关着), 必须自己查 */
	}

	attr.name       = "nordic_tx";
	attr.stack_size = SVC_NORDIC_TX_STACK;
	attr.priority   = osPriorityNormal;
	if (NULL == osThreadNew(nordic_bsp_tx_entry, NULL, &attr))
	{
		(void)nordic_bsp_deinst(); /* 同上: RX 任务会自行退出, 不留泄漏 */
		return -7;
	}

	s_inited = 1u;
	return 0;
}

int8_t service_nordicprotocol_register_feature(uint8_t feature, nordic_rx_cb_t pf_cb)
{
	return nordic_bsp_register_feature(feature, pf_cb);
}

int8_t service_nordicprotocol_send(uint8_t feature, const uint8_t *pdata, uint16_t len,
								   nordic_tx_done_cb_t pf_done)
{
	return nordic_bsp_send(feature, pdata, len, pf_done);
}
