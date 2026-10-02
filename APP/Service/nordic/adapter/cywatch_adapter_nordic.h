/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_adapter_nordic.h
 *
 * @par dependencies
 * - stdint.h
 * - cywatch_bsp_nordic_driver.h
 *
 * @author	zw1194
 *
 * @brief Nordic 链路(BSP/Nordic) → 服务(service/NordicProtocol)的适配层:
 *        4 个 RTOS 对象 + 板级绑定(USART1 + 两个 DMA 流 + 三个中断入口 + NVIC)
 *        + 两个任务入口包装, 全在这里; 服务只经本头的函数碰这条链路,
 *        不碰 HAL、不碰 cmsis_os2。
 *
 * Processing flow:
 *
 *   service_nordicprotocol_init()
 *     → nordic_bsp_inst(&cfg)  建 4 个 RTOS 对象 → 板级绑定 → uart_driver_inst
 *       + uart pf_init(真正初始化 USART1) → nordic_inst(&实例, &cfg, &四个接口)
 *     → nordic_bsp_register_feature(特征号, 回调)   每个要收的特征注册一次
 *     → osThreadNew(nordic_bsp_rx_entry)  武装 RX + 进 BSP 的阻塞任务体
 *     → osThreadNew(nordic_bsp_tx_entry)  进 BSP 的阻塞任务体
 *
 *   BSP 层调注入接口 → 本层的 adapter_* → USART1(115200 8N1)
 *
 * 本层**不创建任务**(任务由服务层用 osThreadNew 挂上面两个入口), 也**不解析协议**
 * (解析全在 BSP/Nordic)。它只回答三个"谁持有"的问题: 持有板级驱动实例、绑定具体
 * 外设、把 4 个 RTOS 对象与四个注入接口装好。
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 命名说明(对 cywatch-bsp-driver skill §11.2 的有意偏离, 非笔误): adapter
 *       惯例导出函数是 `<consumer>_bsp_<动作>`。本 adapter 的唯一消费者是 Nordic
 *       链路服务, 而服务名与设备名同源, 写成 `nordicprotocol_bsp_*` 冗长且与设备名
 *       重复, 故用 `nordic_bsp_*`(设备名即消费者前缀)。一个 adapter = 一个设备 ×
 *       一个服务的对应关系不变(grep `nordic_bsp_` 可一次捞全本 adapter 的所有入口,
 *       与 BSP 层自己的 `nordic_*` 区分开)。
 *
 * @note ★为什么本头 include BSP 驱动头(对 skill §11「服务只 include adapter 头,
 *       不 include 驱动头」的放宽)★ 服务注册接收回调要用 nordic_rx_cb_t, 发数据要
 *       传 nordic_tx_done_cb_t, 构造要传 nordic_cfg_t —— 这三个都是 BSP 头里的类型。
 *       那条规矩的目的是"服务不碰总线/寄存器/pf_*" —— 本服务确实一个都不碰, 它只用
 *       这三个**值语义的类型**。BSP 头本身是 HAL-free 的(只依赖 stdint.h), 所以让
 *       本头转发它是零成本的, 比再镜像一份类型(像 UART/adapter 当年那样)更不容易漂移。
 *
 * @note ★本 adapter 只有一个 .c, 所以宿主回归测**整文件替换**它★(用户决定):
 *       于是下面两件事**不在宿主覆盖范围内** ——
 *         ① `0 → osWaitForever` 那条承重换算(写错 = RX 任务在高优先级空转 = 上电假死);
 *         ② 4 个 RTOS 对象的创建(内核堆不足是静默的)。
 *       这条已记进 demo/proto_regress/README.md §4「宿主测证明不了什么」。改本 .c
 *       时对这两处要格外当心: 宿主用例全 PASS **不能**证明它们没被改坏。
 ******************************************************************************/
#ifndef __CYWATCH_ADAPTER_NORDIC_H__
#define __CYWATCH_ADAPTER_NORDIC_H__

/***********************************Includes***********************************/
#include <stdint.h>
#include "cywatch_bsp_nordic_driver.h"
/***********************************Includes***********************************/

/**********************************Declaring***********************************/
/**
 * @name  nordic_bsp_inst
 * @brief 一条龙: 建 4 个 RTOS 对象 → 板级绑定(DMA/NVIC/USART1 驱动) → 装四个
 *        注入接口 → nordic_inst()
 * @param p_cfg[in] 协议配置(超时/重传上限)
 * @return  0 成功
 *         -1 p_cfg 为空
 *         -2 4 个 RTOS 对象之一创建失败(内核堆不足; 失败是静默的, 只能在这里查)
 *         -3 DMA 初始化失败(HAL_DMA_Init 非 HAL_OK)
 *         -4 NVIC 中断优先级不满足 FreeRTOS 约束(见 .c 里的说明)
 *         -5 UART 驱动起不来: uart_driver_inst 的参数校验失败(见其 @return,
 *            该码原样透传) 或之后的 uart pf_init(HAL_UART_Init)失败 —— 两者都是
 *            "UART 驱动没起来", 不拆成两个码
 *         -6 nordic_inst 失败(见其 @return: -1 实例 / -2 cfg / -3 uart /
 *            -4 semaphore / -5 mutex / -6 timebase)
 * @note   可在 osKernelStart() 之前调用(只配置寄存器/建 RTOS 对象/复位状态, 不含
 *         任何阻塞动作; HAL_DMA_Init 用 HAL tick, 而 HAL_Init() 早于本调用)。
 * @note   ★幂等★: 重复调用直接返回 0。
 * @note   ★本函数**不创建任务**★: 返回后调用方须自己用 osThreadNew 挂
 *         nordic_bsp_rx_entry / nordic_bsp_tx_entry 两个入口。
 * @note   -3/-4/-5 为什么不合并成一个码: -4 是"上电即死"那条路径(NVIC 优先级配错,
 *         configASSERT 关着时是静默内核破坏), 压成一个码等于作废上板排障经验。
 */
int8_t nordic_bsp_inst(nordic_cfg_t *p_cfg);

/**
 * @name  nordic_bsp_deinst
 * @brief 析构: 先叫醒可能还阻塞着的两个任务, 再停中断 + 关 NVIC + 析构 UART 驱动
 *        (会把 PA9/PA10 反初始化成悬空输入)
 * @return 0 成功 / -1 未实例化
 * @note   正常产品固件不调它; 留给"切换协议口/复现问题"用。★不是原子的★: 唤醒的
 *         任务要等调度才真正退出, 与本函数的拆除动作有窗口重叠, 故只适合调试路径。
 */
int8_t nordic_bsp_deinst(void);

/**
 * @name  nordic_bsp_register_feature
 * @brief 把某个特征号的接收回调装进协议核的 256 项回调表(下标即 Feature id)
 * @param feature[in] 特征号(应用可用 0x02-0xFF; 0x00/0x01 是协议自用)
 * @param pf_cb[in]   回调, NULL 表示不关心该特征
 * @return 0 成功 / -1 未实例化
 * @note   ★必须在挂任务之前调完★: 表只被 RX 任务读取, 协议核不做并发保护。
 */
int8_t nordic_bsp_register_feature(uint8_t feature, nordic_rx_cb_t pf_cb);

/**
 * @name  nordic_bsp_rx_start
 * @brief 武装 RX: CIRCULAR DMA + IDLE 中断(此后接收中断会释放 RX 信号量)
 * @return 0 成功 / -1 未实例化 / 其它 = UART 驱动 pf_rx_start 的返回码
 * @note   必须在 nordic_bsp_inst() 之后、且必须在**调度器起来之后**调用(本工程由
 *         nordic_bsp_rx_entry 作为 RX 任务的第一句话调用): 调度器未起时来的字节会
 *         进解析器, 而上电瞬间 PA10 悬空易被噪声触发起始位。
 */
int8_t nordic_bsp_rx_start(void);

/**
 * @name  nordic_bsp_rx_entry
 * @brief RX 任务入口(交给服务的 osThreadNew): 武装 RX → 进 BSP 的阻塞任务体 →
 *        返回后 osThreadExit()
 * @param p_arg[in] 原样透传给任务体(BSP 不用)
 * @return 无
 * @note   ★osThreadExit() 不能漏★: CMSIS-RTOS v2 下从线程函数 return 是未定义行为
 *         (底层是 FreeRTOS 的 vTaskDelete(NULL))。宿主测**抓不到**这一条(pthread
 *         里 return 合法), 只能靠审查与上板。
 * @note   本函数写完后**永不返回**(任务体只在 deinst 之后才 return)。
 */
void nordic_bsp_rx_entry(void *p_arg);

/**
 * @name  nordic_bsp_tx_entry
 * @brief TX 任务入口(交给服务的 osThreadNew): 进 BSP 的阻塞任务体 → 返回后
 *        osThreadExit()
 * @param p_arg[in] 原样透传给任务体(BSP 不用)
 * @return 无
 */
void nordic_bsp_tx_entry(void *p_arg);

/**
 * @name  nordic_bsp_send
 * @brief 应用侧的发送口(加"未初始化"守卫后转发协议核)
 * @param feature[in] 特征号(0x02-0x80 单帧 / 0x81-0xFF 消息, 由协议核按区间判)
 * @param pdata[in]   数据
 * @param len[in]     长度(≤ NORDIC_TX_MSG_MAX = 4096)
 * @param pf_done[in] 发送完成回调(可为 NULL), 在 **TX 任务上下文**触发
 * @return 同协议核 pf_send: 0 成功 / -1 未实例化或未初始化 / -2 参数错 /
 *         -3 超长 / -4 环空间不够
 * @note   ★非阻塞★: 环里放不下立即返回, 绝不挂起调用方。入环即拷贝, 返回后
 *         pdata 可立刻复用。
 */
int8_t nordic_bsp_send(uint8_t feature, const uint8_t *pdata, uint16_t len,
					   nordic_tx_done_cb_t pf_done);

/**********************************Declaring***********************************/

#endif // __CYWATCH_ADAPTER_NORDIC_H__
