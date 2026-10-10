/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_bsp_nordic_driver.h
 *
 * @par dependencies
 * - stdint.h
 *
 * @author	zw1194
 *
 * @brief Provide the HAL APIs of NORDIC and corresponding opetions.
 *
 * Processing flow:
 *
 * call nordic_inst() to construct, then use function pointers.
 *
 * 线上帧格式见 Script/Doc/协议.txt, 软件架构与全部设计决策见
 * Script/Doc/协议解析架构设计.md —— 本头只列会踩坑的约束.
 *
 * 【本层在架构中的位置】
 *   上层  service/            ─ osThreadNew 把两个任务体挂起来
 *   本层  Nordic/             ─ 协议核: 接收解析 + 封包发送
 *   总线  Nordic/adapter/     ─ RTOS 对象 + USART1/DMA/NVIC + 中断入口
 *   底下  Core/UART/uart_hal  ─ 纯 HAL
 * 本层**只解析、不创建任务**, 依赖全部以**注入接口**(下面四个)进来, 故本头不
 * include 任何 HAL / RTOS 头 —— 既能编进固件, 也能在 PC 上用假总线真编译.
 * 全部状态都在实例结构体里, 本层 .c 不留 static 全局.
 *
 * 【两条铁律】违反必出帧错乱:
 *   一, 发送状态只由 TX 任务改写. RX 侧探到 ACK 时只写"事件标志 + 值"再 Release
 *       信号量, 由 TX 任务醒来消费.
 *   二, 整帧输出口有两个写者(RX 任务直发 ACK / TX 任务发数据与控制帧), 每次
 *       "把一整帧交给总线"都必须持 p_mtx_bus, 否则两帧字节会交错成缝合怪帧.
 *
 * @note ★ RX 优先级必须高于 TX(单轮工作有界靠它兜住), 由挂载方保证.
 *
 * @note ★ ISR 优先级约束(强): p_sem_rx 的释放发生在总线 RX 中断上下文里. 若它落到
 *       osSemaphoreRelease, 则 UART/DMA 中断的抢占优先级数值必须 >=
 *       configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(本工程为 5); 同时 RX DMA 流的
 *       优先级仍须高于 USART 本身(见 Core/UART/uart_hal.c 顶部).
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/
#ifndef __CYWATCH_BSP_NORDIC_DRIVER_H__
#define __CYWATCH_BSP_NORDIC_DRIVER_H__

/***********************************Includes***********************************/
#include <stdint.h>

/***********************************Includes***********************************/

/***********************************Defines************************************/
/* ---------- 线上帧: AA 55 | Feature id | Seq/ACK | Length | payload | CRC16 ---------- */
#define NORDIC_MAX_FRAME		135  /* 帧头2 + Feature1 + Seq1 + Len1 + payload128 + CRC2 */
#define NORDIC_MAX_PAYLOAD		128  /* 单帧 payload 上限 */

/* ---------- Feature id: 取值区间本身就表示类型 ---------- */
#define NORDIC_FEATURE_ACK			0x00U /* ACK,  payload = 0 */
#define NORDIC_FEATURE_CTRL			0x01U /* 链路层控制, payload = 控制码1 + Data(0-127).
										   * ★只在本跳(STM32↔nRF)生效, 当前无定义控制码★ */
#define NORDIC_FEATURE_SINGLE_MIN	0x02U /* 单帧数据包区间起(0x02 已被 OTA 应用层占用) */
#define NORDIC_FEATURE_SINGLE_MAX	0x80U /* 单帧数据包区间止 */
#define NORDIC_FEATURE_MSG_MIN		0x81U /* 消息数据包区间起 */
#define NORDIC_FEATURE_MSG_MAX		0xFFU /* 消息数据包区间止 */

#define NORDIC_SINGLE_DATA_SIZE	128  /* 单帧数据包的数据上限(payload 全是数据) */
#define NORDIC_FRAG_DATA_SIZE	127  /* 分片数据上限(payload = 分包序号1 + Data) */

/* ---------- 缓冲 ---------- */
#define NORDIC_TX_RING_SIZE		5120U /* 发送环: 5KB **总容量**(只放记录的数据字节) */
#define NORDIC_TX_MSG_MAX		4096U /* 单条消息上限 4KB(= 重组缓冲尺寸), 尾包 ACK 到手才归还环空间 */
#define NORDIC_RX_MSG_BUF_SIZE	4096U /* 接收重组缓冲 4KB, 单缓冲 */
#define NORDIC_FEATURE_TAB_SIZE	256U  /* 特征回调表: 下标即 Feature id */
#define NORDIC_TX_SLOT_NUM		16U   /* 发送环最多同时排几条记录(记录头在槽数组里) */

/* ---------- 收字节参数 ---------- */
#define NORDIC_RX_CHUNK			64    /* RX 单块取字节数(与帧边界无关) */
#define NORDIC_RX_BURST			4     /* RX 单轮最多取几块. 单轮工作必须有界:
									   * RX 优先级高于 TX, 无界解析会把 TX 的
									   * 控制帧与重传一起拖住 */
#define NORDIC_RESCAN_MAX_DEPTH	8     /* CRC 失败重扫的递归深度上限, 护任务栈 */

/***********************************Defines************************************/

/**********************************Declaring***********************************/
/* ---------- 应用回调 ---------- */
/* 收到一条数据(单帧整包 / 消息重组完成), 运行在 RX 任务上下文.
 * ★pdata 指向本层内部缓冲, 回调返回后即可被下一条覆盖 —— 要留用必须自己拷走. */
typedef void (*nordic_rx_cb_t)(uint8_t feature, uint8_t *pdata, uint16_t len);

/* 发送完成通知(随记录入环, 不需要额外编号标识是哪条), 运行在 TX 任务上下文:
 *   result =  0 发送成功(单帧=帧ACK到手; 消息=尾包的帧ACK到手)
 *          = -5 帧级失败(某帧 ACK 重传耗尽)
 * ★消息层没有跨端握手(0x10/0x11 已删), 所以"尾包 ACK 到手"就是发送侧能知道的全部;
 *   内容对不对由应用层的整包校验兜底.
 * ★回调触发前环空间已归还, 在回调里立刻发下一条不会撞满. */
typedef void (*nordic_tx_done_cb_t)(int8_t result);

/* ---------- 注入接口一: 字节流总线 ---------- */
typedef struct
{
	/* 把一整帧交给总线并启动发送(全有全无: 装不下整帧就拒收) */
	int8_t (*pf_tx_send)(uint8_t *pdata, uint32_t size);
	/* 总线接收环里已收到的字节数 */
	uint32_t (*pf_rx_get_count)(void);
	/* 从总线接收环取走 size 字节(短读不报错) */
	int8_t (*pf_rx_read)(uint8_t *pdata, uint32_t size);
} nordic_uart_interface_t;

/* ---------- 注入接口二: 信号量 ---------- */
/* ★无句柄★: 对象由调用方创建并常驻, "取/放哪一个"由注入的函数各自闭包进去 —— 与上面
 * 的总线接口、下面的时基接口同构(BSP 这层因此完全不出现 void *). 取/放的实现在调用方,
 * 正常路径不会超时. */
typedef struct
{
	/* 计数: 总线 RX 中断释放 / RX 任务取 / 接收环非空 */
	int8_t (*pf_rx_acquire)(uint32_t timeout_ms); /* 0 = 永久等 */
	int8_t (*pf_rx_release)(void);

	/* 计数: 入环 / ACK / 控制帧释放 / TX 任务取 / 发送侧有事件 */
	int8_t (*pf_tx_acquire)(uint32_t timeout_ms); /* 0 = 永久等 */
	int8_t (*pf_tx_release)(void);
} nordic_semaphore_interface_t;

/* ---------- 注入接口三: 互斥量 ---------- */
typedef struct
{
	/* 整帧输出口: RX 任务发 ACK / TX 任务发数据与控制帧 */
	int8_t (*pf_bus_acquire)(uint32_t timeout_ms);
	int8_t (*pf_bus_release)(void);

	/* 发送环写入口: 应用 / 服务 / RX 任务(控制帧)都往这里写 */
	int8_t (*pf_ring_acquire)(uint32_t timeout_ms);
	int8_t (*pf_ring_release)(void);
} nordic_mutex_interface_t;

/* ---------- 注入接口四: 时基 ---------- */
typedef struct
{
	/* 毫秒时基. ★本链路所有超时都走它, 不要换成 HAL_GetTick(两者起点差一个常量) */
	uint32_t (*pf_get_time)(void);
} nordic_timebase_interface_t;

/* ---------- 协议配置(构造时拷一份进实例, 之后只读) ---------- */
typedef struct
{
	uint32_t frame_timeout_ms; /* 帧级 ACK 超时(协议建议 100) */
	uint8_t frame_max_retry;   /* 帧级重传上限(协议建议 5) */
} nordic_cfg_t;

/* ---------- 发送环里的一条记录: 记录头在槽数组, 数据字节在环里 ---------- */
typedef struct
{
	uint16_t start;				 /* 数据在 tx_ring 里的起始偏移 */
	uint16_t len;				 /* 数据长度 */
	uint8_t  feature;			 /* 特征号, 由它决定走单帧还是消息封装 */
	nordic_tx_done_cb_t pf_done; /* 完成回调, 可为 NULL */
} nordic_tx_slot_t;

/*nordic定义
 * @note 内部状态字段是首版划分, 编码时按实际需要增减, 但**一律留在实例里**,
 *       不要退回 .c 的 static 全局 —— 本层的可重入性靠这个。*/
typedef struct bsp_nordic_driver
{
	/* ---- 注入接口 ---- */
	nordic_uart_interface_t      *p_uart_interface;
	nordic_semaphore_interface_t *p_semaphore_interface;
	nordic_mutex_interface_t     *p_mutex_interface;
	nordic_timebase_interface_t  *p_timebase_interface;

	nordic_cfg_t cfg; /* 构造时拷一份 */
	uint8_t inited;

	/* ---- 接收: 逐字节状态机 ---- */
	uint8_t rx_state;                 /* 8 个状态, 取值见 .c */
	uint8_t rx_feature;
	uint8_t rx_seq;
	uint8_t rx_len;
	uint8_t rx_idx;
	uint8_t rx_frame[NORDIC_MAX_FRAME];
	uint8_t rx_last_seq;              /* 判重: 最近处理过的**对端** seq */
	uint8_t rx_last_seq_valid;        /* 哨兵: 还没处理过任何帧(ACK 帧绕过判重) */

	/* ---- 接收: 分包重组(单缓冲, 收到序号0的新首包即重置) ---- */
	uint8_t  rx_rasm_buf[NORDIC_RX_MSG_BUF_SIZE];
	uint16_t rx_rasm_len;             /* 已重组字节数 */
	uint16_t rx_rasm_pkg_num;         /* 首包报的包数目 */
	uint8_t  rx_rasm_active;

	/* ---- 第二级分发: 下标即 Feature id ---- */
	nordic_rx_cb_t rx_cb_tab[NORDIC_FEATURE_TAB_SIZE];

	/* ---- 发送: 环 ---- */
	uint8_t  tx_ring[NORDIC_TX_RING_SIZE];
	uint16_t tx_wr;                   /* 下一个写偏移(对 RING_SIZE 取模) */
	uint16_t tx_used;                 /* 环里已被记录占用的字节数 */
	nordic_tx_slot_t tx_slot[NORDIC_TX_SLOT_NUM];
	uint8_t  tx_slot_head;            /* 队首(正在发的那条)的下标 */
	uint8_t  tx_slot_cnt;             /* 在环里还没走完的记录条数 */

	/* ---- 发送: 停等状态(只由 TX 任务改写) ---- */
	uint8_t  tx_state;                /* 空闲 / 等帧ACK */
	uint8_t  tx_cur_seq;              /* 本端发送序号, 与接收侧判重的 seq 互不相干 */
	uint8_t  tx_retry;                /* 当前帧已重传次数 */
	uint16_t tx_cur_frame;            /* 当前记录已发到第几包 */
	uint32_t tx_stamp;                /* 当前这一等的起始时刻 */

	/* ---- 发送: RX 侧交给 TX 的事件(RX 写, TX 读; §铁律一 只写值不碰发送状态) ---- */
	/* ★只剩帧级 ACK 这一个事件★: 消息层的 0x11 已删, 不必再给"两个事件各占一个标志"
	 * 那套防顶掉. RX 侧**先写值、后写标志**; TX 侧读到标志后**先读值、再清标志**. */
	uint8_t tx_evt_ack_seq;           /* 帧级 ACK 携带的确认号 */
	uint8_t tx_evt_ack_valid;         /* 1 = 上面有效(TX 消费后清 0) */

	/* ---- 函数指针 ---- */
	int8_t (*pf_inst)(
		struct bsp_nordic_driver *p_nordic_instance,

		nordic_cfg_t *p_cfg,
		nordic_uart_interface_t *p_uart_interface,
		nordic_semaphore_interface_t *p_semaphore_interface,
		nordic_mutex_interface_t *p_mutex_interface,
		nordic_timebase_interface_t *p_timebase_interface);

	int8_t (*pf_deinst)(struct bsp_nordic_driver *p_nordic_instance);

	/* 两个**阻塞式任务体**: 本层不创建任务, 由挂载方包一层 osThreadNew
	 * (RX 优先级须高于 TX). 正常路径下永不返回, 只有 pf_deinst 之后才 return. */
	void (*pf_rx_task)(struct bsp_nordic_driver *p_nordic_instance);
	void (*pf_tx_task)(struct bsp_nordic_driver *p_nordic_instance);

	/* 注册特征回调: 下标即 Feature id, 表项共 256 个. cb 为 NULL 表示不关心.
	 * ★表只被 RX 任务读取, 本接口不做并发保护 —— 在挂任务之前调完. */
	int8_t (*pf_register_feature)(struct bsp_nordic_driver *p_nordic_instance,
								  uint8_t feature, nordic_rx_cb_t pf_cb);

	/* 写一条数据进发送环. 特征号落在哪个区间决定走单帧还是消息封装, 调用方不用管
	 * (应用可用 0x02-0xFF; 0x00/0x01 是协议自用).
	 * ★★特征号 ≤ NORDIC_FEATURE_SINGLE_MAX(0x80) 时, len 必须 ≤
	 *   NORDIC_SINGLE_DATA_SIZE(128)★★ —— 单帧的 payload 就是数据本身, 超了会溢出发送
	 *   侧的栈缓冲. 本层**不做这个校验**(用户决定), 违背 = 未定义行为. 要发长数据就
	 *   换一个落在 0x81-0xFF 的特征号.
	 * ★非阻塞: 放不下整条立即返回错误, 绝不挂起调用方. 入环即拷贝, 返回后 pdata
	 *   可立刻复用. 结果经 pf_done 回报, 可为 NULL. */
	int8_t (*pf_send)(struct bsp_nordic_driver *p_nordic_instance,
					  uint8_t feature, const uint8_t *pdata, uint16_t len,
					  nordic_tx_done_cb_t pf_done);

} bsp_nordic_driver_t;

/*nordic构造函数
 * @param p_nordic_instance[out] NORDIC驱动实例
 * @param p_cfg[in]              协议参数(帧/消息超时与重传上限)
 * @param p_uart_interface[in]   总线接口
 * @param p_semaphore_interface[in] 两个信号量
 * @param p_mutex_interface[in]     两个互斥量
 * @param p_timebase_interface[in]  毫秒时基
 *
 * @return  0 success
 *         -1 p_nordic_instance null
 *         -2 p_cfg null
 *         -3 uart接口为null或其pf_*不完整
 *         -4 semaphore接口为null或其四个函数指针不全
 *         -5 mutex接口为null或其四个函数指针不全
 *         -6 timebase接口为null或pf_get_time为null
 *
 * @note 本函数不创建任务、不做任何阻塞动作, 可在 osKernelStart() 之前调;
 *       构造完由调用方自己 osThreadNew 挂两个任务体.
 * @note 错误码与 APP/Service/nordic 及 adapter 耦合, 改动须同步那两处注释. */
int8_t nordic_inst(bsp_nordic_driver_t *p_nordic_instance,
				   nordic_cfg_t *p_cfg,
				   nordic_uart_interface_t *p_uart_interface,
				   nordic_semaphore_interface_t *p_semaphore_interface,
				   nordic_mutex_interface_t *p_mutex_interface,
				   nordic_timebase_interface_t *p_timebase_interface);

/**********************************Declaring***********************************/

#endif // __CYWATCH_BSP_NORDIC_DRIVER_H__
