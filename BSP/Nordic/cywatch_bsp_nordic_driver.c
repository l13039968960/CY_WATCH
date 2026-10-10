/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_bsp_nordic_driver.c
 *
 * @par dependencies
 * - cywatch_bsp_nordic_driver.h
 * - string.h
 *
 * @author	zw1194
 *
 * @brief Provide the HAL APIs of NORDIC and corresponding opetions.
 *
 * 线上帧格式见 Script/Doc/协议.txt, 软件架构见 Script/Doc/协议解析架构设计.md.
 * 全部状态都在实例结构体里 —— 本文件**没有** static 变量, 照着其他 BSP 驱动的
 * 写法: 每个函数第一参数都是实例, 状态都从实例上取.
 *
 * 两条铁律(违反必出帧错乱):
 *   一, 发送状态只由 TX 任务改写. RX 侧探到 ACK 时只写"事件标志 + 值"再 Release 掉
 *       tx 信号量(pf_tx_release), 由 TX 任务醒来消费 —— 所以本文件里
 *       rx_dispatch_frame() 及其下游一行都不碰 tx_state / tx_cur_*.
 *   二, 总线写口有两个写者(RX 侧直发 ACK / TX 侧发数据与控制帧), 每次"把一整帧
 *       交给总线"都要走 proto_bus_write() 并持总线互斥量(pf_bus_acquire/release).
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 *****************************************************************************/

/***********************************Includes***********************************/
#include "cywatch_bsp_nordic_driver.h"
#include <string.h>
/***********************************Includes***********************************/

/***********************************Defines************************************/
/* 线上帧: AA 55 | Feature id | Seq | Length | payload(0-128) | CRC16(2) */
#define PROTO_SYNC_BYTE_0	0xAAu
#define PROTO_SYNC_BYTE_1	0x55u

/* ACK 帧长度 = 帧头2 + Feature1 + Seq1 + Len1 + CRC2 */
#define PROTO_ACK_FRAME_LEN	7u

/* 取互斥量的等待上限(ms). 锁只覆盖"投递给驱动"一步, 正常路径抢不到这么久;
 * 给个上限是为了出错时不把任务永久挂死. */
#define PROTO_MTX_WAIT_MS	100u

/* CRC16-Modbus 初值. 抽出来是因为整条消息的 CRC 要**跨环尾**分两段算 */
#define PROTO_CRC16_INIT	0xFFFFu
/***********************************Defines************************************/

/**********************************Declaring***********************************/
/* 帧解析状态机(8 态) */
typedef enum
{
	RX_FSM_SEEK_AA = 0u,
	RX_FSM_SEEK_55,
	RX_FSM_FEATURE,
	RX_FSM_SEQ,
	RX_FSM_LEN,
	RX_FSM_PAYLOAD,
	RX_FSM_CRC_LO,
	RX_FSM_CRC_HI,
} proto_rx_fsm_e;

/* 发送侧的停等进度(tx_state) */
typedef enum
{
	TX_STATE_IDLE = 0u,   /* 空闲: 没有在等任何东西 */
	TX_STATE_WAIT_ACK,    /* 已投出一帧, 等它的帧级 ACK(等不到就重传) */
} proto_tx_state_e;
/**********************************Declaring***********************************/

/**********************************Variables***********************************/
/* 基础 */
static uint16_t proto_crc16(const uint8_t *pdata, uint32_t len);
static uint8_t  proto_build_frame(uint8_t feature, uint8_t seq,
								  const uint8_t *p_payload, uint8_t len,
								  uint8_t *p_out);
static void     proto_bus_write(bsp_nordic_driver_t *p_inst,
								const uint8_t *p_frame, uint8_t len);
static void     proto_reset_state(bsp_nordic_driver_t *p_inst);

/* 发送环 */
static int8_t   tx_ring_push(bsp_nordic_driver_t *p_inst, uint8_t feature,
							 const uint8_t *pdata, uint16_t len,
							 nordic_tx_done_cb_t pf_done);
static void     tx_ring_read(bsp_nordic_driver_t *p_inst, uint16_t start,
							 uint16_t offset, uint8_t *p_dst, uint16_t len);

/* 发送侧: 分包与停等重传 */
static uint16_t tx_msg_crc(bsp_nordic_driver_t *p_inst,
						   const nordic_tx_slot_t *p_slot);
static int8_t   tx_send_frame(bsp_nordic_driver_t *p_inst, uint8_t feature,
							  const uint8_t *p_payload, uint8_t len);
static int8_t   tx_send_all_frames(bsp_nordic_driver_t *p_inst,
								   const nordic_tx_slot_t *p_slot, uint16_t pkg_num);
static void     tx_do_single(bsp_nordic_driver_t *p_inst,
							 const nordic_tx_slot_t *p_slot);
static void     tx_do_message(bsp_nordic_driver_t *p_inst,
							  const nordic_tx_slot_t *p_slot);
static void     tx_finish_record(bsp_nordic_driver_t *p_inst, int8_t result);
static void     tx_process_record(bsp_nordic_driver_t *p_inst);

/* 接收 */
static void     rx_reset(bsp_nordic_driver_t *p_inst);
static void     rx_send_ack(bsp_nordic_driver_t *p_inst, uint8_t seq);
static void     ACKProcess(bsp_nordic_driver_t *p_inst, uint8_t seq);
static void     CMDProcess(bsp_nordic_driver_t *p_inst, uint8_t seq,
						   const uint8_t *p_payload, uint8_t len);
static void     DataProcess(bsp_nordic_driver_t *p_inst, uint8_t seq,
							const uint8_t *p_payload, uint8_t len);
static void     MessageProcess(bsp_nordic_driver_t *p_inst, uint8_t seq,
							   const uint8_t *p_payload, uint8_t len);
static void     rx_dispatch_frame(bsp_nordic_driver_t *p_inst);
static void     rx_byte(bsp_nordic_driver_t *p_inst, uint8_t byte, uint8_t depth);

/* 驱动函数指针的实现 */
static int8_t   nordic_deinst(struct bsp_nordic_driver *p_nordic_instance);
static void     nordic_rx_task(struct bsp_nordic_driver *p_nordic_instance);
static void     nordic_tx_task(struct bsp_nordic_driver *p_nordic_instance);
static int8_t   nordic_register_feature(struct bsp_nordic_driver *p_nordic_instance,
										uint8_t feature, nordic_rx_cb_t pf_cb);
static int8_t   nordic_send(struct bsp_nordic_driver *p_nordic_instance,
							uint8_t feature, const uint8_t *pdata, uint16_t len,
							nordic_tx_done_cb_t pf_done);
/**********************************Variables***********************************/

/*****************************基础: CRC16 与组帧*******************************/

/**
 * @brief  CRC16-Modbus(多项式 0xA001 反射)的一段更新
 * @param  crc    进入本段时的校验值(首段传 PROTO_CRC16_INIT)
 * @param  pdata  待校验首地址
 * @param  len    字节数
 * @return 更新后的 16 位校验值
 * @note   分段是为了**跨环尾**的整条消息 CRC: 发送环里一条记录可以横跨环尾, 没法
 *         一次给一个连续缓冲.
 */
static uint16_t proto_crc16_seg(uint16_t crc, const uint8_t *pdata, uint32_t len)
{
	uint32_t i;
	uint8_t  j;

	for (i = 0u; i < len; i++)
	{
		crc ^= (uint16_t)pdata[i];
		for (j = 0u; j < 8u; j++)
		{
			if (0u != (crc & 0x0001u))
			{
				crc = (uint16_t)((crc >> 1) ^ 0xA001u);
			}
			else
			{
				crc = (uint16_t)(crc >> 1);
			}
		}
	}
	return crc;
}

/**
 * @brief  CRC16-Modbus(多项式 0xA001 反射, 初值 0xFFFF), 覆盖 Feature..payload
 * @param  pdata  待校验首地址
 * @param  len    字节数
 * @return 16 位校验值
 */
static uint16_t proto_crc16(const uint8_t *pdata, uint32_t len)
{
	return proto_crc16_seg(PROTO_CRC16_INIT, pdata, len);
}

/**
 * @brief  组一帧: AA 55 | Feature id | seq | len | payload | CRC16(低字节先发)
 * @param  feature    Feature id
 * @param  seq        帧序号
 * @param  p_payload  payload(可为 NULL, 此时 len 必须为 0)
 * @param  len        payload 长度(0-128)
 * @param  p_out      [out] 整帧输出缓冲(至少 7+len 字节)
 * @return 整帧字节数(7 + len)
 * @note   CRC 覆盖 Feature..payload, 不含帧头两字节
 */
static uint8_t proto_build_frame(uint8_t feature, uint8_t seq,
								 const uint8_t *p_payload, uint8_t len,
								 uint8_t *p_out)
{
	uint16_t crc;

	p_out[0] = PROTO_SYNC_BYTE_0;
	p_out[1] = PROTO_SYNC_BYTE_1;
	p_out[2] = feature;
	p_out[3] = seq;
	p_out[4] = len;
	if ((NULL != p_payload) && (0u != len))
	{
		(void)memcpy(&p_out[5], p_payload, (size_t)len);
	}
	crc = proto_crc16(&p_out[2], (uint32_t)len + 3u);
	p_out[5u + len] = (uint8_t)(crc & 0xFFu);
	p_out[6u + len] = (uint8_t)(crc >> 8);
	return (uint8_t)(len + 7u);
}

/*****************************注入依赖的小封装**********************************/

/**
 * @brief  把一整帧交给总线驱动(铁律二)
 * @param  p_inst   驱动实例
 * @param  p_frame  整帧首地址
 * @param  len      整帧字节数
 * @note   这是全文件**唯一**的总线写出口: 两个写者都从这里走, 都必须持总线互斥量,
 *         否则两帧字节会交错成对端无法解析的缝合怪帧. 锁的粒度只覆盖"投递给驱动"
 *         (环内拷贝 + 启动 DMA), 不覆盖线上传输时间.
 */
static void proto_bus_write(bsp_nordic_driver_t *p_inst,
							const uint8_t *p_frame, uint8_t len)
{
	nordic_mutex_interface_t *p_mtx = p_inst->p_mutex_interface;

	(void)p_mtx->pf_bus_acquire(PROTO_MTX_WAIT_MS);
	(void)p_inst->p_uart_interface->pf_tx_send((uint8_t *)p_frame, (uint32_t)len);
	(void)p_mtx->pf_bus_release();
}

/******************************发送环: 入环************************************/

/**
 * @brief  把一条记录写进发送环, 并唤醒 TX 任务
 * @param  p_inst   驱动实例
 * @param  feature  特征号
 * @param  pdata    数据
 * @param  len      数据长度
 * @param  pf_done  完成回调, 可为 NULL(控制帧就是这样)
 * @return 0 成功 / -4 槽数组或字节环放不下整条
 * @note   ★非阻塞★: 放不下立即返回, 绝不挂起调用方(它是应用任务或 RX 任务).
 *         记录头进槽数组、数据字节进环, 靠 slot.start 偏移 + 取模访问, 所以记录
 *         可以跨环尾, 不需要预留连续空间. 入环即拷贝, 返回后 pdata 可复用.
 * @note   槽数组与字节环是两套配额, 都必须够才收.
 */
static int8_t tx_ring_push(bsp_nordic_driver_t *p_inst, uint8_t feature,
						   const uint8_t *pdata, uint16_t len,
						   nordic_tx_done_cb_t pf_done)
{
	nordic_mutex_interface_t *p_mtx = p_inst->p_mutex_interface;
	int8_t   ret = -4;
	uint16_t head;
	uint16_t tail;

	(void)p_mtx->pf_ring_acquire(PROTO_MTX_WAIT_MS);

	if ((p_inst->tx_slot_cnt < (uint8_t)NORDIC_TX_SLOT_NUM) &&
		((uint32_t)p_inst->tx_used + (uint32_t)len <= (uint32_t)NORDIC_TX_RING_SIZE))
	{
		nordic_tx_slot_t *p_slot =
			&p_inst->tx_slot[(uint8_t)((p_inst->tx_slot_head + p_inst->tx_slot_cnt) %
									   (uint8_t)NORDIC_TX_SLOT_NUM)];

		p_slot->start   = p_inst->tx_wr;
		p_slot->len     = len;
		p_slot->feature = feature;
		p_slot->pf_done = pf_done;

		/* 环里写数据: 跨环尾就分两段拷 */
		head = (uint16_t)((uint16_t)NORDIC_TX_RING_SIZE - p_inst->tx_wr);
		if (head > len)
		{
			head = len;
		}
		(void)memcpy(&p_inst->tx_ring[p_inst->tx_wr], pdata, (size_t)head);
		if (len > head)
		{
			tail = (uint16_t)(len - head);
			(void)memcpy(&p_inst->tx_ring[0], &pdata[head], (size_t)tail);
		}
		p_inst->tx_wr = (uint16_t)((p_inst->tx_wr + len) % (uint16_t)NORDIC_TX_RING_SIZE);
		p_inst->tx_used = (uint16_t)(p_inst->tx_used + len);
		p_inst->tx_slot_cnt++;
		ret = 0;
	}

	(void)p_mtx->pf_ring_release();

	if (0 == ret)
	{
		/* 先写好记录再唤醒, 免得 TX 任务醒来看到空槽 */
		(void)p_inst->p_semaphore_interface->pf_tx_release();
	}
	return ret;
}

/**
 * @brief  从发送环里拷出某条记录的一段
 * @param  p_inst  驱动实例
 * @param  start   该记录在环里的起始偏移(记录头的 start)
 * @param  offset  段内偏移(相对记录起点)
 * @param  p_dst   [out] 目标缓冲
 * @param  len     字节数
 * @note   记录可以跨环尾, 所以分两段拷. 不必加锁: TX 是环的唯一读者, 且入环侧受
 *         tx_used 配额约束, 不会写到"还没出队"的区域.
 */
static void tx_ring_read(bsp_nordic_driver_t *p_inst, uint16_t start,
						 uint16_t offset, uint8_t *p_dst, uint16_t len)
{
	uint16_t pos  = (uint16_t)((start + offset) % (uint16_t)NORDIC_TX_RING_SIZE);
	uint16_t head = (uint16_t)((uint16_t)NORDIC_TX_RING_SIZE - pos);

	if (head > len)
	{
		head = len;
	}
	(void)memcpy(p_dst, &p_inst->tx_ring[pos], (size_t)head);
	if (len > head)
	{
		(void)memcpy(&p_dst[head], &p_inst->tx_ring[0], (size_t)(len - head));
	}
}

/***********************************接收***************************************/

/**
 * @brief  复位帧解析状态机(不动 rx_frame 里的残留字节, 它们由 FSM 状态门控)
 */
static void rx_reset(bsp_nordic_driver_t *p_inst)
{
	p_inst->rx_state = (uint8_t)RX_FSM_SEEK_AA;
	p_inst->rx_len   = 0u;
	p_inst->rx_idx   = 0u;
}

/**
 * @brief  直发一个 ACK
 * @param  p_inst 驱动实例
 * @param  seq    要确认的帧序号
 * @note   ACK 是欠对端的债, 必须在收到帧的当刻就还: 组好帧直接交总线, 不进发送环,
 *         因此不占 seq、不占在途, 也就不可能被自己的停等拖住(排队会互锁).
 */
static void rx_send_ack(bsp_nordic_driver_t *p_inst, uint8_t seq)
{
	uint8_t frame[PROTO_ACK_FRAME_LEN];

	(void)proto_build_frame(NORDIC_FEATURE_ACK, seq, NULL, 0u, frame);
	proto_bus_write(p_inst, frame, PROTO_ACK_FRAME_LEN);
}

/**
 * @brief  收到 ACK 帧: 只记事件 + 唤醒 TX 任务(铁律一)
 * @note   ACK 帧自身永不被 ACK, 也不走判重闸门(重复的 ACK 由 TX 侧比 seq 丢掉).
 */
static void ACKProcess(bsp_nordic_driver_t *p_inst, uint8_t seq)
{
	p_inst->tx_evt_ack_seq   = seq;
	p_inst->tx_evt_ack_valid = 1u;      /* ★值先写, 标志后写★ */
	(void)p_inst->p_semaphore_interface->pf_tx_release();
}

/**
 * @brief  链路层控制帧(Feature id 0x01): 当前无定义控制码, 只回帧级 ACK
 * @param  p_inst     驱动实例
 * @param  seq        帧序号
 * @param  p_payload  payload = 控制码1 + Data(0-127), 现在一律忽略
 * @param  len        payload 长度
 * @note   ★这一路只在本跳(STM32↔nRF)内生效, 不穿过 nRF 转发给对端★ —— 所以它不能
 *         当端到端命令通道用(要那个就落在数据区的 0x02).
 * @note   消息层的 0x10/0x11 已删, 所以这里不再有"交回发送侧"的分发: 唯一欠对端的债
 *         就是这个帧级 ACK.
 */
static void CMDProcess(bsp_nordic_driver_t *p_inst, uint8_t seq,
					   const uint8_t *p_payload, uint8_t len)
{
	(void)p_payload;
	(void)len;
	rx_send_ack(p_inst, seq);           /* 应答先行, 内容一律丢弃 */
}

/**
 * @brief  单帧数据包(Feature id 0x02-0x80): 查回调表交付应用
 * @note   特征号就是帧头那个 Feature id, 由 rx_dispatch_frame 存进 rx_feature.
 *         表项为空 = 没人关心, 收到即丢(但 ACK 已经回了).
 */
static void DataProcess(bsp_nordic_driver_t *p_inst, uint8_t seq,
						const uint8_t *p_payload, uint8_t len)
{
	nordic_rx_cb_t pf_cb = p_inst->rx_cb_tab[p_inst->rx_feature];

	rx_send_ack(p_inst, seq);           /* 应答先行 */

	if (NULL != pf_cb)
	{
		pf_cb(p_inst->rx_feature, (uint8_t *)p_payload, (uint16_t)len);
	}
}

/**
 * @brief  消息数据包(Feature id 0x81-0xFF): 分包重组, 齐了才交应用
 * @param  p_payload  payload = 分包序号1 + Data(0-127)
 * @note   首包(序号0)的 Data = 包数目, 尾包(序号 包数目+1)的 Data = 整条消息 CRC16.
 *         收到序号0 的新首包即重置重组(丢弃未完成部分).
 * @note   不校验分片先后: 协议保证链路有序, 且每帧都要停等 ACK 才发下一帧, 所以
 *         分片不会乱序、不会缺; 真有损坏由尾包的整条 CRC 兜住.
 * @note   ★消息层没有跨端握手(0x10/0x11 已删)★: 校验不过就整条丢弃, 不通知对端 ——
 *         对端只知道"帧都发完了"; 内容对不对由应用层的整包校验兜底.
 * @note   顺序纪律: 帧级 ACK 先发, 最后才调应用回调 —— 回调里是应用业务可能很慢,
 *         而应答是欠对端的债, 不能让它等.
 */
static void MessageProcess(bsp_nordic_driver_t *p_inst, uint8_t seq,
						   const uint8_t *p_payload, uint8_t len)
{
	uint8_t  frag;
	uint16_t datalen;

	rx_send_ack(p_inst, seq);           /* 每帧都回帧级 ACK, 尾包也一样 */

	if (0u == len)
	{
		return;                         /* 连分包序号都没有: 已回 ACK, 内容丢弃 */
	}
	frag = p_payload[0];

	if (0u == frag)
	{
		p_inst->rx_rasm_active  = 1u;
		p_inst->rx_rasm_pkg_num = (len >= 2u) ? (uint16_t)p_payload[1] : 0u;
		p_inst->rx_rasm_len     = 0u;
		return;                         /* 首包永远不落地成交付 */
	}

	if (0u == p_inst->rx_rasm_active)
	{
		return;                         /* 没有会话: 非首包一律丢 */
	}

	if ((uint16_t)frag == (uint16_t)(p_inst->rx_rasm_pkg_num + 1u))
	{
		uint16_t msg_len = p_inst->rx_rasm_len;
		uint8_t  ok      = 0u;

		if (len >= 3u)
		{
			uint16_t crc_exp = (uint16_t)((uint16_t)p_payload[1] |
										  ((uint16_t)p_payload[2] << 8));

			ok = (proto_crc16(p_inst->rx_rasm_buf, (uint32_t)msg_len) == crc_exp)
				 ? 1u : 0u;
		}

		p_inst->rx_rasm_active = 0u;
		p_inst->rx_rasm_len    = 0u;

		if (0u != ok)
		{
			nordic_rx_cb_t pf_cb = p_inst->rx_cb_tab[p_inst->rx_feature];

			if (NULL != pf_cb)
			{
				/* ★pdata 指向核内重组缓冲, 回调返回后即可被下一条消息覆盖 */
				pf_cb(p_inst->rx_feature, p_inst->rx_rasm_buf, msg_len);
			}
		}
		return;
	}

	/* 数据分片: 追加到重组缓冲. 放不下就丢弃本片 —— 尾包的整条 CRC 必然对不上,
	 * 整条消息就地丢弃; 不在这里另开一条溢出处理路径. */
	datalen = (uint16_t)(len - 1u);
	if (((uint32_t)p_inst->rx_rasm_len + (uint32_t)datalen) <=
		(uint32_t)NORDIC_RX_MSG_BUF_SIZE)
	{
		(void)memcpy(&p_inst->rx_rasm_buf[p_inst->rx_rasm_len], &p_payload[1],
					 (size_t)datalen);
		p_inst->rx_rasm_len = (uint16_t)(p_inst->rx_rasm_len + datalen);
	}
}

/**
 * @brief  校验通过的整帧分发: 判重闸门 → 按 Feature id 区间选 Process
 * @note   ★顺序: 判重闸门在分发之前, 但 **ACK 帧豁免判重** —— ACK 帧自身永不
 *         被 ACK, 也不参与"最近一个已处理 seq"的记录(判重记的是对端发过来的数据
 *         帧的 seq, 与本端发送用的 seq 是两套, 互不相干).
 * @note   ★本函数(铁律一)一行都不碰发送状态: ACK / 0x10 / 0x11 一律只记事件再
 *         Release 掉 tx 信号量, 由 TX 任务醒来消费.
 */
static void rx_dispatch_frame(bsp_nordic_driver_t *p_inst)
{
	uint8_t        feature = p_inst->rx_feature;
	uint8_t        seq     = p_inst->rx_seq;
	const uint8_t *p_payload = &p_inst->rx_frame[5];
	uint8_t        len     = p_inst->rx_len;

	if (NORDIC_FEATURE_ACK == feature)
	{
		ACKProcess(p_inst, seq);
		return;
	}

	if ((0u != p_inst->rx_last_seq_valid) && (seq == p_inst->rx_last_seq))
	{
		rx_send_ack(p_inst, seq);       /* 重复 seq: 只回 ACK, 不处理 */
		return;
	}
	p_inst->rx_last_seq       = seq;
	p_inst->rx_last_seq_valid = 1u;

	if (NORDIC_FEATURE_CTRL == feature)
	{
		CMDProcess(p_inst, seq, p_payload, len);
	}
	else if (feature <= NORDIC_FEATURE_SINGLE_MAX)
	{
		/* 0x02-0x80: 单帧数据包(0x00/0x01 已在上面拦掉) */
		DataProcess(p_inst, seq, p_payload, len);
	}
	else
	{
		/* 0x81-0xFF: 消息数据包 */
		MessageProcess(p_inst, seq, p_payload, len);
	}
}

/**
 * @brief  逐字节喂帧解析器(8 态流式 FSM)
 * @param  byte   输入字节
 * @param  depth  重扫递归深度(正常解析传 0)
 * @note   Length > 128 非法 → 丢弃重扫, 不回 ACK; CRC 失败 → 丢弃、不回 ACK
 *         (由对端超时重传), 并把"0xAA 之后已收的字节"重新喂回 FSM —— payload 里
 *         正好出现 0xAA 0x55 是可能的, 不重扫会漏掉紧跟其后的真帧.
 */
static void rx_byte(bsp_nordic_driver_t *p_inst, uint8_t byte, uint8_t depth)
{
	uint16_t crc;

	switch (p_inst->rx_state)
	{
	case RX_FSM_SEEK_AA:
		if (PROTO_SYNC_BYTE_0 == byte)
		{
			p_inst->rx_state = (uint8_t)RX_FSM_SEEK_55;
		}
		break;

	case RX_FSM_SEEK_55:
		if (PROTO_SYNC_BYTE_1 == byte)
		{
			p_inst->rx_state = (uint8_t)RX_FSM_FEATURE;
		}
		else
		{
			/* AA AA 55: 第二个 AA 要重新起头 */
			p_inst->rx_state = (PROTO_SYNC_BYTE_0 == byte) ? (uint8_t)RX_FSM_SEEK_55
														   : (uint8_t)RX_FSM_SEEK_AA;
		}
		break;

	case RX_FSM_FEATURE:
		p_inst->rx_frame[2] = byte;
		p_inst->rx_feature  = byte;
		p_inst->rx_state    = (uint8_t)RX_FSM_SEQ;
		break;

	case RX_FSM_SEQ:
		p_inst->rx_frame[3] = byte;
		p_inst->rx_seq      = byte;
		p_inst->rx_state    = (uint8_t)RX_FSM_LEN;
		break;

	case RX_FSM_LEN:
		p_inst->rx_frame[4] = byte;
		p_inst->rx_len      = byte;
		if (byte > (uint8_t)NORDIC_MAX_PAYLOAD)
		{
			rx_reset(p_inst);           /* 非法 Length: 丢弃重扫 */
			break;
		}
		p_inst->rx_idx   = 0u;
		p_inst->rx_state = (0u == byte) ? (uint8_t)RX_FSM_CRC_LO
										: (uint8_t)RX_FSM_PAYLOAD;
		break;

	case RX_FSM_PAYLOAD:
		p_inst->rx_frame[5u + p_inst->rx_idx] = byte;
		p_inst->rx_idx++;
		if (p_inst->rx_idx >= p_inst->rx_len)
		{
			p_inst->rx_state = (uint8_t)RX_FSM_CRC_LO;
		}
		break;

	case RX_FSM_CRC_LO:
		p_inst->rx_frame[5u + p_inst->rx_len] = byte;
		p_inst->rx_state = (uint8_t)RX_FSM_CRC_HI;
		break;

	case RX_FSM_CRC_HI:
		p_inst->rx_frame[6u + p_inst->rx_len] = byte;
		crc = proto_crc16(&p_inst->rx_frame[2], (uint32_t)p_inst->rx_len + 3u);
		if (((uint8_t)(crc & 0xFFu) == p_inst->rx_frame[5u + p_inst->rx_len]) &&
			((uint8_t)(crc >> 8) == p_inst->rx_frame[6u + p_inst->rx_len]))
		{
			rx_dispatch_frame(p_inst);
			rx_reset(p_inst);
		}
		else
		{
			uint8_t rescan[NORDIC_MAX_FRAME];
			uint8_t rescan_len = (uint8_t)(p_inst->rx_len + 6u);
			uint8_t i;

			/* 先拷贝再复位: 重喂会改写 rx_frame */
			(void)memcpy(rescan, &p_inst->rx_frame[1], (size_t)rescan_len);
			rx_reset(p_inst);
			if (depth >= (uint8_t)NORDIC_RESCAN_MAX_DEPTH)
			{
				break;                  /* 病态输入保护: 不再递归, 护任务栈 */
			}
			for (i = 0u; i < rescan_len; i++)
			{
				rx_byte(p_inst, rescan[i], (uint8_t)(depth + 1u));
			}
		}
		break;

	default:
		rx_reset(p_inst);
		break;
	}
}

/***********************************发送***************************************/

/**
 * @brief  算一条消息**整条数据**的 CRC16(尾包用)
 * @param  p_inst 驱动实例
 * @param  p_slot 队首记录
 * @return 整条数据的 CRC16
 * @note   数据在环里, 可以跨环尾, 所以要分两段喂(见 proto_crc16_seg).
 */
static uint16_t tx_msg_crc(bsp_nordic_driver_t *p_inst,
						   const nordic_tx_slot_t *p_slot)
{
	uint16_t pos  = p_slot->start;
	uint16_t head = (uint16_t)((uint16_t)NORDIC_TX_RING_SIZE - pos);
	uint16_t rest = p_slot->len;
	uint16_t crc;

	if (head > rest)
	{
		head = rest;
	}
	crc  = proto_crc16_seg(PROTO_CRC16_INIT, &p_inst->tx_ring[pos], (uint32_t)head);
	rest = (uint16_t)(rest - head);
	if (0u != rest)
	{
		crc = proto_crc16_seg(crc, &p_inst->tx_ring[0], (uint32_t)rest);
	}
	return crc;
}

/**
 * @brief  投一帧出去, 停等它的帧级 ACK, 等不到就重传
 * @param  p_inst    驱动实例
 * @param  feature   特征号
 * @param  p_payload payload(0-128 字节)
 * @param  len       payload 长度
 * @return 0 收到匹配的 ACK / -1 重传耗尽
 * @note   每帧占一个 seq(1 字节循环). ACK 帧里的那个字节是**确认号**, 不占本端 seq.
 * @note   重传上限是 frame_max_retry: 首次发送 + 至多这么多次重传.
 * @note   超时用注入的毫秒时基, 不要换成 HAL_GetTick(两者起点差一个常量).
 */
static int8_t tx_send_frame(bsp_nordic_driver_t *p_inst, uint8_t feature,
							const uint8_t *p_payload, uint8_t len)
{
	uint8_t frame[NORDIC_MAX_FRAME];
	uint8_t frame_len;

	p_inst->tx_cur_seq++;
	p_inst->tx_retry         = 0u;
	p_inst->tx_evt_ack_valid = 0u;  /* 丢掉上一帧的 ACK, 免得新帧拿它当确认(seq 还会比) */

	frame_len = proto_build_frame(feature, p_inst->tx_cur_seq, p_payload, len, frame);

	for (;;)
	{
		uint32_t used;

		p_inst->tx_stamp = p_inst->p_timebase_interface->pf_get_time();
		p_inst->tx_state = (uint8_t)TX_STATE_WAIT_ACK;
		proto_bus_write(p_inst, frame, frame_len);

		/* ---- 停等本帧的 ACK ---- */
		for (;;)
		{
			if (0u != p_inst->tx_evt_ack_valid)
			{
				uint8_t seq = p_inst->tx_evt_ack_seq;

				p_inst->tx_evt_ack_valid = 0u;   /* 先读值再清标志 */
				if (seq == p_inst->tx_cur_seq)
				{
					p_inst->tx_state = (uint8_t)TX_STATE_IDLE;
					return 0;
				}
				continue;           /* 陈旧 ACK(重传迟到的): 继续等本帧的 */
			}

			used = p_inst->p_timebase_interface->pf_get_time() - p_inst->tx_stamp;
			if (used >= p_inst->cfg.frame_timeout_ms)
			{
				break;              /* 本帧超时 → 重传 */
			}
			/* used < 上限, 所以剩下的毫秒数 >= 1: 绝不会传成 0(0 是"永久等") */
			(void)p_inst->p_semaphore_interface->pf_tx_acquire(
				p_inst->cfg.frame_timeout_ms - used);
		}

		if (p_inst->tx_retry >= p_inst->cfg.frame_max_retry)
		{
			p_inst->tx_state = (uint8_t)TX_STATE_IDLE;
			return -1;              /* 重传耗尽 */
		}
		p_inst->tx_retry++;
	}
}

/**
 * @brief  把一条消息的分片逐帧发完: 首包 → 数据片 → 尾包, 每帧停等
 * @param  p_inst   驱动实例
 * @param  p_slot   队首记录
 * @param  pkg_num  数据分片数
 * @return 0 尾包的帧级 ACK 到手 / -5 某帧重传耗尽
 * @note   ★p_slot 在处理期间稳定★: 入环侧只写 (head + cnt) % 16 那一格, 而槽位满
 *         16 就不再收记录, 所以队首那格不会被本次消费之前的任何入环覆写.
 */
static int8_t tx_send_all_frames(bsp_nordic_driver_t *p_inst,
								 const nordic_tx_slot_t *p_slot, uint16_t pkg_num)
{
	uint8_t  payload[NORDIC_MAX_PAYLOAD];
	uint16_t frag;
	uint16_t offset;
	uint16_t rest;
	uint16_t crc;

	p_inst->tx_cur_frame = 0u;

	for (frag = 0u; ; frag++)
	{
		if (0u == frag)
		{
			/* 首包: 序号 0, Data = 包数目 */
			payload[0] = 0u;
			payload[1] = (uint8_t)pkg_num;
			if (0 != tx_send_frame(p_inst, p_slot->feature, payload, 2u))
			{
				return -5;
			}
		}
		else if (frag <= pkg_num)
		{
			/* 数据分片: 序号 frag, Data = 第 frag 片(每片至多 127B) */
			offset = (uint16_t)((uint16_t)(frag - 1u) * (uint16_t)NORDIC_FRAG_DATA_SIZE);
			rest   = (uint16_t)(p_slot->len - offset);
			if (rest > (uint16_t)NORDIC_FRAG_DATA_SIZE)
			{
				rest = (uint16_t)NORDIC_FRAG_DATA_SIZE;
			}
			payload[0] = (uint8_t)frag;
			tx_ring_read(p_inst, p_slot->start, offset, &payload[1], rest);
			if (0 != tx_send_frame(p_inst, p_slot->feature, payload,
								   (uint8_t)(rest + 1u)))
			{
				return -5;
			}
		}
		else
		{
			/* 尾包: 序号 包数目+1, Data = 整条消息的 CRC16(低字节先发).
			 * ★frag 不会越过 0xFF★: 单条消息上限 4096B, 分片数至多 33. */
			crc        = tx_msg_crc(p_inst, p_slot);
			payload[0] = (uint8_t)frag;
			payload[1] = (uint8_t)(crc & 0xFFu);
			payload[2] = (uint8_t)(crc >> 8);
			return (0 == tx_send_frame(p_inst, p_slot->feature, payload, 3u)) ? 0 : -5;
		}

		p_inst->tx_cur_frame = (uint16_t)(frag + 1u);
	}
}

/**
 * @brief  队首记录收尾: 锁内归还槽位与环空间, 锁外调完成回调
 * @param  p_inst 驱动实例
 * @param  result 结果码(0 成功 / -5 帧级失败), 原样传给回调
 * @note   ★归还必须在回调之前★: 回调里很可能立刻发下一条, 不先归还就会撞满.
 * @note   ★回调必须在锁外★: 回调里几乎一定再调 pf_send, 持着环互斥量调 = 自锁死.
 */
static void tx_finish_record(bsp_nordic_driver_t *p_inst, int8_t result)
{
	nordic_mutex_interface_t *p_mtx = p_inst->p_mutex_interface;
	nordic_tx_done_cb_t       pf_done;

	(void)p_mtx->pf_ring_acquire(PROTO_MTX_WAIT_MS);

	pf_done = p_inst->tx_slot[p_inst->tx_slot_head].pf_done;
	p_inst->tx_used = (uint16_t)(p_inst->tx_used -
								 p_inst->tx_slot[p_inst->tx_slot_head].len);
	p_inst->tx_slot_head = (uint8_t)((p_inst->tx_slot_head + 1u) %
									 (uint8_t)NORDIC_TX_SLOT_NUM);
	p_inst->tx_slot_cnt--;

	(void)p_mtx->pf_ring_release();

	if (NULL != pf_done)
	{
		pf_done(result);
	}
}

/**
 * @brief  单帧记录: 一帧发完, 帧级 ACK 到手即成功
 * @note   Feature id <= 0x80 的都走这里, 包括协议自用的控制帧(0x01).
 */
static void tx_do_single(bsp_nordic_driver_t *p_inst,
						 const nordic_tx_slot_t *p_slot)
{
	uint8_t payload[NORDIC_SINGLE_DATA_SIZE];
	int8_t  result;

	tx_ring_read(p_inst, p_slot->start, 0u, payload, p_slot->len);
	result = (0 == tx_send_frame(p_inst, p_slot->feature, payload,
								 (uint8_t)p_slot->len)) ? 0 : -5;
	tx_finish_record(p_inst, result);
}

/**
 * @brief  消息记录: 首包/数据分片/尾包逐帧发完, 尾包的帧级 ACK 到手即收工
 * @note   ★不再有消息级确认★(0x10/0x11 已删), 所以这里只剩一种失败:
 *           -5 某帧的帧级重传耗尽 —— 连一帧都送不到, 链路大概断了.
 */
static void tx_do_message(bsp_nordic_driver_t *p_inst,
						  const nordic_tx_slot_t *p_slot)
{
	uint16_t pkg_num;

	pkg_num = (uint16_t)(((uint32_t)p_slot->len + (uint32_t)NORDIC_FRAG_DATA_SIZE - 1u) /
						 (uint32_t)NORDIC_FRAG_DATA_SIZE);

	tx_finish_record(p_inst,
					 (0 == tx_send_all_frames(p_inst, p_slot, pkg_num)) ? 0 : -5);
}

/**
 * @brief  把队首那条记录发完(可能很久), 然后出队并回调
 */
static void tx_process_record(bsp_nordic_driver_t *p_inst)
{
	const nordic_tx_slot_t *p_slot = &p_inst->tx_slot[p_inst->tx_slot_head];

	if (p_slot->feature <= (uint8_t)NORDIC_FEATURE_SINGLE_MAX)
	{
		tx_do_single(p_inst, p_slot);
	}
	else
	{
		tx_do_message(p_inst, p_slot);
	}
}

/******************************任务体与驱动接口*********************************/

/**
 * @brief  RX 任务体: 阻塞于 rx 信号量 → 取字节 → 喂解析器
 * @param  p_nordic_instance  驱动实例
 * @note   ★本层是 BSP, **不创建**这个任务: 它是上层用 osThreadNew 挂载的阻塞式
 *         入口, 优先级须**高于** TX. 挂载时由 adapter 的入口包装在返回后补
 *         osThreadExit()(CMSIS-RTOS v2 下从线程函数 return 是未定义行为).
 * @note   单轮工作**有界**: 最多取 NORDIC_RX_BURST 块就回到信号量. 中途新到的
 *         字节留在驱动的环形缓冲里, 下一轮处理. RX 优先级高于 TX, 若无界解析会把
 *         TX 的控制帧与重传全部拖住.
 */
static void nordic_rx_task(struct bsp_nordic_driver *p_nordic_instance)
{
	uint8_t  buf[NORDIC_RX_CHUNK];
	uint8_t  burst;
	uint32_t n;
	uint32_t i;

	for (;;)
	{
		if (0u == p_nordic_instance->inited)
		{
			return;                     /* 已被 deinst: 自行退出 */
		}
		/* 超时 0 = 永久等 */
		(void)p_nordic_instance->p_semaphore_interface->pf_rx_acquire(0u);

		for (burst = 0u; burst < (uint8_t)NORDIC_RX_BURST; burst++)
		{
			n = p_nordic_instance->p_uart_interface->pf_rx_get_count();
			if (0u == n)
			{
				break;
			}
			if (n > (uint32_t)sizeof(buf))
			{
				n = (uint32_t)sizeof(buf);
			}
			if (0 != p_nordic_instance->p_uart_interface->pf_rx_read(buf, n))
			{
				break;
			}
			for (i = 0u; i < n; i++)
			{
				rx_byte(p_nordic_instance, buf[i], 0u);
			}
		}
	}
}

/**
 * @brief  TX 任务体: 环空就阻塞等 tx 信号量, 有记录就逐条发完(含停等与重传)
 * @param  p_nordic_instance  驱动实例
 * @note   ★本层是 BSP, **不创建**这个任务: 它是上层用 osThreadNew 挂载的阻塞式
 *         入口, 优先级须**低于** RX. 挂载时由 adapter 的入口包装在返回后补
 *         osThreadExit()(CMSIS-RTOS v2 下从线程函数 return 是未定义行为).
 * @note   ★tx 信号量是"有事发生"的信号, 不是"有记录"的账本★: 入环、帧级 ACK
 *         都会 release 一次, 所以它天然会虚唤醒、计数也会虚高. 主循环只看 tx_slot_cnt
 *         来决定发不发, 虚唤醒最多多转一圈, 不会误发也不会漏发.
 * @note   停等期间的等待与超时都在 tx_send_frame 里, 所以 deinst 之后本任务是
 *         "最迟一个帧超时之后"才看到 inited == 0 并退出.
 */
static void nordic_tx_task(struct bsp_nordic_driver *p_nordic_instance)
{
	for (;;)
	{
		if (0u == p_nordic_instance->inited)
		{
			return;                     /* 已被 deinst: 自行退出 */
		}

		if (0u == p_nordic_instance->tx_slot_cnt)
		{
			/* 环空: 阻塞等唤醒(超时 0 = 永久等) */
			(void)p_nordic_instance->p_semaphore_interface->pf_tx_acquire(0u);
			continue;
		}

		tx_process_record(p_nordic_instance);
	}
}

/**
 * @brief  注册特征回调(下标即 Feature id)
 * @note   表只被 RX 任务读取, 本接口不做并发保护 —— 在挂任务之前调完.
 */
static int8_t nordic_register_feature(struct bsp_nordic_driver *p_nordic_instance,
									  uint8_t feature, nordic_rx_cb_t pf_cb)
{
	if (NULL == p_nordic_instance)
	{
		return -1;
	}
	p_nordic_instance->rx_cb_tab[feature] = pf_cb;
	return 0;
}

/**
 * @brief  写一条数据进发送环
 * @return 0 成功 / -1 实例为空 / -2 参数错 / -3 超长 / -4 环空间不够
 * @note   ★非阻塞★, 入环即拷贝. 特征号落在哪个区间由发送侧判, 本函数不管.
 */
static int8_t nordic_send(struct bsp_nordic_driver *p_nordic_instance,
						  uint8_t feature, const uint8_t *pdata, uint16_t len,
						  nordic_tx_done_cb_t pf_done)
{
	if (NULL == p_nordic_instance)
	{
		return -1;
	}
	if ((NULL == pdata) || (0u == len))
	{
		return -2;
	}
	if (len > (uint16_t)NORDIC_TX_MSG_MAX)
	{
		return -3;
	}
	return tx_ring_push(p_nordic_instance, feature, pdata, len, pf_done);
}

/**
 * @brief 析构: 置"未初始化"并叫醒可能还阻塞着的两个任务
 * @return 0 成功 / -1 实例为空
 * @note   本层不提供 thread_delete(**它也不创建任务**), 故这是**尽力而为**的软
 *         停止: 任务下一轮看到 inited == 0 自行 return, 上层入口包装接着调
 *         osThreadExit() 收尾. 注入接口指针**故意不清空** —— 晚醒的任务还要用它.
 */
static int8_t nordic_deinst(struct bsp_nordic_driver *p_nordic_instance)
{
	if (NULL == p_nordic_instance)
	{
		return -1;
	}

	p_nordic_instance->inited = 0u;
	(void)p_nordic_instance->p_semaphore_interface->pf_rx_release();
	(void)p_nordic_instance->p_semaphore_interface->pf_tx_release();
	return 0;
}

/**
 * @brief  复位接收侧与发送环的全部状态(inst 用)
 */
static void proto_reset_state(bsp_nordic_driver_t *p_inst)
{
	p_inst->rx_state          = (uint8_t)RX_FSM_SEEK_AA;
	p_inst->rx_feature        = 0u;
	p_inst->rx_seq            = 0u;
	p_inst->rx_len            = 0u;
	p_inst->rx_idx            = 0u;
	p_inst->rx_last_seq       = 0u;
	p_inst->rx_last_seq_valid = 0u;

	p_inst->rx_rasm_active  = 0u;
	p_inst->rx_rasm_pkg_num = 0u;
	p_inst->rx_rasm_len     = 0u;

	p_inst->tx_wr        = 0u;
	p_inst->tx_used      = 0u;
	p_inst->tx_slot_head = 0u;
	p_inst->tx_slot_cnt  = 0u;

	p_inst->tx_state            = (uint8_t)TX_STATE_IDLE;
	p_inst->tx_cur_seq          = 0u;
	p_inst->tx_retry            = 0u;
	p_inst->tx_cur_frame        = 0u;
	p_inst->tx_stamp            = 0u;

	p_inst->tx_evt_ack_seq      = 0u;
	p_inst->tx_evt_ack_valid    = 0u;

	(void)memset(p_inst->rx_cb_tab, 0, sizeof(p_inst->rx_cb_tab));
}

/**
 * @brief 构造: 校验四个注入接口 → 存接口与配置 → 挂函数指针 → 复位状态
 * @param  p_nordic_instance[out] NORDIC驱动实例
 * @param  p_cfg[in]               协议参数(帧/消息超时与重传上限)
 * @param  p_uart_interface[in]    总线接口
 * @param  p_semaphore_interface[in] 两个信号量
 * @param  p_mutex_interface[in]     两个互斥量
 * @param  p_timebase_interface[in]  毫秒时基
 * @return 0 成功 / 见头文件 @return
 * @note   本函数**不创建任务**、不做任何阻塞动作, 可在 osKernelStart() 之前调用;
 *         构造完由调用方自己 osThreadNew 挂两个任务体(RX 优先级须高于 TX).
 * @note   inited 在**函数末尾**置位 —— 本函数返回时还没有任何任务在跑, 所以这里
 *         不存在"任务已经跑起来但看见它还是 0"的窗口.
 */
int8_t nordic_inst(bsp_nordic_driver_t *p_nordic_instance,
				   nordic_cfg_t *p_cfg,
				   nordic_uart_interface_t *p_uart_interface,
				   nordic_semaphore_interface_t *p_semaphore_interface,
				   nordic_mutex_interface_t *p_mutex_interface,
				   nordic_timebase_interface_t *p_timebase_interface)
{
	if (NULL == p_nordic_instance)
	{
		return -1;
	}

	if (NULL == p_cfg)
	{
		return -2;
	}

	if ((NULL == p_uart_interface) || (NULL == p_uart_interface->pf_tx_send) ||
		(NULL == p_uart_interface->pf_rx_get_count) ||
		(NULL == p_uart_interface->pf_rx_read))
	{
		return -3;
	}

	if ((NULL == p_semaphore_interface) ||
		(NULL == p_semaphore_interface->pf_rx_acquire) ||
		(NULL == p_semaphore_interface->pf_rx_release) ||
		(NULL == p_semaphore_interface->pf_tx_acquire) ||
		(NULL == p_semaphore_interface->pf_tx_release))
	{
		return -4;
	}

	if ((NULL == p_mutex_interface) ||
		(NULL == p_mutex_interface->pf_bus_acquire) ||
		(NULL == p_mutex_interface->pf_bus_release) ||
		(NULL == p_mutex_interface->pf_ring_acquire) ||
		(NULL == p_mutex_interface->pf_ring_release))
	{
		return -5;
	}

	if ((NULL == p_timebase_interface) || (NULL == p_timebase_interface->pf_get_time))
	{
		return -6;
	}

	p_nordic_instance->p_uart_interface      = p_uart_interface;
	p_nordic_instance->p_semaphore_interface = p_semaphore_interface;
	p_nordic_instance->p_mutex_interface     = p_mutex_interface;
	p_nordic_instance->p_timebase_interface  = p_timebase_interface;
	p_nordic_instance->cfg                   = *p_cfg;

	p_nordic_instance->pf_inst          = nordic_inst;
	p_nordic_instance->pf_deinst        = nordic_deinst;
	p_nordic_instance->pf_rx_task       = nordic_rx_task;
	p_nordic_instance->pf_tx_task       = nordic_tx_task;
	p_nordic_instance->pf_register_feature = nordic_register_feature;
	p_nordic_instance->pf_send          = nordic_send;

	proto_reset_state(p_nordic_instance);

	p_nordic_instance->inited = 1u;     /* 见 @note: 置于末尾即无竞态 */

	return 0;
}

/***********************************END****************************************/
