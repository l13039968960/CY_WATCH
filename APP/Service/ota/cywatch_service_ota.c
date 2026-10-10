/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_service_ota.c
 *
 * @par dependencies
 * - cywatch_service_ota.h
 * - lv_watch_page_ota.h
 * - cywatch_service_nordicprotocol.h
 * - cywatch_service_fatfs.h
 * - cmsis_os2.h
 *
 * @author  zw1194
 *
 * @brief OTA 下载服务(真实路径): 按需创建的任务, 逐相位推进并把显示值投给 OTA 页.
 *
 * Processing flow:
 *
 * service_ota_start() ──> osThreadNew ──> service_ota_run:
 *
 *   发 0x01[本侧版本] ─► 等 0x02[版本/长度/整包CRC32/块大小]
 *        │                    └─ 1s ×3 没来 ─► 0x08(code=1) ─► 失败
 *        ├─ 版本不新 ─► 已是最新版本(终态, 什么都不发)
 *        └─ 有新版本 ─► 等用户点"下载"(60s 超时 → 回待命)
 *             ├─ 发 0x03 ─► 等 0x04[code] ─► code≠0 ─► 失败(静默)
 *             └─ 块循环: 0x05(N) ─► 0x1702 灌第 N 块 ─► 写 ota/blk_NNNN.bin
 *                    整块超时 15s ×3 ─► 0x08(code=2)
 *             ─► 读回全部块复算整包 CRC32 ─► 不符 0x08(code=3)
 *             ─► 相符 0x07[实算 CRC32] ─► 下载完成(终态)
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note ★★上行只走 feature 0x02, 一次会话只发一条终态命令★★ 上位机那侧是单槽
 *       信箱(读不破坏, 断连才清), 两条背靠背的上行命令前一条必被覆盖. 所以
 *       0x07 / 0x08 各自只发一次, 手表一次都不发 0x09.
 *
 * @note ★seq 跨会话单调递增, 只有重启归零★ 也是单槽信箱的后果: 上一轮的最后一条
 *       命令会一直赖在槽里, 新会话第一条若与它相同会被上位机当"重复读"跳过.
 *
 * @note ★文案只能用子集字库里的字★ lv_font_alibaba_puhuiti_14 只含"运动心率睡眠
 *       音乐消息设置血氧检测开始请贴紧手指量中已完成系统更新检查正在是最版本发现
 *       立即下载升级失败"这些字; 缺字**不报错、不告警、不链接失败**, 只是渲染成
 *       空白, 会一路伪装成"页面正常". ★"校""验"二字不在子集里★, 所以 VERIFYING
 *       那一格复用"正在检查".
 *
 * @note ★任务栈 4096★ 不是随便给的: 逐块日志要走 log_printf(内部 vprintf), 而
 *       service_fatfs_open 那条路上, ff.c 在 FF_USE_LFN == 2 时把**长文件名工作
 *       缓冲(512B)+ FILINFO(约 300B)开在调用者栈上**. 1024 会被这三样挤爆 ——
 *       而 configCHECK_FOR_STACK_OVERFLOW == 0, 溢出是静默踩坏内核堆, 不报错.
 *****************************************************************************/
#include "cywatch_service_ota.h"
#include "lv_watch_page_ota.h"
#include "cywatch_service_nordicprotocol.h"
#include "cywatch_service_fatfs.h"
#include "system/log/cywatch_log.h"
#include "cmsis_os2.h"
#include <string.h>

/***********************************Defines************************************/
/* ---- 应用层命令码(见 Script/Doc/固件下载流程-应用层协议设计.md §3) ---- */
#define OTA_CMD_VER_REQ     (0x01u) /* 表→机 版本查询 */
#define OTA_CMD_VER_ACK     (0x02u) /* 机→表 版本应答 */
#define OTA_CMD_DL_REQ      (0x03u) /* 表→机 下载请求 */
#define OTA_CMD_DL_ACK      (0x04u) /* 机→表 下载应答 */
#define OTA_CMD_BLK_REQ     (0x05u) /* 表→机 请求第 N 块 */
#define OTA_CMD_DONE        (0x07u) /* 表→机 下载完成 */
#define OTA_CMD_ABORT       (0x08u) /* 表→机 放弃 */
#define OTA_CMD_CANCEL      (0x09u) /* 机→表 取消会话(★只有上位机能发, 手表一次都不发★) */

/* 上行命令的特征号(单帧区). 所有"表→机"都走它 */
#define OTA_FEATURE_CTRL    (0x02u)
/* 镜像块进来的特征号(消息区, 协议核会分包重组) */
#define OTA_FEATURE_FW      (0x81u)

/* 本侧固件版本: [major][minor], major 在低字节. ★编译期常量, 不是 AT24C02★ */
#define OTA_CUR_VERSION         OTA_VER_MAKE(1u, 0u)

/* 镜像块大小上限 = 协议核的重组缓冲(一块 4096B = 35 帧) */
#define OTA_BLK_MAX             (4096u)

/* 超时与重试(见协议文档 §8) */
#define OTA_ACK_TIMEOUT_MS      (1000U)  /* 等 0x02 / 0x04 */
#define OTA_ACK_RETRY           (3U)
#define OTA_BLK_TIMEOUT_MS      (15000U) /* 等一整块(35 帧) */
#define OTA_BLK_RETRY           (3U)

/* 发现新版本后等用户点"下载". ★不能用 0(无限等待)★: 用户在"下载"按钮前划走,
   按钮事件永远不会来, 任务就永久挂在 s_alive == 1 上, 用户回来再按会被守卫吞掉 */
#define SERVICE_OTA_CONFIRM_MS  (60000U)

/* 事件队列: RX 回调与 appcore 投, OTA 任务取 */
#define SERVICE_OTA_QUEUE_LEN   (4U)

/* 0x08 的错误码(见协议文档 §3) */
#define OTA_ABORT_TIMEOUT       (1u)      /* 等应答超时重试用尽 */
#define OTA_ABORT_BLOCK         (2u)      /* 某块反复收不对 */
#define OTA_ABORT_CRC           (3u)      /* 整包 CRC32 不符 */
#define OTA_ABORT_SAVE          (4u)      /* 落盘失败 */
#define OTA_ABORT_NOBLK         (0xFFFFu) /* 还没收到任何一块时填这个 */

/* 镜像存放目录. 一块一个文件 ota/blk_NNNN.bin(4 位十进制, 0 基, 与 0x05 同号) */
#define OTA_DIR_PATH            "ota"

/* ★版本号 2B 定死为 [major][minor] 字节序★(线上第一个字节是 major), 所以存成 u16
   时 major 落在低字节. ★必须逐字节比, 别拿拼出来的 u16 直接比大小★ ——
   major=1,minor=2 的 u16 是 513, 比 major=2,minor=1 的 258 还大, 结论正好反过来 */
#define OTA_VER_MAJOR(v)        ((uint8_t)((uint16_t)(v) & 0xFFU))
#define OTA_VER_MINOR(v)        ((uint8_t)(((uint16_t)(v) >> 8) & 0xFFU))
#define OTA_VER_MAKE(maj, mnr)  ((uint16_t)((uint16_t)(maj) | ((uint16_t)(mnr) << 8)))

/* 一轮下载的相位. ★IDLE/UPTODATE/READY/FAILED 是终态★, 由
   service_ota_phase_is_end() 认出来 —— 认出来就投完这组值自退 */
typedef enum
{
    OTA_PHASE_IDLE = 0,    /* 待命: 页面上"检查新版本"那个样子 —— 终态 */
    OTA_PHASE_CHECKING,    /* 发 0x01, 等 0x02, 比版本 */
    OTA_PHASE_FOUND,       /* 发现新版本, 等用户点"下载" + 0x03/0x04 */
    OTA_PHASE_DOWNLOADING, /* 逐块 0x05(N) ↔ 整块, 经 fatfs 写盘 */
    OTA_PHASE_VERIFYING,   /* 读回全部块, 复算整包 CRC32 与 0x02 的值比 */
    OTA_PHASE_READY,       /* 下载完成 —— 终态 */
    OTA_PHASE_UPTODATE,    /* 已是最新版本 —— 终态 */
    OTA_PHASE_FAILED,      /* 任一步失败 —— 终态 */
} ota_phase_e;

/* 队列里传的事件. 块数据与应答体不在队列里(太大), 各自放在下面那两个静态槽中,
   事件只当"槽里有新东西了"的信号. ★严格一问一答, 在途最多一条★ 是这个做法成立
   的前提: OTA 任务取走信号后, 下一次发送之前不会再有人覆盖那个槽 */
#define OTA_EV_CTRL     (1u)   /* s_ctrl 里有一条应用层应答 */
#define OTA_EV_BLOCK    (2u)   /* s_blk_buf 里有一整块镜像 */
#define OTA_EV_CONFIRM  (3u)   /* 用户点了"下载" */
/***********************************Defines************************************/

/**********************************Variables***********************************/
static volatile uint8_t s_alive;    /* start() 置 1, 任务退出前置 0 */
static volatile uint8_t s_phase;    /* 当前相位(appcore 读它判断"是不是在等下载") */
static osMessageQueueId_t s_q;      /* 首次 start() 惰性建, 建一次不删 */

/* 上行命令的 seq: ★跨会话单调递增, 只有重启归零★(理由见文件头 @note) */
static uint8_t s_tx_seq;

/* 最新一条应用层应答(特征 0x02 进来的). RX 回调填, OTA 任务读 */
static volatile uint8_t  s_ctrl[24];
static volatile uint8_t  s_ctrl_len;

/* 一整块镜像. 0x81 回调 memcpy 进来, OTA 任务取走写盘 */
static uint8_t          s_blk_buf[OTA_BLK_MAX];
static volatile uint16_t s_blk_rx_len;

/* 本次会话的镜像参数(0x02 应答里拿到的) */
static uint32_t s_img_len;
static uint32_t s_img_crc32;
static uint16_t s_blk_size;
static uint16_t s_blk_num;

/* 任务属性. 优先级 Low: 下载是长流程, 不该跟 LVGL / 心率抢.
   ★栈 4096 的算法见文件头 @note★ */
static const osThreadAttr_t s_ota_attr =
{
    .name       = "ota",
    .attr_bits  = 0U,
    .cb_mem     = NULL,
    .cb_size    = 0U,
    .stack_mem  = NULL,
    .stack_size = 4096U,
    .priority   = osPriorityLow,
};
/**********************************Variables***********************************/

/**********************************Functions***********************************/
static void ota_ctrl_rx_cb(uint8_t feature, uint8_t *pdata, uint16_t len);
static void ota_fw_rx_cb(uint8_t feature, uint8_t *pdata, uint16_t len);

/******************************************************************************
 * @name    ota_post
 * @brief   往事件队列投一个事件(非阻塞)
 * @param   ev[in] OTA_EV_*
 *
 * @return  无
 *
 * @note    ★timeout 必须传 0★: 调用方可能是 nordic RX 任务或 appcore, 都不许在这儿挂住
 *****************************************************************************/
static void ota_post(uint8_t ev)
{
    if (NULL != s_q)
    {
        (void)osMessageQueuePut(s_q, &ev, 0U, 0U);
    }
}

/******************************************************************************
 * @name    ota_ctrl_rx_cb
 * @brief   特征 0x02 收到一条应答(0x02 / 0x04 / 0x09): 拷进静态槽 + 投信号
 * @param   feature[in] 特征号(恒为 0x02)
 * @param   pdata[in]   应答体, ★回调返回后即失效★
 * @param   len[in]     长度
 *
 * @return  无
 *
 * @note    ★跑在 nordic RX 任务上下文★: 只做有界的一次小拷贝, 不写盘不 printf —— RX
 *          优先级高于 TX, 在这儿干重活会把整条链路的发送方向一起拖住
 *****************************************************************************/
static void ota_ctrl_rx_cb(uint8_t feature, uint8_t *pdata, uint16_t len)
{
    uint16_t i;

    (void)feature;

    if ((0u == len) || (len > (uint16_t)sizeof(s_ctrl)))
    {
        return; /* 空体 / 超长: 丢弃(帧级 ACK 已由协议核回过) */
    }

    for (i = 0u; i < len; i++)
    {
        s_ctrl[i] = pdata[i];
    }
    s_ctrl_len = (uint8_t)len;

    ota_post(OTA_EV_CTRL);
}

/******************************************************************************
 * @name    ota_fw_rx_cb
 * @brief   特征 0x81 收到一整块镜像: 拷进静态缓冲 + 投信号
 * @param   feature[in] 特征号(恒为 0x81)
 * @param   pdata[in]   重组好的整块, ★回调返回后即失效★
 * @param   len[in]     整块字节数
 *
 * @return  无
 *
 * @note    ★跑在 nordic RX 任务上下文★: 一次 memcpy(≤4KB)是有界的, 写盘一律留在
 *          OTA 任务里 —— 在这儿写 flash 会把帧级 ACK 拖到超时
 *****************************************************************************/
static void ota_fw_rx_cb(uint8_t feature, uint8_t *pdata, uint16_t len)
{
    (void)feature;

    if ((0u == len) || (len > (uint16_t)OTA_BLK_MAX))
    {
        return;
    }

    (void)memcpy(s_blk_buf, pdata, (size_t)len);
    s_blk_rx_len = len;

    ota_post(OTA_EV_BLOCK);
}

void service_ota_init(void)
{
    /* ★只注册, 不建任务也不碰设备★. main.c 在 osKernelStart() 之前调它, 那时
       RX 任务还没有真正跑起来 —— 满足 BSP 那句"表要在挂任务之前调完" */
    (void)service_nordicprotocol_register_feature(OTA_FEATURE_CTRL, ota_ctrl_rx_cb);
    (void)service_nordicprotocol_register_feature(OTA_FEATURE_FW, ota_fw_rx_cb);
}

/******************************************************************************
 * @name    ota_send
 * @brief   发一条上行命令: [cmd:1][seq:2][payload...] 走特征 0x02
 * @param   cmd[in]         命令码
 * @param   p_payload[in]   负载(可为 NULL)
 * @param   payload_len[in] 负载长度
 *
 * @return 0 已入环 / 非 0 = service_nordicprotocol_send 的错误码
 *
 * @note    ★seq 在这里自增★, 每次调用都是新的一条, 上位机才不会把它当"重复读"跳过
 * @note    ★非阻塞★: 环里放不下立即返回, 绝不挂起调用方
 *****************************************************************************/
static int8_t ota_send(uint8_t cmd, const uint8_t *p_payload, uint8_t payload_len)
{
    uint8_t buf[8];

    buf[0] = cmd;
    buf[1] = (uint8_t)(s_tx_seq & 0xFFu);
    buf[2] = (uint8_t)((s_tx_seq >> 8) & 0xFFu);
    s_tx_seq++;

    if ((NULL != p_payload) && (payload_len > 0u))
    {
        (void)memcpy(&buf[3], p_payload, (size_t)payload_len);
    }

    return service_nordicprotocol_send(OTA_FEATURE_CTRL, buf,
                                       (uint16_t)(3u + (uint16_t)payload_len), NULL);
}

/******************************************************************************
 * @name    ota_wait_ctrl
 * @brief   等一条应用层应答
 * @param   timeout_ms[in] 超时
 *
 * @return 应答的命令码(≥0) / -1 超时或不是应答
 *****************************************************************************/
static int16_t ota_wait_ctrl(uint32_t timeout_ms)
{
    uint8_t ev;

    if (osOK != osMessageQueueGet(s_q, &ev, NULL, timeout_ms))
    {
        return -1;
    }
    if (OTA_EV_CTRL != ev)
    {
        return -1; /* 不是控制应答(块几乎不可能在这时到) */
    }
    if (0u == s_ctrl_len)
    {
        return -1;
    }

    return (int16_t)s_ctrl[0];
}

/******************************************************************************
 * @name    ota_exchange
 * @brief   发一条上行命令并等它的应答(每次等 1s, 重发 OTA_ACK_RETRY 次)
 * @param   cmd[in]         要发的命令码
 * @param   p_payload[in]   负载
 * @param   payload_len[in] 负载长度
 * @param   expect_cmd[in]  期望收到的命令码
 *
 * @return 收到的命令码(正常 = expect_cmd; 也可能是 0x09 取消) / -1 重试用尽
 *
 * @note    每次重发前清一次队列: 上一轮的迟到应答不应该被当成这一轮的
 *****************************************************************************/
static int16_t ota_exchange(uint8_t cmd, const uint8_t *p_payload, uint8_t payload_len,
                            uint8_t expect_cmd)
{
    uint8_t try_i;
    int16_t got;

    for (try_i = 0u; try_i < OTA_ACK_RETRY; try_i++)
    {
        (void)osMessageQueueReset(s_q);

        if (0 != ota_send(cmd, p_payload, payload_len))
        {
            continue; /* 入环就失败: 当成一次超时, 下一轮重发 */
        }

        got = ota_wait_ctrl(OTA_ACK_TIMEOUT_MS);

        if ((int16_t)OTA_CMD_CANCEL == got)
        {
            return got; /* 上位机取消: 交给调用方收尾 */
        }
        if ((int16_t)expect_cmd == got)
        {
            return got;
        }
    }

    return -1;
}

/******************************************************************************
 * @name    ota_wait_block
 * @brief   等一整块镜像
 * @param   timeout_ms[in] 超时
 *
 * @return 1 = 拿到块 / 0 = 超时 / -1 = 收到上位机取消(0x09)
 *****************************************************************************/
static int8_t ota_wait_block(uint32_t timeout_ms)
{
    uint8_t ev;

    if (osOK != osMessageQueueGet(s_q, &ev, NULL, timeout_ms))
    {
        return 0;
    }
    if (OTA_EV_BLOCK == ev)
    {
        return 1;
    }
    if ((OTA_EV_CTRL == ev) && ((uint8_t)OTA_CMD_CANCEL == s_ctrl[0]))
    {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @name    ota_blk_path
 * @brief   组一块镜像的文件路径 "ota/blk_NNNN.bin"
 * @param   p_out[out] 至少 17B
 * @param   blk[in]    块号(0 基)
 *
 * @return  无
 *
 * @note    ★手写十进制, 不走 snprintf★: OTA 任务栈要留给 ff.c 的长文件名缓冲,
 *          能省的都省掉
 *****************************************************************************/
static void ota_blk_path(char *p_out, uint16_t blk)
{
    p_out[0]  = 'o'; p_out[1]  = 't'; p_out[2]  = 'a'; p_out[3]  = '/';
    p_out[4]  = 'b'; p_out[5]  = 'l'; p_out[6]  = 'k'; p_out[7]  = '_';
    p_out[8]  = (char)('0' + ((blk / 1000u) % 10u));
    p_out[9]  = (char)('0' + ((blk / 100u) % 10u));
    p_out[10] = (char)('0' + ((blk / 10u) % 10u));
    p_out[11] = (char)('0' + (blk % 10u));
    p_out[12] = '.'; p_out[13] = 'b'; p_out[14] = 'i'; p_out[15] = 'n';
    p_out[16] = '\0';
}

/******************************************************************************
 * @name    ota_save_block
 * @brief   把一块写进 ota/blk_NNNN.bin(close 成功才算落盘)
 * @param   blk[in]   块号
 * @param   p_buf[in] 数据
 * @param   len[in]   字节数
 *
 * @return  1 = 成功 / 0 = 失败
 *
 * @note    ★写入顺序不能改★: open(CREATE_ALWAYS) → write → close 返回 0, 三步都过
 *          才算这一块成功. 半途失败留下的残块由下一次 CREATE_ALWAYS 自愈
 * @note    fatfs_file_t 约 584B, ★必须 static★: 叠上 ff.c 开在调用者栈上的长文件名
 *          缓冲, 放栈上会吃掉任务栈的一大块
 *****************************************************************************/
static uint8_t ota_save_block(uint16_t blk, const uint8_t *p_buf, uint16_t len)
{
    static fatfs_file_t file;
    char     path[20];
    uint32_t written = 0u;

    ota_blk_path(path, blk);

    if (0 != service_fatfs_open(&file, path, (BYTE)(FA_WRITE | FA_CREATE_ALWAYS)))
    {
        return 0u;
    }

    /* 写少 = 空间不足(FatFs 唯一一处"成功返回里藏着失败") */
    if ((0 != service_fatfs_write(&file, p_buf, len, &written)) || (written != len))
    {
        (void)service_fatfs_close(&file);
        return 0u;
    }

    /* ★只有 close 返回 0 才代表真的落盘★ */
    if (0 != service_fatfs_close(&file))
    {
        return 0u;
    }

    return 1u;
}

/******************************************************************************
 * @name    ota_load_block
 * @brief   把 ota/blk_NNNN.bin 读回 s_blk_buf(校验阶段用)
 * @param   blk[in]   块号
 * @param   p_len[out] 读到的字节数
 *
 * @return  1 = 成功 / 0 = 失败
 *****************************************************************************/
static uint8_t ota_load_block(uint16_t blk, uint32_t *p_len)
{
    static fatfs_file_t file;
    char     path[20];
    uint32_t size = 0u;
    uint32_t got  = 0u;

    *p_len = 0u;
    ota_blk_path(path, blk);

    if (0 != service_fatfs_open(&file, path, (BYTE)FA_READ))
    {
        return 0u;
    }

    if ((0 != service_fatfs_size(&file, &size)) || (size > (uint32_t)OTA_BLK_MAX))
    {
        (void)service_fatfs_close(&file);
        return 0u;
    }

    if (0 != service_fatfs_read(&file, s_blk_buf, size, &got))
    {
        (void)service_fatfs_close(&file);
        return 0u;
    }

    (void)service_fatfs_close(&file);

    if (got != size)
    {
        return 0u;
    }

    *p_len = size;

    return 1u;
}

/******************************************************************************
 * @name    ota_crc32_update
 * @brief   增量算 CRC-32/IEEE(zlib 兼容, 反射多项式 0xEDB88320)
 * @param   crc[in] 上一段的结果(首段传 0xFFFFFFFF)
 * @param   p[in]   数据
 * @param   len[in] 字节数
 *
 * @return  更新后的 crc(全部算完再 ^ 0xFFFFFFFF)
 *
 * @note    ★不能用 STM32F4 的硬件 CRC★: 它的输入/输出不反射, 出来的值和 zlib 对不上
 * @note    按位算, 不建 1KB 的表 —— 200KB 也就 160 万次内层循环, 几十毫秒
 *****************************************************************************/
static uint32_t ota_crc32_update(uint32_t crc, const uint8_t *p, uint32_t len)
{
    uint32_t i;
    uint8_t  j;

    for (i = 0u; i < len; i++)
    {
        crc ^= (uint32_t)p[i];
        for (j = 0u; j < 8u; j++)
        {
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
        }
    }

    return crc;
}

/******************************************************************************
 * @name    service_ota_phase_is_end
 * @brief   这个相位是不是终态(投完这组值就该自退)
 * @param   phase[in] 相位
 *
 * @return  1 = 终态; 0 = 还要继续走
 *****************************************************************************/
static uint8_t service_ota_phase_is_end(ota_phase_e phase)
{
    switch (phase)
    {
        case OTA_PHASE_IDLE:
        case OTA_PHASE_READY:
        case OTA_PHASE_UPTODATE:
        case OTA_PHASE_FAILED:
            return 1U;

        default:
            return 0U;
    }
}

/******************************************************************************
 * @name    ota_ver_is_newer
 * @brief   远端版本是否比本侧新(★逐字节比 major → minor★)
 * @param   new_ver[in] 远端版本([major][minor] 字节序)
 * @param   cur_ver[in] 本侧版本(同编码)
 *
 * @return  1 = 更新; 0 = 不新(含相同)
 *
 * @note    ★不能写成 `new_ver > cur_ver`★ 见 OTA_VER_* 上面那段注释
 *****************************************************************************/
static uint8_t ota_ver_is_newer(uint16_t new_ver, uint16_t cur_ver)
{
    uint8_t new_maj = OTA_VER_MAJOR(new_ver);
    uint8_t new_mnr = OTA_VER_MINOR(new_ver);
    uint8_t cur_maj = OTA_VER_MAJOR(cur_ver);
    uint8_t cur_mnr = OTA_VER_MINOR(cur_ver);

    if (new_maj != cur_maj)
    {
        return (new_maj > cur_maj) ? 1U : 0U;
    }

    return (new_mnr > cur_mnr) ? 1U : 0U;
}

/******************************************************************************
 * @name    ota_send_abort
 * @brief   发一条 0x08 放弃(code + 块号)
 * @param   code[in] 0x08 的错误码
 * @param   blk[in]  出错时的块号(还没开始收块就填 OTA_ABORT_NOBLK)
 *
 * @return  无
 *****************************************************************************/
static void ota_send_abort(uint8_t code, uint16_t blk)
{
    uint8_t payload[3];

    payload[0] = code;
    payload[1] = (uint8_t)(blk & 0xFFu);
    payload[2] = (uint8_t)((blk >> 8) & 0xFFu);

    (void)ota_send(OTA_CMD_ABORT, payload, 3u);
}

/******************************************************************************
 * @name    service_ota_render
 * @brief   把一个相位的显示值整组投给 OTA 页
 * @param   phase[in]    相位
 * @param   progress[in] 进度条百分比(DOWNLOADING 用, 其余相位相位内固定)
 *
 * @return  无
 *
 * @note    整组投而不是连调几个单字段口: 分几次投会被抢占, 渲染出一个逻辑上不
 *          存在的中间态(文案已经"下载中"、进度条却还隐藏着)
 * @note    ★三个终态 + IDLE 一律 enabled = 1★ 置灰的按钮只有 appcore 能放回来,
 *          而写者已经是本任务、任务又要自退 —— 不恢复就是永久死按钮
 *****************************************************************************/
static void service_ota_render(ota_phase_e phase, uint8_t progress)
{
    watch_page_ota_state_t st;
    const char *p_text;
    uint8_t     bar;
    uint8_t     green;
    uint8_t     enabled;
    uint8_t     i;

    switch (phase)
    {
        case OTA_PHASE_IDLE:
            p_text = "检查新版本";  bar = 0U; green = 0U; enabled = 1U; progress = 0U;
            break;

        case OTA_PHASE_CHECKING:
            p_text = "正在检查";    bar = 0U; green = 0U; enabled = 0U; progress = 0U;
            break;

        case OTA_PHASE_FOUND:
            p_text = "下载";        bar = 0U; green = 1U; enabled = 1U; progress = 0U;
            break;

        case OTA_PHASE_DOWNLOADING:
            /* progress 用调用方传进来的: 已下载块数 / 总块数 */
            p_text = "正在下载";    bar = 1U; green = 1U; enabled = 0U;
            break;

        case OTA_PHASE_VERIFYING:
            /* ★文案是"正在检查"不是"正在校验"★: "校""验"不在子集字库里 */
            p_text = "正在检查";    bar = 1U; green = 1U; enabled = 0U; progress = 100U;
            break;

        case OTA_PHASE_READY:
            p_text = "下载完成";    bar = 1U; green = 1U; enabled = 1U; progress = 100U;
            break;

        case OTA_PHASE_UPTODATE:
            p_text = "已是最新版本"; bar = 0U; green = 0U; enabled = 1U; progress = 0U;
            break;

        case OTA_PHASE_FAILED:
        default:
            p_text = "升级失败";    bar = 0U; green = 0U; enabled = 1U; progress = 0U;
            break;
    }

    st.progress    = progress;
    st.enabled     = enabled;
    st.bar_visible = bar;
    st.btn_green   = green;

    for (i = 0u; (i < (WATCH_OTA_TEXT_SIZE - 1U)) && ('\0' != p_text[i]); i++)
    {
        st.text[i] = p_text[i];
    }
    st.text[i] = '\0';

    (void)watch_page_ota_post_state(&st);
}

/******************************************************************************
 * @name    ota_do_check
 * @brief   CHECKING: 发 0x01, 等 0x02, 解析并比版本
 * @param   p_ver[out] 远端版本(仅在返回 1 时有效)
 *
 * @return  1 = 有新版本 / 0 = 已是最新 / -1 = 失败(已发 0x08) / -2 = 上位机取消
 *****************************************************************************/
static int8_t ota_do_check(uint16_t *p_ver)
{
    uint8_t  payload[2];
    int16_t  got;

    payload[0] = (uint8_t)(OTA_CUR_VERSION & 0xFFu);
    payload[1] = (uint8_t)((OTA_CUR_VERSION >> 8) & 0xFFu);

    got = ota_exchange(OTA_CMD_VER_REQ, payload, 2u, OTA_CMD_VER_ACK);
    if ((int16_t)OTA_CMD_CANCEL == got)
    {
        return -2;
    }
    if (got < 0)
    {
        (void)ota_send_abort(OTA_ABORT_TIMEOUT, OTA_ABORT_NOBLK);
        return -1;
    }

    /* 应答体: [cmd:1][ver:2][img_len:4][img_crc32:4][blk_size:2] = 13B
       ★下行不带 seq 前缀★(协议 §3: 只有上行带 [cmd][seq:2]), 所以 ver 从 [1] 起 */
    if (s_ctrl_len < 13u)
    {
        (void)ota_send_abort(OTA_ABORT_TIMEOUT, OTA_ABORT_NOBLK);
        return -1;
    }

    *p_ver = (uint16_t)((uint16_t)s_ctrl[1] | ((uint16_t)s_ctrl[2] << 8));

    s_img_len   = (uint32_t)s_ctrl[3]
                | ((uint32_t)s_ctrl[4] << 8)
                | ((uint32_t)s_ctrl[5] << 16)
                | ((uint32_t)s_ctrl[6] << 24);

    s_img_crc32 = (uint32_t)s_ctrl[7]
                | ((uint32_t)s_ctrl[8] << 8)
                | ((uint32_t)s_ctrl[9] << 16)
                | ((uint32_t)s_ctrl[10] << 24);

    s_blk_size  = (uint16_t)((uint16_t)s_ctrl[11] | ((uint16_t)s_ctrl[12] << 8));

    if ((0u == s_blk_size) || (s_blk_size > (uint16_t)OTA_BLK_MAX) || (0u == s_img_len))
    {
        (void)ota_send_abort(OTA_ABORT_TIMEOUT, OTA_ABORT_NOBLK);
        return -1;
    }

    s_blk_num = (uint16_t)((s_img_len + (uint32_t)s_blk_size - 1u) / (uint32_t)s_blk_size);

    log_printf("[OTA] ver %u.%u img %u blk %u x%u crc %08X\r\n",
               (unsigned)OTA_VER_MAJOR(*p_ver), (unsigned)OTA_VER_MINOR(*p_ver),
               (unsigned)s_img_len, (unsigned)s_blk_size, (unsigned)s_blk_num,
               (unsigned)s_img_crc32);

    return (0u != ota_ver_is_newer(*p_ver, OTA_CUR_VERSION)) ? 1 : 0;
}

/******************************************************************************
 * @name    ota_wait_confirm
 * @brief   FOUND: 等用户点"下载"(60s)
 *
 * @return  1 = 点了 / 0 = 超时或收到取消
 *
 * @note    用 tick 差算总时长: 队列里可能混进应答类事件, 单个 Get 的超时会被它们
 *          提前打断, 直接拿它当 60s 就不准了
 *****************************************************************************/
static uint8_t ota_wait_confirm(void)
{
    uint32_t t0 = osKernelGetTickCount();
    uint8_t  ev;

    while ((osKernelGetTickCount() - t0) < SERVICE_OTA_CONFIRM_MS)
    {
        if (osOK != osMessageQueueGet(s_q, &ev, NULL, SERVICE_OTA_CONFIRM_MS))
        {
            break;
        }
        if (OTA_EV_CONFIRM == ev)
        {
            return 1u;
        }
        if ((OTA_EV_CTRL == ev) && ((uint8_t)OTA_CMD_CANCEL == s_ctrl[0]))
        {
            break;
        }
    }

    return 0u;
}

/******************************************************************************
 * @name    ota_do_download
 * @brief   发 0x03 等 0x04 拿到许可
 *
 * @return  1 = 允许 / 0 = 被拒绝或取消(静默结束) / -1 = 失败(已发 0x08)
 *****************************************************************************/
static int8_t ota_do_permit(void)
{
    uint8_t payload[2];
    int16_t got;

    payload[0] = (uint8_t)(OTA_CUR_VERSION & 0xFFu);
    payload[1] = (uint8_t)((OTA_CUR_VERSION >> 8) & 0xFFu);

    got = ota_exchange(OTA_CMD_DL_REQ, payload, 2u, OTA_CMD_DL_ACK);
    if ((int16_t)OTA_CMD_CANCEL == got)
    {
        return 0;
    }
    if (got < 0)
    {
        (void)ota_send_abort(OTA_ABORT_TIMEOUT, OTA_ABORT_NOBLK);
        return -1;
    }
    if ((s_ctrl_len < 2u) || (0u != s_ctrl[1]))
    {
        return 0; /* 被拒绝: 静默结束(下载没开始过, 上位机无需上报即可自理) */
    }

    return 1;
}

/******************************************************************************
 * @name    ota_do_verify
 * @brief   读回全部块复算整包 CRC32, 与 0x02 给的值比
 * @param   p_crc[out] 实算的整包 CRC32
 *
 * @return  1 = 相符 / 0 = 不符 / -1 = 读文件失败
 *
 * @note    逐块打一行 "OTA R <blk> <len> <crc32>", 上位机按它逐块比对板上内容
 *****************************************************************************/
static int8_t ota_do_verify(uint32_t *p_crc)
{
    uint32_t crc = 0xFFFFFFFFu;
    uint16_t i;
    uint16_t expect_len;

    for (i = 0u; i < s_blk_num; i++)
    {
        uint32_t len = 0u;
        uint32_t blk_crc;

        expect_len = (uint16_t)((i == (uint16_t)(s_blk_num - 1u))
                                ? (s_img_len - ((uint32_t)i * (uint32_t)s_blk_size))
                                : (uint32_t)s_blk_size);

        if (0u == ota_load_block(i, &len))
        {
            return -1;
        }
        if (len != (uint32_t)expect_len)
        {
            return -1;
        }

        blk_crc = ota_crc32_update(0xFFFFFFFFu, s_blk_buf, len) ^ 0xFFFFFFFFu;

        log_printf("[OTA] R %u %u %08X\r\n", (unsigned)i, (unsigned)len,
                   (unsigned)blk_crc);

        crc = ota_crc32_update(crc, s_blk_buf, len);
    }

    *p_crc = crc ^ 0xFFFFFFFFu;

    return (*p_crc == s_img_crc32) ? 1 : 0;
}

/******************************************************************************
 * @name    service_ota_run
 * @brief   下载任务: 逐相位推进, 终态那组值投完再自退
 * @param   p_arg[in] 未使用
 *
 * @return  无(退出走 osThreadExit; 从线程函数 return 是未定义行为)
 *
 * @note    ★退出顺序写死, 不能颠倒★: 投终态 → s_alive = 0 → osThreadExit.
 *          s_alive 置 0 之后不得再碰任何共享量
 * @note    ★等队列的 timeout 单位是 tick, 这里恰好 1 tick == 1ms★
 *          (configTICK_RATE_HZ == 1000)
 *****************************************************************************/
static void service_ota_run(void *p_arg)
{
    ota_phase_e phase    = OTA_PHASE_CHECKING;
    uint8_t     progress = 0u;
    uint16_t    blk_done = 0u;
    uint16_t    ver      = 0u;
    int8_t      ret;

    (void)p_arg;

    /* 起一轮之前: 冲掉上一轮可能残留的信号(比如 FOUND 超时之后用户才按的那一下) */
    (void)osMessageQueueReset(s_q);

    while (1U)
    {
        s_phase = (uint8_t)phase;
        service_ota_render(phase, progress);

        if (0U != service_ota_phase_is_end(phase))
        {
            break;
        }

        switch (phase)
        {
            case OTA_PHASE_CHECKING:
            {
                ret = ota_do_check(&ver);

                if (ret < 0)
                {
                    phase = OTA_PHASE_FAILED; /* -1 已发过 0x08; -2 是取消, 静默 */
                }
                else
                {
                    phase = (1 == ret) ? OTA_PHASE_FOUND : OTA_PHASE_UPTODATE;
                }
                break;
            }

            case OTA_PHASE_FOUND:
            {
                /* 1) 等用户点"下载". 超时就当没这回事: 投回待命那组值再自退 */
                if (0u == ota_wait_confirm())
                {
                    phase = OTA_PHASE_IDLE;
                    break;
                }

                /* 2) 0x03 → 0x04 */
                ret = ota_do_permit();
                if (ret < 0)
                {
                    phase = OTA_PHASE_FAILED;
                    break;
                }
                if (0 == ret)
                {
                    phase = OTA_PHASE_FAILED; /* 被拒绝 / 取消: 静默结束 */
                    break;
                }

                /* 3) 建 ota/ 目录(已存在也算成功) */
                if (0 != service_fatfs_mkdir(OTA_DIR_PATH))
                {
                    (void)ota_send_abort(OTA_ABORT_SAVE, OTA_ABORT_NOBLK);
                    phase = OTA_PHASE_FAILED;
                    break;
                }

                blk_done = 0u;
                progress = 0u;
                phase    = OTA_PHASE_DOWNLOADING;
                break;
            }

            case OTA_PHASE_DOWNLOADING:
            {
                uint8_t  payload[2];
                uint8_t  try_i;
                uint8_t  ok = 0u;
                int8_t   wret;
                uint32_t blk_crc;

                payload[0] = (uint8_t)(blk_done & 0xFFu);
                payload[1] = (uint8_t)((blk_done >> 8) & 0xFFu);

                for (try_i = 0u; try_i < OTA_BLK_RETRY; try_i++)
                {
                    (void)osMessageQueueReset(s_q);

                    if (0 != ota_send(OTA_CMD_BLK_REQ, payload, 2u))
                    {
                        continue;
                    }

                    wret = ota_wait_block(OTA_BLK_TIMEOUT_MS);
                    if (1 == wret)
                    {
                        ok = 1u;
                        break;
                    }
                    if (wret < 0)
                    {
                        break; /* 上位机取消 */
                    }
                }

                if (0u == ok)
                {
                    (void)ota_send_abort(OTA_ABORT_BLOCK, blk_done);
                    phase = OTA_PHASE_FAILED;
                    break;
                }

                if (s_blk_rx_len !=
                    (uint16_t)((blk_done == (uint16_t)(s_blk_num - 1u))
                               ? (s_img_len - ((uint32_t)blk_done * (uint32_t)s_blk_size))
                               : (uint32_t)s_blk_size))
                {
                    (void)ota_send_abort(OTA_ABORT_BLOCK, blk_done);
                    phase = OTA_PHASE_FAILED;
                    break;
                }

                if (0u == ota_save_block(blk_done, s_blk_buf, s_blk_rx_len))
                {
                    (void)ota_send_abort(OTA_ABORT_SAVE, blk_done);
                    phase = OTA_PHASE_FAILED;
                    break;
                }

                /* 逐块打一行, 上位机按它比对"发出去的块"——这是写盘前 RAM 里的那份 */
                blk_crc = ota_crc32_update(0xFFFFFFFFu, s_blk_buf, s_blk_rx_len)
                          ^ 0xFFFFFFFFu;
                log_printf("[OTA] W %u %u %08X\r\n", (unsigned)blk_done,
                           (unsigned)s_blk_rx_len, (unsigned)blk_crc);

                blk_done++;
                progress = (uint8_t)(((uint32_t)blk_done * 100u) / (uint32_t)s_blk_num);

                phase = (blk_done >= s_blk_num)
                        ? OTA_PHASE_VERIFYING : OTA_PHASE_DOWNLOADING;
                break;
            }

            case OTA_PHASE_VERIFYING:
            {
                uint32_t crc = 0u;

                ret = ota_do_verify(&crc);

                if (ret < 0)
                {
                    (void)ota_send_abort(OTA_ABORT_SAVE, OTA_ABORT_NOBLK);
                    phase = OTA_PHASE_FAILED;
                }
                else if (0 == ret)
                {
                    log_printf("[OTA] CRC mismatch: got %08X want %08X\r\n",
                               (unsigned)crc, (unsigned)s_img_crc32);
                    (void)ota_send_abort(OTA_ABORT_CRC, OTA_ABORT_NOBLK);
                    phase = OTA_PHASE_FAILED;
                }
                else
                {
                    uint8_t payload[4];

                    payload[0] = (uint8_t)(crc & 0xFFu);
                    payload[1] = (uint8_t)((crc >> 8) & 0xFFu);
                    payload[2] = (uint8_t)((crc >> 16) & 0xFFu);
                    payload[3] = (uint8_t)((crc >> 24) & 0xFFu);

                    /* ★单发, 后面不跟任何命令★: 上位机那侧是单槽, 多发一条会覆盖它 */
                    if (0 != ota_send(OTA_CMD_DONE, payload, 4u))
                    {
                        osDelay(20U);
                        (void)ota_send(OTA_CMD_DONE, payload, 4u);
                    }

                    log_printf("[OTA] done crc %08X\r\n", (unsigned)crc);
                    phase = OTA_PHASE_READY;
                }
                break;
            }

            default:
                phase = OTA_PHASE_IDLE;
                break;
        }
    }

    s_alive = 0U;
    osThreadExit();
}
/**********************************Functions***********************************/

void service_ota_start(void)
{
    osThreadId_t h;
    uint8_t      sig = OTA_EV_CONFIRM;

    /* ★按钮的唯一入口, 按相位分三路★ */
    if (0U != s_alive)
    {
        if ((uint8_t)OTA_PHASE_FOUND == s_phase)
        {
            /* 正停在"发现新版本"上等 —— 用户点的是"下载", 发个信号催它往下走.
               ★timeout 传 0: appcore 的任务不能在这儿阻塞★ */
            (void)osMessageQueuePut(s_q, &sig, 0U, 0U);
        }
        /* 其它相位(检查中 / 下载中 / 正在退出): 按钮本来是灰的, 什么都不做 */
        return;
    }

    /* 惰性建队列: 建一次不删, 与任务的生命周期无关 */
    if (NULL == s_q)
    {
        s_q = osMessageQueueNew(SERVICE_OTA_QUEUE_LEN, sizeof(uint8_t), NULL);
        if (NULL == s_q)
        {
            watch_page_ota_post_text("升级失败");
            watch_page_ota_post_enabled(1U);
            return;
        }
    }

    /* 清掉可能残留的信号(比如上一轮 FOUND 超时之后用户才按的那一下), 否则新任务
       一进 FOUND 就会被它催着往下走 */
    (void)osMessageQueueReset(s_q);

    h = osThreadNew(service_ota_run, NULL, &s_ota_attr);
    if (NULL == h)
    {
        /* ★建不出来必须给页面一个交代★: 32KB 内核堆(fatfs 的一次性任务要走
           4.6KB), 失败是真会发生的. 不投的话用户看到的是"点过了但什么都没发生" */
        watch_page_ota_post_text("升级失败");
        watch_page_ota_post_enabled(1U);
        return;
    }

    /* ★在这儿置 1, 不在任务入口置★: 关掉 osThreadNew 返回到任务真正跑起来之间的
       窗口, 否则窗口内再按一次会建出第二个任务 */
    s_alive = 1U;
}
