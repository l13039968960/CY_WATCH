/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_service_Lvgl.c
 *
 * @par dependencies
 * - cywatch_service_Lvgl.h
 * - ../../LVGL/port/lv_port_disp.h
 * - ../../LVGL/port/lv_port_indev.h
 * - ../../LVGL/port/lv_watch_ui.h
 * - lvgl.h
 * - cmsis_os2.h
 *
 * @author	zw1194
 *
 * @brief LVGL 服务实现: 初始化状态机 + UI 泵循环.
 *
 * Processing flow:
 *
 * EVT_INIT        : lv_init() + lv_tick_set_cb(内核tick) —— 只执行一次
 * EVT_HW_DISP_INIT: lv_port_disp_init()(ST7789 adapter), 失败重试 5 次
 * EVT_HW_INDEV_INIT: lv_port_indev_init()(CST816T adapter), 失败重试 5 次
 *                   → 成功后载入手表UI(lv_watch_ui_init)
 * EVT_RUN         : lv_timer_handler() + osDelay(5ms)
 * EVT_ERROR       : 打印一次后空转(不自动恢复, 见下)
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 为什么把初始化拆成状态而不是 for 循环重试:
 *       1. `lv_init()` **不可重复调用**, 重入会清掉已建对象/定时器 → UI 半死.
 *          故它单独放 EVT_INIT, 之后再不回头;
 *       2. 显示与触摸分两个状态: 若显示已成功而触摸失败, 从"显示"重试会
 *          lv_display_create 出第二个显示实例. 分状态保证**已成功的步骤不复跑**;
 *       3. 两个 "port init" 失败时都还没创建 lv_display_t/lv_indev_t(先构造
 *          adapter 再建 LVGL 对象), 所以各自单独重试是安全的.
 *
 * @note 失败恢复策略: 5 次重试仍失败 → EVT_ERROR 空转, 不自愈. 显示/触摸是
 *       上电自检类外设, 反复重试会持续占用软 I2C 总线并刷屏日志; 真要恢复,
 *       由上层决定(例如看门狗复位). 这与 attitude 服务的现状一致.
 *
 * @note 本任务在 flush_cb 里是**阻塞**的(单缓冲 PARTIAL, 一次40行≈3ms DMA 等待),
 *       这期间不出触摸、不跑 timer —— 单缓冲下属必然. 优先级用 Normal 让
 *       configUSE_TIME_SLICING 把这段阻塞切给同级任务, 不会饿死 appcore/storage.
 *
 * @note 跨任务 LVGL 访问契约见 cywatch_service_Lvgl.h 头注释(必读).
 ******************************************************************************/
#include "cywatch_service_Lvgl.h"

#include <stdio.h>
#include "lvgl.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include "lv_watch_ui.h"
#include "cmsis_os2.h"

/***********************************Defines************************************/
/* ---- 测试UI已移除(2026-09-16) ----
   这里原先有两个编译期开关, 各自指向一个测试页面, 现已连同页面文件一起删除:
     LVGL_BRINGUP_TEST=1   → lv_demo_app 上电自检页(显示/触摸 PASS/FAIL 总结)
     LVGL_PAGESTACK_TEST=1 → page_demo 的 4 个 PageMem 示例页(验证 LRU 淘汰)
   删除理由: 产品UI只有手表UI一套, 这两个测试页的 .c/.h 与 Keil 组条目都是残留.

   所以现在只有**一条路**: 载入手表UI. 别在 .uvprojx 或 C/C++ → Define 里找这两个
   宏 —— 它们已经不存在了, 定义了也没有任何代码会看它.

   2026-09-17 加的 lv_watch_selftest(11 步上电自检)已于 2026-09-28 连同文件与
   Keil 组条目一起删除 —— 它从未被调用过(lv_watch_selftest_start() 全工程零调用者,
   .o 整个被链接器 GC), 属于残留. */


/* UI 泵节拍: LVGL 自身刷新周期 LV_DEF_REFR_PERIOD=33ms, 5ms 只是让触摸采样与
   timer 到期判定更及时, 不会造成额外重绘(lv_timer_handler 内部按各 timer
   period 判断). 注意不要用 osDelay(33) —— 会让触摸采样率掉到 30Hz */
#define SERVICE_LVGL_LOOP_TICK          (5u)
/* 单步初始化失败重试上限与间隔 */
#define SERVICE_LVGL_INIT_RETRY_MAX     (5)
#define SERVICE_LVGL_RETRY_DELAY_MS     (500u)
/* 错误态空转节拍 */
#define SERVICE_LVGL_ERROR_TICK         (1000u)
/***********************************Defines************************************/

/**********************************Declaring***********************************/
/* 内部状态枚举(只服务内状态机用, 不是对外事件) */
typedef enum State_Lvgl_Service
{
	EVT_INIT = 0,     /* 一次性: lv_init + 时基 */
	EVT_HW_DISP_INIT, /* 显示初始化(可重试) */
	EVT_HW_INDEV_INIT,/* 触摸初始化(可重试) + 载入UI */
	EVT_RUN,          /* UI 泵循环 */
	EVT_ERROR,        /* 初始化失败, 空转 */
} State_Lvgl_Service_t;

static State_Lvgl_Service_t ServiceState;

/* 任务属性: 栈 4096 —— LVGL 渲染递归(对象树/flex/chart) + printf 的 newlib 栈开销
   都比普通任务深, 不要沿用 skill 里的 2048 默认值 */
static const osThreadAttr_t g_service_lvgl_attr =
{
	.name       = "lvgl",
	.attr_bits  = 0,
	.cb_mem     = NULL,
	.cb_size    = 0,
	.stack_mem  = NULL,
	.stack_size = 4096,
	.priority   = osPriorityNormal,
};

static void service_lvgl_run(void *pvParameters);

/******************************************************************************
 * @name    lvgl_tick_cb
 * @brief   LVGL 时基回调(由 lv_tick_set_cb 注册, 在各 timer 判定时被调用)
 * @param   无
 *
 * @return  当前内核 tick(ms), 1kHz 与 LVGL 期望一致
 *
 * @note    用 osKernelGetTickCount 而非 HAL_GetTick: 两者同源(SysTick 共享),
 *          但内核 tick 与调度器同生共死 —— 将来若开 configUSE_TICKLESS_IDLE 或
 *          有人调 HAL_SuspendTick(), HAL_GetTick 会停/跳, 表现为动画与图表卡住
 *          且无任何报错, 极难定位
 *****************************************************************************/
static uint32_t lvgl_tick_cb(void)
{
	return osKernelGetTickCount();
}

/******************************************************************************
 * @name    service_lvgl_run
 * @brief   LVGL 服务任务: 初始化状态机 + UI 泵循环
 * @param   pvParameters[in] 未使用
 *
 * @return  无(不返回)
 *****************************************************************************/
static void service_lvgl_run(void *pvParameters)
{
	int8_t ret = 0;
	int error_cnt = 0;
	static uint8_t s_error_reported = 0;

	(void)pvParameters;

	while (1)
	{
		switch (ServiceState)
		{
		case EVT_INIT:
			/* 1. LVGL 内核初始化 + 时基(只做一次, lv_init 不可重入) */
			lv_init();
			lv_tick_set_cb(lvgl_tick_cb);
			error_cnt = 0;
			ServiceState = EVT_HW_DISP_INIT;
			break;

		case EVT_HW_DISP_INIT:
			/* 2. 显示: ST7789 adapter(SPI1 + 面板初始化) + LVGL 显示对象 */
			ret = lv_port_disp_init();
			if (0 == ret)
			{
				error_cnt = 0;
				ServiceState = EVT_HW_INDEV_INIT;
				break;
			}

			error_cnt++;
			if (error_cnt >= SERVICE_LVGL_INIT_RETRY_MAX)
			{
				printf("LVGL: display init failed %d times\r\n", error_cnt);
				ServiceState = EVT_ERROR;
				break;
			}
			osDelay(SERVICE_LVGL_RETRY_DELAY_MS);
			break;

		case EVT_HW_INDEV_INIT:
			/* 3. 触摸: CST816T adapter(独立软I2C + RST + EXTI + ChipID自检) */
			ret = lv_port_indev_init();
			if (0 != ret)
			{
				error_cnt++;
				if (error_cnt >= SERVICE_LVGL_INIT_RETRY_MAX)
				{
					printf("LVGL: touch init failed %d times\r\n", error_cnt);
					ServiceState = EVT_ERROR;
					break;
				}
				osDelay(SERVICE_LVGL_RETRY_DELAY_MS);
				break;
			}

			/* 4. 载入手表UI(必须在显示/触摸端口就绪之后) */
			if (0 != lv_watch_ui_init())
			{
				printf("LVGL: watch UI create failed\r\n");
				ServiceState = EVT_ERROR;
				break;
			}

			error_cnt = 0;
			ServiceState = EVT_RUN;
			break;

		case EVT_RUN:
			/* 5. UI 泵: 跑定时器/动画/渲染, 触摸读回调也由它按 indev 周期驱动 */
			lv_timer_handler();
			osDelay(SERVICE_LVGL_LOOP_TICK);
			break;

		case EVT_ERROR:
			/* 终端错误态: 只报一次, 之后空转让出(不自愈, 理由见文件头 @note) */
			if (0 == s_error_reported)
			{
				printf("LVGL: service stopped in ERROR state\r\n");
				s_error_reported = 1;
			}
			osDelay(SERVICE_LVGL_ERROR_TICK);
			break;

		default:
			ServiceState = EVT_ERROR;
			break;
		}
	}
}

/******************************************************************************
 * @name    service_lvgl_init
 * @brief   启动 LVGL 服务(只建任务, 不碰设备)
 * @param   无
 *
 * @return  0 success
 *         -1 任务创建失败
 *
 * @note    在 osKernelInitialize() 之后、osKernelStart() 之前调用; 设备实例化
 *          (含 osDelay)在任务体内做, 见 cywatch_service_Lvgl.c 文件头
 *****************************************************************************/
int8_t service_lvgl_init(void)
{
	ServiceState = EVT_INIT;

	if (NULL == osThreadNew(service_lvgl_run, NULL, &g_service_lvgl_attr))
	{
		printf("LVGL: osThreadNew failed\r\n");
		return -1;
	}

	return 0;
}
