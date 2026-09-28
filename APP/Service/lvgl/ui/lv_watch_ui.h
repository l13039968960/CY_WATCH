/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file lv_watch_ui.h
 *
 * @par dependencies
 * - lvgl.h
 * - ../Lvgl_PageManger/PageMem.h
 *
 * @author zw1194
 *
 * @brief 手表UI(自研): 表盘/菜单/心率三页 + 手势切页 + 每秒刷新.
 *        早期那套UI的页面切换会卡死且根因未定位, 本套改用验证过不会卡的
 *        demo同款模式编写, 作为稳定可用的手表界面.
 *
 * Processing flow:
 *
 * lv_watch_ui_init():
 *   1. 建 PageMem 管理器并注册三页(home/menu/heart) —— 此时一个控件都没建;
 *   2. 建模型定时器(1s, 推进时钟/步数/心率);
 *   3. 切到 home(瞬切), home 的 pf_create 在这一步才被调到.
 *   menu/heart 的控件到**第一次切过去时**才建(懒创建, 省 LVGL 内存池).
 *
 * 页面结构/生命周期全部由 PageMem 托管, 见 ../Lvgl_PageManger/PageMem.h:
 *   最近3页缓冲区(LRU), 超出即淘汰 + 下次切到时 pf_create 重建.
 *   本文件不自己 lv_obj_delete 页面根屏幕, 也不在 pf_* 回调里再发起切页.
 *
 * 文件构成(每页一个 .c, 内部接口见 lv_watch_page.h):
 *   lv_watch_ui.c        入口 + 模型数据 + 共用工具(切页/手势条/拖拽方向)
 *   lv_watch_page_home.c 表盘页   lv_watch_page_menu.c 菜单页
 *   lv_watch_page_heart.c心率页
 *
 * 手势: home下滑→menu(OVER_BOTTOM), home右滑→heart(MOVE_RIGHT),
 *       menu上滑→home(OUT_TOP), heart左滑→home(MOVE_LEFT).
 *
 * @version V2.1
 *
 * @note 1 tab == 4 spaces!
 *
 * @note ★须在 lv_init/lv_port_disp_init/lv_port_indev_init 之后调用,
 *         且只能在 "lvgl" 任务里调用(跨任务访问契约见 PageMem.h)★
 *
 * @note V1.0→V2.0: 从"init 时一次性建三页并常驻"改为"注册三页 + 懒创建 +
 *       最近3页 LRU"(数据结构与生命周期交给页面管理器);
 *       V2.0→V2.1: 三页各拆一个文件, 本头文件对外接口不变.
 ******************************************************************************/
#ifndef __LV_WATCH_UI_H__
#define __LV_WATCH_UI_H__

#include <stdint.h>

#include "../Lvgl_PageManger/PageMem.h"

/* 注册手表UI三页并加载表盘首页; 0=成功, 见 .c 的 @return */
int8_t lv_watch_ui_init(void);

/******************************************************************************
 * @name    lv_watch_ui_mgr
 * @brief   取本UI的页面管理器实例(自检/诊断用, 生产逻辑不用)
 * @param   无
 *
 * @return  管理器指针(初始化后恒非 NULL); lv_watch_ui_init() 之前调用返回的
 *          实例虽非 NULL 但未初始化(注册表为空)
 *
 * @note    为观察生产管理器而开的口子 —— 管理器实例是本文件的 static, 没有它
 *          台架上就只能靠 watch_switch() 黑箱驱动, 看不到缓冲区/状态/返回码.
 *          **页面代码仍然只走 watch_switch()**, 别拿这个函数去绕过它切页.
 *          (它最初的唯一使用者是 lv_watch_selftest.c, 该模块已于 2026-09-28 删除,
 *          所以本函数当前零调用者; 留着是台架诊断口子)
 * @note    与所有 PageMem API 同契约: 只能在 "lvgl" 任务里调.
 *****************************************************************************/
page_mgr_t *lv_watch_ui_mgr(void);

#endif // __LV_WATCH_UI_H__
