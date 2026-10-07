/******************************************************************************
 * Copyright (C) 2024 EternalChip, Inc.(Gmbh) or its affiliates.
 *
 * All Rights Reserved.
 *
 * @file cywatch_service_appcore.h
 *
 * @author zw1194
 *
 * @brief app_core 服务: EasyAPP 事件总线唯一的消费者, 轮询 easyapp_core_run()
 *        把服务任务投进来的事件分发到各 APP 页面的处理函数.
 *
 * @version V1.0
 *
 * @note 1 tab == 4 spaces!
 *
 * @note 没有它整条总线是死的: 事件只进不出, 页面处理函数永远不会被调用.
 *****************************************************************************/
#ifndef __CYWATCH_SERVICE_APPCORE_H__
#define __CYWATCH_SERVICE_APPCORE_H__

/**********************************Declaring***********************************/
/* app_core 服务初始化: 创建分发任务 */
void service_appcore_init(void);
/**********************************Declaring***********************************/

#endif // __CYWATCH_SERVICE_APPCORE_H__
