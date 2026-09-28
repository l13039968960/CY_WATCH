#ifndef __CYWATCH_APP_DATAMODEL_H__
#define __CYWATCH_APP_DATAMODEL_H__

#include <stdint.h>

/* @note volatile 必须**声明与定义同时带**: .c 里写成 volatile uint8_t HeartRate;
   (定义), 这里若省掉 volatile, 就是 error: redefinition of 'HeartRate' with a
   different type —— C 里限定符属于类型的一部分, 不匹配就重定义报错, 而且报的是
   "重定义"这种看着像重复定义、实际是类型不一致的错.
   @note 这里加 volatile 的意义是**文档性**的: 读端(lvgl 任务)与写端(appcore 任务)
   分属两个任务, 声明上写出来等于把"这是跨任务共享的"写在接口上. 它**不能**替代
   互斥 —— volatile 不保证多字段的原子性(单字节 uint8_t 本身是原子的, 但
   Step/KCal 是 4 字节, 读侧可能读到写了一半的值). */
extern volatile uint8_t HeartRate;
extern volatile uint32_t Step;
extern volatile uint32_t KCal;

/* @note 0 = **本帧没有有效读数**, 不是"血氧 0%". 生产者
   (service/HeartRate 的 cywatch_service_HeartRate.c) 已经把无效帧统一写成 0:
       spo2_percent = (0 != sp.valid) ? sp.spo2 : 0.0f;
   所以这里不需要再加一个 valid 字段 —— 0 本身就是哨兵, 与 HeartRate 的既有写法
   一致(见 LVGL/port/lv_watch_page_spo2.c 里对它的判读) */
extern volatile uint8_t SpO2;
extern volatile uint8_t FingerOn;

#endif
