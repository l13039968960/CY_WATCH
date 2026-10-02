#ifndef __CYWATCH_ADAPTER_AHT21_H__
#define __CYWATCH_ADAPTER_AHT21_H__

#include <stdint.h>

int8_t humiture_bsp_inst(void);
int8_t humiture_bsp_deinst(void);
int8_t humiture_bsp_read_id(void);

/* 一次触发同时得到温度(℃)与湿度(%RH) —— AHT21 的温湿度由同一次测量产出,
   分两次调用要各触发一次(各花 80ms), 故不提供单独取温度或湿度的口 */
int8_t humiture_bsp_read_temp_humi(float *p_temperature, float *p_humidity);

int8_t humiture_bsp_hibernating(void);
int8_t humiture_bsp_wakeup(void);

#endif
