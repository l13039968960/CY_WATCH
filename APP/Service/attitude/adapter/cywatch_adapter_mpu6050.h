#ifndef __CYWATCH_ADAPTER_MPU6050_H__
#define __CYWATCH_ADAPTER_MPU6050_H__

#include <stdint.h>

int8_t attitudecalculation_bsp_inst(void);
int8_t attitudecalculation_bsp_deinst(void);
int8_t attitudecalculation_bsp_read_id(void);
int8_t attitudecalculation_bsp_read_accel(float *p_accel_x, float *p_accel_y, float *p_accel_z);
int8_t attitudecalculation_bsp_read_gyro(float *p_gyro_x, float *p_gyro_y, float *p_gyro_z);
int8_t attitudecalculation_bsp_hibernating(void);
int8_t attitudecalculation_bsp_wakeup(void);



#endif
