#ifndef __CYWATCH_ADAPTER_MAX30102_H__
#define __CYWATCH_ADAPTER_MAX30102_H__

#include <stdint.h>

int8_t heartrate_bsp_inst(void);
int8_t heartrate_bsp_deinst(void);
int8_t heartrate_bsp_read_id(void);
int8_t heartrate_bsp_change_to_HR(void);
int8_t heartrate_bsp_change_to_spo2(void);
int8_t heartrate_bsp_enable_FIFO_FULL_interrupt(void);
int8_t heartrate_bsp_disable_FIFO_FULL_interrupt(void);
int8_t heartrate_bsp_read_all_FIFO_samples(uint32_t *p_red_buff, uint32_t *p_ir_buff);
int8_t heartrate_bsp_read_one_sample(uint32_t *p_red, uint32_t *p_ir, uint32_t *sample_size);
int8_t heartrate_bsp_wait_interrupt(void);
int8_t heartrate_bsp_hibernating(void);
int8_t heartrate_bsp_wakeup(void);

/* INT(PA3/EXTI3)中断转发: 应用层 EXTI 回调(ISR)内调用, 转驱动的 pf_interrupt_cb,
   仅释放信号量/置标志, 不做任何 I2C */
int8_t heartrate_bsp_interrupt_cb(void);

#endif
