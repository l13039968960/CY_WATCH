#ifndef __CYWATCH_ADAPTER_AT24C02_H__
#define __CYWATCH_ADAPTER_AT24C02_H__

#include <stdint.h>

int8_t storage_bsp_at24c02_inst(void);
int8_t storage_bsp_at24c02_deinst(void);
int8_t storage_bsp_at24c02_read(uint16_t addr, uint8_t *pdata, uint16_t size);
int8_t storage_bsp_at24c02_write(uint16_t addr, uint8_t *pdata, uint16_t size);

#endif
