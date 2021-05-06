#ifndef __DRIVER_IIC_STM32_H__
#define __DRIVER_IIC_STM32_H__

#include "env.h"

int stm32_iic_master_open(uint8_t port, uint32_t freq);
int stm32_iic_close(uint8_t port);
int stm32_iic_master_write(uint8_t port, uint8_t addr, uint8_t reg, uint8_t *data, uint32_t length, uint32_t timeout);
int stm32_iic_master_read(uint8_t port, uint8_t addr, uint8_t reg, uint8_t *data, uint32_t length, uint32_t timeout);
int stm32_iic_master_bitwrite(uint8_t port, uint8_t addr, uint8_t reg, uint8_t bit, uint8_t value, uint8_t length, uint32_t timeout);
int stm32_iic_master_bitget(uint8_t port, uint8_t addr, uint8_t reg, uint8_t bit, uint32_t timeout);
int stm32_iic_master_bitset(uint8_t port, uint8_t addr, uint8_t reg, uint8_t bit, uint32_t timeout);
int stm32_iic_master_bitclear(uint8_t port, uint8_t addr, uint8_t reg, uint8_t bit, uint32_t timeout);

#endif
