#include "driver_iic_stm32.h"
#include "sigma_log.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"
#include "interface_os.h"

extern I2C_HandleTypeDef hi2c1;

int stm32_iic_master_open(uint8_t port, uint32_t freq)
{
    return 0;
}

int stm32_iic_close(uint8_t port)
{
    return 0;
}

int stm32_iic_master_write(uint8_t port, uint8_t addr, uint8_t reg, uint8_t *data, uint32_t length, uint32_t timeout)
{
    if (HAL_I2C_Mem_Write(&hi2c1, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, data, length, timeout) != HAL_OK)
    {
        SigmaLogError(&reg, 1, "HAL_I2C_Master_Transmit failed. reg:");
        return -1;
    }
    return length;
}

int stm32_iic_master_read(uint8_t port, uint8_t addr, uint8_t reg, uint8_t *data, uint32_t length, uint32_t timeout)
{
    if (HAL_I2C_Master_Transmit(&hi2c1, addr << 1, &reg, 1, timeout) != HAL_OK)
    {
    	SigmaLogError(&reg, 1, "HAL_I2C_Master_Receive failed. reg:");
    	return -1;
    }

    if (HAL_I2C_Master_Receive(&hi2c1, addr << 1, data, length, timeout) != HAL_OK)
    {
        SigmaLogError(data, length, "HAL_I2C_Master_Receive failed. data:");
        return -1;
    }
    return length;
}

int stm32_iic_master_bitwrite(uint8_t port, uint8_t addr, uint8_t reg, uint8_t bit, uint8_t value, uint8_t length, uint32_t timeout)
{
    uint8_t data;
    if (stm32_iic_master_read(port, addr, reg, &data, 1, timeout) < 0)
        return -1;
    uint8_t mask = ((1 << length) - 1) << (bit - length + 1);
    value <<= (bit - length + 1);
    data &= ~mask;
    data |= value;
    if (stm32_iic_master_write(port, addr, reg, &data, 1, timeout) < 0)
        return -1;
    return 0;
}

int stm32_iic_master_bitget(uint8_t port, uint8_t addr, uint8_t reg, uint8_t bit, uint32_t timeout)
{
    uint8_t data;
    if (stm32_iic_master_read(port, addr, reg, &data, 1, timeout) < 0)
        return -1;
    return !!(data & (1 << bit));
}

int stm32_iic_master_bitset(uint8_t port, uint8_t addr, uint8_t reg, uint8_t bit, uint32_t timeout)
{
    uint8_t data;
    if (stm32_iic_master_read(port, addr, reg, &data, 1, timeout) < 0)
        return -1;
    data |= 1 << bit;
    if (stm32_iic_master_write(port, addr, reg, &data, 1, timeout) < 0)
        return -1;
    return 0;
}

int stm32_iic_master_bitclear(uint8_t port, uint8_t addr, uint8_t reg, uint8_t bit, uint32_t timeout)
{
    uint8_t data;
    if (stm32_iic_master_read(port, addr, reg, &data, 1, timeout) < 0)
        return -1;
    data &= ~(1 << bit);
    if (stm32_iic_master_write(port, addr, reg, &data, 1, timeout) < 0)
        return -1;
    return 0;
}
