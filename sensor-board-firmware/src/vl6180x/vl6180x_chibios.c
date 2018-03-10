#include <ch.h>
#include <hal.h>
#include "vl6180x.h"

#ifdef HAL_USE_I2C
void vl6180x_write_register(vl6180x_t *dev, uint16_t reg, uint8_t val)
{
    uint8_t buf[] = {(reg >> 8), reg & 0xff, val};
    i2cMasterTransmit(dev->i2c, dev->address, buf, 3, NULL, 0);
}

uint8_t vl6180x_read_register(vl6180x_t *dev, uint16_t reg)
{
    uint8_t ret;
    uint8_t buf[] = {(reg >> 8), reg & 0xff};
    i2cMasterTransmit(dev->i2c, dev->address, buf, 2, &ret, 1);
    return ret;
}
#else
#error "VL6180X driver requires I2C. Please enable HAL_USE_I2C."
#endif
