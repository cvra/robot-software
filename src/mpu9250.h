#ifndef MPU9250_H
#define MPU9250_H

#include <hal.h>
#include <stdint.h>

typedef struct {
    SPIDriver *spi;
} mpu9250_t;


void mpu9250_init(mpu9250_t *dev, SPIDriver *spi_dev);
bool mpu9250_ping(mpu9250_t *dev);

#endif // MPU9250_H
