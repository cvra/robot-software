#ifndef MPU9250_H
#define MPU9250_H

#ifdef __cplusplus
extern "C" {
#endif

#include <hal.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    SPIDriver *spi;
} mpu9250_t;

/** Initializes the given MPU9250 driver instance, but does not configure the
 * actual chip. */
void mpu9250_init(mpu9250_t *dev, SPIDriver *spi_dev);

/** Returns true if the MPU9250 is correctly detected, false otherwise. */
bool mpu9250_ping(mpu9250_t *dev);

/** Sends the configuration to the MPU9250. */
void mpu9250_configure(mpu9250_t *dev);

/** Reset the internal registers and restores the default settings. */
void mpu9250_reset(mpu9250_t *dev);

/** Enables readout of the built in magnetometer. */
void mpu9250_enable_magnetometer(mpu9250_t *dev);

/** Returns the interrupt status register, clearing the interrupt at the same
 * time. */
uint8_t mpu9250_interrupt_read_and_clear(mpu9250_t *dev);

/** Reads raw values from the gyroscope.
 *
 * @note results are in radians / s.
 */
void mpu9250_gyro_read(mpu9250_t *dev, float *x, float *y, float *z);

/** Reads values from the magnetometer.
 *
 * @todo Convert the values.
 */
void mpu9250_mag_read(mpu9250_t *dev, float *x, float *y, float *z);

/** Reads raw values from the accelerometer.
 *
 * @note results are in m/s/s.
 */
void mpu9250_acc_read(mpu9250_t *dev, float *x, float *y, float *z);

/** Reads temperature data from the MPU.
 *
 * @note result is in C.
 */
float mpu9250_temp_read(mpu9250_t *dev);

#ifdef __cplusplus
}
#endif
#endif // MPU9250_H
