#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <unistd.h>

typedef struct {
    void* arg;
    void (*transmit)(void* arg, const uint8_t* tx, uint8_t* rx, size_t n);
    void (*select)(void* arg);
    void (*unselect)(void* arg);
} mpr_driver_t;

/** Starts a pressure reading on this sensor.
 *
 * The conversion should take up to 6.2 ms, and then the result should be
 * available.
 */
void mpr_start_measurement(mpr_driver_t* drv);

/** Reads the status byte for this sensor.
 *
 * The status byte is made of the following fields:
 * - Bit 6: Power status. Equals to 1 if the device is powered up.
 * - Bit 5: Busy bit. Equals to 1 if the device is busy on a conversion.
 * - Bit 2: Memory error. Equals to 1 if the device has a checksum error on its
 *   memory.
 * - Bit 0: Math saturation. Equals to 1 if a math saturation occured.
 *
 * All the other bits should be zero.
 */
uint8_t mpr_read_status(mpr_driver_t* drv);

/** Reads the raw pressure data from the device. */
uint32_t mpr_read_data(mpr_driver_t* drv);

/** Returns true if the given device status represents an error. */
int mpr_status_is_error(uint8_t status);

/** Returns true if the given device status represents a busy device. */
int mpr_status_is_busy(uint8_t status);

float mpr_pressure_raw_to_pascal(uint32_t raw_pressure);

#ifdef __cplusplus
}
#endif

#endif
