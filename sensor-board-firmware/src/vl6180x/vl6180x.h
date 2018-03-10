#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Default address of the sensor after powerup. */
#define VL6180X_DEFAULT_ADDRESS 0x29

typedef struct {
    void *i2c;
    uint8_t address;
} vl6180x_t;

/** Configures the device using default values recommended in the datasheet.
 *
 * @parameter dev The driver instance to configure.
 * @parameter i2c_dev This pointer can be used by the vl6180x_write_register
 * and vl6180x_read_register to, for example point to the correct driver
 * interface.
 * @parameter address The device address to use. Unless you changed the address
 * in software, it should be VL6180X_DEFAULT_ADDRESS.
 * */
void vl6180x_init(vl6180x_t *dev, void *i2c_dev, uint8_t address);

/** Returns 0 if the distance was measured correctly, the error code otherwise. */
uint8_t vl6180x_measure_distance(vl6180x_t *dev, uint8_t *out_mm);

/** Sends initial configuration to device. */
void vl6180x_configure(vl6180x_t *dev);

/** Those functions are hardware specific and must be provided by the user. */
extern uint8_t vl6180x_read_register(vl6180x_t *dev, uint16_t reg);
extern void vl6180x_write_register(vl6180x_t *dev, uint16_t reg, uint8_t val);

#ifdef __cplusplus
}
#endif
#endif
