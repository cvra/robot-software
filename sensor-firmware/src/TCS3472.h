#ifndef TCS3472_H
#define TCS3472_H

#ifdef __cplusplus
extern "C" {
#endif

#include <hal.h>
#include <stdint.h>
#include <stdbool.h>

// INTEGRATION_TIME = 2.4 ms × (256 − ATIME)
// => ATIME = 256 − Integration Time / 2.4 ms
#define TCS3472_INTEGRATION_TIME_2_4_MS (0xFF)
#define TCS3472_INTEGRATION_TIME_24_MS  (0xF6)
#define TCS3472_INTEGRATION_TIME_101_MS (0xD5)
#define TCS3472_INTEGRATION_TIME_154_MS (0xC0)
#define TCS3472_INTEGRATION_TIME_700_MS (0x00)
#define TCS3472_INTEGRATION_TIME_MS(t)  ((uint8_t)(256U - (10 * t) / 24))

#define TCS34721_GAIN_1X                (0x00)
#define TCS34721_GAIN_4X                (0x01)
#define TCS34721_GAIN_16X               (0x02)
#define TCS34721_GAIN_60X               (0x03)

typedef struct {
    I2CDriver *i2c;
} TCS3472_t;

void TCS3472_init(TCS3472_t *dev, I2CDriver *i2c);
void TCS3472_configure(TCS3472_t *dev, uint8_t gain, uint8_t integration_time);
bool TCS3472_ping(TCS3472_t *dev);
bool TCS3472_read_color(TCS3472_t *dev, uint16_t *rgbc);

#ifdef __cplusplus
}
#endif

#endif /* TCS3472_H */