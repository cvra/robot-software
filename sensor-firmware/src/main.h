#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <bootloader_config.h>
extern bootloader_config_t config;

#include <vl6180x/vl6180x.h>
extern vl6180x_t vl6180x_dev;

#include <TCS3472.h>
extern TCS3472_t color_sensor;

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
