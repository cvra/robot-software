#ifndef PRESSURE_SENSOR_INTERFACE_H
#define PRESSURE_SENSOR_INTERFACE_H

#include "pressure_sensor.h"

void mpr_start(void);

extern mpr_driver_t pressure_sensors[2];

#endif
