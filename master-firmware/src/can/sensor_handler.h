#ifndef SENSOR_HANDLER_H
#define SENSOR_HANDLER_H

#include <uavcan/uavcan.hpp>
#include "bus_enumerator.h"

int sensor_handler_init(uavcan::INode& node, bus_enumerator_t* enumerator);

#endif /* SENSOR_HANDLER_H */
