#pragma once

#include <uavcan/uavcan.hpp>
#include "bus_enumerator.h"

int actuator_handler_init(uavcan::INode& node, bus_enumerator_t* enumerator);
