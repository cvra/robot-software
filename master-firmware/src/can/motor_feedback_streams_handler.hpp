#ifndef MOTOR_FEEDBACK_STREAMS_HANDLER_HPP
#define MOTOR_FEEDBACK_STREAMS_HANDLER_HPP

#include <uavcan/uavcan.hpp>
#include "bus_enumerator.h"

int motor_feedback_stream_handler_init(uavcan::INode &node, bus_enumerator_t *enumerator);

#endif
