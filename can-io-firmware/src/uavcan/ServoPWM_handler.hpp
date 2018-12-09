#ifndef PWM_SERVER_H
#define PWM_SERVER_H

#include <uavcan/uavcan.hpp>
#include <cvra/io/ServoPWM.hpp>

void ServoPWM_handler(const uavcan::ReceivedDataStructure<cvra::io::ServoPWM>&);

#endif
