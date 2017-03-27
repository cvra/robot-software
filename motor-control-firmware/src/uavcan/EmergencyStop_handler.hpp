#ifndef EMERGENCYSTOP_HANDLER_HPP
#define EMERGENCYSTOP_HANDLER_HPP

#include <uavcan/uavcan.hpp>
#include <cvra/motor/EmergencyStop.hpp>

void EmergencyStop_handler(const uavcan::ReceivedDataStructure<cvra::motor::EmergencyStop> &msg);



#endif
