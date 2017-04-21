#include <ch.h>
#include <hal.h>
#include "emergency_stop_handler.hpp"
#include <cvra/motor/EmergencyStop.hpp>


int emergency_stop_init(uavcan::INode &node)
{
    int res;

    uavcan::Subscriber<cvra::motor::EmergencyStop> emergency_stop_sub(node);
    res = emergency_stop_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::EmergencyStop>& msg)
    {
        (void) msg;
        NVIC_SystemReset();
    });

    return res;
}
