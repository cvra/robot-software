#include <ch.h>
#include <hal.h>
#include "emergency_stop_handler.hpp"
#include <cvra/motor/EmergencyStop.hpp>

static void emergency_stop_cb(const uavcan::ReceivedDataStructure<cvra::motor::EmergencyStop>& msg)
{
    (void)msg;
    NVIC_SystemReset();
}

int emergency_stop_init(uavcan::INode& node)
{
    uavcan::Subscriber<cvra::motor::EmergencyStop> emergency_stop_sub(node);
    return emergency_stop_sub.start(emergency_stop_cb);
}
