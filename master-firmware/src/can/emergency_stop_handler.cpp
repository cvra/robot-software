#include "emergency_stop_handler.hpp"
#include <cvra/motor/EmergencyStop.hpp>
#include <error/error.h>

static void emergency_stop_cb(const uavcan::ReceivedDataStructure<cvra::motor::EmergencyStop>& msg)
{
    (void)msg;
    WARNING("received emergency stop, exiting...");
    exit(1);
}

int emergency_stop_init(uavcan::INode& node)
{
    uavcan::Subscriber<cvra::motor::EmergencyStop> emergency_stop_sub(node);
    return emergency_stop_sub.start(emergency_stop_cb);
}
