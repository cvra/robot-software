#include <cvra/motor/control/Velocity.hpp>
#include "Velocity_handler.hpp"
#include "uavcan_node.h"
#include <timestamp/timestamp.h>
#include "control.h"

int Velocity_handler_start(Node& node)
{
    int ret;
    static uavcan::Subscriber<cvra::motor::control::Velocity> sub(node);

    ret = sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::control::Velocity>& msg) {
            if (uavcan::NodeID(msg.node_id) == node.getNodeID()) {
                control_update_velocity_setpoint(msg.velocity);
            }
        });

    return ret;
}
