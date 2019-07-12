#include <cvra/motor/control/Torque.hpp>
#include "Torque_handler.hpp"
#include "uavcan_node.h"
#include <timestamp/timestamp.h>
#include "control.h"

int Torque_handler_start(Node& node)
{
    int ret;
    static uavcan::Subscriber<cvra::motor::control::Torque> sub(node);

    ret = sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::control::Torque>& msg) {
            if (uavcan::NodeID(msg.node_id) == node.getNodeID()) {
                control_update_torque_setpoint(msg.torque);
            }
        });

    return ret;
}
