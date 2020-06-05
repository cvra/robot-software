#include <cvra/motor/control/Position.hpp>
#include "Position_handler.hpp"
#include "uavcan_node.h"
#include <timestamp/timestamp.h>
#include "control.h"

int Position_handler_start(Node& node)
{
    int ret;
    static uavcan::Subscriber<cvra::motor::control::Position> sub(node);

    ret = sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::control::Position>& msg) {
            if (uavcan::NodeID(msg.node_id) == node.getNodeID()) {
                control_update_position_setpoint(msg.position);
            }
        });

    return ret;
}
