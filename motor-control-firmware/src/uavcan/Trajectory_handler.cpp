#include <cvra/motor/control/Trajectory.hpp>
#include "Trajectory_handler.hpp"
#include "uavcan_node.h"
#include <timestamp/timestamp.h>
#include "control.h"

int Trajectory_handler_start(Node& node)
{
    int ret;
    static uavcan::Subscriber<cvra::motor::control::Trajectory> sub(node);

    ret = sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::control::Trajectory>& msg) {
            if (uavcan::NodeID(msg.node_id) == node.getNodeID()) {
                timestamp_t timestamp = timestamp_get();
                control_update_trajectory_setpoint(msg.position,
                                                   msg.velocity,
                                                   msg.acceleration,
                                                   msg.torque,
                                                   timestamp);
            }
        });

    return ret;
}
