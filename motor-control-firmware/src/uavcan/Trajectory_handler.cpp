#include "Trajectory_handler.hpp"
#include "uavcan_node.h"
#include "timestamp/timestamp.h"
#include "control.h"

void Trajectory_handler(const uavcan::ReceivedDataStructure<cvra::motor::control::Trajectory> &msg)
{
    if (uavcan::NodeID(msg.node_id) == get_node().getNodeID()) {
        timestamp_t timestamp = timestamp_get();
        control_update_trajectory_setpoint(msg.position,
                msg.velocity,
                msg.acceleration,
                msg.torque,
                timestamp);
    }
}

