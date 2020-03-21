#include "Command_handler.hpp"
#include "servo.h"

#include <error/error.h>

#include <cvra/actuator/Command.hpp>

#include "uavcan/build_config.hpp"

int Command_handler_start(uavcan::INode& node)
{
    static uavcan::Subscriber<cvra::actuator::Command> sub(node);
    return sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::actuator::Command>& msg) {
            /* Filter to make sure the message is intended for us. */
            if (uavcan::NodeID(msg.node_id) != node.getNodeID()) {
                DEBUG("invalid nodeid, expected %d, got %d",
                      static_cast<int>(node.getNodeID().get()),
                      static_cast<int>(msg.node_id));
                return;
            }
            /* TODO: Implement a watchdog to stop the movements if no setpoints come from master */
            /* TODO: Handle commands other than servomotors. */

            UAVCAN_ASSERT(SERVO_COUNT == msg.servo_trajectories.size());

            for (int i = 0; i < SERVO_COUNT; i++) {
                servo_set(i, msg.servo_trajectories[i].position, msg.servo_trajectories[i].velocity, msg.servo_trajectories[i].acceleration);
                DEBUG_EVERY_N(10, "received setpoint %03d for servo %d", static_cast<int>(msg.servo_trajectories[i].position * 1000), i);
            }
        });
}
