#include <ch.h>
#include <hal.h>
#include "Command_handler.hpp"

#include "servo.h"
#include "pump.h"
#include "safety.h"

#include <error/error.h>

#include <cvra/actuator/Command.hpp>

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
            UAVCAN_ASSERT(SERVO_COUNT == msg.servo_trajectories.size());

            /* Servomotors commands */
            for (int i = 0; i < SERVO_COUNT; i++) {
                servo_set(i, msg.servo_trajectories[i].position, msg.servo_trajectories[i].velocity, msg.servo_trajectories[i].acceleration);
                DEBUG_EVERY_N(10, "received setpoint %03d for servo %d", static_cast<int>(msg.servo_trajectories[i].position * 1000), i);
            }

            /* Pump commands */
            DEBUG_EVERY_N(10, "pumps: [%03d; %03d], solenoids: [%d; %d]",
                          static_cast<int>(msg.pump[0] * 1000),
                          static_cast<int>(msg.pump[1] * 1000),
                          msg.solenoid[0],
                          msg.solenoid[1]);
            pump_set_pwm(0, msg.pump[0]);
            pump_set_pwm(1, msg.pump[1]);
            pump_set_solenoid(0, msg.solenoid[0]);
            pump_set_solenoid(1, msg.solenoid[1]);

            safety_timer_restart();
        });
}
