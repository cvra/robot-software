#include <error/error.h>
#include "can/actuator_driver_uavcan.hpp"
#include <uavcan/uavcan.hpp>
#include <cvra/actuator/Command.hpp>
#include "can/actuator_driver.h"

/* for bus_enumerator */
#include "can/uavcan_node.h"
#include "main.h"

void publish_command(actuator_driver_t* drv, uavcan::Publisher<cvra::actuator::Command>& pub)
{
    cvra::actuator::Command msg;
    int node_id;
    float pump[2];
    bool solenoid[2];
    float position;

    bool ready = actuator_driver_prepare_uavcan_msg(drv,
                                                    &bus_enumerator,
                                                    &node_id,
                                                    &pump[0],
                                                    &pump[1],
                                                    &solenoid[0],
                                                    &solenoid[1],
                                                    &position);

    msg.node_id = node_id;
    msg.pump[0] = pump[0];
    msg.pump[1] = pump[1];
    msg.solenoid[0] = solenoid[0];
    msg.solenoid[1] = solenoid[1];

    // Send it on both, even if we only have one channel
    msg.servo_trajectories[0].position = position;
    msg.servo_trajectories[1].position = position;

    if (ready) {
        pub.broadcast(msg);
    }
}

int actuator_driver_uavcan_init(uavcan::INode& node)
{
    static uavcan::Publisher<cvra::actuator::Command> pub(node);
    static uavcan::Timer periodic_timer(node);

    periodic_timer.setCallback([&](const uavcan::TimerEvent& event) {
        (void)event;
        publish_command(&actuator_front_left, pub);
        publish_command(&actuator_front_center, pub);
        publish_command(&actuator_front_right, pub);
        publish_command(&actuator_back_left, pub);
        publish_command(&actuator_back_center, pub);
        publish_command(&actuator_back_right, pub);
    });

    periodic_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(100));

    return 0;
}
