#include "actuator_board_emulator.h"
#include <error/error.h>

ActuatorBoardEmulator::ActuatorBoardEmulator(std::string can_iface, std::string board_name, int node_number)
    : driver(clock)
    , digital_input(false)
{
    pressure_pa[0] = 0.f;
    pressure_pa[1] = 0.f;

    NOTICE("actuator board emulator on %s", can_iface.c_str());
    if (driver.addIface(can_iface) < 0) {
        ERROR("Failed to add iface %s", can_iface.c_str());
    }
    node = std::make_unique<Node>(driver, clock);
    node->setHealthOk();
    node->setModeOperational();
    if (!node->setNodeID(node_number)) {
        ERROR("Invalid node number %d", node_number);
    }
    node->setName(board_name.c_str());

    feedback_pub = std::make_unique<ActuatorFeedbackPub>(*node);
    publish_timer = std::make_unique<uavcan::Timer>(*node);
    publish_timer->setCallback(
        [&](const uavcan::TimerEvent& event) {
            (void)event;
            cvra::actuator::Feedback msg;
            {
                absl::MutexLock _(&lock);
                msg.pressure[0] = pressure_pa[0];
                msg.pressure[1] = pressure_pa[1];
                msg.digital_input = digital_input;
            }
            WARNING("pub");
            feedback_pub->broadcast(msg);
        });
    publish_timer->startPeriodic(uavcan::MonotonicDuration::fromMSec(100));
}

void ActuatorBoardEmulator::start()
{
    std::thread new_thread(&ActuatorBoardEmulator::spin, this);
    std::swap(new_thread, can_thread);
}

void ActuatorBoardEmulator::spin()
{
    node->start();
    while (true) {
        const int res = node->spin(uavcan::MonotonicDuration::fromMSec(1000));
        if (res < 0) {
            WARNING("UAVCAN failure: %d", res);
        }
    }
}
