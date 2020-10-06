#include "ProximityBeaconEmulator.h"
#include <error/error.h>

constexpr float reflector_diameter = 0.08f;

// TODO: Calibrate this
constexpr float min_detection_distance = 1.5f;

ProximityBeaconEmulator::ProximityBeaconEmulator(std::string can_iface, std::string board_name, int node_number)
    : driver(clock)
{
    NOTICE("Proximity beacon emulator on %s", can_iface.c_str());
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

    pub = std::make_unique<Publisher>(*node);
    publish_timer = std::make_unique<uavcan::Timer>(*node);
    publish_timer->setCallback(
        [&](const uavcan::TimerEvent& event) {
            (void)event;
            cvra::proximity_beacon::Signal msg;

            float distance, angle;

            {
                absl::MutexLock l(&lock);
                auto delta = opponent_pos - robot_pos;
                distance = delta.Length();
                angle = atan2f(delta.y, delta.x);
                msg.start_angle = angle - robot_heading;
            }

            if (distance < min_detection_distance) {
                msg.length = reflector_diameter / distance;
                pub->broadcast(msg);
            }
        });
    publish_timer->startPeriodic(uavcan::MonotonicDuration::fromMSec(100));
}

void ProximityBeaconEmulator::start()
{
    std::thread new_thread(&ProximityBeaconEmulator::spin, this);
    std::swap(new_thread, can_thread);
}

void ProximityBeaconEmulator::spin()
{
    node->start();
    while (true) {
        const int res = node->spin(uavcan::MonotonicDuration::fromMSec(1000));
        if (res < 0) {
            WARNING("UAVCAN failure: %d", res);
        }
    }
}
