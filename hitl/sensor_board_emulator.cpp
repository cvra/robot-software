#include "sensor_board_emulator.h"
#include <error/error.h>

SensorBoardEmulator::SensorBoardEmulator(std::string can_iface, std::string board_name, int node_number)
    : driver(clock)
    , distance_mm(0)
{
    NOTICE("Motor board emulator on %s", can_iface.c_str());
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

    distance_pub = std::make_unique<DistanceVL6180XPub>(*node);
    publish_timer = std::make_unique<uavcan::Timer>(*node);
    publish_timer->setCallback(
        [&](const uavcan::TimerEvent& event) {
            (void)event;
            cvra::sensor::DistanceVL6180X msg;
            {
                absl::MutexLock _(&lock);
                msg.distance_mm = distance_mm;
            }
            distance_pub->broadcast(msg);
        });
    publish_timer->startPeriodic(uavcan::MonotonicDuration::fromMSec(100));
}

void SensorBoardEmulator::start()
{
    std::thread new_thread(&SensorBoardEmulator::spin, this);
    std::swap(new_thread, can_thread);
}

void SensorBoardEmulator::set_distance(int d)
{
    absl::MutexLock _(&lock);
    distance_mm = d;
}

void SensorBoardEmulator::spin()
{
    node->start();
    while (true) {
        const int res = node->spin(uavcan::MonotonicDuration::fromMSec(1000));
        if (res < 0) {
            WARNING("UAVCAN failure: %d", res);
        }
    }
}
