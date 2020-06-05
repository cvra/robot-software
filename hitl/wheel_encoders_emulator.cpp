#include "wheel_encoders_emulator.h"
#include <error/error.h>

WheelEncoderEmulator::WheelEncoderEmulator(std::string can_iface, std::string board_name, int node_number)
    : driver(clock)
    , left_encoder(0)
    , right_encoder(0)
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

    encoder_pub = std::make_unique<WheelEncoderPub>(*node);
    publish_timer = std::make_unique<uavcan::Timer>(*node);
    publish_timer->setCallback(
        [&](const uavcan::TimerEvent& event) {
            (void)event;
            DEBUG("publishing wheel position %d %d", left_encoder, right_encoder);
            cvra::odometry::WheelEncoder msg;
            {
                absl::MutexLock _(&lock);
                msg.left_encoder_raw = left_encoder;
                msg.right_encoder_raw = right_encoder;
            }
            encoder_pub->broadcast(msg);
        });
    publish_timer->startPeriodic(uavcan::MonotonicDuration::fromMSec(10));
}

void WheelEncoderEmulator::start()
{
    std::thread new_thread(&WheelEncoderEmulator::spin, this);
    std::swap(new_thread, can_thread);
}

void WheelEncoderEmulator::set_encoders(int left, int right)
{
    absl::MutexLock _(&lock);
    left_encoder = left;
    right_encoder = right;
}

void WheelEncoderEmulator::spin()
{
    node->start();
    while (true) {
        const int res = node->spin(uavcan::MonotonicDuration::fromMSec(1000));
        if (res < 0) {
            WARNING("UAVCAN failure: %d", res);
        }
    }
}
