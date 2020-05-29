#ifndef WHEEL_ENCODERS_EMULATOR_H
#define WHEEL_ENCODERS_EMULATOR_H

#include <absl/synchronization/mutex.h>
#include <memory>
#include <thread>
#include <uavcan_linux/uavcan_linux.hpp>
#include "uavcan_node.h"
#include <cvra/odometry/WheelEncoder.hpp>

class WheelEncoderEmulator {
    uavcan_linux::SystemClock clock;
    uavcan_linux::SocketCanDriver driver;
    std::unique_ptr<Node> node;
    std::thread can_thread;
    std::unique_ptr<uavcan::Timer> publish_timer;

    using WheelEncoderPub = uavcan::Publisher<cvra::odometry::WheelEncoder>;
    std::unique_ptr<WheelEncoderPub> encoder_pub;

    absl::Mutex lock;

    int left_encoder;
    int right_encoder;

public:
    WheelEncoderEmulator(std::string can_iface, std::string board_name, int node_number);
    void start();
    void set_encoders(int left, int right);

private:
    void spin();
};

#endif
