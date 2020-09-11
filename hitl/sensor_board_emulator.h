#pragma once
#include <absl/synchronization/mutex.h>
#include <memory>
#include <thread>
#include <uavcan_linux/uavcan_linux.hpp>
#include "uavcan_node.h"
#include <cvra/sensor/DistanceVL6180X.hpp>

class SensorBoardEmulator {
    uavcan_linux::SystemClock clock;
    uavcan_linux::SocketCanDriver driver;
    std::unique_ptr<Node> node;
    std::thread can_thread;
    std::unique_ptr<uavcan::Timer> publish_timer;

    using DistanceVL6180XPub = uavcan::Publisher<cvra::sensor::DistanceVL6180X>;
    std::unique_ptr<DistanceVL6180XPub> distance_pub;

    absl::Mutex lock;

    int distance_mm GUARDED_BY(lock);

public:
    SensorBoardEmulator(std::string can_iface, std::string board_name, int node_number);
    void start();
    void set_distance(int distance_mm);

private:
    void spin();
};
