#pragma once

#include <uavcan_linux/uavcan_linux.hpp>
#include "uavcan_node.h"
#include <thread>
#include <cvra/proximity_beacon/Signal.hpp>
#include <absl/synchronization/mutex.h>
#include <box2d/box2d.h>

class ProximityBeaconEmulator {
    uavcan_linux::SystemClock clock;
    uavcan_linux::SocketCanDriver driver;
    std::unique_ptr<Node> node;
    std::thread can_thread;
    std::unique_ptr<uavcan::Timer> publish_timer;
    absl::Mutex lock;

    b2Vec2 robot_pos;
    b2Vec2 opponent_pos;
    float robot_heading;

    using Publisher = uavcan::Publisher<cvra::proximity_beacon::Signal>;
    std::unique_ptr<Publisher> pub;

    void spin();

public:
    ProximityBeaconEmulator(std::string can_iface, std::string board_name, int node_number);
    void start();

    void set_positions(b2Vec2 robot_pos_, float robot_heading_, b2Vec2 opponent_pos_)
    {
        absl::MutexLock _(&lock);
        robot_pos = robot_pos_;
        opponent_pos = opponent_pos_;
        robot_heading = robot_heading_;
    }
};
