#pragma once
#include <absl/synchronization/mutex.h>
#include <memory>
#include <thread>
#include <uavcan_linux/uavcan_linux.hpp>
#include "uavcan_node.h"
#include <cvra/actuator/Feedback.hpp>

class ActuatorBoardEmulator {
    uavcan_linux::SystemClock clock;
    uavcan_linux::SocketCanDriver driver;
    std::unique_ptr<Node> node;
    std::thread can_thread;
    std::unique_ptr<uavcan::Timer> publish_timer;

    using ActuatorFeedbackPub = uavcan::Publisher<cvra::actuator::Feedback>;
    std::unique_ptr<ActuatorFeedbackPub> feedback_pub;

    absl::Mutex lock;

    int pressure_pa[2] GUARDED_BY(lock);
    bool digital_input GUARDED_BY(lock);

public:
    ActuatorBoardEmulator(std::string can_iface, std::string board_name, int node_number);
    void start();
    void set_pressure(float pressure[2])
    {
        absl::MutexLock _(&lock);
        pressure_pa[0] = pressure[0];
        pressure_pa[1] = pressure[1];
    }
    void set_digital_input(bool val)
    {
        absl::MutexLock _(&lock);
        digital_input = val;
    }

private:
    void spin();
};
