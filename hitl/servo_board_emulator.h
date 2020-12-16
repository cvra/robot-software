#pragma once
#include <absl/synchronization/mutex.h>
#include <memory>
#include <thread>
#include <uavcan_linux/uavcan_linux.hpp>
#include "uavcan_node.h"
#include <cvra/io/ServoPWM.hpp>

class ServoBoardEmulator {
    uavcan_linux::SystemClock clock;
    uavcan_linux::SocketCanDriver driver;
    std::unique_ptr<Node> node;
    std::thread can_thread;
    std::unique_ptr<uavcan::Timer> publish_timer;

    using CommandSub = uavcan::Subscriber<cvra::io::ServoPWM>;
    std::unique_ptr<CommandSub> command_sub;

    absl::Mutex lock;

    float servo_pos[4] GUARDED_BY(lock);

public:
    ServoBoardEmulator(std::string can_iface, std::string board_name, int node_number);
    float get_servo_pos(int index)
    {
        absl::ReaderMutexLock _(&lock);
        return servo_pos[index];
    }

    void start();

private:
    void spin();
};
