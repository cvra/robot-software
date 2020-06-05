#ifndef MOTOR_BOARD_EMULATOR_H
#define MOTOR_BOARD_EMULATOR_H

#include <thread>
#include <memory>
#include <absl/synchronization/mutex.h>
#include <cvra/motor/control/Voltage.hpp>
#include <uavcan_linux/uavcan_linux.hpp>
#include "uavcan_node.h"

class UavcanMotorEmulator {
    uavcan_linux::SystemClock clock;
    uavcan_linux::SocketCanDriver driver;
    std::unique_ptr<Node> node;
    std::thread can_thread;

    using VoltageSub = uavcan::Subscriber<cvra::motor::control::Voltage>;
    std::unique_ptr<VoltageSub> voltage_sub;
    float voltage;

    absl::Mutex lock;

public:
    UavcanMotorEmulator(std::string can_iface, std::string board_name, int node_number);
    void start();
    float get_voltage();

private:
    void spin();
};

#endif
