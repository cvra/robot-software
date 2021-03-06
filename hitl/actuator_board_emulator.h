#pragma once
#include <absl/synchronization/mutex.h>
#include <memory>
#include <thread>
#include <uavcan_linux/uavcan_linux.hpp>
#include "uavcan_node.h"
#include <cvra/actuator/Feedback.hpp>
#include <cvra/actuator/Command.hpp>

class ActuatorBoardEmulator {
    uavcan_linux::SystemClock clock;
    uavcan_linux::SocketCanDriver driver;
    std::unique_ptr<Node> node;
    std::thread can_thread;
    std::unique_ptr<uavcan::Timer> publish_timer;

    using ActuatorFeedbackPub = uavcan::Publisher<cvra::actuator::Feedback>;
    std::unique_ptr<ActuatorFeedbackPub> feedback_pub;

    using CommandSub = uavcan::Subscriber<cvra::actuator::Command>;
    std::unique_ptr<CommandSub> command_sub;

    absl::Mutex lock;

    int pressure_pa[2] GUARDED_BY(lock);
    bool digital_input GUARDED_BY(lock);

    float pump_pwm[2] GUARDED_BY(lock);
    bool solenoid[2] GUARDED_BY(lock);
    float servo_pos[2] GUARDED_BY(lock);

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

    float get_pwm(int index)
    {
        absl::ReaderMutexLock _(&lock);
        return pump_pwm[index];
    }

    bool get_solenoid(int index)
    {
        absl::ReaderMutexLock _(&lock);
        return solenoid[index];
    }

    float get_servo_pos(int index)
    {
        absl::ReaderMutexLock _(&lock);
        return servo_pos[index];
    }

private:
    void spin();
};
