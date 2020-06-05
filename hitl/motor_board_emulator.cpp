#include "motor_board_emulator.h"
#include <thread>
#include <error/error.h>

UavcanMotorEmulator::UavcanMotorEmulator(std::string can_iface, std::string board_name, int node_number)
    : driver(clock)
    , voltage(0.f)
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

    voltage_sub = std::make_unique<UavcanMotorEmulator::VoltageSub>(*node);
    voltage_sub->start([&](const uavcan::ReceivedDataStructure<cvra::motor::control::Voltage>& msg) {
        if (msg.node_id != node->getNodeID().get()) {
            DEBUG("dropping message. msg.node_id is %d, ours is %d", msg.node_id, node->getNodeID().get());
            return;
        }

        DEBUG("Received voltage setpoint %.2f for board %d", msg.voltage, msg.node_id);

        {
            absl::MutexLock _(&lock);
            voltage = msg.voltage;
        }
    });
}

void UavcanMotorEmulator::start()
{
    std::thread new_thread(&UavcanMotorEmulator::spin, this);
    std::swap(new_thread, can_thread);
}

float UavcanMotorEmulator::get_voltage()
{
    absl::MutexLock _(&lock);
    return voltage;
}

void UavcanMotorEmulator::spin()
{
    node->start();
    while (true) {
        const int res = node->spin(uavcan::MonotonicDuration::fromMSec(1000));
        if (res < 0) {
            WARNING("UAVCAN failure: %d", res);
        }
    }
}
