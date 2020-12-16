#include <iostream>
#include "servo_board_emulator.h"
#include <thread>
#include <error/error.h>

ServoBoardEmulator::ServoBoardEmulator(std::string can_iface, std::string board_name, int node_number)
    : driver(clock)
{
    servo_pos[0] = 0.f;
    servo_pos[1] = 0.f;
    servo_pos[2] = 0.f;
    servo_pos[3] = 0.f;

    NOTICE("CAN IO (servo) board emulator on %s", can_iface.c_str());

    std::cerr << "board on " << node_number << std::endl;

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

    command_sub = std::make_unique<ServoBoardEmulator::CommandSub>(*node);
    command_sub->start([&](const uavcan::ReceivedDataStructure<cvra::io::ServoPWM>& msg) {
        if (msg.node_id != node->getNodeID().get()) {
            std::cerr << "droppint msg" << std::endl;
            DEBUG("dropping message. msg.node_id is %d, ours is %d", msg.node_id, node->getNodeID().get());
            return;
        }

        {
            absl::MutexLock _(&lock);
            for (int i = 0; i < 4; i++) {
                servo_pos[i] = msg.servo_pos[i];
            }
        }
    });
}

void ServoBoardEmulator::start()
{
    std::thread new_thread(&ServoBoardEmulator::spin, this);
    std::swap(new_thread, can_thread);
}

void ServoBoardEmulator::spin()
{
    node->start();
    while (true) {
        const int res = node->spin(uavcan::MonotonicDuration::fromMSec(1000));
        if (res < 0) {
            WARNING("UAVCAN failure: %d", res);
        }
    }
}
