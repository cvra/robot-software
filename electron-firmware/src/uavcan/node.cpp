#include <ch.h>
#include <hal.h>
#include <error/error.h>
#include <version/version.h>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include "data_packet_handler.hpp"
#include "motor_manager.hpp"

#include "node.h"

#define UAVCAN_SPIN_FREQ 10 // [Hz]
#define UAVCAN_RX_QUEUE_SIZE    32
#define UAVCAN_CAN_BITRATE      1000000UL
#define UAVCAN_MEMORY_POOL_SIZE 4096

namespace uavcan_node {

using Node = uavcan::Node<UAVCAN_MEMORY_POOL_SIZE>;
using CanInterface = uavcan_stm32::CanInitHelper<UAVCAN_RX_QUEUE_SIZE>;
using SystemClock = uavcan_stm32::SystemClock;

void main(unsigned int id, const char* name)
{
    int res;

    chRegSetThreadName("uavcan");

    static CanInterface can;
    res = can.init(UAVCAN_CAN_BITRATE);
    if (res < 0) {
        chSysHalt("CAN driver init");
    }

    static Node node(can.driver, SystemClock::instance());
    node.setNodeID(uavcan::NodeID(id));

    uavcan::protocol::SoftwareVersion sw_version;
    sw_version.major = 1;
    sw_version.optional_field_flags = sw_version.OPTIONAL_FIELD_FLAG_VCS_COMMIT;
    sw_version.vcs_commit = software_version_short;
    node.setSoftwareVersion(sw_version);

    uavcan::protocol::HardwareVersion hw_version;
    hw_version.major = 1;
    node.setHardwareVersion(hw_version);

    node.setName(name);

    if (node.start() < 0) {
        chSysHalt("node start");
    }

    res = motor_manager_init(node);
    if (res < 0) {
        ERROR("Motor manager init");
    }

    data_packet_handler_init(node);

    node.getNodeStatusProvider().setModeOperational();
    node.getNodeStatusProvider().setHealthOk();

    while (true) {
        node.spin(uavcan::MonotonicDuration::fromMSec(1000 / UAVCAN_SPIN_FREQ));

        if (!palReadPad(GPIOA, GPIOA_PIN0)) {
            motor_voltage_set(6.0f);
        } else if (!palReadPad(GPIOA, GPIOA_PIN1)) {
            motor_voltage_set(-6.0f);
        } else {
            motor_voltage_set(0.0f);
        }

        motor_voltage_publish(node);
    }
}
} // namespace uavcan_node

void uavcan_start(unsigned int node_id, const char* node_name)
{
    uavcan_node::main(node_id, node_name);
}
