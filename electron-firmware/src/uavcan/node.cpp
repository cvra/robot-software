#include <ch.h>
#include <hal.h>
#include <error/error.h>
#include <version/version.h>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include <uavcan/protocol/node_info_retriever.hpp>
#include "data_packet_handler.hpp"
#include "MotorVoltage_pub.hpp"

#include "node.h"

#define UAVCAN_SPIN_FREQ 10 // [Hz]
#define UAVCAN_RX_QUEUE_SIZE    32
#define UAVCAN_CAN_BITRATE      1000000UL
// #define UAVCAN_MEMORY_POOL_SIZE 4096
#define UAVCAN_MEMORY_POOL_SIZE 3500
#define MOTOR_VOLTAGE_FORWARD_SIGN 1.0f

namespace uavcan_node {

using Node = uavcan::Node<UAVCAN_MEMORY_POOL_SIZE>;
using CanInterface = uavcan_stm32::CanInitHelper<UAVCAN_RX_QUEUE_SIZE>;
using SystemClock = uavcan_stm32::SystemClock;

static float motor_voltage = 0.0f;

class MotorInfo final : public uavcan::INodeInfoListener {
public:
    bool found = false;
    uint8_t can_id = 0;

    void handleNodeInfoRetrieved(uavcan::NodeID node_id,
                                 const uavcan::protocol::GetNodeInfo::Response& node_info) override
    {
        if (!strcmp(node_info.name.c_str(), "electron-accelerator")) {
            NOTICE("Discovered node \"%s\" -> %d", node_info.name.c_str(), node_id.get());
            can_id = node_id.get();
            found = true;
        }
    }

    void handleNodeInfoUnavailable(uavcan::NodeID node_id) override
    {
        WARNING("Could not obtain node information for node %d", node_id.get());
    }
};

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

    // A bit overkill but works
    static uavcan::NodeInfoRetriever retriever(node);
    res = retriever.start();
    if (res < 0) {
        ERROR("NodeInfoRetriever");
    }

    /*
     * This class is defined above in this file.
     */
    static MotorInfo motor_info;
    res = retriever.addListener(&motor_info);
    if (res < 0) {
        ERROR("MotorInfo");
    }

    data_packet_handler_init(node);

    node.getNodeStatusProvider().setModeOperational();
    node.getNodeStatusProvider().setHealthOk();

    while (true) {
        node.spin(uavcan::MonotonicDuration::fromMSec(1000 / UAVCAN_SPIN_FREQ));

        if (motor_info.found) {
            motor_voltage_publish(node, motor_info.can_id, motor_voltage);
        }
    }
}
} // namespace uavcan_node

void motor_voltage_set(float voltage) {
    uavcan_node::motor_voltage = voltage * MOTOR_VOLTAGE_FORWARD_SIGN;
}

void uavcan_start(unsigned int node_id, const char* node_name)
{
    uavcan_node::main(node_id, node_name);
}
