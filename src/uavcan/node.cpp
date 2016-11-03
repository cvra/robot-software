#include <ch.h>
#include <hal.h>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/protocol/NodeStatus.hpp>

#include "node.h"

#define UAVCAN_SPIN_FREQ    500 // [Hz]

namespace uavcan_node {
static const int RxQueueSize = 32;
static const uint32_t BitRate = 1000000;

const unsigned NodeMemoryPoolSize = 4096;
typedef uavcan::Node<NodeMemoryPoolSize> Node;


static uavcan::ISystemClock& getSystemClock()
{
    return uavcan_stm32::SystemClock::instance();
}

static uavcan::ICanDriver& getCanDriver()
{
    static uavcan_stm32::CanInitHelper<RxQueueSize> can;
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        int res = can.init(BitRate);
        if (res < 0) {
            chSysHalt("CAN driver");
        }
    }
    return can.driver;
}

static Node& getNode()
{
    static Node node(getCanDriver(), getSystemClock());
    return node;
}

void main(unsigned int id, const char *name)
{
    chRegSetThreadName("uavcan");

    Node& node = getNode();

    node.setNodeID(uavcan::NodeID(id));

    uavcan::protocol::SoftwareVersion sw_version;
    sw_version.major = 1;
    node.setSoftwareVersion(sw_version);

    uavcan::protocol::HardwareVersion hw_version;
    hw_version.major = 1;
    node.setHardwareVersion(hw_version);

    node.setName(name);

    if (node.start() < 0) {
        chSysHalt("node start");
    }

    node.getNodeStatusProvider().setModeOperational();
    node.getNodeStatusProvider().setHealthOk();


    while (true) {
        node.spin(uavcan::MonotonicDuration::fromMSec(1000/UAVCAN_SPIN_FREQ));
    }
}
}

void uavcan_start(unsigned int node_id, const char *node_name)
{
    uavcan_node::main(node_id, node_name);
}

