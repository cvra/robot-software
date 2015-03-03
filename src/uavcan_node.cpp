#include <ch.h>
#include <hal.h>
#include <uavcan/uavcan.hpp>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/protocol/NodeStatus.hpp>

#define NODE_ID 42
#define NODE_NAME "motor-board"

#define CAN_BITRATE 1000000

uavcan_stm32::CanInitHelper<128> can;

typedef uavcan::Node<4096> Node;

uavcan::LazyConstructor<Node> node_;

Node& get_node()
{
    if (!node_.isConstructed()) {
        node_.construct<uavcan::ICanDriver&, uavcan::ISystemClock&>(can.driver, uavcan_stm32::SystemClock::instance());
    }
    return *node_;
}

void uavcan_failure(const char *reason)
{
    chSysHalt(reason);
}

static THD_WORKING_AREA(uavcan_node_wa, 4000);
static THD_FUNCTION(uavcan_node, arg)
{
    chRegSetThreadName("uavcan node");

    (void)arg;
    if (can.init(CAN_BITRATE) != 0) {
        uavcan_failure("CAN driver");
    }

    Node& node = get_node();

    node.setNodeID(NODE_ID);
    node.setName(NODE_NAME);

    if (node.start() != 0) {
        uavcan_failure("UAVCAN node start");
    }

    node.setStatusOk();

    while (true) {
        int res = node.spin(uavcan::MonotonicDuration::fromMSec(100));

        if (res < 0) {
            uavcan_failure("UAVCAN spin");
        }
    }
}

extern "C"
void uavcan_node_start(void *arg)
{
    chThdCreateStatic(uavcan_node_wa, sizeof(uavcan_node_wa), NORMALPRIO, uavcan_node, arg);
}
