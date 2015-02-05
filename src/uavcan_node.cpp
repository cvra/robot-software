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

static THD_WORKING_AREA(uavcan_node_wa, 1000);
static THD_FUNCTION(uavcan_node, arg)
{
    if (can.init(CAN_BITRATE) != 0) {
        return 0;
    }

    Node& node = get_node();

    node.setNodeID(NODE_ID);
    node.setName(NODE_NAME);

    if (node.start() != 0) {
        return 0;
    }

    node.setStatusOk();

    while (true) {
        int res = node.spin(uavcan::MonotonicDuration::fromMSec(1000));

        if (res < 0) {
            // std::printf("Spin failure: %i\n", spin_res);
        }
    }
}

extern "C"
void uavcan_node_start(void *arg)
{
    chThdCreateStatic(uavcan_node_wa, sizeof(uavcan_node_wa), NORMALPRIO, uavcan_node, arg);
}
