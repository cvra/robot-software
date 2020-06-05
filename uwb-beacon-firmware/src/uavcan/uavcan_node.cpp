#include <ch.h>
#include <hal.h>
#include <chprintf.h>

#include <uavcan/uavcan.hpp>
#include <uavcan_stm32/uavcan_stm32.hpp>

#include <main.h>
#include "./uavcan/uavcan_node.h"
#include "./uavcan/topics_publisher.hpp"
#include "./uavcan/parameter_server.hpp"
#include "./uavcan/restart_server.hpp"
#include "./uavcan/position_handler.hpp"
#include "./uavcan/data_packet_handler.hpp"

#define CAN_BITRATE 1000000

uavcan_stm32::CanInitHelper<128> can;

struct uavcan_node_arg {
    const char* node_name;
    uint8_t node_id : 7;
};

void uavcan_failure(const char* reason)
{
    chSysHalt(reason);
}

static THD_WORKING_AREA(uavcan_node_wa, 8000);
static THD_FUNCTION(uavcan_node, arg)
{
    struct uavcan_node_arg* node_arg;
    node_arg = (struct uavcan_node_arg*)arg;

    chRegSetThreadName(__FUNCTION__);

    /* Create the CAN interface driver. */
    if (can.init((uavcan::uint32_t)CAN_BITRATE) != 0) {
        uavcan_failure("CAN driver");
    }

    /* Create the UAVCAN instance. */
    static Node node(can.driver, uavcan_stm32::SystemClock::instance());

    /* Give it basic properties. */
    node.setNodeID(node_arg->node_id);
    node.setName(node_arg->node_name);

    node.start();

    // Mark the node as correctly initialized
    node.getNodeStatusProvider().setModeOperational();
    node.getNodeStatusProvider().setHealthOk();

    /* TODO: set software and hardware version */

    topics_publisher_start(node);
    parameter_server_start(node);
    restart_server_start(node);
    position_handler_init(node);
    data_packet_handler_init(node);

    /* Spin forever */
    while (true) {
        auto res = node.spin(uavcan::MonotonicDuration::fromMSec(1000 / UAVCAN_SPIN_FREQUENCY));

        if (res < 0) {
            uavcan_failure("UAVCAN spin");
        }

        topics_publisher_spin(node);
    }
}

extern "C" void uavcan_node_start(uint8_t id, const char* name)
{
    static struct uavcan_node_arg arg;
    arg.node_name = name;
    arg.node_id = id;
    chThdCreateStatic(uavcan_node_wa, sizeof(uavcan_node_wa), NORMALPRIO, uavcan_node, &arg);
}
