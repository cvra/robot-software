#include <ch.h>
#include <hal.h>
#include <error/error.h>
#include <version/version.h>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/protocol/NodeStatus.hpp>

#include "node.h"

#include "pressure_sensor.h"
#include "pressure_sensor_interface.h"
#include "feedback_publisher.h"
#include "Command_handler.hpp"

#define UAVCAN_SPIN_FREQ 10 // [Hz]

namespace uavcan_node {
static const int RxQueueSize = 32;
static const uint32_t BitRate = 1000000;

static bool node_ok = true;

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

void main(unsigned int id, const char* name)
{
    chRegSetThreadName("uavcan");

    NOTICE("starting UAVCAN thread");

    Node& node = getNode();

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
        ERROR("node start");
    }

    if (Command_handler_start(node) < 0) {
        ERROR("Command_handler_start");
    }

    node.getNodeStatusProvider().setModeOperational();

    bool sensor_ok = true;
    while (true) {
        if (node_ok && sensor_ok) {
            node.getNodeStatusProvider().setHealthOk();
        } else {
            node.getNodeStatusProvider().setHealthError();
        }

        node.spin(uavcan::MonotonicDuration::fromMSec(1000 / UAVCAN_SPIN_FREQ));

        sensor_ok = feedback_publish(node);
    }
}
} // namespace uavcan_node

struct uavcan_args {
    unsigned int node_id;
    const char* node_name;
};

static void uavcan_main(void* arg)
{
    struct uavcan_args* args = static_cast<struct uavcan_args*>(arg);
    uavcan_node::main(args->node_id, args->node_name);
}

void uavcan_start(unsigned int node_id, const char* node_name)
{
    static THD_WORKING_AREA(uavcan_wa, 2000);
    static struct uavcan_args args = {node_id, node_name};
    chThdCreateStatic(uavcan_wa, sizeof(uavcan_wa), HIGHPRIO, uavcan_main, &args);
}

void uavcan_set_node_is_ok(int ok)
{
    uavcan_node::node_ok = static_cast<bool>(ok);
}
