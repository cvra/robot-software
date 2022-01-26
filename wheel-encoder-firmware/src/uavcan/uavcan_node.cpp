#include <ch.h>
#include <hal.h>
#include <chprintf.h>

#include <uavcan/uavcan.hpp>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/protocol/SoftwareVersion.hpp>

#include <version/version.h>

#include <main.h>
#include "uavcan_node.h"
#include "WheelEncoder_pub.hpp"

#define CAN_BITRATE 1000000

uavcan_stm32::CanInitHelper<128> can;

// Used to signal when the node init is complete
BSEMAPHORE_DECL(node_init_complete, true);

void uavcan_failure(const char* reason)
{
    chSysHalt(reason);
}

static int uavcan_node_start(Node& node)
{
    return node.start();
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

    uavcan::protocol::SoftwareVersion sw_version;
    sw_version.major = 1;
    sw_version.optional_field_flags = sw_version.OPTIONAL_FIELD_FLAG_VCS_COMMIT;
    sw_version.vcs_commit = software_version_short;
    node.setSoftwareVersion(sw_version);

    node.getNodeStatusProvider().setModeInitialization();

    wheel_encoder_init(node);

    /* Spin forever */
    while (true) {
        int res = node.spin(uavcan::MonotonicDuration::fromMSec(1000 / UAVCAN_SPIN_FREQUENCY));

        if (res < 0) {
            uavcan_failure("UAVCAN spin");
        }

        if (chBSemWaitTimeout(&node_init_complete, TIME_IMMEDIATE) == MSG_OK) {
            node.getNodeStatusProvider().setModeOperational();
            node.getNodeStatusProvider().setHealthOk();
        }
    }
}

extern "C" void uavcan_node_start(void* arg)
{
    chThdCreateStatic(uavcan_node_wa, sizeof(uavcan_node_wa), NORMALPRIO, uavcan_node, arg);
}

extern "C" void uavcan_init_complete(void)
{
    chBSemSignal(&node_init_complete);
}
