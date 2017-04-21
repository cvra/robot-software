#include <unistd.h>
#include <ch.h>
#include <hal.h>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include <cvra/motor/EmergencyStop.hpp>
#include <uavcan/protocol/node_info_retriever.hpp>
#include "motor_feedback_streams_handler.hpp"
#include "beacon_signal_handler.hpp"
#include "error/error.h"
#include "motor_driver.h"
#include "motor_driver_uavcan.hpp"
#include "hand_driver.h"
#include "config.h"
#include "rocket_driver.h"
#include "uavcan_node.h"
#include "priorities.h"
#include "main.h"
#include <timestamp/timestamp.h>

#include <errno.h>


#define UAVCAN_SPIN_FREQ    500 // [Hz]

#define UAVCAN_NODE_STACK_SIZE 8192

#define UAVCAN_MEMORY_POOL_SIZE 16384
#define UAVCAN_RX_QUEUE_SIZE 64
#define UAVCAN_CAN_BITRATE 1000000UL


bus_enumerator_t bus_enumerator;


namespace uavcan_node
{

static void node_status_cb(const uavcan::ReceivedDataStructure<uavcan::protocol::NodeStatus>& msg);
static void node_fail(const char *reason);

uavcan::ISystemClock& getSystemClock()
{
    return uavcan_stm32::SystemClock::instance();
}

uavcan::ICanDriver& getCanDriver()
{
    static uavcan_stm32::CanInitHelper<UAVCAN_RX_QUEUE_SIZE> can;
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        int res = can.init(UAVCAN_CAN_BITRATE);
        if (res < 0) {
            node_fail("CAN driver");
        }
    }
    return can.driver;
}



/** This class is used by libuavcan to connect node information to our bus
 * enumerator. */
class BusEnumeratorNodeInfoAdapter final : public uavcan::INodeInfoListener
{
    void handleNodeInfoRetrieved(uavcan::NodeID node_id,
                                 const uavcan::protocol::GetNodeInfo::Response& node_info) override
    {
        uint8_t can_id = node_id.get();
        NOTICE("Discovered node \"%s\" -> %d", node_info.name.c_str(), node_id.get());
        if (bus_enumerator_get_str_id(&bus_enumerator, can_id) == NULL) {
            bus_enumerator_update_node_info(&bus_enumerator, node_info.name.c_str(), can_id);

            /* Signal that we received one answer. */
            palSetPad(GPIOF, GPIOF_LED_READY);
        }
    }

    void handleNodeInfoUnavailable(uavcan::NodeID node_id) override
    {
        WARNING("Could not obtain node information for node %d", node_id.get());
    }
};

THD_WORKING_AREA(thread_wa, UAVCAN_NODE_STACK_SIZE);

void main(void *arg)
{
    chRegSetThreadName("uavcan");

    static uavcan::Node<UAVCAN_MEMORY_POOL_SIZE> node(getCanDriver(), getSystemClock());

    uint8_t id = *(uint8_t *)arg;
    node.setNodeID(uavcan::NodeID(id));

    node.setName("cvra.master");

    uavcan::protocol::SoftwareVersion sw_version;
    sw_version.major = 1;
    node.setSoftwareVersion(sw_version);

    uavcan::protocol::HardwareVersion hw_version;
    hw_version.major = 1;
    node.setHardwareVersion(hw_version);

    int res;
    res = node.start();
    if (res < 0) {
        node_fail("node start");
    }

    /*
     * NodeStatus subscriber
     */
    uavcan::Subscriber<uavcan::protocol::NodeStatus> ns_sub(node);
    res = ns_sub.start(node_status_cb);
    if (res < 0) {
        node_fail("NodeStatus subscribe");
    }

    res = motor_feedback_stream_handler_init(node, &bus_enumerator);
    if (res < 0) {
        node_fail("motor feedback");
    }

    res = beacon_signal_handler_init(node);
    if (res < 0) {
        node_fail("beacon signal handler");
    }

    res = motor_driver_uavcan_init(node);
    if (res < 0) {
        node_fail("motor driver");
    }

    res = hand_driver_init(node);
    if (res < 0) {
        node_fail("hand driver");
    }

    uavcan::Subscriber<cvra::motor::EmergencyStop> emergency_stop_sub(node);
    res = emergency_stop_sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::EmergencyStop>& msg)
        {
            (void)msg;
            NVIC_SystemReset();
        }
    );
    if (res != 0) {
        node_fail("cvra::motor::EmergencyStop subscriber");
    }

    static uavcan::NodeInfoRetriever retriever(node);

    res = retriever.start();
    if (res < 0) {
        node_fail("NodeInfoRetriever");
    }

    /*
     * This class is defined above in this file.
     */
    static BusEnumeratorNodeInfoAdapter collector;
    res = retriever.addListener(&collector);
    if (res < 0) {
        node_fail("BusEnumeratorAdapter");
    }

    res = rocket_init(node);
    if (res < 0) {
        node_fail("Rocket driver");
    }

    // Mark the node as correctly initialized
    node.getNodeStatusProvider().setModeOperational();
    node.getNodeStatusProvider().setHealthOk();

    while (true)
    {
        res = node.spin(uavcan::MonotonicDuration::fromMSec(1000/UAVCAN_SPIN_FREQ));
        if (res < 0) {
            WARNING("UAVCAN spin warning %d", res);
        }
    }
}

static void node_status_cb(const uavcan::ReceivedDataStructure<uavcan::protocol::NodeStatus>& msg)
{
    if (msg.health != uavcan::protocol::NodeStatus::HEALTH_OK) {
        WARNING("UAVCAN node %u health", msg.getSrcNodeID().get());
    }
}

static void node_fail(const char *reason)
{
    (void) reason;
    ERROR("UAVCAN error: %s", reason);
    chSysHalt(reason);
}

} // namespace uavcan_node

extern "C" {

void uavcan_node_start(uint8_t id)
{
    static uint8_t node_id = id;
    chThdCreateStatic(uavcan_node::thread_wa, UAVCAN_NODE_STACK_SIZE, UAVCAN_PRIO, uavcan_node::main, &node_id);
}

} // extern "C"
