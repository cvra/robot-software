#include <unistd.h>
#include <ch.h>
#include <hal.h>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include <cvra/motor/EmergencyStop.hpp>
#include <uavcan/protocol/node_info_retriever.hpp>
#include "motor_feedback_streams_handler.hpp"
#include "beacon_signal_handler.hpp"
#include <cvra/Reboot.hpp>
#include <msgbus/messagebus.h>
#include "error/error.h"
#include "motor_driver.h"
#include "motor_driver_uavcan.h"
#include "config.h"
#include "uavcan_node_private.hpp"
#include "rocket_driver.h"
#include "uavcan_node.h"
#include "priorities.h"
#include "main.h"
#include <timestamp/timestamp.h>

#include <errno.h>


#define UAVCAN_SPIN_FREQ    500 // [Hz]

#define UAVCAN_NODE_STACK_SIZE 8192


bus_enumerator_t bus_enumerator;


namespace uavcan_node
{

static void node_status_cb(const uavcan::ReceivedDataStructure<uavcan::protocol::NodeStatus>& msg);
static void node_fail(const char *reason);


static constexpr int RxQueueSize = 64;
static constexpr std::uint32_t BitRate = 1000000;

constexpr unsigned NodeMemoryPoolSize = 16384;
typedef uavcan::Node<NodeMemoryPoolSize> Node;

uavcan::ISystemClock& getSystemClock()
{
    return uavcan_stm32::SystemClock::instance();
}

uavcan::ICanDriver& getCanDriver()
{
    static uavcan_stm32::CanInitHelper<RxQueueSize> can;
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        int res = can.init(BitRate);
        if (res < 0) {
            node_fail("CAN driver");
        }
    }
    return can.driver;
}

Node& getNode()
{
    static Node node(getCanDriver(), getSystemClock());
    return node;
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

    Node& node = getNode();

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


    uavcan::Publisher<cvra::Reboot> reboot_pub(node);
    const int reboot_pub_init_res = reboot_pub.init();
    if (reboot_pub_init_res < 0) {
        node_fail("cvra::Reboot publisher");
    }

    while (true)
    {
        res = node.spin(uavcan::MonotonicDuration::fromMSec(1000/UAVCAN_SPIN_FREQ));
        if (res < 0) {
            // log warning
        }

        // reboot command
        if (board_button_pressed()) {
            cvra::Reboot reboot_msg;
            reboot_msg.bootmode = reboot_msg.BOOTLOADER_TIMEOUT;
            reboot_pub.broadcast(reboot_msg);
        }

        motor_driver_t *drv_list;
        uint16_t drv_list_len;
        motor_manager_get_list(&motor_manager, &drv_list, &drv_list_len);
        int i;
        for (i = 0; i < drv_list_len; i++) {
            motor_driver_uavcan_update_config(&drv_list[i]);
            motor_driver_uavcan_send_setpoint(&drv_list[i]);
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
