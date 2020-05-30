//#include <unistd.h>
//#include <ch.h>
//#include <hal.h>
#include <string>
#include <thread>
#include <uavcan_linux/uavcan_linux.hpp>
//#include <uavcan/protocol/NodeStatus.hpp>
#include <uavcan/protocol/node_info_retriever.hpp>
//#include "emergency_stop_handler.hpp"
//#include "motor_feedback_streams_handler.hpp"
//#include "beacon_signal_handler.hpp"
#include "motor_driver.h"
#include "motor_driver_uavcan.hpp"
#include "wheel_encoders_handler.hpp"
//#include "can_io_driver.h"
//#include "sensor_handler.h"
//#include "uwb_position_handler.h"
//#include "can_uwb_ip_netif.hpp"
//#include "electron_starter.hpp"
//#include "config.h"
#include <can/uavcan_node.h>
//#include "priorities.h"
#include "control_panel.h"
//#include "main.h"
//#include <timestamp/timestamp.h>

#include <error/error.h>

#define UAVCAN_SPIN_FREQ 500 // [Hz]

#define UAVCAN_NODE_STACK_SIZE 8192

#define UAVCAN_MEMORY_POOL_SIZE 65536
#define UAVCAN_RX_QUEUE_SIZE 256
#define UAVCAN_CAN_BITRATE 1000000UL

bus_enumerator_t bus_enumerator;

namespace uavcan_node {

static void node_status_cb(const uavcan::ReceivedDataStructure<uavcan::protocol::NodeStatus>& msg);

/** This class is used by libuavcan to connect node information to our bus
 * enumerator. */
class BusEnumeratorNodeInfoAdapter final : public uavcan::INodeInfoListener {
    void handleNodeInfoRetrieved(uavcan::NodeID node_id,
                                 const uavcan::protocol::GetNodeInfo::Response& node_info) override
    {
        uint8_t can_id = node_id.get();
        NOTICE("Discovered node \"%s\" -> %d", node_info.name.c_str(), node_id.get());
        if (bus_enumerator_get_str_id(&bus_enumerator, can_id) == NULL) {
            bus_enumerator_update_node_info(&bus_enumerator, node_info.name.c_str(), can_id);

            /* If we received as many nodes as expected, signal it using the
             * ready LED. */
            if (bus_enumerator_discovered_nodes_count(&bus_enumerator) == bus_enumerator_total_nodes_count(&bus_enumerator)) {
                NOTICE("Everything ready!");
                control_panel_set(LED_READY);
            }
        }
    }

    void handleNodeInfoUnavailable(uavcan::NodeID node_id) override
    {
        WARNING("Could not obtain node information for node %d", node_id.get());
    }
};

uavcan::ISystemClock& getSystemClock()
{
    static uavcan_linux::SystemClock clock;
    return clock;
}

uavcan::ICanDriver& getCanDriver(std::string iface)
{
    static uavcan_linux::SocketCanDriver driver(dynamic_cast<const uavcan_linux::SystemClock&>(getSystemClock()));
    if (driver.getNumIfaces() == 0) // Will be executed once
    {
        if (driver.addIface(iface) < 0) {
            ERROR("Failed to add iface '%s'", iface.c_str());
        }
    }
    return driver;
}

static void main(std::string can_iface, uint8_t id)
{
    int res;

    uavcan::Node<UAVCAN_MEMORY_POOL_SIZE> node(getCanDriver(can_iface),
                                               getSystemClock());

    node.setNodeID(uavcan::NodeID(id));
    node.setName("cvra.master");

    uavcan::protocol::SoftwareVersion sw_version;
    sw_version.major = 1;
    node.setSoftwareVersion(sw_version);

    uavcan::protocol::HardwareVersion hw_version;
    hw_version.major = 1;
    node.setHardwareVersion(hw_version);

    res = node.start();
    if (res < 0) {
        ERROR("node start");
    }

    uavcan::Subscriber<uavcan::protocol::NodeStatus> ns_sub(node);
    res = ns_sub.start(node_status_cb);
    if (res < 0) {
        ERROR("NodeStatus subscribe");
    }

    res = motor_driver_uavcan_init(node);
    if (res < 0) {
        ERROR("motor driver");
    }

    res = wheel_encoder_handler_init(node);
    if (res < 0) {
        ERROR("wheel encoder");
    }

#if 0
    res = motor_feedback_stream_handler_init(node, &bus_enumerator);
    if (res < 0) {
        ERROR("motor feedback");
    }

    res = beacon_signal_handler_init(node);
    if (res < 0) {
        ERROR("beacon signal handler");
    }

    if (can_io_init(node) < 0) {
        ERROR("CAN IO driver");
    }

    if (sensor_handler_init(node, &bus_enumerator) < 0) {
        ERROR("sensor init");
    }

    if (uwb_position_handler_init(node) < 0) {
        ERROR("UWB TagPosition init");
    }

    res = emergency_stop_init(node);
    if (res != 0) {
        ERROR("Emergency stop handler");
    }
#endif

    uavcan::NodeInfoRetriever retriever(node);

    res = retriever.start();
    if (res < 0) {
        ERROR("NodeInfoRetriever");
    }

    BusEnumeratorNodeInfoAdapter collector;
    res = retriever.addListener(&collector);
    if (res < 0) {
        ERROR("BusEnumeratorAdapter");
    }

    // Mark the node as correctly initialized
    node.getNodeStatusProvider().setModeOperational();
    node.getNodeStatusProvider().setHealthOk();

    while (true) {
        res = node.spin(uavcan::MonotonicDuration::fromMSec(1000 / UAVCAN_SPIN_FREQ));
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
    DEBUG("UAVCAN node %u health", msg.getSrcNodeID().get());
}

} // namespace uavcan_node

void uavcan_node_start(std::string can_iface, uint8_t id)
{
    std::thread uavcan_thread(uavcan_node::main, can_iface, id);
    uavcan_thread.detach();
}
