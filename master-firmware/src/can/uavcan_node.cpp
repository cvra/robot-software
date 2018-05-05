#include <unistd.h>
#include <ch.h>
#include <hal.h>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include <uavcan/protocol/node_info_retriever.hpp>
#include "emergency_stop_handler.hpp"
#include "motor_feedback_streams_handler.hpp"
#include "beacon_signal_handler.hpp"
#include "error/error.h"
#include "motor_driver.h"
#include "motor_driver_uavcan.hpp"
#include "can_io_driver.h"
#include "sensor_handler.h"
#include "uwb_position_handler.h"
#include "config.h"
#include "uavcan_node.h"
#include "priorities.h"
#include "control_panel.h"
#include "main.h"
#include "base/base_controller.h"
#include <timestamp/timestamp.h>

#include <errno.h>
#include <cvra/master/feedback/DistancePID.hpp>
#include <cvra/master/feedback/AnglePID.hpp>


#define UAVCAN_SPIN_FREQ    500 // [Hz]

#define UAVCAN_NODE_STACK_SIZE 8192

#define UAVCAN_MEMORY_POOL_SIZE 16384
#define UAVCAN_RX_QUEUE_SIZE 64
#define UAVCAN_CAN_BITRATE 1000000UL


bus_enumerator_t bus_enumerator;

uavcan::LazyConstructor<uavcan::Publisher<cvra::master::feedback::DistancePID> > distance_pid_pub;
uavcan::LazyConstructor<uavcan::Publisher<cvra::master::feedback::AnglePID> > angle_pid_pub;

namespace uavcan_node
{

static void node_status_cb(const uavcan::ReceivedDataStructure<uavcan::protocol::NodeStatus>& msg);

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
            control_panel_set(LED_DEBUG);

            /* If we received as many nodes as expected, signal it using the
             * ready LED. */
            if (bus_enumerator_discovered_nodes_count(&bus_enumerator) ==
                bus_enumerator_total_nodes_count(&bus_enumerator)) {
                control_panel_set(LED_READY);
            }
        }
    }

    void handleNodeInfoUnavailable(uavcan::NodeID node_id) override
    {
        WARNING("Could not obtain node information for node %d", node_id.get());
    }
};


static void main(void *arg)
{
    chRegSetThreadName("uavcan");
    int res;
    unsigned int id = (unsigned int)arg;

    /* Inits the hardware CAN interface. */
    static uavcan_stm32::CanInitHelper<UAVCAN_RX_QUEUE_SIZE> can_interface;
    res = can_interface.init(UAVCAN_CAN_BITRATE);

    if (res < 0) {
        ERROR("Hardware interface init");
    }

    using Node = uavcan::Node<UAVCAN_MEMORY_POOL_SIZE>;
    static Node node(can_interface.driver, uavcan_stm32::SystemClock::instance());

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

    res = motor_feedback_stream_handler_init(node, &bus_enumerator);
    if (res < 0) {
        ERROR("motor feedback");
    }

    res = beacon_signal_handler_init(node);
    if (res < 0) {
        ERROR("beacon signal handler");
    }

    res = motor_driver_uavcan_init(node);
    if (res < 0) {
        ERROR("motor driver");
    }

    if (can_io_init(node) < 0) {
        ERROR("CAN IO driver");
    }

    if (sensor_handler_init(node) < 0) {
        ERROR("sensor init");
    }

    if (uwb_position_handler_init(node) < 0) {
        ERROR("UWB TagPosition init");
    }

    res = emergency_stop_init(node);
    if (res != 0) {
        ERROR("Emergency stop handler");
    }

    distance_pid_pub.construct<Node&>(node);
    res = distance_pid_pub->init();
    if (res < 0) {
        ERROR("DistancePID publisher");
    }
    angle_pid_pub.construct<Node&>(node);
    res = angle_pid_pub->init();
    if (res < 0) {
        ERROR("AnglePID publisher");
    }

    uavcan::Timer pid_feedback_timer(node);
    pid_feedback_timer.setCallback([&](const uavcan::TimerEvent& event)
        {
            (void)event;

            cvra::master::feedback::DistancePID distance_pid;
            distance_pid.setpoint = (robot.distance_cs.filtered_consign_value) / robot.distance_pid.divider;
            distance_pid.measured = (robot.distance_cs.filtered_consign_value - robot.distance_cs.error_value) / robot.distance_pid.divider;

            cvra::master::feedback::AnglePID angle_pid;
            angle_pid.setpoint = (robot.angle_cs.filtered_consign_value) / robot.angle_pid.divider;
            angle_pid.measured = (robot.angle_cs.filtered_consign_value - robot.angle_cs.error_value) / robot.angle_pid.divider;

            distance_pid_pub->broadcast(distance_pid);
            angle_pid_pub->broadcast(angle_pid);
        });
    pid_feedback_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(50));

    static uavcan::NodeInfoRetriever retriever(node);

    res = retriever.start();
    if (res < 0) {
        ERROR("NodeInfoRetriever");
    }

    /*
     * This class is defined above in this file.
     */
    static BusEnumeratorNodeInfoAdapter collector;
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
}

} // namespace uavcan_node

extern "C" {

void uavcan_node_start(uint8_t id)
{
    unsigned int node_id = id;
    static THD_WORKING_AREA(thread_wa, UAVCAN_NODE_STACK_SIZE);
    chThdCreateStatic(thread_wa,
                      UAVCAN_NODE_STACK_SIZE,
                      UAVCAN_PRIO,
                      uavcan_node::main,
                      (void *)node_id);
}

} // extern "C"
