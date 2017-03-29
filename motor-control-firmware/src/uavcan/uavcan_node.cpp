#include <ch.h>
#include <hal.h>
#include <chprintf.h>

#include <uavcan/uavcan.hpp>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/protocol/NodeStatus.hpp>

#include <main.h>
#include "uavcan_node.h"
#include "parameter_server.hpp"
#include "uavcan_streams.hpp"

#include "Reboot_handler.hpp"
#include "EmergencyStop_handler.hpp"
#include "Trajectory_handler.hpp"
#include "Velocity_handler.hpp"
#include "Position_handler.hpp"
#include "Torque_handler.hpp"
#include "Voltage_handler.hpp"
#include "LoadConfiguration_server.hpp"
#include "EnableMotor_server.hpp"
#include "CurrentPID_server.hpp"
#include "VelocityPID_server.hpp"
#include "PositionPID_server.hpp"
#include "TorqueLimit_server.hpp"
#include "stream.h"

#define CAN_BITRATE             1000000

uavcan_stm32::CanInitHelper<128> can;


void uavcan_failure(const char *reason)
{
    chSysHalt(reason);
}

static int uavcan_node_start(Node &node)
{
    return node.start();
}

/** Start all UAVCAN services. */
static void uavcan_services_start(Node &node)
{
    const struct {
        int (*start)(Node &);
        const char *name;
    } services[] = {
        {uavcan_node_start, "Node start"},
        {Reboot_handler_start, "Reboot subscriber"},
        {EmergencyStop_handler_start, "Emergency stop subscriber"},
        {Trajectory_handler_start, "cvra::motor::control::Trajectory subscriber"},
        {Velocity_handler_start, "cvra::motor::control::Velocity subscriber"},
        {Position_handler_start, "cvra::motor::control::Position subscriber"},
        {Torque_handler_start, "cvra::motor::control::Torque subscriber"},
        {Voltage_handler_start, "cvra::motor::control::Voltage subscriber"},
        {LoadConfiguration_server_start, "cvra::motor::config::LoadConfiguration server"},
        {CurrentPID_server_start, "cvra::motor::config::CurrentPID server"},
        {VelocityPID_server_start, "cvra::motor::config::VelocityPID server"},
        {PositionPID_server_start, "cvra::motor::config::PositionPID server"},
        {TorqueLimit_server_start, "cvra::motor::config::TorqueLimit server"},
        {EnableMotor_server_start, "cvra::motor::config::EnableMotor server"},
        {parameter_server_start, "UAVCAN parameter server"},
        {uavcan_streams_start, "UAVCAN state streamer"},
        {NULL, NULL} /* Must be last */
    };

    for (int i = 0; services[i].start; i++) {
        if (services[i].start(node) < 0) {
            uavcan_failure(services[i].name);
        }
    }
}

static THD_WORKING_AREA(uavcan_node_wa, 8000);
static THD_FUNCTION(uavcan_node, arg)
{
    struct uavcan_node_arg *node_arg;
    node_arg = (struct uavcan_node_arg *)arg;

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

    /* Start all the subscribers and publishers linked to that node. */
    uavcan_services_start(node);

    /* Spin forever */
    while (true) {
        int res = node.spin(uavcan::MonotonicDuration::fromMSec(1000 / UAVCAN_SPIN_FREQUENCY));

        if (res < 0) {
            uavcan_failure("UAVCAN spin");
        }

        error_uavcan_logger_spin(&node);
        uavcan_streams_spin(node);
    }
}

extern "C"
void uavcan_node_start(void *arg)
{
    chThdCreateStatic(uavcan_node_wa, sizeof(uavcan_node_wa), NORMALPRIO, uavcan_node, arg);
}
