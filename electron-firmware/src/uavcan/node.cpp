#include <ch.h>
#include <hal.h>
#include <error/error.h>
#include <version/version.h>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include "data_packet_handler.hpp"
#include "motor_manager.hpp"

#include "node.h"

#define UAVCAN_SPIN_FREQ 10 // [Hz]
#define UAVCAN_RX_QUEUE_SIZE    32
#define UAVCAN_CAN_BITRATE      1000000UL
#define UAVCAN_MEMORY_POOL_SIZE 4096

namespace uavcan_node {

using Node = uavcan::Node<UAVCAN_MEMORY_POOL_SIZE>;
using CanInterface = uavcan_stm32::CanInitHelper<UAVCAN_RX_QUEUE_SIZE>;
using SystemClock = uavcan_stm32::SystemClock;

static void timeout_cb(void *flag_p)
{
    *(bool*)flag_p = true;
}

static void start_timer(unsigned int timeout_sec, bool* flag_p)
{
    static virtual_timer_t timer;
    const sysinterval_t timeout = OSAL_S2I(timeout_sec);
    chVTObjectInit(&timer);
    chVTDoSetI(&timer, timeout, timeout_cb, flag_p);
}

void main(unsigned int id, const char* name)
{
    int res;

    chRegSetThreadName("uavcan");

    static CanInterface can;
    res = can.init(UAVCAN_CAN_BITRATE);
    if (res < 0) {
        chSysHalt("CAN driver init");
    }

    static Node node(can.driver, SystemClock::instance());
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
        chSysHalt("node start");
    }

    res = motor_manager_init(node);
    if (res < 0) {
        ERROR("Motor manager init");
    }

    data_packet_handler_init(node);

    node.getNodeStatusProvider().setModeOperational();
    node.getNodeStatusProvider().setHealthOk();

    bool timed_out = false;
    const unsigned int timeout = 30; // seconds
    while (true) {
        node.spin(uavcan::MonotonicDuration::fromMSec(1000 / UAVCAN_SPIN_FREQ));

        const auto voltage = 10.f;

        if (electron_state == INIT && motor_ready()) {
            electron_state = READY;
        }

        if (electron_state == READY && data_packet_start_signal_received() && !front_hall_sensor()) {
            electron_state = RUNNING;
            start_timer(timeout, &timed_out);
        }

        if (electron_state == RUNNING) {
            if (data_packet_start_signal_received() && !front_hall_sensor()) {
                motor_voltage_set(voltage);
            } else {
                motor_voltage_set(0.0f);
            }

            if (front_hall_sensor() && timed_out) {
                electron_state = ARRIVED;
            }
        }

        if (electron_state == ARRIVED) {
            motor_voltage_set(0.0f);
        }

        motor_voltage_publish(node);
    }
}
} // namespace uavcan_node

int electron_state = INIT;

void uavcan_start(unsigned int node_id, const char* node_name)
{
    uavcan_node::main(node_id, node_name);
}
