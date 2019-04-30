#include <ch.h>
#include <hal.h>
#include <error/error.h>
#include <uavcan/uavcan.hpp>
#include <cvra/motor/control/Voltage.hpp>

#include "motor_manager.hpp"

using namespace uavcan;

#define MOTOR_VOLTAGE_FORWARD_SIGN 1.0f

static bool motor_found = false;
static uint8_t motor_can_id = 0;
static float motor_voltage = 0.0f;

static ServiceClient<protocol::GetNodeInfo>* node_info_client_p = NULL;

static void node_status_cb(const ReceivedDataStructure<protocol::NodeStatus>& msg)
{
    const NodeID node_id = msg.getSrcNodeID();

    if (!motor_found && node_info_client_p) {
        int res;

        res = node_info_client_p->call(node_id, protocol::GetNodeInfo::Request());
        if (res < 0) {
            ERROR("GetNodeInfo");
        }
    }
}

static void node_info_cb(const ServiceCallResult<protocol::GetNodeInfo>& result)
{
    NodeID node_id = result.getCallID().server_node_id;

    if (!result.isSuccessful()) {
        NOTICE("GetNodeInfo failed for node %d", node_id.get());
    }

    const protocol::GetNodeInfo::Response& node_info = result.getResponse();

    NOTICE("Discovered node \"%s\" -> %d", node_info.name.c_str(), node_id.get());

    if (!strcmp(node_info.name.c_str(), "electron-accelerator")) {
        motor_can_id = node_id.get();
        motor_found = true;
    }
}

int motor_manager_init(INode& node)
{
    int res;

    static Subscriber<protocol::NodeStatus> node_status_sub(node);
    res = node_status_sub.start(node_status_cb);
    if (res != 0) {
        return res;
    }

    static ServiceClient<protocol::GetNodeInfo> node_info_client(node);
    res = node_info_client.init();
    if (res != 0) {
        return res;
    }
    node_info_client.setCallback(node_info_cb);
    node_info_client_p = &node_info_client;

    return 0;
}

void motor_voltage_publish(INode& node)
{
    static Publisher<cvra::motor::control::Voltage> pub(node);

    cvra::motor::control::Voltage voltage_setpoint;
    voltage_setpoint.node_id = motor_can_id;
    voltage_setpoint.voltage = motor_voltage;

    if (motor_found) {
        pub.broadcast(voltage_setpoint);
    }
}

void motor_voltage_set(float voltage)
{
    if (motor_found) {
        motor_voltage = voltage * MOTOR_VOLTAGE_FORWARD_SIGN;
    }
}

bool motor_ready(void)
{
    return motor_found;
}
