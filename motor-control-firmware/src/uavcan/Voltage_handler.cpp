#include <cvra/motor/control/Voltage.hpp>
#include "Voltage_handler.hpp"
#include "uavcan_node.h"
#include <timestamp/timestamp.h>
#include "control.h"

int Voltage_handler_start(Node& node)
{
    int ret;
    static uavcan::Subscriber<cvra::motor::control::Voltage> sub(node);

    ret = sub.start(
        [&](const uavcan::ReceivedDataStructure<cvra::motor::control::Voltage>& msg) {
            if (uavcan::NodeID(msg.node_id) == node.getNodeID()) {
                control_update_voltage_setpoint(msg.voltage);
            }
        });

    return ret;
}
