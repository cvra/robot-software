#include <ch.h>
#include <hal.h>
#include "MotorVoltage_pub.hpp"
#include <cvra/motor/control/Voltage.hpp>

using namespace cvra::motor;

void motor_voltage_publish(uavcan::INode& node, uint8_t dst_id, float voltage)
{
    static uavcan::Publisher<control::Voltage> pub(node);

    control::Voltage voltage_setpoint;
    voltage_setpoint.node_id = dst_id;
    voltage_setpoint.voltage = voltage;

    pub.broadcast(voltage_setpoint);
}
