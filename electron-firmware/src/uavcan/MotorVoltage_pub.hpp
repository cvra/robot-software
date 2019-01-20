#ifndef MOTORVOLTAGE_PUB_HPP
#define MOTORVOLTAGE_PUB_HPP

#include <uavcan/uavcan.hpp>

void motor_voltage_publish(uavcan::INode& node, uint8_t dst_id, float voltage);

#endif
