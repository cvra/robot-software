#ifndef MOTOR_MANAGER_HPP
#define MOTOR_MANAGER_HPP

#include <uavcan/uavcan.hpp>

int motor_manager_init(uavcan::INode& node);
void motor_voltage_publish(uavcan::INode& node);
void motor_voltage_set(float voltage);
bool motor_ready(void);

#endif /* MOTOR_MANAGER_HPP */
