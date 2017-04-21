#include "motor_driver.h"

#ifndef MOTOR_DRIVER_UAVCAN_H
#define MOTOR_DRIVER_UAVCAN_H

#ifdef __cplusplus
extern "C" {
#endif

void motor_driver_send_initial_config(motor_driver_t *d);

void motor_driver_uavcan_update_config(motor_driver_t *d);

void motor_driver_uavcan_send_setpoint(motor_driver_t *d);


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#include <uavcan/uavcan.hpp>
int motor_driver_uavcan_init(uavcan::INode &node);
#endif

#endif /* MOTOR_DRIVER_UAVCAN_H */
