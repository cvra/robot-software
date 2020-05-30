#ifndef MAIN_H
#define MAIN_H

#include "can/motor_manager.h"
#include "can/actuator_driver.h"
#include <parameter/parameter.h>
#include "msgbus_protobuf.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_NB_MOTOR_DRIVERS 20
#define MAX_NB_BUS_ENUMERATOR_ENTRIES 21

/** Robot wide interthread bus. */
extern messagebus_t bus;

extern motor_manager_t motor_manager;
extern parameter_namespace_t global_config;

extern actuator_driver_t actuator_front_left, actuator_front_center, actuator_front_right;
extern actuator_driver_t actuator_back_left, actuator_back_center, actuator_back_right;

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
