#ifndef MAIN_H
#define MAIN_H

#include "motor_manager.h"
#include "msgbus_protobuf.h"
#include <parameter/parameter.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_NB_MOTOR_DRIVERS 20
#define MAX_NB_BUS_ENUMERATOR_ENTRIES 21

/** Robot wide interthread bus. */
extern messagebus_t bus;

extern motor_manager_t motor_manager;

extern parameter_namespace_t global_config;

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
