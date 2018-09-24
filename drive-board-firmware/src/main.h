#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_NB_MOTOR_DRIVERS            20
#define MAX_NB_BUS_ENUMERATOR_ENTRIES   21

#include "motor_manager.h"
#include "base/base.h"
#include <parameter/parameter.h>

extern motor_manager_t motor_manager;
extern base_t rover_base;

extern parameter_namespace_t global_config;

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
