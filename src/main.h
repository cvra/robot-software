#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <msgbus/messagebus.h>
#include <parameter/parameter.h>

extern messagebus_t bus;
extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
