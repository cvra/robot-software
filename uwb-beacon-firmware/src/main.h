#ifndef MAIN_H
#define MAIN_H

#include <msgbus/messagebus.h>
#include <parameter/parameter.h>

#ifdef __cplusplus
extern "C" {
#endif

extern messagebus_t bus;
extern parameter_namespace_t parameter_root;

/* Addresses provided by the linker script. */
extern int _config_start, _config_end;

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
