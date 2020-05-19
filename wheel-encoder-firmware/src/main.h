#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <hal.h>
#include <parameter/parameter.h>

extern BaseSequentialStream* ch_stdout;
extern parameter_namespace_t parameter_root_ns;

/* Addresses provided by the linker script. */
extern int _config_start, _config_end;

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
