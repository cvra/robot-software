#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <hal.h>
#include "parameter/parameter.h"

extern BaseSequentialStream* stdout;
extern parameter_namespace_t parameter_root_ns;

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */