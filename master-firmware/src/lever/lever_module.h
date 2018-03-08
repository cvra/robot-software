#ifndef LEVER_MODULE_H
#define LEVER_MODULE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lever.h"

#define LEVER_FREQUENCY 10

extern lever_t right_lever, left_lever;

void lever_module_start(void);

#ifdef __cplusplus
}
#endif

#endif /* LEVER_MODULE_H */
