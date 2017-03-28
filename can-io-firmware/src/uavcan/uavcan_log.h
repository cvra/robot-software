#ifndef UAVCAN_LOG_HPP
#define UAVCAN_LOG_HPP

#include <stdarg.h>

#ifdef __cplusplus
#include "node.h"

int uavcan_log_spin(Node &node);
#endif

#ifdef __cplusplus
extern "C" {
#endif

int uavcan_log_start(void);
void uavcan_log_write(const struct error *e, va_list args);


#ifdef __cplusplus
}
#endif

#endif
