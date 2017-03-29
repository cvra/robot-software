#ifndef UAVCAN_ERROR_LOGGER_H
#define UAVCAN_ERROR_LOGGER_H

#include <stdarg.h>

#ifdef __cplusplus
#include <uavcan/uavcan.hpp>

int error_uavcan_logger_spin(uavcan::INode *node);
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include <error/error.h>

int error_uavcan_logger_start(void);
void error_uavcan_logger_write(const struct error *e, va_list args);


#ifdef __cplusplus
}
#endif


#endif
