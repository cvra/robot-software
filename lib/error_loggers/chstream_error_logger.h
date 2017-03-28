#ifndef CHSTREAM_ERROR_LOGGER_H
#define CHSTREAM_ERROR_LOGGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdarg.h>
#include <ch.h>
#include <hal.h>
#include <error/error.h>

void error_chstream_logger_write(BaseSequentialStream *stream, struct error *e, va_list args);

#ifdef __cplusplus
}
#endif

#endif
