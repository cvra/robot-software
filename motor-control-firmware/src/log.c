#include <hal.h>
#include <chprintf.h>
#include <stdarg.h>
#include <string.h>
#include "error/error.h"
#include "error_loggers/uavcan_error_logger.h"
#include "error_loggers/chstream_error_logger.h"

MUTEX_DECL(log_lock);

static void log_message(struct error *e, ...)
{
    va_list va;
    chMtxLock(&log_lock);

    static va_list va_copy;

    va_start(va, e);

    va_copy(va_copy, va);

    error_chstream_logger_write((BaseSequentialStream *)&SD3, e, va);
    error_uavcan_logger_write(e, va_copy);

    chMtxUnlock(&log_lock);
}

void log_init(void)
{
    error_uavcan_logger_start();

    error_register_error(log_message);
    error_register_warning(log_message);
    error_register_notice(log_message);

    /* Disabled by default to avoid verbose messages. */
    // error_register_debug(log_message);
}
