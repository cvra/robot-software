#include <hal.h>
#include <chprintf.h>
#include <string.h>
#include "timestamp/timestamp.h"
#include <error/error.h>

#include "log.h"

#define OUTPUT_STREAM ((BaseSequentialStream*)&SD3)


MUTEX_DECL(log_lock);

static const char *get_severity_name(uint8_t severity)
{
    char *result;
    switch (severity) {
        case ERROR_SEVERITY_ERROR:
            result = "ERROR";
        case ERROR_SEVERITY_WARNING:
            result = "WARNING";
        case ERROR_SEVERITY_NOTICE:
            result = "NOTICE";
        case ERROR_SEVERITY_DEBUG:
            result = "DEBUG";
        default:
            result = "UNKNOWN";
    }

    return result;
}

static void log_message(struct error *e, ...)
{
    chMtxLock(&log_lock);

    va_list args;
    const char *thread_name = NULL;
    if (ch.rlist.r_current) {
        thread_name = ch.rlist.r_current->p_name;
    }

    /* Print time */
    uint32_t ts = timestamp_get();
    uint32_t s = ts / 1000000;
    uint32_t us = ts - s * 1000000;
    chprintf(OUTPUT_STREAM, "[%4d.%06d]\t", s, us);

    /* Print location. */
    chprintf(OUTPUT_STREAM, "%s:%d\t", strrchr(e->file, '/'), e->line);

    /* Print current thread */
    if (thread_name != NULL) {
        chprintf(OUTPUT_STREAM, "%s\t", thread_name);
    }

    /* Print severity message */
    chprintf(OUTPUT_STREAM, "%s\t", get_severity_name(e->severity));

    /* Print message */
    va_start(args, e);
    chvprintf(OUTPUT_STREAM, e->text, args);
    va_end(args);

    chprintf(OUTPUT_STREAM, "\n");

    chMtxUnlock(&log_lock);
}

void log_init(void)
{
    error_register_error(log_message);
    error_register_warning(log_message);
    error_register_notice(log_message);

    /* Disabled by default to avoid verbose messages. */
    // error_register_debug(log_message);
}
