#include <hal.h>
#include <chprintf.h>
#include "timestamp/timestamp.h"

#include "log.h"

MUTEX_DECL(log_lock);

void log_message(const char *fmt, ...)
{
    chMtxLock(&log_lock);

    va_list args;
    const char *thread_name = "";
    if (ch.rlist.r_current != NULL && ch.rlist.r_current->p_name != NULL) {
        thread_name = ch.rlist.r_current->p_name;
    }

    uint32_t ts = timestamp_get();
    uint32_t s = ts / 1000000;
    uint32_t us = ts - s;
    chprintf((BaseSequentialStream *)&SD3, "[%4d.%06d] %s: ", s, us, thread_name);

    va_start(args, fmt);
    chvprintf((BaseSequentialStream *)&SD3, fmt, args);
    va_end(args);

    chprintf((BaseSequentialStream *)&SD3, "\n");

    chMtxUnlock(&log_lock);
}

