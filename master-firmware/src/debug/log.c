#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <error/error.h>
#include <signal.h>
#include <pthread.h>

#include "log.h"

static pthread_mutex_t log_lock = PTHREAD_MUTEX_INITIALIZER;

static void log_message(struct error* e, ...)
{
    pthread_mutex_lock(&log_lock);
    va_list va;

    printf("%7s ", error_severity_get_name(e->severity));
    printf("%s:%d\t", strrchr(e->file, '/') + 1, e->line);

    va_start(va, e);
    vprintf(e->text, va);
    va_end(va);

    printf("\n");

    fflush(stdout);

    if (e->severity >= ERROR_SEVERITY_ERROR) {
        /* break if run under a debugger */
        raise(SIGINT);
        exit(1);
    }
    pthread_mutex_unlock(&log_lock);
}

void log_init(int enable_verbose)
{
    error_register_error(log_message);
    error_register_warning(log_message);
    error_register_notice(log_message);

    if (enable_verbose) {
        error_register_debug(log_message);
    }
}
