#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <error/error.h>
#include <signal.h>

#include "log.h"

static void log_message(struct error* e, ...)
{
    va_list va;

    printf("%s:%d\t", strrchr(e->file, '/') + 1, e->line);
    printf("%s\t", error_severity_get_name(e->severity));

    va_start(va, e);
    vprintf(e->text, va);
    va_end(va);

    printf("\n");

    if (e->severity >= ERROR_SEVERITY_ERROR) {
        /* break if run under a debugger */
        raise(SIGINT);
        exit(1);
    }
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
