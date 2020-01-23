#include "error/error.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

static void log_message(struct error* e, ...)
{
    va_list va;
    va_start(va, e);
    fprintf(stderr, "%s\t%s:%d\t", error_severity_get_name(e->severity), strrchr(e->file, '/') + 1, e->line);
    vfprintf(stderr, e->text, va);
    va_end(va);
}

void logging_init()
{
    error_register_error(log_message);
    error_register_warning(log_message);
    error_register_notice(log_message);
    error_register_debug(log_message);
}
