#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include <error/error.h>

static void mylog(struct error *e, ...)
{
    va_list va;
    va_start(va, e);

    const char *filename_start = strrchr(e->file, '/') + 1;

    printf("%s:%d: %s: ", filename_start, e->line, error_severity_get_name(e->severity));
    vprintf(e->text, va);
    printf("\n");

    va_end(va);
}

void register_error_handlers(void)
{
    error_register_error(mylog);
    error_register_warning(mylog);
    error_register_notice(mylog);
    error_register_debug(mylog);
}
