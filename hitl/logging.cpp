#include <cstdio>
#include <cstdarg>
#include "error/error.h"
#include "absl/flags/flag.h"

ABSL_FLAG(bool, verbose, false, "Enable verbose logging.");

static void log_message(struct error* e, ...)
{
    va_list va;
    va_start(va, e);
    printf("%s ", error_severity_get_name(e->severity));
    vprintf(e->text, va);
    va_end(va);
}

void logging_init()
{
    error_register_error(log_message);
    error_register_warning(log_message);
    error_register_notice(log_message);

    if (absl::GetFlag(FLAGS_verbose)) {
        error_register_debug(log_message);
    }
}
